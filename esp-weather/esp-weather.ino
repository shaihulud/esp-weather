/*
 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Uncomment section starting with: For ESP32 Dev board (only tested with ILI9341 display)

 To convert images use PNG to XBM converter.
 Ex.: https://convertio.co/ru/
 */
#include <WiFi.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"
#include <TFT_eSPI.h>

#include "tiles/neko_cold.h"

// WiFi + PostgreSQL credentials live in secrets.h (gitignored);
// copy secrets.h.example to secrets.h and fill in your values
#include "secrets.h"

#ifndef SECRETS_STATION_INDOOR
#error "Wrong secrets.h: this is the indoor station, use its own secrets file"
#endif

// Hardware configuration
const uint8_t PMS_RX_PIN = 34;
const uint8_t PMS_TX_PIN = 33;
const uint8_t S8_RX_PIN = 16;  // Senseair S8 on hardware UART2
const uint8_t S8_TX_PIN = 17;
const uint8_t SHT31_ADDR = 0x44;
const uint8_t BME280_ADDR = 0x76;

// Per-board BME280 calibration, measured 2026-07-16 side by side with the
// SHT31; re-tune if the sensor is replaced (this board's die reads true,
// so no temperature compensation - unlike the outdoor board)
const float BME_TEMP_COMPENSATION = 0.0;
const float BME_HUMIDITY_OFFSET = 13.3;  // humidity cell drifted low
const float BME_PRESSURE_OFFSET = 0.0;   // mmHg

// Timing constants
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;
const unsigned long NORMAL_DELAY = 30000;
const unsigned long ERROR_DELAY = 10000;
const unsigned long SENSOR_INIT_DELAY = 1000;
const unsigned long DB_STATE_TIMEOUT = 30000;
const unsigned long INSERT_WATCHDOG_TIMEOUT = 600000;  // restart if no insert for 10 min
const unsigned long S8_RESPONSE_TIMEOUT = 1000;
const unsigned long OUTDOOR_STALE_SECONDS = 300;  // grey out outdoor data older than this
const uint8_t MAX_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_REINIT_THRESHOLD = 5;  // consecutive bad cycles before re-init

// Buffer configuration
const size_t QUERY_BUFFER_SIZE = 256;

// Database connection states
enum class DatabaseState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    EXECUTING_QUERY,
    ERROR
};

// Sensor data structure
struct SensorData {
    float temp_sht31;
    float humi_sht31;
    float temp_bme280;
    float humi_bme280;
    float pres_bme280;
    uint16_t pm01;
    uint16_t pm25;
    uint16_t pm10;
    uint16_t co2;
    bool sht31_valid;
    bool bme280_valid;
    bool pms_valid;
    bool s8_valid;
};

// Latest row from the outdoor station, shown on the display
struct OutdoorData {
    float temp_bme;
    float pres_bme;
    float humi_bme;
    float temp_sht;
    float humi_sht;
    int pm01;
    int pm25;
    int pm10;
    long ageSeconds;           // row age reported by the server at fetch time
    unsigned long fetchedAtMs; // millis() when the row was fetched
    bool valid;
};

// Global objects
WiFiClient client;
char pgBuffer[PG_BUFFER_SIZE];
PGconnection conn(&client, 0, PG_BUFFER_SIZE, pgBuffer);

Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);
TFT_eSPI tft = TFT_eSPI();

byte s8ReadCommand[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};

DatabaseState dbState = DatabaseState::DISCONNECTED;
unsigned long dbStateSince = 0;

// Consecutive invalid-reading counters, trigger sensor re-init at threshold
uint8_t sht31Failures = 0;
uint8_t bme280Failures = 0;
uint8_t pmsFailures = 0;
uint8_t s8Failures = 0;

unsigned long lastSuccessfulInsert = 0;
bool queryHadError = false;  // server rejected the current query

// Query currently driven through the state machine; rowSink receives
// SELECT rows when set
const char* pendingQuery = nullptr;
bool pendingProgmem = false;
OutdoorData* rowSink = nullptr;

// Function declarations
bool initializeWiFi();
bool initializeSensors();
bool initSensorWithRetry(const char* name, bool (*beginSensor)());
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
bool initializeS8();
SensorData readSensorData();
bool readS8CO2(uint16_t& co2);
bool validateSensorData(SensorData& data);
void handleSensorHealth(const SensorData& data);
void logSensorData(const SensorData& data);
bool sendDataToDatabase(const SensorData& data);
void fetchOutdoorData(OutdoorData& out);
void parseOutdoorRow(OutdoorData& out);
bool runQuery(const char* query, bool progmem, OutdoorData* sink);
bool advanceDbStateMachine();
void startDbConnection();
void pollDbConnection();
bool dispatchPendingQuery();
bool processQueryResult();
void handleDatabaseError(const char* error);
void resetDatabaseConnection();
void setDbState(DatabaseState newState);
void checkInsertWatchdog();
void drawTft(const SensorData& data, const OutdoorData& outdoor);

void setup() {
    Serial.begin(9600);
    Serial.println("\nESP32 Weather Station Starting...");

    // Initialize TFT
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString("Connecting to WiFi", 13, 158, 4);

    // Station mode only; persistent(false) avoids rewriting credentials
    // to NVS flash on every begin()
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    if (!initializeWiFi()) {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Connection failed", 13, 158, 4);
    } else {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Connection established", 13, 158, 4);
    }

    if (!initializeSensors()) {
        Serial.println("WARNING: Some sensors failed to initialize");
    }

    Serial.println("Setup complete");
}

void loop() {
    // Last-resort recovery for any failure mode the code doesn't handle
    checkInsertWatchdog();

    // Ensure WiFi is connected
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, attempting reconnection...");
        if (!initializeWiFi()) {
            Serial.println("WiFi reconnection failed, waiting before retry");
            delay(ERROR_DELAY);
            return;
        }
        // The old PostgreSQL socket died with the previous association
        resetDatabaseConnection();
    }

    // Last known outdoor values, kept between cycles when a fetch fails
    static OutdoorData outdoor = {};

    // Read sensor data
    SensorData data = readSensorData();

    // Validate per sensor and re-init sensors that keep failing;
    // a single bad sensor must not block the others from being written
    bool anyValid = validateSensorData(data);
    handleSensorHealth(data);
    logSensorData(data);

    bool sent = false;
    if (anyValid) {
        // Send to database (invalid sensors are written as NULL)
        sent = sendDataToDatabase(data);
        Serial.println(sent ? "Data sent successfully" : "Database operation failed");
    } else {
        Serial.println("No valid sensor data, skipping insert");
    }

    // Refresh outdoor readings for the display (keeps old values on failure)
    fetchOutdoorData(outdoor);

    drawTft(data, outdoor);

    delay(sent ? NORMAL_DELAY : ERROR_DELAY);
}

bool initializeWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.println("Connecting to WiFi...");
    WiFi.disconnect();  // clear any stuck association attempt before retrying
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed");
        return false;
    }

    Serial.println("\nWiFi connected successfully");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    return true;
}

bool initializeSensors() {
    bool sht31Success = initializeSHT31();
    bool bme280Success = initializeBME280();
    bool pmsSuccess = initializePMS();
    bool s8Success = initializeS8();

    return sht31Success && bme280Success && pmsSuccess && s8Success;
}

bool initSensorWithRetry(const char* name, bool (*beginSensor)()) {
    Serial.printf("Initializing %s sensor...\n", name);

    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        if (beginSensor()) {
            Serial.printf("%s initialized successfully\n", name);
            return true;
        }

        Serial.printf("%s initialization attempt %d failed, retrying...\n", name, attempt);
        delay(SENSOR_INIT_DELAY);
    }

    Serial.printf("ERROR: %s initialization failed after multiple attempts\n", name);
    return false;
}

bool initializeSHT31() {
    return initSensorWithRetry("SHT31", [] { return sht31.begin(SHT31_ADDR); });
}

bool initializeBME280() {
    return initSensorWithRetry("BME280", [] {
        if (!bme.begin(BME280_ADDR)) {
            return false;
        }
        // Forced mode: sensor sleeps between readings instead of sampling
        // continuously, which self-heats the die by several degrees and
        // skews temperature (high) and relative humidity (low)
        bme.setSampling(Adafruit_BME280::MODE_FORCED,
                        Adafruit_BME280::SAMPLING_X1,  // temperature
                        Adafruit_BME280::SAMPLING_X1,  // pressure
                        Adafruit_BME280::SAMPLING_X1,  // humidity
                        Adafruit_BME280::FILTER_OFF);
        bme.setTemperatureCompensation(BME_TEMP_COMPENSATION);
        return true;
    });
}

bool initializePMS() {
    Serial.println("Initializing PMS sensor...");
    pms.init();
    Serial.println("PMS sensor initialized");
    return true; // PMS init doesn't return status
}

bool initializeS8() {
    Serial.println("Initializing S8 sensor...");
    Serial2.begin(9600, SERIAL_8N1, S8_RX_PIN, S8_TX_PIN);
    Serial.println("S8 sensor initialized");
    return true; // S8 init doesn't return status
}

SensorData readSensorData() {
    SensorData data = {};

    // Read SHT31 data
    data.temp_sht31 = sht31.readTemperature();
    data.humi_sht31 = sht31.readHumidity();

    // Read BME280 data (forced mode: trigger one measurement on demand)
    if (bme.takeForcedMeasurement()) {
        data.temp_bme280 = bme.readTemperature();
        data.humi_bme280 = fminf(bme.readHumidity() + BME_HUMIDITY_OFFSET, 100.0f);
        data.pres_bme280 = bme.readPressure() / 133.322F + BME_PRESSURE_OFFSET; // Pa -> mmHg
    } else {
        data.temp_bme280 = NAN;
        data.humi_bme280 = NAN;
        data.pres_bme280 = NAN;
    }

    // Read PMS data
    pms.read();
    data.pms_valid = pms.has_particulate_matter();
    data.pm01 = pms.pm01;
    data.pm25 = pms.pm25;
    data.pm10 = pms.pm10;

    // Read S8 CO2 data
    data.s8_valid = readS8CO2(data.co2);

    return data;
}

// Sends one Modbus read command to the Senseair S8 and waits for the 7-byte
// response, with a timeout so a dead sensor can't hang the whole station
bool readS8CO2(uint16_t& co2) {
    while (Serial2.available()) {
        Serial2.read();  // flush stale bytes from a previous attempt
    }

    Serial2.write(s8ReadCommand, sizeof(s8ReadCommand));

    unsigned long start = millis();
    while (Serial2.available() < 7) {
        if (millis() - start > S8_RESPONSE_TIMEOUT) {
            return false;
        }
        delay(10);
    }

    byte response[7];
    for (int i = 0; i < 7; i++) {
        response[i] = Serial2.read();
    }

    // Sanity-check the Modbus header (address 0xFE, function 0x44)
    if (response[0] != 0xFE || response[1] != 0x44) {
        return false;
    }

    co2 = response[3] * 256 + response[4];
    return true;
}

bool validateSensorData(SensorData& data) {
    data.sht31_valid = !isnan(data.temp_sht31) && !isnan(data.humi_sht31) &&
                       data.temp_sht31 >= -40 && data.temp_sht31 <= 85 &&
                       data.humi_sht31 >= 0 && data.humi_sht31 <= 100;
    data.bme280_valid = !isnan(data.temp_bme280) && !isnan(data.humi_bme280) &&
                        !isnan(data.pres_bme280) &&
                        data.temp_bme280 >= -40 && data.temp_bme280 <= 85 &&
                        data.humi_bme280 >= 0 && data.humi_bme280 <= 100 &&
                        // mmHg (recorded extremes ~650-815 at sea level)
                        data.pres_bme280 >= 500 && data.pres_bme280 <= 850;

    // pms_valid comes from the read status; also reject glitch values
    // that would overflow the smallint columns
    if (data.pms_valid &&
        (data.pm01 > 2000 || data.pm25 > 2000 || data.pm10 > 2000)) {
        data.pms_valid = false;
    }

    // s8_valid comes from the read status; reject implausible concentrations
    // (the sensor floor is ~400ppm outdoor air, garbage reads show as 0 or 65535)
    if (data.s8_valid && (data.co2 < 350 || data.co2 > 5000)) {
        data.s8_valid = false;
    }

    return data.sht31_valid || data.bme280_valid || data.pms_valid || data.s8_valid;
}

void handleSensorHealth(const SensorData& data) {
    sht31Failures = data.sht31_valid ? 0 : sht31Failures + 1;
    bme280Failures = data.bme280_valid ? 0 : bme280Failures + 1;
    pmsFailures = data.pms_valid ? 0 : pmsFailures + 1;
    s8Failures = data.s8_valid ? 0 : s8Failures + 1;

    if (sht31Failures >= SENSOR_REINIT_THRESHOLD) {
        Serial.println("SHT31 failing repeatedly, re-initializing...");
        initializeSHT31();
        sht31Failures = 0;
    }
    if (bme280Failures >= SENSOR_REINIT_THRESHOLD) {
        Serial.println("BME280 failing repeatedly, re-initializing...");
        initializeBME280();
        bme280Failures = 0;
    }
    if (pmsFailures >= SENSOR_REINIT_THRESHOLD) {
        Serial.println("PMS failing repeatedly, re-initializing...");
        initializePMS();
        pmsFailures = 0;
    }
    if (s8Failures >= SENSOR_REINIT_THRESHOLD) {
        Serial.println("S8 failing repeatedly, re-initializing...");
        initializeS8();
        s8Failures = 0;
    }
}

void logSensorData(const SensorData& data) {
    Serial.println("=== Sensor Readings ===");
    Serial.printf("SHT31  - Temp: %.2f°C, Humidity: %.2f%%%s\n",
                  data.temp_sht31, data.humi_sht31,
                  data.sht31_valid ? "" : " [INVALID]");
    Serial.printf("BME280 - Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2f mmHg%s\n",
                  data.temp_bme280, data.humi_bme280, data.pres_bme280,
                  data.bme280_valid ? "" : " [INVALID]");
    Serial.printf("PMS    - PM1.0: %d µg/m³, PM2.5: %d µg/m³, PM10: %d µg/m³%s\n",
                  data.pm01, data.pm25, data.pm10,
                  data.pms_valid ? "" : " [INVALID]");
    Serial.printf("S8     - CO2: %d ppm%s\n",
                  data.co2, data.s8_valid ? "" : " [INVALID]");
    Serial.println("=======================\n");
}

bool sendDataToDatabase(const SensorData& data) {
    char query[QUERY_BUFFER_SIZE];
    char bmeVals[32];
    char shtVals[24];
    char co2Val[8];
    char pmVals[24];

    if (data.bme280_valid) {
        snprintf(bmeVals, sizeof(bmeVals), "%.2f, %.2f, %.2f",
                 data.temp_bme280, data.pres_bme280, data.humi_bme280);
    } else {
        strcpy(bmeVals, "NULL, NULL, NULL");
    }
    if (data.sht31_valid) {
        snprintf(shtVals, sizeof(shtVals), "%.2f, %.2f",
                 data.temp_sht31, data.humi_sht31);
    } else {
        strcpy(shtVals, "NULL, NULL");
    }
    if (data.s8_valid) {
        snprintf(co2Val, sizeof(co2Val), "%u", data.co2);
    } else {
        strcpy(co2Val, "NULL");
    }
    if (data.pms_valid) {
        snprintf(pmVals, sizeof(pmVals), "%u, %u, %u",
                 data.pm01, data.pm25, data.pm10);
    } else {
        strcpy(pmVals, "NULL, NULL, NULL");
    }

    int result = snprintf(query, sizeof(query),
        "INSERT INTO room_mine VALUES (DEFAULT, %s, %s, %s, %s)",
        bmeVals, shtVals, co2Val, pmVals);

    if (result >= (int)sizeof(query)) {
        Serial.println("ERROR: Query buffer overflow");
        return false;
    }

    return runQuery(query, false, nullptr);
}

// Fetches the newest outdoor row for the display; leaves `out` untouched
// when the query fails or returns no row
void fetchOutdoorData(OutdoorData& out) {
    static const char query[] PROGMEM =
        "SELECT temperature_bme, pressure_bme, humidity_bme, "
        "temperature_sht, humidity_sht, pm01, pm25, pm10, "
        "EXTRACT(EPOCH FROM (now() - dt))::int "
        "FROM outside ORDER BY dt DESC LIMIT 1";

    OutdoorData fresh = {};
    if (runQuery(query, true, &fresh) && fresh.valid) {
        out = fresh;
    }
}

// Columns arrive in the SELECT's order; outdoor sensors that failed are
// NULL in the database and become NAN here
void parseOutdoorRow(OutdoorData& out) {
    if (conn.nfields() < 9) {
        return;
    }
    const char* v;
    v = conn.getValue(0); out.temp_bme = v ? atof(v) : NAN;
    v = conn.getValue(1); out.pres_bme = v ? atof(v) : NAN;
    v = conn.getValue(2); out.humi_bme = v ? atof(v) : NAN;
    v = conn.getValue(3); out.temp_sht = v ? atof(v) : NAN;
    v = conn.getValue(4); out.humi_sht = v ? atof(v) : NAN;
    v = conn.getValue(5); out.pm01 = v ? atoi(v) : 0;
    v = conn.getValue(6); out.pm25 = v ? atoi(v) : 0;
    v = conn.getValue(7); out.pm10 = v ? atoi(v) : 0;
    v = conn.getValue(8); out.ageSeconds = v ? atol(v) : 0;
    if (out.ageSeconds < 0) {
        out.ageSeconds = 0;  // guard against clock skew
    }
    out.fetchedAtMs = millis();
    out.valid = true;
}

void setDbState(DatabaseState newState) {
    dbState = newState;
    dbStateSince = millis();
}

bool runQuery(const char* query, bool progmem, OutdoorData* sink) {
    pendingQuery = query;
    pendingProgmem = progmem;
    rowSink = sink;

    // Drive the state machine to completion: <1s when healthy, one 30s
    // stuck-state reset plus a retry at worst. On a hard failure (ERROR
    // state) give up so the caller backs off via ERROR_DELAY.
    bool completed = false;
    unsigned long start = millis();
    while (millis() - start < 2 * DB_STATE_TIMEOUT) {
        if (advanceDbStateMachine()) {
            completed = true;
            break;
        }
        if (dbState == DatabaseState::ERROR) {
            break;
        }
        delay(50);
    }

    rowSink = nullptr;
    pendingQuery = nullptr;
    return completed && !queryHadError;
}

// Performs one step of the connect/query cycle; returns true once the
// current query's round-trip has completed
bool advanceDbStateMachine() {
    // A dead TCP connection never produces bytes, so getData()/status() alone
    // can leave the waiting states stuck forever - enforce wall-clock progress
    bool waitingState = dbState == DatabaseState::CONNECTING ||
                        dbState == DatabaseState::EXECUTING_QUERY;
    if (waitingState && millis() - dbStateSince > DB_STATE_TIMEOUT) {
        Serial.println("ERROR: Database state timeout, resetting connection");
        resetDatabaseConnection();
        return false;
    }

    switch (dbState) {
        case DatabaseState::DISCONNECTED:
            startDbConnection();
            return false;

        case DatabaseState::CONNECTING:
            pollDbConnection();
            return false;

        case DatabaseState::CONNECTED:
            return dispatchPendingQuery();

        case DatabaseState::EXECUTING_QUERY:
            return processQueryResult();

        case DatabaseState::ERROR:
            resetDatabaseConnection();
            return false;

        default:
            Serial.println("dbState ERROR: Unknown database state");
            resetDatabaseConnection();
            return false;
    }
}

void startDbConnection() {
    Serial.println("Connecting to PostgreSQL database...");
    conn.setDbLogin(PG_IP, PG_USER, PG_PASSWORD, PG_DBNAME, "utf8");
    setDbState(DatabaseState::CONNECTING);
}

void pollDbConnection() {
    int status = conn.status();

    if (status == CONNECTION_BAD || status == CONNECTION_NEEDED) {
        const char* error = conn.getMessage();
        handleDatabaseError(error ? error : "Connection failed");
    } else if (status == CONNECTION_OK) {
        Serial.println("Database connected successfully");
        setDbState(DatabaseState::CONNECTED);
    }
    // Otherwise still connecting - stay in CONNECTING and poll again
}

bool dispatchPendingQuery() {
    if (!pendingQuery) {
        return false;
    }

    Serial.print("Executing query: ");
    if (pendingProgmem) {
        Serial.println((const __FlashStringHelper*)pendingQuery);
    } else {
        Serial.println(pendingQuery);
    }

    if (conn.execute(pendingQuery, pendingProgmem) < 0) {
        handleDatabaseError("Query execution failed");
        return false;
    }

    queryHadError = false;
    setDbState(DatabaseState::EXECUTING_QUERY);
    return false; // Not complete yet
}

bool processQueryResult() {
    int result = conn.getData();

    if (result < 0) {
        handleDatabaseError("Error retrieving query result");
        return false;
    }

    if (result == 0) {
        // No response bytes yet
        return false;
    }

    if (result & PG_RSTAT_HAVE_ERROR) {
        queryHadError = true;
    }

    if (result & PG_RSTAT_HAVE_MESSAGE) {
        const char* message = conn.getMessage();
        if (message) {
            Serial.printf("Database message: %s\n", message);
        }
    }

    if ((result & PG_RSTAT_HAVE_ROW) && rowSink != nullptr) {
        parseOutdoorRow(*rowSink);
    }

    if (result & PG_RSTAT_HAVE_SUMMARY) {
        Serial.printf("Query completed - Rows affected: %d\n", conn.ntuples());
    }

    if (result & PG_RSTAT_READY) {
        if (queryHadError) {
            Serial.println("Query finished with error");
        }
        // Even a rejected query proves the connection is alive - feed the
        // watchdog either way, a restart can't fix a bad statement
        lastSuccessfulInsert = millis();
        setDbState(DatabaseState::CONNECTED);
        return true; // Round-trip complete
    }

    // Continue processing
    return false;
}

void handleDatabaseError(const char* error) {
    Serial.printf("Database error: %s\n", error ? error : "Unknown error");
    // Always land in ERROR so the query loop backs off for ERROR_DELAY
    // instead of hammering reconnection attempts; the ERROR state resets
    // to DISCONNECTED on the next cycle
    conn.close();
    setDbState(DatabaseState::ERROR);
}

void resetDatabaseConnection() {
    Serial.println("Resetting database connection");
    conn.close();
    setDbState(DatabaseState::DISCONNECTED);
}

void checkInsertWatchdog() {
    if (millis() - lastSuccessfulInsert > INSERT_WATCHDOG_TIMEOUT) {
        Serial.printf("WATCHDOG: no successful insert for %lu minutes, restarting\n",
                      INSERT_WATCHDOG_TIMEOUT / 60000UL);
        Serial.flush();
        delay(100);
        ESP.restart();
    }
}

void drawTft(const SensorData& data, const OutdoorData& outdoor) {
    char tftBuffer[40];

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);

    // Indoor temperature: average when both sensors are healthy.
    // Humidity: SHT31 preferred - the BME280 humidity cell drifts.
    float indoorTemp;
    if (data.sht31_valid && data.bme280_valid) {
        indoorTemp = (data.temp_sht31 + data.temp_bme280) / 2;
    } else if (data.bme280_valid) {
        indoorTemp = data.temp_bme280;
    } else {
        indoorTemp = data.temp_sht31;
    }
    float indoorHumi = data.sht31_valid ? data.humi_sht31 : data.humi_bme280;

    tft.drawString("Temp:", 13, 0, 4);
    tft.drawString("Humi:", 143, 0, 4);
    tft.drawFloat(indoorTemp, 1, 0, 25, 6);
    tft.drawFloat(indoorHumi, 1, 130, 25, 6);

    // Pressure and CO2
    tft.drawString("Press:", 13, 75, 4);
    tft.drawString("CO2:", 143, 75, 4);
    tft.drawFloat(data.pres_bme280, 0, 0, 100, 6);
    if (data.co2 < 1000) {
        tft.drawNumber(data.co2, 130, 100, 6);
    } else {
        tft.drawNumber(data.co2, 115, 100, 6);
    }

    // Indoor PM values (font 2 so both PM columns fit side by side)
    sprintf(tftBuffer, "PM: %d %d %d", data.pm01, data.pm25, data.pm10);
    tft.drawString(tftBuffer, 1, 155, 2);

    // Outdoor data (if ever fetched; NAN fields mean that outdoor sensor
    // was down and its columns were NULL)
    if (outdoor.valid) {
        // Row age = age reported by the server plus time since the fetch;
        // grey out the outdoor block and show its age when it goes stale
        unsigned long ageSec = (unsigned long)outdoor.ageSeconds +
                               (millis() - outdoor.fetchedAtMs) / 1000;
        bool stale = ageSec > OUTDOOR_STALE_SECONDS;
        tft.setTextColor(stale ? TFT_DARKGREY : TFT_WHITE, TFT_BLACK);

        sprintf(tftBuffer, "PM: %d %d %d", outdoor.pm01, outdoor.pm25, outdoor.pm10);
        tft.drawString(tftBuffer, 121, 155, 2);

        if (stale) {
            if (ageSec < 3600) {
                sprintf(tftBuffer, "%lum old", ageSec / 60);
            } else if (ageSec < 86400) {
                sprintf(tftBuffer, "%luh old", ageSec / 3600);
            } else {
                strcpy(tftBuffer, ">1d old");
            }
            tft.drawString(tftBuffer, 121, 172, 2);
        }

        float outdoorTemp;
        if (isnan(outdoor.temp_sht)) {
            outdoorTemp = outdoor.temp_bme;
        } else if (isnan(outdoor.temp_bme)) {
            outdoorTemp = outdoor.temp_sht;
        } else {
            outdoorTemp = (outdoor.temp_sht + outdoor.temp_bme) / 2;
        }
        float outdoorHumi = isnan(outdoor.humi_sht) ? outdoor.humi_bme : outdoor.humi_sht;

        tft.drawString("Temp:", 13, 180, 4);
        tft.drawFloat(outdoorTemp, 1, 0, 205, 6);
        tft.drawString("Humi:", 13, 258, 4);
        tft.drawFloat(outdoorHumi, 1, 0, 283, 6);

        tft.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    // Draw cat animation
    tft.drawXBitmap(115, 190, NekoCold, 128, 128, TFT_BLACK, TFT_WHITE);
}
