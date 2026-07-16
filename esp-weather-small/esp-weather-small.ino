#include <WiFi.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"

#include "secrets.h"

#ifndef SECRETS_STATION_OUTDOOR
#error "Wrong secrets.h: this is the outdoor station, use its own secrets file"
#endif

// Hardware configuration
const uint8_t PMS_RX_PIN = 34;
const uint8_t PMS_TX_PIN = 33;
const uint8_t SHT31_ADDR = 0x44;
const uint8_t BME280_ADDR = 0x76;

// Per-board BME280 calibration, measured 2026-07-16 side by side with the
// SHT31 (humidity) and the indoor station (pressure); re-tune if the sensor
// is replaced. Pressure offset counters the t_fine shift introduced by
// setTemperatureCompensation(), which also skews the compensated pressure.
const float BME_TEMP_COMPENSATION = -2.3;  // die reads high vs SHT31
const float BME_HUMIDITY_OFFSET = 16.5;
const float BME_PRESSURE_OFFSET = 3.0;  // mmHg

// Timing constants
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;
const unsigned long NORMAL_DELAY = 30000;
const unsigned long ERROR_DELAY = 10000;
const unsigned long SENSOR_INIT_DELAY = 1000;
const unsigned long DB_STATE_TIMEOUT = 30000;
const unsigned long INSERT_WATCHDOG_TIMEOUT = 600000;  // restart if no insert for 10 min
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
    bool sht31_valid;
    bool bme280_valid;
    bool pms_valid;
};

// Global objects
WiFiClient client;
char pgBuffer[PG_BUFFER_SIZE];
PGconnection conn(&client, 0, PG_BUFFER_SIZE, pgBuffer);

Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);

DatabaseState dbState = DatabaseState::DISCONNECTED;
unsigned long dbStateSince = 0;

// Consecutive invalid-reading counters, trigger sensor re-init at threshold
uint8_t sht31Failures = 0;
uint8_t bme280Failures = 0;
uint8_t pmsFailures = 0;

unsigned long lastSuccessfulInsert = 0;
bool queryHadError = false;  // server rejected the current query

// Function declarations
bool initializeWiFi();
bool initializeSensors();
bool initSensorWithRetry(const char* name, bool (*beginSensor)());
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
SensorData readSensorData();
bool validateSensorData(SensorData& data);
void handleSensorHealth(const SensorData& data);
void logSensorData(const SensorData& data);
bool sendDataToDatabase(const SensorData& data);
bool advanceDbStateMachine(const SensorData& data);
void startDbConnection();
void pollDbConnection();
bool executeInsertQuery(const SensorData& data);
bool processQueryResult();
void handleDatabaseError(const char* error);
void resetDatabaseConnection();
void setDbState(DatabaseState newState);
void checkInsertWatchdog();

void setup() {
    Serial.begin(9600);
    Serial.println("\nESP32 Weather Station Starting...");

    // Station mode only; persistent(false) avoids rewriting credentials to NVS flash on every begin()
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Initialize WiFi
    if (!initializeWiFi()) {
        Serial.println("ERROR: WiFi initialization failed");
    }

    // Initialize sensors
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

    // Read sensor data
    SensorData data = readSensorData();

    // Validate per sensor and re-init sensors that keep failing;
    // a single bad sensor must not block the others from being written
    bool anyValid = validateSensorData(data);
    handleSensorHealth(data);
    logSensorData(data);

    if (anyValid) {
        // Send to database (invalid sensors are written as NULL)
        if (sendDataToDatabase(data)) {
            Serial.println("Data sent successfully");
            delay(NORMAL_DELAY);
        } else {
            Serial.println("Database operation failed");
            delay(ERROR_DELAY);
        }
    } else {
        Serial.println("No valid sensor data, skipping this cycle");
        delay(ERROR_DELAY);
    }
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

    return sht31Success && bme280Success && pmsSuccess;
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

    return data;
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

    return data.sht31_valid || data.bme280_valid || data.pms_valid;
}

void handleSensorHealth(const SensorData& data) {
    sht31Failures = data.sht31_valid ? 0 : sht31Failures + 1;
    bme280Failures = data.bme280_valid ? 0 : bme280Failures + 1;
    pmsFailures = data.pms_valid ? 0 : pmsFailures + 1;

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
    Serial.println("=======================\n");
}

void setDbState(DatabaseState newState) {
    dbState = newState;
    dbStateSince = millis();
}

bool sendDataToDatabase(const SensorData& data) {
    // Drive the state machine to completion: <1s when healthy, one 30s
    // stuck-state reset plus a retry at worst. On a hard failure (ERROR
    // state) give up so the caller backs off via ERROR_DELAY.
    unsigned long start = millis();
    while (millis() - start < 2 * DB_STATE_TIMEOUT) {
        if (advanceDbStateMachine(data)) {
            return !queryHadError;
        }
        if (dbState == DatabaseState::ERROR) {
            return false;
        }
        delay(50);
    }
    return false;
}

// Performs one step of the connect/insert cycle; returns true once the
// current query's round-trip has completed
bool advanceDbStateMachine(const SensorData& data) {
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
            return executeInsertQuery(data);

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

bool executeInsertQuery(const SensorData& data) {
    char query[QUERY_BUFFER_SIZE];
    char bmeVals[32];
    char shtVals[24];
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
    if (data.pms_valid) {
        snprintf(pmVals, sizeof(pmVals), "%u, %u, %u",
                 data.pm01, data.pm25, data.pm10);
    } else {
        strcpy(pmVals, "NULL, NULL, NULL");
    }

    int result = snprintf(query, sizeof(query),
        "INSERT INTO outside VALUES (DEFAULT, %s, %s, %s)",
        bmeVals, shtVals, pmVals);

    if (result >= sizeof(query)) {
        Serial.println("ERROR: Query buffer overflow");
        return false;
    }

    Serial.printf("Executing query: %s\n", query);

    if (conn.execute(query, false) < 0) {
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
    // Always land in ERROR so the send loop backs off for ERROR_DELAY
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
