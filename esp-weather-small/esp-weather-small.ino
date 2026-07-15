#include <WiFi.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"

// Configuration constants
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";
const char* PG_USER = "";
const char* PG_PASSWORD = "";
const char* PG_DBNAME = "weather";
const IPAddress PG_IP(0, 0, 0, 0);

// Hardware configuration
const uint8_t PMS_RX_PIN = 34;
const uint8_t PMS_TX_PIN = 33;
const uint8_t SHT31_ADDR = 0x44;
const uint8_t BME280_ADDR = 0x76;

// Timing constants
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;
const unsigned long NORMAL_DELAY = 30000;
const unsigned long ERROR_DELAY = 10000;
const unsigned long SENSOR_INIT_DELAY = 1000;
const unsigned long DB_STATE_TIMEOUT = 30000;
const uint8_t MAX_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_REINIT_THRESHOLD = 5;  // consecutive bad cycles before re-init

// Buffer configuration
// const size_t PG_BUFFER_SIZE = 1024;
const size_t QUERY_BUFFER_SIZE = 256;

// Database connection states
enum class DatabaseState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    EXECUTING_QUERY,
    PROCESSING_RESULT,
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

// Function declarations
bool initializeWiFi();
bool initializeSensors();
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
SensorData readSensorData();
bool validateSensorData(SensorData& data);
void handleSensorHealth(const SensorData& data);
void logSensorData(const SensorData& data);
bool handleDatabaseConnection();
bool sendDataToDatabase(const SensorData& data);
void handleDatabaseError(const char* error);
void resetDatabaseConnection();
void setDbState(DatabaseState newState);

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

    if (anyValid) {
        logSensorData(data);

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

bool initializeSHT31() {
    Serial.println("Initializing SHT31 sensor...");

    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        if (sht31.begin(SHT31_ADDR)) {
            Serial.println("SHT31 initialized successfully");
            return true;
        }

        Serial.printf("SHT31 initialization attempt %d failed, retrying...\n", attempt);
        delay(SENSOR_INIT_DELAY);
    }

    Serial.println("ERROR: SHT31 initialization failed after multiple attempts");
    return false;
}

bool initializeBME280() {
    Serial.println("Initializing BME280 sensor...");

    for (int attempt = 1; attempt <= MAX_INIT_ATTEMPTS; attempt++) {
        if (bme.begin(BME280_ADDR)) {
            Serial.println("BME280 initialized successfully");
            return true;
        }

        Serial.printf("BME280 initialization attempt %d failed (SensorID: 0x%02X), retrying...\n", 
                     attempt, bme.sensorID());
        delay(SENSOR_INIT_DELAY);
    }

    Serial.println("ERROR: BME280 initialization failed after multiple attempts");
    return false;
}

bool initializePMS() {
    Serial.println("Initializing PMS sensor...");
    pms.init();
    Serial.println("PMS sensor initialized");
    return true; // PMS init doesn't return status
}

SensorData readSensorData() {
    SensorData data = {0};

    // Read SHT31 data
    data.temp_sht31 = sht31.readTemperature();
    data.humi_sht31 = sht31.readHumidity();

    // Read BME280 data
    data.temp_bme280 = bme.readTemperature();
    data.humi_bme280 = bme.readHumidity();
    data.pres_bme280 = bme.readPressure() / 133.322F; // Convert Pa to mmHg

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
    if (!data.sht31_valid) {
        Serial.println("ERROR: SHT31 data invalid (NaN or out of range)");
    }

    data.bme280_valid = !isnan(data.temp_bme280) && !isnan(data.humi_bme280) &&
                        !isnan(data.pres_bme280) &&
                        data.temp_bme280 >= -40 && data.temp_bme280 <= 85 &&
                        data.humi_bme280 >= 0 && data.humi_bme280 <= 100 &&
                        data.pres_bme280 >= 300 && data.pres_bme280 <= 1100;
    if (!data.bme280_valid) {
        Serial.println("ERROR: BME280 data invalid (NaN or out of range)");
    }

    // pms_valid comes from the read status; also reject glitch values
    // that would overflow the smallint columns
    if (data.pms_valid &&
        (data.pm01 > 2000 || data.pm25 > 2000 || data.pm10 > 2000)) {
        data.pms_valid = false;
    }
    if (!data.pms_valid) {
        Serial.println("ERROR: PMS data invalid (read failed or out of range)");
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
    // A dead TCP connection never produces bytes, so getData()/status() alone
    // can leave the waiting states stuck forever - enforce wall-clock progress
    bool waitingState = dbState == DatabaseState::CONNECTING ||
                        dbState == DatabaseState::EXECUTING_QUERY ||
                        dbState == DatabaseState::PROCESSING_RESULT;
    if (waitingState && millis() - dbStateSince > DB_STATE_TIMEOUT) {
        Serial.println("ERROR: Database state timeout, resetting connection");
        resetDatabaseConnection();
        return false;
    }

    switch (dbState) {
        case DatabaseState::DISCONNECTED:
            Serial.print("dbState DatabaseState::DISCONNECTED; ");
            return handleDatabaseConnection();
            
        case DatabaseState::CONNECTING:
            Serial.print("dbState DatabaseState::CONNECTING; ");
            return handleDatabaseConnection();
            
        case DatabaseState::CONNECTED:
            Serial.print("dbState DatabaseState::CONNECTED; ");
            return executeInsertQuery(data);
            
        case DatabaseState::EXECUTING_QUERY:
            Serial.print("dbState DatabaseState::EXECUTING_QUERY; ");
            return processQueryResult();
            
        case DatabaseState::PROCESSING_RESULT:
            Serial.print("dbState DatabaseState::PROCESSING_RESULT; ");
            return processQueryResult();
            
        case DatabaseState::ERROR:
            Serial.print("dbState DatabaseState::ERROR; ");
            resetDatabaseConnection();
            return false;
            
        default:
            Serial.println("dbState ERROR: Unknown database state");
            resetDatabaseConnection();
            return false;
    }
}

bool handleDatabaseConnection() {
    if (dbState == DatabaseState::DISCONNECTED) {
        Serial.println("Connecting to PostgreSQL database...");
        conn.setDbLogin(PG_IP, PG_USER, PG_PASSWORD, PG_DBNAME, "utf8");
        setDbState(DatabaseState::CONNECTING);
        return false; // Not ready yet
    }

    if (dbState == DatabaseState::CONNECTING) {
        int status = conn.status();
        
        if (status == CONNECTION_BAD || status == CONNECTION_NEEDED) {
            const char* error = conn.getMessage();
            handleDatabaseError(error ? error : "Connection failed");
            return false;
        }
        
        if (status == CONNECTION_OK) {
            Serial.println("Database connected successfully");
            setDbState(DatabaseState::CONNECTED);
            return true;
        }
        
        // Still connecting
        return false;
    }

    return false;
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
        // Still processing
        return false;
    }

    // Process different types of results
    if (result & PG_RSTAT_HAVE_COLUMNS) {
        Serial.print("Columns: ");
        for (int i = 0; i < conn.nfields(); i++) {
            if (i > 0) Serial.print(" | ");
            Serial.print(conn.getColumn(i));
        }
        Serial.println();
    }

    if (result & PG_RSTAT_HAVE_ROW) {
        Serial.print("Row: ");
        for (int i = 0; i < conn.nfields(); i++) {
            if (i > 0) Serial.print(" | ");
            const char* value = conn.getValue(i);
            Serial.print(value ? value : "NULL");
        }
        Serial.println();
    }

    if (result & PG_RSTAT_HAVE_SUMMARY) {
        Serial.printf("Query completed - Rows affected: %d\n", conn.ntuples());
    }

    if (result & PG_RSTAT_HAVE_MESSAGE) {
        const char* message = conn.getMessage();
        if (message) {
            Serial.printf("Database message: %s\n", message);
        }
    }

    if (result & PG_RSTAT_READY) {
        Serial.println("Database ready for next query");
        setDbState(DatabaseState::CONNECTED);
        return true; // Operation complete
    }

    // Continue processing
    return false;
}

void handleDatabaseError(const char* error) {
    Serial.printf("Database error: %s\n", error ? error : "Unknown error");

    if (conn.status() == CONNECTION_BAD) {
        Serial.println("Database connection lost");
        resetDatabaseConnection();
    } else {
        setDbState(DatabaseState::ERROR);
    }
}

void resetDatabaseConnection() {
    Serial.println("Resetting database connection");
    conn.close();
    setDbState(DatabaseState::DISCONNECTED);
}
