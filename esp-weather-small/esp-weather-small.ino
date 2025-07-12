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
const uint8_t MAX_INIT_ATTEMPTS = 3;

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
    bool valid;
};

// Global objects
WiFiClient client;
char pgBuffer[PG_BUFFER_SIZE];
PGconnection conn(&client, 0, PG_BUFFER_SIZE, pgBuffer);

Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);

DatabaseState dbState = DatabaseState::DISCONNECTED;

// Function declarations
bool initializeWiFi();
bool initializeSensors();
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
SensorData readSensorData();
bool validateSensorData(const SensorData& data);
void logSensorData(const SensorData& data);
bool handleDatabaseConnection();
bool sendDataToDatabase(const SensorData& data);
void handleDatabaseError(const char* error);
void resetDatabaseConnection();

void setup() {
    Serial.begin(9600);
    Serial.println("\nESP32 Weather Station Starting...");

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
    }

    // Read sensor data
    SensorData data = readSensorData();

    // Validate and log data
    if (validateSensorData(data)) {
        logSensorData(data);

        // Send to database
        if (sendDataToDatabase(data)) {
            Serial.println("Data sent successfully");
            delay(NORMAL_DELAY);
        } else {
            Serial.println("Database operation failed");
            delay(ERROR_DELAY);
        }
    } else {
        Serial.println("Invalid sensor data, skipping this cycle");
        delay(ERROR_DELAY);
    }
}

bool initializeWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.println("Connecting to WiFi...");
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
    data.pm01 = pms.pm01;
    data.pm25 = pms.pm25;
    data.pm10 = pms.pm10;

    return data;
}

bool validateSensorData(const SensorData& data) {
    // Check for NaN values
    if (isnan(data.temp_sht31) || isnan(data.humi_sht31) || 
        isnan(data.temp_bme280) || isnan(data.humi_bme280) || 
        isnan(data.pres_bme280)) {
        Serial.println("ERROR: NaN values detected in sensor data");
        return false;
    }

    // Check reasonable ranges
    if (data.temp_sht31 < -40 || data.temp_sht31 > 85 ||
        data.temp_bme280 < -40 || data.temp_bme280 > 85) {
        Serial.println("ERROR: Temperature out of valid range");
        return false;
    }

    if (data.humi_sht31 < 0 || data.humi_sht31 > 100 ||
        data.humi_bme280 < 0 || data.humi_bme280 > 100) {
        Serial.println("ERROR: Humidity out of valid range");
        return false;
    }

    if (data.pres_bme280 < 300 || data.pres_bme280 > 1100) {
        Serial.println("ERROR: Pressure out of valid range");
        return false;
    }

    return true;
}

void logSensorData(const SensorData& data) {
    Serial.println("=== Sensor Readings ===");
    Serial.printf("SHT31  - Temp: %.2f°C, Humidity: %.2f%%\n", data.temp_sht31, data.humi_sht31);
    Serial.printf("BME280 - Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2f mmHg\n", 
                  data.temp_bme280, data.humi_bme280, data.pres_bme280);
    Serial.printf("PMS    - PM1.0: %d µg/m³, PM2.5: %d µg/m³, PM10: %d µg/m³\n", 
                  data.pm01, data.pm25, data.pm10);
    Serial.println("=====================");
}

bool sendDataToDatabase(const SensorData& data) {
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
        dbState = DatabaseState::CONNECTING;
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
            dbState = DatabaseState::CONNECTED;
            return true;
        }
        
        // Still connecting
        return false;
    }

    return false;
}

bool executeInsertQuery(const SensorData& data) {
    char query[QUERY_BUFFER_SIZE];

    int result = snprintf(query, sizeof(query),
        "INSERT INTO outside VALUES (DEFAULT, %.2f, %.2f, %.2f, %.2f, %.2f, %u, %u, %u)",
        data.temp_bme280, data.pres_bme280, data.humi_bme280, 
        data.temp_sht31, data.humi_sht31, 
        data.pm01, data.pm25, data.pm10);

    if (result >= sizeof(query)) {
        Serial.println("ERROR: Query buffer overflow");
        return false;
    }

    Serial.printf("Executing query: %s\n", query);

    if (conn.execute(query, false) < 0) {
        handleDatabaseError("Query execution failed");
        return false;
    }

    dbState = DatabaseState::EXECUTING_QUERY;
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
        dbState = DatabaseState::CONNECTED;
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
        dbState = DatabaseState::ERROR;
    }
}

void resetDatabaseConnection() {
    Serial.println("Resetting database connection");
    dbState = DatabaseState::DISCONNECTED;
    // Note: The SimplePgSQL library may not have an explicit disconnect method
    // The connection will be reset on next connection attempt
}
