/*
 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Uncomment section starting with: For ESP32 Dev board (only tested with ILI9341 display)

 To convert images use PNG to XBM converter.
 Ex.: https://convertio.co/ru/
 */
#include <WiFi.h>
#include "SoftwareSerial.h"
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"
#include <TFT_eSPI.h>

#include "tiles/neko_cold.h"

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
    bool valid;
};

// Global objects
WiFiClient client;
char pgBuffer[PG_BUFFER_SIZE];
PGconnection conn(&client, 0, PG_BUFFER_SIZE, pgBuffer);

Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);
SoftwareSerial s8(16,17);  // Sets up a virtual serial port
int pg_status = 0;
bool doWrite = true;
byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  // Command packet to read Co2
byte response[] = {0,0,0,0,0,0,0};  // Create an array to store the response
TFT_eSPI tft = TFT_eSPI();
#define LOOP_PERIOD 35 // Display updates every 35 ms

// Function declarations
bool initializeWiFi();
bool initializeSensors();
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
bool initializeS8();
SensorData readSensorData();
bool validateSensorData(const SensorData& data);
void logSensorData(const SensorData& data);
void sendRequest(byte packet[]);
unsigned long getValue(byte packet[]);
void drawTft(const SensorData& data, String* outdoorData);
void doPg(const SensorData& data);
String* selectPg();

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

    // Initialize WiFi
    if (!initializeWiFi()) {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Connection failed", 13, 158, 4);
    } else {
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Connection established", 13, 158, 4);
    }

    // Initialize sensors
    if (!initializeSensors()) {
        Serial.println("WARNING: Some sensors failed to initialize");
    }

    Serial.println("Setup complete");
}

void loop() {
    // Read all sensor data
    SensorData data = readSensorData();

    // Validate sensor data
    bool dataValid = validateSensorData(data);

    if (dataValid) {
        // Log to Serial
        logSensorData(data);
    } else {
        Serial.println("Invalid sensor data detected, displaying anyway");
    }

    // ALWAYS display data on screen, regardless of WiFi/DB status
    String* outdoorData = NULL;

    // Try to get outdoor data if WiFi is connected and we're in read mode
    if (WiFi.status() == WL_CONNECTED && !doWrite) {
        outdoorData = selectPg();
        if (pg_status == 2) doWrite = true;  // Toggle to write mode next
    }

    // Display data on screen (always)
    drawTft(data, outdoorData);

    // Clean up outdoor data if allocated
    if (outdoorData) {
        delete[] outdoorData;
    }

    // Try to send to database if WiFi is connected, data is valid, and we're in write mode
    if (dataValid) {
        // Check WiFi connection
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected, attempting reconnection...");
            initializeWiFi();
        }

        // Send to database if in write mode and WiFi is connected
        if (WiFi.status() == WL_CONNECTED && doWrite) {
            doPg(data);
            if (pg_status == 2) doWrite = false;  // Toggle to read mode next
        }
    }

    // Delay based on connection status
    if (pg_status >= 2) {
        delay(NORMAL_DELAY);
    } else {
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
    bool s8Success = initializeS8();

    return sht31Success && bme280Success && pmsSuccess && s8Success;
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

bool initializeS8() {
    // Connect to Senseair S8
    Serial.println("Initializing S8 sensor...");
    s8.begin(9600);
    Serial.println("S8 sensor initialized");
    return true; // S8 init doesn't return status
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

    // Read S8 CO2 data
    sendRequest(readCO2);
    data.co2 = getValue(response);

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

    if (data.co2 < 400 || data.co2 > 5000) {
        Serial.println("WARNING: CO2 out of typical range");
        // Don't return false - just warn
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
    Serial.printf("S8     - CO2: %d ppm\n", data.co2);
    Serial.println("=====================");
}

void sendRequest(byte packet[])
{
  while(!s8.available())  //keep sending request until we start to get a response
  {
    s8.write(readCO2,7);
    delay(50);
  }

  int timeout=0;  //set a timeoute counter
  while(s8.available() < 7 ) //Wait to get a 7 byte response
  {
    timeout++;
    if(timeout > 10)    //if it takes to long there was probably an error
      {
        while(s8.available())  //flush whatever we have
          s8.read();
          break;                        //exit and try again
      }
      delay(50);
  }

  for (int i=0; i < 7; i++)
  {
    response[i] = s8.read();
  }
}

unsigned long getValue(byte packet[])
{
  int high = packet[3];  // high byte for value is 4th byte in packet in the packet
  int low = packet[4];   // low byte for value is 5th byte in the packet

  unsigned long val = high*256 + low;  // Combine high byte and low byte with this formula to get value
  return val;
}

void drawTft(const SensorData& data, String* outdoorData)
{
  char tftBuffer[40];

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextSize(1);
  // Indoor temperature and humidity (averaged from both sensors)
  tft.drawString("Temp:",                                    0+13,   0,  4);
  tft.drawString("Humi:",                                    120+23, 0,  4);
  tft.drawFloat((data.temp_sht31 + data.temp_bme280) / 2, 1, 0,      25, 6);
  tft.drawFloat((data.humi_sht31 + data.humi_bme280) / 2, 1, 120+10, 25, 6);

  // Pressure and CO2
  tft.drawString("Press:",                    0+13,   75, 4);
  tft.drawString("CO2:",                      120+23, 75, 4);
  tft.drawFloat(data.pres_bme280,          0, 0,      100, 6);
  if (data.co2 < 1000) {
    tft.drawNumber(data.co2,                  120+10, 100, 6);
  } else {
    tft.drawNumber(data.co2,                  115,    100, 6);
  }

  // Indoor PM values
  sprintf(tftBuffer, "PM: %d %d %d", data.pm01, data.pm25, data.pm10);
  tft.drawString(tftBuffer, 1, 150, 4);

  // Outdoor data (if available)
  if (outdoorData) {
    sprintf(tftBuffer, "PM: %s %s %s", outdoorData[5].c_str(), outdoorData[6].c_str(), outdoorData[7].c_str());
    tft.drawString(tftBuffer, 120+1, 150, 4);

    tft.drawString("Temp:",                                                  0+13, 180, 4);
    tft.drawFloat((outdoorData[0].toInt() + outdoorData[3].toInt()) / 2, 1, 0,    205, 6);
    tft.drawString("Humi:",                                                  0+13, 258, 4);
    tft.drawFloat((outdoorData[2].toInt() + outdoorData[4].toInt()) / 2, 1, 0,    283, 6);
  }

  // Draw cat animation
  tft.drawXBitmap(115, 190, NekoCold, 128, 128, TFT_BLACK, TFT_WHITE);
}

// Подсоединяется к PostgreSQL и отправляет в него данные
void doPg(const SensorData& data)
{
    char *msg;
    int rc;
    if (!pg_status) {
        conn.setDbLogin(PG_IP, PG_USER, PG_PASSWORD, PG_DBNAME, "utf8");
        pg_status = 1;
        Serial.println("Status: connecting to PSQL...");
        return;
    }

    if (pg_status == 1) {
        rc = conn.status();
        if (rc == CONNECTION_BAD || rc == CONNECTION_NEEDED) {
            char *c=conn.getMessage();
            if (c) {Serial.print("Status: ");Serial.println(c);}
            pg_status = -1;
        }
        else if (rc == CONNECTION_OK) {
            pg_status = 2;
            Serial.println("Status: connected.");
            goto status_2;
        }
        return;
    }

  status_2:
    if (pg_status == 2) {
        char query[90];
        snprintf_P(
          query,
          sizeof(query),
          PSTR("INSERT INTO room_mine VALUES (DEFAULT, %.1f, %.1f, %.1f, %.1f, %.1f, %u, %u, %u, %u)"),
          data.temp_bme280, data.pres_bme280, data.humi_bme280,
          data.temp_sht31, data.humi_sht31, data.co2,
          data.pm01, data.pm25, data.pm10
        );

        if (conn.execute(query, false)) goto error;
        pg_status = 3;
        Serial.print("Status: sending SQL ");Serial.println(query);
        return;
    }

    if (pg_status == 3) {
        rc=conn.getData();
        int i;

        if (rc < 0) goto error;
        if (!rc) return;

        if (rc & PG_RSTAT_HAVE_COLUMNS) {
            Serial.print("Status: ");
            for (i=0; i < conn.nfields(); i++) {
                if (i) Serial.print(" | ");
                Serial.print(conn.getColumn(i));
            }
        }
        else if (rc & PG_RSTAT_HAVE_ROW) {
            Serial.print("Status: ");
            for (i=0; i < conn.nfields(); i++) {
                if (i) Serial.print(" | ");
                msg = conn.getValue(i);
                if (!msg) msg=(char *)"NULL";
                Serial.print(msg);
            }
            Serial.println();
        }
        else if (rc & PG_RSTAT_HAVE_SUMMARY) {
            Serial.print("Status: ");
            Serial.print("Rows affected: ");
            Serial.println(conn.ntuples());
        }
        else if (rc & PG_RSTAT_HAVE_MESSAGE) {
            msg = conn.getMessage();
            if (msg) {Serial.print("Status: ");Serial.println(msg);}
        }

        if (rc & PG_RSTAT_READY) {
            pg_status = 2;
            Serial.println("Status: ready for the next query.");
            //goto status_2;
        } else {
            rc=conn.getData();
            if (rc & PG_RSTAT_READY) {
                pg_status = 2;
                Serial.println("Status: ready for the next query.");
                //goto status_2;
            }
        }
    }
    return;

  error:
    msg = conn.getMessage();
    if (msg) Serial.println(msg);
    else Serial.println("UNKNOWN ERROR");
    if (conn.status() == CONNECTION_BAD) {
        Serial.println("Connection is bad");
        pg_status = -1;
    }
    Serial.print("Status:");Serial.println(pg_status);
}

String* selectPg()
{
    static PROGMEM const char query_outside[] = "SELECT \
temperature_bme, pressure_bme, humidity_bme, temperature_sht, humidity_sht, \
pm01, pm25, pm10 FROM outside ORDER BY dt DESC LIMIT 1";

    String* readValues = NULL;
    char *msg;
    int rc;
    if (!pg_status) {
        conn.setDbLogin(PG_IP, PG_USER, PG_PASSWORD, PG_DBNAME, "utf8");
        pg_status = 1;
        Serial.println("Status: connecting to PSQL...");
        return readValues;
    }

    if (pg_status == 1) {
        rc = conn.status();
        if (rc == CONNECTION_BAD || rc == CONNECTION_NEEDED) {
            char *c=conn.getMessage();
            if (c) {Serial.print("Status: ");Serial.println(c);}
            pg_status = -1;
        }
        else if (rc == CONNECTION_OK) {
            pg_status = 2;
            Serial.println("Status: connected.");
            goto status_read_2;
        }
        return readValues;
    }

  status_read_2:
    if (pg_status == 2) {
        if (conn.execute(query_outside, true)) goto read_error;
        pg_status = 3;
        Serial.print("Status: sending SQL ");Serial.println(query_outside);
        return readValues;
    }

  status_read_3:
    if (pg_status == 3) {
        rc=conn.getData();
        int i;

        if (rc < 0) goto read_error;
        if (!rc) return readValues;

        if (rc & PG_RSTAT_HAVE_COLUMNS) {
            for (i=0; i < conn.nfields(); i++) {
                if (i) Serial.print(" | ");
                Serial.print(conn.getColumn(i));
            }
            goto status_read_3;
        }
        else if (rc & PG_RSTAT_HAVE_ROW) {
            readValues = new String[8];
            for (i=0; i < conn.nfields(); i++) {
                if (i) Serial.print(" | ");
                msg = conn.getValue(i);
                readValues[i] = msg;
                if (!msg) msg=(char *)"NULL";
                Serial.print(msg);
            }
            Serial.println();
            goto status_read_3;
        }
        else if (rc & PG_RSTAT_HAVE_SUMMARY) {
            Serial.print("Rows affected: ");
            Serial.println(conn.ntuples());
            goto status_read_3;
        }
        else if (rc & PG_RSTAT_HAVE_MESSAGE) {
            msg = conn.getMessage();
            if (msg) {Serial.print("Status: ");Serial.println(msg);}
            goto status_read_3;
        }

        if (rc & PG_RSTAT_READY) {
            pg_status = 2;
            Serial.println("Status: ready for the next query.");
            //goto status_read_2;
        } else {
            rc=conn.getData();
            if (rc & PG_RSTAT_READY) {
                pg_status = 2;
                Serial.println("Status: ready for the next query.");
                //goto status_read_2;
            }
        }
    }
    return readValues;

  read_error:
    msg = conn.getMessage();
    if (msg) Serial.println(msg);
    else Serial.println("UNKNOWN ERROR");
    if (conn.status() == CONNECTION_BAD) {
        Serial.println("Connection is bad");
        pg_status = -1;
    }
}
