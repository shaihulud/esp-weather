#include <WiFi.h>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"


const char* ssid     = "";
const char* password = "";

const char* pg_user     = "";
const char* pg_password = "";
const char* pg_dbname   = "weather";

const uint8_t PMS_RX_PIN = 34;
const uint8_t PMS_TX_PIN = 33;
const uint8_t SHT31_ADDR = 0x44;
const uint8_t BME280_ADDR = 0x76;

IPAddress PGIP(0, 0, 0, 0);
WiFiClient client;

char pgbuffer[1024];
PGconnection conn(&client, 0, 1024, pgbuffer);

// Sensors config
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);

int pg_status = 0;


void setup() {
    Serial.begin(9600);

    connectToWiFi();
    bool sht31Status = initializeSHT31();
    bool bme280Status = initializeBME280();

    if (!sht31Status || !bme280Status) {
        Serial.println("WARNING: Some sensors failed to initialize.");
    }

    // Connect to PMSx003
    pms.init();
}

void loop() {
    // SHT31
    float temp_sht = sht31.readTemperature();
    float humi_sht = sht31.readHumidity();

    // BME280
    float temp_bme = bme.readTemperature();
    float humi_bme = bme.readHumidity();
    float pres_bme = bme.readPressure() / 133.322F;

    // PMSx003
    pms.read();

    // Show data for debugging
    // logSerial(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, pms.pm01, pms.pm25, pms.pm10);

    // Send to PostgreSQL
    if (WiFi.status() != WL_CONNECTED) connectToWiFi();
    if (WiFi.status() == WL_CONNECTED) doPg(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, pms.pm01, pms.pm25, pms.pm10);

    if (pg_status >= 2) delay(30000);
    else delay(10000);
}

void logSerial(float temp_bme, float pres_bme, float humi_bme, float temp_sht, float humi_sht, unsigned long pm01, unsigned long pm25, unsigned long pm10)
{
    Serial.print("Temp SHT31:  "); Serial.print(temp_sht); Serial.println(" *C");
    Serial.print("Hum SHT31:   "); Serial.print(humi_sht); Serial.println(" %");
    Serial.print("Temp BME280: "); Serial.print(temp_bme); Serial.println(" *C");
    Serial.print("Hum BME280:  "); Serial.print(humi_bme); Serial.println(" %");
    Serial.print("Press BME280: "); Serial.println(pres_bme);
    Serial.print(F("PM0.1: "));Serial.print(pm01);Serial.println(F(" [ug/m3]"));
    Serial.print(F("PM2.5: "));Serial.print(pm25);Serial.println(F(" [ug/m3]"));
    Serial.print(F("PM10:  ")) ;Serial.print(pm10);Serial.println(F(" [ug/m3]"));
    Serial.println();
}

// Подсоединяется к PostgreSQL и отправляет в него данные
void doPg(float temp_bme, float pres_bme, float humi_bme, float temp_sht, float humi_sht, unsigned long pm01, unsigned long pm25, unsigned long pm10)
{
    char *msg;
    int rc;
    if (!pg_status) {
        conn.setDbLogin(PGIP, pg_user, pg_password, pg_dbname, "utf8");
        pg_status = 1;
        Serial.println("Status: connecting to PSQL...");
        return;
    }

    if (pg_status == 1) {
        rc = conn.status();
        if (rc == CONNECTION_BAD || rc == CONNECTION_NEEDED) {
            char *c=conn.getMessage();
            if (c) Serial.println(c);
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
            PSTR("INSERT INTO outside VALUES (DEFAULT, %.1f, %.1f, %.1f, %.1f, %.1f, %u, %u, %u)"),
            temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, pm01, pm25, pm10
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
            Serial.println("\n==========");
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
            goto status_2;
        } else {
            rc=conn.getData();
            if (rc & PG_RSTAT_READY) {
                pg_status = 2;
                Serial.println("Status: ready for the next query.");
                goto status_2;
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

void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("\nConnecting to Wi-Fi");
    unsigned long startAttemptTime = millis();

    // Attempt to connect for 15 seconds
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
        Serial.print(".");
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to Wi-Fi");
        // TODO: Handle failed connection attempt
    } else {
        Serial.println("\nConnected to Wi-Fi");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
}

bool initializeSHT31() {
    Serial.println("Initializing SHT31...");
    for (int attempt = 1; attempt <= 3; attempt++) {  // Retry up to 3 times
        if (sht31.begin(SHT31_ADDR)) {
            Serial.println("SHT31 initialized successfully.");
            return true;
        }
        Serial.printf("SHT31 init attempt %d failed. Retrying...\n", attempt);
        delay(1000);
    }
    Serial.println("ERROR: Failed to initialize SHT31 after multiple attempts.");
    return false;
}

bool initializeBME280() {
    Serial.println("Initializing BME280...");
    for (int attempt = 1; attempt <= 3; attempt++) {  // Retry up to 3 times
        if (bme.begin(BME280_ADDR)) {
            Serial.println("BME280 initialized successfully.");
            return true;
        }
        Serial.printf("BME280 init attempt %d failed. SensorID was: 0x%d. Retrying...\n", bme.sensorID(), attempt);
        delay(1000);
    }
    Serial.println("ERROR: Failed to initialize BME280 after multiple attempts.");
    return false;
}
