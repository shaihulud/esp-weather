/*

 */
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

IPAddress PGIP(0,0,0,0);
WiFiClient client;

char pgbuffer[1024];
PGconnection conn(&client, 0, 1024, pgbuffer);

// Sensors config
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, 34, 33); // PMSx003, RX, TX

int pg_status = 0;


void setup() {
    Serial.begin(9600);

    // Connect to the network
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
        delay(500);
        Serial.print('.');
    }
    Serial.println('\n');
    Serial.println("Connection established");
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP());

    // Connect to SHT31
    if (!sht31.begin(0x44)) {
        Serial.println("Couldn't find SHT31");
    }

    // Connect to BME280
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
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

    // Отправим данные в Serial
  logSerial(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, pms.pm01, pms.pm25, pms.pm10);

  // Отправим данные в PostgreSQL
  doPg(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, pms.pm01, pms.pm25, pms.pm10);

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
