/*
 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Uncomment section starting with: For ESP32 Dev board (only tested with ILI9341 display)

 To convert images use PNG to XBM converter.
 */
#include <WiFi.h>
#include "SoftwareSerial.h"
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"
#include <TFT_eSPI.h>

#include "tiles/neko_cold.h"


const char* ssid     = "";
const char* password = "";

const char* pg_user     = "";
const char* pg_password = "";
const char* pg_dbname   = "weather";

IPAddress PGIP(0,0,0,0);
WiFiClient client;

char pgBuffer[1024];
PGconnection conn(&client, 0, 1024, pgBuffer);
int pg_status = 0;
bool doWrite = true;

// Sensors config
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, 34, 33); // PMSx003, RX, TX
SoftwareSerial s8(16,17);  // Sets up a virtual serial port

byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  // Command packet to read Co2
byte response[] = {0,0,0,0,0,0,0};  // Create an array to store the response


TFT_eSPI tft = TFT_eSPI();
#define LOOP_PERIOD 35 // Display updates every 35 ms


void setup() {
    Serial.begin(9600);

    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString("Connecting to WiFi", 13, 158, 4);

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
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Connection established", 13, 158, 4);

    // Connect to Senseair S8
    s8.begin(9600);

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
  unsigned long pm01 = pms.pm01;
  unsigned long pm25 = pms.pm25;
  unsigned long pm10 = pms.pm10;
  //   if (pms) {
  //   }

  // Senseair S8
  sendRequest(readCO2);
  unsigned long co2val = getValue(response);

  // Отправим данные в Serial
  logSerial(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val, pms.pm01, pms.pm25, pms.pm10);

  // Отправим данные в PostgreSQL
  String* readValues = NULL;
  if (doWrite) {
    doPg(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val, pms.pm01, pms.pm25, pms.pm10);
    if (pg_status == 2) doWrite = false;
  }
  else {
    readValues = selectPg();
    if (pg_status == 2) doWrite = true;
  }

  if (readValues) {
    // Отправим данные на экран
    drawTft(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val, pms.pm01, pms.pm25, pms.pm10, readValues);
    delete[] readValues;
  }

  if (pg_status >= 2) delay(15000);
  else delay(10000);
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

void drawTft(
  float temp_bme,
  float pres_bme,
  float humi_bme,
  float temp_sht,
  float humi_sht,
  unsigned long co2val,
  unsigned long pm01,
  unsigned long pm25,
  unsigned long pm10,
  String* readValues
)
{
  char tftBuffer[40];

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextSize(1);
  tft.drawString("Temp:",                     0+13,   0,  4);
  tft.drawString("Humi:",                     120+23, 0,  4);
  tft.drawFloat((temp_sht + temp_bme) / 2, 1, 0,      25, 6);
  tft.drawFloat((humi_sht + humi_bme) / 2, 1, 120+10, 25, 6);

  tft.drawString("Press:",                    0+13,   75, 4);
  tft.drawString("CO2:",                      120+23, 75, 4);
  tft.drawFloat(pres_bme,                  0, 0,      100, 6);
  if (co2val < 1000) {
    tft.drawNumber(co2val,                    120+10, 100, 6);
  } else {
    tft.drawNumber(co2val,                    115,    100, 6);
  }

  sprintf(tftBuffer, "PM: %d %d %d", pms.pm01, pms.pm25, pms.pm10);
  tft.drawString(tftBuffer, 1, 150, 4);

  if (readValues) {
    sprintf(tftBuffer, "PM: %s %s %s", readValues[5], readValues[6], readValues[7]);
    tft.drawString(tftBuffer, 120+1, 150, 4);

    tft.drawString("Temp:",                                               0+13, 180, 4);
    tft.drawFloat((readValues[0].toInt() + readValues[3].toInt()) / 2, 1, 0,    205, 6);
    tft.drawString("Humi:",                                               0+13, 258, 4);
    tft.drawFloat((readValues[2].toInt() + readValues[4].toInt()) / 2, 1, 0,    283, 6);
  }

  // tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.drawXBitmap(115, 190, NekoCold, 128, 128, TFT_BLACK, TFT_WHITE);
}

void logSerial(float temp_bme, float pres_bme, float humi_bme, float temp_sht, float humi_sht, unsigned long co2val, unsigned long pm01, unsigned long pm25, unsigned long pm10)
{
  Serial.print("Temp SHT31:  "); Serial.print(temp_sht); Serial.println(" *C");
  Serial.print("Hum SHT31:   "); Serial.print(humi_sht); Serial.println(" %");
  Serial.print("Temp BME280: "); Serial.print(temp_bme); Serial.println(" *C");
  Serial.print("Hum BME280:  "); Serial.print(humi_bme); Serial.println(" %");
  Serial.print("Press BME280: "); Serial.println(pres_bme);
  Serial.print(F("PM0.1: "));Serial.print(pm01);Serial.println(F(" [ug/m3]"));
  Serial.print(F("PM2.5: "));Serial.print(pm25);Serial.println(F(" [ug/m3]"));
  Serial.print(F("PM10:  ")) ;Serial.print(pm10);Serial.println(F(" [ug/m3]"));
  Serial.print("CO2 ppm = "); Serial.println(co2val);
  Serial.println();
}

// Подсоединяется к PostgreSQL и отправляет в него данные
void doPg(float temp_bme, float pres_bme, float humi_bme, float temp_sht, float humi_sht, unsigned long co2val, unsigned long pm01, unsigned long pm25, unsigned long pm10)
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
          temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val, pm01, pm25, pm10
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
        conn.setDbLogin(PGIP, pg_user, pg_password, pg_dbname, "utf8");
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
