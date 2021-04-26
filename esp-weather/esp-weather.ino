/*
     CO2, temperature, humidity and pressure monitoring with ESP32.
     Licensed under MIT license.
*/
#include <WiFi.h>
//#include <Wire.h>
//#include <SPIFFS.h>

#include "SoftwareSerial.h"
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <SimplePgSQL.h>

const char* ssid     = "SSID NAME HERE";
const char* password = "PASSWORD HERE";

const char* pg_user     = "POSTGRES USER HERE";
const char* pg_password = "POSTGRES PASSWORD HERE";
const char* pg_dbname   = "weather";


IPAddress PGIP(192,168,1,5);
WiFiClient client;

char buffer[1024];
PGconnection conn(&client, 0, 1024, buffer);

// Sensors config
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SoftwareSerial s8(16,17);  //Sets up a virtual serial port

byte readCO2[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};  //Command packet to read Co2
byte response[] = {0,0,0,0,0,0,0};  //create an array to store the response


//String getConfig(const String& fn) {
//    File f = SPIFFS.open(fn, "r");
//    String r = f.readString();
//    f.close();
//    return r;
//}


int pg_status = 0;

// Подсоединяется к PostgreSQL и отправляет в него данные
void doPg(float temp_bme, float pres_bme, float humi_bme, float temp_sht, float humi_sht, unsigned long co2val)
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
          PSTR("INSERT INTO room_mine VALUES (DEFAULT, %.1f, %.1f, %.1f, %.1f, %.1f, %u)"), 
          temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val
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


void setup() {
    Serial.begin(9600);

    //SPIFFS.begin(true);  // Will format on the first run after failing to mount
    //String ssid = getConfig("/wifi-ssid");
    //String pass = getConfig("/wifi-password");

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

    GetExternalIP();

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
}

void loop() {
  // SHT31
  float temp_sht = sht31.readTemperature();
  float humi_sht = sht31.readHumidity();
  Serial.print("Temp SHT31: "); Serial.print(temp_sht); Serial.println(" *C"); 
  Serial.print("Hum SHT31: "); Serial.print(humi_sht); Serial.println(" %");

  // BME280
  float temp_bme = bme.readTemperature();
  float humi_bme = bme.readHumidity();
  float pres_bme = bme.readPressure() / 133.322F;
  Serial.print("Temp BME280: "); Serial.print(temp_bme); Serial.println(" *C");
  Serial.print("Hum BME280: "); Serial.print(humi_bme); Serial.println(" %");
  Serial.print("Press BME280: "); Serial.println(pres_bme);

  // Senseair S8
  sendRequest(readCO2);
  unsigned long co2val = getValue(response);
  Serial.print("CO2 ppm = "); Serial.println(co2val);

  // Send data to PostgreSQL
  doPg(temp_bme, pres_bme, humi_bme, temp_sht, humi_sht, co2val);

  // Wait 30sec till next read
  Serial.println();
  delay(30000);
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
    int high = packet[3];                        //high byte for value is 4th byte in packet in the packet
    int low = packet[4];                         //low byte for value is 5th byte in the packet

    unsigned long val = high*256 + low;                //Combine high byte and low byte with this formula to get value
    return val;
}

void GetExternalIP() {
  if (!client.connect("api.ipify.org", 80)) {
    Serial.println("Failed to connect with 'api.ipify.org' !");
  }
  else {
    int timeout = millis() + 5000;
    client.print("GET /?format=json HTTP/1.1\r\nHost: api.ipify.org\r\n\r\n");
    while (client.available() == 0) {
      if (timeout - millis() < 0) {
        Serial.println(">>> Client Timeout !");
        client.stop();
        return;
      }
    }
    int size;
    while ((size = client.available()) > 0) {
      uint8_t* msg = (uint8_t*)malloc(size);
      size = client.read(msg, size);
      Serial.write(msg, size);
      free(msg);
      Serial.println("\n\n");
    }
  }
}
