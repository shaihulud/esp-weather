# esp-weather

Two ESP32 weather stations that write sensor data straight into PostgreSQL and show on the screen. Grafana draws the graphs.

- **Indoor** (`esp-weather/`): BME280, SHT31, PMSA003, Senseair S8 (CO2) and an ILI9341 screen. Shows readings from both stations and a catgirl dressed for the weather outside.
- **Outdoor** (`esp-weather-small/`): same sensors minus the screen and CO2.

Each station reads its sensors every 30 seconds and inserts a row (`room_mine` / `outside`). If one sensor fails, its columns go NULL and the rest is still written. Stations survive WiFi drops and dead database connections on their own, and reboot themselves if nothing was written for 10 minutes.

## Setup

Server side (any machine with Docker):

```
docker-compose up -d
psql -h localhost -U postgres -d weather -f db.sql
```

Put Postgres and Grafana passwords in `.env`. Import `Grafana/MyRoomWeather.json` into Grafana at http://localhost:3000/grafana/.

Firmware (Arduino IDE, ESP32 board):

1. Install libraries: Adafruit_BME280, Adafruit_SHT31, PMserial, SimplePgSQL, and TFT_eSPI for the indoor station.
2. Indoor only: in TFT_eSPI's `User_Setup.h`, uncomment the "For ESP32 Dev board" section.
3. In each sketch folder, copy `secrets.h.example` to `secrets.h` and fill in WiFi and database credentials.
4. Flash. Serial monitor runs at 9600.

## Grafana dashboard

![Grafana](https://github.com/shaihulud/esp-weather/blob/main/img/Grafana.png?raw=true)

## The catgirl

The indoor screen shows a catgirl dressed for the current outdoor weather: scarf in the cold, snow coat below zero, umbrella in the rain, hand fan in the heat, face mask when PM2.5 is high. Tiles are 128x128 XBM headers in `esp-weather/tiles/`, picked by `pickCat()` in the indoor sketch.
