/*
 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Uncomment section starting with: For ESP32 Dev board (only tested with ILI9341 display)

 To convert images use PNG to XBM converter.
 Ex.: https://convertio.co/ru/
 */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include <esp_task_wdt.h>
#include <atomic>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include "SimplePgSQL.h"
#include <TFT_eSPI.h>
#include <WeatherLink.h>  // libraries/WeatherLink, symlinked into the sketchbook

#include "tiles/neko_cold.h"
#include "tiles/neko_hot.h"
#include "tiles/neko_pollution.h"
#include "tiles/neko_rain.h"
#include "tiles/neko_snow.h"
#include "tiles/neko_summer.h"

// WiFi, PostgreSQL and ESP-NOW credentials live in secrets.h (gitignored);
// copy secrets.h.example to secrets.h and fill in your values
#include "secrets.h"

#ifndef SECRETS_STATION_INDOOR
#error "Wrong secrets.h: this is the indoor station, use its own secrets file"
#endif

using weatherlink::OutdoorPacket;
using weatherlink::monoMs;

// Test hooks driven by single-letter serial commands (see
// handleFaultInjection); must stay 0 in production builds
#define FAULT_INJECTION 0

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
const unsigned long NORMAL_DELAY = 30000;            // sample start-to-start
const unsigned long SENSOR_INIT_DELAY = 1000;
const unsigned long S8_RESPONSE_TIMEOUT = 1000;
const unsigned long WIFI_CONNECT_TIMEOUT = 15000;
const unsigned long WIFI_STOP_TIMEOUT = 2000;        // wait for the driver to report a cancelled attempt
const unsigned long WIFI_STOP_SETTLE_MS = 200;
const unsigned long CHANNEL_SETTLE_MS = 600;         // a restored channel must hold this long
const unsigned long CHANNEL_RESTORE_TIMEOUT = 3000;
const unsigned long WIFI_RETRY_MIN = 30000;          // reconnect backoff, with jitter
const unsigned long WIFI_RETRY_MAX = 300000;
const unsigned long RADIO_RESTART_MIN_INTERVAL = 60000;
const unsigned long DB_STATE_TIMEOUT = 30000;
const unsigned long DB_BACKOFF_MIN = 10000;          // after a connection failure, with jitter
const unsigned long DB_BACKOFF_MAX = 300000;
const unsigned long DB_JOB_MAX_AGE_MS = 90000;       // older measurements are dropped, never replayed
const unsigned long LINK_RETRY_MIN = 2000;           // ESP-NOW init retry
const unsigned long LINK_RETRY_MAX = 60000;
const unsigned long LINK_QUIESCE_MS = 500;
const unsigned long OUTDOOR_STALE_SECONDS = 300;     // grey out outdoor data older than this
const unsigned long STATUS_REFRESH_MS = 1000;
const unsigned long STATS_LOG_INTERVAL = 600000;
const unsigned long LOOP_TICK_MS = 10;
const uint32_t TASK_WDT_TIMEOUT_MS = 60000;          // task-health only, never availability

// Weather-cat thresholds (outdoor conditions)
const int CAT_PM25_POLLUTION = 35;      // µg/m³: mask goes on
const float CAT_RAIN_HUMIDITY = 93.0;   // %: no rain sensor, so near-saturation = precipitation
const float CAT_SNOW_MAX_TEMP = 0.0;    // °C: freezing means snow gear, always
const float CAT_COLD_MAX_TEMP = 15.0;   // °C: scarf weather below this
const float CAT_HOT_MIN_TEMP = 26.0;    // °C: hand-fan weather above this
const uint8_t MAX_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_REINIT_THRESHOLD = 5;  // consecutive bad cycles before re-init

// Buffer configuration
const size_t QUERY_BUFFER_SIZE = 256;
const UBaseType_t RX_QUEUE_LENGTH = 4;
const UBaseType_t DB_QUEUE_LENGTH = 4;
const uint32_t DB_WORKER_STACK = 8192;

// Status band on the display: outdoor link state (grey), uplink state (red)
const int STATUS_X = 121;
const int STATUS_Y = 172;
const int STATUS_W = 119;
const int STATUS_H = 17;
const int STATUS_GAP = 6;

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

// Latest accepted packet from the outdoor station, owned by loop(). Invalid
// groups are NAN (PM: 0 with the flag cleared).
struct OutdoorData {
    float temp_bme;
    float pres_bme;
    float humi_bme;
    float temp_sht;
    float humi_sht;
    int pm01;
    int pm25;
    int pm10;
    uint8_t validFlags;        // weatherlink::VALID_*
    uint32_t sampleAgeMs;      // sample age when the packet was sent
    uint64_t receivedAtMs;     // monoMs() taken in the receive callback
    bool valid;                // a packet was accepted since boot
};

// Outdoor packet as received, timestamped in the receive callback
struct RxItem {
    uint8_t bytes[sizeof(OutdoorPacket)];
    uint64_t receivedAtMs;
};

enum class DbTable : uint8_t { ROOM, OUTSIDE };

// One row for the DB worker. Measurement age at time T is
// (T - capturedAtMs) + priorAgeMs, so queueing time is always counted.
struct DbJob {
    DbTable table;
    SensorData data;
    uint64_t capturedAtMs;     // indoor: sample time; outdoor: reception time
    uint32_t priorAgeMs;       // outdoor: sample age at transmission
    uint32_t wifiGeneration;   // WiFi session the job was queued in
};

enum class DbResult { OK, SQL_ERROR, CONN_ERROR, DROPPED };
enum class DbConn : uint8_t { UNKNOWN, UP, DOWN, NO_WORKER };  // not DISABLED: Arduino macro
enum class TableStatus : uint8_t { UNKNOWN, OK, SQL_ERR };

// Published as a whole by the DB worker (authoritative), read by loop()
// for the display and as an early filter
struct DbStatus {
    DbConn conn;
    TableStatus room;
    TableStatus outside;
    uint64_t retryAtMs;        // while DOWN: no DB work before this
    uint64_t discardBeforeMs;  // outage watermark: never write measurements older than this
};

// Published as a whole by loop(), read by the DB worker
struct WifiStatus {
    bool up;
    uint32_t generation;       // +1 per new connection
    uint64_t upSinceMs;        // start of the current connection
};

enum class WifiState {
    IDLE,         // listening for ESP-NOW (verified channel), waiting for the next attempt
    CONNECTING,
    CONNECTED,
    STOPPING,     // attempt/connection cancelled, waiting for confirmation
    RESTORING,    // putting the radio back on the listening channel
    RADIO_FAULT   // radio restart failed or deferred: unverified, waiting to retry
};

// Global objects
// Owned by dbWorker: socket, connection and all DB state-machine variables
WiFiClient client;
char pgBuffer[PG_BUFFER_SIZE];
PGconnection conn(&client, 0, PG_BUFFER_SIZE, pgBuffer);

Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);
TFT_eSPI tft = TFT_eSPI();

byte s8ReadCommand[] = {0xFE, 0x44, 0x00, 0x08, 0x02, 0x9F, 0x25};

// --- dbWorker task only ---
DatabaseState dbState = DatabaseState::DISCONNECTED;
unsigned long dbStateSince = 0;
bool queryHadError = false;   // server rejected the current query
bool dbTimedOut = false;      // stuck-state reset during the current job
bool jobDropped = false;      // pre-submission check rejected the current job
const DbJob* pendingJob = nullptr;
bool workerWdtActive = false;
uint64_t discardBeforeMs = 0; // outage watermark (published in DbStatus)

// --- loop task only ---
uint8_t sht31Failures = 0;    // consecutive invalid readings, re-init at threshold
uint8_t bme280Failures = 0;
uint8_t pmsFailures = 0;
uint8_t s8Failures = 0;

bool wdtConfigured = false;
bool loopWdtActive = false;
uint64_t nextSampleAt = 0;
unsigned long maxLoopPassMs = 0;

SensorData indoor = {};
OutdoorData outdoor = {};
bool outdoorSeen = false;
uint32_t lastOutdoorBootId = 0;
uint32_t lastOutdoorSeq = 0;
bool outdoorWasStale = false;

WifiState wifiState = WifiState::IDLE;
uint64_t wifiStateSince = 0;
uint64_t wifiNextAttemptAt = 0;
unsigned long wifiRetryMs = WIFI_RETRY_MIN;
uint8_t channelMin = weatherlink::CHANNEL_MIN;  // allowed channels (shared range
uint8_t channelMax = weatherlink::CHANNEL_MAX;  // limited by the regulatory domain)
uint8_t routerChannel = weatherlink::DEFAULT_CHANNEL;
uint8_t listenChannel = weatherlink::DEFAULT_CHANNEL;
uint32_t attemptEventsBaseline = 0;
uint32_t stopEventsBaseline = 0;
uint32_t connectedEventsBaseline = 0;
bool channelSet = false;
uint64_t channelSetAt = 0;
bool radioRestarted = false;
uint64_t lastRadioRestartAt = 0;
uint64_t radioRetryAt = 0;    // RADIO_FAULT: next restart attempt
bool radioVerified = false;   // radio confirmed on the listening channel
bool radioRestartRequested = false;

bool linkEnabled = false;     // rx queue exists and config is valid
bool linkReady = false;       // ESP-NOW session up
bool linkSessionDirty = false; // a session could not be closed: only a radio restart clears it
uint64_t linkRetryAt = 0;
unsigned long linkRetryMs = LINK_RETRY_MIN;

bool statusCacheValid = false;
char statusCacheLink[12] = "";
const char* statusCacheUplink = "";

// --- shared between tasks ---
QueueHandle_t rxQueue = nullptr;    // receive callback -> loop
QueueHandle_t dbQueue = nullptr;    // loop -> dbWorker
TaskHandle_t dbWorkerHandle = nullptr;

portMUX_TYPE wifiStatusMux = portMUX_INITIALIZER_UNLOCKED;
WifiStatus wifiStatus = {false, 0, 0};
portMUX_TYPE dbStatusMux = portMUX_INITIALIZER_UNLOCKED;
DbStatus dbStatus = {DbConn::UNKNOWN, TableStatus::UNKNOWN, TableStatus::UNKNOWN, 0, 0};

// Written by the Arduino event task
portMUX_TYPE wifiEventMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t staDisconnectEvents = 0;
uint8_t lastDisconnectReason = 0;
bool coreRetryUsed = false;
bool coreRetryPending = false;

// Counters, each written by a single task
std::atomic<uint32_t> rxQueueDrops{0};     // receive callback
std::atomic<uint32_t> rxForeign{0};
std::atomic<uint32_t> rxBadEnvelope{0};
uint32_t rxAccepted = 0;                   // loop
uint32_t rxRejected = 0;
uint32_t rxTooOld = 0;
uint32_t rxDuplicates = 0;
uint32_t rxOutOfOrder = 0;
uint32_t rxMissed = 0;
uint32_t outdoorRestarts = 0;
uint32_t jobsNotQueued = 0;
uint32_t jobsQueueFull = 0;
uint32_t radioRestarts = 0;
std::atomic<uint32_t> dbInserted{0};       // dbWorker
std::atomic<uint32_t> dbSqlErrors{0};
std::atomic<uint32_t> dbConnErrors{0};
std::atomic<uint32_t> dbJobsDropped{0};
std::atomic<unsigned long> maxJobMs{0};

#if FAULT_INJECTION
uint64_t fiStallRxUntil = 0;
bool fiChannelFail = false;
#endif

// Function declarations
bool initTaskWatchdog();
void feedLoopWatchdog();
void feedWorkerWatchdog();
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
void takeIndoorSample();
WifiStatus readWifiStatus();
void publishWifiStatus(bool up);
DbStatus readDbStatus();
void publishDbStatus(const DbStatus& status);
unsigned long withJitter(unsigned long ms);
void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info);
void maintainWiFi();
void onWifiConnected(uint64_t now);
void beginStopping(uint64_t now, bool knownIdle);
void enterRestoring(uint64_t now);
void serviceRestoring(uint64_t now);
void enterWifiState(WifiState state, uint64_t now);
void scheduleNextWifiAttempt(uint64_t now);
void restartRadio(uint64_t now);
void enterRadioFault(uint64_t now, uint64_t retryAt, const char* why);
void readAllowedChannels();
bool channelAllowed(uint8_t ch);
uint8_t fallbackChannel();
bool setRadioChannel(uint8_t ch);
bool radioOnChannel(uint8_t ch);
bool allZero(const uint8_t* bytes, size_t len);
bool validateLinkConfig();
bool linkInit();
bool closeEspNowSession();
bool linkTeardown();
void maintainLink(uint64_t now);
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
bool drainOutdoorRx();
bool acceptSequence(const OutdoorPacket& pkt);
void applyOutdoorPacket(const OutdoorPacket& pkt, uint64_t receivedAtMs);
SensorData outdoorAsSensorData();
uint64_t outdoorDataAgeMs();
bool outdoorIsStale();
void enqueueDbJob(DbTable table, const SensorData& data, uint64_t capturedAtMs, uint32_t priorAgeMs);
const char* tableName(DbTable table);
uint64_t jobAgeMs(const DbJob& job, uint64_t now);
uint64_t jobMeasuredAtMs(const DbJob& job);
const char* jobRejectReason(const DbJob& job, const WifiStatus& wifi, uint64_t dbWatermarkMs,
                            uint64_t now);
void dbWorker(void* param);
bool formatInsert(const DbJob& job, uint64_t now, char* query, size_t size);
DbResult runQuery(const DbJob& job);
bool advanceDbStateMachine();
void startDbConnection();
void pollDbConnection();
bool dispatchPendingQuery();
bool processQueryResult();
void handleDatabaseError(const char* error);
void resetDatabaseConnection();
void setDbState(DatabaseState newState);
const unsigned char* pickCat(const OutdoorData& outdoor);
void drawValue(float value, uint8_t decimals, int32_t x, int32_t y);
void drawTft(const SensorData& data, const OutdoorData& outdoor);
void drawOutdoor(const OutdoorData& outdoor);
void outdoorStatusText(char* buf, size_t size);
const char* uplinkStatusText();
void drawStatus();
void checkStatusFit();
void logStats();
#if FAULT_INJECTION
void handleFaultInjection();
#endif

void setup() {
    Serial.begin(9600);
    Serial.println("\nESP32 Weather Station Starting...");

    // Initialize TFT
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString("Starting...", 13, 158, 4);

    loopWdtActive = initTaskWatchdog();

    // Queues first: the ESP-NOW receive callback and the DB worker use them
    rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(RxItem));
    if (!rxQueue) {
        Serial.println("ERROR: rx queue allocation failed, outdoor link disabled");
    }
    dbQueue = xQueueCreate(DB_QUEUE_LENGTH, sizeof(DbJob));
    if (!dbQueue) {
        Serial.println("ERROR: DB queue allocation failed, database disabled");
    }

    // Station mode only; persistent(false) avoids rewriting credentials
    // to NVS flash on every begin(). No modem sleep: it drops ESP-NOW frames
    // while associated. Reconnects are paced by maintainWiFi(), not the core.
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(false);
    WiFi.onEvent(onWiFiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());

    // Cold boot: listen on the configured default channel until the router
    // tells us otherwise
    readAllowedChannels();
    listenChannel = fallbackChannel();
    routerChannel = listenChannel;
    radioVerified = setRadioChannel(listenChannel);
    if (!radioVerified) {
        // maintainWiFi() keeps retrying (and escalates); ESP-NOW waits
        Serial.println("WARNING: could not set the default channel");
        enterRestoring(monoMs());
    }

    if (!initializeSensors()) {
        Serial.println("WARNING: Some sensors failed to initialize");
    }

    checkStatusFit();

    // ESP-NOW last, once everything its callback touches exists; on an
    // unverified radio maintainLink() opens it after RESTORING succeeds
    linkEnabled = rxQueue != nullptr && validateLinkConfig();
    if (linkEnabled && radioVerified) {
        linkReady = linkInit();
        if (!linkReady) {
            linkRetryAt = monoMs() + linkRetryMs;
        }
    }

    if (dbQueue && xTaskCreate(dbWorker, "dbWorker", DB_WORKER_STACK, nullptr, 1,
                               &dbWorkerHandle) == pdPASS) {
        Serial.println("DB worker started");
    } else {
        dbWorkerHandle = nullptr;
        Serial.println("ERROR: DB worker could not start, database disabled");
        DbStatus disabled = {DbConn::NO_WORKER, TableStatus::UNKNOWN, TableStatus::UNKNOWN, 0, 0};
        publishDbStatus(disabled);
    }

    nextSampleAt = monoMs();
    Serial.println("Setup complete");
}

void loop() {
    uint64_t passStart = monoMs();
    feedLoopWatchdog();

#if FAULT_INJECTION
    handleFaultInjection();
#endif

    maintainWiFi();
    maintainLink(monoMs());
    bool outdoorDirty = drainOutdoorRx();

    // Fixed start-to-start sampling, independent of WiFi and DB; after a
    // long stall the schedule moves forward instead of bursting to catch up
    uint64_t now = monoMs();
    if (now >= nextSampleAt) {
        nextSampleAt += NORMAL_DELAY;
        if (nextSampleAt <= now) {
            nextSampleAt = now + NORMAL_DELAY;
        }
        takeIndoorSample();
        drawTft(indoor, outdoor);
        outdoorDirty = false;
    }

    static uint64_t lastStatusAt = 0;
    now = monoMs();
    if (now - lastStatusAt >= STATUS_REFRESH_MS) {
        lastStatusAt = now;
        // Grey out (or back) the outdoor block when its data crosses the
        // staleness threshold in either direction
        if (outdoorIsStale() != outdoorWasStale) {
            outdoorDirty = true;
        }
        drawStatus();
    }
    if (outdoorDirty) {
        drawOutdoor(outdoor);
        drawStatus();
    }

    static uint64_t lastStatsAt = 0;
    if (monoMs() - lastStatsAt >= STATS_LOG_INTERVAL) {
        lastStatsAt = monoMs();
        logStats();
    }

    maxLoopPassMs = max(maxLoopPassMs, (unsigned long)(monoMs() - passStart));
    delay(LOOP_TICK_MS);
}

// Reconfigures the task watchdog (default 5s) to TASK_WDT_TIMEOUT_MS and
// subscribes the loop task. On any failure the task stays unsubscribed, so
// it never ends up on the 5s default; the DB worker subscribes only if the
// reconfiguration succeeded.
bool initTaskWatchdog() {
    esp_task_wdt_config_t config = {
        .timeout_ms = TASK_WDT_TIMEOUT_MS,
        .idle_core_mask = 1 << 0,  // core default: watch CPU0's idle task
        .trigger_panic = true,
    };
    esp_err_t err = esp_task_wdt_reconfigure(&config);
    if (err != ESP_OK) {
        Serial.printf("WARNING: task watchdog reconfigure failed (%s), running without it\n",
                      esp_err_to_name(err));
        return false;
    }
    wdtConfigured = true;
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        return true;  // already subscribed
    }
    err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        Serial.printf("WARNING: task watchdog subscribe failed (%s), loop unwatched\n",
                      esp_err_to_name(err));
        return false;
    }
    return true;
}

void feedLoopWatchdog() {
    if (loopWdtActive) {
        esp_task_wdt_reset();
    }
}

void feedWorkerWatchdog() {
    if (workerWdtActive) {
        esp_task_wdt_reset();
    }
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

void takeIndoorSample() {
    uint64_t sampledAt = monoMs();
    SensorData data = readSensorData();

    // Validate per sensor and re-init sensors that keep failing;
    // a single bad sensor must not block the others from being written
    bool anyValid = validateSensorData(data);
    handleSensorHealth(data);
    logSensorData(data);
    indoor = data;

    if (anyValid) {
        // Invalid sensors are written as NULL
        enqueueDbJob(DbTable::ROOM, data, sampledAt, 0);
    } else {
        Serial.println("No valid sensor data, skipping insert");
    }
}

// ---------------------------------------------------------------------------
// Cross-task state

WifiStatus readWifiStatus() {
    portENTER_CRITICAL(&wifiStatusMux);
    WifiStatus status = wifiStatus;
    portEXIT_CRITICAL(&wifiStatusMux);
    return status;
}

// Loop only. A new connection starts a new generation: jobs queued in an
// older one, or measured before this connection started, are discarded
void publishWifiStatus(bool up) {
    uint64_t now = monoMs();
    portENTER_CRITICAL(&wifiStatusMux);
    if (up && !wifiStatus.up) {
        wifiStatus.generation++;
        wifiStatus.upSinceMs = now;
    }
    wifiStatus.up = up;
    portEXIT_CRITICAL(&wifiStatusMux);
}

DbStatus readDbStatus() {
    portENTER_CRITICAL(&dbStatusMux);
    DbStatus status = dbStatus;
    portEXIT_CRITICAL(&dbStatusMux);
    return status;
}

void publishDbStatus(const DbStatus& status) {
    portENTER_CRITICAL(&dbStatusMux);
    dbStatus = status;
    portEXIT_CRITICAL(&dbStatusMux);
}

// ±10% so retries of separate devices don't stay in lockstep
unsigned long withJitter(unsigned long ms) {
    if (ms < 10) {
        return ms;
    }
    return ms - ms / 10 + esp_random() % (ms / 5 + 1);
}

// ---------------------------------------------------------------------------
// WiFi (optional uplink). Every state has a deadline; nothing here blocks
// for more than WiFi.disconnect()'s 100ms wait.

// Arduino event task. The core retries a failed connection by itself once
// per boot (STA.cpp first_connect: first disconnect with any reason but
// ASSOC_LEAVE, even with auto-reconnect off); mirror that so maintainWiFi()
// knows an attempt may have restarted behind its back.
void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
    uint8_t reason = info.wifi_sta_disconnected.reason;
    portENTER_CRITICAL(&wifiEventMux);
    staDisconnectEvents++;
    lastDisconnectReason = reason;
    if (reason != WIFI_REASON_ASSOC_LEAVE && !coreRetryUsed) {
        coreRetryUsed = true;
        coreRetryPending = true;
    }
    portEXIT_CRITICAL(&wifiEventMux);
}

void maintainWiFi() {
    uint64_t now = monoMs();

    portENTER_CRITICAL(&wifiEventMux);
    uint32_t events = staDisconnectEvents;
    uint8_t reason = lastDisconnectReason;
    bool coreRetry = coreRetryPending;
    coreRetryPending = false;
    portEXIT_CRITICAL(&wifiEventMux);

    // ESP-NOW could not close a session: only a driver restart can
    if (radioRestartRequested) {
        radioRestartRequested = false;
        if (wifiState != WifiState::RADIO_FAULT) {
            restartRadio(now);
            return;
        }
        // RADIO_FAULT restarts the radio when its retry is due anyway
    }

    switch (wifiState) {
        case WifiState::IDLE:
            if (coreRetry) {
                Serial.println("WiFi: the core restarted a connection attempt, cancelling it");
                beginStopping(now, false);
                return;
            }
            if (now < wifiNextAttemptAt) {
                return;
            }
            Serial.println("Connecting to WiFi...");
            WiFi.disconnect();  // clear any stuck association attempt before retrying
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD, routerChannel);
            attemptEventsBaseline = events;
            enterWifiState(WifiState::CONNECTING, now);
            return;

        case WifiState::CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                connectedEventsBaseline = events;
                onWifiConnected(now);
                return;
            }
            if (events != attemptEventsBaseline) {
                attemptEventsBaseline = events;
                if (coreRetry) {
                    // The core's one-time retry is part of this attempt
                } else if (reason != WIFI_REASON_ASSOC_LEAVE) {
                    // The driver reported the failure: the attempt is over,
                    // so return to the listening channel now rather than
                    // after the full timeout
                    Serial.printf("WiFi connection failed (reason %u)\n", reason);
                    beginStopping(now, true);
                    return;
                }
            }
            if (now - wifiStateSince < WIFI_CONNECT_TIMEOUT) {
                return;
            }
            Serial.printf("WiFi connection attempt timed out (last reason %u)\n", reason);
            beginStopping(now, false);
            return;

        case WifiState::CONNECTED:
            // A disconnect event counts as a loss even if the status already
            // looks connected again: a quick re-association behind our back
            // would leave the DB worker with a dead socket and no new generation
            if (WiFi.status() == WL_CONNECTED && events == connectedEventsBaseline) {
                return;
            }
            Serial.printf("WiFi connection lost (reason %u)\n", reason);
            publishWifiStatus(false);
            wifiRetryMs = 0;  // first reconnect right after the channel is restored
            beginStopping(now, false);
            return;

        case WifiState::STOPPING:
            if (coreRetry) {
                beginStopping(now, false);
                return;
            }
            if (events != stopEventsBaseline) {
                // The driver reported the disconnect: the attempt has ended
                if (now - wifiStateSince >= WIFI_STOP_SETTLE_MS) {
                    enterRestoring(now);
                }
                return;
            }
            if (now - wifiStateSince >= WIFI_STOP_TIMEOUT) {
                // No event: the station was most likely already idle. That
                // is not proven yet - RESTORING checks that the channel
                // holds, which an attempt still scanning would not allow.
                Serial.println("WiFi: no disconnect event, verifying the radio is idle");
                enterRestoring(now);
            }
            return;

        case WifiState::RESTORING:
            if (coreRetry) {
                beginStopping(now, false);
                return;
            }
            serviceRestoring(now);
            return;

        case WifiState::RADIO_FAULT:
            // No WiFi attempts and no new ESP-NOW session on an unverified
            // radio; only the next restart can leave this state
            if (now >= radioRetryAt) {
                restartRadio(now);
            }
            return;
    }
}

void onWifiConnected(uint64_t now) {
    uint8_t ch = WiFi.channel();
    Serial.printf("WiFi connected, IP %s, channel %u\n", WiFi.localIP().toString().c_str(), ch);
    if (channelAllowed(ch)) {
        routerChannel = ch;
    } else {
        Serial.printf("WARNING: router channel %u is outside %u-%u, the outdoor station "
                      "can't reach this one while it is connected\n", ch, channelMin, channelMax);
    }
    wifiRetryMs = WIFI_RETRY_MIN;
    publishWifiStatus(true);
    enterWifiState(WifiState::CONNECTED, now);
}

// Cancels the attempt (or what is left of a lost connection). knownIdle:
// the driver already reported the attempt's end, so there is nothing to
// wait for; otherwise STOPPING waits for a disconnect event. Either way
// RESTORING verifies that the radio really is idle.
void beginStopping(uint64_t now, bool knownIdle) {
    portENTER_CRITICAL(&wifiEventMux);
    stopEventsBaseline = staDisconnectEvents;
    portEXIT_CRITICAL(&wifiEventMux);
    if (!WiFi.disconnect()) {
        Serial.println("WARNING: WiFi disconnect request failed");
    }
    if (knownIdle) {
        enterRestoring(now);
    } else {
        enterWifiState(WifiState::STOPPING, now);
    }
}

// Listening channel: the router's (where the outdoor station last found
// us) if both stations support it, otherwise the shared fallback
void enterRestoring(uint64_t now) {
    listenChannel = channelAllowed(routerChannel) ? routerChannel : fallbackChannel();
    channelSet = false;
    enterWifiState(WifiState::RESTORING, now);
}

// The only way to IDLE: the channel was set and still holds after the
// settle time (an attempt still scanning would have moved it)
void serviceRestoring(uint64_t now) {
    if (!channelSet) {
        if (setRadioChannel(listenChannel)) {
            channelSet = true;
            channelSetAt = now;
        }
    } else if (now - channelSetAt >= CHANNEL_SETTLE_MS) {
        if (radioOnChannel(listenChannel)) {
            Serial.printf("Listening for the outdoor station on channel %u\n", listenChannel);
            radioVerified = true;
            scheduleNextWifiAttempt(now);
            enterWifiState(WifiState::IDLE, now);
            return;
        }
        Serial.println("WiFi: channel did not hold, an attempt may still be running");
        WiFi.disconnect();
        channelSet = false;
    }

    if (now - wifiStateSince >= CHANNEL_RESTORE_TIMEOUT) {
        Serial.println("ERROR: could not restore the listening channel");
        restartRadio(now);
    }
}

void enterWifiState(WifiState state, uint64_t now) {
    wifiState = state;
    wifiStateSince = now;
}

void scheduleNextWifiAttempt(uint64_t now) {
    unsigned long delayMs = withJitter(wifiRetryMs);
    wifiNextAttemptAt = now + delayMs;
    Serial.printf("Next WiFi attempt in %lus\n", delayMs / 1000);
    wifiRetryMs = max(WIFI_RETRY_MIN, min(wifiRetryMs * 2, WIFI_RETRY_MAX));
}

// Subsystem recovery, only on evidence of a radio malfunction (a channel
// that can't be restored, an ESP-NOW session that can't be closed) - never
// because the router is simply unavailable. Rate-limited; never a reboot.
// Success leads to RESTORING, which verifies the channel before IDLE; a
// deferred or failed restart waits in RADIO_FAULT.
void restartRadio(uint64_t now) {
    if (radioRestarted && now - lastRadioRestartAt < RADIO_RESTART_MIN_INTERVAL) {
        enterRadioFault(now, lastRadioRestartAt + RADIO_RESTART_MIN_INTERVAL,
                        "restart deferred, the last one was too recent");
        return;
    }
    radioRestarted = true;
    lastRadioRestartAt = now;
    radioRestarts++;
    radioVerified = false;
    Serial.println("Restarting WiFi radio");
    publishWifiStatus(false);

    // ESP-NOW must be closed before the WiFi driver stops; it is reopened by
    // maintainLink() once the radio is verified. If closing fails, the
    // driver deinit below is what ends the session.
    if (linkReady || linkSessionDirty) {
        if (!linkTeardown()) {
            Serial.println("WiFi: ESP-NOW session not closed, relying on the driver restart");
        }
    }
    // WIFI_OFF stops and deinitializes the driver (esp_wifi_stop + deinit)
    if (!WiFi.mode(WIFI_OFF)) {
        enterRadioFault(now, now + RADIO_RESTART_MIN_INTERVAL, "WiFi driver did not stop");
        return;
    }
    delay(100);
    if (!WiFi.mode(WIFI_STA)) {
        enterRadioFault(now, now + RADIO_RESTART_MIN_INTERVAL, "WiFi driver did not start");
        return;
    }
    if (!WiFi.setSleep(false)) {
        enterRadioFault(now, now + RADIO_RESTART_MIN_INTERVAL, "modem sleep not disabled");
        return;
    }
    linkSessionDirty = false;  // the driver restart ended any old session
    linkRetryAt = 0;
    linkRetryMs = LINK_RETRY_MIN;
    Serial.println("WiFi radio restarted, verifying the listening channel");
    enterRestoring(now);
}

// The uplink counts as down in RADIO_FAULT even if the fault was entered
// from CONNECTED (a deferred restart request): the association is no
// longer supervised and the next restart ends it
void enterRadioFault(uint64_t now, uint64_t retryAt, const char* why) {
    publishWifiStatus(false);
    radioVerified = false;
    radioRetryAt = retryAt;
    Serial.printf("ERROR: WiFi radio fault (%s), next restart in %lus\n", why,
                  (unsigned long)((retryAt > now ? retryAt - now : 0) / 1000));
    enterWifiState(WifiState::RADIO_FAULT, now);
}

// Allowed channels: the shared CHANNEL_MIN..MAX range limited to the
// radio's regulatory domain
void readAllowedChannels() {
    wifi_country_t country;
    bool haveCountry = esp_wifi_get_country(&country) == ESP_OK && country.nchan > 0;
    if (haveCountry) {
        channelMin = max(weatherlink::CHANNEL_MIN, country.schan);
        channelMax = min(weatherlink::CHANNEL_MAX, (uint8_t)(country.schan + country.nchan - 1));
    } else {
        Serial.println("WARNING: could not read the regulatory domain, using the configured channels");
    }
    if (channelMin > channelMax) {
        // Use a channel the radio actually supports; the outdoor station
        // only searches CHANNEL_MIN..MAX, so the link may not work at all
        uint8_t ch = haveCountry ? country.schan : weatherlink::DEFAULT_CHANNEL;
        Serial.printf("ERROR: the regulatory domain allows no channel in %u-%u, using %u; "
                      "the outdoor link may not work\n",
                      weatherlink::CHANNEL_MIN, weatherlink::CHANNEL_MAX, ch);
        channelMin = channelMax = ch;
    }
    Serial.printf("Allowed channels: %u-%u\n", channelMin, channelMax);
}

bool channelAllowed(uint8_t ch) {
    return ch >= channelMin && ch <= channelMax;
}

// Where both stations meet without other information: the configured
// default if the regulatory domain allows it, else the lowest allowed one
uint8_t fallbackChannel() {
    return channelAllowed(weatherlink::DEFAULT_CHANNEL) ? weatherlink::DEFAULT_CHANNEL : channelMin;
}

// Sets the radio channel and reads it back; false if either step fails
bool setRadioChannel(uint8_t ch) {
#if FAULT_INJECTION
    if (fiChannelFail) {
        Serial.printf("FI: forced failure setting channel %u\n", ch);
        return false;
    }
#endif
    esp_err_t err = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("ERROR: set channel %u failed: %s\n", ch, esp_err_to_name(err));
        return false;
    }
    if (!radioOnChannel(ch)) {
        Serial.printf("ERROR: channel %u not confirmed\n", ch);
        return false;
    }
    return true;
}

bool radioOnChannel(uint8_t ch) {
    uint8_t primary = 0;
    wifi_second_chan_t second;
    return esp_wifi_get_channel(&primary, &second) == ESP_OK && primary == ch;
}

// ---------------------------------------------------------------------------
// ESP-NOW receiver (outdoor station link)

// A wrong peer address can't be fixed by retrying: leave the link off
bool validateLinkConfig() {
    bool macZero = true;
    for (int i = 0; i < 6; i++) {
        macZero = macZero && OUTDOOR_MAC[i] == 0;
    }
    if (macZero || (OUTDOOR_MAC[0] & 0x01)) {
        Serial.println("ERROR: OUTDOOR_MAC in secrets.h is not a unicast MAC, outdoor link disabled");
        return false;
    }
    if (allZero(ESPNOW_PMK, ESP_NOW_KEY_LEN)) {
        Serial.println("WARNING: ESPNOW_PMK in secrets.h is all zero (placeholder)");
    }
    if (allZero(ESPNOW_LMK, ESP_NOW_KEY_LEN)) {
        Serial.println("WARNING: ESPNOW_LMK in secrets.h is all zero (placeholder)");
    }
    return true;
}

bool allZero(const uint8_t* bytes, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (bytes[i] != 0) {
            return false;
        }
    }
    return true;
}

// Opens the ESP-NOW session; a failure after esp_now_init() is unwound so
// the next attempt starts clean - or, if the unwind fails, marks the
// session dirty so only a radio restart can open the next one
bool linkInit() {
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("ERROR: esp_now_init failed: %s\n", esp_err_to_name(err));
        return false;
    }

    const char* step = "set_pmk";
    err = esp_now_set_pmk(ESPNOW_PMK);
    if (err == ESP_OK) {
        step = "register_recv_cb";
        err = esp_now_register_recv_cb(onEspNowRecv);
    }
    if (err == ESP_OK) {
        step = "add_peer";
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, OUTDOOR_MAC, ESP_NOW_ETH_ALEN);
        memcpy(peer.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
        peer.channel = 0;  // follow the radio's current channel
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = true;
        err = esp_now_add_peer(&peer);
    }
    if (err != ESP_OK) {
        Serial.printf("ERROR: ESP-NOW %s failed: %s\n", step, esp_err_to_name(err));
        if (!closeEspNowSession()) {
            linkSessionDirty = true;
        }
        return false;
    }

    Serial.println("ESP-NOW receiver ready");
    return true;
}

// Unregisters the receive callback, then deinitializes; true only if the
// session is confirmed closed (NOT_INIT counts: nothing was open)
bool closeEspNowSession() {
    esp_err_t err = esp_now_unregister_recv_cb();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.printf("WARNING: unregister_recv_cb failed: %s\n", esp_err_to_name(err));
    }
    err = esp_now_deinit();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.printf("ERROR: esp_now_deinit failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

// Closes the session; the quiesce lets a callback that is already running
// finish (rxQueue is never deleted, so it stays valid either way). A failed
// close leaves the session dirty until a radio restart.
bool linkTeardown() {
    linkReady = false;
    bool closed = closeEspNowSession();
    if (!closed) {
        linkSessionDirty = true;
    }
    delay(LINK_QUIESCE_MS);
    return closed;
}

// Retries opening the session with backoff while it is down. Never on an
// unverified radio, and never while an old session could not be closed:
// that one needs a radio restart first.
void maintainLink(uint64_t now) {
    if (!linkEnabled || linkReady || !radioVerified || now < linkRetryAt) {
        return;
    }
    // New sessions only on a settled channel; during CONNECTING, STOPPING
    // and RESTORING the radio may be scanning. An open session stays open.
    if (wifiState != WifiState::IDLE && wifiState != WifiState::CONNECTED) {
        return;
    }
    if (linkSessionDirty) {
        if (!radioRestartRequested) {
            Serial.println("ESP-NOW: old session not closed, requesting a radio restart");
            radioRestartRequested = true;
        }
        return;
    }
    linkReady = linkInit();
    if (linkReady) {
        linkRetryMs = LINK_RETRY_MIN;
    } else {
        linkRetryAt = now + linkRetryMs;
        linkRetryMs = min(linkRetryMs * 2, LINK_RETRY_MAX);
    }
}

// WiFi task context: envelope check, timestamp, hand over to the loop.
// Acceptance, dedup and all outdoor state belong to the loop.
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    uint64_t receivedAt = monoMs();
    if (memcmp(info->src_addr, OUTDOOR_MAC, ESP_NOW_ETH_ALEN) != 0) {
        rxForeign++;
        return;
    }
    if (!weatherlink::envelopeOk(data, len)) {
        rxBadEnvelope++;
        return;
    }
    RxItem item;
    memcpy(item.bytes, data, sizeof(item.bytes));
    item.receivedAtMs = receivedAt;
    if (xQueueSend(rxQueue, &item, 0) != pdTRUE) {
        rxQueueDrops++;
    }
}

// Processes queued outdoor packets in arrival order. Each accepted packet
// becomes a DB job; only the newest ends up in OutdoorData, so a burst is
// rendered once. Returns true when OutdoorData changed.
bool drainOutdoorRx() {
    if (!rxQueue) {
        return false;
    }
#if FAULT_INJECTION
    if (monoMs() < fiStallRxUntil) {
        return false;
    }
#endif

    bool changed = false;
    RxItem item;
    while (xQueueReceive(rxQueue, &item, 0) == pdTRUE) {
        OutdoorPacket pkt;
        weatherlink::DecodeResult result = weatherlink::decodePacket(item.bytes, sizeof(item.bytes), pkt);
        if (result == weatherlink::DecodeResult::TOO_OLD) {
            rxTooOld++;
            Serial.printf("Outdoor packet rejected: sample %lus old\n",
                          (unsigned long)(pkt.sampleAgeMs / 1000));
            continue;
        }
        if (result != weatherlink::DecodeResult::OK) {
            rxRejected++;
            Serial.println("Outdoor packet rejected: malformed");
            continue;
        }
        if (!acceptSequence(pkt)) {
            continue;
        }

        rxAccepted++;
        applyOutdoorPacket(pkt, item.receivedAtMs);
        changed = true;
        Serial.printf("Outdoor seq %lu: flags %02x, sample age %lums, queued %lums\n",
                      (unsigned long)pkt.seq, pkt.validFlags, (unsigned long)pkt.sampleAgeMs,
                      (unsigned long)(monoMs() - item.receivedAtMs));

        if (pkt.validFlags != 0) {
            enqueueDbJob(DbTable::OUTSIDE, outdoorAsSensorData(), item.receivedAtMs, pkt.sampleAgeMs);
        } else {
            Serial.println("Outdoor heartbeat without valid sensors, skipping insert");
        }
    }
    return changed;
}

// Dedup by (bootId, seq): a new bootId is an outdoor restart, an equal seq
// a duplicate, a lower one out of order; gaps are counted as missed samples
bool acceptSequence(const OutdoorPacket& pkt) {
    if (!outdoorSeen || pkt.bootId != lastOutdoorBootId) {
        if (outdoorSeen) {
            outdoorRestarts++;
            Serial.printf("Outdoor station restarted (boot id %08lx)\n", (unsigned long)pkt.bootId);
        }
        outdoorSeen = true;
        lastOutdoorBootId = pkt.bootId;
        lastOutdoorSeq = pkt.seq;
        return true;
    }
    if (pkt.seq == lastOutdoorSeq) {
        rxDuplicates++;
        Serial.printf("Outdoor seq %lu duplicate, ignored\n", (unsigned long)pkt.seq);
        return false;
    }
    if (pkt.seq < lastOutdoorSeq) {
        rxOutOfOrder++;
        Serial.printf("Outdoor seq %lu out of order (last %lu), ignored\n",
                      (unsigned long)pkt.seq, (unsigned long)lastOutdoorSeq);
        return false;
    }
    if (pkt.seq > lastOutdoorSeq + 1) {
        uint32_t gap = pkt.seq - lastOutdoorSeq - 1;
        rxMissed += gap;
        Serial.printf("Outdoor: %lu sample(s) not received\n", (unsigned long)gap);
    }
    lastOutdoorSeq = pkt.seq;
    return true;
}

void applyOutdoorPacket(const OutdoorPacket& pkt, uint64_t receivedAtMs) {
    bool sht = pkt.validFlags & weatherlink::VALID_SHT31;
    bool bmeOk = pkt.validFlags & weatherlink::VALID_BME280;
    bool pmsOk = pkt.validFlags & weatherlink::VALID_PMS;

    outdoor.temp_sht = sht ? pkt.temp_sht31 : NAN;
    outdoor.humi_sht = sht ? pkt.humi_sht31 : NAN;
    outdoor.temp_bme = bmeOk ? pkt.temp_bme280 : NAN;
    outdoor.humi_bme = bmeOk ? pkt.humi_bme280 : NAN;
    outdoor.pres_bme = bmeOk ? pkt.pres_bme280 : NAN;
    outdoor.pm01 = pmsOk ? pkt.pm01 : 0;
    outdoor.pm25 = pmsOk ? pkt.pm25 : 0;
    outdoor.pm10 = pmsOk ? pkt.pm10 : 0;
    outdoor.validFlags = pkt.validFlags;
    outdoor.sampleAgeMs = pkt.sampleAgeMs;
    outdoor.receivedAtMs = receivedAtMs;
    outdoor.valid = true;
}

SensorData outdoorAsSensorData() {
    SensorData data = {};
    data.temp_sht31 = outdoor.temp_sht;
    data.humi_sht31 = outdoor.humi_sht;
    data.temp_bme280 = outdoor.temp_bme;
    data.humi_bme280 = outdoor.humi_bme;
    data.pres_bme280 = outdoor.pres_bme;
    data.pm01 = outdoor.pm01;
    data.pm25 = outdoor.pm25;
    data.pm10 = outdoor.pm10;
    data.sht31_valid = outdoor.validFlags & weatherlink::VALID_SHT31;
    data.bme280_valid = outdoor.validFlags & weatherlink::VALID_BME280;
    data.pms_valid = outdoor.validFlags & weatherlink::VALID_PMS;
    data.s8_valid = false;  // no CO2 sensor outdoors
    return data;
}

// Measurement age: time since reception plus the age it already had when
// sent - two separate terms, so an outdoor sample older than this boot can't
// underflow. Reception age alone (link freshness) is monoMs() - receivedAtMs.
uint64_t outdoorDataAgeMs() {
    return (monoMs() - outdoor.receivedAtMs) + outdoor.sampleAgeMs;
}

bool outdoorIsStale() {
    return outdoor.valid && outdoorDataAgeMs() > OUTDOOR_STALE_SECONDS * 1000;
}

// ---------------------------------------------------------------------------
// Database (optional uplink). Only the dbWorker task touches the socket.

// Producer side: skip rows the worker would drop anyway. Only a filter -
// the worker re-checks everything before doing any DB work.
void enqueueDbJob(DbTable table, const SensorData& data, uint64_t capturedAtMs, uint32_t priorAgeMs) {
    if (!dbWorkerHandle) {
        return;  // database disabled at boot (logged there)
    }
    WifiStatus wifi = readWifiStatus();
    DbStatus db = readDbStatus();
    DbJob job = {table, data, capturedAtMs, priorAgeMs, wifi.generation};
    const char* reason = jobRejectReason(job, wifi, db.discardBeforeMs, monoMs());
    if (reason) {
        jobsNotQueued++;
        Serial.printf("Not writing %s row: %s\n", tableName(table), reason);
        return;
    }

    if (xQueueSend(dbQueue, &job, 0) != pdTRUE) {
        jobsQueueFull++;
        Serial.printf("Not writing %s row: DB queue full\n", tableName(table));
    }
}

const char* tableName(DbTable table) {
    return table == DbTable::ROOM ? "room_mine" : "outside";
}

uint64_t jobAgeMs(const DbJob& job, uint64_t now) {
    uint64_t sinceCapture = now >= job.capturedAtMs ? now - job.capturedAtMs : 0;
    return sinceCapture + job.priorAgeMs;
}

// Measurement time on this station's clock; a measurement older than this
// boot maps to 0 (underflow-safe, and older than any watermark)
uint64_t jobMeasuredAtMs(const DbJob& job) {
    return job.capturedAtMs > job.priorAgeMs ? job.capturedAtMs - job.priorAgeMs : 0;
}

// Checks shared by the producer (filter), the worker (before any DB work)
// and the pre-submission check (right before the query is sent). The two
// watermarks drop measurements taken during a detected outage, wherever
// they waited (rx queue, DB queue): before the current WiFi connection, or
// before the end of a DB backoff. They are not a claim about when an
// undetected outage physically began.
const char* jobRejectReason(const DbJob& job, const WifiStatus& wifi, uint64_t dbWatermarkMs,
                            uint64_t now) {
    if (!wifi.up) {
        return "no WiFi";
    }
    if (job.wifiGeneration != wifi.generation) {
        return "queued before a WiFi reconnect";
    }
    uint64_t measuredAt = jobMeasuredAtMs(job);
    if (measuredAt < wifi.upSinceMs) {
        return "measured before the WiFi connection";
    }
    if (measuredAt < dbWatermarkMs) {
        return "measured during a database outage";
    }
    if (jobAgeMs(job, now) > DB_JOB_MAX_AGE_MS) {
        return "measurement too old";
    }
    return nullptr;
}

// The only owner of the PostgreSQL connection. It is authoritative for DB
// state: it publishes DbStatus as a whole, and every job is re-validated
// here (WiFi, generation, watermarks, age) before any connection or query
// work. Failed rows are dropped, never retried, so recovery never replays a
// backlog. The DB watermark covers the backoff: anything processed before
// the backoff ends was necessarily measured before it ended.
void dbWorker(void* param) {
    if (wdtConfigured) {
        esp_err_t err = esp_task_wdt_add(NULL);
        workerWdtActive = err == ESP_OK;
        if (!workerWdtActive) {
            Serial.printf("WARNING: task watchdog subscribe failed for dbWorker (%s)\n",
                          esp_err_to_name(err));
        }
    }

    DbStatus status = {DbConn::UNKNOWN, TableStatus::UNKNOWN, TableStatus::UNKNOWN, 0, 0};
    unsigned long backoff = DB_BACKOFF_MIN;
    uint32_t seenGeneration = 0;

    for (;;) {
        feedWorkerWatchdog();
        DbJob job;
        bool haveJob = xQueueReceive(dbQueue, &job, pdMS_TO_TICKS(1000)) == pdTRUE;

        // New WiFi session: the old socket died with the previous
        // association, and the old backoff was most likely about that. The
        // DB watermark is capped at the session start: everything measured
        // before it is rejected by the WiFi watermark anyway.
        WifiStatus wifi = readWifiStatus();
        if (wifi.generation != seenGeneration) {
            seenGeneration = wifi.generation;
            resetDatabaseConnection();  // may block on the dead socket
            backoff = DB_BACKOFF_MIN;
            discardBeforeMs = min(discardBeforeMs, wifi.upSinceMs);
            status.conn = DbConn::UNKNOWN;
            status.retryAtMs = 0;
            status.discardBeforeMs = discardBeforeMs;
            publishDbStatus(status);
            wifi = readWifiStatus();  // the cleanup took time: use a fresh snapshot
        }
        if (!haveJob) {
            continue;
        }

        uint64_t now = monoMs();
        const char* reason = jobRejectReason(job, wifi, discardBeforeMs, now);
        if (reason) {
            dbJobsDropped++;
            Serial.printf("DB: dropped %s row (%s)\n", tableName(job.table), reason);
            continue;
        }

        unsigned long jobStart = millis();
        DbResult result = runQuery(job);
        unsigned long jobMs = millis() - jobStart;
        if (jobMs > maxJobMs.load()) {
            maxJobMs.store(jobMs);  // single writer
        }

        TableStatus& table = job.table == DbTable::ROOM ? status.room : status.outside;
        switch (result) {
            case DbResult::OK:
                dbInserted++;
                status.conn = DbConn::UP;
                table = TableStatus::OK;
                backoff = DB_BACKOFF_MIN;
                Serial.printf("DB: %s row written\n", tableName(job.table));
                break;
            case DbResult::SQL_ERROR:
                // The server answered, so the connection is fine (transport
                // backoff resets); only this table's status changes
                dbSqlErrors++;
                status.conn = DbConn::UP;
                table = TableStatus::SQL_ERR;
                backoff = DB_BACKOFF_MIN;
                Serial.printf("DB: %s row rejected by the server\n", tableName(job.table));
                break;
            case DbResult::CONN_ERROR: {
                dbConnErrors++;
                unsigned long wait = withJitter(backoff);
                status.conn = DbConn::DOWN;
                status.retryAtMs = monoMs() + wait;
                // Everything measured before the backoff ends belongs to
                // this outage; the watermark only ever moves forward here
                discardBeforeMs = max(discardBeforeMs, status.retryAtMs);
                status.discardBeforeMs = discardBeforeMs;
                backoff = min(backoff * 2, DB_BACKOFF_MAX);
                Serial.printf("DB: connection failed, %s row dropped, next attempt in %lus\n",
                              tableName(job.table), wait / 1000);
                break;
            }
            case DbResult::DROPPED:
                dbJobsDropped++;
                break;
        }
        publishDbStatus(status);
    }
}

// The row's dt is its measurement time, computed from the age at submission
bool formatInsert(const DbJob& job, uint64_t now, char* query, size_t size) {
    const SensorData& data = job.data;
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

    unsigned long ageSec = (unsigned long)(jobAgeMs(job, now) / 1000);
    int result;
    if (job.table == DbTable::ROOM) {
        result = snprintf(query, size,
            "INSERT INTO room_mine VALUES (statement_timestamp() - make_interval(secs => %lu), "
            "%s, %s, %s, %s)",
            ageSec, bmeVals, shtVals, co2Val, pmVals);
    } else {
        result = snprintf(query, size,
            "INSERT INTO outside VALUES (statement_timestamp() - make_interval(secs => %lu), "
            "%s, %s, %s)",
            ageSec, bmeVals, shtVals, pmVals);
    }
    return result > 0 && result < (int)size;
}

void setDbState(DatabaseState newState) {
    dbState = newState;
    dbStateSince = millis();
}

// Drives the state machine for one job: <1s when healthy. A connection
// error or a stuck-state reset ends the job instead of reconnecting and
// re-sending it - a failed measurement is dropped, never retried.
DbResult runQuery(const DbJob& job) {
    pendingJob = &job;
    dbTimedOut = false;
    jobDropped = false;

    DbResult result = DbResult::CONN_ERROR;
    bool finished = false;
    unsigned long start = millis();
    while (millis() - start < 2 * DB_STATE_TIMEOUT) {
        feedWorkerWatchdog();
        if (advanceDbStateMachine()) {
            result = queryHadError ? DbResult::SQL_ERROR : DbResult::OK;
            finished = true;
            break;
        }
        if (jobDropped) {
            result = DbResult::DROPPED;
            finished = true;
            break;
        }
        if (dbState == DatabaseState::ERROR || dbTimedOut) {
            finished = true;
            break;
        }
        delay(50);
    }
    if (!finished) {
        Serial.println("ERROR: Database job took too long, resetting connection");
        resetDatabaseConnection();
    }

    pendingJob = nullptr;
    return result;
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
        dbTimedOut = true;
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

// Runs once the connection is up. Re-checks the job right before submission
// (connecting may have taken a while, an outage may have started) and only
// then formats it, so dt reflects the age at submission.
bool dispatchPendingQuery() {
    if (!pendingJob) {
        return false;
    }

    uint64_t now = monoMs();
    const char* reason = jobRejectReason(*pendingJob, readWifiStatus(), discardBeforeMs, now);
    if (reason) {
        Serial.printf("DB: dropped %s row before submission (%s)\n",
                      tableName(pendingJob->table), reason);
        jobDropped = true;
        return false;
    }

    char query[QUERY_BUFFER_SIZE];
    if (!formatInsert(*pendingJob, now, query, sizeof(query))) {
        Serial.println("ERROR: Query buffer overflow");
        jobDropped = true;
        return false;
    }

    Serial.print("Executing query: ");
    Serial.println(query);

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
        setDbState(DatabaseState::CONNECTED);
        return true; // Round-trip complete
    }

    // Continue processing
    return false;
}

void handleDatabaseError(const char* error) {
    Serial.printf("Database error: %s\n", error ? error : "Unknown error");
    // Always land in ERROR: the job ends and the worker backs off; the
    // ERROR state resets to DISCONNECTED when the next job starts
    conn.close();
    setDbState(DatabaseState::ERROR);
}

void resetDatabaseConnection() {
    Serial.println("Resetting database connection");
    conn.close();
    setDbState(DatabaseState::DISCONNECTED);
}

// ---------------------------------------------------------------------------
// Display

// Chooses the cat for the current outdoor conditions.
// Priority: air quality beats precipitation beats temperature.
const unsigned char* pickCat(const OutdoorData& outdoor) {
    if (!outdoor.valid) {
        return NekoSummer;  // nothing known yet
    }

    float temp = isnan(outdoor.temp_sht) ? outdoor.temp_bme : outdoor.temp_sht;
    float humi = isnan(outdoor.humi_sht) ? outdoor.humi_bme : outdoor.humi_sht;

    // A missing PMS reading is unknown air, not clean air: it just doesn't
    // choose the mask
    if ((outdoor.validFlags & weatherlink::VALID_PMS) && outdoor.pm25 > CAT_PM25_POLLUTION) {
        return NekoPollution;
    }
    if (!isnan(temp) && temp < CAT_SNOW_MAX_TEMP) {
        return NekoSnow;
    }
    if (!isnan(humi) && humi >= CAT_RAIN_HUMIDITY) {
        return NekoRain;
    }
    if (isnan(temp)) {
        return NekoSummer;
    }
    if (temp < CAT_COLD_MAX_TEMP) {
        return NekoCold;
    }
    if (temp >= CAT_HOT_MIN_TEMP) {
        return NekoHot;
    }
    return NekoSummer;
}

// Big font-6 number, or "--" when the reading is missing
void drawValue(float value, uint8_t decimals, int32_t x, int32_t y) {
    if (isnan(value)) {
        tft.drawString("--", x, y, 6);
    } else {
        tft.drawFloat(value, decimals, x, y, 6);
    }
}

void drawTft(const SensorData& data, const OutdoorData& outdoor) {
    char tftBuffer[40];

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);

    // Indoor temperature: average when both sensors are healthy.
    // Humidity: SHT31 preferred - the BME280 humidity cell drifts.
    float indoorTemp = NAN;
    if (data.sht31_valid && data.bme280_valid) {
        indoorTemp = (data.temp_sht31 + data.temp_bme280) / 2;
    } else if (data.bme280_valid) {
        indoorTemp = data.temp_bme280;
    } else if (data.sht31_valid) {
        indoorTemp = data.temp_sht31;
    }
    float indoorHumi = data.sht31_valid ? data.humi_sht31
                     : data.bme280_valid ? data.humi_bme280 : NAN;

    tft.drawString("Temp:", 13, 0, 4);
    tft.drawString("Humi:", 143, 0, 4);
    drawValue(indoorTemp, 1, 0, 25);
    drawValue(indoorHumi, 1, 130, 25);

    // Pressure and CO2
    tft.drawString("Press:", 13, 75, 4);
    tft.drawString("CO2:", 143, 75, 4);
    drawValue(data.bme280_valid ? data.pres_bme280 : NAN, 0, 0, 100);
    if (!data.s8_valid) {
        tft.drawString("--", 130, 100, 6);
    } else if (data.co2 < 1000) {
        tft.drawNumber(data.co2, 130, 100, 6);
    } else {
        tft.drawNumber(data.co2, 115, 100, 6);
    }

    // Indoor PM values (font 2 so both PM columns fit side by side)
    if (data.pms_valid) {
        sprintf(tftBuffer, "PM: %d %d %d", data.pm01, data.pm25, data.pm10);
    } else {
        strcpy(tftBuffer, "PM: --");
    }
    tft.drawString(tftBuffer, 1, 155, 2);

    drawOutdoor(outdoor);

    // fillScreen erased the status band: repaint it regardless of the cache
    statusCacheValid = false;
    drawStatus();
}

// Outdoor block and cat; clears only its own regions, so it can be redrawn
// without the indoor half when a packet arrives or the data goes stale
void drawOutdoor(const OutdoorData& outdoor) {
    char tftBuffer[40];

    tft.fillRect(121, 155, 119, 17, TFT_BLACK);  // outdoor PM row (right column)
    tft.fillRect(0, 175, 115, 145, TFT_BLACK);   // outdoor temperature and humidity

    // Grey when nothing was received yet or the data is stale; the age
    // itself is shown in the status band
    bool stale = outdoorIsStale();
    outdoorWasStale = stale;
    tft.setTextColor(!outdoor.valid || stale ? TFT_DARKGREY : TFT_WHITE, TFT_BLACK);

    if (outdoor.valid && (outdoor.validFlags & weatherlink::VALID_PMS)) {
        sprintf(tftBuffer, "PM: %d %d %d", outdoor.pm01, outdoor.pm25, outdoor.pm10);
    } else {
        strcpy(tftBuffer, "PM: --");
    }
    tft.drawString(tftBuffer, 121, 155, 2);

    // NAN fields mean that outdoor sensor is down
    float outdoorTemp = NAN;
    float outdoorHumi = NAN;
    if (outdoor.valid) {
        if (isnan(outdoor.temp_sht)) {
            outdoorTemp = outdoor.temp_bme;
        } else if (isnan(outdoor.temp_bme)) {
            outdoorTemp = outdoor.temp_sht;
        } else {
            outdoorTemp = (outdoor.temp_sht + outdoor.temp_bme) / 2;
        }
        outdoorHumi = isnan(outdoor.humi_sht) ? outdoor.humi_bme : outdoor.humi_sht;
    }

    tft.drawString("Temp:", 13, 180, 4);
    drawValue(outdoorTemp, 1, 0, 205);
    tft.drawString("Humi:", 13, 258, 4);
    drawValue(outdoorHumi, 1, 0, 283);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // Weather cat: outfit follows the outdoor conditions
    tft.drawXBitmap(115, 190, pickCat(outdoor), 128, 128, TFT_BLACK, TFT_WHITE);
}

// Outdoor part of the status band; empty while everything is fine
void outdoorStatusText(char* buf, size_t size) {
    if (!outdoor.valid) {
        snprintf(buf, size, "no link");
        return;
    }
    unsigned long ageSec = (unsigned long)(outdoorDataAgeMs() / 1000);
    if (ageSec > OUTDOOR_STALE_SECONDS) {
        if (ageSec < 3600) {
            snprintf(buf, size, "%lum old", ageSec / 60);
        } else if (ageSec < 86400) {
            snprintf(buf, size, "%luh old", ageSec / 3600);
        } else {
            snprintf(buf, size, ">1d old");
        }
        return;
    }
    if (outdoor.validFlags == 0) {
        snprintf(buf, size, "no sens");  // station alive, every sensor down
        return;
    }
    buf[0] = '\0';
}

// Uplink part of the status band; empty while everything is fine
const char* uplinkStatusText() {
    if (!readWifiStatus().up) {
        return "no WiFi";
    }
    DbStatus db = readDbStatus();
    if (db.conn == DbConn::DOWN || db.conn == DbConn::NO_WORKER) {
        return "no DB";
    }
    if (db.room == TableStatus::SQL_ERR || db.outside == TableStatus::SQL_ERR) {
        return "DB err";
    }
    if (db.conn == DbConn::UNKNOWN) {
        return "DB...";  // WiFi is up, no DB result in this session yet
    }
    return "";
}

// Repaints the status band only when its text changes (or after a full
// redraw invalidated the cache)
void drawStatus() {
    char linkText[sizeof(statusCacheLink)];
    outdoorStatusText(linkText, sizeof(linkText));
    const char* uplinkText = uplinkStatusText();

    if (statusCacheValid && strcmp(linkText, statusCacheLink) == 0 &&
        uplinkText == statusCacheUplink) {
        return;
    }
    strcpy(statusCacheLink, linkText);
    statusCacheUplink = uplinkText;
    statusCacheValid = true;

    tft.fillRect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, TFT_BLACK);
    int32_t x = STATUS_X;
    if (linkText[0]) {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        x += tft.drawString(linkText, x, STATUS_Y, 2) + STATUS_GAP;
    }
    if (uplinkText[0]) {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString(uplinkText, x, STATUS_Y, 2);
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
}

// Logs any status combination that would not fit the band
void checkStatusFit() {
    const char* links[] = {"no link", "no sens", "59m old", "23h old", ">1d old"};
    const char* uplinks[] = {"no WiFi", "no DB", "DB err", "DB..."};
    for (const char* link : links) {
        for (const char* uplink : uplinks) {
            int width = tft.textWidth(link, 2) + STATUS_GAP + tft.textWidth(uplink, 2);
            if (width > STATUS_W) {
                Serial.printf("WARNING: status \"%s %s\" is %dpx, the band is %dpx\n",
                              link, uplink, width, STATUS_W);
            }
        }
    }
}

void logStats() {
    WifiStatus wifi = readWifiStatus();
    DbStatus db = readDbStatus();
    Serial.printf("STATS uptime %lus, wifi %s, radio restarts %lu, link %s, max loop pass %lums\n",
                  (unsigned long)(monoMs() / 1000), wifi.up ? "up" : "down",
                  (unsigned long)radioRestarts, linkReady ? "ready" : "down", maxLoopPassMs);
    Serial.printf("STATS outdoor: accepted %lu, not received %lu, duplicates %lu, out of order %lu, "
                  "restarts %lu, too old %lu, rejected %lu, rx queue drops %lu, foreign %lu, "
                  "bad envelope %lu, last reception %lus ago\n",
                  (unsigned long)rxAccepted, (unsigned long)rxMissed, (unsigned long)rxDuplicates,
                  (unsigned long)rxOutOfOrder, (unsigned long)outdoorRestarts,
                  (unsigned long)rxTooOld, (unsigned long)rxRejected,
                  (unsigned long)rxQueueDrops.load(), (unsigned long)rxForeign.load(),
                  (unsigned long)rxBadEnvelope.load(),
                  outdoor.valid ? (unsigned long)((monoMs() - outdoor.receivedAtMs) / 1000) : 0UL);
    Serial.printf("STATS db: conn %u, room %u, outside %u, inserted %lu, sql errors %lu, "
                  "conn errors %lu, dropped %lu, not queued %lu, queue full %lu, max job %lums\n",
                  (unsigned)db.conn, (unsigned)db.room, (unsigned)db.outside,
                  (unsigned long)dbInserted.load(), (unsigned long)dbSqlErrors.load(),
                  (unsigned long)dbConnErrors.load(), (unsigned long)dbJobsDropped.load(),
                  (unsigned long)jobsNotQueued, (unsigned long)jobsQueueFull, maxJobMs.load());
    Serial.printf("STATS free stack: loop %u, dbWorker %u bytes; heap free %lu, min %lu\n",
                  (unsigned)uxTaskGetStackHighWaterMark(NULL),
                  dbWorkerHandle ? (unsigned)uxTaskGetStackHighWaterMark(dbWorkerHandle) : 0U,
                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
}

#if FAULT_INJECTION
// r: stall outdoor packet processing for 3 minutes. The rx queue keeps the
//    first 4 packets of the stall, the rest are counted as rx queue drops.
//    At the end the 4 are processed at once: those older than 90s are
//    dropped as too old; depending on phase the youngest may still be
//    written, with its correct dt. If a DB outage was detected meanwhile,
//    the DB watermark drops all of them.
// f: toggle failure of every channel change (persistent channel-restore
//    failure: RESTORING fails, a radio restart, then RADIO_FAULT with a
//    restart at most once a minute; never IDLE, no reboot). Toggle again to
//    recover: the next restart verifies the channel and reaches IDLE.
void handleFaultInjection() {
    while (Serial.available()) {
        switch (Serial.read()) {
            case 'r':
                fiStallRxUntil = monoMs() + 180000;
                Serial.println("FI: outdoor packet processing stalled for 3 minutes");
                break;
            case 'f':
                fiChannelFail = !fiChannelFail;
                Serial.printf("FI: channel changes %s\n", fiChannelFail ? "fail" : "work");
                break;
        }
    }
}
#endif
