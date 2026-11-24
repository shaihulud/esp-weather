#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_random.h>
#include <esp_task_wdt.h>
#include <atomic>
#include <Adafruit_BME280.h>
#include "Adafruit_SHT31.h"
#include <PMserial.h>
#include <WeatherLink.h>  // libraries/WeatherLink, symlinked into the sketchbook

#include "secrets.h"

#ifndef SECRETS_STATION_OUTDOOR
#error "Wrong secrets.h: this is the outdoor station, use its own secrets file"
#endif

using weatherlink::OutdoorPacket;
using weatherlink::monoMs;

// Test hooks driven by single-letter serial commands (see
// handleFaultInjection); must stay 0 in production builds
#define FAULT_INJECTION 0

// Hardware configuration
const uint8_t PMS_RX_PIN = 34;
const uint8_t PMS_TX_PIN = 33;
const uint8_t SHT31_ADDR = 0x44;
const uint8_t BME280_ADDR = 0x76;

// Per-board BME280 calibration, measured 2026-07-16 side by side with the
// SHT31 (humidity) and the indoor station (pressure); re-tune if the sensor
// is replaced. Pressure offset counters the t_fine shift introduced by
// setTemperatureCompensation(), which also skews the compensated pressure.
const float BME_TEMP_COMPENSATION = -4.3;  // die reads high vs SHT31
const float BME_HUMIDITY_OFFSET = 16.5;
const float BME_PRESSURE_OFFSET = 3.0;  // mmHg

// Timing constants
const unsigned long NORMAL_DELAY = weatherlink::OUTDOOR_SAMPLE_INTERVAL_MS;  // sample start-to-start
const unsigned long SENSOR_INIT_DELAY = 1000;
const unsigned long LOOP_TICK_MS = 10;
const unsigned long SEND_CB_TIMEOUT_MS = 1000;    // the send callback normally arrives within milliseconds
const unsigned long RETRY_GAP_MS = 20;            // between attempts of one channel sweep
const unsigned long NO_MEM_RETRY_MS = 50;
const uint8_t NO_MEM_MAX_RETRIES = 5;             // then fall back to the link backoff
const unsigned long LINK_BACKOFF_MIN_MS = 2000;
const unsigned long LINK_BACKOFF_MAX_MS = 30000;
const unsigned long LINK_QUIESCE_MS = 500;        // between ESP-NOW teardown and re-init
const uint8_t LINK_FAILURES_BEFORE_RADIO_RESTART = 3;
const unsigned long STATS_LOG_INTERVAL = 600000;
const uint32_t TASK_WDT_TIMEOUT_MS = 60000;       // task-health only, never availability
const uint8_t MAX_INIT_ATTEMPTS = 3;
const uint8_t SENSOR_REINIT_THRESHOLD = 5;  // consecutive bad cycles before re-init

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

// ESP-NOW link to the indoor station. One transmission at a time: a new one
// starts only after the previous callback was processed or the session was
// torn down.
enum class LinkState {
    DOWN,       // no ESP-NOW session; linkInit() retried on the backoff
    IDLE,       // session up, nothing in flight
    IN_FLIGHT   // waiting for the send callback of inFlightSeq
};

// Global objects
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();
SerialPM pms(PMSA003, PMS_RX_PIN, PMS_TX_PIN);

// Consecutive invalid-reading counters, trigger sensor re-init at threshold
uint8_t sht31Failures = 0;
uint8_t bme280Failures = 0;
uint8_t pmsFailures = 0;

bool wdtActive = false;
uint32_t bootId = 0;
uint64_t nextSampleAt = 0;

// Link state, owned by loop()
LinkState linkState = LinkState::DOWN;
bool linkConfigOk = false;
OutdoorPacket pending;            // newest sample not yet radio-acked
bool hasPending = false;
uint64_t pendingSampledAtMs = 0;
uint32_t lastSeq = 0;
uint32_t inFlightSeq = 0;
uint8_t inFlightChannel = 0;
uint64_t inFlightDeadline = 0;
uint8_t channels[weatherlink::CHANNEL_MAX];  // allowed channels, ascending
uint8_t channelCount = 0;
uint8_t goodChannel = weatherlink::DEFAULT_CHANNEL;
uint8_t sweepIndex = 0;
uint64_t nextAttemptAt = 0;
unsigned long backoffMs = LINK_BACKOFF_MIN_MS;
uint8_t noMemRetries = 0;
uint8_t linkFailures = 0;         // consecutive init/recovery failures
bool sweepRadioOk = false;        // some channel of the current sweep accepted a send
bool needRadioRestart = false;    // session not closable or radio unusable: no linkInit() until a restart succeeds

// Written by the send callback (WiFi task), read by loop() under linkMux
portMUX_TYPE linkMux = portMUX_INITIALIZER_UNLOCKED;
bool cbDone = false;
esp_now_send_status_t cbStatus = ESP_NOW_SEND_FAIL;

// Counters for the periodic stats line
uint32_t radioAcks = 0;
uint32_t sendFailures = 0;
uint32_t samplesSuperseded = 0;
uint32_t samplesExpired = 0;
uint32_t linkRecoveries = 0;
uint32_t radioRestarts = 0;

#if FAULT_INJECTION
std::atomic<bool> fiDropCallback{false};  // swallow the next send callback (WiFi task)
bool fiHoldAck = false;        // take a new sample before processing the next SUCCESS callback
bool fiChannelFail = false;    // make every channel change fail
OutdoorPacket fiInFlight;      // copy of the packet in flight
OutdoorPacket fiSent[2];       // last two radio-acked packets, newest first
uint8_t fiSentCount = 0;
int8_t fiResend = -1;          // index into fiSent to send once instead of pending
#endif

// Function declarations
bool initTaskWatchdog();
void feedWatchdog();
bool initializeSensors();
bool initSensorWithRetry(const char* name, bool (*beginSensor)());
bool initializeSHT31();
bool initializeBME280();
bool initializePMS();
SensorData readSensorData();
bool validateSensorData(SensorData& data);
void handleSensorHealth(const SensorData& data);
void logSensorData(const SensorData& data);
void takeSample();
void buildPacket(const SensorData& data, OutdoorPacket& pkt);
void buildChannelList();
bool setRadioChannel(uint8_t ch);
bool allZero(const uint8_t* bytes, size_t len);
bool validateLinkConfig();
bool linkInit();
bool closeEspNowSession();
bool linkTeardown();
void linkRecover(uint64_t now, bool forceRadioRestart);
bool restartRadio();
void serviceLink();
void startAttempt(uint64_t now);
void finishAttempt(bool acked, uint64_t now);
void advanceSweep(uint64_t now);
uint8_t sweepLength();
uint8_t sweepChannel(uint8_t index);
void scheduleBackoff(uint64_t now);
void onEspNowSent(const esp_now_send_info_t* info, esp_now_send_status_t status);
void logStats();
#if FAULT_INJECTION
void handleFaultInjection();
#endif

void setup() {
    Serial.begin(9600);
    Serial.println("\nESP32 Weather Station (outdoor) Starting...");

    wdtActive = initTaskWatchdog();

    // Radio only: this station never joins the router, it just transmits
    // to the indoor station over ESP-NOW. persistent(false) avoids
    // rewriting WiFi config to NVS flash.
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    Serial.printf("MAC address: %s\n", WiFi.macAddress().c_str());

    buildChannelList();
    // Cold boot: always start from the configured default channel. If the
    // radio can't even do that, no session is opened before a radio restart.
    if (!setRadioChannel(goodChannel)) {
        Serial.println("WARNING: could not set the default channel");
        needRadioRestart = true;
    }

    bootId = esp_random();  // hardware RNG, seeded by the running radio
    Serial.printf("Boot id: %08lx\n", (unsigned long)bootId);

    if (!initializeSensors()) {
        Serial.println("WARNING: Some sensors failed to initialize");
    }

    linkConfigOk = validateLinkConfig();
    if (linkConfigOk) {
        if (!needRadioRestart && linkInit()) {
            linkState = LinkState::IDLE;
        } else {
            linkFailures++;
            scheduleBackoff(monoMs());
        }
    }

    nextSampleAt = monoMs();
    Serial.println("Setup complete");
}

void loop() {
    feedWatchdog();

#if FAULT_INJECTION
    handleFaultInjection();
#endif

    // Fixed start-to-start sampling, independent of the link; after a long
    // stall the schedule moves forward instead of bursting to catch up
    uint64_t now = monoMs();
    if (now >= nextSampleAt) {
        nextSampleAt += NORMAL_DELAY;
        if (nextSampleAt <= now) {
            nextSampleAt = now + NORMAL_DELAY;
        }
        takeSample();
    }

    serviceLink();

    static uint64_t lastStatsAt = 0;
    if (monoMs() - lastStatsAt >= STATS_LOG_INTERVAL) {
        lastStatsAt = monoMs();
        logStats();
    }

    delay(LOOP_TICK_MS);
}

// Reconfigures the task watchdog (default 5s) to TASK_WDT_TIMEOUT_MS and
// subscribes the loop task. Any failure leaves the task unsubscribed, so it
// never ends up on the 5s default.
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
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        return true;  // already subscribed
    }
    err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        Serial.printf("WARNING: task watchdog subscribe failed (%s), running without it\n",
                      esp_err_to_name(err));
        return false;
    }
    return true;
}

void feedWatchdog() {
    if (wdtActive) {
        esp_task_wdt_reset();
    }
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

// Reads the sensors and makes the result the pending packet. Runs even when
// every sensor is invalid: the packet then serves as a heartbeat, so the
// indoor station can tell sensor failure from loss of communication.
void takeSample() {
    uint64_t sampledAt = monoMs();
    SensorData data = readSensorData();
    validateSensorData(data);
    handleSensorHealth(data);
    logSensorData(data);

    if (hasPending) {
        samplesSuperseded++;
        Serial.printf("Sample seq %lu superseded before it was radio-acked\n",
                      (unsigned long)pending.seq);
    }
    buildPacket(data, pending);
    pending.seq = ++lastSeq;
    pendingSampledAtMs = sampledAt;
    hasPending = true;
}

// Values are sent already calibrated; invalid groups go as NAN/0 with
// their flag cleared
void buildPacket(const SensorData& data, OutdoorPacket& pkt) {
    pkt = {};
    pkt.magic = weatherlink::MAGIC;
    pkt.version = weatherlink::VERSION;
    pkt.bootId = bootId;

    if (data.sht31_valid) {
        pkt.validFlags |= weatherlink::VALID_SHT31;
        pkt.temp_sht31 = data.temp_sht31;
        pkt.humi_sht31 = data.humi_sht31;
    } else {
        pkt.temp_sht31 = NAN;
        pkt.humi_sht31 = NAN;
    }
    if (data.bme280_valid) {
        pkt.validFlags |= weatherlink::VALID_BME280;
        pkt.temp_bme280 = data.temp_bme280;
        pkt.humi_bme280 = data.humi_bme280;
        pkt.pres_bme280 = data.pres_bme280;
    } else {
        pkt.temp_bme280 = NAN;
        pkt.humi_bme280 = NAN;
        pkt.pres_bme280 = NAN;
    }
    if (data.pms_valid) {
        pkt.validFlags |= weatherlink::VALID_PMS;
        pkt.pm01 = data.pm01;
        pkt.pm25 = data.pm25;
        pkt.pm10 = data.pm10;
    }
}

// Allowed channels: the shared CHANNEL_MIN..MAX range limited to the
// radio's regulatory domain
void buildChannelList() {
    uint8_t lo = weatherlink::CHANNEL_MIN;
    uint8_t hi = weatherlink::CHANNEL_MAX;
    wifi_country_t country;
    if (esp_wifi_get_country(&country) == ESP_OK && country.nchan > 0) {
        lo = max(lo, country.schan);
        hi = min(hi, (uint8_t)(country.schan + country.nchan - 1));
    } else {
        Serial.println("WARNING: could not read the regulatory domain, using the configured channels");
    }

    channelCount = 0;
    for (uint8_t ch = lo; ch <= hi; ch++) {
        channels[channelCount++] = ch;
    }
    if (channelCount == 0) {
        Serial.println("ERROR: no allowed channel, falling back to the default channel");
        channels[channelCount++] = weatherlink::DEFAULT_CHANNEL;
    }

    goodChannel = channels[0];
    for (uint8_t i = 0; i < channelCount; i++) {
        if (channels[i] == weatherlink::DEFAULT_CHANNEL) {
            goodChannel = weatherlink::DEFAULT_CHANNEL;
        }
    }
    if (goodChannel != weatherlink::DEFAULT_CHANNEL) {
        Serial.printf("WARNING: default channel %u not allowed, starting on %u\n",
                      weatherlink::DEFAULT_CHANNEL, goodChannel);
    }
    Serial.printf("Allowed channels: %u-%u\n", channels[0], channels[channelCount - 1]);
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
    uint8_t primary = 0;
    wifi_second_chan_t second;
    err = esp_wifi_get_channel(&primary, &second);
    if (err != ESP_OK || primary != ch) {
        Serial.printf("ERROR: channel %u not confirmed (%s, now %u)\n",
                      ch, esp_err_to_name(err), primary);
        return false;
    }
    return true;
}

// A wrong peer address can't be fixed by retrying: leave the link off
bool validateLinkConfig() {
    bool macZero = true;
    for (int i = 0; i < 6; i++) {
        macZero = macZero && INDOOR_MAC[i] == 0;
    }
    if (macZero || (INDOOR_MAC[0] & 0x01)) {
        Serial.println("ERROR: INDOOR_MAC in secrets.h is not a unicast MAC, link disabled");
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

// Opens the ESP-NOW session. Callback state is reset before the callback is
// registered; a failure after esp_now_init() is unwound so the next attempt
// starts clean - or, if the unwind fails, a radio restart is required first.
bool linkInit() {
    portENTER_CRITICAL(&linkMux);
    cbDone = false;
    cbStatus = ESP_NOW_SEND_FAIL;
    portEXIT_CRITICAL(&linkMux);

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("ERROR: esp_now_init failed: %s\n", esp_err_to_name(err));
        return false;
    }

    const char* step = "set_pmk";
    err = esp_now_set_pmk(ESPNOW_PMK);
    if (err == ESP_OK) {
        step = "register_send_cb";
        err = esp_now_register_send_cb(onEspNowSent);
    }
    if (err == ESP_OK) {
        step = "add_peer";
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, INDOOR_MAC, ESP_NOW_ETH_ALEN);
        memcpy(peer.lmk, ESPNOW_LMK, ESP_NOW_KEY_LEN);
        peer.channel = 0;  // follow the radio's current channel
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = true;
        err = esp_now_add_peer(&peer);
    }
    if (err != ESP_OK) {
        Serial.printf("ERROR: ESP-NOW %s failed: %s\n", step, esp_err_to_name(err));
        if (!closeEspNowSession()) {
            needRadioRestart = true;
        }
        return false;
    }

    Serial.println("ESP-NOW link ready");
    return true;
}

// Unregisters the send callback, then deinitializes; true only if the
// session is confirmed closed (NOT_INIT counts: nothing was open)
bool closeEspNowSession() {
    esp_err_t err = esp_now_unregister_send_cb();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.printf("WARNING: unregister_send_cb failed: %s\n", esp_err_to_name(err));
    }
    err = esp_now_deinit();
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_NOT_INIT) {
        Serial.printf("ERROR: esp_now_deinit failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

// Closes the ESP-NOW session; a close that can't be confirmed requires a
// radio restart before the next session
bool linkTeardown() {
    linkState = LinkState::DOWN;
    bool closed = closeEspNowSession();
    if (!closed) {
        needRadioRestart = true;
    }
    return closed;
}

// Recovery from a broken or unresolved session (missing callback, session
// errors) or an unusable radio (forceRadioRestart: no channel of a whole
// sweep could be used). The callback is unregistered and the session
// closed, then a quiesce period far longer than any MAC retry sequence
// passes before a new session starts. ESP-NOW is closed source, so a late
// callback of the old session can't be ruled out completely; at worst it
// would be credited to the next transmission (an occasional lost sample,
// accepted). A failed teardown, repeated failures or a forced restart
// escalate to a radio restart; while that hasn't succeeded no new session
// is opened. The link stays DOWN, sending nothing, until an init succeeds.
void linkRecover(uint64_t now, bool forceRadioRestart) {
    linkRecoveries++;
    Serial.println("ESP-NOW: recovering link");
    bool closed = linkTeardown();
    if (closed && ++linkFailures >= LINK_FAILURES_BEFORE_RADIO_RESTART) {
        needRadioRestart = true;
    }
    if (forceRadioRestart) {
        needRadioRestart = true;
    }
    if (needRadioRestart && restartRadio()) {
        linkFailures = 0;
    }
    delay(LINK_QUIESCE_MS);

    if (needRadioRestart) {
        Serial.println("ESP-NOW: radio not recovered, retrying the restart later");
    } else if (linkInit()) {
        linkState = LinkState::IDLE;
    } else {
        linkFailures++;
    }
    sweepIndex = 0;
    sweepRadioOk = false;
    scheduleBackoff(now);
}

// Restarts the WiFi driver (WIFI_OFF stops and deinitializes it, which
// also ends an ESP-NOW session that could not be closed). Every step is
// checked; only a full success clears needRadioRestart.
bool restartRadio() {
    radioRestarts++;
    Serial.println("Restarting WiFi radio");
    bool ok = WiFi.mode(WIFI_OFF);
    if (!ok) {
        Serial.println("ERROR: WiFi driver did not stop");
    } else {
        delay(100);
        ok = WiFi.mode(WIFI_STA);
        if (!ok) {
            Serial.println("ERROR: WiFi driver did not start");
        }
    }
    if (ok) {
        ok = setRadioChannel(goodChannel);
        if (!ok) {
            Serial.println("ERROR: radio unusable after the restart (channel not set)");
        }
    }
    needRadioRestart = !ok;
    return ok;
}

void serviceLink() {
    if (!linkConfigOk) {
        return;
    }
    uint64_t now = monoMs();

    switch (linkState) {
        case LinkState::DOWN:
            if (now < nextAttemptAt) {
                return;
            }
            if (linkFailures >= LINK_FAILURES_BEFORE_RADIO_RESTART) {
                needRadioRestart = true;
            }
            if (needRadioRestart) {
                if (!restartRadio()) {
                    scheduleBackoff(now);  // never a session on an unrecovered radio
                    return;
                }
                linkFailures = 0;
            }
            if (linkInit()) {
                linkState = LinkState::IDLE;
                linkFailures = 0;
            } else {
                linkFailures++;
                scheduleBackoff(now);
            }
            return;

        case LinkState::IDLE:
            if (now < nextAttemptAt) {
                return;
            }
            // The indoor station rejects samples past this age: don't send them
            if (hasPending && now - pendingSampledAtMs > weatherlink::MAX_SAMPLE_AGE_MS) {
                samplesExpired++;
                hasPending = false;
                Serial.printf("Sample seq %lu expired unsent (%lus old)\n", (unsigned long)pending.seq,
                              (unsigned long)((now - pendingSampledAtMs) / 1000));
            }
#if FAULT_INJECTION
            if (!hasPending && fiResend < 0) {
                return;
            }
#else
            if (!hasPending) {
                return;
            }
#endif
            startAttempt(now);
            return;

        case LinkState::IN_FLIGHT: {
            bool done;
            esp_now_send_status_t status;
            portENTER_CRITICAL(&linkMux);
            done = cbDone;
            status = cbStatus;
            portEXIT_CRITICAL(&linkMux);

            if (done) {
#if FAULT_INJECTION
                // Only a SUCCESS exercises "ACK for an old seq with a newer
                // pending sample"; a FAIL keeps the hook armed
                if (fiHoldAck && status == ESP_NOW_SEND_SUCCESS) {
                    fiHoldAck = false;
                    Serial.printf("FI: taking a new sample before processing the ACK of seq %lu\n",
                                  (unsigned long)inFlightSeq);
                    takeSample();
                } else if (fiHoldAck) {
                    Serial.println("FI: send callback was FAIL, hold-ack stays armed");
                }
#endif
                finishAttempt(status == ESP_NOW_SEND_SUCCESS, now);
            } else if (now >= inFlightDeadline) {
                Serial.printf("ESP-NOW: no send callback for seq %lu within %lums\n",
                              (unsigned long)inFlightSeq, SEND_CB_TIMEOUT_MS);
                linkRecover(now, false);
            }
            return;
        }
    }
}

void startAttempt(uint64_t now) {
    if (sweepIndex == 0) {
        sweepRadioOk = false;  // a new sweep starts
    }
    uint8_t ch = sweepChannel(sweepIndex);
    if (!setRadioChannel(ch)) {
        advanceSweep(now);
        return;
    }

    // esp_now_send copies the data, so a stack buffer is fine
    OutdoorPacket buf = pending;
    buf.sampleAgeMs = (uint32_t)min<uint64_t>(now - pendingSampledAtMs, UINT32_MAX);
#if FAULT_INJECTION
    if (fiResend >= 0) {
        buf = fiSent[fiResend];
        Serial.printf("FI: resending old seq %lu\n", (unsigned long)buf.seq);
        fiResend = -1;
    }
#endif

    portENTER_CRITICAL(&linkMux);
    cbDone = false;
    portEXIT_CRITICAL(&linkMux);

    esp_err_t err = esp_now_send(INDOOR_MAC, (const uint8_t*)&buf, sizeof(buf));
    switch (err) {
        case ESP_OK:
            // The radio is usable on this channel; whether the peer answers
            // is a separate question (FAIL callbacks never restart the radio)
            sweepRadioOk = true;
            noMemRetries = 0;
            inFlightSeq = buf.seq;
            inFlightChannel = ch;
            inFlightDeadline = now + SEND_CB_TIMEOUT_MS;
            linkState = LinkState::IN_FLIGHT;
#if FAULT_INJECTION
            fiInFlight = buf;
#endif
            return;

        case ESP_ERR_ESPNOW_NO_MEM:
            // Transient per the API docs: retry shortly, then back off
            if (++noMemRetries > NO_MEM_MAX_RETRIES) {
                noMemRetries = 0;
                Serial.println("ESP-NOW: out of memory repeatedly, backing off");
                scheduleBackoff(now);
            } else {
                nextAttemptAt = now + NO_MEM_RETRY_MS;
            }
            return;

        case ESP_ERR_ESPNOW_CHAN:
            // Radio is not on the channel just set: try the next one
            Serial.printf("ESP-NOW: channel mismatch on %u\n", ch);
            advanceSweep(now);
            return;

        default:
            // NOT_INIT, NOT_FOUND, IF (interface mismatch), ARG, INTERNAL:
            // the session or its configuration is broken, sweeping can't fix it
            Serial.printf("ESP-NOW: send failed: %s\n", esp_err_to_name(err));
            linkRecover(now, false);
            return;
    }
}

void finishAttempt(bool acked, uint64_t now) {
    linkState = LinkState::IDLE;

    if (!acked) {
        sendFailures++;
        advanceSweep(now);
        return;
    }

    // A MAC-layer ACK: the indoor radio got the frame. Whether the station
    // accepted it is only known there.
    radioAcks++;
    if (inFlightChannel != goodChannel) {
        Serial.printf("ESP-NOW: indoor station found on channel %u\n", inFlightChannel);
    }
    Serial.printf("Radio-acked seq %lu on channel %u\n", (unsigned long)inFlightSeq, inFlightChannel);
    goodChannel = inFlightChannel;
    sweepIndex = 0;
    backoffMs = LINK_BACKOFF_MIN_MS;
    linkFailures = 0;
#if FAULT_INJECTION
    // Remember radio-acked packets for the d/o resend tests (a MAC ACK, so
    // still only "very likely received": check the indoor log)
    if (fiSentCount == 0 || fiInFlight.seq > fiSent[0].seq) {
        fiSent[1] = fiSent[0];
        fiSent[0] = fiInFlight;
        fiSentCount = min(fiSentCount + 1, 2);
    }
#endif

    // A newer sample may have replaced pending while this one was in flight;
    // only the acknowledged measurement is cleared
    if (hasPending && pending.seq == inFlightSeq) {
        hasPending = false;
    }
    nextAttemptAt = now;
}

// Next attempt of the sweep (goodChannel twice, then every other allowed
// channel); after a full failed sweep go back to goodChannel and back off.
// A sweep in which no channel could be used at all (every channel change
// or send rejected locally) is a local radio failure, not an absent peer:
// that escalates to a radio restart.
void advanceSweep(uint64_t now) {
    sweepIndex++;
    if (sweepIndex < sweepLength()) {
        nextAttemptAt = now + RETRY_GAP_MS;
        return;
    }
    sweepIndex = 0;
    if (!sweepRadioOk) {
        Serial.println("ESP-NOW: no channel of the sweep was usable, recovering the radio");
        linkRecover(now, true);
        return;
    }
    Serial.printf("ESP-NOW: indoor station not reachable on any channel, retry in %lus\n",
                  backoffMs / 1000);
    setRadioChannel(goodChannel);
    scheduleBackoff(now);
}

uint8_t sweepLength() {
    bool goodInList = false;
    for (uint8_t i = 0; i < channelCount; i++) {
        goodInList = goodInList || channels[i] == goodChannel;
    }
    return 2 + channelCount - (goodInList ? 1 : 0);
}

uint8_t sweepChannel(uint8_t index) {
    if (index < 2) {
        return goodChannel;
    }
    uint8_t n = index - 2;
    for (uint8_t i = 0; i < channelCount; i++) {
        if (channels[i] == goodChannel) {
            continue;
        }
        if (n-- == 0) {
            return channels[i];
        }
    }
    return goodChannel;
}

void scheduleBackoff(uint64_t now) {
    nextAttemptAt = now + backoffMs;
    backoffMs = min(backoffMs * 2, LINK_BACKOFF_MAX_MS);
}

// WiFi task context: record the result only, the loop does the rest
void onEspNowSent(const esp_now_send_info_t* info, esp_now_send_status_t status) {
#if FAULT_INJECTION
    if (fiDropCallback.exchange(false)) {
        return;
    }
#endif
    portENTER_CRITICAL(&linkMux);
    cbStatus = status;
    cbDone = true;
    portEXIT_CRITICAL(&linkMux);
}

void logStats() {
    Serial.printf("STATS uptime %lus: radio-acks %lu, send failures %lu, superseded %lu, expired %lu, "
                  "link recoveries %lu, radio restarts %lu, channel %u, free heap %lu (min %lu)\n",
                  (unsigned long)(monoMs() / 1000), (unsigned long)radioAcks,
                  (unsigned long)sendFailures, (unsigned long)samplesSuperseded,
                  (unsigned long)samplesExpired,
                  (unsigned long)linkRecoveries, (unsigned long)radioRestarts, goodChannel,
                  (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap());
}

#if FAULT_INJECTION
// c: drop the next send callback (missing-callback recovery). The same seq
//    is resent afterwards: if the first frame did arrive, the indoor log
//    shows "duplicate"; if not, it is simply accepted.
// h: take a new sample before processing the next SUCCESS callback (the
//    ACK for the old seq must not clear the newer pending sample: the
//    newer seq is sent next)
// f: toggle failure of every channel change (whole sweeps fail locally ->
//    radio restart; restarts keep failing on the link backoff, no session
//    is opened; toggle again to recover)
// d: resend the last radio-acked packet (duplicate)
// o: resend the radio-acked packet before it (out of order)
// A radio ACK is not proof of acceptance: verify d/o against the indoor
// log ("Outdoor seq N duplicate" / "out of order") for the seq printed here.
void handleFaultInjection() {
    while (Serial.available()) {
        int command = Serial.read();
        switch (command) {
            case 'c':
                fiDropCallback = true;
                Serial.println("FI: next send callback will be dropped");
                break;
            case 'h':
                fiHoldAck = true;
                Serial.println("FI: next send callback processed after a new sample");
                break;
            case 'f':
                fiChannelFail = !fiChannelFail;
                Serial.printf("FI: channel changes %s\n", fiChannelFail ? "fail" : "work");
                break;
            case 'd':
            case 'o': {
                uint8_t index = command == 'o' ? 1 : 0;
                if (fiSentCount > index) {
                    fiResend = index;
                    Serial.printf("FI: will resend seq %lu\n", (unsigned long)fiSent[index].seq);
                } else {
                    Serial.println("FI: not enough sent packets yet");
                }
                break;
            }
        }
    }
}
#endif
