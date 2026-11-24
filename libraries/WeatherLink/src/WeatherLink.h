// Outdoor -> indoor ESP-NOW protocol, shared by both station sketches.
// Bump VERSION whenever the packet layout or the meaning of a field changes;
// a station rejects packets of any other version.
#ifndef WeatherLink_h
#define WeatherLink_h

#include <Arduino.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <math.h>
#include <string.h>

namespace weatherlink {

const uint8_t MAGIC = 0x57;  // 'W'
const uint8_t VERSION = 1;

// Sensor-group validity bits in OutdoorPacket::validFlags
const uint8_t VALID_SHT31 = 0x01;
const uint8_t VALID_BME280 = 0x02;
const uint8_t VALID_PMS = 0x04;
const uint8_t VALID_MASK = VALID_SHT31 | VALID_BME280 | VALID_PMS;

// Channels both stations may use (intersected at runtime with the radio's
// regulatory domain). While the indoor station is on WiFi it sits on the
// router's channel, so the router must be on one of these. DEFAULT_CHANNEL
// is where both stations start after a cold boot: set it to the router's
// usual channel to make the first contact immediate.
const uint8_t CHANNEL_MIN = 1;
const uint8_t CHANNEL_MAX = 11;
const uint8_t DEFAULT_CHANNEL = 1;

// The outdoor station samples this often and replaces an unsent sample with
// the next one, so a genuine packet is never much older than one interval
// (plus a channel sweep); anything beyond MAX_SAMPLE_AGE_MS is rejected
const uint32_t OUTDOOR_SAMPLE_INTERVAL_MS = 30000;
const uint32_t MAX_SAMPLE_AGE_MS = 2 * OUTDOOR_SAMPLE_INTERVAL_MS;

struct __attribute__((packed)) OutdoorPacket {
    uint8_t magic;
    uint8_t version;
    uint8_t validFlags;
    uint8_t reserved;
    uint32_t bootId;       // random per outdoor boot: detects restarts
    uint32_t seq;          // per sample, starts at 1; resends keep it
    uint32_t sampleAgeMs;  // sample age when this copy was transmitted
    float temp_sht31;
    float humi_sht31;
    float temp_bme280;
    float humi_bme280;
    float pres_bme280;     // mmHg, already calibrated like all values
    uint16_t pm01;
    uint16_t pm25;
    uint16_t pm10;
};

static_assert(sizeof(OutdoorPacket) == 42, "OutdoorPacket layout changed: bump VERSION");
static_assert(sizeof(OutdoorPacket) <= ESP_NOW_MAX_DATA_LEN, "OutdoorPacket exceeds ESP-NOW payload");

// Monotonic milliseconds since boot; 64-bit, so it never rolls over
inline uint64_t monoMs() {
    return (uint64_t)esp_timer_get_time() / 1000;
}

// Same limits as validateSensorData() in both sketches (repeated there so
// the working sensor code stays untouched) - keep them in sync
inline bool tempInRange(float t) { return isfinite(t) && t >= -40 && t <= 85; }
inline bool humiInRange(float h) { return isfinite(h) && h >= 0 && h <= 100; }
inline bool presInRange(float p) { return isfinite(p) && p >= 500 && p <= 850; }  // mmHg
inline bool pmInRange(uint16_t v) { return v <= 2000; }

// Cheap check for the receive callback: is this one of our packets at all
inline bool envelopeOk(const uint8_t* data, int len) {
    return len == (int)sizeof(OutdoorPacket) && data[0] == MAGIC && data[1] == VERSION;
}

enum class DecodeResult { OK, BAD_ENVELOPE, BAD_FLAGS, TOO_OLD };

// Full receiver-side validation. Copies the packet (memcpy: the radio buffer
// may be unaligned) and clears the flag of any group whose values are not
// plausible. Applies no calibration: values arrive already compensated.
inline DecodeResult decodePacket(const uint8_t* data, int len, OutdoorPacket& out) {
    if (!envelopeOk(data, len)) {
        return DecodeResult::BAD_ENVELOPE;
    }
    memcpy(&out, data, sizeof(out));
    if (out.validFlags & ~VALID_MASK) {
        return DecodeResult::BAD_FLAGS;
    }
    if (out.sampleAgeMs > MAX_SAMPLE_AGE_MS) {
        return DecodeResult::TOO_OLD;
    }

    if ((out.validFlags & VALID_SHT31) &&
        !(tempInRange(out.temp_sht31) && humiInRange(out.humi_sht31))) {
        out.validFlags &= ~VALID_SHT31;
    }
    if ((out.validFlags & VALID_BME280) &&
        !(tempInRange(out.temp_bme280) && humiInRange(out.humi_bme280) &&
          presInRange(out.pres_bme280))) {
        out.validFlags &= ~VALID_BME280;
    }
    if ((out.validFlags & VALID_PMS) &&
        !(pmInRange(out.pm01) && pmInRange(out.pm25) && pmInRange(out.pm10))) {
        out.validFlags &= ~VALID_PMS;
    }
    return DecodeResult::OK;
}

}  // namespace weatherlink

#endif
