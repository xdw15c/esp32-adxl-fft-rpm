#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"

#if ENABLE_OLED
#include <Wire.h>
#include <U8g2lib.h>
#endif

#if ENABLE_ADXL
#include <SPI.h>
#include <Adafruit_ADXL345_U.h>
#include <arduinoFFT.h>
#endif

#ifndef ENABLE_OLED
#define ENABLE_OLED 0
#endif
#ifndef ENABLE_ADXL
#define ENABLE_ADXL 0
#endif
#ifndef ENABLE_MODBUS
#define ENABLE_MODBUS 0
#endif
#ifndef CONFIG_SDA_PIN
#define CONFIG_SDA_PIN 8
#endif
#ifndef CONFIG_SCL_PIN
#define CONFIG_SCL_PIN 9
#endif
#ifndef CONFIG_OLED_ADDR
#define CONFIG_OLED_ADDR 0x3C
#endif
#ifndef CONFIG_ADXL_SPI_CS_PIN
#define CONFIG_ADXL_SPI_CS_PIN 5
#endif
#ifndef CONFIG_ADXL_SPI_SCK_PIN
#define CONFIG_ADXL_SPI_SCK_PIN 18
#endif
#ifndef CONFIG_ADXL_SPI_MOSI_PIN
#define CONFIG_ADXL_SPI_MOSI_PIN 23
#endif
#ifndef CONFIG_ADXL_SPI_MISO_PIN
#define CONFIG_ADXL_SPI_MISO_PIN 19
#endif
#ifndef CONFIG_WIFI_TIMEOUT_MS
#define CONFIG_WIFI_TIMEOUT_MS 15000
#endif
#ifndef CONFIG_HEARTBEAT_MS
#define CONFIG_HEARTBEAT_MS 5000
#endif
#ifndef CONFIG_SENSOR_LOG_MS
#define CONFIG_SENSOR_LOG_MS 2000
#endif
#ifndef CONFIG_WIFI_RETRY_MS
#define CONFIG_WIFI_RETRY_MS 10000
#endif
#ifndef CONFIG_FFT_SIZE
#define CONFIG_FFT_SIZE 256
#endif
#ifndef CONFIG_FFT_FS
#define CONFIG_FFT_FS 800
#endif
#ifndef CONFIG_FFT_PEAK_MIN_HZ
#define CONFIG_FFT_PEAK_MIN_HZ 5.0f
#endif
#ifndef CONFIG_FFT_PEAK_MAX_HZ
#define CONFIG_FFT_PEAK_MAX_HZ 300.0f
#endif
#ifndef CONFIG_RPM_ORDER
#define CONFIG_RPM_ORDER 1.0f
#endif
#ifndef CONFIG_HPF_CUTOFF_HZ
#define CONFIG_HPF_CUTOFF_HZ 2.0f
#endif
#ifndef CONFIG_READ_ERROR_CONF_THRESH_PCT
#define CONFIG_READ_ERROR_CONF_THRESH_PCT 60
#endif
#ifndef CONFIG_MODBUS_PORT
#define CONFIG_MODBUS_PORT 502
#endif
#ifndef CONFIG_MODBUS_UNIT_ID
#define CONFIG_MODBUS_UNIT_ID 1
#endif
#ifndef CONFIG_RMS_ALARM_THRESHOLD_X10
#define CONFIG_RMS_ALARM_THRESHOLD_X10 110
#endif
#ifndef CONFIG_SPIKE_N
#define CONFIG_SPIKE_N 5
#endif
#ifndef CONFIG_TREND_THRESHOLD_PCT
#define CONFIG_TREND_THRESHOLD_PCT 20
#endif
#ifndef CONFIG_HARM_RATIO_THRESH_PCT
#define CONFIG_HARM_RATIO_THRESH_PCT 30
#endif
#ifndef CONFIG_HARM_MAX_ORDER
#define CONFIG_HARM_MAX_ORDER 4
#endif

WebServer server(80);
#if ENABLE_MODBUS
WiFiServer modbusServer(CONFIG_MODBUS_PORT);
#endif

static unsigned long g_lastHeartbeatMs = 0;
static unsigned long g_lastWifiRetryMs = 0;
static wl_status_t g_prevWifiStatus = WL_NO_SHIELD;
static uint8_t g_peakConfThreshPct = CONFIG_READ_ERROR_CONF_THRESH_PCT;
static uint16_t g_rmsAlarmThresholdX10 = CONFIG_RMS_ALARM_THRESHOLD_X10;
static uint16_t g_spikeN = CONFIG_SPIKE_N;
static uint16_t g_trendThresholdPct = CONFIG_TREND_THRESHOLD_PCT;
static uint16_t g_fftBandMinHz = (uint16_t)CONFIG_FFT_PEAK_MIN_HZ;
static uint16_t g_fftBandMaxHz = (uint16_t)CONFIG_FFT_PEAK_MAX_HZ;
static uint16_t g_rpmOrder = (uint16_t)CONFIG_RPM_ORDER;
static uint16_t g_alarmFlags = 0;
static bool g_sensorPresent = ENABLE_ADXL != 0;

#if ENABLE_OLED
static unsigned long g_lastOledMs = 0;
static void oledMsg(const char* line1, const char* line2 = nullptr);
#endif

#if ENABLE_ADXL
static float g_lastX = 0.0f;
static float g_lastY = 0.0f;
static float g_lastZ = 0.0f;
static float g_lastMag = 0.0f;
static unsigned long g_samples = 0;
static unsigned long g_lastSensorLogMs = 0;

static uint32_t g_lastSampleUs = 0;
static const uint32_t g_samplePeriodUs = 1000000UL / CONFIG_FFT_FS;

static float g_fftReal[CONFIG_FFT_SIZE];
static float g_fftImag[CONFIG_FFT_SIZE];
static float g_fftMag[CONFIG_FFT_SIZE / 2];
static float g_fftPeakHz = 0.0f;
static float g_fftPeakHzFilt = 0.0f;
static float g_fftPeakAmp = 0.0f;
static float g_fftPeakRpm = 0.0f;
static float g_fftPeakConfidencePct = 0.0f;
static float g_rmsX = 0.0f;
static float g_rmsY = 0.0f;
static float g_rmsZ = 0.0f;
static float g_rmsTotal = 0.0f;
static float g_winSumSqX = 0.0f;
static float g_winSumSqY = 0.0f;
static float g_winSumSqZ = 0.0f;
static uint16_t g_winSampleCount = 0;
static uint16_t g_fftIdx = 0;
static float g_hpfPrevIn = 0.0f;
static float g_hpfPrevOut = 0.0f;
static float g_harmonicAmp[3] = {0.0f, 0.0f, 0.0f};  // index 0=2x, 1=3x, 2=4x of fundamental
static uint8_t g_harmonicAlarmFlags = 0;               // bits: 0=2x active, 1=3x active, 2=4x active, 3=multiple
static uint8_t g_harmRatioThreshPct = CONFIG_HARM_RATIO_THRESH_PCT;
static uint8_t g_harmMaxOrder = CONFIG_HARM_MAX_ORDER;

static ArduinoFFT<float> g_fft(g_fftReal, g_fftImag, CONFIG_FFT_SIZE, (float)CONFIG_FFT_FS);

Adafruit_ADXL345_Unified accel(
    CONFIG_ADXL_SPI_SCK_PIN,
    CONFIG_ADXL_SPI_MISO_PIN,
    CONFIG_ADXL_SPI_MOSI_PIN,
    CONFIG_ADXL_SPI_CS_PIN,
    12345);

static float ms2ToMg(float valueMs2) {
    return valueMs2 * (1000.0f / 9.80665f);
}

static uint16_t clampToU16(long value) {
    if (value < 0) return 0;
    if (value > 65535L) return 65535;
    return (uint16_t)value;
}

static int16_t clampToI16(long value) {
    if (value < -32768L) return -32768;
    if (value > 32767L) return 32767;
    return (int16_t)value;
}

// Searches g_fftMag[] for energy at 2x, 3x, 4x of fundHz relative to the fundamental peak.
// Sets g_harmonicAmp[0..2] and g_harmonicAlarmFlags bits.
static void computeHarmonics(float fundHz) {
    const float binHz = (float)CONFIG_FFT_FS / (float)CONFIG_FFT_SIZE;
    const uint16_t halfN = CONFIG_FFT_SIZE / 2;
    g_harmonicAlarmFlags = 0;
    uint8_t harmCount = 0;
    for (uint8_t k = 2; k <= g_harmMaxOrder && k <= 4; k++) {
        const uint8_t idx = k - 2;  // 2x->0, 3x->1, 4x->2
        const float targetHz = (float)k * fundHz;
        if (targetHz >= (float)(CONFIG_FFT_FS / 2)) {
            g_harmonicAmp[idx] = 0.0f;
            continue;
        }
        const uint16_t centerBin = (uint16_t)(targetHz / binHz);
        const uint16_t lo = (centerBin > 2) ? (centerBin - 2) : 1;
        const uint16_t hi = min((uint16_t)(centerBin + 2), (uint16_t)(halfN - 1));
        float maxAmp = 0.0f;
        for (uint16_t i = lo; i <= hi; i++) {
            if (g_fftMag[i] > maxAmp) maxAmp = g_fftMag[i];
        }
        g_harmonicAmp[idx] = maxAmp;
        const float ratio = (g_fftPeakAmp > 1e-6f) ? (maxAmp / g_fftPeakAmp) : 0.0f;
        if (ratio * 100.0f > (float)g_harmRatioThreshPct) {
            harmCount++;
            g_harmonicAlarmFlags |= (uint8_t)(1u << idx);
        }
    }
    if (harmCount >= 2) {
        g_harmonicAlarmFlags |= (uint8_t)(1u << 3);  // multiple harmonics
    }
}

static void updateAlarmFlags() {
    g_alarmFlags = 0;
    if (clampToU16(lroundf(g_rmsTotal * 10.0f)) > g_rmsAlarmThresholdX10) {
        g_alarmFlags |= 1u << 0;
    }
    if (g_fftPeakConfidencePct >= g_peakConfThreshPct &&
        (g_fftPeakHzFilt < (float)g_fftBandMinHz || g_fftPeakHzFilt > (float)g_fftBandMaxHz)) {
        g_alarmFlags |= 1u << 3;
    }
    if (!g_sensorPresent) {
        g_alarmFlags |= 1u << 4;
    }
    // bits 5-8: harmonic alarms (shifted from g_harmonicAlarmFlags bits 0-3)
    g_alarmFlags |= (uint16_t)(g_harmonicAlarmFlags << 5);
}

static void computeFFT() {
    float mean = 0.0f;
    for (uint16_t i = 0; i < CONFIG_FFT_SIZE; i++) {
        mean += g_fftReal[i];
    }
    mean /= CONFIG_FFT_SIZE;

    for (uint16_t i = 0; i < CONFIG_FFT_SIZE; i++) {
        g_fftReal[i] -= mean;
        g_fftImag[i] = 0.0f;
    }

    g_fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    g_fft.compute(FFTDirection::Forward);
    g_fft.complexToMagnitude();

    const float binHz = (float)CONFIG_FFT_FS / (float)CONFIG_FFT_SIZE;
    const uint16_t halfN = CONFIG_FFT_SIZE / 2;
    uint16_t minBin = (uint16_t)((float)g_fftBandMinHz / binHz);
    uint16_t maxBin = (uint16_t)((float)g_fftBandMaxHz / binHz);
    if (minBin < 1) minBin = 1;
    if (maxBin >= (halfN - 1)) maxBin = halfN - 1;
    const float norm = 2.0f / (float)CONFIG_FFT_SIZE;
    const float magAlpha = 0.22f;

    uint16_t peakBin = minBin;
    float peakAmp = 0.0f;
    float noiseSum = 0.0f;
    uint16_t noiseCount = 0;
    g_fftMag[0] = 0.0f;

    for (uint16_t i = 1; i < halfN; i++) {
        const float magNow = g_fftReal[i] * norm;
        g_fftMag[i] = (1.0f - magAlpha) * g_fftMag[i] + magAlpha * magNow;
        if (i >= minBin && i <= maxBin && g_fftMag[i] > peakAmp) {
            peakAmp = g_fftMag[i];
            peakBin = i;
        }
    }

    for (uint16_t i = minBin; i <= maxBin; i++) {
        if (i + 1 < peakBin || i > peakBin + 1) {
            noiseSum += g_fftMag[i];
            noiseCount++;
        }
    }

    float peakHz = peakBin * binHz;
    if (peakBin > minBin && peakBin < maxBin) {
        const float ym1 = g_fftMag[peakBin - 1];
        const float y0 = g_fftMag[peakBin];
        const float yp1 = g_fftMag[peakBin + 1];
        const float d = (ym1 - 2.0f * y0 + yp1);
        if (fabsf(d) > 1e-6f) {
            const float delta = 0.5f * (ym1 - yp1) / d;
            peakHz = (peakBin + constrain(delta, -1.0f, 1.0f)) * binHz;
        }
    }

    g_fftPeakHz = peakHz;
    g_fftPeakAmp = peakAmp;
    g_fftPeakHzFilt = 0.85f * g_fftPeakHzFilt + 0.15f * g_fftPeakHz;
    const float noiseFloor = (noiseCount > 0) ? (noiseSum / (float)noiseCount) : 0.0f;
    const float prominence = peakAmp / (noiseFloor + 1e-6f);
    float confNow = ((prominence - 1.0f) / 7.0f) * 100.0f;
    confNow = constrain(confNow, 0.0f, 100.0f);
    g_fftPeakConfidencePct = 0.85f * g_fftPeakConfidencePct + 0.15f * confNow;

    if (g_fftPeakConfidencePct >= g_peakConfThreshPct) {
        g_fftPeakRpm = (g_fftPeakHzFilt * 60.0f) / max<uint16_t>(1, g_rpmOrder);
        computeHarmonics(g_fftPeakHzFilt);
    } else {
        g_fftPeakRpm = 0.0f;
        g_harmonicAlarmFlags = 0;
        g_harmonicAmp[0] = 0.0f;
        g_harmonicAmp[1] = 0.0f;
        g_harmonicAmp[2] = 0.0f;
    }

    updateAlarmFlags();
}
#endif

#if ENABLE_OLED
static void scanI2CBus() {
    Serial.println("[I2C] Scanning bus...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("[I2C] Device found at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[I2C] No devices detected");
    } else {
        Serial.printf("[I2C] Scan done, devices=%u\n", found);
    }
}
#endif

static const char* wifiStatusName(wl_status_t status) {
    switch (status) {
        case WL_IDLE_STATUS: return "IDLE";
        case WL_NO_SSID_AVAIL: return "NO_SSID";
        case WL_SCAN_COMPLETED: return "SCAN_DONE";
        case WL_CONNECTED: return "CONNECTED";
        case WL_CONNECT_FAILED: return "CONNECT_FAILED";
        case WL_CONNECTION_LOST: return "CONNECTION_LOST";
        case WL_DISCONNECTED: return "DISCONNECTED";
        case WL_NO_SHIELD: return "NO_SHIELD";
        default: return "UNKNOWN";
    }
}

static void logStatusSnapshot(const char* tag) {
    const bool wifiOk = (WiFi.status() == WL_CONNECTED);
    const String ip = wifiOk ? WiFi.localIP().toString() : String("-");
    const int rssi = wifiOk ? WiFi.RSSI() : 0;
#if ENABLE_ADXL
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm samples=%lu vib=%.2f peak=%.1fHz conf=%.0f%% rpm=%.0f\n",
                  tag, millis(), wifiStatusName(WiFi.status()), ip.c_str(), rssi, g_samples, g_lastMag, g_fftPeakHzFilt,
                  g_fftPeakConfidencePct, g_fftPeakRpm);
#else
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm heap=%u\n",
                  tag, millis(), wifiStatusName(WiFi.status()), ip.c_str(), rssi, ESP.getFreeHeap());
#endif
}

#if ENABLE_MODBUS
static uint16_t modbusReadInputRegister(uint16_t addr, bool& ok) {
    ok = true;
    switch (addr) {
        case 0: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastX) / 3.9f));
        case 1: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastY) / 3.9f));
        case 2: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastZ) / 3.9f));
        case 3: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastX)));
        case 4: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastY)));
        case 5: return (uint16_t)clampToI16(lroundf(ms2ToMg(g_lastZ)));
        case 6: return clampToU16(lroundf(ms2ToMg(g_rmsX) * 10.0f));
        case 7: return clampToU16(lroundf(ms2ToMg(g_rmsY) * 10.0f));
        case 8: return clampToU16(lroundf(ms2ToMg(g_rmsZ) * 10.0f));
        case 9: return clampToU16(lroundf(ms2ToMg(g_rmsTotal) * 10.0f));
        case 10: return g_alarmFlags;
        case 11: return 0;
        case 12: return clampToU16(lroundf(g_fftPeakHzFilt));
        case 13: return clampToU16(lroundf(g_fftPeakAmp * 10.0f));
        case 14: return CONFIG_FFT_FS;
        case 15: return g_sensorPresent ? 0 : 1;
        case 16: return clampToU16(lroundf(g_harmonicAmp[0] * 100.0f));  // 2x amp x100
        case 17: return clampToU16(lroundf(g_harmonicAmp[1] * 100.0f));  // 3x amp x100
        case 18: return clampToU16(lroundf(g_harmonicAmp[2] * 100.0f));  // 4x amp x100
        case 19: return g_harmonicAlarmFlags;  // bits: 0=2x, 1=3x, 2=4x, 3=multiple
        default:
            ok = false;
            return 0;
    }
}

static uint16_t modbusReadHoldingRegister(uint16_t addr, bool& ok) {
    ok = true;
    switch (addr) {
        case 100: return g_rmsAlarmThresholdX10;
        case 101: return g_spikeN;
        case 102: return g_trendThresholdPct;
        case 103: return g_fftBandMinHz;
        case 104: return g_fftBandMaxHz;
        case 105: return g_rpmOrder;
        case 106: return g_harmRatioThreshPct;
        case 107: return g_harmMaxOrder;
        default:
            ok = false;
            return 0;
    }
}

static bool modbusWriteHoldingRegister(uint16_t addr, uint16_t value) {
    switch (addr) {
        case 100:
            g_rmsAlarmThresholdX10 = value;
            return true;
        case 101:
            g_spikeN = max<uint16_t>(1, value);
            return true;
        case 102:
            g_trendThresholdPct = value;
            return true;
        case 103:
            if (value >= g_fftBandMaxHz) return false;
            g_fftBandMinHz = value;
            return true;
        case 104:
            if (value <= g_fftBandMinHz) return false;
            g_fftBandMaxHz = value;
            return true;
        case 105:
            g_rpmOrder = max<uint16_t>(1, value);
            return true;
        case 106:
            g_harmRatioThreshPct = (uint8_t)min<uint16_t>(value, 100);
            if (g_harmRatioThreshPct < 1) g_harmRatioThreshPct = 1;
            return true;
        case 107:
            g_harmMaxOrder = (uint8_t)constrain((int)value, 2, 4);
            return true;
        default:
            return false;
    }
}

static void modbusPutU16(uint8_t* dst, uint16_t value) {
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)(value & 0xFF);
}

static void modbusSendException(WiFiClient& client, uint16_t txId, uint8_t unitId, uint8_t function, uint8_t exceptionCode) {
    uint8_t resp[9] = {0};
    modbusPutU16(&resp[0], txId);
    modbusPutU16(&resp[2], 0);
    modbusPutU16(&resp[4], 3);
    resp[6] = unitId;
    resp[7] = (uint8_t)(function | 0x80u);
    resp[8] = exceptionCode;
    client.write(resp, sizeof(resp));
}

static void modbusSendReadResponse(WiFiClient& client, uint16_t txId, uint8_t unitId, uint8_t function, const uint16_t* regs, uint16_t quantity) {
    const uint16_t byteCount = quantity * 2;
    uint8_t resp[9 + 125 * 2] = {0};
    modbusPutU16(&resp[0], txId);
    modbusPutU16(&resp[2], 0);
    modbusPutU16(&resp[4], (uint16_t)(3 + byteCount));
    resp[6] = unitId;
    resp[7] = function;
    resp[8] = (uint8_t)byteCount;
    for (uint16_t i = 0; i < quantity; i++) {
        modbusPutU16(&resp[9 + i * 2], regs[i]);
    }
    client.write(resp, 9 + byteCount);
}

static void modbusSendWriteSingleResponse(WiFiClient& client, uint16_t txId, uint8_t unitId, uint16_t addr, uint16_t value) {
    uint8_t resp[12] = {0};
    modbusPutU16(&resp[0], txId);
    modbusPutU16(&resp[2], 0);
    modbusPutU16(&resp[4], 6);
    resp[6] = unitId;
    resp[7] = 6;
    modbusPutU16(&resp[8], addr);
    modbusPutU16(&resp[10], value);
    client.write(resp, sizeof(resp));
}

static void handleModbusRequest(WiFiClient& client) {
    if (client.available() < 7) return;

    uint8_t header[7];
    if (client.readBytes(header, sizeof(header)) != sizeof(header)) return;

    const uint16_t txId = (uint16_t)((header[0] << 8) | header[1]);
    const uint16_t protoId = (uint16_t)((header[2] << 8) | header[3]);
    const uint16_t length = (uint16_t)((header[4] << 8) | header[5]);
    const uint8_t unitId = header[6];
    if (protoId != 0 || length < 2 || length > 253 || unitId != CONFIG_MODBUS_UNIT_ID) {
        return;
    }

    const uint16_t pduLen = length - 1;
    uint8_t pdu[253] = {0};
    if (client.readBytes(pdu, pduLen) != pduLen) return;

    const uint8_t function = pdu[0];
    if (function == 3 || function == 4) {
        if (pduLen < 5) {
            modbusSendException(client, txId, unitId, function, 3);
            return;
        }
        const uint16_t startAddr = (uint16_t)((pdu[1] << 8) | pdu[2]);
        const uint16_t quantity = (uint16_t)((pdu[3] << 8) | pdu[4]);
        if (quantity < 1 || quantity > 125) {
            modbusSendException(client, txId, unitId, function, 3);
            return;
        }
        uint16_t regs[125] = {0};
        for (uint16_t i = 0; i < quantity; i++) {
            bool ok = false;
            regs[i] = (function == 4)
                ? modbusReadInputRegister((uint16_t)(startAddr + i), ok)
                : modbusReadHoldingRegister((uint16_t)(startAddr + i), ok);
            if (!ok) {
                modbusSendException(client, txId, unitId, function, 2);
                return;
            }
        }
        modbusSendReadResponse(client, txId, unitId, function, regs, quantity);
        return;
    }

    if (function == 6) {
        if (pduLen < 5) {
            modbusSendException(client, txId, unitId, function, 3);
            return;
        }
        const uint16_t addr = (uint16_t)((pdu[1] << 8) | pdu[2]);
        const uint16_t value = (uint16_t)((pdu[3] << 8) | pdu[4]);
        if (!modbusWriteHoldingRegister(addr, value)) {
            modbusSendException(client, txId, unitId, function, 2);
            return;
        }
        updateAlarmFlags();
        modbusSendWriteSingleResponse(client, txId, unitId, addr, value);
        return;
    }

    modbusSendException(client, txId, unitId, function, 1);
}

static void handleModbusTcp() {
    static WiFiClient client;

    if (!client || !client.connected()) {
        if (client) client.stop();
        client = modbusServer.available();
        if (client) {
            client.setTimeout(20);
            Serial.printf("[MODBUS] Client connected: %s\n", client.remoteIP().toString().c_str());
        }
    }

    if (client && client.connected()) {
        while (client.available() >= 7) {
            handleModbusRequest(client);
        }
    }
}
#endif

static bool ensureWebAuth() {
#if defined(WEB_USERNAME) && defined(WEB_PASSWORD)
    if (!server.authenticate(WEB_USERNAME, WEB_PASSWORD)) {
        Serial.printf("[HTTP] Unauthorized request from %s to %s\n",
                      server.client().remoteIP().toString().c_str(),
                      server.uri().c_str());
        server.requestAuthentication();
        return false;
    }
#endif
    return true;
}

static void handleStatusJson() {
    if (!ensureWebAuth()) return;

    const bool wifiOk = (WiFi.status() == WL_CONNECTED);
    char json[512];
#if ENABLE_ADXL
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,"
             "\"samples\":%lu,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vibration\":%.3f,"
             "\"peak_hz\":%.2f,\"peak_amp\":%.3f,\"peak_confidence_pct\":%.1f,\"rpm\":%.1f,\"peak_confidence_threshold_pct\":%u}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             g_samples, g_lastX, g_lastY, g_lastZ, g_lastMag, g_fftPeakHzFilt, g_fftPeakAmp, g_fftPeakConfidencePct,
             g_fftPeakRpm, g_peakConfThreshPct);
#else
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,\"heap\":%u,\"peak_confidence_threshold_pct\":%u}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             ESP.getFreeHeap(),
             g_peakConfThreshPct);
#endif
    server.send(200, "application/json", json);
}

static void handleConfigJson() {
    if (!ensureWebAuth()) return;

    if (server.hasArg("peak_confidence_threshold_pct")) {
        int v = server.arg("peak_confidence_threshold_pct").toInt();
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        g_peakConfThreshPct = (uint8_t)v;
        Serial.printf("[CFG] peak_confidence_threshold_pct=%u\n", g_peakConfThreshPct);
    } else if (server.hasArg("read_error_threshold_pct")) {
        int v = server.arg("read_error_threshold_pct").toInt();
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        g_peakConfThreshPct = (uint8_t)v;
        Serial.printf("[CFG] peak_confidence_threshold_pct=%u\n", g_peakConfThreshPct);
    } else if (server.hasArg("harm_ratio_thresh_pct")) {
        int v = server.arg("harm_ratio_thresh_pct").toInt();
        if (v < 1) v = 1;
        if (v > 100) v = 100;
        g_harmRatioThreshPct = (uint8_t)v;
        Serial.printf("[CFG] harm_ratio_thresh_pct=%u\n", g_harmRatioThreshPct);
    }

    char json[128];
    snprintf(json, sizeof(json),
             "{\"peak_confidence_threshold_pct\":%u,\"harm_ratio_thresh_pct\":%u,\"harm_max_order\":%u}",
             g_peakConfThreshPct, g_harmRatioThreshPct, g_harmMaxOrder);
    server.send(200, "application/json", json);
}

static void handleFftJson() {
    if (!ensureWebAuth()) return;

    String resp;
    resp.reserve(1400);
    resp += F("{\"fs\":");
    resp += CONFIG_FFT_FS;
    resp += F(",\"n\":");
    resp += (CONFIG_FFT_SIZE / 2);
    resp += F(",\"resolution\":");
    resp += ((float)CONFIG_FFT_FS / CONFIG_FFT_SIZE);
#if ENABLE_ADXL
    resp += F(",\"peak_hz\":");
    resp += g_fftPeakHzFilt;
    resp += F(",\"peak_amp\":");
    resp += g_fftPeakAmp;
    resp += F(",\"peak_confidence_pct\":");
    resp += g_fftPeakConfidencePct;
    resp += F(",\"rpm\":");
    resp += g_fftPeakRpm;
    resp += F(",\"rpm_order\":");
    resp += g_rpmOrder;
    resp += F(",\"band_min_hz\":");
    resp += g_fftBandMinHz;
    resp += F(",\"band_max_hz\":");
    resp += g_fftBandMaxHz;
    resp += F(",\"mag\":[");
    for (uint16_t i = 0; i < CONFIG_FFT_SIZE / 2; i++) {
        if (i) resp += ',';
        resp += g_fftMag[i];
    }
    resp += F("]}");
#else
    resp += F(",\"peak_hz\":0,\"mag\":[]}");
#endif

    server.send(200, "application/json", resp);
}

static void handleRoot() {
    if (!ensureWebAuth()) return;

    static const char page[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pl">
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32-C3 ADXL345 FFT</title>
    <style>
        :root { color-scheme: light; }
        body {
            margin: 0;
            font-family: "Segoe UI", Tahoma, sans-serif;
            background: linear-gradient(140deg, #f7fbff, #e5f2ff);
            color: #0f172a;
        }
        .wrap {
            max-width: 740px;
            margin: 24px auto;
            padding: 0 14px;
        }
        .card {
            background: #ffffff;
            border-radius: 14px;
            padding: 16px;
            box-shadow: 0 10px 28px rgba(2, 6, 23, 0.08);
            margin-bottom: 12px;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 10px;
        }
        .label { color: #334155; font-size: 0.86rem; }
        .value { font-size: 1.06rem; font-weight: 650; margin-top: 4px; }
        .mono { font-family: Consolas, "Courier New", monospace; }
        #fftCanvas { width: 100%; height: 180px; background: #f8fafc; border-radius: 8px; display: block; }
        .cfg-row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin-top: 6px; }
        .cfg-row input[type="number"] { width: 84px; padding: 4px 6px; }
        .cfg-row button { padding: 5px 10px; border: 0; border-radius: 6px; background: #2563eb; color: #fff; cursor: pointer; }
        .muted { color: #64748b; font-size: 0.83rem; }
    </style>
</head>
<body>
    <div class="wrap">
        <div class="card">
            <h1>ESP32-C3 ADXL345</h1>
            <div class="grid">
                <div><div class="label">Wi-Fi</div><div class="value" id="wifi">-</div></div>
                <div><div class="label">IP</div><div class="value mono" id="ip">-</div></div>
                <div><div class="label">RSSI</div><div class="value" id="rssi">-</div></div>
                <div><div class="label">Uptime [ms]</div><div class="value mono" id="uptime">-</div></div>
            </div>
        </div>

        <div class="card">
            <div class="grid">
                <div><div class="label">X [m/s2]</div><div class="value mono" id="x">-</div></div>
                <div><div class="label">Y [m/s2]</div><div class="value mono" id="y">-</div></div>
                <div><div class="label">Z [m/s2]</div><div class="value mono" id="z">-</div></div>
                <div><div class="label">Vibration [m/s2]</div><div class="value mono" id="v">-</div></div>
            </div>
            <p class="label">Probek: <span class="mono" id="samples">-</span> | Peak FFT: <span class="mono" id="peak">-</span> | RPM: <span class="mono" id="rpm">-</span></p>
            <div class="cfg-row">
                <span class="label">Prog pewnosci bledu odczytu [%]</span>
                <input id="errThr" type="number" min="0" max="100" step="1" value="60" />
                <button id="saveErrThr" type="button">Zapisz</button>
                <span class="muted">Pewnosc: <span class="mono" id="errConf">0</span>%</span>
            </div>
            <div class="cfg-row">
                <span class="label">Prog amplitudy harmonicznej [%]</span>
                <input id="harmThr" type="number" min="1" max="100" step="1" value="30" />
                <button id="saveHarmThr" type="button">Zapisz</button>
                <span class="muted">Harmoniczne: <span class="mono" id="harmStatus">-</span></span>
            </div>
        </div>

        <div class="card">
            <div style="font-size:.9rem;font-weight:600;color:#334155;margin-bottom:6px">Widmo FFT |a| (0 - 400 Hz)</div>
            <canvas id="fftCanvas"></canvas>
            <p class="label">Fs=<span id="fftFs">-</span> Hz | <span id="fftRes">-</span> Hz/bin | N=<span id="fftN">-</span> | Pasmo: <span id="fftBand">-</span></p>
        </div>
    </div>

    <script>
        const $ = id => document.getElementById(id);
        const fc = $('fftCanvas');
        const fctx = fc.getContext('2d');
        let peakConfidenceThresholdPct = 60;
        let peakConfidencePct = 0;
        let harmRatioThreshPct = 30;

        async function saveErrorThreshold() {
            const raw = Number($('errThr').value);
            const val = Number.isFinite(raw) ? Math.max(0, Math.min(100, Math.round(raw))) : peakConfidenceThresholdPct;
            $('errThr').value = String(val);
            try {
                const r = await fetch('/api/config?peak_confidence_threshold_pct=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                peakConfidenceThresholdPct = Number(d.peak_confidence_threshold_pct || val);
                $('errThr').value = String(peakConfidenceThresholdPct);
            } catch (_) {
            }
        }

        async function saveHarmThreshold() {
            const raw = Number($('harmThr').value);
            const val = Number.isFinite(raw) ? Math.max(1, Math.min(100, Math.round(raw))) : harmRatioThreshPct;
            $('harmThr').value = String(val);
            try {
                const r = await fetch('/api/config?harm_ratio_thresh_pct=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                harmRatioThreshPct = Number(d.harm_ratio_thresh_pct || val);
                $('harmThr').value = String(harmRatioThreshPct);
            } catch (_) {
            }
        }

        async function refreshStatus() {
            try {
                const r = await fetch('/api/status', { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const s = await r.json();
                $('wifi').textContent = s.wifi_connected ? 'Polaczono' : 'Brak polaczenia';
                $('ip').textContent = s.ip;
                $('rssi').textContent = s.rssi + ' dBm';
                $('uptime').textContent = s.uptime_ms;
                $('x').textContent = s.x.toFixed(3);
                $('y').textContent = s.y.toFixed(3);
                $('z').textContent = s.z.toFixed(3);
                $('v').textContent = s.vibration.toFixed(3);
                $('samples').textContent = s.samples;
                $('peak').textContent = (s.peak_hz || 0).toFixed(1) + ' Hz';
                peakConfidencePct = Number(s.peak_confidence_pct || 0);
                if (typeof s.peak_confidence_threshold_pct === 'number') {
                    peakConfidenceThresholdPct = Math.max(0, Math.min(100, Math.round(s.peak_confidence_threshold_pct)));
                    $('errThr').value = String(peakConfidenceThresholdPct);
                }
                if (peakConfidencePct >= peakConfidenceThresholdPct) {
                    $('rpm').textContent = (s.rpm || 0).toFixed(0);
                } else {
                    $('rpm').textContent = '0';
                }
                $('errConf').textContent = peakConfidencePct.toFixed(0);
            } catch (_) {
                $('errConf').textContent = '0';
                $('rpm').textContent = '0';
                $('wifi').textContent = 'Blad odczytu';
            }
        }

        async function refreshFFT() {
            try {
                const r = await fetch('/api/fft', { cache: 'no-store' });
                if (!r.ok) return;
                const d = await r.json();

                $('fftFs').textContent = d.fs;
                $('fftRes').textContent = d.resolution.toFixed(2);
                $('fftN').textContent = d.n;
                $('fftBand').textContent = d.band_min_hz.toFixed(1) + '-' + d.band_max_hz.toFixed(1) + ' Hz';

                const dpr = window.devicePixelRatio || 1;
                const W = fc.clientWidth;
                const H = fc.clientHeight;
                fc.width = W * dpr;
                fc.height = H * dpr;
                fctx.setTransform(dpr, 0, 0, dpr, 0, 0);
                fctx.clearRect(0, 0, W, H);

                const mag = d.mag || [];
                const n = mag.length;
                if (!n) return;

                let mx = 1e-6;
                for (let i = 1; i < n; i++) {
                    if (mag[i] > mx) mx = mag[i];
                }

                // Draw smoothed log-scale line for better readability across wide dynamic range.
                const yBase = H - 22;
                fctx.beginPath();
                for (let i = 1; i < n; i++) {
                    const x = (i / (n - 1)) * W;
                    const yNorm = Math.log10(1.0 + 30.0 * (mag[i] / mx));
                    const y = yBase - (yNorm / Math.log10(31.0)) * (H - 34);
                    if (i == 1) fctx.moveTo(x, y);
                    else fctx.lineTo(x, y);
                }
                fctx.strokeStyle = '#2563eb';
                fctx.lineWidth = 1.8;
                fctx.stroke();

                // Fill under the curve.
                fctx.lineTo(W, yBase);
                fctx.lineTo(0, yBase);
                fctx.closePath();
                fctx.fillStyle = 'rgba(37,99,235,0.12)';
                fctx.fill();

                fctx.fillStyle = '#94a3b8';
                fctx.font = '10px sans-serif';
                fctx.textAlign = 'center';
                for (let f = 0; f <= d.fs / 2; f += 100) {
                    const x = (f / (d.fs / 2)) * W;
                    fctx.fillStyle = 'rgba(0,0,0,.06)';
                    fctx.fillRect(x, 0, 1, H - 22);
                    fctx.fillStyle = '#94a3b8';
                    fctx.fillText(f + 'Hz', x, H - 5);
                }

                // Peak marker.
                const peakX = (d.peak_hz / (d.fs / 2)) * W;
                fctx.strokeStyle = 'rgba(239,68,68,0.85)';
                fctx.lineWidth = 1;
                fctx.beginPath();
                fctx.moveTo(peakX, 0);
                fctx.lineTo(peakX, yBase);
                fctx.stroke();
                fctx.fillStyle = '#ef4444';
                fctx.textAlign = 'left';
                fctx.fillText(d.peak_hz.toFixed(1) + ' Hz', Math.min(peakX + 4, W - 56), 12);

                // Harmonic markers (2x, 3x, 4x)
                const harmAmps = d.harmonic_amps || [0, 0, 0];
                const harmAlarm = d.harmonic_alarm_flags || 0;
                if (typeof d.harm_ratio_thresh_pct === 'number') {
                    harmRatioThreshPct = d.harm_ratio_thresh_pct;
                    $('harmThr').value = String(harmRatioThreshPct);
                }
                const harmLabels = ['2x', '3x', '4x'];
                const harmStatus = [];
                fctx.font = '10px sans-serif';
                for (let k = 0; k < 3; k++) {
                    const harmHz = (k + 2) * d.peak_hz;
                    if (harmHz >= d.fs / 2) continue;
                    const hx = (harmHz / (d.fs / 2)) * W;
                    const active = harmAlarm & (1 << k);
                    fctx.strokeStyle = active ? 'rgba(234,179,8,0.9)' : 'rgba(74,222,128,0.55)';
                    fctx.lineWidth = 1;
                    fctx.setLineDash([4, 3]);
                    fctx.beginPath();
                    fctx.moveTo(hx, 0);
                    fctx.lineTo(hx, yBase);
                    fctx.stroke();
                    fctx.setLineDash([]);
                    fctx.fillStyle = active ? '#ca8a04' : '#16a34a';
                    fctx.textAlign = 'center';
                    fctx.fillText(harmLabels[k], hx, 34);
                    if (active) harmStatus.push(harmLabels[k]);
                }
                $('harmStatus').textContent = harmStatus.length ? harmStatus.join(' ') + ' !' : 'brak';
                $('harmStatus').style.color = harmStatus.length ? '#ca8a04' : '#16a34a';
            } catch (_) {
            }
        }

        refreshStatus();
        refreshFFT();
        $('saveErrThr').addEventListener('click', saveErrorThreshold);
        $('saveHarmThr').addEventListener('click', saveHarmThreshold);
        setInterval(refreshStatus, 1000);
        setInterval(refreshFFT, 600);
    </script>
</body>
</html>
)HTML";

    server.send(200, "text/html", page);
}

#if ENABLE_OLED
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

static void oledMsg(const char* line1, const char* line2) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 12, line1);
    if (line2) display.drawStr(0, 26, line2);
    display.sendBuffer();
}
#endif

static void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

#if ENABLE_OLED
    oledMsg("Laczenie Wi-Fi...", WIFI_SSID);
#endif

    Serial.printf("[WIFI] Connecting to SSID: %s\n", WIFI_SSID);
    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < CONFIG_WIFI_TIMEOUT_MS) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        const String ip = WiFi.localIP().toString();
        Serial.printf("[WIFI] Connected, IP: %s RSSI: %d dBm\n", ip.c_str(), WiFi.RSSI());
#if ENABLE_OLED
        oledMsg("Wi-Fi OK", ip.c_str());
#endif
    } else {
        Serial.printf("[WIFI] Connection timeout after %u ms, running offline\n", CONFIG_WIFI_TIMEOUT_MS);
#if ENABLE_OLED
        oledMsg("Wi-Fi timeout", "Tryb offline");
#endif
    }

    g_prevWifiStatus = WiFi.status();
}

void setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println("==== ESP32-C3 ADXL345 START ====");
    Serial.printf("[BOOT] Build date: %s %s\n", __DATE__, __TIME__);

#if ENABLE_OLED
    Wire.begin(CONFIG_SDA_PIN, CONFIG_SCL_PIN);
    Serial.printf("[I2C] SDA=%d SCL=%d\n", CONFIG_SDA_PIN, CONFIG_SCL_PIN);
    scanI2CBus();

    display.setI2CAddress(CONFIG_OLED_ADDR << 1);
    display.begin();
    display.setContrast(128);
    oledMsg("ESP32-C3 ADXL345", "Inicjalizacja...");
    Serial.println("[OLED] Ready");
#endif

#if ENABLE_ADXL
    if (!accel.begin()) {
        Serial.println("[ADXL] ERROR: sensor not found");
        g_sensorPresent = false;
#if ENABLE_OLED
        oledMsg("ADXL345 BLAD", "Sprawdz SPI");
#endif
        while (true) { delay(1000); }
    }

    g_sensorPresent = true;

    accel.setRange(ADXL345_RANGE_16_G);
    accel.setDataRate(ADXL345_DATARATE_800_HZ);

    Serial.printf("[ADXL] Initialized SPI: CS=%d SCK=%d MOSI=%d MISO=%d Fs=%dHz\n",
                  CONFIG_ADXL_SPI_CS_PIN,
                  CONFIG_ADXL_SPI_SCK_PIN,
                  CONFIG_ADXL_SPI_MOSI_PIN,
                  CONFIG_ADXL_SPI_MISO_PIN,
                  CONFIG_FFT_FS);
#endif

    connectWiFi();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatusJson);
    server.on("/api/fft", HTTP_GET, handleFftJson);
    server.on("/api/config", HTTP_GET, handleConfigJson);
    server.begin();

#if ENABLE_MODBUS
    modbusServer.begin();
    modbusServer.setNoDelay(true);
#endif

    Serial.println("[HTTP] Server started on port 80");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[HTTP] Open: http://%s/\n", WiFi.localIP().toString().c_str());
    }
#if ENABLE_MODBUS
    Serial.printf("[MODBUS] Server started on port %u unit=%u\n", CONFIG_MODBUS_PORT, CONFIG_MODBUS_UNIT_ID);
#endif

    logStatusSnapshot("setup-done");

#if ENABLE_OLED
    oledMsg("ADXL345 OK", "HTTP gotowy");
    delay(1200);
#endif
}

void loop() {
    server.handleClient();

#if ENABLE_MODBUS
    handleModbusTcp();
#endif

#if ENABLE_ADXL
    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - g_lastSampleUs) >= g_samplePeriodUs) {
        g_lastSampleUs += g_samplePeriodUs;

        sensors_event_t event;
        accel.getEvent(&event);

        g_lastX = event.acceleration.x;
        g_lastY = event.acceleration.y;
        g_lastZ = event.acceleration.z;
        g_lastMag = sqrtf(g_lastX * g_lastX + g_lastY * g_lastY + g_lastZ * g_lastZ);
        g_winSumSqX += g_lastX * g_lastX;
        g_winSumSqY += g_lastY * g_lastY;
        g_winSumSqZ += g_lastZ * g_lastZ;
        g_winSampleCount++;
        g_samples++;

        const float dt = 1.0f / (float)CONFIG_FFT_FS;
        const float rc = 1.0f / (2.0f * PI * CONFIG_HPF_CUTOFF_HZ);
        const float hpAlpha = rc / (rc + dt);
        const float hpOut = hpAlpha * (g_hpfPrevOut + g_lastMag - g_hpfPrevIn);
        g_hpfPrevIn = g_lastMag;
        g_hpfPrevOut = hpOut;

        g_fftReal[g_fftIdx] = hpOut;
        g_fftImag[g_fftIdx] = 0.0f;
        g_fftIdx++;
        if (g_fftIdx >= CONFIG_FFT_SIZE) {
            g_fftIdx = 0;
            if (g_winSampleCount > 0) {
                g_rmsX = sqrtf(g_winSumSqX / g_winSampleCount);
                g_rmsY = sqrtf(g_winSumSqY / g_winSampleCount);
                g_rmsZ = sqrtf(g_winSumSqZ / g_winSampleCount);
                g_rmsTotal = sqrtf(g_rmsX * g_rmsX + g_rmsY * g_rmsY + g_rmsZ * g_rmsZ);
            }
            g_winSumSqX = 0.0f;
            g_winSumSqY = 0.0f;
            g_winSumSqZ = 0.0f;
            g_winSampleCount = 0;
            computeFFT();
        }
    }
#endif

    const wl_status_t wifiStatus = WiFi.status();
    if (wifiStatus != g_prevWifiStatus) {
        Serial.printf("[WIFI] State change: %s -> %s\n", wifiStatusName(g_prevWifiStatus), wifiStatusName(wifiStatus));
        if (wifiStatus == WL_CONNECTED) {
            Serial.printf("[WIFI] Reconnected, IP=%s RSSI=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }
        g_prevWifiStatus = wifiStatus;
    }

    const unsigned long nowMs = millis();
    if (wifiStatus != WL_CONNECTED && (nowMs - g_lastWifiRetryMs) >= CONFIG_WIFI_RETRY_MS) {
        g_lastWifiRetryMs = nowMs;
        Serial.println("[WIFI] Retry requested");
        WiFi.reconnect();
    }

#if ENABLE_ADXL
    if ((nowMs - g_lastSensorLogMs) >= CONFIG_SENSOR_LOG_MS) {
        g_lastSensorLogMs = nowMs;
        Serial.printf("[SENSOR] X=%+6.2f Y=%+6.2f Z=%+6.2f V=%.2f m/s2 peak=%.1fHz\n",
                      g_lastX, g_lastY, g_lastZ, g_lastMag, g_fftPeakHz);
    }
#endif

    if ((nowMs - g_lastHeartbeatMs) >= CONFIG_HEARTBEAT_MS) {
        g_lastHeartbeatMs = nowMs;
        logStatusSnapshot("heartbeat");
    }

#if ENABLE_OLED
    if ((nowMs - g_lastOledMs) >= 120) {
        g_lastOledMs = nowMs;
        char buf[32];
        display.clearBuffer();
        display.setFont(u8g2_font_6x10_tf);
#if ENABLE_ADXL
    snprintf(buf, sizeof(buf), "X: %+6.2f", g_lastX); display.drawStr(0, 12, buf);
    snprintf(buf, sizeof(buf), "Y: %+6.2f", g_lastY); display.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "Z: %+6.2f", g_lastZ); display.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "RPM: %6.0f", g_fftPeakRpm); display.drawStr(0, 48, buf);
    if (g_harmonicAlarmFlags & 0x07u) {
        char hbuf[16] = "HARM:";
        if (g_harmonicAlarmFlags & 1u) strcat(hbuf, "2x");
        if (g_harmonicAlarmFlags & 2u) strcat(hbuf, "3x");
        if (g_harmonicAlarmFlags & 4u) strcat(hbuf, "4x");
        display.drawStr(0, 62, hbuf);
    } else {
        snprintf(buf, sizeof(buf), "CONF:%5.0f%%", g_fftPeakConfidencePct); display.drawStr(0, 62, buf);
    }
#else
        snprintf(buf, sizeof(buf), "heap: %u B", ESP.getFreeHeap());
        display.drawStr(0, 12, "ESP32-C3");
        display.drawStr(0, 28, buf);
#endif
        display.sendBuffer();
    }
#else
    delay(1);
#endif
}
