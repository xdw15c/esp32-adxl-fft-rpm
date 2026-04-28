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

WebServer server(80);

static unsigned long g_lastHeartbeatMs = 0;
static unsigned long g_lastWifiRetryMs = 0;
static wl_status_t g_prevWifiStatus = WL_NO_SHIELD;

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
static uint16_t g_fftIdx = 0;
static float g_hpfPrevIn = 0.0f;
static float g_hpfPrevOut = 0.0f;

static ArduinoFFT<float> g_fft(g_fftReal, g_fftImag, CONFIG_FFT_SIZE, (float)CONFIG_FFT_FS);

Adafruit_ADXL345_Unified accel(
    CONFIG_ADXL_SPI_SCK_PIN,
    CONFIG_ADXL_SPI_MISO_PIN,
    CONFIG_ADXL_SPI_MOSI_PIN,
    CONFIG_ADXL_SPI_CS_PIN,
    12345);

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
    uint16_t minBin = (uint16_t)(CONFIG_FFT_PEAK_MIN_HZ / binHz);
    uint16_t maxBin = (uint16_t)(CONFIG_FFT_PEAK_MAX_HZ / binHz);
    if (minBin < 1) minBin = 1;
    if (maxBin >= (halfN - 1)) maxBin = halfN - 1;
    const float norm = 2.0f / (float)CONFIG_FFT_SIZE;
    const float magAlpha = 0.22f;

    uint16_t peakBin = minBin;
    float peakAmp = 0.0f;
    g_fftMag[0] = 0.0f;

    for (uint16_t i = 1; i < halfN; i++) {
        const float magNow = g_fftReal[i] * norm;
        g_fftMag[i] = (1.0f - magAlpha) * g_fftMag[i] + magAlpha * magNow;
        if (i >= minBin && i <= maxBin && g_fftMag[i] > peakAmp) {
            peakAmp = g_fftMag[i];
            peakBin = i;
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
    g_fftPeakRpm = (g_fftPeakHzFilt * 60.0f) / CONFIG_RPM_ORDER;
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
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm samples=%lu vib=%.2f peak=%.1fHz rpm=%.0f\n",
                  tag, millis(), wifiStatusName(WiFi.status()), ip.c_str(), rssi, g_samples, g_lastMag, g_fftPeakHzFilt, g_fftPeakRpm);
#else
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm heap=%u\n",
                  tag, millis(), wifiStatusName(WiFi.status()), ip.c_str(), rssi, ESP.getFreeHeap());
#endif
}

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
    char json[448];
#if ENABLE_ADXL
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,"
             "\"samples\":%lu,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vibration\":%.3f,"
             "\"peak_hz\":%.2f,\"peak_amp\":%.3f,\"rpm\":%.1f}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             g_samples, g_lastX, g_lastY, g_lastZ, g_lastMag, g_fftPeakHzFilt, g_fftPeakAmp, g_fftPeakRpm);
#else
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,\"heap\":%u}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             ESP.getFreeHeap());
#endif
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
    resp += F(",\"rpm\":");
    resp += g_fftPeakRpm;
    resp += F(",\"rpm_order\":");
    resp += CONFIG_RPM_ORDER;
    resp += F(",\"band_min_hz\":");
    resp += CONFIG_FFT_PEAK_MIN_HZ;
    resp += F(",\"band_max_hz\":");
    resp += CONFIG_FFT_PEAK_MAX_HZ;
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
                $('rpm').textContent = (s.rpm || 0).toFixed(0);
            } catch (_) {
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
            } catch (_) {
            }
        }

        refreshStatus();
        refreshFFT();
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
#if ENABLE_OLED
        oledMsg("ADXL345 BLAD", "Sprawdz SPI");
#endif
        while (true) { delay(1000); }
    }

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
    server.begin();

    Serial.println("[HTTP] Server started on port 80");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[HTTP] Open: http://%s/\n", WiFi.localIP().toString().c_str());
    }

    logStatusSnapshot("setup-done");

#if ENABLE_OLED
    oledMsg("ADXL345 OK", "HTTP gotowy");
    delay(1200);
#endif
}

void loop() {
    server.handleClient();

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
        display.drawStr(0, 10, "Wibracje [m/s2]");
        snprintf(buf, sizeof(buf), "X: %+6.2f", g_lastX); display.drawStr(0, 24, buf);
        snprintf(buf, sizeof(buf), "Y: %+6.2f", g_lastY); display.drawStr(0, 36, buf);
        snprintf(buf, sizeof(buf), "Z: %+6.2f", g_lastZ); display.drawStr(0, 48, buf);
        snprintf(buf, sizeof(buf), "P: %6.1fHz", g_fftPeakHz); display.drawStr(0, 62, buf);
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
