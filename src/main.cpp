#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"

#if ENABLE_OLED || ENABLE_ADXL
#include <Wire.h>
#endif
#if ENABLE_ADXL
#include <SPI.h>
#include <Adafruit_ADXL345_U.h>
#endif
#if ENABLE_OLED
#include <U8g2lib.h>
#endif

// ─── Domyślne wartości (można nadpisać przez build_flags) ───────────────────
#ifndef ENABLE_OLED
#define ENABLE_OLED 0
#endif
#ifndef ENABLE_ADXL
#define ENABLE_ADXL 0
#endif
#ifndef CONFIG_SDA_PIN
#define CONFIG_SDA_PIN  8
#endif
#ifndef CONFIG_SCL_PIN
#define CONFIG_SCL_PIN  9
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

WebServer server(80);

#if ENABLE_ADXL
static float g_lastX = 0.0f;
static float g_lastY = 0.0f;
static float g_lastZ = 0.0f;
static float g_lastMag = 0.0f;
static unsigned long g_samples = 0;
static unsigned long g_lastSensorLogMs = 0;
#endif
static unsigned long g_lastHeartbeatMs = 0;
static unsigned long g_lastWifiRetryMs = 0;
static wl_status_t g_prevWifiStatus = WL_NO_SHIELD;

#if ENABLE_OLED
static void oledMsg(const char* line1, const char* line2 = nullptr);
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
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm samples=%lu vib=%.2f\n",
                  tag, millis(), wifiStatusName(WiFi.status()),
                  ip.c_str(), rssi, g_samples, g_lastMag);
#else
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm heap=%u\n",
                  tag, millis(), wifiStatusName(WiFi.status()),
                  ip.c_str(), rssi, ESP.getFreeHeap());
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
    char json[320];
#if ENABLE_ADXL
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,"
             "\"samples\":%lu,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vibration\":%.3f}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             g_samples, g_lastX, g_lastY, g_lastZ, g_lastMag);
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

static void handleRoot() {
        if (!ensureWebAuth()) return;

    Serial.printf("[HTTP] GET / from %s\n", server.client().remoteIP().toString().c_str());

        static const char page[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pl">
<head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32-C3 ADXL345 Status</title>
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
        h1 { margin: 0 0 12px; font-size: 1.3rem; }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 10px;
        }
        .label { color: #334155; font-size: 0.86rem; }
        .value { font-size: 1.06rem; font-weight: 650; margin-top: 4px; }
        .mono { font-family: Consolas, "Courier New", monospace; }
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
            <p class="label">Probek: <span class="mono" id="samples">-</span></p>
        </div>
    </div>

    <script>
        const ids = {
            wifi: document.getElementById('wifi'),
            ip: document.getElementById('ip'),
            rssi: document.getElementById('rssi'),
            uptime: document.getElementById('uptime'),
            x: document.getElementById('x'),
            y: document.getElementById('y'),
            z: document.getElementById('z'),
            v: document.getElementById('v'),
            samples: document.getElementById('samples')
        };

        async function refresh() {
            try {
                const res = await fetch('/api/status', { cache: 'no-store' });
                if (!res.ok) throw new Error('HTTP ' + res.status);
                const s = await res.json();
                ids.wifi.textContent = s.wifi_connected ? 'Polaczono' : 'Brak polaczenia';
                ids.ip.textContent = s.ip;
                ids.rssi.textContent = s.rssi + ' dBm';
                ids.uptime.textContent = s.uptime_ms;
                ids.x.textContent = s.x.toFixed(3);
                ids.y.textContent = s.y.toFixed(3);
                ids.z.textContent = s.z.toFixed(3);
                ids.v.textContent = s.vibration.toFixed(3);
                ids.samples.textContent = s.samples;
            } catch (_err) {
                ids.wifi.textContent = 'Blad odczytu';
            }
        }

        refresh();
        setInterval(refresh, 1000);
    </script>
</body>
</html>
)HTML";

        server.send(200, "text/html", page);
}

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
        String ip = WiFi.localIP().toString();
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

#if ENABLE_OLED
// ─── Wyświetlacz OLED SH1106 128×64 na HW I2C ────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

static void oledMsg(const char* line1, const char* line2) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 12, line1);
    if (line2) display.drawStr(0, 26, line2);
    display.sendBuffer();
}
#endif  // ENABLE_OLED

#if ENABLE_ADXL
// ─── Czujnik ADXL345 ─────────────────────────────────────────────────────────
Adafruit_ADXL345_Unified accel(
    CONFIG_ADXL_SPI_SCK_PIN,
    CONFIG_ADXL_SPI_MISO_PIN,
    CONFIG_ADXL_SPI_MOSI_PIN,
    CONFIG_ADXL_SPI_CS_PIN,
    12345);
#endif  // ENABLE_ADXL

void setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println("==== ESP32-C3 ADXL345 START ====");
    Serial.printf("[BOOT] Build date: %s %s\n", __DATE__, __TIME__);

#if ENABLE_OLED
    // I2C na GPIO8/9
    Wire.begin(CONFIG_SDA_PIN, CONFIG_SCL_PIN);
    Serial.printf("[I2C] SDA=%d SCL=%d\n", CONFIG_SDA_PIN, CONFIG_SCL_PIN);
    scanI2CBus();
#endif

#if ENABLE_OLED
    // ── OLED init ──────────────────────────────────────────────────────────
    display.setI2CAddress(CONFIG_OLED_ADDR << 1);  // U8g2 oczekuje adresu 8-bit
    display.begin();
    display.setContrast(128);
    oledMsg("ESP32-C3 ADXL345", "Inicjalizacja...");
    Serial.println("[OLED] Ready");
#endif

#if ENABLE_ADXL
    // ── ADXL345 init ───────────────────────────────────────────────────────
    if (!accel.begin()) {
        Serial.println("[ADXL] ERROR: sensor not found");
#if ENABLE_OLED
    oledMsg("ADXL345 BLAD", "Sprawdz SPI");
#endif
        while (true) { delay(1000); }
    }
    accel.setRange(ADXL345_RANGE_16_G);
    accel.setDataRate(ADXL345_DATARATE_800_HZ);
    Serial.printf("[ADXL] Initialized SPI: CS=%d SCK=%d MOSI=%d MISO=%d range=16G odr=800Hz\n",
          CONFIG_ADXL_SPI_CS_PIN,
          CONFIG_ADXL_SPI_SCK_PIN,
          CONFIG_ADXL_SPI_MOSI_PIN,
          CONFIG_ADXL_SPI_MISO_PIN);

    sensors_event_t firstEvent;
    accel.getEvent(&firstEvent);
    Serial.printf("[ADXL] First sample X=%.2f Y=%.2f Z=%.2f m/s2\n",
                  firstEvent.acceleration.x,
                  firstEvent.acceleration.y,
                  firstEvent.acceleration.z);
#endif

    connectWiFi();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatusJson);
    server.begin();
    Serial.println("[HTTP] Server started on port 80");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[HTTP] Open: http://%s/\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[HTTP] Device offline, web UI available after Wi-Fi reconnect");
    }
#if defined(WEB_USERNAME) && defined(WEB_PASSWORD)
    Serial.println("[HTTP] Basic Auth enabled");
#else
    Serial.println("[HTTP] Basic Auth disabled");
#endif

    logStatusSnapshot("setup-done");

#if ENABLE_OLED
    oledMsg("ADXL345 OK", "HTTP gotowy");
    delay(1500);
#endif
}

void loop() {
    server.handleClient();

#if ENABLE_ADXL
    sensors_event_t event;
    accel.getEvent(&event);

    g_lastX = event.acceleration.x;
    g_lastY = event.acceleration.y;
    g_lastZ = event.acceleration.z;
    g_lastMag = sqrtf(g_lastX * g_lastX + g_lastY * g_lastY + g_lastZ * g_lastZ);
    g_samples++;
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
        Serial.printf("[SENSOR] X=%+6.2f Y=%+6.2f Z=%+6.2f V=%.2f m/s2\n", g_lastX, g_lastY, g_lastZ, g_lastMag);
    }
#endif

    if ((nowMs - g_lastHeartbeatMs) >= CONFIG_HEARTBEAT_MS) {
        g_lastHeartbeatMs = nowMs;
        logStatusSnapshot("heartbeat");
    }

#if ENABLE_OLED
    char buf[32];
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
#if ENABLE_ADXL
    display.drawStr(0, 10, "Wibracje [m/s2]");
    snprintf(buf, sizeof(buf), "X: %+6.2f", g_lastX);  display.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "Y: %+6.2f", g_lastY);  display.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "Z: %+6.2f", g_lastZ);  display.drawStr(0, 48, buf);
    snprintf(buf, sizeof(buf), "V: %6.2f",  g_lastMag); display.drawStr(0, 62, buf);
#else
    snprintf(buf, sizeof(buf), "heap: %u B", ESP.getFreeHeap());
    display.drawStr(0, 12, "ESP32-C3");
    display.drawStr(0, 28, buf);
#endif  // ENABLE_ADXL
    display.sendBuffer();
    delay(200);
#else
    delay(2);
#endif  // ENABLE_OLED
}
