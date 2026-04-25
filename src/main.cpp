#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_ADXL345_U.h>
#include <U8g2lib.h>
#include "secrets.h"

// ─── Konfiguracja ────────────────────────────────────────────────────────────
#ifndef CONFIG_SDA_PIN
#define CONFIG_SDA_PIN  8
#endif
#ifndef CONFIG_SCL_PIN
#define CONFIG_SCL_PIN  9
#endif
#ifndef CONFIG_OLED_ADDR
#define CONFIG_OLED_ADDR 0x3C
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

// Inicjalizacja serwera WWW na porcie 80
WebServer server(80);

static float g_lastX = 0.0f;
static float g_lastY = 0.0f;
static float g_lastZ = 0.0f;
static float g_lastMag = 0.0f;
static unsigned long g_samples = 0;
static unsigned long g_lastHeartbeatMs = 0;
static unsigned long g_lastSensorLogMs = 0;
static unsigned long g_lastWifiRetryMs = 0;
static wl_status_t g_prevWifiStatus = WL_NO_SHIELD;

static void oledMsg(const char* line1, const char* line2 = nullptr);

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
    Serial.printf("[STATUS][%s] uptime=%lu ms wifi=%s ip=%s rssi=%d dBm samples=%lu vib=%.2f\n",
                  tag,
                  millis(),
                  wifiStatusName(WiFi.status()),
                  ip.c_str(),
                  rssi,
                  g_samples,
                  g_lastMag);
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

        char json[320];
        const bool wifiOk = (WiFi.status() == WL_CONNECTED);
        snprintf(json,
                         sizeof(json),
                         "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,\"samples\":%lu,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vibration\":%.3f}",
                         millis(),
                         wifiOk ? "true" : "false",
                         wifiOk ? WiFi.localIP().toString().c_str() : "-",
                         wifiOk ? WiFi.RSSI() : 0,
                         g_samples,
                         g_lastX,
                         g_lastY,
                         g_lastZ,
                         g_lastMag);
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
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        oledMsg("Laczenie Wi-Fi...", WIFI_SSID);
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
                oledMsg("Wi-Fi OK", ip.c_str());
        } else {
            Serial.printf("[WIFI] Connection timeout after %u ms, running offline\n", CONFIG_WIFI_TIMEOUT_MS);
                oledMsg("Wi-Fi timeout", "Tryb offline");
        }

        g_prevWifiStatus = WiFi.status();
}

// ─── Wyświetlacz OLED SH1106 128×64 na HW I2C ────────────────────────────────
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// ─── Czujnik ADXL345 ─────────────────────────────────────────────────────────
Adafruit_ADXL345_Unified accel(12345);

// ─── Pomocnicze: komunikat na OLED ───────────────────────────────────────────
static void oledMsg(const char* line1, const char* line2) {
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);
    display.drawStr(0, 12, line1);
    if (line2) display.drawStr(0, 26, line2);
    display.sendBuffer();
}

void setup() {
    Serial.begin(115200);
    delay(150);
    Serial.println();
    Serial.println("==== ESP32-C3 ADXL345 START ====");
    Serial.printf("[BOOT] Build date: %s %s\n", __DATE__, __TIME__);

    // I2C na GPIO8/9
    Wire.begin(CONFIG_SDA_PIN, CONFIG_SCL_PIN);
    Serial.printf("[I2C] SDA=%d SCL=%d OLED_ADDR=0x%02X\n", CONFIG_SDA_PIN, CONFIG_SCL_PIN, CONFIG_OLED_ADDR);
    scanI2CBus();

    // ── OLED init ──────────────────────────────────────────────────────────
    display.setI2CAddress(CONFIG_OLED_ADDR << 1);  // U8g2 oczekuje adresu 8-bit
    display.begin();
    display.setContrast(128);
    oledMsg("ESP32-C3 ADXL345", "Inicjalizacja...");
    Serial.println("[OLED] Ready");

    // ── ADXL345 init ───────────────────────────────────────────────────────
    if (!accel.begin()) {
        Serial.println("[ADXL] ERROR: sensor not found");
        oledMsg("ADXL345 BLAD", "Sprawdz I2C");
        while (true) { delay(1000); }
    }
    accel.setRange(ADXL345_RANGE_16_G);
    accel.setDataRate(ADXL345_DATARATE_800_HZ);
    Serial.println("[ADXL] Initialized: range=16G odr=800Hz");

    sensors_event_t event;
    accel.getEvent(&event);
    Serial.printf("[ADXL] First sample X=%.2f Y=%.2f Z=%.2f m/s2\n",
                  event.acceleration.x,
                  event.acceleration.y,
                  event.acceleration.z);

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

    oledMsg("ADXL345 OK", "HTTP gotowy");
    delay(1500);
}

void loop() {
    server.handleClient();

    sensors_event_t event;
    accel.getEvent(&event);

    g_lastX = event.acceleration.x;
    g_lastY = event.acceleration.y;
    g_lastZ = event.acceleration.z;
    g_lastMag = sqrtf(g_lastX * g_lastX + g_lastY * g_lastY + g_lastZ * g_lastZ);
    g_samples++;

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

    if ((nowMs - g_lastSensorLogMs) >= CONFIG_SENSOR_LOG_MS) {
        g_lastSensorLogMs = nowMs;
        Serial.printf("[SENSOR] X=%+6.2f Y=%+6.2f Z=%+6.2f V=%.2f m/s2\n", g_lastX, g_lastY, g_lastZ, g_lastMag);
    }

    if ((nowMs - g_lastHeartbeatMs) >= CONFIG_HEARTBEAT_MS) {
        g_lastHeartbeatMs = nowMs;
        logStatusSnapshot("heartbeat");
    }

    // ── Wyświetl na OLED ───────────────────────────────────────────────────
    char buf[32];
    display.clearBuffer();
    display.setFont(u8g2_font_6x10_tf);

    display.drawStr(0, 10, "Wibracje [m/s2]");
    snprintf(buf, sizeof(buf), "X: %+6.2f", g_lastX);     display.drawStr(0, 24, buf);
    snprintf(buf, sizeof(buf), "Y: %+6.2f", g_lastY);     display.drawStr(0, 36, buf);
    snprintf(buf, sizeof(buf), "Z: %+6.2f", g_lastZ);     display.drawStr(0, 48, buf);
    snprintf(buf, sizeof(buf), "V: %6.2f", g_lastMag);    display.drawStr(0, 62, buf);

    display.sendBuffer();

    delay(200);  // ~5 Hz dla celów testowych; docelowo: ciągły odczyt @ ODR
}
