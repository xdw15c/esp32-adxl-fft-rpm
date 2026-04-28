# ESP32 – Monitor wibracji silnika z FFT i estymacją RPM (ADXL345 SPI)

## Cel projektu

System monitorowania stanu technicznego silnika elektrycznego na podstawie analizy drgań. Czujnik ADXL345 (interfejs **SPI**) zamocowany na obudowie silnika dostarcza surowych danych w osiach X/Y/Z. ESP32 (WROOM) zbiera 256 próbek przy Fs = 800 Hz, wykonuje FFT z filtrem HP i wygładzaniem widma, wyznacza dominującą częstotliwość drgań oraz szacuje **RPM**. Wyniki są dostępne przez wbudowany serwer HTTP: strona statusu oraz endpoint JSON `/api/fft` z pełnym widmem i metadanymi. Opcjonalnie OLED 1,3" (SH1106, I²C) wyświetla X/Y/Z i wykryty pik Hz.

---

## Sprzęt

| Element | Model / parametry |
|---|---|
| Mikrokontroler | ESP32 WROOM (esp32dev) |
| Czujnik drgań | ADXL345 (±16 g, 13-bit, **SPI**) |
| Wyświetlacz | OLED 1,3" SH1106, 128×64 px, I²C (opcjonalnie) |
| Interfejs komunikacyjny | Wi-Fi 802.11 b/g/n + HTTP WebServer |
| Zasilanie | 3,3 V |

### Połączenie ADXL345 ↔ ESP32 (SPI)

| ADXL345 | ESP32 GPIO |
|---|---|
| VCC | 3V3 |
| GND | GND |
| CS | GPIO5 |
| SCK | GPIO18 |
| SDA (MOSI) | GPIO23 |
| SDO (MISO) | GPIO19 |

> Adafruit ADXL345 wymaga **SPI_MODE3** (nie MODE1 jak w upstream). Projekt zawiera lokalną poprawioną kopię biblioteki w `lib/Adafruit_ADXL345/`.

### Połączenie OLED 1,3" SH1106 ↔ ESP32 (I²C)

| OLED | ESP32 GPIO |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

Adres I²C: **0x3C** (konfigurowalny przez `CONFIG_OLED_ADDR`).

---

## Architektura oprogramowania

```
┌─────────────────────────────────────────────────────┐
│                    ESP32 (loop)                      │
│                                                      │
│  [Akwizycja – mikros()-based, Fs = 800 Hz]           │
│   - ADXL345 SPI → X/Y/Z raw                         │
│   - HP IIR filtr (fc ≈ 2 Hz, usuwa składową DC/tilt)│
│   - bufor kołowy 256 próbek                          │
│         │                                            │
│  [computeFFT() – po 256 próbkach]                    │
│   - okno Hamming                                     │
│   - arduinoFFT 2.x (256-pkt)                        │
│   - wygładzanie widma (EMA między ramkami)           │
│   - szukanie piku w paśmie 5–300 Hz                 │
│   - interpolacja paraboliczna (sub-bin)              │
│   - RPM = peak_hz × 60 / order                      │
│         │                                            │
│  [HTTP WebServer]                                    │
│   GET /           → strona statusu + wykres FFT      │
│   GET /api/status → JSON: X/Y/Z, IP, uptime, RPM    │
│   GET /api/fft    → JSON: mag[], peak_hz, rpm, …    │
│         │                                            │
│  [OLED – co 120 ms, non-blocking]                   │
│   - X / Y / Z [mg], P: peak_hz                      │
└─────────────────────────────────────────────────────┘
```

---

## API HTTP

### `GET /api/status`

```json
{
  "wifi_connected": true,
  "ip": "192.168.1.x",
  "uptime_ms": 12345,
  "acc_x": 12.3,
  "acc_y": -4.5,
  "acc_z": 1002.1,
  "peak_hz": 49.8,
  "rpm": 2988
}
```

### `GET /api/fft`

```json
{
  "mag": [0.0, 0.1, …],
  "peak_hz": 49.8,
  "peak_amp": 312.5,
  "rpm": 2988,
  "band_min_hz": 5,
  "band_max_hz": 300,
  "resolution": 3.125,
  "fs": 800,
  "n": 256
}
```

## Konfiguracja projektu (`platformio.ini`)

```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino

lib_deps =
    adafruit/Adafruit Unified Sensor
    adafruit/Adafruit BusIO
    olikraus/U8g2
    kosme/arduinoFFT          ; 2.x

build_flags =
    -D ENABLE_OLED=1
    -D ENABLE_ADXL=1
    ; I2C / OLED
    -D CONFIG_SDA_PIN=21
    -D CONFIG_SCL_PIN=22
    -D CONFIG_OLED_ADDR=0x3C
    ; ADXL345 SPI
    -D CONFIG_ADXL_SPI_CS_PIN=5
    -D CONFIG_ADXL_SPI_SCK_PIN=18
    -D CONFIG_ADXL_SPI_MOSI_PIN=23
    -D CONFIG_ADXL_SPI_MISO_PIN=19
    -D CONFIG_ADXL_ODR=800
    ; FFT
    -D CONFIG_FFT_SIZE=256
    -D CONFIG_FFT_FS=800
    ; Timery
    -D CONFIG_WIFI_TIMEOUT_MS=15000
```

Dane dostępowe Wi-Fi trzymaj w `include/secrets.h` (plik wykluczony z repozytorium).  
Szablon: `include/secrets.h.example`.

---

## Parametry czujnika ADXL345

| Parametr | Wartość / zakres |
|---|---|
| Zakres pomiarowy | ±16 g (ustawiony w firmware) |
| Rozdzielczość | 13 bit (full-resolution ON) |
| Output Data Rate (ODR) | 800 Hz (`CONFIG_ADXL_ODR`) |
| Interfejs | SPI @ 5 MHz, SPI_MODE3 |
| Napięcie zasilania | 3,3 V |

---

## Montaż mechaniczny czujnika

- Przykręcić / przykleić sztywno do obudowy silnika (okolice łożysk).
- Oś Z prostopadle do osi obrotu wału – najlepsza detekcja niewyważenia.
- Unikać montażu na elastycznych elementach – tłumią sygnał.
- SPI: przewód ekranowany ≤ 30 cm, linia CS z rezystorem szeregowym 33–100 Ω.

---

## Stan implementacji

| Funkcja | Status |
|---|---|
| ADXL345 SPI (MODE3, lokalna lib) | ✅ działa |
| OLED SH1106 I²C (U8g2) | ✅ działa |
| Próbkowanie 800 Hz (micros-based) | ✅ działa |
| FFT 256-pkt, Hamming window | ✅ działa |
| HP IIR filtr (dc/tilt rejection) | ✅ działa |
| Wygładzanie widma (EMA) | ✅ działa |
| Interpolacja paraboliczna piku | ✅ działa |
| Estymacja RPM | ✅ działa |
| Web UI z wykresem canvas (log) | ✅ działa |
| `/api/status` + `/api/fft` | ✅ działa |
| Modbus TCP | ❌ nie zaimplementowano |

---

## Literatura i źródła

- ADXL345 Datasheet – Analog Devices: https://cdn-shop.adafruit.com/datasheets/ADXL345.pdf
- S. R. Pandit et al., *Vibration-Based Motor Health Monitoring System Using ESP32 and ADXL345*, IJPREMS Vol. 05 Issue 04, April 2025, pp. 3550-3552
- Espressif Systems, *ESP32 Technical Reference Manual*
- kosme/arduinoFFT: https://github.com/kosme/arduinoFFT
- ISO 10816 – Mechanical vibration – Evaluation of machine vibration
