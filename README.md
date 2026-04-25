# ESP32-C3 SuperMini – Monitor wibracji silnika (ADXL345 + Modbus TCP)

## Cel projektu

System monitorowania stanu technicznego silnika elektrycznego na podstawie analizy drgań. Czujnik ADXL345 zamocowany bezpośrednio na obudowie silnika dostarcza surowych danych akcelerometrycznych w trzech osiach (X, Y, Z). Mikrokontroler ESP32-C3 SuperMini przetwarza dane i udostępnia je przez protokół **Modbus TCP** na porcie **502**, umożliwiając integrację z dowolnym systemem SCADA / HMI. Opcjonalnie dane są buforowane i analizowane pod kątem anomalii wibracyjnych.

---

## Sprzęt

| Element | Model / parametry |
|---|---|
| Mikrokontroler | ESP32-C3 SuperMini |
| Czujnik drgań | ADXL345 (±16 g, 13-bit, I²C / SPI) |
| Wyświetlacz | OLED 1,3" SH1106 / SSD1306, 128×64 px, I²C |
| Interfejs komunikacyjny | Wi-Fi 802.11 b/g/n (wbudowany) |
| Zasilanie | 3,3 V (z pinu VCC ESP32-C3) lub 5 V przez regulator |

### Połączenie ADXL345 ↔ ESP32-C3 SuperMini (I²C)

| ADXL345 | ESP32-C3 SuperMini |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 |
| SCL | GPIO9 |
| SDO/ALT | GND → adres I²C: 0x53 |
| CS | 3V3 (tryb I²C) |

> Piny SDA/SCL można zmienić w `platformio.ini` przez `build_flags`.

### Połączenie OLED 1,3" ↔ ESP32-C3 SuperMini (I²C – wspólna magistrala z ADXL345)

| OLED | ESP32-C3 SuperMini |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO8 (wspólnie z ADXL345) |
| SCL | GPIO9 (wspólnie z ADXL345) |

Typowe adresy I²C: SH1106 → **0x3C**, SSD1306 → **0x3C** (lub 0x3D gdy SA0=VCC).  
ADXL345 używa adresu 0x53 – brak kolizji na magistrali.

> Wyświetlacze 1,3" najczęściej stosują kontroler **SH1106** (nie SSD1306). Użyj biblioteki `U8g2` lub `U8x8` – obsługuje oba kontrolery tym samym API.

---

## Architektura oprogramowania

```
┌─────────────────────────────────────────────────┐
│                  ESP32-C3                        │
│                                                  │
│  [ADXL345 Task]                                  │
│   - pomiar X,Y,Z @ 800 Hz (ODR)                 │
│   - obliczenie RMS per okno (np. 256 próbek)    │
│   - zapis do ring-buffera                        │
│         │                                        │
│  [Analiza Task]                                  │
│   - wykrywanie anomalii                          │
│     • próg RMS (statyczny)                       │
│     • FFT – dominujące częstotliwości            │
│     • detekcja trendu (ruchoma średnia)          │
│   - zapis flagi alarmowej                        │
│         │                                        │
│  [Modbus TCP Task]                               │
│   - serwer na porcie 502                         │
│   - rejestr holdingowy / input registers         │
│   - udostępnia: X,Y,Z raw, RMS, alarm, itp.     │
│         │                                        │
│  [OLED Display Task]                             │
│   - odświeżanie co ~500 ms                       │
│   - ekran 1: RMS_TOTAL, status, IP               │
│   - ekran 2: X/Y/Z [mg], ODR                     │
│   - ekran 3: aktywne alarmy (ikonki / tekst)     │
│   - animowany wskaźnik przy alarmie              │
└─────────────────────────────────────────────────┘
```

---

## Mapa rejestrów Modbus TCP

Typ rejestrów: **Input Registers (FC 04)**, base address = 0.

| Adres | Nazwa | Format | Opis |
|---|---|---|---|
| 0 | ACC_X_RAW | INT16 | Surowy odczyt osi X [LSB] |
| 1 | ACC_Y_RAW | INT16 | Surowy odczyt osi Y [LSB] |
| 2 | ACC_Z_RAW | INT16 | Surowy odczyt osi Z [LSB] |
| 3 | ACC_X_MG | INT16 | Przyspieszenie X [mg] |
| 4 | ACC_Y_MG | INT16 | Przyspieszenie Y [mg] |
| 5 | ACC_Z_MG | INT16 | Przyspieszenie Z [mg] |
| 6 | RMS_X | UINT16 | RMS osi X × 10 [mg] |
| 7 | RMS_Y | UINT16 | RMS osi Y × 10 [mg] |
| 8 | RMS_Z | UINT16 | RMS osi Z × 10 [mg] |
| 9 | RMS_TOTAL | UINT16 | √(RMS_X²+RMS_Y²+RMS_Z²) × 10 |
| 10 | ALARM_FLAGS | UINT16 | Bity alarmów (patrz niżej) |
| 11 | TEMP_DEG10 | INT16 | Temperatura × 10 [°C] (ADXL345 wewn.) |
| 12 | FFT_PEAK_HZ | UINT16 | Dominująca częstotliwość [Hz] |
| 13 | FFT_PEAK_AMP | UINT16 | Amplituda dominującej składowej × 10 |
| 14 | SAMPLE_RATE | UINT16 | Aktualna częstotliwość próbkowania [Hz] |
| 15 | STATUS | UINT16 | Status urządzenia (0=OK, 1=FAULT) |

### Bity rejestru ALARM_FLAGS (rejestr 10)

| Bit | Znaczenie |
|---|---|
| 0 | Przekroczenie progu RMS (statyczny) |
| 1 | Wykryto impuls udarowy (krótkotrwały spike > N×σ) |
| 2 | Wzrost trendu wibracji > 20% w ciągu 60 s |
| 3 | Częstotliwość dominująca poza zakresem nominalnym |
| 4 | Utrata komunikacji z ADXL345 |
| 5–15 | Zarezerwowane |

> Rejestr **Holding Registers (FC 03 / FC 06)** – do konfiguracji progów alarmowych w runtime (adres 100–109).

---

## Detekcja anomalii

### 1. Próg RMS (statyczny)
Użytkownik konfiguruje wartości progowe przez rejestry Modbus HR.  
Alarm gdy: `RMS_TOTAL > próg_wysoki`

Wzór na magnitudę drgań (Euclidean norm, za IJPREMS 2025):

$$V = \sqrt{x^2 + y^2 + z^2}$$

Jednostka: m/s². Domyślny próg startowy: **11,0 m/s²** (wyznaczony eksperymentalnie w cytowanym artykule). Wartość konfigurowalna przez rejestr Modbus HR 100.

### 2. Detekcja skoków (spike detection)
Próbka uznawana za impuls gdy: `|a_i| > μ + N·σ`  
gdzie μ i σ są obliczane w ruchomym oknie (np. 1024 próbki).  
Parametr N (domyślnie 5) konfigurowalny przez Modbus.

### 3. Analiza trendu
Ruchoma średnia RMS w oknie 60 s porównywana z wartością bazową zmierzoną po uruchomieniu (baseline). Alarm gdy wzrost przekroczy 20%.

### 4. Analiza FFT (opcjonalna, faza 2)
FFT na oknie 256 próbek (Hann window). Wykrywanie:
- harmoniczna częstotliwości obrotu (1×, 2×, 3× RPM/60)
- charakterystyczne częstotliwości uszkodzenia łożysk (BPFI, BPFO, BSF)

---

## Parametry czujnika ADXL345

| Parametr | Wartość / zakres |
|---|---|
| Zakres pomiarowy | ±2 / ±4 / ±8 / ±16 g |
| Rozdzielczość | 10–13 bit (full-resolution ON) |
| Output Data Rate (ODR) | 0,1 – 3200 Hz (domyślnie: 800 Hz) |
| Interfejs | I²C (do 400 kHz) / SPI (do 5 MHz) |
| Napięcie zasilania | 2,0 – 3,6 V |
| Pobór prądu (pomiar) | ~140 µA |

---

## Konfiguracja projektu (platformio.ini)

```ini
[env:esp32-c3-devkitm-1]
platform  = espressif32
board     = esp32-c3-devkitm-1
framework = arduino

lib_deps =
    sparkfun/SparkFun ADXL345 Arduino Library  ; lub adafruit/Adafruit ADXL345
    ; modbus tcp – np. eModbus (Andre Tromp)
    https://github.com/eModbus/eModbus.git
    ; OLED – obsługuje SH1106 i SSD1306 tym samym API
    olikraus/U8g2

build_flags =
    -D CONFIG_SDA_PIN=8
    -D CONFIG_SCL_PIN=9
    -D CONFIG_ADXL_ODR=800
    -D CONFIG_RMS_WINDOW=256
    -D CONFIG_MODBUS_PORT=502
    -D CONFIG_OLED_ADDR=0x3C
    -D CONFIG_WIFI_SSID=\"YourSSID\"
    -D CONFIG_WIFI_PASS=\"YourPassword\"
```

---

## Montaż mechaniczny czujnika

- Czujnik należy przykręcić / przykleić sztywno do obudowy silnika (najlepiej w okolicach łożysk).
- Oś Z prostopadle do osi obrotu wału – umożliwia najlepszą detekcję niewyważenia.
- Unikać montażu na elastycznych elementach (guma, folia) – tłumią sygnał.
- Ekranowany przewód I²C maks. 30 cm lub przejście na SPI przy dłuższych połączeniach.

---

## Plan realizacji

- [ ] **Faza 0** – konfiguracja środowiska PlatformIO, test komunikacji I²C z ADXL345
- [ ] **Faza 1** – ciągły odczyt X/Y/Z, serwer Modbus TCP, podstawowe rejestry raw + RMS
- [ ] **Faza 2** – ring-buffer, detekcja skoków, analiza trendu, bity alarmów
- [ ] **Faza 3** – wyświetlacz OLED 1,3" (U8g2): ekrany statusu, RMS, alarmy
- [ ] **Faza 4** – FFT, wykrywanie charakterystycznych częstotliwości uszkodzeń
- [ ] **Faza 5** – konfiguracja progów przez Modbus HR, persystencja w NVS/Flash

### Plan na przyszłość: wdrożenie FFT (arduinoFFT)

W projekcie planowane jest użycie biblioteki **arduinoFFT** do analizy widmowej drgań, ale dopiero po spełnieniu warunków akwizycji danych. Celem jest uzyskanie wiarygodnej informacji o częstotliwości dominującej i jej amplitudzie, bez przeciążania mikrokontrolera ESP32-C3.

#### Etap A – stabilne próbkowanie (warunek konieczny)

- Zapewnić stałą częstotliwość próbkowania (`Fs`) niezależną od odświeżania OLED i obsługi HTTP.
- Odejść od opóźnień blokujących w pętli głównej jako mechanizmu akwizycji.
- Zbierać próbki do ring-buffera i znacznika czasu, tak aby możliwa była kontrola jitteru.

**Kryterium zakończenia etapu:** stabilne `Fs` (np. 800 Hz) i kompletne okna próbek bez gubienia danych.

#### Etap B – FFT minimalne (1 oś, 1 metryka)

- Uruchomić FFT najpierw dla jednej osi (rekomendowana oś Z).
- Okno analizy: `N=256` (lub `N=512` po testach wydajności).
- Zastosować okno Hann przed obliczeniem widma.
- Wyznaczać `FFT_PEAK_HZ` oraz `FFT_PEAK_AMP` na podstawie maksimum w paśmie roboczym.

**Kryterium zakończenia etapu:** poprawne i powtarzalne wskazanie piku częstotliwości dla sygnału testowego.

#### Etap C – integracja z Modbus i alarmami

- Publikować wyniki FFT w rejestrach Modbus (adresy 12 i 13).
- Dodać logikę alarmu dla częstotliwości dominującej poza zakresem nominalnym.
- Powiązać alarm FFT z istniejącym `ALARM_FLAGS` (bit 3).

**Kryterium zakończenia etapu:** stabilny odczyt FFT z poziomu SCADA/HMI oraz poprawne ustawianie flag alarmowych.

#### Etap D – rozszerzenie analizy

- Rozszerzyć analizę na trzy osie i/lub magnitudę wektorową.
- Dodać obserwację harmonicznych 1×/2×/3× częstotliwości obrotowej.
- Rozważyć wyznaczanie energii w pasmach diagnostycznych (pod łożyska i niewyważenie).

**Kryterium zakończenia etapu:** wykrywanie trendów i zmian charakterystyki widma w dłuższym horyzoncie czasu.

#### Uwagi licencyjne

Biblioteka `arduinoFFT` jest udostępniana na licencji **GPL-3.0**. Dla wdrożeń zamkniętych/komercyjnych należy wcześniej potwierdzić zgodność licencyjną albo rozważyć alternatywę o bardziej liberalnej licencji.

---

## Różnice względem artykułu referencyjnego (IJPREMS 2025)

Artykuł *Vibration-Based Motor Health Monitoring System Using ESP32 and ADXL345*  
(S. R. Pandit, K. Patil, V. Vaidya – MVPS's KBTCOE Nashik, Vol. 05 Issue 04, pp. 3550-3552)
stosuje podejście uproszczone:

| Cecha | Artykuł IJPREMS | Nasz projekt |
|---|---|---|
| Mikrokontroler | ESP32 (standard) | ESP32-C3 SuperMini |
| Interfejs | I²C GPIO21/27 | I²C GPIO8/9 |
| Pętla pomiarowa | 1 s (polling) | 800 Hz ciągły ODR |
| Alarm | Telegram Bot | Modbus TCP (rejestr bitowy) |
| Analiza | Próg statyczny 11 m/s² | Próg + spike + trend + FFT |
| Buforowanie | Brak | Ring-buffer (256–1024 próbek) |
| Konfiguracja runtime | Brak | Rejestry Modbus HR |
| Wyświetlacz lokalny | Brak | OLED 1,3" 128×64 (SH1106/SSD1306) |

Wzór na magnitudę z artykułu (przyjęty w projekcie): `V = √(x²+y²+z²)` [m/s²]

---

## Literatura i źródła

- ADXL345 Datasheet – Analog Devices: https://cdn-shop.adafruit.com/datasheets/ADXL345.pdf
- S. R. Pandit et al., *Vibration-Based Motor Health Monitoring System Using ESP32 and ADXL345*, IJPREMS Vol. 05 Issue 04, April 2025, pp. 3550-3552
- Espressif Systems, *ESP32 Technical Reference Manual*: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
- ISO 10816 – Mechanical vibration – Evaluation of machine vibration
- eModbus library: https://github.com/eModbus/eModbus
- https://github.com/kosme/arduinoFFT - przeanalizować czy FFT może być zastosowana w tym projecie
