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
#define CONFIG_FFT_SIZE 4096
#endif
#ifndef CONFIG_FFT_N_DEFAULT
#define CONFIG_FFT_N_DEFAULT 512
#endif
#ifndef CONFIG_FFT_FS
#define CONFIG_FFT_FS 400
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
#ifndef CONFIG_HARM_WINDOW_BINS
#define CONFIG_HARM_WINDOW_BINS 2
#endif
#ifndef CONFIG_TREND_WINDOW_SEC_DEFAULT
#define CONFIG_TREND_WINDOW_SEC_DEFAULT 30
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
static float g_manualRefRpm = 0.0f;
static uint16_t g_alarmFlags = 0;
static bool g_sensorPresent = ENABLE_ADXL != 0;
static uint8_t g_trendWindowSec = CONFIG_TREND_WINDOW_SEC_DEFAULT;
static float g_cpuLoadPct = 0.0f;
static uint32_t g_cpuProcAccUs = 0;
static uint32_t g_cpuBudgetAccUs = 0;
static uint32_t g_cpuWindowStartUs = 0;

#if ENABLE_OLED
static unsigned long g_lastOledMs = 0;
static void oledMsg(const char* line1, const char* line2 = nullptr);
#endif

#if ENABLE_ADXL
enum {
    FFT_AXIS_X = 0,
    FFT_AXIS_Y = 1,
    FFT_AXIS_Z = 2,
    FFT_AXIS_RESULTANT = 3,
    FFT_AXIS_COUNT = 4
};

static const uint8_t FFT_ANALYTICS_X = 1u << FFT_AXIS_X;
static const uint8_t FFT_ANALYTICS_Y = 1u << FFT_AXIS_Y;
static const uint8_t FFT_ANALYTICS_Z = 1u << FFT_AXIS_Z;
static const uint8_t FFT_ANALYTICS_RESULTANT = 1u << FFT_AXIS_RESULTANT;
static const uint8_t FFT_ANALYTICS_ALL = FFT_ANALYTICS_X | FFT_ANALYTICS_Y | FFT_ANALYTICS_Z | FFT_ANALYTICS_RESULTANT;

static float g_lastX = 0.0f;
static float g_lastY = 0.0f;
static float g_lastZ = 0.0f;
static float g_lastMag = 0.0f;
static unsigned long g_samples = 0;
static unsigned long g_lastSensorLogMs = 0;

static uint32_t g_lastSampleUs = 0;
static uint32_t g_samplePeriodUs = 1000000UL / CONFIG_FFT_FS;
static uint16_t g_actualFs = CONFIG_FFT_FS;
static uint16_t g_fftNActive = (CONFIG_FFT_N_DEFAULT <= CONFIG_FFT_SIZE) ? CONFIG_FFT_N_DEFAULT : CONFIG_FFT_SIZE;
static uint8_t g_fftAnalyticsMask = FFT_ANALYTICS_Z;
static uint8_t g_fftPrimaryAxis = FFT_AXIS_Z;

static const float FFT_INPUT_SCALE = 512.0f;
static const float FFT_INPUT_INV_SCALE = 1.0f / FFT_INPUT_SCALE;
static int16_t g_fftInX[CONFIG_FFT_SIZE];
static int16_t g_fftInY[CONFIG_FFT_SIZE];
static int16_t g_fftInZ[CONFIG_FFT_SIZE];
static int16_t g_fftInResultant[CONFIG_FFT_SIZE];
static float g_fftWorkReal[CONFIG_FFT_SIZE];
static float g_fftWorkImag[CONFIG_FFT_SIZE];
static float g_fftMag[CONFIG_FFT_SIZE / 2];
static float g_fftAxisPeakHz[FFT_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float g_fftAxisPeakAmp[FFT_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float g_fftPeakHz = 0.0f;
static float g_fftPeakHzFilt = 0.0f;
static float g_fftPeakAmp = 0.0f;
static float g_fftPeakRpm = 0.0f;
static float g_fftPeakConfidencePct = 0.0f;
static float g_rmsX = 0.0f;
static float g_rmsY = 0.0f;
static float g_rmsZ = 0.0f;
static float g_rmsTotal = 0.0f;
static float g_winSumX = 0.0f;
static float g_winSumY = 0.0f;
static float g_winSumZ = 0.0f;
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
static uint8_t g_harmWindowBins = CONFIG_HARM_WINDOW_BINS;
static float g_prevFundHz = 0.0f;

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

static float getManualRefHz() {
    if (g_manualRefRpm <= 0.0f) return 0.0f;
    return (g_manualRefRpm * max<uint16_t>(1, g_rpmOrder)) / 60.0f;
}

static bool isValidFftN(uint16_t n) {
    if (n < 64 || n > CONFIG_FFT_SIZE) return false;
    return (n & (n - 1)) == 0;
}

static uint8_t getFftNOptions(uint16_t* out, uint8_t maxOut) {
    uint8_t c = 0;
    for (uint16_t n = 64; n <= CONFIG_FFT_SIZE && c < maxOut; n <<= 1) {
        out[c++] = n;
    }
    if (c == 0 && maxOut > 0) {
        out[c++] = CONFIG_FFT_SIZE;
    }
    return c;
}

static void resetAnalysisState() {
    g_lastSampleUs = micros();
    g_fftIdx = 0;
    g_hpfPrevIn = 0.0f;
    g_hpfPrevOut = 0.0f;
    g_winSumSqX = 0.0f;
    g_winSumSqY = 0.0f;
    g_winSumSqZ = 0.0f;
    g_winSumX = 0.0f;
    g_winSumY = 0.0f;
    g_winSumZ = 0.0f;
    g_winSampleCount = 0;
    g_rmsX = 0.0f;
    g_rmsY = 0.0f;
    g_rmsZ = 0.0f;
    g_rmsTotal = 0.0f;
    g_fftPeakHz = 0.0f;
    g_fftPeakHzFilt = 0.0f;
    g_fftPeakAmp = 0.0f;
    g_fftPeakRpm = 0.0f;
    g_fftPeakConfidencePct = 0.0f;
    g_prevFundHz = 0.0f;
    g_harmonicAmp[0] = 0.0f;
    g_harmonicAmp[1] = 0.0f;
    g_harmonicAmp[2] = 0.0f;
    g_harmonicAlarmFlags = 0;
    memset(g_fftInX, 0, sizeof(g_fftInX));
    memset(g_fftInY, 0, sizeof(g_fftInY));
    memset(g_fftInZ, 0, sizeof(g_fftInZ));
    memset(g_fftInResultant, 0, sizeof(g_fftInResultant));
    memset(g_fftWorkReal, 0, sizeof(g_fftWorkReal));
    memset(g_fftWorkImag, 0, sizeof(g_fftWorkImag));
    memset(g_fftMag, 0, sizeof(g_fftMag));
    memset(g_fftAxisPeakHz, 0, sizeof(g_fftAxisPeakHz));
    memset(g_fftAxisPeakAmp, 0, sizeof(g_fftAxisPeakAmp));
}

static void applyFftN(uint16_t n) {
    if (!isValidFftN(n)) return;
    if (n == g_fftNActive) return;
    g_fftNActive = n;
    resetAnalysisState();
    Serial.printf("[CFG] FFT_N=%u\n", g_fftNActive);
}

// Maps an ODR value [Hz] to ADXL345 enum and applies it at runtime.
// Also resets HPF state and clears FFT buffers to avoid transient artefacts.
static void applyOdr(uint16_t hz) {
    dataRate_t dr;
    uint16_t actualHz;
    if      (hz >= 800) { dr = ADXL345_DATARATE_800_HZ; actualHz = 800; }
    else if (hz >= 400) { dr = ADXL345_DATARATE_400_HZ; actualHz = 400; }
    else if (hz >= 200) { dr = ADXL345_DATARATE_200_HZ; actualHz = 200; }
    else if (hz >= 100) { dr = ADXL345_DATARATE_100_HZ; actualHz = 100; }
    else                { dr = ADXL345_DATARATE_50_HZ;  actualHz = 50;  }
    if (actualHz == g_actualFs) return;
    accel.setDataRate(dr);
    g_actualFs = actualHz;
    g_samplePeriodUs = 1000000UL / actualHz;
    // Reset analysis state because previous samples were captured with different timing.
    resetAnalysisState();
    Serial.printf("[CFG] ODR=%u Hz\n", actualHz);
}

// Searches g_fftMag[] for energy at 2x, 3x, 4x of fundHz relative to the fundamental peak.
// Sets g_harmonicAmp[0..2] and g_harmonicAlarmFlags bits.
static void computeHarmonics(float fundHz) {
    const float binHz = (float)g_actualFs / (float)g_fftNActive;
    const uint16_t halfN = g_fftNActive / 2;
    const uint16_t halfWin = g_harmWindowBins;
    g_harmonicAlarmFlags = 0;
    uint8_t harmCount = 0;
    for (uint8_t k = 2; k <= g_harmMaxOrder && k <= 4; k++) {
        const uint8_t idx = k - 2;  // 2x->0, 3x->1, 4x->2
        const float targetHz = (float)k * fundHz;
        if (targetHz >= (float)(g_actualFs / 2)) {
            g_harmonicAmp[idx] = 0.0f;
            continue;
        }
        const uint16_t centerBin = (uint16_t)(targetHz / binHz);
        const uint16_t lo = (centerBin > halfWin) ? (centerBin - halfWin) : 1;
        const uint16_t hi = min((uint16_t)(centerBin + halfWin), (uint16_t)(halfN - 1));
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

static uint8_t resolveFftPrimaryAxis() {
    if ((g_fftAnalyticsMask & FFT_ANALYTICS_ALL) == 0u) {
        return FFT_AXIS_Z;
    }
    if ((g_fftPrimaryAxis < FFT_AXIS_COUNT) && (g_fftAnalyticsMask & (1u << g_fftPrimaryAxis))) {
        return g_fftPrimaryAxis;
    }
    for (uint8_t axis = 0; axis < FFT_AXIS_COUNT; axis++) {
        if (g_fftAnalyticsMask & (1u << axis)) {
            return axis;
        }
    }
    return FFT_AXIS_Z;
}

static void computeMagnitudeForAxis(const int16_t* input, float* magOut, uint16_t nActive) {
    float mean = 0.0f;
    for (uint16_t i = 0; i < nActive; i++) {
        mean += (float)input[i] * FFT_INPUT_INV_SCALE;
    }
    mean /= nActive;

    for (uint16_t i = 0; i < nActive; i++) {
        g_fftWorkReal[i] = ((float)input[i] * FFT_INPUT_INV_SCALE) - mean;
        g_fftWorkImag[i] = 0.0f;
    }

    ArduinoFFT<float> fft(g_fftWorkReal, g_fftWorkImag, nActive, (float)g_actualFs);
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    const uint16_t halfN = nActive / 2;
    const float norm = 2.0f / (float)nActive;
    const float magAlpha = 0.22f;
    magOut[0] = 0.0f;
    for (uint16_t i = 1; i < halfN; i++) {
        const float magNow = g_fftWorkReal[i] * norm;
        magOut[i] = (1.0f - magAlpha) * magOut[i] + magAlpha * magNow;
    }
}

static void updateAxisPeakFromMag(uint8_t axis, const float* mag, uint16_t nActive, uint16_t minBin, uint16_t maxBin, float binHz) {
    float peakAmp = 0.0f;
    uint16_t peakBin = minBin;
    for (uint16_t i = minBin; i <= maxBin; i++) {
        if (mag[i] > peakAmp) {
            peakAmp = mag[i];
            peakBin = i;
        }
    }
    g_fftAxisPeakAmp[axis] = peakAmp;
    g_fftAxisPeakHz[axis] = peakBin * binHz;
}

static void computeAxisPeakOnly(uint8_t axis, const int16_t* input, uint16_t nActive, uint16_t minBin, uint16_t maxBin, float binHz) {
    float mean = 0.0f;
    for (uint16_t i = 0; i < nActive; i++) {
        mean += (float)input[i] * FFT_INPUT_INV_SCALE;
    }
    mean /= nActive;

    for (uint16_t i = 0; i < nActive; i++) {
        g_fftWorkReal[i] = ((float)input[i] * FFT_INPUT_INV_SCALE) - mean;
        g_fftWorkImag[i] = 0.0f;
    }

    ArduinoFFT<float> fft(g_fftWorkReal, g_fftWorkImag, nActive, (float)g_actualFs);
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();

    const float norm = 2.0f / (float)nActive;
    float peakAmp = 0.0f;
    uint16_t peakBin = minBin;
    for (uint16_t i = minBin; i <= maxBin; i++) {
        const float magNow = g_fftWorkReal[i] * norm;
        if (magNow > peakAmp) {
            peakAmp = magNow;
            peakBin = i;
        }
    }
    g_fftAxisPeakAmp[axis] = peakAmp;
    g_fftAxisPeakHz[axis] = peakBin * binHz;
}

static void computeFFT() {
    if (!isValidFftN(g_fftNActive)) {
        g_fftNActive = CONFIG_FFT_SIZE;
    }
    const uint16_t nActive = g_fftNActive;
    const uint16_t halfN = nActive / 2;
    const float binHz = (float)g_actualFs / (float)nActive;

    uint16_t minBin = (uint16_t)((float)g_fftBandMinHz / binHz);
    uint16_t maxBin = (uint16_t)((float)g_fftBandMaxHz / binHz);
    if (minBin < 1) minBin = 1;
    if (maxBin >= (halfN - 1)) maxBin = halfN - 1;

    const uint8_t mask = g_fftAnalyticsMask & FFT_ANALYTICS_ALL;
    if (mask == 0u) {
        memset(g_fftMag, 0, sizeof(g_fftMag));
        g_fftAxisPeakHz[FFT_AXIS_X] = 0.0f;
        g_fftAxisPeakHz[FFT_AXIS_Y] = 0.0f;
        g_fftAxisPeakHz[FFT_AXIS_Z] = 0.0f;
        g_fftAxisPeakHz[FFT_AXIS_RESULTANT] = 0.0f;
        g_fftAxisPeakAmp[FFT_AXIS_X] = 0.0f;
        g_fftAxisPeakAmp[FFT_AXIS_Y] = 0.0f;
        g_fftAxisPeakAmp[FFT_AXIS_Z] = 0.0f;
        g_fftAxisPeakAmp[FFT_AXIS_RESULTANT] = 0.0f;
        g_fftPeakHz = 0.0f;
        g_fftPeakAmp = 0.0f;
        g_fftPeakHzFilt = 0.0f;
        g_fftPeakConfidencePct = 0.0f;
        g_fftPeakRpm = 0.0f;
        g_prevFundHz = 0.0f;
        g_harmonicAlarmFlags = 0;
        g_harmonicAmp[0] = 0.0f;
        g_harmonicAmp[1] = 0.0f;
        g_harmonicAmp[2] = 0.0f;
        updateAlarmFlags();
        return;
    }

    const uint8_t activeAxis = resolveFftPrimaryAxis();
    g_fftPrimaryAxis = activeAxis;

    auto processAxis = [&](uint8_t axis, const int16_t* input) {
        if (!(mask & (1u << axis))) {
            g_fftAxisPeakHz[axis] = 0.0f;
            g_fftAxisPeakAmp[axis] = 0.0f;
            return;
        }

        if (axis == activeAxis) {
            computeMagnitudeForAxis(input, g_fftMag, nActive);
            updateAxisPeakFromMag(axis, g_fftMag, nActive, minBin, maxBin, binHz);
        } else {
            computeAxisPeakOnly(axis, input, nActive, minBin, maxBin, binHz);
        }
    };

    processAxis(FFT_AXIS_X, g_fftInX);
    processAxis(FFT_AXIS_Y, g_fftInY);
    processAxis(FFT_AXIS_Z, g_fftInZ);
    processAxis(FFT_AXIS_RESULTANT, g_fftInResultant);

    uint16_t peakBin = minBin;
    float peakAmp = 0.0f;
    float noiseSum = 0.0f;
    uint16_t noiseCount = 0;

    for (uint16_t i = 1; i < halfN; i++) {
        if (i >= minBin && i <= maxBin && g_fftMag[i] > peakAmp) {
            peakAmp = g_fftMag[i];
            peakBin = i;
        }
    }

    // Step 1: build candidate fundamentals from strongest line and its subharmonics.
    uint16_t candidates[6] = {peakBin, (uint16_t)(peakBin / 2), (uint16_t)(peakBin / 3), 0, 0, 0};
    const float manualRefHz = getManualRefHz();
    if (manualRefHz > 0.0f) {
        uint16_t refBin = (uint16_t)lroundf(manualRefHz / binHz);
        refBin = constrain(refBin, minBin, maxBin);
        candidates[3] = refBin;
        candidates[4] = (uint16_t)constrain((int)(refBin / 2), (int)minBin, (int)maxBin);
        candidates[5] = (uint16_t)constrain((int)(refBin * 2), (int)minBin, (int)maxBin);
    }

    auto localPeak = [&](uint16_t center, uint16_t& outBin) -> float {
        if (center < minBin) center = minBin;
        if (center > maxBin) center = maxBin;
        const uint16_t lo = (center > 1) ? (center - 1) : center;
        const uint16_t hi = min((uint16_t)(center + 1), maxBin);
        float a = 0.0f;
        outBin = center;
        for (uint16_t i = lo; i <= hi; i++) {
            if (g_fftMag[i] > a) {
                a = g_fftMag[i];
                outBin = i;
            }
        }
        return a;
    };

    auto scoreCandidate = [&](uint16_t center, uint16_t& outBin, float& outA1) -> float {
        if (center <= minBin || center >= maxBin) {
            outBin = center;
            outA1 = 0.0f;
            return -1e9f;
        }
        uint16_t b1 = center;
        const float a1 = localPeak(center, b1);
        if (a1 <= 1e-6f) {
            outBin = b1;
            outA1 = 0.0f;
            return -1e9f;
        }

        float score = a1;
        if ((uint16_t)(2 * b1) <= maxBin) {
            uint16_t b2 = (uint16_t)(2 * b1);
            const float a2 = localPeak(b2, b2);
            score += 0.65f * a2;
            if (a2 > 0.18f * a1) score += 0.10f * a1;
        }
        if ((uint16_t)(3 * b1) <= maxBin) {
            uint16_t b3 = (uint16_t)(3 * b1);
            const float a3 = localPeak(b3, b3);
            score += 0.45f * a3;
            if (a3 > 0.12f * a1) score += 0.08f * a1;
        }

        if (g_prevFundHz > 0.0f) {
            const float candHz = b1 * binHz;
            const float rel = fabsf(candHz - g_prevFundHz) / (g_prevFundHz + 1e-6f);
            if (rel <= 0.20f) score += 0.20f * a1;
            else if (rel >= 0.70f) score -= 0.20f * a1;
        }

        if (manualRefHz > 0.0f) {
            const float candHz = b1 * binHz;
            const float relRef = fabsf(candHz - manualRefHz) / (manualRefHz + 1e-6f);
            // Jeśli peak jest bardzo blisko referencji, wymuś bardzo wysoki score
            if (relRef <= 0.04f) score += 100.0f * a1;
            else if (relRef <= 0.08f) score += 0.80f * a1;
            else if (relRef <= 0.20f) score += 0.30f * a1;
            else if (relRef >= 0.60f) score -= 0.25f * a1;
        }

        outBin = b1;
        outA1 = a1;
        return score;
    };

    // Step 2: choose best fundamental candidate from 1x/2x/3x-consistency score.
    float bestScore = -1e9f;
    uint16_t bestBin = peakBin;
    float bestAmp = peakAmp;
    for (uint8_t ci = 0; ci < 6; ci++) {
        const uint16_t cand = candidates[ci];
        if (cand <= minBin || cand >= maxBin) continue;
        bool duplicate = false;
        for (uint8_t cj = 0; cj < ci; cj++) {
            if (candidates[cj] == cand) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;
        uint16_t candBin = cand;
        float candAmp = 0.0f;
        const float candScore = scoreCandidate(cand, candBin, candAmp);
        if (candScore > bestScore) {
            bestScore = candScore;
            bestBin = candBin;
            bestAmp = candAmp;
        }
    }

    // Step 3: hysteresis against sudden 1x<->2x/3x jumps.
    if (g_prevFundHz > 0.0f) {
        uint16_t prevCenter = (uint16_t)lroundf(g_prevFundHz / binHz);
        if (prevCenter < minBin) prevCenter = minBin;
        if (prevCenter > maxBin) prevCenter = maxBin;
        uint16_t prevBin = prevCenter;
        float prevAmp = 0.0f;
        const float prevScore = scoreCandidate(prevCenter, prevBin, prevAmp);
        const float bestHz = bestBin * binHz;
        const float ratio = bestHz / (g_prevFundHz + 1e-6f);
        const bool harmonicJump =
            (ratio > 1.75f && ratio < 2.25f) || (ratio > 0.44f && ratio < 0.57f) ||
            (ratio > 2.75f && ratio < 3.25f) || (ratio > 0.30f && ratio < 0.38f);

        if (harmonicJump && prevScore > -1e8f && bestScore < prevScore * 1.15f) {
            bestBin = prevBin;
            bestAmp = prevAmp;
            bestScore = prevScore;
        }
    }

    peakBin = bestBin;
    peakAmp = bestAmp;

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
        g_prevFundHz = g_fftPeakHzFilt;
        computeHarmonics(g_fftPeakHzFilt);
    } else {
        g_fftPeakRpm = 0.0f;
        g_prevFundHz = 0.0f;
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
        case 14: return g_actualFs;
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

// Resetuje bufory i liczniki analizy FFT i RMS bez zmiany parametrów.
static void handleResetAnalysis() {
    if (!ensureWebAuth()) return;
#if ENABLE_ADXL
    resetAnalysisState();
    g_samples = 0;
    Serial.println("[RESET] Bufory i liczniki analizy zresetowane przez WWW");
#endif
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleStatusJson() {
    if (!ensureWebAuth()) return;

    const bool wifiOk = (WiFi.status() == WL_CONNECTED);
    const uint32_t heapFree = ESP.getFreeHeap();
    const uint32_t heapTotal = ESP.getHeapSize();
    const uint32_t heapUsed = (heapTotal >= heapFree) ? (heapTotal - heapFree) : 0;
    char json[640];
#if ENABLE_ADXL
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,"
             "\"samples\":%lu,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vibration\":%.3f,"
             "\"rms_x\":%.3f,\"rms_y\":%.3f,\"rms_z\":%.3f,\"rms_total\":%.3f,"
             "\"peak_hz\":%.2f,\"peak_amp\":%.3f,\"peak_confidence_pct\":%.1f,\"rpm\":%.1f,"
             "\"peak_confidence_threshold_pct\":%u,\"manual_ref_rpm\":%.1f,\"cpu_load_pct\":%.1f,"
             "\"heap_free_bytes\":%lu,\"heap_used_bytes\":%lu,\"heap_total_bytes\":%lu}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             g_samples, g_lastX, g_lastY, g_lastZ, g_lastMag, g_rmsX, g_rmsY, g_rmsZ, g_rmsTotal,
             g_fftPeakHzFilt, g_fftPeakAmp, g_fftPeakConfidencePct,
             g_fftPeakRpm, g_peakConfThreshPct, g_manualRefRpm, g_cpuLoadPct,
             (unsigned long)heapFree, (unsigned long)heapUsed, (unsigned long)heapTotal);
#else
    snprintf(json, sizeof(json),
             "{\"uptime_ms\":%lu,\"wifi_connected\":%s,\"ip\":\"%s\",\"rssi\":%d,\"heap\":%u,"
             "\"peak_confidence_threshold_pct\":%u,\"cpu_load_pct\":%.1f,"
             "\"heap_free_bytes\":%lu,\"heap_used_bytes\":%lu,\"heap_total_bytes\":%lu}",
             millis(), wifiOk ? "true" : "false",
             wifiOk ? WiFi.localIP().toString().c_str() : "-",
             wifiOk ? WiFi.RSSI() : 0,
             ESP.getFreeHeap(),
             g_peakConfThreshPct,
             g_cpuLoadPct,
             (unsigned long)heapFree, (unsigned long)heapUsed, (unsigned long)heapTotal);
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
    } else if (server.hasArg("harm_window_bins")) {
        int v = server.arg("harm_window_bins").toInt();
        if (v < 0) v = 0;
        if (v > 32) v = 32;
        g_harmWindowBins = (uint8_t)v;
        Serial.printf("[CFG] harm_window_bins=%u\n", g_harmWindowBins);
    } else if (server.hasArg("odr_hz")) {
#if ENABLE_ADXL
        applyOdr((uint16_t)max(0, (int)server.arg("odr_hz").toInt()));
#endif
    } else if (server.hasArg("trend_window_sec")) {
        int v = server.arg("trend_window_sec").toInt();
        if (v < 5) v = 5;
        if (v > 60) v = 60;
        g_trendWindowSec = (uint8_t)v;
        Serial.printf("[CFG] trend_window_sec=%u\n", g_trendWindowSec);
    } else if (server.hasArg("fft_n")) {
#if ENABLE_ADXL
        applyFftN((uint16_t)max(0, (int)server.arg("fft_n").toInt()));
#endif
    } else if (server.hasArg("manual_ref_rpm")) {
        float v = server.arg("manual_ref_rpm").toFloat();
        if (!isfinite(v) || v < 0.0f) v = 0.0f;
        if (v > 120000.0f) v = 120000.0f;
        g_manualRefRpm = v;
        Serial.printf("[CFG] manual_ref_rpm=%.1f\n", g_manualRefRpm);
    } else if (server.hasArg("fft_analytics_mask")) {
        int v = server.arg("fft_analytics_mask").toInt();
        if (v < 0) v = 0;
        if (v > 15) v = 15;
        g_fftAnalyticsMask = (uint8_t)v;
        resetAnalysisState();
        Serial.printf("[CFG] fft_analytics_mask=0x%X\n", g_fftAnalyticsMask);
    } else if (server.hasArg("fft_primary_axis")) {
        int v = server.arg("fft_primary_axis").toInt();
        if (v < 0) v = 0;
        if (v >= FFT_AXIS_COUNT) v = FFT_AXIS_COUNT - 1;
        g_fftPrimaryAxis = (uint8_t)v;
        Serial.printf("[CFG] fft_primary_axis=%u\n", g_fftPrimaryAxis);
    }

    uint16_t nOptions[8] = {0};
    const uint8_t nOptionsCount = getFftNOptions(nOptions, 8);

    char json[420];
    const float binHz = (float)g_actualFs / (float)g_fftNActive;
    const float harmToleranceHz = g_harmWindowBins * binHz;
    snprintf(json, sizeof(json),
             "{\"peak_confidence_threshold_pct\":%u,\"manual_ref_rpm\":%.1f,\"harm_ratio_thresh_pct\":%u,\"harm_max_order\":%u,\"harm_window_bins\":%u,\"harm_tolerance_hz\":%.3f,\"odr_hz\":%u,\"trend_window_sec\":%u,\"fft_n\":%u,\"fft_n_max\":%u,\"fft_analytics_mask\":%u,\"fft_primary_axis\":%u",
             g_peakConfThreshPct, g_manualRefRpm, g_harmRatioThreshPct, g_harmMaxOrder, g_harmWindowBins, harmToleranceHz,
             g_actualFs, g_trendWindowSec, g_fftNActive, (uint16_t)CONFIG_FFT_SIZE,
             g_fftAnalyticsMask, g_fftPrimaryAxis);

    String resp = json;
    resp += F(",\"fft_n_options\":[");
    for (uint8_t i = 0; i < nOptionsCount; i++) {
        if (i) resp += ',';
        resp += nOptions[i];
    }
    resp += F("]}");
    server.send(200, "application/json", resp);
}

static void handleFftJson() {
    if (!ensureWebAuth()) return;

    String resp;
    resp.reserve(1800);
    const uint16_t nActive = g_fftNActive;
    resp += F("{\"fs\":");
    resp += g_actualFs;
    resp += F(",\"n\":");
    resp += (nActive / 2);
    resp += F(",\"fft_n\":");
    resp += nActive;
    resp += F(",\"resolution\":");
    resp += ((float)g_actualFs / nActive);
    resp += F(",\"trend_window_sec\":");
    resp += g_trendWindowSec;
#if ENABLE_ADXL
    resp += F(",\"fft_analytics_mask\":");
    resp += g_fftAnalyticsMask;
    resp += F(",\"fft_primary_axis\":");
    resp += g_fftPrimaryAxis;
    resp += F(",\"axis_peak_hz\":[");
    resp += g_fftAxisPeakHz[FFT_AXIS_X]; resp += ',';
    resp += g_fftAxisPeakHz[FFT_AXIS_Y]; resp += ',';
    resp += g_fftAxisPeakHz[FFT_AXIS_Z]; resp += ',';
    resp += g_fftAxisPeakHz[FFT_AXIS_RESULTANT];
    resp += F("]");
    resp += F(",\"axis_peak_amp\":[");
    resp += g_fftAxisPeakAmp[FFT_AXIS_X]; resp += ',';
    resp += g_fftAxisPeakAmp[FFT_AXIS_Y]; resp += ',';
    resp += g_fftAxisPeakAmp[FFT_AXIS_Z]; resp += ',';
    resp += g_fftAxisPeakAmp[FFT_AXIS_RESULTANT];
    resp += F("]");
    resp += F(",\"peak_hz\":");
    resp += g_fftPeakHz;
    resp += F(",\"peak_hz_filt\":");
    resp += g_fftPeakHzFilt;
    resp += F(",\"peak_amp\":");
    resp += g_fftPeakAmp;
    resp += F(",\"peak_confidence_pct\":");
    resp += g_fftPeakConfidencePct;
    resp += F(",\"rpm\":");
    resp += g_fftPeakRpm;
    resp += F(",\"rpm_order\":");
    resp += g_rpmOrder;
    resp += F(",\"manual_ref_rpm\":");
    resp += g_manualRefRpm;
    resp += F(",\"manual_ref_hz\":");
    resp += getManualRefHz();
    resp += F(",\"band_min_hz\":");
    resp += g_fftBandMinHz;
    resp += F(",\"band_max_hz\":");
    resp += g_fftBandMaxHz;
    resp += F(",\"harmonic_amps\":[");
    resp += g_harmonicAmp[0]; resp += ',';
    resp += g_harmonicAmp[1]; resp += ',';
    resp += g_harmonicAmp[2];
    resp += F("],\"harmonic_alarm_flags\":");
    resp += g_harmonicAlarmFlags;
    resp += F(",\"harm_ratio_thresh_pct\":");
    resp += g_harmRatioThreshPct;
    resp += F(",\"harm_window_bins\":");
    resp += g_harmWindowBins;
    resp += F(",\"harm_tolerance_hz\":");
    resp += (g_harmWindowBins * ((float)g_actualFs / nActive));
    resp += F(",\"harm_max_order\":");
    resp += g_harmMaxOrder;
    resp += F(",\"mag\":[");
    for (uint16_t i = 0; i < (nActive / 2); i++) {
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
        #trendCanvas { width: 100%; height: 140px; background: #f8fafc; border-radius: 8px; display: block; }
        .cfg-row { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin-top: 6px; }
        .cfg-row input[type="number"] { width: 84px; padding: 4px 6px; }
        .cfg-row button { padding: 5px 10px; border: 0; border-radius: 6px; background: #2563eb; color: #fff; cursor: pointer; }
        .muted { color: #64748b; font-size: 0.83rem; }
        .help-target { cursor: help; }
        .help-btn {
            border: 0;
            border-radius: 999px;
            width: 22px;
            height: 22px;
            font-size: 12px;
            font-weight: 700;
            line-height: 22px;
            color: #1e3a8a;
            background: #dbeafe;
            cursor: pointer;
        }
        .mini-help {
            position: fixed;
            z-index: 40;
            max-width: 290px;
            background: #0f172a;
            color: #f8fafc;
            border-radius: 10px;
            padding: 10px 12px;
            font-size: 12px;
            line-height: 1.35;
            box-shadow: 0 12px 26px rgba(2, 6, 23, 0.24);
            opacity: 0;
            pointer-events: none;
            transform: translateY(4px);
            transition: opacity .18s ease, transform .18s ease;
        }
        .mini-help.show {
            opacity: 1;
            transform: translateY(0);
        }
        .modal-backdrop {
            position: fixed;
            inset: 0;
            background: rgba(15, 23, 42, 0.48);
            display: none;
            align-items: center;
            justify-content: center;
            z-index: 45;
            padding: 16px;
        }
        .modal-backdrop.show { display: flex; }
        .modal-card {
            width: min(760px, 100%);
            max-height: 86vh;
            overflow: auto;
            background: #fff;
            border-radius: 14px;
            padding: 16px;
            box-shadow: 0 20px 44px rgba(2, 6, 23, .28);
        }
        .modal-card h2 { margin: 0 0 8px; font-size: 1.1rem; }
        .modal-card h3 { margin: 14px 0 4px; font-size: .98rem; color: #1e3a8a; }
        .modal-card p { margin: 0; color: #334155; font-size: .92rem; }
        .modal-actions { display: flex; justify-content: flex-end; margin-top: 12px; }
    </style>
</head>
<body>
    <div id="miniHelp" class="mini-help" role="status" aria-live="polite"></div>

    <div class="wrap">
        <div class="card">
            <div style="display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap">
                <h1 style="margin:0">ESP32-C3 ADXL345</h1>
                <button id="openParamGuide" class="help-btn" type="button" title="Wiekszy opis parametrow zmienianych przez WWW">i</button>
            </div>
            <div class="grid">
                <div><div class="label">Wi-Fi</div><div class="value" id="wifi">-</div></div>
                <div><div class="label">IP</div><div class="value mono" id="ip">-</div></div>
                <div><div class="label">RSSI</div><div class="value" id="rssi">-</div></div>
                <div><div class="label">Uptime [ms]</div><div class="value mono" id="uptime">-</div></div>
                <div><div class="label">CPU [%]</div><div class="value mono" id="cpu">-</div></div>
                <div><div class="label">Heap free [KB]</div><div class="value mono" id="heapFree">-</div></div>
                <div><div class="label">Heap used [KB]</div><div class="value mono" id="heapUsed">-</div></div>
                <div><div class="label">Heap total [KB]</div><div class="value mono" id="heapTotal">-</div></div>
            </div>
        </div>

        <div class="card">
            <div class="grid">
                <div><div class="label">Acc X chwilowe [m/s2]</div><div class="value mono" id="x">-</div></div>
                <div><div class="label">Acc Y chwilowe [m/s2]</div><div class="value mono" id="y">-</div></div>
                <div><div class="label">Acc Z chwilowe [m/s2]</div><div class="value mono" id="z">-</div></div>
                <div><div class="label">Modul chwilowy [m/s2]</div><div class="value mono" id="v">-</div></div>
            </div>
            <div class="grid" style="margin-top:8px">
                <div><div class="label">Drgania RMS X (AC) [m/s2]</div><div class="value mono" id="rmsX">-</div></div>
                <div><div class="label">Drgania RMS Y (AC) [m/s2]</div><div class="value mono" id="rmsY">-</div></div>
                <div><div class="label">Drgania RMS Z (AC) [m/s2]</div><div class="value mono" id="rmsZ">-</div></div>
                <div><div class="label">Drgania RMS Total [m/s2]</div><div class="value mono" id="rmsT">-</div></div>
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
            <div class="cfg-row">
                <span class="label">Szerokosc okna harmonicznych [biny]</span>
                <input id="harmBins" type="number" min="0" max="32" step="1" value="2" />
                <button id="saveHarmBins" type="button">Zapisz</button>
                <span class="muted">Tolerancja: +/- <span class="mono" id="harmTol">-</span> Hz (okno <span class="mono" id="harmWinHz">-</span> Hz)</span>
            </div>
            <div class="cfg-row">
                <span class="label">ODR czujnika [Hz]</span>
                <select id="odrSelect">
                    <option value="50">50 Hz</option>
                    <option value="100">100 Hz</option>
                    <option value="200">200 Hz</option>
                    <option value="400">400 Hz</option>
                    <option value="800" selected>800 Hz</option>
                </select>
                <button id="saveOdr" type="button">Ustaw</button>
                <span class="muted">Rozdzielczosc: <span class="mono" id="fftRes2">-</span> Hz/bin</span>
            </div>
            <div class="cfg-row">
                <span class="label">FFT N</span>
                <select id="fftNSelect"></select>
                <button id="saveFftN" type="button">Ustaw</button>
                <span class="muted">Dostepne N: potegi 2 (64..max)</span>
            </div>
            <div class="cfg-row">
                <span class="label">Analityki FFT</span>
                <label style="font-weight:normal"><input id="anaX" type="checkbox" /> X</label>
                <label style="font-weight:normal"><input id="anaY" type="checkbox" /> Y</label>
                <label style="font-weight:normal"><input id="anaZ" type="checkbox" checked /> Z</label>
                <label style="font-weight:normal"><input id="anaR" type="checkbox" /> Wypadkowa</label>
                <span class="label" style="margin-left:8px">Oś glowna</span>
                <select id="fftPrimaryAxis">
                    <option value="0">X</option>
                    <option value="1">Y</option>
                    <option value="2" selected>Z</option>
                    <option value="3">Wypadkowa</option>
                </select>
                <button id="saveFftAnalytics" type="button">Ustaw</button>
                <span class="muted mono" id="fftAnalyticsStatus">-</span>
                <span class="muted">Wylacz nieuzywane osie, aby zmniejszyc obciazenie CPU.</span>
            </div>
            <div class="cfg-row">
                <span class="label">RPM referencyjne reczne</span>
                <input id="manualRefRpm" type="number" min="0" max="120000" step="1" value="0" />
                <button id="saveManualRefRpm" type="button">Ustaw</button>
                <label style="margin-left:12px;font-weight:normal"><input type="checkbox" id="enableManualRefRpm" checked /> użyj</label>
                <span class="muted">Marker FFT: <span class="mono" id="manualRefHz">0.00</span> Hz (odznacz = wyłącz)</span>
            </div>
            <div class="cfg-row">
                <span class="label">Trend [s]</span>
                <input id="trendSec" type="number" min="5" max="60" step="1" value="30" />
                <button id="saveTrendSec" type="button">Ustaw</button>
                <span class="muted">Zakres 5-60 s</span>
            </div>
            <div class="cfg-row">
                <span class="label">Analiza</span>
                <button id="resetAnalysis" type="button">Reset danych</button>
                <span class="muted">Czyści bufor FFT, RMS i licznik probek. Ustawienia zostaja bez zmian.</span>
            </div>
        </div>

        <div class="card">
            <div style="font-size:.9rem;font-weight:600;color:#334155;margin-bottom:6px">Trend RMS (ostatnie <span id="trendLbl">30</span>s)</div>
            <canvas id="trendCanvas"></canvas>
            <p class="label">Linie: X (czerwona), Y (zielona), Z (niebieska), Total (czarna)</p>
        </div>

        <div class="card">
            <div style="font-size:.9rem;font-weight:600;color:#334155;margin-bottom:6px">Widmo FFT |a| (0 - 400 Hz)</div>
            <canvas id="fftCanvas"></canvas>
            <p class="label">Fs=<span id="fftFs">-</span> Hz | <span id="fftRes">-</span> Hz/bin | N=<span id="fftN">-</span> | Pasmo: <span id="fftBand">-</span> | Oś: <span id="fftSourceLabel">-</span></p>
        </div>
    </div>

    <div id="paramGuideModal" class="modal-backdrop" role="dialog" aria-modal="true" aria-labelledby="paramGuideTitle">
        <div class="modal-card">
            <h2 id="paramGuideTitle">Parametry zmieniane przez WWW</h2>
            <h3>Prog pewnosci bledu odczytu [%]</h3>
            <p>Minimalna pewnosc piku FFT potrzebna do uznania RPM za wiarygodne. Gdy pewnosc spadnie ponizej progu, RPM jest zerowane i alarmy zalezne od piku sa mniej istotne.</p>
            <h3>Prog amplitudy harmonicznej [%]</h3>
            <p>Minimalny stosunek amplitudy harmonicznej do amplitudy fundamentalnej, przy ktorym harmoniczna jest oznaczana jako aktywna. Nizszy prog = wiecej czuly, wyzszy prog = mniej falszywych alarmow.</p>
            <h3>Szerokosc okna harmonicznych [biny]</h3>
            <p>Polowa okna wyszukiwania maksimum harmonicznej wokol czestotliwosci docelowej. Dla wartosci 2 szukamy od -2 do +2 binow. Tolerancja w Hz zalezy od Fs oraz N i jest liczona na biezaco.</p>
            <h3>ODR czujnika [Hz]</h3>
            <p>Czestotliwosc probkowania ADXL345 i jednoczesnie Fs analizy FFT. Wieksze ODR daje szersze pasmo, mniejsze ODR poprawia rozdzielczosc czestotliwosci dla aktualnie ustawionego N.</p>
            <h3>FFT N</h3>
            <p>Liczba probek okna FFT. Wieksze N poprawia rozdzielczosc czestotliwosci, ale wyraznie zwieksza opoznienie i koszt obliczen. Uzywaj z glowa: zwiekszaj stopniowo (np. 256 -&gt; 512 -&gt; 1024) i obserwuj CPU oraz plynosc odswiezania.</p>
            <h3>Trend [s]</h3>
            <p>Zakres czasu historii wykresu RMS. Parametr ustawiany tylko przez WWW, zakres 5..60 sekund.</p>
            <div class="modal-actions">
                <button id="closeParamGuide" type="button">Zamknij</button>
            </div>
        </div>
    </div>

    <script>
        const $ = id => document.getElementById(id);
        const fc = $('fftCanvas');
        const fctx = fc.getContext('2d');
        const tc = $('trendCanvas');
        const tctx = tc.getContext('2d');
        const miniHelp = $('miniHelp');
        const paramGuideModal = $('paramGuideModal');
        let peakConfidenceThresholdPct = 60;
        let peakConfidencePct = 0;
        let harmRatioThreshPct = 30;
        let harmWindowBins = 2;
        let trendWindowSec = 30;
        const fftSourceLabels = ['X', 'Y', 'Z', 'Wypadkowa'];
        const trendPoints = [];

        const miniHelpTexts = {
            wifi: 'Stan polaczenia ESP32 z siecia Wi-Fi.',
            ip: 'Adres IP urzadzenia w sieci lokalnej.',
            rssi: 'Sila sygnalu Wi-Fi w dBm. Blizej 0 oznacza mocniejszy sygnal.',
            uptime: 'Czas pracy od ostatniego restartu [ms].',
            cpu: 'Przyblizone obciazenie toru pomiar+FFT wzgledem budzetu czasowego probkowania [%].',
            heapFree: 'Wolna pamiec sterty ESP (heap).',
            heapUsed: 'Uzyta pamiec sterty ESP (heap).',
            heapTotal: 'Calkowity rozmiar sterty ESP (heap).',
            x: 'Przyspieszenie osi X [m/s2].',
            y: 'Przyspieszenie osi Y [m/s2].',
            z: 'Przyspieszenie osi Z [m/s2].',
            v: 'Modul wektora drgan: sqrt(x^2 + y^2 + z^2).',
            rmsX: 'Drgania RMS osi X (AC): skladowa zmienna, bez stalej grawitacyjnej.',
            rmsY: 'Drgania RMS osi Y (AC): skladowa zmienna, bez stalej grawitacyjnej.',
            rmsZ: 'Drgania RMS osi Z (AC): skladowa zmienna, bez stalej grawitacyjnej.',
            rmsT: 'Wypadkowe drgania RMS: sqrt(rmsX^2 + rmsY^2 + rmsZ^2).',
            samples: 'Liczba pobranych probek od startu.',
            peak: 'Dominujaca czestotliwosc piku FFT po filtracji [Hz].',
            rpm: 'Obroty/min liczone z piku FFT i rzedu RPM.',
            errThr: 'Konfigurowalny prog pewnosci piku FFT [%].',
            errConf: 'Biezaca pewnosc piku FFT [%].',
            harmThr: 'Konfigurowalny prog aktywacji harmonicznej [% amplitudy fundamentalnej].',
            harmStatus: 'Ktore harmoniczne (2x/3x/4x) przekroczyly prog.',
            harmBins: 'Konfigurowana polowa okna wyszukiwania harmonicznych w binach FFT.',
            harmTol: 'Przeliczona tolerancja jednostronna: bins * Fs / N [Hz].',
            harmWinHz: 'Pelne okno analizy harmonicznej: 2 * tolerancja [Hz].',
            odrSelect: 'Konfigurowana czestotliwosc probkowania ADXL345 [Hz].',
            fftNSelect: 'Aktywna liczba probek okna FFT (N). Wieksze N = lepsza rozdzielczosc, ale wieksze opoznienie i obciazenie CPU. Uzywaj z glowa.',
            anaX: 'Wlacza analize FFT dla osi X.',
            anaY: 'Wlacza analize FFT dla osi Y.',
            anaZ: 'Wlacza analize FFT dla osi Z (domyslnie aktywna po starcie).',
            anaR: 'Wlacza analize FFT dla sygnalu wypadkowego.',
            fftPrimaryAxis: 'Wybiera os glowna do piku/RPM i wykresu FFT.',
            saveFftAnalytics: 'Zapisuje wlaczone analityki FFT i os glowna.',
            manualRefRpm: 'Recznie podana predkosc obrotowa [RPM], uzywana jako referencja przy wyborze piku/fundamentalnej z FFT. 0 = brak referencji.',
            manualRefHz: 'Czestotliwosc referencyjna [Hz] liczona z RPM i rzedu RPM.',
            trendSec: 'Zakres czasu historii trendu RMS [s], tylko przez WWW (max 60).',
            resetAnalysis: 'Usuwa z pamieci analizowane dane i zaczyna nowa analize bez zmiany ustawien.',
            fftRes2: 'Rozdzielczosc FFT wynikajaca z bieżącego Fs i N [Hz/bin].',
            fftFs: 'Aktualne Fs analizy FFT [Hz].',
            fftRes: 'Aktualna rozdzielczosc FFT [Hz/bin].',
            fftN: 'Liczba punktow polowy widma (N/2).',
            fftBand: 'Pasmo przeszukiwania piku FFT [Hz].',
            openParamGuide: 'Otwiera pelny opis parametrow konfigurowanych przez WWW.'
        };

        function showMiniHelp(target) {
            if (!target || !target.id) return;
            const text = miniHelpTexts[target.id];
            if (!text) return;
            miniHelp.textContent = text;
            const r = target.getBoundingClientRect();
            const left = Math.min(window.innerWidth - 308, Math.max(8, r.left));
            const top = Math.min(window.innerHeight - 84, r.bottom + 8);
            miniHelp.style.left = left + 'px';
            miniHelp.style.top = top + 'px';
            miniHelp.classList.add('show');
            clearTimeout(showMiniHelp._t);
            showMiniHelp._t = setTimeout(() => miniHelp.classList.remove('show'), 2600);
        }

        function initMiniHelpTargets() {
            Object.keys(miniHelpTexts).forEach(id => {
                const el = $(id);
                if (!el) return;
                el.classList.add('help-target');
                el.addEventListener('click', () => showMiniHelp(el));
            });
        }

        function openParamGuide() { paramGuideModal.classList.add('show'); }
        function closeParamGuide() { paramGuideModal.classList.remove('show'); }

        function setValueIfNotFocused(id, valueStr) {
            const el = $(id);
            if (!el) return;
            if (document.activeElement === el) return;
            el.value = valueStr;
        }

        function getAnalyticsMaskFromUi() {
            let mask = 0;
            if ($('anaX').checked) mask |= 1;
            if ($('anaY').checked) mask |= 2;
            if ($('anaZ').checked) mask |= 4;
            if ($('anaR').checked) mask |= 8;
            return mask;
        }

        function applyAnalyticsMaskToUi(mask) {
            $('anaX').checked = !!(mask & 1);
            $('anaY').checked = !!(mask & 2);
            $('anaZ').checked = !!(mask & 4);
            $('anaR').checked = !!(mask & 8);
        }

        async function saveFftAnalytics() {
            const mask = getAnalyticsMaskFromUi();
            const primary = Number($('fftPrimaryAxis').value);
            const statusEl = $('fftAnalyticsStatus');
            statusEl.textContent = 'Zapisywanie...';
            statusEl.style.color = '#64748b';
            try {
                let r = await fetch('/api/config?fft_analytics_mask=' + mask, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                let d = await r.json();
                if (typeof d.fft_analytics_mask === 'number') {
                    applyAnalyticsMaskToUi(Number(d.fft_analytics_mask));
                }

                r = await fetch('/api/config?fft_primary_axis=' + primary, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                d = await r.json();
                if (typeof d.fft_primary_axis === 'number') {
                    setValueIfNotFocused('fftPrimaryAxis', String(d.fft_primary_axis));
                    $('fftSourceLabel').textContent = fftSourceLabels[d.fft_primary_axis] || '-';
                }
                statusEl.textContent = 'Zapisano';
                statusEl.style.color = '#16a34a';
            } catch (_) {
                statusEl.textContent = 'Blad zapisu';
                statusEl.style.color = '#dc2626';
            }
        }

        function renderTrendChart() {
            const dpr = window.devicePixelRatio || 1;
            const W = tc.clientWidth;
            const H = tc.clientHeight;
            tc.width = W * dpr;
            tc.height = H * dpr;
            tctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            tctx.clearRect(0, 0, W, H);

            if (!trendPoints.length) return;
            const now = Date.now();
            const minTs = now - trendWindowSec * 1000;
            while (trendPoints.length && trendPoints[0].ts < minTs) trendPoints.shift();
            if (!trendPoints.length) return;

            let yMax = 1e-6;
            for (const p of trendPoints) {
                yMax = Math.max(yMax, p.rx, p.ry, p.rz, p.rt);
            }
            const padL = 34;
            const padT = 8;
            const padB = 16;
            const plotW = W - padL - 6;
            const plotH = H - padT - padB;

            tctx.strokeStyle = 'rgba(0,0,0,.08)';
            tctx.beginPath();
            tctx.moveTo(padL, padT + plotH);
            tctx.lineTo(W - 6, padT + plotH);
            tctx.moveTo(padL, padT);
            tctx.lineTo(padL, padT + plotH);
            tctx.stroke();

            tctx.fillStyle = '#64748b';
            tctx.font = '10px sans-serif';
            tctx.fillText('0', 6, padT + plotH + 3);
            tctx.fillText(yMax.toFixed(2), 2, padT + 8);

            const drawLine = (key, color) => {
                tctx.strokeStyle = color;
                tctx.lineWidth = 1.4;
                tctx.beginPath();
                for (let i = 0; i < trendPoints.length; i++) {
                    const p = trendPoints[i];
                    const x = padL + ((p.ts - minTs) / (trendWindowSec * 1000)) * plotW;
                    const y = padT + plotH - (p[key] / yMax) * plotH;
                    if (i === 0) tctx.moveTo(x, y);
                    else tctx.lineTo(x, y);
                }
                tctx.stroke();
            };

            drawLine('rx', '#ef4444');
            drawLine('ry', '#16a34a');
            drawLine('rz', '#2563eb');
            drawLine('rt', '#0f172a');
        }

        async function saveErrorThreshold() {
            const raw = Number($('errThr').value);
            const val = Number.isFinite(raw) ? Math.max(0, Math.min(100, Math.round(raw))) : peakConfidenceThresholdPct;
            $('errThr').value = String(val);
            try {
                const r = await fetch('/api/config?peak_confidence_threshold_pct=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                peakConfidenceThresholdPct = Number(d.peak_confidence_threshold_pct || val);
                setValueIfNotFocused('errThr', String(peakConfidenceThresholdPct));
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
                setValueIfNotFocused('harmThr', String(harmRatioThreshPct));
            } catch (_) {
            }
        }

        async function saveHarmWindowBins() {
            const raw = Number($('harmBins').value);
            const val = Number.isFinite(raw) ? Math.max(0, Math.min(32, Math.round(raw))) : harmWindowBins;
            $('harmBins').value = String(val);
            try {
                const r = await fetch('/api/config?harm_window_bins=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                if (typeof d.harm_window_bins === 'number') {
                    harmWindowBins = Math.max(0, Math.min(32, Math.round(d.harm_window_bins)));
                    setValueIfNotFocused('harmBins', String(harmWindowBins));
                }
                if (typeof d.harm_tolerance_hz === 'number') {
                    $('harmTol').textContent = d.harm_tolerance_hz.toFixed(2);
                    $('harmWinHz').textContent = (2 * d.harm_tolerance_hz).toFixed(2);
                }
            } catch (_) {
            }
        }

        async function saveOdr() {
            const val = Number($('odrSelect').value);
            try {
                const r = await fetch('/api/config?odr_hz=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                if (d.odr_hz) {
                    setValueIfNotFocused('odrSelect', String(d.odr_hz));
                    const nNow = Number(d.fft_n || $('fftNSelect').value || 256);
                    $('fftRes2').textContent = (d.odr_hz / nNow).toFixed(2);
                }
            } catch (_) {
            }
        }

        async function saveTrendWindowSec() {
            const raw = Number($('trendSec').value);
            const val = Number.isFinite(raw) ? Math.max(5, Math.min(60, Math.round(raw))) : trendWindowSec;
            $('trendSec').value = String(val);
            try {
                const r = await fetch('/api/config?trend_window_sec=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                if (typeof d.trend_window_sec === 'number') {
                    trendWindowSec = Math.max(5, Math.min(60, Math.round(d.trend_window_sec)));
                    setValueIfNotFocused('trendSec', String(trendWindowSec));
                    $('trendLbl').textContent = String(trendWindowSec);
                }
            } catch (_) {
            }
        }

        async function saveFftN() {
            const val = Number($('fftNSelect').value);
            try {
                const r = await fetch('/api/config?fft_n=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                if (typeof d.fft_n === 'number') {
                    setValueIfNotFocused('fftNSelect', String(d.fft_n));
                    $('fftN').textContent = Math.floor(d.fft_n / 2);
                    $('fftRes2').textContent = ((d.odr_hz || 0) / d.fft_n).toFixed(2);
                }
            } catch (_) {
            }
        }

        async function saveManualReferenceRpm() {
            const enabled = $('enableManualRefRpm').checked;
            let val = 0;
            if (enabled) {
                const raw = Number($('manualRefRpm').value);
                val = Number.isFinite(raw) ? Math.max(0, Math.min(120000, Math.round(raw))) : 0;
            }
            $('manualRefRpm').value = String(val);
            try {
                const r = await fetch('/api/config?manual_ref_rpm=' + val, { cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                const d = await r.json();
                if (typeof d.manual_ref_rpm === 'number') {
                    setValueIfNotFocused('manualRefRpm', String(Math.round(d.manual_ref_rpm)));
                    const refHz = (d.manual_ref_rpm * (Number(d.rpm_order || 1))) / 60.0;
                    $('manualRefHz').textContent = refHz.toFixed(2);
                }
            } catch (_) {
            }
        }

        async function resetAnalysis() {
            const btn = $('resetAnalysis');
            const prevText = btn.textContent;
            btn.disabled = true;
            btn.textContent = 'Reset...';
            try {
                const r = await fetch('/api/reset', { method: 'POST', cache: 'no-store' });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                trendPoints.length = 0;
                renderTrendChart();
                $('samples').textContent = '0';
                $('peak').textContent = '0.0 Hz';
                $('rpm').textContent = '0';
                $('errConf').textContent = '0';
                $('harmStatus').textContent = 'brak';
                $('harmStatus').style.color = '#16a34a';
                await refreshStatus();
                await refreshFFT();
            } catch (_) {
            } finally {
                btn.disabled = false;
                btn.textContent = prevText;
            }
        }

        async function loadConfig() {
            try {
                const r = await fetch('/api/config', { cache: 'no-store' });
                if (!r.ok) return;
                const d = await r.json();
                if (typeof d.trend_window_sec === 'number') {
                    trendWindowSec = Math.max(5, Math.min(60, Math.round(d.trend_window_sec)));
                    setValueIfNotFocused('trendSec', String(trendWindowSec));
                    $('trendLbl').textContent = String(trendWindowSec);
                }
                if (typeof d.harm_ratio_thresh_pct === 'number') {
                    harmRatioThreshPct = Number(d.harm_ratio_thresh_pct);
                    setValueIfNotFocused('harmThr', String(harmRatioThreshPct));
                }
                if (typeof d.manual_ref_rpm === 'number') {
                    setValueIfNotFocused('manualRefRpm', String(Math.round(d.manual_ref_rpm)));
                }
                if (typeof d.harm_window_bins === 'number') {
                    harmWindowBins = Math.max(0, Math.min(32, Math.round(d.harm_window_bins)));
                    setValueIfNotFocused('harmBins', String(harmWindowBins));
                }
                if (typeof d.odr_hz === 'number') {
                    setValueIfNotFocused('odrSelect', String(d.odr_hz));
                }
                if (Array.isArray(d.fft_n_options)) {
                    const sel = $('fftNSelect');
                    sel.innerHTML = '';
                    d.fft_n_options.forEach(n => {
                        const o = document.createElement('option');
                        o.value = String(n);
                        o.textContent = String(n);
                        sel.appendChild(o);
                    });
                }
                if (typeof d.fft_n === 'number') {
                    setValueIfNotFocused('fftNSelect', String(d.fft_n));
                }
                if (typeof d.fft_analytics_mask === 'number') {
                    applyAnalyticsMaskToUi(Number(d.fft_analytics_mask));
                }
                if (typeof d.fft_primary_axis === 'number') {
                    setValueIfNotFocused('fftPrimaryAxis', String(d.fft_primary_axis));
                    $('fftSourceLabel').textContent = fftSourceLabels[d.fft_primary_axis] || '-';
                }
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
                $('cpu').textContent = (typeof s.cpu_load_pct === 'number') ? s.cpu_load_pct.toFixed(1) : '-';
                $('heapFree').textContent = (typeof s.heap_free_bytes === 'number') ? (s.heap_free_bytes / 1024).toFixed(1) : '-';
                $('heapUsed').textContent = (typeof s.heap_used_bytes === 'number') ? (s.heap_used_bytes / 1024).toFixed(1) : '-';
                $('heapTotal').textContent = (typeof s.heap_total_bytes === 'number') ? (s.heap_total_bytes / 1024).toFixed(1) : '-';
                $('x').textContent = s.x.toFixed(3);
                $('y').textContent = s.y.toFixed(3);
                $('z').textContent = s.z.toFixed(3);
                $('v').textContent = s.vibration.toFixed(3);
                $('rmsX').textContent = (s.rms_x || 0).toFixed(3);
                $('rmsY').textContent = (s.rms_y || 0).toFixed(3);
                $('rmsZ').textContent = (s.rms_z || 0).toFixed(3);
                $('rmsT').textContent = (s.rms_total || 0).toFixed(3);
                trendPoints.push({
                    ts: Date.now(),
                    rx: Number(s.rms_x || 0),
                    ry: Number(s.rms_y || 0),
                    rz: Number(s.rms_z || 0),
                    rt: Number(s.rms_total || 0)
                });
                renderTrendChart();
                $('samples').textContent = s.samples;
                $('peak').textContent = (s.peak_hz || 0).toFixed(1) + ' Hz';
                peakConfidencePct = Number(s.peak_confidence_pct || 0);
                if (typeof s.peak_confidence_threshold_pct === 'number') {
                    peakConfidenceThresholdPct = Math.max(0, Math.min(100, Math.round(s.peak_confidence_threshold_pct)));
                    setValueIfNotFocused('errThr', String(peakConfidenceThresholdPct));
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
                $('cpu').textContent = '-';
                $('heapFree').textContent = '-';
                $('heapUsed').textContent = '-';
                $('heapTotal').textContent = '-';
                $('rmsX').textContent = '-';
                $('rmsY').textContent = '-';
                $('rmsZ').textContent = '-';
                $('rmsT').textContent = '-';
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
                if (typeof d.fft_primary_axis === 'number') {
                    $('fftSourceLabel').textContent = fftSourceLabels[d.fft_primary_axis] || '-';
                }
                if (typeof d.fft_n === 'number') {
                    setValueIfNotFocused('fftNSelect', String(d.fft_n));
                }
                // Sync ODR dropdown and resolution hint
                if (d.fs && $('odrSelect').value !== String(d.fs)) {
                    setValueIfNotFocused('odrSelect', String(d.fs));
                }
                $('fftRes2').textContent = d.resolution.toFixed(2);
                if (typeof d.manual_ref_rpm === 'number') {
                    setValueIfNotFocused('manualRefRpm', String(Math.round(d.manual_ref_rpm)));
                    $('manualRefHz').textContent = Number(d.manual_ref_hz || 0).toFixed(2);
                }

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

                // Manual RPM reference marker.
                if ((d.manual_ref_hz || 0) > 0 && (d.manual_ref_hz || 0) < (d.fs / 2)) {
                    const refX = (d.manual_ref_hz / (d.fs / 2)) * W;
                    fctx.strokeStyle = 'rgba(234,88,12,0.95)';
                    fctx.lineWidth = 1;
                    fctx.setLineDash([6, 3]);
                    fctx.beginPath();
                    fctx.moveTo(refX, 0);
                    fctx.lineTo(refX, yBase);
                    fctx.stroke();
                    fctx.setLineDash([]);
                    fctx.fillStyle = '#c2410c';
                    fctx.textAlign = 'left';
                    fctx.fillText('REF ' + Number(d.manual_ref_rpm || 0).toFixed(0) + ' rpm', Math.min(refX + 4, W - 94), 24);
                }

                // Harmonic markers (2x, 3x, 4x)
                const harmAmps = d.harmonic_amps || [0, 0, 0];
                const harmAlarm = d.harmonic_alarm_flags || 0;
                if (typeof d.harm_ratio_thresh_pct === 'number') {
                    harmRatioThreshPct = d.harm_ratio_thresh_pct;
                    setValueIfNotFocused('harmThr', String(harmRatioThreshPct));
                }
                if (typeof d.harm_window_bins === 'number') {
                    harmWindowBins = Math.max(0, Math.min(32, Math.round(d.harm_window_bins)));
                    setValueIfNotFocused('harmBins', String(harmWindowBins));
                }
                if (typeof d.harm_tolerance_hz === 'number') {
                    $('harmTol').textContent = d.harm_tolerance_hz.toFixed(2);
                    $('harmWinHz').textContent = (2 * d.harm_tolerance_hz).toFixed(2);
                }
                if (typeof d.trend_window_sec === 'number') {
                    trendWindowSec = Math.max(5, Math.min(60, Math.round(d.trend_window_sec)));
                    setValueIfNotFocused('trendSec', String(trendWindowSec));
                    $('trendLbl').textContent = String(trendWindowSec);
                }
                if (Array.isArray(d.fft_n_options)) {
                    const sel = $('fftNSelect');
                    if (!sel.dataset.inited || sel.options.length !== d.fft_n_options.length) {
                        sel.innerHTML = '';
                        d.fft_n_options.forEach(n => {
                            const o = document.createElement('option');
                            o.value = String(n);
                            o.textContent = String(n);
                            sel.appendChild(o);
                        });
                        sel.dataset.inited = '1';
                    }
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

        loadConfig();
        refreshStatus();
        refreshFFT();
        initMiniHelpTargets();
        $('saveErrThr').addEventListener('click', saveErrorThreshold);
        $('saveHarmThr').addEventListener('click', saveHarmThreshold);
        $('saveHarmBins').addEventListener('click', saveHarmWindowBins);
        $('saveOdr').addEventListener('click', saveOdr);
        $('saveTrendSec').addEventListener('click', saveTrendWindowSec);
        $('saveFftN').addEventListener('click', saveFftN);
        $('saveFftAnalytics').addEventListener('click', saveFftAnalytics);
        $('saveManualRefRpm').addEventListener('click', saveManualReferenceRpm);
        $('enableManualRefRpm').addEventListener('change', saveManualReferenceRpm);
        $('resetAnalysis').addEventListener('click', resetAnalysis);
        $('openParamGuide').addEventListener('click', openParamGuide);
        $('closeParamGuide').addEventListener('click', closeParamGuide);
        paramGuideModal.addEventListener('click', e => {
            if (e.target === paramGuideModal) closeParamGuide();
        });
        document.addEventListener('keydown', e => {
            if (e.key === 'Escape') closeParamGuide();
        });
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

// Draws a 4-bar signal-strength WiFi icon in the top-right corner (x=113..127, y=2..9).
// Filled bars = connected; disconnected = no bars drawn for better readability.
static void drawWifiIcon(bool connected) {
    for (uint8_t i = 0; i < 4; i++) {
        const uint8_t bx = 113 + i * 4u;   // 3-px bar + 1-px gap
        const uint8_t bh = 2u + i * 2u;    // heights: 2,4,6,8
        const uint8_t by = 10u - bh;       // align bottoms at y=9
        if (connected) {
            display.drawBox(bx, by, 3, bh);
        }
    }
}

// Draws XYZ axis arrows in the bottom-right corner (isometric style, like ADXL345 silkscreen).
// Origin ~(112,57): X->right, Y->up, Z->diagonal up-left.
static void drawAxesIcon() {
    const uint8_t ox = 112, oy = 57;
    // X axis (right)
    display.drawLine(ox, oy, ox + 10, oy);
    display.drawLine(ox + 10, oy, ox + 8, oy - 2);
    display.drawLine(ox + 10, oy, ox + 8, oy + 2);
    // Y axis (up)
    display.drawLine(ox, oy, ox, oy - 10);
    display.drawLine(ox, oy - 10, ox - 2, oy - 8);
    display.drawLine(ox, oy - 10, ox + 2, oy - 8);
    // Z axis (diagonal up-left, isometric)
    display.drawLine(ox, oy, ox - 7, oy - 7);
    display.drawLine(ox - 7, oy - 7, ox - 4, oy - 7);
    display.drawLine(ox - 7, oy - 7, ox - 7, oy - 4);
    // Labels
    display.setFont(u8g2_font_4x6_tf);
    display.drawStr(ox + 12, oy + 3, "X");
    display.drawStr(ox + 2,  oy - 12, "Y");
    display.drawStr(ox - 13, oy - 7, "Z");
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

    accel.setRange(ADXL345_RANGE_2_G);
    applyOdr(CONFIG_FFT_FS);

    Serial.printf("[ADXL] Initialized SPI: CS=%d SCK=%d MOSI=%d MISO=%d Fs=%dHz\n",
                  CONFIG_ADXL_SPI_CS_PIN,
                  CONFIG_ADXL_SPI_SCK_PIN,
                  CONFIG_ADXL_SPI_MOSI_PIN,
                  CONFIG_ADXL_SPI_MISO_PIN,
                  g_actualFs);
#endif

    connectWiFi();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatusJson);
    server.on("/api/fft", HTTP_GET, handleFftJson);
    server.on("/api/config", HTTP_GET, handleConfigJson);
    server.on("/api/reset", HTTP_POST, handleResetAnalysis);
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
    const uint32_t nowCpuUs = micros();
    if (g_cpuWindowStartUs == 0) {
        g_cpuWindowStartUs = nowCpuUs;
    }

    server.handleClient();

#if ENABLE_MODBUS
    handleModbusTcp();
#endif

#if ENABLE_ADXL
    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - g_lastSampleUs) >= g_samplePeriodUs) {
        const uint32_t procStartUs = micros();
        g_lastSampleUs += g_samplePeriodUs;

        sensors_event_t event;
        accel.getEvent(&event);

        g_lastX = event.acceleration.x;
        g_lastY = event.acceleration.y;
        g_lastZ = event.acceleration.z;
        g_lastMag = sqrtf(g_lastX * g_lastX + g_lastY * g_lastY + g_lastZ * g_lastZ);
        g_winSumX += g_lastX;
        g_winSumY += g_lastY;
        g_winSumZ += g_lastZ;
        g_winSumSqX += g_lastX * g_lastX;
        g_winSumSqY += g_lastY * g_lastY;
        g_winSumSqZ += g_lastZ * g_lastZ;
        g_winSampleCount++;
        g_samples++;

        const float dt = 1.0f / (float)g_actualFs;
        const float rc = 1.0f / (2.0f * PI * CONFIG_HPF_CUTOFF_HZ);
        const float hpAlpha = rc / (rc + dt);
        const float hpOut = hpAlpha * (g_hpfPrevOut + g_lastMag - g_hpfPrevIn);
        g_hpfPrevIn = g_lastMag;
        g_hpfPrevOut = hpOut;

        g_fftInX[g_fftIdx] = clampToI16(lroundf(g_lastX * FFT_INPUT_SCALE));
        g_fftInY[g_fftIdx] = clampToI16(lroundf(g_lastY * FFT_INPUT_SCALE));
        g_fftInZ[g_fftIdx] = clampToI16(lroundf(g_lastZ * FFT_INPUT_SCALE));
        g_fftInResultant[g_fftIdx] = clampToI16(lroundf(hpOut * FFT_INPUT_SCALE));
        g_fftIdx++;
        if (g_fftIdx >= g_fftNActive) {
            g_fftIdx = 0;
            if (g_winSampleCount > 0) {
                const float invN = 1.0f / g_winSampleCount;
                const float meanX = g_winSumX * invN;
                const float meanY = g_winSumY * invN;
                const float meanZ = g_winSumZ * invN;
                const float varX = max(0.0f, (g_winSumSqX * invN) - meanX * meanX);
                const float varY = max(0.0f, (g_winSumSqY * invN) - meanY * meanY);
                const float varZ = max(0.0f, (g_winSumSqZ * invN) - meanZ * meanZ);
                g_rmsX = sqrtf(varX);
                g_rmsY = sqrtf(varY);
                g_rmsZ = sqrtf(varZ);
                g_rmsTotal = sqrtf(g_rmsX * g_rmsX + g_rmsY * g_rmsY + g_rmsZ * g_rmsZ);
            }
            g_winSumX = 0.0f;
            g_winSumY = 0.0f;
            g_winSumZ = 0.0f;
            g_winSumSqX = 0.0f;
            g_winSumSqY = 0.0f;
            g_winSumSqZ = 0.0f;
            g_winSampleCount = 0;
            computeFFT();
        }
        const uint32_t procEndUs = micros();
        g_cpuProcAccUs += (uint32_t)(procEndUs - procStartUs);
        g_cpuBudgetAccUs += g_samplePeriodUs;
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
        display.drawStr(0, 10, "GMINSTAL.PL");
        drawWifiIcon(wifiStatus == WL_CONNECTED);
#if ENABLE_ADXL
        if (!g_sensorPresent) {
            display.setFont(u8g2_font_7x13B_tf);
            display.drawStr(0, 30, "BLAD ADXL345");
            display.setFont(u8g2_font_6x10_tf);
            display.drawStr(0, 47, "Sprawdz SPI");
        } else {
        snprintf(buf, sizeof(buf), "RMS: %.2f m/s2", g_rmsTotal);   display.drawStr(0, 23, buf);
        snprintf(buf, sizeof(buf), "Hz:  %.1f",      g_fftPeakHzFilt); display.drawStr(0, 35, buf);
        snprintf(buf, sizeof(buf), "RPM: %.0f",      g_fftPeakRpm);  display.drawStr(0, 47, buf);
        if (g_harmonicAlarmFlags & 0x07u) {
            char hbuf[16] = "HARM:";
            if (g_harmonicAlarmFlags & 1u) strcat(hbuf, "2x");
            if (g_harmonicAlarmFlags & 2u) strcat(hbuf, "3x");
            if (g_harmonicAlarmFlags & 4u) strcat(hbuf, "4x");
            display.drawStr(0, 62, hbuf);
        } else {
            snprintf(buf, sizeof(buf), "CONF: %.0f%%", g_fftPeakConfidencePct); display.drawStr(0, 62, buf);
        }
        }
#else
        snprintf(buf, sizeof(buf), "heap: %u B", ESP.getFreeHeap());
        display.drawStr(0, 28, buf);
#endif
        drawAxesIcon();
        display.sendBuffer();
    }
#else
    delay(1);
#endif

    const uint32_t loopEndUs = micros();
    const uint32_t elapsedUs = (uint32_t)(loopEndUs - g_cpuWindowStartUs);
    if (elapsedUs >= 1000000UL) {
        if (g_cpuBudgetAccUs > 0) {
            g_cpuLoadPct = constrain((100.0f * (float)g_cpuProcAccUs) / (float)g_cpuBudgetAccUs, 0.0f, 100.0f);
        } else {
            g_cpuLoadPct = 0.0f;
        }
        g_cpuProcAccUs = 0;
        g_cpuBudgetAccUs = 0;
        g_cpuWindowStartUs = loopEndUs;
    }
}
