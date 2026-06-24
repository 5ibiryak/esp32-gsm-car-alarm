#include "BatteryMonitor.h"
#include "config.h"
#include "Modem.h"
#include "RuntimeConfig.h"

#include <Wire.h>
#include <math.h>

// INA226 register map
//   0x00 Configuration   (16-bit)
//   0x01 Shunt Voltage   (signed, LSB = 2.5 µV)
//   0x02 Bus Voltage     (unsigned, LSB = 1.25 mV)
//   0x05 Calibration     (we don't program — we derive current from shunt V)
//   0xFE Manufacturer ID (must read 0x5449 = "TI")

static constexpr uint8_t  INA226_REG_CONFIG       = 0x00;
static constexpr uint8_t  INA226_REG_SHUNT        = 0x01;
static constexpr uint8_t  INA226_REG_BUS          = 0x02;
static constexpr uint8_t  INA226_REG_MFG_ID       = 0xFE;
static constexpr uint16_t INA226_MFG_ID_TI        = 0x5449;

// Configuration with 16-sample averaging and continuous shunt+bus mode.
//   AVG=001 (4 samples), VBUSCT=100 (1.1 ms), VSHCT=100 (1.1 ms), MODE=111
// 0x4527 — moderate averaging, ~4.4 ms refresh.
static constexpr uint16_t INA226_DEFAULT_CONFIG   = 0x4527;

static constexpr float    INA226_SHUNT_LSB_V      = 2.5e-6f;     // 2.5 µV / LSB
static constexpr float    INA226_BUS_LSB_V        = 1.25e-3f;    // 1.25 mV / LSB

void BatteryMonitor::begin() {
#if INA226_ENABLED
    Serial.println(F("[INA226] enabled"));
    Serial.print  (F("[INA226] addr=0x"));
    if (INA226_I2C_ADDR < 0x10) Serial.print('0');
    Serial.print  (INA226_I2C_ADDR, HEX);
    Serial.print  (F(" shunt="));
    Serial.print  (INA226_SHUNT_OHMS, 3);
    Serial.println(F(" ohm"));
    Serial.print  (F("[BAT] type="));
    Serial.println(BATTERY_TYPE);
    Serial.print  (F("[BAT] capacity="));
    Serial.print  (runtimeConfig.batteryCapacityAh(), 1);
    Serial.print  (F("Ah reserve="));
    Serial.print  (runtimeConfig.lowBatteryReserveDays(), 1);
    Serial.println(F("d"));
    Serial.print  (F("[BAT] warn="));
    Serial.print  (runtimeConfig.batteryWarnVoltage(), 2);
    Serial.print  (F("V critical="));
    Serial.print  (runtimeConfig.batteryCriticalVoltage(), 2);
    Serial.print  (F("V shutdown="));
    Serial.print  (runtimeConfig.batteryShutdownVoltage(), 2);
    Serial.println(F("V"));
#if BATTERY_STATS_ENABLED
    Serial.print  (F("[BAT-STATS] enabled sample="));
    Serial.print  (BATTERY_STATS_SAMPLE_PERIOD_MS);
    Serial.print  (F("ms log="));
    Serial.print  (BATTERY_STATS_LOG_PERIOD_MS);
    Serial.println(F("ms"));
#endif

    // Wire is shared with MPU-6050. Calling Wire.begin again with the same
    // pins is idempotent on ESP32; we just make sure the bus is up in case
    // BatteryMonitor::begin() runs before Accelerometer::begin().
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);

    _ina226Present = ina226Probe();
    if (_ina226Present) {
        Serial.println(F("[INA226] found at 0x40"));
        ina226Configure();
        ina226PrintBootDiagnostics();
    } else {
        Serial.println(F("[INA226] not found at 0x40"));
    }
#else
    // Legacy ADC path retained for INA226_ENABLED=0 only.
    Serial.println(F("[BAT] INA226 disabled — ADC battery monitor not initialized in this build"));
#endif
}

bool BatteryMonitor::ina226Probe() {
    uint16_t mfg = 0;
    if (!ina226ReadRegister(INA226_REG_MFG_ID, mfg)) return false;
    return mfg == INA226_MFG_ID_TI;
}

void BatteryMonitor::ina226Configure() {
    ina226WriteRegister(INA226_REG_CONFIG, INA226_DEFAULT_CONFIG);
}

bool BatteryMonitor::ina226ReadRegister(uint8_t reg, uint16_t& out) {
    Wire.beginTransmission(INA226_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;

    const uint8_t n = Wire.requestFrom((uint8_t)INA226_I2C_ADDR, (uint8_t)2, (uint8_t)true);
    if (n != 2) return false;

    const uint8_t hi = (uint8_t)Wire.read();
    const uint8_t lo = (uint8_t)Wire.read();
    out = ((uint16_t)hi << 8) | (uint16_t)lo;
    return true;
}

bool BatteryMonitor::ina226WriteRegister(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(INA226_I2C_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    return Wire.endTransmission(true) == 0;
}

void BatteryMonitor::ina226ReadAll() {
    uint16_t shuntRaw = 0;
    uint16_t busRaw   = 0;

    if (!ina226ReadRegister(INA226_REG_SHUNT, shuntRaw)) return;
    if (!ina226ReadRegister(INA226_REG_BUS,   busRaw))   return;

    const int16_t shuntSigned = (int16_t)shuntRaw;
    const float   shuntV      = shuntSigned * INA226_SHUNT_LSB_V;

    _voltageV = busRaw * INA226_BUS_LSB_V;
    _currentA = shuntV / INA226_SHUNT_OHMS;

#if INA226_INVERT_CURRENT
    _currentA = -_currentA;
#endif

    _powerW = _voltageV * _currentA;

#if BATTERY_STATS_ENABLED
    updateStatistics();
#endif
}

void BatteryMonitor::updateStatistics() {
    if (!hasValidMeasurement()) return;

    // Consumption is unsigned even if wiring polarity left _currentA negative.
    const float consumed = fabsf(_currentA);
    const float power    = _voltageV * consumed;

    const uint32_t now = millis();

    if (!_statsInitialized) {
        _avgCurrentFastA      = consumed;
        _avgCurrentSlowA      = consumed;
        _avgPowerFastW        = power;
        _avgPowerSlowW        = power;
        _minVoltageV          = _voltageV;
        _maxVoltageV          = _voltageV;
        _maxCurrentA          = consumed;
        _maxPowerW            = power;
        _lastStatsUpdateMs    = now;
        _statsInitialized     = true;
        return;
    }

    _avgCurrentFastA += BATTERY_AVG_FAST_ALPHA * (consumed - _avgCurrentFastA);
    _avgCurrentSlowA += BATTERY_AVG_SLOW_ALPHA * (consumed - _avgCurrentSlowA);
    _avgPowerFastW   += BATTERY_AVG_FAST_ALPHA * (power    - _avgPowerFastW);
    _avgPowerSlowW   += BATTERY_AVG_SLOW_ALPHA * (power    - _avgPowerSlowW);

    if (_voltageV < _minVoltageV) _minVoltageV = _voltageV;
    if (_voltageV > _maxVoltageV) _maxVoltageV = _voltageV;
    if (consumed  > _maxCurrentA) _maxCurrentA = consumed;
    if (power     > _maxPowerW)   _maxPowerW   = power;

    // Integrate Ah/Wh. Skip implausible intervals (e.g. cold start or a
    // millis() rollover artifact) to prevent counter blow-up.
    const uint32_t deltaMs = now - _lastStatsUpdateMs;
    if (deltaMs > 0 && deltaMs < 60000) {
        const float deltaHours = deltaMs / 3600000.0f;
        _consumedAhSinceBoot += consumed * deltaHours;
        _consumedWhSinceBoot += power    * deltaHours;
    }
    _lastStatsUpdateMs = now;
}

bool BatteryMonitor::hasValidMeasurement() const {
    return _ina226Present && _voltageV >= INA226_MIN_VALID_BUS_VOLTAGE;
}

bool BatteryMonitor::hasValidEstimate() const {
    if (!hasValidMeasurement()) return false;
    return fabsf(_avgCurrentSlowA) >= BATTERY_MIN_CURRENT_FOR_ESTIMATE_A;
}

// Conservative 4S LiFePO4 voltage -> SOC table. Linear interpolation
// between adjacent rows. LiFePO4 is very flat between ~13.0-13.2 V so the
// SOC reading is approximate.
namespace {
struct SocPoint { float v; float p; };
static const SocPoint kLifepoSoc[] = {
    { 13.40f, 100.0f },
    { 13.30f,  90.0f },
    { 13.20f,  80.0f },
    { 13.10f,  70.0f },
    { 13.00f,  60.0f },
    { 12.90f,  50.0f },
    { 12.80f,  40.0f },
    { 12.60f,  30.0f },
    { 12.40f,  20.0f },
    { 12.20f,  10.0f },
    { 11.80f,   5.0f },
    { 11.60f,   0.0f },
};
}  // namespace

float BatteryMonitor::estimateSocPercentFromVoltage(float voltageV) const {
    const size_t n = sizeof(kLifepoSoc) / sizeof(kLifepoSoc[0]);
    if (voltageV >= kLifepoSoc[0].v)        return kLifepoSoc[0].p;
    if (voltageV <= kLifepoSoc[n - 1].v)    return kLifepoSoc[n - 1].p;

    for (size_t i = 1; i < n; ++i) {
        if (voltageV >= kLifepoSoc[i].v) {
            const float v1 = kLifepoSoc[i - 1].v;
            const float v0 = kLifepoSoc[i].v;
            const float p1 = kLifepoSoc[i - 1].p;
            const float p0 = kLifepoSoc[i].p;
            const float t  = (voltageV - v0) / (v1 - v0);
            return p0 + t * (p1 - p0);
        }
    }
    return 0.0f;
}

float BatteryMonitor::getSocPercent() const {
    if (!hasValidMeasurement()) return 0.0f;
    return estimateSocPercentFromVoltage(_voltageV);
}

float BatteryMonitor::getEstimatedRemainingAh() const {
    return runtimeConfig.batteryCapacityAh() * (getSocPercent() / 100.0f);
}

float BatteryMonitor::getReserveNeededAh() const {
    return fabsf(_avgCurrentSlowA) * 24.0f * runtimeConfig.lowBatteryReserveDays();
}

float BatteryMonitor::getEstimatedRemainingDays() const {
    if (!hasValidEstimate()) return NAN;
    const float remainingAh = getEstimatedRemainingAh();
    const float i           = fabsf(_avgCurrentSlowA);
    return remainingAh / i / 24.0f;
}

bool BatteryMonitor::isLowBatteryWarning() const {
    if (!hasValidMeasurement()) return false;
    return _voltageV <= runtimeConfig.batteryWarnVoltage();
}

bool BatteryMonitor::isCriticalBattery() const {
    if (!hasValidMeasurement()) return false;
    return _voltageV <= runtimeConfig.batteryCriticalVoltage();
}

String BatteryMonitor::getStatusText() const {
    if (!_ina226Present) {
        return String("MoskvichAlarm BATTERY: INA226 not available");
    }
    if (!hasValidMeasurement()) {
        return String("MoskvichAlarm BATTERY:\nINA226 detected, but no battery voltage.\nCheck VBS/VIN wiring.");
    }

    const long mA       = (long)lroundf(_currentA        * 1000.0f);
    const long avgSlowmA = (long)lroundf(_avgCurrentSlowA * 1000.0f);

    String t = "MoskvichAlarm BATTERY:\nV=";
    t += String(_voltageV, 2);
    t += "V SOC=";
    t += String((int)lroundf(getSocPercent()));
    t += "%\nI=";
    t += String(mA);
    t += "mA avg=";
    if (hasValidEstimate()) {
        t += String(avgSlowmA);
        t += "mA";
    } else {
        t += "UNKNOWN";
    }
    t += "\nP=";
    t += String(_powerW, 2);
    t += "W\nest=";
    if (hasValidEstimate()) {
        t += String(getEstimatedRemainingDays(), 1);
        t += "d";
    } else {
        t += "UNKNOWN";
    }
    t += "\nused=";
    t += String(_consumedAhSinceBoot, 2);
    t += "Ah\nlow<=";
    t += String(runtimeConfig.batteryCriticalVoltage(), 1);
    t += "V";
    return t;
}

String BatteryMonitor::getExtendedStatusText() const {
    if (!_ina226Present)        return String("MoskvichAlarm BAT DETAIL: INA226 not available");
    if (!hasValidMeasurement()) return String("MoskvichAlarm BAT DETAIL: no battery voltage");

    String t = "MoskvichAlarm BAT DETAIL:\nV=";
    t += String(_voltageV, 2);
    t += "V\nSOC=";
    t += String((int)lroundf(getSocPercent()));
    t += "%\nI=";
    t += String((long)lroundf(_currentA * 1000.0f));
    t += "mA\navgFast=";
    t += String((long)lroundf(_avgCurrentFastA * 1000.0f));
    t += "mA\navgSlow=";
    t += String((long)lroundf(_avgCurrentSlowA * 1000.0f));
    t += "mA\nP=";
    t += String(_powerW, 2);
    t += "W\nusedAh=";
    t += String(_consumedAhSinceBoot, 2);
    t += "\nusedWh=";
    t += String(_consumedWhSinceBoot, 2);
    t += "\nminV=";
    t += String(_minVoltageV, 2);
    t += "\nmaxV=";
    t += String(_maxVoltageV, 2);
    t += "\nmaxI=";
    t += String((long)lroundf(_maxCurrentA * 1000.0f));
    t += "mA\nest=";
    if (hasValidEstimate()) {
        t += String(getEstimatedRemainingDays(), 1);
        t += "d";
    } else {
        t += "UNKNOWN";
    }
    return t;
}

void BatteryMonitor::checkLowBatteryAlert() {
    if (!runtimeConfig.lowBatterySmsEnabled()) return;
    if (!_modem) return;
    // Only act on real measurements — 0 V most likely means wiring fault,
    // not real discharge.
    if (!hasValidMeasurement()) return;

    const bool lowVoltage = (_voltageV <= runtimeConfig.batteryCriticalVoltage());
    bool lowAutonomy      = false;
    if (hasValidEstimate()) {
        lowAutonomy = (getEstimatedRemainingDays() <= runtimeConfig.lowBatteryReserveDays());
    }
    const bool isLow = lowVoltage || lowAutonomy;
    if (!isLow) return;

    const uint32_t now = millis();
    bool shouldAlert = false;
    if (!_lowBatSmsSent) {
        shouldAlert = true;          // first-time trigger after boot
    } else if ((now - _lastLowBatSmsMs) >= runtimeConfig.lowBatterySmsRepeatMs()) {
        shouldAlert = true;          // condition persists across the repeat window
    }
    if (!shouldAlert) return;

    // Critical (voltage <= SHUTDOWN) produces a stronger SMS body.
    const bool isCritical = (_voltageV <= runtimeConfig.batteryShutdownVoltage());
    _modem->queueLowBatterySms(_voltageV, getEstimatedRemainingDays(), isCritical);
    _lastLowBatSmsMs = now;
    _lowBatSmsSent   = true;
}

void BatteryMonitor::update() {
#if INA226_ENABLED
    const uint32_t now = millis();

    if (!_ina226Present) {
        // Re-probe at 30 s intervals — avoids spamming the log if the chip
        // is permanently missing.
        if (_lastNotFoundLogMs != 0 && (now - _lastNotFoundLogMs) < 30000) return;
        _lastNotFoundLogMs = now;

        _ina226Present = ina226Probe();
        if (_ina226Present) {
            Serial.println(F("[INA226] found at 0x40"));
            ina226Configure();
            ina226PrintBootDiagnostics();
        } else {
            Serial.println(F("[INA226] not found at 0x40"));
        }
        return;
    }

    if (_lastReadMs != 0 && (now - _lastReadMs) < BATTERY_STATS_SAMPLE_PERIOD_MS) return;
    _lastReadMs = now;

    ina226ReadAll();

#if DEBUG_BATTERY && !BATTERY_STATS_ENABLED
    // Per-sample low-level log only when stats are off — otherwise we
    // emit the compact [BAT-STATS] every 30 s and the per-sample line
    // would be noise.
    const long mA = (long)lroundf(_currentA * 1000.0f);
    Serial.print  (F("[INA226] V="));    Serial.print(_voltageV, 2);
    Serial.print  (F("V I="));           Serial.print(_currentA, 3);
    Serial.print  (F("A "));             Serial.print(mA);
    Serial.print  (F("mA P="));          Serial.print(_powerW, 2);
    Serial.print  (F("W SOC="));         Serial.print((int)lroundf(getSocPercent()));
    Serial.print  (F("% avgI="));        Serial.print(_avgCurrentSlowA, 3);
    if (hasValidEstimate()) {
        Serial.print  (F("A est="));     Serial.print(getEstimatedRemainingDays(), 1);
        Serial.println(F("d"));
    } else {
        Serial.println(F("A est=UNKNOWN"));
    }
#endif

#if BATTERY_STATS_ENABLED
    logStatsLine();
    checkLowCurrentWarning();
#endif

#if DEBUG_INA226_RAW
    if (_lastRawDiagMs == 0 || (now - _lastRawDiagMs) >= runtimeConfig.inaRawLogIntervalMs()) {
        _lastRawDiagMs = now;
        ina226PrintRawDiagnostics();
    }
#endif

    checkLowBatteryAlert();
#endif
}

void BatteryMonitor::logStatsLine() {
    if (!hasValidMeasurement()) return;
    const uint32_t now = millis();
    if (_lastStatsLogMs != 0 && (now - _lastStatsLogMs) < BATTERY_STATS_LOG_PERIOD_MS) return;
    _lastStatsLogMs = now;

    Serial.print  (F("[BAT-STATS] V="));
    Serial.print  (_voltageV, 2);
    Serial.print  (F("V SOC="));
    Serial.print  ((int)lroundf(getSocPercent()));
    Serial.print  (F("% I="));
    Serial.print  ((long)lroundf(_currentA * 1000.0f));
    Serial.print  (F("mA avgFast="));
    Serial.print  ((long)lroundf(_avgCurrentFastA * 1000.0f));
    Serial.print  (F("mA avgSlow="));
    Serial.print  ((long)lroundf(_avgCurrentSlowA * 1000.0f));
    Serial.print  (F("mA P="));
    Serial.print  (_powerW, 2);
    Serial.print  (F("W used="));
    Serial.print  (_consumedAhSinceBoot, 2);
    Serial.print  (F("Ah/"));
    Serial.print  (_consumedWhSinceBoot, 2);
    Serial.print  (F("Wh est="));
    if (hasValidEstimate()) {
        Serial.print  (getEstimatedRemainingDays(), 1);
        Serial.println(F("d"));
    } else {
        Serial.println(F("UNKNOWN"));
    }
}

void BatteryMonitor::checkLowCurrentWarning() {
    if (!hasValidMeasurement()) {
        _lowCurrentSinceMs = 0;
        return;
    }
    if (_avgCurrentSlowA >= BATTERY_MIN_CURRENT_FOR_ESTIMATE_A) {
        _lowCurrentSinceMs = 0;
        _lastUsbWarningMs  = 0;
        return;
    }

    const uint32_t now = millis();
    if (_lowCurrentSinceMs == 0) _lowCurrentSinceMs = now;
    if ((now - _lowCurrentSinceMs) < 60000UL) return;
    if (_lastUsbWarningMs != 0 && (now - _lastUsbWarningMs) < 60000UL) return;

    _lastUsbWarningMs = now;
    Serial.println(F("[BAT-STATS] warning: current too low for estimate. If USB is connected, current may bypass INA226."));
}

void BatteryMonitor::ina226PrintBootDiagnostics() {
    uint16_t mfg = 0, die = 0, cfg = 0, busRaw = 0;
    int16_t  shuntRaw = 0;

    if (ina226ReadRegister(INA226_REG_MFG_ID, mfg)) {
        Serial.print  (F("[INA226] manufacturer ID=0x"));
        Serial.println(mfg, HEX);
    }
    {
        uint16_t d = 0;
        if (ina226ReadRegister(0xFF, d)) {
            die = d;
            Serial.print  (F("[INA226] die ID=0x"));
            Serial.println(die, HEX);
        }
    }
    if (ina226ReadRegister(INA226_REG_CONFIG, cfg)) {
        Serial.print  (F("[INA226] config=0x"));
        Serial.println(cfg, HEX);
    }

    uint16_t tmp = 0;
    if (ina226ReadRegister(INA226_REG_BUS, tmp)) {
        busRaw = tmp;
        const float vbus = busRaw * INA226_BUS_LSB_V;
        Serial.print  (F("[INA226] bus_raw="));
        Serial.print  (busRaw);
        Serial.print  (F(" bus_V="));
        Serial.println(vbus, 3);
        if (vbus < INA226_MIN_VALID_BUS_VOLTAGE) {
            Serial.println(F("[INA226] warning: bus voltage is 0V/too low. Check VIN+/VIN- wiring."));
            Serial.println(F("[INA226] SDA/SCL only confirm I2C, not battery measurement."));
        }
    }
    if (ina226ReadRegister(INA226_REG_SHUNT, tmp)) {
        shuntRaw = (int16_t)tmp;
        const float shuntMv = shuntRaw * INA226_SHUNT_LSB_V * 1000.0f;
        Serial.print  (F("[INA226] shunt_raw="));
        Serial.print  (shuntRaw);
        Serial.print  (F(" shunt_mV="));
        Serial.println(shuntMv, 4);
    }
}

void BatteryMonitor::ina226PrintRawDiagnostics() {
    uint16_t cfg = 0, busRaw = 0, shuntRawU = 0;
    if (ina226ReadRegister(INA226_REG_CONFIG, cfg)) {
        Serial.print  (F("[INA226-RAW] config=0x"));
        Serial.println(cfg, HEX);
    }
    const bool shOk = ina226ReadRegister(INA226_REG_SHUNT, shuntRawU);
    const bool buOk = ina226ReadRegister(INA226_REG_BUS,   busRaw);
    if (shOk && buOk) {
        const int16_t shuntRaw = (int16_t)shuntRawU;
        Serial.print  (F("[INA226-RAW] shunt_raw="));
        Serial.print  (shuntRaw);
        Serial.print  (F(" bus_raw="));
        Serial.println(busRaw);

        Serial.print  (F("[INA226-RAW] shunt_mV="));
        Serial.print  (shuntRaw * INA226_SHUNT_LSB_V * 1000.0f, 4);
        Serial.print  (F(" bus_V="));
        Serial.println(busRaw * INA226_BUS_LSB_V, 4);
    }
}
