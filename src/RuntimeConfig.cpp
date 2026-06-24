#include "RuntimeConfig.h"

#include <Preferences.h>
#include <math.h>

// Single global instance — declared `extern` in the header so subsystems
// can reach it without plumbing a pointer through every constructor.
RuntimeConfig runtimeConfig;

// Out-of-line definition of the static name table.
const RuntimeConfig::FieldNamesType RuntimeConfig::FieldNames;

static constexpr const char* kNvsNamespace = "runtime_cfg";

namespace {

// Validation helpers.
bool inRange(float v, float lo, float hi) {
    return isfinite(v) && v >= lo && v <= hi;
}
bool inRangeL(long v, long lo, long hi) {
    return v >= lo && v <= hi;
}

}  // namespace

void RuntimeConfig::logInvalid(const char* field) const {
    Serial.print  (F("[CFG] invalid value for "));
    Serial.print  (field);
    Serial.println(F(", using default"));
}

// ----- Defaults / load / save -----------------------------------------------

void RuntimeConfig::applyDefaults() {
    _motionDeltaThresholdG            = ACCEL_MOTION_DELTA_G;
    _motionConfirmSamples             = 2;
    _motionCooldownMs                 = 3000UL;

    _armingDelayMs                    = ALARM_ARMING_DELAY_MS;
    _armedSensorGraceMs               = ALARM_ARMED_SENSOR_GRACE_MS;

    _sirenAlarmContinuousMs           = SIREN_ALARM_CONTINUOUS_MS;
    _sirenAfterTimeoutReminderEnabled = SIREN_AFTER_TIMEOUT_REMINDER_ENABLED != 0;
    _sirenReminderOnMs                = SIREN_REMINDER_ON_MS;
    _sirenReminderOffMs               = SIREN_REMINDER_OFF_MS;

    _lowBatterySmsEnabled             = LOW_BATTERY_SMS_ENABLED != 0;
    _batteryCapacityAh                = BATTERY_CAPACITY_AH;
    _batteryWarnVoltage               = BAT_VOLTAGE_WARN;
    _batteryCriticalVoltage           = BAT_VOLTAGE_CRITICAL;
    _batteryShutdownVoltage           = BAT_VOLTAGE_SHUTDOWN;
    _lowBatteryReserveDays            = LOW_BATTERY_RESERVE_DAYS;
    _lowBatterySmsRepeatHours         = (uint16_t)(LOW_BATTERY_SMS_REPEAT_MS / 3600000UL);
    if (_lowBatterySmsRepeatHours == 0) _lowBatterySmsRepeatHours = 24;

    _stateLogIntervalMs               = STATE_LOG_PERIOD_MS;
    _inaRawLogIntervalMs              = 10000UL;

    _otaActiveWindowMs                = OTA_ACTIVE_WINDOW_MS;

    _smartMotionEnabled       = true;
    _motionCalibrationMs      = 30000UL;
    _motionBaselineAlphaQuiet = 0.002f;
    _impactThresholdG         = 0.080f;
    _impactConfirmWindowMs    = 800UL;
    _impactMinPeaks           = 1;
    _tiltThresholdDeg         = 4.0f;
    _tiltConfirmMs            = 3000UL;
    _moveRmsThresholdG        = 0.020f;
    _moveConfirmMs            = 5000UL;
    _moveMinActivePercent     = 40;
    _motionEarlyWindowMs      = 600000UL;
    _motionEarlyMultiplier    = 1.5f;
}

void RuntimeConfig::begin() {
    Serial.println(F("[CFG] loading runtime config"));
    applyDefaults();
    if (loadFromNvs()) {
        Serial.println(F("[CFG] loaded from NVS"));
    } else {
        Serial.println(F("[CFG] using defaults"));
    }
}

bool RuntimeConfig::loadFromNvs() {
    Preferences p;
    if (!p.begin(kNvsNamespace, /*readOnly*/true)) return false;
    // Sentinel key — if absent, no saved config yet.
    if (!p.isKey("v")) {
        p.end();
        return false;
    }

    // Helper macros: read; if value falls outside valid range, log and
    // keep the default already in place. `name` is the bare field token
    // (e.g. motionDeltaThresholdG). #name turns it into the NVS key
    // string; _##name names the private member variable. Using the bare
    // token directly would resolve to the same-named getter method —
    // that was the original compile failure.
#define LOAD_FLOAT(name, lo, hi)                              \
    do {                                                      \
        const char* key = #name;                              \
        if (p.isKey(key)) {                                   \
            const float v = p.getFloat(key, NAN);             \
            if (inRange(v, lo, hi)) _##name = v;              \
            else                    logInvalid(key);          \
        }                                                     \
    } while (0)

#define LOAD_U32(name, lo, hi)                                \
    do {                                                      \
        const char* key = #name;                              \
        if (p.isKey(key)) {                                   \
            const long v = (long)p.getULong(key, 0);          \
            if (inRangeL(v, (long)(lo), (long)(hi))) _##name = (uint32_t)v; \
            else                                     logInvalid(key); \
        }                                                     \
    } while (0)

#define LOAD_U16(name, lo, hi)                                \
    do {                                                      \
        const char* key = #name;                              \
        if (p.isKey(key)) {                                   \
            const long v = (long)p.getUShort(key, 0);         \
            if (inRangeL(v, (long)(lo), (long)(hi))) _##name = (uint16_t)v; \
            else                                     logInvalid(key); \
        }                                                     \
    } while (0)

#define LOAD_U8(name, lo, hi)                                 \
    do {                                                      \
        const char* key = #name;                              \
        if (p.isKey(key)) {                                   \
            const long v = (long)p.getUChar(key, 0);          \
            if (inRangeL(v, (long)(lo), (long)(hi))) _##name = (uint8_t)v; \
            else                                     logInvalid(key); \
        }                                                     \
    } while (0)

#define LOAD_BOOL(name)                                       \
    do {                                                      \
        const char* key = #name;                              \
        if (p.isKey(key)) _##name = p.getBool(key, _##name);  \
    } while (0)

    LOAD_FLOAT(motionDeltaThresholdG,    MOTION_THRESHOLD_MIN_G, MOTION_THRESHOLD_MAX_G);
    LOAD_U8   (motionConfirmSamples,     MOTION_SAMPLES_MIN,     MOTION_SAMPLES_MAX);
    LOAD_U32  (motionCooldownMs,         MOTION_COOLDOWN_MIN_MS, MOTION_COOLDOWN_MAX_MS);

    LOAD_U32  (armingDelayMs,            ARMING_DELAY_MIN_MS,    ARMING_DELAY_MAX_MS);
    LOAD_U32  (armedSensorGraceMs,       ARMED_GRACE_MIN_MS,     ARMED_GRACE_MAX_MS);

    LOAD_U32  (sirenAlarmContinuousMs,   SIREN_CONT_MIN_MS,      SIREN_CONT_MAX_MS);
    LOAD_BOOL (sirenAfterTimeoutReminderEnabled);
    LOAD_U32  (sirenReminderOnMs,        SIREN_REM_ON_MIN_MS,    SIREN_REM_ON_MAX_MS);
    LOAD_U32  (sirenReminderOffMs,       SIREN_REM_OFF_MIN_MS,   SIREN_REM_OFF_MAX_MS);

    LOAD_BOOL (lowBatterySmsEnabled);
    LOAD_FLOAT(batteryCapacityAh,        BAT_CAP_MIN_AH,         BAT_CAP_MAX_AH);
    LOAD_FLOAT(batteryWarnVoltage,       BAT_V_LOW,              BAT_V_HIGH);
    LOAD_FLOAT(batteryCriticalVoltage,   BAT_V_LOW,              BAT_V_HIGH);
    LOAD_FLOAT(batteryShutdownVoltage,   BAT_V_SHUTDOWN_LOW,     BAT_V_HIGH);
    LOAD_FLOAT(lowBatteryReserveDays,    BAT_RESERVE_MIN_D,      BAT_RESERVE_MAX_D);
    LOAD_U16  (lowBatterySmsRepeatHours, BAT_REPEAT_MIN_H,       BAT_REPEAT_MAX_H);

    LOAD_U32  (stateLogIntervalMs,       STATE_LOG_MIN_MS,       STATE_LOG_MAX_MS);
    LOAD_U32  (inaRawLogIntervalMs,      INA_RAW_MIN_MS,         INA_RAW_MAX_MS);

    LOAD_U32  (otaActiveWindowMs,        OTA_WINDOW_MIN_MS,      OTA_WINDOW_MAX_MS);

    LOAD_BOOL (smartMotionEnabled);
    LOAD_U32  (motionCalibrationMs,      MOTION_CALIB_MIN_MS,    MOTION_CALIB_MAX_MS);
    LOAD_FLOAT(motionBaselineAlphaQuiet, BASELINE_ALPHA_MIN,     BASELINE_ALPHA_MAX);
    LOAD_FLOAT(impactThresholdG,         IMPACT_THR_MIN_G,       IMPACT_THR_MAX_G);
    LOAD_U32  (impactConfirmWindowMs,    IMPACT_WIN_MIN_MS,      IMPACT_WIN_MAX_MS);
    LOAD_U8   (impactMinPeaks,           IMPACT_PEAKS_MIN,       IMPACT_PEAKS_MAX);
    LOAD_FLOAT(tiltThresholdDeg,         TILT_THR_MIN_DEG,       TILT_THR_MAX_DEG);
    LOAD_U32  (tiltConfirmMs,            TILT_CONFIRM_MIN_MS,    TILT_CONFIRM_MAX_MS);
    LOAD_FLOAT(moveRmsThresholdG,        MOVE_RMS_MIN_G,         MOVE_RMS_MAX_G);
    LOAD_U32  (moveConfirmMs,            MOVE_CONFIRM_MIN_MS,    MOVE_CONFIRM_MAX_MS);
    LOAD_U8   (moveMinActivePercent,     MOVE_ACTIVE_PCT_MIN,    MOVE_ACTIVE_PCT_MAX);
    LOAD_U32  (motionEarlyWindowMs,      EARLY_WINDOW_MIN_MS,    EARLY_WINDOW_MAX_MS);
    LOAD_FLOAT(motionEarlyMultiplier,    EARLY_MULT_MIN,         EARLY_MULT_MAX);

#undef LOAD_FLOAT
#undef LOAD_U32
#undef LOAD_U16
#undef LOAD_U8
#undef LOAD_BOOL

    p.end();
    return true;
}

bool RuntimeConfig::saveToNvs() {
    Preferences p;
    if (!p.begin(kNvsNamespace, /*readOnly*/false)) return false;

    p.putUChar("v", 1);                                   // sentinel / version
    p.putFloat (FieldNames.motionDeltaThresholdG,            _motionDeltaThresholdG);
    p.putUChar (FieldNames.motionConfirmSamples,             _motionConfirmSamples);
    p.putULong (FieldNames.motionCooldownMs,                 _motionCooldownMs);
    p.putULong (FieldNames.armingDelayMs,                    _armingDelayMs);
    p.putULong (FieldNames.armedSensorGraceMs,               _armedSensorGraceMs);
    p.putULong (FieldNames.sirenAlarmContinuousMs,           _sirenAlarmContinuousMs);
    p.putBool  (FieldNames.sirenAfterTimeoutReminderEnabled, _sirenAfterTimeoutReminderEnabled);
    p.putULong (FieldNames.sirenReminderOnMs,                _sirenReminderOnMs);
    p.putULong (FieldNames.sirenReminderOffMs,               _sirenReminderOffMs);
    p.putBool  (FieldNames.lowBatterySmsEnabled,             _lowBatterySmsEnabled);
    p.putFloat (FieldNames.batteryCapacityAh,                _batteryCapacityAh);
    p.putFloat (FieldNames.batteryWarnVoltage,               _batteryWarnVoltage);
    p.putFloat (FieldNames.batteryCriticalVoltage,           _batteryCriticalVoltage);
    p.putFloat (FieldNames.batteryShutdownVoltage,           _batteryShutdownVoltage);
    p.putFloat (FieldNames.lowBatteryReserveDays,            _lowBatteryReserveDays);
    p.putUShort(FieldNames.lowBatterySmsRepeatHours,         _lowBatterySmsRepeatHours);
    p.putULong (FieldNames.stateLogIntervalMs,               _stateLogIntervalMs);
    p.putULong (FieldNames.inaRawLogIntervalMs,              _inaRawLogIntervalMs);
    p.putULong (FieldNames.otaActiveWindowMs,                _otaActiveWindowMs);

    p.putBool  (FieldNames.smartMotionEnabled,               _smartMotionEnabled);
    p.putULong (FieldNames.motionCalibrationMs,              _motionCalibrationMs);
    p.putFloat (FieldNames.motionBaselineAlphaQuiet,         _motionBaselineAlphaQuiet);
    p.putFloat (FieldNames.impactThresholdG,                 _impactThresholdG);
    p.putULong (FieldNames.impactConfirmWindowMs,            _impactConfirmWindowMs);
    p.putUChar (FieldNames.impactMinPeaks,                   _impactMinPeaks);
    p.putFloat (FieldNames.tiltThresholdDeg,                 _tiltThresholdDeg);
    p.putULong (FieldNames.tiltConfirmMs,                    _tiltConfirmMs);
    p.putFloat (FieldNames.moveRmsThresholdG,                _moveRmsThresholdG);
    p.putULong (FieldNames.moveConfirmMs,                    _moveConfirmMs);
    p.putUChar (FieldNames.moveMinActivePercent,             _moveMinActivePercent);
    p.putULong (FieldNames.motionEarlyWindowMs,              _motionEarlyWindowMs);
    p.putFloat (FieldNames.motionEarlyMultiplier,            _motionEarlyMultiplier);
    p.end();
    Serial.println(F("[CFG] saved"));
    return true;
}

void RuntimeConfig::resetToDefaults() {
    applyDefaults();
    saveToNvs();
    Serial.println(F("[CFG] reset to defaults"));
}

// ----- Validating setters ---------------------------------------------------

bool RuntimeConfig::setMotionDeltaThresholdG(float v) {
    if (!inRange(v, MOTION_THRESHOLD_MIN_G, MOTION_THRESHOLD_MAX_G)) return false;
    _motionDeltaThresholdG = v; return true;
}
bool RuntimeConfig::setMotionConfirmSamples(int v) {
    if (!inRangeL(v, MOTION_SAMPLES_MIN, MOTION_SAMPLES_MAX)) return false;
    _motionConfirmSamples = (uint8_t)v; return true;
}
bool RuntimeConfig::setMotionCooldownMs(long v) {
    if (!inRangeL(v, (long)MOTION_COOLDOWN_MIN_MS, (long)MOTION_COOLDOWN_MAX_MS)) return false;
    _motionCooldownMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setArmingDelayMs(long v) {
    if (!inRangeL(v, (long)ARMING_DELAY_MIN_MS, (long)ARMING_DELAY_MAX_MS)) return false;
    _armingDelayMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setArmedSensorGraceMs(long v) {
    if (!inRangeL(v, (long)ARMED_GRACE_MIN_MS, (long)ARMED_GRACE_MAX_MS)) return false;
    _armedSensorGraceMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setSirenAlarmContinuousMs(long v) {
    if (!inRangeL(v, (long)SIREN_CONT_MIN_MS, (long)SIREN_CONT_MAX_MS)) return false;
    _sirenAlarmContinuousMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setSirenAfterTimeoutReminderEnabled(bool v) {
    _sirenAfterTimeoutReminderEnabled = v; return true;
}
bool RuntimeConfig::setSirenReminderOnMs(long v) {
    if (!inRangeL(v, (long)SIREN_REM_ON_MIN_MS, (long)SIREN_REM_ON_MAX_MS)) return false;
    _sirenReminderOnMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setSirenReminderOffMs(long v) {
    if (!inRangeL(v, (long)SIREN_REM_OFF_MIN_MS, (long)SIREN_REM_OFF_MAX_MS)) return false;
    _sirenReminderOffMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setLowBatterySmsEnabled(bool v) {
    _lowBatterySmsEnabled = v; return true;
}
bool RuntimeConfig::setBatteryCapacityAh(float v) {
    if (!inRange(v, BAT_CAP_MIN_AH, BAT_CAP_MAX_AH)) return false;
    _batteryCapacityAh = v; return true;
}
bool RuntimeConfig::setBatteryWarnVoltage(float v) {
    if (!inRange(v, 10.0f, BAT_V_HIGH)) return false;
    _batteryWarnVoltage = v; return true;
}
bool RuntimeConfig::setBatteryCriticalVoltage(float v) {
    if (!inRange(v, 10.0f, BAT_V_HIGH)) return false;
    _batteryCriticalVoltage = v; return true;
}
bool RuntimeConfig::setBatteryShutdownVoltage(float v) {
    if (!inRange(v, BAT_V_SHUTDOWN_LOW, BAT_V_HIGH)) return false;
    _batteryShutdownVoltage = v; return true;
}
bool RuntimeConfig::setLowBatteryReserveDays(float v) {
    if (!inRange(v, BAT_RESERVE_MIN_D, BAT_RESERVE_MAX_D)) return false;
    _lowBatteryReserveDays = v; return true;
}
bool RuntimeConfig::setLowBatterySmsRepeatHours(int v) {
    if (!inRangeL(v, BAT_REPEAT_MIN_H, BAT_REPEAT_MAX_H)) return false;
    _lowBatterySmsRepeatHours = (uint16_t)v; return true;
}
bool RuntimeConfig::setStateLogIntervalMs(long v) {
    if (!inRangeL(v, (long)STATE_LOG_MIN_MS, (long)STATE_LOG_MAX_MS)) return false;
    _stateLogIntervalMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setInaRawLogIntervalMs(long v) {
    if (!inRangeL(v, (long)INA_RAW_MIN_MS, (long)INA_RAW_MAX_MS)) return false;
    _inaRawLogIntervalMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setOtaActiveWindowMs(long v) {
    if (!inRangeL(v, (long)OTA_WINDOW_MIN_MS, (long)OTA_WINDOW_MAX_MS)) return false;
    _otaActiveWindowMs = (uint32_t)v; return true;
}

bool RuntimeConfig::setSmartMotionEnabled(bool v) {
    _smartMotionEnabled = v; return true;
}
bool RuntimeConfig::setMotionCalibrationMs(long v) {
    if (!inRangeL(v, (long)MOTION_CALIB_MIN_MS, (long)MOTION_CALIB_MAX_MS)) return false;
    _motionCalibrationMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setMotionBaselineAlphaQuiet(float v) {
    if (!inRange(v, BASELINE_ALPHA_MIN, BASELINE_ALPHA_MAX)) return false;
    _motionBaselineAlphaQuiet = v; return true;
}
bool RuntimeConfig::setImpactThresholdG(float v) {
    if (!inRange(v, IMPACT_THR_MIN_G, IMPACT_THR_MAX_G)) return false;
    _impactThresholdG = v; return true;
}
bool RuntimeConfig::setImpactConfirmWindowMs(long v) {
    if (!inRangeL(v, (long)IMPACT_WIN_MIN_MS, (long)IMPACT_WIN_MAX_MS)) return false;
    _impactConfirmWindowMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setImpactMinPeaks(int v) {
    if (!inRangeL(v, IMPACT_PEAKS_MIN, IMPACT_PEAKS_MAX)) return false;
    _impactMinPeaks = (uint8_t)v; return true;
}
bool RuntimeConfig::setTiltThresholdDeg(float v) {
    if (!inRange(v, TILT_THR_MIN_DEG, TILT_THR_MAX_DEG)) return false;
    _tiltThresholdDeg = v; return true;
}
bool RuntimeConfig::setTiltConfirmMs(long v) {
    if (!inRangeL(v, (long)TILT_CONFIRM_MIN_MS, (long)TILT_CONFIRM_MAX_MS)) return false;
    _tiltConfirmMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setMoveRmsThresholdG(float v) {
    if (!inRange(v, MOVE_RMS_MIN_G, MOVE_RMS_MAX_G)) return false;
    _moveRmsThresholdG = v; return true;
}
bool RuntimeConfig::setMoveConfirmMs(long v) {
    if (!inRangeL(v, (long)MOVE_CONFIRM_MIN_MS, (long)MOVE_CONFIRM_MAX_MS)) return false;
    _moveConfirmMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setMoveMinActivePercent(int v) {
    if (!inRangeL(v, MOVE_ACTIVE_PCT_MIN, MOVE_ACTIVE_PCT_MAX)) return false;
    _moveMinActivePercent = (uint8_t)v; return true;
}
bool RuntimeConfig::setMotionEarlyWindowMs(long v) {
    if (!inRangeL(v, (long)EARLY_WINDOW_MIN_MS, (long)EARLY_WINDOW_MAX_MS)) return false;
    _motionEarlyWindowMs = (uint32_t)v; return true;
}
bool RuntimeConfig::setMotionEarlyMultiplier(float v) {
    if (!inRange(v, EARLY_MULT_MIN, EARLY_MULT_MAX)) return false;
    _motionEarlyMultiplier = v; return true;
}

// ----- Output ---------------------------------------------------------------

String RuntimeConfig::toJson() const {
    String j;
    j.reserve(700);
    j += '{';
    j += "\"motionDeltaThresholdG\":";       j += String(_motionDeltaThresholdG, 3); j += ',';
    j += "\"motionConfirmSamples\":";        j += _motionConfirmSamples; j += ',';
    j += "\"motionCooldownMs\":";            j += _motionCooldownMs; j += ',';
    j += "\"armingDelayMs\":";               j += _armingDelayMs; j += ',';
    j += "\"armedSensorGraceMs\":";          j += _armedSensorGraceMs; j += ',';
    j += "\"sirenAlarmContinuousMs\":";      j += _sirenAlarmContinuousMs; j += ',';
    j += "\"sirenAfterTimeoutReminderEnabled\":"; j += (_sirenAfterTimeoutReminderEnabled ? "true":"false"); j += ',';
    j += "\"sirenReminderOnMs\":";           j += _sirenReminderOnMs; j += ',';
    j += "\"sirenReminderOffMs\":";          j += _sirenReminderOffMs; j += ',';
    j += "\"lowBatterySmsEnabled\":";        j += (_lowBatterySmsEnabled ? "true":"false"); j += ',';
    j += "\"batteryCapacityAh\":";           j += String(_batteryCapacityAh, 1); j += ',';
    j += "\"batteryWarnVoltage\":";          j += String(_batteryWarnVoltage, 2); j += ',';
    j += "\"batteryCriticalVoltage\":";      j += String(_batteryCriticalVoltage, 2); j += ',';
    j += "\"batteryShutdownVoltage\":";      j += String(_batteryShutdownVoltage, 2); j += ',';
    j += "\"lowBatteryReserveDays\":";       j += String(_lowBatteryReserveDays, 1); j += ',';
    j += "\"lowBatterySmsRepeatHours\":";    j += _lowBatterySmsRepeatHours; j += ',';
    j += "\"stateLogIntervalMs\":";          j += _stateLogIntervalMs; j += ',';
    j += "\"inaRawLogIntervalMs\":";         j += _inaRawLogIntervalMs; j += ',';
    j += "\"otaActiveWindowMs\":";           j += _otaActiveWindowMs; j += ',';
    j += "\"smartMotionEnabled\":";          j += (_smartMotionEnabled ? "true":"false"); j += ',';
    j += "\"motionCalibrationMs\":";         j += _motionCalibrationMs; j += ',';
    j += "\"motionBaselineAlphaQuiet\":";    j += String(_motionBaselineAlphaQuiet, 4); j += ',';
    j += "\"impactThresholdG\":";            j += String(_impactThresholdG, 3); j += ',';
    j += "\"impactConfirmWindowMs\":";       j += _impactConfirmWindowMs; j += ',';
    j += "\"impactMinPeaks\":";              j += _impactMinPeaks; j += ',';
    j += "\"tiltThresholdDeg\":";            j += String(_tiltThresholdDeg, 1); j += ',';
    j += "\"tiltConfirmMs\":";               j += _tiltConfirmMs; j += ',';
    j += "\"moveRmsThresholdG\":";           j += String(_moveRmsThresholdG, 3); j += ',';
    j += "\"moveConfirmMs\":";               j += _moveConfirmMs; j += ',';
    j += "\"moveMinActivePercent\":";        j += _moveMinActivePercent; j += ',';
    j += "\"motionEarlyWindowMs\":";         j += _motionEarlyWindowMs; j += ',';
    j += "\"motionEarlyMultiplier\":";       j += String(_motionEarlyMultiplier, 2);
    j += '}';
    return j;
}

String RuntimeConfig::toShortSmsSummary() const {
    String s;
    s.reserve(140);
    s += "CFG:\nmotion=";   s += String(_motionDeltaThresholdG, 3); s += "g";
    s += "\narmDelay=";     s += (_armingDelayMs / 1000UL);          s += "s";
    s += "\nsiren=";        s += (_sirenAlarmContinuousMs / 1000UL); s += "s";
    s += "\nbatCrit=";      s += String(_batteryCriticalVoltage, 1); s += "V";
    s += "\nbatSms=";       s += (_lowBatterySmsEnabled ? "on" : "off");
    s += "\nota=";          s += (_otaActiveWindowMs / 60000UL);     s += "m";
    return s;
}
