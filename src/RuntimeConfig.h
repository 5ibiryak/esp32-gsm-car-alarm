#pragma once
#include <Arduino.h>
#include "config.h"

// Runtime-tunable configuration backed by NVS (Preferences namespace
// "runtime_cfg"). Defaults mirror the compile-time constants in config.h
// so behavior on a fresh device matches the firmware build.
//
// Pins, modem UART, I2C addresses, SMS whitelist, OTA AP credentials, and
// partition layout are deliberately NOT exposed — those stay compile-time.
//
// Logs:
//   [CFG] loading runtime config
//   [CFG] using defaults
//   [CFG] loaded from NVS
//   [CFG] saved
//   [CFG] invalid value for <field>, using default
//   [CFG] reset to defaults
class RuntimeConfig {
public:
    // Field metadata. Used by validation, web form rendering, and the
    // settings JSON dump. Mirrors the public field names so callers can
    // refer to them by string.
    struct FieldNamesType {
        // 1. Motion — legacy single-threshold
        const char* motionDeltaThresholdG     = "motionDeltaThresholdG";
        const char* motionConfirmSamples      = "motionConfirmSamples";
        const char* motionCooldownMs          = "motionCooldownMs";
        // 1b. Motion — smart detector
        const char* smartMotionEnabled        = "smartMotionEnabled";
        const char* motionCalibrationMs       = "motionCalibrationMs";
        const char* motionBaselineAlphaQuiet  = "motionBaselineAlphaQuiet";
        const char* impactThresholdG          = "impactThresholdG";
        const char* impactConfirmWindowMs     = "impactConfirmWindowMs";
        const char* impactMinPeaks            = "impactMinPeaks";
        const char* tiltThresholdDeg          = "tiltThresholdDeg";
        const char* tiltConfirmMs             = "tiltConfirmMs";
        const char* moveRmsThresholdG         = "moveRmsThresholdG";
        const char* moveConfirmMs             = "moveConfirmMs";
        const char* moveMinActivePercent      = "moveMinActivePercent";
        const char* motionEarlyWindowMs       = "motionEarlyWindowMs";
        const char* motionEarlyMultiplier     = "motionEarlyMultiplier";
        // 2. Arming
        const char* armingDelayMs             = "armingDelayMs";
        const char* armedSensorGraceMs        = "armedSensorGraceMs";
        // 3. Siren
        const char* sirenAlarmContinuousMs    = "sirenAlarmContinuousMs";
        const char* sirenAfterTimeoutReminderEnabled
                                              = "sirenAfterTimeoutReminderEnabled";
        const char* sirenReminderOnMs         = "sirenReminderOnMs";
        const char* sirenReminderOffMs        = "sirenReminderOffMs";
        // 4. Battery
        const char* lowBatterySmsEnabled      = "lowBatterySmsEnabled";
        const char* batteryCapacityAh         = "batteryCapacityAh";
        const char* batteryWarnVoltage        = "batteryWarnVoltage";
        const char* batteryCriticalVoltage    = "batteryCriticalVoltage";
        const char* batteryShutdownVoltage    = "batteryShutdownVoltage";
        const char* lowBatteryReserveDays     = "lowBatteryReserveDays";
        const char* lowBatterySmsRepeatHours  = "lowBatterySmsRepeatHours";
        // 5. Logging
        const char* stateLogIntervalMs        = "stateLogIntervalMs";
        const char* inaRawLogIntervalMs       = "inaRawLogIntervalMs";
        // 6. OTA
        const char* otaActiveWindowMs         = "otaActiveWindowMs";
    };
    static const FieldNamesType FieldNames;

    // Bounds (min/max). Defaults are compile-time fallbacks. Keep in sync
    // with the web form's HTML attributes.
    static constexpr float    MOTION_THRESHOLD_MIN_G  = 0.005f;
    static constexpr float    MOTION_THRESHOLD_MAX_G  = 0.300f;
    static constexpr uint8_t  MOTION_SAMPLES_MIN      = 1;
    static constexpr uint8_t  MOTION_SAMPLES_MAX      = 10;
    static constexpr uint32_t MOTION_COOLDOWN_MIN_MS  = 0;
    static constexpr uint32_t MOTION_COOLDOWN_MAX_MS  = 60000UL;
    static constexpr uint32_t ARMING_DELAY_MIN_MS     = 0;
    static constexpr uint32_t ARMING_DELAY_MAX_MS     = 120000UL;
    static constexpr uint32_t ARMED_GRACE_MIN_MS      = 0;
    static constexpr uint32_t ARMED_GRACE_MAX_MS      = 30000UL;
    static constexpr uint32_t SIREN_CONT_MIN_MS       = 10000UL;
    static constexpr uint32_t SIREN_CONT_MAX_MS       = 600000UL;
    static constexpr uint32_t SIREN_REM_ON_MIN_MS     = 50;
    static constexpr uint32_t SIREN_REM_ON_MAX_MS     = 5000UL;
    static constexpr uint32_t SIREN_REM_OFF_MIN_MS    = 1000UL;
    static constexpr uint32_t SIREN_REM_OFF_MAX_MS    = 60000UL;
    static constexpr float    BAT_CAP_MIN_AH          = 1.0f;
    static constexpr float    BAT_CAP_MAX_AH          = 300.0f;
    static constexpr float    BAT_V_LOW               = 9.0f;
    static constexpr float    BAT_V_HIGH              = 15.0f;
    static constexpr float    BAT_V_SHUTDOWN_LOW      = 9.0f;
    static constexpr float    BAT_RESERVE_MIN_D       = 0.5f;
    static constexpr float    BAT_RESERVE_MAX_D       = 30.0f;
    static constexpr uint16_t BAT_REPEAT_MIN_H        = 1;
    static constexpr uint16_t BAT_REPEAT_MAX_H        = 168;
    static constexpr uint32_t STATE_LOG_MIN_MS        = 1000UL;
    static constexpr uint32_t STATE_LOG_MAX_MS        = 60000UL;
    static constexpr uint32_t INA_RAW_MIN_MS          = 5000UL;
    static constexpr uint32_t INA_RAW_MAX_MS          = 300000UL;
    static constexpr uint32_t OTA_WINDOW_MIN_MS       = 60000UL;
    static constexpr uint32_t OTA_WINDOW_MAX_MS       = 1800000UL;

    // Smart motion detector bounds.
    static constexpr uint32_t MOTION_CALIB_MIN_MS     = 5000UL;
    static constexpr uint32_t MOTION_CALIB_MAX_MS     = 180000UL;
    static constexpr float    BASELINE_ALPHA_MIN      = 0.0001f;
    static constexpr float    BASELINE_ALPHA_MAX      = 0.05f;
    static constexpr float    IMPACT_THR_MIN_G        = 0.020f;
    static constexpr float    IMPACT_THR_MAX_G        = 0.500f;
    static constexpr uint32_t IMPACT_WIN_MIN_MS       = 100UL;
    static constexpr uint32_t IMPACT_WIN_MAX_MS       = 5000UL;
    static constexpr uint8_t  IMPACT_PEAKS_MIN        = 1;
    static constexpr uint8_t  IMPACT_PEAKS_MAX        = 5;
    static constexpr float    TILT_THR_MIN_DEG        = 1.0f;
    static constexpr float    TILT_THR_MAX_DEG        = 30.0f;
    static constexpr uint32_t TILT_CONFIRM_MIN_MS     = 500UL;
    static constexpr uint32_t TILT_CONFIRM_MAX_MS     = 30000UL;
    static constexpr float    MOVE_RMS_MIN_G          = 0.005f;
    static constexpr float    MOVE_RMS_MAX_G          = 0.200f;
    static constexpr uint32_t MOVE_CONFIRM_MIN_MS     = 1000UL;
    static constexpr uint32_t MOVE_CONFIRM_MAX_MS     = 60000UL;
    static constexpr uint8_t  MOVE_ACTIVE_PCT_MIN     = 5;
    static constexpr uint8_t  MOVE_ACTIVE_PCT_MAX     = 100;
    static constexpr uint32_t EARLY_WINDOW_MIN_MS     = 0;
    static constexpr uint32_t EARLY_WINDOW_MAX_MS     = 1800000UL;
    static constexpr float    EARLY_MULT_MIN          = 1.0f;
    static constexpr float    EARLY_MULT_MAX          = 5.0f;

    void begin();              // load from NVS or apply defaults
    bool saveToNvs();          // returns false on NVS failure
    void resetToDefaults();    // re-load defaults and persist

    // ---- Getters ----
    float    motionDeltaThresholdG()    const { return _motionDeltaThresholdG; }
    uint8_t  motionConfirmSamples()     const { return _motionConfirmSamples; }
    uint32_t motionCooldownMs()         const { return _motionCooldownMs; }

    uint32_t armingDelayMs()            const { return _armingDelayMs; }
    uint32_t armedSensorGraceMs()       const { return _armedSensorGraceMs; }

    uint32_t sirenAlarmContinuousMs()   const { return _sirenAlarmContinuousMs; }
    bool     sirenAfterTimeoutReminderEnabled() const { return _sirenAfterTimeoutReminderEnabled; }
    uint32_t sirenReminderOnMs()        const { return _sirenReminderOnMs; }
    uint32_t sirenReminderOffMs()       const { return _sirenReminderOffMs; }

    bool     lowBatterySmsEnabled()     const { return _lowBatterySmsEnabled; }
    float    batteryCapacityAh()        const { return _batteryCapacityAh; }
    float    batteryWarnVoltage()       const { return _batteryWarnVoltage; }
    float    batteryCriticalVoltage()   const { return _batteryCriticalVoltage; }
    float    batteryShutdownVoltage()   const { return _batteryShutdownVoltage; }
    float    lowBatteryReserveDays()    const { return _lowBatteryReserveDays; }
    uint16_t lowBatterySmsRepeatHours() const { return _lowBatterySmsRepeatHours; }
    uint32_t lowBatterySmsRepeatMs()    const { return (uint32_t)_lowBatterySmsRepeatHours * 3600000UL; }

    uint32_t stateLogIntervalMs()       const { return _stateLogIntervalMs; }
    uint32_t inaRawLogIntervalMs()      const { return _inaRawLogIntervalMs; }

    uint32_t otaActiveWindowMs()        const { return _otaActiveWindowMs; }

    // Smart motion detector.
    bool     smartMotionEnabled()       const { return _smartMotionEnabled; }
    uint32_t motionCalibrationMs()      const { return _motionCalibrationMs; }
    float    motionBaselineAlphaQuiet() const { return _motionBaselineAlphaQuiet; }
    float    impactThresholdG()         const { return _impactThresholdG; }
    uint32_t impactConfirmWindowMs()    const { return _impactConfirmWindowMs; }
    uint8_t  impactMinPeaks()           const { return _impactMinPeaks; }
    float    tiltThresholdDeg()         const { return _tiltThresholdDeg; }
    uint32_t tiltConfirmMs()            const { return _tiltConfirmMs; }
    float    moveRmsThresholdG()        const { return _moveRmsThresholdG; }
    uint32_t moveConfirmMs()            const { return _moveConfirmMs; }
    uint8_t  moveMinActivePercent()     const { return _moveMinActivePercent; }
    uint32_t motionEarlyWindowMs()      const { return _motionEarlyWindowMs; }
    float    motionEarlyMultiplier()    const { return _motionEarlyMultiplier; }

    // ---- Validating setters ----
    // Each returns true if the value was within bounds and stored, false
    // if rejected (existing value kept). The web layer collects per-field
    // outcomes and emits the warning list.
    bool setMotionDeltaThresholdG(float v);
    bool setMotionConfirmSamples(int v);
    bool setMotionCooldownMs(long v);
    bool setArmingDelayMs(long v);
    bool setArmedSensorGraceMs(long v);
    bool setSirenAlarmContinuousMs(long v);
    bool setSirenAfterTimeoutReminderEnabled(bool v);
    bool setSirenReminderOnMs(long v);
    bool setSirenReminderOffMs(long v);
    bool setLowBatterySmsEnabled(bool v);
    bool setBatteryCapacityAh(float v);
    bool setBatteryWarnVoltage(float v);
    bool setBatteryCriticalVoltage(float v);
    bool setBatteryShutdownVoltage(float v);
    bool setLowBatteryReserveDays(float v);
    bool setLowBatterySmsRepeatHours(int v);
    bool setStateLogIntervalMs(long v);
    bool setInaRawLogIntervalMs(long v);
    bool setOtaActiveWindowMs(long v);

    bool setSmartMotionEnabled(bool v);
    bool setMotionCalibrationMs(long v);
    bool setMotionBaselineAlphaQuiet(float v);
    bool setImpactThresholdG(float v);
    bool setImpactConfirmWindowMs(long v);
    bool setImpactMinPeaks(int v);
    bool setTiltThresholdDeg(float v);
    bool setTiltConfirmMs(long v);
    bool setMoveRmsThresholdG(float v);
    bool setMoveConfirmMs(long v);
    bool setMoveMinActivePercent(int v);
    bool setMotionEarlyWindowMs(long v);
    bool setMotionEarlyMultiplier(float v);

    // JSON dump (for /settings.json). Plain ASCII, no quotes-in-keys gore.
    String toJson() const;

    // Compact CFG SMS summary (under one SMS segment).
    String toShortSmsSummary() const;

private:
    void applyDefaults();
    bool loadFromNvs();   // false on namespace empty/corrupt

    void   logInvalid(const char* field) const;

    // Persisted fields. Defaults mirror config.h values.
    float    _motionDeltaThresholdG            = ACCEL_MOTION_DELTA_G;
    uint8_t  _motionConfirmSamples             = 2;
    uint32_t _motionCooldownMs                 = 3000UL;

    uint32_t _armingDelayMs                    = ALARM_ARMING_DELAY_MS;
    uint32_t _armedSensorGraceMs               = ALARM_ARMED_SENSOR_GRACE_MS;

    uint32_t _sirenAlarmContinuousMs           = SIREN_ALARM_CONTINUOUS_MS;
    bool     _sirenAfterTimeoutReminderEnabled = SIREN_AFTER_TIMEOUT_REMINDER_ENABLED != 0;
    uint32_t _sirenReminderOnMs                = SIREN_REMINDER_ON_MS;
    uint32_t _sirenReminderOffMs               = SIREN_REMINDER_OFF_MS;

    bool     _lowBatterySmsEnabled             = LOW_BATTERY_SMS_ENABLED != 0;
    float    _batteryCapacityAh                = BATTERY_CAPACITY_AH;
    float    _batteryWarnVoltage               = BAT_VOLTAGE_WARN;
    float    _batteryCriticalVoltage           = BAT_VOLTAGE_CRITICAL;
    float    _batteryShutdownVoltage           = BAT_VOLTAGE_SHUTDOWN;
    float    _lowBatteryReserveDays            = LOW_BATTERY_RESERVE_DAYS;
    uint16_t _lowBatterySmsRepeatHours         = (uint16_t)(LOW_BATTERY_SMS_REPEAT_MS / 3600000UL);

    uint32_t _stateLogIntervalMs               = STATE_LOG_PERIOD_MS;
    uint32_t _inaRawLogIntervalMs              = 10000UL;

    uint32_t _otaActiveWindowMs                = OTA_ACTIVE_WINDOW_MS;

    // Smart motion detector (production defaults per spec part 12).
    bool     _smartMotionEnabled        = true;
    uint32_t _motionCalibrationMs       = 30000UL;
    float    _motionBaselineAlphaQuiet  = 0.002f;
    float    _impactThresholdG          = 0.080f;
    uint32_t _impactConfirmWindowMs     = 800UL;
    uint8_t  _impactMinPeaks            = 1;
    float    _tiltThresholdDeg          = 4.0f;
    uint32_t _tiltConfirmMs             = 3000UL;
    float    _moveRmsThresholdG         = 0.020f;
    uint32_t _moveConfirmMs             = 5000UL;
    uint8_t  _moveMinActivePercent      = 40;
    uint32_t _motionEarlyWindowMs       = 600000UL;
    float    _motionEarlyMultiplier     = 1.5f;
};

extern RuntimeConfig runtimeConfig;
