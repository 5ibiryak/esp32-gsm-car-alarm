#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include "config.h"

// MPU-6050 (GY-521) accelerometer with smart motion classifier.
//
// The MPU-6050 is operated in accelerometer-only mode: all three gyroscope
// axes are put into standby via PWR_MGMT_2 to reduce power, and the
// internal temperature sensor is disabled via the TEMP_DIS bit in
// PWR_MGMT_1.
//
// Motion classification has three flavors:
//   IMPACT — jerk (|a(t) - a(t-1)|) above impactThresholdG, optionally
//            confirmed by impactMinPeaks within impactConfirmWindowMs.
//   TILT   — angle between current acceleration and the slowly tracked
//            baseline gravity vector >= tiltThresholdDeg, held for
//            tiltConfirmMs.
//   MOVE   — sustained dynamic-acceleration RMS or "active sample"
//            percentage above thresholds for moveConfirmMs.
//
// State machine: CALIBRATING -> QUIET -> CANDIDATE_* -> CONFIRMED.
// Once CONFIRMED, the detector latches until resetAndCalibrate() is
// called by the FSM on a fresh arm cycle.
class Accelerometer {
public:
    enum class MotionReason : uint8_t { NONE, IMPACT, TILT, MOVE };
    enum class State : uint8_t {
        CALIBRATING,
        QUIET,
        CANDIDATE_IMPACT,
        CANDIDATE_TILT,
        CANDIDATE_MOVE,
        CONFIRMED
    };

    void  begin();
    bool  update();                                  // true when a fresh sample was taken

    bool  isAvailable()      const { return _present; }
    // Smart mode: only fire when the classifier has confirmed a reason.
    // Legacy mode: fall back to the single-threshold delta flag.
    bool  isMotionDetected() const;

    MotionReason currentMotionReason() const { return _confirmedReason; }
    State        detectorState()       const { return _state; }

    // FSM hooks.
    // Called on entry to ARMING_DELAY: clears all detector state and
    // re-enters the calibration window. No motion can be reported until
    // calibration completes.
    void  resetAndCalibrate();
    // Called on entry to ARMED: starts the "early-window" stabilization
    // timer that multiplies impact / move thresholds by
    // motionEarlyMultiplier for motionEarlyWindowMs ms.
    void  markArmed();

    // Live metrics for /status, SMS MOTION, and debug logs.
    float getXg()              const { return _x; }
    float getYg()              const { return _y; }
    float getZg()              const { return _z; }
    float getMagnitude()       const { return _magnitude; }
    float getDeltaG()          const { return _delta; }
    float getJerkG()           const { return _jerk; }
    float getTiltDeg()         const { return _tiltDeg; }
    float getNoiseRmsG()       const { return _noiseRmsG; }
    float getMoveRmsG()        const { return _moveRmsG; }
    float getBaselineX()       const { return _baselineX; }
    float getBaselineY()       const { return _baselineY; }
    float getBaselineZ()       const { return _baselineZ; }
    uint32_t timeSinceArmedMs() const;
    const char* stateName()    const { return stateName(_state); }
    static const char* stateName(State s);
    static const char* reasonName(MotionReason r);

    uint8_t i2cAddress() const { return _addr; }

private:
    // ---- Low-level I2C register helpers ----
    bool writeRegister (uint8_t reg, uint8_t value);
    bool readRegister  (uint8_t reg, uint8_t& value);
    bool readRegisters (uint8_t startReg, uint8_t* buffer, size_t len);

    // ---- High-level steps ----
    bool detectAt   (uint8_t addr);                  // WHO_AM_I probe at `addr`
    bool configureAccelOnly();                       // wake + disable gyro + accel range/DLPF

    // ---- Smart detector internals ----
    void  smartClassify(uint32_t now);
    void  legacyClassify();
    float impactThresholdScaled() const;
    float moveThresholdScaled()   const;

    bool     _present              = false;
    uint8_t  _addr                 = 0;

    // Latest sample
    float    _x = 0.0f, _y = 0.0f, _z = 0.0f;
    float    _magnitude            = 0.0f;
    float    _delta                = 0.0f;        // |mag - prevMag|
    float    _jerk                 = 0.0f;        // |a - prevA|
    float    _prevX = 0.0f, _prevY = 0.0f, _prevZ = 0.0f;
    float    _prevMagnitude        = 0.0f;
    bool     _hasPrev              = false;
    bool     _motion               = false;       // legacy compat (delta>thr)

    // Baseline (slow gravity vector estimate).
    float    _baselineX = 0.0f, _baselineY = 0.0f, _baselineZ = 0.0f;
    float    _baselineMag = 1.0f;
    bool     _baselineValid        = false;

    // Calibration accumulator.
    uint32_t _calibStartMs         = 0;
    uint32_t _calibSamples         = 0;
    float    _calibSumX = 0.0f, _calibSumY = 0.0f, _calibSumZ = 0.0f;
    float    _calibSumDeltaSq      = 0.0f;
    float    _noiseRmsG            = 0.0f;        // post-calibration noise floor

    // Smart-detector live values
    float    _tiltDeg              = 0.0f;
    float    _moveRmsG             = 0.0f;        // EMA of sqrt(dyn²)
    uint16_t _moveSamplesActive    = 0;           // within rolling window
    uint16_t _moveSamplesTotal     = 0;

    // State machine
    State        _state            = State::CALIBRATING;
    MotionReason _confirmedReason  = MotionReason::NONE;
    uint32_t     _armedAtMs        = 0;

    // Candidate timers / counters
    uint32_t _impactCandidateStartMs = 0;
    uint8_t  _impactPeaks            = 0;
    uint32_t _tiltCandidateStartMs   = 0;
    uint32_t _moveCandidateStartMs   = 0;

    // Quiet-tracking
    uint32_t _quietSinceMs           = 0;

    uint32_t _lastSampleMs           = 0;
    uint32_t _lastQuietLogMs         = 0;
};
