#include "Accelerometer.h"
#include "config.h"
#include "RuntimeConfig.h"

#include <Wire.h>
#include <math.h>

// =====================================================================
// MPU-6050 accelerometer driver (accelerometer-only mode)
// =====================================================================
// Power-saving notes:
//   - PWR_MGMT_1: TEMP_DIS bit is set so the internal temperature sensor
//     is not clocked.
//   - PWR_MGMT_2: STBY_XG | STBY_YG | STBY_ZG = 0x07 puts every gyroscope
//     axis into standby. The accelerometer axes stay enabled.
// The gyroscope is not used by MoskvichAlarm; disabling it cuts the
// idle current draw of the MPU-6050 noticeably.
// =====================================================================

namespace {

void print2Hex(uint8_t v) {
    if (v < 0x10) Serial.print('0');
    Serial.print(v, HEX);
}

}  // namespace

// ----- I2C register helpers --------------------------------------------------

bool Accelerometer::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
}

bool Accelerometer::readRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) return false;
    if (Wire.requestFrom(_addr, (uint8_t)1, (uint8_t)true) != 1) return false;
    value = Wire.read();
    return true;
}

bool Accelerometer::readRegisters(uint8_t startReg, uint8_t* buffer, size_t len) {
    if (buffer == nullptr || len == 0) return false;
    Wire.beginTransmission(_addr);
    Wire.write(startReg);
    if (Wire.endTransmission(true) != 0) return false;
    const uint8_t got = Wire.requestFrom(_addr, (uint8_t)len, (uint8_t)true);
    if (got != len) return false;
    for (size_t i = 0; i < len; ++i) buffer[i] = Wire.read();
    return true;
}

// ----- High-level steps ------------------------------------------------------

bool Accelerometer::detectAt(uint8_t addr) {
    _addr = addr;
    uint8_t v = 0;
    if (!readRegister(MPU6050_REG_WHO_AM_I, v)) return false;
#if DEBUG_ACCEL
    Serial.print(F("[MPU] WHO_AM_I @ 0x")); print2Hex(addr);
    Serial.print(F(" = 0x"));               print2Hex(v);
    Serial.println();
#endif
    return v == MPU6050_WHO_AM_I_EXPECTED;
}

// Wake the chip, disable the temperature sensor, put gyroscope into standby,
// set accelerometer to ±2g, and engage the ~44 Hz digital low-pass filter.
bool Accelerometer::configureAccelOnly() {
    // Wake from sleep — write 0x00 to PWR_MGMT_1, then set TEMP_DIS.
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) return false;
    delay(100);

    // PWR_MGMT_1: TEMP_DIS=1 (bit 3). CLKSEL stays at 0 (internal 8 MHz osc).
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT)) return false;

    // PWR_MGMT_2: gyroscope X/Y/Z into standby; accelerometer stays active.
    if (!writeRegister(MPU6050_REG_PWR_MGMT_2, MPU6050_PWR2_GYRO_STANDBY)) return false;

    // Accelerometer full-scale = ±2g (AFS_SEL = 0).
    if (!writeRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_2G)) return false;

    // DLPF_CFG = 3 → accelerometer bandwidth ~44 Hz, ~4.9 ms delay.
    if (!writeRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG_44HZ)) return false;

    return true;
}

// ----- begin / update --------------------------------------------------------

void Accelerometer::begin() {
    Serial.println();
    Serial.println(F("[MPU] Phase 3 MPU-6050 accelerometer-only bring-up"));
    Serial.print  (F("[MPU]   SDA=GPIO")); Serial.print(PIN_I2C_SDA);
    Serial.print  (F(" SCL=GPIO"));        Serial.print(PIN_I2C_SCL);
    Serial.print  (F(" freq="));           Serial.print(ACCEL_MPU6050_I2C_FREQ);
    Serial.println(F(" Hz"));

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(ACCEL_MPU6050_I2C_FREQ);

    // Check the primary address first, then the secondary.
    if      (detectAt(ACCEL_MPU6050_ADDR_PRIMARY))   { /* _addr already set */ }
    else if (detectAt(ACCEL_MPU6050_ADDR_SECONDARY)) { /* _addr already set */ }
    else {
        Serial.println(F("[MPU] WHO_AM_I did not match at 0x68 or 0x69 — sensor disabled"));
        _addr    = 0;
        _present = false;
        return;
    }

    if (!configureAccelOnly()) {
        Serial.println(F("[MPU] configuration failed — sensor disabled"));
        _present = false;
        return;
    }

    _present = true;

    Serial.print  (F("[MPU] initialized at 0x")); print2Hex(_addr); Serial.println();
    Serial.println(F("[MPU] gyro disabled, accelerometer only"));
    Serial.println(F("[MPU] accel range: +/-2g"));
    Serial.println(F("[MPU] scale: 16384 LSB/g"));

#if DEBUG_ACCEL
    Serial.print  (F("[MPU] polling every "));
    Serial.print  (ACCEL_POLL_PERIOD_MS);
    Serial.println(F(" ms"));
    Serial.print  (F("[MPU] motion delta threshold = "));
    Serial.print  (ACCEL_MOTION_DELTA_G, 3);
    Serial.println(F(" g"));
#endif
}

bool Accelerometer::update() {
    if (!_present) return false;

    const uint32_t now = millis();
    if (_lastSampleMs != 0 && (now - _lastSampleMs) < ACCEL_POLL_PERIOD_MS) return false;
    _lastSampleMs = now;

    // 6 bytes auto-increment over ACCEL_XOUT_H..ACCEL_ZOUT_L (0x3B..0x40).
    uint8_t buf[6];
    if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
#if DEBUG_ACCEL
        Serial.println(F("[MPU] read FAILED"));
#endif
        return false;
    }

    const int16_t rawX = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    const int16_t rawY = (int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3]);
    const int16_t rawZ = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);

    _x = (float)rawX / MPU6050_ACCEL_SENS_2G;
    _y = (float)rawY / MPU6050_ACCEL_SENS_2G;
    _z = (float)rawZ / MPU6050_ACCEL_SENS_2G;

    _magnitude = sqrtf(_x*_x + _y*_y + _z*_z);

    if (_hasPrev) {
        _delta = fabsf(_magnitude - _prevMagnitude);
        const float dx = _x - _prevX;
        const float dy = _y - _prevY;
        const float dz = _z - _prevZ;
        _jerk = sqrtf(dx*dx + dy*dy + dz*dz);
    } else {
        _delta   = 0.0f;
        _jerk    = 0.0f;
        _hasPrev = true;
    }
    _prevX = _x; _prevY = _y; _prevZ = _z;
    _prevMagnitude = _magnitude;

    // Legacy single-threshold flag stays available for backward compat
    // and any code reading isMotionDetected() under non-smart mode.
    legacyClassify();

#if DEBUG_ACCEL && DEBUG_MPU_VERBOSE
    Serial.print(F("[MPU] raw X=")); Serial.print(rawX);
    Serial.print(F(" Y="));          Serial.print(rawY);
    Serial.print(F(" Z="));          Serial.println(rawZ);

    Serial.print(F("[MPU] g X=")); Serial.print(_x, 3);
    Serial.print(F(" Y="));        Serial.print(_y, 3);
    Serial.print(F(" Z="));        Serial.print(_z, 3);
    Serial.print(F(" mag="));      Serial.print(_magnitude, 3);
    Serial.print(F(" delta="));    Serial.println(_delta, 3);
#endif

    if (runtimeConfig.smartMotionEnabled()) {
        smartClassify(now);
    }
    return true;
}

void Accelerometer::legacyClassify() {
    _motion = (_delta > runtimeConfig.motionDeltaThresholdG());
}

bool Accelerometer::isMotionDetected() const {
    if (runtimeConfig.smartMotionEnabled()) {
        return _confirmedReason != MotionReason::NONE;
    }
    return _motion;
}

// ----- Smart classifier -----------------------------------------------------

const char* Accelerometer::stateName(State s) {
    switch (s) {
        case State::CALIBRATING:      return "CALIBRATING";
        case State::QUIET:            return "QUIET";
        case State::CANDIDATE_IMPACT: return "CANDIDATE_IMPACT";
        case State::CANDIDATE_TILT:   return "CANDIDATE_TILT";
        case State::CANDIDATE_MOVE:   return "CANDIDATE_MOVE";
        case State::CONFIRMED:        return "CONFIRMED";
    }
    return "?";
}

const char* Accelerometer::reasonName(MotionReason r) {
    switch (r) {
        case MotionReason::NONE:   return "NONE";
        case MotionReason::IMPACT: return "IMPACT";
        case MotionReason::TILT:   return "TILT";
        case MotionReason::MOVE:   return "MOVE";
    }
    return "?";
}

uint32_t Accelerometer::timeSinceArmedMs() const {
    if (_armedAtMs == 0) return 0;
    return millis() - _armedAtMs;
}

void Accelerometer::resetAndCalibrate() {
    _state            = State::CALIBRATING;
    _confirmedReason  = MotionReason::NONE;
    _calibStartMs     = millis();
    _calibSamples     = 0;
    _calibSumX = _calibSumY = _calibSumZ = 0.0f;
    _calibSumDeltaSq  = 0.0f;
    _impactCandidateStartMs = 0;
    _impactPeaks            = 0;
    _tiltCandidateStartMs   = 0;
    _moveCandidateStartMs   = 0;
    _quietSinceMs           = 0;
    _moveRmsG               = 0.0f;
    _tiltDeg                = 0.0f;
    _armedAtMs              = 0;
    _baselineValid          = false;
#if DEBUG_ACCEL
    Serial.println(F("[MOTION] calibration start"));
#endif
}

void Accelerometer::markArmed() {
    _armedAtMs = millis();
#if DEBUG_ACCEL
    Serial.print  (F("[MOTION] early stabilization active "));
    Serial.print  (runtimeConfig.motionEarlyWindowMs() / 1000UL);
    Serial.print  (F("s multiplier="));
    Serial.println(runtimeConfig.motionEarlyMultiplier(), 2);
#endif
}

float Accelerometer::impactThresholdScaled() const {
    const uint32_t early = runtimeConfig.motionEarlyWindowMs();
    if (_armedAtMs != 0 && early > 0 && (millis() - _armedAtMs) < early) {
        return runtimeConfig.impactThresholdG() * runtimeConfig.motionEarlyMultiplier();
    }
    return runtimeConfig.impactThresholdG();
}

float Accelerometer::moveThresholdScaled() const {
    const uint32_t early = runtimeConfig.motionEarlyWindowMs();
    if (_armedAtMs != 0 && early > 0 && (millis() - _armedAtMs) < early) {
        return runtimeConfig.moveRmsThresholdG() * runtimeConfig.motionEarlyMultiplier();
    }
    return runtimeConfig.moveRmsThresholdG();
}

void Accelerometer::smartClassify(uint32_t now) {
    // ----- CALIBRATING -----
    if (_state == State::CALIBRATING) {
        // Accumulate baseline + noise.
        _calibSumX += _x; _calibSumY += _y; _calibSumZ += _z;
        _calibSumDeltaSq += (_delta * _delta);
        ++_calibSamples;

        const uint32_t calibMs = runtimeConfig.motionCalibrationMs();
        if ((now - _calibStartMs) >= calibMs && _calibSamples >= 4) {
            const float n = (float)_calibSamples;
            _baselineX = _calibSumX / n;
            _baselineY = _calibSumY / n;
            _baselineZ = _calibSumZ / n;
            _baselineMag = sqrtf(_baselineX*_baselineX +
                                 _baselineY*_baselineY +
                                 _baselineZ*_baselineZ);
            if (_baselineMag < 0.01f) _baselineMag = 1.0f;
            _noiseRmsG = sqrtf(_calibSumDeltaSq / n);
            _baselineValid = true;
            _state         = State::QUIET;
            _quietSinceMs  = now;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] calibration done baseline=("));
            Serial.print  (_baselineX, 3); Serial.print(',');
            Serial.print  (_baselineY, 3); Serial.print(',');
            Serial.print  (_baselineZ, 3);
            Serial.print  (F(") noise="));
            Serial.print  (_noiseRmsG, 4);
            Serial.println(F("g"));
#endif
        }
        return;
    }

    if (_state == State::CONFIRMED) return;        // latched until reset
    if (!_baselineValid)            return;        // safety

    // ----- Derived metrics -----
    // dyn = a - baseline (dynamic acceleration vector).
    const float dx = _x - _baselineX;
    const float dy = _y - _baselineY;
    const float dz = _z - _baselineZ;
    const float dynMag = sqrtf(dx*dx + dy*dy + dz*dz);

    // Tilt angle between current accel and baseline gravity vector.
    float cosA = 0.0f;
    if (_magnitude > 0.001f && _baselineMag > 0.001f) {
        cosA = (_x*_baselineX + _y*_baselineY + _z*_baselineZ) /
               (_magnitude * _baselineMag);
        if (cosA >  1.0f) cosA =  1.0f;
        if (cosA < -1.0f) cosA = -1.0f;
    }
    _tiltDeg = acosf(cosA) * 57.29577951f;          // 180/π

    // EMA of dyn magnitude — used as move RMS-ish indicator.
    const float moveAlpha = 0.10f;
    _moveRmsG = _moveRmsG * (1.0f - moveAlpha) + dynMag * moveAlpha;

    // ----- IMPACT detection (highest priority) -----
    const float impactThr = impactThresholdScaled();
    if (_jerk >= impactThr) {
        if (_impactCandidateStartMs == 0) {
            _impactCandidateStartMs = now;
            _impactPeaks            = 1;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] IMPACT candidate peak="));
            Serial.print  (_jerk, 3);
            Serial.print  (F("g threshold="));
            Serial.print  (impactThr, 3);
            Serial.println(F("g"));
#endif
            _state = State::CANDIDATE_IMPACT;
        } else {
            ++_impactPeaks;
        }
    }
    if (_impactCandidateStartMs != 0) {
        const uint32_t winMs = runtimeConfig.impactConfirmWindowMs();
        if (_impactPeaks >= runtimeConfig.impactMinPeaks()) {
            _confirmedReason = MotionReason::IMPACT;
            _state           = State::CONFIRMED;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] IMPACT confirmed peaks="));
            Serial.println(_impactPeaks);
            Serial.println(F("[MOTION] confirmed reason=IMPACT"));
#endif
            return;
        }
        if ((now - _impactCandidateStartMs) > winMs) {
            // window expired without enough peaks — reset candidate
            _impactCandidateStartMs = 0;
            _impactPeaks            = 0;
            if (_state == State::CANDIDATE_IMPACT) _state = State::QUIET;
        }
    }

    // ----- TILT detection -----
    const float tiltThr = runtimeConfig.tiltThresholdDeg();
    if (_tiltDeg >= tiltThr) {
        if (_tiltCandidateStartMs == 0) {
            _tiltCandidateStartMs = now;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] TILT candidate angle="));
            Serial.print  (_tiltDeg, 1);
            Serial.print  (F("deg threshold="));
            Serial.print  (tiltThr, 1);
            Serial.println(F("deg"));
#endif
            if (_state != State::CANDIDATE_IMPACT) _state = State::CANDIDATE_TILT;
        } else if ((now - _tiltCandidateStartMs) >= runtimeConfig.tiltConfirmMs()) {
            _confirmedReason = MotionReason::TILT;
            _state           = State::CONFIRMED;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] TILT confirmed angle="));
            Serial.print  (_tiltDeg, 1);
            Serial.print  (F("deg duration="));
            Serial.print  (now - _tiltCandidateStartMs);
            Serial.println(F("ms"));
            Serial.println(F("[MOTION] confirmed reason=TILT"));
#endif
            return;
        }
    } else {
        if (_tiltCandidateStartMs != 0) {
            _tiltCandidateStartMs = 0;
            if (_state == State::CANDIDATE_TILT) _state = State::QUIET;
        }
    }

    // ----- MOVE detection -----
    const float moveThr  = moveThresholdScaled();
    const bool  active   = (dynMag >= moveThr);
    if (active) {
        if (_moveCandidateStartMs == 0) {
            _moveCandidateStartMs = now;
            _moveSamplesActive    = 1;
            _moveSamplesTotal     = 1;
#if DEBUG_ACCEL
            Serial.print  (F("[MOTION] MOVE candidate rms="));
            Serial.print  (_moveRmsG, 3);
            Serial.println(F("g"));
#endif
            if (_state != State::CANDIDATE_IMPACT &&
                _state != State::CANDIDATE_TILT) _state = State::CANDIDATE_MOVE;
        } else {
            ++_moveSamplesActive;
            ++_moveSamplesTotal;
        }
    } else if (_moveCandidateStartMs != 0) {
        ++_moveSamplesTotal;
    }
    if (_moveCandidateStartMs != 0) {
        const uint32_t durMs = now - _moveCandidateStartMs;
        const uint32_t confirmMs = runtimeConfig.moveConfirmMs();
        if (durMs >= confirmMs) {
            const uint32_t pct = (_moveSamplesTotal == 0) ? 0
                : (uint32_t)((_moveSamplesActive * 100UL) / _moveSamplesTotal);
            const bool rmsHigh = (_moveRmsG >= moveThr);
            const bool activeEnough = (pct >= runtimeConfig.moveMinActivePercent());
            if (rmsHigh || activeEnough) {
                _confirmedReason = MotionReason::MOVE;
                _state           = State::CONFIRMED;
#if DEBUG_ACCEL
                Serial.print  (F("[MOTION] MOVE confirmed rms="));
                Serial.print  (_moveRmsG, 3);
                Serial.print  (F("g duration="));
                Serial.print  (durMs);
                Serial.println(F("ms"));
                Serial.println(F("[MOTION] confirmed reason=MOVE"));
#endif
                return;
            }
            // Window elapsed without sustained activity → reset.
            _moveCandidateStartMs = 0;
            _moveSamplesActive    = 0;
            _moveSamplesTotal     = 0;
            if (_state == State::CANDIDATE_MOVE) _state = State::QUIET;
        }
    }

    // ----- Quiet baseline drift -----
    const bool quietNow = (_state == State::QUIET) &&
                          (dynMag < moveThr) &&
                          (_tiltDeg < (tiltThr * 0.25f));
    if (quietNow) {
        const float alpha = runtimeConfig.motionBaselineAlphaQuiet();
        _baselineX = _baselineX * (1.0f - alpha) + _x * alpha;
        _baselineY = _baselineY * (1.0f - alpha) + _y * alpha;
        _baselineZ = _baselineZ * (1.0f - alpha) + _z * alpha;
        _baselineMag = sqrtf(_baselineX*_baselineX +
                             _baselineY*_baselineY +
                             _baselineZ*_baselineZ);
        if (_baselineMag < 0.01f) _baselineMag = 1.0f;

#if DEBUG_ACCEL
        // Throttle the quiet log so it doesn't fill the serial output.
        if ((now - _lastQuietLogMs) >= 60000UL) {
            _lastQuietLogMs = now;
            Serial.print  (F("[MOTION] quiet mag="));
            Serial.print  (_magnitude, 3);
            Serial.print  (F(" noise="));
            Serial.print  (_noiseRmsG, 4);
            Serial.print  (F(" tilt="));
            Serial.print  (_tiltDeg, 1);
            Serial.println(F("deg"));
        }
#endif
    }
}
