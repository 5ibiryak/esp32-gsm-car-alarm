#include "AlarmStateMachine.h"
#include "Accelerometer.h"
#include "ReedSensor.h"
#include "StatusLed.h"
#include "Siren.h"
#include "Modem.h"
#include "BatteryMonitor.h"
#include "RecipientManager.h"
#include "OtaManager.h"
#include "RuntimeConfig.h"
#include "config.h"

// ----- Name tables ----------------------------------------------------------

const char* AlarmStateMachine::stateName(State s) {
    switch (s) {
        case DISARMED:     return "DISARMED";
        case ARMING_DELAY: return "ARMING_DELAY";
        case ARMED:        return "ARMED";
        case ALARM:        return "ALARM";
    }
    return "?";
}

const char* AlarmStateMachine::triggerReasonName(AlarmTriggerReason r) {
    switch (r) {
        case TRIGGER_NONE:      return "none";
        case TRIGGER_MOTION:    return "MOTION";
        case TRIGGER_REED_OPEN: return "REED_OPEN";
        case TRIGGER_IMPACT:    return "IMPACT";
        case TRIGGER_TILT:      return "TILT";
        case TRIGGER_MOVE:      return "MOVE";
    }
    return "?";
}

const char* AlarmStateMachine::commandName(RemoteCommand c) {
    switch (c) {
        case RemoteCommand::STATUS:         return "STATUS";
        case RemoteCommand::ARM:            return "ARM";
        case RemoteCommand::DISARM:         return "DISARM";
        case RemoteCommand::SILENT_DISARM:  return "SILENT_DISARM";
        case RemoteCommand::SIREN_TEST:     return "SIREN_TEST";
        case RemoteCommand::MODEM_STATUS:   return "MODEM_STATUS";
        case RemoteCommand::BATTERY_STATUS: return "BATTERY_STATUS";
        case RemoteCommand::GPS_LOCATION:   return "GPS_LOCATION";
        case RemoteCommand::OTA_UPDATE:     return "OTA_UPDATE";
        case RemoteCommand::HELP:           return "HELP";
    }
    return "?";
}

bool AlarmStateMachine::parseRemoteCommand(const String& body, RemoteCommand& out) {
    String t = body;
    t.trim();
    if (t.length() == 0) return false;
    for (size_t i = 0; i < t.length(); ++i) {
        if (!isDigit(t.charAt(i))) return false;
    }
    const int v = t.toInt();
    if (v < 0 || v > 9) return false;
    out = (RemoteCommand)v;
    return true;
}

// ----- Setup ----------------------------------------------------------------

void AlarmStateMachine::begin(Accelerometer* accel, ReedSensor* reed,
                              StatusLed* led, Siren* siren,
                              Modem* modem, BatteryMonitor* battery,
                              OtaManager* ota) {
    _accel   = accel;
    _reed    = reed;
    _led     = led;
    _siren   = siren;
    _modem   = modem;
    _battery = battery;
    _ota     = ota;

    // Automatic arming on boot (per current power-switch design).
    enterArmingDelay(SIREN_CONFIRM_ON_BOOT_ARM != 0);
}

// ----- State entries --------------------------------------------------------

void AlarmStateMachine::enterDisarmed(bool playConfirm) {
    const State prev    = _state;
    _state              = DISARMED;
    _stateEnteredMs     = millis();
    _triggerReason      = TRIGGER_NONE;
    _alarmBannerPrinted = false;
    _sirenMuted         = false;

    if (_led)   _led->setMode(StatusLed::DISARMED);
    if (_modem) _modem->resetAlarmSmsLatch();

    // Always cancel any current siren activity first; then play disarm
    // confirmation beeps if the caller asked for them.
    if (_siren) _siren->off();
    if (playConfirm && _siren) _siren->confirmDisarm();

#if DEBUG_ALARM
    if (prev == ALARM) {
        Serial.println(F("[FSM] ALARM -> DISARMED"));
    } else {
        Serial.println(F("[FSM] -> DISARMED"));
    }
#else
    (void)prev;
#endif
}

void AlarmStateMachine::enterArmingDelay(bool playConfirm) {
    _state              = ARMING_DELAY;
    _stateEnteredMs     = millis();
    _lastArmingPrintMs  = _stateEnteredMs;
    _triggerReason      = TRIGGER_NONE;
    _sirenMuted         = false;

    if (_led)   _led->setMode(StatusLed::ARMING_DELAY);
    if (_modem) _modem->resetAlarmSmsLatch();   // new session can SMS again
    // Kick the smart motion detector into its calibration window so it
    // can build a fresh baseline before ARMED protection kicks in.
    if (_accel) _accel->resetAndCalibrate();

    // Cancel any prior siren state and optionally start the arm confirm
    // beep. Siren manages its own timing — no FSM-side beep tracking.
    if (_siren) _siren->off();
    if (playConfirm && _siren) _siren->confirmArm();

#if DEBUG_ALARM
    Serial.println(F("[FSM] -> ARMING_DELAY"));
    Serial.print  (F("[FSM] ARMING_DELAY duration = "));
    Serial.print  (runtimeConfig.armingDelayMs());
    Serial.println(F(" ms"));
#endif
}

void AlarmStateMachine::enterArmed() {
    _state          = ARMED;
    _stateEnteredMs = millis();

    if (_siren) _siren->off();
    if (_led)   _led->setMode(StatusLed::ARMED);
    // Start the smart detector's early-stabilization window.
    if (_accel) _accel->markArmed();

#if DEBUG_ALARM
    Serial.println(F("[FSM] ARMING_DELAY -> ARMED"));
    Serial.println(F("[FSM] ARMED sensor grace period"));
#endif
}

void AlarmStateMachine::enterAlarm(AlarmTriggerReason reason) {
    _state              = ALARM;
    _triggerReason      = reason;
    _stateEnteredMs     = millis();
    _alarmBannerPrinted = false;
    _sirenMuted         = false;

    if (_led)   _led->setMode(StatusLed::ALARM);
    if (_siren) _siren->alarmPattern();

#if DEBUG_ALARM
    Serial.print  (F("[FSM] ARMED -> ALARM reason="));
    Serial.println(triggerReasonName(reason));
#endif

#if SMS_ALERT_ENABLED
    if (_modem) {
        _modem->queueAlarmSms(triggerReasonName(reason));
    }
#endif
}

// ----- update ---------------------------------------------------------------

void AlarmStateMachine::update() {
    const uint32_t now = millis();

    // Siren-test auto-stop runs regardless of state.
    if (_sirenTestEndMs > 0 && (int32_t)(now - _sirenTestEndMs) >= 0) {
        if (_siren) _siren->off();
        _sirenTestEndMs = 0;
#if DEBUG_ALARM
        Serial.println(F("[FSM] siren test ended"));
#endif
    }

    switch (_state) {
        case DISARMED: {
            // Reed and motion are ignored; LED stays in DISARMED pattern.
            break;
        }

        case ARMING_DELAY: {
            if ((now - _lastArmingPrintMs) >= runtimeConfig.stateLogIntervalMs()) {
                _lastArmingPrintMs = now;
                const uint32_t elapsed = now - _stateEnteredMs;
                const uint32_t armingDelay = runtimeConfig.armingDelayMs();
                if (elapsed < armingDelay) {
                    const uint32_t remainingMs = armingDelay - elapsed;
#if DEBUG_ALARM
                    Serial.print  (F("[FSM] arming in "));
                    Serial.print  (remainingMs / 1000U);
                    Serial.println(F("s"));
#endif
                }
            }

            if ((now - _stateEnteredMs) >= runtimeConfig.armingDelayMs()) {
                enterArmed();
            }
            break;
        }

        case ARMED: {
            if ((now - _stateEnteredMs) < runtimeConfig.armedSensorGraceMs()) break;

            if (_reed && _reed->isOpen()) {
                enterAlarm(TRIGGER_REED_OPEN);
                break;
            }
            if (_accel && _accel->isMotionDetected()) {
                // Smart detector reports the specific reason. Legacy
                // single-threshold path stays as TRIGGER_MOTION.
                const Accelerometer::MotionReason r = _accel->currentMotionReason();
                switch (r) {
                    case Accelerometer::MotionReason::IMPACT: enterAlarm(TRIGGER_IMPACT); break;
                    case Accelerometer::MotionReason::TILT:   enterAlarm(TRIGGER_TILT);   break;
                    case Accelerometer::MotionReason::MOVE:   enterAlarm(TRIGGER_MOVE);   break;
                    case Accelerometer::MotionReason::NONE:   enterAlarm(TRIGGER_MOTION); break;
                }
            }
            break;
        }

        case ALARM: {
            if (!_alarmBannerPrinted) {
#if DEBUG_ALARM
                Serial.println(F("[FSM] ALARM active — turn off physical power switch to disarm"));
#endif
                _alarmBannerPrinted = true;
            }
            break;
        }
    }
}

// ----- Remote command dispatch ---------------------------------------------

String AlarmStateMachine::handleRemoteCommand(RemoteCommand cmd) {
#if DEBUG_ALARM
    Serial.print  (F("[FSM] remote command: "));
    Serial.println(commandName(cmd));
#endif

    switch (cmd) {
        case RemoteCommand::STATUS:
            return buildStatusReply();

        case RemoteCommand::HELP:
            return buildHelpReply();

        case RemoteCommand::ARM: {
            const bool playBeep = (SIREN_CONFIRM_ON_SMS_ARM != 0);
            if (_state == DISARMED) {
                enterArmingDelay(playBeep);
                return F("MoskvichAlarm: ARMING");
            }
            if (_state == ARMING_DELAY || _state == ARMED) {
                return F("MoskvichAlarm: ALREADY ARMED");
            }
            // ALARM -> re-arm. enterArmingDelay() stops the alarm siren
            // before optionally playing the confirm beep.
            enterArmingDelay(playBeep);
            return F("MoskvichAlarm: REARMING");
        }

        case RemoteCommand::DISARM:
            enterDisarmed(SIREN_CONFIRM_ON_SMS_DISARM != 0);
            return F("MoskvichAlarm: DISARMED");

        case RemoteCommand::SILENT_DISARM:
            enterDisarmed(SIREN_CONFIRM_ON_SMS_DISARM != 0);
            _sirenMuted = true;
            return F("MoskvichAlarm: DISARMED, SIREN OFF");

        case RemoteCommand::SIREN_TEST:
            // Steady-on 10s test, capped by _sirenTestEndMs in update().
            // Deliberately does NOT use alarmPattern() — that latches the
            // 2-minute ALARM continuous timer.
            if (_siren) _siren->on();
            _sirenTestEndMs = millis() + 10000;
            return F("MoskvichAlarm: SIREN TEST");

        case RemoteCommand::MODEM_STATUS:
            return buildModemStatusReply();

        case RemoteCommand::BATTERY_STATUS:
            if (_battery) return _battery->getStatusText();
            return F("MoskvichAlarm BATTERY: INA226 not available");

        case RemoteCommand::GPS_LOCATION:
            if (_modem) return _modem->requestGpsLocationBlocking(GPS_FIX_TIMEOUT_MS);
            return F("MoskvichAlarm GPS: modem not available");

        case RemoteCommand::OTA_UPDATE:
            if (!_ota) return F("MoskvichAlarm OTA: not available");
            if (_ota->isActive()) {
                return F("MoskvichAlarm OTA already active:\nhttp://192.168.4.1");
            }
            if (_ota->start()) {
                return _ota->getActivationReplyText();
            }
            return F("MoskvichAlarm OTA: start failed");
    }
    return F("MoskvichAlarm: ?");
}

String AlarmStateMachine::buildStatusReply() const {
    // Compact single-screen status. Must stay under one SMS segment.
    String r = "MoskvichAlarm STATUS:\nstate=";
    r += stateName(_state);
    r += "\nreed=";
    r += (_reed && _reed->isOpen()) ? "OPEN" : "CLOSED";
    r += "\nmotion=";
    r += (_accel && _accel->isMotionDetected()) ? "1" : "0";
    r += "\nbat=";
    if (_battery && _battery->hasValidMeasurement()) {
        r += String(_battery->getVoltageV(), 2);
        r += "V";
    } else {
        r += "UNKNOWN";
    }
    r += "\nmodem=";
    if (_modem && _modem->isAvailable() && _modem->isNetworkRegistered()) {
        r += "OK";
    } else if (_modem && _modem->isAvailable()) {
        r += "no network";
    } else {
        r += "down";
    }
    r += "\nfw=";
    r += FW_VERSION;
    r += "\nrcpt=";
    r += String((unsigned long)((_modem && _modem->recipients())
                                ? _modem->recipients()->count() : 0));
    return r;
}

String AlarmStateMachine::buildMotionStatusReply() const {
    if (!_accel) return String("MOTION: accel not available");
    String r = "MOTION:";
    r += "\nstate=";   r += _accel->stateName();
    r += "\nmag=";     r += String(_accel->getMagnitude(), 3);
    r += "\ndelta=";   r += String(_accel->getDeltaG(), 3);
    r += "\njerk=";    r += String(_accel->getJerkG(), 3);
    r += "\ntilt=";    r += String(_accel->getTiltDeg(), 1); r += "deg";
    r += "\nrms=";     r += String(_accel->getMoveRmsG(), 3);
    r += "\nthrI=";    r += String(runtimeConfig.impactThresholdG(), 3);
    r += "\nthrT=";    r += String(runtimeConfig.tiltThresholdDeg(), 1);
    return r;
}

String AlarmStateMachine::buildHelpReply() const {
    // Compact full command list — fits in a single SMS segment.
    String r = "HELP:";
    r += "\n0 status";
    r += "\n1 arm";
    r += "\n2 disarm";
    r += "\n3 siren off";
    r += "\n4 siren test";
    r += "\n5 modem";
    r += "\n6 battery";
    r += "\n7 gps";
    r += "\n8 ota";
    r += "\n9 help";
    r += "\nLIST";
    r += "\nADD +7";
    r += "\nDEL +7";
    r += "\nCFG";
    r += "\nMOTION";
#if SMS_DIAGNOSTIC_COMMANDS_ENABLED
    r += "\nDIAG: TESTSMS TESTSMS2G NETAUTO NETWCDMA";
#endif
    return r;
}

String AlarmStateMachine::buildModemStatusReply() const {
    String r = "MoskvichAlarm MODEM:\n";
    if (!_modem) {
        r += "modem object missing\n";
    } else {
        r += _modem->isRfOn() ? "LTE OK\n" : "LTE NOK\n";
        r += "CSQ=";
        r += String(_modem->getSignalQuality());
        r += "\nregistered=";
        r += _modem->isNetworkRegistered() ? "yes" : "no";
        r += "\nAT fails=";
        r += String(_modem->getConsecutiveAtFailures());
        r += "\nlast recovery=";
        r += _modem->getLastRecoveryStatus();
        r += "\n";
    }
    r += "FW=";
    r += FW_VERSION;
    r += "\nBuild=";
    r += FW_BUILD_DATE;
    r += " ";
    r += FW_BUILD_TIME;
    r += "\nOTA=";
    r += (_ota && _ota->isActive()) ? "active" : "inactive";
    return r;
}
