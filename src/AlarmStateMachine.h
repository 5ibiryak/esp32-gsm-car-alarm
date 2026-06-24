#pragma once
#include <Arduino.h>

class Accelerometer;
class ReedSensor;
class StatusLed;
class Siren;
class Modem;
class BatteryMonitor;
class OtaManager;

// Local alarm FSM with SMS remote-command control.
//   DISARMED <-> ARMING_DELAY -> ARMED -> ALARM
// Boot enters ARMING_DELAY (automatic arming). SMS command ARM/DISARM
// switches between DISARMED and ARMING_DELAY. ALARM is latched until a
// remote DISARM or re-ARM, or until the physical power switch.
class AlarmStateMachine {
public:
    enum State { DISARMED, ARMING_DELAY, ARMED, ALARM };

    enum AlarmTriggerReason {
        TRIGGER_NONE,
        TRIGGER_MOTION,            // legacy single-threshold detector
        TRIGGER_REED_OPEN,
        TRIGGER_IMPACT,            // smart detector: sharp jerk
        TRIGGER_TILT,              // smart detector: sustained angle change
        TRIGGER_MOVE,              // smart detector: sustained movement
    };

    enum class RemoteCommand : uint8_t {
        STATUS         = 0,    // compact status
        ARM            = 1,
        DISARM         = 2,
        SILENT_DISARM  = 3,    // command 3: disarm + siren off
        SIREN_TEST     = 4,
        MODEM_STATUS   = 5,
        BATTERY_STATUS = 6,
        GPS_LOCATION   = 7,
        OTA_UPDATE     = 8,
        HELP           = 9,    // full command list (numeric or text "HELP")
    };

    void  begin(Accelerometer* accel, ReedSensor* reed, StatusLed* led,
                Siren* siren, Modem* modem, BatteryMonitor* battery,
                OtaManager* ota);
    void  update();

    State              state()         const { return _state; }
    AlarmTriggerReason triggerReason() const { return _triggerReason; }

    // Compact one-segment summary of the smart motion detector for the
    // MOTION SMS command.
    String buildMotionStatusReply() const;

    // SMS hook. Mutates FSM state as needed and returns the reply text
    // that the caller should SMS back to SMS_ALERT_PHONE.
    String handleRemoteCommand(RemoteCommand cmd);

    // Parses a body string ("0", "5", with optional whitespace/CRLF) into
    // a RemoteCommand. Returns true on a valid digit in 0..5.
    static bool parseRemoteCommand(const String& body, RemoteCommand& out);

    static const char* stateName(State s);
    static const char* triggerReasonName(AlarmTriggerReason r);
    static const char* commandName(RemoteCommand c);

    const char* stateName()         const { return stateName(_state); }
    const char* triggerReasonName() const { return triggerReasonName(_triggerReason); }

private:
    void enterDisarmed(bool playConfirm);
    void enterArmingDelay(bool playConfirm);
    void enterArmed();
    void enterAlarm(AlarmTriggerReason reason);

    String buildStatusReply()      const;
    String buildModemStatusReply() const;
    String buildHelpReply()        const;

    Accelerometer*  _accel   = nullptr;
    ReedSensor*     _reed    = nullptr;
    StatusLed*      _led     = nullptr;
    Siren*          _siren   = nullptr;
    Modem*          _modem   = nullptr;
    BatteryMonitor* _battery = nullptr;
    OtaManager*     _ota     = nullptr;

    State              _state              = DISARMED;
    AlarmTriggerReason _triggerReason      = TRIGGER_NONE;
    uint32_t           _stateEnteredMs     = 0;

    uint32_t           _lastArmingPrintMs  = 0;

    bool               _alarmBannerPrinted = false;
    bool               _sirenMuted         = false;
    uint32_t           _sirenTestEndMs     = 0;
};
