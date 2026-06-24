#pragma once
#include <Arduino.h>
#include "config.h"

class AlarmStateMachine;
class RecipientManager;

// SIM7600G-H modem driver — stable bring-up.
//
// begin():
//   1. SerialAT (UART1) configured on RX=GPIO26 / TX=GPIO27 @ 115200.
//   2. DTR (GPIO25) driven to MODEM_DTR_ACTIVE_LEVEL (HIGH on this board)
//      so the radio can leave low-power mode.
//   3. Single PWRKEY pulse, MODEM_BOOT_WAIT_MS boot wait, AT probe loop.
//   4. AT+CMEE=2, ATI, AT+CPIN?, AT+CFUN? — if CFUN=0, AT+CFUN=1 once.
//   5. refreshStatus() + printNetworkSummary().
// update():
//   - Every MODEM_STATUS_LOG_PERIOD_MS: refresh CFUN/CSQ/CPSI/CEREG/COPS
//     and print the one-line network summary.
class Modem {
public:
    void begin();
    void update();

    bool        isAvailable()         const { return _atOk; }
    bool        isSimReady()          const { return _simReady; }
    bool        isNetworkRegistered() const { return _registered; }
    bool        isRfOn()              const { return _cfun == 1; }
    int         getSignalQuality()    const { return _csq; }
    int         getCfun()             const { return _cfun; }
    const char* getOperator()         const { return _operatorName; }
    const char* getCpsi()             const { return _cpsiLine; }

    void printNetworkSummary();

    // Generic AT helper. Flushes input, writes `cmd\r\n`, collects bytes
    // until OK / ERROR / CME / CMS error / timeout. Returns true on "OK".
    bool sendAT(const char* cmd, String& response, uint32_t timeoutMs);

    // ---- SMS ----
    // True when SIM is READY, CFUN=1, and network is registered.
    bool isReadyForSms() const;

    // Bounded blocking text-mode SMS send with profile fallback.
    // `firstAttempt=true` (default) tries the full profile ladder; pass
    // false from retry paths to use the last-known working profile only
    // and avoid burning the full ~30 s ladder per retry.
    bool sendSms(const char* phone, const String& text, bool firstAttempt = true);

    // FSM hook: latch an alarm SMS for every entry in SMS_ALERT_RECIPIENTS.
    // No-op if a session is already in flight or one was sent for the
    // current alarm session.
    void queueAlarmSms(const char* reason);

    bool wasSmsSentForCurrentAlarm() const { return _smsAlarmSent; }
    void resetAlarmSmsLatch();

    // BatteryMonitor hook: queue a LOW BATTERY SMS to every recipient.
    // `critical=true` switches the body to the stronger "CRITICAL BATTERY /
    // Charge now" wording. Uses an independent per-recipient retry tracker
    // (the alarm queue is not disturbed).
    void queueLowBatterySms(float voltageV, float estimatedDays, bool critical);

    // ---- GNSS / GPS ----
    bool        gnssBegin();                                  // AT+CGPS=1
    void        gnssUpdate();                                 // one AT+CGPSINFO poll
    bool        hasGpsFix()    const { return _gpsFixValid; }
    bool        gnssStarted()  const { return _gnssStarted; }
    String      getGpsStatusText() const;
    String      requestGpsLocationBlocking(uint32_t timeoutMs);

    // ---- Health / recovery (read-only diagnostics) ----
    int         getConsecutiveAtFailures() const { return _consecutiveAtFailures; }
    const char* getLastRecoveryStatus()    const { return _lastRecoveryStatus; }
    int         pendingSmsCount()          const;

    // ---- Incoming SMS commands ----
    void setAlarmFsm(AlarmStateMachine* fsm) { _fsm = fsm; }
    void setRecipientManager(RecipientManager* m) { _recipients = m; }
    const RecipientManager* recipients() const { return _recipients; }

    // Tolerant phone-list membership test. Returns true when `phone`
    // exactly equals an entry in `list` OR when its last 10 characters
    // match an entry's last 10 characters.
    static bool isPhoneInList(const String& phone,
                              const char* const* list, size_t count);

    // Dumps a wide set of SMS-related registers + state to Serial.
    // Called automatically after a +CMS ERROR on send.
    void printSmsDiagnostics(const char* reason);

private:
    void powerKeyPulse();
    bool waitForAtBoot(uint32_t totalWaitMs, uint32_t probeIntervalMs);
    void checkBootUrc(const String& line);
    void runFullInit();
    void refreshStatus();

    void parseCpin(const String& resp);
    void parseCsq (const String& resp);
    void parseCops(const String& resp);
    void parseCpsi(const String& resp);
    void parseCfun(const String& resp);
    bool parseCregLike(const String& resp, const char* tag, int& outStatus);

    void   processSmsQueue();
    void   processLowBatterySmsQueue();
    String buildAlarmSmsText(const char* reason) const;

    void   pollIncomingSms();
    void   processCmglResponse(const String& resp);
    void   handleIncomingSms(int index, const String& sender, const String& body);
    void   deleteSmsByIndex(int index);
    bool   handleTextRecipientCommand(const String& body, const String& sender);
    bool   handleTestSms2G(const String& sender);
    bool   trySmsInNetworkMode(const String& sender, const char* name, const char* cnmpCommand);

    bool   isOperatorMegaFon() const;
    void   forceWcdmaForSmsIfNeeded();          // boot-time workaround
    bool   waitForRegistration(uint32_t timeoutMs);
    bool   handleNetAuto(const String& sender);
    bool   handleNetWcdma(const String& sender);

    bool   parseCgpsInfo(const String& resp);
    float  nmeaToDecimal(const String& nmea, char hemisphere) const;
    String buildGpsMapsReply() const;

    // ---- SMS profile fallback ----
    enum class SmsProfile : uint8_t { DEFAULT_, CSMS_1, CGSMS_1, CGSMS_3, CMGF_ONLY };
    enum class SmsResult  : uint8_t { Sent, CmsError, OtherFailure };

    static const char* profileName(SmsProfile p);
    SmsResult trySendOneProfile(const char* phone, const String& text, SmsProfile p);

    void   healthCheckTick();
    bool   doSoftRecovery();
    bool   doHardRecovery();

    bool    _atOk            = false;
    bool    _simReady        = false;
    bool    _registered      = false;
    int     _csq             = 99;       // 99 = unknown per 3GPP
    int     _cfun            = 0;        // last +CFUN: <n>
    int     _cregStatus      = 0;   // +CREG:  GSM/UMTS CS registration
    int     _ceregStatus     = 0;   // +CEREG: LTE/EPS registration
    char    _operatorName[32] = {0};
    char    _cpsiLine[96]     = {0};

    uint32_t _lastStatusLogMs    = 0;
    uint32_t _nextAtRecoveryMs   = 0;     // when to retry AT if _atOk is false

    // ---- SMS queue state ----
    bool     _smsAlarmQueued    = false;
    bool     _smsAlarmSent      = false;     // any recipient delivered in this session
    char     _smsAlarmReason[32] = {0};

    // Per-recipient delivery tracking — sized to the runtime maximum.
    // Recipients are captured into the snapshot at queue time so the list
    // can be edited mid-broadcast without disturbing in-flight sends.
    bool     _smsRecipientDone[SMS_ALERT_RECIPIENT_MAX]            {};
    uint8_t  _smsRecipientAttempts[SMS_ALERT_RECIPIENT_MAX]        {};
    uint32_t _smsRecipientNextMs[SMS_ALERT_RECIPIENT_MAX]          {};
    char     _smsAlarmRecipientSnapshot[SMS_ALERT_RECIPIENT_MAX][20] {{0}};
    uint8_t  _smsAlarmRecipientCount                               = 0;

    // ---- Low-battery SMS queue (independent of alarm queue) ----
    bool     _lowBatQueued                                          = false;
    String   _lowBatBody;
    bool     _lowBatRecipientDone[SMS_ALERT_RECIPIENT_MAX]          {};
    uint8_t  _lowBatRecipientAttempts[SMS_ALERT_RECIPIENT_MAX]      {};
    uint32_t _lowBatRecipientNextMs[SMS_ALERT_RECIPIENT_MAX]        {};
    char     _lowBatRecipientSnapshot[SMS_ALERT_RECIPIENT_MAX][20]  {{0}};
    uint8_t  _lowBatRecipientCount                                  = 0;

    // ---- SMS command polling ----
    AlarmStateMachine* _fsm        = nullptr;
    RecipientManager*  _recipients = nullptr;
    uint32_t _lastSmsPollMs        = 0;
    bool     _smsSendInProgress    = false;   // serializes CMGS vs CMGL etc.
    int8_t   _lastWorkingProfileIdx = -1;     // -1 = no profile has succeeded yet

    // ---- GNSS / GPS ----
    bool     _gnssStarted   = false;
    bool     _gpsFixValid   = false;
    float    _gpsLat        = 0.0f;
    float    _gpsLon        = 0.0f;
    float    _gpsAlt        = 0.0f;
    float    _gpsSpeed      = 0.0f;
    uint32_t _gpsFixTimeMs  = 0;

    // ---- Health / recovery ----
    uint32_t _lastHealthCheckMs   = 0;
    uint32_t _lastAtOkMs          = 0;
    int      _consecutiveAtFailures = 0;
    bool     _modemRecovering    = false;
    uint32_t _lastRecoveryMs     = 0;
    int      _recoveryAttempts   = 0;
    bool     _wasRegistered      = false;
    char     _lastRecoveryStatus[16] = "none";
};
