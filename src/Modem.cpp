#include "Modem.h"
#include "AlarmStateMachine.h"
#include "RecipientManager.h"
#include "RuntimeConfig.h"
#include "config.h"

#if WATCHDOG_ENABLED
#include <esp_task_wdt.h>
#endif

static inline void feedWatchdog() {
#if WATCHDOG_ENABLED
    esp_task_wdt_reset();
#endif
}

// SIM7600 is wired to UART1 on the LILYGO board.
#define SerialAT Serial1

// ----- Power / AT helpers ---------------------------------------------------

void Modem::powerKeyPulse() {
    pinMode(MODEM_PIN_PWRKEY, OUTPUT);
    digitalWrite(MODEM_PIN_PWRKEY, HIGH);
    delay(100);
    digitalWrite(MODEM_PIN_PWRKEY, LOW);
    delay(MODEM_PWRKEY_PULSE_MS);
    digitalWrite(MODEM_PIN_PWRKEY, HIGH);
}

// Time-bounded boot probe. Continuously reads pending bytes from the modem
// and detects URCs line-by-line (RDY / +CPIN: READY / SMS DONE / PB DONE).
// Sends "AT" every `probeIntervalMs` and returns true the moment AT
// returns OK. Returns false if `totalWaitMs` elapses without an OK.
bool Modem::waitForAtBoot(uint32_t totalWaitMs, uint32_t probeIntervalMs) {
    const uint32_t deadline = millis() + totalWaitMs;
    uint32_t nextProbeMs = millis();
    String   lineBuf;

    while ((int32_t)(millis() - deadline) < 0) {
        // Drain pending bytes; emit URC lines as they complete.
        while (SerialAT.available()) {
            const char c = (char)SerialAT.read();
            lineBuf += c;
            if (c == '\n') {
                String line = lineBuf;
                line.trim();
                lineBuf = "";
                if (line.length() > 0) checkBootUrc(line);
            }
            if (lineBuf.length() > 192) {
                // Bound the buffer in case the modem spews garbage.
                lineBuf = lineBuf.substring(lineBuf.length() - 64);
            }
        }

        if ((int32_t)(millis() - nextProbeMs) >= 0) {
            nextProbeMs = millis() + probeIntervalMs;
            String resp;
            if (sendAT("AT", resp, MODEM_AT_TIMEOUT_MS)) return true;
        }

        feedWatchdog();
        yield();
    }
    return false;
}

void Modem::checkBootUrc(const String& line) {
#if DEBUG_MODEM
    if (line == "RDY" ||
        line.indexOf("+CPIN: READY") >= 0 ||
        line.indexOf("SMS DONE")     >= 0 ||
        line == "PB DONE") {
        Serial.print  (F("[MODEM] boot URC received: "));
        Serial.println(line);
    }
#endif
}

bool Modem::sendAT(const char* cmd, String& response, uint32_t timeoutMs) {
    while (SerialAT.available()) SerialAT.read();
    response = "";

#if MODEM_AT_DEBUG && DEBUG_MODEM && DEBUG_AT_LOG
    Serial.print  (F("[AT>] "));
    Serial.println(cmd);
#endif

    SerialAT.print(cmd);
    SerialAT.print("\r\n");

    const uint32_t deadline = millis() + timeoutMs;
    bool ok  = false;
    bool err = false;

    while ((int32_t)(millis() - deadline) < 0) {
        while (SerialAT.available()) {
            const char c = (char)SerialAT.read();
            response += c;

            if (response.endsWith("\r\nOK\r\n")) { ok = true; break; }
            if (response.endsWith("\r\nERROR\r\n") ||
                response.indexOf("+CME ERROR") >= 0 ||
                response.indexOf("+CMS ERROR") >= 0) { err = true; break; }
        }
        if (ok || err) break;
        feedWatchdog();
        yield();
    }

#if MODEM_AT_DEBUG && DEBUG_MODEM && DEBUG_AT_LOG
    Serial.print  (F("[AT<] "));
    Serial.println(response);
#endif

    return ok;
}

// ----- begin ----------------------------------------------------------------

void Modem::begin() {
#if DEBUG_MODEM
    Serial.println(F("[MODEM] begin() — SIM7600 stable bring-up"));
    Serial.print  (F("[MODEM] SerialAT RX=GPIO"));
    Serial.print  (MODEM_PIN_RX);
    Serial.print  (F(" TX=GPIO"));
    Serial.print  (MODEM_PIN_TX);
    Serial.print  (F(" baud="));
    Serial.println(MODEM_UART_BAUD);
#endif

    SerialAT.begin(MODEM_UART_BAUD, SERIAL_8N1, MODEM_PIN_RX, MODEM_PIN_TX);

    // DTR must be HIGH on this LILYGO board to leave low-power mode.
    pinMode(MODEM_PIN_DTR, OUTPUT);
    digitalWrite(MODEM_PIN_DTR, MODEM_DTR_ACTIVE_LEVEL);
#if DEBUG_MODEM
    Serial.println(F("[MODEM] DTR GPIO25 = HIGH, modem RF enabled mode"));
#endif

#if MODEM_DO_PWRKEY_ON_BOOT
#if DEBUG_MODEM
    Serial.println(F("[MODEM] power key pulse"));
#endif
    powerKeyPulse();
#endif

#if DEBUG_MODEM
    Serial.println(F("[MODEM] waiting for modem boot"));
#endif
    delay(MODEM_BOOT_WAIT_MS);

#if DEBUG_MODEM
    Serial.print  (F("[MODEM] probing AT (up to "));
    Serial.print  (MODEM_AT_BOOT_TOTAL_WAIT_MS / 1000U);
    Serial.println(F("s)"));
#endif

    if (!waitForAtBoot(MODEM_AT_BOOT_TOTAL_WAIT_MS, MODEM_AT_BOOT_PROBE_INTERVAL_MS)) {
#if DEBUG_MODEM
        Serial.println(F("[MODEM] AT failed — modem not responding"));
        Serial.println(F("[MODEM] update() will keep retrying AT recovery"));
#endif
        _atOk             = false;
        _nextAtRecoveryMs = millis() + MODEM_AT_RECOVERY_INTERVAL_MS;
        return;
    }
    _atOk = true;
#if DEBUG_MODEM
    Serial.println(F("[MODEM] AT OK"));
#endif

    runFullInit();

#if DEBUG_MODEM
    Serial.print  (F("[SMS] command whitelist: "));
    Serial.println(SMS_COMMAND_WHITELIST_COUNT);
    // Alert-recipient count is logged by RecipientManager::begin().
#endif
}

void Modem::runFullInit() {
    String resp;

    // Verbose CME errors throughout the session.
    sendAT("AT+CMEE=2", resp, 2000);

    // SIM and basic identity.
    sendAT("ATI",       resp, 2000);
    sendAT("AT+CPIN?",  resp, 2000); parseCpin(resp);
    sendAT("AT+CFUN?",  resp, 2000); parseCfun(resp);

    // If the modem booted with RF off, ask for it once.
    if (_cfun == 0) {
#if DEBUG_MODEM
        Serial.println(F("[MODEM] CFUN=0 — issuing AT+CFUN=1"));
#endif
        sendAT("AT+CFUN=1", resp, 5000);
        delay(5000);
        sendAT("AT+CFUN?",  resp, 2000); parseCfun(resp);
    }

    refreshStatus();
    printNetworkSummary();

    // Operator-specific SMS workaround. Done after refreshStatus so
    // _operatorName is populated.
    forceWcdmaForSmsIfNeeded();
}

// ----- Periodic refresh -----------------------------------------------------

void Modem::refreshStatus() {
    if (!_atOk) return;

    String resp;
    sendAT("AT+CFUN?",  resp, 2000); parseCfun(resp);
    sendAT("AT+CSQ",    resp, 1000); parseCsq(resp);
    sendAT("AT+CPSI?",  resp, 2000); parseCpsi(resp);
    // Query CREG (CS/GSM/UMTS) and CEREG (EPS/LTE) separately. WCDMA-only
    // mode reports +CREG: 0,1 while +CEREG: 0,4 — only checking CEREG
    // would falsely mark the modem as unregistered.
    sendAT("AT+CREG?",  resp, 1000); parseCregLike(resp, "+CREG:",  _cregStatus);
    sendAT("AT+CEREG?", resp, 1000); parseCregLike(resp, "+CEREG:", _ceregStatus);
    sendAT("AT+COPS?",  resp, 2000); parseCops(resp);

    const bool prevReg = _registered;
    // Accept either CREG (CS/GSM/UMTS) or CEREG (EPS/LTE) as registered.
    // In WCDMA-only mode CEREG can sit at 0,4 while CREG is 0,1 — that
    // still counts as a usable network for SMS.
    _registered = (_cregStatus  == 1 || _cregStatus  == 5 ||
                   _ceregStatus == 1 || _ceregStatus == 5);

#if DEBUG_MODEM
    if (prevReg && !_registered) {
        Serial.println(F("[MODEM] network lost"));
    } else if (!prevReg && _registered) {
        Serial.println(F("[MODEM] network recovered"));
    }
#endif
    _wasRegistered = _registered;
}

void Modem::update() {
#if MODEM_BASIC_DIAG
    const uint32_t now = millis();

    // Recovery-in-progress short-circuit (defensive — recovery is bounded
    // blocking so this generally isn't reentered, but it guards against
    // any future async call sites).
    if (_modemRecovering) return;

    healthCheckTick();

    // AT recovery path — if begin() timed out before the modem booted,
    // keep retrying so a queued alarm SMS can still go out once the
    // modem comes up.
    if (!_atOk) {
        if ((int32_t)(now - _nextAtRecoveryMs) < 0) return;
        _nextAtRecoveryMs = now + MODEM_AT_RECOVERY_INTERVAL_MS;

        String resp;
        if (!sendAT("AT", resp, MODEM_AT_TIMEOUT_MS)) return;

        _atOk = true;
#if DEBUG_MODEM
        Serial.println(F("[MODEM] AT recovered after boot delay"));
#endif
        runFullInit();
        return;
    }

    // Drive the SMS retry/send queue first so an SMS attempt does not
    // sit behind a future periodic refresh. Alarm queue is higher
    // priority than low-battery queue.
    processSmsQueue();
    processLowBatterySmsQueue();

#if SMS_COMMANDS_ENABLED
    if ((now - _lastSmsPollMs) >= SMS_COMMAND_POLL_INTERVAL_MS) {
        _lastSmsPollMs = now;
        pollIncomingSms();
    }
#endif

    if ((now - _lastStatusLogMs) < MODEM_STATUS_LOG_PERIOD_MS) return;
    _lastStatusLogMs = now;

    refreshStatus();
    printNetworkSummary();
#endif
}

// ----- Output ---------------------------------------------------------------

void Modem::printNetworkSummary() {
#if DEBUG_MODEM
    Serial.print  (F("[MODEM] network: cfun="));
    Serial.print  (_cfun);
    Serial.print  (F(" sim="));
    Serial.print  (_simReady ? F("READY") : F("NOT_READY"));
    Serial.print  (F(" csq="));
    Serial.print  (_csq);
    Serial.print  (F(" registered="));
    Serial.print  (_registered ? F("yes") : F("no"));
    Serial.print  (F(" creg="));
    Serial.print  (_cregStatus);
    Serial.print  (F(" cereg="));
    Serial.print  (_ceregStatus);
    Serial.print  (F(" operator="));
    Serial.println(_operatorName[0] ? _operatorName : "?");

    if (_cpsiLine[0]) {
        Serial.print  (F("[MODEM] cpsi: "));
        Serial.println(_cpsiLine);
    }
#endif
}

// ----- Response parsers -----------------------------------------------------

void Modem::parseCpin(const String& resp) {
    _simReady = (resp.indexOf("READY") >= 0);
}

void Modem::parseCsq(const String& resp) {
    const int idx = resp.indexOf("+CSQ:");
    if (idx < 0) return;
    const int comma = resp.indexOf(',', idx);
    if (comma < 0) return;
    _csq = resp.substring(idx + 5, comma).toInt();
}

void Modem::parseCops(const String& resp) {
    const int q1 = resp.indexOf('"');
    if (q1 < 0) return;
    const int q2 = resp.indexOf('"', q1 + 1);
    if (q2 <= q1) return;

    memset(_operatorName, 0, sizeof(_operatorName));
    const int len = q2 - q1 - 1;
    const int cap = (int)sizeof(_operatorName) - 1;
    const int n   = len < cap ? len : cap;
    for (int i = 0; i < n; ++i) _operatorName[i] = resp.charAt(q1 + 1 + i);
}

void Modem::parseCpsi(const String& resp) {
    memset(_cpsiLine, 0, sizeof(_cpsiLine));
    const int idx = resp.indexOf("+CPSI:");
    if (idx < 0) return;
    int eol = resp.indexOf('\r', idx);
    if (eol < 0) eol = resp.length();
    const int len = eol - idx;
    const int cap = (int)sizeof(_cpsiLine) - 1;
    const int n   = len < cap ? len : cap;
    for (int i = 0; i < n; ++i) _cpsiLine[i] = resp.charAt(idx + i);
}

void Modem::parseCfun(const String& resp) {
    const int idx = resp.indexOf("+CFUN:");
    if (idx < 0) return;
    int p = idx + 6;
    while (p < (int)resp.length() && resp.charAt(p) == ' ') ++p;

    int val = 0;
    bool gotDigit = false;
    while (p < (int)resp.length() && isDigit(resp.charAt(p))) {
        val = val * 10 + (resp.charAt(p) - '0');
        ++p;
        gotDigit = true;
    }
    if (gotDigit) _cfun = val;
}

bool Modem::parseCregLike(const String& resp, const char* tag, int& outStatus) {
    const int idx = resp.indexOf(tag);
    if (idx < 0) return false;

    const int comma = resp.indexOf(',', idx);
    if (comma < 0) return false;

    int p = comma + 1;
    while (p < (int)resp.length() && resp.charAt(p) == ' ') ++p;

    int val = 0;
    bool gotDigit = false;
    while (p < (int)resp.length() && isDigit(resp.charAt(p))) {
        val = val * 10 + (resp.charAt(p) - '0');
        ++p;
        gotDigit = true;
    }
    if (!gotDigit) return false;

    outStatus = val;
    return (val == 1 || val == 5);
}

// ----- SMS ------------------------------------------------------------------

bool Modem::isReadyForSms() const {
    return _atOk && _simReady && _cfun == 1 && _registered;
}

void Modem::resetAlarmSmsLatch() {
    _smsAlarmQueued = false;
    _smsAlarmSent   = false;
    memset(_smsAlarmReason, 0, sizeof(_smsAlarmReason));
    for (size_t i = 0; i < SMS_ALERT_RECIPIENT_MAX; ++i) {
        _smsRecipientDone[i]              = false;
        _smsRecipientAttempts[i]          = 0;
        _smsRecipientNextMs[i]            = 0;
        _smsAlarmRecipientSnapshot[i][0]  = 0;
    }
    _smsAlarmRecipientCount = 0;
}

void Modem::queueAlarmSms(const char* reason) {
    if (_smsAlarmSent)   return;     // session already concluded
    if (_smsAlarmQueued) return;     // already in flight

    memset(_smsAlarmReason, 0, sizeof(_smsAlarmReason));
    if (reason) {
        const size_t n   = strlen(reason);
        const size_t cap = sizeof(_smsAlarmReason) - 1;
        memcpy(_smsAlarmReason, reason, n < cap ? n : cap);
    }

    // Snapshot the current runtime recipient list so edits mid-broadcast
    // can't disturb the in-flight queue.
    const size_t total = _recipients ? _recipients->count() : 0;
    if (total == 0) {
#if DEBUG_MODEM
        Serial.println(F("[SMS] no alert recipients configured"));
#endif
        return;
    }

    _smsAlarmRecipientCount = (uint8_t)(total > SMS_ALERT_RECIPIENT_MAX ? SMS_ALERT_RECIPIENT_MAX : total);
    const uint32_t now = millis();
    for (size_t i = 0; i < _smsAlarmRecipientCount; ++i) {
        const char* p = _recipients->at(i);
        strncpy(_smsAlarmRecipientSnapshot[i], p ? p : "", 19);
        _smsAlarmRecipientSnapshot[i][19] = 0;
        _smsRecipientDone[i]     = false;
        _smsRecipientAttempts[i] = 0;
        _smsRecipientNextMs[i]   = now;
    }
    _smsAlarmQueued = true;

#if DEBUG_MODEM
    Serial.print  (F("[SMS] queued reason="));
    Serial.print  (_smsAlarmReason);
    Serial.print  (F(" for "));
    Serial.print  (_smsAlarmRecipientCount);
    Serial.println(F(" recipient(s)"));
#endif
}

String Modem::buildAlarmSmsText(const char* reason) const {
    // Owner-facing wording — minimal, no technical state.
    if (reason && strcmp(reason, "reed_open") == 0) return "MoskvichAlarm: DOOR OPEN";
    if (reason && strcmp(reason, "motion")    == 0) return "MoskvichAlarm: MOTION";
    return "MoskvichAlarm: ALARM";
}

void Modem::processSmsQueue() {
    if (!_smsAlarmQueued) return;

    const uint32_t now = millis();
    bool anyPending = false;

    for (size_t i = 0; i < _smsAlarmRecipientCount; ++i) {
        if (_smsRecipientDone[i]) continue;
        anyPending = true;

        if ((int32_t)(now - _smsRecipientNextMs[i]) < 0) continue;

        if (!isReadyForSms()) {
#if DEBUG_MODEM
            Serial.println(F("[SMS] waiting: modem not ready for SMS"));
            Serial.print  (F("[SMS] modem state: atOk="));
            Serial.print  (_atOk      ? 1 : 0);
            Serial.print  (F(" sim="));
            Serial.print  (_simReady  ? 1 : 0);
            Serial.print  (F(" cfun="));
            Serial.print  (_cfun);
            Serial.print  (F(" registered="));
            Serial.println(_registered ? 1 : 0);
#endif
            for (size_t j = 0; j < _smsAlarmRecipientCount; ++j) {
                if (!_smsRecipientDone[j]) _smsRecipientNextMs[j] = now + SMS_RETRY_INTERVAL_MS;
            }
            return;
        }

        _smsRecipientAttempts[i]++;
#if DEBUG_MODEM
        Serial.print  (F("[SMS] sending alarm to recipient "));
        Serial.print  ((unsigned)(i + 1));
        Serial.print  (F("/"));
        Serial.print  (_smsAlarmRecipientCount);
        Serial.print  (F(": "));
        Serial.println(_smsAlarmRecipientSnapshot[i]);
        Serial.print  (F("[SMS] attempt "));
        Serial.print  (_smsRecipientAttempts[i]);
        Serial.print  (F("/"));
        Serial.println(SMS_MAX_RETRIES);
#endif

        const String body         = buildAlarmSmsText(_smsAlarmReason);
        const bool   firstAttempt = (_smsRecipientAttempts[i] == 1);
        const bool   ok           = sendSms(_smsAlarmRecipientSnapshot[i], body, firstAttempt);

        if (ok) {
#if DEBUG_MODEM
            Serial.print  (F("[SMS] sent OK to "));
            Serial.println(_smsAlarmRecipientSnapshot[i]);
#endif
            _smsRecipientDone[i] = true;
            _smsAlarmSent        = true;
        } else {
#if DEBUG_MODEM
            Serial.print  (F("[SMS] failed to "));
            Serial.println(_smsAlarmRecipientSnapshot[i]);
#endif
            if (_smsRecipientAttempts[i] >= SMS_MAX_RETRIES) {
#if DEBUG_MODEM
                Serial.print  (F("[SMS] giving up on "));
                Serial.println(_smsAlarmRecipientSnapshot[i]);
#endif
                _smsRecipientDone[i] = true;
            } else {
                _smsRecipientNextMs[i] = millis() + SMS_RETRY_INTERVAL_MS;
            }
        }

        return;
    }

    if (!anyPending) {
        _smsAlarmQueued = false;
    }
}

const char* Modem::profileName(SmsProfile p) {
    switch (p) {
        case SmsProfile::DEFAULT_:  return "DEFAULT";
        case SmsProfile::CSMS_1:    return "CSMS_1";
        case SmsProfile::CGSMS_1:   return "CGSMS_1";
        case SmsProfile::CGSMS_3:   return "CGSMS_3";
        case SmsProfile::CMGF_ONLY: return "CMGF_ONLY";
    }
    return "?";
}

// Single attempt with the given profile's setup sequence. Returns
// SmsResult::Sent on +CMGS+OK, CmsError on +CMS ERROR, OtherFailure
// for any other path (no prompt, +CME ERROR, ERROR, timeout). Caller
// (sendSms) decides whether to try the next profile.
Modem::SmsResult Modem::trySendOneProfile(const char* phone, const String& text, SmsProfile p) {
    String resp;

    const uint32_t kSetup = SMS_PROFILE_SETUP_TIMEOUT_MS;

    // Per-profile setup. All AT commands go through sendAT so they show
    // up in the [AT>]/[AT<] trace; we don't gate on their success here
    // because the next profile may rescue what this one couldn't do.
    switch (p) {
        case SmsProfile::DEFAULT_:
            sendAT("AT+CMGF=1",          resp, kSetup);
            sendAT("AT+CSCS=\"GSM\"",    resp, kSetup);
            sendAT("AT+CSMP=17,167,0,0", resp, kSetup);
            break;
        case SmsProfile::CSMS_1:
            sendAT("AT+CSMS=1",          resp, kSetup);
            sendAT("AT+CMGF=1",          resp, kSetup);
            sendAT("AT+CSCS=\"GSM\"",    resp, kSetup);
            sendAT("AT+CSMP=17,167,0,0", resp, kSetup);
            break;
        case SmsProfile::CGSMS_1:
            sendAT("AT+CGSMS=1",         resp, kSetup);
            sendAT("AT+CMGF=1",          resp, kSetup);
            sendAT("AT+CSCS=\"GSM\"",    resp, kSetup);
            sendAT("AT+CSMP=17,167,0,0", resp, kSetup);
            break;
        case SmsProfile::CGSMS_3:
            sendAT("AT+CGSMS=3",         resp, kSetup);
            sendAT("AT+CMGF=1",          resp, kSetup);
            sendAT("AT+CSCS=\"GSM\"",    resp, kSetup);
            sendAT("AT+CSMP=17,167,0,0", resp, kSetup);
            break;
        case SmsProfile::CMGF_ONLY:
            sendAT("AT+CMGF=1", resp, kSetup);
            // CMGF_ONLY intentionally skips CSCS and CSMP — some networks
            // refuse messages whose DCS/character-set has been touched.
            break;
    }
    feedWatchdog();

    // AT+CMGS="<phone>" — modem responds with '>' prompt, not OK.
    String cmgs = "AT+CMGS=\"";
    cmgs += phone;
    cmgs += "\"";

    while (SerialAT.available()) SerialAT.read();

#if MODEM_AT_DEBUG && DEBUG_MODEM && DEBUG_AT_LOG
    Serial.print  (F("[AT>] "));
    Serial.println(cmgs);
#endif
    SerialAT.print(cmgs);
    SerialAT.print("\r\n");

    uint32_t deadline = millis() + SMS_PROMPT_TIMEOUT_MS;
    String   prompt;
    bool     gotPrompt = false;
    while ((int32_t)(millis() - deadline) < 0) {
        while (SerialAT.available()) {
            const char c = (char)SerialAT.read();
            prompt += c;
            if (c == '>') { gotPrompt = true; break; }
        }
        if (gotPrompt) break;
        feedWatchdog();
        yield();
    }

#if MODEM_AT_DEBUG && DEBUG_MODEM && DEBUG_AT_LOG
    Serial.print  (F("[AT<] "));
    Serial.println(prompt);
#endif

    if (!gotPrompt) {
        SerialAT.write((uint8_t)0x1B);          // ESC — abort SMS write mode
        return SmsResult::OtherFailure;
    }

    // Body + Ctrl+Z.
    SerialAT.print(text);
    SerialAT.write((uint8_t)0x1A);

    deadline = millis() + SMS_FINAL_RESPONSE_TIMEOUT_MS;
    String sendResp;
    bool   gotOk     = false;
    bool   gotErr    = false;
    bool   gotCmsErr = false;
    bool   gotCmeErr = false;
    while ((int32_t)(millis() - deadline) < 0) {
        while (SerialAT.available()) {
            const char c = (char)SerialAT.read();
            sendResp += c;
            // +CMTI URCs are tolerated: they don't end with the OK/ERROR
            // markers below, so they accumulate harmlessly until OK arrives.
            if (sendResp.endsWith("\r\nOK\r\n"))      { gotOk     = true; break; }
            if (sendResp.endsWith("\r\nERROR\r\n"))   { gotErr    = true; break; }
            if (sendResp.indexOf("+CME ERROR") >= 0)  { gotCmeErr = true; break; }
            if (sendResp.indexOf("+CMS ERROR") >= 0)  { gotCmsErr = true; break; }
        }
        if (gotOk || gotErr || gotCmsErr || gotCmeErr) break;
        feedWatchdog();
        yield();
    }

#if MODEM_AT_DEBUG && DEBUG_MODEM && DEBUG_AT_LOG
    Serial.print  (F("[AT<] "));
    Serial.println(sendResp);
#endif

    if (gotOk && sendResp.indexOf("+CMGS:") >= 0) return SmsResult::Sent;
    if (gotCmsErr) return SmsResult::CmsError;
    return SmsResult::OtherFailure;
}

bool Modem::sendSms(const char* phone, const String& text, bool firstAttempt) {
    if (!_atOk) return false;
    if (_smsSendInProgress) return false;        // re-entrancy guard
    _smsSendInProgress = true;

#if DEBUG_MODEM
    Serial.print  (F("[SMS] sending to "));
    Serial.println(phone);
    Serial.print  (F("[SMS] text: "));
    Serial.println(text);
#endif

    static const SmsProfile kAllProfiles[] = {
        SmsProfile::DEFAULT_,
        SmsProfile::CSMS_1,
        SmsProfile::CGSMS_1,
        SmsProfile::CGSMS_3,
        SmsProfile::CMGF_ONLY,
    };
    constexpr int kAllProfileCount =
        (int)(sizeof(kAllProfiles) / sizeof(kAllProfiles[0]));

    // Build the order for this send.
    // Production policy (SMS_PROFILE_FALLBACK_ENABLED=0 or
    // SMS_PROFILE_FULL_FALLBACK_FOR_DIAG_ONLY=1): try only the last known
    // working profile (or DEFAULT on first boot). WCDMA + DEFAULT works
    // reliably on MegaFon so burning the 25 s ladder on each retry is waste.
    // Retries (firstAttempt == false) always use the short path too.
    SmsProfile order[SMS_PROFILE_MAX_PER_SEND];
    int        orderCount = 0;

    const bool useShortPath =
        !firstAttempt
        || !SMS_PROFILE_FALLBACK_ENABLED
        || SMS_PROFILE_FULL_FALLBACK_FOR_DIAG_ONLY
        || (firstAttempt && SMS_PROFILE_FALLBACK_FULL_ONLY_ON_FIRST_ATTEMPT && !firstAttempt);

    if (useShortPath) {
        if (_lastWorkingProfileIdx >= 0 && _lastWorkingProfileIdx < kAllProfileCount) {
            order[orderCount++] = kAllProfiles[_lastWorkingProfileIdx];
        } else {
            order[orderCount++] = SmsProfile::DEFAULT_;
        }
    } else {
        if (_lastWorkingProfileIdx >= 0 && _lastWorkingProfileIdx < kAllProfileCount) {
            order[orderCount++] = kAllProfiles[_lastWorkingProfileIdx];
        }
        for (int i = 0; i < kAllProfileCount && orderCount < SMS_PROFILE_MAX_PER_SEND; ++i) {
            bool dup = false;
            for (int j = 0; j < orderCount; ++j) {
                if (order[j] == kAllProfiles[i]) { dup = true; break; }
            }
            if (!dup) order[orderCount++] = kAllProfiles[i];
        }
    }

    const uint32_t totalStart = millis();
    bool timedOut = false;

    for (int i = 0; i < orderCount; ++i) {
        if ((millis() - totalStart) > SMS_SEND_TOTAL_TIMEOUT_MS) {
            timedOut = true;
            break;
        }

#if DEBUG_MODEM
        Serial.print  (F("[SMS] trying profile "));
        Serial.println(profileName(order[i]));
#endif
        const uint32_t profileStart = millis();
        const SmsResult r           = trySendOneProfile(phone, text, order[i]);
        const uint32_t profileMs    = millis() - profileStart;
        feedWatchdog();

        if (r == SmsResult::Sent) {
            // Remember which profile worked for future retries.
            for (int k = 0; k < kAllProfileCount; ++k) {
                if (kAllProfiles[k] == order[i]) {
                    _lastWorkingProfileIdx = (int8_t)k;
                    break;
                }
            }
#if DEBUG_MODEM
            Serial.print  (F("[SMS] sent OK after "));
            Serial.print  (profileMs / 1000.0f, 1);
            Serial.println(F("s"));
#endif
            _smsSendInProgress = false;
            return true;
        }

#if DEBUG_MODEM
        Serial.print  (F("[SMS] profile "));
        Serial.print  (profileName(order[i]));
        Serial.print  (F(" failed after "));
        Serial.print  (profileMs / 1000.0f, 1);
        Serial.println(F("s"));
#endif
    }

    const uint32_t totalMs = millis() - totalStart;

    if (timedOut) {
        // Try to abort whatever the modem might still expect.
        SerialAT.write((uint8_t)0x1B);
#if DEBUG_MODEM
        Serial.print  (F("[SMS] failed: total send timeout total="));
        Serial.print  (totalMs / 1000.0f, 1);
        Serial.println(F("s"));
#endif
        _smsSendInProgress = false;
        return false;
    }

#if DEBUG_MODEM
    Serial.print  (F("[SMS] all profiles failed total="));
    Serial.print  (totalMs / 1000.0f, 1);
    Serial.println(F("s"));
#endif
    printSmsDiagnostics("ALL_PROFILES_FAILED");

    _smsSendInProgress = false;
    return false;
}

// ----- Incoming SMS command polling ----------------------------------------

void Modem::pollIncomingSms() {
    if (!isReadyForSms()) return;
    if (_smsSendInProgress) return;  // don't interleave with a CMGS send

    String resp;
    sendAT("AT+CMGF=1",        resp, 2000);
    sendAT("AT+CSCS=\"GSM\"",  resp, 2000);
    sendAT("AT+CMGL=\"REC UNREAD\"", resp, 10000);

    if (resp.indexOf("+CMGL:") >= 0) {
        processCmglResponse(resp);
    }
}

void Modem::processCmglResponse(const String& resp) {
    int pos = 0;
    while (true) {
        const int hdrStart = resp.indexOf("+CMGL:", pos);
        if (hdrStart < 0) break;

        const int hdrEol = resp.indexOf('\n', hdrStart);
        if (hdrEol < 0) break;

        // Body runs from after the header line to the next +CMGL: or the
        // final OK terminator, whichever comes first.
        const int nextHdr = resp.indexOf("+CMGL:",     hdrEol);
        const int okPos   = resp.indexOf("\r\nOK\r\n", hdrEol);

        int bodyEnd;
        if      (nextHdr < 0 && okPos < 0) bodyEnd = resp.length();
        else if (nextHdr < 0)              bodyEnd = okPos;
        else if (okPos   < 0)              bodyEnd = nextHdr;
        else                               bodyEnd = (nextHdr < okPos) ? nextHdr : okPos;

        String header = resp.substring(hdrStart, hdrEol);
        String body   = resp.substring(hdrEol + 1, bodyEnd);
        body.trim();

        // Header: +CMGL: <idx>,"<stat>","<sender>","<alpha>","<scts>"
        const int colon  = header.indexOf(": ");
        const int comma1 = header.indexOf(',', colon);
        int idx = -1;
        if (colon >= 0 && comma1 > colon) {
            idx = header.substring(colon + 2, comma1).toInt();
        }

        const int q1 = header.indexOf('"');
        const int q2 = (q1 >= 0) ? header.indexOf('"', q1 + 1) : -1;
        const int q3 = (q2 >= 0) ? header.indexOf('"', q2 + 1) : -1;
        const int q4 = (q3 >= 0) ? header.indexOf('"', q3 + 1) : -1;
        String sender;
        if (q3 >= 0 && q4 > q3) sender = header.substring(q3 + 1, q4);

        if (idx >= 0) handleIncomingSms(idx, sender, body);

        pos = bodyEnd;
    }
}

void Modem::handleIncomingSms(int index, const String& sender, const String& body) {
#if DEBUG_MODEM
    Serial.print  (F("[SMS-CMD] received from "));
    Serial.print  (sender);
    Serial.print  (F(": "));
    Serial.println(body);
#endif

    const bool authorized = isPhoneInList(sender,
                                          SMS_COMMAND_WHITELIST,
                                          SMS_COMMAND_WHITELIST_COUNT);
    if (!authorized) {
#if DEBUG_MODEM
        Serial.print  (F("[SMS-CMD] ignored unauthorized sender: "));
        Serial.println(sender);
#endif
#if SMS_COMMAND_DELETE_PROCESSED
        deleteSmsByIndex(index);
#endif
        return;
    }

#if DEBUG_MODEM
    Serial.println(F("[SMS-CMD] authorized sender"));
#endif

    // Text-mode recipient management commands (ADD / DEL / LIST + aliases)
    // are handled before the numeric command path so they don't collide.
    if (handleTextRecipientCommand(body, sender)) {
#if SMS_COMMAND_DELETE_PROCESSED
        deleteSmsByIndex(index);
#endif
        return;
    }

    AlarmStateMachine::RemoteCommand cmd;
    if (!AlarmStateMachine::parseRemoteCommand(body, cmd)) {
#if DEBUG_MODEM
        Serial.print  (F("[SMS-CMD] unknown command: "));
        Serial.println(body);
#endif
#if SMS_COMMAND_DELETE_PROCESSED
        deleteSmsByIndex(index);
#endif
        return;
    }

#if DEBUG_MODEM
    Serial.print  (F("[SMS-CMD] command="));
    Serial.println(AlarmStateMachine::commandName(cmd));
#endif

    if (!_fsm) {
#if SMS_COMMAND_DELETE_PROCESSED
        deleteSmsByIndex(index);
#endif
        return;
    }

    const String reply = _fsm->handleRemoteCommand(cmd);

    if (reply.length() > 0) {
        // Reply to the sender (already validated against the whitelist),
        // not necessarily to every alert recipient.
        const bool ok = sendSms(sender.c_str(), reply);
#if DEBUG_MODEM
        Serial.println(ok ? F("[SMS-CMD] reply sent OK")
                          : F("[SMS-CMD] reply send failed"));
#endif
    }

#if SMS_COMMAND_DELETE_PROCESSED
    deleteSmsByIndex(index);
#endif
}

void Modem::deleteSmsByIndex(int index) {
    String resp;
    String cmd = "AT+CMGD=";
    cmd += index;
    sendAT(cmd.c_str(), resp, 5000);
}

void Modem::printSmsDiagnostics(const char* reason) {
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] reason="));
    Serial.println(reason ? reason : "?");
#endif

    String r;

    sendAT("AT+CSCA?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CSCA="));
    Serial.println(r);
#endif
    // Detect empty SMSC. Format: +CSCA: "<number>",<type>
    const int csIdx = r.indexOf("+CSCA:");
    if (csIdx >= 0) {
        const int q1 = r.indexOf('"', csIdx);
        const int q2 = (q1 >= 0) ? r.indexOf('"', q1 + 1) : -1;
        if (q1 < 0 || q2 <= q1 + 1) {
#if DEBUG_MODEM
            Serial.println(F("[SMS-DIAG] warning: SMS center address is empty"));
#endif
        }
    }
    feedWatchdog();

    sendAT("AT+CPMS?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CPMS="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CMGF?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CMGF="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CSCS?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CSCS="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CSMP?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CSMP="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CREG?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CREG="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CEREG?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CEREG="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CSQ", r, 1000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CSQ="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+COPS?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] COPS="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CSMS?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CSMS="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CGSMS?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CGSMS="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CNMI?", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CNMI="));
    Serial.println(r);
#endif
    feedWatchdog();

    sendAT("AT+CEER", r, 2000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-DIAG] CEER="));
    Serial.println(r);
#endif
    feedWatchdog();
}

bool Modem::handleTextRecipientCommand(const String& body, const String& sender) {
    // Split into first word + optional argument.
    String trimmed = body;
    trimmed.trim();
    int sp = trimmed.indexOf(' ');
    String first = (sp >= 0) ? trimmed.substring(0, sp) : trimmed;
    String rest  = (sp >= 0) ? trimmed.substring(sp + 1) : String("");
    first.toUpperCase();
    rest.trim();

    const bool isAdd        = (first == "ADD")     || (first == "A");
    const bool isDel        = (first == "DEL")     || (first == "D");
    const bool isList       = (first == "LIST")    || (first == "L");
    const bool isHelp       = (first == "HELP")    || (first == "H");
    const bool isCfg        = (first == "CFG");
    const bool isMotion     = (first == "MOTION");
    const bool isTestSms    = (first == "TESTSMS");
    const bool isTestSms2G  = (first == "TESTSMS2G");
    const bool isNetAuto    = (first == "NETAUTO");
    const bool isNetWcdma   = (first == "NETWCDMA");
    if (!isAdd && !isDel && !isList && !isHelp && !isCfg && !isMotion &&
        !isTestSms && !isTestSms2G && !isNetAuto && !isNetWcdma) {
        return false;
    }

    if (isCfg) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=CFG"));
#endif
        sendSms(sender.c_str(), runtimeConfig.toShortSmsSummary());
        return true;
    }

    if (isMotion) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=MOTION"));
#endif
        const String reply = _fsm ? _fsm->buildMotionStatusReply()
                                  : String("MOTION: FSM not available");
        sendSms(sender.c_str(), reply);
        return true;
    }

    String reply;

    const bool isDiagCmd = isTestSms || isTestSms2G || isNetAuto || isNetWcdma;
#if !SMS_DIAGNOSTIC_COMMANDS_ENABLED
    if (isDiagCmd) {
#if DEBUG_MODEM
        Serial.print  (F("[SMS-CMD] diagnostic command rejected: "));
        Serial.println(first);
#endif
        sendSms(sender.c_str(), String("DIAG disabled"));
        return true;
    }
#else
    if (isTestSms2G) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=TESTSMS2G"));
#endif
        handleTestSms2G(sender);
        return true;
    }

    if (isNetAuto) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=NETAUTO"));
#endif
        handleNetAuto(sender);
        return true;
    }

    if (isNetWcdma) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=NETWCDMA"));
#endif
        handleNetWcdma(sender);
        return true;
    }

    if (isTestSms) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-CMD] command=TESTSMS"));
#endif
        const bool ok = sendSms(sender.c_str(), String("TEST"));
#if DEBUG_MODEM
        Serial.println(ok ? F("[SMS-CMD] reply sent OK")
                          : F("[SMS-CMD] reply failed"));
#endif
        return true;
    }
#endif
    (void)isDiagCmd;

    if (isHelp) {
        if (_fsm) {
            reply = _fsm->handleRemoteCommand(AlarmStateMachine::RemoteCommand::HELP);
        } else {
            reply = "MoskvichAlarm HELP: not available";
        }
    } else if (!_recipients) {
        reply = "RECIPIENT: not available";
    } else if (isList) {
        reply = _recipients->compactListText();
    } else if (isAdd) {
        if (rest.length() == 0) {
            reply = "RECIPIENT missing phone";
        } else {
            const String normalized = RecipientManager::normalizePhoneNumber(rest);
            if (!RecipientManager::isValidPhoneNumber(normalized)) {
                reply = "RECIPIENT invalid phone";
            } else if (_recipients->contains(normalized)) {
                reply = "RECIPIENT already exists";
            } else if (_recipients->count() >= SMS_ALERT_RECIPIENT_MAX) {
                reply = "RECIPIENT list full";
            } else if (_recipients->add(normalized)) {
#if DEBUG_MODEM
                Serial.print  (F("[SMS-RCPT] added "));
                Serial.println(normalized);
#endif
                if (_recipients->saveToNvs()) {
#if DEBUG_MODEM
                    Serial.println(F("[SMS-RCPT] saved to NVS"));
#endif
                } else {
                    Serial.println(F("[SMS] recipient NVS save failed"));
                }
                reply = "RECIPIENT added";
            } else {
                reply = "RECIPIENT add failed";
            }
        }
    } else {  // isDel
        if (rest.length() == 0) {
            reply = "RECIPIENT missing phone";
        } else {
            const String normalized = RecipientManager::normalizePhoneNumber(rest);
            if (!_recipients->contains(normalized)) {
                reply = "RECIPIENT not found";
            } else if (_recipients->count() <= 1 && !SMS_ALERT_ALLOW_ZERO_RECIPIENTS) {
                reply = "RECIPIENT cannot remove last";
            } else if (_recipients->remove(normalized)) {
#if DEBUG_MODEM
                Serial.print  (F("[SMS-RCPT] removed "));
                Serial.println(normalized);
#endif
                if (_recipients->saveToNvs()) {
#if DEBUG_MODEM
                    Serial.println(F("[SMS-RCPT] saved to NVS"));
#endif
                } else {
                    Serial.println(F("[SMS] recipient NVS save failed"));
                }
                reply = "RECIPIENT removed";
            } else {
                reply = "RECIPIENT remove failed";
            }
        }
    }

    if (reply.length() > 0) {
        const bool ok = sendSms(sender.c_str(), reply);
#if DEBUG_MODEM
        Serial.println(ok ? F("[SMS-CMD] reply sent OK")
                          : F("[SMS-CMD] reply failed"));
#endif
    }
    return true;
}

bool Modem::isPhoneInList(const String& phone,
                          const char* const* list, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        if (list[i] == nullptr) continue;
        if (phone.equals(list[i])) return true;

        const size_t entryLen = strlen(list[i]);
        if (phone.length() >= 10 && entryLen >= 10) {
            // Compare last 10 chars — handles +7…, 8…, 7… variants.
            const char* p1 = phone.c_str() + (phone.length() - 10);
            const char* p2 = list[i]       + (entryLen       - 10);
            if (strcmp(p1, p2) == 0) return true;
        }
    }
    return false;
}

// ----- Low-battery SMS queue -----------------------------------------------

void Modem::queueLowBatterySms(float voltageV, float estimatedDays, bool critical) {
    if (_lowBatQueued) return;     // one in-flight at a time

    if (critical) {
        // Strong CRITICAL wording — no estimate field, urgent.
        _lowBatBody  = "MoskvichAlarm: CRITICAL BATTERY\nV=";
        _lowBatBody += String(voltageV, 2);
        _lowBatBody += "V\nCharge now";
    } else {
        _lowBatBody  = "MoskvichAlarm: LOW BATTERY\nV=";
        _lowBatBody += String(voltageV, 2);
        _lowBatBody += "V\nest=";
        if (isnan(estimatedDays)) {
            _lowBatBody += "UNKNOWN";
        } else {
            _lowBatBody += String(estimatedDays, 1);
            _lowBatBody += "d";
        }
        _lowBatBody += "\nCharge soon";
    }

    const size_t total = _recipients ? _recipients->count() : 0;
    if (total == 0) {
#if DEBUG_MODEM
        Serial.println(F("[SMS] no alert recipients configured"));
#endif
        return;
    }

    _lowBatRecipientCount = (uint8_t)(total > SMS_ALERT_RECIPIENT_MAX ? SMS_ALERT_RECIPIENT_MAX : total);
    const uint32_t now = millis();
    for (size_t i = 0; i < _lowBatRecipientCount; ++i) {
        const char* p = _recipients->at(i);
        strncpy(_lowBatRecipientSnapshot[i], p ? p : "", 19);
        _lowBatRecipientSnapshot[i][19] = 0;
        _lowBatRecipientDone[i]     = false;
        _lowBatRecipientAttempts[i] = 0;
        _lowBatRecipientNextMs[i]   = now;
    }
    _lowBatQueued = true;

#if DEBUG_MODEM
    Serial.print  (F("[BAT-SMS] queued LOW BATTERY for "));
    Serial.print  (_lowBatRecipientCount);
    Serial.println(F(" recipient(s)"));
    Serial.print  (F("[BAT-SMS] body: "));
    Serial.println(_lowBatBody);
#endif
}

void Modem::processLowBatterySmsQueue() {
    if (!_lowBatQueued) return;

    const uint32_t now = millis();
    bool anyPending = false;

    for (size_t i = 0; i < _lowBatRecipientCount; ++i) {
        if (_lowBatRecipientDone[i]) continue;
        anyPending = true;

        if ((int32_t)(now - _lowBatRecipientNextMs[i]) < 0) continue;

        if (!isReadyForSms()) {
#if DEBUG_MODEM
            Serial.println(F("[BAT-SMS] waiting: modem not ready"));
#endif
            for (size_t j = 0; j < _lowBatRecipientCount; ++j) {
                if (!_lowBatRecipientDone[j]) _lowBatRecipientNextMs[j] = now + SMS_RETRY_INTERVAL_MS;
            }
            return;
        }

        _lowBatRecipientAttempts[i]++;
#if DEBUG_MODEM
        Serial.print  (F("[BAT-SMS] sending LOW BATTERY to recipient "));
        Serial.print  ((unsigned)(i + 1));
        Serial.print  (F("/"));
        Serial.print  (_lowBatRecipientCount);
        Serial.print  (F(": "));
        Serial.println(_lowBatRecipientSnapshot[i]);
        Serial.print  (F("[BAT-SMS] attempt "));
        Serial.print  (_lowBatRecipientAttempts[i]);
        Serial.print  (F("/"));
        Serial.println(SMS_MAX_RETRIES);
#endif

        const bool firstAttempt = (_lowBatRecipientAttempts[i] == 1);
        const bool ok = sendSms(_lowBatRecipientSnapshot[i], _lowBatBody, firstAttempt);

        if (ok) {
#if DEBUG_MODEM
            Serial.print  (F("[BAT-SMS] sent OK to "));
            Serial.println(_lowBatRecipientSnapshot[i]);
#endif
            _lowBatRecipientDone[i] = true;
        } else {
#if DEBUG_MODEM
            Serial.print  (F("[BAT-SMS] failed to "));
            Serial.println(_lowBatRecipientSnapshot[i]);
#endif
            if (_lowBatRecipientAttempts[i] >= SMS_MAX_RETRIES) {
#if DEBUG_MODEM
                Serial.print  (F("[BAT-SMS] giving up on "));
                Serial.println(_lowBatRecipientSnapshot[i]);
#endif
                _lowBatRecipientDone[i] = true;
            } else {
                _lowBatRecipientNextMs[i] = millis() + SMS_RETRY_INTERVAL_MS;
            }
        }

        return;     // one send per tick (sendSms is bounded-blocking)
    }

    if (!anyPending) {
        _lowBatQueued = false;
    }
}

// ----- GNSS / GPS ----------------------------------------------------------

bool Modem::gnssBegin() {
    if (!_atOk) return false;
    if (_gnssStarted) return true;

#if DEBUG_MODEM
    Serial.println(F("[GPS] starting GNSS"));
#endif

    String resp;
    // AT+CGPS=0 first — clear stale state. ERROR is fine (already off).
    sendAT("AT+CGPS=0", resp, 3000);

    if (!sendAT("AT+CGPS=1", resp, 5000)) {
#if DEBUG_MODEM
        Serial.println(F("[GPS] failed to enable GNSS"));
#endif
        return false;
    }

    sendAT("AT+CGPS?", resp, 2000);     // informational only

    _gnssStarted = true;
#if DEBUG_MODEM
    Serial.println(F("[GPS] GNSS enabled"));
#endif
    return true;
}

void Modem::gnssUpdate() {
    if (!_atOk || !_gnssStarted) return;

    String resp;
    if (!sendAT("AT+CGPSINFO", resp, 3000)) return;

    if (parseCgpsInfo(resp)) {
#if DEBUG_MODEM
        Serial.print  (F("[GPS] fix lat="));
        Serial.print  (_gpsLat, 6);
        Serial.print  (F(" lon="));
        Serial.print  (_gpsLon, 6);
        Serial.print  (F(" alt="));
        Serial.print  (_gpsAlt, 1);
        Serial.print  (F(" speed="));
        Serial.println(_gpsSpeed, 2);
        Serial.print  (F("[GPS] maps=https://maps.google.com/?q="));
        Serial.print  (_gpsLat, 6);
        Serial.print  (',');
        Serial.println(_gpsLon, 6);
#endif
    } else {
#if DEBUG_MODEM
        Serial.println(F("[GPS] no fix yet"));
#endif
    }
}

float Modem::nmeaToDecimal(const String& nmea, char hemisphere) const {
    // Latitude  format:  ddmm.mmmm   (dot at index 4)
    // Longitude format:  dddmm.mmmm  (dot at index 5)
    const int dot = nmea.indexOf('.');
    if (dot < 3) return 0.0f;
    const int degDigits = dot - 2;          // # of degree digits before the minutes

    const int   degrees = nmea.substring(0, degDigits).toInt();
    const float minutes = nmea.substring(degDigits).toFloat();

    float decimal = (float)degrees + minutes / 60.0f;
    if (hemisphere == 'S' || hemisphere == 'W') decimal = -decimal;
    return decimal;
}

bool Modem::parseCgpsInfo(const String& resp) {
    const int idx = resp.indexOf("+CGPSINFO:");
    if (idx < 0) return false;

    int start = idx + 10;                   // skip "+CGPSINFO:"
    while (start < (int)resp.length() && resp.charAt(start) == ' ') ++start;

    int end = resp.indexOf('\r', start);
    if (end < 0) end = resp.length();

    String data = resp.substring(start, end);
    data.trim();
    if (data.length() == 0) return false;
    // Empty fix line is just commas: ",,,,,,,,"
    if (data.charAt(0) == ',') return false;

    // Split into up to 9 comma-separated fields:
    //   <lat>,<N/S>,<lon>,<E/W>,<date>,<UTC time>,<alt>,<speed>,<course>
    String fields[9];
    int fieldIdx = 0;
    int prev     = -1;
    for (int i = 0; i <= (int)data.length() && fieldIdx < 9; ++i) {
        if (i == (int)data.length() || data.charAt(i) == ',') {
            fields[fieldIdx++] = data.substring(prev + 1, i);
            prev = i;
        }
    }

    if (fields[0].length() == 0 || fields[2].length() == 0) return false;

    const char ns = fields[1].length() > 0 ? fields[1].charAt(0) : 'N';
    const char ew = fields[3].length() > 0 ? fields[3].charAt(0) : 'E';

    _gpsLat       = nmeaToDecimal(fields[0], ns);
    _gpsLon       = nmeaToDecimal(fields[2], ew);
    _gpsAlt       = fields[6].toFloat();
    _gpsSpeed     = fields[7].toFloat();
    _gpsFixValid  = true;
    _gpsFixTimeMs = millis();
    return true;
}

String Modem::buildGpsMapsReply() const {
    String r = "MoskvichAlarm GPS:\nhttps://maps.google.com/?q=";
    r += String(_gpsLat, 6);
    r += ",";
    r += String(_gpsLon, 6);
    return r;
}

String Modem::getGpsStatusText() const {
    if (!_gpsFixValid) return String("MoskvichAlarm GPS: no fix");
    return buildGpsMapsReply();
}

String Modem::requestGpsLocationBlocking(uint32_t timeoutMs) {
#if DEBUG_MODEM
    Serial.println(F("[GPS] command received"));
#endif

    if (!_atOk) {
        return String("MoskvichAlarm GPS: modem not ready");
    }

    // Cached fix that's recent enough?
    if (_gpsFixValid && (millis() - _gpsFixTimeMs) <= GPS_MAX_AGE_MS) {
        return buildGpsMapsReply();
    }

    if (!gnssBegin()) {
        return String("MoskvichAlarm GPS: GNSS start failed");
    }

    // Cap the total wait at GPS_COMMAND_BLOCKING_MAX_MS so this call
    // cannot starve the watchdog or block the alarm indefinitely.
    const uint32_t hardCap  = GPS_COMMAND_BLOCKING_MAX_MS;
    const uint32_t budget   = (timeoutMs < hardCap) ? timeoutMs : hardCap;
    const uint32_t deadline = millis() + budget;

    while ((int32_t)(millis() - deadline) < 0) {
        if (!_atOk) {
#if DEBUG_MODEM
            Serial.println(F("[GPS] modem lost during GPS request"));
#endif
            return String("MoskvichAlarm GPS: modem lost during GPS request");
        }
        gnssUpdate();
        if (_gpsFixValid) return buildGpsMapsReply();
        feedWatchdog();
        delay(GPS_POLL_INTERVAL_MS);
        feedWatchdog();
    }

#if DEBUG_MODEM
    Serial.println(F("[GPS] no fix after timeout"));
#endif
    return String("MoskvichAlarm GPS: no fix. Try outside or near window.");
}

// ----- Health check / recovery ---------------------------------------------

int Modem::pendingSmsCount() const {
    int n = 0;
    if (_smsAlarmQueued) {
        for (size_t i = 0; i < _smsAlarmRecipientCount; ++i) {
            if (!_smsRecipientDone[i]) ++n;
        }
    }
    if (_lowBatQueued) {
        for (size_t i = 0; i < _lowBatRecipientCount; ++i) {
            if (!_lowBatRecipientDone[i]) ++n;
        }
    }
    return n;
}

void Modem::healthCheckTick() {
    const uint32_t now = millis();
    if (_lastHealthCheckMs != 0 && (now - _lastHealthCheckMs) < MODEM_HEALTH_CHECK_INTERVAL_MS) return;
    _lastHealthCheckMs = now;

    String resp;
    const bool ok = sendAT("AT", resp, MODEM_AT_TIMEOUT_MS);
    if (ok) {
        _lastAtOkMs            = now;
        _consecutiveAtFailures = 0;
        _atOk                  = true;
        return;
    }

    _consecutiveAtFailures++;
#if DEBUG_MODEM
    Serial.print  (F("[MODEM] health check failed count="));
    Serial.println(_consecutiveAtFailures);
#endif

    const uint32_t sinceLastRecovery = now - _lastRecoveryMs;
    const bool cooldownOk = (_lastRecoveryMs == 0) ||
                            (sinceLastRecovery >= MODEM_RECOVERY_COOLDOWN_MS);

    if (_consecutiveAtFailures >= MODEM_HARD_RECOVERY_AFTER_FAILURES && cooldownOk) {
        doHardRecovery();
    } else if (_consecutiveAtFailures >= MODEM_AT_FAILURE_LIMIT && cooldownOk) {
        doSoftRecovery();
    }
}

bool Modem::doSoftRecovery() {
    _modemRecovering = true;
#if DEBUG_MODEM
    Serial.println(F("[MODEM] soft recovery started"));
#endif

    // Reapply DTR HIGH and flush UART.
    pinMode(MODEM_PIN_DTR, OUTPUT);
    digitalWrite(MODEM_PIN_DTR, MODEM_DTR_ACTIVE_LEVEL);
    while (SerialAT.available()) SerialAT.read();
    feedWatchdog();
    delay(100);

    String resp;
    bool   atOk = false;
    for (int i = 0; i < 5; ++i) {
        if (sendAT("AT", resp, MODEM_AT_TIMEOUT_MS)) { atOk = true; break; }
        feedWatchdog();
        delay(500);
    }

    if (atOk) {
        sendAT("AT+CMEE=2", resp, 2000);
        sendAT("AT+CFUN?",  resp, 2000); parseCfun(resp);
        if (_cfun == 0) {
            sendAT("AT+CFUN=1", resp, 5000);
            feedWatchdog();
            delay(3000);
            sendAT("AT+CFUN?", resp, 2000); parseCfun(resp);
        }
        refreshStatus();
        // Restore SMS settings.
        sendAT("AT+CMGF=1",       resp, 2000);
        sendAT("AT+CSCS=\"GSM\"", resp, 2000);
    }

    _lastRecoveryMs = millis();
    _recoveryAttempts++;
    _modemRecovering = false;

    if (atOk) {
        _atOk                  = true;
        _consecutiveAtFailures = 0;
        strncpy(_lastRecoveryStatus, "soft OK", sizeof(_lastRecoveryStatus) - 1);
        _lastRecoveryStatus[sizeof(_lastRecoveryStatus) - 1] = 0;
#if DEBUG_MODEM
        Serial.println(F("[MODEM] soft recovery OK"));
#endif
        return true;
    }

    strncpy(_lastRecoveryStatus, "soft FAIL", sizeof(_lastRecoveryStatus) - 1);
    _lastRecoveryStatus[sizeof(_lastRecoveryStatus) - 1] = 0;
#if DEBUG_MODEM
    Serial.println(F("[MODEM] soft recovery failed"));
#endif
    return false;
}

bool Modem::doHardRecovery() {
    _modemRecovering = true;
#if DEBUG_MODEM
    Serial.println(F("[MODEM] hard recovery: PWRKEY cycle"));
#endif

    powerKeyPulse();

#if DEBUG_MODEM
    Serial.println(F("[MODEM] waiting for modem boot"));
#endif
    feedWatchdog();
    delay(MODEM_BOOT_WAIT_MS);
    feedWatchdog();

    // Reapply DTR HIGH after the modem has re-asserted itself.
    pinMode(MODEM_PIN_DTR, OUTPUT);
    digitalWrite(MODEM_PIN_DTR, MODEM_DTR_ACTIVE_LEVEL);

    const bool atOk = waitForAtBoot(MODEM_AT_BOOT_TOTAL_WAIT_MS,
                                    MODEM_AT_BOOT_PROBE_INTERVAL_MS);
    feedWatchdog();

    if (atOk) {
        String resp;
        sendAT("AT+CMEE=2", resp, 2000);
        sendAT("AT+CPIN?",  resp, 2000); parseCpin(resp);
        sendAT("AT+CFUN?",  resp, 2000); parseCfun(resp);
        if (_cfun == 0) {
            sendAT("AT+CFUN=1", resp, 5000);
            feedWatchdog();
            delay(3000);
            sendAT("AT+CFUN?", resp, 2000); parseCfun(resp);
        }
        refreshStatus();
        sendAT("AT+CMGF=1",       resp, 2000);
        sendAT("AT+CSCS=\"GSM\"", resp, 2000);
        _atOk                  = true;
        _consecutiveAtFailures = 0;
        // GNSS state was lost across the cycle; the next GPS command will
        // re-init it via gnssBegin().
        _gnssStarted = false;
    } else {
        _atOk = false;
    }

    _lastRecoveryMs = millis();
    _recoveryAttempts++;
    _modemRecovering = false;

    if (atOk) {
        strncpy(_lastRecoveryStatus, "hard OK", sizeof(_lastRecoveryStatus) - 1);
        _lastRecoveryStatus[sizeof(_lastRecoveryStatus) - 1] = 0;
#if DEBUG_MODEM
        Serial.println(F("[MODEM] hard recovery OK"));
#endif
    } else {
        strncpy(_lastRecoveryStatus, "hard FAIL", sizeof(_lastRecoveryStatus) - 1);
        _lastRecoveryStatus[sizeof(_lastRecoveryStatus) - 1] = 0;
#if DEBUG_MODEM
        Serial.println(F("[MODEM] hard recovery failed"));
#endif
    }
    return atOk;
}

// ----- Manual diagnostic: TESTSMS2G ----------------------------------------
// Switches the modem out of LTE-only mode, waits for registration on
// WCDMA (or GSM as fallback), tries an SMS via the normal sendSms()
// ladder, then restores automatic network mode. Watchdog is fed across
// every wait. Not invoked automatically.

// Try AT+CNMP=<mode>, wait up to 60 s for registration on that mode,
// then attempt one SMS via the existing sendSms() profile ladder. Used
// only by TESTSMS2G — production behavior is unchanged.
bool Modem::trySmsInNetworkMode(const String& sender,
                                const char* name, const char* cnmpCommand) {
#if DEBUG_MODEM
    Serial.print  (F("[SMS-NETMODE] trying "));
    Serial.println(name);
#endif

    String resp;
    const bool modeOk = sendAT(cnmpCommand, resp, 5000);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-NETMODE] CNMP response: "));
    Serial.println(resp);
#endif
    feedWatchdog();

    if (!modeOk) {
#if DEBUG_MODEM
        Serial.print  (F("[SMS-NETMODE] "));
        Serial.print  (name);
        Serial.println(F(" not accepted, skipping"));
#endif
        return false;
    }

    const uint32_t deadline = millis() + 60000UL;
    bool registered = false;

    while ((int32_t)(millis() - deadline) < 0) {
        feedWatchdog();
        delay(2500);
        feedWatchdog();
        delay(2500);
        feedWatchdog();

        sendAT("AT+CPSI?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
        sendAT("AT+CREG?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
        parseCregLike(resp, "+CREG:",  _cregStatus);  feedWatchdog();
        sendAT("AT+CEREG?", resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
        parseCregLike(resp, "+CEREG:", _ceregStatus); feedWatchdog();
        sendAT("AT+CSQ",    resp, 1000);
        parseCsq(resp); feedWatchdog();

        registered = (_cregStatus  == 1 || _cregStatus  == 5 ||
                      _ceregStatus == 1 || _ceregStatus == 5);
#if DEBUG_MODEM
        Serial.print  (F("[SMS-NETMODE] registered="));
        Serial.println(registered ? F("yes") : F("no"));
#endif
        if (registered) break;
    }

    if (!registered) {
#if DEBUG_MODEM
        Serial.print  (F("[SMS-NETMODE] "));
        Serial.print  (name);
        Serial.println(F(" failed (no registration)"));
#endif
        return false;
    }

#if DEBUG_MODEM
    Serial.println(F("[SMS-NETMODE] sending TEST"));
#endif
    const bool smsOk = sendSms(sender.c_str(), String("TEST"));

#if DEBUG_MODEM
    Serial.print  (F("[SMS-NETMODE] "));
    Serial.print  (name);
    Serial.println(smsOk ? F(" success") : F(" failed"));
#endif
    return smsOk;
}

bool Modem::isOperatorMegaFon() const {
    if (_operatorName[0] == 0) return false;
    String op(_operatorName);
    op.toUpperCase();
    return op.indexOf("MEGAFON") >= 0;
}

bool Modem::waitForRegistration(uint32_t timeoutMs) {
    const uint32_t deadline = millis() + timeoutMs;
    String resp;
    while ((int32_t)(millis() - deadline) < 0) {
        feedWatchdog();
        delay(2500); feedWatchdog();
        delay(2500); feedWatchdog();

        sendAT("AT+CREG?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
        parseCregLike(resp, "+CREG:",  _cregStatus);             feedWatchdog();
        sendAT("AT+CEREG?", resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
        parseCregLike(resp, "+CEREG:", _ceregStatus);            feedWatchdog();
        sendAT("AT+CPSI?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
        sendAT("AT+CSQ",    resp, 1000);
        parseCsq(resp);                                          feedWatchdog();

        _registered = (_cregStatus  == 1 || _cregStatus  == 5 ||
                       _ceregStatus == 1 || _ceregStatus == 5);
        if (_registered) return true;
    }
    return false;
}

void Modem::forceWcdmaForSmsIfNeeded() {
#if SMS_FORCE_WCDMA_FOR_MEGAFON
    if (!_atOk) return;
    if (!isOperatorMegaFon()) return;

#if DEBUG_MODEM
    Serial.println(F("[SMS-NETMODE] MegaFon detected, forcing WCDMA_ONLY for SMS reliability"));
#endif

    String resp;
    const bool ok = sendAT(SMS_WCDMA_CNMP_CMD, resp, 5000);
    feedWatchdog();

#if DEBUG_MODEM
    if (ok) {
        Serial.print  (F("[SMS-NETMODE] "));
        Serial.print  (SMS_WCDMA_CNMP_CMD);
        Serial.println(F(" -> OK"));
    } else {
        Serial.print  (F("[SMS-NETMODE] "));
        Serial.print  (SMS_WCDMA_CNMP_CMD);
        Serial.println(F(" failed"));
    }
#endif
    if (!ok) return;

    const bool reg = waitForRegistration(60000UL);
#if DEBUG_MODEM
    Serial.print  (F("[SMS-NETMODE] WCDMA registered="));
    Serial.println(reg ? F("yes") : F("no"));
#endif
#endif
}

bool Modem::handleNetWcdma(const String& sender) {
#if DEBUG_MODEM
    Serial.println(F("[SMS-NETMODE] NETWCDMA"));
#endif
    String resp;
    const bool ok = sendAT(SMS_WCDMA_CNMP_CMD, resp, 5000);
    feedWatchdog();

    bool reg = false;
    if (ok) reg = waitForRegistration(60000UL);

    String reply = "NETWCDMA: ";
    reply += ok  ? "set"          : "set FAILED";
    reply += ", registered=";
    reply += reg ? "yes" : "no";
    sendSms(sender.c_str(), reply);
    return ok && reg;
}

bool Modem::handleNetAuto(const String& sender) {
#if DEBUG_MODEM
    Serial.println(F("[SMS-NETMODE] NETAUTO"));
#endif
    String resp;
    bool ok = sendAT(SMS_AUTO_CNMP_CMD, resp, 5000);
    feedWatchdog();

    if (!ok) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-NETMODE] CNMP=2 refused, trying deregister cycle"));
#endif
        sendAT("AT+COPS=2", resp, 30000); feedWatchdog();
        ok = sendAT(SMS_AUTO_CNMP_CMD, resp, 5000); feedWatchdog();
        sendAT("AT+COPS=0", resp, 30000); feedWatchdog();
    }

    const bool reg = waitForRegistration(60000UL);

    String reply = "NETAUTO: ";
    reply += ok  ? "set"          : "set FAILED";
    reply += ", registered=";
    reply += reg ? "yes" : "no";
    sendSms(sender.c_str(), reply);
    return ok && reg;
}

bool Modem::handleTestSms2G(const String& sender) {
#if DEBUG_MODEM
    Serial.println(F("[SMS-NETMODE] TESTSMS2G start"));
#endif

    String resp;

    // 1. Baseline snapshot (every sendAT echoes via [AT>]/[AT<]).
    sendAT("AT+CPSI?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CNMP?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CGSMS?", resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CSCA?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CREG?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CEREG?", resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
    sendAT("AT+CSQ",    resp, 1000);                         feedWatchdog();
    sendAT("AT+COPS?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();

    // 2. Non-LTE candidates, in order. SIM7600 CNMP values:
    //      2 = AUTO, 13 = GSM only, 14 = WCDMA only, 48 = GSM+WCDMA,
    //      38 = LTE only.
    // A CNMP value that the firmware rejects with ERROR simply skips
    // to the next entry — we don't treat unsupported modes as fatal.
    struct ModeCandidate { const char* name; const char* cnmp; };
    static const ModeCandidate kModes[] = {
        { "WCDMA_ONLY", "AT+CNMP=14" },
        { "GSM_ONLY",   "AT+CNMP=13" },
        { "GSM_WCDMA",  "AT+CNMP=48" },
    };

    bool anySuccess = false;
    for (const auto& m : kModes) {
        if (trySmsInNetworkMode(sender, m.name, m.cnmp)) {
            anySuccess = true;
            break;
        }
    }

    if (!anySuccess) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-NETMODE] TESTSMS2G failed in all modes"));
#endif
    }

    // 3. Cleanup. If the MegaFon SMS workaround is on and the test
    //    succeeded in a non-LTE mode, keep that mode — restoring AUTO
    //    would re-introduce the +CMS ERROR bug. Otherwise restore AUTO
    //    and let the modem re-register normally.
#if SMS_FORCE_WCDMA_FOR_MEGAFON
    const bool keepWcdma = anySuccess && isOperatorMegaFon();
#else
    const bool keepWcdma = false;
#endif

    if (keepWcdma) {
#if DEBUG_MODEM
        Serial.println(F("[SMS-NETMODE] keeping WCDMA_ONLY because MegaFon LTE SMS fails"));
#endif
    } else {
#if DEBUG_MODEM
        Serial.println(F("[SMS-NETMODE] restoring default mode"));
#endif
        sendAT(SMS_AUTO_CNMP_CMD, resp, 5000); feedWatchdog();

        const uint32_t restoreDeadline = millis() + 60000UL;
        while ((int32_t)(millis() - restoreDeadline) < 0) {
            feedWatchdog();
            delay(2500); feedWatchdog();
            delay(2500); feedWatchdog();

            sendAT("AT+CPSI?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
            sendAT("AT+CREG?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
            parseCregLike(resp, "+CREG:",  _cregStatus);             feedWatchdog();
            sendAT("AT+CEREG?", resp, SMS_PROFILE_SETUP_TIMEOUT_MS);
            parseCregLike(resp, "+CEREG:", _ceregStatus);            feedWatchdog();
            sendAT("AT+COPS?",  resp, SMS_PROFILE_SETUP_TIMEOUT_MS); feedWatchdog();
            sendAT("AT+CSQ",    resp, 1000);
            parseCsq(resp);                                          feedWatchdog();

            if (_cregStatus  == 1 || _cregStatus  == 5 ||
                _ceregStatus == 1 || _ceregStatus == 5) break;
        }
#if DEBUG_MODEM
        Serial.println(F("[SMS-NETMODE] restored default mode"));
#endif
    }

    // 4. On total failure, dump extended diagnostics so CEER reflects the
    //    final post-test state of the modem.
    if (!anySuccess) {
        printSmsDiagnostics("TESTSMS2G_FAILED");
    }

    return anySuccess;
}
