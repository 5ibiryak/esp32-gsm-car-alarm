#include "OtaManager.h"
#include "config.h"
#include "RuntimeConfig.h"
#include "AlarmStateMachine.h"
#include "Modem.h"
#include "BatteryMonitor.h"
#include "ReedSensor.h"
#include "Accelerometer.h"

#include <WiFi.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <math.h>

#if WATCHDOG_ENABLED
#include <esp_task_wdt.h>
#endif

static inline void otaFeedWatchdog() {
#if WATCHDOG_ENABLED
    esp_task_wdt_reset();
#endif
}

OtaManager* OtaManager::s_instance = nullptr;

void OtaManager::begin() {
    Serial.println(F("[OTA] manager ready"));
    Serial.print  (F("[OTA] FW version="));
    Serial.println(FW_VERSION);

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next    = esp_ota_get_next_update_partition(NULL);
    if (running) {
        Serial.print  (F("[OTA] partition: running="));
        Serial.println(running->label);
    }
    if (next) {
        Serial.print  (F("[OTA] free OTA space="));
        Serial.print  ((unsigned)next->size);
        Serial.println(F(" bytes"));
    }

    // Make sure the radio is off until SMS command 8 asks for it.
    WiFi.mode(WIFI_OFF);
    Serial.println(F("[OTA] WiFi initially off"));
}

bool OtaManager::start() {
    if (_active) {
        Serial.println(F("[OTA] already active"));
        return true;
    }

    _uploadInProgress   = false;
    _uploadedBytes      = 0;
    _lastProgressLogB   = 0;

    WiFi.mode(WIFI_AP);
    const bool apOk = WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD,
                                  OTA_AP_CHANNEL, /*hidden*/false,
                                  OTA_AP_MAX_CLIENTS);
    if (!apOk) {
        Serial.println(F("[OTA] WiFi softAP start failed"));
        WiFi.mode(WIFI_OFF);
        return false;
    }

    s_instance = this;
    _server.on("/",                 HTTP_GET,  []() { if (s_instance) s_instance->handleRoot();          });
    _server.on("/status",           HTTP_GET,  []() { if (s_instance) s_instance->handleStatus();        });
    _server.on("/settings",         HTTP_GET,  []() { if (s_instance) s_instance->handleSettingsGet();   });
    _server.on("/settings",         HTTP_POST, []() { if (s_instance) s_instance->handleSettingsPost();  });
    _server.on("/settings/reset",   HTTP_POST, []() { if (s_instance) s_instance->handleSettingsReset(); });
    _server.on("/settings.json",    HTTP_GET,  []() { if (s_instance) s_instance->handleSettingsJson();  });
    _server.on("/update",           HTTP_POST,
        []() { if (s_instance) s_instance->handleUpdateComplete(); },
        []() { if (s_instance) s_instance->handleUpdateChunk();    });
    _server.onNotFound(             []() { if (s_instance) s_instance->handleNotFound();      });
    _server.begin();

    _active          = true;
    _startedAtMs     = millis();
    // Snapshot the active window at start so a config change mid-session
    // does not retroactively shrink/extend the running OTA window.
    _activeWindowMs  = runtimeConfig.otaActiveWindowMs();

    Serial.print  (F("[OTA] AP started ssid="));
    Serial.print  (OTA_AP_SSID);
    Serial.print  (F(" ip="));
    Serial.println(WiFi.softAPIP());
    Serial.print  (F("[OTA] active window="));
    Serial.print  (_activeWindowMs / 1000UL);
    Serial.println(F("s"));
    return true;
}

void OtaManager::stop() {
    if (!_active) return;
    if (_uploadInProgress) return;        // never tear down mid-upload

    _server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);

    _active    = false;
    s_instance = nullptr;
    Serial.println(F("[OTA] stopped"));
}

void OtaManager::update() {
    if (!_active) return;

    _server.handleClient();

    const uint32_t now = millis();
    if (!_uploadInProgress && (now - _startedAtMs) > _activeWindowMs) {
        Serial.println(F("[OTA] timeout, WiFi disabled"));
        stop();
    }
}

String OtaManager::getActivationReplyText() const {
    String t = "MoskvichAlarm OTA:\nWiFi: ";
    t += OTA_AP_SSID;
    t += "\nPass: ";
    t += OTA_AP_PASSWORD;
    t += "\nURL: http://192.168.4.1";
    t += "\nActive: ";
    t += String(runtimeConfig.otaActiveWindowMs() / 60000UL);
    t += " min";
    return t;
}

String OtaManager::getStatusText() const {
    return _active ? String("OTA=active") : String("OTA=inactive");
}

// ----- HTTP handlers -------------------------------------------------------

String OtaManager::renderNavHtml() const {
    String n;
    n += F("<nav style='margin-bottom:1em'>");
    n += F("<a href='/'>Home</a> | ");
    n += F("<a href='/settings'>Settings</a> | ");
    n += F("<a href='/status'>Status</a> | ");
    n += F("<a href='/settings.json'>JSON</a>");
    n += F("</nav>");
    return n;
}

void OtaManager::handleRoot() {
    const uint32_t now      = millis();
    const uint32_t elapsed  = now - _startedAtMs;
    const uint32_t remainSec = (elapsed < _activeWindowMs)
                                ? (_activeWindowMs - elapsed) / 1000UL
                                : 0;

    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next    = esp_ota_get_next_update_partition(NULL);

    String html;
    html.reserve(1500);
    html += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
    html += F("<title>MoskvichAlarm</title></head><body>");
    html += F("<h1>MoskvichAlarm</h1>");
    html += renderNavHtml();
    html += F("<p>FW: "); html += FW_VERSION; html += F("</p>");
    html += F("<p>Build: "); html += FW_BUILD_DATE; html += F(" "); html += FW_BUILD_TIME; html += F("</p>");
    if (running) {
        html += F("<p>Partition: "); html += running->label; html += F("</p>");
    }
    if (next) {
        html += F("<p>Free OTA space: ");
        html += String((unsigned long)next->size);
        html += F(" bytes</p>");
    }
    html += F("<p>OTA time left: "); html += String(remainSec); html += F("s</p>");
    html += F("<h2>Firmware upload</h2>");
    html += F("<form method='POST' action='/update' enctype='multipart/form-data'>");
    html += F("<input type='file' name='firmware'> ");
    html += F("<input type='submit' value='Upload firmware.bin'>");
    html += F("</form>");
    html += F("<p><b>WARNING:</b> upload firmware.bin only.</p>");
    html += F("</body></html>");

    _server.send(200, "text/html", html);
}

void OtaManager::handleStatus() {
    const uint32_t now       = millis();
    const uint32_t elapsed   = _active ? (now - _startedAtMs) : 0;
    const uint32_t remainSec = (_active && elapsed < _activeWindowMs)
                                ? (_activeWindowMs - elapsed) / 1000UL
                                : 0;

    String t;
    t.reserve(700);
    t += F("FW=");        t += FW_VERSION;                  t += '\n';
    t += F("build=");     t += FW_BUILD_DATE;
    t += ' ';             t += FW_BUILD_TIME;               t += '\n';
    t += F("uptime=");    t += String(now / 1000UL);        t += "s\n";
    t += F("ota_active=");      t += (_active ? "yes" : "no"); t += '\n';
    t += F("ota_upload=");      t += (_uploadInProgress ? "yes" : "no"); t += '\n';
    t += F("ota_time_left=");   t += String(remainSec);     t += "s\n";

    if (_fsm) {
        t += F("state=");        t += _fsm->stateName();
        if (_fsm->state() == AlarmStateMachine::ALARM) {
            t += F(" reason=");
            t += _fsm->triggerReasonName();
        }
        t += '\n';
    }
    if (_reed)  { t += F("reed=");   t += (_reed->isOpen() ? "OPEN" : "CLOSED"); t += '\n'; }
    if (_accel) {
        t += F("smartMotionEnabled="); t += (runtimeConfig.smartMotionEnabled() ? "yes" : "no"); t += '\n';
        t += F("detectorState=");      t += _accel->stateName(); t += '\n';
        t += F("lastMotionReason=");
        t += Accelerometer::reasonName(_accel->currentMotionReason()); t += '\n';
        t += F("motion=");  t += (_accel->isMotionDetected() ? "1" : "0"); t += '\n';
        t += F("mag=");     t += String(_accel->getMagnitude(), 3); t += '\n';
        t += F("delta=");   t += String(_accel->getDeltaG(), 3); t += '\n';
        t += F("jerk=");    t += String(_accel->getJerkG(), 3); t += '\n';
        t += F("tiltDeg="); t += String(_accel->getTiltDeg(), 1); t += '\n';
        t += F("noiseRms=");t += String(_accel->getNoiseRmsG(), 4); t += '\n';
        t += F("moveRms="); t += String(_accel->getMoveRmsG(), 3); t += '\n';
        t += F("baseline=("); t += String(_accel->getBaselineX(), 3); t += ',';
                              t += String(_accel->getBaselineY(), 3); t += ',';
                              t += String(_accel->getBaselineZ(), 3); t += ")\n";
        t += F("current=(");  t += String(_accel->getXg(), 3); t += ',';
                              t += String(_accel->getYg(), 3); t += ',';
                              t += String(_accel->getZg(), 3); t += ")\n";
        t += F("timeSinceArmed=");
        t += String(_accel->timeSinceArmedMs() / 1000UL); t += "s\n";
    }
    if (_battery) {
        if (_battery->hasValidMeasurement()) {
            t += F("battery_v=");  t += String(_battery->getVoltageV(), 2); t += '\n';
            t += F("battery_mA="); t += String((long)lroundf(_battery->getCurrentA() * 1000.0f)); t += '\n';
            t += F("battery_est=");
            if (_battery->hasValidEstimate()) {
                t += String(_battery->getEstimatedRemainingDays(), 1);
                t += "d";
            } else {
                t += "UNKNOWN";
            }
            t += '\n';
        } else {
            t += F("battery_v=UNKNOWN\n");
        }
    }
    if (_modem) {
        t += F("modem_op=");      t += _modem->getOperator(); t += '\n';
        t += F("modem_csq=");     t += String(_modem->getSignalQuality()); t += '\n';
        t += F("registered=");    t += (_modem->isNetworkRegistered() ? "yes" : "no"); t += '\n';
        t += F("modem_cpsi=");    t += _modem->getCpsi(); t += '\n';
    }
    _server.send(200, "text/plain", t);
}

// ----- Settings page -------------------------------------------------------

String OtaManager::renderSettingsForm(const String& warnings) const {
    const RuntimeConfig& c = runtimeConfig;
    String h;
    h.reserve(6000);
    h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
    h += F("<title>MoskvichAlarm settings</title></head><body>");
    h += F("<h1>Runtime settings</h1>");
    h += renderNavHtml();
    if (warnings.length() > 0) {
        h += F("<div style='color:#a00'><b>Warnings:</b><pre>");
        h += warnings;
        h += F("</pre></div>");
    }
    h += F("<form method='POST' action='/settings'>");

    // 1. Motion (legacy single-threshold)
    h += F("<h2>1. Motion (legacy)</h2>");
    h += F("Legacy delta threshold, g: <input name='motionDeltaThresholdG' type='number' step='0.001' min='0.005' max='0.300' value='");
    h += String(c.motionDeltaThresholdG(), 3); h += F("'> (0.005..0.300)<br>");
    h += F("Motion confirm samples: <input name='motionConfirmSamples' type='number' min='1' max='10' value='");
    h += String(c.motionConfirmSamples()); h += F("'> (1..10)<br>");
    h += F("Motion cooldown, ms: <input name='motionCooldownMs' type='number' min='0' max='60000' value='");
    h += String(c.motionCooldownMs()); h += F("'> (0..60000)<br>");

    // 1b. Motion — Smart Detector
    h += F("<h2>1b. Motion — Smart Detector</h2>");
    h += F("Smart motion enabled: <input name='smartMotionEnabled' type='checkbox' value='1'");
    if (c.smartMotionEnabled()) h += F(" checked");
    h += F("><br>");
    h += F("Calibration time, sec: <input name='motionCalibrationSec' type='number' min='5' max='180' value='");
    h += String(c.motionCalibrationMs() / 1000UL); h += F("'> (5..180)<br>");
    h += F("Baseline alpha (quiet): <input name='motionBaselineAlphaQuiet' type='number' step='0.0001' min='0.0001' max='0.05' value='");
    h += String(c.motionBaselineAlphaQuiet(), 4); h += F("'> (0.0001..0.05)<br>");
    h += F("Impact threshold, g: <input name='impactThresholdG' type='number' step='0.001' min='0.020' max='0.500' value='");
    h += String(c.impactThresholdG(), 3); h += F("'> (0.020..0.500)<br>");
    h += F("Impact confirm window, ms: <input name='impactConfirmWindowMs' type='number' min='100' max='5000' value='");
    h += String(c.impactConfirmWindowMs()); h += F("'> (100..5000)<br>");
    h += F("Impact min peaks: <input name='impactMinPeaks' type='number' min='1' max='5' value='");
    h += String(c.impactMinPeaks()); h += F("'> (1..5)<br>");
    h += F("Tilt threshold, deg: <input name='tiltThresholdDeg' type='number' step='0.1' min='1.0' max='30.0' value='");
    h += String(c.tiltThresholdDeg(), 1); h += F("'> (1..30)<br>");
    h += F("Tilt confirm, sec: <input name='tiltConfirmSec' type='number' min='1' max='30' value='");
    h += String(c.tiltConfirmMs() / 1000UL); h += F("'> (1..30)<br>");
    h += F("Move RMS threshold, g: <input name='moveRmsThresholdG' type='number' step='0.001' min='0.005' max='0.200' value='");
    h += String(c.moveRmsThresholdG(), 3); h += F("'> (0.005..0.200)<br>");
    h += F("Move confirm, sec: <input name='moveConfirmSec' type='number' min='1' max='60' value='");
    h += String(c.moveConfirmMs() / 1000UL); h += F("'> (1..60)<br>");
    h += F("Move min active percent: <input name='moveMinActivePercent' type='number' min='5' max='100' value='");
    h += String(c.moveMinActivePercent()); h += F("'> (5..100)<br>");
    h += F("Early stabilization window, min: <input name='motionEarlyWindowMin' type='number' min='0' max='30' value='");
    h += String(c.motionEarlyWindowMs() / 60000UL); h += F("'> (0..30)<br>");
    h += F("Early multiplier: <input name='motionEarlyMultiplier' type='number' step='0.1' min='1.0' max='5.0' value='");
    h += String(c.motionEarlyMultiplier(), 2); h += F("'> (1..5)<br>");

    // 2. Arming
    h += F("<h2>2. Arming</h2>");
    h += F("Arming delay, sec: <input name='armingDelaySec' type='number' min='0' max='120' value='");
    h += String(c.armingDelayMs() / 1000UL); h += F("'> (0..120)<br>");
    h += F("Armed sensor grace, ms: <input name='armedSensorGraceMs' type='number' min='0' max='30000' value='");
    h += String(c.armedSensorGraceMs()); h += F("'> (0..30000)<br>");

    // 3. Siren
    h += F("<h2>3. Siren</h2>");
    h += F("Siren continuous time, sec: <input name='sirenAlarmContinuousSec' type='number' min='10' max='600' value='");
    h += String(c.sirenAlarmContinuousMs() / 1000UL); h += F("'> (10..600)<br>");
    h += F("Reminder after timeout: <input name='sirenAfterTimeoutReminderEnabled' type='checkbox' value='1'");
    if (c.sirenAfterTimeoutReminderEnabled()) h += F(" checked");
    h += F("><br>");
    h += F("Reminder ON, ms: <input name='sirenReminderOnMs' type='number' min='50' max='5000' value='");
    h += String(c.sirenReminderOnMs()); h += F("'> (50..5000)<br>");
    h += F("Reminder OFF, ms: <input name='sirenReminderOffMs' type='number' min='1000' max='60000' value='");
    h += String(c.sirenReminderOffMs()); h += F("'> (1000..60000)<br>");

    // 4. Battery
    h += F("<h2>4. Battery</h2>");
    h += F("Low battery SMS enabled: <input name='lowBatterySmsEnabled' type='checkbox' value='1'");
    if (c.lowBatterySmsEnabled()) h += F(" checked");
    h += F("><br>");
    h += F("Battery capacity, Ah: <input name='batteryCapacityAh' type='number' step='0.1' min='1.0' max='300.0' value='");
    h += String(c.batteryCapacityAh(), 1); h += F("'> (1..300)<br>");
    h += F("Warn voltage, V: <input name='batteryWarnVoltage' type='number' step='0.1' min='10.0' max='15.0' value='");
    h += String(c.batteryWarnVoltage(), 2); h += F("'> (10..15)<br>");
    h += F("Critical voltage, V: <input name='batteryCriticalVoltage' type='number' step='0.1' min='10.0' max='15.0' value='");
    h += String(c.batteryCriticalVoltage(), 2); h += F("'> (10..15)<br>");
    h += F("Shutdown voltage, V: <input name='batteryShutdownVoltage' type='number' step='0.1' min='9.0' max='15.0' value='");
    h += String(c.batteryShutdownVoltage(), 2); h += F("'> (9..15)<br>");
    h += F("Reserve days: <input name='lowBatteryReserveDays' type='number' step='0.5' min='0.5' max='30.0' value='");
    h += String(c.lowBatteryReserveDays(), 1); h += F("'> (0.5..30)<br>");
    h += F("SMS repeat, hours: <input name='lowBatterySmsRepeatHours' type='number' min='1' max='168' value='");
    h += String(c.lowBatterySmsRepeatHours()); h += F("'> (1..168)<br>");

    // 5. Logging
    h += F("<h2>5. Logging</h2>");
    h += F("State log interval, ms: <input name='stateLogIntervalMs' type='number' min='1000' max='60000' value='");
    h += String(c.stateLogIntervalMs()); h += F("'> (1000..60000)<br>");
    h += F("INA226 raw log interval, ms: <input name='inaRawLogIntervalMs' type='number' min='5000' max='300000' value='");
    h += String(c.inaRawLogIntervalMs()); h += F("'> (5000..300000)<br>");

    // 6. OTA
    h += F("<h2>6. OTA</h2>");
    h += F("OTA active window, sec: <input name='otaActiveWindowSec' type='number' min='60' max='1800' value='");
    h += String(c.otaActiveWindowMs() / 1000UL); h += F("'> (60..1800)<br>");

    h += F("<p><input type='submit' value='Save'></p>");
    h += F("</form>");
    h += F("<form method='POST' action='/settings/reset'>");
    h += F("<input type='submit' value='Reset settings to defaults'>");
    h += F("</form>");
    h += F("</body></html>");
    return h;
}

void OtaManager::handleSettingsGet() {
    _server.send(200, "text/html", renderSettingsForm(String("")));
}

bool OtaManager::takeFloat(const char* name, float& out) {
    if (!_server.hasArg(name)) return false;
    const String v = _server.arg(name);
    if (v.length() == 0) return false;
    const float f = v.toFloat();
    // toFloat returns 0 on garbage — guard against NaN/inf and pure-zero
    // values where the input wasn't actually "0".
    if (!isfinite(f)) return false;
    if (f == 0.0f && v != "0" && v != "0.0" && v != "0.00") return false;
    out = f;
    return true;
}

bool OtaManager::takeLong(const char* name, long& out) {
    if (!_server.hasArg(name)) return false;
    const String v = _server.arg(name);
    if (v.length() == 0) return false;
    char* end = nullptr;
    const long parsed = strtol(v.c_str(), &end, 10);
    if (end == v.c_str() || *end != '\0') return false;
    out = parsed;
    return true;
}

bool OtaManager::takeInt(const char* name, int& out) {
    long v = 0;
    if (!takeLong(name, v)) return false;
    out = (int)v;
    return true;
}

void OtaManager::takeBool(const char* name, bool& out) {
    out = _server.hasArg(name);
}

void OtaManager::handleSettingsPost() {
    String warnings;
    RuntimeConfig& c = runtimeConfig;

    auto reject = [&](const char* field) {
        const String v = _server.arg(field);
        Serial.print  (F("[CFG] invalid web value "));
        Serial.print  (field);
        Serial.print  (F("="));
        Serial.print  (v);
        Serial.println(F(", keeping previous"));
        warnings += field; warnings += F("=");
        warnings += v;     warnings += F(" rejected\n");
    };

    {
        float fv;
        if (takeFloat("motionDeltaThresholdG", fv)) {
            if (!c.setMotionDeltaThresholdG(fv)) reject("motionDeltaThresholdG");
            else { Serial.print(F("[CFG] motionDeltaThresholdG=")); Serial.println(fv, 3); }
        }
    }
    {
        int iv;
        if (takeInt("motionConfirmSamples", iv)) {
            if (!c.setMotionConfirmSamples(iv)) reject("motionConfirmSamples");
            else { Serial.print(F("[CFG] motionConfirmSamples=")); Serial.println(iv); }
        }
    }
    {
        long lv;
        if (takeLong("motionCooldownMs", lv)) {
            if (!c.setMotionCooldownMs(lv)) reject("motionCooldownMs");
            else { Serial.print(F("[CFG] motionCooldownMs=")); Serial.println(lv); }
        }
        if (takeLong("armingDelaySec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setArmingDelayMs(ms)) reject("armingDelaySec");
            else { Serial.print(F("[CFG] armingDelayMs=")); Serial.println(ms); }
        }
        if (takeLong("armedSensorGraceMs", lv)) {
            if (!c.setArmedSensorGraceMs(lv)) reject("armedSensorGraceMs");
            else { Serial.print(F("[CFG] armedSensorGraceMs=")); Serial.println(lv); }
        }
        if (takeLong("sirenAlarmContinuousSec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setSirenAlarmContinuousMs(ms)) reject("sirenAlarmContinuousSec");
            else { Serial.print(F("[CFG] sirenAlarmContinuousMs=")); Serial.println(ms); }
        }
        if (takeLong("sirenReminderOnMs", lv)) {
            if (!c.setSirenReminderOnMs(lv)) reject("sirenReminderOnMs");
            else { Serial.print(F("[CFG] sirenReminderOnMs=")); Serial.println(lv); }
        }
        if (takeLong("sirenReminderOffMs", lv)) {
            if (!c.setSirenReminderOffMs(lv)) reject("sirenReminderOffMs");
            else { Serial.print(F("[CFG] sirenReminderOffMs=")); Serial.println(lv); }
        }
        if (takeLong("stateLogIntervalMs", lv)) {
            if (!c.setStateLogIntervalMs(lv)) reject("stateLogIntervalMs");
            else { Serial.print(F("[CFG] stateLogIntervalMs=")); Serial.println(lv); }
        }
        if (takeLong("inaRawLogIntervalMs", lv)) {
            if (!c.setInaRawLogIntervalMs(lv)) reject("inaRawLogIntervalMs");
            else { Serial.print(F("[CFG] inaRawLogIntervalMs=")); Serial.println(lv); }
        }
        if (takeLong("otaActiveWindowSec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setOtaActiveWindowMs(ms)) reject("otaActiveWindowSec");
            else { Serial.print(F("[CFG] otaActiveWindowMs=")); Serial.println(ms); }
        }
    }
    // Booleans: present = true, absent = false. This means a POST always
    // overwrites bool state — that is the intentional "form submit"
    // semantics for checkboxes.
    bool bv;
    takeBool("sirenAfterTimeoutReminderEnabled", bv);
    c.setSirenAfterTimeoutReminderEnabled(bv);
    Serial.print(F("[CFG] sirenAfterTimeoutReminderEnabled="));
    Serial.println(bv ? "true" : "false");

    takeBool("lowBatterySmsEnabled", bv);
    c.setLowBatterySmsEnabled(bv);
    Serial.print(F("[CFG] lowBatterySmsEnabled="));
    Serial.println(bv ? "true" : "false");

    takeBool("smartMotionEnabled", bv);
    c.setSmartMotionEnabled(bv);
    Serial.print(F("[CFG] smartMotionEnabled="));
    Serial.println(bv ? "true" : "false");

    {
        long lv;
        if (takeLong("motionCalibrationSec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setMotionCalibrationMs(ms)) reject("motionCalibrationSec");
            else { Serial.print(F("[CFG] motionCalibrationMs=")); Serial.println(ms); }
        }
        if (takeLong("impactConfirmWindowMs", lv)) {
            if (!c.setImpactConfirmWindowMs(lv)) reject("impactConfirmWindowMs");
            else { Serial.print(F("[CFG] impactConfirmWindowMs=")); Serial.println(lv); }
        }
        if (takeLong("tiltConfirmSec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setTiltConfirmMs(ms)) reject("tiltConfirmSec");
            else { Serial.print(F("[CFG] tiltConfirmMs=")); Serial.println(ms); }
        }
        if (takeLong("moveConfirmSec", lv)) {
            const long ms = lv * 1000L;
            if (!c.setMoveConfirmMs(ms)) reject("moveConfirmSec");
            else { Serial.print(F("[CFG] moveConfirmMs=")); Serial.println(ms); }
        }
        if (takeLong("motionEarlyWindowMin", lv)) {
            const long ms = lv * 60000L;
            if (!c.setMotionEarlyWindowMs(ms)) reject("motionEarlyWindowMin");
            else { Serial.print(F("[CFG] motionEarlyWindowMs=")); Serial.println(ms); }
        }
    }
    {
        int iv;
        if (takeInt("impactMinPeaks", iv)) {
            if (!c.setImpactMinPeaks(iv)) reject("impactMinPeaks");
            else { Serial.print(F("[CFG] impactMinPeaks=")); Serial.println(iv); }
        }
        if (takeInt("moveMinActivePercent", iv)) {
            if (!c.setMoveMinActivePercent(iv)) reject("moveMinActivePercent");
            else { Serial.print(F("[CFG] moveMinActivePercent=")); Serial.println(iv); }
        }
    }
    {
        float fv;
        if (takeFloat("motionBaselineAlphaQuiet", fv)) {
            if (!c.setMotionBaselineAlphaQuiet(fv)) reject("motionBaselineAlphaQuiet");
            else { Serial.print(F("[CFG] motionBaselineAlphaQuiet=")); Serial.println(fv, 4); }
        }
        if (takeFloat("impactThresholdG", fv)) {
            if (!c.setImpactThresholdG(fv)) reject("impactThresholdG");
            else { Serial.print(F("[CFG] impactThresholdG=")); Serial.println(fv, 3); }
        }
        if (takeFloat("tiltThresholdDeg", fv)) {
            if (!c.setTiltThresholdDeg(fv)) reject("tiltThresholdDeg");
            else { Serial.print(F("[CFG] tiltThresholdDeg=")); Serial.println(fv, 1); }
        }
        if (takeFloat("moveRmsThresholdG", fv)) {
            if (!c.setMoveRmsThresholdG(fv)) reject("moveRmsThresholdG");
            else { Serial.print(F("[CFG] moveRmsThresholdG=")); Serial.println(fv, 3); }
        }
        if (takeFloat("motionEarlyMultiplier", fv)) {
            if (!c.setMotionEarlyMultiplier(fv)) reject("motionEarlyMultiplier");
            else { Serial.print(F("[CFG] motionEarlyMultiplier=")); Serial.println(fv, 2); }
        }
    }

    {
        float fv;
        if (takeFloat("batteryCapacityAh", fv)) {
            if (!c.setBatteryCapacityAh(fv)) reject("batteryCapacityAh");
            else { Serial.print(F("[CFG] batteryCapacityAh=")); Serial.println(fv, 1); }
        }
        if (takeFloat("batteryWarnVoltage", fv)) {
            if (!c.setBatteryWarnVoltage(fv)) reject("batteryWarnVoltage");
            else { Serial.print(F("[CFG] batteryWarnVoltage=")); Serial.println(fv, 2); }
        }
        if (takeFloat("batteryCriticalVoltage", fv)) {
            if (!c.setBatteryCriticalVoltage(fv)) reject("batteryCriticalVoltage");
            else { Serial.print(F("[CFG] batteryCriticalVoltage=")); Serial.println(fv, 2); }
        }
        if (takeFloat("batteryShutdownVoltage", fv)) {
            if (!c.setBatteryShutdownVoltage(fv)) reject("batteryShutdownVoltage");
            else { Serial.print(F("[CFG] batteryShutdownVoltage=")); Serial.println(fv, 2); }
        }
        if (takeFloat("lowBatteryReserveDays", fv)) {
            if (!c.setLowBatteryReserveDays(fv)) reject("lowBatteryReserveDays");
            else { Serial.print(F("[CFG] lowBatteryReserveDays=")); Serial.println(fv, 1); }
        }
    }
    {
        int iv;
        if (takeInt("lowBatterySmsRepeatHours", iv)) {
            if (!c.setLowBatterySmsRepeatHours(iv)) reject("lowBatterySmsRepeatHours");
            else { Serial.print(F("[CFG] lowBatterySmsRepeatHours=")); Serial.println(iv); }
        }
    }

    if (!c.saveToNvs()) {
        warnings += F("NVS save failed\n");
        Serial.println(F("[CFG] NVS save failed"));
    }

    _server.send(200, "text/html", renderResultPage(
        F("Settings saved. Some values apply immediately. If behavior looks wrong, use Reset to defaults."),
        warnings));
}

void OtaManager::handleSettingsReset() {
    runtimeConfig.resetToDefaults();
    _server.send(200, "text/html", renderResultPage(
        F("Settings reset to defaults."), String()));
}

String OtaManager::renderResultPage(const String& msg, const String& warnings) const {
    String h;
    h.reserve(800);
    h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>");
    h += F("<title>MoskvichAlarm settings</title></head><body>");
    h += F("<h1>Settings</h1>");
    h += renderNavHtml();
    h += F("<p>"); h += msg; h += F("</p>");
    if (warnings.length() > 0) {
        h += F("<div style='color:#a00'><b>Warnings:</b><pre>");
        h += warnings;
        h += F("</pre></div>");
    }
    h += F("<p><a href='/settings'>Back to settings</a></p>");
    h += F("</body></html>");
    return h;
}

void OtaManager::handleSettingsJson() {
    _server.send(200, "application/json", runtimeConfig.toJson());
}

void OtaManager::handleNotFound() {
    _server.send(404, "text/plain", "Not Found");
}

void OtaManager::handleUpdateChunk() {
    HTTPUpload& upload = _server.upload();

    if (upload.status == UPLOAD_FILE_START) {
        Serial.print  (F("[OTA] upload start filename="));
        Serial.println(upload.filename);
        _uploadInProgress   = true;
        _uploadedBytes      = 0;
        _lastProgressLogB   = 0;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Serial.println(F("[OTA] Update.begin failed"));
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        const size_t written = Update.write(upload.buf, upload.currentSize);
        if (written != upload.currentSize) {
            Serial.println(F("[OTA] write error"));
            Update.printError(Serial);
        }
        _uploadedBytes += upload.currentSize;
        if (_uploadedBytes - _lastProgressLogB >= 65536) {
            _lastProgressLogB = _uploadedBytes;
            Serial.print  (F("[OTA] upload progress bytes="));
            Serial.println(_uploadedBytes);
        }
        otaFeedWatchdog();
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.print  (F("[OTA] update success bytes="));
            Serial.println(_uploadedBytes);
        } else {
            Serial.println(F("[OTA] update failed"));
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.abort();
        Serial.println(F("[OTA] upload aborted"));
        _uploadInProgress = false;
    }
}

void OtaManager::handleUpdateComplete() {
    _server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        _server.send(200, "text/html",
                     "<h1>Update FAILED</h1><p>See serial log for details.</p>");
        _uploadInProgress = false;
        return;
    }

    _server.send(200, "text/html",
                 "<h1>Update OK</h1><p>Rebooting...</p>");
    Serial.println(F("[OTA] rebooting"));
    delay(1000);
    ESP.restart();
}
