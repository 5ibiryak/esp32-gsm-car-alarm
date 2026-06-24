#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "config.h"

class AlarmStateMachine;
class Modem;
class BatteryMonitor;
class ReedSensor;
class Accelerometer;

// Wi-Fi SoftAP + HTTP OTA upload + runtime configuration UI.
//
// Wi-Fi is normally off. begin() prints partition info but does NOT start
// the AP. start() (called from SMS command 8) brings up the AP, listens
// on OTA_HTTP_PORT, and serves:
//   GET  /              — index page (FW + uptime + links + upload form)
//   GET  /settings      — runtime config form
//   POST /settings      — save validated values to NVS, apply immediately
//   POST /settings/reset— restore defaults and persist
//   GET  /settings.json — current config dump
//   GET  /status        — plain-text live status snapshot
//   POST /update        — firmware.bin upload
// After a successful upload the device reboots into the new image. If no
// upload happens within runtimeConfig.otaActiveWindowMs() the AP is
// auto-disabled.
class OtaManager {
public:
    void   begin();
    bool   start();                 // returns true if AP / HTTP came up
    void   stop();                  // graceful (won't stop mid-upload)
    void   update();                // tick from main loop

    bool   isActive() const            { return _active; }
    bool   isUploadInProgress() const  { return _uploadInProgress; }
    String getCurrentFirmwareVersion() const { return String(FW_VERSION); }
    String getStatusText() const;
    String getActivationReplyText() const;

    // Optional wiring for the /status page.
    void   setAlarmFsm   (AlarmStateMachine* fsm)  { _fsm     = fsm;     }
    void   setModem      (Modem* m)                { _modem   = m;       }
    void   setBattery    (BatteryMonitor* b)       { _battery = b;       }
    void   setReed       (ReedSensor* r)           { _reed    = r;       }
    void   setAccel      (Accelerometer* a)        { _accel   = a;       }

private:
    void   handleRoot();
    void   handleStatus();
    void   handleSettingsGet();
    void   handleSettingsPost();
    void   handleSettingsReset();
    void   handleSettingsJson();
    void   handleNotFound();
    void   handleUpdateChunk();
    void   handleUpdateComplete();

    // POST-helpers. Each returns true if the value was accepted.
    // Not const — WebServer::hasArg() / WebServer::arg() are non-const.
    bool   takeFloat (const char* name, float&    out);
    bool   takeLong  (const char* name, long&     out);
    bool   takeInt   (const char* name, int&      out);
    void   takeBool  (const char* name, bool&     out);

    String renderNavHtml() const;
    String renderSettingsForm(const String& warnings) const;
    String renderResultPage(const String& msg, const String& warnings) const;

    WebServer _server { OTA_HTTP_PORT };

    bool      _active             = false;
    bool      _uploadInProgress   = false;
    uint32_t  _startedAtMs        = 0;
    uint32_t  _activeWindowMs     = OTA_ACTIVE_WINDOW_MS;
    size_t    _uploadedBytes      = 0;
    size_t    _lastProgressLogB   = 0;

    AlarmStateMachine* _fsm     = nullptr;
    Modem*             _modem   = nullptr;
    BatteryMonitor*    _battery = nullptr;
    ReedSensor*        _reed    = nullptr;
    Accelerometer*     _accel   = nullptr;

    static OtaManager* s_instance;   // set in start(), used by route lambdas
};
