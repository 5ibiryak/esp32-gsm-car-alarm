#include <Arduino.h>
#include "config.h"

#if WATCHDOG_ENABLED
#include <esp_task_wdt.h>
#endif
#include <esp_system.h>

#include "BatteryMonitor.h"
#include "Accelerometer.h"
#include "ReedSensor.h"
#include "Siren.h"
#include "StatusLed.h"
#include "Button.h"
#include "Modem.h"
#include "RecipientManager.h"
#include "OtaManager.h"
#include "AlarmStateMachine.h"
#include "RuntimeConfig.h"

static BatteryMonitor    battery;
static Accelerometer     accel;
static ReedSensor        reed;
static Siren             siren;
static StatusLed         led;
static Button            button;
static Modem             modem;
static RecipientManager  recipients;
static OtaManager        ota;
static AlarmStateMachine fsm;

static void printBanner() {
    Serial.println();
    Serial.println(F("======================================================"));
    Serial.print  (F("  ")); Serial.print(FW_NAME);
    Serial.print  (F("  v")); Serial.println(FW_VERSION);
    Serial.print  (F("  build: ")); Serial.println(FW_BUILD);
    Serial.println(F("  target: LILYGO T-SIM7600G-H (ESP32-WROVER)"));
    Serial.println(F("======================================================"));
}

static void printPinMap() {
    Serial.println(F("[PINS] --- external peripherals ---"));
    Serial.print(F("  I2C SDA       = GPIO")); Serial.println(PIN_I2C_SDA);
    Serial.print(F("  I2C SCL       = GPIO")); Serial.println(PIN_I2C_SCL);
    Serial.print(F("  ACCEL_INT     = GPIO")); Serial.println(PIN_ACCEL_INT);
    Serial.print(F("  REED          = GPIO")); Serial.println(PIN_REED);
    Serial.print(F("  SIREN         = GPIO")); Serial.println(PIN_SIREN);
    Serial.print(F("  STATUS_LED    = GPIO")); Serial.println(PIN_STATUS_LED);
    Serial.print(F("  BUTTON        = GPIO")); Serial.print(PIN_BUTTON);
    Serial.println(F(" (unused — physical power switch disarms)"));
    Serial.print(F("  BAT_ADC       = GPIO")); Serial.println(PIN_BAT_ADC);

    Serial.println(F("[PINS] --- SIM7600 modem (LILYGO defaults, do not change) ---"));
    Serial.print(F("  MODEM_TX      = GPIO")); Serial.println(MODEM_PIN_TX);
    Serial.print(F("  MODEM_RX      = GPIO")); Serial.println(MODEM_PIN_RX);
    Serial.print(F("  MODEM_PWRKEY  = GPIO")); Serial.println(MODEM_PIN_PWRKEY);
    Serial.print(F("  MODEM_DTR     = GPIO")); Serial.println(MODEM_PIN_DTR);
    Serial.print(F("  MODEM_BAUD    = "));     Serial.println(MODEM_UART_BAUD);
}

#if DEBUG_I2C_LINE_TEST
// Drives GPIO21 / GPIO22 directly so you can verify with a multimeter
// that the signals reach the sensor's SDA / SCL pads.
//
// LOW phase  -> pinMode(OUTPUT_OPEN_DRAIN) + digitalWrite(LOW)  : active pull-down
// RELEASED   -> digitalWrite(HIGH) + pinMode(INPUT_PULLUP)      : ESP32 internal
//               ~45kΩ pull-up plus any module pull-up bring the line to 3.3V
//
// We need INPUT_PULLUP on release because OUTPUT_OPEN_DRAIN with the pin
// driven HIGH still leaves the line floating if no external pull-up is
// fitted — which is what was happening here (lines sat near 0V).
// Loops forever — power-cycle or reset to exit.
static void releaseLine(uint8_t pin) {
    digitalWrite(pin, HIGH);
    pinMode(pin, INPUT_PULLUP);
}
static void pullLineLow(uint8_t pin) {
    pinMode(pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(pin, LOW);
}

static void runI2cLineTest() {
    Serial.println(F("[I2C-LINE] testing GPIO21/GPIO22 physical wiring"));

    releaseLine(21);
    releaseLine(22);

    for (;;) {
        // Phase A: SDA (GPIO21) actively pulled low, SCL released high.
        Serial.println(F("[I2C-LINE] GPIO21 LOW: measure SDA"));
        pullLineLow(21);
        releaseLine(22);
        delay(10000);

        // Phase B: SDA released high, SCL (GPIO22) actively pulled low.
        Serial.println(F("[I2C-LINE] GPIO22 LOW: measure SCL"));
        releaseLine(21);
        pullLineLow(22);
        delay(10000);

        // Phase C: both released to INPUT_PULLUP — both should read ~3.3 V.
        Serial.println(F("[I2C-LINE] RELEASED: both pins INPUT_PULLUP, measure SDA/SCL"));
        releaseLine(21);
        releaseLine(22);
        delay(10000);
    }
}
#endif

static const char* resetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:    return "POWERON";
        case ESP_RST_EXT:        return "EXT";
        case ESP_RST_SW:         return "SW";
        case ESP_RST_PANIC:      return "PANIC";
        case ESP_RST_INT_WDT:    return "INT_WDT";
        case ESP_RST_TASK_WDT:   return "TASK_WDT";
        case ESP_RST_WDT:        return "WDT";
        case ESP_RST_DEEPSLEEP:  return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:   return "BROWNOUT";
        case ESP_RST_SDIO:       return "SDIO";
        default:                 return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);   // let the host USB enumerate before banner

    const esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.print  (F("[BOOT] reset reason: "));
    Serial.println(resetReasonName(resetReason));
    if (resetReason == ESP_RST_TASK_WDT ||
        resetReason == ESP_RST_INT_WDT  ||
        resetReason == ESP_RST_WDT) {
        Serial.println(F("[BOOT] previous reset was watchdog"));
    }

#if WATCHDOG_ENABLED
    esp_task_wdt_init(WATCHDOG_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);
    Serial.print  (F("[WDT] enabled timeout="));
    Serial.print  (WATCHDOG_TIMEOUT_SEC);
    Serial.println(F("s"));
#endif

    Serial.print  (F("[FW] version="));
    Serial.print  (FW_VERSION);
    Serial.print  (F(" build="));
    Serial.print  (FW_BUILD_DATE);
    Serial.print  (F(" "));
    Serial.println(FW_BUILD_TIME);

#if DEBUG_GPIO19_INPUT_TEST
    // Standalone GPIO19 input test. Runs BEFORE any subsystem init so
    // nothing else in firmware can drive the pin. Plain INPUT, no internal
    // pull-up / pull-down — an external jumper to GND or 3V3 chooses the
    // level. Never returns — power-cycle or flash with the flag set to 0
    // to exit.
    Serial.println();
    Serial.println(F("[GPIO19] standalone GPIO19 INPUT read test — no SPI, no LIS3DH"));
    Serial.println(F("[GPIO19] manually tie GPIO19 to GND or 3V3 to verify reads"));
    pinMode(19, INPUT);
    for (;;) {
        Serial.print(F("[GPIO19] GPIO19 read = "));
        Serial.println(digitalRead(19));
        delay(500);
    }
#endif

    printBanner();
    printPinMap();

#if DEBUG_GPIO23_TEST
    // Standalone GPIO23 toggle test. Runs BEFORE any subsystem init so
    // nothing else in firmware can drive the pin. Never returns —
    // power-cycle or flash with DEBUG_GPIO23_TEST=0 to exit.
    Serial.println(F("[GPIO23] standalone GPIO23 toggle test — no SPI, no LIS3DH"));
    pinMode(23, OUTPUT);
    for (;;) {
        digitalWrite(23, HIGH);
        delay(50);   // settle before readback
        Serial.print(F("GPIO23 HIGH, readback = "));
        Serial.println(digitalRead(23));
        delay(3000);

        digitalWrite(23, LOW);
        delay(50);
        Serial.print(F("GPIO23 LOW, readback = "));
        Serial.println(digitalRead(23));
        delay(3000);
    }
#endif

#if DEBUG_I2C_LINE_TEST
    runI2cLineTest();   // never returns — disable the flag to resume normal boot
#endif

    runtimeConfig.begin();

    // Subsystem bring-up.
    battery.begin();
    accel.begin();
    reed.begin();
    siren.begin();
    led.begin();

#if ALARM_HAS_ARM_BUTTON
    button.begin();
#else
    Serial.println(F("[BTN] disabled — physical power switch is used instead"));
#endif

    recipients.begin();
    modem.setRecipientManager(&recipients);
    modem.begin();
    battery.setModem(&modem);
    ota.begin();
    fsm.begin(&accel, &reed, &led, &siren, &modem, &battery, &ota);
    modem.setAlarmFsm(&fsm);
    ota.setAlarmFsm(&fsm);
    ota.setModem(&modem);
    ota.setBattery(&battery);
    ota.setReed(&reed);
    ota.setAccel(&accel);

    Serial.println(F("[BOOT] setup() complete — entering main loop"));
}

static void logStateLine() {
    Serial.print(F("[STATE] state="));   Serial.print(fsm.stateName());
    if (fsm.state() == AlarmStateMachine::ALARM) {
        Serial.print(F(" reason="));     Serial.print(fsm.triggerReasonName());
    }
    Serial.print(F(" reed="));           Serial.print(reed.isOpen() ? F("OPEN") : F("CLOSED"));
    Serial.print(F(" motion="));         Serial.print(accel.isMotionDetected() ? 1 : 0);
    Serial.print(F(" mag="));            Serial.print(accel.getMagnitude(), 3);
    Serial.print(F(" delta="));          Serial.print(accel.getDeltaG(), 3);

    if (battery.hasValidMeasurement()) {
        Serial.print(F(" battery="));
        Serial.print(battery.getVoltageV(), 2);
        Serial.print(F("V current="));
        Serial.print((long)lroundf(battery.getCurrentA() * 1000.0f));
        Serial.print(F("mA est="));
        if (battery.hasValidEstimate()) {
            Serial.print(battery.getEstimatedRemainingDays(), 1);
            Serial.println(F("d"));
        } else {
            Serial.println(F("UNKNOWN"));
        }
    } else {
        Serial.println(F(" battery=UNKNOWN"));
    }
}

static void logHealthLine() {
    Serial.print  (F("[HEALTH] modem="));
    Serial.print  (modem.isAvailable() ? F("OK") : F("DOWN"));
    Serial.print  (F(" registered="));
    Serial.print  (modem.isNetworkRegistered() ? F("yes") : F("no"));
    Serial.print  (F(" atFail="));
    Serial.print  (modem.getConsecutiveAtFailures());
    Serial.print  (F(" smsQ="));
    Serial.print  (modem.pendingSmsCount());
    Serial.print  (F(" gps="));
    Serial.println(modem.gnssStarted() ? F("active") : F("idle"));
}

void loop() {
    battery.update();
    accel.update();
    reed.update();
    fsm.update();
    led.update();
    siren.update();
    modem.update();
    ota.update();

#if WATCHDOG_ENABLED
    esp_task_wdt_reset();
#endif

    static uint32_t lastStateLogMs  = 0;
    static uint32_t lastHealthLogMs = 0;
    const uint32_t now = millis();
    if ((now - lastStateLogMs) >= runtimeConfig.stateLogIntervalMs()) {
        lastStateLogMs = now;
        logStateLine();
    }
    if ((now - lastHealthLogMs) >= DEBUG_HEALTH_PERIOD_MS) {
        lastHealthLogMs = now;
        logHealthLine();
    }
}
