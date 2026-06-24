#pragma once

// =====================================================================
// Smart car alarm firmware — central configuration
// Board: LILYGO T-SIM7600G-H / T-SIM7600E-H (ESP32-WROVER + SIM7600)
// =====================================================================

#include <Arduino.h>
#include <driver/adc.h>   // adc1_channel_t, adc_atten_t, adc_bits_width_t (ESP-IDF v4)

// ---------------------------------------------------------------------
// Firmware version & identity
// ---------------------------------------------------------------------
#define FW_NAME       "MoskvichAlarm"
#define FW_VERSION    "0.1.0-webcfg"
#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__
#define FW_BUILD      __DATE__ " " __TIME__

// ---------------------------------------------------------------------
// Debug flags — set to 1 to enable verbose Serial logs per subsystem
// ---------------------------------------------------------------------
#define DEBUG_BATTERY 1
#define DEBUG_ACCEL   1
#define DEBUG_REED    1
#define DEBUG_MODEM   1
#define DEBUG_ALARM   1
#define DEBUG_SIREN   1
#define DEBUG_BUTTON  1
#define DEBUG_LED     1

// MPU verbose polling. When 1, every ACCEL_POLL_PERIOD_MS update prints
// the raw + g + magnitude + delta values. When 0, those lines are silenced
// but the values are still computed and exposed via the Accelerometer
// getters (used by the periodic [STATE] log). Motion-edge events still
// print regardless of this flag.
#define DEBUG_MPU_VERBOSE  0

// ---- Local alarm core flags ----
// ALARM_HAS_ARM_BUTTON = 0: no GPIO arm/disarm input. The whole device is
// powered through a physical switch on the DC-DC input; cutting that
// switch is the only way to disarm. Button.cpp/h stay in the tree but
// are never initialized or polled.
#define ALARM_HAS_ARM_BUTTON            0

// When 1, BatteryMonitor CRITICAL state must NOT block alarm logic. The
// battery divider is not wired yet; the monitor still runs and prints,
// but the FSM ignores its result.
#define DEBUG_IGNORE_BATTERY_CRITICAL   1

// When 1, the siren is driven with a low-duty-cycle "safe" pulse pattern
// (200 ms ON / 800 ms OFF) and an optional short confirmation beep at the
// start of ARMING_DELAY. Set to 0 for a continuous-on siren later.
#define DEBUG_SIREN_SAFE_MODE           1

// Phase 3 SPI / single-GPIO diagnostics — retired. These flags must remain
// at 0; the firmware no longer contains SPI or standalone GPIO test code.
#define ACCEL_USE_SPI_DIAG       0
#define DEBUG_SPI_BITBANG_TEST   0
#define DEBUG_GPIO23_TEST        0
#define DEBUG_GPIO19_INPUT_TEST  0

// =====================================================================
// PIN MAP
// =====================================================================
// ESP32 pins to AVOID for external peripherals on this board:
//   GPIO0,2,15  — boot strapping
//   GPIO5,12    — boot strapping / can hang boot if pulled wrong
//   GPIO16,17   — used by PSRAM on ESP32-WROVER (do not touch)
//
// =====================================================================
// !!! FORBIDDEN MODEM PINS — hard-wired to SIM7600 on the LILYGO PCB.
// !!! DO NOT use these for any external peripheral.
// =====================================================================
//   GPIO27 = MODEM_TX      (ESP32 -> SIM7600 RX)
//   GPIO26 = MODEM_RX      (SIM7600 TX -> ESP32)
//   GPIO4  = MODEM_PWRKEY  (~1s LOW pulse powers the modem on)
//   GPIO25 = MODEM_DTR     (sleep / wake control)
// =====================================================================

// ---- I2C bus (MPU-6050 accelerometer) ----
// ESP32 default I2C pins. Free on LILYGO T-SIM7600G-H.
#define PIN_I2C_SDA      21
#define PIN_I2C_SCL      22

// ---- Accelerometer interrupt ----
#define PIN_ACCEL_INT    14   // INT from MPU-6050 (disconnected for Phase 3)

// ---- Reed switch ----
#define PIN_REED         13   // INPUT_PULLUP, GND on event (NC preferred)

// ---- Siren / buzzer via MOSFET ----
// External 100kΩ gate pulldown holds Q1 OFF during boot/reset.
#define PIN_SIREN        32

// ---- Status LED ----
// GPIO19 is a plain digital output, fine for an LED.
#define PIN_STATUS_LED   19

// ---- ARM/DISARM button ----
// GPIO33 is RTC-capable (RTC_GPIO8) — usable as an EXT0 deep-sleep wake
// source in Phase 11.
#define PIN_BUTTON       33

// ---- Battery voltage ADC ----
// Input-only ADC1_CH7. Works fine with the 100k/47k divider.
#define PIN_BAT_ADC      35

// =====================================================================
// SIM7600 modem pins — OFFICIAL LILYGO T-SIM7600 DEFINITIONS
// Sourced from: https://github.com/Xinyuan-LilyGO/T-SIM7600X
// DO NOT change these — they are hard-wired on the PCB.
// =====================================================================
#define MODEM_UART_BAUD  115200
#define MODEM_PIN_TX     27   // ESP32 -> SIM7600 RX
#define MODEM_PIN_RX     26   // SIM7600 TX -> ESP32
#define MODEM_PIN_PWRKEY 4    // ~1s LOW pulse to power on
#define MODEM_PIN_DTR    25   // Sleep / wake control
// No software-controlled power-enable / reset pin on this revision.

// Compile-time guard: any external pin must never collide with the modem.
static_assert(PIN_SIREN       != MODEM_PIN_TX     && PIN_SIREN       != MODEM_PIN_RX &&
              PIN_SIREN       != MODEM_PIN_PWRKEY && PIN_SIREN       != MODEM_PIN_DTR,
              "PIN_SIREN collides with a SIM7600 modem pin");
static_assert(PIN_STATUS_LED  != MODEM_PIN_TX     && PIN_STATUS_LED  != MODEM_PIN_RX &&
              PIN_STATUS_LED  != MODEM_PIN_PWRKEY && PIN_STATUS_LED  != MODEM_PIN_DTR,
              "PIN_STATUS_LED collides with a SIM7600 modem pin");
static_assert(PIN_BUTTON      != MODEM_PIN_TX     && PIN_BUTTON      != MODEM_PIN_RX &&
              PIN_BUTTON      != MODEM_PIN_PWRKEY && PIN_BUTTON      != MODEM_PIN_DTR,
              "PIN_BUTTON collides with a SIM7600 modem pin");

// =====================================================================
// BATTERY MONITOR — Phase 2
// =====================================================================
// Voltage divider on PIN_BAT_ADC:
//   VBAT --[R4 = 100kΩ]--+--[R5 = 47kΩ]-- GND
//                        |
//                       ADC node ----- C4 = 100nF -- GND
//
// Divider ratio K = (R4 + R5) / R5 = (100 + 47) / 47 = 3.12766
// VBAT = V_adc * K * BATTERY_CALIBRATION
//
// ESP32 ADC reference at attenuation ADC_ATTEN_DB_11 is ~3.3V (nonlinear).
// We use the IDF esp_adc_cal API for calibration; the constants below
// stay as fallback in case calibration is unavailable.
// ---------------------------------------------------------------------

#define BAT_R4_OHMS              100000.0f
#define BAT_R5_OHMS              47000.0f
#define BAT_DIVIDER_RATIO        ((BAT_R4_OHMS + BAT_R5_OHMS) / BAT_R5_OHMS)

// Fine-tune after measuring real battery with a multimeter.
// Increase if reported voltage is lower than real; decrease otherwise.
#define BATTERY_CALIBRATION      1.0f

// ADC sampling — typed with ESP-IDF v4 enums so they match the
// adc1_*/esp_adc_cal API signatures (adc_atten_t, not adc_attenuation_t).
#define BAT_ADC_SAMPLES          16        // averaged samples per read
static constexpr adc1_channel_t   BAT_ADC_CHANNEL         = ADC1_CHANNEL_7;   // GPIO35
static constexpr adc_bits_width_t BAT_ADC_WIDTH           = ADC_WIDTH_BIT_12;
static constexpr adc_atten_t      BAT_ADC_ATTEN           = ADC_ATTEN_DB_11;  // full 0..~3.3 V range
static constexpr uint32_t         BAT_ADC_VREF_DEFAULT_MV = 1100;             // used if eFuse Vref absent

// =====================================================================
// INA226 battery monitor (V / I / P over I2C, shared bus with MPU-6050)
// =====================================================================
// Check the shunt marking on the INA226 module:
//   R100 usually means 0.1 ohm
//   R010 usually means 0.01 ohm
// Update INA226_SHUNT_OHMS accordingly.
#define INA226_ENABLED                    1
#define INA226_I2C_ADDR                   0x40
#define INA226_SHUNT_OHMS                 0.1f
#define INA226_MAX_EXPECTED_CURRENT_A     5.0f
#define INA226_INVERT_CURRENT             0       // set to 1 if shunt direction reversed

// When 1, BatteryMonitor prints the raw register dump at boot and once
// every 10 s during update(). Helps diagnose the "INA226 detected over
// I2C but VIN+/VIN- not wired" case (bus voltage stays 0).
#define DEBUG_INA226_RAW                  1

// Bus-voltage threshold below which the INA226 reading is considered
// invalid (VIN+/VIN- not connected). Status reports show UNKNOWN, the
// estimate is suppressed, and the low-battery SMS does NOT fire.
#define INA226_MIN_VALID_BUS_VOLTAGE      1.0f

// Minimum |avg current| required to publish a remaining-days estimate.
// Below this we cannot distinguish 200-day vs 2000-day runtime — show
// est=UNKNOWN instead of an unrealistic number.
#define BATTERY_MIN_CURRENT_FOR_ESTIMATE_A 0.02f

// BATTERY_CAPACITY_AH must be set to the real battery capacity.
// LOW_BATTERY_RESERVE_DAYS controls when to warn before expected discharge.
// For example, 4 means warn when estimated remaining autonomy is <= 4 days.
#define BATTERY_TYPE                      "4S LiFePO4 / 12V"
#define BATTERY_CAPACITY_AH               40.0f
#define LOW_BATTERY_RESERVE_DAYS          4.0f
#define BATTERY_AVG_CURRENT_WINDOW_MS     3600000
#define BATTERY_AVG_CURRENT_ALPHA         0.02f          // legacy single-EMA alpha — unused once stats are on

// ---- Energy accounting and statistics ----
// Fast EMA reacts quickly to modem/siren load spikes (UI/debug only).
// Slow EMA smooths transient spikes and is what we use for the autonomy
// estimate. Consumed Ah/Wh are integrated since boot (not persisted —
// future stage can move counters to NVS).
#define BATTERY_STATS_ENABLED             1
#define BATTERY_STATS_SAMPLE_PERIOD_MS    2000UL
#define BATTERY_AVG_FAST_ALPHA            0.10f
#define BATTERY_AVG_SLOW_ALPHA            0.01f
#define BATTERY_STATS_LOG_PERIOD_MS       30000UL

// Voltage thresholds for 4S LiFePO4 / 12V battery.
// LiFePO4 voltage curve is very flat in the middle of the SOC range, so
// voltage-only SOC is approximate. The low-battery alert combines this
// voltage check with an estimated-reserve check (remaining Ah / avgI).
#define BAT_VOLTAGE_WARN                  12.6f
#define BAT_VOLTAGE_CRITICAL              12.2f
#define BAT_VOLTAGE_SHUTDOWN              11.8f

// Low-battery SMS alert (sent to SMS_ALERT_RECIPIENTS).
// For bench testing on a 7.4V battery, set LOW_BATTERY_SMS_ENABLED=0 to
// avoid spurious WARN/CRITICAL alerts caused by 4S LiFePO4 thresholds
// tripping on a lower-voltage supply.
// For real 4S LiFePO4 / 12V battery, set LOW_BATTERY_SMS_ENABLED=1.
#define LOW_BATTERY_SMS_ENABLED           1
#define LOW_BATTERY_SMS_REPEAT_MS         86400000UL   // at most once per day per recipient

// Voltage thresholds for the legacy ADC monitor (only used when
// INA226_ENABLED=0). Kept for compile compatibility.
#define LOW_BAT_WARN_V           6.8f
#define LOW_BAT_SLEEP_V          6.6f
#define LOW_BAT_CRITICAL_V       6.2f

// Hysteresis to avoid threshold flapping
#define LOW_BAT_HYSTERESIS_V     0.05f

// Log period for periodic battery prints (ms)
#define BAT_LOG_PERIOD_MS        2000

// =====================================================================
// Future phases — placeholders, tuned later
// =====================================================================

// =====================================================================
// Accelerometer (Phase 3) — MPU-6050 / GY-521 in accelerometer-only mode
// =====================================================================
// The MPU-6050 is brought up over I2C and operated as an accelerometer
// only. The three gyroscope axes are placed into standby via PWR_MGMT_2
// (STBY_XG | STBY_YG | STBY_ZG = 0x07) to reduce power consumption. The
// internal temperature sensor is disabled via the TEMP_DIS bit in
// PWR_MGMT_1. INT and AD0 are left disconnected; the chip lives at I2C
// address 0x68 by default (AD0 floats LOW on the GY-521 module).

#define ACCEL_USE_MPU6050           1
#define ACCEL_USE_LIS3DHTR          0
#define ACCEL_USE_LIS3DHTR_DIAG     0

// ---- MPU-6050 I2C addresses ----
#define ACCEL_MPU6050_ADDR_PRIMARY      0x68    // AD0 = GND / floating
#define ACCEL_MPU6050_ADDR_SECONDARY    0x69    // AD0 = VCC
#define ACCEL_MPU6050_I2C_FREQ          100000  // 100 kHz

// ---- MPU-6050 register map (subset used by this firmware) ----
#define MPU6050_REG_CONFIG          0x1A    // DLPF select (bits 2:0)
#define MPU6050_REG_ACCEL_CONFIG    0x1C    // AFS_SEL (full-scale range)
#define MPU6050_REG_ACCEL_XOUT_H    0x3B    // X_H, X_L, Y_H, Y_L, Z_H, Z_L (auto-increment)
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

// ---- MPU-6050 constants ----
#define MPU6050_WHO_AM_I_EXPECTED   0x68
#define MPU6050_ACCEL_SENS_2G       16384.0f   // LSB/g at ±2g full-scale
#define MPU6050_PWR1_TEMP_DIS_BIT   0x08       // PWR_MGMT_1: disable temperature sensor
#define MPU6050_PWR2_GYRO_STANDBY   0x07       // PWR_MGMT_2: STBY_{X,Y,Z}G = 1, accel stays on
#define MPU6050_ACCEL_CONFIG_2G     0x00       // AFS_SEL = 0 -> ±2g
#define MPU6050_DLPF_CFG_44HZ       0x03       // CONFIG: ~44 Hz accel low-pass filter

// ---- Polling and motion detection ----
#define ACCEL_POLL_PERIOD_MS        500

// Motion sensitivity tuning:
// lower value = more sensitive
// suggested range:
//   0.12f = very sensitive
//   0.15f = recommended initial value
//   0.18f = medium
//   0.25f = less sensitive
#define ACCEL_MOTION_DELTA_G        0.025f      // |Δ magnitude| trigger (g)

// Reed sensor
#define REED_DEBOUNCE_MS         50

// Button — class kept stubbed; ALARM_HAS_ARM_BUTTON=0 disables it.
#define BUTTON_DEBOUNCE_MS       50
#define BUTTON_LONGPRESS_MS      3000

// Siren — safe-mode pulse pattern and confirmation beep sequence.
#define SIREN_MAX_ON_MS          10000    // hard cap, future use
#define SIREN_SAFE_ON_MS         200      // ON duration in DEBUG_SIREN_SAFE_MODE alarm pattern
#define SIREN_SAFE_OFF_MS        800      // OFF duration in DEBUG_SIREN_SAFE_MODE alarm pattern

// ALARM siren behavior on FSM transition to ALARM.
// SIREN_ALARM_MODE_CONTINUOUS = 1: siren sounds continuously for
//   SIREN_ALARM_CONTINUOUS_MS, then turns off. FSM ALARM state stays
//   latched until SMS disarm.
// SIREN_AFTER_TIMEOUT_REMINDER_ENABLED = 1 enables a low-duty reminder
//   pulse train after the continuous window expires. Default 0 — once
//   the continuous window ends, the siren stays off.
#define SIREN_ALARM_MODE_CONTINUOUS            1
#define SIREN_ALARM_CONTINUOUS_MS              120000UL
#define SIREN_AFTER_TIMEOUT_REMINDER_ENABLED   0
#define SIREN_REMINDER_ON_MS                   200UL
#define SIREN_REMINDER_OFF_MS                  10000UL

// Confirmation beeps (non-blocking, driven from Siren::update()):
//   confirmArm()    — SIREN_ARM_CONFIRM_BEEPS short beeps
//   confirmDisarm() — SIREN_DISARM_CONFIRM_BEEPS short beeps
#define SIREN_CONFIRM_BEEP_MS         80
#define SIREN_CONFIRM_GAP_MS          120
#define SIREN_ARM_CONFIRM_BEEPS       1
#define SIREN_DISARM_CONFIRM_BEEPS    2

// Per-transition gates for whether to actually play the beep.
#define SIREN_CONFIRM_ON_BOOT_ARM     1
#define SIREN_CONFIRM_ON_SMS_ARM      1
#define SIREN_CONFIRM_ON_SMS_DISARM   1

// Status LED — non-blocking patterns driven by millis().
#define LED_ARMING_HALF_PERIOD_MS   100   // 5 Hz fast blink during ARMING_DELAY

// "Car alarm style" ARMED pattern: a burst of short flashes followed by a
// long quiet pause, repeating forever while ARMED.
#define LED_ARMED_FLASH_COUNT       3     // flashes per burst
#define LED_ARMED_FLASH_ON_MS       70    // each flash ON duration
#define LED_ARMED_FLASH_OFF_MS      120   // gap between flashes inside the burst
#define LED_ARMED_PAUSE_MS          4000  // quiet pause after the burst

#define LED_ALARM_ON_MS             80    // ON duration inside a double-blink
#define LED_ALARM_OFF_MS            80    // gap between the two flashes of a double-blink
#define LED_ALARM_PAUSE_MS          280   // pause between successive double-blinks

// DISARMED — single short blink every 6 s (very low duty cycle).
#define LED_DISARMED_FLASH_MS       50
#define LED_DISARMED_PERIOD_MS      6000

// Alarm FSM timing
#define ALARM_ARMING_DELAY_MS         15000    // ARMING_DELAY duration before ARMED
#define ALARM_ARMED_SENSOR_GRACE_MS    2000    // ignore reed/motion for this long after entering ARMED
#define STATE_LOG_PERIOD_MS            2000    // [STATE] line cadence in main loop

// =====================================================================
// SIM7600 modem — stable bring-up
// =====================================================================
// Modem::begin() pulses PWRKEY once, drives DTR (GPIO25) HIGH (required
// on this LILYGO board to leave low-power mode), waits for the modem to
// boot, runs the AT probe loop, enables verbose errors, reads SIM/CFUN,
// forces CFUN=1 if needed, and refreshes basic network state. Modem::
// update() refreshes the same state every MODEM_STATUS_LOG_PERIOD_MS and
// prints a one-line network summary. SMS / GPS / HTTP / deep sleep are
// NOT wired up here and the alarm FSM does not consume modem state.

#define MODEM_BASIC_DIAG              1
#define MODEM_AT_DEBUG                1     // [AT>] / [AT<] per-command trace

// ---- Power and timing ----
#define MODEM_PWRKEY_PULSE_MS         1000  // PWRKEY held LOW to power modem on
#define MODEM_BOOT_WAIT_MS            10000 // wait after PWRKEY release before talking
#define MODEM_AT_TIMEOUT_MS           1000  // per-probe AT timeout
#define MODEM_STATUS_LOG_PERIOD_MS    10000 // periodic refresh cadence

// ---- Boot-tolerant AT bring-up ----
// Replaces the old fixed-attempts probe with a time-bounded loop that
// continuously reads boot URCs (RDY / +CPIN: READY / SMS DONE / ...) and
// reissues AT every MODEM_AT_BOOT_PROBE_INTERVAL_MS until either AT
// returns OK or MODEM_AT_BOOT_TOTAL_WAIT_MS expires.
#define MODEM_AT_BOOT_TOTAL_WAIT_MS      30000
#define MODEM_AT_BOOT_PROBE_INTERVAL_MS  1000

// If begin() never sees AT OK, Modem::update() keeps retrying recovery
// at this interval. On success it runs the full bring-up sequence so a
// queued alarm SMS can still go out.
#define MODEM_AT_RECOVERY_INTERVAL_MS    5000

#define MODEM_DO_PWRKEY_ON_BOOT       1     // pulse PWRKEY once during begin()

// ---- DTR (sleep/wake control) ----
// On this LILYGO T-SIM7600G-H board GPIO25 / MODEM_DTR must be held HIGH
// for the modem to leave low-power mode. Determined empirically: with
// DTR=LOW the modem reports +CFUN: 0 / "NO SERVICE, Low Power Mode" and
// AT+CFUN=1 returns +CME ERROR. With DTR=HIGH the radio comes up, CFUN=1
// is accepted, and the modem registers normally.
#define MODEM_DTR_ACTIVE_LEVEL        HIGH

// ---- Operator scan ----
// AT+COPS=? takes up to 60 s. Disabled by default; enable only to debug
// network coverage.
#define MODEM_DO_COPS_SCAN            0
#define MODEM_COPS_SCAN_TIMEOUT_MS    60000

// ---- Future-phase placeholders (HTTP) ----
#define MODEM_AT_DEFAULT_TIMEOUT_MS   3000
// (GPS_FIX_TIMEOUT_MS is defined further below alongside the GPS block.)

// =====================================================================
// SMS alert + remote command control
// =====================================================================
// On ARMED -> ALARM the modem queues a "DOOR OPEN" / "MOTION" SMS to
// every number in SMS_ALERT_RECIPIENTS. Numbers in SMS_COMMAND_WHITELIST
// are allowed to send numeric command SMS. The two lists are independent;
// the same phone may appear in both. Numbers must be in international
// format (e.g. +79991234567). Phone matching is tolerant: exact string
// match, or matching last 10 digits, so the modem reporting "8…" /
// "+7…" / "79…" all map to the same person.
//
// Command map:
//   0 = STATUS (compact one-screen status),
//   1 = ARM, 2 = DISARM,
//   3 = SILENT_DISARM (disarm + siren off), 4 = SIREN_TEST,
//   5 = MODEM_STATUS, 6 = BATTERY_STATUS, 7 = GPS_LOCATION,
//   8 = OTA_UPDATE (Wi-Fi AP for firmware upload),
//   9 = HELP (full command list; text alias: HELP)

#define SMS_ALERT_ENABLED        1
#define SMS_TEXT_MODE            1            // AT+CMGF=1 (text, not PDU)
#define SMS_SEND_TIMEOUT_MS      30000        // legacy alias, unused once fallback ladder is on
#define SMS_MAX_RETRIES          3            // per recipient
#define SMS_RETRY_INTERVAL_MS    30000        // between retries for the same recipient

// ---- Watchdog-safe SMS send budgets ----
// Per-step bounds (used inside trySendOneProfile). Setup AT commands
// (CMGF / CSCS / CSMP / CSMS / CGSMS) all use SMS_PROFILE_SETUP_TIMEOUT_MS.
// The "> prompt" wait and the final +CMGS/OK/+CMS ERROR wait have their own.
#define SMS_PROFILE_SETUP_TIMEOUT_MS    3000UL
#define SMS_PROMPT_TIMEOUT_MS           5000UL
#define SMS_FINAL_RESPONSE_TIMEOUT_MS   12000UL

// Upper bound on a whole sendSms() call across all profile attempts.
// Anything past this returns false so the loop can keep running.
#define SMS_SEND_TOTAL_TIMEOUT_MS       45000UL

// Profile fallback limits.
#define SMS_PROFILE_MAX_PER_SEND        5
// 1 = first attempt for a given recipient tries every profile; subsequent
// retries use only the last known working profile (or DEFAULT) so we don't
// burn the full ~30 s ladder repeatedly.
#define SMS_PROFILE_FALLBACK_FULL_ONLY_ON_FIRST_ATTEMPT 1

// ---- Operator-specific SMS workaround ----
// On MegaFon + SIM7600G-H, every SMS profile fails with +CMS ERROR over
// LTE but succeeds in WCDMA. When this flag is on the firmware forces
// AT+CNMP=14 (WCDMA only) right after operator detection so SMS sends
// reliably. Manual NETAUTO command can restore AUTO mode if needed.
#define SMS_FORCE_WCDMA_FOR_MEGAFON     1
#define SMS_WCDMA_CNMP_CMD              "AT+CNMP=14"
#define SMS_AUTO_CNMP_CMD               "AT+CNMP=2"

// Alert recipients. The runtime list lives in RecipientManager (loaded
// from NVS; editable by SMS commands ADD/DEL/LIST from a whitelisted
// number). SMS_ALERT_RECIPIENTS_DEFAULT seeds the list on first boot or
// when NVS is empty/corrupt. SMS_ALERT_RECIPIENT_MAX is the hard cap;
// SMS_ALERT_ALLOW_ZERO_RECIPIENTS controls whether DEL of the last
// recipient is permitted.
#define SMS_ALERT_RECIPIENT_MAX               5
#define SMS_ALERT_ALLOW_ZERO_RECIPIENTS       0
#define SMS_ALERT_RECIPIENT_DEFAULT_COUNT     1
static const char* const SMS_ALERT_RECIPIENTS_DEFAULT[SMS_ALERT_RECIPIENT_DEFAULT_COUNT] = {
    ""
};
static_assert(SMS_ALERT_RECIPIENT_DEFAULT_COUNT <= SMS_ALERT_RECIPIENT_MAX,
              "Too many default SMS alert recipients");

// People who are allowed to send commands. Increase _COUNT and add
// more numbers. The same phone may also be in SMS_ALERT_RECIPIENTS.
#define SMS_COMMAND_WHITELIST_COUNT 3
static const char* const SMS_COMMAND_WHITELIST[SMS_COMMAND_WHITELIST_COUNT] = {
    ""
};

#define SMS_COMMANDS_ENABLED            1
#define SMS_COMMAND_POLL_INTERVAL_MS    15000
#define SMS_COMMAND_DELETE_PROCESSED    1

// ---- Diagnostic SMS commands (TESTSMS / TESTSMS2G / NETAUTO / NETWCDMA) ----
// These were used during MegaFon WCDMA bring-up. Off by default for
// production: HELP omits them and authorized senders get back "DIAG
// disabled". Flip to 1 to expose them again during development.
#define SMS_DIAGNOSTIC_COMMANDS_ENABLED 0

// ---- SMS profile fallback gating ----
// WCDMA + DEFAULT profile works on MegaFon, so production sends should not
// burn the full ~25 s fallback ladder on every retry. When
// SMS_PROFILE_FALLBACK_ENABLED=0, only DEFAULT (or the last known working
// profile) is tried. When SMS_PROFILE_FULL_FALLBACK_FOR_DIAG_ONLY=1, the
// full ladder is allowed only for TESTSMS-class diagnostic commands.
#define SMS_PROFILE_FALLBACK_ENABLED              1
#define SMS_PROFILE_FULL_FALLBACK_FOR_DIAG_ONLY   1

// ---- Verbose AT logging ----
// 1 = print every [AT>] / [AT<] line for normal SMS sends. 0 = only the
// short send-result lines stay. Keep 1 during development; switch to 0
// for quieter production logs.
#define DEBUG_AT_LOG                    1

// =====================================================================
// GPS / GNSS — SMS command 7
// =====================================================================
// GPS_FIX_TIMEOUT_MS:    how long command 7 may wait for the first fix.
// GPS_POLL_INTERVAL_MS:  how often AT+CGPSINFO is polled during that wait.
// GPS_MAX_AGE_MS:        allows replying immediately with a recent cached
//                        fix without re-polling.
// GPS may require outdoor testing or window placement; the GNSS antenna
// must be connected to the SIM7600 GPS / GNSS connector.
#define GPS_COMMAND_ENABLED             1
#define GPS_FIX_TIMEOUT_MS              180000UL
#define GPS_POLL_INTERVAL_MS            5000UL
#define GPS_MAX_AGE_MS                  300000UL

// Hard upper bound on how long the GPS command may block the main loop.
// Kept equal to GPS_FIX_TIMEOUT_MS so any future tightening only needs to
// change this single constant.
#define GPS_COMMAND_BLOCKING_MAX_MS     GPS_FIX_TIMEOUT_MS

// =====================================================================
// Reliability: watchdog, modem health, health log
// =====================================================================
#define WATCHDOG_ENABLED                1
#define WATCHDOG_TIMEOUT_SEC            15

// Modem health-check and recovery. consecutiveAtFailures is bumped each
// time AT times out; at MODEM_AT_FAILURE_LIMIT the firmware runs a soft
// recovery (flush UART + retry AT + CFUN + refresh). After
// MODEM_HARD_RECOVERY_AFTER_FAILURES it pulses PWRKEY (no more often
// than MODEM_RECOVERY_COOLDOWN_MS).
#define MODEM_HEALTH_CHECK_INTERVAL_MS  10000
#define MODEM_AT_FAILURE_LIMIT          3
#define MODEM_RECOVERY_COOLDOWN_MS      60000
#define MODEM_HARD_RECOVERY_AFTER_FAILURES 6

// Periodic compact [HEALTH] line for monitoring without per-subsystem spam.
#define DEBUG_HEALTH_PERIOD_MS          30000

// =====================================================================
// Wi-Fi OTA firmware update (SMS command 8)
// =====================================================================
// Wi-Fi is normally OFF. It is enabled only after authorized SMS command
// 8 and is automatically disabled after OTA_ACTIVE_WINDOW_MS (or after
// a successful upload triggers a reboot). Internet OTA is NOT used —
// the firmware is uploaded over LAN through the device's own SoftAP.
#define OTA_WIFI_ENABLED         1
#define OTA_SMS_COMMAND_ENABLED  1
#define OTA_COMMAND_CODE         8

#define OTA_AP_SSID              "MoskvichAlarm-OTA"
#define OTA_AP_PASSWORD          "12345678"            // WPA2 needs >= 8 chars
#define OTA_AP_CHANNEL           6
#define OTA_AP_MAX_CLIENTS       1

#define OTA_ACTIVE_WINDOW_MS     600000UL              // auto-disable after 10 min
#define OTA_HTTP_PORT            80

