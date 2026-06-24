#pragma once

#include <Arduino.h>
#include "config.h"

class Modem;

// INA226-backed battery monitor with energy accounting.
//
// Reads V/I/P every BATTERY_STATS_SAMPLE_PERIOD_MS from the INA226. Tracks
// instant values, fast + slow EMA of current/power, min/max envelope, and
// integrated consumed Ah/Wh since boot. The autonomy estimate uses the
// slow EMA so a brief modem TX spike doesn't crash the estimate.
class BatteryMonitor {
public:
    void begin();
    void update();

    bool        isAvailable()              const { return _ina226Present; }
    bool        hasValidMeasurement()      const;
    bool        hasValidEstimate()         const;

    // Instant readings (latest INA226 sample).
    float       getVoltageV()              const { return _voltageV; }
    float       getCurrentA()              const { return _currentA; }
    float       getPowerW()                const { return _powerW; }
    float       getInstantVoltageV()       const { return _voltageV; }
    float       getInstantCurrentA()       const { return _currentA; }
    float       getInstantPowerW()         const { return _powerW; }

    // Smoothed values.
    float       getAverageCurrentA()       const { return _avgCurrentSlowA; }
    float       getAverageCurrentFastA()   const { return _avgCurrentFastA; }
    float       getAverageCurrentSlowA()   const { return _avgCurrentSlowA; }
    float       getAveragePowerFastW()     const { return _avgPowerFastW; }
    float       getAveragePowerSlowW()     const { return _avgPowerSlowW; }

    // Envelope and energy accounting.
    float       getMinVoltageV()           const { return _minVoltageV; }
    float       getMaxVoltageV()           const { return _maxVoltageV; }
    float       getMaxCurrentA()           const { return _maxCurrentA; }
    float       getMaxPowerW()             const { return _maxPowerW; }
    float       getConsumedAhSinceBoot()   const { return _consumedAhSinceBoot; }
    float       getConsumedWhSinceBoot()   const { return _consumedWhSinceBoot; }

    float       getEstimatedRemainingDays() const;   // NaN when no valid estimate

    // LiFePO4 SOC estimation (voltage table + linear interpolation).
    float       estimateSocPercentFromVoltage(float voltageV) const;
    float       getSocPercent()            const;
    float       getEstimatedRemainingAh()  const;
    float       getReserveNeededAh()       const;

    bool        isLowBatteryWarning()      const;
    bool        isCriticalBattery()        const;

    String      getStatusText()            const;       // command 6 reply
    String      getExtendedStatusText()    const;       // future use

    void        setModem(Modem* modem) { _modem = modem; }

    // Backward-compat shim.
    float       lastVoltage()              const { return _voltageV; }

private:
    bool        ina226Probe();
    void        ina226Configure();
    bool        ina226ReadRegister(uint8_t reg, uint16_t& out);
    bool        ina226WriteRegister(uint8_t reg, uint16_t value);
    void        ina226ReadAll();
    void        ina226PrintBootDiagnostics();
    void        ina226PrintRawDiagnostics();
    void        updateStatistics();
    void        logStatsLine();
    void        checkLowCurrentWarning();
    void        checkLowBatteryAlert();

    Modem*      _modem               = nullptr;

    bool        _ina226Present       = false;

    // Instant
    float       _voltageV            = 0.0f;
    float       _currentA            = 0.0f;
    float       _powerW              = 0.0f;

    // EMAs (positive consumption units)
    float       _avgCurrentFastA     = 0.0f;
    float       _avgCurrentSlowA     = 0.0f;
    float       _avgPowerFastW       = 0.0f;
    float       _avgPowerSlowW       = 0.0f;

    // Envelope
    float       _minVoltageV         = 0.0f;
    float       _maxVoltageV         = 0.0f;
    float       _maxCurrentA         = 0.0f;
    float       _maxPowerW           = 0.0f;

    // Integrated since boot
    float       _consumedAhSinceBoot = 0.0f;
    float       _consumedWhSinceBoot = 0.0f;

    bool        _statsInitialized    = false;

    uint32_t    _lastReadMs          = 0;
    uint32_t    _lastNotFoundLogMs   = 0;
    uint32_t    _lastRawDiagMs       = 0;
    uint32_t    _lastStatsUpdateMs   = 0;
    uint32_t    _lastStatsLogMs      = 0;
    uint32_t    _lowCurrentSinceMs   = 0;
    uint32_t    _lastUsbWarningMs    = 0;

    bool        _lowBatSmsSent       = false;
    uint32_t    _lastLowBatSmsMs     = 0;
};
