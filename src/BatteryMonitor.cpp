/*
 * BatteryMonitor.cpp
 * ---------------------------------------------------------
 * See BatteryMonitor.h for hardware/wiring notes.
 * ---------------------------------------------------------
 */

#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(uint8_t adcPin,
                                int8_t ledPin,
                                float rTopOhms,
                                float rBottomOhms,
                                uint16_t adcMaxCounts,
                                float adcRefVolts)
    : _adcPin(adcPin),
      _ledPin(ledPin),
      _rTop(rTopOhms),
      _rBottom(rBottomOhms),
      _adcMaxCounts(adcMaxCounts),
      _adcRefVolts(adcRefVolts),
      _calibration(1.0f),
      _lowBattThreshold(3.3f),
      _lastVoltage(0.0f),
      _lastRaw(0)
{
}

void BatteryMonitor::begin() {
    // ESP32-S3 ADC pins default to 12-bit resolution (0-4095)
    analogReadResolution(12);

    // Full 0-3.3V input range on the ADC pin (no extra internal attenuation change needed
    // since the divider already brings the signal into range)
    pinMode(_adcPin, INPUT);

    if (_ledPin >= 0) {
        pinMode(_ledPin, OUTPUT);
        digitalWrite(_ledPin, LOW);
    }
}

uint16_t BatteryMonitor::readAveragedRaw() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(_adcPin);
        delayMicroseconds(200);
    }
    return static_cast<uint16_t>(sum / NUM_SAMPLES);
}

float BatteryMonitor::update() {
    _lastRaw = readAveragedRaw();

    // Voltage present at the ADC pin (after the divider)
    float adcVoltage = (static_cast<float>(_lastRaw) / static_cast<float>(_adcMaxCounts)) * _adcRefVolts;

    // Reconstruct the original battery voltage using the divider ratio:
    //   Vbat = Vadc * (R_TOP + R_BOTTOM) / R_BOTTOM
    float dividerRatio = (_rTop + _rBottom) / _rBottom;
    _lastVoltage = adcVoltage * dividerRatio * _calibration;

    if (_ledPin >= 0) {
        digitalWrite(_ledPin, isLowBattery() ? HIGH : LOW);
    }

    return _lastVoltage;
}
