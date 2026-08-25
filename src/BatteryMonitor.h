/*
 * BatteryMonitor.h
 * ---------------------------------------------------------
 * Reads battery voltage on the ESP-FLY drone board through a
 * resistive voltage divider (two 10KΩ resistors, per schematic)
 * and reports the actual battery voltage back to the caller.
 *
 * Hardware context (from ESP-FLY schematic):
 *   - System/battery rail (Vbat)  : up to ~5V (per board spec)
 *   - ESP32-S3 ADC max safe input : 3.3V
 *   - Divider: Vbat -> R_TOP (10K) -> ADC_PIN -> R_BOTTOM (10K) -> GND
 *
 *   With R_TOP == R_BOTTOM == 10K, the divider ratio is exactly 2:1,
 *   so a 5.0V battery rail becomes 2.5V at the ADC pin - safely
 *   under the 3.3V limit, with headroom for a fully charged
 *   pack or supply ripple.
 *
 * Author: (generated for ESP-FLY project by Max Imagination)
 * ---------------------------------------------------------
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

class BatteryMonitor {
public:
    // adcPin       : ESP32-S3 ADC-capable GPIO tied to the divider midpoint
    // ledPin       : Optional low-battery indicator LED pin (-1 to disable)
    // rTopOhms     : Resistor from Vbat to ADC node   (default 10,000Ω)
    // rBottomOhms  : Resistor from ADC node to GND    (default 10,000Ω)
    // adcMaxCounts : ADC resolution counts (ESP32-S3 default is 12-bit = 4095)
    // adcRefVolts  : ADC reference voltage in volts (ESP32-S3 = 3.3V)
    BatteryMonitor(uint8_t adcPin = 2,
                   int8_t ledPin = -1,
                   float rTopOhms = 10000.0f,
                   float rBottomOhms = 10000.0f,
                   uint16_t adcMaxCounts = 4095,
                   float adcRefVolts = 3.3f);

    // Call once in setup()
    void begin();

    // Call periodically (e.g. every 500ms-1s) in loop()
    // Returns the estimated battery voltage in volts
    float update();

    // Last computed battery voltage (volts) without triggering a new read
    float getVoltage() const { return _lastVoltage; }

    // Raw averaged ADC reading from the last update()
    uint16_t getRawADC() const { return _lastRaw; }

    // Configure the low-battery cutoff voltage (default 3.3V for 1S LiPo storage-safe level)
    void setLowBatteryThreshold(float volts) { _lowBattThreshold = volts; }

    // True if last reading was at/below the low battery threshold
    bool isLowBattery() const { return _lastVoltage <= _lowBattThreshold; }

    // Optional: apply a manual calibration multiplier if measured values
    // drift from real-world readings (accounts for resistor tolerance).
    void setCalibrationFactor(float factor) { _calibration = factor; }

private:
    uint8_t  _adcPin;
    int8_t   _ledPin;
    float    _rTop;
    float    _rBottom;
    uint16_t _adcMaxCounts;
    float    _adcRefVolts;
    float    _calibration;
    float    _lowBattThreshold;

    float    _lastVoltage;
    uint16_t _lastRaw;

    static const uint8_t NUM_SAMPLES = 8; // averaging for ADC noise reduction

    uint16_t readAveragedRaw();
};

#endif // BATTERY_MONITOR_H
