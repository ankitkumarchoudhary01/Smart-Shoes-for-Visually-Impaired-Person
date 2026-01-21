/*
 * Configuration file for Smart Navigation Shoes
 * Adjust these values based on your hardware setup
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============== HARDWARE CONFIGURATION ==============
// Uncomment your board type
#define ESP32_BOARD
// #define ARDUINO_NANO
// #define ARDUINO_UNO

// ============== SENSOR CALIBRATION ==============
#define ULTRASONIC_CALIBRATION_OFFSET  0.0  // cm
#define IR_THRESHOLD_VOLTAGE           2.5  // volts

// ============== DISTANCE THRESHOLDS ==============
// Adjust based on user testing feedback
#define SAFE_DISTANCE_CM              200
#define WARNING_DISTANCE_CM           100
#define DANGER_DISTANCE_CM            50
#define CRITICAL_DISTANCE_CM          20

// ============== VIBRATION PATTERNS ==============
#define VIBRATION_INTENSITY_MIN       50
#define VIBRATION_INTENSITY_MAX       255
#define VIBRATION_PULSE_CRITICAL      100   // ms
#define VIBRATION_PULSE_DANGER        200   // ms
#define VIBRATION_PULSE_WARNING       500   // ms

// ============== AUDIO SETTINGS ==============
#define BUZZER_ENABLED                true
#define BUZZER_VOLUME                 128   // 0-255
#define BUZZER_FREQUENCY_CRITICAL     2000  // Hz
#define BUZZER_FREQUENCY_WARNING      1000  // Hz

// ============== POWER MANAGEMENT ==============
#define BATTERY_VOLTAGE_MAX           4.2   // Li-ion full charge
#define BATTERY_VOLTAGE_MIN           3.0   // Li-ion cutoff
#define BATTERY_LOW_THRESHOLD         3.3   // Warning threshold
#define AUTO_SLEEP_MINUTES            5
#define BATTERY_CHECK_SECONDS         60

// ============== CONNECTIVITY ==============
#define BLUETOOTH_ENABLED             true
#define BLUETOOTH_DEVICE_NAME         "SmartNavShoes"

// ============== DEBUG SETTINGS ==============
// Uncomment for verbose logging
// #define DEBUG
// #define DEBUG_SENSORS
// #define DEBUG_VIBRATION
// #define DEBUG_BATTERY

#endif