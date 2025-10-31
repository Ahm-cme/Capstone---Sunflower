#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * Battery Monitor Component for LiFePO4 12V 15Ah Battery
 * 
 * Monitors battery voltage, calculates state of charge (SOC), and provides
 * low-voltage warnings for the LiFePO4 battery system.
 * 
 * LiFePO4 Voltage Characteristics (per cell, 4 cells in series):
 *  - Fully Charged: 3.65V/cell → 14.6V total (absorption voltage)
 *  - Nominal: 3.2V/cell → 12.8V total (float voltage)
 *  - Storage: 3.3V/cell → 13.2V typical
 *  - Low Voltage Cutoff: 2.5V/cell → 10.0V total (BMS cutoff)
 *  - Critical: 2.0V/cell → 8.0V total (damage threshold)
 * 
 * ADC Configuration:
 *  - ESP32 12-bit ADC (0-4095 range)
 *  - Input voltage range: 0-3.3V with attenuation
 *  - Voltage divider required for 12V battery measurement
 */

/**
 * Battery state of charge levels
 */
typedef enum {
    BATTERY_SOC_CRITICAL = 0,   // <10% - Immediate shutdown needed
    BATTERY_SOC_LOW      = 1,   // 10-20% - Low voltage warning
    BATTERY_SOC_MEDIUM   = 2,   // 20-50% - Normal operation, consider charging
    BATTERY_SOC_GOOD     = 3,   // 50-80% - Good operating range
    BATTERY_SOC_FULL     = 4,   // >80% - Battery is well charged
} battery_soc_level_t;

/**
 * Battery health status
 */
typedef enum {
    BATTERY_HEALTH_GOOD     = 0,  // Normal operation
    BATTERY_HEALTH_LOW      = 1,  // Low voltage warning
    BATTERY_HEALTH_CRITICAL = 2,  // Critical - shutdown recommended
    BATTERY_HEALTH_FAULT    = 3,  // Sensor fault or invalid reading
} battery_health_t;

/**
 * Battery monitoring data structure
 */
typedef struct {
    uint16_t adc_raw;              // Raw ADC reading (0-4095)
    float    voltage;              // Battery voltage (V)
    float    soc_percent;          // State of charge (0-100%)
    battery_soc_level_t soc_level; // SOC category
    battery_health_t health;       // Health status
    bool     is_charging;          // Charging detected (voltage > 13.8V)
    uint32_t last_update_ms;       // Timestamp of last reading
} battery_data_t;

/**
 * Battery monitor configuration
 */
typedef struct {
    uint8_t  adc_channel;          // ADC1 channel (0-7)
    uint8_t  gpio_num;             // GPIO number for ADC pin
    float    voltage_divider_ratio;// Voltage divider ratio (Vbat/Vadc)
    float    v_full;               // Fully charged voltage (14.6V for LiFePO4)
    float    v_nominal;            // Nominal voltage (12.8V)
    float    v_cutoff;             // Low voltage cutoff (10.0V)
    float    v_critical;           // Critical voltage (9.0V)
    uint16_t samples_per_read;     // Number of ADC samples to average
} battery_cfg_t;

/**
 * Initialize battery monitoring system
 * 
 * Sets up ADC peripheral and configures voltage divider parameters.
 * 
 * Voltage divider for 12V → 3.3V (actual hardware):
 *   R1 = 177.9 kΩ (top resistor, to battery)
 *   R2 = 30.22 kΩ (bottom resistor, to ground)
 *   Ratio = (R1 + R2) / R2 = 208.12 / 30.22 = 6.89
 *   Max measurable voltage = 3.3V × 6.89 = 22.7V ✓
 * 
 * Wiring:
 *   Battery +12V ──┬──> Power system
 *                  │
 *                R1 (177.9kΩ)
 *                  │
 *                  ├────> ESP32 GPIO35 (ADC1_CH7)
 *                  │
 *                R2 (30.22kΩ)
 *                  │
 *   Battery GND ───┴──> ESP32 GND
 * 
 * @param cfg  Battery configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_init(const battery_cfg_t *cfg);

/**
 * Read current battery state
 * 
 * Performs ADC reading with averaging, calculates voltage and SOC.
 * 
 * @param data  Pointer to store battery data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t battery_read(battery_data_t *data);

/**
 * Get last cached battery reading
 * 
 * Returns the most recent battery data without performing a new ADC read.
 * Useful for frequent queries without ADC overhead.
 * 
 * @param data  Pointer to store battery data
 * @return ESP_OK if data is valid, ESP_ERR_INVALID_STATE if never initialized
 */
esp_err_t battery_get_last(battery_data_t *data);

/**
 * Check if battery voltage is below safe operating threshold
 * 
 * @return true if voltage is critically low
 */
bool battery_is_critical(void);

/**
 * Check if battery is currently charging
 * 
 * Detects charging by voltage above float threshold (>13.8V for LiFePO4)
 * 
 * @return true if charging detected
 */
bool battery_is_charging(void);

/**
 * Get human-readable battery status string
 * 
 * @param data  Battery data structure
 * @return Status string (e.g., "GOOD", "LOW", "CRITICAL")
 */
const char* battery_get_status_string(const battery_data_t *data);

/**
 * Convert state of charge level to string
 * 
 * @param level  SOC level enum
 * @return Level string (e.g., "FULL", "GOOD", "LOW")
 */
const char* battery_soc_level_to_string(battery_soc_level_t level);

/**
 * Calibrate voltage reading with known reference voltage
 * 
 * Use a multimeter to measure actual battery voltage, then call this
 * function to adjust the ADC calibration.
 * 
 * @param measured_voltage  Actual voltage measured with multimeter (V)
 * @return ESP_OK on success
 */
esp_err_t battery_calibrate(float measured_voltage);

#endif // BATTERY_H