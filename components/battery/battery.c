#include "battery.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BATTERY"

// Default configuration values
#define DEFAULT_V_FULL      14.6f   // LiFePO4 full charge (absorption)
#define DEFAULT_V_NOMINAL   12.8f   // LiFePO4 nominal voltage
#define DEFAULT_V_CUTOFF    10.0f   // BMS cutoff voltage
#define DEFAULT_V_CRITICAL   9.0f   // Emergency shutdown voltage
#define DEFAULT_SAMPLES      32     // ADC samples to average

// State variables
static battery_cfg_t s_config;
static battery_data_t s_last_reading;
static bool s_initialized = false;
static float s_calibration_offset = 0.0f;

// ADC handles
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;

/**
 * Calculate state of charge from voltage using LiFePO4 discharge curve
 * 
 * LiFePO4 has a very flat discharge curve between 13.0V and 12.0V (most of capacity).
 * This is a simplified linear approximation:
 * 
 *   100% → 14.6V (fully charged)
 *    90% → 13.6V
 *    80% → 13.2V
 *    50% → 12.8V (nominal)
 *    20% → 12.0V
 *    10% → 11.0V
 *     0% → 10.0V (cutoff)
 *  
 * Voltage Divider (actual hardware values):
 *   R1 = 177.9 kΩ (top resistor, battery to ADC)
 *   R2 = 30.22 kΩ (bottom resistor, ADC to GND)
 * 
 * Voltage divider calculation:
 *   Vout = Vin × R2 / (R1 + R2)
 *   Vout = 14.6V × 30.22kΩ / 208.12kΩ = 2.12V ✓ (safe for 3.3V ADC)
 *   
 * Ratio = (R1 + R2) / R2 = 208.12kΩ / 30.22kΩ = 6.89
 * Max measurable voltage = 3.3V × 6.89 = 22.7V (sufficient for 12V LiFePO4)
 * 
 */
static float voltage_to_soc(float voltage) {
    if (voltage >= s_config.v_full) {
        return 100.0f;
    } else if (voltage >= 13.6f) {
        // 90-100%: 14.6V → 13.6V
        return 90.0f + (voltage - 13.6f) / (s_config.v_full - 13.6f) * 10.0f;
    } else if (voltage >= 13.2f) {
        // 80-90%: 13.6V → 13.2V
        return 80.0f + (voltage - 13.2f) / (13.6f - 13.2f) * 10.0f;
    } else if (voltage >= s_config.v_nominal) {
        // 50-80%: 13.2V → 12.8V
        return 50.0f + (voltage - s_config.v_nominal) / (13.2f - s_config.v_nominal) * 30.0f;
    } else if (voltage >= 12.0f) {
        // 20-50%: 12.8V → 12.0V
        return 20.0f + (voltage - 12.0f) / (s_config.v_nominal - 12.0f) * 30.0f;
    } else if (voltage >= 11.0f) {
        // 10-20%: 12.0V → 11.0V
        return 10.0f + (voltage - 11.0f) / (12.0f - 11.0f) * 10.0f;
    } else if (voltage >= s_config.v_cutoff) {
        // 0-10%: 11.0V → 10.0V
        return (voltage - s_config.v_cutoff) / (11.0f - s_config.v_cutoff) * 10.0f;
    } else {
        return 0.0f;
    }
}

/**
 * Determine SOC category from percentage
 */
static battery_soc_level_t soc_to_level(float soc_percent) {
    if (soc_percent >= 80.0f) return BATTERY_SOC_FULL;
    if (soc_percent >= 50.0f) return BATTERY_SOC_GOOD;
    if (soc_percent >= 20.0f) return BATTERY_SOC_MEDIUM;
    if (soc_percent >= 10.0f) return BATTERY_SOC_LOW;
    return BATTERY_SOC_CRITICAL;
}

/**
 * Determine health status from voltage
 */
static battery_health_t voltage_to_health(float voltage) {
    if (voltage < s_config.v_critical) return BATTERY_HEALTH_CRITICAL;
    if (voltage < s_config.v_cutoff)   return BATTERY_HEALTH_LOW;
    return BATTERY_HEALTH_GOOD;
}

esp_err_t battery_init(const battery_cfg_t *cfg) {
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    if (!cfg) {
        ESP_LOGE(TAG, "NULL configuration");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Copy configuration
    memcpy(&s_config, cfg, sizeof(battery_cfg_t));
    
    // Apply defaults if not set
    if (s_config.v_full == 0.0f)     s_config.v_full = DEFAULT_V_FULL;
    if (s_config.v_nominal == 0.0f)  s_config.v_nominal = DEFAULT_V_NOMINAL;
    if (s_config.v_cutoff == 0.0f)   s_config.v_cutoff = DEFAULT_V_CUTOFF;
    if (s_config.v_critical == 0.0f) s_config.v_critical = DEFAULT_V_CRITICAL;
    if (s_config.samples_per_read == 0) s_config.samples_per_read = DEFAULT_SAMPLES;
    
    ESP_LOGI(TAG, "Initializing battery monitor on GPIO %d (ADC1 CH%d)", 
             cfg->gpio_num, cfg->adc_channel);
    ESP_LOGI(TAG, "Voltage divider ratio: %.2f (max %.1fV)", 
             cfg->voltage_divider_ratio, cfg->voltage_divider_ratio * 3.3f);
    ESP_LOGI(TAG, "LiFePO4 thresholds: Full=%.1fV Nominal=%.1fV Cutoff=%.1fV Critical=%.1fV",
             s_config.v_full, s_config.v_nominal, s_config.v_cutoff, s_config.v_critical);
    
    // Initialize ADC oneshot mode
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,  // 0-3.3V range
    };
    ret = adc_oneshot_config_channel(s_adc_handle, cfg->adc_channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }
    
    // Initialize ADC calibration (eFuse-based)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration not available, using raw values");
        s_adc_cali_handle = NULL;
    } else {
        ESP_LOGI(TAG, "ADC calibration enabled (eFuse)");
    }
    
    // Initialize last reading structure
    memset(&s_last_reading, 0, sizeof(battery_data_t));
    
    s_initialized = true;
    ESP_LOGI(TAG, "Battery monitor initialized successfully");
    
    // Perform initial reading
    battery_data_t initial;
    ret = battery_read(&initial);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial reading: %.2fV (%.0f%% SOC)", 
                 initial.voltage, initial.soc_percent);
    }
    
    return ESP_OK;
}

esp_err_t battery_read(battery_data_t *data) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data) {
        ESP_LOGE(TAG, "NULL data pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Accumulate multiple ADC samples for averaging
    uint32_t adc_sum = 0;
    uint16_t valid_samples = 0;
    
    for (int i = 0; i < s_config.samples_per_read; i++) {
        int raw_value;
        esp_err_t ret = adc_oneshot_read(s_adc_handle, s_config.adc_channel, &raw_value);
        
        if (ret == ESP_OK && raw_value >= 0 && raw_value <= 4095) {
            adc_sum += raw_value;
            valid_samples++;
        }
        
        // Small delay between samples (100µs)
        vTaskDelay(1);
    }
    
    if (valid_samples == 0) {
        ESP_LOGE(TAG, "Failed to read ADC");
        data->health = BATTERY_HEALTH_FAULT;
        return ESP_FAIL;
    }
    
    // Calculate average ADC value
    uint16_t adc_avg = adc_sum / valid_samples;
    data->adc_raw = adc_avg;
    
    // Convert ADC to voltage
    float voltage_adc;
    if (s_adc_cali_handle) {
        // Use calibrated voltage
        int voltage_mv;
        adc_cali_raw_to_voltage(s_adc_cali_handle, adc_avg, &voltage_mv);
        voltage_adc = voltage_mv / 1000.0f;
    } else {
        // Use raw conversion (3.3V full scale)
        voltage_adc = (adc_avg / 4095.0f) * 3.3f;
    }
    
    // Apply voltage divider ratio to get battery voltage
    float voltage_battery = voltage_adc * s_config.voltage_divider_ratio;
    
    // Apply calibration offset
    voltage_battery += s_calibration_offset;
    
    data->voltage = voltage_battery;
    
    // Calculate state of charge
    data->soc_percent = voltage_to_soc(voltage_battery);
    data->soc_level = soc_to_level(data->soc_percent);
    
    // Determine health status
    data->health = voltage_to_health(voltage_battery);
    
    // Detect charging (voltage above float threshold)
    data->is_charging = (voltage_battery > 13.8f);
    
    // Update timestamp
    data->last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Cache the reading
    memcpy(&s_last_reading, data, sizeof(battery_data_t));
    
    return ESP_OK;
}

esp_err_t battery_get_last(battery_data_t *data) {
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &s_last_reading, sizeof(battery_data_t));
    return ESP_OK;
}

bool battery_is_critical(void) {
    if (!s_initialized) return false;
    return s_last_reading.health == BATTERY_HEALTH_CRITICAL;
}

bool battery_is_charging(void) {
    if (!s_initialized) return false;
    return s_last_reading.is_charging;
}

const char* battery_get_status_string(const battery_data_t *data) {
    if (!data) return "UNKNOWN";
    
    switch (data->health) {
        case BATTERY_HEALTH_GOOD:     return "GOOD";
        case BATTERY_HEALTH_LOW:      return "LOW";
        case BATTERY_HEALTH_CRITICAL: return "CRITICAL";
        case BATTERY_HEALTH_FAULT:    return "FAULT";
        default:                      return "UNKNOWN";
    }
}

const char* battery_soc_level_to_string(battery_soc_level_t level) {
    switch (level) {
        case BATTERY_SOC_FULL:     return "FULL";
        case BATTERY_SOC_GOOD:     return "GOOD";
        case BATTERY_SOC_MEDIUM:   return "MEDIUM";
        case BATTERY_SOC_LOW:      return "LOW";
        case BATTERY_SOC_CRITICAL: return "CRITICAL";
        default:                   return "UNKNOWN";
    }
}

esp_err_t battery_calibrate(float measured_voltage) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    battery_data_t current;
    esp_err_t ret = battery_read(&current);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read battery for calibration");
        return ret;
    }
    
    // Calculate offset needed to match measured voltage
    float offset = measured_voltage - current.voltage;
    s_calibration_offset = offset;
    
    ESP_LOGI(TAG, "Calibration: measured=%.2fV, read=%.2fV, offset=%.3fV",
             measured_voltage, current.voltage, offset);
    
    // Re-read to update cached value
    battery_read(&current);
    
    return ESP_OK;
}