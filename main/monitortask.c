/*
 * monitortask.c
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#include "freertos/idf_additions.h"
#include "monitortask.h"
#include "ledcontroller.h"
#include "adc_sampler.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "monitor";
static ring_buf_t *s_rb = NULL;

// ── ACS712 current conversion ─────────────────────────────────────────────────
static int32_t raw_to_milliamps(uint16_t raw) {
    float voltage   = (raw / 4095.0f) * 3.3f;
    float current_A = (voltage - 1.65f) / 0.185f;
    return (int32_t)(current_A * 1000.0f);
}

// ── Brightness level indicator ──────────────────────────────────────────────
static const char* brightness_level(uint8_t pct) {
    if (pct >= 95) return "HIGH (near 100%)";
    if (pct >= 85) return "HIGH";
    if (pct >= 75) return "ELEVATED";
    if (pct >= 65) return "NORMAL";
    if (pct >= 55) return "REDUCED";
    return "LOW (near 50%)";
}

// ── Monitor task ──────────────────────────────────────────────────────────────
static void monitor_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(3000));

        // Collect all metrics
        uint16_t sensor_raw = led_get_last_sensor_raw();
        uint8_t  brightness = led_get_brightness_pct();
        uint32_t buf_pct    = rb_percent_full(s_rb);
        uint32_t overruns   = adc_sampler_get_overruns();
        uint32_t free_heap  = esp_get_free_heap_size();
        uint32_t min_heap   = esp_get_minimum_free_heap_size();
        int32_t  milliamps  = raw_to_milliamps(sensor_raw);

        // Print health report
        ESP_LOGI(TAG, "═══════════════ SYSTEM HEALTH REPORT ═══════════════");
        ESP_LOGI(TAG, "  Current sensor: %4u raw  (%+6ld mA)", 
                 sensor_raw, (long)milliamps);
        ESP_LOGI(TAG, "  LED brightness: %3u%%  (%s)", 
                 brightness, brightness_level(brightness));
        ESP_LOGI(TAG, "  Buffer fill   : %lu%%", (unsigned long)buf_pct);
        ESP_LOGI(TAG, "  ISR overruns  : %lu", (unsigned long)overruns);
        ESP_LOGI(TAG, "  Heap free     : %lu B  (min ever: %lu B)",
                 (unsigned long)free_heap, (unsigned long)min_heap);
        ESP_LOGI(TAG, "  Monitor HWM   : %u words",
                 uxTaskGetStackHighWaterMark(NULL));

        // Warnings
        if (overruns > 0) {
            ESP_LOGW(TAG, "⚠ ISR overruns detected (%lu) — consumer falling behind",
                     (unsigned long)overruns);
            adc_sampler_reset_overruns();
        }
        if (brightness >= 95) {
            ESP_LOGW(TAG, "⚠ BRIGHTNESS NEAR MAX - Current very high");
        }
        if (brightness <= 55) {
            ESP_LOGW(TAG, "⚠ BRIGHTNESS NEAR MIN - Current very low");
        }
        if (min_heap < 20000) {
            ESP_LOGE(TAG, "✗ LOW HEAP — possible memory leak");
        }
        if (buf_pct > 75) {
            ESP_LOGW(TAG, "⚠ Ring buffer >75%% full — risk of overrun");
        }
    }
}

void monitor_task_start(ring_buf_t *rb)
{
    s_rb = rb;

    xTaskCreatePinnedToCore(
        monitor_task,
        "monitor",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    ESP_LOGI(TAG, "Monitor task started, reporting every 3 seconds");
}