/*
 * adc_sampler.c
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#include "adc_sampler.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_private/adc_private.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "adc_sampler";

// ── Private state ─────────────────────────────────────────────────────────────
static ring_buf_t        *s_rb       = NULL;
static SemaphoreHandle_t  s_wake     = NULL;
static volatile uint32_t  s_overruns = 0;

static adc_oneshot_unit_handle_t s_adc1_handle;

// ── ISR ───────────────────────────────────────────────────────────────────────
// Runs at 1 kHz (every 1 ms).
// IRAM_ATTR: pins function in IRAM so flash cache miss can't stall the ISR.
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_data)
{
    adc_sample_t sample;

    // Read both ADC channels.
    // adc_oneshot_read_isr() is the ISR-safe version of adc_oneshot_read().
    // Returns raw 12-bit value: 0 = 0V, 4095 = 3.3V
    adc_oneshot_read_isr(s_adc1_handle, ADC_CHANNEL_6, (int*)&sample.potentiometer);
    adc_oneshot_read_isr(s_adc1_handle, ADC_CHANNEL_7, (int*)&sample.current_sensor);
    sample.timestamp_us = (uint32_t)esp_timer_get_time();

    // Push into ring buffer — lock-free, no mutex needed
    if (!rb_push(s_rb, &sample)) {
        s_overruns++;   // Buffer full — consumer task is too slow
    }

    // Wake consumer task every 16 samples (not every sample).
    // Giving semaphore 1000x/sec has overhead — batching to 62.5 Hz is plenty.
    if ((s_rb->head & 0x0F) == 0) {
        BaseType_t higher_prio_woken = pdFALSE;
        xSemaphoreGiveFromISR(s_wake, &higher_prio_woken);
        // Return true = request context switch if consumer has higher priority
        return higher_prio_woken == pdTRUE;
    }

    return false;
}

// ── Public API ────────────────────────────────────────────────────────────────
void adc_sampler_start(ring_buf_t *rb, SemaphoreHandle_t wake_sem)
{
    s_rb   = rb;
    s_wake = wake_sem;

    // ── Configure ADC1 unit ───────────────────────────────────────────────────
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &s_adc1_handle));

    // ── Configure ADC channels ────────────────────────────────────────────────
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,   // 0–3.3V range
        .bitwidth = ADC_BITWIDTH_12,   // 12-bit → 0..4095
    };

    // GPIO34 → ADC1 channel 6 (potentiometer)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_6, &chan_cfg));

    // GPIO35 → ADC1 channel 7 (current sensor / ACS712 output)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, ADC_CHANNEL_7, &chan_cfg));

    ESP_LOGI(TAG, "ADC1 configured: CH6=GPIO34 (pot), CH7=GPIO35 (sensor)");

    // ── Configure GPTimer ─────────────────────────────────────────────────────
    gptimer_handle_t timer;

    gptimer_config_t timer_cfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,  // 80 MHz APB clock
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,                   // 1 tick = 1 microsecond
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer));

    // Fire every 1000 µs = 1 ms = 1 kHz sampling rate
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count                = 1000,
        .reload_count               = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));

    gptimer_event_callbacks_t cbs = { .on_alarm = on_timer_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    ESP_LOGI(TAG, "GPTimer started: 1 kHz sampling");
}

uint32_t adc_sampler_get_overruns(void)   { return s_overruns; }
void     adc_sampler_reset_overruns(void) { s_overruns = 0;    }


