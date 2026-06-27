
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "ringbuf.h"
#include "adc_sampler.h"
#include "ledcontroller.h"
#include "monitortask.h"

static const char *TAG = "main";

// ── Global shared resources ───────────────────────────────────────────────────
// Declared global so they live in .bss (zero-initialized) — no stack risk.

static ring_buf_t        g_rb;    // lock-free sample buffer
static SemaphoreHandle_t g_wake;  // binary semaphore: ISR → LED task signal

// ── app_main ──────────────────────────────────────────────────────────────────
// Initialization ORDER is critical:
//   1. rb_init         — zero head/tail before ISR can touch them
//   2. semaphore       — create before any task waits on it
//   3. led_controller  — task must exist before ISR gives semaphore
//   4. monitor_task    — start after subjects exist
//   5. adc_sampler     — ISR LAST — everything else must be ready

void app_main(void)
{
    ESP_LOGI(TAG, "  Analog Control System — ESP32");
    ESP_LOGI(TAG, "  Potentiometer + Current Sensor + RGB LED");
    ESP_LOGI(TAG, "Pins: POT=GPIO34  SENSOR=GPIO35");
    ESP_LOGI(TAG, "      RED=GPIO25  GREEN=GPIO26  BLUE=GPIO27");

    // Step 1: Initialize ring buffer
    rb_init(&g_rb);
    ESP_LOGI(TAG, "[1/5] Ring buffer initialized (%d slots)", RB_SIZE);

    // Step 2: Create binary semaphore
    // Binary semaphore = max 1 token. Giving it twice = same as once.
    // ISR gives it → LED task unblocks → drains buffer → sleeps again.
    g_wake = xSemaphoreCreateBinary();
    configASSERT(g_wake != NULL);
    ESP_LOGI(TAG, "[2/5] Wake semaphore created");

    // Step 3: Start LED controller task (waits on semaphore)
    led_controller_start(&g_rb, g_wake);
    ESP_LOGI(TAG, "[3/5] LED controller task started");

    // Step 4: Start monitor task (reads shared state, prints every 3s)
    monitor_task_start(&g_rb);
    ESP_LOGI(TAG, "[4/5] Monitor task started");

    // Step 5: Start ADC sampler + GPTimer ISR — starts firing immediately
    adc_sampler_start(&g_rb, g_wake);
    ESP_LOGI(TAG, "[5/5] ADC sampler ISR started at 1 kHz");

    ESP_LOGI(TAG, "System running. Adjust potentiometer to change brightness.");
    ESP_LOGI(TAG, "Current sensor zone → LED color:");
    ESP_LOGI(TAG, "  0–30%%  of scale → GREEN");
    ESP_LOGI(TAG, "  31–60%% of scale → BLUE");
    ESP_LOGI(TAG, "  61–85%% of scale → YELLOW");
    ESP_LOGI(TAG, "  >85%%   of scale → RED blink (DANGER)");

    // app_main returns here. FreeRTOS scheduler takes over.
    // Our tasks keep running independently.
}