/*
 * ledcontroller.h
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#ifndef LEDCONTROLLER_H_
#define LEDCONTROLLER_H_

#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

// ─────────────────────────────────────────────────────────────────────────────
// LED Controller Task
//
// Controls RGB LED brightness based on current sensor reading:
//   - Idle/Normal: 70% brightness (white light)
//   - Current increases: brightness increases up to 100%
//   - Current decreases: brightness decreases down to 50%
//   - Limits: 50% minimum, 100% maximum
//
// RGB LED wiring (common cathode):
//   Red   → GPIO2  (LEDC channel 0)
//   Green → GPIO2  (LEDC channel 1)
//   Blue  → GPIO2  (LEDC channel 2)
//   GND   → GND via 220Ω resistor on each channel
// ─────────────────────────────────────────────────────────────────────────────

#define LED_GPIO_RED    4
#define LED_GPIO_GREEN  4
#define LED_GPIO_BLUE   4

void led_controller_start(ring_buf_t *rb, SemaphoreHandle_t wake_sem);

// Last computed values — read by monitor task
uint16_t led_get_last_sensor_raw(void);
uint8_t  led_get_brightness_pct(void);    // Current brightness (50-100%)

#endif /* LEDCONTROLLER_H_ */