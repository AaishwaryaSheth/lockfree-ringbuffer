/*
 * adc_sampler.h
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#ifndef ADC_SAMPLER_H_
#define ADC_SAMPLER_H_





#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

// ─────────────────────────────────────────────────────────────────────────────
// ADC Sampler
//
// Uses ESP32 GPTimer at 1 kHz (every 1 ms).
// ISR reads both ADC channels and pushes one adc_sample_t into the ring buffer.
// Gives wake_sem every 16 samples to wake the LED controller task.
//
// Hardware connections:
//   Potentiometer  wiper  → GPIO34  (ADC1_CH6)
//   ACS712 / ZMPT  output → GPIO35  (ADC1_CH7)
//   Both need 0–3.3V range. Never exceed 3.3V on ESP32 ADC pins.
// ─────────────────────────────────────────────────────────────────────────────

// Start the ADC sampler ISR.
// rb       : ring buffer to write samples into
// wake_sem : binary semaphore — given every 16 samples to wake consumer task
void adc_sampler_start(ring_buf_t *rb, SemaphoreHandle_t wake_sem);

// Total dropped samples since boot (ISR couldn't push — buffer was full)
uint32_t adc_sampler_get_overruns(void);
void     adc_sampler_reset_overruns(void);
#endif /* ADC_SAMPLER_H_ */
