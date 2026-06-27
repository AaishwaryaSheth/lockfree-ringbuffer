/*
 * ringbuf.h
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_attr.h"

// ─────────────────────────────────────────────────────────────────────────────
// Lock-free SPSC ring buffer
// Writer: ISR only (adc_sampler ISR)
// Reader: consumer task only (led_controller task)
// Rule: ONE writer + ONE reader = no lock needed
// ─────────────────────────────────────────────────────────────────────────────

#define RB_SIZE  256          // must be power of 2
#define RB_MASK  (RB_SIZE-1)

// Each sample carries both ADC channels
typedef struct {
    uint16_t potentiometer;   // GPIO34 — 0..4095 (12-bit ADC)
    uint16_t current_sensor;  // GPIO35 — 0..4095 (12-bit ADC)
    uint32_t timestamp_us;    // from esp_timer_get_time()
} adc_sample_t;

typedef struct {
    adc_sample_t buf[RB_SIZE];
    volatile uint32_t head;   // moved ONLY by ISR (writer)
    volatile uint32_t tail;   // moved ONLY by task (reader)
} ring_buf_t;

static inline void rb_init(ring_buf_t *rb) {
    rb->head = 0;
    rb->tail = 0;
}

// Called from ISR only — returns false if buffer full (overrun)
static inline IRAM_ATTR bool rb_push(ring_buf_t *rb, const adc_sample_t *sample) {
    uint32_t head = __atomic_load_n(&rb->head, __ATOMIC_RELAXED);
    uint32_t next = (head + 1) & RB_MASK;

    // Full check — ACQUIRE: see latest tail written by consumer task
    if (next == __atomic_load_n(&rb->tail, __ATOMIC_ACQUIRE)) {
        return false;  // buffer full — caller increments overrun counter
    }

    rb->buf[head] = *sample;

    // RELEASE: data write visible before head pointer update
    __atomic_store_n(&rb->head, next, __ATOMIC_RELEASE);
    return true;
}

// Called from task only — returns false if buffer empty
static inline bool rb_pop(ring_buf_t *rb, adc_sample_t *out) {
    uint32_t tail = __atomic_load_n(&rb->tail, __ATOMIC_RELAXED);

    // Empty check — ACQUIRE: see latest head written by ISR
    if (tail == __atomic_load_n(&rb->head, __ATOMIC_ACQUIRE)) {
        return false;
    }

    *out = rb->buf[tail];

    // RELEASE: tail update visible to ISR full-check
    __atomic_store_n(&rb->tail, (tail + 1) & RB_MASK, __ATOMIC_RELEASE);
    return true;
}

static inline uint32_t rb_count(ring_buf_t *rb) {
    uint32_t h = __atomic_load_n(&rb->head, __ATOMIC_ACQUIRE);
    uint32_t t = __atomic_load_n(&rb->tail, __ATOMIC_RELAXED);
    return (h - t) & RB_MASK;
}

static inline uint32_t rb_percent_full(ring_buf_t *rb) {
    return (rb_count(rb) * 100) / RB_SIZE;
}



#endif /* RINGBUF_H_ */
