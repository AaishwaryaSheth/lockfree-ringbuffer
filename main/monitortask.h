/*
 * monitortask.h
 *
 *  Created on: 07-Jun-2026
 *      Author: avshe
 */

#ifndef MONITORTASK_H_
#define MONITORTASK_H_


#pragma once
#include "ringbuf.h"

// ─────────────────────────────────────────────────────────────────────────────
// Monitor Task
//
// Runs every 3 seconds. Prints live system health over UART:
//   - Potentiometer raw value + voltage equivalent
//   - Current sensor raw value + estimated mA (ACS712 5A model)
//   - Active LED zone and color name
//   - Ring buffer fill percentage
//   - ISR overrun count
//   - Free heap
//   - Stack high-water marks for all tasks
// ─────────────────────────────────────────────────────────────────────────────

void monitor_task_start(ring_buf_t *rb);


#endif /* MONITORTASK_H_ */
