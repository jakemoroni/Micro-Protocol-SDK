/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements methods for initializing and reading a tick counter.
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include "platform.h"

#define TIMER_TICKS_PER_SEC            (F_CPU / 1024u)

/* Starts the timer. */
void timer_init(void);

/* Returns the current timer value.
 * NOTE: This is not interrupt-safe.
 */
uint16_t timer_ticks_get(void);

#endif /* TIMER_H_ */
