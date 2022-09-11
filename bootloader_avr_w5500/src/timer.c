/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements methods for initializing and reading a tick counter.
 */

#include "timer.h"

void timer_init(void)
{
	TCCR1A = 0;
	TCCR1B = 0x5; /* F_CPU / 1024 */
}

uint16_t timer_ticks_get(void)
{
	const uint16_t ret = TCNT1;
	return ret;
}
