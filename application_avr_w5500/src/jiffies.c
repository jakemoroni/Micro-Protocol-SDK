/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements a jiffies() function which returns the absolute
 * number of ticks since the time of the last jiffies_init()
 * call.
 */

#include <stdbool.h>
#include <avr/interrupt.h>
#include "platform.h"
#include "jiffies.h"

static jiffies_t timer_ticks_hi;

ISR(TIMER0_OVF_vect)
{
	timer_ticks_hi += 256u;
}

void jiffies_init(void)
{
	uint8_t flags = spin_lock_irqsave();

	TCCR0A = 0;
	TCCR0B = 0; /* Stop*/
	TCNT0 = 0;

	/* Clear any potential interrupt flags. */
	TIFR0 = (1 << TOV0);
	TIMSK0 = (1 << TOIE0);

	timer_ticks_hi = 0;

	TCCR0B = 0x3; /* Start timer - F_CPU / 64, 4uS per tick */

	spin_unlock_irqrestore(flags);
}

jiffies_t jiffies(void)
{
	bool tifr_before;
	bool tifr_after;
	uint8_t tcnt;
	uint8_t flags;
	jiffies_t ret;

	flags = spin_lock_irqsave();

	do {
		tifr_before = (TIFR0 & (1u << TOV0));
		tcnt = TCNT0;
		tifr_after = (TIFR0 & (1u << TOV0));
	} while (tifr_before != tifr_after);

	ret = timer_ticks_hi + tcnt;

	if (tifr_before) {
		ret += 256u;
	}

	spin_unlock_irqrestore(flags);

	return ret;
}
