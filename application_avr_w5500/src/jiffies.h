/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements a jiffies() function which returns the absolute
 * number of ticks since the time of the last jiffies_init()
 * call.
 */

#ifndef JIFFIES_H_
#define JIFFIES_H_

#include <stdint.h>

#define MICROSECONDS_PER_JIFFY         4u
#define JIFFIES_PER_SECOND             (1000000u / MICROSECONDS_PER_JIFFY)
#define JIFFIES_64                     1

#ifdef JIFFIES_64
/* Jiffies overflows every ~2 million years. */
typedef uint64_t jiffies_t;
#else
/* Jiffies overflows every 286 minutes. */
typedef uint32_t jiffies_t;
#endif

/* Initializes and starts the jiffies counter. */
void jiffies_init(void);

/* Returns the number of ticks since the last jiffies_init() call. */
jiffies_t jiffies(void);

#endif /* JIFFIES_H_ */
