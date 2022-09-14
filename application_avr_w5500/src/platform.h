/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements simple platform-specific helper functions.
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_

/* I think F_CPU is used by some AVR header files, so define
 * it here first.
 */
#define F_CPU                          16000000u

#include <stdint.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

/* Spins for a number of milliseconds. */
static inline void platform_mdelay(uint32_t millis)
{
	uint32_t i;
	for (i = 0; i < millis; i++) {
		__builtin_avr_delay_cycles(F_CPU / 1000u); /* Cycles. */
	}
}

/* Performs a 16 bit byte swap. */
static inline uint16_t bswap16(uint16_t val)
{
	uint16_t ret;
	ret = (val & 0xFF);
	ret <<= 8u;
	ret |= val >> 8;
	return ret;
}

/* Performs a 32 bit byte swap. */
static inline uint32_t bswap32(uint32_t val)
{
	uint32_t ret;
	ret = (val & 0xFF);
	ret <<= 8u;
	ret |= (val >> 8) & 0xFF;
	ret <<= 8u;
	ret |= (val >> 16) & 0xFF;
	ret <<= 8u;
	ret |= (val >> 24) & 0xFF;
	return ret;
}

/* Converts a 16 bit value to big endian. */
static inline uint16_t cpu_to_be16(uint16_t val)
{
	return bswap16(val);
}

/* Converts a 32 bit value to big endian. */
static inline uint32_t cpu_to_be32(uint32_t val)
{
	return bswap32(val);
}

/* Converts a 32 bit value to little endian. */
static inline uint32_t be32_to_cpu(uint32_t val)
{
	return bswap32(val);
}

/* Read a byte from the EEPROM. */
static inline uint8_t read_eeprom(uint16_t offset)
{
	/* Wait for completion of previous write */
	while (EECR & (1 << EEPE));

	EEAR = offset;
	EECR |= (1<<EERE);

	return EEDR;
}

/* Copies the base MAC address into the supplied buffer. */
static inline void platform_get_mac(uint8_t *buf)
{
	size_t i;

	for (i = 0; i < 6u; i++) {
		buf[i] = read_eeprom(EEPROM_MAC_OFFSET + i);
	}
}

#endif /* PLATFORM_H_ */
