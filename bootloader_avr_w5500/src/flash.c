/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements methods for writing the system flash.
 */

#include "flash.h"

void flash_page_program(uint32_t addr, uint8_t *page_buf)
{
	uint16_t i;
	uint16_t le_word;

	/* Erase the page. */
	boot_page_erase(addr);
	boot_spm_busy_wait();

	/* Fill page buffer. */
	for (i = 0; i < SPM_PAGESIZE; i += 2) {
		le_word = page_buf[1];
		le_word <<= 8u;
		le_word |= page_buf[0];
		boot_page_fill(addr + i, le_word);
		page_buf += 2;
	}

	/* Write. */
	boot_page_write(addr);
	boot_spm_busy_wait();
	boot_rww_enable();
}
