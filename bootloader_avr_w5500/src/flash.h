/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements methods for writing the system flash.
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>
#include <avr/boot.h>

#define FLASH_PAGE_SIZE                SPM_PAGESIZE

/* Writes a page of data to flash. */
void flash_page_program(uint16_t addr, uint8_t *page_buf);

#endif /* FLASH_H_ */
