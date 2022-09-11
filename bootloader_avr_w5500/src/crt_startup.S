/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Sets up the stack before jumping to C code.
 * The data and BSS init gets handled in C code (main).
 */

#include <avr/io.h>

/* Force this code to be located at the beginning of the text section. */
.section .crt_startup,"ax",@progbits

clr	r1 /* R1 is always 0. */
out	AVR_STATUS_ADDR, r1
ldi	r28, lo8(RAMEND)
ldi	r29, hi8(RAMEND)
out	AVR_STACK_POINTER_HI_ADDR, r29
out	AVR_STACK_POINTER_LO_ADDR, r28
call	main
