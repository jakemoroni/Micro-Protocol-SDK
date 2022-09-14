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

/* The program counter on these devices is 16 bits and represents
 * a word offset, so 128 KiB can be addressed. For devices with
 * > 128 KiB of flash, the PC is expanded to 3 bytes and a special
 * segment register is used to indicate which 128 KiB region indirect
 * jumps/calls should be referenced to. The compiler expects this to be
 * set correctly, such that it points to the segment that contains
 * the trampoline section (i.e., in the bootloader area).
 * For now, we only support the ATmega2560 and ATmega328P, and the
 * 2560 has 256 KiB of flash. Since the bootloader is always at the
 * end of flash, just force the segment to 1 for the 2560.
 */
#ifdef __AVR_ATmega2560__
ldi	r28, 1
out	_SFR_IO_ADDR(EIND), r28
#endif

call	main
