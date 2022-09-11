/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Linker script for the bootloader.
 */

OUTPUT_FORMAT("elf32-avr")
OUTPUT_ARCH(avr:5)

/* Specific to the ATmega328P. */
__FLASH_SIZE__ = 0x8000;
__SRAM_SIZE__ = 0x800;

/* Bootloader size must match fuse settings. */
__BOOTLOADER_SIZE__ = 0x1000;

MEMORY {
	/* The bootloader is located at the end of flash. */
	FLASH_REGION (rx) : ORIGIN = (__FLASH_SIZE__ - __BOOTLOADER_SIZE__), LENGTH = __BOOTLOADER_SIZE__

	/* Use 1/2 of the total SRAM for .data and .bss, leaving the other half for the stack.
	 * The ATmega328P SRAM starts at 0x100, and GCC offsets all SRAM addresses by 0x800000
	 * to deal with the Harvard architecture of the AVR. I think GCC also does a fixup for
	 * this depending on the -mmcu argument, but we might as well set it correctly.
	 */
	SRAM_REGION (rw!x) : ORIGIN = 0x800100, LENGTH = (__SRAM_SIZE__ / 2)
}

SECTIONS {
	.text : {
		*(.crt_startup) KEEP (*(.crt_startup))
		. = ALIGN(2);
		*(.text)
		. = ALIGN(2);
		*(.text.*)
		. = ALIGN(2);
		/* The data section initialization values end up getting stored right after
		 * the last text data in flash and get loaded to SRAM in main().
		 */
	} > FLASH_REGION

	.data : {
		__data_sram_start = .; /* Symbol used by C code for data init. */
		*(.data)
		*(.data*)
		/* rodata needs to be manually placed into the data section to prevent
		 * GCC from placing it in the text section which isn't directly
		 * addressable on the AVR since it's in flash.
		 */
		*(.rodata)
		*(.rodata*)
		. = ALIGN(2); /* Align for storage into word addressed flash. */
	} > SRAM_REGION AT > FLASH_REGION

	__data_flash_start = LOADADDR(.data);
	__data_flash_end = __data_flash_start + SIZEOF(.data);

	.bss (NOLOAD) : {
		__bss_sram_start = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		__bss_sram_end = .;
	} > SRAM_REGION
}
