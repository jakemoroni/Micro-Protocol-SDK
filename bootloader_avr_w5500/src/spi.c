/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Implements an SPI bus driver for AVR devices.
 *
 * NOTE: The API defined here is generalized to support
 * multiple devices and configurations, but the driver
 * itself is hard-coded specifically for communication
 * with the W5500 chip on an Arduino UNO compatible deivce
 * and ethernet shield conbination in order to reduce the
 * size and complexity in the bootloader. Most of the API params
 * defined here therefore have no effect in the bootloader.
 */

#include <avr/io.h>
#include "spi.h"

#define SPI_DDR                        DDRB
#define CS                             PINB2
#define MOSI                           PINB3
#define MISO                           PINB4
#define SCK                            PINB5


void spi_assert_cs(struct spi_device *inst)
{
	/* Hard-coded for the W5500 shield. */
	PORTB &= ~(1u << CS);
}

void spi_deassert_cs(struct spi_device *inst)
{
	/* Hard-coded for the W5500 shield. */
	PORTB |= (1u << CS);
}

size_t spi_transfer(struct spi_device *inst,
		    uint8_t *tx_buf,
		    uint8_t *rx_buf,
		    size_t len)
{
	size_t i;

	for (i = 0; i < len; i++) {
		SPDR = tx_buf[i];

		/* Wait for transfer. */
		while (!(SPSR & (1 << SPIF)));

		rx_buf[i] = SPDR;
	}

	return len;
}

void spi_bus_init(void)
{
	/* MISO is input. */
	SPI_DDR &= ~(1u << MISO);

	/* The other pins are outputs. */
	SPI_DDR |= ((1u << MOSI) | (1u << CS) | (1u << SCK));

	/* Enable pullup on MISO. */
	PORTB |= (1u << MISO);

	/* Double the SPI rate. In the bootrom, run it slower... */
	/* SPSR = SPI2X; */

	/* Enable, MSb first, Master, Mode 0, Clock = F_CPU / 4 */
	SPCR = ((1 << SPE) | (0 << DORD) | (1 << MSTR));
}

void spi_bus_exit(void)
{
	SPCR = 0;
}
