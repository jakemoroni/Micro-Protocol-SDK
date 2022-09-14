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

#ifdef __AVR_ATmega2560__
#define SPI_DDR                        DDRB
#define DEVICE_CS0                     PINB4
#define MOSI                           PINB2
#define MISO                           PINB3
#define SCK                            PINB1
#define NATIVE_CS                      PINB0
#else
#define SPI_DDR                        DDRB
#define DEVICE_CS0                     PINB2
#define MOSI                           PINB3
#define MISO                           PINB4
#define SCK                            PINB5
#define NATIVE_CS                      DEVICE_CS0
#endif

void spi_assert_cs(struct spi_device *inst)
{
	/* Hard-coded for the W5500 shield. */
	PORTB &= ~(1u << DEVICE_CS0);
}

void spi_deassert_cs(struct spi_device *inst)
{
	/* Hard-coded for the W5500 shield. */
	PORTB |= (1u << DEVICE_CS0);
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

	/* Set all CS high (activates a pull-up until they're configured as outputs)
	 * and activate a pull-up on MISO.
	 */
	PORTB |= ((1u << NATIVE_CS) | (1u << DEVICE_CS0) | (1u << MISO));

	/* Enable outputs.
	 * NOTE: The native CS pin (i.e., the one used when in slave mode) must
	 *       be set to output before enabling the SPI controller.
	 */
	SPI_DDR |= ((1u << MOSI) | (1u << SCK) | (1u << NATIVE_CS) | (1u << DEVICE_CS0));

	/* Double the SPI rate. In the bootrom, run it slower... */
	/* SPSR = SPI2X; */

	/* Enable, MSb first, Master, Mode 0, Clock = F_CPU / 4 */
	SPCR = ((1 << SPE) | (0 << DORD) | (1 << MSTR));
}

void spi_bus_exit(void)
{
	SPCR = 0;
}
