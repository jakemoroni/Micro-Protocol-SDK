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

#ifndef SPI_H_
#define SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* SPI bus modes. */
enum spi_bus_mode {
	SPI_BUS_MODE_0,
	SPI_BUS_MODE_1,
	SPI_BUS_MODE_2,
	SPI_BUS_MODE_3,
};

/* SPI bus numbers.
 * TODO - Add description of bus number to physical itf mapping.
 */
enum spi_bus_num {
	SPI_BUS_NUM_0,
};

/* Dedicated chip select pins. If the device doesn't use
 * a dedicated CS pin, then it must be controlled by the user.
 */
enum spi_bus_cs_pin {
	/* Maps to Digital Pin 10 on an Arduino UNO compatible device, pin B2 on AVR. */
	SPI_BUS_CS_PIN_0,
};

/* SPI device instance. All fields must be initialized by user. */
struct spi_device {
	enum spi_bus_num bus_num;
	enum spi_bus_mode mode;
	enum spi_bus_cs_pin cs_pin;
	/* -1 for freq_hz == max */
	uint32_t freq_hz;
	/* TODO - Perhaps add CS timing parameters to ensure that
	 * any required CS setup times can be met. For now, the W5500
	 * specifies a setup time of 5 ns, and the AVR has a cycle period
	 * of 60 ns, so it's not necessary.
	 */
};

/* Drives the device's CS pin low. */
void spi_assert_cs(struct spi_device *inst);

/* Drives the device's CS pin high (actively driven high on AVR). */
void spi_deassert_cs(struct spi_device *inst);

/* Shifts len bytes out of tx_buf and writes received data into rx_buf.
 * Returns the number of bytes transferred.
 */
size_t spi_transfer(struct spi_device *inst,
		    uint8_t *tx_buf,
		    uint8_t *rx_buf,
		    size_t len);

/* Initializes the SPI driver. */
void spi_bus_init(void);

/* Disables the SPI driver (necessary for re-using the pins). */
void spi_bus_exit(void);

#endif /* SPI_H_ */
