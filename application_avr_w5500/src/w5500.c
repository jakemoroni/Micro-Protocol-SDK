/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * W5500 Ethernet Driver (Tested with an Arduino Ethernet Shield V2).
 */

#include "platform.h"
#include "w5500.h"
#include <string.h>

/* Number of milliseconds to wait for the reset bit to self-clear. */
#define W5500_RESET_TIMEOUT            4000

/* Number of milliseconds to wait for the device to begin accepting writes. */
#define W5500_FIRST_WRITE_TIMEOUT      4000

/* Number of milliseconds to wait for the raw socket to open. */
#define W5500_SOCKET_OPEN_TIMEOUT      2000

/* Write enable bit. */
#define W5500_CONTROL_RW_WRITE         0x04

/* Common regs. */
#define W5500_COMMON_REG_MODE          0x00
  #define W5500_COMMON_REG_MODE_RST    0x80
#define W5500_COMMON_REG_GWIP_0        0x01
#define W5500_COMMON_REG_GWIP_1        0x02
#define W5500_COMMON_REG_GWIP_2        0x03
#define W5500_COMMON_REG_GWIP_3        0x04

/* Socket regs. */
#define W5500_SOCKET_REG_MODE                    0x00
  #define W5500_SOCKET_REG_MODE_MAC_FILTER         0x80
  #define W5500_SOCKET_REG_MODE_MACRAW             0x04
#define W5500_SOCKET_REG_COMMAND                 0x01
  #define W5500_SOCKET_REG_COMMAND_OPEN            0x01
  #define W5500_SOCKET_REG_COMMAND_SEND            0x20
  #define W5500_SOCKET_REG_COMMAND_RECV            0x40
#define W5500_SOCKET_REG_INTERRUPT               0x02
  #define W5500_SOCKET_REG_INTERRUPT_SENDOK        0x10
#define W5500_SOCKET_REG_STATUS                  0x03
#define W5500_SOCKET_REG_STATUS_SOCK_MACRAW      0x42
#define W5500_SOCKET_REG_TX_FREE_SIZE_UPPER      0x20
#define W5500_SOCKET_REG_TX_FREE_SIZE_LOWER      0x21
#define W5500_SOCKET_REG_TX_WRITE_IDX_UPPER      0x24
#define W5500_SOCKET_REG_TX_WRITE_IDX_LOWER      0x25
#define W5500_SOCKET_REG_RCVD_DATA_SIZE_UPPER    0x26
#define W5500_SOCKET_REG_RCVD_DATA_SIZE_LOWER    0x27
#define W5500_SOCKET_REG_RX_READ_IDX_UPPER       0x28
#define W5500_SOCKET_REG_RX_READ_IDX_SIZE_LOWER  0x29

/* Buffer sizes are in units of 1 KiB. */
#define W5500_SOCKET_REG_RX_BUF_SIZE   0x1E
#define W5500_SOCKET_REG_TX_BUF_SIZE   0x1F

/* W5500 register blocks. */
enum w5500_block_select {
	W5500_BLOCK_SELECT_COMMON = 0x00,
	W5500_BLOCK_SELECT_SOCKET_0_REG = 0x08,
	W5500_BLOCK_SELECT_SOCKET_0_TX_BUF = 0x10,
	W5500_BLOCK_SELECT_SOCKET_0_RX_BUF = 0x18,
	W5500_BLOCK_SELECT_SOCKET_1_REG = 0x28,
	W5500_BLOCK_SELECT_SOCKET_2_REG = 0x48,
	W5500_BLOCK_SELECT_SOCKET_3_REG = 0x68,
	W5500_BLOCK_SELECT_SOCKET_4_REG = 0x88,
	W5500_BLOCK_SELECT_SOCKET_5_REG = 0xA8,
	W5500_BLOCK_SELECT_SOCKET_6_REG = 0xC8,
	W5500_BLOCK_SELECT_SOCKET_7_REG = 0xE8,
};

/* Reads an 8 bit register. */
static uint8_t reg_read_8(struct w5500_device *inst,
			  enum w5500_block_select block_select,
			  uint16_t offset)
{
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];

	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = offset >> 8u;
	tx_buf[1] = offset;
	tx_buf[2] = block_select;
	tx_buf[3] = 0;

	spi_assert_cs(inst->spi_dev);
	spi_transfer(inst->spi_dev, tx_buf, rx_buf, 4);
	spi_deassert_cs(inst->spi_dev);

	return rx_buf[3];
}

/* Writes an 8 bit register. */
static void reg_write_8(struct w5500_device *inst,
			enum w5500_block_select block_select,
			uint16_t offset,
			uint8_t val)
{
	uint8_t tx_buf[4];
	uint8_t rx_buf[4];

	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = offset >> 8u;
	tx_buf[1] = offset;
	tx_buf[2] = block_select | W5500_CONTROL_RW_WRITE;
	tx_buf[3] = val;

	spi_assert_cs(inst->spi_dev);
	spi_transfer(inst->spi_dev, tx_buf, rx_buf, 4);
	spi_deassert_cs(inst->spi_dev);
}

/* Read a 16 bit register. */
static uint16_t reg_read_16(struct w5500_device *inst,
			    enum w5500_block_select block_select,
			    uint16_t offset)
{
	uint8_t tx_buf[5];
	uint8_t rx_buf[5];
	uint16_t ret;

	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = offset >> 8u;
	tx_buf[1] = offset;
	tx_buf[2] = block_select;
	tx_buf[3] = 0;
	tx_buf[4] = 0;

	spi_assert_cs(inst->spi_dev);
	spi_transfer(inst->spi_dev, tx_buf, rx_buf, 5);
	spi_deassert_cs(inst->spi_dev);

	ret = rx_buf[3];
	ret <<= 8u;
	ret |= rx_buf[4];

	return ret;
}

/* Writes a 16 bit register. */
static void reg_write_16(struct w5500_device *inst,
			 enum w5500_block_select block_select,
			 uint16_t offset,
			 uint16_t val)
{
	uint8_t tx_buf[5];
	uint8_t rx_buf[5];

	memset(rx_buf, 0, sizeof(rx_buf));

	tx_buf[0] = offset >> 8u;
	tx_buf[1] = offset;
	tx_buf[2] = block_select | W5500_CONTROL_RW_WRITE;
	tx_buf[3] = val >> 8u;
	tx_buf[4] = val;

	spi_assert_cs(inst->spi_dev);
	spi_transfer(inst->spi_dev, tx_buf, rx_buf, 5);
	spi_deassert_cs(inst->spi_dev);

	return;
}

/* Returns the value of the scratch register (uses the gateway IP regs). */
static uint32_t scratch_get(struct w5500_device *inst)
{
	uint32_t ret;

	ret = reg_read_8(inst, W5500_BLOCK_SELECT_COMMON, W5500_COMMON_REG_GWIP_0);
	ret <<= 8u;
	ret |= reg_read_8(inst, W5500_BLOCK_SELECT_COMMON, W5500_COMMON_REG_GWIP_1);
	ret <<= 8u;
	ret |= reg_read_8(inst, W5500_BLOCK_SELECT_COMMON, W5500_COMMON_REG_GWIP_2);
	ret <<= 8u;
	ret |= reg_read_8(inst, W5500_BLOCK_SELECT_COMMON, W5500_COMMON_REG_GWIP_3);

	return ret;
}

/* Sets the value of the scratch register. */
static void scratch_set(struct w5500_device *inst, uint32_t val)
{
	reg_write_8(inst, W5500_BLOCK_SELECT_COMMON,  W5500_COMMON_REG_GWIP_3, val);
	val >>= 8u;
	reg_write_8(inst, W5500_BLOCK_SELECT_COMMON,  W5500_COMMON_REG_GWIP_2, val);
	val >>= 8u;
	reg_write_8(inst, W5500_BLOCK_SELECT_COMMON,  W5500_COMMON_REG_GWIP_1, val);
	val >>= 8u;
	reg_write_8(inst, W5500_BLOCK_SELECT_COMMON,  W5500_COMMON_REG_GWIP_0, val);
}

int w5500_init(struct w5500_device *inst, struct spi_device *spidev)
{
	inst->spi_dev = spidev;
	uint32_t timeout;
	uint32_t tmp;
	uint8_t socket_reg;
	size_t i;

	/* The Arduino Ethernet Shield 2 has a CAT811 chip that controls the reset to
	 * the W5500. This generates a reset pulse that can range from 140 ms to 400 ms,
	 * so wait 500 milliseconds before attempting to communicate with the chip.
	 */
	platform_mdelay(500);

	timeout = W5500_RESET_TIMEOUT;
	reg_write_8(inst, W5500_BLOCK_SELECT_COMMON, W5500_COMMON_REG_MODE, W5500_COMMON_REG_MODE_RST);
	while (timeout) {
		platform_mdelay(1);
		timeout--;
		if (!(reg_read_8(inst,
				 W5500_BLOCK_SELECT_COMMON,
				 W5500_COMMON_REG_MODE) & W5500_COMMON_REG_MODE_RST)) {
			break;
		}
	}
	if (!timeout) {
		return -1;
	}

	/* There might be a bug in the silicon. Sometimes after a reset, it's impossible
	 * to write to any registers (the write simply has no effect). Reads work fine
	 * during this period though. If I insert a long delay after a reset, then it works.
	 * You'd think that waiting for the reset bit to clear itself would be enough
	 * but apparently not...
	 * So, do a test of writing to some scratch register to determine if the device
	 * is ready to accept writes yet.
	 */
	tmp = scratch_get(inst);

	timeout = W5500_FIRST_WRITE_TIMEOUT;
	while (timeout) {
		scratch_set(inst, 0xABAB55AA);
		if (scratch_get(inst) == 0xABAB55AA) {
			break;
		}

		platform_mdelay(1);
		timeout--;
	}
	if (!timeout) {
		return -1;
	}

	scratch_set(inst, tmp);

	/* Reserve all space for socket 0. To save space in the bootloader, loop
	 * starting from the socket 1 base, adding 0x20 each time.
	 */
	socket_reg = W5500_BLOCK_SELECT_SOCKET_1_REG;
	for (i = 0; i < 7; i++) {
		reg_write_8(inst, socket_reg, W5500_SOCKET_REG_RX_BUF_SIZE, 0);
		reg_write_8(inst, socket_reg, W5500_SOCKET_REG_TX_BUF_SIZE, 0);
		socket_reg += 0x20;
	}

	/* 16 KiB for RX and TX (32 KiB total). */
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_BUF_SIZE, 16);
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_TX_BUF_SIZE, 16);

	/* Set to MACRAW mode. */
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_MODE, W5500_SOCKET_REG_MODE_MACRAW);

	/* Open. */
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_COMMAND, W5500_SOCKET_REG_COMMAND_OPEN);

	timeout = W5500_SOCKET_OPEN_TIMEOUT;
	while (timeout) {
		platform_mdelay(1);
		timeout--;
		if (reg_read_8(inst,
			       W5500_BLOCK_SELECT_SOCKET_0_REG,
			       W5500_SOCKET_REG_STATUS) == W5500_SOCKET_REG_STATUS_SOCK_MACRAW) {
			break;
		}
	}
	if (!timeout) {
		return -1;
	}

	/* TODO - Register ops with Ethernet core. */

	return 0;
}

int w5500_drop_frame(struct w5500_device *inst)
{
	uint16_t read_idx;
	uint16_t buffer_header;
	uint16_t total_data_ready;

	total_data_ready  = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RCVD_DATA_SIZE_UPPER);
	if (!total_data_ready) {
		return -1;
	}

	read_idx = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_READ_IDX_UPPER);
	buffer_header = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_RX_BUF, read_idx);
	read_idx += buffer_header;
	reg_write_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_READ_IDX_UPPER, read_idx);
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_COMMAND, W5500_SOCKET_REG_COMMAND_RECV);

	return 0;
}

uint16_t w5500_next_frame_bytes_avail(struct w5500_device *inst)
{
	uint16_t read_idx;
	uint16_t buffer_header;
	uint16_t total_data_ready;

	total_data_ready = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RCVD_DATA_SIZE_UPPER);
	if (!total_data_ready) {
		return 0;
	}

	/* There's at least one frame. */
	read_idx = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_READ_IDX_UPPER);
	buffer_header = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_RX_BUF, read_idx);

	/* It's impossible for buffer_header to be less than 2 since it includes the size of itself... */

	return (buffer_header - 2u);
}

int16_t w5500_receive_frame(struct w5500_device *inst,
			    uint8_t *rx_buf,
			    size_t maxlen)
{
	uint16_t read_idx;
	uint16_t buffer_header;
	uint16_t payload_addr;
	uint8_t tx_buf[3];
	uint8_t tmp_rx_buf[3];
	size_t i;
	uint16_t total_data_ready = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RCVD_DATA_SIZE_UPPER);

	if (!total_data_ready) {
		return 0;
	}

	/* There's data, but it could be multiple frames. */
	read_idx = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_READ_IDX_UPPER);
	buffer_header = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_RX_BUF, read_idx);

	/* The first two bytes are a descriptor that contain the length of the frame,
         * including the length of the descriptor itself.
	 */
	if ((buffer_header - 2u) > maxlen) {
		return -1;
	}

	/* TODO - This is kind of ugly.
         * The SPI interface should be extended to add a new transfer method that allows
	 * for separate command and RX buffers, with a specified command length. This would
	 * allow the user's buffer to be passed straight through without having to
	 * issue individual 1 byte transfers as is done now...
	 */
	payload_addr = read_idx + 2;

	tx_buf[0] = payload_addr >> 8u;
	tx_buf[1] = payload_addr;
	tx_buf[2] = W5500_BLOCK_SELECT_SOCKET_0_RX_BUF;

	spi_assert_cs(inst->spi_dev);

	/* Send the command. */
	spi_transfer(inst->spi_dev, tx_buf, tmp_rx_buf, 3);

	/* Shift out the frame bytes one by one. */
	tx_buf[0] = 0;
	for (i = 0; i < (buffer_header - 2u); i++) {
		spi_transfer(inst->spi_dev, tx_buf, &rx_buf[i], 1);
	}

	spi_deassert_cs(inst->spi_dev);

	read_idx += buffer_header;
	reg_write_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_RX_READ_IDX_UPPER, read_idx);

	/* Send recv command to atomically acknowledge advance read pointer. */
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_COMMAND, W5500_SOCKET_REG_COMMAND_RECV);

	return (buffer_header - 2u);
}

void w5500_send_frame(struct w5500_device *inst,
		      uint8_t *tx_buf,
		      size_t len)
{
	uint16_t tx_space;
	uint16_t write_idx;
	uint8_t tmp;
	uint8_t rx_buf;
	size_t i;

	/* Not sure if this chip supports jumbo frames or not but I doubt it. */
	if (len > 1514) {
		return;
	}

	/* Wait for there to be space.  */
	do {
		tx_space = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_TX_FREE_SIZE_UPPER);
	} while (tx_space < len);

	write_idx = reg_read_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_TX_WRITE_IDX_UPPER);

	spi_assert_cs(inst->spi_dev);

	tmp = write_idx >> 8u;
	spi_transfer(inst->spi_dev, &tmp, &rx_buf, 1);
	tmp = write_idx;
	spi_transfer(inst->spi_dev, &tmp, &rx_buf, 1);
	tmp = W5500_BLOCK_SELECT_SOCKET_0_TX_BUF | W5500_CONTROL_RW_WRITE;
	spi_transfer(inst->spi_dev, &tmp, &rx_buf, 1);

	for (i = 0; i < len; i++) {
		spi_transfer(inst->spi_dev, tx_buf + i, &rx_buf, 1);
	}

	spi_deassert_cs(inst->spi_dev);

	write_idx += len;
	reg_write_16(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_TX_WRITE_IDX_UPPER, write_idx);
	reg_write_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_COMMAND, W5500_SOCKET_REG_COMMAND_SEND);

	/* Wait for transmit to complete. */
	while (1) {
		tmp = reg_read_8(inst, W5500_BLOCK_SELECT_SOCKET_0_REG, W5500_SOCKET_REG_INTERRUPT);
		if (tmp & W5500_SOCKET_REG_INTERRUPT_SENDOK) {
			reg_write_8(inst,
				    W5500_BLOCK_SELECT_SOCKET_0_REG,
				    W5500_SOCKET_REG_INTERRUPT,
				    W5500_SOCKET_REG_INTERRUPT_SENDOK);
			break;
		}
	}
}
