/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * An example Micro Protocol application.
 *
 * Supports stdout redirection over Micro Protocol.
 */

#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>
#include "platform.h"
#include "spi.h"
#include "w5500.h"
#include "micro_protocol.h"

/* Application firmware version. */
#define FW_VER_MAJOR                   1
#define FW_VER_MINOR                   1

/* If the bootloader finds these values at address 0x200 in SRAM, then
 * it indicates that the next byte contains a reset command/cause.
 * These values are optionally set by the application before allowing
 * the WDT to reset the device. This works because the SRAM contents
 * are preserved on a WDT reset on AVRs.
 */
#define RESET_MAGIC_SRAM_BASE          0x200
#define RESET_MAGIC_SIZE               8u
#define RESET_MAGIC_BYTES              { 0xD5, 0xF8, 0x31, 0x8C, 0xBB, 0xDD, 0xF6, 0xF7 }

/* Command to cause the bootloader to wait an extra 30 seconds for an
 * update request. This is used in cases where the application knows
 * that the Ethernet link may not be available immediately after
 * power up for some reason.
 */
#define RESET_COMMAND_WAIT_FOR_UPD     0x01

/* Largest frame that we can process.
 * We can't process even a full MTU since there's only 2 KiB of SRAM...
 */
#define ETHERNET_RX_MAX_SIZE           256u

/* Minimum Ethernet frame size (64 bytes including FCS). */
#define ETHERNET_RX_MIN_SIZE           60u

/* Maximum number of stdout bytes that can accumulated.
 * NOTE: Any bigger than 128 and the index type size must be increased.
 */
#define STDOUT_BUFFER_SIZE             128u
#define STDOUT_BUFFER_SIZE_MASK        (STDOUT_BUFFER_SIZE - 1u)

/* Micro Protocol App. */
struct mp_app {
	struct w5500_device *eth;
	uint8_t mac[ETHERNET_MAC_LEN];
	uint64_t uptime_seconds;
	uint8_t stdout_buffer[STDOUT_BUFFER_SIZE];
	uint8_t stdout_write_idx;
	uint8_t stdout_read_idx;

	struct {
		uint32_t rx_too_big_frames;
		uint32_t rx_runt_frames;
		uint32_t rx_good_frames;
		uint32_t rx_mp_frames;
		uint32_t rx_mp_frames_ignored;
		uint32_t tx_frames;
	} stats;
} app;

/* All possible RX messages should be added to this union to
 * avoid illegal C type punning.
 * In reality, we could just add -fno-strict-aliasing and
 * cast away, but this is just an example.
 */
union eth_rx_msg {
	union mp_msg mp_msg;
	uint8_t raw[ETHERNET_RX_MAX_SIZE];
};

/* Software initiated reset. */
static void reset(void)
{
	wdt_enable(WDTO_15MS);

	/* Wait for watchdog to reset us. */
	while (1);
}

/* Software initiated reset, with an update command to tell the
 * bootloader to wait for 30 additional seconds before booting
 * the application.
 */
static void reset_update(void)
{
	uint8_t *sram_base = (uint8_t *)RESET_MAGIC_SRAM_BASE;
	const uint8_t magic[RESET_MAGIC_SIZE] = RESET_MAGIC_BYTES;

	memcpy(sram_base, magic, RESET_MAGIC_SIZE);

	sram_base[RESET_MAGIC_SIZE] = RESET_COMMAND_WAIT_FOR_UPD;

	wdt_enable(WDTO_15MS);

	/* Wait for watchdog to reset us. */
	while (1);
}

/* Transmits an IDENTIFY_NOTICE message. */
static void mp_send_identify_notice(struct mp_app *inst,
				    uint8_t seq_nr,
				    uint8_t *dst_mac)
{
	union mp_msg tx_msg;

	memset(&tx_msg, 0, sizeof(tx_msg));

	/* Header. */
	memcpy(tx_msg.packet.header.dst_mac, dst_mac, ETHERNET_MAC_LEN);
	memcpy(tx_msg.packet.header.src_mac, inst->mac, ETHERNET_MAC_LEN);
	tx_msg.packet.header.ethertype = cpu_to_be16(MP_ETHERTYPE);
	tx_msg.packet.header.magic = cpu_to_be32(MP_MAGIC_NUMBER);
	tx_msg.packet.header.version = MP_VERSION;
	tx_msg.packet.header.state = MP_SYSTEM_STATE_RUNNING;
	tx_msg.packet.header.msg_type = MP_MSG_TYPE_IDENTIFY_NOTICE;

	/* Payload. */
	tx_msg.packet.payload.identify_notice.sequence_nr = seq_nr;
	tx_msg.packet.payload.identify_notice.platform_type = MP_PLATFORM_TYPE_UNO_W5500;
	tx_msg.packet.payload.identify_notice.system_type = MP_SYSTEM_TYPE_GENERAL_PURPOSE;
	tx_msg.packet.payload.identify_notice.uptime_seconds = inst->uptime_seconds;
	tx_msg.packet.payload.identify_notice.fw_ver_major = FW_VER_MAJOR;
	tx_msg.packet.payload.identify_notice.fw_ver_minor = FW_VER_MINOR;
	tx_msg.packet.payload.identify_notice.fw_image_size = cpu_to_be32(FIRMWARE_IMAGE_SIZE);
	tx_msg.packet.payload.identify_notice.fw_crc_bad = 0;

	inst->stats.tx_frames++;
	w5500_send_frame(inst->eth, tx_msg.raw, sizeof(tx_msg.raw));
}

/* Handles a Micro Protocol message. */
static void mp_message_process(struct mp_app *inst, union mp_msg *msg)
{
	size_t i;

	if (msg->packet.header.msg_type == MP_MSG_TYPE_IDENTIFY_QUERY) {
		/* Respond with a notice. */
		mp_send_identify_notice(inst,
					msg->packet.payload.identify_query.sequence_nr,
					msg->packet.header.src_mac);
		return;
	}

	if (msg->packet.header.msg_type == MP_MSG_TYPE_REBOOT_REQUEST) {
		for (i = 0; i < ETHERNET_MAC_LEN; i++) {
			if (msg->packet.header.dst_mac[i] != inst->mac[i]) {
				/* Not for us. */
				return;
			}
		}
		reset();
		/* NO RETURN */
	}

	if (msg->packet.header.msg_type == MP_MSG_TYPE_UPDATE_REQUEST) {
		for (i = 0; i < ETHERNET_MAC_LEN; i++) {
			if (msg->packet.header.dst_mac[i] != inst->mac[i]) {
				/* Not for us. */
				return;
			}
		}
		reset_update();
		/* NO RETURN */
	}

	inst->stats.rx_mp_frames_ignored++;
}

/* Handles incoming Ethernet frames. */
static void eth_rx_process(struct mp_app *inst, union eth_rx_msg *rx_msg, size_t len)
{
	if (len < ETHERNET_RX_MIN_SIZE) {
		inst->stats.rx_runt_frames++;
		return;
	}

	inst->stats.rx_good_frames++;

	if (len >= MP_MSG_SIZE) {
		if (mp_msg_valid(rx_msg->raw)) {
			inst->stats.rx_mp_frames++;
			/* Handle Micro Protocol message. */
			mp_message_process(inst, &rx_msg->mp_msg);
		}
	}
}

/* Transmits CONSOLE messages until the stdout buffer is empty. */
static void drain_stdout_buffer(struct mp_app *inst)
{
	union mp_msg tx_msg;
	uint8_t to_send;
	uint8_t bytes_waiting;
	size_t i;

	if (inst->stdout_read_idx == inst->stdout_write_idx) {
		return;
	}

	memset(&tx_msg, 0, sizeof(tx_msg));

	/* Header. */
	memset(tx_msg.packet.header.dst_mac, 0xFF, ETHERNET_MAC_LEN);
	memcpy(tx_msg.packet.header.src_mac, inst->mac, ETHERNET_MAC_LEN);
	tx_msg.packet.header.ethertype = cpu_to_be16(MP_ETHERTYPE);
	tx_msg.packet.header.magic = cpu_to_be32(MP_MAGIC_NUMBER);
	tx_msg.packet.header.version = MP_VERSION;
	tx_msg.packet.header.state = MP_SYSTEM_STATE_RUNNING;
	tx_msg.packet.header.msg_type = MP_MSG_TYPE_CONSOLE;

	/* While there is data to send... */
	while (inst->stdout_read_idx != inst->stdout_write_idx) {
		bytes_waiting = inst->stdout_write_idx - inst->stdout_read_idx;
		to_send = (bytes_waiting > MP_MSG_CONSOLE_MAX_LEN)? MP_MSG_CONSOLE_MAX_LEN : bytes_waiting;

		tx_msg.packet.payload.console.len = to_send;
		for (i = 0; i < to_send; i++) {
			tx_msg.packet.payload.console.data[i] =
				inst->stdout_buffer[inst->stdout_read_idx & STDOUT_BUFFER_SIZE_MASK];
			inst->stdout_read_idx++;
		}

		inst->stats.tx_frames++;
		w5500_send_frame(inst->eth, tx_msg.raw, sizeof(tx_msg.raw));
	}
}

/* Initializes a Micro Protocol app instance. */
static void mp_app_init(struct mp_app *inst, struct w5500_device *eth)
{
	memset(inst, 0, sizeof(struct mp_app));

	inst->eth = eth;

	platform_get_mac(inst->mac);
	if (inst->mac[0] & 0x01) {
		/* Invalid MAC. Can't have the multicsat bit set, so it's
		 * probably just a blank EEPROM.
		 * Use a random MAC for now.
		 */
		inst->mac[0] = 0x02;
		inst->mac[1] = 0x00;
		inst->mac[2] = 0x00;
		inst->mac[3] = 0x01;
		inst->mac[4] = 0x02;
		inst->mac[5] = 0x03;
	}
}

/* stdout output handler. */
static int stdout_put_char(char c, FILE *stream)
{
	struct mp_app *inst = &app;

	/* There must be at least 2 bytes free in the buffer because we
	 * may end up inserting a carriage return when a newline char arrives.
	 */
	if ((inst->stdout_write_idx - inst->stdout_read_idx) > (STDOUT_BUFFER_SIZE - 2u)) {
		/* Full. Force a flush. */
		drain_stdout_buffer(inst);
	}

	if (c == '\n') {
		inst->stdout_buffer[inst->stdout_write_idx & STDOUT_BUFFER_SIZE_MASK] = '\r';
		inst->stdout_write_idx++;
	}

	inst->stdout_buffer[inst->stdout_write_idx & STDOUT_BUFFER_SIZE_MASK] = c;
	inst->stdout_write_idx++;

	return 0;
}

FILE stdout_stream = FDEV_SETUP_STREAM(stdout_put_char, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
	struct mp_app *inst = &app;
	struct spi_device w5500_spi_dev;
	struct w5500_device w5500_dev;
	union eth_rx_msg rx_msg;
	uint8_t ident_dst[ETHERNET_MAC_LEN];
	int16_t ret;

	/* The SPI bus driver was already initialized by the bootloader, but do it again. */
	spi_bus_init();

	/* For now, these fields aren't used since we just copied the bootloader's
	 * simplified SPI bus driver.
	 */
#if 0
	w5500_spi_dev.bus_num = SPI_BUS_NUM_0;
	w5500_spi_dev.mode = SPI_BUS_MODE_0;
	w5500_spi_dev.cs_pin = SPI_BUS_CS_PIN_0;
	w5500_spi_dev.freq_hz = -1;
#endif

	if (w5500_init(&w5500_dev, &w5500_spi_dev)) {
		/* Could not initialize the Ethernet device.
		 * Toggle the LED on pin 13 (B5), which is also the SPI SCK and spin forever.
		 */
		spi_bus_exit();
		DDRB |= (1u << PINB5);
		while (1) {
			/* ~1 Hz toggle. */
			PORTB |= (1u << PINB5);
			platform_mdelay(500);
			PORTB &= ~(1u << PINB5);
			platform_mdelay(500);
		}
	}

	mp_app_init(inst, &w5500_dev);

	/* Send an identify notice message upon bootup. */
	memset(ident_dst, 0xFF, sizeof(ident_dst));
	mp_send_identify_notice(inst, 0, ident_dst);

	/* Redirect stdout over Ethernet via MP CONSOLE messages. */
	stdout = &stdout_stream;

	/* Other init code goes here... */

	/* Handle incoming Ethernet frames. */
	while (1) {
		ret = w5500_receive_frame(&w5500_dev, rx_msg.raw, sizeof(rx_msg.raw));
		if (ret == -1) {
			/* Frame was too big to handle. Drop it. */
			w5500_drop_frame(&w5500_dev);
			inst->stats.rx_too_big_frames++;
		} else if (ret) {
			/* Frame was between 1 and sizeof(rx_buf) bytes. */
			eth_rx_process(inst, &rx_msg, ret);
		}

		drain_stdout_buffer(inst);

		/* Other exec code goes here... */

		/* Keep dumping the stats. */
		printf("\n");
		printf("RX Good Frames:\t\t%"PRIu32"\n", inst->stats.rx_good_frames);
		printf("RX Too Big Frames:\t%"PRIu32"\n", inst->stats.rx_too_big_frames);
		printf("RX Runt Frames:\t\t%"PRIu32"\n", inst->stats.rx_runt_frames);
		printf("RX MP Frames:\t\t%"PRIu32"\n", inst->stats.rx_mp_frames);
		printf("RX MP Frames Ignored:\t%"PRIu32"\n", inst->stats.rx_mp_frames_ignored);
		printf("TX Frames:\t\t%"PRIu32"\n", inst->stats.tx_frames);
		printf("\n");
	}
}
