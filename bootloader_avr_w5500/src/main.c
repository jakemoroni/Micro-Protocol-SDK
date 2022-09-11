/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Main C entry into the bootloader. Since the bootloader
 * is free standing and there's no C library or startup files,
 * the data and BSS initialization must be handled here.
 *
 * Implements Micro Protocol.
 */

#include <stdint.h> /* This can be used without the standard library. */
#include "platform.h"
#include "spi.h"
#include "w5500.h"
#include "flash.h"
#include "timer.h"
#include "micro_protocol.h"

/* Bootloader firmware version. */
#define FW_VER_MAJOR                   1
#define FW_VER_MINOR                   0

#define BOOT_TIMEOUT_SECONDS           5 /* It takes ~3 seconds for the link to come up. */
#define FIRMWARE_IMAGE_SIZE            28672u /* 32 KiB - 4 KiB bootlodaer */

typedef void (*firmware_entry)(void);

/* Symbols defined in the linker script. */
extern unsigned int *__data_flash_start;  /* Data section start address in flash. */
extern unsigned int *__data_flash_end;    /* Data section end address in flash. */
extern unsigned int *__data_sram_start;   /* Data section start address in SRAM. */
extern unsigned int *__bss_sram_start;    /* BSS section start address in SRAM. */
extern unsigned int *__bss_sram_end;      /* BSS section end address in SRAM. */
extern unsigned int *__BOOTLOADER_SIZE__; /* Maximum bootloader size. */

/* Example. */
/* const uintptr_t bootloadersize = (uintptr_t)&__BOOTLOADER_SIZE__; */

/* Internal Micro Protocol system context. */
struct mp_system_context {
	uint8_t mac[ETHERNET_MAC_LEN];
	union mp_msg tx_ident_notice_msg;
	uint8_t tx_ident_notice_msg_seq_nr;
	enum mp_system_state state;

	/* Extended state variables. */
	uint8_t boot_timeout;

	/* Transfer context. */
	struct {
		uint8_t upgrader_host_mac[ETHERNET_MAC_LEN];
		uint8_t upgrade_transfer_seq_num;
		uint16_t upgrade_timer_start;
		uint16_t total_bytes_received;
		uint16_t page_bytes_received;
		uint16_t last_page_programmed_addr;
		uint8_t page[FLASH_PAGE_SIZE];
	} transfer;
};

/* Provide a global memset routine. It's used mainly by the w5500 driver. */
void *memset(void *str, int c, size_t n)
{
	size_t i;
	char *buf = str;

	for (i = 0; i < n; i++) {
		buf[i] = c;
	}

	return str;
}

/* Initialize the .data section SRAM contents. */
static void init_data(void)
{
	const uintptr_t flash_start = (uintptr_t)&__data_flash_start;
	const uintptr_t flash_end = (uintptr_t)&__data_flash_end;
	const uintptr_t data_section_len = flash_end - flash_start;
	uint8_t *data_start = (uint8_t *)&__data_sram_start;
	uintptr_t i;

	for (i = 0; i < data_section_len; i++) {
		data_start[i] = pgm_read_byte(flash_start + i);
	}
}

/* Zero out the .bss section. */
static void init_bss(void)
{
	uint8_t *bss_start = (uint8_t *)&__bss_sram_start;
	uint8_t *bss_end = (uint8_t *)&__bss_sram_end;
	const uintptr_t bss_len = bss_end - bss_start;
	uintptr_t i;

	for (i = 0; i < bss_len; i++) {
		bss_start[i] = 0;
	}
}

/* Simple memcpy since there's no libc. */
static void bl_memcpy(uint8_t *dst, uint8_t *src, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		dst[i] = src[i];
	}
}

/* Recalculate the CRC-32 for another byte. */
static uint32_t crc32_update_byte(uint32_t crc_in, uint8_t byte)
{
	size_t i;
	uint32_t crc = ~crc_in;

	crc = crc ^ byte;
	for (i = 0; i < 8; i++) {
		if (crc & 1u) {
			crc = (crc >> 1u) ^ 0xEDB88320; /* Use Ethernet polynomial, reversed. */
		} else {
			crc >>= 1u;
		}
	}

	return ~crc;
}

/* Returns true if the firmware has a valid CRC. */
static bool check_firmware_crc(void)
{
	uint32_t i;
	uint32_t calculated_crc;
	uint32_t stored_crc;

	calculated_crc = 0;
	for (i = 0; i < FIRMWARE_IMAGE_SIZE - 4u; i++) {
		calculated_crc = crc32_update_byte(calculated_crc, pgm_read_byte(i));
	}

	/* CRC is stored as big endian. */
	stored_crc = pgm_read_byte(FIRMWARE_IMAGE_SIZE - 4u);
	stored_crc <<= 8u;
	stored_crc |= pgm_read_byte(FIRMWARE_IMAGE_SIZE - 3u);
	stored_crc <<= 8u;
	stored_crc |= pgm_read_byte(FIRMWARE_IMAGE_SIZE - 2u);
	stored_crc <<= 8u;
	stored_crc |= pgm_read_byte(FIRMWARE_IMAGE_SIZE - 1u);

	if (calculated_crc != stored_crc) {
		return false;
	}

	return true;
}

/* Initializes the message header for messages while in the
 * updating state.
 */
static void mp_transfer_msg_header_init(struct mp_system_context *inst,
					union mp_msg *tx_msg,
					enum mp_msg_type msg_type)
{
	bl_memcpy(tx_msg->packet.header.dst_mac, inst->transfer.upgrader_host_mac, ETHERNET_MAC_LEN);
	bl_memcpy(tx_msg->packet.header.src_mac, inst->mac, ETHERNET_MAC_LEN);
	tx_msg->packet.header.ethertype = cpu_to_be16(MP_ETHERTYPE);
	tx_msg->packet.header.magic = cpu_to_be32(MP_MAGIC_NUMBER);
	tx_msg->packet.header.version = MP_VERSION;
	tx_msg->packet.header.state = inst->state;
	tx_msg->packet.header.msg_type = msg_type;
}

/* Transmits a TRANSFER_BEGIN_ACK message. */
static void mp_send_transfer_begin_ack(struct mp_system_context *inst,
				       struct w5500_device *eth)
{
	union mp_msg tx_msg;

	memset(&tx_msg, 0, sizeof(tx_msg));

	mp_transfer_msg_header_init(inst, &tx_msg, MP_MSG_TYPE_TRANSFER_BEGIN_ACK);

	w5500_send_frame(eth, tx_msg.raw, sizeof(tx_msg.raw));
}

/* Transmits a TRANSFER_ACK message. */
static void mp_send_transfer_ack(struct mp_system_context *inst,
				 struct w5500_device *eth,
				 uint8_t sequence_nr)
{
	union mp_msg tx_msg;

	memset(&tx_msg, 0, sizeof(tx_msg));

	mp_transfer_msg_header_init(inst, &tx_msg, MP_MSG_TYPE_TRANSFER_ACK);

	tx_msg.packet.payload.transfer_ack.sequence_nr = sequence_nr;

	w5500_send_frame(eth, tx_msg.raw, sizeof(tx_msg.raw));
}

/* Transmits a TRANSFER_ABORT message. */
static void mp_send_transfer_abort(struct mp_system_context *inst,
				   struct w5500_device *eth,
				   enum mp_transfer_abort_reason reason)
{
	union mp_msg tx_msg;

	memset(&tx_msg, 0, sizeof(tx_msg));

	mp_transfer_msg_header_init(inst, &tx_msg, MP_MSG_TYPE_TRANSFER_ABORT);

	tx_msg.packet.payload.transfer_abort.reason = reason;

	w5500_send_frame(eth, tx_msg.raw, sizeof(tx_msg.raw));
}

/* Processes a received Micro Protocol message. Returns true if there
 * has been a state change.
 */
static bool mp_fsm_message_process(struct mp_system_context *inst,
				   struct w5500_device *eth,
				   union mp_msg *rx_msg)
{
	size_t i;

	if (!mp_msg_valid(rx_msg->raw)) {
		return false;
	}

	for (i = 0; i < ETHERNET_MAC_LEN; i++) {
		if (rx_msg->packet.header.dst_mac[i] != inst->mac[i]) {
			/* Not for us. */
			return false;
		}
	}

	if (inst->state == MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN) {
		/* The only message we care about in the bootloader is TRANSFER_BEGIN. */
		if (rx_msg->packet.header.msg_type != MP_MSG_TYPE_TRANSFER_BEGIN) {
			return false;
		}

		if (be32_to_cpu(rx_msg->packet.payload.transfer_begin.file_size) != FIRMWARE_IMAGE_SIZE) {
			/* The host is trying to send us a file that doesn't match the
			 * size of our firmware image. They should be able to figure that
			 * out since we send the desired size in the IDENTIFY_NOTICE...
			 */
			return false;
		}

		/* Update internal state. */
		inst->state = MP_SYSTEM_STATE_UPDATE_IN_PROGRESS;

		bl_memcpy(inst->transfer.upgrader_host_mac, rx_msg->packet.header.src_mac, ETHERNET_MAC_LEN);
		inst->transfer.upgrade_transfer_seq_num = 0;
		inst->transfer.total_bytes_received = 0;
		inst->transfer.page_bytes_received = 0;
		inst->transfer.last_page_programmed_addr = 0;

		mp_send_transfer_begin_ack(inst, eth);

		inst->transfer.upgrade_timer_start = timer_ticks_get();

		return true;
	} else if (inst->state == MP_SYSTEM_STATE_UPDATE_IN_PROGRESS) {
		if (rx_msg->packet.header.msg_type != MP_MSG_TYPE_TRANSFER) {
			return false;
		}

		for (i = 0; i < ETHERNET_MAC_LEN; i++) {
			if (rx_msg->packet.header.src_mac[i] != inst->transfer.upgrader_host_mac[i]) {
				/* Not from our upgrade host. */
				return false;
			}
		}

		if (rx_msg->packet.payload.transfer.sequence_nr == (inst->transfer.upgrade_transfer_seq_num - 1u)) {
			/* This sequence number was already received. Our ACK must have gotten dropped.
			 * Send another ACK (don't reset the upgrade timer).
			 */
			mp_send_transfer_ack(inst, eth, rx_msg->packet.payload.transfer.sequence_nr);
		} else if (rx_msg->packet.payload.transfer.sequence_nr == inst->transfer.upgrade_transfer_seq_num) {
			/* Good data. Accumulate a page then flash. */
			for (i = 0; i < rx_msg->packet.payload.transfer.len; i++) {
				if (inst->transfer.total_bytes_received == FIRMWARE_IMAGE_SIZE) {
					/* The host is trying to send more data than expected.
					 * Allow the transfer to time out.
					 * This check is done here so that the outer loop will see a full
					 * transfer and not transmit a timeout message.
					 */
					mp_send_transfer_abort(inst, eth, MP_TRANSFER_ABORT_REASON_TOO_LARGE);
					return false;
				}

				inst->transfer.page[inst->transfer.page_bytes_received] = rx_msg->packet.payload.transfer.data[i];
				inst->transfer.page_bytes_received++;
				inst->transfer.total_bytes_received++;

				if (inst->transfer.page_bytes_received == FLASH_PAGE_SIZE) {
					flash_page_program(inst->transfer.last_page_programmed_addr, inst->transfer.page);
					inst->transfer.last_page_programmed_addr += FLASH_PAGE_SIZE;
					inst->transfer.page_bytes_received = 0;
				}
			}

			mp_send_transfer_ack(inst, eth, rx_msg->packet.payload.transfer.sequence_nr);

			if (inst->transfer.total_bytes_received == FIRMWARE_IMAGE_SIZE) {
				/* Done. Return true to cause the outer loop to bail early. */
				return true;
			}

			inst->transfer.upgrade_transfer_seq_num++;
			inst->transfer.upgrade_timer_start = timer_ticks_get();
		} else {
			/* Invalid sequence number. Nothing to do other than to wait. */
		}
	}

	return false;
}

/* Micro Protocol FSM exec. */
static void mp_fsm_run(struct mp_system_context *inst, struct w5500_device *eth)
{
	union mp_msg rx_msg;
	uint16_t timer_val;
	firmware_entry fw = (void *)0;
	int tmp;

	if (inst->state == MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN) {
		if (!inst->boot_timeout) {
			/* Try to boot. */
			if (check_firmware_crc()) {
				fw();
			} else {
				/* The firmware image has a bad CRC. Just stay in the bootloader
				 * forever and wait for an update.
				 */
				inst->state = MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN;
				inst->boot_timeout = -1;
				inst->tx_ident_notice_msg.packet.payload.identify_notice.fw_crc_bad = 1;
			}
		}

		/* Send IDENTIFY_NOTICE. */
		w5500_send_frame(eth, inst->tx_ident_notice_msg.raw, sizeof(inst->tx_ident_notice_msg.raw));
		inst->tx_ident_notice_msg.packet.payload.identify_notice.sequence_nr++;

		/* Check for responses for one second. TODO - Check to see how the W5500 handles
		 * overflow on RX. This sort of relies on it tail dropping and not pushing
		 * out old received frames.
		 */
		timer_val = timer_ticks_get();
		while ((timer_ticks_get() - timer_val) < TIMER_TICKS_PER_SEC) {
			tmp = w5500_receive_frame(eth, rx_msg.raw, sizeof(rx_msg.raw));
			if (tmp == 0) {
				continue;
			} else if ((tmp == -1) || (tmp < sizeof(rx_msg.raw))) {
				/* The explicit check for -1 is there just for documentation purposes. */
				w5500_drop_frame(eth);
				continue;
			}

			if (mp_fsm_message_process(inst, eth, &rx_msg)) {
				/* A state change occured. Break out of the RX loop early. */
				break;
			}
		}
		inst->boot_timeout--;
	} else if (inst->state == MP_SYSTEM_STATE_UPDATE_IN_PROGRESS) {
		/* Wait up to one second for a new message from the upgrade host. */
		while ((timer_ticks_get() - inst->transfer.upgrade_timer_start) < TIMER_TICKS_PER_SEC) {
			tmp = w5500_receive_frame(eth, rx_msg.raw, sizeof(rx_msg.raw));
			if (tmp == 0) {
				continue;
			} else if ((tmp == -1) || (tmp < sizeof(rx_msg.raw))) {
				w5500_drop_frame(eth);
				continue;
			}

			/* While in the upgrading state, mp_message_process keeps bumping the timer. */
			if (mp_fsm_message_process(inst, eth, &rx_msg)) {
				/* A state change occured. Break out of the RX loop early. */
				break;
			}
		}

		/* If we're returning to the READY_FOR_UPDATE_BEGIN state without having
		 * received the entire update image, then send an abort with reason = timeout.
		 */
		if (inst->transfer.total_bytes_received != FIRMWARE_IMAGE_SIZE) {
			mp_send_transfer_abort(inst, eth, MP_TRANSFER_ABORT_REASON_TIMEOUT);
		}

		/* If we got here, then the upgrade has finished or it has timed out.
		 * Either way, go back to the initial state.
		 */
		inst->state = MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN;
		inst->boot_timeout = BOOT_TIMEOUT_SECONDS;
	}
}

/* Initializes the main FSM. */
static void mp_fsm_init(struct mp_system_context *inst)
{
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

	/* Initial state. */
	inst->state = MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN;
	inst->boot_timeout = BOOT_TIMEOUT_SECONDS;

	/* Identify notice message. */
	inst->tx_ident_notice_msg_seq_nr = 0;
	memset(&inst->tx_ident_notice_msg, 0, sizeof(inst->tx_ident_notice_msg));

	/* Header. */
	mp_transfer_msg_header_init(inst, &inst->tx_ident_notice_msg, MP_MSG_TYPE_IDENTIFY_NOTICE);
	/* The mp_transfer_msg_header_init function uses the updater host MAC, but IDENTIFY_NOTICE
	 * should be broadcast.
	 */
	memset(&inst->tx_ident_notice_msg.packet.header.dst_mac, 0xFF, ETHERNET_MAC_LEN);

	/* Payload. */
	inst->tx_ident_notice_msg.packet.payload.identify_notice.platform_type = MP_PLATFORM_TYPE_UNO_W5500;
	inst->tx_ident_notice_msg.packet.payload.identify_notice.system_type = MP_SYSTEM_TYPE_GENERAL_PURPOSE;
	inst->tx_ident_notice_msg.packet.payload.identify_notice.fw_ver_major = FW_VER_MAJOR;
	inst->tx_ident_notice_msg.packet.payload.identify_notice.fw_ver_minor = FW_VER_MINOR;
	inst->tx_ident_notice_msg.packet.payload.identify_notice.fw_image_size = cpu_to_be32(FIRMWARE_IMAGE_SIZE);
	/* Other fields (such as uptime) are already initialized to 0. */
}

/* Bootloader entry point. */
void main(void)
{
	struct spi_device w5500_spi_dev;
	struct w5500_device w5500_dev;
	struct mp_system_context mp_ctx;

	/* CRT setup. */
	init_data();
	init_bss();
	__sync_synchronize();

	/* Init timer core. */
	timer_init();

	/* Init SPI core. */
	spi_bus_init();

	/* Create SPI device. None of these fields are actually used in the
	 * bootloader's SPI core driver (for now).
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

	mp_fsm_init(&mp_ctx);

	while (1) {
		/* The MP FSM will jump to the main application. */
		mp_fsm_run(&mp_ctx, &w5500_dev);
	}
}
