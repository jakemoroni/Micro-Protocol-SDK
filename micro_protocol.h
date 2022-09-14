/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Micro Protocol Version 1
 *
 * Micro Protocol is a pure Ethernet protocol that defines a set
 * of low level functionalities for microcontrollers with an
 * Ethernet connection. The primary purpose is to enable remote
 * firmware updates, but it also supports tunneling console data
 * as well as general device identification/data transfers.
 *
 * Micro Protocol support consists of two main parts:
 *     - A bootloader capable of performing firmware updates
 *     - A hook in the application code to check for and respond
 *       to Micro Protocol messages, specifically, update requests
 *       that typically cause the device to reset back into the
 *       bootloader.
 *
 * Hardware:
 *     - Micro Protocol doesn't require any specific hardware, but
 *       it was designed and tested with an Arduino UNO with a W5500
 *       Ethernet shield.
 *       There are platform types defined, but the update process doesn't
 *       really depend on any specific type. It's just informational.
 *
 * NOTES:
 *     - All multi-byte fields are big endian.
 *     - All reserved fields must be set to 0 on TX and must not be
 *       observed on RX.
 *     - All Micro Protocol messages are exactly 64 bytes (68 including FCS).
 *       However, receivers should accept valid essages that are padded to
 *       a larger size.
 */

#ifndef MICRO_PROTOCOL_H_
#define MICRO_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MP_VERSION                     1u

#define ETHERNET_MAC_LEN               6u

/* Message is 68 bytes including 4-byte FCS. */
#define MP_MSG_SIZE                    64u

/* Use the "NIC testing" ethertype. The EtherType combined with
 * the magic number should be adequate for identifying a valid
 * Micro Protocol message.
 */
#define MP_ETHERTYPE                   0x8822
#define MP_MAGIC_NUMBER                0xC7DA3B3F /* Random */

/* Maximum number of bytes per console message. */
#define MP_MSG_CONSOLE_MAX_LEN         32u

/* Maximum number of bytes per console message. */
#define MP_MSG_TRANSFER_MAX_LEN        32u

/* Top level message micro protocol message types. */
enum mp_msg_type {
	MP_MSG_TYPE_INVALID,

	/* Causes every device on the L2 domain to respond with an IDENTIFY_NOTICE message. */
	MP_MSG_TYPE_IDENTIFY_QUERY,

	/* A response to a query, but can also be transmitted autonomously during state changes.
	 * If the latter, then the destination will be broadcast, otherwise, the destination is
	 * the source of the IDENTIFY_QUERY.
	 */
	MP_MSG_TYPE_IDENTIFY_NOTICE,

	/* Message containing serial port data. Since there's no concept of a connection,
	 * messages originating from the uC will always be sent to broadcast,
	 * but messages from the PC must be addressed to the specific uC.
	 */
	MP_MSG_TYPE_CONSOLE,

	/* Request a system reboot. If destination is broadcast, then all uCs on the
	 * L2 domain should reboot after waiting 1 second. The 1 second delay allows
	 * the host to transmit several reboot requests to ensure that all uCs have
	 * received it.
	 * This message is ignored in the bootloader.
	 */
	MP_MSG_TYPE_REBOOT_REQUEST,

	/* Requests an update. For the AVR, this will cause the uC to reboot
	 * into the bootloader, which will then begin broadcasting periodic
	 * IDENTIFY_NOTICE messages indicating the READY_FOR_UPDATE_BEGIN state.
	 * If it's already in the bootloader, then this message is ignored.
	 * The bootloader will wait 3-10 seconds for the TRANSFER_BEGIN message before
	 * giving up and booting the main application.
	 * For the AVR, this is basically the same as a reboot request, but it's
	 * separated in case we need to support a different type of platform that
	 * can perform updates while running the main application (i.e., it runs
	 * from RAM).
	 */
	MP_MSG_TYPE_UPDATE_REQUEST,

	/* If the uC is in the READY_FOR_UPDATE_BEGIN state, then this begins
	 * the update file transfer/flash process. Otherwise, it's a generic file
	 * transfer.
	 * NOTE: There is no NACK for this. Instead, the host should abort after
	 *       some number of retries after not receiving an ACK.
	 */
	MP_MSG_TYPE_TRANSFER_BEGIN,

	/* Response from the uC when an TRANSFER_BEGIN message is received. */
	MP_MSG_TYPE_TRANSFER_BEGIN_ACK,

	/* File transfer data.
	 * NOTE: There's no way for the host to terminate the transfer. If the host
	 *       needs to terminate, then it must stop sending transfer messages and
	 *       wait for the target to timeout and send a NACK.
	 *       The typical timeout is 1 second.
	 */
	MP_MSG_TYPE_TRANSFER,
	MP_MSG_TYPE_TRANSFER_ACK,

	/* Transmitted by the host or target to preemptively terminate a transfer.
	 * Targets may or may not act on this message. They may just time out.
	 */
	MP_MSG_TYPE_TRANSFER_ABORT,
};

/* Micro protocol system state. */
enum mp_system_state {
	MP_SYSTEM_STATE_INVALID,

	/* The system has booted into the main application and is
	 * in a normal running state.
	 */
	MP_SYSTEM_STATE_RUNNING,

	/* The system is ready to accept a transfer request for
	 * a firmware update.
	 */
	MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN,

	/* A firmware update is in progress. This state is entered as soon
	 * as the TRANSFER_BEGIN is received while in the READY_FOR_UPDATE_BEGIN
	 * state.
	 */
	MP_SYSTEM_STATE_UPDATE_IN_PROGRESS,

	/* A generic transfer is in progress. */
	MP_SYSTEM_STATE_TRANSFER_IN_PROGRESS,
};

/* Predefined platform types. Informational purposes only. */
enum mp_platform_type {
	MP_PLATFORM_TYPE_INVALID,
	MP_PLATFORM_TYPE_UNO_W5500,
	MP_PLATFORM_TYPE_MEGA_W5500,
};

/* System type (for IDENTIFY_NOTICE). Informational purposes only. */
enum mp_system_type {
	MP_SYSTEM_TYPE_INVALID,
	MP_SYSTEM_TYPE_GENERAL_PURPOSE,
	MP_SYSTEM_TYPE_REMOTE_SENSOR,
	MP_SYSTEM_TYPE_REMOTE_GPIO,
};

/* Indicates the cause behind a TRANSFER_ABORT message. */
enum mp_transfer_abort_reason {
	MP_TRANSFER_ABORT_REASON_INVALID,
	MP_TRANSFER_ABORT_REASON_GENERAL_ERROR,

	/* Receiver timed out waiting for a transfer message. */
	MP_TRANSFER_ABORT_REASON_TIMEOUT,

	/* Host tried to send more data than expected. */
	MP_TRANSFER_ABORT_REASON_TOO_LARGE,
};

/* All fields big endian. */
union mp_msg {
	struct {
		struct {
			uint8_t dst_mac[ETHERNET_MAC_LEN];
			uint8_t src_mac[ETHERNET_MAC_LEN];
			uint16_t ethertype;
			uint32_t magic;
			uint8_t version;
			uint8_t state;
			uint8_t msg_type;
			uint8_t reserved[3];
			/* 24 byte header. */
		} __attribute__((packed)) header;

		union {
			/* MP_MSG_TYPE_IDENTIFY_QUERY */
			struct {
				uint8_t sequence_nr;
			} __attribute__((packed)) identify_query;

			/* MP_MSG_TYPE_IDENTIFY_NOTICE */
			struct {
				/* Broadcasted IDENTIFY_NOTICE messages maintain their own sequence number.
				 * Responses due to a QUERY will have the sequence number of the query.
				 */
				uint8_t sequence_nr;
				uint8_t platform_type;
				uint8_t system_type;
				uint64_t uptime_seconds;
				uint8_t fw_ver_major;
				uint8_t fw_ver_minor;
				/* For sanity checking. */
				uint32_t fw_image_size;
				uint8_t fw_crc_bad;
			} __attribute__((packed)) identify_notice;

			/* MP_MSG_TYPE_CONSOLE */
			struct {
				uint8_t len;
				uint8_t data[MP_MSG_CONSOLE_MAX_LEN];
			} __attribute__((packed)) console;

			/* MP_MSG_TYPE_TRANSFER_BEGIN */
			struct {
				uint32_t file_size;
			} __attribute__((packed)) transfer_begin;

			/* MP_MSG_TYPE_TRANSFER */
			struct {
				uint8_t sequence_nr;
				uint8_t len;
				uint8_t data[MP_MSG_TRANSFER_MAX_LEN];
			} __attribute__((packed)) transfer;

			/* MP_MSG_TYPE_TRANSFER_ACK */
			struct {
				uint8_t sequence_nr;
			} __attribute__((packed)) transfer_ack;

			/* MP_MSG_TYPE_TRANSFER_ABORT */
			struct {
				uint8_t reason;
			} __attribute__((packed)) transfer_abort;

			/* Messge types not defined here simply have no payload data (yet). */

		} __attribute__((packed)) payload;
	} packet;
	uint8_t raw[MP_MSG_SIZE];
} __attribute__((packed));

_Static_assert(sizeof(union mp_msg) == MP_MSG_SIZE, "union mp_msg must be 64 bytes");

/* Returns the string name of each message type. */
static inline char *mp_msg_type_str(enum mp_msg_type type)
{
	char *ret = "MP_MSG_TYPE_UNKNOWN";

	switch (type) {
	case MP_MSG_TYPE_INVALID:
		ret = "MP_MSG_TYPE_INVALID";
		break;
	case MP_MSG_TYPE_IDENTIFY_QUERY:
		ret = "MP_MSG_TYPE_IDENTIFY_QUERY";
		break;
	case MP_MSG_TYPE_IDENTIFY_NOTICE:
		ret = "MP_MSG_TYPE_IDENTIFY_NOTICE";
		break;
	case MP_MSG_TYPE_CONSOLE:
		ret = "MP_MSG_TYPE_CONSOLE";
		break;
	case MP_MSG_TYPE_REBOOT_REQUEST:
		ret = "MP_MSG_TYPE_REBOOT_REQUEST";
		break;
	case MP_MSG_TYPE_UPDATE_REQUEST:
		ret = "MP_MSG_TYPE_UPDATE_REQUEST";
		break;
	case MP_MSG_TYPE_TRANSFER_BEGIN:
		ret = "MP_MSG_TYPE_TRANSFER_BEGIN";
		break;
	case MP_MSG_TYPE_TRANSFER_BEGIN_ACK:
		ret = "MP_MSG_TYPE_TRANSFER_BEGIN_ACK";
		break;
	case MP_MSG_TYPE_TRANSFER:
		ret = "MP_MSG_TYPE_TRANSFER";
		break;
	case MP_MSG_TYPE_TRANSFER_ACK:
		ret = "MP_MSG_TYPE_TRANSFER_ACK";
		break;
	case MP_MSG_TYPE_TRANSFER_ABORT:
		ret = "MP_MSG_TYPE_TRANSFER_ABORT";
		break;
	default:
		break;
	}

	return ret;
}

/* Returns the string name of the system state */
static inline char *mp_system_state_str(enum mp_system_state state)
{
	char * ret = "MP_SYSTEM_STATE_UNKNOWN";

	switch (state) {
	case MP_SYSTEM_STATE_INVALID:
		ret = "MP_SYSTEM_STATE_INVALID";
		break;
	case MP_SYSTEM_STATE_RUNNING:
		ret = "MP_SYSTEM_STATE_RUNNING";
		break;
	case MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN:
		ret = "MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN";
		break;
	case MP_SYSTEM_STATE_UPDATE_IN_PROGRESS:
		ret = "MP_SYSTEM_STATE_UPDATE_IN_PROGRESS";
		break;
	case MP_SYSTEM_STATE_TRANSFER_IN_PROGRESS:
		ret = "MP_SYSTEM_STATE_TRANSFER_IN_PROGRESS";
		break;
	default:
		break;

	}

	return ret;
}

/* Returns the string name of the transfer abort reason. */
static inline char *mp_transfer_abort_reason_str(enum mp_transfer_abort_reason reason)
{
	char * ret = "MP_TRANSFER_ABORT_REASON_UNKNOWN";

	switch (reason) {
	case MP_TRANSFER_ABORT_REASON_INVALID:
		ret = "MP_TRANSFER_ABORT_REASON_INVALID";
		break;
	case MP_TRANSFER_ABORT_REASON_GENERAL_ERROR:
		ret = "MP_TRANSFER_ABORT_REASON_GENERAL_ERROR";
		break;
	case MP_TRANSFER_ABORT_REASON_TIMEOUT:
		ret = "MP_TRANSFER_ABORT_REASON_TIMEOUT";
		break;
	case MP_TRANSFER_ABORT_REASON_TOO_LARGE:
		ret = "MP_TRANSFER_ABORT_REASON_TOO_LARGE";
		break;
	default:
		break;
	}

	return ret;
}

/* Returns true if provided a valid Micro Protocol message.
 * The message must be at least the minimum size of 64 bytes.
 */
static inline bool mp_msg_valid(uint8_t *raw)
{
	uint16_t ethertype;
	uint32_t magic;

	/* Avoid aliasing. */
	ethertype = raw[offsetof(union mp_msg, packet.header.ethertype)];
	ethertype <<= 8u;
	ethertype |= raw[offsetof(union mp_msg, packet.header.ethertype) + 1u];

	if (ethertype != MP_ETHERTYPE) {
		return false;
	}

	magic = raw[offsetof(union mp_msg, packet.header.magic)];
	magic <<= 8u;
	magic |= raw[offsetof(union mp_msg, packet.header.magic) + 1u];
	magic <<= 8u;
	magic |= raw[offsetof(union mp_msg, packet.header.magic) + 2u];
	magic <<= 8u;
	magic |= raw[offsetof(union mp_msg, packet.header.magic) + 3u];

	if (magic != MP_MAGIC_NUMBER) {
		return false;
	}

	return true;
}

#endif /* MICRO_PROTOCOL_H_ */
