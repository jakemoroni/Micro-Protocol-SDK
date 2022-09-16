/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone firmware update utility for devices implementing
 * micro protocol.
 */

#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include "micro_protocol.h"

/* For now, only images up to 16 MiB are supported. */
#define FIRMWARE_IMAGE_MAX_SIZE        0x1000000

enum update_state {
	UPDATE_STATE_INVALID,
	UPDATE_STATE_WAITING_FOR_IDENTIFY_NOTICE,
	UPDATE_STATE_WAITING_FOR_TRANSFER_BEGIN_ACK,
	UPDATE_STATE_TRANSFERRING,
	UPDATE_STATE_FAILED
};

struct update_context {
	enum update_state state;
	int socket;
	uint8_t host_mac[ETHERNET_MAC_LEN];
	uint8_t target_mac[ETHERNET_MAC_LEN];
	uint32_t expected_fw_size;
	uint32_t bytes_sent;
	uint8_t transfer_seq_nr;
	uint32_t retransmits;
} update_context;

/* Opens a raw Ethernet socket. Returns true on success. */
static bool raw_socket_open(struct update_context *inst, char *itf_name)
{
	struct sockaddr_ll bind_info;
	/* struct packet_mreq mreq; */
	struct ifreq if_mac;

	inst->socket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (inst->socket == -1) {
		perror("Could not open raw socket");
		return false;
	}

	/* Bind to interface. */
	memset(&bind_info, 0, sizeof(bind_info));
	bind_info.sll_family = AF_PACKET;
	bind_info.sll_protocol = htons(ETH_P_ALL);
	errno = 0;
	if ((bind_info.sll_ifindex = if_nametoindex(itf_name)) == 0) {
		if (errno) {
			perror("Could not find interface");
			close(inst->socket);
			return false;
		}
	}
	if (bind(inst->socket, (struct sockaddr *)&bind_info, sizeof(struct sockaddr_ll))) {
		perror("Could not bind to interface");
		close(inst->socket);
		return false;
	}

#if 0 /* Promisc mode is not needed. */
	/* Set interface to promisc mode. */
	memset(&mreq, 0, sizeof(mreq));
	mreq.mr_ifindex = if_nametoindex(itf_name);
	mreq.mr_type = PACKET_MR_PROMISC;
	if (setsockopt(eth_socket, SOL_PACKET, PACKET_ADD_MEMBERSHIP, &mreq, sizeof(mreq))) {
		perror("setsockopt");
		close(eth_socket);
		return -1;
	}
#endif

	memset(&if_mac, 0, sizeof(struct ifreq));
	strncpy(if_mac.ifr_name, itf_name, IFNAMSIZ - 1);

	if (ioctl(inst->socket, SIOCGIFHWADDR, &if_mac) < 0) {
		perror("SIOCGIFHWADDR");
		close(inst->socket);
		return false;
	}

	memcpy(inst->host_mac, if_mac.ifr_hwaddr.sa_data, ETHERNET_MAC_LEN);

	return true;
}

/* Returns 64 bit nanoseconds monotonically increasing nanoseconds value. */
static uint64_t get_nanos(void)
{
	struct timespec ts;
	uint64_t ret;

	clock_gettime(CLOCK_MONOTONIC, &ts);

	ret = ts.tv_sec;
	ret *= 1000000000u;
	ret += ts.tv_nsec;

	return ret;
}

/* Returns true if a potential micro protocol message was received, false if timeout. */
static bool message_wait(int sock_fd, union mp_msg *msg, uint32_t timeout_ms)
{
	int poll_ret;
	ssize_t read_ret;
	struct pollfd pfd;

	pfd.fd = sock_fd;
	pfd.events = POLLIN;

	poll_ret = poll(&pfd, 1, timeout_ms);
	if (poll_ret == -1) {
		perror("poll");
		return false;
	}

	if (pfd.revents & POLLIN) {
		read_ret = read(sock_fd, msg, sizeof(union mp_msg));
		if (read_ret == -1) {
			perror("read");
			return false;
		}

		/* MP messages are all 64 bytes. More is okay but less is not.
		 * If the message is larger than 64, it will be truncated to 64 on read.
		 */
		if (read_ret != sizeof(union mp_msg)) {
			return false;
		}

		return true;
	}

	return false;
}

/* Transmits a MP_MSG_TYPE_UPDATE_REQUEST message. */
static bool update_request_send(struct update_context *inst)
{
	ssize_t tmp;
	union mp_msg msg;

	memset(&msg, 0, sizeof(msg));

	memcpy(msg.packet.header.dst_mac, inst->target_mac, ETHERNET_MAC_LEN);
	memcpy(msg.packet.header.src_mac, inst->host_mac, ETHERNET_MAC_LEN);
	msg.packet.header.ethertype = htons(MP_ETHERTYPE);
	msg.packet.header.magic = htonl(MP_MAGIC_NUMBER);
	msg.packet.header.version = MP_VERSION;
	msg.packet.header.msg_type = MP_MSG_TYPE_UPDATE_REQUEST;

	tmp = write(inst->socket, msg.raw, sizeof(msg));
	if (tmp == -1) {
		perror("Could not send frame");
		return false;
	} else if (tmp != sizeof(msg)) {
		fprintf(stderr, "Failed to send entire frame\n");
		return false;
	}

	return true;
}

/* Transmits a MP_MSG_TYPE_TRANSFER_BEGIN message. */
static bool transfer_begin_send(struct update_context *inst)
{
	ssize_t tmp;
	union mp_msg msg;

	memset(&msg, 0, sizeof(msg));

	memcpy(msg.packet.header.dst_mac, inst->target_mac, ETHERNET_MAC_LEN);
	memcpy(msg.packet.header.src_mac, inst->host_mac, ETHERNET_MAC_LEN);
	msg.packet.header.ethertype = htons(MP_ETHERTYPE);
	msg.packet.header.magic = htonl(MP_MAGIC_NUMBER);
	msg.packet.header.version = MP_VERSION;
	msg.packet.header.msg_type = MP_MSG_TYPE_TRANSFER_BEGIN;

	msg.packet.payload.transfer_begin.file_size = htonl(inst->expected_fw_size);

	tmp = write(inst->socket, msg.raw, sizeof(msg));
	if (tmp == -1) {
		perror("Could not send frame");
		return false;
	} else if (tmp != sizeof(msg)) {
		fprintf(stderr, "Failed to send entire frame\n");
		return false;
	}

	return true;
}

/* Transmits a MP_MSG_TYPE_TRANSFER message containing update file data. */
static bool transfer_chunk_send(struct update_context *inst, uint8_t *buf, size_t len)
{
	ssize_t tmp;
	union mp_msg msg;

	memset(&msg, 0, sizeof(msg));

	memcpy(msg.packet.header.dst_mac, inst->target_mac, ETHERNET_MAC_LEN);
	memcpy(msg.packet.header.src_mac, inst->host_mac, ETHERNET_MAC_LEN);
	msg.packet.header.ethertype = htons(MP_ETHERTYPE);
	msg.packet.header.magic = htonl(MP_MAGIC_NUMBER);
	msg.packet.header.version = MP_VERSION;
	msg.packet.header.msg_type = MP_MSG_TYPE_TRANSFER;
	msg.packet.payload.transfer.len = len;
	memcpy(msg.packet.payload.transfer.data, buf, len);
	msg.packet.payload.transfer.sequence_nr = inst->transfer_seq_nr;

	tmp = write(inst->socket, msg.raw, sizeof(msg));
	if (tmp == -1) {
		perror("Could not send frame");
		return false;
	} else if (tmp != sizeof(msg)) {
		fprintf(stderr, "Failed to send entire frame\n");
		return false;
	}

	return true;
}

/* Waits up to timeout_ms for an IDENTIFY_NOTICE to arrive from our
 * target that indicates it's ready to begin the update process.
 */
static void fsm_wait_for_rdy_for_update_state(struct update_context *inst,
					      uint32_t timeout_ms)
{
	union mp_msg rx_msg;
	uint64_t timeout_nanos;
	uint64_t timer_start;

	timeout_nanos = timeout_ms;
	timeout_nanos *= 1000000u;
	timer_start = get_nanos();

	while ((get_nanos() - timer_start) < timeout_nanos) {
		if (message_wait(inst->socket, &rx_msg, 10)) {
			if (!mp_msg_valid(rx_msg.raw)) {
				continue;
			}

			if (memcmp(rx_msg.packet.header.src_mac, inst->target_mac, ETHERNET_MAC_LEN)) {
				continue;
			}

			if (rx_msg.packet.header.version != MP_VERSION) {
				continue;
			}

			if (rx_msg.packet.header.msg_type != MP_MSG_TYPE_IDENTIFY_NOTICE) {
				continue;
			}

			if (rx_msg.packet.header.state != MP_SYSTEM_STATE_READY_FOR_UPDATE_BEGIN) {
				printf("Target device responded, but is not ready for update (target state = %s). Waiting...\n",
				       mp_system_state_str(rx_msg.packet.header.state));
				continue;
			}

			printf("Target device responded and is ready for update:\n");
			printf("\tState:\t\t\t\t%s\n", mp_system_state_str(rx_msg.packet.header.state));
			printf("\tSequence Number:\t\t%u\n", rx_msg.packet.payload.identify_notice.sequence_nr);
			printf("\tPlatform Type:\t\t\t%s\n", mp_platform_type_str(rx_msg.packet.payload.identify_notice.platform_type));
			printf("\tSystem Type:\t\t\t%s\n", mp_system_type_str(rx_msg.packet.payload.identify_notice.system_type));
			printf("\tUptime:\t\t\t\t%"PRIu64" seconds\n", __builtin_bswap64(rx_msg.packet.payload.identify_notice.uptime_seconds));
			printf("\tFW Version Major:\t\t%u\n", rx_msg.packet.payload.identify_notice.fw_ver_major);
			printf("\tFW Version Minor:\t\t%u\n", rx_msg.packet.payload.identify_notice.fw_ver_minor);
			printf("\tFW Image Size:\t\t\t%u\n", __builtin_bswap32(rx_msg.packet.payload.identify_notice.fw_image_size));
			printf("\tCRC Bad:\t\t\t%s\n", rx_msg.packet.payload.identify_notice.fw_crc_bad? "TRUE" : "FALSE");

			inst->expected_fw_size = ntohl(rx_msg.packet.payload.identify_notice.fw_image_size);
			inst->state = UPDATE_STATE_WAITING_FOR_TRANSFER_BEGIN_ACK;
			return;
		}
	}
}

/* Waits for up to timeout_ms for an TRANSFER_BEGIN_ACK to arrive. */
static void fsm_wait_for_begin_ack(struct update_context *inst,
				   uint32_t timeout_ms)
{
	union mp_msg rx_msg;
	uint64_t timeout_nanos;
	uint64_t timer_start;

	timeout_nanos = timeout_ms;
	timeout_nanos *= 1000000u;
	timer_start = get_nanos();

	while ((get_nanos() - timer_start) < timeout_nanos) {
		if (message_wait(inst->socket, &rx_msg, 10)) {
			if (!mp_msg_valid(rx_msg.raw)) {
				continue;
			}

			if (memcmp(rx_msg.packet.header.src_mac, inst->target_mac, ETHERNET_MAC_LEN)) {
				continue;
			}

			if (rx_msg.packet.header.version != MP_VERSION) {
				continue;
			}

			if (rx_msg.packet.header.state != MP_SYSTEM_STATE_UPDATE_IN_PROGRESS) {
				printf("WARNING: The target device has unexpectedly transitioned "
				       "from the update state. Retrying...\n");
				inst->state = UPDATE_STATE_WAITING_FOR_IDENTIFY_NOTICE;
				return;
			}

			if (rx_msg.packet.header.msg_type != MP_MSG_TYPE_TRANSFER_BEGIN_ACK) {
				continue;
			}

			/* Transfer begin ACK needs to be addressed to us. */
			if (memcmp(rx_msg.packet.header.dst_mac, inst->host_mac, ETHERNET_MAC_LEN)) {
				fprintf(stderr, "ERROR: An update is in progress with another host - aborting\n");
				inst->state = UPDATE_STATE_FAILED;
				return;
			}

			printf("Starting transfer...\n");

			inst->bytes_sent = 0;
			inst->transfer_seq_nr = 0;
			inst->retransmits = 0;
			inst->state = UPDATE_STATE_TRANSFERRING;
			return;
		}
	}
}

/* Waits for up to timeout_ms for an TRANSFER_ACK to arrive.
 * Returns true if good ACK is received.
 */
static bool fsm_wait_for_transfer_ack(struct update_context *inst,
				      uint32_t timeout_ms)
{
	union mp_msg rx_msg;
	uint64_t timeout_nanos;
	uint64_t timer_start;

	timeout_nanos = timeout_ms;
	timeout_nanos *= 1000000u;
	timer_start = get_nanos();

	while ((get_nanos() - timer_start) < timeout_nanos) {
		if (message_wait(inst->socket, &rx_msg, 10)) {
			if (!mp_msg_valid(rx_msg.raw)) {
				continue;
			}

			if (memcmp(rx_msg.packet.header.src_mac, inst->target_mac, ETHERNET_MAC_LEN)) {
				continue;
			}

			if (rx_msg.packet.header.version != MP_VERSION) {
				continue;
			}

			if (rx_msg.packet.header.state != MP_SYSTEM_STATE_UPDATE_IN_PROGRESS) {
				printf("WARNING: The target device has unexpectedly transitioned "
				       "from the update state. Retrying...\n");
				inst->state = UPDATE_STATE_WAITING_FOR_IDENTIFY_NOTICE;
				return false;

			}

			if (rx_msg.packet.header.msg_type != MP_MSG_TYPE_TRANSFER_ACK) {
				continue;
			}

			/* Transfer ACK needs to be addressed to us. */
			if (memcmp(rx_msg.packet.header.dst_mac, inst->host_mac, ETHERNET_MAC_LEN)) {
				printf("An update is in progress with another host. Cannot recover.\n");
				inst->state = UPDATE_STATE_FAILED;
				return false;
			}

			if (rx_msg.packet.payload.transfer_ack.sequence_nr != inst->transfer_seq_nr) {
				printf("WARNING: Unexpected sequence number received (expected %u, got %u)\n",
				       inst->transfer_seq_nr,
				       rx_msg.packet.payload.transfer_ack.sequence_nr);
			}

			return true;
		}
	}

	return false;
}

/* Returns false if string was not a MAC. */
static bool string_to_mac(char *str, uint8_t *mac)
{
	int tmp;

	tmp = sscanf(str,
		     "%02hhX:%02hhX:%02hhX:%02hhX:%02hhX:%02hhX",
		     &mac[0],
		     &mac[1],
		     &mac[2],
		     &mac[3],
		     &mac[4],
		     &mac[5]);
	if (tmp != ETHERNET_MAC_LEN) {
		return false;
	}

	return true;
}

int main(int argc, char *argv[])
{
	const char *usage =
		"Usage: mp_update interface target_mac filename.update\n"
		"\tinterface: the Ethernet device connected to the remote device's network\n"
		"\ttarget_mac: the remote device's MAC address\n";
	struct update_context *inst = &update_context;
	uint8_t *image_buffer;
	int update_file_fd;
	ssize_t read_ret;
	int ret = EXIT_SUCCESS;

	if (argc != 4) {
		fprintf(stderr, "%s", usage);
		return EXIT_FAILURE;
	}

	if (!string_to_mac(argv[2], inst->target_mac)) {
		fprintf(stderr, "Invalid target MAC\n%s", usage);
		return EXIT_FAILURE;
	}

	image_buffer = malloc(FIRMWARE_IMAGE_MAX_SIZE);
	if (!image_buffer) {
		fprintf(stderr, "Could not allocate image buffer\n");
		return EXIT_FAILURE;
	}

	memset(image_buffer, 0xFF, FIRMWARE_IMAGE_MAX_SIZE);

	update_file_fd = open(argv[3], O_RDONLY);
	if (update_file_fd == -1) {
		perror("Failed to open update file");
		ret = EXIT_FAILURE;
		goto fail_file_open;
	}

	read_ret = read(update_file_fd, image_buffer, FIRMWARE_IMAGE_MAX_SIZE);
	if (read_ret == -1) {
		perror("Failed to read update file");
		ret = EXIT_FAILURE;
		goto fail_file_read;
	} else if (read_ret == 0) {
		fprintf(stderr, "Update file is too small\n");
		ret = EXIT_FAILURE;
		goto fail_file_read;
	}

	if (!raw_socket_open(inst, argv[1])) {
		ret = EXIT_FAILURE;
		goto fail_file_read;
	}

	/* Initial state. */
	inst->state = UPDATE_STATE_WAITING_FOR_IDENTIFY_NOTICE;

	/* FSM */
	while (1) {
		if (inst->state == UPDATE_STATE_WAITING_FOR_IDENTIFY_NOTICE) {
			printf("Sending update request to %s...\n", argv[2]);

			if (!update_request_send(inst)) {
				fprintf(stderr, "ERROR: Failed to send update request - aborting\n");
				ret = EXIT_FAILURE;
				goto fail_update;
			}

			/* Wait up to 1 second for an IDENTIFY_NOTICE to arrive indicating the
			 * "ready for update begin" state.
			 * If it arrives, the function will change the top level state.
			 */
			fsm_wait_for_rdy_for_update_state(inst, 1000u);
		} else if (inst->state == UPDATE_STATE_WAITING_FOR_TRANSFER_BEGIN_ACK) {
			if (inst->expected_fw_size != read_ret) {
				fprintf(stderr,
					"ERROR: The file size provided does not match the expected "
					"value reported by the device (%lu vs %u) - aborting\n",
					read_ret,
					inst->expected_fw_size);
				ret = EXIT_FAILURE;
				goto fail_update;
			}

			if (!transfer_begin_send(inst)) {
				fprintf(stderr, "ERROR: Failed to send update begin - aborting\n");
				ret = EXIT_FAILURE;
				goto fail_update;
			}

			/* Wait up to 1 second for a TRANSFER_BEGIN_ACK to arrive. */
			fsm_wait_for_begin_ack(inst, 1000u);
		} else if (inst->state == UPDATE_STATE_TRANSFERRING) {
			uint32_t bytes_remaining = read_ret - inst->bytes_sent;
			uint32_t bytes_to_send =
				(bytes_remaining < MP_MSG_TRANSFER_MAX_LEN)? bytes_remaining : MP_MSG_TRANSFER_MAX_LEN;

			if (bytes_remaining) {
				/* TODO - Refresh current line instead. */
				printf("Sending transfer sequence %03u (%d of %ld bytes)\n",
				       inst->transfer_seq_nr,
				       inst->bytes_sent,
				       read_ret);

				if (!transfer_chunk_send(inst, &image_buffer[inst->bytes_sent], bytes_to_send)) {
					fprintf(stderr, "ERROR: Failed to send transfer chunk - aborting\n");
					ret = EXIT_FAILURE;
					goto fail_update;
				}

				/* Wait 100 milliseconds before sending the sequence again.
			         * The target devices have a 1 second timeout, so this gives us 10 retries.
				 */
				if (fsm_wait_for_transfer_ack(inst, 100u)) {
					inst->transfer_seq_nr++;
					inst->bytes_sent += bytes_to_send;
				} else {
					inst->retransmits++;
				}
			} else {
				printf("\nUpgrade completed successfully (%u retransmissions)\n", inst->retransmits);
				goto done;
			}
		}
	}

done:
fail_update:
	close(inst->socket);
fail_file_read:
	close(update_file_fd);
fail_file_open:
	free(image_buffer);
	return ret;
}
