/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone Micro Protocol query utility.
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

struct util_context {
	int socket;
	uint8_t host_mac[ETHERNET_MAC_LEN];
	uint8_t target_mac[ETHERNET_MAC_LEN];
} util_context;

/* Opens a raw Ethernet socket. Returns true on success. */
static bool raw_socket_open(struct util_context *inst, char *itf_name)
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

/* Transmits a MP_MSG_TYPE_IDENTIFY_QUERY message.
 * Returns true on success.
 */
static bool query_send(struct util_context *inst)
{
	ssize_t tmp;
	union mp_msg msg;

	memset(&msg, 0, sizeof(msg));

	memcpy(msg.packet.header.dst_mac, inst->target_mac, ETHERNET_MAC_LEN);
	memcpy(msg.packet.header.src_mac, inst->host_mac, ETHERNET_MAC_LEN);
	msg.packet.header.ethertype = htons(MP_ETHERTYPE);
	msg.packet.header.magic = htonl(MP_MAGIC_NUMBER);
	msg.packet.header.version = MP_VERSION;
	msg.packet.header.msg_type = MP_MSG_TYPE_IDENTIFY_QUERY;

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
	const char *usage = "Usage: mp_query interface [target_mac]\n";
	struct util_context *inst = &util_context;
	union mp_msg rx_msg;

	if (argc < 2 || argc > 3) {
		fprintf(stderr, "%s", usage);
		return EXIT_FAILURE;
	}

	if (argc == 3) {
		if (!string_to_mac(argv[2], inst->target_mac)) {
			fprintf(stderr, "Invalid target MAC\n%s", usage);
			return EXIT_FAILURE;
		}
	} else {
		/* Broadcast. */
		memset(inst->target_mac, 0xFF, ETHERNET_MAC_LEN);
	}

	/* Also sets the host MAC. */
	if (!raw_socket_open(inst, argv[1])) {
		return EXIT_FAILURE;
	}

	if (!query_send(inst)) {
		fprintf(stderr, "Failed to send update request\n");
		close(inst->socket);
		return EXIT_FAILURE;
	}

	printf("\nQuery sent. Waiting for responses...\n\n");

	while (1) {
		/* Wait in units of 10 ms so that a predefined timeout can be added in the future. */
		if (message_wait(inst->socket, &rx_msg, 10)) {
			if (!mp_msg_valid(rx_msg.raw)) {
				continue;
			}

			if (rx_msg.packet.header.msg_type == MP_MSG_TYPE_IDENTIFY_NOTICE) {
				printf("***********************************************************************\n");
				printf("Got notice from %02x:%02x:%02x:%02x:%02x:%02x\n",
				       rx_msg.packet.header.src_mac[0],
				       rx_msg.packet.header.src_mac[1],
				       rx_msg.packet.header.src_mac[2],
				       rx_msg.packet.header.src_mac[3],
				       rx_msg.packet.header.src_mac[4],
				       rx_msg.packet.header.src_mac[5]);

				/* TODO - Replace the tabs with format specifiers. */
				printf("\tState:\t\t\t\t%s\n", mp_system_state_str(rx_msg.packet.header.state));
				printf("\tSequence Number:\t\t%u\n", rx_msg.packet.payload.identify_notice.sequence_nr);
				printf("\tPlatform Type:\t\t\t%s\n", mp_platform_type_str(rx_msg.packet.payload.identify_notice.platform_type));
				printf("\tSystem Type:\t\t\t%s\n", mp_system_type_str(rx_msg.packet.payload.identify_notice.system_type));
				printf("\tUptime:\t\t\t\t%"PRIu64" seconds\n", __builtin_bswap64(rx_msg.packet.payload.identify_notice.uptime_seconds));
				printf("\tFW Version Major:\t\t%u\n", rx_msg.packet.payload.identify_notice.fw_ver_major);
				printf("\tFW Version Minor:\t\t%u\n", rx_msg.packet.payload.identify_notice.fw_ver_minor);
				printf("\tFW Image Size:\t\t\t%u\n", __builtin_bswap32(rx_msg.packet.payload.identify_notice.fw_image_size));
				printf("\tCRC Bad:\t\t\t%s\n", rx_msg.packet.payload.identify_notice.fw_crc_bad? "TRUE" : "FALSE");
				printf("***********************************************************************\n");
			}
		}
	}

	return EXIT_SUCCESS;
}
