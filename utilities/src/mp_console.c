/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Standalone Micro Protocol console tunnelling utility.
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
	bool target_any;
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
	const char *usage = "Usage: mp_console interface target_mac\n";
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
		inst->target_any = false;
	} else {
		inst->target_any = true;
		/* Broadcast. */
		memset(inst->target_mac, 0xFF, ETHERNET_MAC_LEN);
	}

	/* Also sets the host MAC. */
	if (!raw_socket_open(inst, argv[1])) {
		return EXIT_FAILURE;
	}

	/* Don't print anything to the console on success. The user may be
	 * relying on data only from the device making it to stdout.
	 */

	while (1) {
		/* Wait in units of 10 ms so that a predefined timeout can be added in the future. */
		if (message_wait(inst->socket, &rx_msg, 10)) {
			if (!mp_msg_valid(rx_msg.raw)) {
				continue;
			}

			if (inst->target_any || !memcmp(inst->target_mac, rx_msg.packet.header.src_mac, ETHERNET_MAC_LEN)) {
				if (rx_msg.packet.header.msg_type == MP_MSG_TYPE_CONSOLE) {
					write(STDOUT_FILENO,
					      rx_msg.packet.payload.console.data,
					      rx_msg.packet.payload.console.len);
				}
			}
		}
	}

	return EXIT_SUCCESS;
}
