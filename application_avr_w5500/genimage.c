/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Simple utility to generate Micro Protocol update images
 * for Arduino UNO compatible devices.
 */

#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#define FIRMWARE_IMAGE_SIZE            28672u /* 32 KiB - 4 KiB bootlodaer */

/* Recalculate the CRC-32 for another byte. */
static uint32_t crc32_update_byte(uint32_t crc_in, uint8_t byte)
{
	size_t i;
	uint32_t crc = ~crc_in;

	crc = crc ^ byte;
	for (i = 0; i < 8; i++) {
		if (crc & 1u) {
			crc = (crc >> 1u) ^ 0xEDB88320; /* Use Ethernet polynomial. */
		} else {
			crc >>= 1u;
		}
	}

	return ~crc;
}

int main(int argc, char *argv[])
{
	int file;
	ssize_t read_ret;
	uint32_t crc;
	uint8_t *buffer;
	ssize_t tmp;
	size_t i;
	int ret = EXIT_SUCCESS;

	if (argc != 2) {
		fprintf(stderr, "Usage: genimage input.bin\nOutput goes to stdout\n");
		return EXIT_FAILURE;
	}

	buffer = malloc(FIRMWARE_IMAGE_SIZE);
	if (!buffer) {
		fprintf(stderr, "Could not allocate buffer\n");
		return EXIT_FAILURE;
	}

	memset(buffer, 0xFF, FIRMWARE_IMAGE_SIZE);

	file = open(argv[1], O_RDONLY);
	if (file == -1) {
		perror("Failed to open firmware file");
		free(buffer);
		return EXIT_FAILURE;
	}

	read_ret = read(file, buffer, FIRMWARE_IMAGE_SIZE);
	if (read_ret == -1) {
		perror("Failed to read firmware file");
		ret = EXIT_FAILURE;
		goto fail;
	} else if (read_ret == 0) {
		fprintf(stderr, "Update file was 0 bytes\n");
		ret = EXIT_FAILURE;
		goto fail;
	} else if (read_ret > (FIRMWARE_IMAGE_SIZE - 4u)) {
		fprintf(stderr,
			"Firmware file is too big (must be %d bytes or less)\n",
			FIRMWARE_IMAGE_SIZE - 4u);
		ret = EXIT_FAILURE;
		goto fail;
	}

	crc = 0;
	for (i = 0; i < (FIRMWARE_IMAGE_SIZE - 4u); i++) {
		crc = crc32_update_byte(crc, buffer[i]);
	}

	buffer[FIRMWARE_IMAGE_SIZE - 4u] = (crc >> 24u);
	buffer[FIRMWARE_IMAGE_SIZE - 3u] = (crc >> 16u);
	buffer[FIRMWARE_IMAGE_SIZE - 2u] = (crc >> 8u);
	buffer[FIRMWARE_IMAGE_SIZE - 1u] = crc;

	/* TODO - Should be a loop here. */
	tmp = write(STDOUT_FILENO, buffer, FIRMWARE_IMAGE_SIZE);
	if (tmp != FIRMWARE_IMAGE_SIZE) {
		fprintf(stderr, "Failed to write all file bytes\n");
		ret = EXIT_FAILURE;
	}

fail:
	close(file);
	free(buffer);
	return ret;
}
