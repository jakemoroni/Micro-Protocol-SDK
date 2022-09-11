/* Copyright (C) 2022 Jacob Moroni (opensource@jakemoroni.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * W5500 Ethernet Driver (Tested with an Arduino Ethernet Shield V2).
 */

#ifndef W5500_H_
#define W5500_H_

#include <stdint.h>
#include <stddef.h>
#include "spi.h"

struct w5500_device {
	struct spi_device *spi_dev;
};

/* Initializes a w5500 device. Returns -1 on failure and 0 on success. */
int w5500_init(struct w5500_device *inst, struct spi_device *spidev);

/* Discards the next frame in the ring buffer.
 * Returns 0 on success and -1 on error (no frame).
 */
int w5500_drop_frame(struct w5500_device *inst);

/* Returns the size of the next frame in the ring buffer. */
uint16_t w5500_next_frame_bytes_avail(struct w5500_device *inst);

/* Reads a frame from the ring buffer. If maxlen is smaller than the frame,
 * then -1 is returned and the frame remains in the ring buffer.
 * If 0 is returned, then no frame was available.
 * Otherwise, the size of the frame is returned.
 */
int16_t w5500_receive_frame(struct w5500_device *inst,
			    uint8_t *rx_buf,
			    size_t maxlen);

/* Transmits a frame (blocking).
 * TODO - This will block forever on contention.
 * Frames larger than 1514 will be silently dropped.
 */
void w5500_send_frame(struct w5500_device *inst,
		      uint8_t *tx_buf,
		      size_t len);

#endif /* W5500_H_ */
