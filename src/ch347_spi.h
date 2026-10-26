/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef SNANDER_CH347_SPI_H
#define SNANDER_CH347_SPI_H

#include <stdbool.h>

/* Bulk/VCP SPI only: CH347T mode 1 and CH347F. Mode 0, MSB first,
 * active-low CS0, single data lane. Clock selection is configuration-only
 * before init; accepted frequencies are exactly 60 MHz / 2^N, N = 0..7.
 * The default is 3.75 MHz. This does not configure board supply voltage.
 */
int ch347_spi_set_clock(unsigned int hz);
int ch347_spi_init(void);
int ch347_spi_shutdown(void);

/* CS belongs to the caller's complete transaction, not each byte/fragment.
 * One asserted CS may span several send_command calls, as SNANDer requires
 * for opcode/address/data. An error best-effort deselects and latches the
 * session failed; no partial command is replayed or silently retried.
 */
int ch347_spi_select_chip(bool active);
int ch347_spi_send_command(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr);

#endif
