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
/* Opt-in "dma" uses WCH's C2 automatic-CS protocol, not E1/E2 commands.
 * CH347T firmware must be >=0x0341; CH347F is supported by WCH's driver.
 * Firmware is queried through CA, never inferred from the USB bcdDevice.
 * The default "normal" mode retains the existing 507-byte protocol.
 */
int ch347_spi_set_mode(const char *mode);
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

/* ONE complete transaction, starting/ending with CS0 inactive. Normal mode
 * wraps the manual-CS API; DMA mode controls CS in its first/last C2 frame.
 * It never merges distinct commands, nor changes the fragmented API above.
 */
int ch347_spi_transaction(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr);

#endif
