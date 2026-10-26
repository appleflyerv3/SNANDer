/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef SNANDER_PROGRAMMER_H
#define SNANDER_PROGRAMMER_H

#include <stdbool.h>

/* Select before init. No implicit fallback to another attached programmer.
 * "ch341a" is the backward-compatible default; "ch347" selects bulk SPI.
 * I2C/Microwire retain their CH341-only drivers and must be rejected for
 * CH347 by the caller before initialization.
 */
int programmer_select(const char *name);
bool programmer_is_ch341a(void);

/* CH347 only, exact supported Hz, before init. Reject rather than silently
 * ignore a clock request on the CH341 backend.
 */
int programmer_set_spi_clock(unsigned int hz);
int programmer_set_spi_mode(const char *mode);
int programmer_init(void);
int programmer_shutdown(void);
int programmer_select_chip(bool active);
int programmer_send_command(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr);

/* Complete command with caller-independent CS ownership. Required for the
 * opt-in CH347 DMA path; other backends keep ordinary manual-CS behavior.
 */
int programmer_transaction(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr);

/* All results are zero/-1. Shutdown still releases resources after failure
 * and reports the latched failure, even if a legacy caller ignored an earlier
 * transport result. It is safe to call shutdown without successful init.
 */
#endif
