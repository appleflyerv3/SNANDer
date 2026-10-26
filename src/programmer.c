/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "programmer.h"
#include "ch341a_spi.h"
#include "ch347_spi.h"

#include <stdio.h>
#include <string.h>

enum programmer_kind {
    PROGRAMMER_CH341A,
    PROGRAMMER_CH347,
};

static enum programmer_kind selected = PROGRAMMER_CH341A;
static bool initialized;
static bool failed;

int programmer_select(const char *name)
{
    if (initialized) {
        fprintf(stderr, "Cannot change programmer while it is open.\n");
        return -1;
    }

    if (name != NULL && strcmp(name, "ch341a") == 0) {
        selected = PROGRAMMER_CH341A;
        return 0;
    }

    if (name != NULL && strcmp(name, "ch347") == 0) {
        selected = PROGRAMMER_CH347;
        return 0;
    }

    if (name == NULL) {
        name = "(null)";
    }

    fprintf(stderr, "Unknown programmer '%s'; use ch341a or ch347.\n", name);
    return -1;
}

bool programmer_is_ch341a(void)
{
    return selected == PROGRAMMER_CH341A;
}

int programmer_set_spi_clock(unsigned int hz)
{
    if (initialized) {
        fprintf(stderr, "Set the SPI clock before opening the programmer.\n");
        return -1;
    }

    if (programmer_is_ch341a()) {
        fprintf(stderr, "--spi-clock is supported only by the CH347 backend.\n");
        return -1;
    }

    return ch347_spi_set_clock(hz);
}

int programmer_init(void)
{
    int result;

    if (initialized) {
        fprintf(stderr, "Programmer is already open.\n");
        return -1;
    }

    failed = false;

    if (programmer_is_ch341a()) {
        result = ch341a_spi_init();
    } else {
        result = ch347_spi_init();
    }

    if (result != 0) {
        return -1;
    }

    initialized = true;
    return 0;
}

int programmer_select_chip(bool active)
{
    int result;

    if (!initialized || (failed && active)) {
        return -1;
    }

    /* Deselect is still permitted after failure, so callers can clean up.
     * Keep the original CH341 pin driver unchanged.
     */
    if (programmer_is_ch341a()) {
        result = enable_pins(active);
    } else {
        result = ch347_spi_select_chip(active);
    }

    if (result != 0) {
        failed = true;
    }

    if (failed) {
        return -1;
    }

    return 0;
}

int programmer_send_command(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr)
{
    int result;

    if (!initialized || failed) {
        return -1;
    }

    if ((writecnt != 0 && writearr == NULL) ||
        (readcnt != 0 && readarr == NULL)) {
        fprintf(stderr, "SPI transfer has a missing data buffer.\n");
        (void)programmer_select_chip(false);
        failed = true;
        return -1;
    }

    if (writecnt == 0 && readcnt == 0) {
        return 0;
    }

    if (programmer_is_ch341a()) {
        result = ch341a_spi_send_command(writecnt, readcnt, writearr, readarr);
    } else {
        result = ch347_spi_send_command(writecnt, readcnt, writearr, readarr);
    }

    if (result != 0) {
        failed = true;
        (void)programmer_select_chip(false);
        return -1;
    }

    return 0;
}

int programmer_shutdown(void)
{
    int result;
    bool session_failed = failed;

    if (!initialized) {
        return 0;
    }

    if (programmer_is_ch341a()) {
        result = ch341a_spi_shutdown();
    } else {
        result = ch347_spi_shutdown();
    }

    initialized = false;
    failed = false;

    if (session_failed || result != 0) {
        return -1;
    }

    return 0;
}
