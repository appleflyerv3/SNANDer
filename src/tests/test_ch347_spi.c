/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Standalone, hardware-free CH347 protocol test.
 *
 * Build with the real libusb declarations, but DO NOT link libusb: every USB
 * function used by ch347_spi.c is replaced below by a scripted fake. Thus even
 * the initialization tests cannot open a USB device. Example, from src/:
 * cc -std=c99 -Wall -Wextra -Werror -I. tests/test_ch347_spi.c ch347_spi.c \
 *     -o /tmp/test_ch347_spi
 *
 * These tests verify this backend's wire contract, not hardware compatibility,
 * SPI signal integrity, flash behavior or firmware-revision compatibility.
 */
#include "ch347_spi.h"

#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: check failed: %s\n", \
            __FILE__, __LINE__, #condition); \
        exit(1); \
    } \
} while (0)

/* The public libusb header declares these as opaque records. Only their
 * addresses are used by the fake; no real libusb implementation is linked.
 */
struct libusb_context { int unused; };
struct libusb_device_handle { int unused; };
static struct libusb_context context;
static struct libusb_device_handle handle;

enum { PACKET_BYTES = 510, PAYLOAD_BYTES = 507, MAX_EVENTS = 64 };

struct event {
    unsigned char endpoint;
    int length;
    int result;
    int transferred;
    unsigned char bytes[PACKET_BYTES];
};

static struct {
    struct event events[MAX_EVENTS];
    unsigned int count;
    unsigned int next;
    unsigned int open_calls;
    unsigned int init_calls;
    unsigned int exit_calls;
    unsigned int close_calls;
    unsigned int detach_calls;
    unsigned int attach_calls;
    unsigned int claim_calls;
    unsigned int release_calls;
    bool variant_f;
    bool no_device;
    int interface;
    int driver_active;
    int claim_result;
} fake;

static struct event *queue_event(unsigned char endpoint, int length,
    int result, int transferred)
{
    CHECK(fake.count < MAX_EVENTS);
    CHECK(length >= 0 && length <= PACKET_BYTES);
    CHECK(transferred >= 0 && transferred <= PACKET_BYTES);
    struct event *event = &fake.events[fake.count++];
    event->endpoint = endpoint;
    event->length = length;
    event->result = result;
    event->transferred = transferred;
    return event;
}

static void expect_packet(unsigned char command, const unsigned char *payload,
    unsigned int length)
{
    CHECK(length <= PAYLOAD_BYTES);
    struct event *event = queue_event(0x06, 3 + (int)length, 0,
        3 + (int)length);
    event->bytes[0] = command;
    event->bytes[1] = length & 0xff;
    event->bytes[2] = length >> 8;
    if (length != 0) {
        memcpy(event->bytes + 3, payload, length);
    }
}

static void reply_packet(unsigned char command, const unsigned char *payload,
    unsigned int length)
{
    CHECK(length <= PAYLOAD_BYTES);
    struct event *event = queue_event(0x86, PACKET_BYTES, 0,
        3 + (int)length);
    event->bytes[0] = command;
    event->bytes[1] = length & 0xff;
    event->bytes[2] = length >> 8;
    if (length != 0) {
        memcpy(event->bytes + 3, payload, length);
    }
}

static void reply_status(unsigned char command, unsigned char status)
{
    reply_packet(command, &status, 1);
}

static void expect_cs(bool active)
{
    const unsigned char payload[10] = {
        active ? 0x80 : 0xc0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    };
    expect_packet(0xc1, payload, sizeof(payload));
}

static void reset_fixture(bool variant_f, int driver_active)
{
    memset(&fake, 0, sizeof(fake));
    fake.variant_f = variant_f;
    fake.interface = variant_f ? 4 : 2;
    fake.driver_active = driver_active;
}

static void expect_initialization(unsigned char prescaler)
{
    unsigned char query = 1;
    unsigned char original[26];
    /* Explicit expected wire bytes, including preserved CRC/reserved bytes
     * and the unrelated low six OtherCfg bits from the queried config.
     */
    unsigned char configured[26] = {
        0x00, 0x00, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x02, prescaler, 0x00, 0x00, 0x00,
        0xa5, 0xa5, 0x00, 0x00, 0x00, 0x25, 0xa5, 0xa5,
        0xa5, 0xa5,
    };
    memset(original, 0xa5, sizeof(original));
    expect_packet(0xca, &query, 1);
    reply_packet(0xca, original, sizeof(original));
    expect_packet(0xc0, configured, sizeof(configured));
    reply_status(0xc0, 0);
    expect_cs(false);
}

static void check_finished(bool detached)
{
    CHECK(fake.next == fake.count);
    CHECK(fake.init_calls == 1);
    CHECK(fake.exit_calls == 1);
    CHECK(fake.close_calls == 1);
    CHECK(fake.claim_calls == 1);
    CHECK(fake.release_calls == 1);
    CHECK(fake.detach_calls == (unsigned int)detached);
    CHECK(fake.attach_calls == (unsigned int)detached);
    CHECK(fake.open_calls == (fake.variant_f ? 2u : 1u));
}

int LIBUSB_CALL libusb_init(libusb_context **output)
{
    CHECK(output != NULL);
    fake.init_calls++;
    *output = &context;
    return 0;
}

void LIBUSB_CALL libusb_exit(libusb_context *input)
{
    CHECK(input == &context);
    fake.exit_calls++;
}

libusb_device_handle *LIBUSB_CALL libusb_open_device_with_vid_pid(
    libusb_context *input, uint16_t vendor_id, uint16_t product_id)
{
    CHECK(input == &context);
    CHECK(vendor_id == 0x1a86);
    fake.open_calls++;
    CHECK(product_id == (fake.open_calls == 1 ? 0x55db : 0x55de));
    if (fake.no_device || (fake.variant_f && product_id == 0x55db)) {
        return NULL;
    }
    return &handle;
}

int LIBUSB_CALL libusb_kernel_driver_active(libusb_device_handle *input,
    int interface)
{
    CHECK(input == &handle && interface == fake.interface);
    return fake.driver_active;
}

int LIBUSB_CALL libusb_detach_kernel_driver(libusb_device_handle *input,
    int interface)
{
    CHECK(input == &handle && interface == fake.interface);
    fake.detach_calls++;
    return 0;
}

int LIBUSB_CALL libusb_attach_kernel_driver(libusb_device_handle *input,
    int interface)
{
    CHECK(input == &handle && interface == fake.interface);
    fake.attach_calls++;
    return 0;
}

int LIBUSB_CALL libusb_claim_interface(libusb_device_handle *input,
    int interface)
{
    CHECK(input == &handle && interface == fake.interface);
    fake.claim_calls++;
    return fake.claim_result;
}

int LIBUSB_CALL libusb_release_interface(libusb_device_handle *input,
    int interface)
{
    CHECK(input == &handle && interface == fake.interface);
    fake.release_calls++;
    return 0;
}

void LIBUSB_CALL libusb_close(libusb_device_handle *input)
{
    CHECK(input == &handle);
    fake.close_calls++;
}

const char *LIBUSB_CALL libusb_error_name(int error)
{
    (void)error;
    return "scripted USB result";
}

int LIBUSB_CALL libusb_bulk_transfer(libusb_device_handle *input,
    unsigned char endpoint, unsigned char *data, int length,
    int *transferred, unsigned int timeout)
{
    CHECK(input == &handle);
    CHECK(timeout == 10000);
    CHECK(fake.next < fake.count);
    const struct event *event = &fake.events[fake.next++];
    CHECK(endpoint == event->endpoint);
    CHECK(length == event->length);
    if (endpoint == 0x06) {
        CHECK(memcmp(data, event->bytes, (size_t)length) == 0);
    } else {
        CHECK(endpoint == 0x86);
        CHECK(event->transferred <= length);
        memcpy(data, event->bytes, (size_t)event->transferred);
    }
    *transferred = event->transferred;
    return event->result;
}

static void test_fragmented_transaction(void)
{
    unsigned char data[508];
    unsigned char output[508];
    const unsigned char prefix[] = {0x03, 0x00, 0x01, 0x20};
    const unsigned char count[] = {0xfc, 0x01, 0x00, 0x00};
    for (unsigned int index = 0; index < sizeof(data); index++) {
        data[index] = index & 0xff;
    }
    reset_fixture(false, 1);
    /* First test checks the backend's real default, before any setter call. */
    expect_initialization(4 << 3);
    expect_cs(true);
    expect_packet(0xc4, prefix, sizeof(prefix));
    reply_status(0xc4, 0);
    expect_packet(0xc4, data, 507);
    reply_status(0xc4, 0);
    expect_packet(0xc4, data + 507, 1);
    reply_status(0xc4, 0);
    expect_packet(0xc3, count, sizeof(count));
    reply_packet(0xc3, data, 507);
    reply_packet(0xc3, data + 507, 1);
    expect_cs(false);
    expect_cs(false); /* shutdown independently attempts CS cleanup */
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_set_clock(7500000) == -1);
    CHECK(ch347_spi_select_chip(true) == 0);
    CHECK(ch347_spi_send_command(sizeof(prefix), 0, prefix, NULL) == 0);
    CHECK(ch347_spi_send_command(sizeof(data), 0, data, NULL) == 0);
    CHECK(ch347_spi_send_command(0, sizeof(output), NULL, output) == 0);
    CHECK(memcmp(data, output, sizeof(data)) == 0);
    CHECK(ch347_spi_send_command(0, 0, NULL, NULL) == 0);
    CHECK(ch347_spi_select_chip(false) == 0);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(true);
    CHECK(ch347_spi_shutdown() == 0);
}

static void test_f_variant_and_clocks(void)
{
    for (unsigned int divisor = 0; divisor <= 7; divisor++) {
        CHECK(ch347_spi_set_clock(60000000u >> divisor) == 0);
    }
    CHECK(ch347_spi_set_clock(0) == -1);
    CHECK(ch347_spi_set_clock(4000000) == -1);
    CHECK(ch347_spi_set_clock(7500000) == 0);
    reset_fixture(true, LIBUSB_ERROR_NOT_SUPPORTED);
    expect_initialization(3 << 3);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(false);
    CHECK(ch347_spi_set_clock(3750000) == 0);
}

enum fault {
    BAD_STATUS, WRONG_COMMAND, SHORT_HEADER, WRONG_LENGTH,
    ZERO_PROGRESS, OVERSIZED_REPLY, READ_TIMEOUT, SHORT_WRITE,
};

static void test_transfer_fault(enum fault fault, bool cleanup_fails)
{
    unsigned char byte = 0x9f;
    unsigned char output = 0x5a;
    const unsigned char count[] = {1, 0, 0, 0};
    reset_fixture(false, 1);
    expect_initialization(4 << 3);
    expect_cs(true);
    bool write_fault = fault == BAD_STATUS || fault == READ_TIMEOUT ||
        fault == SHORT_WRITE;
    if (write_fault) {
        expect_packet(0xc4, &byte, 1);
        if (fault == SHORT_WRITE) {
            fake.events[fake.count - 1].transferred = 3;
        } else {
            reply_status(0xc4, fault == BAD_STATUS ? 1 : 0);
            if (fault == READ_TIMEOUT) {
                fake.events[fake.count - 1].result = LIBUSB_ERROR_TIMEOUT;
            }
        }
    } else {
        expect_packet(0xc3, count, sizeof(count));
        reply_status(fault == WRONG_COMMAND ? 0xc4 : 0xc3, 0x12);
        struct event *reply = &fake.events[fake.count - 1];
        if (fault == SHORT_HEADER) {
            reply->transferred = 2;
        } else if (fault == WRONG_LENGTH) {
            reply->bytes[1] = 2;
        } else if (fault == ZERO_PROGRESS) {
            reply->bytes[1] = 0;
            reply->transferred = 3;
        } else if (fault == OVERSIZED_REPLY) {
            reply->bytes[1] = 2;
            reply->transferred = 5;
        }
    }
    expect_cs(false); /* failure handler */
    if (cleanup_fails) {
        fake.events[fake.count - 1].result = LIBUSB_ERROR_IO;
    }
    expect_cs(false); /* shutdown must proceed even if this also fails */
    if (cleanup_fails) {
        fake.events[fake.count - 1].result = LIBUSB_ERROR_IO;
    }
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_select_chip(true) == 0);
    CHECK(ch347_spi_send_command(write_fault ? 1 : 0,
        write_fault ? 0 : 1, write_fault ? &byte : NULL,
        write_fault ? NULL : &output) == -1);
    CHECK(output == 0x5a); /* malformed/partial replies never publish data */
    unsigned int consumed = fake.next;
    CHECK(ch347_spi_send_command(1, 0, &byte, NULL) == -1);
    CHECK(ch347_spi_select_chip(true) == -1);
    CHECK(fake.next == consumed); /* failed commands are never replayed */
    CHECK(ch347_spi_shutdown() == -1);
    check_finished(true);
}

static void test_claim_failure_cleanup(void)
{
    reset_fixture(true, 1);
    fake.claim_result = LIBUSB_ERROR_BUSY;
    CHECK(ch347_spi_init() == -1);
    CHECK(fake.count == 0 && fake.next == 0); /* no configuration I/O */
    CHECK(fake.open_calls == 2 && fake.claim_calls == 1);
    CHECK(fake.release_calls == 0);
    CHECK(fake.detach_calls == 1 && fake.attach_calls == 1);
    CHECK(fake.close_calls == 1 && fake.exit_calls == 1);
    CHECK(ch347_spi_shutdown() == 0);
}

static void test_no_device_cleanup(void)
{
    reset_fixture(false, 0);
    fake.no_device = true;
    CHECK(ch347_spi_init() == -1);
    CHECK(fake.count == 0 && fake.next == 0);
    CHECK(fake.open_calls == 2 && fake.claim_calls == 0);
    CHECK(fake.close_calls == 0 && fake.exit_calls == 1);
    CHECK(ch347_spi_shutdown() == 0);
}

int main(void)
{
    test_fragmented_transaction();
    test_f_variant_and_clocks();
    for (enum fault fault = BAD_STATUS; fault <= SHORT_WRITE; fault++) {
        test_transfer_fault(fault, false);
    }
    test_transfer_fault(BAD_STATUS, true);
    test_claim_failure_cleanup();
    test_no_device_cleanup();
    puts("CH347 offline protocol tests passed (no real USB implementation linked).");
    return 0;
}
