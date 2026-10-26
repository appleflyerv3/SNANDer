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
struct libusb_device { int unused; };
static struct libusb_context context;
static struct libusb_device_handle handle;
static struct libusb_device usb_device;

enum { PACKET_BYTES = 510, PAYLOAD_BYTES = 507,
    DMA_PACKET_BYTES = 4096, DMA_PAYLOAD_BYTES = 4093, MAX_EVENTS = 64 };

struct event {
    unsigned char endpoint;
    int length;
    int result;
    int transferred;
    unsigned char bytes[DMA_PACKET_BYTES];
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
    uint16_t firmware;
    uint16_t usb_revision;
} fake;

static struct event *queue_event(unsigned char endpoint, int length,
    int result, int transferred)
{
    CHECK(fake.count < MAX_EVENTS);
    CHECK(length >= 0 && length <= DMA_PACKET_BYTES);
    CHECK(transferred >= 0 && transferred <= DMA_PACKET_BYTES);
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
    fake.firmware = 0x0341;
    fake.usb_revision = 0x0200;
}

static void expect_adapter_info(void)
{
    unsigned char selector = 0;
    const unsigned char info[] = {
        fake.firmware & 0xff, fake.firmware >> 8, 0, 0,
    };
    expect_packet(0xca, &selector, 1);
    reply_packet(0xca, info, sizeof(info));
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
    expect_adapter_info();
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

libusb_device *LIBUSB_CALL libusb_get_device(libusb_device_handle *input)
{
    CHECK(input == &handle);
    return &usb_device;
}

int LIBUSB_CALL libusb_get_device_descriptor(libusb_device *input,
    struct libusb_device_descriptor *descriptor)
{
    CHECK(input == &usb_device);
    memset(descriptor, 0, sizeof(*descriptor));
    descriptor->idVendor = 0x1a86;
    descriptor->idProduct = fake.variant_f ? 0x55de : 0x55db;
    descriptor->bcdDevice = fake.usb_revision;
    return 0;
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
    CHECK(ch347_spi_set_mode("dma") == -1);
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

static void expect_dma_frame(const unsigned char *output,
    const unsigned char *input, unsigned int length, unsigned char cs_flags)
{
    CHECK(length != 0 && length <= DMA_PAYLOAD_BYTES);
    struct event *sent = queue_event(0x06, 3 + (int)length, 0, 3 + (int)length);
    sent->bytes[0] = 0xc2;
    sent->bytes[1] = length & 0xff;
    sent->bytes[2] = (length >> 8) | cs_flags;
    memcpy(sent->bytes + 3, output, length);
    struct event *reply = queue_event(0x86, 3 + (int)length, 0, 3 + (int)length);
    reply->bytes[0] = 0xc2;
    reply->bytes[1] = length & 0xff;
    /* Also exercise firmware that echoes the high CS control nibble. */
    reply->bytes[2] = (length >> 8) | cs_flags;
    if (input != NULL) {
        memcpy(reply->bytes + 3, input, length);
    }
}

static void test_normal_complete_transaction(void)
{
    const unsigned char command[] = {0x0f, 0xc0};
    const unsigned char count[] = {1, 0, 0, 0};
    unsigned char output = 0xff;
    reset_fixture(false, 0);
    CHECK(ch347_spi_set_mode("normal") == 0);
    expect_initialization(4 << 3);
    expect_cs(true);
    expect_packet(0xc4, command, sizeof(command));
    reply_status(0xc4, 0);
    expect_packet(0xc3, count, sizeof(count));
    reply_status(0xc3, 0x10);
    expect_cs(false);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_transaction(sizeof(command), 1, command, &output) == 0);
    CHECK(output == 0x10);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(false);
}

static void test_dma_page(void)
{
    const unsigned char identify[] = {0x9f, 0};
    const unsigned char identify_clocks[] = {0x9f, 0, 0, 0};
    const unsigned char identify_reply[] = {0xaa, 0xbb, 0xc9, 0x52};
    const unsigned char load[] = {0x13, 0, 1, 0x23};
    const unsigned char status[] = {0x0f, 0xc0};
    const unsigned char status_clocks[] = {0x0f, 0xc0, 0};
    const unsigned char status_reply[] = {0xaa, 0xbb, 0};
    const unsigned char cache[] = {0x03, 0, 0, 0xff};
    unsigned char cache_clocks[2180] = {0x03, 0, 0, 0xff};
    unsigned char cache_reply[2180];
    unsigned char page[2176];
    unsigned char program_load[3 + 2176];
    unsigned char id[2], observed_status;
    memset(cache_reply, 0xa5, sizeof(cache_reply));
    memset(program_load, 0x5a, sizeof(program_load));
    program_load[0] = 0x02;
    program_load[1] = program_load[2] = 0;
    reset_fixture(false, 1);
    CHECK(ch347_spi_set_mode("dma") == 0);
    expect_initialization(4 << 3);
    /* Each complete command uses one C2 OUT/reply pair, no extra C1s. */
    expect_dma_frame(identify_clocks, identify_reply, 4, 0xc0);
    expect_dma_frame(load, NULL, sizeof(load), 0xc0);
    expect_dma_frame(status_clocks, status_reply, 3, 0xc0);
    expect_dma_frame(cache_clocks, cache_reply, sizeof(cache_clocks), 0xc0);
    expect_dma_frame(program_load, NULL, sizeof(program_load), 0xc0);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_transaction(2, 2, identify, id) == 0);
    CHECK(id[0] == 0xc9 && id[1] == 0x52);
    CHECK(ch347_spi_transaction(sizeof(load), 0, load, NULL) == 0);
    CHECK(ch347_spi_transaction(2, 1, status, &observed_status) == 0);
    CHECK(observed_status == 0);
    CHECK(ch347_spi_transaction(sizeof(cache), sizeof(page), cache, page) == 0);
    CHECK(page[0] == 0xa5 && page[2175] == 0xa5);
    /* A full main+OOB program-load is write-only at the NAND level, but
     * must drain its C2 duplex reply before any following command.
     */
    CHECK(ch347_spi_transaction(sizeof(program_load), 0, program_load, NULL) == 0);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(true);
    CHECK(ch347_spi_set_mode("normal") == 0);
}

static void test_dma_chunk_boundary(void)
{
    unsigned char command[4094];
    unsigned char first_reply[4093];
    const unsigned char last_clocks[] = {0x7b, 0, 0};
    const unsigned char last_reply[] = {0xee, 0xc9, 0x52};
    unsigned char output[2] = {0};
    memset(command, 0x6a, sizeof(command));
    command[4093] = 0x7b;
    memset(first_reply, 0xcc, sizeof(first_reply));
    reset_fixture(false, 0);
    CHECK(ch347_spi_set_mode("dma") == 0);
    expect_initialization(4 << 3);
    expect_dma_frame(command, first_reply, 4093, 0x80);
    expect_dma_frame(last_clocks, last_reply, 3, 0x40);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_transaction(sizeof(command), 2, command, output) == 0);
    CHECK(output[0] == 0xc9 && output[1] == 0x52);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(false);
    CHECK(ch347_spi_set_mode("normal") == 0);
}

static void test_dma_split_usb_reply(void)
{
    const unsigned char command[] = {0x9f, 0};
    const unsigned char clocks[] = {0x9f, 0, 0, 0};
    const unsigned char reply[] = {0xaa, 0xbb, 0xc9, 0x52};
    unsigned char output[2];
    reset_fixture(false, 0);
    CHECK(ch347_spi_set_mode("dma") == 0);
    expect_initialization(4 << 3);
    expect_dma_frame(clocks, reply, 4, 0xc0);
    unsigned char full_reply[7];
    memcpy(full_reply, fake.events[fake.count - 1].bytes, sizeof(full_reply));
    fake.events[fake.count - 1].transferred = 2;
    struct event *rest = queue_event(0x86, 5, 0, 5);
    memcpy(rest->bytes, full_reply + 2, 5);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_transaction(2, 2, command, output) == 0);
    CHECK(output[0] == 0xc9 && output[1] == 0x52);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(false);
    CHECK(ch347_spi_set_mode("normal") == 0);
}

static void test_dma_fault(unsigned int fault)
{
    const unsigned char command[] = {0x9f, 0};
    const unsigned char clocks[] = {0x9f, 0, 0, 0};
    const unsigned char reply[] = {0xaa, 0xbb, 0xc9, 0x52};
    unsigned char output[2] = {0x11, 0x22};
    reset_fixture(false, 1);
    CHECK(ch347_spi_set_mode("dma") == 0);
    expect_initialization(4 << 3);
    expect_dma_frame(clocks, reply, 4, 0xc0);
    struct event *response = &fake.events[fake.count - 1];
    if (fault == 0) {
        response->bytes[0] = 0xc3;
    } else if (fault == 1) {
        response->bytes[1] = 3;
    } else if (fault == 2) {
        response->result = LIBUSB_ERROR_TIMEOUT;
        response->transferred = 4;
    } else if (fault == 3) {
        response->transferred = 0;
    } else {
        fake.count--; /* no IN after a short OUT */
        fake.events[fake.count - 1].transferred--;
    }
    expect_cs(false); /* failure handler */
    expect_cs(false); /* shutdown */
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_transaction(2, 2, command, output) == -1);
    CHECK(output[0] == 0x11 && output[1] == 0x22);
    unsigned int consumed = fake.next;
    CHECK(ch347_spi_transaction(2, 2, command, output) == -1);
    CHECK(fake.next == consumed);
    CHECK(ch347_spi_shutdown() == -1);
    check_finished(true);
    CHECK(ch347_spi_set_mode("normal") == 0);
}

static void test_dma_firmware_gate(void)
{
    reset_fixture(false, 1);
    fake.firmware = 0x0340;
    fake.usb_revision = 0x9999; /* A newer USB number cannot bypass CA gate. */
    CHECK(ch347_spi_set_mode("dma") == 0);
    expect_adapter_info();
    expect_cs(false); /* cleanup; no SPI config or flash commands occurred */
    CHECK(ch347_spi_init() == -1);
    check_finished(true);

    reset_fixture(true, 0);
    fake.firmware = 0x0001; /* WCH supports F independently of T's version. */
    expect_initialization(4 << 3);
    expect_cs(false);
    CHECK(ch347_spi_init() == 0);
    CHECK(ch347_spi_shutdown() == 0);
    check_finished(false);
    CHECK(ch347_spi_set_mode("normal") == 0);
    CHECK(ch347_spi_set_mode("quick") == -1);
    CHECK(ch347_spi_set_mode(NULL) == -1);
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
    test_normal_complete_transaction();
    test_dma_page();
    test_dma_chunk_boundary();
    test_dma_split_usb_reply();
    for (unsigned int fault = 0; fault < 5; fault++) {
        test_dma_fault(fault);
    }
    test_dma_firmware_gate();
    puts("CH347 offline protocol tests passed (no real USB implementation linked).");
    return 0;
}
