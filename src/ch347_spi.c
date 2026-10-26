/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2022 Nicholas Chin <nic.c3.14@gmail.com>
 *
 * SNANDer adaptation of the flashrom CH347 bulk SPI transport:
 * https://github.com/flashrom/flashrom/blob/93546a16f2e096eab3f67a015dc7487f81a25e29/programmers/ch347_spi.c
 *
 * Changes: explicit transaction CS for SNANDer's fragmented commands,
 * configuration read/modify/write, strict reply/USB-length validation,
 * bounded buffers, failed-session latching and complete error cleanup.
 *
 * Configuration/status meanings are also corroborated by WCH's GPL-2.0+
 * driver/ch34x_mphsi_master_spi.c in WCHSoftGroup/ch34x_mphsi_master_linux:
 * GET_CFG is CA/01/00/01, config payload is 26 bytes, and INIT/OUT status
 * byte zero means success. No proprietary vendor library is linked.
 *
 * Optional DMA C2 transport is adapted from WCH's GPL-2.0+
 * driver, Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.:
 * https://github.com/WCHSoftGroup/ch34x_mphsi_master_linux/tree/57be7ed01e2adb14d34bcca5e1eb71ff6247aba7
 * ch34x_mphsi_master_spi.c: ch347_spi_native_dma (medium mode), plus
 * ch347_spi_native and ch347_spi_transfer_one_message for manual C1/C2 CS;
 * ch34x_mphsi_master_usb.c: ch347_get_chipinfo;
 * ch34x_mphsi.h: 4093-byte C2 payload limit. E1 is clock control and
 * E2 is CH347F function switching; neither enables DMA here.
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include "ch347_spi.h"

#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

enum {
    CH347_VENDOR_ID = 0x1a86,
    CH347T_PRODUCT_ID = 0x55db,
    CH347F_PRODUCT_ID = 0x55de,
    CH347T_INTERFACE = 2,
    CH347F_INTERFACE = 4,
    CH347_WRITE_EP = 0x06,
    CH347_READ_EP = 0x86,
    CH347_CMD_SET_CONFIG = 0xc0,
    CH347_CMD_CHIP_SELECT = 0xc1,
    CH347_CMD_DUPLEX = 0xc2,
    CH347_CMD_READ = 0xc3,
    CH347_CMD_WRITE = 0xc4,
    CH347_CMD_GET_CONFIG = 0xca,
    CH347_HEADER_BYTES = 3,
    /* Both established libusb implementations use 510, not USB's 512.
     * The command and LE16 payload length leave 507 data bytes per frame.
     */
    CH347_PACKET_BYTES = 510,
    CH347_DATA_BYTES = CH347_PACKET_BYTES - CH347_HEADER_BYTES,
    CH347_CONFIG_BYTES = 26,
    CH347_USB_TIMEOUT_MS = 10000,
    CH347_MAX_CLOCK_HZ = 60000000,
    CH347_DEFAULT_DIVISOR = 4,
    CH347_DMA_PACKET_BYTES = 4096,
    CH347_DMA_DATA_BYTES = CH347_DMA_PACKET_BYTES - CH347_HEADER_BYTES,
    CH347_DMA_MAX_TRANSFER = 64 * 1024 - 16 * CH347_HEADER_BYTES,
    CH347T_DMA_MIN_FIRMWARE = 0x0341,
};

/* A private libusb context avoids sharing the CH341 driver's default context.
 * The selected interface is only the bulk SPI function, never either UART.
 */
static struct {
    libusb_context *usb;
    libusb_device_handle *handle;
    int interface;
    uint16_t product_id;
    uint16_t usb_revision;
    uint16_t firmware;
    bool claimed;
    bool detached;
    bool selected;
    bool failed;
} device;

static unsigned int clock_divisor = CH347_DEFAULT_DIVISOR;
static bool dma_mode;

static int dma_transfer(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr, bool automatic_cs);

static void put_le16(uint8_t *bytes, unsigned int value)
{
    bytes[0] = value & 0xff;
    bytes[1] = (value >> 8) & 0xff;
}

static int usb_write(const uint8_t *bytes, unsigned int length)
{
    int transferred = 0;
    int result = libusb_bulk_transfer(device.handle, CH347_WRITE_EP,
        (unsigned char *)bytes, (int)length, &transferred,
        CH347_USB_TIMEOUT_MS);

    if (result != 0 || transferred != (int)length) {
        fprintf(stderr, "CH347 USB write failed: %s, %d of %u bytes.\n",
            libusb_error_name(result), transferred, length);
        /* Never resend a partial command: its SPI effects may already exist. */
        return -1;
    }

    return 0;
}

static int write_packet(uint8_t command, const uint8_t *payload,
    unsigned int length)
{
    uint8_t packet[CH347_PACKET_BYTES];

    if (length > CH347_DATA_BYTES) {
        fprintf(stderr, "CH347 packet exceeds its bounded payload.\n");
        return -1;
    }

    packet[0] = command;
    put_le16(packet + 1, length);

    if (length != 0) {
        memcpy(packet + CH347_HEADER_BYTES, payload, length);
    }

    return usb_write(packet, CH347_HEADER_BYTES + length);
}

/* One device frame, not an unbounded stream parser. A successful USB return
 * can still be short, and a timeout can follow partial I/O. Neither is a
 * successful/retryable SPI command. Reject zero progress and excess data.
 */
static int read_packet(uint8_t command, uint8_t *payload,
    unsigned int capacity, unsigned int *payload_length)
{
    uint8_t packet[CH347_PACKET_BYTES];
    int transferred = 0;
    int result = libusb_bulk_transfer(device.handle, CH347_READ_EP,
        packet, sizeof(packet), &transferred, CH347_USB_TIMEOUT_MS);

    if (result != 0) {
        fprintf(stderr, "CH347 USB read failed: %s, %d bytes received.\n",
            libusb_error_name(result), transferred);
        return -1;
    }

    if (transferred < CH347_HEADER_BYTES ||
        transferred > CH347_PACKET_BYTES || packet[0] != command) {
        fprintf(stderr, "CH347 returned an invalid reply header.\n");
        return -1;
    }

    unsigned int length = packet[1] | ((unsigned int)packet[2] << 8);

    if (length == 0 || length > capacity || length > CH347_DATA_BYTES ||
        length + CH347_HEADER_BYTES != (unsigned int)transferred) {
        fprintf(stderr, "CH347 returned an invalid reply length.\n");
        return -1;
    }

    memcpy(payload, packet + CH347_HEADER_BYTES, length);
    *payload_length = length;
    return 0;
}

static int read_status(uint8_t command)
{
    uint8_t status;
    unsigned int length;

    if (read_packet(command, &status, sizeof(status), &length) != 0) {
        return -1;
    }

    if (length != 1 || status != 0) {
        fprintf(stderr, "CH347 command 0x%02x returned an error status.\n",
            command);
        return -1;
    }

    return 0;
}

/* CONTROL has two five-byte CS entries. Bit7 applies the selected entry;
 * bit6 is its inactive level. Delays and automatic deselect stay disabled.
 * CS1's all-zero entry means do not change it. CONTROL has no reply packet.
 */
static int set_chip_select_raw(bool active)
{
    uint8_t control[10] = {0};

    if (active) {
        control[0] = 0x80;
    } else {
        control[0] = 0xc0;
    }

    if (write_packet(CH347_CMD_CHIP_SELECT, control, sizeof(control)) != 0) {
        return -1;
    }

    device.selected = active;
    return 0;
}

static int fail_session(void)
{
    device.failed = true;

    /* Best effort only: after a USB failure the actual CS level is unknown.
     * Latching prevents legacy callers that ignore results from issuing more
     * opcodes or retrying a partly delivered erase/program operation.
     */
    if (device.claimed) {
        (void)set_chip_select_raw(false);
    }

    return -1;
}

int ch347_spi_set_clock(unsigned int hz)
{
    if (device.usb != NULL) {
        fprintf(stderr, "Set the CH347 SPI clock before initialization.\n");
        return -1;
    }

    for (unsigned int divisor = 0; divisor <= 7; divisor++) {
        if (hz == ((unsigned int)CH347_MAX_CLOCK_HZ >> divisor)) {
            clock_divisor = divisor;
            return 0;
        }
    }

    fprintf(stderr, "Unsupported CH347 SPI clock %u Hz; use exactly "
        "60000000, 30000000, 15000000, 7500000, 3750000, 1875000, "
        "937500 or 468750.\n", hz);
    return -1;
}

int ch347_spi_set_mode(const char *mode)
{
    if (device.usb != NULL) {
        fprintf(stderr, "Set the CH347 transfer mode before initialization.\n");
        return -1;
    }

    if (mode != NULL && strcmp(mode, "normal") == 0) {
        dma_mode = false;
        return 0;
    }

    if (mode != NULL && strcmp(mode, "dma") == 0) {
        dma_mode = true;
        return 0;
    }

    fprintf(stderr, "Unknown CH347 mode; use normal or dma.\n");
    return -1;
}

static int identify_adapter(void)
{
    struct libusb_device_descriptor descriptor;
    libusb_device *usb_device = libusb_get_device(device.handle);

    if (usb_device == NULL ||
        libusb_get_device_descriptor(usb_device, &descriptor) != 0) {
        fprintf(stderr, "Cannot read the CH347 USB descriptor.\n");
        return -1;
    }

    if (descriptor.idVendor != CH347_VENDOR_ID ||
        descriptor.idProduct != device.product_id) {
        fprintf(stderr, "CH347 USB descriptor identity mismatch.\n");
        return -1;
    }

    device.usb_revision = descriptor.bcdDevice;

    /* WCH queries selector zero for four chip-info bytes. The first two
     * encode firmware LE16; bcdDevice is a separate USB descriptor value.
     */
    uint8_t selector = 0;
    uint8_t info[4];
    unsigned int length;

    if (write_packet(CH347_CMD_GET_CONFIG, &selector, 1) != 0 ||
        read_packet(CH347_CMD_GET_CONFIG, info, sizeof(info), &length) != 0 ||
        length != sizeof(info)) {
        fprintf(stderr, "Cannot obtain a complete CH347 firmware revision.\n");
        return -1;
    }

    device.firmware = info[0] | ((uint16_t)info[1] << 8);
    printf("CH347 USB bcdDevice: 0x%04x; adapter firmware: 0x%04x; mode: %s.\n",
        device.usb_revision, device.firmware, dma_mode ? "dma" : "normal");

    if (dma_mode && device.product_id == CH347T_PRODUCT_ID &&
        device.firmware < CH347T_DMA_MIN_FIRMWARE) {
        fprintf(stderr, "CH347T DMA requires firmware >=0x0341 according to WCH; "
            "use --ch347-mode normal for this adapter.\n");
        return -1;
    }

    return 0;
}

static int configure_spi(void)
{
    uint8_t config[CH347_CONFIG_BYTES];
    uint8_t query = 1;
    unsigned int length;

    /* Preserve vendor-reserved bytes and unrelated misc/I2C settings instead
     * of inventing values for fields that older drivers called "mystery".
     */
    if (write_packet(CH347_CMD_GET_CONFIG, &query, sizeof(query)) != 0 ||
        read_packet(CH347_CMD_GET_CONFIG, config, sizeof(config), &length) != 0) {
        return -1;
    }

    if (length != sizeof(config)) {
        fprintf(stderr, "CH347 configuration reply is incomplete.\n");
        return -1;
    }

    /* This is the wire layout of the vendor's 26-byte SPI hardware config,
     * encoded explicitly so host packing and endianness cannot change it.
     */
    put_le16(config + 0, 0x0000); /* two-line full duplex */
    put_le16(config + 2, 0x0104); /* master */
    put_le16(config + 4, 0x0000); /* eight-bit words */
    put_le16(config + 6, 0x0000); /* CPOL low */
    put_le16(config + 8, 0x0000); /* first-edge sampling: SPI mode 0 */
    put_le16(config + 10, 0x0200); /* software-controlled CS */
    put_le16(config + 12, clock_divisor << 3);
    put_le16(config + 14, 0x0000); /* MSB first, no CH341 bit swapping */
    put_le16(config + 18, 0x0000); /* no inter-byte delay */
    config[20] = 0; /* vendor default MOSI byte for block reads */
    config[21] &= 0x3f; /* both CS polarities active-low; keep other bits */

    if (write_packet(CH347_CMD_SET_CONFIG, config, sizeof(config)) != 0 ||
        read_status(CH347_CMD_SET_CONFIG) != 0) {
        return -1;
    }

    return set_chip_select_raw(false);
}

int ch347_spi_init(void)
{
    static const struct {
        uint16_t product_id;
        int interface;
        const char *name;
    } variants[] = {
        {CH347T_PRODUCT_ID, CH347T_INTERFACE, "CH347T (bulk mode 1)"},
        {CH347F_PRODUCT_ID, CH347F_INTERFACE, "CH347F"},
    };
    const char *name = NULL;
    int result;

    if (device.usb != NULL) {
        fprintf(stderr, "CH347 is already initialized.\n");
        return -1;
    }

    result = libusb_init(&device.usb);

    if (result != 0) {
        fprintf(stderr, "Could not initialize libusb: %s.\n",
            libusb_error_name(result));
        device.usb = NULL;
        return -1;
    }

    for (unsigned int index = 0;
         index < sizeof(variants) / sizeof(variants[0]); index++) {
        device.handle = libusb_open_device_with_vid_pid(device.usb,
            CH347_VENDOR_ID, variants[index].product_id);

        if (device.handle != NULL) {
            device.interface = variants[index].interface;
            device.product_id = variants[index].product_id;
            name = variants[index].name;
            break;
        }
    }

    if (device.handle == NULL) {
        fprintf(stderr, "Could not open CH347 bulk SPI (1a86:55db or "
            "1a86:55de). Check USB permissions and the hardware mode. "
            "HID/JTAG/UART-only modes are not supported.\n");
        (void)ch347_spi_shutdown();
        return -1;
    }

    result = libusb_kernel_driver_active(device.handle, device.interface);

    if (result == 1) {
        result = libusb_detach_kernel_driver(device.handle, device.interface);

        if (result != 0) {
            fprintf(stderr, "Could not detach CH347 SPI interface: %s.\n",
                libusb_error_name(result));
            (void)ch347_spi_shutdown();
            return -1;
        }

        device.detached = true;
    } else if (result < 0 && result != LIBUSB_ERROR_NOT_SUPPORTED) {
        fprintf(stderr, "Could not inspect CH347 SPI interface: %s.\n",
            libusb_error_name(result));
        (void)ch347_spi_shutdown();
        return -1;
    }

    result = libusb_claim_interface(device.handle, device.interface);

    if (result != 0) {
        fprintf(stderr, "Could not claim CH347 SPI interface %d: %s.\n",
            device.interface, libusb_error_name(result));
        (void)ch347_spi_shutdown();
        return -1;
    }

    device.claimed = true;

    if (identify_adapter() != 0 || configure_spi() != 0) {
        device.failed = true;
        (void)ch347_spi_shutdown();
        return -1;
    }

    printf("Found programmer device: WCH %s, SPI mode 0, MSB first, "
        "CS0 active-low, %u Hz.\n", name,
        (unsigned int)CH347_MAX_CLOCK_HZ >> clock_divisor);
    return 0;
}

int ch347_spi_select_chip(bool active)
{
    if (!device.claimed || (device.failed && active)) {
        return -1;
    }

    if (set_chip_select_raw(active) != 0) {
        return fail_session();
    }

    if (device.failed) {
        return -1;
    }

    return 0;
}

int ch347_spi_send_command(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr)
{
    if (!device.claimed || device.failed) {
        return -1;
    }

    if (writecnt == 0 && readcnt == 0) {
        return 0;
    }

    if (!device.selected || (writecnt != 0 && writearr == NULL) ||
        (readcnt != 0 && readarr == NULL)) {
        fprintf(stderr, "CH347 SPI transfer requires asserted CS0 and "
            "valid data buffers.\n");
        return fail_session();
    }

    if (dma_mode) {
        /* Keep the caller's C1 CS edges, including empty transactions. WCH's
         * normal duplex path and DMA continuation frames both use C2 without
         * automatic-CS bits. Each fragment completes before returning, so
         * status reads and program/erase command ordering remain unchanged.
         */
        return dma_transfer(writecnt, readcnt, writearr, readarr, false);
    }

    while (writecnt != 0) {
        unsigned int length = writecnt;

        if (length > CH347_DATA_BYTES) {
            length = CH347_DATA_BYTES;
        }

        if (write_packet(CH347_CMD_WRITE, writearr, length) != 0 ||
            read_status(CH347_CMD_WRITE) != 0) {
            return fail_session();
        }

        writearr += length;
        writecnt -= length;
    }

    if (readcnt != 0) {
        uint8_t count[4];

        /* Block-read's payload is a LE32 total count, not a LE16 byte
         * count accidentally truncated to one USB frame or flash page.
         */
        count[0] = readcnt & 0xff;
        count[1] = (readcnt >> 8) & 0xff;
        count[2] = (readcnt >> 16) & 0xff;
        count[3] = (readcnt >> 24) & 0xff;

        if (write_packet(CH347_CMD_READ, count, sizeof(count)) != 0) {
            return fail_session();
        }
    }

    while (readcnt != 0) {
        unsigned int length;
        unsigned int capacity = readcnt;

        if (capacity > CH347_DATA_BYTES) {
            capacity = CH347_DATA_BYTES;
        }

        if (read_packet(CH347_CMD_READ, readarr, capacity, &length) != 0) {
            return fail_session();
        }

        readarr += length;
        readcnt -= length;
    }

    /* CS stays asserted until the caller's explicit Chip_Select_High. */
    return 0;
}

static int monotonic_milliseconds(uint64_t *milliseconds)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        fprintf(stderr, "Cannot read the CH347 DMA reply deadline clock.\n");
        return -1;
    }

    *milliseconds = (uint64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
    return 0;
}

/* WCH's bulk helper accumulates IN fragments of one DMA reply until the
 * requested total is present. A short USB completion is not another C2
 * frame: do not issue SPI again or expect an extra header in the middle.
 * All fragments share one deadline; progress cannot restart the timeout.
 */
static int read_dma_reply(uint8_t *packet, unsigned int payload_bytes)
{
    unsigned int received = 0;
    unsigned int expected = CH347_HEADER_BYTES + payload_bytes;
    uint64_t now;

    if (monotonic_milliseconds(&now) != 0) {
        return -1;
    }

    uint64_t deadline = now + CH347_USB_TIMEOUT_MS;

    while (received < expected) {
        if (monotonic_milliseconds(&now) != 0) {
            return -1;
        }

        if (now >= deadline) {
            fprintf(stderr, "CH347 DMA reply deadline expired.\n");
            return -1;
        }

        int transferred = 0;
        int result = libusb_bulk_transfer(device.handle, CH347_READ_EP,
            packet + received, (int)(expected - received), &transferred,
            (unsigned int)(deadline - now));

        if (result != 0 || transferred <= 0 ||
            transferred > (int)(expected - received)) {
            fprintf(stderr, "CH347 DMA USB read failed: %s, %d bytes.\n",
                libusb_error_name(result), transferred);
            return -1;
        }

        received += (unsigned int)transferred;
    }

    /* The upper nibble is CS control, not part of the 12-bit C2 length.
     * Do not invent an echo/zero requirement for those response control bits:
     * WCH validates opcode and total length, not their returned values.
     */
    unsigned int length = packet[1] | ((unsigned int)(packet[2] & 0x0f) << 8);

    if (packet[0] != CH347_CMD_DUPLEX || length != payload_bytes) {
        fprintf(stderr, "CH347 DMA returned an invalid opcode/payload length.\n");
        return -1;
    }

    return 0;
}

static int dma_transfer(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr, bool automatic_cs)
{
    uint8_t output[CH347_DMA_PACKET_BYTES];
    uint8_t input[CH347_DMA_PACKET_BYTES];
    bool first = true;

    /* The complete-transaction API matches WCH's medium-mode transfer limit.
     * Legacy callers may stream larger fragments under their own C1 edges;
     * decrement separate counts instead of adding them and risking overflow.
     */
    if (automatic_cs && (readcnt > CH347_DMA_MAX_TRANSFER ||
        writecnt > CH347_DMA_MAX_TRANSFER - readcnt)) {
        fprintf(stderr, "CH347 DMA transaction exceeds %u SPI bytes.\n",
            (unsigned int)CH347_DMA_MAX_TRANSFER);
        return fail_session();
    }

    while (writecnt != 0 || readcnt != 0) {
        unsigned int prefix = writecnt;

        if (prefix > CH347_DMA_DATA_BYTES) {
            prefix = CH347_DMA_DATA_BYTES;
        }

        unsigned int reads = readcnt;

        if (reads > CH347_DMA_DATA_BYTES - prefix) {
            reads = CH347_DMA_DATA_BYTES - prefix;
        }

        unsigned int length = prefix + reads;
        output[0] = CH347_CMD_DUPLEX;
        put_le16(output + 1, length);

        if (automatic_cs && first) {
            output[2] |= 0x80; /* CS0 active on the first frame only. */
        }

        if (automatic_cs && prefix == writecnt && reads == readcnt) {
            output[2] |= 0x40; /* CS0 inactive after the final frame. */
        }

        if (prefix != 0) {
            memcpy(output + CH347_HEADER_BYTES, writearr, prefix);
        }

        /* Match configure_spi's zero default MOSI byte during reads. Full
         * duplex receives prefix garbage too; only the following read bytes
         * belong to the caller. Write-only transactions still drain replies.
         */
        memset(output + CH347_HEADER_BYTES + prefix, 0, length - prefix);
        device.selected = true;

        if (usb_write(output, CH347_HEADER_BYTES + length) != 0 ||
            read_dma_reply(input, length) != 0) {
            return fail_session();
        }

        if (prefix != 0) {
            writearr += prefix;
        }

        if (reads != 0) {
            memcpy(readarr, input + CH347_HEADER_BYTES + prefix, reads);
            readarr += reads;
        }

        writecnt -= prefix;
        readcnt -= reads;
        first = false;
    }

    if (automatic_cs) {
        device.selected = false;
    }

    return 0;
}

int ch347_spi_transaction(unsigned int writecnt, unsigned int readcnt,
    const unsigned char *writearr, unsigned char *readarr)
{
    if (!device.claimed || device.failed) {
        return -1;
    }

    if (device.selected || (writecnt != 0 && writearr == NULL) ||
        (readcnt != 0 && readarr == NULL)) {
        fprintf(stderr, "CH347 transaction needs inactive CS and valid data buffers.\n");
        return fail_session();
    }

    if (writecnt == 0 && readcnt == 0) {
        return 0;
    }

    if (dma_mode) {
        return dma_transfer(writecnt, readcnt, writearr, readarr, true);
    }

    if (ch347_spi_select_chip(true) != 0) {
        return -1;
    }

    int result = ch347_spi_send_command(writecnt, readcnt, writearr, readarr);
    int released = ch347_spi_select_chip(false);

    return result == 0 && released == 0 ? 0 : -1;
}

int ch347_spi_shutdown(void)
{
    int result = 0;

    if (device.failed) {
        result = -1;
    }

    if (device.handle != NULL) {
        if (device.claimed) {
            if (set_chip_select_raw(false) != 0) {
                result = -1;
            }

            if (libusb_release_interface(device.handle, device.interface) != 0) {
                fprintf(stderr, "Could not release CH347 SPI interface.\n");
                result = -1;
            }
        }

        if (device.detached &&
            libusb_attach_kernel_driver(device.handle, device.interface) != 0) {
            fprintf(stderr, "Could not reattach the CH347 SPI driver.\n");
            result = -1;
        }

        libusb_close(device.handle);
    }

    if (device.usb != NULL) {
        libusb_exit(device.usb);
    }

    memset(&device, 0, sizeof(device));
    return result;
}
