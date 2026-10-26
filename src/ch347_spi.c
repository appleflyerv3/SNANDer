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
 */
#include "ch347_spi.h"

#include <libusb-1.0/libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
};

/* A private libusb context avoids sharing the CH341 driver's default context.
 * The selected interface is only the bulk SPI function, never either UART.
 */
static struct {
    libusb_context *usb;
    libusb_device_handle *handle;
    int interface;
    bool claimed;
    bool detached;
    bool selected;
    bool failed;
} device;

static unsigned int clock_divisor = CH347_DEFAULT_DIVISOR;

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

    if (configure_spi() != 0) {
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
