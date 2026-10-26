/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Hardware-free checks for the legacy NAND command batching. Including the
 * implementation exercises its private protocol routines without exporting
 * test-only interfaces. Link this file alone: its SPI/timer fakes replace all
 * external dependencies; no programmer or libusb implementation is linked.
 *
 * From src, use the legacy module's normal warning/language flags:
 * cc -O1 -std=gnu99 -Wall -I. tests/test_spi_nand_batch.c \
 *     -o /tmp/test_spi_nand_batch
 */
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

#include "../spi_nand_flash.c"

enum event_kind { SELECT, WRITE_BYTES, READ_BYTES, DESELECT };
enum { MAX_EVENTS = 24, MAX_PREFIX = 5 };

static struct {
    struct {
        enum event_kind kind;
        u32 length;
        SPI_CONTROLLER_SPEED_T speed;
        u8 bytes[MAX_PREFIX];
        u8 read_value;
    } events[MAX_EVENTS];
    unsigned int count;
    unsigned int next;
    unsigned int writes;
    unsigned int reads;
    unsigned int minimum_ch347_calls;
    bool selected;
} fake;

static void reset_fixture(void)
{
    CHECK(!fake.selected);
    memset(&fake, 0, sizeof(fake));
    memset(&_current_flash_info_t, 0, sizeof(_current_flash_info_t));
    _current_flash_info_t.mfr_id = 0xc9;
    _current_flash_info_t.dev_id = 0x52;
    _current_flash_info_t.page_size = 2048;
    _current_flash_info_t.oob_size = 128;
    _current_flash_info_t.dummy_mode = SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND;
    _current_flash_info_t.read_mode = SPI_NAND_FLASH_READ_SPEED_MODE_DUAL;
    _current_page_num = 0xffffffff;
    _plane_select_bit = 0;
    ECC_fcheck = 1;
    ECC_ignore = 0;
}

static void queue_event(enum event_kind kind, const u8 *bytes, u32 length,
    SPI_CONTROLLER_SPEED_T speed, u8 read_value)
{
    CHECK(fake.count < MAX_EVENTS);
    unsigned int index = fake.count++;
    fake.events[index].kind = kind;
    fake.events[index].length = length;
    fake.events[index].speed = speed;
    fake.events[index].read_value = read_value;
    if (kind == WRITE_BYTES) {
        CHECK(length <= MAX_PREFIX);
        memcpy(fake.events[index].bytes, bytes, length);
    }
}

static void expect_transaction(const u8 *prefix, u32 length, u32 read_length,
    SPI_CONTROLLER_SPEED_T speed, u8 read_value)
{
    queue_event(SELECT, NULL, 0, SPI_CONTROLLER_SPEED_SINGLE, 0);
    queue_event(WRITE_BYTES, prefix, length, SPI_CONTROLLER_SPEED_SINGLE, 0);
    if (read_length != 0) {
        queue_event(READ_BYTES, NULL, read_length, speed, read_value);
    }
    queue_event(DESELECT, NULL, 0, SPI_CONTROLLER_SPEED_SINGLE, 0);
}

static void consume_event(enum event_kind kind)
{
    CHECK(fake.next < fake.count);
    CHECK(fake.events[fake.next].kind == kind);
    fake.next++;
}

static void check_finished(void)
{
    CHECK(fake.next == fake.count);
    CHECK(!fake.selected);
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Enable_Manual_Mode(void)
{
    return SPI_CONTROLLER_RTN_NO_ERROR;
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_Low(void)
{
    CHECK(!fake.selected);
    consume_event(SELECT);
    fake.selected = true;
    fake.minimum_ch347_calls++;
    return SPI_CONTROLLER_RTN_NO_ERROR;
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Chip_Select_High(void)
{
    CHECK(fake.selected);
    consume_event(DESELECT);
    fake.selected = false;
    fake.minimum_ch347_calls++;
    return SPI_CONTROLLER_RTN_NO_ERROR;
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_One_Byte(u8 data)
{
    (void)data;
    /* Every write in the tested read path must now use the grouped helper. */
    CHECK(0);
    return SPI_CONTROLLER_RTN_WRITE_DATAPFIFO_ERROR;
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Write_NByte(u8 *data, u32 length,
    SPI_CONTROLLER_SPEED_T speed)
{
    CHECK(fake.selected);
    unsigned int index = fake.next;
    consume_event(WRITE_BYTES);
    CHECK(length == fake.events[index].length);
    CHECK(speed == fake.events[index].speed);
    CHECK(memcmp(data, fake.events[index].bytes, length) == 0);
    fake.writes++;
    /* Model the current backend: one C4 OUT plus its required status IN.
     * This is a call-count model, not a USB/hardware performance test.
     */
    fake.minimum_ch347_calls += 2;
    return SPI_CONTROLLER_RTN_NO_ERROR;
}

SPI_CONTROLLER_RTN_T SPI_CONTROLLER_Read_NByte(u8 *data, u32 length,
    SPI_CONTROLLER_SPEED_T speed)
{
    CHECK(fake.selected);
    unsigned int index = fake.next;
    consume_event(READ_BYTES);
    CHECK(length == fake.events[index].length);
    CHECK(speed == fake.events[index].speed);
    memset(data, fake.events[index].read_value, length);
    fake.reads++;
    fake.minimum_ch347_calls += 1 + (length + 506) / 507;
    return SPI_CONTROLLER_RTN_NO_ERROR;
}

void timer_start(void) {}
void timer_end(void) {}
int timer_progress(void) { return 0; }

static void test_get_feature(void)
{
    const u8 prefix[] = {0x0f, 0xc0};
    u8 status = 0;
    reset_fixture();
    expect_transaction(prefix, sizeof(prefix), 1, SPI_CONTROLLER_SPEED_SINGLE, 0x30);
    CHECK(spi_nand_protocol_get_feature(0xc0, &status) == SPI_NAND_FLASH_RTN_NO_ERROR);
    CHECK(status == 0x30);
    CHECK(fake.writes == 1 && fake.reads == 1);
    CHECK(fake.minimum_ch347_calls == 6);
    check_finished();
}

static void test_cache_prefixes(void)
{
    static const struct {
        u32 offset;
        u32 feature;
        u8 plane;
        SPI_NAND_FLASH_READ_DUMMY_BYTE_T dummy;
        SPI_NAND_FLASH_READ_SPEED_MODE_T mode;
        SPI_CONTROLLER_SPEED_T speed;
        u8 prefix[MAX_PREFIX];
        u32 length;
        bool read;
    } cases[] = {
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_DUAL, SPI_CONTROLLER_SPEED_DUAL,
         {0x03, 0x12, 0x34, 0xff}, 4, true},
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_SINGLE, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0x12, 0x34, 0xff}, 4, true},
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_PREPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_SINGLE, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0xff, 0x12, 0x34}, 4, true},
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_PREPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_DUAL, SPI_CONTROLLER_SPEED_DUAL,
         {0x03, 0xff, 0x12, 0x34, 0xff}, 5, true},
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_PREPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_QUAD, SPI_CONTROLLER_SPEED_QUAD,
         {0x03, 0xff, 0x12, 0x34, 0xff}, 5, true},
        {0x1a34, SPI_NAND_FLASH_PLANE_SELECT_HAVE, 0,
         SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_SINGLE, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0x0a, 0x34, 0xff}, 4, true},
        {0x0a34, SPI_NAND_FLASH_PLANE_SELECT_HAVE, 1,
         SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_SINGLE, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0x1a, 0x34, 0xff}, 4, true},
        {0x1a34, SPI_NAND_FLASH_PLANE_SELECT_HAVE, 0,
         SPI_NAND_FLASH_READ_DUMMY_BYTE_PREPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_QUAD, SPI_CONTROLLER_SPEED_QUAD,
         {0x03, 0xff, 0x0a, 0x34, 0xff}, 5, true},
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_DEF_NO,
         SPI_NAND_FLASH_READ_SPEED_MODE_SINGLE, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0x12, 0x34}, 3, true},
        /* Preserve the original no-data/default branch too; no new guard. */
        {0x1234, 0, 0, SPI_NAND_FLASH_READ_DUMMY_BYTE_APPEND,
         SPI_NAND_FLASH_READ_SPEED_MODE_DEF_NO, SPI_CONTROLLER_SPEED_SINGLE,
         {0x03, 0x12, 0x34, 0xff}, 4, false},
    };
    for (unsigned int index = 0; index < sizeof(cases) / sizeof(cases[0]); index++) {
        u8 output[8] = {0};
        reset_fixture();
        _current_flash_info_t.feature = cases[index].feature;
        _plane_select_bit = cases[index].plane;
        expect_transaction(cases[index].prefix, cases[index].length,
            cases[index].read ? sizeof(output) : 0, cases[index].speed, 0xa5);
        CHECK(spi_nand_protocol_read_from_cache(cases[index].offset,
            sizeof(output), output, cases[index].mode, cases[index].dummy) ==
            SPI_NAND_FLASH_RTN_NO_ERROR);
        CHECK(fake.writes == 1);
        CHECK(fake.reads == (unsigned int)cases[index].read);
        CHECK(output[0] == (cases[index].read ? 0xa5 : 0));
        check_finished();
    }
}

static void test_hyf_ecc_decode(void)
{
    reset_fixture();
    CHECK(ecc_fail_check(0x123, 0x00) == SPI_NAND_FLASH_RTN_NO_ERROR);
    CHECK(ecc_fail_check(0x123, 0x10) == SPI_NAND_FLASH_RTN_NO_ERROR);
    CHECK(ecc_fail_check(0x123, 0x20) == SPI_NAND_FLASH_RTN_DETECTED_BAD_BLOCK);
    CHECK(ecc_fail_check(0x123, 0x30) == SPI_NAND_FLASH_RTN_NO_ERROR);
    CHECK(fake.writes == 0 && fake.reads == 0);
    check_finished();
}

static void test_hyf_page(u8 status, bool busy_first)
{
    const u8 load[] = {0x13, 0x00, 0x01, 0x23};
    const u8 feature[] = {0x0f, 0xc0};
    const u8 cache[] = {0x03, 0x00, 0x00, 0xff};
    reset_fixture();
    expect_transaction(load, sizeof(load), 0, SPI_CONTROLLER_SPEED_SINGLE, 0);
    if (busy_first) {
        expect_transaction(feature, sizeof(feature), 1,
            SPI_CONTROLLER_SPEED_SINGLE, 1);
    }
    expect_transaction(feature, sizeof(feature), 1,
        SPI_CONTROLLER_SPEED_SINGLE, status);
    /* Original behavior reads cache even when ECC says uncorrectable. */
    expect_transaction(cache, sizeof(cache), 2176, SPI_CONTROLLER_SPEED_DUAL, 0x5a);
    CHECK(spi_nand_read_page(0x123, SPI_NAND_FLASH_READ_SPEED_MODE_DUAL) ==
        (status == 0x20 ? SPI_NAND_FLASH_RTN_DETECTED_BAD_BLOCK :
                         SPI_NAND_FLASH_RTN_NO_ERROR));
    CHECK(fake.writes == (busy_first ? 4u : 3u));
    CHECK(fake.reads == (busy_first ? 3u : 2u));
    CHECK(fake.minimum_ch347_calls == (busy_first ? 26u : 20u));
    CHECK(_current_cache_page_data[0] == 0x5a);
    CHECK(_current_cache_page_oob[127] == 0x5a);
    check_finished();
}

int main(void)
{
    test_get_feature();
    test_cache_prefixes();
    test_hyf_ecc_decode();
    test_hyf_page(0x00, false);
    test_hyf_page(0x10, true);
    test_hyf_page(0x20, false);
    test_hyf_page(0x30, false);
    puts("Legacy NAND batching tests passed (fake SPI only).");
    return 0;
}
