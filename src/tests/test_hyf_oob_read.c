/* SPDX-License-Identifier: GPL-2.0-or-later
 * Bounded offline OOB-reader contract test. Link ONLY hyf_nand.c and this
 * file: the programmer below is a fake, with no libusb or hardware backend.
 * It generates individual 128-byte OOB pages, never a full NAND image.
 */
#define _POSIX_C_SOURCE 200809L
#include "../hyf_nand.h"
#include "../programmer.h"

#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

enum {
    MAIN_PAGE = 2048, OOB_PAGE = 128, CHIP_BYTES = 0x10000000,
    FIRST_ROW = 263 * 64 + 3, TEST_PAGES = 3,
};

static bool selected, wrong_id, interrupt_busy;
static unsigned char config, saved_config;
static unsigned calls, selections, config_writes, page_loads, cache_reads;
static unsigned first_row, row_count, loaded_row;
static unsigned fail_load_at, fail_cache_at, fail_config_at;
static unsigned interrupt_cache_at;
static int interrupt_signal;
static volatile sig_atomic_t restored_handler_calls;

static void previous_handler(int signal_number)
{
    (void)signal_number;
    restored_handler_calls++;
}

static unsigned char oob_byte(unsigned row, unsigned offset)
{
    return (unsigned char)(row * 13 + offset * 7 + 5);
}

static void reset_mock(unsigned start, unsigned count)
{
    selected = wrong_id = interrupt_busy = false;
    config = saved_config = 0x15;
    calls = selections = config_writes = page_loads = cache_reads = 0;
    first_row = start;
    row_count = count;
    loaded_row = 0;
    fail_load_at = fail_cache_at = fail_config_at = interrupt_cache_at = 0;
    interrupt_signal = SIGTERM;
}

int programmer_select_chip(bool active)
{
    selections++;
    selected = active;
    return 0;
}

int programmer_send_command(unsigned int write_count, unsigned int read_count,
    const unsigned char *write_data, unsigned char *read_data)
{
    assert(selected && write_count != 0 && write_data != NULL);
    calls++;

    switch (write_data[0]) {
    case 0x9f:
        assert(write_count == 2 && write_data[1] == 0 && read_count == 2);
        read_data[0] = 0xc9;
        read_data[1] = wrong_id ? 0x51 : 0x52;
        break;
    case 0x0f:
        assert(write_count == 2 && read_count == 1);
        if (write_data[1] == 0xb0) {
            *read_data = config;
        } else {
            /* Reading A0 is not part of an OOB capture either. A preexisting
             * WEL remains set throughout: the reader must not send WRDI.
             */
            assert(write_data[1] == 0xc0);
            *read_data = 2;
            if (interrupt_busy && page_loads != 0) {
                interrupt_busy = false;
                *read_data |= 1;
                assert(raise(SIGINT) == 0);
            }
        }
        break;
    case 0x1f:
        assert(write_count == 3 && read_count == 0 && write_data[1] == 0xb0);
        assert(write_data[2] == saved_config ||
               write_data[2] == (saved_config & (unsigned char)~0x10));
        config_writes++;
        config = write_data[2];
        if (config_writes == fail_config_at) {
            /* The command took effect despite an uncertain USB result. */
            return -1;
        }
        break;
    case 0x13: {
        assert(write_count == 4 && read_count == 0 && !(config & 0x10));
        unsigned row = (unsigned)write_data[1] << 16 |
            (unsigned)write_data[2] << 8 | write_data[3];
        assert(page_loads < row_count && row == first_row + page_loads);
        loaded_row = row;
        page_loads++;
        if (page_loads == fail_load_at) {
            return -1;
        }
        break;
    }
    case 0x03:
        assert(write_count == 4 && read_count == OOB_PAGE);
        assert(write_data[1] == 0x08 && write_data[2] == 0 && write_data[3] == 0xff);
        assert(page_loads == cache_reads + 1 && !(config & 0x10));
        cache_reads++;
        if (cache_reads == fail_cache_at) {
            return -1;
        }
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            read_data[i] = oob_byte(loaded_row, i);
        }
        if (cache_reads == interrupt_cache_at) {
            assert(raise(interrupt_signal) == 0);
        }
        break;
    default:
        /* In particular, forbid 06/04 WREN/WRDI, 02/84 program loads,
         * 10 program execute, D8 erase and any reset command.
         */
        assert(!"non-read-only SPI opcode in OOB reader");
    }

    return 0;
}

static void check_cleanup(void)
{
    struct sigaction action;

    assert(!selected && config == saved_config);
    assert(sigaction(SIGINT, NULL, &action) == 0 && action.sa_handler == previous_handler);
    assert(sigaction(SIGTERM, NULL, &action) == 0 && action.sa_handler == previous_handler);
}

static void check_output(const char *path, unsigned start, unsigned count)
{
    struct stat info;
    unsigned char bytes[OOB_PAGE];
    FILE *stream = fopen(path, "rb");

    assert(stream != NULL && fstat(fileno(stream), &info) == 0);
    assert(info.st_size == (off_t)count * OOB_PAGE);
    for (unsigned page = 0; page < count; page++) {
        assert(fread(bytes, 1, sizeof(bytes), stream) == sizeof(bytes));
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            assert(bytes[i] == oob_byte(start + page, i));
        }
    }
    assert(fgetc(stream) == EOF && !ferror(stream));
    assert(fclose(stream) == 0);
}

static void check_absent(const char *path)
{
    struct stat info;

    assert(lstat(path, &info) < 0 && errno == ENOENT);
}

int main(void)
{
    char directory[] = "/tmp/snander-hyf-oob-test-XXXXXX";
    char output_path[512], target_path[512];
    struct sigaction action, old_interrupt, old_terminate;

    assert(mkdtemp(directory) != NULL);
    int size = snprintf(output_path, sizeof(output_path), "%s/oob.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(output_path));
    size = snprintf(target_path, sizeof(target_path), "%s/target.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(target_path));
    memset(&action, 0, sizeof(action));
    action.sa_handler = previous_handler;
    sigemptyset(&action.sa_mask);
    assert(sigaction(SIGINT, &action, &old_interrupt) == 0);
    assert(sigaction(SIGTERM, &action, &old_terminate) == 0);

    struct hyf_nand_read_options options = {
        .oob_path = output_path,
        .address = (uint64_t)FIRST_ROW * MAIN_PAGE,
        .length = TEST_PAGES * MAIN_PAGE,
    };

    /* A deliberately non-block-aligned range must keep exact physical rows,
     * OOB byte order and selected-range output size (no main-data prefix).
     */
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) == 0);
    assert(page_loads == TEST_PAGES && cache_reads == TEST_PAGES && config_writes == 2);
    check_cleanup();
    check_output(output_path, FIRST_ROW, TEST_PAGES);
    assert(unlink(output_path) == 0);

    /* Length zero resolves to the remainder, including exactly the last
     * page. Already-disabled ECC requires no B0 write at all.
     */
    reset_mock(CHIP_BYTES / MAIN_PAGE - 1, 1);
    config = saved_config = 0x05;
    options.address = CHIP_BYTES - MAIN_PAGE;
    options.length = 0;
    assert(hyf_nand_read_oob(&options) == 0);
    assert(page_loads == 1 && cache_reads == 1 && config_writes == 0);
    check_cleanup();
    check_output(output_path, CHIP_BYTES / MAIN_PAGE - 1, 1);
    assert(unlink(output_path) == 0);
    options.address = (uint64_t)FIRST_ROW * MAIN_PAGE;
    options.length = TEST_PAGES * MAIN_PAGE;

    reset_mock(FIRST_ROW, TEST_PAGES);
    options.dry_run = true;
    assert(hyf_nand_read_oob(&options) == 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    check_cleanup();
    options.dry_run = false;

    FILE *existing = fopen(output_path, "wb");
    assert(existing != NULL && fputs("keep", existing) >= 0 && fclose(existing) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    existing = fopen(output_path, "rb");
    char sentinel[5] = {0};
    assert(existing != NULL && fread(sentinel, 1, 4, existing) == 4);
    assert(strcmp(sentinel, "keep") == 0 && fclose(existing) == 0);
    assert(unlink(output_path) == 0);

    assert(symlink(target_path, output_path) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    check_absent(target_path);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    wrong_id = true;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 0 && config_writes == 0);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    config = saved_config = 0x55; /* OTP enabled: do not alter configuration. */
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 0 && config_writes == 0);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    fail_config_at = 1;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 0 && config_writes == 2);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    fail_config_at = 2; /* Even complete data cannot hide a cleanup failure. */
    assert(hyf_nand_read_oob(&options) != 0 && cache_reads == TEST_PAGES);
    check_cleanup();
    check_output(output_path, FIRST_ROW, TEST_PAGES);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    fail_load_at = 2;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 2 && cache_reads == 1);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    fail_cache_at = 2;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 2 && cache_reads == 2);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(output_path) == 0);

    /* SIGTERM after one complete cache transfer keeps that page. SIGINT
     * while the first page is busy skips its cache read and restores B0.
     */
    reset_mock(FIRST_ROW, TEST_PAGES);
    interrupt_cache_at = 1;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 1 && cache_reads == 1);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    interrupt_busy = true;
    assert(hyf_nand_read_oob(&options) != 0 && page_loads == 1 && cache_reads == 0);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(output_path) == 0);

    /* Force a real host write error after one page, without filling a disk
     * or using a device file. Restore both the resource limit and handler.
     */
    struct rlimit original_limit, file_limit;
    struct sigaction old_file_signal;

    assert(getrlimit(RLIMIT_FSIZE, &original_limit) == 0);
    file_limit = original_limit;
    file_limit.rlim_cur = OOB_PAGE;
    memset(&action, 0, sizeof(action));
    action.sa_handler = SIG_IGN;
    sigemptyset(&action.sa_mask);
    assert(sigaction(SIGXFSZ, &action, &old_file_signal) == 0);
    assert(setrlimit(RLIMIT_FSIZE, &file_limit) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) != 0 && cache_reads == 2);
    assert(setrlimit(RLIMIT_FSIZE, &original_limit) == 0);
    assert(sigaction(SIGXFSZ, &old_file_signal, NULL) == 0);
    check_cleanup();
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    options.address++;
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    options.address--;
    options.length = MAIN_PAGE + 1;
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    options.length = UINT64_MAX;
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    options.address = CHIP_BYTES - MAIN_PAGE;
    options.length = 2 * MAIN_PAGE;
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    options.address = CHIP_BYTES;
    options.length = 0;
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    check_cleanup();

    assert(restored_handler_calls == 0); /* Signals used only the reader's handler. */
    assert(raise(SIGINT) == 0 && restored_handler_calls == 1);
    assert(raise(SIGTERM) == 0 && restored_handler_calls == 2);
    assert(sigaction(SIGINT, &old_interrupt, NULL) == 0);
    assert(sigaction(SIGTERM, &old_terminate, NULL) == 0);
    assert(rmdir(directory) == 0);
    puts("HYF OOB-reader offline mock cases passed (no USB/hardware linked).");
    return 0;
}
