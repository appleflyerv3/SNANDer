/* SPDX-License-Identifier: GPL-2.0-or-later
 * Bounded offline raw main/OOB reader test. Link ONLY hyf_nand.c and this
 * file: the programmer below is a fake, with no libusb or hardware backend.
 * It generates individual cache pages, never a full NAND image.
 */
#define _POSIX_C_SOURCE 200809L
#include "../hyf_nand.h"
#include "../programmer.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>

enum {
    MAIN_PAGE = 2048, OOB_PAGE = 128, PAGE_BYTES = MAIN_PAGE + OOB_PAGE,
    CHIP_BYTES = 0x10000000,
    FIRST_ROW = 263 * 64 + 3, TEST_PAGES = 3, BUFFER_PAGES = 33,
    OOB_BUFFER_PAGES = 64 * 1024 / OOB_PAGE + 1,
};

static bool selected, wrong_id, interrupt_busy, paired_mode;
static unsigned char config, saved_config;
static unsigned calls, selections, config_writes, page_loads, cache_reads;
static unsigned cache_read_bytes;
static unsigned status_polls, busy_responses, busy_per_page, busy_left;
static unsigned first_row, row_count, loaded_row;
static unsigned fail_load_at, fail_cache_at, fail_config_at;
static unsigned interrupt_cache_at;
static int interrupt_signal;
static const char *observed_main_path, *observed_oob_path;
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

static unsigned char main_byte(unsigned row, unsigned offset)
{
    return (unsigned char)(row * 19 + offset * 11 + 9);
}

static void reset_mock(unsigned start, unsigned count)
{
    selected = wrong_id = interrupt_busy = paired_mode = false;
    config = saved_config = 0x15;
    calls = selections = config_writes = page_loads = cache_reads = 0;
    cache_read_bytes = 0;
    status_polls = busy_responses = busy_per_page = busy_left = 0;
    first_row = start;
    row_count = count;
    loaded_row = 0;
    fail_load_at = fail_cache_at = fail_config_at = interrupt_cache_at = 0;
    interrupt_signal = SIGTERM;
    observed_main_path = observed_oob_path = NULL;
}

int programmer_select_chip(bool active)
{
    selections++;
    selected = active;
    return 0;
}

int programmer_transaction(unsigned int write_count, unsigned int read_count,
    const unsigned char *write_data, unsigned char *read_data)
{
    if (programmer_select_chip(true) < 0) {
        (void)programmer_select_chip(false);
        return -1;
    }

    int result = programmer_send_command(write_count, read_count, write_data, read_data);
    int released = programmer_select_chip(false);

    return result < 0 || released < 0 ? -1 : 0;
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
            status_polls++;
            *read_data = 2;
            if (interrupt_busy && page_loads != 0) {
                interrupt_busy = false;
                *read_data |= 1;
                busy_responses++;
                assert(raise(SIGINT) == 0);
            } else if (busy_left != 0) {
                busy_left--;
                busy_responses++;
                *read_data |= 1;
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
        busy_left = busy_per_page;
        if (page_loads == fail_load_at) {
            return -1;
        }
        break;
    }
    case 0x03:
        assert(write_count == 4 && read_count == (paired_mode ? PAGE_BYTES : OOB_PAGE));
        assert(write_data[1] == (paired_mode ? 0 : 0x08));
        assert(write_data[2] == 0 && write_data[3] == 0xff);
        assert(page_loads == cache_reads + 1 && !(config & 0x10));
        cache_reads++;
        cache_read_bytes += read_count;
        if (observed_main_path != NULL && (cache_reads == 2 || cache_reads == BUFFER_PAGES)) {
            struct stat main_info, oob_info;
            assert(stat(observed_main_path, &main_info) == 0);
            assert(stat(observed_oob_path, &oob_info) == 0);
            /* The first page stays buffered; the 32-page main batch is on
             * disk before reading page33. OOB remains in its own buffer.
             */
            assert(main_info.st_size == (cache_reads == 2 ? 0 : 64 * 1024));
            assert(oob_info.st_size == 0);
        }
        if (cache_reads == fail_cache_at) {
            return -1;
        }
        unsigned char *spare = read_data;
        if (paired_mode) {
            for (unsigned i = 0; i < MAIN_PAGE; i++) {
                read_data[i] = main_byte(loaded_row, i);
            }
            spare += MAIN_PAGE;
        }
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            spare[i] = oob_byte(loaded_row, i);
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

static void check_main_output(const char *path, unsigned start, unsigned byte_count)
{
    struct stat info;
    FILE *stream = fopen(path, "rb");

    assert(stream != NULL && fstat(fileno(stream), &info) == 0);
    assert(info.st_size == (off_t)byte_count);
    for (unsigned offset = 0; offset < byte_count; offset++) {
        unsigned row = start + offset / MAIN_PAGE;
        assert(fgetc(stream) == main_byte(row, offset % MAIN_PAGE));
    }
    assert(fgetc(stream) == EOF && !ferror(stream));
    assert(fclose(stream) == 0);
}

int main(void)
{
    char directory[] = "/tmp/snander-hyf-oob-test-XXXXXX";
    char output_path[512], target_path[512], main_path[512];
    char alias_path[512], parent_link[512];
    struct sigaction action, old_interrupt, old_terminate;
    struct sigaction old_file_signal;
    struct rlimit original_limit, file_limit;

    assert(mkdtemp(directory) != NULL);
    int size = snprintf(output_path, sizeof(output_path), "%s/oob.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(output_path));
    size = snprintf(target_path, sizeof(target_path), "%s/target.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(target_path));
    size = snprintf(main_path, sizeof(main_path), "%s/main.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(main_path));
    size = snprintf(parent_link, sizeof(parent_link), "%s/linked-parent", directory);
    assert(size > 0 && (size_t)size < sizeof(parent_link));
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
    assert(cache_read_bytes == TEST_PAGES * OOB_PAGE);
    check_cleanup();
    check_output(output_path, FIRST_ROW, TEST_PAGES);
    assert(unlink(output_path) == 0);

    /* Every page can require several busy polls, not just one ready read.
     * Count initial + per-page + restoration polls, without sleeping.
     */
    reset_mock(FIRST_ROW, TEST_PAGES);
    busy_per_page = 3;
    assert(hyf_nand_read_oob(&options) == 0);
    assert(busy_responses == TEST_PAGES * 3);
    assert(status_polls == 1 + TEST_PAGES * 4 + 1);
    assert(page_loads == TEST_PAGES && cache_reads == TEST_PAGES);
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

    /* Paired mode obtains main and OOB from exactly one 2176-byte cache
     * transfer for each physical row, then emits two separate streams.
     */
    options.main_path = main_path;
    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = true;
    assert(hyf_nand_read(&options) == 0);
    assert(page_loads == TEST_PAGES && cache_reads == TEST_PAGES && config_writes == 2);
    assert(cache_read_bytes == TEST_PAGES * PAGE_BYTES);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, TEST_PAGES * MAIN_PAGE);
    check_output(output_path, FIRST_ROW, TEST_PAGES);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    /* Cross the explicit 64-KiB main buffer boundary with only33 generated
     * pages. Verify the bulk write while capture is still in progress.
     */
    options.length = BUFFER_PAGES * MAIN_PAGE;
    reset_mock(FIRST_ROW, BUFFER_PAGES);
    paired_mode = true;
    observed_main_path = main_path;
    observed_oob_path = output_path;
    assert(hyf_nand_read(&options) == 0);
    assert(page_loads == BUFFER_PAGES && cache_reads == BUFFER_PAGES);
    assert(cache_read_bytes == BUFFER_PAGES * PAGE_BYTES);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, BUFFER_PAGES * MAIN_PAGE);
    check_output(output_path, FIRST_ROW, BUFFER_PAGES);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    /* The buffered tail of an interrupted capture is flushed too. Already
     * captured complete pages survive, but the operation still fails.
     */
    reset_mock(FIRST_ROW, BUFFER_PAGES);
    paired_mode = true;
    observed_main_path = main_path;
    observed_oob_path = output_path;
    interrupt_cache_at = BUFFER_PAGES;
    assert(hyf_nand_read(&options) != 0);
    assert(page_loads == BUFFER_PAGES && cache_reads == BUFFER_PAGES);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, BUFFER_PAGES * MAIN_PAGE);
    check_output(output_path, FIRST_ROW, BUFFER_PAGES);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);
    options.length = TEST_PAGES * MAIN_PAGE;

    /* Also cross the less frequent OOB buffer boundary, with only about
     * 1 MiB of generated main output, never a full-chip fixture.
     */
    options.length = OOB_BUFFER_PAGES * MAIN_PAGE;
    reset_mock(FIRST_ROW, OOB_BUFFER_PAGES);
    paired_mode = true;
    assert(hyf_nand_read(&options) == 0);
    assert(page_loads == OOB_BUFFER_PAGES && cache_reads == OOB_BUFFER_PAGES);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, OOB_BUFFER_PAGES * MAIN_PAGE);
    check_output(output_path, FIRST_ROW, OOB_BUFFER_PAGES);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);
    options.length = TEST_PAGES * MAIN_PAGE;

    /* The full-chip plan is host-only; no giant fixture or transfer loop. */
    reset_mock(FIRST_ROW, TEST_PAGES);
    options.address = 0;
    options.length = 0;
    options.dry_run = true;
    assert(hyf_nand_read(&options) == 0 && calls == 0 && selections == 0);
    check_absent(main_path);
    check_absent(output_path);
    options.address = (uint64_t)FIRST_ROW * MAIN_PAGE;
    options.length = TEST_PAGES * MAIN_PAGE;
    options.dry_run = false;

    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) != 0 && calls == 0 && selections == 0);
    check_absent(main_path);
    check_absent(output_path);

    reset_mock(FIRST_ROW, TEST_PAGES);
    options.main_path = output_path;
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    size = snprintf(alias_path, sizeof(alias_path), "%s/./oob.bin", directory);
    assert(size > 0 && (size_t)size < sizeof(alias_path));
    options.main_path = alias_path;
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    assert(symlink(directory, parent_link) == 0);
    size = snprintf(alias_path, sizeof(alias_path), "%s/oob.bin", parent_link);
    assert(size > 0 && (size_t)size < sizeof(alias_path));
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    assert(unlink(parent_link) == 0);
    options.main_path = main_path;

    /* If EITHER output already exists, do not create the other one. */
    existing = fopen(main_path, "wb");
    assert(existing != NULL && fputs("keep", existing) >= 0 && fclose(existing) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    check_absent(output_path);
    assert(unlink(main_path) == 0);
    existing = fopen(output_path, "wb");
    assert(existing != NULL && fputs("keep", existing) >= 0 && fclose(existing) == 0);
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    check_absent(main_path);
    assert(unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = wrong_id = true;
    assert(hyf_nand_read(&options) != 0 && page_loads == 0 && config_writes == 0);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, 0);
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = true;
    fail_cache_at = 2;
    assert(hyf_nand_read(&options) != 0 && page_loads == 2 && cache_reads == 2);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, MAIN_PAGE);
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = true;
    interrupt_cache_at = 1;
    assert(hyf_nand_read(&options) != 0 && page_loads == 1 && cache_reads == 1);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, MAIN_PAGE);
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = interrupt_busy = true;
    assert(hyf_nand_read(&options) != 0 && page_loads == 1 && cache_reads == 0);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, 0);
    check_output(output_path, FIRST_ROW, 0);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    /* All three pages fit in the host buffers. At cleanup the file limit
     * permits only128 bytes of EACH bulk output. Preserve these unequal-to-
     * accepted lengths, flush the other output too, and report failure.
     */
    assert(getrlimit(RLIMIT_FSIZE, &original_limit) == 0);
    file_limit = original_limit;
    file_limit.rlim_cur = OOB_PAGE;
    memset(&action, 0, sizeof(action));
    action.sa_handler = SIG_IGN;
    sigemptyset(&action.sa_mask);
    assert(sigaction(SIGXFSZ, &action, &old_file_signal) == 0);
    assert(setrlimit(RLIMIT_FSIZE, &file_limit) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = true;
    assert(hyf_nand_read(&options) != 0 && cache_reads == TEST_PAGES);
    assert(setrlimit(RLIMIT_FSIZE, &original_limit) == 0);
    assert(sigaction(SIGXFSZ, &old_file_signal, NULL) == 0);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, OOB_PAGE);
    check_output(output_path, FIRST_ROW, 1);
    assert(unlink(main_path) == 0 && unlink(output_path) == 0);

    /* Permit exactly one extra descriptor: first output opens, second open
     * fails with EMFILE. This exercises the creation-race/error cleanup
     * without relying on directory permissions (tests may run as root).
     */
    struct rlimit original_descriptors, descriptor_limit;
    int probe = open("/dev/null", O_RDONLY);
    assert(probe >= 0 && getrlimit(RLIMIT_NOFILE, &original_descriptors) == 0);
    descriptor_limit = original_descriptors;
    descriptor_limit.rlim_cur = (rlim_t)probe + 1;
    assert(close(probe) == 0 && setrlimit(RLIMIT_NOFILE, &descriptor_limit) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    paired_mode = true;
    assert(hyf_nand_read(&options) != 0 && calls == 0 && selections == 0);
    assert(setrlimit(RLIMIT_NOFILE, &original_descriptors) == 0);
    check_cleanup();
    check_main_output(main_path, FIRST_ROW, 0);
    check_absent(output_path);
    assert(unlink(main_path) == 0);
    options.main_path = NULL;

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

    /* Force a real host bulk-write error: three pages were accepted into
     * the buffer but only one page fits on disk. Restore limit/handler.
     */
    assert(getrlimit(RLIMIT_FSIZE, &original_limit) == 0);
    file_limit = original_limit;
    file_limit.rlim_cur = OOB_PAGE;
    memset(&action, 0, sizeof(action));
    action.sa_handler = SIG_IGN;
    sigemptyset(&action.sa_mask);
    assert(sigaction(SIGXFSZ, &action, &old_file_signal) == 0);
    assert(setrlimit(RLIMIT_FSIZE, &file_limit) == 0);
    reset_mock(FIRST_ROW, TEST_PAGES);
    assert(hyf_nand_read_oob(&options) != 0 && cache_reads == TEST_PAGES);
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
    puts("HYF raw main/OOB reader mock cases passed (no USB/hardware linked).");
    return 0;
}
