/* SPDX-License-Identifier: GPL-2.0-or-later
 * Small offline writer contract test. Link ONLY hyf_nand.c and this file:
 * programmer calls below are mocks, not libusb/CH341/CH347 implementations.
 * Mock parity is a visible placeholder, NOT an ECC/BCH implementation.
 */
#define _POSIX_C_SOURCE 200809L
#include "../hyf_nand.h"
#include "../programmer.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum {
    MAIN_PAGE = 2048, OOB_PAGE = 128, PAGE_BYTES = 2176,
    PAGES = 64, MAIN_BLOCK = 131072, OOB_BLOCK = 8192,
    BASE_BLOCK = 263, BASE_ROW = BASE_BLOCK * PAGES,
};

static unsigned char flash[2 * PAGES][PAGE_BYTES];
static unsigned char cache[PAGE_BYTES];
static unsigned char config, protection, status;
static unsigned char saved_config, saved_protection;
static bool selected, metadata_loads_masked;
static unsigned calls, page_reads, cache_reads, erase_calls, execute_calls;
static unsigned reads_before_first_erase, selected_row;
static unsigned fail_erase_at, fail_program_at, read_ecc_status;
static unsigned fail_erase_transport_at, fail_program_transport_at;
static unsigned char erase_failure_status, program_failure_status;
static bool wrong_id, fail_erase, corrupt_readback;
static bool change_config_on_failure, change_protection_on_failure;
static bool failure_pending, failure_disabled, failure_ready_checked;
static bool failure_config_checked, failure_protection_checked;

static unsigned char input_main(unsigned page, unsigned offset)
{
    return (unsigned char)(page * 13 + offset * 7 + 3);
}

static unsigned char live_main(unsigned page, unsigned offset)
{
    return input_main(page, offset) ^ 0x5a;
}

static unsigned char input_oob(unsigned page, unsigned offset)
{
    /* Valid OEM tags can be zero on the first page: not a bad-block test. */
    if (page == 0 && offset < 8) {
        return 0;
    }

    return (unsigned char)(page * 11 + offset * 5 + 1);
}

static void reset_mock(void)
{
    config = saved_config = 0x01;
    protection = saved_protection = 0x3a;
    status = 0;
    selected = false;
    metadata_loads_masked = true;
    calls = page_reads = cache_reads = erase_calls = execute_calls = 0;
    reads_before_first_erase = selected_row = 0;
    fail_erase_at = fail_program_at = read_ecc_status = 0;
    fail_erase_transport_at = fail_program_transport_at = 0;
    erase_failure_status = 4;
    program_failure_status = 8;
    wrong_id = fail_erase = corrupt_readback = false;
    change_config_on_failure = change_protection_on_failure = false;
    failure_pending = failure_disabled = failure_ready_checked = false;
    failure_config_checked = failure_protection_checked = false;

    for (unsigned page = 0; page < 2 * PAGES; page++) {
        for (unsigned i = 0; i < MAIN_PAGE; i++) {
            flash[page][i] = live_main(page, i);
        }

        memset(flash[page] + MAIN_PAGE, 0xff, OOB_PAGE);
    }
}

static void record_chip_failure(void)
{
    failure_pending = true;
    failure_disabled = failure_ready_checked = false;
    failure_config_checked = failure_protection_checked = false;

    if (change_config_on_failure) {
        config ^= 0x10;
    }
    if (change_protection_on_failure) {
        protection |= 0x08;
    }
}

int programmer_select_chip(bool active)
{
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
    assert(selected);
    assert(write_count != 0 && write_data != NULL);
    calls++;

    switch (write_data[0]) {
    case 0x9f:
        assert(write_count == 2 && read_count == 2);
        read_data[0] = 0xc9;
        read_data[1] = 0x52;
        if (wrong_id) {
            read_data[1] = 0x51;
        }
        break;
    case 0x0f:
        assert(write_count == 2 && read_count == 1);
        if (write_data[1] == 0xa0) {
            *read_data = protection;
            if (failure_pending) {
                failure_protection_checked = true;
            }
        } else if (write_data[1] == 0xb0) {
            *read_data = config;
            if (failure_pending) {
                failure_config_checked = true;
            }
        } else {
            assert(write_data[1] == 0xc0);
            *read_data = status;
            if (failure_pending && failure_disabled && (status & 3) == 0) {
                failure_ready_checked = true;
            }
        }
        break;
    case 0x1f:
        assert(write_count == 3 && read_count == 0);
        if (write_data[1] == 0xa0) {
            protection = write_data[2];
        } else {
            assert(write_data[1] == 0xb0);
            config = write_data[2];
        }
        break;
    case 0x06:
        assert(write_count == 1 && read_count == 0);
        status |= 2;
        break;
    case 0x04:
        assert(write_count == 1 && read_count == 0);
        status &= (unsigned char)~2;
        if (failure_pending) {
            failure_disabled = true;
        }
        break;
    case 0xd8: {
        assert(write_count == 4 && read_count == 0 && (status & 2));
        unsigned row = (unsigned)write_data[1] << 16 |
            (unsigned)write_data[2] << 8 | write_data[3];
        assert(row >= BASE_ROW && row < BASE_ROW + 2 * PAGES && row % PAGES == 0);
        if (failure_pending) {
            /* A failed physical block may be abandoned only after recovery,
             * not by blindly carrying its WEL/configuration into the next.
             */
            assert(failure_disabled && failure_ready_checked);
            assert(failure_config_checked && failure_protection_checked);
            failure_pending = false;
        }
        if (erase_calls == 0) {
            reads_before_first_erase = page_reads;
        }
        erase_calls++;
        if (fail_erase_transport_at != 0 && erase_calls == fail_erase_transport_at) {
            return -1;
        }
        if (fail_erase || (fail_erase_at != 0 && erase_calls == fail_erase_at)) {
            status = erase_failure_status;
            record_chip_failure();
            break;
        }
        memset(flash[row - BASE_ROW], 0xff, PAGES * PAGE_BYTES);
        status = 0;
        break;
    }
    case 0x02:
        assert(write_count == 3 + PAGE_BYTES && read_count == 0 && (status & 2));
        assert(write_data[1] == 0 && write_data[2] == 0);
        memcpy(cache, write_data + 3, PAGE_BYTES);
        if (config & 0x10) {
            for (unsigned i = 0; i < OOB_PAGE; i++) {
                if (i % 32 >= 8 && cache[MAIN_PAGE + i] != 0xff) {
                    metadata_loads_masked = false;
                }
            }
        }
        break;
    case 0x10: {
        assert(write_count == 4 && read_count == 0 && (status & 2));
        unsigned row = (unsigned)write_data[1] << 16 |
            (unsigned)write_data[2] << 8 | write_data[3];
        assert(row >= BASE_ROW && row < BASE_ROW + 2 * PAGES);
        execute_calls++;
        if (fail_program_transport_at != 0 && execute_calls == fail_program_transport_at) {
            return -1;
        }
        status = 0;
        if (fail_program_at != 0 && execute_calls == fail_program_at) {
            status = program_failure_status;
            record_chip_failure();
            break;
        }
        memcpy(flash[row - BASE_ROW], cache, PAGE_BYTES);
        if (config & 0x10) {
            for (unsigned i = 0; i < OOB_PAGE; i++) {
                if (i % 32 >= 8) {
                    flash[row - BASE_ROW][MAIN_PAGE + i] = 0xa5;
                }
            }
        }
        break;
    }
    case 0x13: {
        assert(write_count == 4 && read_count == 0);
        unsigned row = (unsigned)write_data[1] << 16 |
            (unsigned)write_data[2] << 8 | write_data[3];
        assert(row >= BASE_ROW && row < BASE_ROW + 2 * PAGES);
        selected_row = row - BASE_ROW;
        page_reads++;
        status = (unsigned char)(read_ecc_status << 4);
        break;
    }
    case 0x03:
        assert(write_count == 4 && (read_count == MAIN_PAGE || read_count == PAGE_BYTES));
        assert(write_data[1] == 0 && write_data[2] == 0);
        cache_reads++;
        memcpy(read_data, flash[selected_row], read_count);
        if (corrupt_readback) {
            read_data[0] ^= 1;
        }
        break;
    default:
        assert(!"unexpected SPI command");
    }

    return 0;
}

static void check_cleanup(void)
{
    assert(!selected && !(status & 2));
    assert(config == saved_config && protection == saved_protection);
}

static void check_block_contents(unsigned block, bool raw, bool preserve)
{
    assert(block < 2);
    for (unsigned page = block * PAGES; page < (block + 1) * PAGES; page++) {
        for (unsigned i = 0; i < MAIN_PAGE; i++) {
            unsigned char expected = input_main(page, i);
            if (preserve) {
                expected = live_main(page, i);
            }
            assert(flash[page][i] == expected);
        }
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            unsigned char expected = input_oob(page, i);
            if (!raw && i % 32 >= 8) {
                expected = 0xa5;
            }
            assert(flash[page][MAIN_PAGE + i] == expected);
        }
    }
}

static void check_untouched_block(unsigned block)
{
    assert(block < 2);
    for (unsigned page = block * PAGES; page < (block + 1) * PAGES; page++) {
        for (unsigned i = 0; i < MAIN_PAGE; i++) {
            assert(flash[page][i] == live_main(page, i));
        }
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            assert(flash[page][MAIN_PAGE + i] == 0xff);
        }
    }
}

static void check_contents(bool raw, bool preserve)
{
    check_block_contents(0, raw, preserve);
    /* A one-block request must leave its adjacent physical block untouched. */
    check_untouched_block(1);
}

static void path_in(char *path, size_t capacity, const char *directory, const char *name)
{
    int length = snprintf(path, capacity, "%s/%s", directory, name);
    assert(length > 0 && (size_t)length < capacity);
}

/* Sparse full-chip fixtures exercise physical seeks without a full-chip
 * buffer. Slice fixtures start at file offset zero. Each physical block has
 * a distinct page pattern, so skipping cannot accidentally compact inputs.
 */
static void make_input(const char *path, bool main_area, bool full_chip, unsigned blocks)
{
    unsigned char bytes[MAIN_PAGE];
    unsigned page_bytes = main_area ? MAIN_PAGE : OOB_PAGE;
    FILE *stream = fopen(path, "wb");
    assert(stream != NULL);
    assert(blocks != 0 && blocks <= 2);
    if (full_chip) {
        assert(ftruncate(fileno(stream), (off_t)2048 * PAGES * page_bytes) == 0);
        assert(fseeko(stream, (off_t)BASE_BLOCK * PAGES * page_bytes, SEEK_SET) == 0);
    }
    for (unsigned page = 0; page < blocks * PAGES; page++) {
        for (unsigned i = 0; i < page_bytes; i++) {
            bytes[i] = main_area ? input_main(page, i) : input_oob(page, i);
        }
        assert(fwrite(bytes, 1, page_bytes, stream) == page_bytes);
    }
    assert(fclose(stream) == 0);
}

static void check_skip_bad_blocks(const char *main_path, const char *oob_path)
{
    struct hyf_nand_options options = {
        .main_path = main_path, .oob_path = oob_path,
        .address = (uint64_t)BASE_BLOCK * MAIN_BLOCK, .length = 2 * MAIN_BLOCK,
        .skip_bad_blocks = true,
    };

    for (unsigned full_chip = 0; full_chip < 2; full_chip++) {
        make_input(main_path, true, full_chip != 0, 2);
        make_input(oob_path, false, full_chip != 0, 2);
        options.main_path = main_path;
        options.raw = false;
        options.verify = false;

        reset_mock();
        fail_erase_at = 1;
        assert(hyf_nand_write(&options) == 0);
        assert(erase_calls == 2 && execute_calls == PAGES);
        assert(page_reads == 0 && cache_reads == 0 && metadata_loads_masked);
        check_untouched_block(0);
        check_block_contents(1, false, false);
        check_cleanup();

        /* E_FAIL with WEL still set is recoverable too. Only the successfully
         * programmed second block receives optional verification reads.
         */
        reset_mock();
        options.raw = true;
        options.verify = true;
        fail_erase_at = 1;
        erase_failure_status = 6;
        assert(hyf_nand_write(&options) == 0);
        assert(erase_calls == 2 && execute_calls == PAGES);
        assert(page_reads == PAGES && cache_reads == PAGES);
        check_untouched_block(0);
        check_block_contents(1, true, false);
        check_cleanup();

        reset_mock();
        options.raw = false;
        options.verify = false;
        fail_program_at = 4;
        assert(hyf_nand_write(&options) == 0);
        assert(erase_calls == 2 && execute_calls == PAGES + 4);
        assert(page_reads == 0 && cache_reads == 0);
        check_block_contents(1, false, false);
        check_cleanup();

        /* OOB-only writes must preserve each block's own main area. These
         * 128 reads are preservation, not a marker scan or verification.
         */
        reset_mock();
        options.main_path = NULL;
        fail_erase_at = 1;
        assert(hyf_nand_write(&options) == 0);
        assert(erase_calls == 2 && execute_calls == PAGES);
        assert(page_reads == 2 * PAGES && cache_reads == 2 * PAGES);
        assert(reads_before_first_erase == PAGES);
        check_untouched_block(0);
        check_block_contents(1, false, true);
        check_cleanup();

        reset_mock();
        options.raw = true;
        fail_program_at = 4;
        assert(hyf_nand_write(&options) == 0);
        assert(erase_calls == 2 && execute_calls == PAGES + 4);
        assert(page_reads == 2 * PAGES && cache_reads == 2 * PAGES);
        check_block_contents(1, true, true);
        check_cleanup();
    }

    /* The final loop left sparse full-chip inputs, valid for either selected
     * length. Even a one-block request with no programmed blocks can complete
     * with a reported skip, while the default remains fail-fast.
     */
    options.main_path = main_path;
    options.raw = false;
    options.length = MAIN_BLOCK;
    reset_mock();
    fail_erase_at = 1;
    erase_failure_status = 6;
    assert(hyf_nand_write(&options) == 0);
    assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
    check_untouched_block(0);
    check_untouched_block(1);
    check_cleanup();

    reset_mock();
    fail_program_at = 4;
    assert(hyf_nand_write(&options) == 0);
    assert(erase_calls == 1 && execute_calls == 4 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    options.length = 2 * MAIN_BLOCK;
    options.skip_bad_blocks = false;
    reset_mock();
    fail_erase_at = 1;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    reset_mock();
    fail_program_at = 4;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 4 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    /* Neither an uncertain SPI result nor a later verification mismatch is
     * a completed NAND E_FAIL/P_FAIL, even with skipping explicitly enabled.
     */
    options.skip_bad_blocks = true;
    reset_mock();
    fail_erase_transport_at = 1;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    reset_mock();
    fail_program_transport_at = 4;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 4 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    reset_mock();
    options.verify = true;
    corrupt_readback = true;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == PAGES);
    assert(page_reads == 1 && cache_reads == 1);
    check_untouched_block(1);
    check_cleanup();
    options.verify = false;

    for (unsigned changed_feature = 0; changed_feature < 2; changed_feature++) {
        reset_mock();
        fail_erase_at = 1;
        change_config_on_failure = changed_feature == 0;
        change_protection_on_failure = changed_feature == 1;
        assert(hyf_nand_write(&options) != 0);
        assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
        check_untouched_block(1);
        check_cleanup();
    }

    /* WEL by itself is not a chip-declared failure that permits a skip. */
    reset_mock();
    fail_erase_at = 1;
    erase_failure_status = 2;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();

    reset_mock();
    fail_program_at = 4;
    program_failure_status = 2;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 4 && page_reads == 0);
    check_untouched_block(1);
    check_cleanup();
}

int main(void)
{
    char directory[] = "/tmp/snander-hyf-test-XXXXXX";
    char main_path[512], oob_path[512], bad_path[512];
    assert(mkdtemp(directory) != NULL);
    path_in(main_path, sizeof(main_path), directory, "main.bin");
    path_in(oob_path, sizeof(oob_path), directory, "oob.bin");
    path_in(bad_path, sizeof(bad_path), directory, "bad.bin");
    FILE *main_file = fopen(main_path, "wb");
    FILE *oob_file = fopen(oob_path, "wb");
    FILE *bad_file = fopen(bad_path, "wb");
    assert(main_file && oob_file && bad_file);

    for (unsigned page = 0; page < PAGES; page++) {
        unsigned char bytes[MAIN_PAGE];
        for (unsigned i = 0; i < MAIN_PAGE; i++) {
            bytes[i] = input_main(page, i);
        }
        assert(fwrite(bytes, 1, MAIN_PAGE, main_file) == MAIN_PAGE);
        for (unsigned i = 0; i < OOB_PAGE; i++) {
            bytes[i] = input_oob(page, i);
        }
        assert(fwrite(bytes, 1, OOB_PAGE, oob_file) == OOB_PAGE);
    }
    assert(fputc(0, bad_file) != EOF);
    assert(fclose(main_file) == 0 && fclose(oob_file) == 0 && fclose(bad_file) == 0);

    struct hyf_nand_options options = {
        .main_path = main_path, .oob_path = oob_path,
        .address = (uint64_t)BASE_BLOCK * MAIN_BLOCK, .length = MAIN_BLOCK,
    };

    reset_mock();
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == 0 && cache_reads == 0);
    assert(erase_calls == 1 && execute_calls == PAGES && metadata_loads_masked);
    check_contents(false, false);
    check_cleanup();

    reset_mock();
    options.verify = true;
    corrupt_readback = true;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == PAGES && page_reads == 1);
    check_cleanup();

    reset_mock();
    options.verify = true;
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == PAGES && cache_reads == PAGES && reads_before_first_erase == 0);
    check_contents(false, false);
    check_cleanup();

    reset_mock();
    options.raw = true;
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == PAGES && cache_reads == PAGES);
    check_contents(true, false);
    check_cleanup();

    reset_mock();
    options.raw = false;
    options.verify = false;
    options.main_path = NULL;
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == PAGES && cache_reads == PAGES && reads_before_first_erase == PAGES);
    check_contents(false, true);
    check_cleanup();

    reset_mock();
    read_ecc_status = 2;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 0 && execute_calls == 0 && page_reads == 1);
    check_cleanup();

    reset_mock();
    options.main_path = main_path;
    fail_program_at = 4;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 4 && page_reads == 0);
    check_cleanup();

    reset_mock();
    fail_erase = true;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 1 && execute_calls == 0 && page_reads == 0);
    check_cleanup();

    reset_mock();
    config = saved_config = 0x41;
    status = 2;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 0 && execute_calls == 0 && page_reads == 0);
    check_cleanup();

    reset_mock();
    wrong_id = true;
    assert(hyf_nand_write(&options) != 0);
    assert(erase_calls == 0 && execute_calls == 0 && page_reads == 0 && !selected);

    reset_mock();
    options.main_path = bad_path;
    assert(hyf_nand_write(&options) != 0 && calls == 0);
    options.main_path = main_path;

    reset_mock();
    options.dry_run = true;
    assert(hyf_nand_write(&options) == 0 && calls == 0);
    options.dry_run = false;

    make_input(main_path, true, true, 1);
    reset_mock();
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == 0 && erase_calls == 1 && execute_calls == PAGES);
    check_contents(false, false); /* full main, independently mapped OOB slice */
    check_cleanup();

    make_input(oob_path, false, true, 1);
    reset_mock();
    assert(hyf_nand_write(&options) == 0);
    assert(page_reads == 0 && erase_calls == 1 && execute_calls == PAGES);
    check_contents(false, false); /* both full-chip inputs seek to block 263 */
    check_cleanup();

    check_skip_bad_blocks(main_path, oob_path);

    reset_mock();
    options.address = (uint64_t)MAIN_BLOCK * 2047;
    options.length = MAIN_BLOCK * 2;
    assert(hyf_nand_write(&options) != 0 && calls == 0);
    options.address = (uint64_t)MAIN_BLOCK * 2048;
    options.length = MAIN_BLOCK;
    assert(hyf_nand_write(&options) != 0 && calls == 0);
    options.address = 1;
    assert(hyf_nand_write(&options) != 0 && calls == 0);
    options.address = 0;
    options.length = UINT64_MAX;
    assert(hyf_nand_write(&options) != 0 && calls == 0);

    assert(unlink(main_path) == 0 && unlink(oob_path) == 0 && unlink(bad_path) == 0);
    assert(rmdir(directory) == 0);
    puts("HYF offline mock contract cases passed (no USB/hardware linked).");
    return 0;
}
