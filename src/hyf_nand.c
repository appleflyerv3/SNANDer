/* SPDX-License-Identifier: GPL-2.0-or-later
 * HYF2GQ4UAACAE physical main/OOB writer.
 * Primary reference: HY SPI 2G v2.7, pp9-14,20-22,26,28-34:
 * https://datasheet.lcsc.com/datasheet/pdf/cb04af556e5204a6f44a4350d72f54c9.pdf
 * The PX2131 OEM uses spare metadata as content; no generic bad-block marker
 * interpretation is safe for these same-device input images. See header.
 */
#define _POSIX_C_SOURCE 200809L
#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS 64
#endif

#include "hyf_nand.h"
#include "programmer.h"

#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

enum {
    MAIN_PAGE = 2048,
    SPARE_PAGE = 128,
    PAGE_BYTES = MAIN_PAGE + SPARE_PAGE,
    PAGES_PER_BLOCK = 64,
    MAIN_BLOCK = MAIN_PAGE * PAGES_PER_BLOCK,
    SPARE_BLOCK = SPARE_PAGE * PAGES_PER_BLOCK,
    BLOCK_COUNT = 2048,
    MAIN_CAPACITY = MAIN_BLOCK * BLOCK_COUNT,
    SPARE_CAPACITY = SPARE_BLOCK * BLOCK_COUNT,
    FEATURE_LOCK = 0xa0,
    FEATURE_CONFIG = 0xb0,
    FEATURE_STATUS = 0xc0,
    CONFIG_ECC = 0x10,
    CONFIG_OTP = 0x40,
    LOCK_BP_MASK = 0x38,
    STATUS_BUSY = 0x01,
    STATUS_WEL = 0x02,
    STATUS_ERASE_FAIL = 0x04,
    STATUS_PROGRAM_FAIL = 0x08,
};

struct input_file {
    FILE *stream;
    const char *path;
    bool full_chip;
};

struct workspace {
    unsigned char main[MAIN_BLOCK];
    unsigned char oob[SPARE_BLOCK];
    unsigned char command[3 + PAGE_BYTES];
    unsigned char readback[PAGE_BYTES];
};

/* A command-line operation owns the programmer exclusively. Interrupts may
 * stop before the next erase, but never intentionally abandon a block just
 * erased while its complete replacement is already resident in workspace.
 */
static volatile sig_atomic_t stop_requested;

static void request_stop(int signal_number)
{
    stop_requested = signal_number;
}

static int transfer(const unsigned char *command, unsigned write_count,
                    unsigned char *result, unsigned read_count)
{
    if (programmer_select_chip(true) < 0) {
        (void)programmer_select_chip(false);
        return -1;
    }

    int status = programmer_send_command(write_count, read_count, command, result);
    int released = programmer_select_chip(false);

    if (status < 0 || released < 0) {
        fprintf(stderr, "HYF SPI transfer/chip-select failed.\n");
        return -1;
    }

    return 0;
}

static int get_feature(unsigned char address, unsigned char *value)
{
    const unsigned char command[] = {0x0f, address};

    return transfer(command, sizeof(command), value, 1);
}

static int set_feature(unsigned char address, unsigned char value)
{
    const unsigned char command[] = {0x1f, address, value};
    unsigned char observed = 0;

    if (transfer(command, sizeof(command), NULL, 0) < 0 ||
        get_feature(address, &observed) < 0 || observed != value) {
        fprintf(stderr, "Cannot set/read back HYF feature %02x=%02x.\n",
                address, value);
        return -1;
    }

    return 0;
}

static int wait_ready(unsigned char *status)
{
    struct timespec start;

    if (clock_gettime(CLOCK_MONOTONIC, &start) < 0) {
        perror("CLOCK_MONOTONIC");
        return -1;
    }

    for (;;) {
        struct timespec now;

        if (get_feature(FEATURE_STATUS, status) < 0) {
            return -1;
        }

        if ((*status & STATUS_BUSY) == 0) {
            return 0;
        }

        if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
            perror("CLOCK_MONOTONIC");
            return -1;
        }

        if (now.tv_sec - start.tv_sec > 2 ||
            (now.tv_sec - start.tv_sec == 2 && now.tv_nsec >= start.tv_nsec)) {
            fprintf(stderr, "HYF busy timeout (2 seconds, status=%02x).\n", *status);
            return -1;
        }

        const struct timespec delay = {0, 1000000};

        (void)nanosleep(&delay, NULL);
    }
}

static int row_command(unsigned char opcode, uint32_t row)
{
    if (row >= BLOCK_COUNT * PAGES_PER_BLOCK ||
        (opcode != 0x13 && opcode != 0x10 && opcode != 0xd8) ||
        (opcode == 0xd8 && row % PAGES_PER_BLOCK != 0)) {
        fprintf(stderr, "Internal HYF row/command range error.\n");
        return -1;
    }

    const unsigned char command[] = {
        opcode, (unsigned char)(row >> 16),
        (unsigned char)(row >> 8), (unsigned char)row
    };

    return transfer(command, sizeof(command), NULL, 0);
}

static int write_enable(void)
{
    const unsigned char command[] = {0x06};
    unsigned char status = 0;

    if (transfer(command, sizeof(command), NULL, 0) < 0 ||
        get_feature(FEATURE_STATUS, &status) < 0 ||
        (status & (STATUS_BUSY | STATUS_WEL)) != STATUS_WEL) {
        fprintf(stderr, "HYF did not enter ready/write-enabled state (%02x).\n", status);
        return -1;
    }

    return 0;
}

static int write_disable(void)
{
    const unsigned char command[] = {0x04};
    unsigned char status = 0;

    if (transfer(command, sizeof(command), NULL, 0) < 0 ||
        get_feature(FEATURE_STATUS, &status) < 0 || (status & STATUS_WEL) != 0) {
        fprintf(stderr, "Cannot clear/verify HYF WEL; power-cycle before reuse.\n");
        return -1;
    }

    return 0;
}

static int read_page(uint32_t row, unsigned char *destination,
                     unsigned byte_count, unsigned char *status)
{
    /* Column0, wrap bits00 and one dummy byte: 2176-byte cache addressing.
     * OOB-only preservation requests just2048 bytes; optional verification
     * requests the whole page. There is no implicit readback elsewhere.
     */
    const unsigned char command[] = {0x03, 0x00, 0x00, 0xff};

    if (byte_count > PAGE_BYTES || row_command(0x13, row) < 0 ||
        wait_ready(status) < 0 ||
        transfer(command, sizeof(command), destination, byte_count) < 0) {
        fprintf(stderr, "HYF page read failed at physical row 0x%05" PRIx32 ".\n", row);
        return -1;
    }

    return 0;
}

static int preserve_main(struct workspace *work, uint32_t first_row, bool raw)
{
    unsigned corrected = 0;

    for (unsigned page = 0; page < PAGES_PER_BLOCK; page++) {
        unsigned char status = 0;

        if (stop_requested || read_page(first_row + page,
                work->main + page * MAIN_PAGE, MAIN_PAGE, &status) < 0) {
            return -1;
        }

        if (!raw) {
            unsigned ecc = (status >> 4) & 3;

            if (ecc == 2) {
                fprintf(stderr, "Uncorrectable live main at row 0x%05" PRIx32
                        "; refusing to erase its block.\n", first_row + page);
                return -1;
            }

            if (ecc != 0) {
                corrected++;
            }
        }
    }

    if (corrected != 0) {
        fprintf(stderr, "Preserved corrected live main on %u page(s), including"
                " corrected-at-limit status if reported.\n", corrected);
    }

    return 0;
}

static int erase_block(uint32_t first_row)
{
    unsigned char status = 0;

    if (write_enable() < 0 || row_command(0xd8, first_row) < 0 ||
        wait_ready(&status) < 0 ||
        (status & STATUS_ERASE_FAIL) != 0) {
        fprintf(stderr, "HYF erase failed/uncertain at block %u (status=%02x).\n",
                first_row / PAGES_PER_BLOCK, status);
        return -1;
    }

    return 0;
}

static int program_block(struct workspace *work, uint32_t first_row, bool raw)
{
    work->command[0] = 0x02;
    work->command[1] = 0;
    work->command[2] = 0;

    for (unsigned page = 0; page < PAGES_PER_BLOCK; page++) {
        const unsigned char *spare = work->oob + page * SPARE_PAGE;
        unsigned char *loaded_spare = work->command + 3 + MAIN_PAGE;
        unsigned char status = 0;

        memcpy(work->command + 3, work->main + page * MAIN_PAGE, MAIN_PAGE);

        if (raw) {
            memcpy(loaded_spare, spare, SPARE_PAGE);
        } else {
            memset(loaded_spare, 0xff, SPARE_PAGE);

            for (unsigned segment = 0; segment < 4; segment++) {
                memcpy(loaded_spare + segment * 32, spare + segment * 32, 8);
            }
        }

        /* Exactly ONE program-load and ONE execute per erased page. The
         * cache receives main+spare together; random-load shortcuts are not
         * substitutes for erase/read-modify-write of existing OOB bytes.
         */
        if (write_enable() < 0 ||
            transfer(work->command, sizeof(work->command), NULL, 0) < 0 ||
            row_command(0x10, first_row + page) < 0 || wait_ready(&status) < 0 ||
            (status & (STATUS_PROGRAM_FAIL | STATUS_WEL)) != 0) {
            fprintf(stderr, "HYF program failed/uncertain at row 0x%05" PRIx32
                    " (status=%02x). No remap/retry performed.\n",
                    first_row + page, status);
            return -1;
        }
    }

    return 0;
}

static int verify_block(struct workspace *work, uint32_t first_row, bool raw)
{
    for (unsigned page = 0; page < PAGES_PER_BLOCK; page++) {
        unsigned char status = 0;
        const unsigned char *spare = work->oob + page * SPARE_PAGE;

        if (read_page(first_row + page, work->readback, PAGE_BYTES, &status) < 0) {
            return -1;
        }

        bool equal = memcmp(work->readback, work->main + page * MAIN_PAGE,
                            MAIN_PAGE) == 0;

        if (raw) {
            equal = equal && memcmp(work->readback + MAIN_PAGE, spare, SPARE_PAGE) == 0;
        } else {
            if (((status >> 4) & 3) != 0) {
                fprintf(stderr, "Fresh HYF row 0x%05" PRIx32
                        " reports non-clean ECC status %u.\n",
                        first_row + page, (status >> 4) & 3);
                equal = false;
            }

            for (unsigned segment = 0; segment < 4; segment++) {
                if (memcmp(work->readback + MAIN_PAGE + segment * 32,
                           spare + segment * 32, 8) != 0) {
                    equal = false;
                }
            }
        }

        if (!equal) {
            fprintf(stderr, "HYF readback mismatch at physical row 0x%05" PRIx32
                    ". No remap/retry performed.\n", first_row + page);
            return -1;
        }
    }

    return 0;
}

static int open_input(struct input_file *input, const char *path,
                      uint64_t full_size, uint64_t range_size)
{
    struct stat info;

    input->path = path;
    input->stream = fopen(path, "rb");

    if (input->stream == NULL) {
        perror(path);
        return -1;
    }

    if (fstat(fileno(input->stream), &info) < 0 ||
        !S_ISREG(info.st_mode) || info.st_size < 0) {
        fprintf(stderr, "HYF input must be a readable regular file: %s\n", path);
        return -1;
    }

    uint64_t size = (uint64_t)info.st_size;

    if (size == full_size) {
        input->full_chip = true;
    } else if (size == range_size) {
        input->full_chip = false;
    } else {
        fprintf(stderr, "%s: got %" PRIu64 " bytes; need full-chip %" PRIu64
                " or exact range slice %" PRIu64 ".\n", path, size, full_size, range_size);
        return -1;
    }

    return 0;
}

static int load_block(struct input_file *input, uint64_t physical_offset,
                      uint64_t range_offset, void *destination, size_t bytes)
{
    uint64_t offset = range_offset;

    if (input->full_chip) {
        offset = physical_offset;
    }

    if (fseeko(input->stream, (off_t)offset, SEEK_SET) < 0 ||
        fread(destination, 1, bytes, input->stream) != bytes) {
        fprintf(stderr, "HYF input seek/read failed at file offset 0x%" PRIx64
                ": %s\n", offset, input->path);
        return -1;
    }

    return 0;
}

int hyf_nand_write(const struct hyf_nand_options *options)
{
    struct input_file main_input = {0};
    struct input_file oob_input = {0};
    struct workspace *work = NULL;
    struct sigaction old_interrupt, old_terminate;
    bool interrupt_handler = false, terminate_handler = false;
    bool identified = false, config_touched = false, lock_touched = false;
    bool destructive_started = false, current_block_incomplete = false;
    unsigned char original_config = 0, original_lock = 0;
    uint64_t current_block = 0, completed = 0;
    int result = -1;

    stop_requested = 0;

    if (options == NULL || options->oob_path == NULL || *options->oob_path == '\0' ||
        (options->main_path != NULL && *options->main_path == '\0') ||
        options->address >= MAIN_CAPACITY || options->address % MAIN_BLOCK != 0) {
        fprintf(stderr, "HYF needs OOB input and a block-aligned physical main address.\n");
        return -1;
    }

    uint64_t length = options->length;

    if (length == 0) {
        length = MAIN_CAPACITY - options->address;
    }

    if (length > MAIN_CAPACITY - options->address || length % MAIN_BLOCK != 0) {
        fprintf(stderr, "HYF range must contain whole blocks within 256 MiB.\n");
        return -1;
    }

    uint64_t spare_length = length / MAIN_PAGE * SPARE_PAGE;

    if (options->main_path != NULL &&
        open_input(&main_input, options->main_path, MAIN_CAPACITY, length) < 0) {
        goto cleanup;
    }

    if (open_input(&oob_input, options->oob_path, SPARE_CAPACITY, spare_length) < 0) {
        goto cleanup;
    }

    printf("HYF physical main range [0x%08" PRIx64 ",0x%08" PRIx64
           "), %" PRIu64 " erase blocks; ", options->address,
           options->address + length, length / MAIN_BLOCK);

    if (options->main_path == NULL) {
        puts("OOB-only: live main is preserved one block at a time.");
    } else {
        puts("paired main/OOB input; no live-main preflight.");
    }

    if (options->raw) {
        puts("RAW: ECC off, all 128 spare bytes written. Supplied parity MUST match main.");
    } else {
        puts("ECC: on; preserve 8 metadata bytes/32-byte spare segment; regenerate parity.");
    }

    if (options->verify) {
        puts("Verification: enabled (-v), page readback after each programmed block.");
    } else {
        puts("Verification: disabled; no readback. Status/configuration checks remain enabled.");
    }

    if (options->dry_run) {
        puts("DRY RUN: host files/range validated; no SPI/USB calls made.");
        result = 0;
        goto cleanup;
    }

    work = malloc(sizeof(*work));

    if (work == NULL) {
        perror("HYF one-block workspace");
        goto cleanup;
    }

    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = request_stop;
    action.sa_flags = SA_RESTART;
    sigemptyset(&action.sa_mask);
    stop_requested = 0;

    if (sigaction(SIGINT, &action, &old_interrupt) < 0) {
        perror("sigaction SIGINT");
        goto cleanup;
    }

    interrupt_handler = true;

    if (sigaction(SIGTERM, &action, &old_terminate) < 0) {
        perror("sigaction SIGTERM");
        goto cleanup;
    }

    terminate_handler = true;

    unsigned char status = 0;
    const unsigned char identify[] = {0x9f, 0};
    unsigned char id[2] = {0};

    if (wait_ready(&status) < 0 ||
        transfer(identify, sizeof(identify), id, sizeof(id)) < 0) {
        goto cleanup;
    }

    if (id[0] != 0xc9 || id[1] != 0x52) {
        fprintf(stderr, "Expected HYF C9:52, observed %02x:%02x. No erase/program.\n",
                id[0], id[1]);
        goto cleanup;
    }

    identified = true;

    if (get_feature(FEATURE_CONFIG, &original_config) < 0 ||
        get_feature(FEATURE_LOCK, &original_lock) < 0) {
        goto cleanup;
    }

    if ((original_config & CONFIG_OTP) != 0) {
        fprintf(stderr, "HYF OTP mode enabled; refusing all erase/program operations.\n");
        goto cleanup;
    }

    unsigned char desired_config = original_config | CONFIG_ECC;

    if (options->raw) {
        desired_config = original_config & (unsigned char)~CONFIG_ECC;
    }

    config_touched = true;

    if (set_feature(FEATURE_CONFIG, desired_config) < 0) {
        goto cleanup;
    }

    lock_touched = true;

    if (set_feature(FEATURE_LOCK, original_lock & (unsigned char)~LOCK_BP_MASK) < 0) {
        goto cleanup;
    }

    for (uint64_t offset = 0; offset < length; offset += MAIN_BLOCK) {
        current_block = options->address + offset;
        current_block_incomplete = false;
        uint32_t first_row = (uint32_t)(current_block / MAIN_PAGE);

        if (stop_requested ||
            load_block(&oob_input, current_block / MAIN_PAGE * SPARE_PAGE,
                       offset / MAIN_PAGE * SPARE_PAGE, work->oob, SPARE_BLOCK) < 0) {
            goto cleanup;
        }

        if (options->main_path != NULL) {
            if (load_block(&main_input, current_block, offset,
                           work->main, MAIN_BLOCK) < 0) {
                goto cleanup;
            }
        } else if (preserve_main(work, first_row, options->raw) < 0) {
            goto cleanup;
        }

        if (stop_requested) {
            goto cleanup;
        }

        /* Files for THIS complete block are already buffered before erase.
         * Once the command may have reached the chip, a transport error is
         * destructive/uncertain too. Never retry at a shifted physical block.
         */
        destructive_started = true;
        current_block_incomplete = true;

        if (erase_block(first_row) < 0 || program_block(work, first_row, options->raw) < 0) {
            goto cleanup;
        }

        if (options->verify && verify_block(work, first_row, options->raw) < 0) {
            goto cleanup;
        }

        completed++;
        current_block_incomplete = false;
        printf("Block %" PRIu64 "/%" PRIu64 " programmed at 0x%08" PRIx64,
               completed, length / MAIN_BLOCK, current_block);

        if (options->verify) {
            printf("; verified");
        }

        putchar('\n');
        fflush(stdout);
    }

    if (!stop_requested) {
        result = 0;
    }

cleanup:
    if (identified) {
        unsigned char cleanup_status = 0;

        if (wait_ready(&cleanup_status) < 0) {
            fprintf(stderr, "HYF still busy/unknown; configuration restoration not safe.\n");
            result = -1;
        } else {
            if (write_disable() < 0) {
                result = -1;
            }

            if (lock_touched && set_feature(FEATURE_LOCK, original_lock) < 0) {
                fprintf(stderr, "Cannot restore original HYF A0 protection.\n");
                result = -1;
            }

            if (config_touched && set_feature(FEATURE_CONFIG, original_config) < 0) {
                fprintf(stderr, "Cannot restore original HYF B0 configuration.\n");
                result = -1;
            }
        }
    }

    if (terminate_handler && sigaction(SIGTERM, &old_terminate, NULL) < 0) {
        result = -1;
    }

    if (interrupt_handler && sigaction(SIGINT, &old_interrupt, NULL) < 0) {
        result = -1;
    }

    if (main_input.stream != NULL && fclose(main_input.stream) != 0) {
        result = -1;
    }

    if (oob_input.stream != NULL && fclose(oob_input.stream) != 0) {
        result = -1;
    }

    free(work);

    if (stop_requested && !options->dry_run) {
        fprintf(stderr, "HYF interrupted; in-progress block completion was deferred safely"
                " where transport permitted.\n");
        result = -1;
    }

    if (result != 0 && destructive_started) {
        if (current_block_incomplete) {
            fprintf(stderr, "WARNING: block at 0x%08" PRIx64
                    " may be erased/partial or unverified.\n", current_block);
        }

        fprintf(stderr, "%" PRIu64 " block(s) completed before failure/interruption/cleanup;"
                " no automatic recovery.\n", completed);
    }

    return result;
}
