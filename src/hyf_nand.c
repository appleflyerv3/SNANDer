/* SPDX-License-Identifier: GPL-2.0-or-later
 * HYF2GQ4UAACAE physical main/OOB writer and raw spare reader.
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
#include <fcntl.h>
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

/* A completed NAND rejection/failure is different from uncertain transport
 * or protocol state. Only the former may be skipped by an explicit request.
 */
enum block_result {
    BLOCK_ERROR = -1,
    BLOCK_OK = 0,
    BLOCK_DEVICE_FAILURE = 1,
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
    if (programmer_transaction(write_count, read_count, command, result) < 0) {
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

static enum block_result erase_block(uint32_t first_row)
{
    unsigned char status = 0;

    if (write_enable() < 0 || row_command(0xd8, first_row) < 0 ||
        wait_ready(&status) < 0) {
        fprintf(stderr, "HYF erase failed/uncertain at block %u (status=%02x).\n",
                first_row / PAGES_PER_BLOCK, status);
        return BLOCK_ERROR;
    }

    if ((status & STATUS_ERASE_FAIL) != 0) {
        fprintf(stderr, "HYF NAND reported erase failure at block %u (status=%02x).\n",
                first_row / PAGES_PER_BLOCK, status);
        return BLOCK_DEVICE_FAILURE;
    }

    if ((status & STATUS_WEL) != 0) {
        fprintf(stderr, "HYF erase left WEL set without E_FAIL at block %u"
                " (status=%02x); protocol state uncertain.\n",
                first_row / PAGES_PER_BLOCK, status);
        return BLOCK_ERROR;
    }

    return BLOCK_OK;
}

static enum block_result program_block(struct workspace *work, uint32_t first_row, bool raw)
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
            row_command(0x10, first_row + page) < 0 || wait_ready(&status) < 0) {
            fprintf(stderr, "HYF program failed/uncertain at row 0x%05" PRIx32
                    " (status=%02x). No remap/retry performed.\n",
                    first_row + page, status);
            return BLOCK_ERROR;
        }

        if ((status & STATUS_PROGRAM_FAIL) != 0) {
            fprintf(stderr, "HYF NAND reported program failure at row 0x%05" PRIx32
                    " (status=%02x).\n", first_row + page, status);
            return BLOCK_DEVICE_FAILURE;
        }

        if ((status & STATUS_WEL) != 0) {
            fprintf(stderr, "HYF program left WEL set without P_FAIL at row 0x%05"
                    PRIx32 " (status=%02x); protocol state uncertain.\n",
                    first_row + page, status);
            return BLOCK_ERROR;
        }
    }

    return BLOCK_OK;
}

/* This does not fix or reset a failed block. It establishes that proceeding
 * to a DIFFERENT block will still use the intended unlocked/configured NAND.
 * Failure bits may stay set until their next corresponding command; do not
 * mistake a previous P_FAIL for an erase failure, or vice versa.
 */
static int prepare_after_skipped_block(unsigned char expected_lock,
                                       unsigned char expected_config)
{
    unsigned char status = 0, lock = 0, config = 0;

    if (write_disable() < 0 || get_feature(FEATURE_STATUS, &status) < 0 ||
        get_feature(FEATURE_LOCK, &lock) < 0 ||
        get_feature(FEATURE_CONFIG, &config) < 0) {
        fprintf(stderr, "Cannot establish safe NAND state after failed block; stopping.\n");
        return -1;
    }

    if ((status & (STATUS_BUSY | STATUS_WEL)) != 0 ||
        lock != expected_lock || config != expected_config) {
        fprintf(stderr, "NAND state changed after failed block: A0=%02x B0=%02x"
                " C0=%02x; expected A0=%02x B0=%02x and ready/WEL clear. Stopping.\n",
                lock, config, status, expected_lock, expected_config);
        return -1;
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
    uint64_t current_block = 0, completed = 0, skipped = 0;
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

    if (options->skip_bad_blocks) {
        puts("Skip policy: reported NAND erase/program failures only; physical offsets"
             " unchanged. Skipped blocks are NOT restored or verified.");
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
    unsigned char desired_lock = original_lock & (unsigned char)~LOCK_BP_MASK;

    if (set_feature(FEATURE_LOCK, desired_lock) < 0) {
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

        const char *failed_operation = "erase";
        enum block_result block_result = erase_block(first_row);

        if (block_result == BLOCK_OK) {
            failed_operation = "program";
            block_result = program_block(work, first_row, options->raw);
        }

        if (block_result == BLOCK_DEVICE_FAILURE && options->skip_bad_blocks) {
            if (prepare_after_skipped_block(desired_lock, desired_config) < 0) {
                goto cleanup;
            }

            skipped++;
            current_block_incomplete = false;
            fprintf(stderr, "SKIPPED physical block %u at 0x%08" PRIx64
                    " after NAND %s failure; not restored/verified, may be partial."
                    " No retry/remap; next input keeps its physical offset.\n",
                    first_row / PAGES_PER_BLOCK, current_block, failed_operation);
            continue;
        }

        if (block_result != BLOCK_OK) {
            goto cleanup;
        }

        if (options->verify && verify_block(work, first_row, options->raw) < 0) {
            goto cleanup;
        }

        completed++;
        current_block_incomplete = false;
        printf("Block %" PRIu64 "/%" PRIu64 " programmed at 0x%08" PRIx64,
               completed + skipped, length / MAIN_BLOCK, current_block);

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

    if (options->skip_bad_blocks && !options->dry_run) {
        printf("HYF write summary: %" PRIu64 " programmed, %" PRIu64
               " skipped, %" PRIu64 " requested blocks; %s.\n",
               completed, skipped, length / MAIN_BLOCK,
               result == 0 ? "range processed" : "stopped with error");
        if (skipped != 0) {
            puts("INCOMPLETE RESTORATION: skipped blocks were not restored/verified;"
                 " no data was relocated.");
        }
    }

    if (result != 0 && destructive_started) {
        if (current_block_incomplete) {
            fprintf(stderr, "WARNING: block at 0x%08" PRIx64
                    " may be erased/partial or unverified.\n", current_block);
        }

        fprintf(stderr, "%" PRIu64 " block(s) programmed, %" PRIu64
                " skipped before failure/interruption/cleanup; no automatic recovery.\n",
                completed, skipped);
    }

    return result;
}

/* Unlike an erased block being rewritten, a read may stop between any two
 * commands. Keep this interruptible wait separate from the writer's wait:
 * the writer deliberately finishes a block after its erase has begun.
 */
struct capture_read_stats {
    uint64_t polls;
    uint64_t busy_polls;
    double polling_seconds;
    double cache_seconds;
};

static double capture_elapsed(const struct timespec *start);

static int wait_read_ready(struct capture_read_stats *stats, bool interruptible)
{
    struct timespec start;
    int result = -1;

    if (clock_gettime(CLOCK_MONOTONIC, &start) < 0) {
        perror("CLOCK_MONOTONIC");
        return -1;
    }

    for (;;) {
        unsigned char status = 0;
        struct timespec now;

        if (interruptible && stop_requested) {
            break;
        }

        stats->polls++;

        if (get_feature(FEATURE_STATUS, &status) < 0) {
            break;
        }

        if ((status & STATUS_BUSY) == 0) {
            result = 0;
            break;
        }

        stats->busy_polls++;

        if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
            perror("CLOCK_MONOTONIC");
            break;
        }

        if (now.tv_sec - start.tv_sec > 2 ||
            (now.tv_sec - start.tv_sec == 2 && now.tv_nsec >= start.tv_nsec)) {
            fprintf(stderr, "HYF read busy timeout (2 seconds, status=%02x).\n", status);
            break;
        }

        /* Each status transaction already takes transport time. Poll again
         * immediately instead of adding an unconditional 1-ms sleep to a
         * short page load; the stop check and two-second deadline remain.
         */
    }

    double elapsed = capture_elapsed(&start);

    if (elapsed < 0) {
        return -1;
    }

    stats->polling_seconds += elapsed;
    return result;
}

static int read_capture_page(uint32_t row, unsigned char *bytes, bool paired,
                              struct capture_read_stats *stats)
{
    /* The raw pair must come from ONE cache read of ONE loaded page. OOB
     * alone instead starts at column 0x0800 and never transfers main bytes.
     */
    unsigned char command[] = {0x03, 0x08, 0x00, 0xff};
    unsigned byte_count = SPARE_PAGE;

    if (paired) {
        command[1] = 0;
        byte_count = PAGE_BYTES;
    }

    if (stop_requested || row_command(0x13, row) < 0 ||
        wait_read_ready(stats, true) < 0 || stop_requested) {
        fprintf(stderr, "HYF raw read stopped/failed at physical row 0x%05"
                PRIx32 ".\n", row);
        return -1;
    }

    struct timespec started;

    if (clock_gettime(CLOCK_MONOTONIC, &started) < 0) {
        perror("CLOCK_MONOTONIC");
        return -1;
    }

    int result = transfer(command, sizeof(command), bytes, byte_count);
    double elapsed = capture_elapsed(&started);

    if (elapsed < 0) {
        return -1;
    }

    stats->cache_seconds += elapsed;

    if (result < 0) {
        fprintf(stderr, "HYF raw cache read failed at physical row 0x%05" PRIx32 ".\n", row);
        return -1;
    }

    return 0;
}

static int require_new_output(const char *path)
{
    struct stat info;

    /* lstat also rejects dangling symlinks. O_EXCL below closes the race
     * between this read-only check and the actual file creation.
     */
    if (lstat(path, &info) == 0) {
        fprintf(stderr, "HYF output already exists; choose a new path: %s\n", path);
        return -1;
    }

    if (errno != ENOENT) {
        perror(path);
        return -1;
    }

    return 0;
}

/* Resolve only a new output's parent, not the nonexistent output itself.
 * Comparing parent identity and final name detects two spellings of the
 * same target, including ../ and symlinked parent directories, in dry runs.
 */
static int output_parent(const char *path, struct stat *info, const char **name)
{
    const char *slash = strrchr(path, '/');
    char *directory;

    *name = path;

    if (slash == NULL) {
        directory = strdup(".");
    } else {
        *name = slash + 1;
        size_t length = (size_t)(slash - path);

        if (length == 0) {
            length = 1; /* Parent of /name is /, not an empty string. */
        }

        directory = strndup(path, length);
    }

    if (directory == NULL) {
        perror("HYF output parent");
        return -1;
    }

    int result = stat(directory, info);

    if (result < 0) {
        perror(directory);
    } else if (!S_ISDIR(info->st_mode) || **name == '\0') {
        fprintf(stderr, "HYF output needs an existing parent directory and filename: %s\n",
                path);
        result = -1;
    }

    free(directory);
    return result;
}

static int require_distinct_outputs(const char *main_path, const char *oob_path)
{
    struct stat main_parent, oob_parent;
    const char *main_name, *oob_name;

    if (output_parent(main_path, &main_parent, &main_name) < 0 ||
        output_parent(oob_path, &oob_parent, &oob_name) < 0) {
        return -1;
    }

    if (main_parent.st_dev == oob_parent.st_dev &&
        main_parent.st_ino == oob_parent.st_ino && strcmp(main_name, oob_name) == 0) {
        fprintf(stderr, "HYF main and OOB outputs must be different new files.\n");
        return -1;
    }

    return 0;
}

struct capture_output {
    const char *path;
    FILE *stream;
    bool created;
    bool failed;
    uint64_t accepted;
    uint64_t written;
    double seconds;
    size_t pending;
    /* Two output buffers plus the one-page cache stay below 134 KiB.
     * This is bounded batching, never a full-image allocation.
     */
    unsigned char buffer[64 * 1024];
};

static int open_capture_output(struct capture_output *output)
{
    int descriptor = open(output->path, O_WRONLY | O_CREAT | O_EXCL, 0666);

    if (descriptor < 0) {
        perror(output->path);
        return -1;
    }

    output->created = true;
    output->stream = fdopen(descriptor, "wb");

    if (output->stream == NULL) {
        perror("HYF capture fdopen");

        if (close(descriptor) < 0) {
            perror("HYF capture descriptor close");
        }

        return -1;
    }

    /* Our explicit buffer below batches pages. Keep stdio unbuffered so a
     * bulk fwrite's result counts bytes actually handed to the file, not
     * merely accepted into another hidden buffer.
     */
    if (setvbuf(output->stream, NULL, _IONBF, 0) != 0) {
        fprintf(stderr, "Cannot configure HYF capture stream: %s\n", output->path);
        return -1;
    }

    return 0;
}

static int append_capture_output(struct capture_output *output,
                                 const unsigned char *bytes, size_t count)
{
    if (output->failed || count > sizeof(output->buffer) - output->pending) {
        fprintf(stderr, "HYF capture output buffer state invalid: %s\n", output->path);
        return -1;
    }

    memcpy(output->buffer + output->pending, bytes, count);
    output->pending += count;
    output->accepted += count;
    return 0;
}

static int flush_capture_output(struct capture_output *output)
{
    if (output->failed) {
        return -1;
    }

    if (output->pending == 0) {
        return 0;
    }

    struct timespec started;
    bool timed = clock_gettime(CLOCK_MONOTONIC, &started) == 0;

    if (!timed) {
        perror("CLOCK_MONOTONIC");
    }

    size_t count = output->pending;
    size_t written = fwrite(output->buffer, 1, count, output->stream);
    int write_error = errno;

    output->written += written;
    output->pending = 0;

    /* Timing failure must not discard data already accepted from the chip. */
    double elapsed = timed ? capture_elapsed(&started) : -1;

    if (elapsed >= 0) {
        output->seconds += elapsed;
    }

    if (written != count) {
        /* Do not retry a failed/short bulk write or pretend its tail was
         * saved. Accepted versus retained counts keep the loss explicit;
         * cleanup can still flush the OTHER output's complete pages.
         */
        output->failed = true;
        errno = write_error;
        perror(output->path);
        return -1;
    }

    return elapsed < 0 ? -1 : 0;
}

static int close_capture_output(struct capture_output *output)
{
    int result = 0;

    if (output->stream != NULL) {
        /* Even an interrupted read should save its already captured pages.
         * A previously failed bulk write is not retried, but the other
         * output remains independently flushable.
         */
        if (flush_capture_output(output) < 0) {
            result = -1;
        }

        struct timespec started;
        bool timed = clock_gettime(CLOCK_MONOTONIC, &started) == 0;

        if (!timed) {
            perror("CLOCK_MONOTONIC");
            result = -1;
        }

        if (fflush(output->stream) != 0) {
            perror("HYF capture output flush");
            result = -1;
        }

        struct stat info;

        if (fstat(fileno(output->stream), &info) < 0 || info.st_size < 0) {
            perror("HYF capture retained length");
            result = -1;
        } else if ((uint64_t)info.st_size != output->written) {
            fprintf(stderr, "HYF capture file length differs from its write count: %s\n",
                    output->path);
            output->written = (uint64_t)info.st_size;
            result = -1;
        }

        if (fclose(output->stream) != 0) {
            perror("HYF capture output close");
            result = -1;
        }

        if (timed) {
            double elapsed = capture_elapsed(&started);

            if (elapsed < 0) {
                result = -1;
            } else {
                output->seconds += elapsed;
            }
        }

        output->stream = NULL;
    }

    return result;
}

static double capture_elapsed(const struct timespec *start)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) {
        perror("CLOCK_MONOTONIC");
        return -1;
    }

    return (double)(now.tv_sec - start->tv_sec) +
        (double)(now.tv_nsec - start->tv_nsec) / 1000000000.0;
}

int hyf_nand_read(const struct hyf_nand_read_options *options)
{
    struct sigaction old_interrupt, old_terminate;
    bool interrupt_handler = false, terminate_handler = false;
    bool config_touched = false;
    unsigned char original_config = 0;
    struct capture_output main_output = {0}, oob_output = {0};
    struct capture_read_stats stats = {0};
    struct timespec started;
    double last_progress = 0;
    int result = -1;

    stop_requested = 0;

    if (options == NULL || options->oob_path == NULL || *options->oob_path == '\0' ||
        (options->main_path != NULL && *options->main_path == '\0') ||
        options->address >= MAIN_CAPACITY || options->address % MAIN_PAGE != 0) {
        fprintf(stderr, "HYF raw read needs new outputs and a page-aligned main address.\n");
        return -1;
    }

    uint64_t length = options->length;

    if (length == 0) {
        length = MAIN_CAPACITY - options->address;
    }

    /* Subtract before comparing: even UINT64_MAX cannot wrap the bound. */
    if (length > MAIN_CAPACITY - options->address || length % MAIN_PAGE != 0) {
        fprintf(stderr, "HYF raw range invalid; range must be whole pages"
                " within 256 MiB.\n");
        return -1;
    }

    /* Validate BOTH absent paths before creating either output. Exclusive
     * opens remain necessary because another process can race validation.
     */
    if (require_new_output(options->oob_path) < 0 ||
        (options->main_path != NULL &&
         (require_new_output(options->main_path) < 0 ||
          require_distinct_outputs(options->main_path, options->oob_path) < 0))) {
        return -1;
    }

    bool paired = options->main_path != NULL;
    uint64_t expected_oob = length / MAIN_PAGE * SPARE_PAGE;
    uint32_t first_row = (uint32_t)(options->address / MAIN_PAGE);
    uint32_t page_count = (uint32_t)(length / MAIN_PAGE);

    printf("HYF raw %s read: main range [0x%08" PRIx64 ",0x%08" PRIx64
           "), %" PRIu32 " physical pages; main %" PRIu64 ", OOB %" PRIu64 " bytes.\n",
           paired ? "main+OOB" : "OOB-only", options->address,
           options->address + length, page_count, paired ? length : 0, expected_oob);

    if (paired) {
        puts("RAW main is UNCORRECTED (ECC off); it may differ from a legacy ECC read.");
    }

    if (options->dry_run) {
        puts("DRY RUN: range/new output validated; no file created or SPI/USB calls made.");
        return 0;
    }

    main_output.path = options->main_path;
    oob_output.path = options->oob_path;

    if (clock_gettime(CLOCK_MONOTONIC, &started) < 0) {
        perror("CLOCK_MONOTONIC");
        return -1;
    }

    struct sigaction action;

    memset(&action, 0, sizeof(action));
    action.sa_handler = request_stop;
    action.sa_flags = SA_RESTART;
    sigemptyset(&action.sa_mask);

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

    /* No truncation or following of an existing final-component symlink.
     * Check/open output before sending commands, so path failures do not
     * needlessly change the chip configuration.
     */
    if ((paired && open_capture_output(&main_output) < 0) ||
        open_capture_output(&oob_output) < 0) {
        goto cleanup;
    }

    const unsigned char identify[] = {0x9f, 0};
    unsigned char id[2] = {0};

    if (wait_read_ready(&stats, true) < 0 || stop_requested ||
        transfer(identify, sizeof(identify), id, sizeof(id)) < 0) {
        goto cleanup;
    }

    if (id[0] != 0xc9 || id[1] != 0x52) {
        fprintf(stderr, "Expected HYF C9:52, observed %02x:%02x; no data captured.\n",
                id[0], id[1]);
        goto cleanup;
    }

    if (stop_requested || get_feature(FEATURE_CONFIG, &original_config) < 0) {
        goto cleanup;
    }

    if ((original_config & CONFIG_OTP) != 0) {
        fprintf(stderr, "HYF OTP mode enabled; refusing normal-array raw capture.\n");
        goto cleanup;
    }

    if (stop_requested) {
        goto cleanup;
    }

    if ((original_config & CONFIG_ECC) != 0) {
        /* Mark before sending: a failed transfer may still have changed B0. */
        config_touched = true;

        if (set_feature(FEATURE_CONFIG,
                        original_config & (unsigned char)~CONFIG_ECC) < 0) {
            goto cleanup;
        }
    }

    for (uint32_t page = 0; page < page_count; page++) {
        unsigned char bytes[PAGE_BYTES];

        if (read_capture_page(first_row + page, bytes, paired, &stats) < 0) {
            goto cleanup;
        }

        const unsigned char *spare = bytes;

        if (paired) {
            if (append_capture_output(&main_output, bytes, MAIN_PAGE) < 0) {
                goto cleanup;
            }

            spare = bytes + MAIN_PAGE;
        }

        if (append_capture_output(&oob_output, spare, SPARE_PAGE) < 0) {
            goto cleanup;
        }

        /* Accept BOTH parts of this complete cache page before attempting
         * a full-buffer disk write. If one output fails, cleanup can still
         * retain the other part rather than silently discard captured OOB.
         */
        int flush_result = 0;

        if (paired && main_output.pending == sizeof(main_output.buffer) &&
            flush_capture_output(&main_output) < 0) {
            flush_result = -1;
        }

        if (oob_output.pending == sizeof(oob_output.buffer) &&
            flush_capture_output(&oob_output) < 0) {
            flush_result = -1;
        }

        if (flush_result < 0) {
            goto cleanup;
        }

        double elapsed = capture_elapsed(&started);

        if (elapsed < 0) {
            goto cleanup;
        }

        if (elapsed - last_progress >= 1.0 || page + 1 == page_count) {
            uint64_t total = main_output.accepted + oob_output.accepted;
            double rate = elapsed > 0 ? (double)total / elapsed / 1024.0 : 0;

            printf("RAW: %" PRIu32 "/%" PRIu32 " pages; accepted main %" PRIu64
                   ", OOB %" PRIu64 "; file-written main %" PRIu64 ", OOB %" PRIu64
                   " bytes; %.1f s, %.1f KiB/s captured.\n", page + 1, page_count,
                   main_output.accepted, oob_output.accepted,
                   main_output.written, oob_output.written, elapsed, rate);
            fflush(stdout);
            last_progress = elapsed;
        }
    }

    if (!stop_requested) {
        result = 0;
    }

cleanup:
    if (config_touched) {
        /* Read interruption must not suppress restoration. A pending page
         * load is bounded by the read ready timeout before changing B0.
         * No WREN/WRDI or A0 command is part of this read-only path.
         */
        if (wait_read_ready(&stats, false) < 0 ||
            set_feature(FEATURE_CONFIG, original_config) < 0) {
            fprintf(stderr, "Cannot restore original HYF B0; power-cycle before reuse.\n");
            result = -1;
        }
    }

    if (close_capture_output(&main_output) < 0) {
        result = -1;
    }

    if (close_capture_output(&oob_output) < 0) {
        result = -1;
    }

    if (main_output.accepted != main_output.written ||
        oob_output.accepted != oob_output.written) {
        result = -1;
    }

    if (terminate_handler && sigaction(SIGTERM, &old_terminate, NULL) < 0) {
        perror("restore SIGTERM");
        result = -1;
    }

    if (interrupt_handler && sigaction(SIGINT, &old_interrupt, NULL) < 0) {
        perror("restore SIGINT");
        result = -1;
    }

    if (stop_requested) {
        fprintf(stderr, "HYF raw capture interrupted by signal %d.\n", (int)stop_requested);
        result = -1;
    }

    double elapsed = capture_elapsed(&started);

    if (elapsed < 0) {
        result = -1;
    }

    const char *outcome = result == 0 ? "COMPLETE" : "PARTIAL/ERROR";

    if (main_output.created) {
        printf("%s main capture retained: %" PRIu64 "/%" PRIu64
               " file bytes; %" PRIu64 " accepted in %s.\n",
               outcome, main_output.written, length, main_output.accepted, main_output.path);
    }

    if (oob_output.created) {
        printf("%s OOB capture retained: %" PRIu64 "/%" PRIu64
               " file bytes; %" PRIu64 " accepted in %s.\n",
               outcome, oob_output.written, expected_oob, oob_output.accepted, oob_output.path);
    }

    if (elapsed >= 0) {
        uint64_t total = main_output.written + oob_output.written;
        double rate = elapsed > 0 ? (double)total / elapsed / 1024.0 : 0;

        printf("Raw capture elapsed %.3f s; %" PRIu64 " total bytes, %.1f KiB/s.\n",
               elapsed, total, rate);
    }

    printf("Read polling: %" PRIu64 " status polls, %" PRIu64
           " busy, %.6f s; intentional polling sleeps: 0.\n",
           stats.polls, stats.busy_polls, stats.polling_seconds);
    printf("Read timing: cache transfers %.6f s; bulk file output/close %.6f s.\n",
           stats.cache_seconds, main_output.seconds + oob_output.seconds);

    return result;
}

int hyf_nand_read_oob(const struct hyf_nand_read_options *options)
{
    if (options != NULL && options->main_path != NULL) {
        fprintf(stderr, "OOB-only entry point does not accept a main output.\n");
        return -1;
    }

    return hyf_nand_read(options);
}
