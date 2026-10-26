/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef SNANDER_HYF_NAND_H
#define SNANDER_HYF_NAND_H

#include <stdbool.h>
#include <stdint.h>

struct hyf_nand_options {
    /* NULL main_path selects OOB-only read/modify/write. In that mode the
     * existing main data must be read before each block is erased.
     */
    const char *main_path;
    const char *oob_path;
    /* Physical MAIN-byte addresses, not an interleaved main+OOB address.
     * Address and resolved length must be whole 128-KiB erase blocks.
     * A zero length selects the remainder of this 256-MiB chip.
     */
    uint64_t address;
    uint64_t length;
    /* Default: copy 8 metadata bytes per 32-byte OOB segment and let the
     * enabled on-die ECC generate parity. Raw: disable ECC and write all128
     * OOB bytes; the caller must supply parity matching the written main.
     */
    bool raw;
    bool verify;
    bool dry_run;
};

/* Exact-physical same-device HYF2GQ4UAACAE writer, not a bad-block mapper.
 * OEM first-spare bytes are content/tags in the supplied PX2131 images:
 * zero-valued tags occur on a block successfully programmed by the existing
 * tool. They cannot safely be reinterpreted as generic factory bad markers.
 * No marker scan, remap, shifted file mapping or automatic retry is done.
 * Program/erase failure status aborts instead of choosing a different block.
 *
 * Each input must be a regular full-chip file or an exact selected-range
 * slice: main256MiB or selected length, OOB16MiB or selected length/16.
 * Files are mapped independently and never modified. Memory is bounded to
 * one block plus page-sized command/verification buffers, below200KiB.
 *
 * dry_run validates host files/range and prints the plan without ANY SPI/USB
 * calls. Otherwise the caller already initialized the selected programmer.
 * This function never opens/closes USB. Readback occurs ONLY with verify;
 * OOB-only main preservation and ID/status/configuration checks are required
 * independently of verification. A0/B0 and signal handlers are restored on
 * ordinary cleanup, and WEL is cleared. Never disconnect power during writes.
 * Returns0 on success, -1 on failure; no backups or output files are created.
 */
int hyf_nand_write(const struct hyf_nand_options *options);

#endif
