# SNANDer: CH347 and separate HYF main/OOB files

Local extension of [upstream SNANDer](https://github.com/McMCCRU/SNANDer),
based on commit `5fbd17ff0d7ca11d7abd85e0f163136b9f046380`.
The original CH341A driver and the existing `../snander_oob` helper are unchanged.

## Build on Linux

Requires a C compiler, make, pkg-config and libusb-1.0 development headers.
The build uses the installed libusb; it does not install software or access USB.

```sh
make -C src -j1
./src/SNANDer -h
```

Select CH347 with `-p ch347`; the default remains `-p ch341a`.
CH347 support is for bulk SPI: CH347T mode 1 (`1a86:55db`, interface 2)
and CH347F (`1a86:55de`, interface 4). It is not a HID/JTAG/UART backend.
I2C and Microwire remain CH341A-only. SPI uses mode 0, MSB first,
active-low CS0 and a default 3.75 MHz clock. `--spi-clock HZ` accepts exactly
60000000, 30000000, 15000000, 7500000, 3750000, 1875000, 937500 or 468750.
Only single-lane SPI is implemented. User-reported CH347T main-data captures
at 30 and 60 MHz produced identical SHA-256 hashes, taking 258 and 220 seconds
respectively **before the batching changes below**. This is not validation
of every adapter/wiring setup or of the write paths.

### CH347T normal and DMA modes

`--ch347-mode normal` is the default and keeps the existing manual-CS,
507-byte USB framing. `--ch347-mode dma` opts into WCH's C2 full-duplex
protocol, using up to 4093 SPI bytes per USB command. A full HYF cache read
or program-load fits in one such command. Distinct NAND commands remain
distinct transactions; page-load/ready checks are not skipped.
No proprietary WCH library or kernel-driver installation is required.

The DMA protocol requires **CH347T firmware 0x0341 or newer**, following
WCH's driver compatibility gate; CH347F also supports that path. Firmware is
queried from the adapter, not guessed from its USB `bcdDevice` revision.
An unsupported adapter is rejected before any NAND command. Check adapter
information without issuing NAND commands using:

```sh
sudo ./src/SNANDer -p ch347 --programmer-info
```

This configures the adapter's SPI interface and leaves CS inactive; it does
not probe/unlock the NAND like the inherited `-i` operation. DMA now supports
both the separate-file HYF operations and the inherited SPI controller path,
including plain `-r MAIN`, `-w MAIN`, SPI chip identification and erase.
SPI EEPROM traffic also uses that controller; I2C/Microwire remain CH341A-only.
This is a transport option, not a change to the selected flash operation.

The complete HYF transaction API uses C2 automatic CS assertion/release.
The legacy controller instead retains its explicit C1 CS edges, sending each
opcode/address/data fragment through C2 with no automatic CS flags. This
preserves CS across fragments, including separate write/read calls, without
buffering a whole transaction or changing when bytes are sent. Large
fragments are streamed in bounded 4093-byte chunks. Small fragments still
have their individual USB overhead, so this bridge need not achieve the same
speed as the complete HYF transaction API. Existing normal-mode framing is
unchanged.

Each flash driver's existing identification/status checks and ECC/protection
behavior are unchanged. If DMA is unsupported or a capture fails, use normal
mode with new output names; failed captures are never automatically replayed.

DMA reduces transport overhead for both HYF reads **and writes**, including
program-load commands and OOB-only preservation reads. It does not reduce
the NAND's internal erase/program time. Read-back verification still occurs
only with `-v`; required command acknowledgements/status checks remain.

### Main-only DMA operations

Plain main-only reads keep the original reader and its progress display/ECC
behavior; adding DMA changes only the CH347 USB transport. For an 8-MiB
read-only comparison, use new output names and record shell `time` output:

```sh
sudo -v
time sudo ./src/SNANDer -p ch347 --spi-clock 60000000 \
  --ch347-mode normal -a 0 -l 0x800000 -r perf_main_normal.bin
time sudo ./src/SNANDer -p ch347 --spi-clock 60000000 \
  --ch347-mode dma -a 0 -l 0x800000 -r perf_main_dma.bin
cmp perf_main_normal.bin perf_main_dma.bin
```

`sudo -v` authenticates before timing, so password entry is not included.
Unlike the separate-OOB reader, the inherited main-only reader overwrites an
existing output file. Choose unused names. Compare these two ECC-enabled
captures to each other, not necessarily to the earlier ECC-disabled raw pair.
The 2.87x speedup below measured the complete HYF API, not this legacy bridge.

Main-only `-w MAIN` also accepts DMA, with its existing limitations unchanged:
the legacy NAND writer does **not** erase automatically and skips all-`FF`
input pages. It attempts to carry forward cached OOB, but is not a complete
erase-and-restore procedure for an already programmed dictionary. For an
exact raw restoration use `-w MAIN --oob OOB --oob-mode raw` with a matching
pair, which uses the block erase/rewrite writer. Adding DMA does not change
which blocks are erased or introduce automatic verification.

Use voltage-compatible SPI wiring and a common ground. This software does
not set adapter supply/IO voltage. Do not use 5 V signals for this NAND or
power it simultaneously from the programmer and dictionary batteries.

## Separate main and OOB files

The new separate-OOB operations are deliberately specific to
**HYF2GQ4UAACAE, ID C9:52**:

- 2048 main bytes plus 128 OOB bytes per page;
- 64 pages per 128 KiB main-data erase block;
- 2048 blocks: 256 MiB main plus 16 MiB OOB.

OOB is accessed through each page's cache column, not a second flat flash
address space. `-a` and `-l` always describe **physical main-area bytes** in
these modes. Writes require multiples of `0x20000` (one erase block);
OOB reads require multiples of `0x800` (one page). Omitted length means the
remainder of the chip. No bad-block skipping, remapping or logical-address
translation occurs. Anyka OOB tags are preserved as supplied, not interpreted
as a generic bad-block map. This is intended for same-device physical
captures; it is not a transplant tool for NAND chips with different bad blocks.

Each write input must be either a full-chip file or exactly the selected range's
slice. For a full-chip input, the writer seeks to the corresponding physical
offset. For a slice it starts at byte zero. Main and OOB are mapped independently.
OOB slice size is main range length divided by 16.

The provided original `hyf2gq4_dump.bin` and `hyf2gq4_oob.bin` captures are in
`dumps/original` in this workspace. From `/home/appleflyer/dict`, a
**non-writing** restoration plan is:

```sh
./SNANDer/src/SNANDer -p ch347 -w dumps/original/hyf2gq4_dump.bin \
  --oob dumps/original/hyf2gq4_oob.bin --oob-mode raw --dry-run
```

`--dry-run` checks file types/sizes and range alignment. It does not open USB,
read every file byte, validate captured ECC mathematically or prove that the
inputs belong to the connected chip.

### Read only OOB

Save all raw spare bytes using CH347, without transferring the main bytes:

```sh
cd /home/appleflyer/dict
sudo ./SNANDer/src/SNANDer -p ch347 --read-oob hyf2gq4_oob_new.bin
```

Use `-p ch341a` instead for the original programmer. The full-chip output is
16 MiB: 128 raw OOB bytes per physical page, in ascending page order, with
no headers or skipped pages. It has the same layout as `hyf2gq4_oob.bin`.
The output must be a **new file**; existing files and symlinks are refused.

To capture only block 263's OOB (8192 bytes), for example:

```sh
sudo ./SNANDer/src/SNANDer -p ch347 --read-oob block263_oob.bin \
  -a 0x020e0000 -l 0x20000
```

`-a`/`-l` describe the main-area span, not offsets into the OOB file. Smaller
page-aligned ranges are also supported. Append `--dry-run` to check the range
and output-name availability without opening USB or creating a file.

Reading temporarily disables on-die ECC to capture **all** raw OOB bytes,
including parity, and restores the original configuration afterward where
communication permits. No `-d` is needed, and `--oob-mode` and `-v` are write-only.
Each NAND page is loaded into its internal cache, then only cache column
`0x0800` onward is transferred over USB. No erase/program commands or flash
protection changes occur. Memory use is bounded, not a full-chip allocation.
On interruption/read/output failure, the command returns an error and keeps
the partial file, reporting how many output bytes were written. Do not treat
a failed capture as a complete dump.

### Capture main and OOB together, in one pass

Use `-r MAIN --oob OOB` to save a matching **raw** pair. Both files must be new:

```sh
cd /home/appleflyer/dict/dumps/bkp1
sudo ../../SNANDer/src/SNANDer -p ch347 --spi-clock 60000000 \
  -r hyf2gq4_main_raw_fast.bin --oob hyf2gq4_oob_raw_fast.bin
```

Each page is loaded once, and all 2176 cache bytes are read in one logical
SPI transaction. The first 2048 bytes go to the main file and the remaining
128 to the OOB file. Full-chip output sizes are 256 MiB and 16 MiB. One cache
page and explicit 64-KiB output buffers per file keep capture buffers below
134 KiB; no full-chip RAM allocation is needed. Pending output is flushed
during ordinary cleanup, including interruption. Abrupt termination/power loss
can lose buffered host output; the final report checks retained file lengths.

**ECC is temporarily disabled for this paired capture.** Main bytes therefore
include any physical bit errors, rather than corrections made by the NAND.
They may differ from an ordinary ECC-enabled `-r` dump if the chip corrected
errors in that earlier read. Raw main and raw OOB belong together for a
physical backup. Keep both files; restoring that raw pair uses the writer's
`--oob-mode raw`, not regenerated ECC over potentially erroneous raw main
bytes. No `-d` or `--oob-mode` is needed while reading; `--oob-mode` remains
write-only.

As with `--read-oob`, optional `-a`/`-l` describe a page-aligned main-area range,
and output starts at byte zero for the selected slice. `--dry-run` checks the
range and both new paths without USB or file creation. Existing files/symlinks
are refused. Failure or interruption retains partial outputs and returns an
error; the final report gives counts for both files. No erase/program or
protection-register operations occur, and the original ECC configuration is
restored where communication permits.

The inherited main-read path also now batches each command/address prefix
instead of sending its bytes as separate acknowledged USB writes, and reuses
the final ready-status value for ECC checking. Plain `-r MAIN` keeps its
original ECC-enabled semantics. Required ready checks, chip-select boundaries
and CH347 acknowledgements are not removed. The paired reader avoids the
second full traversal previously required by separate main/OOB commands.
These are software changes, not a measured promise of a particular speedup.

### Bounded read-performance comparison

The reader now polls ready status without the previous fixed 1 ms sleep
after every busy reply. Interruption, transport errors and the two-second
busy deadline still stop a failed operation. Writer erase/program waits are
unchanged. File output is batched into the buffers described above instead
of making unbuffered writes for each page. Final diagnostics separate ready
polling, cache transfer and file-output timing, and count busy replies.

The user-provided pre-change CH347T baseline at 60 MHz was **9.080 seconds,
958.5 KiB/s** for 8 MiB main plus 512 KiB OOB (4096 physical pages). Keep
`perf_before_main.bin` and `perf_before_oob.bin` for exact comparison.

The user then ran both updated modes on the same CH347T, reporting adapter
firmware `0x0544` (independently queried through CA; USB `bcdDevice` also
reported `0x0544`). Results for that same range:

| Mode | Elapsed | Combined throughput | Speedup over baseline |
| --- | ---: | ---: | ---: |
| Before these changes | 9.080 s | 958.5 KiB/s | 1.00x |
| Improved normal | 5.213 s | 1669.6 KiB/s | 1.74x |
| C2 DMA | 3.160 s | 2754.6 KiB/s | 2.87x |

All four `cmp` comparisons passed; independently rechecked locally with
matching SHA-256 values across all three captures:

- Main: `5c98467380d0ad5449b12c0f11764666d487e6bc2d2060fdffa9c1b0c5df33d0`
- OOB: `e9f172d106661ad1f505eb1a387b22e8a41e3f8822b8a7f8b2cc02f82d780bbe`

DMA logged more busy replies (7635 versus 3913), but spent less time polling
(1.047 versus 2.081 seconds): faster status transactions can observe a still-
busy page more often. Neither mode inserted polling sleeps. File-output time
was about 0.01 seconds in each updated run. These measurements validate this
8-MiB raw read range on this adapter/chip, not full-chip captures or writes.

From `/home/appleflyer/dict/dumps/bkp1`, run the improved normal mode first:

```sh
sudo ../../SNANDer/src/SNANDer -p ch347 --spi-clock 60000000 \
  --ch347-mode normal -a 0 -l 0x800000 \
  -r perf_after_main.bin --oob perf_after_oob.bin
cmp perf_before_main.bin perf_after_main.bin
cmp perf_before_oob.bin perf_after_oob.bin
```

Then, if the adapter passes the firmware gate, compare DMA mode:

```sh
sudo ../../SNANDer/src/SNANDer -p ch347 --spi-clock 60000000 \
  --ch347-mode dma -a 0 -l 0x800000 \
  -r perf_dma_main.bin --oob perf_dma_oob.bin
cmp perf_before_main.bin perf_dma_main.bin
cmp perf_before_oob.bin perf_dma_oob.bin
```

These captures do not erase/program flash. All outputs must be new files.
Successful `cmp` is silent. Record the final elapsed-time/statistics lines;
offline tests alone cannot establish the speedup or hardware compatibility.
No write-speed benchmark is performed on live firmware.

### Two OOB modes

`--oob-mode metadata` is the default: turn on chip ECC, write the main bytes
and each 8-byte metadata field in the four 32-byte OOB segments, and let the
chip generate its 24 ECC-reserved bytes per segment. Captured ECC parity is
not restored in this mode. Use this for modified main data.

`--oob-mode raw` turns ECC off and programs all 2048+128 bytes per page,
including captured ECC. **The supplied ECC must match the supplied main data.**
Use matching original main/OOB captures for a raw restoration. Legacy `-d`
also selects raw mode unless an explicit conflicting metadata mode was given.

### Actual writing commands

These commands erase/rewrite the selected physical blocks. Keep stable power.
They create no backup directories and request no confirmation phrase.

Restore matching full-chip captures, **without read-back verification**:

```sh
sudo ./SNANDer/src/SNANDer -p ch347 -w dumps/original/hyf2gq4_dump.bin \
  --oob dumps/original/hyf2gq4_oob.bin --oob-mode raw
```

To limit that operation, append `-a ADDRESS -l LENGTH`. For example,
`-a 0x020e0000 -l 0x20000` selects only physical block 263, with full-size
inputs automatically seeking to that block's main/OOB data.

Replace OOB metadata while retaining the connected chip's current main data:

```sh
sudo ./SNANDer/src/SNANDer -p ch347 --write-oob dumps/original/hyf2gq4_oob.bin
```

This is **not an in-place spare-only program**. NAND erase clears main and OOB
together, so each block's main data is first read into RAM, then the block is
erased and all 64 pages are programmed with the replacement OOB metadata.
Default ECC mode preserves corrected main data and refuses an uncorrectable
read before erasing that block. Raw OOB-only mode is also available, but old
parity may be invalid for the current main data; there is no automatic parity
compatibility check. With no `-a`/`-l`, OOB-only mode rewrites the whole chip.

### Verification is optional

Append **`-v`** to either writing command to read back each programmed block.
Raw mode compares all bytes. Metadata mode compares main/metadata and requires
clean ECC status, without comparing newly generated parity to captured parity.

Without `-v`, there are no page read-back checks. OOB-only preservation reads
are still necessary, and chip identification, configuration checks, ready/WEL
polling and erase/program failure checks always remain enabled. Paired-file
mode does not read live main pages before programming.

Writes use about 142 KiB of block/page buffers, not a full-chip allocation.
Both files' bytes for a block are buffered before erasing it. SIGINT/SIGTERM
requests stop before the next erase; a block already being rewritten is
completed first if transport remains functional. Power loss, SIGKILL or a USB
failure can still leave an erased/partial block. Failures stop without retries
or relocation and return an error. Original protection/ECC configuration is
restored on cleanup where communication permits.

The legacy plain `-r`/`-w` paths retain upstream's full-range RAM allocation.
Use the new paired-file path for the provided large main/OOB images.

## Implementation references and validation

The CH347 transport adapts the GPL-2.0-or-later
[flashrom backend](https://github.com/flashrom/flashrom/blob/93546a16f2e096eab3f67a015dc7487f81a25e29/programmers/ch347_spi.c),
with explicit transaction chip select, bounded frames and strict error handling.
Configuration/status and C2 DMA semantics are checked against
[WCH's Linux driver, pinned at 57be7ed](https://github.com/WCHSoftGroup/ch34x_mphsi_master_linux/blob/57be7ed01e2adb14d34bcca5e1eb71ff6247aba7/driver/ch34x_mphsi_master_spi.c)
and its
[firmware query](https://github.com/WCHSoftGroup/ch34x_mphsi_master_linux/blob/57be7ed01e2adb14d34bcca5e1eb71ff6247aba7/driver/ch34x_mphsi_master_usb.c).
HYF geometry/ECC behavior follows the
[HYF SPI 2G v2.7 datasheet](https://datasheet.lcsc.com/datasheet/pdf/cb04af556e5204a6f44a4350d72f54c9.pdf?productCode=C7426875).
Original notices are retained; no proprietary vendor USB library is linked.

Hardware-free tests use fake USB/programmer implementations. They cannot
open a physical programmer. Passing them does not establish electrical or
hardware compatibility. The read-only hardware comparison above validates
the tested range. **No real chip has been programmed with this new code yet.**

```sh
make -C src -j1 check
```

The checks cover CH347 normal/DMA framing and failures, firmware gating,
HYF main/OOB preservation, range
mapping (full-chip files and slices), optional verification, ECC-mode handling
and failure cleanup, plus raw OOB-only and paired reads with fake programmer calls.
NAND fixtures use only a few blocks of actual disk/RAM;
full-chip input mapping is tested with sparse files, not a full-chip buffer.
Legacy DMA checks also cover caller-owned CS across separate opcode/address/
data fragments, full-page program loads, fragments larger than 64 KiB and
the shared monotonic deadline for split USB replies. No hardware write is
performed by these checks.
