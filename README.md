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
Only single-lane SPI is implemented; speed and hardware compatibility have
not yet been measured on a physical CH347.

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

The provided `hyf2gq4_dump.bin` and `hyf2gq4_oob.bin` have the required full-chip
sizes. From `/home/appleflyer/dict`, a **non-writing** plan is:

```sh
./SNANDer/src/SNANDer -p ch347 -w hyf2gq4_dump.bin \
  --oob hyf2gq4_oob.bin --oob-mode raw --dry-run
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
protection changes occur. Memory use is page-sized, not a full-chip allocation.
On interruption/read/output failure, the command returns an error and keeps
the partial file, reporting how many output bytes were written. Do not treat
a failed capture as a complete dump.

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
sudo ./SNANDer/src/SNANDer -p ch347 -w hyf2gq4_dump.bin \
  --oob hyf2gq4_oob.bin --oob-mode raw
```

To limit that operation, append `-a ADDRESS -l LENGTH`. For example,
`-a 0x020e0000 -l 0x20000` selects only physical block 263, with full-size
inputs automatically seeking to that block's main/OOB data.

Replace OOB metadata while retaining the connected chip's current main data:

```sh
sudo ./SNANDer/src/SNANDer -p ch347 --write-oob hyf2gq4_oob.bin
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
Configuration/status semantics are also checked against
[WCH's Linux driver](https://github.com/WCHSoftGroup/ch34x_mphsi_master_linux/blob/main/driver/ch34x_mphsi_master_spi.c).
HYF geometry/ECC behavior follows the
[HYF SPI 2G v2.7 datasheet](https://datasheet.lcsc.com/datasheet/pdf/cb04af556e5204a6f44a4350d72f54c9.pdf?productCode=C7426875).
Original notices are retained; no proprietary vendor USB library is linked.

Hardware-free tests use fake USB/programmer implementations. They cannot
open a physical programmer. Passing them does not establish electrical or
hardware compatibility. **No real chip has been programmed with this new code yet.**

```sh
make -C src -j1 check
```

The checks cover CH347 framing/failures and HYF main/OOB preservation, range
mapping (full-chip files and slices), optional verification, ECC-mode handling
and failure cleanup, plus raw OOB-only reads with fake programmer calls.
NAND fixtures use only a few blocks of actual disk/RAM;
full-chip input mapping is tested with sparse files, not a full-chip buffer.
