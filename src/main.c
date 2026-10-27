/*
 * Copyright (C) 2018-2022 McMCC <mcmcc@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <time.h>
#include <errno.h>
#include <limits.h>

#include "flashcmd_api.h"
#include "ch341a_spi.h"
#include "spi_nand_flash.h"
#include "programmer.h"
#include "hyf_nand.h"

struct flash_cmd prog;
extern unsigned int bsize;

#ifdef EEPROM_SUPPORT
#include "ch341a_i2c.h"
#include "bitbang_microwire.h"
#include "spi_eeprom.h"
extern struct EEPROM eeprom_info;
extern struct spi_eeprom seeprom_info;
extern char eepromname[12];
extern int eepromsize;
extern int seepromsize;
extern int mw_eepromsize;
extern int spage_size;
extern int org;
#define EHELP	" -E             select I2C EEPROM {24c01|24c02|24c04|24c08|24c16|24c32|24c64|24c128|24c256|24c512|24c1024|24c2048}\n" \
		"                select Microwire EEPROM {93c06|93c16|93c46|93c56|93c66|93c76|93c86|93c96} (need SPI-to-MW adapter)\n" \
		"                select SPI EEPROM 25xxx {25010|25020|25040|25080|25160|25320|25640|25128|25256|25512|251024}\n" \
		" -8             set organization 8-bit for Microwire EEPROM(default 16-bit) and set jumper on SPI-to-MW adapter\n" \
		" -f <addr len>  set manual address size in bits for Microwire EEPROM(default auto)\n" \
		" -s <bytes>     set page size from datasheet for fast write SPI EEPROM(default not usage)\n"
#else
#define EHELP	""
#endif

#define _VER	"1.7.9.3+ch347-oob"

void title(void)
{
#ifdef EEPROM_SUPPORT
	printf("\nSNANDer - Serial Nor/nAND/Eeprom programmeR v." _VER " by McMCC <mcmcc@mail.ru>\n\n");
#else
	printf("\nSNANDer - Spi Nor/nAND programmER v." _VER " by McMCC <mcmcc@mail.ru>\n\n");
#endif
}

void usage(void)
{
	const char use[] =
		"  Usage:\n"\
		" -h             display this message\n"\
		" -d             disable internal ECC(use read and write page size + OOB size - On-die ECC)\n"\
		" -o <bytes>     manual set OOB size with disable internal ECC(default 0)\n"\
		" -I             ECC ignore errors(for read test only)\n"\
		" -k             Skip BAD pages, try read or write in to next page\n"\
		" -L             print list support chips\n"\
		" -i             read the chip ID info\n"\
		"" EHELP ""\
		" -e             erase chip(full or use with -a [-l])\n"\
		" -l <bytes>     manually set length\n"\
		" -a <address>   manually set address\n"\
		" -w <filename>  write chip with data from filename\n"\
		" -r <filename>  read chip and save data to filename\n"\
		" -v             verify after write on chip\n";
	printf("%s", use);
	printf(
		" -p <name>      SPI programmer: ch341a (default) or ch347\n"
		" --spi-clock HZ CH347 clock: 60000000 divided by a power of two (default 3750000)\n"
		" --ch347-mode MODE  CH347 SPI transport: normal (default) or dma\n"
		" --programmer-info  report CH347 adapter/firmware, without issuing NAND commands\n"
		" --oob FILE     HYF OOB file: -w MAIN restores both; -r MAIN captures both raw\n"
		" --write-oob FILE  replace OOB while preserving live main data block by block\n"
		" --read-oob FILE   save raw OOB only to a NEW file; main bytes are not transferred\n"
		" --oob-mode MODE   writes only: metadata (default, regenerate ECC) or raw\n"
		" --skip-bad-blocks HYF OOB writes: skip reported erase/program failures; no remap\n"
		" --dry-run      check HYF paths/range and print plan without opening USB\n"
		" HYF -a/-l use physical MAIN-byte addresses: reads align to 2048-byte pages;\n"
		" writes align to 128 KiB blocks. Post-write read-back occurs only with -v.\n");
}

/* Reject malformed/negative ranges before touching a programmer. The legacy
 * path uses signed lengths, so values above LLONG_MAX are not representable.
 */
static int parse_range_number(const char *text, long long *value)
{
	char *end;
	unsigned long long parsed;

	if (!text[0] || text[0] == '-')
		return -1;

	errno = 0;
	parsed = strtoull(text, &end, 0);
	if (errno || *end || parsed > LLONG_MAX)
		return -1;

	*value = (long long)parsed;
	return 0;
}

int main(int argc, char* argv[])
{
	int c, vr = 0, svr = 0, ret = 0;
	char *str, *fname = NULL, op = 0;
	unsigned char *buf;
	int long long len = 0, addr = 0, flen = 0, wlen = 0;
	FILE *fp;
	const char *programmer = "ch341a", *oob_file = NULL, *oob_output = NULL;
	const char *spi_mode = NULL;
	int oob_mode = -1; /* Unspecified: metadata, or raw if legacy -d was given. */
	bool dry_run = false, clock_set = false, skip_bad_blocks = false;
	unsigned int spi_clock = 0;
	enum {
		OPT_OOB = 256, OPT_WRITE_OOB, OPT_OOB_MODE, OPT_DRY_RUN,
		OPT_SPI_CLOCK, OPT_READ_OOB, OPT_CH347_MODE, OPT_PROGRAMMER_INFO,
		OPT_SKIP_BAD_BLOCKS
	};
	static const struct option long_options[] = {
		{"programmer", required_argument, NULL, 'p'},
		{"spi-clock", required_argument, NULL, OPT_SPI_CLOCK},
		{"ch347-mode", required_argument, NULL, OPT_CH347_MODE},
		{"programmer-info", no_argument, NULL, OPT_PROGRAMMER_INFO},
		{"oob", required_argument, NULL, OPT_OOB},
		{"write-oob", required_argument, NULL, OPT_WRITE_OOB},
		{"read-oob", required_argument, NULL, OPT_READ_OOB},
		{"oob-mode", required_argument, NULL, OPT_OOB_MODE},
		{"dry-run", no_argument, NULL, OPT_DRY_RUN},
		{"skip-bad-blocks", no_argument, NULL, OPT_SKIP_BAD_BLOCKS},
		{"help", no_argument, NULL, 'h'},
		{NULL, 0, NULL, 0}
	};

	title();

#ifdef EEPROM_SUPPORT
	while ((c = getopt_long(argc, argv, "diIhveLkp:l:a:w:r:o:s:E:f:8", long_options, NULL)) != -1)
#else
	while ((c = getopt_long(argc, argv, "diIhveLkp:l:a:w:r:o:", long_options, NULL)) != -1)
#endif
	{
		switch(c)
		{
			case 'p':
				programmer = optarg;
				break;
			case OPT_CH347_MODE:
				spi_mode = optarg;
				break;
			case OPT_PROGRAMMER_INFO:
				op = op ? 'x' : 'P';
				break;
			case OPT_SPI_CLOCK: {
				long long value;
				if (parse_range_number(optarg, &value) < 0 || value > UINT_MAX) {
					fprintf(stderr, "Invalid SPI clock.\n");
					return 2;
				}
				spi_clock = (unsigned int)value;
				clock_set = true;
				break;
			}
			case OPT_OOB:
			case OPT_WRITE_OOB:
				if (oob_file) {
					fprintf(stderr, "Specify only one OOB file.\n");
					return 2;
				}
				oob_file = optarg;
				if (c == OPT_WRITE_OOB) {
					if (op)
						op = 'x';
					else
						op = 'O';
				}
				break;
			case OPT_READ_OOB:
				oob_output = optarg;
				if (op)
					op = 'x';
				else
					op = 'R';
				break;
			case OPT_OOB_MODE:
				if (strcmp(optarg, "metadata") == 0)
					oob_mode = 0;
				else if (strcmp(optarg, "raw") == 0)
					oob_mode = 1;
				else {
					fprintf(stderr, "OOB mode must be metadata or raw.\n");
					return 2;
				}
				break;
			case OPT_DRY_RUN:
				dry_run = true;
				break;
			case OPT_SKIP_BAD_BLOCKS:
				skip_bad_blocks = true;
				break;
#ifdef EEPROM_SUPPORT
			case 'E':
				if ((eepromsize = parseEEPsize(optarg, &eeprom_info)) > 0) {
					memset(eepromname, 0, sizeof(eepromname));
					strncpy(eepromname, optarg, 10);
					if (len > eepromsize) {
						printf("Error set size %lld, max size %d for EEPROM %s!!!\n", len, eepromsize, eepromname);
						exit(0);
					}
				} else if ((mw_eepromsize = deviceSize_3wire(optarg)) > 0) {
					memset(eepromname, 0, sizeof(eepromname));
					strncpy(eepromname, optarg, 10);
					org = 1;
					if (len > mw_eepromsize) {
						printf("Error set size %lld, max size %d for EEPROM %s!!!\n", len, mw_eepromsize, eepromname);
						exit(0);
					}
				} else if ((seepromsize = parseSEEPsize(optarg, &seeprom_info)) > 0) {
					memset(eepromname, 0, sizeof(eepromname));
					strncpy(eepromname, optarg, 10);
					if (len > seepromsize) {
						printf("Error set size %lld, max size %d for EEPROM %s!!!\n", len, seepromsize, eepromname);
						exit(0);
					}
				} else {
					printf("Unknown EEPROM chip %s!!!\n", optarg);
					exit(0);
				}
				break;
			case '8':
				if (mw_eepromsize <= 0)
				{
					printf("-8 option only for Microwire EEPROM chips!!!\n");
					exit(0);
				}
				org = 0;
				break;
			case 'f':
				if (mw_eepromsize <= 0)
				{
					printf("-f option only for Microwire EEPROM chips!!!\n");
					exit(0);
				}
				str = strdup(optarg);
				fix_addr_len = strtoll(str, NULL, *str && *(str + 1) == 'x' ? 16 : 10);
				if (fix_addr_len > 32) {
						printf("Address len is very big!!!\n");
						exit(0);
				}
				break;
#endif
			case 'I':
				ECC_ignore = 1;
				break;
			case 'k':
				Skip_BAD_page = 1;
				break;
			case 'd':
				ECC_fcheck = 0;
				_ondie_ecc_flag = 0;
				break;
			case 'l':
				if (parse_range_number(optarg, &len) < 0) {
					fprintf(stderr, "Invalid length.\n");
					return 2;
				}
				break;
			case 'o':
				str = strdup(optarg);
				OOB_size = strtoll(str, NULL, *str && *(str + 1) == 'x' ? 16 : 10);
				break;
#ifdef EEPROM_SUPPORT
			case 's':
				str = strdup(optarg);
				spage_size = strtoll(str, NULL, *str && *(str + 1) == 'x' ? 16 : 10);
				break;
#endif
			case 'a':
				if (parse_range_number(optarg, &addr) < 0) {
					fprintf(stderr, "Invalid address.\n");
					return 2;
				}
				break;
			case 'v':
				vr = 1;
				break;
			case 'i':
			case 'e':
				if(!op)
					op = c;
				else
					op = 'x';
				break;
			case 'r':
			case 'w':
				if(!op) {
					op = c;
					fname = optarg;
				} else
					op = 'x';
				break;
			case 'L':
				support_flash_list();
				exit(0);
			case 'h':
				usage();
				return 0;
			default:
				usage();
				return 2;
		}
	}

	if (op == 0 || optind != argc) {
		usage();
		return 2;
	}

	if (op == 'x' || (ECC_ignore && !ECC_fcheck) || (ECC_ignore && Skip_BAD_page) || (op == 'w' && ECC_ignore)) {
		printf("Conflicting options, only one option at a time.\n\n");
		return -1;
	}

	if (skip_bad_blocks && (!oob_file || (op != 'w' && op != 'O'))) {
		fprintf(stderr, "--skip-bad-blocks requires -w MAIN --oob OOB or --write-oob OOB.\n");
		return 2;
	}

	if (programmer_select(programmer) < 0 ||
	    (clock_set && programmer_set_spi_clock(spi_clock) < 0) ||
	    (spi_mode && programmer_set_spi_mode(spi_mode) < 0))
		return 2;

#ifdef EEPROM_SUPPORT
	if (!programmer_is_ch341a() && (eepromsize || mw_eepromsize)) {
		fprintf(stderr, "CH347 support is SPI only; I2C/Microwire require CH341A.\n");
		return 2;
	}
#endif

	if (op == 'P') {
		if (programmer_is_ch341a() || oob_file || oob_output ||
		    oob_mode >= 0 || dry_run || vr || addr || len ||
		    ECC_ignore || !ECC_fcheck || Skip_BAD_page || OOB_size) {
			fprintf(stderr, "Use --programmer-info with -p ch347 and no flash operation options.\n");
			return 2;
		}
#ifdef EEPROM_SUPPORT
		if (eepromsize || mw_eepromsize || seepromsize || spage_size) {
			fprintf(stderr, "Programmer information cannot be combined with EEPROM options.\n");
			return 2;
		}
#endif
		/* USB configuration/firmware queries only: unlike legacy -i, this
		 * does not call flash_cmd_init(), identify NAND or unlock flash.
		 */
		if (programmer_init() < 0)
			return 1;
		return programmer_shutdown() == 0 ? 0 : 1;
	}

	if (op == 'R' || (op == 'r' && oob_file)) {
		/* Separate-OOB captures are always raw and exactly physical. A paired
		 * read saves matching main/spare bytes from ONE cache load, rather
		 * than mixing ECC-corrected main with a later raw spare capture.
		 * Plain -r without --oob retains its legacy ECC behavior below.
		 */
		if ((op == 'R' && oob_file) || oob_mode >= 0 || vr ||
		    ECC_ignore || Skip_BAD_page || OOB_size) {
			fprintf(stderr, "Separate-OOB reads are raw; do not combine them with write/ECC/verify options.\n");
			return 2;
		}
#ifdef EEPROM_SUPPORT
		if (eepromsize || mw_eepromsize || seepromsize || spage_size) {
			fprintf(stderr, "HYF OOB reading cannot be combined with EEPROM options.\n");
			return 2;
		}
#endif
		struct hyf_nand_read_options options = {
			.main_path = NULL,
			.oob_path = oob_output,
			.address = (uint64_t)addr,
			.length = (uint64_t)len,
			.dry_run = dry_run,
		};
		if (op == 'r') {
			options.main_path = fname;
			options.oob_path = oob_file;
		}

		if (!dry_run && programmer_init() < 0)
			return 1;

		int result = hyf_nand_read(&options);
		if (!dry_run && programmer_shutdown() < 0)
			result = -1;

		return result == 0 ? 0 : 1;
	}

	if (oob_file) {
		if ((op != 'w' && op != 'O') || ECC_ignore || Skip_BAD_page || OOB_size ||
		    (oob_mode == 0 && !ECC_fcheck)) {
			fprintf(stderr, "Conflicting HYF OOB options. Use -w MAIN --oob OOB or --write-oob OOB.\n");
			return 2;
		}
#ifdef EEPROM_SUPPORT
		if (eepromsize || mw_eepromsize || seepromsize || spage_size) {
			fprintf(stderr, "HYF OOB options cannot be combined with EEPROM options.\n");
			return 2;
		}
#endif
		struct hyf_nand_options options = {
			.main_path = NULL,
			.oob_path = oob_file,
			.address = (uint64_t)addr,
			.length = (uint64_t)len,
			.raw = oob_mode == 1 || (oob_mode < 0 && !ECC_fcheck),
			.verify = vr != 0,
			.dry_run = dry_run,
			.skip_bad_blocks = skip_bad_blocks,
		};
		if (op == 'w')
			options.main_path = fname;

		if (!dry_run && programmer_init() < 0)
			return 1;

		int result = hyf_nand_write(&options);
		if (!dry_run && programmer_shutdown() < 0)
			result = -1;

		return result == 0 ? 0 : 1;
	}

	if (oob_mode >= 0 || dry_run) {
		fprintf(stderr, "OOB mode requires a write; dry-run requires --oob, --write-oob or --read-oob.\n");
		return 2;
	}

	if (programmer_init() < 0) {
		printf("Programmer device not found!\n\n");
		return -1;
	}

	if((flen = flash_cmd_init(&prog)) <= 0)
		goto out;

#ifdef EEPROM_SUPPORT
	if ((eepromsize || mw_eepromsize || seepromsize) && op == 'i') {
		printf("Programmer not supported auto detect EEPROM!\n\n");
		goto out;
	}
	if (spage_size) {
		if (!seepromsize) {
			printf("Use only for SPI EEPROM!\n\n");
			goto out;
		}
		if (((spage_size % 8) != 0) || (spage_size > (MAX_SEEP_PSIZE / 2))){
			printf("Invalid parameter %dB for page size SPI EEPROM!\n\n", spage_size);
			goto out;
		}
		if (op == 'r')
			printf("Ignored set page size SPI EEPROM on READ.\n");
		else
			printf("Setting page size %dB for write.\n", spage_size);
	}
#endif
	if (op == 'i')
		goto okout;
	if (OOB_size) {
		if (ECC_fcheck == 1) {
			printf("Ignore option -o, use with -d only!\n");
			OOB_size = 0;
		} else {
			if (OOB_size > 256) {
				printf("Error: Maximum set OOB size <= 256!!!\n");
				goto out;
			}
			if (OOB_size < 64) {
				printf("Error: Minimum set OOB size >= 64!!!\n");
				goto out;
			}
			printf("Set manual OOB size = %d.\n", OOB_size);
		}
	}
	if (op == 'e') {
		printf("ERASE:\n");
		if(addr && !len)
			len = flen - addr;
		else if(!addr && !len) {
			len = flen;
			printf("Set full erase chip!\n");
		}
		if(len % bsize) {
			printf("Please set len = 0x%016llX multiple of the block size 0x%08X\n", len, bsize);
			goto out;
		}
		printf("Erase addr = 0x%016llX, len = 0x%016llX\n", addr, len);
		ret = prog.flash_erase(addr, len);
		if(!ret){
			printf("Status: OK\n");
			goto okout;
		}
		else
			printf("Status: BAD(%d)\n", ret);
		goto out;
	}

	if ((op == 'r') || (op == 'w')) {
		if(addr && !len)
			len = flen - addr;
		else if(!addr && !len) {
			len = flen;
		}
		buf = (unsigned char *)malloc(len + 1);
		if (!buf) {
			printf("Malloc failed for read buffer.\n");
			goto out;
		}
	}

	if (op == 'w') {
		printf("WRITE:\n");
		fp = fopen(fname, "rb");
		if (!fp) {
			printf("Couldn't open file %s for reading.\n", fname);
			free(buf);
			goto out;
		}
		wlen = fread(buf, 1, len, fp);
		if (ferror(fp)) {
			printf("Error reading file [%s]\n", fname);
			if (fp)
				fclose(fp);
			free(buf);
			goto out;
		}
		if(len == flen)
			len = wlen;
		else if (wlen != len) {
			fprintf(stderr, "Input is shorter than the requested write; no write performed.\n");
			fclose(fp);
			free(buf);
			goto out;
		}
		if (len <= 0) {
			fprintf(stderr, "Refusing an empty write.\n");
			fclose(fp);
			free(buf);
			goto out;
		}
		printf("Write addr = 0x%016llX, len = 0x%016llX\n", addr, len);
		ret = prog.flash_write(buf, addr, len);
		if(ret > 0) {
			printf("Status: OK\n");
			if (vr) {
				op = 'r';
				svr = 1;
				printf("VERIFY:\n");
				goto very;
			}
		}
		else
			printf("Status: BAD(%d)\n", ret);
		fclose(fp);
		free(buf);
		/* Upstream fell through to the error exit after a successful write
		 * without -v. Skipping optional verification is still success.
		 */
		if (ret > 0)
			goto okout;
		goto out;
	}

very:
	if (op == 'r') {
		if (!svr) printf("READ:\n");
		else memset(buf, 0, len);
		printf("Read addr = 0x%016llX, len = 0x%016llX\n", addr, len);
		ret = prog.flash_read(buf, addr, len);
		if (ret < 0) {
			printf("Status: BAD(%d)\n", ret);
			free(buf);
			goto out;
		}
		if (svr) {
			unsigned char expected[4096];
			bool equal = fseeko(fp, 0, SEEK_SET) == 0;
			long long offset = 0;

			/* Compare every requested byte, including FF and the last byte.
			 * The old unsigned-char/getc loop could misreport a mismatch as
			 * success after advancing its index. Verification remains opt-in.
			 */
			while (equal && offset < len) {
				size_t count = sizeof(expected);
				if ((unsigned long long)(len - offset) < count)
					count = (size_t)(len - offset);
				if (fread(expected, 1, count, fp) != count ||
				    memcmp(expected, buf + offset, count) != 0)
					equal = false;
				else
					offset += count;
			}

			if (equal) {
				printf("Status: OK\n");
				fclose(fp);
				free(buf);
				goto okout;
			}
			else
				printf("Status: BAD\n");
			fclose(fp);
			free(buf);
			goto out;
		}
		fp = fopen(fname, "wb");
		if (!fp) {
			printf("Couldn't open file %s for writing.\n", fname);
			free(buf);
			goto out;
		}
		fwrite(buf, 1, len, fp);
		if (ferror(fp)){
			printf("Error writing file [%s]\n", fname);
			fclose(fp);
			free(buf);
			goto out;
		}
		fclose(fp);
		free(buf);
		printf("Status: OK\n");
		goto okout;
	}

out:	//exit with errors
	programmer_shutdown();
	return -1;
okout:	//exit without errors
	if (programmer_shutdown() < 0)
		return -1;
	return 0;
}
