# Use the installed libusb by default: a normal build must not configure or
# install a second library. USB_CFLAGS/USB_LIBS and CC remain overridable for
# cross/static builds. No build target accesses a programmer.
CC ?= cc
PKG_CONFIG ?= pkg-config
EEPROM_SUPPORT ?= y
USB_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libusb-1.0)
USB_LIBS ?= $(shell $(PKG_CONFIG) --libs libusb-1.0)
CPPFLAGS += -D_FILE_OFFSET_BITS=64 $(USB_CFLAGS)
CFLAGS ?= -O2 -std=gnu99 -Wall
LDLIBS += $(USB_LIBS) -pthread

OBJS= flashcmd_api.o spi_controller.o spi_nand_flash.o spi_nor_flash.o \
      ch341a_spi.o ch347_spi.o programmer.o hyf_nand.o timer.o main.o

ifeq ($(EEPROM_SUPPORT),y)
CPPFLAGS += -DEEPROM_SUPPORT
OBJS += ch341a_i2c.o i2c_eeprom.o spi_eeprom.o
OBJS += bitbang_microwire.o mw_eeprom.o ch341a_gpio.o
endif

.PHONY: all check clean
.NOTPARALLEL: check
all: SNANDer

SNANDer: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

.c.o:
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(OBJS:.o=.d)

# These binaries link fake programmer/USB implementations, never libusb.
# Run serially; their fixtures are bounded to a couple of erase blocks.
TEST_CFLAGS = -O1 -std=c99 -Wall -Wextra -Werror -UNDEBUG
../build/test_ch347_spi: tests/test_ch347_spi.c ch347_spi.c ch347_spi.h
	mkdir -p ../build
	$(CC) $(TEST_CFLAGS) -I. tests/test_ch347_spi.c ch347_spi.c -o $@

../build/test_hyf_nand: tests/test_hyf_nand.c hyf_nand.c hyf_nand.h programmer.h
	mkdir -p ../build
	$(CC) $(TEST_CFLAGS) -I. tests/test_hyf_nand.c hyf_nand.c -o $@

../build/test_hyf_oob_read: tests/test_hyf_oob_read.c hyf_nand.c hyf_nand.h programmer.h
	mkdir -p ../build
	$(CC) $(TEST_CFLAGS) -I. tests/test_hyf_oob_read.c hyf_nand.c -o $@

check: ../build/test_ch347_spi ../build/test_hyf_nand ../build/test_hyf_oob_read
	../build/test_ch347_spi
	../build/test_hyf_nand
	../build/test_hyf_oob_read

clean:
	rm -f $(OBJS) $(OBJS:.o=.d) SNANDer
