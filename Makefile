# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build for the Sony CXD2841ER and Rafael Micro R820T drivers.
#
# Usage:
#   make                    # build against the running kernel
#   make LLVM=1             # force clang build (CachyOS kernels need this)
#   ./build.sh install      # build locally and install both modules
#   ./build.sh uninstall    # remove these overrides
#
# The kernel build tree at /usr/lib/modules/$(uname -r)/build (symlink to
# /usr/src/linux-*) is the only external dependency.

obj-m := cxd2841er.o r820t.o
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)/upstream/linux

ifneq ($(shell grep -s '^CONFIG_CC_IS_CLANG=y' "$(KDIR)/.config"),)
LLVM ?= 1
export LLVM
endif

default:
	$(MAKE) -C "$(KDIR)" M="$(PWD)" modules

clean:
	$(MAKE) -C "$(KDIR)" M="$(PWD)" clean

diagnostic: dvb-check

dvb-check: tools/dvb-check.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -std=c11 -O2 -Wall -Wextra -Werror $< $(LDFLAGS) -o $@

.PHONY: default clean diagnostic
