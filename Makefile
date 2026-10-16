# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build for the fixed Sony CXD2841ER demodulator driver.
#
# Usage:
#   make                    # build against the running kernel
#   make LLVM=1             # force clang build (CachyOS kernels need this)
#   sudo make install       # install + load (replaces stock module)
#
# The kernel build tree at /usr/lib/modules/$(uname -r)/build (symlink to
# /usr/src/linux-*) is the only external dependency.

obj-m := cxd2841er.o
KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)/upstream/linux

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install: default
	sudo rmmod cxd2841er 2>/dev/null || true
	sudo install -m 644 upstream/linux/cxd2841er.ko /lib/modules/$(shell uname -r)/kernel/drivers/media/dvb-frontends/
	sudo depmod -a
	sudo modprobe cxd2841er
	@echo "cxd2841er fixed module loaded"

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

.PHONY: default install clean
