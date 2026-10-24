# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build for the modified DVB modules in upstream/linux/.
#
# Usage:
#   make                    # build against the running kernel
#   make LLVM=1             # force clang build (CachyOS kernels need this)
#   ./build.sh install      # build locally and install modified modules
#   ./build.sh uninstall    # remove these overrides
#
# The kernel build tree at /usr/lib/modules/$(uname -r)/build (symlink to
# /usr/src/linux-*) is the only external dependency.

.DEFAULT_GOAL := default
KERNELRELEASE ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KERNELRELEASE)/build
MODULE_SRC := $(CURDIR)/upstream/linux
-include $(KDIR)/.config
include upstream/linux/Kbuild
MODULE_CONFIGS := DVB_CXD2841ER MEDIA_TUNER_R820T DVB_MN88472 DVB_MN88473 DVB_RTL2832 \
	MEDIA_TUNER_TDA18250 DVB_CORE DVB_USB DVB_USB_V2 DVB_USB_DIB0700 DVB_USB_RTL28XXU
ifeq ($(CONFIG_DVB_CORE),y)
MODULE_CONFIGS := $(filter-out DVB_CORE,$(MODULE_CONFIGS))
endif

ifneq ($(shell grep -s '^CONFIG_CC_IS_CLANG=y' "$(KDIR)/.config"),)
LLVM ?= 1
export LLVM
endif

default: check-config
	$(MAKE) -C "$(KDIR)" M="$(MODULE_SRC)" modules

check-config:
ifeq ($(CONFIG_DVB_CORE),y)
	@echo "WARNING: CONFIG_DVB_CORE=y; skipping dvb-core. Built-in DVB core is unchanged; core disconnect fixes will not be installed." >&2
endif
	@for config in $(MODULE_CONFIGS); do \
		grep -qx "CONFIG_$$config=m" "$(KDIR)/.config" || { \
			echo "CONFIG_$$config=m is required in $(KDIR)/.config" >&2; exit 1; }; \
	done

module-names:
	@printf '%s\n' $(obj-m:.o=)

clean:
	$(MAKE) -C "$(KDIR)" M="$(MODULE_SRC)" clean

diagnostic: dvb-check

dvb-check: tools/dvb-check.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -std=c11 -O2 -Wall -Wextra -Werror $< $(LDFLAGS) -o $@

.PHONY: default check-config module-names clean diagnostic
