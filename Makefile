# SPDX-License-Identifier: GPL-2.0
#
# Out-of-tree build for the modified DVB modules in upstream/linux/.
#
# Usage:
#   make                    # build against the running kernel
#   make LLVM=1             # force clang build (CachyOS kernels need this)
#   make clean              # remove build outputs, even without headers
#   make install            # build locally and install modified modules
#   make uninstall          # remove these overrides
#   make dkms               # register, build, and install through DKMS
#
# The kernel build tree at /usr/lib/modules/$(uname -r)/build (symlink to
# /usr/src/linux-*) is the only external dependency.

.DEFAULT_GOAL := all
KERNELRELEASE ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KERNELRELEASE)/build
MODULE_SRC := $(CURDIR)/upstream/linux
DKMS_SOURCE_TREE ?= /usr/src
# Only the core setting is needed here; kbuild reads the full configuration.
CONFIG_DVB_CORE := $(shell sed -n 's/^CONFIG_DVB_CORE=//p' "$(KDIR)/.config" 2>/dev/null)
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

all default: modules

modules: check-config
	$(MAKE) -C "$(KDIR)" M="$(MODULE_SRC)" modules

check-config:
	@test -f "$(KDIR)/Makefile" -a -f "$(KDIR)/.config" || { \
		echo "Missing configured kernel build tree at $(KDIR); install headers for $(KERNELRELEASE) or set KDIR." >&2; exit 1; }
ifeq ($(CONFIG_DVB_CORE),y)
	@echo "WARNING: CONFIG_DVB_CORE=y; skipping dvb-core. Built-in DVB core is unchanged; core disconnect fixes will not be installed." >&2
endif
	@for config in $(MODULE_CONFIGS); do \
		grep -qx "CONFIG_$$config=m" "$(KDIR)/.config" || { \
			echo "CONFIG_$$config=m is required in $(KDIR)/.config" >&2; exit 1; }; \
	done

module-names:
	@printf '%s\n' $(obj-m:.o=)

install uninstall:
	KERNELRELEASE="$(KERNELRELEASE)" KDIR="$(KDIR)" INSTALL_MOD_PATH="$(INSTALL_MOD_PATH)" ./build.sh $@

dkms: SHELL := /bin/bash
dkms: check-config
	@command -v dkms >/dev/null || { echo "Install DKMS first." >&2; exit 1; }
	@set -e; \
	kernelver="$(KERNELRELEASE)"; kernel_source_dir="$$(cd "$(KDIR)" && pwd -P)"; \
	source ./dkms.conf; \
	sudo_cmd=; [ "$$(id -u)" = 0 ] || sudo_cmd=sudo; \
	source_dir="$(DKMS_SOURCE_TREE)/$$PACKAGE_NAME-$$PACKAGE_VERSION"; \
	dkms_args=(-m "$$PACKAGE_NAME" -v "$$PACKAGE_VERSION" --sourcetree "$(DKMS_SOURCE_TREE)"); \
	kernel_args=(-k "$$kernelver" --kernelsourcedir "$$kernel_source_dir"); \
	$$sudo_cmd install -d "$$source_dir"; \
	$$sudo_cmd cp -a Makefile dkms.conf upstream "$$source_dir/"; \
	$$sudo_cmd make -C "$$source_dir" clean; \
	status=$$(dkms status "$${dkms_args[@]}"); \
	if [ -z "$$status" ]; then \
		$$sudo_cmd dkms add "$${dkms_args[@]}" "$${kernel_args[@]}"; \
	fi; \
	$$sudo_cmd dkms build "$${dkms_args[@]}" "$${kernel_args[@]}" --force; \
	$$sudo_cmd dkms install "$${dkms_args[@]}" "$${kernel_args[@]}" --force

clean:
	find "$(MODULE_SRC)" -type f \( -name '*.o' -o -name '*.ko' \
		-o -name '*.mod' -o -name '*.mod.c' -o -name '*.order' \
		-o -name '*.symvers' -o -name '.*.cmd' -o -name '.*.d' \
		-o -name 'built-in.a' -o -name 'lib.a' \) -delete
	find "$(MODULE_SRC)" -type d -name .tmp_versions -prune -exec rm -rf -- {} +
	$(RM) dvb-check

diagnostic: dvb-check

dvb-check: tools/dvb-check.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -std=c11 -O2 -Wall -Wextra -Werror $< $(LDFLAGS) -o $@

.PHONY: all default modules check-config module-names install uninstall dkms clean diagnostic
