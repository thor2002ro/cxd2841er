#!/bin/sh
# Build the fixed cxd2841er module against the running kernel.
#
# CachyOS kernels are built with clang, so gcc fails on clang-only flags
# (-mstack-alignment, -mretpoline-external-thunk, ...). Detect this from
# the kernel's own .config and add LLVM=1 automatically.
#
# Usage: ./build.sh [install]
set -e
cd "$(dirname "$0")"

KDIR="/lib/modules/$(uname -r)/build"
CONFIG="$KDIR/.config"

if grep -q '^CONFIG_CC_IS_CLANG=y' "$CONFIG" 2>/dev/null; then
    echo "kernel built with clang -> using LLVM=1"
    MAKE_FLAGS="LLVM=1"
else
    MAKE_FLAGS=""
fi

make $MAKE_FLAGS -C "$KDIR" M="$(pwd)" modules

if [ "$1" = "install" ]; then
    echo "installing to /lib/modules/$(uname -r)/kernel/drivers/media/dvb-frontends/"
    sudo rmmod cxd2841er 2>/dev/null || true
    sudo install -m 644 cxd2841er.ko \
        /lib/modules/$(uname -r)/kernel/drivers/media/dvb-frontends/
    sudo depmod -a
    sudo modprobe cxd2841er
    echo "done"
fi