#!/bin/sh
set -eu
cd "$(dirname "$0")"
[ "$#" -le 1 ] || { echo "Usage: $0 [install|uninstall]" >&2; exit 2; }
KERNELRELEASE=${KERNELRELEASE:-$(uname -r)}
case "$KERNELRELEASE" in
    ''|.|..|*/*) echo "Invalid kernel release: $KERNELRELEASE" >&2; exit 2 ;;
esac
export KERNELRELEASE
module_dir="${INSTALL_MOD_PATH:-}/lib/modules/$KERNELRELEASE/updates"
modules=$(make --no-print-directory -s module-names)
sudo_cmd=
[ "$(id -u)" -eq 0 ] || [ -n "${INSTALL_MOD_PATH:-}" ] || sudo_cmd=sudo
case "${1:-install}" in
    install)
        make
        # Validate the entire set before changing any installed file.
        for module in $modules; do
            file="upstream/linux/$module.ko"
            [ -f "$file" ] || { echo "Missing module: $file" >&2; exit 1; }
            vermagic=$(modinfo -F vermagic "$file")
            [ "${vermagic%% *}" = "$KERNELRELEASE" ] || {
                echo "$file targets a different kernel: $vermagic" >&2; exit 1;
            }
        done
        for module in $modules; do
            $sudo_cmd install -D -m644 "upstream/linux/$module.ko" "$module_dir/$module.ko"
        done ;;
    uninstall)
        for module in $modules; do
            $sudo_cmd rm -f -- "$module_dir/$module.ko"
        done ;;
    *)
        echo "Usage: $0 [install|uninstall]" >&2; exit 2 ;;
esac
if [ -n "${INSTALL_MOD_PATH:-}" ]; then
    $sudo_cmd depmod -b "$INSTALL_MOD_PATH" -a "$KERNELRELEASE"
else
    $sudo_cmd depmod -a "$KERNELRELEASE"
fi
