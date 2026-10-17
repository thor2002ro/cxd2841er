#!/bin/sh
set -e
cd "$(dirname "$0")"
module_dir="/lib/modules/$(uname -r)/updates"
[ "$#" -le 1 ] || { echo "Usage: $0 [install|uninstall]" >&2; exit 2; }
case "${1:-install}" in
    install)
        make
        sudo install -D -m644 -t "$module_dir" upstream/linux/cxd2841er.ko upstream/linux/r820t.ko ;;
    uninstall)
        sudo rm -f -- "$module_dir/cxd2841er.ko" "$module_dir/r820t.ko" ;;
    *)
        echo "Usage: $0 [install|uninstall]" >&2; exit 2 ;;
esac
sudo depmod -a
