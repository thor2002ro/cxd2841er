# cxd2841er-fix

Out-of-tree build of the upstream Sony CXD2841ER DVB demodulator driver with a
reset-timing fix for the Astrometa DVB-T2 stick (RTL2832P + CXD2837ER + R828D,
USB ID `15f4:0131`), I2C error handling for tuning and measurements, and isolated
capabilities for each attached tuner.

## Version and debug logs

This driver revision is **1.0.0**. `CXD2841ER_VERSION` in `cxd2841er.c` is
the version used by both the module metadata and the attachment log. Bump it
when releasing subsequent driver changes.

```sh
modinfo -F version ./cxd2841er.ko    # local build
modinfo -F version cxd2841er        # installed module file
cat /sys/module/cxd2841er/version   # module currently running in the kernel
```

Check the running version as well as the file version: replacing a module on
disk does not replace a copy that is already loaded. A successful attachment
also logs a line like:

```text
cxd2841er v1.0.0: CXD2837ER DVB-C/T/T2 frontend, chip ID 0xb1
```

The extra traces use the kernel's [dynamic debug](https://docs.kernel.org/admin-guide/dynamic-debug-howto.html)
facility. On kernels with `/proc/dynamic_debug/control`, enable the useful
tuning and error messages with:

```sh
printf '%s\n' \
  'module cxd2841er func cxd2841er_attach +p' \
  'module cxd2841er func cxd2841er_tune_done +p' \
  'module cxd2841er func cxd2841er_set_frontend_* +p' \
  'module cxd2841er format "snapshot release failed" +p' \
  | sudo tee /proc/dynamic_debug/control >/dev/null
sudo dmesg -w
```

Reproduce a tune or retune while capturing the log. For attachment details,
enable tracing before reconnecting the tuner. The messages show:

- The failed tune-completion stage: bank selection, software reset, or TS enable.
- Requested delivery system, frequency, symbol rate or bandwidth, lock status,
  and return code. `ret=0` means no reported operation error; the `0x10` bit in
  `status` indicates lock. `wait_lock=0` means lock polling was skipped.
- Snapshot-release errors alongside the original read error, even if both fail.
- I2C addresses, crystal selection, and frontend flags at attachment.

These filters leave the existing per-register debug dump disabled unless it
was enabled separately. Turn off this module's debug messages afterward:

```sh
printf '%s\n' 'module cxd2841er -p' | sudo tee /proc/dynamic_debug/control >/dev/null
```

## The bug

`cxd2841er_tune_done()` writes `0xfe = 0x01` (SW reset) then `0xc3 = 0x00`
(enable TS output) back-to-back:

```c
/* SW Reset */
cxd2841er_write_reg(priv, I2C_SLVT, 0xfe, 0x01);
/* Enable TS output */
cxd2841er_write_reg(priv, I2C_SLVT, 0xc3, 0x00);
```

The CXD2837ER NAKs every I2C transaction for ~10 ms while the SW reset is in
progress (measured: NAK'd at 0–4 ms, ACKed at 9.9 ms). The RTL2832 I2C-master
silently swallows the NAK'd write — the USB control transfer "succeeds", so the
driver never sees an error — and `0xc3` stays `0x01` (TS output disabled).

Symptoms: demod locks fine (TSLOCK=1, C/N ~26 dB, UCB=0) but the USB bulk pipe
carries only 0xFF idle filler; no TS packets ever reach the demux. Every other
register (TS pin config, clock mode, PIP settings) is correct.

## The fix

Wait for the SW reset to finish before re-enabling TS output:

```c
/* SW Reset */
cxd2841er_write_reg(priv, I2C_SLVT, 0xfe, 0x01);
/*
 * CXD2837ER NAKs all I2C transactions for ~10ms while the SW reset
 * is in progress. A write issued during that window is silently
 * dropped (the RTL2832 I2C master swallows the NAK), leaving the
 * TS output disabled even though the enable write below "succeeded".
 * Wait for the reset to complete before re-enabling TS output.
 */
usleep_range(15000, 20000);
/* Enable TS output */
cxd2841er_write_reg(priv, I2C_SLVT, 0xc3, 0x00);
```

## Driver error handling

- Failed bank-selection, reset and TS-enable writes are returned through both
  tuning paths. The existing 15–20 ms reset delay is preserved.
- Status, carrier-offset, signal-strength, SNR, BER and packet-error readers
  check I2C results before using register data. Failed measurements are marked
  unavailable rather than reported as valid readings.
- Frozen register snapshots are released after a read failure. If releasing
  the snapshot also fails, the original error is retained.
- Each frontend customizes its own capability table, so mixed chip variants
  cannot change the defaults used by subsequently attached tuners.

## Build

Requires only the standard kernel build tree (headers + .config), no full kernel
source:

```sh
./build.sh            # auto-detects clang (LLVM=1) for CachyOS kernels
./build.sh install    # also installs to /lib/modules and loads the module
# or manually:
make                   # gcc kernels
make LLVM=1            # clang kernels (CachyOS, some Arch builds)
sudo make install
```

`build.sh` checks `CONFIG_CC_IS_CLANG` in the kernel `.config` and adds
`LLVM=1` automatically — plain `make` fails on CachyOS with
`gcc: unrecognized command-line option '-mstack-alignment=8'` etc.

## Verify

The software regression checks need Python 3, a C compiler with AddressSanitizer
and UndefinedBehaviorSanitizer, and the Linux DVB userspace headers:

```sh
make test
```

They compile the affected functions directly from `cxd2841er.c` with a simulated
register bus, inject failures at each transfer, check snapshot cleanup and
measurement validity, and exercise all mixed-chip attachment orders. They do
not require a tuner or kernel build tree. They do not prove hardware timing or
replace a module build against the target kernel.

For verification with a real tuner:

```sh
dvbv5-scan -a 0 -f 1 -o ~/tvbox/dvb/channels.conf ~/tvbox/dvb/muxes
dvbv5-zap  -a 0 -f 1 -r -o /tmp/stream.ts -c ~/tvbox/dvb/channels.conf 'RTSH Kukesi'
mpv /tmp/stream.ts
```

## Notes

- `cxd2841er.h`/`cxd2841er_priv.h` are the in-tree headers, vendored here.
  The kernel build tree and its build tools are still required.
- BTF metadata is generated by kbuild when the matching `vmlinux` and
  `pahole` are available; otherwise it may be skipped.
- The driver is based on upstream Linux (GPL-2.0) with the reset delay,
  tuning/measurement error handling, and per-frontend capability fixes above.
