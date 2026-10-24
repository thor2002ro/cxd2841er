# cxd2841er-fix

This repository builds patched Linux TV-tuner drivers for the Astrometa DVB-T2
stick and the Xbox One Digital TV Tuner. It packages the eleven affected DVB kernel
modules and provides scripts to install or remove them without replacing the
stock module files.

## Version checks

All eleven custom modules expose a version through their module metadata:

| Module | Version |
| --- | --- |
| `cxd2841er` | 1.1.11 |
| `r820t` | 1.0.3 |
| `mn88472` | 1.0.1 |
| `mn88473` | 1.0.1 |
| `rtl2832` | 1.0.0 |
| `tda18250` | 1.0.0 |
| `dvb-core` | 1.0.0 |
| `dvb-usb` | 1.0.5 |
| `dvb_usb_v2` | 2.0.1 |
| `dvb-usb-dib0700` | 1.0.3 |
| `dvb-usb-rtl28xxu` | 1.0.0 |

`CXD2841ER_VERSION` in `cxd2841er.c` is used by both the module metadata and
the attachment log. Bump the affected module's version when releasing later
driver changes.

```sh
for module in $(make -s module-names); do
	printf '%s: ' "$module"
	modinfo -F version "upstream/linux/$module.ko"
done
modinfo -F version cxd2841er       # installed module file
cat /sys/module/cxd2841er/version  # module currently running in the kernel
```

Check the running version as well as the file version: replacing a module on
disk does not replace a copy that is already loaded. A successful attachment
also logs a line like:

```text
cxd2841er v1.1.11: CXD2837ER DVB-C/T/T2 frontend, chip ID 0xb1
```

## Module changes

### `cxd2841er` 1.1.11

- Waits 15–20 ms after the Sony software reset, verifies that TS output was
  enabled, and returns failed bank, reset, TS, tuner, gate and IF operations.
- Checks register access throughout setup, tuning, power changes and
  measurements. A failed tune invalidates cached state so later polling cannot
  report an earlier lock; recovery reinitializes the demodulator when needed.
- Keeps capabilities per frontend, validates chip identification and tuning
  requests, and requires valid acquisition state for lock, carrier, CNR and
  detected-QAM results.
- Corrects DVB-C and satellite carrier calculations, DVB-S/S2 switching,
  24/41 MHz crystal handling, signed/fractional CNR conversion and PRE/POST
  error-counter placement.
- Exposes `stats_mask`: status is always read; strength (`1`) and CNR (`2`) are
  enabled by default, while block errors (`4`), bit errors (`8`) and detected
  DVB-C QAM (`16`) remain optional.

### `r820t` 1.0.3

- Returns I2C, gate, calibration, standby and tuning failures instead of
  continuing with stale state, including `-ETIMEDOUT` when the PLL does not
  lock without an earlier I/O error.
- Clears cached lock and standard state after failures, checks every remaining
  calibration step, and preserves the original failure during gate cleanup.
- Releases shared tuner state when attachment fails and reports AGC as an
  uncalibrated relative strength rather than an invented dBm value.

### `mn88472` 1.0.1

- Writes firmware through the fixed program-data port, validates configuration
  and transfer limits, identifies parity failures, and clears active/statistics
  state before power or tune changes.
- Rejects false DVB-T, DVB-T2 and DVB-C locks using the demodulator error flags,
  QAM validity and VQLOCK, and accepts only DVB-C acquisition stages 8 or 9.
- Programs cable bandwidth and QAM, DVB-T mode and guard interval, and DVB-T2
  PLP selection without leaking settings from a previous tune. Explicit PLP
  IDs use ID lookup; the automatic fallback uses index lookup.
- Waits 10 ms after resetting the demodulator state machine before returning
  from a successful tune, matching the vendor sequence.
- Reports detected DVB-T modulation, hierarchy, code rates, FFT and guard;
  DVB-T2 modulation, code rate, FFT and guard; and DVB-C modulation and symbol
  rate only after complete valid reads.
- Keeps its state until the final frontend reference is released during USB
  removal. Its `stats_mask` uses the same bits and strength-plus-CNR default as
  the Sony module.

### `mn88473` 1.0.1

- Included in the build, version checks, installation and removal alongside
  MN88472; requires `CONFIG_DVB_MN88473=m` in the target kernel configuration.

- Follows Astrometa's Windows upload sequence: writes `0xf0 = 0x20` before
  entering download mode, uses single-byte writes through the fixed `0xf6`
  program-data port, and pauses at least 1 ms every 200 bytes. On upload `-EAGAIN`
  or an image parity failure, re-enters download mode and restarts the entire
  image, with at most five attempts and a 20 ms pause between attempts.
  Permanent transfer errors return immediately.

- Logs the upload attempt, failing byte offset and error; validates firmware size
  and transfer limits, and clears active/statistics state before initialization.

- Clears old lock and statistics on status-read errors and starts each status
  query unlocked, including inactive and early acquisition states.

- Retains firmware parity checking before enabling output. Cold boot and repeated
  warm reboot with real MN88473 hardware still need verification.

### `rtl2832` 1.0.0

- Stops frontend setup and returns the tuner `set_params` error instead of
  programming the demodulator after a failed tuner operation.

### `tda18250` 1.0.0

- Validates platform data and chip-ID reads, returns the final charge-pump
  write error, and fails probe if initial standby cannot be entered.
- Uses the I2C client directly for power control so teardown does not depend on
  a frontend pointer that may already be detached, and publishes tuner state
  only after setup succeeds.

### `dvb-core` 1.0.0

- Wakes blocked demux and DVR readers during device removal and returns
  `-ENODEV` instead of leaving them asleep.
- Marks active mmap queues as failed and lets blocking `DMX_DQBUF` release the
  mutex needed by device removal.

### `dvb-usb` 1.0.5

- Rolls back PID filters, the PID parser, streaming, URBs and feed counts when
  the first legacy feed fails to start. Final-feed cleanup still completes if
  stopping the device reports an error.
- Drops data from failed bulk transfers and queues immediate recovery,
  including overflow and protocol errors. It pauses resubmission, drains URBs and
  checks the parent port: connected/enabled clears the endpoint and restarts
  receives; connected/disabled queues a USB-core device reset; disconnected
  stops recovery. Only failed repairs wait 500 ms before retrying, without
  restarting a stopped feed. A device reset can rebind the tuner and require
  TVHeadend to reopen it.
- Reports completion length, buffer capacity and endpoint in recovery warnings;
  resubmission failures are identified separately.
- Rebuild and deploy together with `dvb-usb-dib0700` 1.0.3; the shared
  stream/device structures changed.
- Exposes early frontend cleanup for bridge drivers, clears detached frontend
  pointers, and preserves the original error while reporting rollback errors.

### `dvb_usb_v2` 2.0.1

- Initializes remote-control delayed work before any early return so cleanup
  can always cancel it safely.
- Rolls back submitted URBs, filters, streaming state and feed counts when any
  stream-start stage fails, and wakes waiters after clearing the streaming bit.

### `dvb-usb-dib0700` 1.0.3

- Rejects short USB control and I2C transfers, guards transfer lengths, returns
  clock/version errors, and cleans up a probe whose version read fails.
- Stops the Xbox power/reset sequence on the first GPIO error and fully unwinds
  partial demodulator or tuner attachment.
- Detaches Xbox frontends before unregistering their tuner and demodulator I2C
  clients, retaining module references until client removal is complete.
- Rounds bulk receive buffers upward to complete 512-byte USB packets. This
  prevents a final packet from overflowing the old 39,480-byte allocation.

### `dvb-usb-rtl28xxu` 1.0.0

- Treats an incomplete USB control transfer as `-EIO` rather than accepting it
  as a successful register operation.

## Sony and R820T debug logs

Failures are reported in `dmesg` without debug flags. Warnings cover I2C
transfers, Sony tuner programming and checked gate callbacks, IF reads, reset/TS enable
and readback, frontend setup, snapshot release, and R820T initialization,
standby and tuning (including calibration and PLL failures). Tune warnings
identify the I2C device, stage, frequency, delivery system and error code;
TS warnings also include the readback value. A failed gate close or snapshot
release is reported even when an earlier operation has already failed.

Demodulator lock-wait expiry is reported too, without changing the existing
frontend return value. No lock-wait warning is emitted when `NO_WAIT_LOCK`
skips terrestrial/cable polling. Repeated warnings are rate-limited using the
kernel's standard mechanism, so a persistent fault can suppress later messages.
Successful tuning remains quiet unless debug tracing is enabled. These
messages describe driver-detected failures; they cannot detect every missing
transport packet or an application-side timeout.

```sh
sudo dmesg -w | grep --line-buffered -E 'cxd2841er|r820t'
```

The extra traces use the kernel's [dynamic debug](https://docs.kernel.org/admin-guide/dynamic-debug-howto.html)
facility. On kernels with `/proc/dynamic_debug/control`, enable the useful
tuning traces with:

```sh
printf '%s\n' \
  'module cxd2841er func cxd2841er_attach +p' \
  'module cxd2841er func cxd2841er_tune_done +p' \
  'module cxd2841er func cxd2841er_set_frontend_* +p' \
  | sudo tee /proc/dynamic_debug/control >/dev/null
sudo dmesg -w
```

Reproduce a tune or retune while capturing the log. For attachment details,
enable tracing before reconnecting the tuner. The messages show:

- The failed tune-completion stage: bank selection, software reset, or TS enable.
- TS-enable readback (`stage=TS verify`, `ts_enable=0x00` on success). An
  acknowledged write that leaves output disabled returns `-EIO`; no automatic
  reset or retry is added.
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
- Tuner programming and I2C-gate failures reach both tuning paths. Gate closure
  is attempted after tuner failure, preserving the original error if both fail.
- Failed tuner IF-frequency reads stop IF programming and propagate through all
  bandwidth paths, including cold starts. Valid zero IF and fixed-IF defaults
  remain supported.
- R820T calibration and normal tuning return `-ETIMEDOUT` when the PLL reports
  no lock, preserving an actual I2C error when one is present.

## Signal measurements

Terrestrial, cable and ISDB-T signal strength is now the demodulator's raw
16-bit relative AGC reading, rather than an absolute dBm estimate. The old
empirical formulas were not calibrated for the attached R828D tuner and could
produce implausible positive dBm values. Treat the relative value as an AGC
indicator, not a calibrated input level or a cross-device sensitivity comparison.
Satellite strength reporting, C/N, BER, UCB and hardware calibration settings
are unchanged. These correctness changes do not establish an RF sensitivity gain.

## Build

Requires only the standard kernel build tree (headers + .config), no full kernel
source:

```sh
make                  # builds all eleven modified modules
./build.sh install    # builds locally, then installs the modified modules (also the default)
./build.sh uninstall  # removes the modified modules from updates/, then runs depmod
```

`make` detects `CONFIG_CC_IS_CLANG=y` in the kernel build tree and enables
`LLVM=1` automatically; an explicit `LLVM` setting takes precedence.
It requires every overridden driver to be configured as a module. When
`CONFIG_DVB_CORE=m`, all eleven modules are built. When `CONFIG_DVB_CORE=y`, the
built-in core cannot be replaced, so the build warns, skips `dvb-core`, and
builds the other ten modules; the core removal fixes are then absent.

`build.sh` builds in parallel, gets the install/remove set from `module-names`,
and validates every output file and its kernel release before copying anything.
It supports `INSTALL_MOD_PATH` for staged roots and otherwise uses `sudo` to
install under `/lib/modules/$(uname -r)/updates/`, then runs `depmod`. Build
against that exact kernel and architecture. Stop tuner users before manually
reloading the drivers, or reboot after installation or removal. Uninstall
removes only these overrides, preserving stock kernel modules and local build
outputs.

## Verify

For verification with a real tuner:

```sh
dvbv5-scan -a 0 -f 1 -o ~/tvbox/dvb/channels.conf ~/tvbox/dvb/muxes
dvbv5-zap  -a 0 -f 1 -r -o /tmp/stream.ts -c ~/tvbox/dvb/channels.conf 'RTSH Kukesi'
mpv /tmp/stream.ts
```

## DVB diagnostic binary

`tools/dvb-check.c` builds a standalone Linux diagnostic command, independently
of the kernel modules or kernel build tree (Linux DVB userspace headers are
required for compilation):

```sh
make diagnostic
sudo ./dvb-check --kernel-log
sudo ./dvb-check --watch 1 --count 60 --kernel-log | tee dvb-report.txt
sudo ./dvb-check --table --watch 1
```

`--table` displays every discovered frontend together, one status row per
refresh. Columns show adapter/frontend, lock/reinitialization/timeout/read-error
state, frequency in MHz, signal with its reported units, C/N, interval post-BER,
reported uncorrected/error blocks (UCB), and the first read error for that row.
An error on one frontend does not stop the others. Each frontend has its own
BER baseline; retunes, resets, unlock and reinitialization invalidate it.

In a terminal with `TERM` set to something other than `dumb`, the table refreshes
in place. File/pipe output and basic terminals retain plain timestamped frames:

```sh
sudo ./dvb-check --table --watch 1 --count 60 --kernel-log | tee dvb-status.txt
```

Normal rows fit 80 columns for usual adapter numbering; long errors or large
counters may need more space. The table shows global layer 0; use the default
detailed mode for all layers, chip information and all read-error explanations.
`n/a` means no usable value, `baseline` needs another sample, `retune`/`reset`
invalidate an interval, and `no-bits` means its counter did not advance.
Relative AGC stays relative. A `!` after positive dBm flags suspicious driver
calibration/units. Sampling still queries frontends sequentially and does not
claim an atomic hardware snapshot or tune multiple devices.

Prepared static binaries are in `dist/dvb-check-linux-aarch64` (OrangePi ARM64)
and `dist/dvb-check-linux-x86_64`. Copy the matching binary onto the Linux host,
make it executable, and run it there. They do not require libdvbv5 or Python.
Use `dist/SHA256SUMS` to verify the binary files. To build a static binary
locally, run `make -B diagnostic LDFLAGS=-static` with static libc development
files installed.

The report includes all discovered adapters/frontends, kernel-reported names,
named capability flags, delivery systems and ranges, current tune parameters,
lock/status bits, signal/C/N (with their actual units/scales), every exposed
statistics layer, bit/block counters, and interval post-BER when comparable
counters are available. It adds the device's USB IDs/topology/speed/power state,
bound bridge driver, exposed I2C devices on the same USB device, and global
loaded DVB module metadata including versions and R820T `no_imr_cal`.

Frontend names are not TVHeadend labels. Multiple frontends may share a physical
tuner. Physical tuner model/chip IDs are sometimes not exposed by legacy attach
drivers; the tool marks missing information rather than inferring it from a
loaded module. `--kernel-log` includes the full current kernel ring at exit,
including attachment/chip messages and USB errors. It does not clear the ring;
root permissions are usually needed, and older messages may already have
been overwritten. Ctrl+C ends monitoring and still collects the requested log.

The tool opens frontends read-only and only queries information/properties and
status, closing each frontend after inventory and after each sample so shared
frontends cannot block each other throughout monitoring. A busy frontend is
reported as `BUSY` without counting a collection error or failing the run, and
retried on the next sample. Monitoring resumes automatically when it becomes
available. The kernel may wait several seconds even for a nonblocking open,
so a busy retry can extend the refresh interval. Another application's use of
a shared frontend can still prevent access
to its sibling. The [Linux DVB API](https://docs.kernel.org/userspace-api/media/dvb/frontend_f_open.html)
allows multiple read-only monitors alongside an
existing tuning application. It does not retune, scan I2C, reset devices, reload
tuner drivers, change power settings, enable debug flags, or read the transport stream.
One second is the recommended monitoring pause to limit extra polling.
Lock alone does not establish transport delivery or reception health.

BER intervals are rejected across observed tune changes, counter resets,
unlock, timeout and frontend reinitialization. Samples are not atomic: a retune
or reset completely between observations can escape detection. Missing statistics
are reported explicitly; a relative AGC value is not converted to invented dBm.
Devices are inventoried at startup, so restart the tool after hotplug.

For [usbmon](https://docs.kernel.org/usb/usbmon.html), the report checks each tuner's USB bus, its text interface under
`/sys/kernel/debug/usb/usbmon/`, and any `/dev/usbmonN` binary interface. It prints
an optional 20-second text-capture command for readable interfaces. If preparation
is needed, it prompts once before loading `usbmon` and/or mounting `debugfs`.
Only missing components are prepared, using `sudo` when not running as root.
Answering no, EOF, noninteractive input or setup failure disables USB tracing setup
for that run; tuner checks continue. Run with `sudo` for restricted interfaces.
The program undoes its own successful module load and mount on exit, including
Ctrl+C, SIGTERM, terminal hangup and a closed output pipe. Existing modules and
mounts are preserved; partial setup is rolled back. Cleanup uses noninteractive
`sudo` when needed and reports failures without forcing a busy unmount or module
removal. SIGKILL, crashes and power loss cannot guarantee cleanup. Capture still
starts manually.
The text trace includes all devices
on that bus and may grow quickly. Binary usbmon interfaces require an additional
capture tool such as tcpdump. Keep USB traces with the diagnostic report and
TVHeadend logs from the same period.

Exit status is 0 for completed collection, 1 for collection errors, and 2 for no
frontends or invalid arguments. Unsupported optional properties are marked
unavailable without failing collection. An unlocked idle frontend is not an
error. Physical tuner behavior requires a hardware check.

## Notes

- The source set was imported from Linux
  `58161ead29e50d70a317ea26b472e0c7508e69da` (7.3-rc7). Local changes are
  applied directly under `upstream/linux/`; Git retains the clean baseline.
- Headers and unchanged companion objects needed to link the eleven modules are
  vendored with the modified sources. The target kernel build tree and its
  build tools are still required.
- BTF metadata is generated by kbuild when the matching `vmlinux` and
  `pahole` are available; otherwise it may be skipped.
- The drivers remain under their upstream Linux GPL licenses.
