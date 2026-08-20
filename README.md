# cxd2841er-fix

Out-of-tree build of the upstream Sony CXD2841ER DVB demodulator driver with a
fix for the Astrometa DVB-T2 stick (RTL2832P + CXD2837ER + R828D, USB ID
`15f4:0131`).

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

```sh
dvbv5-scan -a 0 -f 1 -o ~/tvbox/dvb/channels.conf ~/tvbox/dvb/muxes
dvbv5-zap  -a 0 -f 1 -r -o /tmp/stream.ts -c ~/tvbox/dvb/channels.conf 'RTSH Kukesi'
mpv /tmp/stream.ts
```

## Notes

- The module depends on nothing; `cxd2841er.h`/`cxd2841er_priv.h` are the
  in-tree headers, vendored here so the build is self-contained.
- BTF (pahole) runs as part of the normal kbuild; no special flags needed.
- This is the stock upstream driver (GPL-2.0) plus the one `usleep_range()`
  change. An upstream fix candidate would be the same one-liner against
  `drivers/media/dvb-frontends/cxd2841er.c`.