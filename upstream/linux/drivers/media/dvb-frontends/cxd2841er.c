// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * cxd2841er.c
 *
 * Sony digital demodulator driver for
 *	CXD2841ER - DVB-S/S2/T/T2/C/C2
 *	CXD2854ER - DVB-S/S2/T/T2/C/C2, ISDB-T/S
 *
 * Copyright 2012 Sony Corporation
 * Copyright (C) 2014 NetUP Inc.
 * Copyright (C) 2014 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2014 Abylay Ospan <aospan@netup.ru>
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/log2.h>
#include <linux/dynamic_debug.h>
#include <linux/kernel.h>
#include <linux/ratelimit.h>

#include <linux/int_log.h>
#include <media/dvb_frontend.h>
#include "cxd2841er.h"
#include "cxd2841er_priv.h"

/*
 * Lock status is always read. Poll signal strength and CNR by default; the
 * error counters and detected modulation remain opt-in.
 */
static unsigned int cxd2841er_stats_mask = CXD2841ER_STATS_STRENGTH |
					  CXD2841ER_STATS_CNR;
module_param_named(stats_mask, cxd2841er_stats_mask, uint, 0644);
MODULE_PARM_DESC(stats_mask,
	"DVB statistics bitmask; status is always read: 0=status only; "
	"optional bits: 1=strength, 2=CNR, 4=block errors, 8=bit errors, "
	"16=detected DVB-C QAM; combine bits: "
	"19=status+strength+CNR+QAM (no block/bit errors), 31=all; "
	"default=3 (status+strength+CNR)");

/* DVB-C constellation */
enum sony_dvbc_constellation_t {
	SONY_DVBC_CONSTELLATION_16QAM,
	SONY_DVBC_CONSTELLATION_32QAM,
	SONY_DVBC_CONSTELLATION_64QAM,
	SONY_DVBC_CONSTELLATION_128QAM,
	SONY_DVBC_CONSTELLATION_256QAM
};

enum cxd2841er_state {
	STATE_SHUTDOWN = 0,
	STATE_SLEEP_S,
	STATE_ACTIVE_S,
	STATE_SLEEP_TC,
	STATE_ACTIVE_TC
};

struct cxd2841er_priv {
	struct dvb_frontend		frontend;
	struct i2c_adapter		*i2c;
	u8				i2c_addr_slvx;
	u8				i2c_addr_slvt;
	const struct cxd2841er_config	*config;
	enum cxd2841er_state		state;
	u8				system;
	enum cxd2841er_xtal		xtal;
	bool				cable_24mhz;
	enum fe_caps caps;
	u32				flags;
	unsigned long			stats_time;
	int				tune_error;
};

static const struct cxd2841er_cnr_data s_cn_data[] = {
	{ 0x033e, 0 }, { 0x0339, 100 }, { 0x0333, 200 },
	{ 0x032e, 300 }, { 0x0329, 400 }, { 0x0324, 500 },
	{ 0x031e, 600 }, { 0x0319, 700 }, { 0x0314, 800 },
	{ 0x030f, 900 }, { 0x030a, 1000 }, { 0x02ff, 1100 },
	{ 0x02f4, 1200 }, { 0x02e9, 1300 }, { 0x02de, 1400 },
	{ 0x02d4, 1500 }, { 0x02c9, 1600 }, { 0x02bf, 1700 },
	{ 0x02b5, 1800 }, { 0x02ab, 1900 }, { 0x02a1, 2000 },
	{ 0x029b, 2100 }, { 0x0295, 2200 }, { 0x0290, 2300 },
	{ 0x028a, 2400 }, { 0x0284, 2500 }, { 0x027f, 2600 },
	{ 0x0279, 2700 }, { 0x0274, 2800 }, { 0x026e, 2900 },
	{ 0x0269, 3000 }, { 0x0262, 3100 }, { 0x025c, 3200 },
	{ 0x0255, 3300 }, { 0x024f, 3400 }, { 0x0249, 3500 },
	{ 0x0242, 3600 }, { 0x023c, 3700 }, { 0x0236, 3800 },
	{ 0x0230, 3900 }, { 0x022a, 4000 }, { 0x0223, 4100 },
	{ 0x021c, 4200 }, { 0x0215, 4300 }, { 0x020e, 4400 },
	{ 0x0207, 4500 }, { 0x0201, 4600 }, { 0x01fa, 4700 },
	{ 0x01f4, 4800 }, { 0x01ed, 4900 }, { 0x01e7, 5000 },
	{ 0x01e0, 5100 }, { 0x01d9, 5200 }, { 0x01d2, 5300 },
	{ 0x01cb, 5400 }, { 0x01c4, 5500 }, { 0x01be, 5600 },
	{ 0x01b7, 5700 }, { 0x01b1, 5800 }, { 0x01aa, 5900 },
	{ 0x01a4, 6000 }, { 0x019d, 6100 }, { 0x0196, 6200 },
	{ 0x018f, 6300 }, { 0x0189, 6400 }, { 0x0182, 6500 },
	{ 0x017c, 6600 }, { 0x0175, 6700 }, { 0x016f, 6800 },
	{ 0x0169, 6900 }, { 0x0163, 7000 }, { 0x015c, 7100 },
	{ 0x0156, 7200 }, { 0x0150, 7300 }, { 0x014a, 7400 },
	{ 0x0144, 7500 }, { 0x013e, 7600 }, { 0x0138, 7700 },
	{ 0x0132, 7800 }, { 0x012d, 7900 }, { 0x0127, 8000 },
	{ 0x0121, 8100 }, { 0x011c, 8200 }, { 0x0116, 8300 },
	{ 0x0111, 8400 }, { 0x010b, 8500 }, { 0x0106, 8600 },
	{ 0x0101, 8700 }, { 0x00fc, 8800 }, { 0x00f7, 8900 },
	{ 0x00f2, 9000 }, { 0x00ee, 9100 }, { 0x00ea, 9200 },
	{ 0x00e6, 9300 }, { 0x00e2, 9400 }, { 0x00de, 9500 },
	{ 0x00da, 9600 }, { 0x00d7, 9700 }, { 0x00d3, 9800 },
	{ 0x00d0, 9900 }, { 0x00cc, 10000 }, { 0x00c7, 10100 },
	{ 0x00c3, 10200 }, { 0x00bf, 10300 }, { 0x00ba, 10400 },
	{ 0x00b6, 10500 }, { 0x00b2, 10600 }, { 0x00ae, 10700 },
	{ 0x00aa, 10800 }, { 0x00a7, 10900 }, { 0x00a3, 11000 },
	{ 0x009f, 11100 }, { 0x009c, 11200 }, { 0x0098, 11300 },
	{ 0x0094, 11400 }, { 0x0091, 11500 }, { 0x008e, 11600 },
	{ 0x008a, 11700 }, { 0x0087, 11800 }, { 0x0084, 11900 },
	{ 0x0081, 12000 }, { 0x007e, 12100 }, { 0x007b, 12200 },
	{ 0x0079, 12300 }, { 0x0076, 12400 }, { 0x0073, 12500 },
	{ 0x0071, 12600 }, { 0x006e, 12700 }, { 0x006c, 12800 },
	{ 0x0069, 12900 }, { 0x0067, 13000 }, { 0x0065, 13100 },
	{ 0x0062, 13200 }, { 0x0060, 13300 }, { 0x005e, 13400 },
	{ 0x005c, 13500 }, { 0x005a, 13600 }, { 0x0058, 13700 },
	{ 0x0056, 13800 }, { 0x0054, 13900 }, { 0x0052, 14000 },
	{ 0x0050, 14100 }, { 0x004e, 14200 }, { 0x004c, 14300 },
	{ 0x004b, 14400 }, { 0x0049, 14500 }, { 0x0047, 14600 },
	{ 0x0046, 14700 }, { 0x0044, 14800 }, { 0x0043, 14900 },
	{ 0x0041, 15000 }, { 0x003f, 15100 }, { 0x003e, 15200 },
	{ 0x003c, 15300 }, { 0x003b, 15400 }, { 0x003a, 15500 },
	{ 0x0037, 15700 }, { 0x0036, 15800 }, { 0x0034, 15900 },
	{ 0x0033, 16000 }, { 0x0032, 16100 }, { 0x0031, 16200 },
	{ 0x0030, 16300 }, { 0x002f, 16400 }, { 0x002e, 16500 },
	{ 0x002d, 16600 }, { 0x002c, 16700 }, { 0x002b, 16800 },
	{ 0x002a, 16900 }, { 0x0029, 17000 }, { 0x0028, 17100 },
	{ 0x0027, 17200 }, { 0x0026, 17300 }, { 0x0025, 17400 },
	{ 0x0024, 17500 }, { 0x0023, 17600 }, { 0x0022, 17800 },
	{ 0x0021, 17900 }, { 0x0020, 18000 }, { 0x001f, 18200 },
	{ 0x001e, 18300 }, { 0x001d, 18500 }, { 0x001c, 18700 },
	{ 0x001b, 18900 }, { 0x001a, 19000 }, { 0x0019, 19200 },
	{ 0x0018, 19300 }, { 0x0017, 19500 }, { 0x0016, 19700 },
	{ 0x0015, 19900 }, { 0x0014, 20000 },
};

static const struct cxd2841er_cnr_data s2_cn_data[] = {
	{ 0x05af, 0 }, { 0x0597, 100 }, { 0x057e, 200 },
	{ 0x0567, 300 }, { 0x0550, 400 }, { 0x0539, 500 },
	{ 0x0522, 600 }, { 0x050c, 700 }, { 0x04f6, 800 },
	{ 0x04e1, 900 }, { 0x04cc, 1000 }, { 0x04b6, 1100 },
	{ 0x04a1, 1200 }, { 0x048c, 1300 }, { 0x0477, 1400 },
	{ 0x0463, 1500 }, { 0x044f, 1600 }, { 0x043c, 1700 },
	{ 0x0428, 1800 }, { 0x0416, 1900 }, { 0x0403, 2000 },
	{ 0x03ef, 2100 }, { 0x03dc, 2200 }, { 0x03c9, 2300 },
	{ 0x03b6, 2400 }, { 0x03a4, 2500 }, { 0x0392, 2600 },
	{ 0x0381, 2700 }, { 0x036f, 2800 }, { 0x035f, 2900 },
	{ 0x034e, 3000 }, { 0x033d, 3100 }, { 0x032d, 3200 },
	{ 0x031d, 3300 }, { 0x030d, 3400 }, { 0x02fd, 3500 },
	{ 0x02ee, 3600 }, { 0x02df, 3700 }, { 0x02d0, 3800 },
	{ 0x02c2, 3900 }, { 0x02b4, 4000 }, { 0x02a6, 4100 },
	{ 0x0299, 4200 }, { 0x028c, 4300 }, { 0x027f, 4400 },
	{ 0x0272, 4500 }, { 0x0265, 4600 }, { 0x0259, 4700 },
	{ 0x024d, 4800 }, { 0x0241, 4900 }, { 0x0236, 5000 },
	{ 0x022b, 5100 }, { 0x0220, 5200 }, { 0x0215, 5300 },
	{ 0x020a, 5400 }, { 0x0200, 5500 }, { 0x01f6, 5600 },
	{ 0x01ec, 5700 }, { 0x01e2, 5800 }, { 0x01d8, 5900 },
	{ 0x01cf, 6000 }, { 0x01c6, 6100 }, { 0x01bc, 6200 },
	{ 0x01b3, 6300 }, { 0x01aa, 6400 }, { 0x01a2, 6500 },
	{ 0x0199, 6600 }, { 0x0191, 6700 }, { 0x0189, 6800 },
	{ 0x0181, 6900 }, { 0x0179, 7000 }, { 0x0171, 7100 },
	{ 0x0169, 7200 }, { 0x0161, 7300 }, { 0x015a, 7400 },
	{ 0x0153, 7500 }, { 0x014b, 7600 }, { 0x0144, 7700 },
	{ 0x013d, 7800 }, { 0x0137, 7900 }, { 0x0130, 8000 },
	{ 0x012a, 8100 }, { 0x0124, 8200 }, { 0x011e, 8300 },
	{ 0x0118, 8400 }, { 0x0112, 8500 }, { 0x010c, 8600 },
	{ 0x0107, 8700 }, { 0x0101, 8800 }, { 0x00fc, 8900 },
	{ 0x00f7, 9000 }, { 0x00f2, 9100 }, { 0x00ec, 9200 },
	{ 0x00e7, 9300 }, { 0x00e2, 9400 }, { 0x00dd, 9500 },
	{ 0x00d8, 9600 }, { 0x00d4, 9700 }, { 0x00cf, 9800 },
	{ 0x00ca, 9900 }, { 0x00c6, 10000 }, { 0x00c2, 10100 },
	{ 0x00be, 10200 }, { 0x00b9, 10300 }, { 0x00b5, 10400 },
	{ 0x00b1, 10500 }, { 0x00ae, 10600 }, { 0x00aa, 10700 },
	{ 0x00a6, 10800 }, { 0x00a3, 10900 }, { 0x009f, 11000 },
	{ 0x009b, 11100 }, { 0x0098, 11200 }, { 0x0095, 11300 },
	{ 0x0091, 11400 }, { 0x008e, 11500 }, { 0x008b, 11600 },
	{ 0x0088, 11700 }, { 0x0085, 11800 }, { 0x0082, 11900 },
	{ 0x007f, 12000 }, { 0x007c, 12100 }, { 0x007a, 12200 },
	{ 0x0077, 12300 }, { 0x0074, 12400 }, { 0x0072, 12500 },
	{ 0x006f, 12600 }, { 0x006d, 12700 }, { 0x006b, 12800 },
	{ 0x0068, 12900 }, { 0x0066, 13000 }, { 0x0064, 13100 },
	{ 0x0061, 13200 }, { 0x005f, 13300 }, { 0x005d, 13400 },
	{ 0x005b, 13500 }, { 0x0059, 13600 }, { 0x0057, 13700 },
	{ 0x0055, 13800 }, { 0x0053, 13900 }, { 0x0051, 14000 },
	{ 0x004f, 14100 }, { 0x004e, 14200 }, { 0x004c, 14300 },
	{ 0x004a, 14400 }, { 0x0049, 14500 }, { 0x0047, 14600 },
	{ 0x0045, 14700 }, { 0x0044, 14800 }, { 0x0042, 14900 },
	{ 0x0041, 15000 }, { 0x003f, 15100 }, { 0x003e, 15200 },
	{ 0x003c, 15300 }, { 0x003b, 15400 }, { 0x003a, 15500 },
	{ 0x0038, 15600 }, { 0x0037, 15700 }, { 0x0036, 15800 },
	{ 0x0034, 15900 }, { 0x0033, 16000 }, { 0x0032, 16100 },
	{ 0x0031, 16200 }, { 0x0030, 16300 }, { 0x002f, 16400 },
	{ 0x002e, 16500 }, { 0x002d, 16600 }, { 0x002c, 16700 },
	{ 0x002b, 16800 }, { 0x002a, 16900 }, { 0x0029, 17000 },
	{ 0x0028, 17100 }, { 0x0027, 17200 }, { 0x0026, 17300 },
	{ 0x0025, 17400 }, { 0x0024, 17500 }, { 0x0023, 17600 },
	{ 0x0022, 17800 }, { 0x0021, 17900 }, { 0x0020, 18000 },
	{ 0x001f, 18200 }, { 0x001e, 18300 }, { 0x001d, 18500 },
	{ 0x001c, 18700 }, { 0x001b, 18900 }, { 0x001a, 19000 },
	{ 0x0019, 19200 }, { 0x0018, 19300 }, { 0x0017, 19500 },
	{ 0x0016, 19700 }, { 0x0015, 19900 }, { 0x0014, 20000 },
};

static int cxd2841er_freeze_regs(struct cxd2841er_priv *priv);
static int cxd2841er_unfreeze_regs(struct cxd2841er_priv *priv);
static void cxd2841er_init_stats(struct dvb_frontend *fe);
static int cxd2841er_init_s(struct dvb_frontend *fe);
static int cxd2841er_init_tc(struct dvb_frontend *fe);

static void cxd2841er_i2c_debug(struct cxd2841er_priv *priv,
				u8 addr, u8 reg, u8 write,
				const u8 *data, u32 len)
{
	dev_dbg(&priv->i2c->dev,
		"cxd2841er: I2C %s addr %02x reg 0x%02x size %d data %*ph\n",
		(write == 0 ? "read" : "write"), addr, reg, len, len, data);
}

static int cxd2841er_write_regs(struct cxd2841er_priv *priv,
				u8 addr, u8 reg, const u8 *data, u32 len)
{
	int ret;
	u8 buf[MAX_WRITE_REGSIZE + 1];
	u8 i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	struct i2c_msg msg[1] = {
		{
			.addr = i2c_addr,
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		}
	};

	if (len > MAX_WRITE_REGSIZE) {
		dev_warn(&priv->i2c->dev, "wr reg=%04x: len=%u is too big!\n",
			 reg, len);
		return -E2BIG;
	}

	cxd2841er_i2c_debug(priv, i2c_addr, reg, 1, data, len);
	buf[0] = reg;
	memcpy(&buf[1], data, len);

	ret = i2c_transfer(priv->i2c, msg, 1);
	if (ret >= 0 && ret != 1)
		ret = -EIO;
	if (ret < 0) {
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s: i2c wr failed=%d addr=%02x reg=%02x len=%d\n",
			KBUILD_MODNAME, ret, i2c_addr, reg, len);
		return ret;
	}
	return 0;
}

static int cxd2841er_write_reg(struct cxd2841er_priv *priv,
			       u8 addr, u8 reg, u8 val)
{
	u8 tmp = val; /* see gcc.gnu.org/bugzilla/show_bug.cgi?id=81715 */

	return cxd2841er_write_regs(priv, addr, reg, &tmp, 1);
}

static int cxd2841er_read_regs(struct cxd2841er_priv *priv,
			       u8 addr, u8 reg, u8 *val, u32 len)
{
	int ret;
	u8 i2c_addr = (addr == I2C_SLVX ?
		priv->i2c_addr_slvx : priv->i2c_addr_slvt);
	struct i2c_msg msg[2] = {
		{
			.addr = i2c_addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = i2c_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = val,
		}
	};

	ret = i2c_transfer(priv->i2c, msg, 2);
	if (ret >= 0 && ret != 2)
		ret = -EIO;
	if (ret < 0) {
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s: i2c rd failed=%d addr=%02x reg=%02x\n",
			KBUILD_MODNAME, ret, i2c_addr, reg);
		return ret;
	}
	cxd2841er_i2c_debug(priv, i2c_addr, reg, 0, val, len);
	return 0;
}

static int cxd2841er_read_reg(struct cxd2841er_priv *priv,
			      u8 addr, u8 reg, u8 *val)
{
	return cxd2841er_read_regs(priv, addr, reg, val, 1);
}

static int cxd2841er_set_reg_bits(struct cxd2841er_priv *priv,
				  u8 addr, u8 reg, u8 data, u8 mask)
{
	int res;
	u8 rdata;

	if (mask != CXD2841ER_BYTE_MASK) {
		res = cxd2841er_read_reg(priv, addr, reg, &rdata);
		if (res)
			return res;
		data = ((data & mask) | (rdata & (mask ^ CXD2841ER_BYTE_MASK)));
	}
	return cxd2841er_write_reg(priv, addr, reg, data);
}

static u32 cxd2841er_calc_iffreq_xtal(enum cxd2841er_xtal xtal, u32 ifhz)
{
	return div_u64(ifhz * 16777216ull,
		       (xtal == SONY_XTAL_24000) ? 48000000 : 41000000);
}

static u32 cxd2841er_calc_iffreq(u32 ifhz)
{
	return cxd2841er_calc_iffreq_xtal(SONY_XTAL_20500, ifhz);
}

static void cxd2841er_warn_tune(struct cxd2841er_priv *priv,
			      const char *stage, int error)
{
	struct dtv_frontend_properties *p = &priv->frontend.dtv_property_cache;

	if (error)
		dev_warn_ratelimited(&priv->i2c->dev,
			"cxd2841er addr=0x%02x %s: delsys=%u frequency=%u %s bandwidth_hz=%u symbol_rate=%u state=%d error=%d\n",
			priv->i2c_addr_slvt, stage, p->delivery_system, p->frequency,
			p->delivery_system == SYS_DVBS || p->delivery_system == SYS_DVBS2 ? "kHz" : "Hz",
			p->bandwidth_hz, p->symbol_rate, priv->state, error);
}

static int cxd2841er_get_if_hz(struct cxd2841er_priv *priv, u32 def_hz,
				    u32 *hz)
{
	int ret = 0;

	*hz = def_hz;
	if (priv->frontend.ops.tuner_ops.get_if_frequency
			&& (priv->flags & CXD2841ER_AUTO_IFHZ))
		ret = priv->frontend.ops.tuner_ops.get_if_frequency(
			&priv->frontend, hz);
	cxd2841er_warn_tune(priv, "IF frequency", ret);
	return ret;
}

static int cxd2841er_tuner_set(struct dvb_frontend *fe)
{
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	bool gate = (priv->flags & CXD2841ER_USE_GATECTRL) &&
		fe->ops.i2c_gate_ctrl;
	int ret = 0, close_ret;

	if (gate) {
		ret = fe->ops.i2c_gate_ctrl(fe, 1);
		cxd2841er_warn_tune(priv, "gate open", ret);
		if (ret)
			return ret;
	}
	if (fe->ops.tuner_ops.set_params)
		ret = fe->ops.tuner_ops.set_params(fe);
	cxd2841er_warn_tune(priv, "tuner programming", ret);
	if (gate) {
		close_ret = fe->ops.i2c_gate_ctrl(fe, 0);
		cxd2841er_warn_tune(priv, "gate close", close_ret);
		if (!ret)
			ret = close_ret;
	}
	return ret;
}

static int cxd2841er_dvbs2_set_symbol_rate(struct cxd2841er_priv *priv,
					   u32 symbol_rate)
{
	int ret;

	u32 reg_value = 0;
	u8 data[3] = {0, 0, 0};

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	/*
	 * regValue = (symbolRateKSps * 2^14 / 1000) + 0.5
	 *          = ((symbolRateKSps * 2^14) + 500) / 1000
	 *          = ((symbolRateKSps * 16384) + 500) / 1000
	 */
	reg_value = DIV_ROUND_CLOSEST(symbol_rate * 16384, 1000);
	if ((reg_value == 0) || (reg_value > CXD2841ER_SYMBOL_RATE_MAX)) {
		dev_err(&priv->i2c->dev,
			"%s(): reg_value is out of range\n", __func__);
		return -EINVAL;
	}
	data[0] = (u8)((reg_value >> CXD2841ER_SYMBOL_RATE_HIGH_SHIFT) &
		CXD2841ER_SYMBOL_RATE_HIGH_MASK);
	data[1] = (u8)((reg_value >> CXD2841ER_SYMBOL_RATE_MIDDLE_SHIFT) &
		CXD2841ER_SYMBOL_RATE_BYTE_MASK);
	data[2] = (u8)(reg_value & CXD2841ER_SYMBOL_RATE_BYTE_MASK);
	/* Set SLV-T Bank : 0xAE */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_SYMBOL_RATE);
	if (ret)
		return ret;
	ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_SYMBOL_RATE, data, 3);
	if (ret)
		return ret;
	return 0;
}

static int cxd2841er_set_ts_clock_mode(struct cxd2841er_priv *priv,
					u8 system);

static int cxd2841er_sleep_s_to_active_s(struct cxd2841er_priv *priv,
					 u8 system, u32 symbol_rate)
{
	int ret;
	u8 data[4] = { 0, 0, 0, 0 };

	if (priv->state != STATE_SLEEP_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, (int)priv->state);
		return -EINVAL;
	}
	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_set_ts_clock_mode(priv, SYS_DVBS);
	if (ret)
		return ret;
	/* Set demod mode */
	if (system == SYS_DVBS) {
		data[0] = CXD2841ER_SET_DEMOD_MODE_DVBS;
	} else if (system == SYS_DVBS2) {
		data[0] = CXD2841ER_SET_DEMOD_MODE_DVBS2;
	} else {
		dev_err(&priv->i2c->dev, "%s(): invalid delsys %d\n",
			__func__, system);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVX, CXD2841ER_REG_DEMOD_MODE, data[0]);
	if (ret)
		return ret;
	/* DVB-S/S2 */
	data[0] = CXD2841ER_SET_SAT_AUTO_DETECT_DISABLE;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable S/S2 auto detection 1 for the requested fixed system. */
	ret = cxd2841er_write_reg(priv, I2C_SLVT, CXD2841ER_REG_SAT_AUTO_DETECT_00, data[0]);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0xAE */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_SYMBOL_RATE);
	if (ret)
		return ret;
	/* Disable S/S2 auto detection 2 for the requested fixed system. */
	ret = cxd2841er_write_reg(priv, I2C_SLVT, CXD2841ER_REG_SAT_AUTO_DETECT_AE, data[0]);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_ENABLE);
	if (ret)
		return ret;
	/* Enable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_SAT_ADC_CLOCK_CONTROL_PRESET_01);
	if (ret)
		return ret;
	/* Enable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_63,
		CXD2841ER_SET_SAT_ADC_CONTROL_63_PRESET_16);
	if (ret)
		return ret;
	/* Enable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_65,
		CXD2841ER_SET_SAT_ADC_CONTROL_65_ACTIVE);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_ON);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0xA3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_A3);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_SETUP_A3_AC_UNKNOWN,
		CXD2841ER_SET_SAT_SETUP_A3_AC_UNKNOWN_PRESET_00);
	if (ret)
		return ret;
	data[0] = 0x07;
	data[1] = 0x3B;
	data[2] = 0x08;
	data[3] = 0xC5;
	/* Set SLV-T Bank : 0xAB */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_AB);
	if (ret)
		return ret;
	ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_SETUP_AB_98_UNKNOWN, data, 4);
	if (ret)
		return ret;
	data[0] = 0x05;
	data[1] = 0x80;
	data[2] = 0x0A;
	data[3] = 0x80;
	ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_SETUP_AB_A8_UNKNOWN, data, 4);
	if (ret)
		return ret;
	data[0] = 0x0C;
	data[1] = 0xCC;
	ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_SETUP_AB_C3_UNKNOWN, data, 2);
	if (ret)
		return ret;
	/* Set demod parameter */
	ret = cxd2841er_dvbs2_set_symbol_rate(priv, symbol_rate);
	if (ret != 0)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_SAT_ACTIVE);
	if (ret)
		return ret;
	/* disable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_DRIVE);
	if (ret)
		return ret;
	priv->system = system;
	priv->state = STATE_ACTIVE_S;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t_band(struct cxd2841er_priv *priv,
					       u32 bandwidth);

static int cxd2841er_sleep_tc_to_active_t2_band(struct cxd2841er_priv *priv,
						u32 bandwidth);

static int cxd2841er_sleep_tc_to_active_c_band(struct cxd2841er_priv *priv,
					       u32 bandwidth);

static int cxd2841er_sleep_tc_to_active_i(struct cxd2841er_priv *priv,
		u32 bandwidth);

static int cxd2841er_active_i_to_sleep_tc(struct cxd2841er_priv *priv);

static int cxd2841er_sleep_tc_to_shutdown(struct cxd2841er_priv *priv);

static int cxd2841er_shutdown_to_sleep_tc(struct cxd2841er_priv *priv);

static int cxd2841er_sleep_tc(struct dvb_frontend *fe);

static int cxd2841er_retune_active(struct cxd2841er_priv *priv,
				   struct dtv_frontend_properties *p)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_S &&
			priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	if (priv->state == STATE_ACTIVE_S)
		return cxd2841er_dvbs2_set_symbol_rate(
				priv, p->symbol_rate / 1000);
	else if (priv->state == STATE_ACTIVE_TC) {
		switch (priv->system) {
		case SYS_DVBT:
			return cxd2841er_sleep_tc_to_active_t_band(
					priv, p->bandwidth_hz);
		case SYS_DVBT2:
			return cxd2841er_sleep_tc_to_active_t2_band(
					priv, p->bandwidth_hz);
		case SYS_DVBC_ANNEX_A:
			return cxd2841er_sleep_tc_to_active_c_band(
					priv, p->bandwidth_hz);
		case SYS_ISDBT:
			ret = cxd2841er_active_i_to_sleep_tc(priv);
			if (ret)
				return ret;
			ret = cxd2841er_sleep_tc_to_shutdown(priv);
			if (ret)
				return ret;
			ret = cxd2841er_shutdown_to_sleep_tc(priv);
			if (ret)
				return ret;
			return cxd2841er_sleep_tc_to_active_i(
					priv, p->bandwidth_hz);
		}
	}
	dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
		__func__, priv->system);
	return -EINVAL;
}

static int cxd2841er_active_s_to_sleep_s(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	/* enable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_SAT_SLEEP);
	if (ret)
		return ret;
	/* enable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_HIGH_Z);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_OFF);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_SAT_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* disable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_63,
		CXD2841ER_SET_SAT_ADC_CONTROL_63_PRESET_16);
	if (ret)
		return ret;
	/* disable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_65,
		CXD2841ER_SET_SAT_ADC_CONTROL_65_BIAS_ON);
	if (ret)
		return ret;
	/* SADC Bias ON */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_BIAS,
		CXD2841ER_SET_SAT_ADC_BIAS_ON);
	if (ret)
		return ret;
	/* disable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_DISABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0xAE */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_SYMBOL_RATE);
	if (ret)
		return ret;
	/* disable S/S2 auto detection1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_AUTO_DETECT_AE,
		CXD2841ER_SET_SAT_AUTO_DETECT_AE_DISABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable S/S2 auto detection2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_AUTO_DETECT_00,
		CXD2841ER_SET_SAT_AUTO_DETECT_00_DISABLE);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_S;
	return 0;
}

static int cxd2841er_sleep_s_to_shutdown(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SLEEP_S) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable DSQOUT */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_SLEEP);
	if (ret)
		return ret;
	/* Disable DSQIN */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_INPUT_CONTROL,
		CXD2841ER_SET_DISEQC_INPUT_CONTROL_DISABLE);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable oscillator */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_OSCILLATOR_DISABLE,
		CXD2841ER_SET_OSCILLATOR_OFF);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBT);
	if (ret)
		return ret;
	priv->state = STATE_SHUTDOWN;
	return 0;
}

static int cxd2841er_sleep_tc_to_shutdown(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SLEEP_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable oscillator */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_OSCILLATOR_DISABLE,
		CXD2841ER_SET_OSCILLATOR_OFF);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBT);
	if (ret)
		return ret;
	priv->state = STATE_SHUTDOWN;
	return 0;
}

static int cxd2841er_active_t_to_sleep_tc(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	/* enable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_SLEEP);
	if (ret)
		return ret;
	/* enable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_HIGH_Z);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_OFF);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Disable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_DISABLE);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_active_t2_to_sleep_tc(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	/* enable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_SLEEP);
	if (ret)
		return ret;
	/* enable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_HIGH_Z);
	if (ret)
		return ret;
	/* Cancel DVB-T2 setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_13);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_ISIC_POSPROTECT,
		CXD2841ER_SET_DVBT2_ISIC_POSPROTECT_CANCEL);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_ISIC_CFNRMOFFSET,
		CXD2841ER_SET_DVBT2_ISIC_CFNRMOFFSET_CANCEL);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_CFUPONTHRESHOLDMAX_HIGH,
		CXD2841ER_SET_DVBT2_CFUPONTHRESHOLDMAX_HIGH_PRESET_09,
		CXD2841ER_MASK_DVBT2_CFUPONTHRESHOLDMAX_HIGH);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_CFUPONTHRESHOLDMAX_LOW,
		CXD2841ER_SET_DVBT2_CFUPONTHRESHOLDMAX_LOW_CANCEL);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2A);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SETUP_2A_38_UNKNOWN,
		CXD2841ER_SET_DVBT2_SETUP_2A_38_UNKNOWN_CANCEL,
		CXD2841ER_MASK_DVBT2_SETUP_2A_38_UNKNOWN);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2B);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SETUP_2B_11_UNKNOWN,
		CXD2841ER_SET_DVBT2_SETUP_2B_11_UNKNOWN_CANCEL,
		CXD2841ER_MASK_DVBT2_SETUP_2B_11_UNKNOWN);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_OFF);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Disable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_DISABLE);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_active_c_to_sleep_tc(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	/* enable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_SLEEP);
	if (ret)
		return ret;
	/* enable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_HIGH_Z);
	if (ret)
		return ret;
	/* Cancel DVB-C setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_11);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBC_IMR_CHANNEL_WIDTH,
		CXD2841ER_SET_DVBC_IMR_CHANNEL_WIDTH_CANCEL,
		CXD2841ER_MASK_DVBC_IMR_CHANNEL_WIDTH);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_OFF);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Disable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_DISABLE);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_active_i_to_sleep_tc(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable TS output */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_OFF);
	if (ret)
		return ret;
	/* enable Hi-Z setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_SLEEP);
	if (ret)
		return ret;
	/* enable Hi-Z setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_HIGH_Z);
	if (ret)
		return ret;

	/* TODO: Cancel demod parameter */

	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* disable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_OFF);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable ADC 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC 3 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* Disable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Disable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_DISABLE);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_shutdown_to_sleep_s(struct cxd2841er_priv *priv)
{
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SHUTDOWN) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Clear all demodulator registers */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_CLEAR_DEMOD_REGISTERS,
		CXD2841ER_SET_CLEAR_DEMOD_REGISTERS_TRIGGER);
	if (ret)
		return ret;
	usleep_range(3000, 5000);
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod SW reset */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_RESET_HOLD,
		CXD2841ER_SET_DEMOD_RESET_HOLD_ASSERT);
	if (ret)
		return ret;

	switch (priv->xtal) {
	case SONY_XTAL_20500:
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_CRYSTAL_FREQUENCY_SELECT,
			CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_20_5_MHZ);
		if (ret)
			return ret;
		break;
	case SONY_XTAL_24000:
		/* Select demod frequency */
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_DEMOD_FREQUENCY_SELECT,
			CXD2841ER_SET_DEMOD_FREQUENCY_SELECT_PRESET_00);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_CRYSTAL_FREQUENCY_SELECT,
			CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_24_MHZ);
		if (ret)
			return ret;
		break;
	case SONY_XTAL_41000:
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_CRYSTAL_FREQUENCY_SELECT,
			CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_41_MHZ);
		if (ret)
			return ret;
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod xtal %d\n",
				__func__, priv->xtal);
		return -EINVAL;
	}

	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBS);
	if (ret)
		return ret;
	/* Clear demod SW reset */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_RESET_HOLD,
		CXD2841ER_SET_DEMOD_RESET_HOLD_RELEASE);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* enable DSQOUT */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_SAT_SLEEP);
	if (ret)
		return ret;
	/* enable DSQIN */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_INPUT_CONTROL,
		CXD2841ER_SET_DISEQC_INPUT_CONTROL_ENABLE);
	if (ret)
		return ret;
	/* TADC Bias On */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* SADC Bias On */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_63,
		CXD2841ER_SET_SAT_ADC_CONTROL_63_PRESET_16);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_65,
		CXD2841ER_SET_SAT_ADC_CONTROL_65_BIAS_ON);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_BIAS,
		CXD2841ER_SET_SAT_ADC_BIAS_ON);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_S;
	return 0;
}

static int cxd2841er_shutdown_to_sleep_tc(struct cxd2841er_priv *priv)
{
	int ret;

	u8 data = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_SHUTDOWN) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Clear all demodulator registers */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_CLEAR_DEMOD_REGISTERS,
		CXD2841ER_SET_CLEAR_DEMOD_REGISTERS_TRIGGER);
	if (ret)
		return ret;
	usleep_range(3000, 5000);
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod SW reset */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_RESET_HOLD,
		CXD2841ER_SET_DEMOD_RESET_HOLD_ASSERT);
	if (ret)
		return ret;
  /* Select ADC clock mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_CLOCK_MODE,
		CXD2841ER_SET_ADC_CLOCK_MODE_PRESET_00);
	if (ret)
		return ret;

	switch (priv->xtal) {
	case SONY_XTAL_20500:
		data = CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_20_5_MHZ;
		break;
	case SONY_XTAL_24000:
		/* Select demod frequency */
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_DEMOD_FREQUENCY_SELECT,
			CXD2841ER_SET_DEMOD_FREQUENCY_SELECT_PRESET_00);
		if (ret)
			return ret;
		data = CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_24_MHZ;
		break;
	case SONY_XTAL_41000:
		ret = cxd2841er_write_reg(priv, I2C_SLVX,
			CXD2841ER_REG_DEMOD_FREQUENCY_SELECT,
			CXD2841ER_SET_DEMOD_FREQUENCY_SELECT_PRESET_00);
		if (ret)
			return ret;
		data = CXD2841ER_SET_CRYSTAL_FREQUENCY_SELECT_41_MHZ;
		break;
	}
	ret = cxd2841er_write_reg(priv, I2C_SLVX, CXD2841ER_REG_CRYSTAL_FREQUENCY_SELECT, data);
	if (ret)
		return ret;
	/* Clear demod SW reset */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_RESET_HOLD,
		CXD2841ER_SET_DEMOD_RESET_HOLD_RELEASE);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* TADC Bias On */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_43_BIAS_ON);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_BIAS_ON);
	if (ret)
		return ret;
	/* SADC Bias On */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_63,
		CXD2841ER_SET_SAT_ADC_CONTROL_63_PRESET_16);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CONTROL_65,
		CXD2841ER_SET_SAT_ADC_CONTROL_65_BIAS_ON);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_BIAS,
		CXD2841ER_SET_SAT_ADC_BIAS_ON);
	if (ret)
		return ret;
	priv->state = STATE_SLEEP_TC;
	return 0;
}

static int cxd2841er_tune_done(struct cxd2841er_priv *priv)
{
	const char *stage = "bank select";
	u8 ts_enable = CXD2841ER_BYTE_MASK;
	int ret;

	/* Stop here if the bank or reset write fails; later writes would be unsafe. */
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		goto done;
	/* SW Reset */
	stage = "software reset";
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_SOFTWARE_RESET,
		CXD2841ER_SET_SOFTWARE_RESET_TRIGGER);
	if (ret)
		goto done;
	/*
	 * CXD2837ER NAKs all I2C transactions for ~10ms while the SW reset
	 * is in progress. A write issued during that window is silently
	 * dropped (the RTL2832 I2C master swallows the NAK), leaving the
	 * TS output disabled even though the enable write below "succeeded".
	 * Wait for the reset to complete before re-enabling TS output.
	 */
	usleep_range(15000, 20000);
	/* Enable TS output */
	stage = "TS enable";
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_OUTPUT_DISABLE,
		CXD2841ER_SET_TS_OUTPUT_ON);
	if (ret)
		goto done;
	/* The USB bridge can acknowledge a write that the demodulator dropped. */
	stage = "TS verify";
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_TS_OUTPUT_DISABLE, &ts_enable);
	if (!ret && (ts_enable & CXD2841ER_TS_OUTPUT_DISABLED_BIT))
		ret = -EIO;
done:
	if (ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"cxd2841er addr=0x%02x %s: frequency=%u %s state=%d system=%u ts_enable=0x%02x error=%d\n",
			priv->i2c_addr_slvt, stage,
			priv->frontend.dtv_property_cache.frequency,
			priv->frontend.dtv_property_cache.delivery_system == SYS_DVBS ||
			priv->frontend.dtv_property_cache.delivery_system == SYS_DVBS2 ? "kHz" : "Hz",
			priv->state, priv->system, ts_enable, ret);
	dev_dbg(&priv->i2c->dev,
		"%s(): stage=%s state=%d system=%u ts_enable=0x%02x ret=%d\n",
		__func__, stage, priv->state, priv->system, ts_enable, ret);
	return ret;
}

/* Set TS parallel mode */
static int cxd2841er_set_ts_clock_mode(struct cxd2841er_priv *priv,
					u8 system)
{
	int ret;

	u8 serial_ts, ts_rate_ctrl_off, ts_in_off;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	ret = cxd2841er_read_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_FORMAT_AND_SERIAL_CLOCK, &serial_ts);
	if (ret)
		return ret;
	ret = cxd2841er_read_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_RATE_CONTROL_OFF, &ts_rate_ctrl_off);
	if (ret)
		return ret;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_TS_INPUT_OFF, &ts_in_off);
	if (ret)
		return ret;
	dev_dbg(&priv->i2c->dev, "%s(): ser_ts=0x%02x rate_ctrl_off=0x%02x in_off=0x%02x\n",
		__func__, serial_ts, ts_rate_ctrl_off, ts_in_off);

	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     C4h     [1:0]  2'b??      OSERCKMODE
	 */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_FORMAT_AND_SERIAL_CLOCK,
		((priv->flags & CXD2841ER_TS_SERIAL)
			? CXD2841ER_SET_TS_FORMAT_AND_SERIAL_CLOCK_SERIAL_CLOCK
			: CXD2841ER_SET_TS_FORMAT_AND_SERIAL_CLOCK_PARALLEL_CLOCK),
		CXD2841ER_MASK_TS_SERIAL_CLOCK_MODE);
	if (ret)
		return ret;
	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     D1h     [1:0]  2'b??      OSERDUTYMODE
	 */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_SERIAL_CLOCK_DUTY,
		((priv->flags & CXD2841ER_TS_SERIAL)
			? CXD2841ER_SET_TS_SERIAL_CLOCK_DUTY_SERIAL
			: CXD2841ER_SET_TS_SERIAL_CLOCK_DUTY_PARALLEL),
		CXD2841ER_MASK_TS_SERIAL_CLOCK_DUTY);
	if (ret)
		return ret;
	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     D9h     [7:0]  8'h08      OTSCKPERIOD
	 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TS_CLOCK_PERIOD,
		CXD2841ER_SET_TS_CLOCK_PERIOD_DEFAULT);
	if (ret)
		return ret;
	/*
	 * Disable TS IF Clock
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSIF_EN
	 */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_INTERFACE_CLOCK_ENABLE,
		CXD2841ER_SET_TS_INTERFACE_CLOCK_DISABLE,
		CXD2841ER_MASK_TS_INTERFACE_CLOCK_ENABLE);
	if (ret)
		return ret;
	/*
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     33h     [1:0]  2'b01      OREG_CKSEL_TSIF
	 */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_INTERFACE_CLOCK_SELECT,
		((priv->flags & CXD2841ER_TS_SERIAL)
			? CXD2841ER_SET_TS_INTERFACE_CLOCK_SELECT_SERIAL
			: CXD2841ER_SET_TS_INTERFACE_CLOCK_SELECT_PARALLEL),
		CXD2841ER_MASK_TS_INTERFACE_CLOCK_SELECT);
	if (ret)
		return ret;
	/*
	 * Enable TS IF Clock
	 * slave    Bank    Addr    Bit    default    Name
	 * <SLV-T>  00h     32h     [0]    1'b1       OREG_CK_TSIF_EN
	 */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_INTERFACE_CLOCK_ENABLE,
		CXD2841ER_SET_TS_INTERFACE_CLOCK_ENABLE,
		CXD2841ER_MASK_TS_INTERFACE_CLOCK_ENABLE);
	if (ret)
		return ret;

	if (system == SYS_DVBT) {
		/* Enable parity period for DVB-T */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_TS_PARITY_PERIOD,
			CXD2841ER_SET_DVBT_TS_PARITY_PERIOD_ENABLE,
			CXD2841ER_MASK_DVBT_TS_PARITY_PERIOD);
		if (ret)
			return ret;
	} else if (system == SYS_DVBC_ANNEX_A) {
		/* Enable parity period for DVB-C */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBC);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TS_PARITY_PERIOD,
			CXD2841ER_SET_DVBC_TS_PARITY_PERIOD_ENABLE,
			CXD2841ER_MASK_DVBC_TS_PARITY_PERIOD);
		if (ret)
			return ret;
	}
	return 0;
}

static u8 cxd2841er_chip_id(struct cxd2841er_priv *priv)
{
	u8 chip_id = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON) == 0) {
		if (cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_CHIP_ID, &chip_id))
			return 0;
	} else if (cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON) == 0) {
		if (cxd2841er_read_reg(priv, I2C_SLVX, CXD2841ER_REG_CHIP_ID, &chip_id))
			return 0;
	}

	return chip_id;
}

/* A bus failure is different from a successful read that reports no signal. */
static int cxd2841er_read_status_s(struct dvb_frontend *fe,
				   enum fe_status *status)
{
	int ret;
	u8 reg = 0;
	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	*status = 0;
	if (priv->tune_error)
		return priv->tune_error;
	if (priv->state != STATE_ACTIVE_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0xA0 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_MONITOR);
	if (ret)
		return ret;
	/*
	 *  slave     Bank      Addr      Bit      Signal name
	 * <SLV-T>    A0h       11h       [2]      ITSLOCK
	 */
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_SAT_TS_LOCK, &reg);
	if (ret)
		return ret;
	if (reg & CXD2841ER_SAT_TS_LOCK_BIT) {
		*status = FE_HAS_SIGNAL
			| FE_HAS_CARRIER
			| FE_HAS_VITERBI
			| FE_HAS_SYNC
			| FE_HAS_LOCK;
	}
	dev_dbg(&priv->i2c->dev, "%s(): result 0x%x\n", __func__, *status);
	return 0;
}

static int cxd2841er_read_status_t_t2(struct cxd2841er_priv *priv,
				      u8 *sync, u8 *tslock, u8 *unlock)
{
	int ret;
	u8 data = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC)
		return -EINVAL;
	if (priv->system == SYS_DVBT) {
		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;
	} else {
		/* Set SLV-T Bank : 0x20 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT2);
		if (ret)
			return ret;
	}
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT_T2_SYNC_STATUS, &data);
	if (ret)
		return ret;
	if ((data & CXD2841ER_DVBT_T2_SYNC_STATE_MASK) == CXD2841ER_DVBT_T2_SYNC_INVALID) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid hardware state detected\n", __func__);
		*sync = 0;
		*tslock = 0;
		*unlock = 0;
	} else {
		*sync = ((data & CXD2841ER_DVBT_T2_SYNC_STATE_MASK)
			== CXD2841ER_DVBT_T2_SYNC_LOCKED ? 1 : 0);
		*tslock = ((data & CXD2841ER_DVBT_T2_TS_LOCK_BIT) ? 1 : 0);
		*unlock = ((data & CXD2841ER_DVBT_T2_UNLOCK_BIT) ? 1 : 0);
	}
	return 0;
}

static int cxd2841er_read_status_c(struct cxd2841er_priv *priv, u8 *tslock)
{
	int ret;
	u8 data;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC)
		return -EINVAL;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_ACQUISITION_LOCK, &data);
	if (ret)
		return ret;
	if ((data & CXD2841ER_DVBC_ACQUISITION_LOCK_BIT) == 0) {
		*tslock = 0;
	} else {
		ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_TS_LOCK, &data);
		if (ret)
			return ret;
		*tslock = ((data & CXD2841ER_DVBC_TS_LOCK_BIT) ? 1 : 0);
	}
	return 0;
}

static int cxd2841er_read_status_i(struct cxd2841er_priv *priv,
		u8 *sync, u8 *tslock, u8 *unlock)
{
	int ret;
	u8 data = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC)
		return -EINVAL;
	/* Set SLV-T Bank : 0x60 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		return ret;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_ISDBT_SYNC_STATUS, &data);
	if (ret)
		return ret;
	dev_dbg(&priv->i2c->dev,
			"%s(): lock=0x%x\n", __func__, data);
	*sync = ((data & CXD2841ER_ISDBT_SYNC_LOCK_BIT) ? 1 : 0);
	*tslock = ((data & CXD2841ER_ISDBT_TS_LOCK_BIT) ? 1 : 0);
	*unlock = ((data & CXD2841ER_ISDBT_UNLOCK_BIT) ? 1 : 0);
	return 0;
}

static int cxd2841er_read_status_tc(struct dvb_frontend *fe,
				    enum fe_status *status)
{
	int ret = 0;
	u8 sync = 0;
	u8 tslock = 0;
	u8 unlock = 0;
	struct cxd2841er_priv *priv = fe->demodulator_priv;

	*status = 0;
	if (priv->tune_error)
		return priv->tune_error;
	if (priv->state == STATE_ACTIVE_TC) {
		if (priv->system == SYS_DVBT || priv->system == SYS_DVBT2) {
			ret = cxd2841er_read_status_t_t2(
				priv, &sync, &tslock, &unlock);
			if (ret)
				goto done;
			if (unlock)
				goto done;
			if (sync)
				*status = FE_HAS_SIGNAL |
					FE_HAS_CARRIER |
					FE_HAS_VITERBI |
					FE_HAS_SYNC;
			if (sync && tslock)
				*status |= FE_HAS_LOCK;
		} else if (priv->system == SYS_ISDBT) {
			ret = cxd2841er_read_status_i(
					priv, &sync, &tslock, &unlock);
			if (ret)
				goto done;
			if (unlock)
				goto done;
			if (sync)
				*status = FE_HAS_SIGNAL |
					FE_HAS_CARRIER |
					FE_HAS_VITERBI |
					FE_HAS_SYNC;
			if (tslock)
				*status |= FE_HAS_LOCK;
		} else if (priv->system == SYS_DVBC_ANNEX_A) {
			ret = cxd2841er_read_status_c(priv, &tslock);
			if (ret)
				goto done;
			if (tslock)
				*status = FE_HAS_SIGNAL |
					FE_HAS_CARRIER |
					FE_HAS_VITERBI |
					FE_HAS_SYNC |
					FE_HAS_LOCK;
		}
	}
done:
	dev_dbg(&priv->i2c->dev, "%s(): status 0x%x\n", __func__, *status);
	return ret;
}

/* Failed offset reads must not change the frequency used by the next tune. */
static int cxd2841er_get_carrier_offset_s_s2(struct cxd2841er_priv *priv,
					     int *offset)
{
	int ret, unfreeze_ret;
	u8 data[3];
	u8 is_hs_mode;
	s32 cfrl_ctrlval;
	s32 temp_num, temp_div, temp_q, temp_r;

	if (priv->state != STATE_ACTIVE_S) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	/*
	 * Get High Sampling Rate mode
	 *  slave     Bank      Addr      Bit      Signal name
	 * <SLV-T>    A0h       10h       [0]      ITRL_LOCK
	 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_MONITOR);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_SAT_TIMING_RECOVERY_LOCK, &data[0]);
	if (ret)
		goto unfreeze;
	if (data[0] & CXD2841ER_SAT_TIMING_LOCK_BIT) {
		/*
		 *  slave     Bank      Addr      Bit      Signal name
		 * <SLV-T>    A0h       50h       [4]      IHSMODE
		 */
		ret = cxd2841er_read_reg(priv, I2C_SLVT,
			CXD2841ER_REG_SAT_HIGH_SAMPLING_MODE, &data[0]);
		if (ret)
			goto unfreeze;
		is_hs_mode = (data[0] & CXD2841ER_SAT_HIGH_SAMPLING_BIT ? 1 : 0);
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s(): unable to detect sampling rate mode\n",
			__func__);
		ret = -EINVAL;
		goto unfreeze;
	}
	/*
	 *  slave     Bank      Addr      Bit      Signal name
	 * <SLV-T>    A0h       45h       [4:0]    ICFRL_CTRLVAL[20:16]
	 * <SLV-T>    A0h       46h       [7:0]    ICFRL_CTRLVAL[15:8]
	 * <SLV-T>    A0h       47h       [7:0]    ICFRL_CTRLVAL[7:0]
	 */
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_CARRIER_OFFSET, data, 3);

unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;
	cfrl_ctrlval = sign_extend32((((u32)data[0] & CXD2841ER_SAT_CARRIER_HIGH_MASK)
		<< CXD2841ER_BYTE2_SHIFT) |
				(((u32)data[1] & CXD2841ER_BYTE_MASK) << CXD2841ER_BYTE1_SHIFT) |
				((u32)data[2] & CXD2841ER_BYTE_MASK),
			CXD2841ER_SAT_CARRIER_SIGN_BIT);
	if (priv->xtal == SONY_XTAL_24000) {
		temp_num = is_hs_mode ? 375 : 125;
		temp_div = is_hs_mode ? 4096 : 2048;
	} else {
		temp_num = 97375;
		temp_div = is_hs_mode ? 1048576 : 1572864;
	}
	if (cfrl_ctrlval > 0) {
		temp_q = div_s64_rem((s64)temp_num * cfrl_ctrlval,
			temp_div, &temp_r);
	} else {
		temp_q = div_s64_rem(-(s64)temp_num * cfrl_ctrlval,
			temp_div, &temp_r);
	}
	if (temp_r >= temp_div / 2)
		temp_q++;
	if (cfrl_ctrlval > 0)
		temp_q *= -1;
	*offset = temp_q;
	return 0;
}

static int cxd2841er_get_carrier_offset_i(struct cxd2841er_priv *priv,
					   u32 bandwidth, int *offset)
{
	int ret;
	u8 data[4];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_ISDBT) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_ISDBT_CARRIER_OFFSET, data, sizeof(data));
	if (ret)
		return ret;
	*offset = sign_extend32(
		((u32)(data[0] & CXD2841ER_TERRESTRIAL_CARRIER_HIGH_MASK)
			<< CXD2841ER_BYTE3_SHIFT) | ((u32)data[1] << CXD2841ER_BYTE2_SHIFT) |
		((u32)data[2] << CXD2841ER_BYTE1_SHIFT) | (u32)data[3],
			CXD2841ER_TERRESTRIAL_CARRIER_SIGN_BIT);

	switch (bandwidth) {
	case 6000000:
		*offset = div_s64((s64)*offset * 8, 264);
		break;
	case 7000000:
		*offset = div_s64((s64)*offset * 8, 231);
		break;
	case 8000000:
		*offset = div_s64((s64)*offset * 8, 198);
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid bandwidth %d\n",
				__func__, bandwidth);
		return -EINVAL;
	}

	dev_dbg(&priv->i2c->dev, "%s(): bandwidth %d offset %d\n",
			__func__, bandwidth, *offset);

	return 0;
}

static int cxd2841er_get_carrier_offset_t(struct cxd2841er_priv *priv,
					   u32 bandwidth, int *offset)
{
	int ret, unfreeze_ret;
	u8 data[4], sync, tslock, unlock;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_DVBT) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_read_status_t_t2(priv, &sync, &tslock, &unlock);
	if (ret)
		goto unfreeze;
	if (!sync) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_CARRIER_OFFSET, data, sizeof(data));

unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;
	*offset = div_s64(-(s64)sign_extend32(
		((u32)(data[0] & CXD2841ER_TERRESTRIAL_CARRIER_HIGH_MASK)
			<< CXD2841ER_BYTE3_SHIFT) | ((u32)data[1] << CXD2841ER_BYTE2_SHIFT) |
		((u32)data[2] << CXD2841ER_BYTE1_SHIFT) | (u32)data[3],
			CXD2841ER_TERRESTRIAL_CARRIER_SIGN_BIT) *
		(bandwidth / 1000000), 235);
	return 0;
}

static int cxd2841er_get_carrier_offset_t2(struct cxd2841er_priv *priv,
					   u32 bandwidth, int *offset)
{
	int ret, unfreeze_ret;
	u8 data[4], sync, tslock, unlock;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_DVBT2) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_read_status_t_t2(priv, &sync, &tslock, &unlock);
	if (ret)
		goto unfreeze;
	if (!sync) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_CARRIER_OFFSET, data, sizeof(data));

unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;
	*offset = -1 * sign_extend32(
		((u32)(data[0] & CXD2841ER_DVBT2_CARRIER_HIGH_MASK) << CXD2841ER_BYTE3_SHIFT)
			| ((u32)data[1] << CXD2841ER_BYTE2_SHIFT) |
		((u32)data[2] << CXD2841ER_BYTE1_SHIFT) | (u32)data[3],
			CXD2841ER_DVBT2_CARRIER_SIGN_BIT);
	switch (bandwidth) {
	case 1712000:
		*offset /= 582;
		break;
	case 5000000:
	case 6000000:
	case 7000000:
	case 8000000:
		*offset *= (bandwidth / 1000000);
		*offset /= 940;
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid bandwidth %d\n",
			__func__, bandwidth);
		return -EINVAL;
	}
	return 0;
}

static int cxd2841er_get_carrier_offset_c(struct cxd2841er_priv *priv,
					  int *offset)
{
	int ret, unfreeze_ret;
	u8 data[2], arlock;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	if (priv->system != SYS_DVBC_ANNEX_A) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid delivery system %d\n",
			__func__, priv->system);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		goto unfreeze;
	/* Validate lock in the same snapshot as the carrier estimate. */
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_ACQUISITION_LOCK, &arlock);
	if (ret)
		goto unfreeze;
	if (!(arlock & CXD2841ER_DVBC_ACQUISITION_LOCK_BIT)) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBC_CARRIER_OFFSET, data, sizeof(data));

unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;
	*offset = div_s64(41000000LL * sign_extend32((((u32)data[0]
		& CXD2841ER_DVBC_CARRIER_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT)
						| (u32)data[1],
			CXD2841ER_DVBC_CARRIER_SIGN_BIT), 16384);
	return 0;
}

/* Publish counters only after every register needed for the reading succeeds. */
static int cxd2841er_read_packet_errors_c(
		struct cxd2841er_priv *priv, u32 *penum)
{
	int ret;
	u8 data[3];

	*penum = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBC_PACKET_ERROR_MONITOR, data, sizeof(data));
	if (ret)
		return ret;
	if (!(data[2] & CXD2841ER_PACKET_ERROR_VALID_BIT))
		return -EINVAL;
	*penum = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	return 0;
}

static int cxd2841er_read_packet_errors_t(
		struct cxd2841er_priv *priv, u32 *penum)
{
	int ret;
	u8 data[3];

	*penum = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_PACKET_ERROR_MONITOR, data, sizeof(data));
	if (ret)
		return ret;
	if (!(data[2] & CXD2841ER_PACKET_ERROR_VALID_BIT))
		return -EINVAL;
	*penum = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	return 0;
}

static int cxd2841er_read_ber_c(struct cxd2841er_priv *priv,
		u32 *bit_error, u64 *bit_count)
{
	int ret;
	u8 data[3];
	u32 bit_err, period_exp;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBC_BIT_ERROR_MONITOR, data, sizeof(data));
	if (ret)
		return ret;
	if (!(data[0] & CXD2841ER_DVBC_BER_VALID_BIT)) {
		dev_dbg(&priv->i2c->dev,
				"%s(): no valid BER data\n", __func__);
		return -EINVAL;
	}
	bit_err = ((u32)(data[0] & CXD2841ER_DVBC_BER_HIGH_MASK) << CXD2841ER_BYTE2_SHIFT) |
		((u32)data[1] << CXD2841ER_BYTE1_SHIFT) |
		(u32)data[2];
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_BER_PERIOD, data);
	if (ret)
		return ret;
	period_exp = data[0] & CXD2841ER_DVBC_BER_PERIOD_MASK;

	if (bit_err > (1ULL << period_exp) * CXD2841ER_RS_PACKET_BYTES * CXD2841ER_BITS_PER_BYTE) {
		dev_dbg(&priv->i2c->dev,
				"%s(): period_exp(%u) or bit_err(%u)  not in range. no valid BER data\n",
				__func__, period_exp, bit_err);
		return -EINVAL;
	}

	dev_dbg(&priv->i2c->dev,
			"%s(): period_exp(%u) or bit_err(%u) count=%llu\n",
			__func__, period_exp, bit_err,
			((1ULL << period_exp) * CXD2841ER_RS_PACKET_BYTES
			* CXD2841ER_BITS_PER_BYTE));

	*bit_error = bit_err;
	*bit_count = ((1ULL << period_exp) * CXD2841ER_RS_PACKET_BYTES * CXD2841ER_BITS_PER_BYTE);

	return 0;
}

static int cxd2841er_read_ber_i(struct cxd2841er_priv *priv,
		u32 *bit_error, u64 *bit_count)
{
	int ret, unfreeze_ret;
	u8 data[3];
	u8 pktnum[2];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
		return -EINVAL;
	}

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_ISDBT_BER_PACKET_COUNT, pktnum, sizeof(pktnum));
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_ISDBT_BIT_ERROR_COUNT, data, sizeof(data));
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	if (!pktnum[0] && !pktnum[1]) {
		dev_dbg(&priv->i2c->dev,
				"%s(): no valid BER data\n", __func__);
		return -EINVAL;
	}

	*bit_error = ((u32)(data[0] & CXD2841ER_ISDBT_BER_HIGH_MASK) << CXD2841ER_BYTE2_SHIFT) |
		((u32)data[1] << CXD2841ER_BYTE1_SHIFT) | data[2];
	*bit_count = ((((u32)pktnum[0] << CXD2841ER_BYTE1_SHIFT) | pktnum[1])
		* CXD2841ER_RS_PACKET_BYTES * CXD2841ER_BITS_PER_BYTE);
	if (*bit_error > *bit_count)
		return -EINVAL;
	dev_dbg(&priv->i2c->dev, "%s(): bit_error=%u bit_count=%llu\n",
			__func__, *bit_error, (unsigned long long)*bit_count);

	return 0;
}

static int cxd2841er_mon_read_ber_s(struct cxd2841er_priv *priv,
				    u32 *bit_error, u64 *bit_count)
{
	int ret;
	u8 data[11];

	/* Set SLV-T Bank : 0xA0 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_MONITOR);
	if (ret)
		return ret;
	/*
	 *  slave     Bank      Addr      Bit      Signal name
	 * <SLV-T>    A0h       35h       [0]      IFVBER_VALID
	 * <SLV-T>    A0h       36h       [5:0]    IFVBER_BITERR[21:16]
	 * <SLV-T>    A0h       37h       [7:0]    IFVBER_BITERR[15:8]
	 * <SLV-T>    A0h       38h       [7:0]    IFVBER_BITERR[7:0]
	 * <SLV-T>    A0h       3Dh       [5:0]    IFVBER_BITNUM[21:16]
	 * <SLV-T>    A0h       3Eh       [7:0]    IFVBER_BITNUM[15:8]
	 * <SLV-T>    A0h       3Fh       [7:0]    IFVBER_BITNUM[7:0]
	 */
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBS_BIT_ERROR_MONITOR, data, 11);
	if (ret)
		return ret;
	if (data[0] & CXD2841ER_DVBS_BER_VALID_BIT) {
		*bit_error = ((u32)(data[1]  & CXD2841ER_DVBS_BER_COUNTER_HIGH_MASK)
			<< CXD2841ER_BYTE2_SHIFT) |
			     ((u32)(data[2]  & CXD2841ER_BYTE_MASK) << CXD2841ER_BYTE1_SHIFT) |
			     (u32)(data[3]  & CXD2841ER_BYTE_MASK);
		*bit_count = ((u32)(data[8]  & CXD2841ER_DVBS_BER_COUNTER_HIGH_MASK)
			<< CXD2841ER_BYTE2_SHIFT) |
			     ((u32)(data[9]  & CXD2841ER_BYTE_MASK) << CXD2841ER_BYTE1_SHIFT) |
			     (u32)(data[10] & CXD2841ER_BYTE_MASK);
		if ((*bit_count == 0) || (*bit_error > *bit_count)) {
			dev_dbg(&priv->i2c->dev,
				"%s(): invalid bit_error %d, bit_count %llu\n",
				__func__, *bit_error, (unsigned long long)*bit_count);
			return -EINVAL;
		}
		return 0;
	}
	dev_dbg(&priv->i2c->dev, "%s(): no data available\n", __func__);
	return -EINVAL;
}


static int cxd2841er_mon_read_ber_s2(struct cxd2841er_priv *priv,
				     u32 *bit_error, u64 *bit_count)
{
	int ret;
	u8 data[5];
	u32 period;

	/* Set SLV-T Bank : 0xB2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBS2_BER);
	if (ret)
		return ret;
	/*
	 *  slave     Bank      Addr      Bit      Signal name
	 * <SLV-T>    B2h       30h       [0]      IFLBER_VALID
	 * <SLV-T>    B2h       31h       [3:0]    IFLBER_BITERR[27:24]
	 * <SLV-T>    B2h       32h       [7:0]    IFLBER_BITERR[23:16]
	 * <SLV-T>    B2h       33h       [7:0]    IFLBER_BITERR[15:8]
	 * <SLV-T>    B2h       34h       [7:0]    IFLBER_BITERR[7:0]
	 */
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBS2_BIT_ERROR_MONITOR, data, 5);
	if (ret)
		return ret;
	if (data[0] & CXD2841ER_DVBS2_BER_VALID_BIT) {
		/* Bit error count */
		*bit_error = ((u32)(data[1] & CXD2841ER_DVBS2_BER_HIGH_MASK)
			<< CXD2841ER_BYTE3_SHIFT) |
			     ((u32)(data[2] & CXD2841ER_BYTE_MASK) << CXD2841ER_BYTE2_SHIFT) |
			     ((u32)(data[3] & CXD2841ER_BYTE_MASK) << CXD2841ER_BYTE1_SHIFT) |
			     (u32)(data[4] & CXD2841ER_BYTE_MASK);

		/* Set SLV-T Bank : 0xA0 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SAT_MONITOR);
		if (ret)
			return ret;
		ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBS2_BER_PERIOD, data);
		if (ret)
			return ret;
		/* Measurement period */
		period = (u32)(1 << (data[0] & CXD2841ER_DVBS2_BER_PERIOD_MASK));
		if (period == 0) {
			dev_dbg(&priv->i2c->dev,
				"%s(): period is 0\n", __func__);
			return -EINVAL;
		}
		if (*bit_error > (period * CXD2841ER_LDPC_NORMAL_BITS)) {
			dev_dbg(&priv->i2c->dev,
				"%s(): invalid bit_err 0x%x period 0x%x\n",
				__func__, *bit_error, period);
			return -EINVAL;
		}
		*bit_count = period * CXD2841ER_LDPC_NORMAL_BITS;

		return 0;
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s(): no data available\n", __func__);
	}
	return -EINVAL;
}

static int cxd2841er_read_ber_t2(struct cxd2841er_priv *priv,
				 u32 *bit_error, u64 *bit_count)
{
	int ret, unfreeze_ret;
	u8 data[4];
	u32 period_exp, n_ldpc;

	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT2);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_BIT_ERROR_MONITOR, data, sizeof(data));
	if (ret)
		goto unfreeze;
	if (!(data[0] & CXD2841ER_DVBT2_BER_VALID_BIT)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): no valid BER data\n", __func__);
		ret = -EINVAL;
		goto unfreeze;
	}
	*bit_error = ((u32)(data[0] & CXD2841ER_DVBT2_BER_HIGH_MASK) << CXD2841ER_BYTE3_SHIFT) |
		     ((u32)data[1] << CXD2841ER_BYTE2_SHIFT) |
		     ((u32)data[2] << CXD2841ER_BYTE1_SHIFT) |
		     (u32)data[3];
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT2_BER_PERIOD, data);
	if (ret)
		goto unfreeze;
	period_exp = data[0] & CXD2841ER_DVBT2_BER_PERIOD_MASK;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_22);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT2_LDPC_BLOCK_SIZE, data);
	if (ret)
		goto unfreeze;
	n_ldpc = ((data[0] & CXD2841ER_DVBT2_LDPC_SIZE_MASK) == 0 ? CXD2841ER_LDPC_SHORT_BITS
		: CXD2841ER_LDPC_NORMAL_BITS);
	if (*bit_error > ((1U << period_exp) * n_ldpc)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid BER value\n", __func__);
		ret = -EINVAL;
		goto unfreeze;
	}

	*bit_count = (1ULL << period_exp) * n_ldpc;
unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	return ret ? ret : unfreeze_ret;
}

static int cxd2841er_read_ber_t(struct cxd2841er_priv *priv,
				u32 *bit_error, u64 *bit_count)
{
	int ret, unfreeze_ret;
	u8 data[2];
	u32 period;

	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}
	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT_BER_VALID, data);
	if (ret)
		goto unfreeze;
	if (!(data[0] & CXD2841ER_DVBT_BER_VALID_BIT)) {
		dev_dbg(&priv->i2c->dev,
			"%s(): no valid BER data\n", __func__);
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_BIT_ERROR_COUNT, data, sizeof(data));
	if (ret)
		goto unfreeze;
	*bit_error = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT_BER_PERIOD, data);
	if (ret)
		goto unfreeze;
	period = ((data[0] & CXD2841ER_DVBT_BER_PERIOD_MASK) == 0)
		? CXD2841ER_DVBT_BER_MIN_PERIOD_BITS : (CXD2841ER_DVBT_BER_PERIOD_BASE_BITS
		<< (data[0] & CXD2841ER_DVBT_BER_PERIOD_MASK));

	if (*bit_error > period) {
		ret = -EINVAL;
		goto unfreeze;
	}
	*bit_count = period;
unfreeze:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	return ret ? ret : unfreeze_ret;
}

static int cxd2841er_freeze_regs(struct cxd2841er_priv *priv)
{
	/*
	 * Freeze registers: ensure multiple separate register reads
	 * are from the same snapshot
	 */
	return cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_MONITOR_FREEZE,
		CXD2841ER_SET_MONITOR_FREEZE);
}

static int cxd2841er_unfreeze_regs(struct cxd2841er_priv *priv)
{
	/*
	 * un-freeze registers
	 */
	return cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_MONITOR_FREEZE,
		CXD2841ER_SET_MONITOR_FREEZE_RELEASE);
}

static int cxd2841er_dvbs_read_snr(struct cxd2841er_priv *priv,
		u8 delsys, s32 *snr)
{
	int ret, unfreeze_ret;
	u8 data[3];
	u32 res = 0, value;
	int min_index, max_index, index;
	/* Keep the S/S2 lookup table local so another frontend cannot change it. */
	const struct cxd2841er_cnr_data *cn_data;

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0xA1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_CNR);
	if (ret)
		goto unfreeze;
	/*
	 *  slave     Bank      Addr      Bit     Signal name
	 * <SLV-T>    A1h       10h       [0]     ICPM_QUICKRDY
	 * <SLV-T>    A1h       11h       [4:0]   ICPM_QUICKCNDT[12:8]
	 * <SLV-T>    A1h       12h       [7:0]   ICPM_QUICKCNDT[7:0]
	 */
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_CNR_QUICK_MONITOR, data, 3);
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	if (data[0] & CXD2841ER_SAT_CNR_VALID_BIT) {
		value = ((u32)(data[1] & CXD2841ER_SAT_CNR_HIGH_MASK)
			<< CXD2841ER_BYTE1_SHIFT) | (u32)(data[2] & CXD2841ER_BYTE_MASK);
		min_index = 0;
		if (delsys == SYS_DVBS) {
			cn_data = s_cn_data;
			max_index = ARRAY_SIZE(s_cn_data) - 1;
		} else {
			cn_data = s2_cn_data;
			max_index = ARRAY_SIZE(s2_cn_data) - 1;
		}
		if (value >= cn_data[min_index].value) {
			res = cn_data[min_index].cnr_x1000;
			goto done;
		}
		if (value <= cn_data[max_index].value) {
			res = cn_data[max_index].cnr_x1000;
			goto done;
		}
		while ((max_index - min_index) > 1) {
			index = (max_index + min_index) / 2;
			if (value == cn_data[index].value) {
				res = cn_data[index].cnr_x1000;
				goto done;
			} else if (value > cn_data[index].value)
				max_index = index;
			else
				min_index = index;
			if ((max_index - min_index) <= 1) {
				if (value == cn_data[max_index].value) {
					res = cn_data[max_index].cnr_x1000;
					goto done;
				} else {
					res = cn_data[min_index].cnr_x1000;
					goto done;
				}
			}
		}
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s(): no data available\n", __func__);
		return -EINVAL;
	}
done:
	*snr = res;
	return 0;
}

static uint32_t sony_log(uint32_t x)
{
	return (((10000>>8)*(intlog2(x)>>16) + LOG2_E_100X/2)/LOG2_E_100X);
}

static int cxd2841er_read_snr_c(struct cxd2841er_priv *priv, s32 *snr)
{
	int ret, unfreeze_ret;
	u32 reg;
	u8 data[2];
	enum sony_dvbc_constellation_t qam = SONY_DVBC_CONSTELLATION_16QAM;

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
				"%s(): invalid state %d\n",
				__func__, priv->state);
		return -EINVAL;
	}

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_ACQUISITION_LOCK, data);
	if (ret)
		goto unfreeze;
	if (!(data[0] & CXD2841ER_DVBC_ACQUISITION_LOCK_BIT)) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBC_DETECTED_QAM, data, 1);
	if (ret)
		goto unfreeze;
	qam = (enum sony_dvbc_constellation_t) (data[0] & CXD2841ER_DVBC_QAM_MASK);
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBC_SNR_ESTIMATE, data, 2);
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	reg = ((u32)(data[0]&CXD2841ER_DVBC_SNR_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
				"%s(): reg value out of range\n", __func__);
		return -EINVAL;
	}

	switch (qam) {
	case SONY_DVBC_CONSTELLATION_16QAM:
	case SONY_DVBC_CONSTELLATION_64QAM:
	case SONY_DVBC_CONSTELLATION_256QAM:
		/* SNR(dB) = -9.50 * ln(IREG_SNR_ESTIMATE / (24320)) */
		if (reg < CXD2841ER_DVBC_SNR_MIN_QAM16_64_256)
			reg = CXD2841ER_DVBC_SNR_MIN_QAM16_64_256;
		*snr = -95 * (int32_t)sony_log(reg) + 95941;
		break;
	case SONY_DVBC_CONSTELLATION_32QAM:
	case SONY_DVBC_CONSTELLATION_128QAM:
		/* SNR(dB) = -8.75 * ln(IREG_SNR_ESTIMATE / (20800)) */
		if (reg < CXD2841ER_DVBC_SNR_MIN_QAM32_128)
			reg = CXD2841ER_DVBC_SNR_MIN_QAM32_128;
		*snr = -88 * (int32_t)sony_log(reg) + 86999;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int cxd2841er_read_snr_t(struct cxd2841er_priv *priv, s32 *snr)
{
	int ret, unfreeze_ret;
	u32 reg;
	u8 data[2];

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT_SYNC_STATUS, data);
	if (ret)
		goto unfreeze;
	if ((data[0] & CXD2841ER_DVBT_T2_SYNC_STATE_MASK) != CXD2841ER_DVBT_T2_SYNC_LOCKED) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_SNR_MONITOR, data, sizeof(data));
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	reg = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
			"%s(): reg value out of range\n", __func__);
		return -EINVAL;
	}
	if (reg > CXD2841ER_DVBT_SNR_MAX)
		reg = CXD2841ER_DVBT_SNR_MAX;
	*snr = 100 * (((s32)INTLOG10X100(reg) - (s32)INTLOG10X100(5350 - reg)) + 285);
	return 0;
}

static int cxd2841er_read_snr_t2(struct cxd2841er_priv *priv, s32 *snr)
{
	int ret, unfreeze_ret;
	u32 reg;
	u8 data[2];

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid state %d\n", __func__, priv->state);
		return -EINVAL;
	}

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT2);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT2_SYNC_STATUS, data);
	if (ret)
		goto unfreeze;
	if ((data[0] & CXD2841ER_DVBT_T2_SYNC_STATE_MASK) != CXD2841ER_DVBT_T2_SYNC_LOCKED) {
		ret = -EINVAL;
		goto unfreeze;
	}
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SNR_MONITOR, data, sizeof(data));
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	reg = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
			"%s(): reg value out of range\n", __func__);
		return -EINVAL;
	}
	if (reg > CXD2841ER_DVBT2_SNR_MAX)
		reg = CXD2841ER_DVBT2_SNR_MAX;
	*snr = 100 * (((s32)INTLOG10X100(reg) - (s32)INTLOG10X100(12600 - reg)) + 320);
	return 0;
}

static int cxd2841er_read_snr_i(struct cxd2841er_priv *priv, s32 *snr)
{
	int ret, unfreeze_ret;
	u32 reg;
	u8 data[2];

	*snr = 0;
	if (priv->state != STATE_ACTIVE_TC) {
		dev_dbg(&priv->i2c->dev,
				"%s(): invalid state %d\n", __func__,
				priv->state);
		return -EINVAL;
	}

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		goto unfreeze;
	ret = cxd2841er_read_regs(priv, I2C_SLVT,
		CXD2841ER_REG_ISDBT_SNR_MONITOR, data, sizeof(data));
	if (ret)
		goto unfreeze;
unfreeze:
	/* Release the snapshot, but keep the read error if both operations fail. */
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;

	reg = ((u32)data[0] << CXD2841ER_BYTE1_SHIFT) | (u32)data[1];
	if (reg == 0) {
		dev_dbg(&priv->i2c->dev,
				"%s(): reg value out of range\n", __func__);
		return -EINVAL;
	}
	*snr = (s32)(((u64)intlog10(reg) * 10000) >> 24) - 9031;
	return 0;
}

/* AGC values fit in int, leaving negative returns available for bus errors. */
static int cxd2841er_read_agc_gain_c(struct cxd2841er_priv *priv,
					u8 delsys)
{
	int ret;
	u8 data[2];

	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBC_IF_AGC_GAIN, data, 2);
	if (ret)
		return ret;
	dev_dbg(&priv->i2c->dev,
			"%s(): AGC value=%u\n",
			__func__, (((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK)
			<< CXD2841ER_BYTE1_SHIFT) |
			(u16)(data[1] & CXD2841ER_BYTE_MASK));
	return ((((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT)
		| (u16)(data[1] & CXD2841ER_BYTE_MASK)) << CXD2841ER_IF_AGC_SCALE_SHIFT;
}

static int cxd2841er_read_agc_gain_t_t2(struct cxd2841er_priv *priv,
					u8 delsys)
{
	int ret;
	u8 data[2];

	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		(delsys == SYS_DVBT
			? CXD2841ER_BANK_DVBT
			: CXD2841ER_BANK_DVBT2));
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBT_T2_IF_AGC_GAIN, data, 2);
	if (ret)
		return ret;
	dev_dbg(&priv->i2c->dev,
			"%s(): AGC value=%u\n",
			__func__, (((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK)
			<< CXD2841ER_BYTE1_SHIFT) |
			(u16)(data[1] & CXD2841ER_BYTE_MASK));
	return ((((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT)
		| (u16)(data[1] & CXD2841ER_BYTE_MASK)) << CXD2841ER_IF_AGC_SCALE_SHIFT;
}

static int cxd2841er_read_agc_gain_i(struct cxd2841er_priv *priv,
		u8 delsys)
{
	int ret;
	u8 data[2];

	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		return ret;
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_ISDBT_IF_AGC_GAIN, data, 2);
	if (ret)
		return ret;

	dev_dbg(&priv->i2c->dev,
			"%s(): AGC value=%u\n",
			__func__, (((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK)
			<< CXD2841ER_BYTE1_SHIFT) |
			(u16)(data[1] & CXD2841ER_BYTE_MASK));
	return ((((u16)data[0] & CXD2841ER_IF_AGC_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT)
		| (u16)(data[1] & CXD2841ER_BYTE_MASK)) << CXD2841ER_IF_AGC_SCALE_SHIFT;
}

static int cxd2841er_read_agc_gain_s(struct cxd2841er_priv *priv)
{
	int ret;
	u8 data[2];

	/* Set SLV-T Bank : 0xA0 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_MONITOR);
	if (ret)
		return ret;
	/*
	 *  slave     Bank      Addr      Bit       Signal name
	 * <SLV-T>    A0h       1Fh       [4:0]     IRFAGC_GAIN[12:8]
	 * <SLV-T>    A0h       20h       [7:0]     IRFAGC_GAIN[7:0]
	 */
	ret = cxd2841er_read_regs(priv, I2C_SLVT, CXD2841ER_REG_SAT_RF_AGC_GAIN, data, 2);
	if (ret)
		return ret;
	return ((((u16)data[0] & CXD2841ER_RF_AGC_HIGH_MASK) << CXD2841ER_BYTE1_SHIFT)
		| (u16)(data[1] & CXD2841ER_BYTE_MASK)) << CXD2841ER_RF_AGC_SCALE_SHIFT;
}

static void cxd2841er_read_ber(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	u32 bit_error = 0;
	u64 bit_count = 0;
	struct dtv_fe_stats *error = &p->post_bit_error;
	struct dtv_fe_stats *count = &p->post_bit_count;
	int ret;

	if (p->delivery_system == SYS_DVBT || p->delivery_system == SYS_DVBT2 ||
	    p->delivery_system == SYS_DVBS || p->delivery_system == SYS_DVBS2) {
		error = &p->pre_bit_error;
		count = &p->pre_bit_count;
	}
	p->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	p->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	switch (p->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBC_ANNEX_C:
		ret = cxd2841er_read_ber_c(priv, &bit_error, &bit_count);
		break;
	case SYS_ISDBT:
		ret = cxd2841er_read_ber_i(priv, &bit_error, &bit_count);
		break;
	case SYS_DVBS:
		ret = cxd2841er_mon_read_ber_s(priv, &bit_error, &bit_count);
		break;
	case SYS_DVBS2:
		ret = cxd2841er_mon_read_ber_s2(priv, &bit_error, &bit_count);
		break;
	case SYS_DVBT:
		ret = cxd2841er_read_ber_t(priv, &bit_error, &bit_count);
		break;
	case SYS_DVBT2:
		ret = cxd2841er_read_ber_t2(priv, &bit_error, &bit_count);
		break;
	default:
		error->stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		count->stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return;
	}

	if (!ret) {
		error->stat[0].scale = FE_SCALE_COUNTER;
		error->stat[0].uvalue += bit_error;
		count->stat[0].scale = FE_SCALE_COUNTER;
		count->stat[0].uvalue += bit_count;
	} else {
		/* Hide the old counters until a complete reading succeeds again. */
		error->stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		count->stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
}

static void cxd2841er_read_signal_strength(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	s32 strength;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		strength = cxd2841er_read_agc_gain_t_t2(priv,
						p->delivery_system);
		break;
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBC_ANNEX_C:
		strength = cxd2841er_read_agc_gain_c(priv,
						p->delivery_system);
		break;
	case SYS_ISDBT:
		strength = cxd2841er_read_agc_gain_i(priv, p->delivery_system);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		strength = cxd2841er_read_agc_gain_s(priv);
		if (strength >= 0)
			strength = 65535 - strength;
		break;
	default:
		goto error;
	}
	if (strength < 0)
		goto error;
	/* Demodulator AGC is not calibrated RF power for the attached tuner. */
	p->strength.stat[0].scale = FE_SCALE_RELATIVE;
	p->strength.stat[0].uvalue = strength;
	return;
error:
	p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
}

static void cxd2841er_read_snr(struct dvb_frontend *fe)
{
	s32 tmp = 0;
	int ret = 0;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBC_ANNEX_C:
		ret = cxd2841er_read_snr_c(priv, &tmp);
		break;
	case SYS_DVBT:
		ret = cxd2841er_read_snr_t(priv, &tmp);
		break;
	case SYS_DVBT2:
		ret = cxd2841er_read_snr_t2(priv, &tmp);
		break;
	case SYS_ISDBT:
		ret = cxd2841er_read_snr_i(priv, &tmp);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		ret = cxd2841er_dvbs_read_snr(priv, p->delivery_system, &tmp);
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): unknown delivery system %d\n",
			__func__, p->delivery_system);
		p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return;
	}

	dev_dbg(&priv->i2c->dev, "%s(): snr=%d\n",
			__func__, (int32_t)tmp);

	if (!ret) {
		p->cnr.stat[0].scale = FE_SCALE_DECIBEL;
		p->cnr.stat[0].svalue = tmp;
	} else {
		p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
}

static void cxd2841er_read_ucblocks(struct dvb_frontend *fe)
{
	int ret;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	u32 ucblocks = 0;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (p->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_B:
	case SYS_DVBC_ANNEX_C:
		ret = cxd2841er_read_packet_errors_c(priv, &ucblocks);
		break;
	case SYS_DVBT:
		ret = cxd2841er_read_packet_errors_t(priv, &ucblocks);
		break;
	default:
		/* T2/ISDB snapshot timing is unknown; do not invent cumulative counts. */
		p->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return;
	}
	if (ret) {
		p->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		return;
	}
	dev_dbg(&priv->i2c->dev, "%s() ucblocks=%u\n", __func__, ucblocks);

	p->block_error.stat[0].scale = FE_SCALE_COUNTER;
	/* Sony T/C monitors count errors over one second, matching our poll interval. */
	p->block_error.stat[0].uvalue += ucblocks;
}

static int cxd2841er_dvbt2_set_profile(
	struct cxd2841er_priv *priv, enum cxd2841er_dvbt2_profile_t profile)
{
	int ret;

	u8 tune_mode;
	u8 seq_not2d_time;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	switch (profile) {
	case DVBT2_PROFILE_BASE:
		tune_mode = CXD2841ER_SET_DVBT2_PROFILE_BASE;
		/* Set early unlock time */
		seq_not2d_time = (priv->xtal == SONY_XTAL_24000)
			? CXD2841ER_SET_DVBT2_UNLOCK_BASE_24_MHZ
			: CXD2841ER_SET_DVBT2_UNLOCK_BASE_OTHER_XTAL;
		break;
	case DVBT2_PROFILE_LITE:
		tune_mode = CXD2841ER_SET_DVBT2_PROFILE_LITE;
		/* Set early unlock time */
		seq_not2d_time = (priv->xtal == SONY_XTAL_24000)
			? CXD2841ER_SET_DVBT2_UNLOCK_LITE_OR_ANY_24_MHZ
			: CXD2841ER_SET_DVBT2_UNLOCK_LITE_OR_ANY_OTHER_XTAL;
		break;
	case DVBT2_PROFILE_ANY:
		tune_mode = CXD2841ER_SET_DVBT2_PROFILE_ANY;
		/* Set early unlock time */
		seq_not2d_time = (priv->xtal == SONY_XTAL_24000)
			? CXD2841ER_SET_DVBT2_UNLOCK_LITE_OR_ANY_24_MHZ
			: CXD2841ER_SET_DVBT2_UNLOCK_LITE_OR_ANY_OTHER_XTAL;
		break;
	default:
		return -EINVAL;
	}
	/* Set SLV-T Bank : 0x2E */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2E);
	if (ret)
		return ret;
	/* Set profile and tune mode */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_PROFILE_TUNE_MODE,
		tune_mode,
		CXD2841ER_MASK_DVBT2_PROFILE_TUNE_MODE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x2B */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2B);
	if (ret)
		return ret;
	/* Set early unlock detection time */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_EARLY_UNLOCK_TIME, seq_not2d_time);
	if (ret)
		return ret;
	return 0;
}

static int cxd2841er_dvbt2_set_plp_config(struct cxd2841er_priv *priv,
					  u8 is_auto, u8 plp_id)
{
	int ret;

	if (is_auto) {
		dev_dbg(&priv->i2c->dev,
			"%s() using auto PLP selection\n", __func__);
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s() using manual PLP selection, ID %d\n",
			__func__, plp_id);
	}
	/* Set SLV-T Bank : 0x23 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_23);
	if (ret)
		return ret;
	if (!is_auto) {
		/* Manual PLP selection mode. Set the data PLP Id. */
		ret = cxd2841er_write_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBT2_DATA_PLP_ID, plp_id);
		if (ret)
			return ret;
	}
	/* Auto PLP select (Scanning mode = 0x00). Data PLP select = 0x01. */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_PLP_SELECTION_MODE,
		(is_auto
			? CXD2841ER_SET_DVBT2_PLP_SELECTION_MODE_AUTO
			: CXD2841ER_SET_DVBT2_PLP_SELECTION_MODE_MANUAL));
	if (ret)
		return ret;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t2_band(struct cxd2841er_priv *priv,
						u32 bandwidth)
{
	int ret;
	u32 iffreq, ifhz;
	u8 data[MAX_WRITE_REGSIZE];

	static const uint8_t nominalRate8bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x11, 0xF0, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x15, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x11, 0xF0, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};

	static const uint8_t nominalRate7bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x14, 0x80, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x18, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x14, 0x80, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};

	static const uint8_t nominalRate6bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x17, 0xEA, 0xAA, 0xAA, 0xAA}, /* 20.5MHz XTal */
		{0x1C, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x17, 0xEA, 0xAA, 0xAA, 0xAA}  /* 41MHz XTal */
	};

	static const uint8_t nominalRate5bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x1C, 0xB3, 0x33, 0x33, 0x33}, /* 20.5MHz XTal */
		{0x21, 0x99, 0x99, 0x99, 0x99}, /* 24MHz XTal */
		{0x1C, 0xB3, 0x33, 0x33, 0x33}  /* 41MHz XTal */
	};

	static const uint8_t nominalRate17bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x58, 0xE2, 0xAF, 0xE0, 0xBC}, /* 20.5MHz XTal */
		{0x68, 0x0F, 0xA2, 0x32, 0xD0}, /* 24MHz XTal */
		{0x58, 0xE2, 0xAF, 0xE0, 0xBC}  /* 41MHz XTal */
	};

	static const uint8_t itbCoef8bw[3][14] = {
		{0x26, 0xAF, 0x06, 0xCD, 0x13, 0xBB, 0x28, 0xBA,
			0x23, 0xA9, 0x1F, 0xA8, 0x2C, 0xC8}, /* 20.5MHz XTal */
		{0x2F, 0xBA, 0x28, 0x9B, 0x28, 0x9D, 0x28, 0xA1,
			0x29, 0xA5, 0x2A, 0xAC, 0x29, 0xB5}, /* 24MHz XTal   */
		{0x26, 0xAF, 0x06, 0xCD, 0x13, 0xBB, 0x28, 0xBA,
			0x23, 0xA9, 0x1F, 0xA8, 0x2C, 0xC8}  /* 41MHz XTal   */
	};

	static const uint8_t itbCoef7bw[3][14] = {
		{0x2C, 0xBD, 0x02, 0xCF, 0x04, 0xF8, 0x23, 0xA6,
			0x29, 0xB0, 0x26, 0xA9, 0x21, 0xA5}, /* 20.5MHz XTal */
		{0x30, 0xB1, 0x29, 0x9A, 0x28, 0x9C, 0x28, 0xA0,
			0x29, 0xA2, 0x2B, 0xA6, 0x2B, 0xAD}, /* 24MHz XTal   */
		{0x2C, 0xBD, 0x02, 0xCF, 0x04, 0xF8, 0x23, 0xA6,
			0x29, 0xB0, 0x26, 0xA9, 0x21, 0xA5}  /* 41MHz XTal   */
	};

	static const uint8_t itbCoef6bw[3][14] = {
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
			0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4}, /* 20.5MHz XTal */
		{0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E,
			0x29, 0xA4, 0x29, 0xA2, 0x29, 0xA8}, /* 24MHz XTal   */
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
			0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4}  /* 41MHz XTal   */
	};

	static const uint8_t itbCoef5bw[3][14] = {
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
			0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4}, /* 20.5MHz XTal */
		{0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E,
			0x29, 0xA4, 0x29, 0xA2, 0x29, 0xA8}, /* 24MHz XTal   */
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
			0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4}  /* 41MHz XTal   */
	};

	static const uint8_t itbCoef17bw[3][14] = {
		{0x25, 0xA0, 0x36, 0x8D, 0x2E, 0x94, 0x28, 0x9B,
			0x32, 0x90, 0x2C, 0x9D, 0x29, 0x99}, /* 20.5MHz XTal */
		{0x33, 0x8E, 0x2B, 0x97, 0x2D, 0x95, 0x37, 0x8B,
			0x30, 0x97, 0x2D, 0x9A, 0x21, 0xA4}, /* 24MHz XTal   */
		{0x25, 0xA0, 0x36, 0x8D, 0x2E, 0x94, 0x28, 0x9B,
			0x32, 0x90, 0x2C, 0x9D, 0x29, 0x99}  /* 41MHz XTal   */
	};

	/* Set SLV-T Bank : 0x20 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT2);
	if (ret)
		return ret;

	switch (bandwidth) {
	case 8000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_DVBT2_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate8bw[priv->xtal], 5);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_TRCG_LMTVAL0,
			CXD2841ER_SET_DVBT2_TRCG_LMTVAL0_5_TO_8_MHZ,
			CXD2841ER_MASK_DVBT2_TRCG_LMTVAL0);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;

		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		 */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef8bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 4800000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_8_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;
		break;
	case 7000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_DVBT2_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate7bw[priv->xtal], 5);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_TRCG_LMTVAL0,
			CXD2841ER_SET_DVBT2_TRCG_LMTVAL0_5_TO_8_MHZ,
			CXD2841ER_MASK_DVBT2_TRCG_LMTVAL0);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;

		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		 */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef7bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 4200000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_7_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;
		break;
	case 6000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_DVBT2_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate6bw[priv->xtal], 5);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_TRCG_LMTVAL0,
			CXD2841ER_SET_DVBT2_TRCG_LMTVAL0_5_TO_8_MHZ,
			CXD2841ER_MASK_DVBT2_TRCG_LMTVAL0);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;

		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		 */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef6bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 3600000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_6_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;
		break;
	case 5000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_DVBT2_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate5bw[priv->xtal], 5);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_TRCG_LMTVAL0,
			CXD2841ER_SET_DVBT2_TRCG_LMTVAL0_5_TO_8_MHZ,
			CXD2841ER_MASK_DVBT2_TRCG_LMTVAL0);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;

		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		 */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef5bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 3600000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_5_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;
		break;
	case 1712000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_DVBT2_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate17bw[priv->xtal], 5);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_TRCG_LMTVAL0,
			CXD2841ER_SET_DVBT2_TRCG_LMTVAL0_1_712_MHZ,
			CXD2841ER_MASK_DVBT2_TRCG_LMTVAL0);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;

		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		 */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef17bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 3500000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_1_712_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t_band(
		struct cxd2841er_priv *priv, u32 bandwidth)
{
	int ret;
	u8 data[MAX_WRITE_REGSIZE];
	u32 iffreq, ifhz;
	static const u8 nominalRate8bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x11, 0xF0, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x15, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x11, 0xF0, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};
	static const u8 nominalRate7bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x14, 0x80, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x18, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x14, 0x80, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};
	static const u8 nominalRate6bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x17, 0xEA, 0xAA, 0xAA, 0xAA}, /* 20.5MHz XTal */
		{0x1C, 0x00, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x17, 0xEA, 0xAA, 0xAA, 0xAA}  /* 41MHz XTal */
	};
	static const u8 nominalRate5bw[3][5] = {
		/* TRCG Nominal Rate [37:0] */
		{0x1C, 0xB3, 0x33, 0x33, 0x33}, /* 20.5MHz XTal */
		{0x21, 0x99, 0x99, 0x99, 0x99}, /* 24MHz XTal */
		{0x1C, 0xB3, 0x33, 0x33, 0x33}  /* 41MHz XTal */
	};

	static const u8 itbCoef8bw[3][14] = {
		{0x26, 0xAF, 0x06, 0xCD, 0x13, 0xBB, 0x28, 0xBA, 0x23, 0xA9,
			0x1F, 0xA8, 0x2C, 0xC8}, /* 20.5MHz XTal */
		{0x2F, 0xBA, 0x28, 0x9B, 0x28, 0x9D, 0x28, 0xA1, 0x29, 0xA5,
			0x2A, 0xAC, 0x29, 0xB5}, /* 24MHz XTal   */
		{0x26, 0xAF, 0x06, 0xCD, 0x13, 0xBB, 0x28, 0xBA, 0x23, 0xA9,
			0x1F, 0xA8, 0x2C, 0xC8}  /* 41MHz XTal   */
	};
	static const u8 itbCoef7bw[3][14] = {
		{0x2C, 0xBD, 0x02, 0xCF, 0x04, 0xF8, 0x23, 0xA6, 0x29, 0xB0,
			0x26, 0xA9, 0x21, 0xA5}, /* 20.5MHz XTal */
		{0x30, 0xB1, 0x29, 0x9A, 0x28, 0x9C, 0x28, 0xA0, 0x29, 0xA2,
			0x2B, 0xA6, 0x2B, 0xAD}, /* 24MHz XTal   */
		{0x2C, 0xBD, 0x02, 0xCF, 0x04, 0xF8, 0x23, 0xA6, 0x29, 0xB0,
			0x26, 0xA9, 0x21, 0xA5}  /* 41MHz XTal   */
	};
	static const u8 itbCoef6bw[3][14] = {
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00, 0xCF,
			0x00, 0xE6, 0x23, 0xA4}, /* 20.5MHz XTal */
		{0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E, 0x29, 0xA4,
			0x29, 0xA2, 0x29, 0xA8}, /* 24MHz XTal   */
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00, 0xCF,
			0x00, 0xE6, 0x23, 0xA4}  /* 41MHz XTal   */
	};
	static const u8 itbCoef5bw[3][14] = {
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00, 0xCF,
			0x00, 0xE6, 0x23, 0xA4}, /* 20.5MHz XTal */
		{0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E, 0x29, 0xA4,
			0x29, 0xA2, 0x29, 0xA8}, /* 24MHz XTal   */
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00, 0xCF,
			0x00, 0xE6, 0x23, 0xA4}  /* 41MHz XTal   */
	};

	if (bandwidth != 5000000 && bandwidth != 6000000 &&
	    bandwidth != 7000000 && bandwidth != 8000000)
		return -EINVAL;

	/* Set SLV-T Bank : 0x13 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_13);
	if (ret)
		return ret;
	/* Echo performance optimization setting */
	data[0] = 0x01;
	data[1] = 0x14;
	ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_DVBT_ECHO_OPTIMIZATION, data, 2);
	if (ret)
		return ret;

	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;

	switch (bandwidth) {
	case 8000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate8bw[priv->xtal], 5);
		if (ret)
			return ret;
		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		*/
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef8bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 4800000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_8_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		if (priv->xtal == SONY_XTAL_24000) {
			data[0] = 0x15;
			data[1] = 0x28;
		} else {
			data[0] = 0x01;
			data[1] = 0xE0;
		}
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Notch filter setting */
		data[0] = 0x01;
		data[1] = 0x02;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_17);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_NOTCH_FILTER, data, 2);
		if (ret)
			return ret;
		break;
	case 7000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate7bw[priv->xtal], 5);
		if (ret)
			return ret;
		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		*/
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef7bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 4200000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_7_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		if (priv->xtal == SONY_XTAL_24000) {
			data[0] = 0x1F;
			data[1] = 0xF8;
		} else {
			data[0] = 0x12;
			data[1] = 0xF8;
		}
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Notch filter setting */
		data[0] = 0x00;
		data[1] = 0x03;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_17);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_NOTCH_FILTER, data, 2);
		if (ret)
			return ret;
		break;
	case 6000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate6bw[priv->xtal], 5);
		if (ret)
			return ret;
		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		*/
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef6bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 3600000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_6_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		if (priv->xtal == SONY_XTAL_24000) {
			data[0] = 0x25;
			data[1] = 0x4C;
		} else {
			data[0] = 0x1F;
			data[1] = 0xDC;
		}
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Notch filter setting */
		data[0] = 0x00;
		data[1] = 0x03;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_17);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_NOTCH_FILTER, data, 2);
		if (ret)
			return ret;
		break;
	case 5000000:
		/* <Timing Recovery setting> */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate5bw[priv->xtal], 5);
		if (ret)
			return ret;
		/* Group delay equaliser settings for
		 * ASCOT2D, ASCOT2E and ASCOT3 tuners
		*/
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef5bw[priv->xtal], 14);
			if (ret)
				return ret;
		}
		/* <IF freq setting> */
		ret = cxd2841er_get_if_hz(priv, 3600000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;
		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_5_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		if (priv->xtal == SONY_XTAL_24000) {
			data[0] = 0x2C;
			data[1] = 0xC2;
		} else {
			data[0] = 0x26;
			data[1] = 0x3C;
		}
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Notch filter setting */
		data[0] = 0x00;
		data[1] = 0x03;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_17);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_NOTCH_FILTER, data, 2);
		if (ret)
			return ret;
		break;
	}

	return 0;
}

static int cxd2841er_sleep_tc_to_active_i_band(
		struct cxd2841er_priv *priv, u32 bandwidth)
{
	int ret;
	u32 iffreq, ifhz;
	u8 data[3];

	/* TRCG Nominal Rate */
	static const u8 nominalRate8bw[3][5] = {
		{0x00, 0x00, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x11, 0xB8, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x00, 0x00, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};

	static const u8 nominalRate7bw[3][5] = {
		{0x00, 0x00, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x14, 0x40, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x00, 0x00, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};

	static const u8 nominalRate6bw[3][5] = {
		{0x14, 0x2E, 0x00, 0x00, 0x00}, /* 20.5MHz XTal */
		{0x17, 0xA0, 0x00, 0x00, 0x00}, /* 24MHz XTal */
		{0x14, 0x2E, 0x00, 0x00, 0x00}  /* 41MHz XTal */
	};

	static const u8 itbCoef8bw[3][14] = {
		{0x00}, /* 20.5MHz XTal */
		{0x2F, 0xBA, 0x28, 0x9B, 0x28, 0x9D, 0x28, 0xA1, 0x29,
			0xA5, 0x2A, 0xAC, 0x29, 0xB5}, /* 24MHz Xtal */
		{0x0}, /* 41MHz XTal   */
	};

	static const u8 itbCoef7bw[3][14] = {
		{0x00}, /* 20.5MHz XTal */
		{0x30, 0xB1, 0x29, 0x9A, 0x28, 0x9C, 0x28, 0xA0, 0x29,
			0xA2, 0x2B, 0xA6, 0x2B, 0xAD}, /* 24MHz Xtal */
		{0x00}, /* 41MHz XTal   */
	};

	static const u8 itbCoef6bw[3][14] = {
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00,
			0xCF, 0x00, 0xE6, 0x23, 0xA4}, /* 20.5MHz XTal */
		{0x31, 0xA8, 0x29, 0x9B, 0x27, 0x9C, 0x28, 0x9E, 0x29,
			0xA4, 0x29, 0xA2, 0x29, 0xA8}, /* 24MHz Xtal   */
		{0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8, 0x00,
			0xCF, 0x00, 0xE6, 0x23, 0xA4}, /* 41MHz XTal   */
	};

	dev_dbg(&priv->i2c->dev, "%s() bandwidth=%u\n", __func__, bandwidth);
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;

	/*  20.5/41MHz Xtal support is not available
	 *  on ISDB-T 7MHzBW and 8MHzBW
	*/
	if (priv->xtal != SONY_XTAL_24000 && bandwidth > 6000000) {
		dev_err(&priv->i2c->dev,
			"%s(): bandwidth %d supported only for 24MHz xtal\n",
			__func__, bandwidth);
		return -EINVAL;
	}

	switch (bandwidth) {
	case 8000000:
		/* TRCG Nominal Rate */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate8bw[priv->xtal], 5);
		if (ret)
			return ret;
		/*  Group delay equaliser settings for ASCOT tuners optimized */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef8bw[priv->xtal], 14);
			if (ret)
				return ret;
		}

		/* IF freq setting */
		ret = cxd2841er_get_if_hz(priv, 4750000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;

		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_8_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		data[0] = 0x13;
		data[1] = 0xFC;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Acquisition optimization setting */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_12);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_SYR_ACTMODE,
			CXD2841ER_SET_ISDB_SYR_ACTMODE_7_8_MHZ,
			CXD2841ER_MASK_ISDB_SYR_ACTMODE);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_15);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_FCS_SYMBOL_INTERVAL,
			CXD2841ER_SET_ISDB_FCS_SYMBOL_INTERVAL_8_MHZ);
		if (ret)
			return ret;
		break;
	case 7000000:
		/* TRCG Nominal Rate */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate7bw[priv->xtal], 5);
		if (ret)
			return ret;
		/*  Group delay equaliser settings for ASCOT tuners optimized */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef7bw[priv->xtal], 14);
			if (ret)
				return ret;
		}

		/* IF freq setting */
		ret = cxd2841er_get_if_hz(priv, 4150000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;

		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_7_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		data[0] = 0x1A;
		data[1] = 0xFA;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Acquisition optimization setting */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_12);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_SYR_ACTMODE,
			CXD2841ER_SET_ISDB_SYR_ACTMODE_7_8_MHZ,
			CXD2841ER_MASK_ISDB_SYR_ACTMODE);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_15);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_FCS_SYMBOL_INTERVAL,
			CXD2841ER_SET_ISDB_FCS_SYMBOL_INTERVAL_6_7_MHZ);
		if (ret)
			return ret;
		break;
	case 6000000:
		/* TRCG Nominal Rate */
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TC_TIMING_RECOVERY_NOMINAL_RATE,
				nominalRate6bw[priv->xtal], 5);
		if (ret)
			return ret;
		/*  Group delay equaliser settings for ASCOT tuners optimized */
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(priv, I2C_SLVT,
				CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				itbCoef6bw[priv->xtal], 14);
			if (ret)
				return ret;
		}

		/* IF freq setting */
		ret = cxd2841er_get_if_hz(priv, 3550000, &ifhz);
		if (ret)
			return ret;
		iffreq = cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz);
		data[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
		data[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
		data[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
		ret = cxd2841er_write_regs(priv, I2C_SLVT, CXD2841ER_REG_IF_FREQUENCY, data, 3);
		if (ret)
			return ret;

		/* System bandwidth setting */
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_CHANNEL_BANDWIDTH,
			CXD2841ER_SET_CHANNEL_BANDWIDTH_6_MHZ,
			CXD2841ER_MASK_CHANNEL_BANDWIDTH);
		if (ret)
			return ret;

		/* Demod core latency setting */
		if (priv->xtal == SONY_XTAL_24000) {
			data[0] = 0x1F;
			data[1] = 0x79;
		} else {
			data[0] = 0x1A;
			data[1] = 0xE2;
		}
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DEMOD_CORE_LATENCY, data, 2);
		if (ret)
			return ret;

		/* Acquisition optimization setting */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_12);
		if (ret)
			return ret;
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_SYR_ACTMODE,
			CXD2841ER_SET_ISDB_SYR_ACTMODE_6_MHZ,
			CXD2841ER_MASK_ISDB_SYR_ACTMODE);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_15);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_ISDB_FCS_SYMBOL_INTERVAL,
			CXD2841ER_SET_ISDB_FCS_SYMBOL_INTERVAL_6_7_MHZ);
		if (ret)
			return ret;
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid bandwidth %d\n",
				__func__, bandwidth);
		return -EINVAL;
	}
	return 0;
}

static int cxd2841er_sleep_tc_to_active_c_band(struct cxd2841er_priv *priv,
					       u32 bandwidth)
{
	int ret;
	u8 bw7_8mhz_b10_a6[] = {
		0x2D, 0xC7, 0x04, 0xF4, 0x07, 0xC5, 0x2A, 0xB8,
		0x27, 0x9E, 0x27, 0xA4, 0x29, 0xAB };
	u8 bw6mhz_b10_a6[] = {
		0x27, 0xA7, 0x28, 0xB3, 0x02, 0xF0, 0x01, 0xE8,
		0x00, 0xCF, 0x00, 0xE6, 0x23, 0xA4 };
	u8 b10_b6[3];
	u32 iffreq, ifhz;

	if (bandwidth != 6000000 &&
			bandwidth != 7000000 &&
			bandwidth != 8000000) {
		dev_info(&priv->i2c->dev, "%s(): unsupported bandwidth %d. Forcing 8Mhz!\n",
				__func__, bandwidth);
		bandwidth = 8000000;
	}

	dev_dbg(&priv->i2c->dev, "%s() bw=%d\n", __func__, bandwidth);
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	switch (bandwidth) {
	case 8000000:
	case 7000000:
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(
				priv, I2C_SLVT, CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				bw7_8mhz_b10_a6, sizeof(bw7_8mhz_b10_a6));
			if (ret)
				return ret;
		}
		ret = cxd2841er_get_if_hz(priv, 4900000, &ifhz);
		if (ret)
			return ret;
		break;
	case 6000000:
		if (priv->flags & CXD2841ER_ASCOT) {
			ret = cxd2841er_write_regs(
				priv, I2C_SLVT, CXD2841ER_REG_TUNER_GROUP_DELAY_COEFFICIENTS,
				bw6mhz_b10_a6, sizeof(bw6mhz_b10_a6));
			if (ret)
				return ret;
		}
		ret = cxd2841er_get_if_hz(priv, 3700000, &ifhz);
		if (ret)
			return ret;
		break;
	default:
		dev_err(&priv->i2c->dev, "%s(): unsupported bandwidth %d\n",
			__func__, bandwidth);
		return -EINVAL;
	}
	iffreq = priv->cable_24mhz ?
		cxd2841er_calc_iffreq_xtal(priv->xtal, ifhz) :
		cxd2841er_calc_iffreq(ifhz);
	/* <IF freq setting> */
	b10_b6[0] = (u8) ((iffreq >> CXD2841ER_BYTE2_SHIFT) & CXD2841ER_BYTE_MASK);
	b10_b6[1] = (u8)((iffreq >> CXD2841ER_BYTE1_SHIFT) & CXD2841ER_BYTE_MASK);
	b10_b6[2] = (u8)(iffreq & CXD2841ER_BYTE_MASK);
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_IF_FREQUENCY, b10_b6, sizeof(b10_b6));
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x11 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_11);
	if (ret)
		return ret;
	switch (bandwidth) {
	case 8000000:
	case 7000000:
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_IMR_CHANNEL_WIDTH,
			CXD2841ER_SET_DVBC_IMR_CHANNEL_WIDTH_7_8_MHZ,
			CXD2841ER_MASK_DVBC_IMR_CHANNEL_WIDTH);
		if (ret)
			return ret;
		break;
	case 6000000:
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_IMR_CHANNEL_WIDTH,
			CXD2841ER_SET_DVBC_IMR_CHANNEL_WIDTH_6_MHZ,
			CXD2841ER_MASK_DVBC_IMR_CHANNEL_WIDTH);
		if (ret)
			return ret;
		break;
	}
	/* Set SLV-T Bank : 0x40 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	switch (bandwidth) {
	case 8000000:
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_HIGH,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_HIGH_8_MHZ,
			CXD2841ER_MASK_DVBC_TRIAL_SYMBOL_RATE_HIGH);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_LOW,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_LOW_8_MHZ);
		if (ret)
			return ret;
		break;
	case 7000000:
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_HIGH,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_HIGH_7_MHZ,
			CXD2841ER_MASK_DVBC_TRIAL_SYMBOL_RATE_HIGH);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_LOW,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_LOW_7_MHZ);
		if (ret)
			return ret;
		break;
	case 6000000:
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_HIGH,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_HIGH_6_MHZ,
			CXD2841ER_MASK_DVBC_TRIAL_SYMBOL_RATE_HIGH);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_TRIAL_SYMBOL_RATE_LOW,
			CXD2841ER_SET_DVBC_TRIAL_SYMBOL_RATE_LOW_6_MHZ);
		if (ret)
			return ret;
		break;
	}
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t(struct cxd2841er_priv *priv,
					  u32 bandwidth)
{
	int ret;
	u8 data[2] = { 0x09, 0x54 };
	u8 data24m[3] = {0xDC, 0x6C, 0x00};

	if (bandwidth != 5000000 && bandwidth != 6000000 &&
	    bandwidth != 7000000 && bandwidth != 8000000)
		return -EINVAL;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_set_ts_clock_mode(priv, SYS_DVBT);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBT);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_ENABLE);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Enable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Enable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_ACTIVE);
	if (ret)
		return ret;
	/* Enable ADC 2 & 3 */
	if (priv->xtal == SONY_XTAL_41000) {
		data[0] = 0x0A;
		data[1] = 0xD4;
	}
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43, data, 2);
	if (ret)
		return ret;
	/* Enable ADC 4 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_ON);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* IFAGC gain settings */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_IF_AGC_GAIN_CONTROL,
		CXD2841ER_SET_IF_AGC_GAIN_CONTROL_TERRESTRIAL,
		CXD2841ER_MASK_IF_AGC_GAIN_CONTROL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x11 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_11);
	if (ret)
		return ret;
	/* BBAGC TARGET level setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BASEBAND_AGC_TARGET,
		CXD2841ER_SET_BASEBAND_AGC_TARGET_TERRESTRIAL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* ASCOT setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ASCOT_EQUALIZER_ENABLE,
		((priv->flags & CXD2841ER_ASCOT)
			? CXD2841ER_SET_ASCOT_EQUALIZER_ENABLE
			: CXD2841ER_SET_ASCOT_EQUALIZER_DISABLE),
		CXD2841ER_MASK_ASCOT_EQUALIZER_ENABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x18 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_18);
	if (ret)
		return ret;
	/* Pre-RS BER monitor setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_PRE_RS_BER_SETTING,
		CXD2841ER_SET_DVBT_PRE_RS_BER_SETTING_PRESET_40,
		CXD2841ER_MASK_DVBT_PRE_RS_BER_SETTING);
	if (ret)
		return ret;
	/* FEC Auto Recovery setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_FEC_AUTO_RECOVERY_1,
		CXD2841ER_SET_DVBT_FEC_AUTO_RECOVERY_1_ENABLE,
		CXD2841ER_MASK_DVBT_FEC_AUTO_RECOVERY_1);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT_FEC_AUTO_RECOVERY_2,
		CXD2841ER_SET_DVBT_FEC_AUTO_RECOVERY_2_ENABLE,
		CXD2841ER_MASK_DVBT_FEC_AUTO_RECOVERY_2);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* TSIF setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY,
		CXD2841ER_SET_TS_NO_PARITY_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY_MANUAL_ON,
		CXD2841ER_SET_TS_NO_PARITY_MANUAL_ON_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY_MANUAL_ON);
	if (ret)
		return ret;

	if (priv->xtal == SONY_XTAL_24000) {
		/* Set SLV-T Bank : 0x10 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_XTAL_SETUP_10_BF_UNKNOWN,
			CXD2841ER_SET_XTAL_SETUP_10_BF_UNKNOWN_PRESET_60);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_18);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT_24MHZ_SETUP_18_24_UNKNOWN, data24m, 3);
		if (ret)
			return ret;
	}

	ret = cxd2841er_sleep_tc_to_active_t_band(priv, bandwidth);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable HiZ Setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_ACTIVE);
	if (ret)
		return ret;
	/* Disable HiZ Setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_DRIVE);
	if (ret)
		return ret;
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_t2(struct cxd2841er_priv *priv,
					   u32 bandwidth)
{
	int ret;
	u8 data[MAX_WRITE_REGSIZE];

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_set_ts_clock_mode(priv, SYS_DVBT2);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBT2);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_ENABLE);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_CONTROL_59,
		CXD2841ER_SET_RF_LEVEL_MONITOR_CONTROL_59_PRESET_00);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Enable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Enable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_ACTIVE);
	if (ret)
		return ret;

	if (priv->xtal == SONY_XTAL_41000) {
		data[0] = 0x0A;
		data[1] = 0xD4;
	} else {
		data[0] = 0x09;
		data[1] = 0x54;
	}

	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43, data, 2);
	if (ret)
		return ret;
	/* Enable ADC 4 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_ON);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* IFAGC gain settings */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_IF_AGC_GAIN_CONTROL,
		CXD2841ER_SET_IF_AGC_GAIN_CONTROL_TERRESTRIAL,
		CXD2841ER_MASK_IF_AGC_GAIN_CONTROL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x11 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_11);
	if (ret)
		return ret;
	/* BBAGC TARGET level setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BASEBAND_AGC_TARGET,
		CXD2841ER_SET_BASEBAND_AGC_TARGET_TERRESTRIAL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* ASCOT setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ASCOT_EQUALIZER_ENABLE,
		((priv->flags & CXD2841ER_ASCOT)
			? CXD2841ER_SET_ASCOT_EQUALIZER_ENABLE
			: CXD2841ER_SET_ASCOT_EQUALIZER_DISABLE),
		CXD2841ER_MASK_ASCOT_EQUALIZER_ENABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x20 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT2);
	if (ret)
		return ret;
	/* Acquisition optimization setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SEQ_SNR_GOODCNT,
		CXD2841ER_SET_DVBT2_SEQ_SNR_GOODCNT_PRESET_3C);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x2b */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2B);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_NOISE_GAIN_ACQ,
		CXD2841ER_SET_DVBT2_NOISE_GAIN_ACQ_PRESET_20,
		CXD2841ER_MASK_DVBT2_NOISE_GAIN_ACQ);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x23 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_23);
	if (ret)
		return ret;
	/* L1 Control setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_L1_CONTROL,
		CXD2841ER_SET_DVBT2_L1_CONTROL_PRESET_00,
		CXD2841ER_MASK_DVBT2_L1_CONTROL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* TSIF setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY,
		CXD2841ER_SET_TS_NO_PARITY_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY_MANUAL_ON,
		CXD2841ER_SET_TS_NO_PARITY_MANUAL_ON_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY_MANUAL_ON);
	if (ret)
		return ret;
	/* DVB-T2 initial setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_13);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_ISIC_POSPROTECT,
		CXD2841ER_SET_DVBT2_ISIC_POSPROTECT_ACTIVE);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_ISIC_CFNRMOFFSET,
		CXD2841ER_SET_DVBT2_ISIC_CFNRMOFFSET_ACTIVE);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_CFUPONTHRESHOLDMAX_HIGH,
		CXD2841ER_SET_DVBT2_CFUPONTHRESHOLDMAX_HIGH_PRESET_09,
		CXD2841ER_MASK_DVBT2_CFUPONTHRESHOLDMAX_HIGH);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_CFUPONTHRESHOLDMAX_LOW,
		CXD2841ER_SET_DVBT2_CFUPONTHRESHOLDMAX_LOW_ACTIVE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x2a */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2A);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SETUP_2A_38_UNKNOWN,
		CXD2841ER_SET_DVBT2_SETUP_2A_38_UNKNOWN_ACTIVE,
		CXD2841ER_MASK_DVBT2_SETUP_2A_38_UNKNOWN);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x2b */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_2B);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBT2_SETUP_2B_11_UNKNOWN,
		CXD2841ER_SET_DVBT2_SETUP_2B_11_UNKNOWN_ACTIVE,
		CXD2841ER_MASK_DVBT2_SETUP_2B_11_UNKNOWN);
	if (ret)
		return ret;

	/* 24MHz Xtal setting */
	if (priv->xtal == SONY_XTAL_24000) {
		/* Set SLV-T Bank : 0x11 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_11);
		if (ret)
			return ret;
		data[0] = 0xEB;
		data[1] = 0x03;
		data[2] = 0x3B;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_11_33_UNKNOWN, data, 3);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x20 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBT2);
		if (ret)
			return ret;
		data[0] = 0x5E;
		data[1] = 0x5E;
		data[2] = 0x47;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_20_95_UNKNOWN, data, 3);
		if (ret)
			return ret;

		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_20_99_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_20_99_UNKNOWN_PRESET_18);
		if (ret)
			return ret;

		data[0] = 0x3F;
		data[1] = 0xFF;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_20_D9_UNKNOWN, data, 2);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x24 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_24);
		if (ret)
			return ret;
		data[0] = 0x0B;
		data[1] = 0x72;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_24_34_UNKNOWN, data, 2);
		if (ret)
			return ret;

		data[0] = 0x93;
		data[1] = 0xF3;
		data[2] = 0x00;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_24_D2_UNKNOWN, data, 3);
		if (ret)
			return ret;

		data[0] = 0x05;
		data[1] = 0xB8;
		data[2] = 0xD8;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_24_DD_UNKNOWN, data, 3);
		if (ret)
			return ret;

		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_24_E0_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_24_E0_UNKNOWN_PRESET_00);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x25 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_25);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_25_ED_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_25_ED_UNKNOWN_PRESET_60);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x27 */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_27);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_27_FA_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_27_FA_UNKNOWN_PRESET_34);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x2B */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_2B);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_2B_4B_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_2B_4B_UNKNOWN_PRESET_2F);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_2B_9E_UNKNOWN,
			CXD2841ER_SET_DVBT2_24MHZ_SETUP_2B_9E_UNKNOWN_PRESET_0E);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x2D */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_2D);
		if (ret)
			return ret;
		data[0] = 0x89;
		data[1] = 0x89;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_2D_24_UNKNOWN, data, 2);
		if (ret)
			return ret;

		/* Set SLV-T Bank : 0x5E */
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_5E);
		if (ret)
			return ret;
		data[0] = 0x24;
		data[1] = 0x95;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBT2_24MHZ_SETUP_5E_8C_UNKNOWN, data, 2);
		if (ret)
			return ret;
	}

	ret = cxd2841er_sleep_tc_to_active_t2_band(priv, bandwidth);
	if (ret)
		return ret;

	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable HiZ Setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_ACTIVE);
	if (ret)
		return ret;
	/* Disable HiZ Setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_DRIVE);
	if (ret)
		return ret;
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

/* ISDB-Tb part */
static int cxd2841er_sleep_tc_to_active_i(struct cxd2841er_priv *priv,
		u32 bandwidth)
{
	int ret;
	u8 data[2] = { 0x09, 0x54 };
	u8 data24m[2] = {0x60, 0x00};
	u8 data24m2[3] = {0xB7, 0x1B, 0x00};

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_set_ts_clock_mode(priv, SYS_DVBT);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_ISDBT);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_ENABLE);
	if (ret)
		return ret;
	/* Enable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_ENABLE);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_CONTROL_59,
		CXD2841ER_SET_RF_LEVEL_MONITOR_CONTROL_59_PRESET_01);
	if (ret)
		return ret;
	/* Enable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Enable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_ACTIVE);
	if (ret)
		return ret;
	/* xtal freq 20.5MHz or 24M */
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43, data, 2);
	if (ret)
		return ret;
	/* Enable ADC 4 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_ON);
	if (ret)
		return ret;
	/* ASCOT setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_ASCOT_SETTING_AT_BANK_00,
		((priv->flags & CXD2841ER_ASCOT)
			? CXD2841ER_SET_ISDB_ASCOT_SETTING_AT_BANK_00_ENABLE
			: CXD2841ER_SET_ISDB_ASCOT_SETTING_AT_BANK_00_DISABLE),
		CXD2841ER_MASK_ISDB_ASCOT_SETTING_AT_BANK_00);
	if (ret)
		return ret;
	/* FEC Auto Recovery setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_01,
		CXD2841ER_MASK_TC_ADC_CLOCK_CONTROL);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_SAT_ADC_CLOCK_CONTROL_PRESET_00,
		CXD2841ER_MASK_SAT_ADC_CLOCK_CONTROL);
	if (ret)
		return ret;
	/* ISDB-T initial setting */
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY,
		CXD2841ER_SET_TS_NO_PARITY_PRESET_00,
		CXD2841ER_MASK_TS_NO_PARITY);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY_MANUAL_ON,
		CXD2841ER_SET_TS_NO_PARITY_MANUAL_ON_PRESET_00,
		CXD2841ER_MASK_TS_NO_PARITY_MANUAL_ON);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_ENOD_DETSEL,
		CXD2841ER_SET_ISDB_ENOD_DETSEL_PRESET_04,
		CXD2841ER_MASK_ISDB_ENOD_DETSEL);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_COSNE_CRANGE,
		CXD2841ER_SET_ISDB_COSNE_CRANGE_PRESET_03,
		CXD2841ER_MASK_ISDB_COSNE_CRANGE);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_MER_ASNOS,
		CXD2841ER_SET_ISDB_MER_ASNOS_PRESET_50,
		CXD2841ER_MASK_ISDB_MER_ASNOS);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_FORCE_MODE_GI,
		CXD2841ER_SET_ISDB_FORCE_MODE_GI_PRESET_06,
		CXD2841ER_MASK_ISDB_FORCE_MODE_GI);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_SEQ_SNRSEL,
		CXD2841ER_SET_ISDB_SEQ_SNRSEL_PRESET_00,
		CXD2841ER_MASK_ISDB_SEQ_SNRSEL);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_PNC_DISABLE,
		CXD2841ER_SET_ISDB_PNC_DISABLE_PRESET_CE,
		CXD2841ER_MASK_ISDB_PNC_DISABLE);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_TITP_NEF_DISABLE,
		CXD2841ER_SET_ISDB_TITP_NEF_DISABLE_PRESET_13,
		CXD2841ER_MASK_ISDB_TITP_NEF_DISABLE);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_FCS_NTHETA,
		CXD2841ER_SET_ISDB_FCS_NTHETA_PRESET_2E,
		CXD2841ER_MASK_ISDB_FCS_NTHETA);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x15 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_15);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_FCS_F0COEFSEL2,
		CXD2841ER_SET_ISDB_FCS_F0COEFSEL2_PRESET_02,
		CXD2841ER_MASK_ISDB_FCS_F0COEFSEL2);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x1E */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_1E);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_DAGC_BANDWIDTH_AND_GAIN,
		CXD2841ER_SET_ISDB_DAGC_BANDWIDTH_AND_GAIN_PRESET_68,
		CXD2841ER_MASK_ISDB_DAGC_BANDWIDTH_AND_GAIN);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x63 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_63);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_NPC_FORCE_MOBILE_MODE_ENABLE,
		CXD2841ER_SET_ISDB_NPC_FORCE_MOBILE_MODE_ENABLE_PRESET_00,
		CXD2841ER_MASK_ISDB_NPC_FORCE_MOBILE_MODE_ENABLE);
	if (ret)
		return ret;

	/* for xtal 24MHz */
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_XTAL_SETUP_10_BF_UNKNOWN, data24m, 2);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x60 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_ISDBT);
	if (ret)
		return ret;
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_ISDB_XTAL_SETUP_60_A8_UNKNOWN, data24m2, 3);
	if (ret)
		return ret;

	ret = cxd2841er_sleep_tc_to_active_i_band(priv, bandwidth);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable HiZ Setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_ACTIVE);
	if (ret)
		return ret;
	/* Disable HiZ Setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_DRIVE);
	if (ret)
		return ret;
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2841er_sleep_tc_to_active_c(struct cxd2841er_priv *priv,
					  u32 bandwidth)
{
	int ret;
	u8 data[2] = { 0x09, 0x54 };

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_set_ts_clock_mode(priv, SYS_DVBC_ANNEX_A);
	if (ret)
		return ret;
	/* Set SLV-X Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Set demod mode */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_DEMOD_MODE,
		CXD2841ER_SET_DEMOD_MODE_DVBC);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Enable demod clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DEMOD_CLOCK_ENABLE,
		CXD2841ER_SET_DEMOD_CLOCK_ENABLE);
	if (ret)
		return ret;
	/* Disable RF level monitor */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_CONTROL_59,
		CXD2841ER_SET_RF_LEVEL_MONITOR_CONTROL_59_PRESET_00);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_RF_LEVEL_MONITOR_ENABLE,
		CXD2841ER_SET_RF_LEVEL_MONITOR_DISABLE);
	if (ret)
		return ret;
	/* Enable ADC clock */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TC_ADC_CLOCK_CONTROL,
		CXD2841ER_SET_TC_ADC_CLOCK_CONTROL_PRESET_00);
	if (ret)
		return ret;
	/* Enable ADC 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_41,
		CXD2841ER_SET_TERRESTRIAL_ADC_CONTROL_41_ACTIVE);
	if (ret)
		return ret;
	if (priv->xtal == SONY_XTAL_41000) {
		data[0] = 0x0a;
		data[1] = 0xd4;
	}
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_TERRESTRIAL_ADC_CONTROL_43, data, 2);
	if (ret)
		return ret;
	/* Enable ADC 4 */
	ret = cxd2841er_write_reg(priv, I2C_SLVX,
		CXD2841ER_REG_ADC_DISABLE,
		CXD2841ER_SET_ADC_ON);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* IFAGC gain settings */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_IF_AGC_GAIN_CONTROL,
		CXD2841ER_SET_IF_AGC_GAIN_CONTROL_CABLE,
		CXD2841ER_MASK_IF_AGC_GAIN_CONTROL);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x11 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SETUP_11);
	if (ret)
		return ret;
	/* BBAGC TARGET level setting */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BASEBAND_AGC_TARGET,
		CXD2841ER_SET_BASEBAND_AGC_TARGET_CABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x10 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		return ret;
	/* ASCOT setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_ASCOT_EQUALIZER_ENABLE,
		((priv->flags & CXD2841ER_ASCOT)
			? CXD2841ER_SET_ASCOT_EQUALIZER_ENABLE
			: CXD2841ER_SET_ASCOT_EQUALIZER_DISABLE),
		CXD2841ER_MASK_ASCOT_EQUALIZER_ENABLE);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x40 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		return ret;
	/* Demod setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_DVBC_BNDET_EN_64,
		CXD2841ER_SET_DVBC_BNDET_EN_64_PRESET_00,
		CXD2841ER_MASK_DVBC_BNDET_EN_64);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* TSIF setting */
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY,
		CXD2841ER_SET_TS_NO_PARITY_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY);
	if (ret)
		return ret;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_NO_PARITY_MANUAL_ON,
		CXD2841ER_SET_TS_NO_PARITY_MANUAL_ON_PRESET_01,
		CXD2841ER_MASK_TS_NO_PARITY_MANUAL_ON);
	if (ret)
		return ret;

	if (priv->cable_24mhz) {
		const u8 b40_54[] = { 0x29, 0x09 };
		const u8 b40_8b[] = { 0x08, 0x38, 0x83, 0x0e };
		const u8 b48_24[] = { 0xdc, 0x6c };
		const u8 b49_11[] = { 0x77, 0x00 };

		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_DVBC);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_24MHZ_SETUP_40_54_UNKNOWN,
					 b40_54, sizeof(b40_54));
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_24MHZ_SETUP_40_8B_UNKNOWN,
					 b40_8b, sizeof(b40_8b));
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_24MHZ_SETUP_40_BF_UNKNOWN,
			CXD2841ER_SET_DVBC_24MHZ_SETUP_40_BF_UNKNOWN_PRESET_60);
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_48);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_24MHZ_SETUP_48_24_UNKNOWN,
					 b48_24, sizeof(b48_24));
		if (ret)
			return ret;
		ret = cxd2841er_write_reg(priv, I2C_SLVT,
			CXD2841ER_REG_BANK_SELECT,
			CXD2841ER_BANK_SETUP_49);
		if (ret)
			return ret;
		ret = cxd2841er_write_regs(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_24MHZ_SETUP_49_11_UNKNOWN,
					 b49_11, sizeof(b49_11));
		if (ret)
			return ret;
	}

	ret = cxd2841er_sleep_tc_to_active_c_band(priv, bandwidth);
	if (ret)
		return ret;
	/* Set SLV-T Bank : 0x00 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		return ret;
	/* Disable HiZ Setting 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_1,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_1_TC_ACTIVE);
	if (ret)
		return ret;
	/* Disable HiZ Setting 2 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_OUTPUT_HIGH_IMPEDANCE_2,
		CXD2841ER_SET_OUTPUT_HIGH_IMPEDANCE_2_DRIVE);
	if (ret)
		return ret;
	priv->state = STATE_ACTIVE_TC;
	return 0;
}

static int cxd2841er_get_frontend(struct dvb_frontend *fe,
                                  struct dtv_frontend_properties *p)
{
	static const enum fe_modulation qam[] = {
		QAM_16, QAM_32, QAM_64, QAM_128, QAM_256
	};
	int ret = 0, unfreeze_ret;
	u8 arlock, constellation = CXD2841ER_BYTE_MASK;
	enum fe_status status = 0;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	unsigned int stats_mask = cxd2841er_stats_mask;

	if (priv->state == STATE_ACTIVE_S)
		ret = cxd2841er_read_status_s(fe, &status);
	else if (priv->state == STATE_ACTIVE_TC)
		ret = cxd2841er_read_status_tc(fe, &status);
	if (ret) {
		c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		goto unavailable;
	}
	/* Properties can change before DTV_TUNE switches the active decoder. */
	if (priv->system != c->delivery_system &&
	    !(priv->system == SYS_DVBC_ANNEX_A &&
	      c->delivery_system == SYS_DVBC_ANNEX_C)) {
		c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		goto unavailable;
	}
	if ((stats_mask & CXD2841ER_STATS_STRENGTH) &&
	    (priv->state == STATE_ACTIVE_TC || priv->state == STATE_ACTIVE_S))
		cxd2841er_read_signal_strength(fe);
	else
		c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	if (status & FE_HAS_LOCK) {
		if (priv->stats_time && !time_after(jiffies, priv->stats_time))
			goto copy;
		priv->stats_time = jiffies + msecs_to_jiffies(1000);
		/*
		 * Each monitor adds I2C traffic while TS is active. Keep the reads
		 * separate so affected USB bridges can disable troublesome queries.
		 */
		if (stats_mask & CXD2841ER_STATS_CNR)
			cxd2841er_read_snr(fe);
		else
			c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		if (stats_mask & CXD2841ER_STATS_BLOCKS)
			cxd2841er_read_ucblocks(fe);
		else
			c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		if (stats_mask & CXD2841ER_STATS_BITS) {
			cxd2841er_read_ber(fe);
		} else {
			c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
			c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
			c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
			c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		}
		goto copy;
	}
unavailable:
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
copy:
	p->strength = c->strength;
	p->cnr = c->cnr;
	p->block_error = c->block_error;
	p->block_count = c->block_count;
	p->pre_bit_error = c->pre_bit_error;
	p->pre_bit_count = c->pre_bit_count;
	p->post_bit_error = c->post_bit_error;
	p->post_bit_count = c->post_bit_count;
	if (!(stats_mask & CXD2841ER_STATS_QAM) ||
	    !(status & FE_HAS_LOCK) || priv->system != SYS_DVBC_ANNEX_A ||
	    (c->delivery_system != SYS_DVBC_ANNEX_A &&
	     c->delivery_system != SYS_DVBC_ANNEX_C))
		return ret;

	ret = cxd2841er_freeze_regs(priv);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBC);
	if (ret)
		goto unfreeze_qam;
	ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DVBC_ACQUISITION_LOCK, &arlock);
	if (ret)
		goto unfreeze_qam;
	if (arlock & CXD2841ER_DVBC_ACQUISITION_LOCK_BIT)
		ret = cxd2841er_read_reg(priv, I2C_SLVT,
			CXD2841ER_REG_DVBC_DETECTED_QAM, &constellation);
unfreeze_qam:
	unfreeze_ret = cxd2841er_unfreeze_regs(priv);
	if (unfreeze_ret)
		dev_warn_ratelimited(&priv->i2c->dev,
			"%s(): QAM snapshot release failed: ret=%d, read ret=%d\n",
			__func__, unfreeze_ret, ret);
	if (ret)
		return ret;
	if (unfreeze_ret)
		return unfreeze_ret;
	constellation &= CXD2841ER_DVBC_QAM_MASK;
	if (constellation < ARRAY_SIZE(qam))
		p->modulation = qam[constellation];
	return 0;
}

static int cxd2841er_set_frontend_s(struct dvb_frontend *fe)
{
	int ret = 0, i, timeout, carr_offset;
	enum fe_status status = 0;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	u32 symbol_rate = p->symbol_rate/1000;

	cxd2841er_init_stats(fe);

	dev_dbg(&priv->i2c->dev, "%s(): %s frequency=%d symbol_rate=%d xtal=%d\n",
		__func__,
		(p->delivery_system == SYS_DVBS ? "DVB-S" : "DVB-S2"),
		 p->frequency, symbol_rate, priv->xtal);

	if ((p->delivery_system != SYS_DVBS && p->delivery_system != SYS_DVBS2) ||
	    !symbol_rate) {
		ret = -EINVAL;
		goto done;
	}
	/* A failed transition can leave hardware partly asleep despite ACTIVE. */
	if (priv->tune_error || priv->state == STATE_SHUTDOWN) {
		ret = cxd2841er_init_s(fe);
		if (ret)
			goto done;
	}
	priv->tune_error = 0;

	if (priv->flags & CXD2841ER_EARLY_TUNE) {
		ret = cxd2841er_tuner_set(fe);
		if (ret)
			goto done;
	}

	switch (priv->state) {
	case STATE_SLEEP_S:
		ret = cxd2841er_sleep_s_to_active_s(
			priv, p->delivery_system, symbol_rate);
		break;
	case STATE_ACTIVE_S:
		if (priv->system != p->delivery_system) {
			ret = cxd2841er_active_s_to_sleep_s(priv);
			if (!ret)
				ret = cxd2841er_sleep_s_to_active_s(
					priv, p->delivery_system, symbol_rate);
		} else {
			ret = cxd2841er_retune_active(priv, p);
		}
		break;
	default:
		dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		ret = -EINVAL;
		goto done;
	}
	if (ret) {
		dev_dbg(&priv->i2c->dev, "%s(): tune failed\n", __func__);
		goto done;
	}

	if (!(priv->flags & CXD2841ER_EARLY_TUNE)) {
		ret = cxd2841er_tuner_set(fe);
		if (ret)
			goto done;
	}

	ret = cxd2841er_tune_done(priv);
	if (ret)
		goto done;
	timeout = DIV_ROUND_UP(3000000, symbol_rate) + 150;

	i = 0;
	do {
		usleep_range(CXD2841ER_DVBS_POLLING_INVL*1000,
			(CXD2841ER_DVBS_POLLING_INVL + 2) * 1000);
		ret = cxd2841er_read_status_s(fe, &status);
		if (ret)
			goto done;
		if (status & FE_HAS_LOCK)
			break;
		i++;
	} while (i < timeout / CXD2841ER_DVBS_POLLING_INVL);

	if (!(status & FE_HAS_LOCK))
		cxd2841er_warn_tune(priv, "demod lock wait expired", -ETIMEDOUT);

	if (status & FE_HAS_LOCK) {
		ret = cxd2841er_get_carrier_offset_s_s2(priv, &carr_offset);
		if (ret)
			goto done;
		dev_dbg(&priv->i2c->dev, "%s(): carrier_offset=%d\n",
			__func__, carr_offset);
	}
done:
	priv->tune_error = ret;
	cxd2841er_warn_tune(priv, __func__, ret);
	dev_dbg(&priv->i2c->dev,
		"%s(): delsys=%u frequency=%u symbol_rate=%u status=0x%x ret=%d\n",
		__func__, p->delivery_system, p->frequency, p->symbol_rate,
		status, ret);

	return ret;
}

static int cxd2841er_set_frontend_tc(struct dvb_frontend *fe)
{
	int ret = 0, timeout;
	enum fe_status status = 0;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	cxd2841er_init_stats(fe);

	dev_dbg(&priv->i2c->dev, "%s() delivery_system=%d bandwidth_hz=%d\n",
		 __func__, p->delivery_system, p->bandwidth_hz);

	if (p->delivery_system == SYS_DVBT &&
	    p->bandwidth_hz != 5000000 && p->bandwidth_hz != 6000000 &&
	    p->bandwidth_hz != 7000000 && p->bandwidth_hz != 8000000) {
		ret = -EINVAL;
		goto done;
	}
	/* Reinitialize before the active-retune fast path after any failed setup. */
	if (priv->tune_error || priv->state == STATE_SHUTDOWN) {
		ret = cxd2841er_init_tc(fe);
		if (ret)
			goto done;
	}
	priv->tune_error = 0;

	if (priv->flags & CXD2841ER_EARLY_TUNE) {
		ret = cxd2841er_tuner_set(fe);
		if (ret)
			goto done;
	}

	/* deconfigure/put demod to sleep on delsys switch if active */
	if (priv->state == STATE_ACTIVE_TC &&
	    priv->system != p->delivery_system) {
		dev_dbg(&priv->i2c->dev, "%s(): old_delsys=%d, new_delsys=%d -> sleep\n",
			 __func__, priv->system, p->delivery_system);
		ret = cxd2841er_sleep_tc(fe);
		if (ret)
			goto done;
	}

	if (p->delivery_system == SYS_DVBT) {
		priv->system = SYS_DVBT;
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2841er_sleep_tc_to_active_t(
				priv, p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2841er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else if (p->delivery_system == SYS_DVBT2) {
		priv->system = SYS_DVBT2;
		ret = cxd2841er_dvbt2_set_plp_config(priv,
			(int)(p->stream_id > 255), p->stream_id);
		if (ret)
			goto done;
		ret = cxd2841er_dvbt2_set_profile(priv, DVBT2_PROFILE_BASE);
		if (ret)
			goto done;
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2841er_sleep_tc_to_active_t2(priv,
				p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2841er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else if (p->delivery_system == SYS_ISDBT) {
		priv->system = SYS_ISDBT;
		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2841er_sleep_tc_to_active_i(
					priv, p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2841er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
					__func__, priv->state);
			ret = -EINVAL;
		}
	} else if (p->delivery_system == SYS_DVBC_ANNEX_A ||
			p->delivery_system == SYS_DVBC_ANNEX_C) {
		priv->system = SYS_DVBC_ANNEX_A;
		/* correct bandwidth */
		if (p->bandwidth_hz != 6000000 &&
				p->bandwidth_hz != 7000000 &&
				p->bandwidth_hz != 8000000) {
			p->bandwidth_hz = 8000000;
			dev_dbg(&priv->i2c->dev, "%s(): forcing bandwidth to %d\n",
					__func__, p->bandwidth_hz);
		}

		switch (priv->state) {
		case STATE_SLEEP_TC:
			ret = cxd2841er_sleep_tc_to_active_c(
				priv, p->bandwidth_hz);
			break;
		case STATE_ACTIVE_TC:
			ret = cxd2841er_retune_active(priv, p);
			break;
		default:
			dev_dbg(&priv->i2c->dev, "%s(): invalid state %d\n",
				__func__, priv->state);
			ret = -EINVAL;
		}
	} else {
		dev_dbg(&priv->i2c->dev,
			"%s(): invalid delivery system %d\n",
			__func__, p->delivery_system);
		ret = -EINVAL;
	}
	if (ret)
		goto done;

	if (!(priv->flags & CXD2841ER_EARLY_TUNE)) {
		ret = cxd2841er_tuner_set(fe);
		if (ret)
			goto done;
	}

	ret = cxd2841er_tune_done(priv);
	if (ret)
		goto done;

	if (priv->flags & CXD2841ER_NO_WAIT_LOCK)
		goto done;

	timeout = 2500;
	while (timeout > 0) {
		ret = cxd2841er_read_status_tc(fe, &status);
		if (ret)
			goto done;
		if (status & FE_HAS_LOCK)
			break;
		msleep(20);
		timeout -= 20;
	}
	if (!(status & FE_HAS_LOCK))
		cxd2841er_warn_tune(priv, "demod lock wait expired", -ETIMEDOUT);
done:
	priv->tune_error = ret;
	cxd2841er_warn_tune(priv, __func__, ret);
	dev_dbg(&priv->i2c->dev,
		"%s(): delsys=%u frequency=%u bandwidth=%u wait_lock=%u status=0x%x ret=%d\n",
		__func__, p->delivery_system, p->frequency, p->bandwidth_hz,
		!(priv->flags & CXD2841ER_NO_WAIT_LOCK), status, ret);
	return ret;
}

static int cxd2841er_tune_s(struct dvb_frontend *fe,
			    bool re_tune,
			    unsigned int mode_flags,
			    unsigned int *delay,
			    enum fe_status *status)
{
	int ret, carrier_offset;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	*status = 0;
	*delay = HZ / 5;

	dev_dbg(&priv->i2c->dev, "%s() re_tune=%d\n", __func__, re_tune);
	if (re_tune) {
		ret = cxd2841er_set_frontend_s(fe);
		if (ret)
			goto error;
		ret = cxd2841er_read_status_s(fe, status);
		if (ret)
			goto error;
		if (*status & FE_HAS_LOCK) {
			ret = cxd2841er_get_carrier_offset_s_s2(priv, &carrier_offset);
			if (ret)
				goto error;
			p->frequency += carrier_offset;
			ret = cxd2841er_set_frontend_s(fe);
			if (ret)
				goto error;
		}
	}
	ret = cxd2841er_read_status_s(fe, status);
	if (ret)
		goto error;
	return 0;
error:
	*status = 0;
	return ret;
}

static int cxd2841er_tune_tc(struct dvb_frontend *fe,
			     bool re_tune,
			     unsigned int mode_flags,
			     unsigned int *delay,
			     enum fe_status *status)
{
	int ret, carrier_offset;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	*status = 0;
	*delay = HZ / 5;

	dev_dbg(&priv->i2c->dev, "%s(): re_tune %d bandwidth=%d\n", __func__,
			re_tune, p->bandwidth_hz);
	if (re_tune) {
		ret = cxd2841er_set_frontend_tc(fe);
		if (ret)
			goto error;
		ret = cxd2841er_read_status_tc(fe, status);
		if (ret)
			goto error;
		if (*status & FE_HAS_LOCK) {
			switch (priv->system) {
			case SYS_ISDBT:
				ret = cxd2841er_get_carrier_offset_i(
						priv, p->bandwidth_hz,
						&carrier_offset);
				if (ret)
					goto error;
				break;
			case SYS_DVBT:
				ret = cxd2841er_get_carrier_offset_t(
					priv, p->bandwidth_hz,
					&carrier_offset);
				if (ret)
					goto error;
				break;
			case SYS_DVBT2:
				ret = cxd2841er_get_carrier_offset_t2(
					priv, p->bandwidth_hz,
					&carrier_offset);
				if (ret)
					goto error;
				break;
			case SYS_DVBC_ANNEX_A:
				ret = cxd2841er_get_carrier_offset_c(
					priv, &carrier_offset);
				if (ret)
					goto error;
				break;
			default:
				dev_dbg(&priv->i2c->dev,
					"%s(): invalid delivery system %d\n",
					__func__, priv->system);
				ret = -EINVAL;
				goto error;
			}
			dev_dbg(&priv->i2c->dev, "%s(): carrier offset %d\n",
				__func__, carrier_offset);
			p->frequency += carrier_offset;
			ret = cxd2841er_set_frontend_tc(fe);
			if (ret)
				goto error;
		}
	}
	ret = cxd2841er_read_status_tc(fe, status);
	if (ret)
		goto error;
	return 0;
error:
	*status = 0;
	return ret;
}

static int cxd2841er_sleep_s(struct dvb_frontend *fe)
{
	int ret;

	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	if (priv->state == STATE_SHUTDOWN)
		return 0;
	if (priv->state == STATE_ACTIVE_S) {
		ret = cxd2841er_active_s_to_sleep_s(priv);
		if (ret)
			goto error;
	}
	ret = cxd2841er_sleep_s_to_shutdown(priv);
	if (ret)
		goto error;
	return 0;
error:
	priv->tune_error = ret;
	return ret;
}

static int cxd2841er_sleep_tc(struct dvb_frontend *fe)
{
	int ret;

	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	if (priv->state == STATE_ACTIVE_TC) {
		switch (priv->system) {
		case SYS_DVBT:
			ret = cxd2841er_active_t_to_sleep_tc(priv);
			if (ret)
				goto error;
			break;
		case SYS_DVBT2:
			ret = cxd2841er_active_t2_to_sleep_tc(priv);
			if (ret)
				goto error;
			break;
		case SYS_ISDBT:
			ret = cxd2841er_active_i_to_sleep_tc(priv);
			if (ret)
				goto error;
			break;
		case SYS_DVBC_ANNEX_A:
			ret = cxd2841er_active_c_to_sleep_tc(priv);
			if (ret)
				goto error;
			break;
		default:
			dev_warn(&priv->i2c->dev,
				"%s(): unknown delivery system %d\n",
				__func__, priv->system);
		}
	}
	if (priv->state != STATE_SLEEP_TC) {
		dev_err(&priv->i2c->dev, "%s(): invalid state %d\n",
			__func__, priv->state);
		ret = -EINVAL;
		goto error;
	}
	return 0;
error:
	priv->tune_error = ret;
	return ret;
}

static int cxd2841er_shutdown_tc(struct dvb_frontend *fe)
{
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	int ret;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);

	if (priv->state == STATE_SHUTDOWN)
		return 0;
	ret = cxd2841er_sleep_tc(fe);
	if (ret)
		return ret;
	ret = cxd2841er_sleep_tc_to_shutdown(priv);
	if (ret)
		priv->tune_error = ret;
	return ret;
}

static int cxd2841er_send_burst(struct dvb_frontend *fe,
				enum fe_sec_mini_cmd burst)
{
	int ret;

	u8 data;
	struct cxd2841er_priv *priv  = fe->demodulator_priv;

	if (burst != SEC_MINI_A && burst != SEC_MINI_B)
		return -EINVAL;
	dev_dbg(&priv->i2c->dev, "%s(): burst mode %s\n", __func__,
		(burst == SEC_MINI_A ? "A" : "B"));
	if (priv->state != STATE_SLEEP_S &&
			priv->state != STATE_ACTIVE_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	data = (burst == SEC_MINI_A ? CXD2841ER_SET_DISEQC_BURST_A
		: CXD2841ER_SET_DISEQC_BURST_B);
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DISEQC);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_MINI_BURST_ENABLE,
		CXD2841ER_SET_DISEQC_MINI_BURST_ENABLE);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT, CXD2841ER_REG_DISEQC_MINI_BURST_SELECT, data);
	if (ret)
		return ret;
	return 0;
}

static int cxd2841er_set_tone(struct dvb_frontend *fe,
			      enum fe_sec_tone_mode tone)
{
	int ret;

	u8 data;
	struct cxd2841er_priv *priv  = fe->demodulator_priv;

	if (tone != SEC_TONE_ON && tone != SEC_TONE_OFF)
		return -EINVAL;
	dev_dbg(&priv->i2c->dev, "%s(): tone %s\n", __func__,
		(tone == SEC_TONE_ON ? "On" : "Off"));
	if (priv->state != STATE_SLEEP_S &&
			priv->state != STATE_ACTIVE_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	data = (tone == SEC_TONE_ON ? CXD2841ER_SET_DISEQC_TONE_ON
		: CXD2841ER_SET_DISEQC_TONE_OFF);
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DISEQC);
	if (ret)
		return ret;
	ret = cxd2841er_write_reg(priv, I2C_SLVT, CXD2841ER_REG_DISEQC_CONTINUOUS_TONE, data);
	if (ret)
		return ret;
	return 0;
}

static int cxd2841er_send_diseqc_msg(struct dvb_frontend *fe,
				     struct dvb_diseqc_master_cmd *cmd)
{
	int ret;

	int i;
	u8 data[12];
	struct cxd2841er_priv *priv  = fe->demodulator_priv;

	if (cmd->msg_len < 3 || cmd->msg_len > sizeof(cmd->msg))
		return -EINVAL;
	if (priv->state != STATE_SLEEP_S &&
			priv->state != STATE_ACTIVE_S) {
		dev_err(&priv->i2c->dev, "%s(): invalid demod state %d\n",
			__func__, priv->state);
		return -EINVAL;
	}
	dev_dbg(&priv->i2c->dev,
		"%s(): cmd->len %d\n", __func__, cmd->msg_len);
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DISEQC);
	if (ret)
		return ret;
	/* DiDEqC enable */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_ENABLE,
		CXD2841ER_SET_DISEQC_ENABLE);
	if (ret)
		return ret;
	/* cmd1 length & data */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_COMMAND1_LENGTH, cmd->msg_len);
	if (ret)
		return ret;
	memset(data, 0, sizeof(data));
	for (i = 0; i < cmd->msg_len && i < sizeof(data); i++)
		data[i] = cmd->msg[i];
	ret = cxd2841er_write_regs(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_COMMAND1_DATA, data, sizeof(data));
	if (ret)
		return ret;
	/* repeat count for cmd1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_COMMAND1_REPEAT_COUNT,
		CXD2841ER_SET_DISEQC_COMMAND1_REPEAT_COUNT_1);
	if (ret)
		return ret;
	/* repeat count for cmd2: always 0 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_COMMAND2_REPEAT_COUNT,
		CXD2841ER_SET_DISEQC_COMMAND2_REPEAT_COUNT_DISABLE);
	if (ret)
		return ret;
	/* start transmit */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_DISEQC_TX_START,
		CXD2841ER_SET_DISEQC_TX_START_TRIGGER);
	if (ret)
		return ret;
	/* wait for 1 sec timeout */
	for (i = 0; i < 50; i++) {
		ret = cxd2841er_read_reg(priv, I2C_SLVT, CXD2841ER_REG_DISEQC_TX_STATUS, data);
		if (ret)
			return ret;
		if (!data[0]) {
			dev_dbg(&priv->i2c->dev,
				"%s(): DiSEqC cmd has been sent\n", __func__);
			return 0;
		}
		msleep(20);
	}
	dev_dbg(&priv->i2c->dev,
		"%s(): DiSEqC cmd transmit timeout\n", __func__);
	return -ETIMEDOUT;
}

static void cxd2841er_release(struct dvb_frontend *fe)
{
	struct cxd2841er_priv *priv  = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	kfree(priv);
}

static int cxd2841er_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	int ret;

	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s(): enable=%d\n", __func__, enable);
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVX,
		CXD2841ER_REG_TUNER_I2C_GATE,
		(enable
			? CXD2841ER_SET_TUNER_I2C_GATE_OPEN
			: CXD2841ER_SET_TUNER_I2C_GATE_CLOSE),
		CXD2841ER_MASK_TUNER_I2C_GATE);
	if (ret)
		return ret;
	return 0;
}

static enum dvbfe_algo cxd2841er_get_algo(struct dvb_frontend *fe)
{
	struct cxd2841er_priv *priv = fe->demodulator_priv;

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	return DVBFE_ALGO_HW;
}

static void cxd2841er_init_stats(struct dvb_frontend *fe)
{
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;
	struct cxd2841er_priv *priv = fe->demodulator_priv;
	bool pre = p->delivery_system == SYS_DVBT ||
		p->delivery_system == SYS_DVBT2 ||
		p->delivery_system == SYS_DVBS || p->delivery_system == SYS_DVBS2;
	bool cable = p->delivery_system == SYS_DVBC_ANNEX_A ||
		p->delivery_system == SYS_DVBC_ANNEX_B ||
		p->delivery_system == SYS_DVBC_ANNEX_C;
	bool post = cable || p->delivery_system == SYS_ISDBT;

	priv->stats_time = 0;
	memset(&p->strength, 0, sizeof(p->strength));
	p->strength.len = 1;
	p->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->cnr, 0, sizeof(p->cnr));
	p->cnr.len = 1;
	p->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->block_error, 0, sizeof(p->block_error));
	p->block_error.len = cable || p->delivery_system == SYS_DVBT;
	p->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->pre_bit_error, 0, sizeof(p->pre_bit_error));
	p->pre_bit_error.len = pre;
	p->pre_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->pre_bit_count, 0, sizeof(p->pre_bit_count));
	p->pre_bit_count.len = pre;
	p->pre_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->post_bit_error, 0, sizeof(p->post_bit_error));
	p->post_bit_error.len = post;
	p->post_bit_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->post_bit_count, 0, sizeof(p->post_bit_count));
	p->post_bit_count.len = post;
	p->post_bit_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	memset(&p->block_count, 0, sizeof(p->block_count));
}


static int cxd2841er_init_s(struct dvb_frontend *fe)
{
	int ret;

	struct cxd2841er_priv *priv = fe->demodulator_priv;

	cxd2841er_init_stats(fe);

	/* sanity. force demod to SHUTDOWN state */
	if (priv->state == STATE_SLEEP_S) {
		dev_dbg(&priv->i2c->dev, "%s() forcing sleep->shutdown\n",
				__func__);
		ret = cxd2841er_sleep_s_to_shutdown(priv);
		if (ret)
			goto error;
	} else if (priv->state == STATE_ACTIVE_S) {
		dev_dbg(&priv->i2c->dev, "%s() forcing active->sleep->shutdown\n",
				__func__);
		ret = cxd2841er_active_s_to_sleep_s(priv);
		if (ret)
			goto error;
		ret = cxd2841er_sleep_s_to_shutdown(priv);
		if (ret)
			goto error;
	}

	dev_dbg(&priv->i2c->dev, "%s()\n", __func__);
	ret = cxd2841er_shutdown_to_sleep_s(priv);
	if (ret)
		goto error;
	/* SONY_DEMOD_CONFIG_SAT_IFAGCNEG set to 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_SAT_MONITOR);
	if (ret)
		goto error;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_SAT_IF_AGC_POLARITY,
		CXD2841ER_SET_SAT_IF_AGC_POLARITY_PRESET_01,
		CXD2841ER_MASK_SAT_IF_AGC_POLARITY);
	if (ret)
		goto error;

	priv->tune_error = 0;
	return 0;
error:
	priv->tune_error = ret;
	return ret;
}

static int cxd2841er_init_tc(struct dvb_frontend *fe)
{
	int ret;

	struct cxd2841er_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *p = &fe->dtv_property_cache;

	cxd2841er_init_stats(fe);

	dev_dbg(&priv->i2c->dev, "%s() bandwidth_hz=%d\n",
			__func__, p->bandwidth_hz);
	if (priv->state == STATE_ACTIVE_TC || priv->state == STATE_SLEEP_TC) {
		ret = cxd2841er_shutdown_tc(fe);
		if (ret)
			goto error;
	}
	ret = cxd2841er_shutdown_to_sleep_tc(priv);
	if (ret)
		goto error;
	/* SONY_DEMOD_CONFIG_IFAGCNEG = 1 (0 for NO_AGCNEG */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_DVBT);
	if (ret)
		goto error;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_IF_AGC_POLARITY,
		((priv->flags & CXD2841ER_NO_AGCNEG)
			? CXD2841ER_SET_IF_AGC_POLARITY_POSITIVE
			: CXD2841ER_SET_IF_AGC_POLARITY_NEGATIVE),
		CXD2841ER_MASK_IF_AGC_POLARITY);
	if (ret)
		goto error;
	/* SONY_DEMOD_CONFIG_IFAGC_ADC_FS = 0 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_IF_AGC_ADC_FULL_SCALE,
		CXD2841ER_SET_IF_AGC_ADC_FULL_SCALE_PRESET_50);
	if (ret)
		goto error;
	/* SONY_DEMOD_CONFIG_PARALLEL_SEL = 1 */
	ret = cxd2841er_write_reg(priv, I2C_SLVT,
		CXD2841ER_REG_BANK_SELECT,
		CXD2841ER_BANK_COMMON);
	if (ret)
		goto error;
	ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
		CXD2841ER_REG_TS_FORMAT_AND_SERIAL_CLOCK,
		((priv->flags & CXD2841ER_TS_SERIAL)
			? CXD2841ER_SET_TS_FORMAT_AND_SERIAL_CLOCK_SERIAL_OUTPUT
			: CXD2841ER_SET_TS_FORMAT_AND_SERIAL_CLOCK_PARALLEL_OUTPUT),
		CXD2841ER_MASK_TS_SERIAL_OUTPUT);
	if (ret)
		goto error;

	/* clear TSCFG bits 3+4 */
	if (priv->flags & CXD2841ER_TSBITS) {
		ret = cxd2841er_set_reg_bits(priv, I2C_SLVT,
			CXD2841ER_REG_TS_FORMAT_AND_SERIAL_CLOCK,
			CXD2841ER_SET_TSCFG_CLEAR_BITS_3_4,
			CXD2841ER_MASK_TSCFG_BITS_3_4);
		if (ret)
			goto error;
	}

	priv->tune_error = 0;
	return 0;
error:
	priv->tune_error = ret;
	return ret;
}

static const struct dvb_frontend_ops cxd2841er_dvbs_s2_ops;
static const struct dvb_frontend_ops cxd2841er_t_c_ops;

static struct dvb_frontend *cxd2841er_attach(struct cxd2841er_config *cfg,
					     struct i2c_adapter *i2c,
					     u8 system)
{
	u8 chip_id = 0;
	const char *type;
	const char *name;
	struct cxd2841er_priv *priv = NULL;

	/* allocate memory for the internal state */
	priv = kzalloc_obj(struct cxd2841er_priv);
	if (!priv)
		return NULL;
	priv->i2c = i2c;
	priv->config = cfg;
	priv->i2c_addr_slvx = (cfg->i2c_addr + 4) >> 1;
	priv->i2c_addr_slvt = (cfg->i2c_addr) >> 1;
	priv->xtal = cfg->xtal;
	priv->flags = cfg->flags;
	priv->frontend.demodulator_priv = priv;
	dev_dbg(&priv->i2c->dev,
		"%s(): SLVX=0x%02x SLVT=0x%02x xtal=%u flags=0x%x\n",
		__func__, priv->i2c_addr_slvx, priv->i2c_addr_slvt,
		priv->xtal, priv->flags);
	/* Customize our copy; changing the template would affect the next tuner. */
	memcpy(&priv->frontend.ops, &cxd2841er_t_c_ops,
		sizeof(struct dvb_frontend_ops));

	chip_id = cxd2841er_chip_id(priv);
	switch (chip_id) {
	case CXD2837ER_CHIP_ID:
		snprintf(priv->frontend.ops.info.name, 128,
				"Sony CXD2837ER DVB-T/T2/C demodulator");
		name = "CXD2837ER";
		type = "C/T/T2";
		break;
	case CXD2838ER_CHIP_ID:
		snprintf(priv->frontend.ops.info.name, 128,
				"Sony CXD2838ER ISDB-T demodulator");
		priv->frontend.ops.delsys[0] = SYS_ISDBT;
		priv->frontend.ops.delsys[1] = SYS_UNDEFINED;
		priv->frontend.ops.delsys[2] = SYS_UNDEFINED;
		name = "CXD2838ER";
		type = "ISDB-T";
		break;
	case CXD2841ER_CHIP_ID:
		snprintf(priv->frontend.ops.info.name, 128,
				"Sony CXD2841ER DVB-T/T2/C demodulator");
		name = "CXD2841ER";
		type = "T/T2/C/ISDB-T";
		break;
	case CXD2843ER_CHIP_ID:
		snprintf(priv->frontend.ops.info.name, 128,
				"Sony CXD2843ER DVB-T/T2/C/C2 demodulator");
		name = "CXD2843ER";
		type = "C/C2/T/T2";
		break;
	case CXD2854ER_CHIP_ID:
		snprintf(priv->frontend.ops.info.name, 128,
				"Sony CXD2854ER DVB-T/T2/C and ISDB-T demodulator");
		priv->frontend.ops.delsys[3] = SYS_ISDBT;
		name = "CXD2854ER";
		type = "C/C2/T/T2/ISDB-T";
		break;
	default:
		dev_err(&priv->i2c->dev, "%s(): invalid chip ID 0x%02x\n",
				__func__, chip_id);
		priv->frontend.demodulator_priv = NULL;
		kfree(priv);
		return NULL;
	}
	/* The recovered 24 MHz cable recipe covers these IDs, not A7. */
	priv->cable_24mhz = priv->xtal == SONY_XTAL_24000 &&
		(chip_id == CXD2837ER_CHIP_ID || chip_id == CXD2843ER_CHIP_ID ||
		 chip_id == CXD2854ER_CHIP_ID);

	/* create dvb_frontend */
	if (system == SYS_DVBS) {
		memcpy(&priv->frontend.ops,
			&cxd2841er_dvbs_s2_ops,
			sizeof(struct dvb_frontend_ops));
		type = "S/S2";
	}

	dev_info(&priv->i2c->dev,
		"cxd2841er v%s: %s DVB-%s frontend, chip ID 0x%02x\n",
		CXD2841ER_VERSION, name, type, chip_id);
	return &priv->frontend;
}

struct dvb_frontend *cxd2841er_attach_s(struct cxd2841er_config *cfg,
					struct i2c_adapter *i2c)
{
	return cxd2841er_attach(cfg, i2c, SYS_DVBS);
}
EXPORT_SYMBOL_GPL(cxd2841er_attach_s);

struct dvb_frontend *cxd2841er_attach_t_c(struct cxd2841er_config *cfg,
					struct i2c_adapter *i2c)
{
	return cxd2841er_attach(cfg, i2c, 0);
}
EXPORT_SYMBOL_GPL(cxd2841er_attach_t_c);

static const struct dvb_frontend_ops cxd2841er_dvbs_s2_ops = {
	.delsys = { SYS_DVBS, SYS_DVBS2 },
	.info = {
		.name		= "Sony CXD2841ER DVB-S/S2 demodulator",
		.frequency_min_hz	=  500 * MHz,
		.frequency_max_hz	= 2500 * MHz,
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 45000000,
		.symbol_rate_tolerance = 500,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK,
	},
	.init = cxd2841er_init_s,
	.sleep = cxd2841er_sleep_s,
	.release = cxd2841er_release,
	.set_frontend = cxd2841er_set_frontend_s,
	.get_frontend = cxd2841er_get_frontend,
	.read_status = cxd2841er_read_status_s,
	.i2c_gate_ctrl = cxd2841er_i2c_gate_ctrl,
	.get_frontend_algo = cxd2841er_get_algo,
	.set_tone = cxd2841er_set_tone,
	.diseqc_send_burst = cxd2841er_send_burst,
	.diseqc_send_master_cmd = cxd2841er_send_diseqc_msg,
	.tune = cxd2841er_tune_s
};

static const struct dvb_frontend_ops cxd2841er_t_c_ops = {
	.delsys = { SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A },
	.info = {
		.name	= "", /* will set in attach function */
		.caps = FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_QAM_16 |
			FE_CAN_QAM_32 |
			FE_CAN_QAM_64 |
			FE_CAN_QAM_128 |
			FE_CAN_QAM_256 |
			FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_MUTE_TS |
			FE_CAN_2G_MODULATION,
		.frequency_min_hz =   42 * MHz,
		.frequency_max_hz = 1002 * MHz,
		.symbol_rate_min = 870000,
		.symbol_rate_max = 11700000
	},
	.init = cxd2841er_init_tc,
	.sleep = cxd2841er_shutdown_tc,
	.release = cxd2841er_release,
	.set_frontend = cxd2841er_set_frontend_tc,
	.get_frontend = cxd2841er_get_frontend,
	.read_status = cxd2841er_read_status_tc,
	.tune = cxd2841er_tune_tc,
	.i2c_gate_ctrl = cxd2841er_i2c_gate_ctrl,
	.get_frontend_algo = cxd2841er_get_algo
};

MODULE_DESCRIPTION("Sony CXD2837/38/41/43/54ER DVB-C/C2/T/T2/S/S2 demodulator driver");
MODULE_AUTHOR("Sergey Kozlov <serjk@netup.ru>, Abylay Ospan <aospan@netup.ru>");
MODULE_LICENSE("GPL");
MODULE_VERSION(CXD2841ER_VERSION);
