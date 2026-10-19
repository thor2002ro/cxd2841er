/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Panasonic MN88472 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2013 Antti Palosaari <crope@iki.fi>
 */

#ifndef MN88472_PRIV_H
#define MN88472_PRIV_H

#include <media/dvb_frontend.h>
#include <linux/int_log.h>
#include "mn88472.h"
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/kref.h>

#define MN88472_FIRMWARE "dvb-demod-mn88472-02.fw"

/* Shared mask values let both demodulator drivers use the same controls. */
#define MN88472_STATS_STRENGTH		0x01
#define MN88472_STATS_CNR		0x02
#define MN88472_STATS_BLOCKS		0x04
#define MN88472_STATS_BITS		0x08
#define MN88472_STATS_QAM		0x10
#define MN88472_STATS_ALL		0x1f

struct mn88472_dev {
	struct i2c_client *client[3];
	struct regmap *regmap[3];
	struct dvb_frontend fe;
	u16 i2c_write_max;
	unsigned int clk;
	unsigned int active:1;
	unsigned int ts_mode:1;
	unsigned int ts_clk:1;
	struct kref refcount;
};

#endif
