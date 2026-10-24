/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Panasonic MN88473 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2014 Antti Palosaari <crope@iki.fi>
 */

#ifndef MN88473_PRIV_H
#define MN88473_PRIV_H

#include <media/dvb_frontend.h>
#include <linux/int_log.h>
#include "mn88473.h"
#include <linux/math64.h>
#include <linux/firmware.h>
#include <linux/regmap.h>

#define MN88473_FIRMWARE "dvb-demod-mn88473-01.fw"

/* Shared statistics mask assignments with CXD2841ER and MN88472. */
#define MN88473_STATS_STRENGTH	0x01
#define MN88473_STATS_CNR	0x02
#define MN88473_STATS_BLOCKS	0x04
#define MN88473_STATS_BITS	0x08
/* Reserved: this driver does not expose detected DVB-C QAM. */
#define MN88473_STATS_QAM	0x10
#define MN88473_STATS_ALL	0x1f


struct mn88473_dev {
	struct i2c_client *client[3];
	struct regmap *regmap[3];
	struct dvb_frontend frontend;
	u16 i2c_wr_max;
	bool active;
	bool initialized;
	u32 clk;
};

#endif
