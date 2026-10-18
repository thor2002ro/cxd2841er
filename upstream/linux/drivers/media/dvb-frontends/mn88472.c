// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Panasonic MN88472 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2013 Antti Palosaari <crope@iki.fi>
 */

#include <linux/limits.h>

#include "mn88472_priv.h"

static int mn88472_get_tune_settings(struct dvb_frontend *fe,
				     struct dvb_frontend_tune_settings *s)
{
	s->min_delay_ms = 1000;
	return 0;
}

static int mn88472_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i, stmp;
	unsigned int utmp, utmp1, utmp2;
	u8 buf[5];

	*status = 0;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
		ret = regmap_read(dev->regmap[0], 0x7f, &utmp);
		if (ret)
			goto err;
		/* OFDM or mode-search errors invalidate the acquisition stage. */
		if (!(utmp & 0xa0) && (utmp & 0x0f) >= 0x09)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else
			*status = 0;
		break;
	case SYS_DVBT2:
		ret = regmap_read(dev->regmap[2], 0x92, &utmp);
		if (ret)
			goto err;
		/* P1 acquisition give-up invalidates the stage nibble. */
		if (utmp & 0x40)
			*status = 0;
		else if ((utmp & 0x0f) >= 0x0d)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else if ((utmp & 0x0f) >= 0x0a)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI;
		else if ((utmp & 0x0f) >= 0x07)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		else
			*status = 0;
		break;
	case SYS_DVBC_ANNEX_A:
		/* QAM errors or missing VQLOCK invalidate the acquisition stage. */
		ret = regmap_read(dev->regmap[1], 0x85, &utmp);
		if (ret)
			goto err;
		if (utmp & 0x40)
			break;
		ret = regmap_read(dev->regmap[1], 0x89, &utmp);
		if (ret)
			goto err;
		if (!(utmp & 0x01))
			break;
		ret = regmap_read(dev->regmap[1], 0x84, &utmp);
		if (ret)
			goto err;
		if ((utmp & 0x0f) >= 0x08)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else
			*status = 0;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	/* Signal strength */
	if (*status & FE_HAS_SIGNAL) {
		for (i = 0; i < 2; i++) {
			ret = regmap_bulk_read(dev->regmap[2], 0x8e + i,
					       &buf[i], 1);
			if (ret)
				goto err;
		}

		/* The ten-bit AGC monitor is MSB-aligned across both registers. */
		utmp1 = buf[0] << 8 | (buf[1] & 0xc0) | buf[0] >> 2;
		dev_dbg(&client->dev, "strength=%u\n", utmp1);

		c->strength.stat[0].scale = FE_SCALE_RELATIVE;
		c->strength.stat[0].uvalue = utmp1;
	} else {
		c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	/* CNR */
	if (*status & FE_HAS_VITERBI && c->delivery_system == SYS_DVBT) {
		/* DVB-T CNR */
		ret = regmap_bulk_read(dev->regmap[0], 0x9c, buf, 2);
		if (ret)
			goto err;

		utmp = buf[0] << 8 | buf[1] << 0;
		if (utmp) {
			/* CNR[dB]: 10 * log10(65536 / value) + 2 */
			/* log10(65536) = 80807124, 0.2 = 3355443 */
			stmp = ((u64)80807124 - intlog10(utmp) + 3355443)
			       * 10000 >> 24;

			dev_dbg(&client->dev, "cnr=%d value=%u\n", stmp, utmp);
		} else {
			stmp = 0;
		}

		c->cnr.stat[0].svalue = stmp;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	} else if (*status & FE_HAS_VITERBI &&
		   c->delivery_system == SYS_DVBT2) {
		/* DVB-T2 CNR */
		for (i = 0; i < 3; i++) {
			ret = regmap_bulk_read(dev->regmap[2], 0xbc + i,
					       &buf[i], 1);
			if (ret)
				goto err;
		}

		utmp = buf[1] << 8 | buf[2] << 0;
		utmp1 = (buf[0] >> 2) & 0x01; /* 0=SISO, 1=MISO */
		if (utmp) {
			if (utmp1) {
				/* CNR[dB]: 10 * log10(16384 / value) - 6 */
				/* log10(16384) = 70706234, 0.6 = 10066330 */
				stmp = ((u64)70706234 - intlog10(utmp)
				       - 10066330) * 10000 >> 24;
				dev_dbg(&client->dev, "cnr=%d value=%u MISO\n",
					stmp, utmp);
			} else {
				/* CNR[dB]: 10 * log10(65536 / value) + 2 */
				/* log10(65536) = 80807124, 0.2 = 3355443 */
				stmp = ((u64)80807124 - intlog10(utmp)
				       + 3355443) * 10000 >> 24;

				dev_dbg(&client->dev, "cnr=%d value=%u SISO\n",
					stmp, utmp);
			}
		} else {
			stmp = 0;
		}

		c->cnr.stat[0].svalue = stmp;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	} else if (*status & FE_HAS_VITERBI &&
		   c->delivery_system == SYS_DVBC_ANNEX_A) {
		/* DVB-C CNR */
		ret = regmap_bulk_read(dev->regmap[1], 0xa1, buf, 4);
		if (ret)
			goto err;

		utmp1 = buf[0] << 8 | buf[1] << 0; /* signal */
		utmp2 = buf[2] << 8 | buf[3] << 0; /* noise */
		if (utmp1 && utmp2) {
			/* CNR[dB]: 10 * log10(8 * (signal / noise)) */
			/* log10(8) = 15151336 */
			stmp = ((u64)15151336 + intlog10(utmp1)
			       - intlog10(utmp2)) * 10000 >> 24;

			dev_dbg(&client->dev, "cnr=%d signal=%u noise=%u\n",
				stmp, utmp1, utmp2);
		} else {
			stmp = 0;
		}

		c->cnr.stat[0].svalue = stmp;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	} else {
		c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	/* PER */
	if (*status & FE_HAS_SYNC) {
		ret = regmap_bulk_read(dev->regmap[0], 0xe1, buf, 4);
		if (ret)
			goto err;

		utmp1 = buf[0] << 8 | buf[1] << 0;
		utmp2 = buf[2] << 8 | buf[3] << 0;
		dev_dbg(&client->dev, "block_error=%u block_count=%u\n",
			utmp1, utmp2);

		c->block_error.stat[0].scale = FE_SCALE_COUNTER;
		c->block_error.stat[0].uvalue += utmp1;
		c->block_count.stat[0].scale = FE_SCALE_COUNTER;
		c->block_count.stat[0].uvalue += utmp2;
	} else {
		c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	return 0;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int mn88472_get_frontend(struct dvb_frontend *fe,
				struct dtv_frontend_properties *c)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	static const enum fe_modulation modulation[] = {
		QPSK, QAM_16, QAM_64, QAM_AUTO,
	};
	static const enum fe_modulation cable_modulation[] = {
		QAM_64, QAM_256, QAM_128, QAM_32, QAM_16,
		QAM_AUTO, QAM_AUTO, QAM_AUTO,
	};
	static const enum fe_modulation t2_modulation[] = {
		QPSK, QAM_16, QAM_64, QAM_256,
		QAM_AUTO, QAM_AUTO, QAM_AUTO, QAM_AUTO,
	};
	static const enum fe_hierarchy hierarchy[] = {
		HIERARCHY_NONE, HIERARCHY_1, HIERARCHY_2, HIERARCHY_4,
		HIERARCHY_AUTO, HIERARCHY_AUTO, HIERARCHY_AUTO, HIERARCHY_AUTO,
	};
	static const enum fe_code_rate code_rate[] = {
		FEC_1_2, FEC_2_3, FEC_3_4, FEC_5_6, FEC_7_8,
		FEC_AUTO, FEC_AUTO, FEC_AUTO,
	};
	static const enum fe_code_rate t2_code_rate[] = {
		FEC_1_2, FEC_3_5, FEC_2_3, FEC_3_4,
		FEC_4_5, FEC_5_6, FEC_AUTO, FEC_AUTO,
	};
	static const enum fe_transmit_mode transmission_mode[] = {
		TRANSMISSION_MODE_2K, TRANSMISSION_MODE_8K,
		TRANSMISSION_MODE_4K, TRANSMISSION_MODE_AUTO,
	};
	static const enum fe_transmit_mode t2_transmission_mode[] = {
		TRANSMISSION_MODE_1K, TRANSMISSION_MODE_2K,
		TRANSMISSION_MODE_4K, TRANSMISSION_MODE_8K,
		TRANSMISSION_MODE_16K, TRANSMISSION_MODE_32K,
		TRANSMISSION_MODE_AUTO, TRANSMISSION_MODE_AUTO,
	};
	static const enum fe_guard_interval guard_interval[] = {
		GUARD_INTERVAL_1_32, GUARD_INTERVAL_1_16,
		GUARD_INTERVAL_1_8, GUARD_INTERVAL_1_4,
	};
	static const enum fe_guard_interval t2_guard_interval[] = {
		GUARD_INTERVAL_1_32, GUARD_INTERVAL_1_16,
		GUARD_INTERVAL_1_8, GUARD_INTERVAL_1_4,
		GUARD_INTERVAL_1_128, GUARD_INTERVAL_19_128,
		GUARD_INTERVAL_19_256, GUARD_INTERVAL_AUTO,
	};
	unsigned int utmp, cable_qam;
	u64 symbol_rate;
	u8 buf[3];
	int ret;

	if (!dev->active)
		return 0;

	if (c->delivery_system == SYS_DVBT2) {
		ret = regmap_read(dev->regmap[2], 0x92, &utmp);
		if (ret)
			goto err;
		if ((utmp & 0x40) || (utmp & 0x0f) < 13)
			return 0;
		ret = regmap_read(dev->regmap[2], 0x93, &utmp);
		if (ret)
			goto err;
		buf[0] = utmp;

		/* Preserve C4's upper control bits when selecting the L1 monitor. */
		ret = regmap_update_bits(dev->regmap[2], 0xc4, 0x1f, 0x00);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[2], 0xc5, 0x00);
		if (ret)
			goto err;
		ret = regmap_read(dev->regmap[2], 0xc8, &utmp);
		if (ret)
			goto err;
		buf[1] = utmp;
		ret = regmap_write(dev->regmap[2], 0xc5, 0x03);
		if (ret)
			goto err;
		ret = regmap_read(dev->regmap[2], 0xc7, &utmp);
		if (ret)
			goto err;

		c->transmission_mode = t2_transmission_mode[buf[0] & 0x07];
		c->guard_interval = t2_guard_interval[(buf[1] >> 4) & 0x07];
		c->modulation = t2_modulation[(utmp >> 5) & 0x07];
		c->fec_inner = t2_code_rate[(utmp >> 2) & 0x07];
		return 0;
	}
	if (c->delivery_system == SYS_DVBC_ANNEX_A) {
		ret = regmap_read(dev->regmap[1], 0x85, &utmp);
		if (ret)
			goto err;
		if (utmp & 0x40)
			return 0;
		ret = regmap_read(dev->regmap[1], 0x89, &utmp);
		if (ret)
			goto err;
		if (!(utmp & 0x01))
			return 0;
		ret = regmap_read(dev->regmap[1], 0x84, &utmp);
		if (ret)
			goto err;
		if ((utmp & 0x0f) < 8)
			return 0;
		ret = regmap_read(dev->regmap[1], 0x83, &utmp);
		if (ret)
			goto err;
		cable_qam = utmp & 0x07;
		if (dev->clk == 20500000) {
			ret = regmap_bulk_read(dev->regmap[1], 0x86, buf, 3);
			if (ret)
				goto err;
			utmp = buf[0] << 16 | buf[1] << 8 | buf[2];
			if (utmp) {
				/* This PLL setup uses a 20.5 MHz ADC clock. */
				symbol_rate = DIV_ROUND_CLOSEST_ULL(20500000ULL << 18, utmp);
				if (symbol_rate <= U32_MAX)
					c->symbol_rate = symbol_rate;
			}
		}

		c->modulation = cable_modulation[cable_qam];
		return 0;
	}
	if (c->delivery_system != SYS_DVBT)
		return 0;

	ret = regmap_read(dev->regmap[0], 0x7f, &utmp);
	if (ret)
		goto err;
	if ((utmp & 0xa0) || (utmp & 0x0f) < 9)
		return 0;

	/* Read decoded TPS only; status sampling also accumulates counters. */
	ret = regmap_bulk_read(dev->regmap[0], 0x89, buf, sizeof(buf));
	if (ret)
		goto err;

	c->modulation = modulation[(buf[0] >> 3) & 0x03];
	c->hierarchy = hierarchy[buf[0] & 0x07];
	c->code_rate_HP = code_rate[(buf[1] >> 3) & 0x07];
	c->code_rate_LP = code_rate[buf[1] & 0x07];
	c->transmission_mode = transmission_mode[buf[2] & 0x03];
	c->guard_interval = guard_interval[(buf[2] >> 2) & 0x03];

	return 0;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int mn88472_set_frontend(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, i;
	unsigned int utmp;
	u32 if_frequency;
	u8 buf[3], delivery_system_val, bandwidth_val, *bandwidth_vals_ptr;
	u8 reg_bank0_b4_val, reg_bank0_cd_val, reg_bank0_d4_val;
	u8 reg_bank0_d6_val;
	u8 reg_bank0_00_val, reg_bank0_01_val;
	u8 reg_bank1_00_val;

	dev_dbg(&client->dev,
		"delivery_system=%u modulation=%u frequency=%u bandwidth_hz=%u symbol_rate=%u inversion=%d stream_id=%d\n",
		c->delivery_system, c->modulation, c->frequency,
		c->bandwidth_hz, c->symbol_rate, c->inversion, c->stream_id);

	if (!dev->active) {
		ret = -EAGAIN;
		goto err;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
		delivery_system_val = 0x02;
		reg_bank0_b4_val = 0x00;
		reg_bank0_cd_val = 0x1f;
		reg_bank0_d4_val = 0x0a;
		reg_bank0_d6_val = 0x48;
		break;
	case SYS_DVBT2:
		delivery_system_val = 0x03;
		reg_bank0_b4_val = 0xf6;
		reg_bank0_cd_val = 0x01;
		reg_bank0_d4_val = 0x09;
		reg_bank0_d6_val = 0x46;
		break;
	case SYS_DVBC_ANNEX_A:
		delivery_system_val = 0x04;
		reg_bank0_b4_val = 0x00;
		reg_bank0_cd_val = 0x17;
		reg_bank0_d4_val = 0x09;
		reg_bank0_d6_val = 0x48;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	switch (c->delivery_system) {
	case SYS_DVBT:
	case SYS_DVBT2:
		switch (c->bandwidth_hz) {
		case 5000000:
			bandwidth_vals_ptr = "\xe5\x99\x9a\x1b\xa9\x1b\xa9";
			bandwidth_val = 0x03;
			break;
		case 6000000:
			bandwidth_vals_ptr = "\xbf\x55\x55\x15\x6b\x15\x6b";
			bandwidth_val = 0x02;
			break;
		case 7000000:
			bandwidth_vals_ptr = "\xa4\x00\x00\x0f\x2c\x0f\x2c";
			bandwidth_val = 0x01;
			break;
		case 8000000:
			bandwidth_vals_ptr = "\x8f\x80\x00\x08\xee\x08\xee";
			bandwidth_val = 0x00;
			break;
		default:
			ret = -EINVAL;
			goto err;
		}
		break;
	case SYS_DVBC_ANNEX_A:
		bandwidth_vals_ptr = "\x8f\x80\x00\x08\xee\x08\xee";
		bandwidth_val = 0x00;
		break;
	default:
		break;
	}

	/* Program tuner */
	if (fe->ops.tuner_ops.set_params) {
		ret = fe->ops.tuner_ops.set_params(fe);
		if (ret)
			goto err;
	}

	if (fe->ops.tuner_ops.get_if_frequency) {
		ret = fe->ops.tuner_ops.get_if_frequency(fe, &if_frequency);
		if (ret)
			goto err;

		dev_dbg(&client->dev, "get_if_frequency=%d\n", if_frequency);
	} else {
		ret = -EINVAL;
		goto err;
	}

	ret = regmap_write(dev->regmap[2], 0x00, 0x66);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x01, 0x00);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x02, 0x01);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x03, delivery_system_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x04, bandwidth_val);
	if (ret)
		goto err;

	/* IF */
	utmp = DIV_ROUND_CLOSEST_ULL((u64)if_frequency * 0x1000000, dev->clk);
	buf[0] = (utmp >> 16) & 0xff;
	buf[1] = (utmp >>  8) & 0xff;
	buf[2] = (utmp >>  0) & 0xff;
	for (i = 0; i < 3; i++) {
		ret = regmap_write(dev->regmap[2], 0x10 + i, buf[i]);
		if (ret)
			goto err;
	}

	/* Bandwidth */
	if (bandwidth_vals_ptr) {
		for (i = 0; i < 7; i++) {
			ret = regmap_write(dev->regmap[2], 0x13 + i,
					   bandwidth_vals_ptr[i]);
			if (ret)
				goto err;
		}
	}

	ret = regmap_write(dev->regmap[0], 0x46,
			   (c->delivery_system == SYS_DVBT &&
			    c->bandwidth_hz == 7000000) ? 0x10 : 0x00);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[0], 0xb4, reg_bank0_b4_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[0], 0xcd, reg_bank0_cd_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[0], 0xd4, reg_bank0_d4_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[0], 0xd6, reg_bank0_d6_val);
	if (ret)
		goto err;

	switch (c->delivery_system) {
	case SYS_DVBT:
		if ((c->transmission_mode == TRANSMISSION_MODE_2K ||
		     c->transmission_mode == TRANSMISSION_MODE_8K) &&
		    (unsigned int)c->guard_interval <= GUARD_INTERVAL_1_4) {
			reg_bank0_00_val = 0xf0 |
				(c->transmission_mode == TRANSMISSION_MODE_8K ?
				 0x08 : 0x00) | c->guard_interval;
			reg_bank0_01_val = 0x00;
		} else {
			reg_bank0_00_val = 0xba;
			reg_bank0_01_val = 0x13;
		}
		ret = regmap_write(dev->regmap[0], 0x07, 0x26);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[0], 0x00, reg_bank0_00_val);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[0], 0x01, reg_bank0_01_val);
		if (ret)
			goto err;
		break;
	case SYS_DVBT2:
		ret = regmap_write(dev->regmap[2], 0x2b, 0x13);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[2], 0x4f, 0x05);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[1], 0xf6, 0x05);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[2], 0x32,
				(c->stream_id > 0xff) ? 0 :
				c->stream_id );
		if (ret)
			goto err;
		break;
	case SYS_DVBC_ANNEX_A:
		switch (c->modulation) {
		case QAM_16:
			reg_bank1_00_val = 0x0c;
			break;
		case QAM_32:
			reg_bank1_00_val = 0x0b;
			break;
		case QAM_64:
			reg_bank1_00_val = 0x08;
			break;
		case QAM_128:
			reg_bank1_00_val = 0x0a;
			break;
		case QAM_256:
			reg_bank1_00_val = 0x09;
			break;
		default:
			reg_bank1_00_val = 0x00;
			break;
		}
		/* Replace the whole field so retunes can return to automatic search. */
		ret = regmap_update_bits(dev->regmap[1], 0x00, 0x0f,
					 reg_bank1_00_val);
		if (ret)
			goto err;
		break;
	default:
		break;
	}

	/* Reset FSM */
	ret = regmap_write(dev->regmap[2], 0xf8, 0x9f);
	if (ret)
		goto err;

	return 0;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int mn88472_init(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret, len, rem;
	unsigned int utmp;
	const struct firmware *firmware;
	const char *name = MN88472_FIRMWARE;

	dev_dbg(&client->dev, "\n");

	/* Power up */
	dev->active = false;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	ret = regmap_write(dev->regmap[2], 0x05, 0x00);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x0b, 0x00);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x0c, 0x00);
	if (ret)
		goto err;

	/* Check if firmware is already running */
	ret = regmap_read(dev->regmap[0], 0xf5, &utmp);
	if (ret)
		goto err;
	if (!(utmp & 0x01))
		goto warm;

	ret = request_firmware(&firmware, name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "firmware file '%s' not found\n", name);
		goto err;
	}

	dev_info(&client->dev, "downloading firmware from file '%s'\n", name);

	if (!firmware->size || firmware->size > INT_MAX ||
	    dev->i2c_write_max < 2) {
		ret = -EINVAL;
		goto err_release_firmware;
	}

	ret = regmap_write(dev->regmap[0], 0xf5, 0x03);
	if (ret)
		goto err_release_firmware;

	for (rem = firmware->size; rem > 0; rem -= (dev->i2c_write_max - 1)) {
		len = min(dev->i2c_write_max - 1, rem);
		/* F6 is a fixed program-data port, including split transfers. */
		ret = regmap_noinc_write(dev->regmap[0], 0xf6,
					 &firmware->data[firmware->size - rem],
					 len);
		if (ret) {
			dev_err(&client->dev, "firmware download failed %d\n",
				ret);
			goto err_release_firmware;
		}
	}

	/* Parity check of firmware */
	ret = regmap_read(dev->regmap[0], 0xf8, &utmp);
	if (ret)
		goto err_release_firmware;
	if (utmp & 0x10) {
		ret = -EINVAL;
		dev_err(&client->dev, "firmware parity check failed\n");
		goto err_release_firmware;
	}

	ret = regmap_write(dev->regmap[0], 0xf5, 0x00);
	if (ret)
		goto err_release_firmware;

	release_firmware(firmware);
warm:
	/* TS config */
	switch (dev->ts_mode) {
	case SERIAL_TS_MODE:
		utmp = 0x1d;
		break;
	case PARALLEL_TS_MODE:
		utmp = 0x00;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}
	ret = regmap_write(dev->regmap[2], 0x08, utmp);
	if (ret)
		goto err;

	switch (dev->ts_clk) {
	case VARIABLE_TS_CLOCK:
		utmp = 0xe3;
		break;
	case FIXED_TS_CLOCK:
		utmp = 0xe1;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}
	ret = regmap_write(dev->regmap[0], 0xd9, utmp);
	if (ret)
		goto err;

	dev->active = true;

	return 0;
err_release_firmware:
	release_firmware(firmware);
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static int mn88472_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;

	dev_dbg(&client->dev, "\n");

	/* Power down */
	dev->active = false;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	ret = regmap_write(dev->regmap[2], 0x0c, 0x30);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x0b, 0x30);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[2], 0x05, 0x3e);
	if (ret)
		goto err;

	return 0;
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static const struct dvb_frontend_ops mn88472_ops = {
	.delsys = {SYS_DVBT, SYS_DVBT2, SYS_DVBC_ANNEX_A},
	.info = {
		.name = "Panasonic MN88472",
		.symbol_rate_min = 1000000,
		.symbol_rate_max = 7200000,
		.caps =	FE_CAN_FEC_1_2                 |
			FE_CAN_FEC_2_3                 |
			FE_CAN_FEC_3_4                 |
			FE_CAN_FEC_5_6                 |
			FE_CAN_FEC_7_8                 |
			FE_CAN_FEC_AUTO                |
			FE_CAN_QPSK                    |
			FE_CAN_QAM_16                  |
			FE_CAN_QAM_32                  |
			FE_CAN_QAM_64                  |
			FE_CAN_QAM_128                 |
			FE_CAN_QAM_256                 |
			FE_CAN_QAM_AUTO                |
			FE_CAN_TRANSMISSION_MODE_AUTO  |
			FE_CAN_GUARD_INTERVAL_AUTO     |
			FE_CAN_HIERARCHY_AUTO          |
			FE_CAN_MUTE_TS                 |
			FE_CAN_2G_MODULATION           |
			FE_CAN_MULTISTREAM
	},

	.get_tune_settings = mn88472_get_tune_settings,

	.init = mn88472_init,
	.sleep = mn88472_sleep,

	.set_frontend = mn88472_set_frontend,
	.get_frontend = mn88472_get_frontend,

	.read_status = mn88472_read_status,
};

static struct dvb_frontend *mn88472_get_dvb_frontend(struct i2c_client *client)
{
	struct mn88472_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	return &dev->fe;
}

static int mn88472_probe(struct i2c_client *client)
{
	struct mn88472_config *pdata = client->dev.platform_data;
	struct mn88472_dev *dev;
	struct dtv_frontend_properties *c;
	int ret;
	unsigned int utmp;
	static const struct regmap_config regmap_config = {
		.reg_bits = 8,
		.val_bits = 8,
	};

	dev_dbg(&client->dev, "\n");

	if (!pdata || !pdata->fe || !pdata->xtal || pdata->i2c_wr_max == 1 ||
	    (pdata->ts_mode != SERIAL_TS_MODE &&
	     pdata->ts_mode != PARALLEL_TS_MODE) ||
	    (pdata->ts_clock != FIXED_TS_CLOCK &&
	     pdata->ts_clock != VARIABLE_TS_CLOCK)) {
		ret = -EINVAL;
		goto err;
	}

	dev = kzalloc_obj(*dev);
	if (!dev) {
		ret = -ENOMEM;
		goto err;
	}

	dev->i2c_write_max = pdata->i2c_wr_max ? pdata->i2c_wr_max : ~0;
	dev->clk = pdata->xtal;
	dev->ts_mode = pdata->ts_mode;
	dev->ts_clk = pdata->ts_clock;
	dev->client[0] = client;
	dev->regmap[0] = regmap_init_i2c(dev->client[0], &regmap_config);
	if (IS_ERR(dev->regmap[0])) {
		ret = PTR_ERR(dev->regmap[0]);
		goto err_kfree;
	}

	/*
	 * Chip has three I2C addresses for different register banks. Used
	 * addresses are 0x18, 0x1a and 0x1c. We register two dummy clients,
	 * 0x1a and 0x1c, in order to get own I2C client for each register bank.
	 *
	 * Also, register bank 2 do not support sequential I/O. Only single
	 * register write or read is allowed to that bank.
	 */
	dev->client[1] = i2c_new_dummy_device(client->adapter, 0x1a);
	if (IS_ERR(dev->client[1])) {
		ret = PTR_ERR(dev->client[1]);
		dev_err(&client->dev, "I2C registration failed\n");
		goto err_regmap_0_regmap_exit;
	}
	dev->regmap[1] = regmap_init_i2c(dev->client[1], &regmap_config);
	if (IS_ERR(dev->regmap[1])) {
		ret = PTR_ERR(dev->regmap[1]);
		goto err_client_1_i2c_unregister_device;
	}
	i2c_set_clientdata(dev->client[1], dev);

	dev->client[2] = i2c_new_dummy_device(client->adapter, 0x1c);
	if (IS_ERR(dev->client[2])) {
		ret = PTR_ERR(dev->client[2]);
		dev_err(&client->dev, "2nd I2C registration failed\n");
		goto err_regmap_1_regmap_exit;
	}
	dev->regmap[2] = regmap_init_i2c(dev->client[2], &regmap_config);
	if (IS_ERR(dev->regmap[2])) {
		ret = PTR_ERR(dev->regmap[2]);
		goto err_client_2_i2c_unregister_device;
	}
	i2c_set_clientdata(dev->client[2], dev);

	/* Check demod answers with correct chip id */
	ret = regmap_read(dev->regmap[2], 0xff, &utmp);
	if (ret)
		goto err_regmap_2_regmap_exit;

	dev_dbg(&client->dev, "chip id=%02x\n", utmp);

	if (utmp != 0x02) {
		ret = -ENODEV;
		goto err_regmap_2_regmap_exit;
	}

	/* Sleep because chip is active by default */
	ret = regmap_write(dev->regmap[2], 0x05, 0x3e);
	if (ret)
		goto err_regmap_2_regmap_exit;

	/* Create dvb frontend */
	memcpy(&dev->fe.ops, &mn88472_ops, sizeof(struct dvb_frontend_ops));
	dev->fe.demodulator_priv = client;
	*pdata->fe = &dev->fe;
	i2c_set_clientdata(client, dev);

	/* Init stats to indicate which stats are supported */
	c = &dev->fe.dtv_property_cache;
	c->strength.len = 1;
	c->cnr.len = 1;
	c->block_error.len = 1;
	c->block_count.len = 1;

	/* Setup callbacks */
	pdata->get_dvb_frontend = mn88472_get_dvb_frontend;

	dev_info(&client->dev, "Panasonic MN88472 successfully identified\n");

	return 0;
err_regmap_2_regmap_exit:
	regmap_exit(dev->regmap[2]);
err_client_2_i2c_unregister_device:
	i2c_unregister_device(dev->client[2]);
err_regmap_1_regmap_exit:
	regmap_exit(dev->regmap[1]);
err_client_1_i2c_unregister_device:
	i2c_unregister_device(dev->client[1]);
err_regmap_0_regmap_exit:
	regmap_exit(dev->regmap[0]);
err_kfree:
	kfree(dev);
err:
	dev_dbg(&client->dev, "failed=%d\n", ret);
	return ret;
}

static void mn88472_remove(struct i2c_client *client)
{
	struct mn88472_dev *dev = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "\n");

	regmap_exit(dev->regmap[2]);
	i2c_unregister_device(dev->client[2]);

	regmap_exit(dev->regmap[1]);
	i2c_unregister_device(dev->client[1]);

	regmap_exit(dev->regmap[0]);

	kfree(dev);
}

static const struct i2c_device_id mn88472_id_table[] = {
	{ .name = "mn88472" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mn88472_id_table);

static struct i2c_driver mn88472_driver = {
	.driver = {
		.name = "mn88472",
		.suppress_bind_attrs = true,
	},
	.probe    = mn88472_probe,
	.remove   = mn88472_remove,
	.id_table = mn88472_id_table,
};

module_i2c_driver(mn88472_driver);

MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_DESCRIPTION("Panasonic MN88472 DVB-T/T2/C demodulator driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MN88472_FIRMWARE);
