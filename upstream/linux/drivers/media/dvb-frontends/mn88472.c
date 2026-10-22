// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Panasonic MN88472 DVB-T/T2/C demodulator driver
 *
 * Copyright (C) 2013 Antti Palosaari <crope@iki.fi>
 */

#include <linux/delay.h>
#include <linux/limits.h>
#include <linux/module.h>

#include "mn88472_priv.h"

/*
 * Use the same bits as CXD2841ER so one sysfs setting works for both
 * frontends. Lock status is always read; the mask only controls extra reads.
 */
static unsigned int mn88472_stats_mask = MN88472_STATS_STRENGTH |
					 MN88472_STATS_CNR;
module_param_named(stats_mask, mn88472_stats_mask, uint, 0644);
MODULE_PARM_DESC(stats_mask, "DVB statistics bitmask; status is always read: 0=status only; optional bits: 1=strength, 2=CNR, 4=block errors, 8=bit errors (unsupported), 16=detected DVB-C QAM; combine bits: 19=strength+CNR+QAM, 31=all supported; default=3 (strength+CNR)");

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
	unsigned int stats_mask = mn88472_stats_mask;
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
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT],
				  MN88472_REG_DVBT_ACQUISITION_STATUS, &utmp);
		if (ret)
			goto err;
		/* OFDM or mode-search errors invalidate the acquisition stage. */
		if (!(utmp & MN88472_MASK_DVBT_ACQUISITION_ERROR) &&
		    (utmp & MN88472_MASK_ACQUISITION_STAGE) >= MN88472_STAGE_DVBT_LOCK)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else
			*status = 0;
		break;
	case SYS_DVBT2:
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2],
				  MN88472_REG_DVBT2_ACQUISITION_STATUS, &utmp);
		if (ret)
			goto err;
		/* P1 acquisition give-up invalidates the stage nibble. */
		if (utmp & MN88472_MASK_DVBT2_P1_GIVE_UP)
			*status = 0;
		else if ((utmp & MN88472_MASK_ACQUISITION_STAGE) >= MN88472_STAGE_DVBT2_LOCK)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		else if ((utmp & MN88472_MASK_ACQUISITION_STAGE) >= MN88472_STAGE_DVBT2_FEC)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI;
		else if ((utmp & MN88472_MASK_ACQUISITION_STAGE) >= MN88472_STAGE_DVBT2_CARRIER)
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER;
		else
			*status = 0;
		break;
	case SYS_DVBC_ANNEX_A:
		/* QAM errors or missing VQLOCK invalidate the acquisition stage. */
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC],
				  MN88472_REG_DVBC_ACQUISITION_STATUS, &utmp);
		if (ret)
			goto err;
		if (utmp & MN88472_MASK_DVBC_QAM_ERROR)
			break;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC], MN88472_REG_DVBC_LOCK_STATUS,
				  &utmp);
		if (ret)
			goto err;
		if (!(utmp & MN88472_MASK_DVBC_VQ_LOCK))
			break;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC],
				  MN88472_REG_DVBC_ACQUISITION_STAGE, &utmp);
		if (ret)
			goto err;
		utmp &= MN88472_MASK_ACQUISITION_STAGE;
		if (utmp == MN88472_STAGE_DVBC_LOCK_8 || utmp == MN88472_STAGE_DVBC_LOCK_9)
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
	if ((stats_mask & MN88472_STATS_STRENGTH) &&
	    (*status & FE_HAS_SIGNAL)) {
		for (i = 0; i < 2; i++) {
			ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBT2],
					       MN88472_REG_AGC_HIGH + i, &buf[i], 1);
			if (ret)
				goto err;
		}

		/* The ten-bit AGC monitor is MSB-aligned across both registers. */
		utmp1 = buf[0] << MN88472_SHIFT_MIDDLE_BYTE |
			 (buf[1] & MN88472_MASK_AGC_LOW) | buf[0] >> MN88472_AGC_REPEAT_SHIFT;
		dev_dbg(&client->dev, "strength=%u\n", utmp1);

		c->strength.stat[0].scale = FE_SCALE_RELATIVE;
		c->strength.stat[0].uvalue = utmp1;
	} else {
		c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}

	/* CNR */
	if ((stats_mask & MN88472_STATS_CNR) &&
	    (*status & FE_HAS_VITERBI) && c->delivery_system == SYS_DVBT) {
		/* DVB-T CNR */
		ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_CNR_HIGH,
				       buf, 2);
		if (ret)
			goto err;

		utmp = buf[0] << MN88472_SHIFT_MIDDLE_BYTE | buf[1] << MN88472_SHIFT_LOW_BYTE;
		if (utmp) {
			/* CNR[dB]: 10 * log10(65536 / value) + 2 */
			/* log10(65536) = 80807124, 0.2 = 3355443 */
			stmp = ((u64)MN88472_LOG10_65536_Q24 - intlog10(utmp)
			       + MN88472_CNR_OFFSET_0_2_Q24)
			       * MN88472_CNR_MILLIDB_SCALE >> MN88472_LOG_FRACTION_BITS;

			dev_dbg(&client->dev, "cnr=%d value=%u\n", stmp, utmp);
		} else {
			stmp = 0;
		}

		c->cnr.stat[0].svalue = stmp;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	} else if ((stats_mask & MN88472_STATS_CNR) &&
		   (*status & FE_HAS_VITERBI) &&
		   c->delivery_system == SYS_DVBT2) {
		/* DVB-T2 CNR */
		for (i = 0; i < 3; i++) {
			ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBT2],
					       MN88472_REG_DVBT2_CNR_FLAGS + i, &buf[i], 1);
			if (ret)
				goto err;
		}

		utmp = buf[1] << MN88472_SHIFT_MIDDLE_BYTE | buf[2] << MN88472_SHIFT_LOW_BYTE;
		utmp1 = (buf[0] >> MN88472_CNR_MISO_SHIFT) &
			MN88472_MASK_CNR_MISO; /* 0=SISO, 1=MISO */
		if (utmp) {
			if (utmp1) {
				/* CNR[dB]: 10 * log10(16384 / value) - 6 */
				/* log10(16384) = 70706234, 0.6 = 10066330 */
				stmp = ((u64)MN88472_LOG10_16384_Q24 - intlog10(utmp)
				       - MN88472_CNR_OFFSET_0_6_Q24) * MN88472_CNR_MILLIDB_SCALE
				       >> MN88472_LOG_FRACTION_BITS;
				dev_dbg(&client->dev, "cnr=%d value=%u MISO\n",
					stmp, utmp);
			} else {
				/* CNR[dB]: 10 * log10(65536 / value) + 2 */
				/* log10(65536) = 80807124, 0.2 = 3355443 */
				stmp = ((u64)MN88472_LOG10_65536_Q24 - intlog10(utmp)
				       + MN88472_CNR_OFFSET_0_2_Q24) * MN88472_CNR_MILLIDB_SCALE
				       >> MN88472_LOG_FRACTION_BITS;

				dev_dbg(&client->dev, "cnr=%d value=%u SISO\n",
					stmp, utmp);
			}
		} else {
			stmp = 0;
		}

		c->cnr.stat[0].svalue = stmp;
		c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	} else if ((stats_mask & MN88472_STATS_CNR) &&
		   (*status & FE_HAS_VITERBI) &&
		   c->delivery_system == SYS_DVBC_ANNEX_A) {
		/* DVB-C CNR */
		ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBC],
				       MN88472_REG_DVBC_SIGNAL_NOISE, buf, 4);
		if (ret)
			goto err;

		utmp1 = buf[0] << MN88472_SHIFT_MIDDLE_BYTE |
			 buf[1] << MN88472_SHIFT_LOW_BYTE; /* signal */
		utmp2 = buf[2] << MN88472_SHIFT_MIDDLE_BYTE |
			 buf[3] << MN88472_SHIFT_LOW_BYTE; /* noise */
		if (utmp1 && utmp2) {
			/* CNR[dB]: 10 * log10(8 * (signal / noise)) */
			/* log10(8) = 15151336 */
			stmp = ((u64)MN88472_LOG10_8_Q24 + intlog10(utmp1)
			       - intlog10(utmp2)) * MN88472_CNR_MILLIDB_SCALE
			       >> MN88472_LOG_FRACTION_BITS;

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
	if ((stats_mask & MN88472_STATS_BLOCKS) &&
	    (*status & FE_HAS_SYNC)) {
		ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBT],
				       MN88472_REG_PACKET_ERROR_HIGH, buf, 4);
		if (ret)
			goto err;

		utmp1 = buf[0] << MN88472_SHIFT_MIDDLE_BYTE | buf[1] << MN88472_SHIFT_LOW_BYTE;
		utmp2 = buf[2] << MN88472_SHIFT_MIDDLE_BYTE | buf[3] << MN88472_SHIFT_LOW_BYTE;
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
	unsigned int stats_mask = mn88472_stats_mask;
	u64 symbol_rate;
	u8 buf[3];
	int ret;

	if (!dev->active)
		return 0;

	if (c->delivery_system == SYS_DVBT2) {
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2],
				  MN88472_REG_DVBT2_ACQUISITION_STATUS, &utmp);
		if (ret)
			goto err;
		if ((utmp & MN88472_MASK_DVBT2_P1_GIVE_UP) ||
		    (utmp & MN88472_MASK_ACQUISITION_STAGE) < MN88472_STAGE_DVBT2_LOCK)
			return 0;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_FFT_MODE,
				  &utmp);
		if (ret)
			goto err;
		buf[0] = utmp;

		/* Preserve C4's upper control bits when selecting the L1 monitor. */
		ret = regmap_update_bits(dev->regmap[MN88472_BANK_DVBT2],
					 MN88472_REG_DVBT2_L1_SELECTOR, MN88472_MASK_L1_SELECTOR,
					 MN88472_SET_L1_MONITOR);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_L1_PAGE,
				   MN88472_SET_L1_COMMON_PAGE);
		if (ret)
			goto err;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_L1_DATA_3,
				  &utmp);
		if (ret)
			goto err;
		buf[1] = utmp;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_L1_PAGE,
				   MN88472_SET_L1_PLP_PAGE);
		if (ret)
			goto err;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_L1_DATA_2,
				  &utmp);
		if (ret)
			goto err;

		c->transmission_mode = t2_transmission_mode[buf[0] & MN88472_MASK_THREE_BITS];
		c->guard_interval = t2_guard_interval[(buf[1] >> MN88472_T2_GUARD_SHIFT) &
			MN88472_MASK_THREE_BITS];
		c->modulation = t2_modulation[(utmp >> MN88472_T2_MODULATION_SHIFT) &
			MN88472_MASK_THREE_BITS];
		c->fec_inner = t2_code_rate[(utmp >> MN88472_T2_CODE_RATE_SHIFT) &
			MN88472_MASK_THREE_BITS];
		return 0;
	}
	if (c->delivery_system == SYS_DVBC_ANNEX_A) {
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC],
				  MN88472_REG_DVBC_ACQUISITION_STATUS, &utmp);
		if (ret)
			goto err;
		if (utmp & MN88472_MASK_DVBC_QAM_ERROR)
			return 0;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC], MN88472_REG_DVBC_LOCK_STATUS,
				  &utmp);
		if (ret)
			goto err;
		if (!(utmp & MN88472_MASK_DVBC_VQ_LOCK))
			return 0;
		ret = regmap_read(dev->regmap[MN88472_BANK_DVBC],
				  MN88472_REG_DVBC_ACQUISITION_STAGE, &utmp);
		if (ret)
			goto err;
		utmp &= MN88472_MASK_ACQUISITION_STAGE;
		if (utmp != MN88472_STAGE_DVBC_LOCK_8 && utmp != MN88472_STAGE_DVBC_LOCK_9)
			return 0;
		/* QAM detection is optional, but symbol-rate readback stays available. */
		if (stats_mask & MN88472_STATS_QAM) {
			ret = regmap_read(dev->regmap[MN88472_BANK_DVBC],
					  MN88472_REG_DVBC_QAM_MONITOR, &utmp);
			if (ret)
				goto err;
			cable_qam = utmp & MN88472_MASK_THREE_BITS;
		}
		if (dev->clk == 20500000) {
			ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBC],
					       MN88472_REG_DVBC_SYMBOL_RATE_HIGH, buf, 3);
			if (ret)
				goto err;
			utmp = buf[0] << MN88472_SHIFT_HIGH_BYTE |
				 buf[1] << MN88472_SHIFT_MIDDLE_BYTE | buf[2];
			if (utmp) {
				/* This PLL setup uses a 20.5 MHz ADC clock. */
				symbol_rate = DIV_ROUND_CLOSEST_ULL(
					20500000ULL << MN88472_SYMBOL_RATE_FRACTION_BITS, utmp);
				if (symbol_rate <= U32_MAX)
					c->symbol_rate = symbol_rate;
			}
		}

		if (stats_mask & MN88472_STATS_QAM)
			c->modulation = cable_modulation[cable_qam];
		return 0;
	}
	if (c->delivery_system != SYS_DVBT)
		return 0;

	ret = regmap_read(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_ACQUISITION_STATUS,
			  &utmp);
	if (ret)
		goto err;
	if ((utmp & MN88472_MASK_DVBT_ACQUISITION_ERROR) ||
	    (utmp & MN88472_MASK_ACQUISITION_STAGE) < MN88472_STAGE_DVBT_LOCK)
		return 0;

	/* Read decoded TPS only; status sampling also accumulates counters. */
	ret = regmap_bulk_read(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_TPS, buf,
			       sizeof(buf));
	if (ret)
		goto err;

	c->modulation = modulation[(buf[0] >> MN88472_TPS_MODULATION_SHIFT) &
		MN88472_MASK_TWO_BITS];
	c->hierarchy = hierarchy[buf[0] & MN88472_MASK_THREE_BITS];
	c->code_rate_HP = code_rate[(buf[1] >> MN88472_TPS_HP_CODE_RATE_SHIFT) &
		MN88472_MASK_THREE_BITS];
	c->code_rate_LP = code_rate[buf[1] & MN88472_MASK_THREE_BITS];
	c->transmission_mode = transmission_mode[buf[2] & MN88472_MASK_TWO_BITS];
	c->guard_interval = guard_interval[(buf[2] >> MN88472_TPS_GUARD_SHIFT) &
		MN88472_MASK_TWO_BITS];

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
		delivery_system_val = MN88472_SET_SYSTEM_DVBT;
		reg_bank0_b4_val = MN88472_SET_AGCREF_DVBT_C;
		reg_bank0_cd_val = MN88472_SET_IMPSET_DVBT;
		reg_bank0_d4_val = MN88472_SET_FDAGCSET_DVBT;
		reg_bank0_d6_val = MN88472_SET_TGAINSET_DVBT_C;
		break;
	case SYS_DVBT2:
		delivery_system_val = MN88472_SET_SYSTEM_DVBT2;
		reg_bank0_b4_val = MN88472_SET_AGCREF_DVBT2;
		reg_bank0_cd_val = MN88472_SET_IMPSET_DVBT2;
		reg_bank0_d4_val = MN88472_SET_FDAGCSET_DVBT2_C;
		reg_bank0_d6_val = MN88472_SET_TGAINSET_DVBT2;
		break;
	case SYS_DVBC_ANNEX_A:
		delivery_system_val = MN88472_SET_SYSTEM_DVBC;
		reg_bank0_b4_val = MN88472_SET_AGCREF_DVBT_C;
		reg_bank0_cd_val = MN88472_SET_IMPSET_DVBC;
		reg_bank0_d4_val = MN88472_SET_FDAGCSET_DVBT2_C;
		reg_bank0_d6_val = MN88472_SET_TGAINSET_DVBT_C;
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
			bandwidth_val = MN88472_SET_BANDWIDTH_5MHZ;
			break;
		case 6000000:
			bandwidth_vals_ptr = "\xbf\x55\x55\x15\x6b\x15\x6b";
			bandwidth_val = MN88472_SET_BANDWIDTH_6MHZ;
			break;
		case 7000000:
			bandwidth_vals_ptr = "\xa4\x00\x00\x0f\x2c\x0f\x2c";
			bandwidth_val = MN88472_SET_BANDWIDTH_7MHZ;
			break;
		case 8000000:
			bandwidth_vals_ptr = "\x8f\x80\x00\x08\xee\x08\xee";
			bandwidth_val = MN88472_SET_BANDWIDTH_8MHZ;
			break;
		default:
			ret = -EINVAL;
			goto err;
		}
		break;
	case SYS_DVBC_ANNEX_A:
		bandwidth_vals_ptr = "\x8f\x80\x00\x08\xee\x08\xee";
		bandwidth_val = MN88472_SET_BANDWIDTH_8MHZ;
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

	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_PLL_SETTING_1,
			   MN88472_SET_PLL1_20_5MHZ);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_PLL_SETTING_2,
			   MN88472_SET_PLL2_20_5MHZ);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_PLL_CONTROL,
			   MN88472_SET_PLL_ENABLE);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DELIVERY_SYSTEM,
			   delivery_system_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_BANDWIDTH, bandwidth_val);
	if (ret)
		goto err;

	/* IF */
	utmp = DIV_ROUND_CLOSEST_ULL((u64)if_frequency * MN88472_IF_NCO_SCALE, dev->clk);
	buf[0] = (utmp >> MN88472_SHIFT_HIGH_BYTE) & MN88472_MASK_BYTE;
	buf[1] = (utmp >>  MN88472_SHIFT_MIDDLE_BYTE) & MN88472_MASK_BYTE;
	buf[2] = (utmp >>  MN88472_SHIFT_LOW_BYTE) & MN88472_MASK_BYTE;
	for (i = 0; i < 3; i++) {
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_IF_NCO_HIGH + i,
				   buf[i]);
		if (ret)
			goto err;
	}

	/* Bandwidth */
	if (bandwidth_vals_ptr) {
		for (i = 0; i < 7; i++) {
			ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2],
					   MN88472_REG_BANDWIDTH_COEFFICIENTS + i,
					   bandwidth_vals_ptr[i]);
			if (ret)
				goto err;
		}
	}

	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_FILTER_2,
			   (c->delivery_system == SYS_DVBT &&
			    c->bandwidth_hz == 7000000) ?
			   MN88472_SET_DVBT_FILTER_7MHZ :
			   MN88472_SET_DVBT_FILTER_DEFAULT);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_AGC_REFERENCE,
			   reg_bank0_b4_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_IMPSET, reg_bank0_cd_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FDAGCSET, reg_bank0_d4_val);
	if (ret)
		goto err;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_TGAINSET, reg_bank0_d6_val);
	if (ret)
		goto err;

	switch (c->delivery_system) {
	case SYS_DVBT:
		if ((c->transmission_mode == TRANSMISSION_MODE_2K ||
		     c->transmission_mode == TRANSMISSION_MODE_8K) &&
		    (unsigned int)c->guard_interval <= GUARD_INTERVAL_1_4) {
			reg_bank0_00_val = MN88472_SET_DVBT_MODE_MANUAL |
				(c->transmission_mode == TRANSMISSION_MODE_8K ?
				 MN88472_SET_DVBT_MODE_8K : MN88472_SET_DVBT_MODE_2K) |
				c->guard_interval;
			reg_bank0_01_val = MN88472_SET_DVBT_MODE_SEARCH_OFF;
		} else {
			reg_bank0_00_val = MN88472_SET_DVBT_MODE_AUTO;
			reg_bank0_01_val = MN88472_SET_DVBT_MODE_SEARCH_AUTO;
		}
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT],
				   MN88472_REG_DVBT_SEARCH_TIMEOUT_2,
				   MN88472_SET_DVBT_SEARCH_TIMEOUT);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_MODE,
				   reg_bank0_00_val);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_DVBT_MODE_SEARCH,
				   reg_bank0_01_val);
		if (ret)
			goto err;
		break;
	case SYS_DVBT2:
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_WIDE_AFC_3,
				   MN88472_SET_DVBT2_WAFC);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_DETECTION_2,
				   MN88472_SET_DVBT2_DETECTION);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBC], MN88472_REG_FECSET3,
				   MN88472_SET_DVBT2_FEC3);
		if (ret)
			goto err;
		/* Explicit IDs and the ordinal-zero fallback use different modes. */
		ret = regmap_update_bits(dev->regmap[MN88472_BANK_DVBT2],
					 MN88472_REG_DVBT2_FEC_PLP_MODE, MN88472_MASK_PLP_ORDINAL,
					 c->stream_id > MN88472_PLP_ID_MAX ?
					 MN88472_SET_PLP_ORDINAL :
					 MN88472_SET_PLP_EXPLICIT_ID);
		if (ret)
			goto err;
		ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_DVBT2_PLP_ID,
				   (c->stream_id > MN88472_PLP_ID_MAX) ?
				   MN88472_SET_PLP_FIRST : c->stream_id);
		if (ret)
			goto err;
		break;
	case SYS_DVBC_ANNEX_A:
		switch (c->modulation) {
		case QAM_16:
			reg_bank1_00_val = MN88472_SET_QAM16;
			break;
		case QAM_32:
			reg_bank1_00_val = MN88472_SET_QAM32;
			break;
		case QAM_64:
			reg_bank1_00_val = MN88472_SET_QAM64;
			break;
		case QAM_128:
			reg_bank1_00_val = MN88472_SET_QAM128;
			break;
		case QAM_256:
			reg_bank1_00_val = MN88472_SET_QAM256;
			break;
		default:
			reg_bank1_00_val = MN88472_SET_QAM_AUTO;
			break;
		}
		/* Replace the whole field so retunes can return to automatic search. */
		ret = regmap_update_bits(dev->regmap[MN88472_BANK_DVBC],
					 MN88472_REG_DVBC_QAM_SEARCH, MN88472_MASK_QAM_SEARCH,
					 reg_bank1_00_val);
		if (ret)
			goto err;
		break;
	default:
		break;
	}

	/* Reset the FSM and allow the demodulator to settle before polling it. */
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_RESET_CONTROL,
			   MN88472_SET_RESET_DEMODULATOR);
	if (ret)
		goto err;
	usleep_range(10000, 11000);

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
	const char *stage = "power-on";

	dev_dbg(&client->dev, "initialization started\n");

	/* Power up */
	dev->active = false;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_POWER_CONTROL,
			   MN88472_SET_POWER_ON);
	if (ret)
		goto err;
	stage = "output HIZ1 restore";
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_OUTPUT_HIZ_1,
			   MN88472_SET_OUTPUT1_ACTIVE);
	if (ret)
		goto err;
	stage = "output HIZ2 restore";
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_OUTPUT_HIZ_2,
			   MN88472_SET_OUTPUT2_ACTIVE);
	if (ret)
		goto err;

	/* Check if firmware is already running */
	stage = "firmware status read";
	ret = regmap_read(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FIRMWARE_CONTROL, &utmp);
	if (ret)
		goto err;
	dev_dbg(&client->dev, "firmware control=0x%02x: %s\n", utmp,
		 utmp & MN88472_MASK_FIRMWARE_STOPPED ?
		 "upload required" : "upload skipped (stop bit clear)");
	if (!(utmp & MN88472_MASK_FIRMWARE_STOPPED))
		goto warm;

	stage = "firmware request";
	ret = request_firmware(&firmware, name, &client->dev);
	if (ret) {
		dev_err(&client->dev, "firmware file '%s' not found\n", name);
		goto err;
	}

	dev_info(&client->dev, "downloading firmware from file '%s'\n", name);

	stage = "firmware upload";
	if (!firmware->size || firmware->size > INT_MAX ||
	    dev->i2c_write_max < 2) {
		ret = -EINVAL;
		goto err_release_firmware;
	}

	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FIRMWARE_CONTROL,
			   MN88472_SET_FIRMWARE_LOAD);
	if (ret)
		goto err_release_firmware;

	for (rem = firmware->size; rem > 0; rem -= (dev->i2c_write_max - 1)) {
		len = min(dev->i2c_write_max - 1, rem);
		/* F6 is a fixed program-data port, including split transfers. */
		ret = regmap_noinc_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FIRMWARE_DATA,
					 &firmware->data[firmware->size - rem], len);
		if (ret) {
			dev_err(&client->dev, "firmware download failed %d\n",
				ret);
			goto err_release_firmware;
		}
	}

	/* Parity check of firmware */
	ret = regmap_read(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FIRMWARE_FLAGS, &utmp);
	if (ret)
		goto err_release_firmware;
	if (utmp & MN88472_MASK_FIRMWARE_PARITY_ERROR) {
		ret = -EINVAL;
		dev_err(&client->dev, "firmware parity check failed\n");
		goto err_release_firmware;
	}

	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_FIRMWARE_CONTROL,
			   MN88472_SET_FIRMWARE_RUN);
	if (ret)
		goto err_release_firmware;

	release_firmware(firmware);
warm:
	stage = "TS configuration";
	/* TS config */
	switch (dev->ts_mode) {
	case SERIAL_TS_MODE:
		utmp = MN88472_SET_TS_SERIAL;
		break;
	case PARALLEL_TS_MODE:
		utmp = MN88472_SET_TS_PARALLEL;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_TS_FORMAT, utmp);
	if (ret)
		goto err;

	switch (dev->ts_clk) {
	case VARIABLE_TS_CLOCK:
		utmp = MN88472_SET_TS_CLOCK_VARIABLE;
		break;
	case FIXED_TS_CLOCK:
		utmp = MN88472_SET_TS_CLOCK_FIXED;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT], MN88472_REG_TS_FIFO, utmp);
	if (ret)
		goto err;

	dev->active = true;
	dev_dbg(&client->dev, "initialization completed\n");

	return 0;
err_release_firmware:
	release_firmware(firmware);
err:
	dev_err(&client->dev, "initialization failed at %s: %d\n",
		stage, ret);
	return ret;
}

static int mn88472_power_sleep(struct mn88472_dev *dev)
{
	int ret;

	/* Deep sleep stops I2C and requires a board-controlled hardware reset. */
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_POWER_CONTROL,
			   MN88472_SET_POWER_STANDBY);
	if (ret)
		dev_err(&dev->client[MN88472_BANK_DVBT]->dev,
			"standby power-control write failed: %d\n", ret);

	return ret;
}

static int mn88472_sleep(struct dvb_frontend *fe)
{
	struct i2c_client *client = fe->demodulator_priv;
	struct mn88472_dev *dev = i2c_get_clientdata(client);
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret;
	const char *stage = "output HIZ2";

	dev_dbg(&client->dev, "\n");

	/* Power down */
	dev->active = false;
	c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	c->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_OUTPUT_HIZ_2,
			   MN88472_SET_OUTPUT2_SLEEP);
	if (ret)
		goto err;
	stage = "output HIZ1";
	ret = regmap_write(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_OUTPUT_HIZ_1,
			   MN88472_SET_OUTPUT1_SLEEP);
	if (ret)
		goto err;

	return mn88472_power_sleep(dev);
err:
	dev_err(&client->dev, "standby %s write failed: %d\n",
		stage, ret);
	return ret;
}

static void mn88472_free(struct kref *ref)
{
	struct mn88472_dev *dev = container_of(ref, struct mn88472_dev, refcount);

	kfree(dev);
}

static void mn88472_release(struct dvb_frontend *fe)
{
	struct mn88472_dev *dev = container_of(fe, struct mn88472_dev, fe);

	kref_put(&dev->refcount, mn88472_free);
#ifndef CONFIG_MEDIA_ATTACH
	module_put(THIS_MODULE);
#endif
}

static const struct dvb_frontend_ops mn88472_ops = {
	.release = mn88472_release,
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
	kref_init(&dev->refcount);
	dev->clk = pdata->xtal;
	dev->ts_mode = pdata->ts_mode;
	dev->ts_clk = pdata->ts_clock;
	dev->client[MN88472_BANK_DVBT] = client;
	dev->regmap[MN88472_BANK_DVBT] =
		regmap_init_i2c(dev->client[MN88472_BANK_DVBT],
				&regmap_config);
	if (IS_ERR(dev->regmap[MN88472_BANK_DVBT])) {
		ret = PTR_ERR(dev->regmap[MN88472_BANK_DVBT]);
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
	dev->client[MN88472_BANK_DVBC] =
		i2c_new_dummy_device(client->adapter, MN88472_I2C_ADDR_DVBC);
	if (IS_ERR(dev->client[MN88472_BANK_DVBC])) {
		ret = PTR_ERR(dev->client[MN88472_BANK_DVBC]);
		dev_err(&client->dev, "I2C registration failed\n");
		goto err_regmap_0_regmap_exit;
	}
	dev->regmap[MN88472_BANK_DVBC] =
		regmap_init_i2c(dev->client[MN88472_BANK_DVBC],
				&regmap_config);
	if (IS_ERR(dev->regmap[MN88472_BANK_DVBC])) {
		ret = PTR_ERR(dev->regmap[MN88472_BANK_DVBC]);
		goto err_client_1_i2c_unregister_device;
	}
	i2c_set_clientdata(dev->client[MN88472_BANK_DVBC], dev);

	dev->client[MN88472_BANK_DVBT2] =
		i2c_new_dummy_device(client->adapter, MN88472_I2C_ADDR_DVBT2);
	if (IS_ERR(dev->client[MN88472_BANK_DVBT2])) {
		ret = PTR_ERR(dev->client[MN88472_BANK_DVBT2]);
		dev_err(&client->dev, "2nd I2C registration failed\n");
		goto err_regmap_1_regmap_exit;
	}
	dev->regmap[MN88472_BANK_DVBT2] =
		regmap_init_i2c(dev->client[MN88472_BANK_DVBT2],
				&regmap_config);
	if (IS_ERR(dev->regmap[MN88472_BANK_DVBT2])) {
		ret = PTR_ERR(dev->regmap[MN88472_BANK_DVBT2]);
		goto err_client_2_i2c_unregister_device;
	}
	i2c_set_clientdata(dev->client[MN88472_BANK_DVBT2], dev);

	/* Check demod answers with correct chip id */
	ret = regmap_read(dev->regmap[MN88472_BANK_DVBT2], MN88472_REG_CHIP_ID, &utmp);
	if (ret)
		goto err_regmap_2_regmap_exit;

	dev_dbg(&client->dev, "chip id=%02x\n", utmp);

	if (utmp != MN88472_CHIP_ID) {
		ret = -ENODEV;
		goto err_regmap_2_regmap_exit;
	}

	/* Standby because chip is active by default. */
	ret = mn88472_power_sleep(dev);
	if (ret)
		goto err_regmap_2_regmap_exit;

	/* Create dvb frontend */
	memcpy(&dev->fe.ops, &mn88472_ops, sizeof(struct dvb_frontend_ops));
	/* The frontend can remain open after its I2C device is removed. */
	kref_get(&dev->refcount);
	/* dvb_frontend_invoke_release() drops this reference with MEDIA_ATTACH. */
	__module_get(THIS_MODULE);
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
	regmap_exit(dev->regmap[MN88472_BANK_DVBT2]);
err_client_2_i2c_unregister_device:
	i2c_unregister_device(dev->client[MN88472_BANK_DVBT2]);
err_regmap_1_regmap_exit:
	regmap_exit(dev->regmap[MN88472_BANK_DVBC]);
err_client_1_i2c_unregister_device:
	i2c_unregister_device(dev->client[MN88472_BANK_DVBC]);
err_regmap_0_regmap_exit:
	regmap_exit(dev->regmap[MN88472_BANK_DVBT]);
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

	regmap_exit(dev->regmap[MN88472_BANK_DVBT2]);
	i2c_unregister_device(dev->client[MN88472_BANK_DVBT2]);

	regmap_exit(dev->regmap[MN88472_BANK_DVBC]);
	i2c_unregister_device(dev->client[MN88472_BANK_DVBC]);

	regmap_exit(dev->regmap[MN88472_BANK_DVBT]);

	i2c_set_clientdata(client, NULL);
	kref_put(&dev->refcount, mn88472_free);
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

MODULE_VERSION("1.0.1");
MODULE_AUTHOR("Antti Palosaari <crope@iki.fi>");
MODULE_DESCRIPTION("Panasonic MN88472 DVB-T/T2/C demodulator driver");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MN88472_FIRMWARE);
