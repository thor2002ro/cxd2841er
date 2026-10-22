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
#define MN88472_STATS_STRENGTH					0x01
#define MN88472_STATS_CNR					0x02
#define MN88472_STATS_BLOCKS					0x04
#define MN88472_STATS_BITS					0x08
#define MN88472_STATS_QAM					0x10
#define MN88472_STATS_ALL					0x1f

/*
 * Register symbols follow Panasonic's MN88472.h (HiSilicon SDK mirror,
 * fd20f78ab02934e71474dbb1d933c6ec911b01c9). Bank indices are Linux regmap
 * indices, not I2C addresses. Recipe values below preserve this driver's
 * settings; a vendor symbol alone does not document its individual fields.
 */
/* DVB-T bank (I2C 0x18) */
/* MDSET_T: DVB-T mode and guard selection */
#define MN88472_REG_DVBT_MODE                        0x00
/* MDASET_T: DVB-T automatic mode search */
#define MN88472_REG_DVBT_MODE_SEARCH                 0x01
/* STTOSET2_T: DVB-T search timeout setting 2 */
#define MN88472_REG_DVBT_SEARCH_TIMEOUT_2            0x07
/* FFILSET2_T: DVB-T filter setting 2 */
#define MN88472_REG_DVBT_FILTER_2                    0x46
/* SSEQRD_T: DVB-T acquisition stage and errors */
#define MN88472_REG_DVBT_ACQUISITION_STATUS          0x7f
/* TMCCD2_T: Decoded DVB-T TPS, first of three bytes */
#define MN88472_REG_DVBT_TPS                         0x89
/* CNRDU_T: DVB-T CNR monitor, high byte */
#define MN88472_REG_DVBT_CNR_HIGH                    0x9c
/* AGCREF: AGC reference */
#define MN88472_REG_AGC_REFERENCE                    0xb4
/* IMP setting; individual fields undocumented here */
#define MN88472_REG_IMPSET                           0xcd
/* FDAGC setting; individual fields undocumented here */
#define MN88472_REG_FDAGCSET                         0xd4
/* T gain setting; individual fields undocumented here */
#define MN88472_REG_TGAINSET                         0xd6
/* FIFOSET: TS FIFO and clock setting */
#define MN88472_REG_TS_FIFO                          0xd9
/* PERRDU: Packet error count, followed by packet count */
#define MN88472_REG_PACKET_ERROR_HIGH                0xe1
/* PSEQSET: Program sequencer control */
#define MN88472_REG_FIRMWARE_CONTROL                 0xf5
/* PSEQPRG: Program sequencer data port (no address increment) */
#define MN88472_REG_FIRMWARE_DATA                    0xf6
/* PSEQFLG: Program sequencer flags */
#define MN88472_REG_FIRMWARE_FLAGS                   0xf8
/* DVB-C bank (I2C 0x1a) */
/* CHSRCHSET_C: DVB-C QAM search selection */
#define MN88472_REG_DVBC_QAM_SEARCH                  0x00
/* SSEQMON1_C: Detected DVB-C QAM */
#define MN88472_REG_DVBC_QAM_MONITOR                 0x83
/* SSEQMON2_C: DVB-C acquisition stage */
#define MN88472_REG_DVBC_ACQUISITION_STAGE           0x84
/* STSMON_C: DVB-C acquisition errors */
#define MN88472_REG_DVBC_ACQUISITION_STATUS          0x85
/* RATERDU_C: DVB-C symbol-rate monitor, high byte */
#define MN88472_REG_DVBC_SYMBOL_RATE_HIGH            0x86
/* DMDSTSMON1_C: DVB-C demodulator lock flags */
#define MN88472_REG_DVBC_LOCK_STATUS                 0x89
/* CNMON1_C: DVB-C signal and noise monitors */
#define MN88472_REG_DVBC_SIGNAL_NOISE                0xa1
/* FEC setting 3 */
#define MN88472_REG_FECSET3                          0xf6
/* DVB-T2/common bank (I2C 0x1c) */
/* PLLSET1: PLL setting 1 */
#define MN88472_REG_PLL_SETTING_1                    0x00
/* PLLSET2: PLL setting 2 */
#define MN88472_REG_PLL_SETTING_2                    0x01
/* PLLSET: PLL control */
#define MN88472_REG_PLL_CONTROL                      0x02
/* DTVSET: Delivery system selection */
#define MN88472_REG_DELIVERY_SYSTEM                  0x03
/* SYSSET: Channel bandwidth selection */
#define MN88472_REG_BANDWIDTH                        0x04
/* PWDSET: Power control */
#define MN88472_REG_POWER_CONTROL                    0x05
/* TSSET1: TS output format */
#define MN88472_REG_TS_FORMAT                        0x08
/* HIZSET1: Output high-impedance setting 1 */
#define MN88472_REG_OUTPUT_HIZ_1                     0x0b
/* HIZSET2: Output high-impedance setting 2 */
#define MN88472_REG_OUTPUT_HIZ_2                     0x0c
/* NCOFREQU: IF NCO frequency, high byte */
#define MN88472_REG_IF_NCO_HIGH                      0x10
/* FADU: Bandwidth coefficients: FAD, FSFTX and FSFTY */
#define MN88472_REG_BANDWIDTH_COEFFICIENTS           0x13
/* WAFCSET3: Wide AFC setting 3 */
#define MN88472_REG_DVBT2_WIDE_AFC_3                 0x2b
/* FECSET1: FEC and PLP selection mode */
#define MN88472_REG_DVBT2_FEC_PLP_MODE               0x30
/* PLPID: PLP identifier or ordinal */
#define MN88472_REG_DVBT2_PLP_ID                     0x32
/* DETSET2: Detection setting 2 */
#define MN88472_REG_DVBT2_DETECTION_2                0x4f
/* AGCRDU: AGC monitor, high byte */
#define MN88472_REG_AGC_HIGH                         0x8e
/* SSEQFLG: DVB-T2 acquisition stage and flags */
#define MN88472_REG_DVBT2_ACQUISITION_STATUS         0x92
/* SSEQRD1: DVB-T2 FFT mode readback */
#define MN88472_REG_DVBT2_FFT_MODE                   0x93
/* CNFLG: DVB-T2 CNR flags, followed by CNR value */
#define MN88472_REG_DVBT2_CNR_FLAGS                  0xbc
/* TPDSET1: L1 monitor selector 1 */
#define MN88472_REG_DVBT2_L1_SELECTOR                0xc4
/* TPDSET2: L1 monitor selector 2 */
#define MN88472_REG_DVBT2_L1_PAGE                    0xc5
/* TPD2: L1 monitor data 2 */
#define MN88472_REG_DVBT2_L1_DATA_2                  0xc7
/* TPD3: L1 monitor data 3 */
#define MN88472_REG_DVBT2_L1_DATA_3                  0xc8
/* RSTSET1: Demodulator reset control */
#define MN88472_REG_RESET_CONTROL                    0xf8
/* CHIPRD: Chip identifier */
#define MN88472_REG_CHIP_ID                          0xff

#define MN88472_SET_PLL1_20_5MHZ			0x66
#define MN88472_SET_PLL2_20_5MHZ			0x00
#define MN88472_SET_PLL_ENABLE				0x01
#define MN88472_SET_POWER_ON				0x00
/* Keep I2C/clock running: vendor sleep 0x3f requires a hardware reset. */
#define MN88472_SET_POWER_STANDBY				0x3e
#define MN88472_SET_OUTPUT1_ACTIVE			0x00
#define MN88472_SET_OUTPUT1_SLEEP			0x30
#define MN88472_SET_OUTPUT2_ACTIVE			0x00
#define MN88472_SET_OUTPUT2_SLEEP			0x30
#define MN88472_SET_FIRMWARE_LOAD			0x03
#define MN88472_SET_FIRMWARE_RUN			0x00
#define MN88472_SET_DVBT_SEARCH_TIMEOUT			0x26
#define MN88472_SET_DVBT2_WAFC				0x13
#define MN88472_SET_DVBT2_DETECTION			0x05
#define MN88472_SET_DVBT2_FEC3				0x05
#define MN88472_SET_RESET_DEMODULATOR			0x9f
#define MN88472_SET_L1_COMMON_PAGE			0x00
#define MN88472_SET_L1_PLP_PAGE				0x03
#define MN88472_MASK_L1_SELECTOR			0x1f
#define MN88472_SET_L1_MONITOR				0x00
#define MN88472_MASK_QAM_SEARCH				0x0f
#define MN88472_MASK_PLP_ORDINAL			0x80
#define MN88472_PLP_ID_MAX				0xff
#define MN88472_SET_PLP_ORDINAL				0x80
#define MN88472_SET_PLP_EXPLICIT_ID			0x00
#define MN88472_SET_PLP_FIRST				0
#define MN88472_SET_DVBT_FILTER_7MHZ			0x10
#define MN88472_SET_DVBT_FILTER_DEFAULT			0x00
#define MN88472_SET_SYSTEM_DVBT				0x02
#define MN88472_SET_SYSTEM_DVBT2			0x03
#define MN88472_SET_SYSTEM_DVBC				0x04
#define MN88472_SET_BANDWIDTH_5MHZ			0x03
#define MN88472_SET_BANDWIDTH_6MHZ			0x02
#define MN88472_SET_BANDWIDTH_7MHZ			0x01
#define MN88472_SET_BANDWIDTH_8MHZ			0x00
#define MN88472_SET_AGCREF_DVBT_C			0x00
#define MN88472_SET_AGCREF_DVBT2			0xf6
#define MN88472_SET_IMPSET_DVBT				0x1f
#define MN88472_SET_IMPSET_DVBT2			0x01
#define MN88472_SET_IMPSET_DVBC				0x17
#define MN88472_SET_FDAGCSET_DVBT			0x0a
#define MN88472_SET_FDAGCSET_DVBT2_C			0x09
#define MN88472_SET_TGAINSET_DVBT_C			0x48
#define MN88472_SET_TGAINSET_DVBT2			0x46
#define MN88472_SET_DVBT_MODE_MANUAL			0xf0
#define MN88472_SET_DVBT_MODE_AUTO			0xba
#define MN88472_SET_DVBT_MODE_SEARCH_OFF		0x00
#define MN88472_SET_DVBT_MODE_SEARCH_AUTO		0x13
#define MN88472_SET_QAM16				0x0c
#define MN88472_SET_QAM32				0x0b
#define MN88472_SET_QAM64				0x08
#define MN88472_SET_QAM128				0x0a
#define MN88472_SET_QAM256				0x09
#define MN88472_SET_QAM_AUTO				0x00
#define MN88472_SET_DVBT_MODE_8K			0x08
#define MN88472_SET_DVBT_MODE_2K			0x00
#define MN88472_SET_TS_SERIAL				0x1d
#define MN88472_SET_TS_PARALLEL				0x00
#define MN88472_SET_TS_CLOCK_VARIABLE			0xe3
#define MN88472_SET_TS_CLOCK_FIXED			0xe1
#define MN88472_MASK_DVBT_ACQUISITION_ERROR		0xa0
#define MN88472_MASK_ACQUISITION_STAGE			0x0f
#define MN88472_STAGE_DVBT_LOCK				0x09
#define MN88472_STAGE_DVBT2_LOCK			0x0d
#define MN88472_STAGE_DVBT2_FEC				0x0a
#define MN88472_STAGE_DVBT2_CARRIER			0x07
#define MN88472_STAGE_DVBC_LOCK_8			0x08
#define MN88472_STAGE_DVBC_LOCK_9			0x09
#define MN88472_MASK_AGC_LOW				0xc0
#define MN88472_MASK_FIRMWARE_PARITY_ERROR		0x10
#define MN88472_CHIP_ID					0x02
#define MN88472_IF_NCO_SCALE				0x1000000
#define MN88472_MASK_FIRMWARE_STOPPED			0x01
#define MN88472_MASK_DVBC_VQ_LOCK			0x01
#define MN88472_MASK_DVBT2_P1_GIVE_UP			0x40
#define MN88472_MASK_DVBC_QAM_ERROR			0x40
#define MN88472_AGC_REPEAT_SHIFT			2
#define MN88472_CNR_MISO_SHIFT				2
#define MN88472_MASK_CNR_MISO				0x01
#define MN88472_MASK_THREE_BITS				0x07
#define MN88472_T2_GUARD_SHIFT				4
#define MN88472_T2_MODULATION_SHIFT			5
#define MN88472_T2_CODE_RATE_SHIFT			2
#define MN88472_TPS_MODULATION_SHIFT			3
#define MN88472_TPS_HP_CODE_RATE_SHIFT			3
#define MN88472_TPS_GUARD_SHIFT				2
#define MN88472_MASK_TWO_BITS				0x03
#define MN88472_MASK_BYTE				0xff
#define MN88472_SHIFT_HIGH_BYTE				16
#define MN88472_SHIFT_MIDDLE_BYTE			8
#define MN88472_SHIFT_LOW_BYTE				0
#define MN88472_LOG10_65536_Q24				80807124
#define MN88472_CNR_OFFSET_0_2_Q24			3355443
#define MN88472_LOG10_16384_Q24				70706234
#define MN88472_CNR_OFFSET_0_6_Q24			10066330
#define MN88472_LOG10_8_Q24				15151336
#define MN88472_CNR_MILLIDB_SCALE			10000
#define MN88472_LOG_FRACTION_BITS			24
#define MN88472_SYMBOL_RATE_FRACTION_BITS		18
#define MN88472_BANK_DVBT				0
#define MN88472_BANK_DVBC				1
#define MN88472_BANK_DVBT2				2
#define MN88472_I2C_ADDR_DVBC				0x1a
#define MN88472_I2C_ADDR_DVBT2				0x1c

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
