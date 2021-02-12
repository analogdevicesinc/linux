/*
 * AD9371/5 RF Transceiver
 *
 * Copyright 2016-2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
//#define DEBUG
//#define _DEBUG
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>

#include <asm/unaligned.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <linux/jesd204/jesd204.h>

#include "mykonos/t_mykonos.h"
#include "mykonos/mykonos.h"
#include "mykonos/mykonos_gpio.h"
#include "mykonos/mykonos_macros.h"
#include "mykonos/mykonos_user.h"

#include <dt-bindings/iio/adc/adi,ad9371.h>

#include "ad9371.h"

#define FIRMWARE       "Mykonos_M3.bin"

static const int ad9371_auxdac_scale_val2_lut[2][4] = {
	{560671, 590157, 613612, 637490}, /* Factor 0 */
	{783951, 798128, 810932, 822608}, /* Factor 1 */
};

static const int ad9371_auxdac_scale_val1_lut[2] = {
	1, 0 /* Factor 0,1 */
};

static const int ad9371_auxdac_offset_val1_lut[2][4] = {
	{204, 524, 838, 1127}, /* Factor 0 */
	{884, 1508, 2120, 2628}, /* Factor 1 */
};

static int16_t txFirCoefs[] = {-15,33,-92,207,-457,1094,-3342,21908,-4607,2226,-862,376,-169,72,-29,19};

static mykonosFir_t txFir = {
	6,              /* Filter gain in dB*/
	16,             /* Number of coefficients in the FIR filter*/
	&txFirCoefs[0]  /* A pointer to an array of filter coefficients*/
};

static int16_t rxFirCoefs[] = {-20,6,66,22,-128,-54,240,126,-402,-248,634,444,-956,-756,1400,1244,-2028,-2050,2978,3538,-4646,-7046,9536,30880,30880,9536,-7046,-4646,3538,2978,-2050,-2028,1244,1400,-756,-956,444,634,-248,-402,126,240,-54,-128,22,66,6,-20};

static mykonosFir_t rxFir = {
	-6,             /* Filter gain in dB*/
	48,             /* Number of coefficients in the FIR filter*/
	&rxFirCoefs[0]  /* A pointer to an array of filter coefficients*/
};

static int16_t obsrxFirCoefs[] = {150,-55,-64,354,-896,1788,-3101,4953,-7622,9914,-14047,25417,25417,-14047,9914,-7622,4953,-3101,1788,-896,354,-64,-55,150};
static mykonosFir_t obsrxFir = {
	0,              /* Filter gain in dB*/
	24,             /* Number of coefficients in the FIR filter*/
	&obsrxFirCoefs[0]/* A pointer to an array of filter coefficients*/
};

static int16_t snifferFirCoefs[] = {-1,-5,-14,-23,-16,24,92,137,80,-120,-378,-471,-174,507,1174,1183,98,-1771,-3216,-2641,942,7027,13533,17738,17738,13533,7027,942,-2641,-3216,-1771,98,1183,1174,507,-174,-471,-378,-120,80,137,92,24,-16,-23,-14,-5,-1};
static mykonosFir_t snifferRxFir= {
	-6,             /* Filter gain in dB*/
	48,             /* Number of coefficients in the FIR filter*/
	&snifferFirCoefs[0]/* A pointer to an array of filter coefficients*/
};

static int ad9371_string_to_val(const char *buf, int *val)
{
	int ret, integer, fract;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &fract);
	if (ret < 0)
		return ret;

	*val = (abs(integer) * 1000) + (abs(fract) / 1000);

	if ((integer < 0) || (fract < 0))
		*val *= -1;

	return 0;
}

enum ad9371_iio_dev_attr {
	AD9371_ENSM_MODE,
	AD9371_ENSM_MODE_AVAIL,
	AD9371_INIT_CAL,
};

int ad9371_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;
	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, buf[2], ret);

	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	return buf[2];
}

int ad9371_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;

	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0) {
		dev_err(&spi->dev, "%s: failed (%d)\n",
			__func__, ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: REG: 0x%X VAL: 0x%X (%d)\n",
		__func__, reg, val, ret);

	return 0;
}

static int ad9371_reset(struct ad9371_rf_phy *phy)
{
	if (!IS_ERR(phy->reset_gpio)) {
		gpiod_direction_output(phy->reset_gpio, 0);
		udelay(1);
		return gpiod_direction_output(phy->reset_gpio, 1);
	}

	return -ENODEV;
}

static int ad9371_fir_cpy(mykonosFir_t *fir_src, mykonosFir_t *fir_dest)
{
	int i;

	if (!fir_src || !fir_dest)
		return -ENODEV;

	fir_dest->gain_dB = fir_src->gain_dB;
	fir_dest->numFirCoefs = fir_src->numFirCoefs;

	for (i = 0; i < fir_src->numFirCoefs; i++)
		fir_dest->coefs[i] = fir_src->coefs[i];

	return 0;
}

#define DEVM_ALLOC_INIT_N(x, n) \
	{x = devm_kzalloc(dev, (n) * sizeof(*x), GFP_KERNEL);\
	if (!x) \
		goto err_out; \
	}

#define DEVM_ALLOC_INIT(x) DEVM_ALLOC_INIT_N(x, 1)

static int ad9371_alloc_mykonos_device(struct ad9371_rf_phy *phy)
{
	struct device *dev = &phy->spi->dev;

	DEVM_ALLOC_INIT(phy->mykDevice);

	DEVM_ALLOC_INIT(phy->mykDevice->rx);
	DEVM_ALLOC_INIT(phy->mykDevice->tx);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx);
	DEVM_ALLOC_INIT(phy->mykDevice->auxIo);
	DEVM_ALLOC_INIT(phy->mykDevice->clocks);

	DEVM_ALLOC_INIT(phy->mykDevice->spiSettings);

	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxProfile);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxProfile->rxFir);
	DEVM_ALLOC_INIT_N(phy->mykDevice->rx->rxProfile->rxFir->coefs, 72);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxAgcCtrl);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxAgcCtrl->peakAgc);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxAgcCtrl->powerAgc);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->framer);
	DEVM_ALLOC_INIT(phy->mykDevice->rx->rxGainCtrl);

	DEVM_ALLOC_INIT(phy->mykDevice->tx->txProfile);
	DEVM_ALLOC_INIT(phy->mykDevice->tx->txProfile->txFir);
	DEVM_ALLOC_INIT_N(phy->mykDevice->tx->txProfile->txFir->coefs, 96);
	DEVM_ALLOC_INIT(phy->mykDevice->tx->deframer);

	if (IS_AD9375(phy)) {
		DEVM_ALLOC_INIT(phy->mykDevice->tx->dpdConfig);
		DEVM_ALLOC_INIT(phy->mykDevice->tx->clgcConfig);
		DEVM_ALLOC_INIT(phy->mykDevice->tx->vswrConfig);
	}

	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxProfile);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxProfile->rxFir);
	DEVM_ALLOC_INIT_N(phy->mykDevice->obsRx->orxProfile->rxFir->coefs, 72);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxGainCtrl);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxAgcCtrl);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxAgcCtrl->peakAgc);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->orxAgcCtrl->powerAgc);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->snifferProfile);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->snifferProfile->rxFir);
	DEVM_ALLOC_INIT_N(phy->mykDevice->obsRx->snifferProfile->rxFir->coefs, 72);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->snifferGainCtrl);
	DEVM_ALLOC_INIT(phy->mykDevice->obsRx->framer);

	DEVM_ALLOC_INIT(phy->mykDevice->auxIo->gpio3v3);
	DEVM_ALLOC_INIT(phy->mykDevice->auxIo->gpio);
	DEVM_ALLOC_INIT(phy->mykDevice->auxIo->armGpio);

	/* Without profile being loaded we need some filter initially */

	ad9371_fir_cpy(&obsrxFir, phy->mykDevice->obsRx->orxProfile->rxFir);
	ad9371_fir_cpy(&snifferRxFir, phy->mykDevice->obsRx->snifferProfile->rxFir);
	ad9371_fir_cpy(&rxFir, phy->mykDevice->rx->rxProfile->rxFir);
	ad9371_fir_cpy(&txFir, phy->mykDevice->tx->txProfile->txFir);

	return 0;

err_out:
	dev_err(dev, "could not allocate memory for device structure\n");
	return ENODEV;

}


static int ad9371_sysref_req(struct ad9371_rf_phy *phy, enum ad9371_sysref_req_mode mode)
{
	int ret = -EINVAL;

	if (!IS_ERR(phy->sysref_req_gpio)) {
		if (mode == SYSREF_CONT_ON) {
			ret = gpiod_direction_output(phy->sysref_req_gpio, 1);
		} else if (mode == SYSREF_CONT_OFF) {
			ret = gpiod_direction_output(phy->sysref_req_gpio, 0);
		} else if (mode == SYSREF_PULSE) {
			gpiod_direction_output(phy->sysref_req_gpio, 1);
			mdelay(1);
			ret = gpiod_direction_output(phy->sysref_req_gpio, 0);
		}
	} else {
		ret = -ENODEV;
	}

	if (ret)
		dev_err(&phy->spi->dev, "%s: failed (%d)\n", __func__, ret);

	return ret;
}

static int ad9371_set_jesd_lanerate(struct ad9371_rf_phy *phy,
				    u32 input_rate_khz,
				    struct clk *link_clk,
				    mykonosJesd204bFramerConfig_t *framer,
				    mykonosJesd204bDeframerConfig_t *deframer,
				    u32 *lmfc)
{
	unsigned long lane_rate_kHz;
	u32 m, l, k, lmfc_tmp;
	int ret;

	if (!lmfc)
		return -EINVAL;

	if (IS_ERR_OR_NULL(link_clk))
		return 0;

	if (framer) {
		m = framer->M;
		l = hweight8(framer->serializerLanesEnabled);
		k = framer->K;
	} else if (deframer) {
		m = deframer->M;
		l = hweight8(deframer->deserializerLanesEnabled);
		k = deframer->K;
	} else {
		return -EINVAL;
	}

	lane_rate_kHz = input_rate_khz * m * 20 / l;

	ret = clk_set_rate(link_clk, lane_rate_kHz);
	if (ret < 0) {
		dev_err(&phy->spi->dev,
			"Request %s lanerate %lu kHz failed (%d)\n",
			framer ? "framer" : "deframer", lane_rate_kHz, ret);
		return ret;
	}

	lmfc_tmp = (lane_rate_kHz * 100) / (k * ((2 * m) / l));

	if (*lmfc)
		*lmfc = min(*lmfc, lmfc_tmp);
	else
		*lmfc = lmfc_tmp;

	return 0;
}

static bool ad9371_check_sysref_rate(unsigned int lmfc, unsigned int sysref)
{
	unsigned int div, mod;

	div = lmfc / sysref;
	mod = lmfc % sysref;

	/* Ignore minor deviations that can be introduced by rounding. */
	return mod <= div || mod >= sysref - div;
}

static int ad9371_update_sysref(struct ad9371_rf_phy *phy, u32 lmfc)
{
	unsigned int n;
	int rate_dev, rate_fmc, ret;

	dev_dbg(&phy->spi->dev, "%s: setting SYSREF for LMFC rate %u Hz\n",
		__func__, lmfc);

	/* No clock - nothing to do */
	if (IS_ERR(phy->sysref_dev_clk))
		return 0;

	rate_dev = clk_get_rate(phy->sysref_dev_clk);
	if (rate_dev < 0) {
		dev_err(&phy->spi->dev, "Failed to get DEV SYSREF rate\n");
		return rate_dev;
	}

	/* Let's keep the second clock optional */
	if (!IS_ERR(phy->sysref_fmc_clk)) {
		rate_fmc = clk_get_rate(phy->sysref_fmc_clk);
		if (rate_fmc < 0) {
			dev_err(&phy->spi->dev,
				"Failed to get FMC SYSREF rate\n");
			return rate_fmc;
		}
	} else {
		rate_fmc = rate_dev;
	}
	/* If the current rate is OK, keep it */
	if (ad9371_check_sysref_rate(lmfc, rate_dev) &&
		(rate_fmc == rate_dev))
		return 0;

	/*
	 * Try to find a rate that integer divides the LMFC. Starting with a low
	 * rate is a good idea and then slowly go up in case the clock generator
	 * can't generate such slow rates.
	 */
	for (n = 64; n > 0; n--) {
		rate_dev = clk_round_rate(phy->sysref_dev_clk, lmfc / n);
		if (ad9371_check_sysref_rate(lmfc, rate_dev))
			break;
	}

	if (n == 0) {
		dev_err(&phy->spi->dev,
			"Could not find suitable SYSREF rate for LMFC of %u\n",
			lmfc);
		return -EINVAL;
	}

	if (!IS_ERR(phy->sysref_fmc_clk)) {
		ret = clk_set_rate(phy->sysref_fmc_clk, rate_dev);
		if (ret)
			dev_err(&phy->spi->dev,
				"Failed to set FMC SYSREF rate to %d Hz: %d\n",
				rate_dev, ret);
	}

	ret = clk_set_rate(phy->sysref_dev_clk, rate_dev);
	if (ret)
		dev_err(&phy->spi->dev,
			"Failed to set DEV SYSREF rate to %d Hz: %d\n",
			rate_dev, ret);

	dev_dbg(&phy->spi->dev, "%s: setting SYSREF %u Hz\n",
		__func__, rate_dev);

	return ret;
}

static int ad9371_set_radio_state(struct ad9371_rf_phy *phy, enum ad9371_radio_states state)
{
	u32 radioStatus;
	int ret;

	switch (state) {
	case RADIO_OFF:
		ret = MYKONOS_radioOff(phy->mykDevice);
		if (ret == MYKONOS_ERR_OK)
			phy->radio_state = false;
		break;
	case RADIO_ON:
		ret = MYKONOS_radioOn(phy->mykDevice);
		if (ret == MYKONOS_ERR_OK)
			phy->radio_state = true;
		break;
	case RADIO_FORCE_OFF:
		if (phy->radio_state == false) {
			phy->saved_radio_state = false;
			return 0;
		}
		ret = MYKONOS_radioOff(phy->mykDevice);
		if (ret == MYKONOS_ERR_OK) {
			phy->radio_state = false;
		}

		/* read radio state to make sure ARM is in radioOff /IDLE */
		ret = MYKONOS_getRadioState(phy->mykDevice, &radioStatus);
		if (ret != MYKONOS_ERR_OK)
		{
			return ret;
		}

		/* throw error if not in radioOff/IDLE state */
		if ((radioStatus & 0x03) != MYKONOS_ARM_SYSTEMSTATE_IDLE)
		{
			dev_err(&phy->spi->dev, "%s: failed: %s\n", __func__,
				getMykonosErrorMessage(MYKONOS_ERR_EN_TRACKING_CALS_ARMSTATE_ERROR));
			return -EFAULT;
		}


		phy->saved_radio_state = true;
		break;
	case RADIO_RESTORE_STATE:
		if (phy->saved_radio_state) {
			ret = MYKONOS_radioOn(phy->mykDevice);
			if (ret == MYKONOS_ERR_OK)
				phy->radio_state = true;
		} else {
			ret = MYKONOS_radioOff(phy->mykDevice);
			if (ret == MYKONOS_ERR_OK)
				phy->radio_state = false;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

#define AD9375_DPD_STATUS_POLL_LIMIT_MS	500

static int ad9371_get_dpd_status(struct ad9371_rf_phy *phy, int chan)
{
	s64 time_elapsed;
	int ret;

	time_elapsed = ktime_ms_delta(ktime_get(), phy->time_prev_dpd[chan]);

	if (time_elapsed < AD9375_DPD_STATUS_POLL_LIMIT_MS)
		return 0;

	ret = MYKONOS_getDpdStatus(phy->mykDevice, (chan == CHAN_TX2) ?
				   TX2 : TX1, &phy->dpdStatus[chan]);
	phy->time_prev_dpd[chan] = ktime_get();

	if (ret) {
		dev_err(&phy->spi->dev, "%s: %s (%d)", __func__,
			getMykonosErrorMessage(ret), ret);
		return ret;
	}

	if (phy->dpdStatus[chan].dpdErrorStatus)
		dev_dbg(&phy->spi->dev, "%s: status error (%d)\n", __func__,
			phy->dpdStatus[chan].dpdErrorStatus);

	return ret;
}

static int ad9371_get_clgc_status(struct ad9371_rf_phy *phy, int chan)
{
	s64 time_elapsed;
	int ret;

	time_elapsed = ktime_ms_delta(ktime_get(), phy->time_prev_clgc[chan]);

	if (time_elapsed < AD9375_DPD_STATUS_POLL_LIMIT_MS)
		return 0;

	ret = MYKONOS_getClgcStatus(phy->mykDevice, (chan == CHAN_TX2) ? TX2 : TX1,
				    &phy->clgcStatus[chan]);
	phy->time_prev_clgc[chan] = ktime_get();

	if (ret) {
		dev_err(&phy->spi->dev, "%s: %s (%d)", __func__,
			getMykonosErrorMessage(ret), ret);
		return ret;
	}

	if (phy->clgcStatus[chan].errorStatus)
		dev_dbg(&phy->spi->dev, "%s: status error (%d)\n", __func__,
			phy->clgcStatus[chan].errorStatus);

	return ret;
}

static int ad9371_get_vswr_status(struct ad9371_rf_phy *phy, int chan)
{
	s64 time_elapsed;
	int ret;

	time_elapsed = ktime_ms_delta(ktime_get(), phy->time_prev_vswr[chan]);

	if (time_elapsed < AD9375_DPD_STATUS_POLL_LIMIT_MS)
		return 0;

	ret = MYKONOS_getVswrStatus(phy->mykDevice, (chan == CHAN_TX2) ?
				    TX2 : TX1, &phy->vswrStatus[chan]);
	phy->time_prev_vswr[chan] = ktime_get();

	if (ret) {
		dev_err(&phy->spi->dev, "%s: %s (%d)", __func__,
			getMykonosErrorMessage(ret), ret);
		return ret;
	}

	if (phy->vswrStatus[chan].errorStatus)
		dev_dbg(&phy->spi->dev, "%s: status error (%d)\n", __func__,
			phy->vswrStatus[chan].errorStatus);

	return ret;
}

static int ad9371_init_cal(struct ad9371_rf_phy *phy, uint32_t initCalMask)
{

	uint8_t errorFlag = 0;
	uint8_t errorCode = 0;
	uint32_t initCalsCompleted = 0;
	uint16_t errorWord = 0;
	uint16_t statusWord = 0;
	uint8_t status = 0;
	mykonosInitCalStatus_t initCalStatus;
	mykonosErr_t mykError;

	/*****************************************************/
	/*** Mykonos ARM Initialization Calibrations       ***/
	/*****************************************************/

	mykError = MYKONOS_runInitCals(phy->mykDevice, initCalMask);
	mykError = MYKONOS_waitInitCals(phy->mykDevice, 60000, &errorFlag, &errorCode);

	if ((errorFlag != 0) || (errorCode != 0)) {
		mykError = MYKONOS_getInitCalStatus(phy->mykDevice, &initCalStatus);
		if(mykError) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(mykError), mykError);
		}

		dev_err(&phy->spi->dev, "calsDoneLifetime 0x%X, calsDoneLastRun 0x%X,"
			" calsMinimum 0x%X, initErrCal 0x%X, initErrCode 0x%X",
			initCalStatus.calsDoneLifetime, initCalStatus.calsDoneLastRun,
			initCalStatus.calsMinimum, initCalStatus.initErrCal,
			initCalStatus.initErrCode);

		//abort init calibrations
		mykError = MYKONOS_abortInitCals(phy->mykDevice, &initCalsCompleted);
		if(mykError) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(mykError), mykError);
		}

		dev_err(&phy->spi->dev, "initCalsCompleted 0x%X", initCalsCompleted);

		mykError = MYKONOS_readArmCmdStatus(phy->mykDevice, &errorWord, &statusWord);
		if(mykError) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(mykError), mykError);
		}

		dev_err(&phy->spi->dev, "errorWord 0x%X, statusWord 0x%X", errorWord, statusWord);

		mykError = MYKONOS_readArmCmdStatusByte(phy->mykDevice, 2, &status);

		dev_err(&phy->spi->dev, "ArmCmdStatusByte 0x%X", status);

		if(mykError) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(mykError), mykError);
		}
	}

	return 0;
}

static const char * const ad9371_ilas_mismatch_table[] = {
	"device ID",
	"bank ID",
	"lane ID",
	"lanes per converter",
	"scrambling",
	"octets per frame",
	"frames per multiframe",
	"number of converters",
	"sample resolution",
	"control bits per sample",
	"bits per sample",
	"samples per frame",
	"control words per frame",
	"high density",
	"checksum"
};

static int ad9371_setup(struct ad9371_rf_phy *phy)
{
	int ret;
	uint8_t mcsStatus = 0;
	uint8_t pllLock, pllLockStatus = 0;

	uint8_t deframerStatus = 0;
	uint8_t obsFramerStatus = 0;
	uint16_t mismatch = 0;
	uint8_t framerStatus = 0;
	long dev_clk, fmc_clk;
	uint32_t initCalMask;
	unsigned int i;
	uint32_t lmfc = 0;

	mykonosDevice_t *mykDevice = phy->mykDevice;
	phy->tracking_cal_mask = 0;

	initCalMask = phy->init_cal_mask |= TX_BB_FILTER | ADC_TUNER | TIA_3DB_CORNER | DC_OFFSET |
			       TX_ATTENUATION_DELAY | RX_GAIN_DELAY | FLASH_CAL |
			       PATH_DELAY | LOOPBACK_RX_LO_DELAY | LOOPBACK_RX_RX_QEC_INIT |
			       RX_LO_DELAY;

	if (has_rx_and_en(phy))
		phy->tracking_cal_mask |= TRACK_RX1_QEC | TRACK_RX2_QEC;

	if (has_tx_and_en(phy))
		phy->tracking_cal_mask |= TRACK_TX1_QEC | TRACK_TX2_QEC;

	/**********************************************************/
	/**********************************************************/
	/************ Mykonos Initialization Sequence *************/
	/**********************************************************/
	/**********************************************************/

	/*******************************/
	/**** Mykonos Initialization ***/
	/*******************************/

	dev_clk = clk_round_rate(phy->dev_clk, mykDevice->clocks->deviceClock_kHz * 1000);
	fmc_clk = clk_round_rate(phy->fmc_clk, mykDevice->clocks->deviceClock_kHz * 1000);

	if (dev_clk > 0 && fmc_clk > 0 && fmc_clk == dev_clk &&
		(dev_clk / 1000) == mykDevice->clocks->deviceClock_kHz) {
		clk_set_rate(phy->fmc_clk, (unsigned long) dev_clk);
		clk_set_rate(phy->dev_clk, (unsigned long) dev_clk);
	} else {
		dev_err(&phy->spi->dev, "Requesting device clock %u failed got %ld",
			mykDevice->clocks->deviceClock_kHz * 1000, dev_clk);
		return -EINVAL;
	}

	if (has_rx_and_en(phy)) {
		ret = ad9371_set_jesd_lanerate(
			phy, mykDevice->rx->rxProfile->iqRate_kHz, phy->jesd_rx_clk,
			mykDevice->rx->framer, NULL, &lmfc);
		if (ret < 0)
			goto out;
	}

	if (has_obs_and_en(phy)) {
		ret = ad9371_set_jesd_lanerate(
			phy, mykDevice->obsRx->orxProfile->iqRate_kHz,
			phy->jesd_rx_os_clk, mykDevice->obsRx->framer, NULL, &lmfc);
		if (ret < 0)
			goto out;
	}

	if (has_tx_and_en(phy)) {
		ret = ad9371_set_jesd_lanerate(
			phy, mykDevice->tx->txProfile->iqRate_kHz, phy->jesd_tx_clk,
			NULL, mykDevice->tx->deframer, &lmfc);
		if (ret < 0)
			goto out;
	}

	ret = ad9371_update_sysref(phy, lmfc);
	if (ret < 0)
		goto out;

	/* Toggle RESETB pin on Mykonos device */

	ret = ad9371_reset(phy);
	if (ret) {
		dev_err(&phy->spi->dev, "RESET Failed");
		goto out;
	}

	/* MYKONOS_initialize() loads the Mykonos device data structure
	 * settings for the Rx/Tx/ORx/Sniffer profiles, digital
	 * filter enables, calibrates the CLKPLL, and loads the user provided Rx
	 * gain tables.
	 */
	ret = MYKONOS_initialize(mykDevice);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	MYKONOS_getProductId(phy->mykDevice, &phy->device_id);
	if (phy->device_id != AD937x_PRODID(phy)) {
		if (!(IS_AD9375(phy) && (phy->device_id == (ID_AD9375_ALT & 0xFF)))) {
			dev_err(&phy->spi->dev, "Failed product ID, expected 0x%X got 0x%X",
				AD937x_PRODID(phy), phy->device_id );
			ret = -ENODEV;
			goto out;
		}
	}

	/*******************************/
	/***** CLKPLL Status Check *****/
	/*******************************/
	ret = MYKONOS_checkPllsLockStatus(mykDevice, &pllLockStatus);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	pllLock = BIT(0); /* CLKPLL Locked */

	if (!(pllLockStatus & pllLock)) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	/*******************************************************/
	/**** Perform MultiChip Sync (MCS) on Mykonos Device ***/
	/*******************************************************/
	ret = MYKONOS_enableMultichipSync(mykDevice, 1, NULL);

	ad9371_sysref_req(phy, SYSREF_PULSE);
	/*** < Request minimum 3 SYSREF pulses from Clock Device - user code here > ***/

	/*******************/
	/***** MCS ****/
	/*******************/
	ret = MYKONOS_enableMultichipSync(mykDevice, 0, &mcsStatus);

	if ((mcsStatus & 0x0B) != 0x0B) {
		/*** < MCS failed - ensure MCS before proceeding - user code here > ***/
		dev_err(&phy->spi->dev, "MCS failed");
		ret = -EFAULT;
		goto out;
	}

	/*************************/
	/**** Load Mykonos ARM ***/
	/*************************/

	ret = MYKONOS_initArm(mykDevice);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	ret = MYKONOS_loadArmFromBinary(mykDevice,
					     (u8 *) phy->fw->data,
					     phy->fw->size);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	/*******************************/
	/**** Set RF PLL Frequencies ***/
	/*******************************/
	if (has_rx_and_en(phy)) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, RX_PLL,
						mykDevice->rx->rxPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			ret = -EFAULT;
			goto out;
		}
		pllLock |= BIT(1); /* RX_PLL Locked */
	}
	if (has_tx_and_en(phy)) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, TX_PLL,
						mykDevice->tx->txPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			ret = -EFAULT;
			goto out;
		}
		pllLock |= BIT(2); /* TX_PLL Locked */
	}
	if (has_obs_and_en(phy)) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, SNIFFER_PLL,
						mykDevice->obsRx->snifferPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			ret = -EFAULT;
			goto out;
		}
		pllLock |= BIT(3); /* SNIFFER_PLL Locked */
	}

	/*** < wait 200ms for PLLs to lock - user code here > ***/

	ret = MYKONOS_checkPllsLockStatus(mykDevice, &pllLockStatus);
	if ((pllLockStatus & pllLock) != pllLock) {
		dev_err(&phy->spi->dev, "PLLs unlocked %x", pllLockStatus & 0x0F);
		ret = -EFAULT;
		goto out;
	}

	if (IS_AD9375(phy) && has_tx_and_en(phy)) {
		ret = MYKONOS_configDpd(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}

		ret = MYKONOS_configClgc(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}

		ret = MYKONOS_configVswr(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}

		phy->dpd_actuator_en[0] = true;
		phy->dpd_actuator_en[1] = true;

	}

	/*****************************************************/
	/*** Mykonos ARM Initialization Calibrations       ***/
	/*****************************************************/

	ret = ad9371_init_cal(phy, initCalMask);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out;
	}

	/* FIXME: Add API call */
	//ad9371_init_cal(phy, TX_LO_LEAKAGE_EXTERNAL);

	/*************************************************/
	/**** Enable SYSREF to Mykonos JESD204B Framers ***/
	/*************************************************/
	/*** < User: Make sure SYSREF is stopped/disabled > ***/
	/*** < User: make sure BBIC JESD is reset and ready to recieve CGS chars> ***/

	if (has_rx_and_en(phy)) {
		ret = MYKONOS_enableSysrefToRxFramer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}
	}
	/*** < User: Mykonos is actively transmitting CGS from the RxFramer> ***/

	if (has_obs_and_en(phy)) {
		ret = MYKONOS_enableSysrefToObsRxFramer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}
	}
	/*** < User: Mykonos is actively transmitting CGS from the ObsRxFramer> ***/

	/***************************************************/
	/**** Enable SYSREF to Mykonos JESD204B Deframer ***/
	/***************************************************/
	/*** < User: Make sure SYSREF is stopped/disabled > ***/

	if (has_tx_and_en(phy)) {
		ret = MYKONOS_enableSysrefToDeframer(mykDevice, 0);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}
		ret = MYKONOS_resetDeframer(mykDevice);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out;
		}

		ret = clk_prepare_enable(phy->jesd_tx_clk);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "jesd_tx_clk enable failed (%d)", ret);
			goto out;
		}

		/*** < User: make sure BBIC JESD framer is actively transmitting CGS> ***/
		ret = MYKONOS_enableSysrefToDeframer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out_disable_tx_clk;
		}
	}

	/*** < User Sends SYSREF Here > ***/

	ad9371_sysref_req(phy, SYSREF_CONT_ON);

	if (has_rx_and_en(phy)) {
		ret = clk_prepare_enable(phy->jesd_rx_clk);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "jesd_rx_clk enable failed (%d)", ret);
			goto out_disable_tx_clk;
		}
	}

	if (has_obs_and_en(phy)) {
		ret = clk_prepare_enable(phy->jesd_rx_os_clk);
		if (ret < 0) {
			dev_err(&phy->spi->dev, "jesd_rx_os_clk enable failed (%d)", ret);
			goto out_disable_rx_clk;
		}
	}

	ad9371_sysref_req(phy, SYSREF_CONT_OFF);

	/*** <  User: Insert User BBIC JESD Sync Verification Code Here > ***/

	/************************************/
	/**** Check Mykonos Framer Status ***/
	/************************************/
	if (has_rx_and_en(phy)) {
		ret = MYKONOS_readRxFramerStatus(mykDevice, &framerStatus);
		if (framerStatus != 0x3E)
			dev_warn(&phy->spi->dev, "framerStatus (0x%X)", framerStatus);
	}

	if (has_obs_and_en(phy)) {
		ret = MYKONOS_readOrxFramerStatus(mykDevice, &obsFramerStatus);
		if (obsFramerStatus != 0x3E)
			dev_warn(&phy->spi->dev, "obsFramerStatus (0x%X)", obsFramerStatus);
	}

	/**************************************/
	/**** Check Mykonos Deframer Status ***/
	/**************************************/
	if (has_tx_and_en(phy)) {
		ret = MYKONOS_readDeframerStatus(mykDevice, &deframerStatus);
		if (deframerStatus != 0x28)
			dev_warn(&phy->spi->dev, "deframerStatus (0x%X)", deframerStatus);

		ret = MYKONOS_jesd204bIlasCheck(mykDevice, &mismatch);
		if (mismatch) {
			dev_warn(&phy->spi->dev, "ILAS mismatch: %04x\n", mismatch);
			for (i = 0; i < ARRAY_SIZE(ad9371_ilas_mismatch_table); i++) {
				if (mismatch & BIT(i))
					dev_warn(&phy->spi->dev, "ILAS %s did not match\n",
						ad9371_ilas_mismatch_table[i]);
			}
		}
	}
	/*** < User: When links have been verified, proceed > ***/

	/* Allow Rx1/2 QEC tracking and Tx1/2 QEC tracking to run when in the radioOn state         */
	/* Tx calibrations will only run if radioOn and the obsRx path is set to OBS_INTERNAL_CALS  */

	ret = MYKONOS_enableTrackingCals(mykDevice, phy->tracking_cal_mask);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	/* Function to turn radio on, Enables transmitters and receivers */
	/* that were setup during MYKONOS_initialize() */

	ret = MYKONOS_setupObsRxAgc(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	ret = MYKONOS_setupRxAgc(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	ret = ad9371_set_radio_state(phy, RADIO_ON);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		goto out_disable_obs_rx_clk;
	}

	/* Allow TxQEC to run when user is not actively using ORx receive path */
	ret = MYKONOS_setObsRxPathSource(mykDevice, OBS_INTERNALCALS);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	ret = MYKONOS_setupAuxAdcs(mykDevice, 4, 1);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	ret = MYKONOS_setupAuxDacs(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	ret = MYKONOS_setupGpio(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		ret = -EFAULT;
		goto out_disable_obs_rx_clk;
	}

	clk_set_rate(phy->clks[RX_SAMPL_CLK],
		     mykDevice->rx->rxProfile->iqRate_kHz * 1000);
	clk_set_rate(phy->clks[OBS_SAMPL_CLK],
		     mykDevice->obsRx->orxProfile->iqRate_kHz * 1000);
	clk_set_rate(phy->clks[TX_SAMPL_CLK],
		     mykDevice->tx->txProfile->iqRate_kHz * 1000);

	phy->rf_bandwith[0] = mykDevice->rx->rxProfile->rfBandwidth_Hz;
	phy->rf_bandwith[1] = mykDevice->obsRx->orxProfile->rfBandwidth_Hz;
	phy->rf_bandwith[2] = mykDevice->tx->txProfile->primarySigBandwidth_Hz;

	phy->is_initialized = 1;

	return 0;

out_disable_obs_rx_clk:
	if (has_obs_and_en(phy))
		clk_disable_unprepare(phy->jesd_rx_os_clk);
out_disable_rx_clk:
	if (has_rx_and_en(phy))
		clk_disable_unprepare(phy->jesd_rx_clk);
out_disable_tx_clk:
	if (has_tx_and_en(phy))
		clk_disable_unprepare(phy->jesd_tx_clk);

out:
	phy->is_initialized = 0;

	return ret;
}

static int ad9371_reinit(struct ad9371_rf_phy *phy)
{
	int ret;

	if (phy->jdev) {
		if(jesd204_dev_is_top(phy->jdev)) {
			int retry = 1;
			do {
				jesd204_fsm_stop(phy->jdev, JESD204_LINKS_ALL);
				jesd204_fsm_clear_errors(phy->jdev, JESD204_LINKS_ALL);
				ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
			} while (ret < 0 && retry--);
		} else {
			dev_warn(&phy->spi->dev, "initialize ignored: in multichip "
				"configuration this is only allowed by the top device");
			ret = -ENOTSUPP;
		}
	} else {
		if (phy->is_initialized) {
			if (has_rx_and_en(phy))
				clk_disable_unprepare(phy->jesd_rx_clk);
			if (has_obs_and_en(phy))
				clk_disable_unprepare(phy->jesd_rx_os_clk);
			if (has_tx_and_en(phy))
				clk_disable_unprepare(phy->jesd_tx_clk);
		}

		ret = ad9371_setup(phy);
		if (ret)
			ret = ad9371_setup(phy);
	}

	return ret;
}

static ssize_t ad9371_phy_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0;
	u32 val;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address & 0xFF) {
	case AD9371_ENSM_MODE:
		if (sysfs_streq(buf, "radio_on")) {
			val = RADIO_ON;
		} else if (sysfs_streq(buf, "radio_off")) {
			val = RADIO_OFF;
		} else {
			ret = -EINVAL;
			break;
		}

		ret = ad9371_set_radio_state(phy, val);
		break;
	case AD9371_INIT_CAL:
		ret = strtobool(buf, &enable);
		if (ret)
			break;

		val = (u32)this_attr->address >> 8;

		if (val) {
			if (enable)
				phy->cal_mask |= val;
			else
				phy->cal_mask &= ~val;
		} else if (enable) {
			ad9371_set_radio_state(phy, RADIO_FORCE_OFF);

			ret  = ad9371_init_cal(phy, phy->cal_mask);
			if (ret != MYKONOS_ERR_OK) {
				dev_err(&phy->spi->dev, "%s (%d)",
					getMykonosErrorMessage(ret), ret);
				ret = -EFAULT;
			}

			ad9371_set_radio_state(phy, RADIO_RESTORE_STATE);
		}
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9371_phy_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u32 val;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address & 0xFF) {
	case AD9371_ENSM_MODE:
		ret = sprintf(buf, "%s\n", phy->radio_state ? "radio_on" : "radio_off");
		break;
	case AD9371_ENSM_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", "radio_on radio_off");
		break;
	case AD9371_INIT_CAL:
		val = (u32)this_attr->address >> 8;

		if (val)
			ret = sprintf(buf, "%d\n", !!(phy->cal_mask & val));
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(ensm_mode, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_ENSM_MODE);

static IIO_DEVICE_ATTR(ensm_mode_available, S_IRUGO,
		       ad9371_phy_show,
		       NULL,
		       AD9371_ENSM_MODE_AVAIL);

static IIO_DEVICE_ATTR(calibrate, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL);

static IIO_DEVICE_ATTR(calibrate_dpd_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (DPD_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_clgc_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (CLGC_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_vswr_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (VSWR_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_rx_qec_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (RX_QEC_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_tx_qec_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (TX_QEC_INIT << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (TX_LO_LEAKAGE_INTERNAL << 8));

static IIO_DEVICE_ATTR(calibrate_tx_lol_ext_en, S_IRUGO | S_IWUSR,
		       ad9371_phy_show,
		       ad9371_phy_store,
		       AD9371_INIT_CAL | (TX_LO_LEAKAGE_EXTERNAL << 8));


static struct attribute *ad9371_phy_attributes[] = {
	&iio_dev_attr_ensm_mode.dev_attr.attr,
	&iio_dev_attr_ensm_mode_available.dev_attr.attr,
	&iio_dev_attr_calibrate.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_ext_en.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9371_phy_attribute_group = {
	.attrs = ad9371_phy_attributes,
};

static struct attribute *ad9375_phy_attributes[] = {
	&iio_dev_attr_ensm_mode.dev_attr.attr,
	&iio_dev_attr_ensm_mode_available.dev_attr.attr,
	&iio_dev_attr_calibrate.dev_attr.attr,
	&iio_dev_attr_calibrate_dpd_en.dev_attr.attr,
	&iio_dev_attr_calibrate_clgc_en.dev_attr.attr,
	&iio_dev_attr_calibrate_rx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_qec_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_en.dev_attr.attr,
	&iio_dev_attr_calibrate_tx_lol_ext_en.dev_attr.attr,
	&iio_dev_attr_calibrate_vswr_en.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad9375_phy_attribute_group = {
	.attrs = ad9375_phy_attributes,
};

static int ad9371_phy_reg_access(struct iio_dev *indio_dev,
				 u32 reg, u32 writeval,
				 u32 *readval)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = ad9371_spi_write(phy->spi, reg, writeval);
	} else {
		*readval = ad9371_spi_read(phy->spi, reg);
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

enum lo_ext_info {
	LOEXT_FREQ,
};

static ssize_t ad9371_phy_lo_write(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	u64 readin;
	u8 status;
	int ret = 0;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		ad9371_set_radio_state(phy, RADIO_FORCE_OFF);
		ret = MYKONOS_setRfPllFrequency(phy->mykDevice, chan->channel + 1, readin);
		if (ret != MYKONOS_ERR_OK)
			break;

		ret = MYKONOS_checkPllsLockStatus(phy->mykDevice, &status);
		if (!((status & BIT(chan->channel + 1) || (ret != MYKONOS_ERR_OK))))
			ret = -EFAULT;

		ad9371_set_radio_state(phy, RADIO_RESTORE_STATE);
		break;
	default:
		ret = -EINVAL;
		break;

	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9371_phy_lo_read(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  char *buf)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	u64 val;
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case LOEXT_FREQ:
		ret = MYKONOS_getRfPllFrequency(phy->mykDevice, chan->channel + 1, &val);
		break;
	default:
		ret = 0;

	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : sprintf(buf, "%llu\n", val);
}

#define _AD9371_EXT_LO_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9371_phy_lo_read, \
	.write = ad9371_phy_lo_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info ad9371_phy_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_AD9371_EXT_LO_INFO("frequency", LOEXT_FREQ),
	{ },
};

static int ad9371_set_agc_mode(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan, u32 mode)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	unsigned val;
	int ret;

	switch (mode) {
	case 0:
		val = MGC;
		break;
	case 1:
		val = AGC;
		break;
	case 2:
		val = HYBRID;
		break;
	default:
		return -EINVAL;
	}

	if (chan->channel >= CHAN_OBS)
		ret = MYKONOS_setObsRxGainControlMode(phy->mykDevice, val);
	else
		ret = MYKONOS_setRxGainControlMode(phy->mykDevice, val);
	if (ret)
		return -EFAULT;

	phy->agc_mode[chan->channel > CHAN_RX2] = mode;

	return 0;
}

static int ad9371_get_agc_mode(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);

	return phy->agc_mode[chan->channel > CHAN_RX2];
}

static const char * const ad9371_agc_modes[] =
{"manual", "automatic", "hybrid"};

static const struct iio_enum ad9371_agc_modes_available = {
	.items = ad9371_agc_modes,
	.num_items = ARRAY_SIZE(ad9371_agc_modes),
	.get = ad9371_get_agc_mode,
	.set = ad9371_set_agc_mode,

};

static const char * const ad9371_obs_rx_port[] = {
	"OFF", "INTERNALCALS", "OBS_SNIFFER", "SN_A", "SN_B", "SN_C",
	"ORX1_TX_LO", "ORX2_TX_LO", "ORX1_SN_LO", "ORX2_SN_LO"
};

static const mykonosObsRxChannels_t ad9371_obs_rx_port_lut[] = {
	OBS_RXOFF, OBS_INTERNALCALS, OBS_SNIFFER,  OBS_SNIFFER_A, OBS_SNIFFER_B, OBS_SNIFFER_C,
	OBS_RX1_TXLO, OBS_RX2_TXLO, OBS_RX1_SNIFFERLO, OBS_RX2_SNIFFERLO
};

static int ad9371_set_obs_rx_path(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, u32 mode)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	ret = MYKONOS_setObsRxPathSource(phy->mykDevice, ad9371_obs_rx_port_lut[mode]);
	if (!ret)
		phy->obs_rx_path_source = mode;

	return ret;

}

static int ad9371_get_obs_rx_path(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	mykonosObsRxChannels_t src;
	int ret, i;

	ret = MYKONOS_getObsRxPathSource(phy->mykDevice, &src);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ad9371_obs_rx_port_lut); i++)
		if (ad9371_obs_rx_port_lut[i] == src)
			return i;

	return -EINVAL;
}

static const struct iio_enum ad9371_rf_obs_rx_port_available = {
	.items = ad9371_obs_rx_port,
	.num_items = ARRAY_SIZE(ad9371_obs_rx_port),
	.get = ad9371_get_obs_rx_path,
	.set = ad9371_set_obs_rx_path,
};

static void ad9371_orx_qec_verify_force_enable(struct ad9371_rf_phy *phy)
{
	/*
	* Note that TRACK_ORX1_QEC, TRACK_ORX2_QEC mask bits must be set in
	* order to have successful DPD, CLGC, and VSWR tracking.
	*/

	if (phy->tracking_cal_mask &
		(TRACK_TX1_DPD | TRACK_TX1_CLGC | TRACK_TX1_VSWR))
		phy->tracking_cal_mask |= TRACK_ORX1_QEC;

	if (phy->tracking_cal_mask &
		(TRACK_TX2_DPD | TRACK_TX2_CLGC | TRACK_TX2_VSWR))
		phy->tracking_cal_mask |= TRACK_ORX2_QEC;
}

static ssize_t ad9371_phy_rx_write(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	int ret = 0, val;
	u32 mask;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case RSSI:

		break;
	case RX_QEC:
		ret = strtobool(buf, &enable);

		switch (chan->channel) {
		case CHAN_RX1:
			mask = TRACK_RX1_QEC;
			break;
		case CHAN_RX2:
			mask = TRACK_RX2_QEC;
			break;
		case CHAN_OBS:
			mask = TRACK_ORX1_QEC | TRACK_ORX2_QEC;
			break;
		default:
			ret = -EINVAL;
			goto unlock;
		}

		if (enable)
			phy->tracking_cal_mask |= mask;
		else
			phy->tracking_cal_mask &= ~mask;

		ad9371_orx_qec_verify_force_enable(phy);

		ad9371_set_radio_state(phy, RADIO_FORCE_OFF);
		ret = MYKONOS_enableTrackingCals(phy->mykDevice, phy->tracking_cal_mask);
		ad9371_set_radio_state(phy, RADIO_RESTORE_STATE);

		break;
	case TEMPCOMP_GAIN:
		ret = ad9371_string_to_val(buf, &val);
		if (ret < 0)
			break;

		val = clamp(val, -3000, 3000);

		switch (chan->channel) {
		case CHAN_RX1:
			ret = MYKONOS_setRx1TempGainComp(phy->mykDevice, val);
			break;
		case CHAN_RX2:
			ret = MYKONOS_setRx2TempGainComp(phy->mykDevice, val);
			break;
		case CHAN_OBS:
			ret = MYKONOS_setObsRxTempGainComp(phy->mykDevice, val);
			break;
		default:
			ret = -EINVAL;
		}
	}

unlock:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9371_phy_rx_read(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  char *buf)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret = 0;
	u16 dec_pwr_mdb;
	s16 val_s16;
	u32 mask;

	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case RSSI:
		switch (chan->channel) {
		case CHAN_RX1:
			ret = MYKONOS_getRx1DecPower(phy->mykDevice, &dec_pwr_mdb);
			break;
		case CHAN_RX2:
			ret = MYKONOS_getRx2DecPower(phy->mykDevice, &dec_pwr_mdb);
			break;
		case CHAN_OBS:
			ret = MYKONOS_getObsRxDecPower(phy->mykDevice, &dec_pwr_mdb);
			break;
		default:
			ret = -EINVAL;
		}

		if (ret == 0)
			ret = sprintf(buf, "%u.%02u dB\n", dec_pwr_mdb / 1000,
				      dec_pwr_mdb % 1000);
		break;
	case RX_QEC:
		switch (chan->channel) {
		case CHAN_RX1:
			mask = TRACK_RX1_QEC;
			break;
		case CHAN_RX2:
			mask = TRACK_RX2_QEC;
			break;
		case CHAN_OBS:
			mask = TRACK_ORX1_QEC | TRACK_ORX2_QEC;
			break;
		default:
			ret = -EINVAL;
		}

		if (ret == 0)
			ret = sprintf(buf, "%d\n", !!(mask & phy->tracking_cal_mask));

		break;
	case TEMPCOMP_GAIN:
		switch (chan->channel) {
		case CHAN_RX1:
			ret = MYKONOS_getRx1TempGainComp(phy->mykDevice, &val_s16);
			break;
		case CHAN_RX2:
			ret = MYKONOS_getRx2TempGainComp(phy->mykDevice, &val_s16);
			break;
		case CHAN_OBS:
			ret = MYKONOS_getObsRxTempGainComp(phy->mykDevice, &val_s16);
			break;
		default:
			ret = -EINVAL;
		}

		if (ret == 0)
			ret = sprintf(buf, "%d.%02u dB\n", val_s16 / 1000,
				      (u32) (abs(val_s16) % 1000));

		break;
	case RX_RF_BANDWIDTH:
		switch (chan->channel) {
		case CHAN_RX1:
		case CHAN_RX2:
			ret = phy->rf_bandwith[0];
			break;
		case CHAN_OBS:
			ret = phy->rf_bandwith[1];
			break;
		default:
			ret = -EINVAL;
		}

		if (ret > 0)
			ret = sprintf(buf, "%u\n", ret);

		break;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

#define _AD9371_EXT_RX_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9371_phy_rx_read, \
	.write = ad9371_phy_rx_write, \
	.private = _ident, \
}

static const mykonosTrackingCalibrations_t tx_track_cal_mask[][2] = {
	[TX_QEC] = {TRACK_TX1_QEC, TRACK_TX2_QEC},
	[TX_LOL] = {TRACK_TX1_LOL, TRACK_TX2_LOL},
	[TX_DPD] = {TRACK_TX1_DPD, TRACK_TX2_DPD},
	[TX_CLGC] = {TRACK_TX1_CLGC, TRACK_TX2_CLGC},
	[TX_VSWR] = {TRACK_TX1_VSWR, TRACK_TX2_VSWR},
};

static ssize_t ad9371_phy_tx_read(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  char *buf)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	u32 mask;
	int val, ret = 0;

	if (chan->channel > CHAN_TX2)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_DPD:
	case TX_CLGC:
	case TX_VSWR:
		mask = tx_track_cal_mask[private][chan->channel];
		val = !!(mask & phy->tracking_cal_mask);
		break;
	case TX_RF_BANDWIDTH:
		switch (chan->channel) {
		case CHAN_TX1:
		case CHAN_TX2:
			val = phy->rf_bandwith[2];
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case TX_DPD_ACT_EN:
		val = phy->dpd_actuator_en[chan->channel];
		break;
	case TX_DPD_TRACKCNT:
		ret = ad9371_get_dpd_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->dpdStatus[chan->channel].dpdTrackCount;
		break;
	case TX_DPD_MODEL_ERR:
		ret = ad9371_get_dpd_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->dpdStatus[chan->channel].dpdModelErrorPercent;
		break;
	case TX_DPD_EXT_PATH_DLY:
		ret = ad9371_get_dpd_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->dpdStatus[chan->channel].dpdExtPathDelay;
		break;
	case TX_DPD_STATUS:
		ret = ad9371_get_dpd_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->dpdStatus[chan->channel].dpdErrorStatus;
		break;
	case TX_DPD_RESET:
		val = 0;
		break;
	case TX_CLGC_TRACKCNT:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].trackCount;
		break;
	case TX_CLGC_DES_GAIN:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].desiredGain;
		break;
	case TX_CLGC_CUR_GAIN:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].currentGain;
		break;
	case TX_CLGC_TX_GAIN:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].txGain;
		break;
	case TX_CLGC_TX_RMS:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].txRms;
		break;
	case TX_CLGC_ORX_RMS:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].orxRms;
		break;
	case TX_CLGC_STATUS:
		ret = ad9371_get_clgc_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->clgcStatus[chan->channel].errorStatus;
		break;
	case TX_VSWR_TRACKCNT:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].trackCount;
		break;
	case TX_VSWR_FW_GAIN:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].forwardGainRms_dB;
		break;
	case TX_VSWR_FW_GAIN_REAL:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].forwardGainReal;
		break;
	case TX_VSWR_FW_GAIN_IMAG:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].forwardGainImag;
		break;
	case TX_VSWR_REF_GAIN:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].reflectedGainRms_dB;
		break;
	case TX_VSWR_REF_GAIN_REAL:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].reflectedGainReal;
		break;
	case TX_VSWR_REF_GAIN_IMAG:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].reflectedGainImag;
		break;
	case TX_VSWR_FW_TX:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].vswr_forward_tx_rms;
		break;
	case TX_VSWR_FW_ORX:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].vswr_forward_orx_rms;
		break;
	case TX_VSWR_REF_TX:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].vswr_reflection_tx_rms;
		break;
	case TX_VSWR_REF_ORX:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].vswr_reflection_orx_rms;
		break;
	case TX_VSWR_STATUS:
		ret = ad9371_get_vswr_status(phy, chan->channel);
		if (ret)
			break;
		val = phy->vswrStatus[chan->channel].errorStatus;
		break;
	default:
		ret = -EINVAL;

	}

	if (!ret)
		ret = sprintf(buf, "%d\n", val);

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9371_phy_tx_write(struct iio_dev *indio_dev,
				   uintptr_t private,
				   const struct iio_chan_spec *chan,
				   const char *buf, size_t len)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	bool enable;
	long res;
	int ret = 0;
	u32 mask;

	if (chan->channel > CHAN_TX2)
		return -EINVAL;



	mutex_lock(&indio_dev->mlock);

	switch (private) {
	case TX_QEC:
	case TX_LOL:
	case TX_DPD:
	case TX_CLGC:
	case TX_VSWR:
		ret = strtobool(buf, &enable);
		if (ret)
			break;

		mask = tx_track_cal_mask[private][chan->channel];

		if (enable)
			phy->tracking_cal_mask |= mask;
		else
			phy->tracking_cal_mask &= ~mask;

		ad9371_orx_qec_verify_force_enable(phy);

		ad9371_set_radio_state(phy, RADIO_FORCE_OFF);
		ret = MYKONOS_enableTrackingCals(phy->mykDevice, phy->tracking_cal_mask);
		ad9371_set_radio_state(phy, RADIO_RESTORE_STATE);

		break;
	case TX_DPD_ACT_EN:
		ret = strtobool(buf, &enable);
		if (ret)
			break;

		ret = MYKONOS_setDpdActState(phy->mykDevice,
					     chan->channel + 1, enable);
		if (!ret)
			phy->dpd_actuator_en[chan->channel] = enable;
		break;
	case TX_DPD_RESET:
		ret = kstrtoul(buf, 0, &res);
		if (ret)
			break;
		ret = MYKONOS_resetDpd(phy->mykDevice,
				       chan->channel + 1, res);
		break;
	case TX_CLGC_DES_GAIN:
		ret = kstrtol(buf, 0, &res);
		if (ret)
			break;
		ret = MYKONOS_setClgcGain(phy->mykDevice,
				       chan->channel + 1, res);
		break;
	case TX_DPD_TRACKCNT:
	case TX_DPD_MODEL_ERR:
	case TX_DPD_EXT_PATH_DLY:
	case TX_CLGC_TRACKCNT:
	case TX_CLGC_CUR_GAIN:
	case TX_CLGC_TX_GAIN:
	case TX_CLGC_TX_RMS:
	case TX_CLGC_ORX_RMS:
	case TX_VSWR_TRACKCNT:
	case TX_VSWR_FW_GAIN:
	case TX_VSWR_FW_GAIN_REAL:
	case TX_VSWR_FW_GAIN_IMAG:
	case TX_VSWR_REF_GAIN:
	case TX_VSWR_REF_GAIN_REAL:
	case TX_VSWR_REF_GAIN_IMAG:
	case TX_VSWR_FW_TX:
	case TX_VSWR_FW_ORX:
	case TX_VSWR_REF_TX:
	case TX_VSWR_REF_ORX:
	case TX_DPD_STATUS:
	case TX_CLGC_STATUS:
	case TX_VSWR_STATUS:
		ret = -EINVAL;
		break;

	}

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

#define _AD9371_EXT_TX_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9371_phy_tx_read, \
	.write = ad9371_phy_tx_write, \
	.private = _ident, \
}

static const struct iio_chan_spec_ext_info ad9371_phy_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE("gain_control_mode", &ad9371_agc_modes_available),
	IIO_ENUM("gain_control_mode", false, &ad9371_agc_modes_available),
	_AD9371_EXT_RX_INFO("rssi", RSSI),
	_AD9371_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_AD9371_EXT_RX_INFO("temp_comp_gain", TEMPCOMP_GAIN),
	_AD9371_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	{ },
};

static const struct iio_chan_spec_ext_info ad9371_phy_obs_rx_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	IIO_ENUM_AVAILABLE("gain_control_mode", &ad9371_agc_modes_available),
	IIO_ENUM("gain_control_mode", false, &ad9371_agc_modes_available),
	IIO_ENUM_AVAILABLE("rf_port_select", &ad9371_rf_obs_rx_port_available),
	IIO_ENUM("rf_port_select", false, &ad9371_rf_obs_rx_port_available),
	_AD9371_EXT_RX_INFO("quadrature_tracking_en", RX_QEC),
	_AD9371_EXT_RX_INFO("rssi", RSSI),
	_AD9371_EXT_RX_INFO("temp_comp_gain", TEMPCOMP_GAIN),
	_AD9371_EXT_RX_INFO("rf_bandwidth", RX_RF_BANDWIDTH),
	{ },
};

static struct iio_chan_spec_ext_info ad9371_phy_tx_ext_info[] = {
	_AD9371_EXT_TX_INFO("quadrature_tracking_en", TX_QEC),
	_AD9371_EXT_TX_INFO("lo_leakage_tracking_en", TX_LOL),
	_AD9371_EXT_TX_INFO("rf_bandwidth", TX_RF_BANDWIDTH),
	/* Below here only AD9375 stuff */
	_AD9371_EXT_TX_INFO("dpd_tracking_en", TX_DPD),
	_AD9371_EXT_TX_INFO("clgc_tracking_en", TX_CLGC),
	_AD9371_EXT_TX_INFO("vswr_tracking_en", TX_VSWR),
	_AD9371_EXT_TX_INFO("dpd_actuator_en", TX_DPD_ACT_EN),
	_AD9371_EXT_TX_INFO("dpd_reset_en", TX_DPD_RESET),
	_AD9371_EXT_TX_INFO("dpd_track_count", TX_DPD_TRACKCNT),
	_AD9371_EXT_TX_INFO("dpd_model_error", TX_DPD_MODEL_ERR),
	_AD9371_EXT_TX_INFO("dpd_external_path_delay", TX_DPD_EXT_PATH_DLY),
	_AD9371_EXT_TX_INFO("dpd_status", TX_DPD_STATUS),
	_AD9371_EXT_TX_INFO("clgc_track_count", TX_CLGC_TRACKCNT),
	_AD9371_EXT_TX_INFO("clgc_desired_gain", TX_CLGC_DES_GAIN),
	_AD9371_EXT_TX_INFO("clgc_current_gain", TX_CLGC_CUR_GAIN),
	_AD9371_EXT_TX_INFO("clgc_tx_gain", TX_CLGC_TX_GAIN),
	_AD9371_EXT_TX_INFO("clgc_tx_rms", TX_CLGC_TX_RMS),
	_AD9371_EXT_TX_INFO("clgc_orx_rms", TX_CLGC_ORX_RMS),
	_AD9371_EXT_TX_INFO("clgc_status", TX_CLGC_STATUS),
	_AD9371_EXT_TX_INFO("vswr_track_count", TX_VSWR_TRACKCNT),
	_AD9371_EXT_TX_INFO("vswr_forward_gain", TX_VSWR_FW_GAIN),
	_AD9371_EXT_TX_INFO("vswr_forward_gain_real", TX_VSWR_FW_GAIN_REAL),
	_AD9371_EXT_TX_INFO("vswr_forward_gain_imag", TX_VSWR_FW_GAIN_IMAG),
	_AD9371_EXT_TX_INFO("vswr_reflected_gain", TX_VSWR_REF_GAIN),
	_AD9371_EXT_TX_INFO("vswr_reflected_gain_real", TX_VSWR_REF_GAIN_REAL),
	_AD9371_EXT_TX_INFO("vswr_reflected_gain_imag", TX_VSWR_REF_GAIN_IMAG),
	_AD9371_EXT_TX_INFO("vswr_forward_tx", TX_VSWR_FW_TX),
	_AD9371_EXT_TX_INFO("vswr_forward_orx", TX_VSWR_FW_ORX),
	_AD9371_EXT_TX_INFO("vswr_reflected_tx", TX_VSWR_REF_TX),
	_AD9371_EXT_TX_INFO("vswr_reflected_orx", TX_VSWR_REF_ORX),
	_AD9371_EXT_TX_INFO("vswr_status", TX_VSWR_STATUS),
	{ },
};

static int ad9371_gainindex_to_gain(struct ad9371_rf_phy *phy, int channel,
			     unsigned index, int *val, int *val2)
{
	int code;

	switch (channel) {
	case CHAN_RX1:
		if (phy->gt_info[RX1_RX2_GT].abs_gain_tbl) {
			code = phy->gt_info[RX1_RX2_GT].abs_gain_tbl[index];
			break;
		}

		if (phy->gt_info[RX1_GT].abs_gain_tbl) {
			code = phy->gt_info[RX1_GT].abs_gain_tbl[index];
			break;
		}

		code = MAX_RX_GAIN_mdB -
		       (phy->mykDevice->rx->rxGainCtrl->rx1MaxGainIndex - index) *
		       RX_GAIN_STEP_mdB;
		break;
	case CHAN_RX2:
		if (phy->gt_info[RX1_RX2_GT].abs_gain_tbl) {
			code = phy->gt_info[RX1_RX2_GT].abs_gain_tbl[index];
			break;
		}

		if (phy->gt_info[RX2_GT].abs_gain_tbl) {
			code = phy->gt_info[RX2_GT].abs_gain_tbl[index];
			break;
		}

		code = MAX_RX_GAIN_mdB -
		       (phy->mykDevice->rx->rxGainCtrl->rx2MaxGainIndex - index) *
		       RX_GAIN_STEP_mdB;
		break;
	case CHAN_OBS:
		if ((ad9371_obs_rx_port_lut[phy->obs_rx_path_source] & 0xF) == OBS_SNIFFER) {
			if (phy->gt_info[SNRX_GT].abs_gain_tbl) {
				code = phy->gt_info[SNRX_GT].abs_gain_tbl[index];
				break;
			}

			code = 52000 - (phy->mykDevice->obsRx->snifferGainCtrl->maxGainIndex - index) * 1000;
			break;
		}

		if (phy->gt_info[ORX_GT].abs_gain_tbl) {
			code = phy->gt_info[ORX_GT].abs_gain_tbl[index];
			break;
		}

		code = MAX_OBS_RX_GAIN_mdB -
		       (phy->mykDevice->obsRx->orxGainCtrl->maxGainIndex - index) *
		       OBS_RX_GAIN_STEP_mdB;
		break;
	default:
		return -EINVAL;

	}

	*val = code / 1000;
	*val2 = (code % 1000) * 1000;
	if (!*val)
		*val2 *= -1;

	return 0;
}

static int find_table_index(struct ad9371_rf_phy *phy, mykonosGainTable_t table, int gain)
{
	u32 i, nm1, n;

	for (i = 0; i < phy->gt_info[table].max_index; i++) {
		if (phy->gt_info[table].abs_gain_tbl[i] <= gain) {
			nm1 = abs(phy->gt_info[table].abs_gain_tbl[
				(i > 0) ? i - 1 : i] - gain);
			n = abs(phy->gt_info[table].abs_gain_tbl[i]
				- gain);
			if (nm1 < n)
				return (i > 0) ? i - 1 : i;
			else
				return i;
		}
	}

	return -EINVAL;
}

static int ad9371_gain_to_gainindex(struct ad9371_rf_phy *phy, int channel,
			     int val, int val2, unsigned *index)
{
	int ret, gain = ((abs(val) * 1000) + (abs(val2) / 1000));

	switch (channel) {
	case CHAN_RX1:
		if (phy->gt_info[RX1_RX2_GT].abs_gain_tbl) {
			ret = find_table_index(phy, RX1_RX2_GT, gain);
			if (ret >= 0) {
				*index = phy->mykDevice->rx->rxGainCtrl->rx1MaxGainIndex - ret;
				break;
			}
		}

		if (phy->gt_info[RX1_GT].abs_gain_tbl) {
			ret = find_table_index(phy, RX1_GT, gain);
			if (ret >= 0) {
				*index = phy->mykDevice->rx->rxGainCtrl->rx1MaxGainIndex - ret;
				break;
			}
		}

		gain = clamp(gain, MIN_GAIN_mdB, MAX_RX_GAIN_mdB);
		*index = (gain - MAX_RX_GAIN_mdB) / RX_GAIN_STEP_mdB +
			 phy->mykDevice->rx->rxGainCtrl->rx1MaxGainIndex;
		break;

	case CHAN_RX2:
		if (phy->gt_info[RX1_RX2_GT].abs_gain_tbl) {
			ret = find_table_index(phy, RX1_RX2_GT, gain);
			if (ret >= 0) {
				*index = phy->mykDevice->rx->rxGainCtrl->rx2MaxGainIndex - ret;
				break;
			}
		}

		if (phy->gt_info[RX2_GT].abs_gain_tbl) {
			ret = find_table_index(phy, RX1_GT, gain);
			if (ret >= 0) {
				*index = phy->mykDevice->rx->rxGainCtrl->rx2MaxGainIndex - ret;
				break;
			}
		}

		gain = clamp(gain, MIN_GAIN_mdB, MAX_RX_GAIN_mdB);
		*index = (gain - MAX_RX_GAIN_mdB) / RX_GAIN_STEP_mdB +
			 phy->mykDevice->rx->rxGainCtrl->rx2MaxGainIndex;
		break;
	case CHAN_OBS:
		if ((ad9371_obs_rx_port_lut[phy->obs_rx_path_source] & 0xF) == OBS_SNIFFER) {
			if (phy->gt_info[SNRX_GT].abs_gain_tbl) {
				ret = find_table_index(phy, SNRX_GT, gain);
				if (ret >= 0) {
					*index = phy->mykDevice->obsRx->snifferGainCtrl->maxGainIndex - ret;
					break;
				}
			}

			gain = clamp(gain, MIN_GAIN_mdB, MAX_OBS_SNRX_GAIN_mdB);
			*index = (gain - MAX_OBS_SNRX_GAIN_mdB) / SNRX_GAIN_STEP_mdB +
				phy->mykDevice->obsRx->snifferGainCtrl->maxGainIndex;
			break;
		}

		if (phy->gt_info[ORX_GT].abs_gain_tbl) {
			ret = find_table_index(phy, ORX_GT, gain);
			if (ret >= 0) {
				*index = phy->mykDevice->obsRx->orxGainCtrl->maxGainIndex - ret;
				break;
			}
		}

		gain = clamp(gain, MIN_GAIN_mdB, MAX_OBS_RX_GAIN_mdB);
		*index = (gain - MAX_OBS_RX_GAIN_mdB) / OBS_RX_GAIN_STEP_mdB +
			 phy->mykDevice->obsRx->orxGainCtrl->maxGainIndex;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9371_read_temperature(struct ad9371_rf_phy *phy, int *val)
{
	int ret, i;
	mykonosTempSensorStatus_t tstat = { 0 };
	mykonosTempSensorConfig_t tconf = { .tempDecimation = 7 };

	ret = MYKONOS_setupTempSensor(phy->mykDevice, &tconf);
	if (ret != MYKONOS_ERR_GPIO_OK)
		return -EIO;

	ret = MYKONOS_setAuxAdcChannel(phy->mykDevice, MYK_TEMPSENSOR);
	if (ret != MYKONOS_ERR_GPIO_OK)
		return -EIO;

	ret = MYKONOS_startTempMeasurement(phy->mykDevice);
	if (ret != MYKONOS_ERR_GPIO_OK)
		return -EIO;

	for (i = 0; i < 3; i++) {
		msleep(2);
		ret = MYKONOS_readTempSensor(phy->mykDevice, &tstat);
		if (ret != MYKONOS_ERR_GPIO_OK)
			return -EIO;
		if (tstat.tempValid) {
			*val = tstat.tempCode;
			return 0;
		}
	}

	return -EIO;
}

static int ad9371_phy_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val,
			       int *val2,
			       long m)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret;


	mutex_lock(&indio_dev->mlock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			u16 atten_mdb;
			switch (chan->channel) {
			case CHAN_TX1:
				MYKONOS_getTx1Attenuation(phy->mykDevice, &atten_mdb);
				break;
			case CHAN_TX2:
				MYKONOS_getTx2Attenuation(phy->mykDevice, &atten_mdb);
				break;
			}

			*val = -1 * (atten_mdb / 1000);
			*val2 = (atten_mdb % 1000) * 1000;
			if (!*val)
				*val2 *= -1;

		} else {
			u8 index;

			switch (chan->channel) {
			case CHAN_RX1:
				MYKONOS_getRx1Gain(phy->mykDevice, &index);
				break;
			case CHAN_RX2:
				MYKONOS_getRx2Gain(phy->mykDevice, &index);
				break;
			case CHAN_OBS:
				MYKONOS_getObsRxGain(phy->mykDevice, &index);
				break;
			}

			ret = ad9371_gainindex_to_gain(phy, chan->channel,
						       index, val, val2);
		}
		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->output) {
			*val = clk_get_rate(phy->clks[TX_SAMPL_CLK]);
		} else {
			switch (chan->channel) {
			case CHAN_RX1:
			case CHAN_RX2:
				*val = clk_get_rate(phy->clks[RX_SAMPL_CLK]);
				break;
			case CHAN_OBS:
				*val = clk_get_rate(phy->clks[OBS_SAMPL_CLK]);
				break;
			}
		}

		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_TEMP) {
			ret = ad9371_read_temperature(phy, val);
			if (ret == 0)
				ret = IIO_VAL_INT;
		} else if (chan->output) {
			*val = phy->mykDevice->auxIo->auxDacValue[chan->channel - CHAN_AUXDAC0];
			ret = IIO_VAL_INT;
		} else {
			u16 adcCode;
			MYKONOS_setAuxAdcChannel(phy->mykDevice, chan->channel - CHAN_AUXADC0);
			ret = MYKONOS_readAuxAdc(phy->mykDevice, &adcCode);
			if (ret == 0) {
				*val = adcCode;
				ret = IIO_VAL_INT;
			}
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->output) {
			*val = ad9371_auxdac_offset_val1_lut
			       [phy->mykDevice->auxIo->auxDacSlope[chan->channel - CHAN_AUXDAC0]]
			       [phy->mykDevice->auxIo->auxDacVref[chan->channel - CHAN_AUXDAC0]];  /* AuxDAC */
			ret = IIO_VAL_INT;
		} else {
			*val = 45; /* AuxADC */
			ret = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			*val = 1000 ; /* Temperature scale */
			*val2 = 0;
		} else if (chan->output) {
			*val = ad9371_auxdac_scale_val1_lut
			       [phy->mykDevice->auxIo->auxDacSlope[chan->channel - CHAN_AUXDAC0]]; /* AuxDAC */
			*val2 = ad9371_auxdac_scale_val2_lut
				[phy->mykDevice->auxIo->auxDacSlope[chan->channel - CHAN_AUXDAC0]]
				[phy->mykDevice->auxIo->auxDacVref[chan->channel - CHAN_AUXDAC0]];  /* AuxDAC */
		} else {
			*val = 0; /* AuxADC */
			*val2 = 775194;
		}

		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
};

static int ad9371_phy_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val,
				int val2,
				long mask)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	u32 code;
	int ret;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->output) {
			if (val > 0 || (val == 0 && val2 > 0)) {
				ret = -EINVAL;
				goto out;
			}

			code = ((abs(val) * 1000) + (abs(val2) / 1000));

			if (chan->channel == CHAN_TX1)
				ret = MYKONOS_setTx1Attenuation(phy->mykDevice, code);
			else
				ret = MYKONOS_setTx2Attenuation(phy->mykDevice, code);
		} else {

			ret = ad9371_gain_to_gainindex(phy, chan->channel,
						       val, val2, &code);
			if (ret < 0)
				break;

			switch (chan->channel) {
			case CHAN_RX1:
				MYKONOS_setRx1ManualGain(phy->mykDevice, code);
				break;
			case CHAN_RX2:
				MYKONOS_setRx2ManualGain(phy->mykDevice, code);
				break;
			case CHAN_OBS:
				MYKONOS_setObsRxManualGain(phy->mykDevice,
					ad9371_obs_rx_port_lut[phy->obs_rx_path_source],
					code);
				break;
			default:
				ret = -EINVAL;
			}
		}
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = -ENOTSUPP;
		break;
	case IIO_CHAN_INFO_RAW:
		if (chan->output) {
			if (phy->mykDevice->auxIo->auxDacEnable & BIT(chan->channel - CHAN_AUXDAC0)) {
				ret = MYKONOS_writeAuxDac(phy->mykDevice, chan->channel - CHAN_AUXDAC0, val);
				if (ret != MYKONOS_ERR_GPIO_OK)
					ret = -EINVAL;
			} else {
				ret = -ENODEV;
			}
		}

		ret = -ENOTSUPP;
		break;
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}



static const struct iio_chan_spec ad9371_phy_chan[] = {
	{	/* RX LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.extend_name = "RX_LO",
		.ext_info = ad9371_phy_ext_info,
	}, {	/* TX LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "TX_LO",
		.ext_info = ad9371_phy_ext_info,
	}, {	/* RX Sniffer/Observation LO */
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 2,
		.extend_name = "RX_SN_LO",
		.ext_info = ad9371_phy_ext_info,
	}, {	/* TX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9371_phy_tx_ext_info,
	}, {	/* RX1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
//	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9371_phy_rx_ext_info,
	}, {	/* TX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9371_phy_tx_ext_info,
	}, {	/* RX2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_RX2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
//	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9371_phy_rx_ext_info,
	}, {	/* RX Sniffer/Observation */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_OBS,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_SAMP_FREQ),
//	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9371_phy_obs_rx_ext_info,
	}, {	/* AUXADC0 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_AUXADC0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXADC1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_AUXADC1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXADC2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_AUXADC2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXADC3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = CHAN_AUXADC3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC0 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC1 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC2 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC3 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC4 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC4,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC5 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC5,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC6 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC6,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC7 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC7,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC8 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC8,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* AUXDAC9 */
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = CHAN_AUXDAC9,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	}, {	/* TEMP */
		.type = IIO_TEMP,
		.indexed = 1,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_info ad9371_phy_info = {
	.read_raw = &ad9371_phy_read_raw,
	.write_raw = &ad9371_phy_write_raw,
	.debugfs_reg_access = &ad9371_phy_reg_access,
	.attrs = &ad9371_phy_attribute_group,
};

static const struct iio_info ad9375_phy_info = {
	.read_raw = &ad9371_phy_read_raw,
	.write_raw = &ad9371_phy_write_raw,
	.debugfs_reg_access = &ad9371_phy_reg_access,
	.attrs = &ad9375_phy_attribute_group,
};

#ifdef CONFIG_OF
static ssize_t ad9371_debugfs_read(struct file *file, char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	struct ad9371_debugfs_entry *entry = file->private_data;
	struct ad9371_rf_phy *phy = entry->phy;
	char buf[700];
	u64 val = 0;
	ssize_t len = 0;
	int ret;

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			val = *(u8*)entry->out_value;
			break;
		case 2:
			val = *(u16*)entry->out_value;
			break;
		case 4:
			val = *(u32*)entry->out_value;
			break;
		case 5:
			val = *(bool*)entry->out_value;
			break;
		case 8:
			val = *(u64*)entry->out_value;
			break;
		default:
			return -EINVAL;
		}

	} else if (entry->cmd) {
		u8 index, mask, status;
		u32 errcnt;

		switch (entry->cmd) {
		case DBGFS_BIST_PRBS_ERR_TX:
			for (index = 0; index < 4; index++) {
				mutex_lock(&phy->indio_dev->mlock);
				ret = MYKONOS_readDeframerPrbsCounters(phy->mykDevice,
								       index, &errcnt);
				mutex_unlock(&phy->indio_dev->mlock);
				if (ret < 0)
					return ret;

				len += snprintf(buf + len, sizeof(buf) - len,
						"0x%08x%c", errcnt,
						index == 3 ? '\n' : ' ');
			}
			break;
		case DBGFS_MONITOR_OUT:
			mutex_lock(&phy->indio_dev->mlock);
			ret = MYKONOS_getGpioMonitorOut(phy->mykDevice,
							&index, &mask);
			mutex_unlock(&phy->indio_dev->mlock);
			if (ret < 0)
				return ret;

			len = snprintf(buf, sizeof(buf), "%u %u\n",
				       index, mask);
			break;
		case DBGFS_PLLS_STATUS:
			mutex_lock(&phy->indio_dev->mlock);
			ret = MYKONOS_checkPllsLockStatus(phy->mykDevice, &status);
			mutex_unlock(&phy->indio_dev->mlock);
			if (ret < 0)
				return ret;

			len = snprintf(buf, sizeof(buf), "0x%02x\n",
				       status);
			break;
		default:
			val = entry->val;
			break;
		}

	} else
		return -EFAULT;

	if (!len)
		len = snprintf(buf, sizeof(buf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t ad9371_debugfs_write(struct file *file,
				    const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct ad9371_debugfs_entry *entry = file->private_data;
	struct ad9371_rf_phy *phy = entry->phy;
	u32 val2, val3, val4;
	s64 val;
	char buf[80];
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%lld %i %i %i", &val, &val2, &val3, &val4);
	if (ret < 1)
		return -EINVAL;


	switch (entry->cmd) {
	case DBGFS_INIT:
		if (!(ret == 1 && val == 1))
			return -EINVAL;
		mutex_lock(&phy->indio_dev->mlock);
		ret = ad9371_reinit(phy);
		mutex_unlock(&phy->indio_dev->mlock);

		return count;
	case DBGFS_LOOPBACK_TX_RX:
		if (ret != 1)
			return -EINVAL;
		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_setRxFramerDataSource(phy->mykDevice, val);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_LOOPBACK_TX_OBS:
		if (ret != 1)
			return -EINVAL;
		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_setObsRxFramerDataSource(phy->mykDevice, val);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_RX:
		if (ret != 1)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_enableRxFramerPrbs(phy->mykDevice,
						 (val > 0) ? val - 1 : 0, !!val);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_ERR_RX:
		if (ret != 1)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_rxInjectPrbsError(phy->mykDevice);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_OBS:
		if (ret != 1)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_enableObsRxFramerPrbs(phy->mykDevice,
						    (val > 0) ? val - 1 : 0, !!val);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_ERR_OBS:
		if (ret != 1)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_obsRxInjectPrbsError(phy->mykDevice);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_TX:
		if (ret != 2)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_enableDeframerPrbsChecker(phy->mykDevice, val,
							(val2 > 0) ? val2 - 1 : 0,
							!!val2);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_PRBS_ERR_TX:
		if (ret != 1)
			return -EINVAL;

		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_clearDeframerPrbsCounters(phy->mykDevice);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_BIST_TONE:
		if (ret != 3)
			return -EINVAL;
		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_enableTxNco(phy->mykDevice, val, val2, val3);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		entry->val = val;
		return count;
	case DBGFS_MONITOR_OUT:
		if (ret != 2)
			return -EINVAL;
		mutex_lock(&phy->indio_dev->mlock);
		ret = MYKONOS_setGpioMonitorOut(phy->mykDevice, val, val2);
		mutex_unlock(&phy->indio_dev->mlock);
		if (ret < 0)
			return ret;

		return count;
	default:
		break;
	}

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			*(u8*)entry->out_value = val;
			break;
		case 2:
			*(u16*)entry->out_value = val;
			break;
		case 4:
			*(u32*)entry->out_value = val;
			break;
		case 5:
			*(bool*)entry->out_value = val;
			break;
		case 8:
			*(u64*)entry->out_value = val;
			break;
		default:
			ret = -EINVAL;
		}
	}

	return count;
}

static const struct file_operations ad9371_debugfs_reg_fops = {
	.open = simple_open,
	.read = ad9371_debugfs_read,
	.write = ad9371_debugfs_write,
};

static void ad9371_add_debugfs_entry(struct ad9371_rf_phy *phy,
				     const char *propname, unsigned int cmd)
{
	unsigned int i = phy->ad9371_debugfs_entry_index;

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].phy = phy;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->ad9371_debugfs_entry_index++;
}

static int ad9371_register_debugfs(struct iio_dev *indio_dev)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	struct dentry *d;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return -ENODEV;

	ad9371_add_debugfs_entry(phy, "initialize", DBGFS_INIT);
	ad9371_add_debugfs_entry(phy, "loopback_tx_rx", DBGFS_LOOPBACK_TX_RX);
	ad9371_add_debugfs_entry(phy, "loopback_tx_obs", DBGFS_LOOPBACK_TX_OBS);
	ad9371_add_debugfs_entry(phy, "bist_prbs_rx", DBGFS_BIST_PRBS_RX);
	ad9371_add_debugfs_entry(phy, "bist_prbs_err_rx", DBGFS_BIST_PRBS_ERR_RX);
	ad9371_add_debugfs_entry(phy, "bist_prbs_obs", DBGFS_BIST_PRBS_OBS);
	ad9371_add_debugfs_entry(phy, "bist_prbs_err_obs", DBGFS_BIST_PRBS_ERR_OBS);
	ad9371_add_debugfs_entry(phy, "bist_prbs_tx", DBGFS_BIST_PRBS_TX);
	ad9371_add_debugfs_entry(phy, "bist_prbs_err_tx", DBGFS_BIST_PRBS_ERR_TX);
	ad9371_add_debugfs_entry(phy, "bist_tone", DBGFS_BIST_TONE);
	ad9371_add_debugfs_entry(phy, "monitor_out", DBGFS_MONITOR_OUT);
	ad9371_add_debugfs_entry(phy, "plls_lock_status", DBGFS_PLLS_STATUS);

	for (i = 0; i < phy->ad9371_debugfs_entry_index; i++)
		d = debugfs_create_file(
			    phy->debugfs_entry[i].propname, 0644,
			    iio_get_debugfs_dentry(indio_dev),
			    &phy->debugfs_entry[i],
			    &ad9371_debugfs_reg_fops);
	return 0;
}

static int __ad9371_of_get_u32(struct iio_dev *indio_dev,
			       struct device_node *np, const char *propname,
			       s64 defval, void *out_value, u32 size)
{
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	u32 tmp;
	u64 tmp64;
	int ret;

	if (size == 8) {
		tmp64 = defval;
		ret = of_property_read_u64(np, propname, &tmp64);
	} else {
		tmp = defval;
		ret = of_property_read_u32(np, propname, &tmp);
	}

	if (out_value) {
		switch (size) {
		case 1:
			*(u8*)out_value = tmp;
			break;
		case 2:
			*(u16*)out_value = tmp;
			break;
		case 4:
			*(u32*)out_value = tmp;
			break;
		case 8:
			*(u64*)out_value = tmp64;
			break;
		default:
			ret = -EINVAL;
		}
	}

	if (WARN_ON(phy->ad9371_debugfs_entry_index >=
		    ARRAY_SIZE(phy->debugfs_entry)))
		return ret;

	phy->debugfs_entry[phy->ad9371_debugfs_entry_index++] =
	(struct ad9371_debugfs_entry) {
		.out_value = out_value,
		 .propname = propname,
		  .size = size,
		   .phy = phy,
	};

	return ret;
}
#define ad9371_of_get_u32(iodev, dnp, name, def, outp) \
	__ad9371_of_get_u32(iodev, dnp, name, def, outp, sizeof(*outp))

static struct ad9371_phy_platform_data
*ad9371_phy_parse_dt(struct iio_dev *iodev, struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9371_phy_platform_data *pdata;
	struct ad9371_rf_phy *phy = iio_priv(iodev);
	int ret;

#define ad9371_of_get_u32(iodev, dnp, name, def, outp) \
	__ad9371_of_get_u32(iodev, dnp, name, def, outp, sizeof(*outp))

#define AD9371_OF_PROP(_dt_name, _member_, _default) \
	__ad9371_of_get_u32(iodev, np, _dt_name, _default, _member_, sizeof(*_member_))

#define AD9371_GET_FIR(_dt_base_name, _member) \
	if (of_find_property(np, _dt_base_name"-coefs", NULL)) {\
	AD9371_OF_PROP(_dt_base_name"-gain_db", &_member->gain_dB, 0); \
	AD9371_OF_PROP(_dt_base_name"-num-fir-coefs", &_member->numFirCoefs, 0); \
	ret = of_property_read_u16_array(np, _dt_base_name"-coefs", _member->coefs, _member->numFirCoefs); \
	if (ret < 0) { \
		dev_err(dev, "Failed to read %d FIR coefficients (%d)\n", _member->numFirCoefs, ret); \
		return NULL; \
	} } \

#define AD9371_GET_PROFILE(_dt_name, _member) \
	if (of_find_property(np, _dt_name, NULL)) {\
	if (_member == NULL) { \
		_member = devm_kzalloc(dev, 37 * sizeof(u16), GFP_KERNEL); \
	} \
	ret = of_property_read_u16_array(np, _dt_name, _member, 16); \
	if (ret < 0) { \
		dev_err(dev, "Failed to read %d coefficients\n", 16); \
		return NULL; \
	} } \

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	AD9371_OF_PROP("adi,default-initial-calibrations-mask", &phy->init_cal_mask,
			TX_BB_FILTER | ADC_TUNER | TIA_3DB_CORNER | DC_OFFSET |
			TX_ATTENUATION_DELAY | RX_GAIN_DELAY | FLASH_CAL |
			PATH_DELAY | TX_LO_LEAKAGE_INTERNAL | TX_QEC_INIT |
			LOOPBACK_RX_LO_DELAY | LOOPBACK_RX_RX_QEC_INIT |
			RX_LO_DELAY | RX_QEC_INIT |
			(IS_AD9375(phy) ? DPD_INIT | CLGC_INIT | VSWR_INIT: 0));

	AD9371_OF_PROP("adi,jesd204-rx-framer-bank-id", &phy->mykDevice->rx->framer->bankId, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-device-id", &phy->mykDevice->rx->framer->deviceId, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-lane0-id", &phy->mykDevice->rx->framer->lane0Id, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-m", &phy->mykDevice->rx->framer->M, 4);
	AD9371_OF_PROP("adi,jesd204-rx-framer-k", &phy->mykDevice->rx->framer->K, 32);
	AD9371_OF_PROP("adi,jesd204-rx-framer-scramble", &phy->mykDevice->rx->framer->scramble, 1);
	AD9371_OF_PROP("adi,jesd204-rx-framer-external-sysref", &phy->mykDevice->rx->framer->externalSysref, 1);
	AD9371_OF_PROP("adi,jesd204-rx-framer-serializer-lanes-enabled", &phy->mykDevice->rx->framer->serializerLanesEnabled, 3);
	AD9371_OF_PROP("adi,jesd204-rx-framer-serializer-lane-crossbar", &phy->mykDevice->rx->framer->serializerLaneCrossbar, 0xe4);
	AD9371_OF_PROP("adi,jesd204-rx-framer-serializer-amplitude", &phy->mykDevice->rx->framer->serializerAmplitude, 22);
	AD9371_OF_PROP("adi,jesd204-rx-framer-pre-emphasis", &phy->mykDevice->rx->framer->preEmphasis, 4);
	AD9371_OF_PROP("adi,jesd204-rx-framer-invert-lane-polarity", &phy->mykDevice->rx->framer->invertLanePolarity, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-lmfc-offset", &phy->mykDevice->rx->framer->lmfcOffset, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-new-sysref-on-relink", &phy->mykDevice->rx->framer->newSysrefOnRelink, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-enable-auto-chan-xbar", &phy->mykDevice->rx->framer->enableAutoChanXbar, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-obs-rx-syncb-select", &phy->mykDevice->rx->framer->obsRxSyncbSelect, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-rx-syncb-mode", &phy->mykDevice->rx->framer->rxSyncbMode, 0);
	AD9371_OF_PROP("adi,jesd204-rx-framer-over-sample", &phy->mykDevice->rx->framer->overSample, 0);

	AD9371_OF_PROP("adi,jesd204-obs-framer-bank-id", &phy->mykDevice->obsRx->framer->bankId, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-device-id", &phy->mykDevice->obsRx->framer->deviceId, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-lane0-id", &phy->mykDevice->obsRx->framer->lane0Id, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-m", &phy->mykDevice->obsRx->framer->M, 2);
	AD9371_OF_PROP("adi,jesd204-obs-framer-k", &phy->mykDevice->obsRx->framer->K, 32);
	AD9371_OF_PROP("adi,jesd204-obs-framer-scramble", &phy->mykDevice->obsRx->framer->scramble, 1);
	AD9371_OF_PROP("adi,jesd204-obs-framer-external-sysref", &phy->mykDevice->obsRx->framer->externalSysref, 1);
	AD9371_OF_PROP("adi,jesd204-obs-framer-serializer-lanes-enabled", &phy->mykDevice->obsRx->framer->serializerLanesEnabled, 0xc);
	AD9371_OF_PROP("adi,jesd204-obs-framer-serializer-lane-crossbar", &phy->mykDevice->obsRx->framer->serializerLaneCrossbar, 0xe4);
	AD9371_OF_PROP("adi,jesd204-obs-framer-serializer-amplitude", &phy->mykDevice->obsRx->framer->serializerAmplitude, 22);
	AD9371_OF_PROP("adi,jesd204-obs-framer-pre-emphasis", &phy->mykDevice->obsRx->framer->preEmphasis, 4);
	AD9371_OF_PROP("adi,jesd204-obs-framer-invert-lane-polarity", &phy->mykDevice->obsRx->framer->invertLanePolarity, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-lmfc-offset", &phy->mykDevice->obsRx->framer->lmfcOffset, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-new-sysref-on-relink", &phy->mykDevice->obsRx->framer->newSysrefOnRelink, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-enable-auto-chan-xbar", &phy->mykDevice->obsRx->framer->enableAutoChanXbar, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-obs-rx-syncb-select", &phy->mykDevice->obsRx->framer->obsRxSyncbSelect, 1);
	AD9371_OF_PROP("adi,jesd204-obs-framer-rx-syncb-mode", &phy->mykDevice->obsRx->framer->rxSyncbMode, 0);
	AD9371_OF_PROP("adi,jesd204-obs-framer-over-sample", &phy->mykDevice->obsRx->framer->overSample, 0);

	AD9371_OF_PROP("adi,jesd204-deframer-bank-id", &phy->mykDevice->tx->deframer->bankId, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-device-id", &phy->mykDevice->tx->deframer->deviceId, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-lane0-id", &phy->mykDevice->tx->deframer->lane0Id, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-m", &phy->mykDevice->tx->deframer->M, 4);
	AD9371_OF_PROP("adi,jesd204-deframer-k", &phy->mykDevice->tx->deframer->K, 32);
	AD9371_OF_PROP("adi,jesd204-deframer-scramble", &phy->mykDevice->tx->deframer->scramble, 1);
	AD9371_OF_PROP("adi,jesd204-deframer-external-sysref", &phy->mykDevice->tx->deframer->externalSysref, 1);
	AD9371_OF_PROP("adi,jesd204-deframer-deserializer-lanes-enabled", &phy->mykDevice->tx->deframer->deserializerLanesEnabled, 0x0F);
	AD9371_OF_PROP("adi,jesd204-deframer-deserializer-lane-crossbar", &phy->mykDevice->tx->deframer->deserializerLaneCrossbar, 0xE4);
	AD9371_OF_PROP("adi,jesd204-deframer-eq-setting", &phy->mykDevice->tx->deframer->EQSetting, 1);
	AD9371_OF_PROP("adi,jesd204-deframer-invert-lane-polarity", &phy->mykDevice->tx->deframer->invertLanePolarity, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-lmfc-offset", &phy->mykDevice->tx->deframer->lmfcOffset, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-new-sysref-on-relink", &phy->mykDevice->tx->deframer->newSysrefOnRelink, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-enable-auto-chan-xbar", &phy->mykDevice->tx->deframer->enableAutoChanXbar, 0);
	AD9371_OF_PROP("adi,jesd204-deframer-tx-syncb-mode", &phy->mykDevice->tx->deframer->txSyncbMode, 0);

	AD9371_OF_PROP("adi,rx-gain-mode", &phy->mykDevice->rx->rxGainCtrl->gainMode, 0);
	AD9371_OF_PROP("adi,rx1-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx1GainIndex, 255);
	AD9371_OF_PROP("adi,rx2-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx2GainIndex, 255);
	AD9371_OF_PROP("adi,rx1-max-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx1MaxGainIndex, 255);
	AD9371_OF_PROP("adi,rx1-min-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx1MinGainIndex, 195);
	AD9371_OF_PROP("adi,rx2-max-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx2MaxGainIndex, 255);
	AD9371_OF_PROP("adi,rx2-min-gain-index", &phy->mykDevice->rx->rxGainCtrl->rx2MinGainIndex, 195);

	AD9371_OF_PROP("adi,orx-gain-mode", &phy->mykDevice->obsRx->orxGainCtrl->gainMode, 0);
	AD9371_OF_PROP("adi,orx1-gain-index", &phy->mykDevice->obsRx->orxGainCtrl->orx1GainIndex, 255);
	AD9371_OF_PROP("adi,orx2-gain-index", &phy->mykDevice->obsRx->orxGainCtrl->orx2GainIndex, 255);
	AD9371_OF_PROP("adi,orx-max-gain-index", &phy->mykDevice->obsRx->orxGainCtrl->maxGainIndex, 255);
	AD9371_OF_PROP("adi,orx-min-gain-index", &phy->mykDevice->obsRx->orxGainCtrl->minGainIndex, 237);

	AD9371_OF_PROP("adi,sniffer-gain-mode", &phy->mykDevice->obsRx->snifferGainCtrl->gainMode, 0);
	AD9371_OF_PROP("adi,sniffer-gain-index", &phy->mykDevice->obsRx->snifferGainCtrl->gainIndex, 255);
	AD9371_OF_PROP("adi,sniffer-max-gain-index", &phy->mykDevice->obsRx->snifferGainCtrl->maxGainIndex, 255);
	AD9371_OF_PROP("adi,sniffer-min-gain-index", &phy->mykDevice->obsRx->snifferGainCtrl->minGainIndex, 203);

	AD9371_OF_PROP("adi,rx-peak-agc-apd-high-thresh", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdHighThresh, 31);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-low-thresh", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdLowThresh, 22);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-high-thresh", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2HighThresh, 181);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-low-thresh", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2LowThresh, 128);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-very-low-thresh", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2VeryLowThresh, 64);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-high-thresh-exceeded-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdHighThreshExceededCnt, 6);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-low-thresh-exceeded-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdLowThreshExceededCnt, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-high-thresh-exceeded-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2HighThreshExceededCnt, 6);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-low-thresh-exceeded-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2LowThreshExceededCnt, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-very-low-thresh-exceeded-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2VeryLowThreshExceededCnt, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-high-gain-step-attack", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdHighGainStepAttack, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-low-gain-step-recovery", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdLowGainStepRecovery, 2);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-high-gain-step-attack", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2HighGainStepAttack, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-low-gain-step-recovery", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2LowGainStepRecovery, 2);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-very-low-gain-step-recovery", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2VeryLowGainStepRecovery, 4);
	AD9371_OF_PROP("adi,rx-peak-agc-apd-fast-attack", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->apdFastAttack, 1);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-fast-attack", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2FastAttack, 1);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-overload-detect-enable", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2OverloadDetectEnable, 1);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-overload-duration-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2OverloadDurationCnt, 1);
	AD9371_OF_PROP("adi,rx-peak-agc-hb2-overload-thresh-cnt", &phy->mykDevice->rx->rxAgcCtrl->peakAgc->hb2OverloadThreshCnt, 1);

	AD9371_OF_PROP("adi,obs-peak-agc-apd-high-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdHighThresh, 42);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-low-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdLowThresh, 22);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-high-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2HighThresh, 181);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-low-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2LowThresh, 128);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-very-low-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2VeryLowThresh, 64);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-high-thresh-exceeded-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdHighThreshExceededCnt, 3);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-low-thresh-exceeded-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdLowThreshExceededCnt, 3);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-high-thresh-exceeded-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2HighThreshExceededCnt, 3);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-low-thresh-exceeded-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2LowThreshExceededCnt, 3);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-very-low-thresh-exceeded-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2VeryLowThreshExceededCnt, 3);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-high-gain-step-attack", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdHighGainStepAttack, 4);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-low-gain-step-recovery", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdLowGainStepRecovery, 2);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-high-gain-step-attack", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2HighGainStepAttack, 4);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-low-gain-step-recovery", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2LowGainStepRecovery, 2);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-very-low-gain-step-recovery", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2VeryLowGainStepRecovery, 4);
	AD9371_OF_PROP("adi,obs-peak-agc-apd-fast-attack", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->apdFastAttack, 0);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-fast-attack", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2FastAttack, 0);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-overload-detect-enable", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2OverloadDetectEnable, 1);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-overload-duration-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2OverloadDurationCnt, 1);
	AD9371_OF_PROP("adi,obs-peak-agc-hb2-overload-thresh-cnt", &phy->mykDevice->obsRx->orxAgcCtrl->peakAgc->hb2OverloadThreshCnt, 1);

	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-upper-high-thresh", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdUpperHighThresh, 1);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-upper-low-thresh", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdUpperLowThresh, 3);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-lower-high-thresh", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdLowerHighThresh, 12);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-lower-low-thresh", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdLowerLowThresh, 4);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-upper-high-gain-step-attack", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdUpperHighGainStepAttack, 4);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-upper-low-gain-step-attack", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdUpperLowGainStepAttack, 2);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-lower-high-gain-step-recovery", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdLowerHighGainStepRecovery, 2);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-lower-low-gain-step-recovery", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdLowerLowGainStepRecovery, 4);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-meas-duration", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdMeasDuration, 8);
	AD9371_OF_PROP("adi,rx-pwr-agc-pmd-meas-config", &phy->mykDevice->rx->rxAgcCtrl->powerAgc->pmdMeasConfig, 2);

	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-upper-high-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdUpperHighThresh, 1);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-upper-low-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdUpperLowThresh, 3);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-lower-high-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdLowerHighThresh, 12);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-lower-low-thresh", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdLowerLowThresh, 4);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-upper-high-gain-step-attack", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdUpperHighGainStepAttack, 0);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-upper-low-gain-step-attack", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdUpperLowGainStepAttack, 0);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-lower-high-gain-step-recovery", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdLowerHighGainStepRecovery, 0);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-lower-low-gain-step-recovery", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdLowerLowGainStepRecovery, 0);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-meas-duration", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdMeasDuration, 8);
	AD9371_OF_PROP("adi,obs-pwr-agc-pmd-meas-config", &phy->mykDevice->obsRx->orxAgcCtrl->powerAgc->pmdMeasConfig, 2);

	AD9371_OF_PROP("adi,rx-agc-conf-agc-rx1-max-gain-index", &phy->mykDevice->rx->rxAgcCtrl->agcRx1MaxGainIndex, 255);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-rx1-min-gain-index", &phy->mykDevice->rx->rxAgcCtrl->agcRx1MinGainIndex, 195);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-rx2-max-gain-index", &phy->mykDevice->rx->rxAgcCtrl->agcRx2MaxGainIndex, 255);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-rx2-min-gain-index", &phy->mykDevice->rx->rxAgcCtrl->agcRx2MinGainIndex, 195);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-peak-threshold-mode", &phy->mykDevice->rx->rxAgcCtrl->agcPeakThresholdMode, 1);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-low-ths-prevent-gain-increase", &phy->mykDevice->rx->rxAgcCtrl->agcLowThsPreventGainIncrease, 1);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-gain-update-counter", &phy->mykDevice->rx->rxAgcCtrl->agcGainUpdateCounter, 30720);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-slow-loop-settling-delay", &phy->mykDevice->rx->rxAgcCtrl->agcSlowLoopSettlingDelay, 3);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-peak-wait-time", &phy->mykDevice->rx->rxAgcCtrl->agcPeakWaitTime, 2);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-reset-on-rx-enable", &phy->mykDevice->rx->rxAgcCtrl->agcResetOnRxEnable, 0);
	AD9371_OF_PROP("adi,rx-agc-conf-agc-enable-sync-pulse-for-gain-counter", &phy->mykDevice->rx->rxAgcCtrl->agcEnableSyncPulseForGainCounter, 0);

	AD9371_OF_PROP("adi,obs-agc-conf-agc-obs-rx-max-gain-index", &phy->mykDevice->obsRx->orxAgcCtrl->agcObsRxMaxGainIndex, 255);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-obs-rx-min-gain-index", &phy->mykDevice->obsRx->orxAgcCtrl->agcObsRxMinGainIndex, 203);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-obs-rx-select", &phy->mykDevice->obsRx->orxAgcCtrl->agcObsRxSelect, 1);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-peak-threshold-mode", &phy->mykDevice->obsRx->orxAgcCtrl->agcPeakThresholdMode, 1);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-low-ths-prevent-gain-increase", &phy->mykDevice->obsRx->orxAgcCtrl->agcLowThsPreventGainIncrease, 1);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-gain-update-counter", &phy->mykDevice->obsRx->orxAgcCtrl->agcGainUpdateCounter, 30720);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-slow-loop-settling-delay", &phy->mykDevice->obsRx->orxAgcCtrl->agcSlowLoopSettlingDelay, 3);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-peak-wait-time", &phy->mykDevice->obsRx->orxAgcCtrl->agcPeakWaitTime, 4);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-reset-on-rx-enable", &phy->mykDevice->obsRx->orxAgcCtrl->agcResetOnRxEnable, 0);
	AD9371_OF_PROP("adi,obs-agc-conf-agc-enable-sync-pulse-for-gain-counter", &phy->mykDevice->obsRx->orxAgcCtrl->agcEnableSyncPulseForGainCounter, 0);

	AD9371_OF_PROP("adi,rx-profile-adc-div", &phy->mykDevice->rx->rxProfile->adcDiv, 1);
	AD9371_OF_PROP("adi,rx-profile-rx-fir-decimation", &phy->mykDevice->rx->rxProfile->rxFirDecimation, 2);
	AD9371_OF_PROP("adi,rx-profile-rx-dec5-decimation", &phy->mykDevice->rx->rxProfile->rxDec5Decimation, 5);
	AD9371_OF_PROP("adi,rx-profile-en-high-rej-dec5", &phy->mykDevice->rx->rxProfile->enHighRejDec5, 1);
	AD9371_OF_PROP("adi,rx-profile-rhb1-decimation", &phy->mykDevice->rx->rxProfile->rhb1Decimation, 1);
	AD9371_OF_PROP("adi,rx-profile-iq-rate_khz", &phy->mykDevice->rx->rxProfile->iqRate_kHz, 122880);
	AD9371_OF_PROP("adi,rx-profile-rf-bandwidth_hz", &phy->mykDevice->rx->rxProfile->rfBandwidth_Hz, 100000000);
	AD9371_OF_PROP("adi,rx-profile-rx-bbf-3db-corner_khz", &phy->mykDevice->rx->rxProfile->rxBbf3dBCorner_kHz, 100000);

	AD9371_GET_FIR("adi,rx-profile-rx-fir", phy->mykDevice->rx->rxProfile->rxFir);
	AD9371_GET_PROFILE("adi,rx-profile-custom-adc-profile", phy->mykDevice->rx->rxProfile->customAdcProfile);

	AD9371_OF_PROP("adi,obs-profile-adc-div", &phy->mykDevice->obsRx->orxProfile->adcDiv, 1);
	AD9371_OF_PROP("adi,obs-profile-rx-fir-decimation", &phy->mykDevice->obsRx->orxProfile->rxFirDecimation, 1);
	AD9371_OF_PROP("adi,obs-profile-rx-dec5-decimation", &phy->mykDevice->obsRx->orxProfile->rxDec5Decimation, 5);
	AD9371_OF_PROP("adi,obs-profile-en-high-rej-dec5", &phy->mykDevice->obsRx->orxProfile->enHighRejDec5, 0);
	AD9371_OF_PROP("adi,obs-profile-rhb1-decimation", &phy->mykDevice->obsRx->orxProfile->rhb1Decimation, 1);
	AD9371_OF_PROP("adi,obs-profile-iq-rate_khz", &phy->mykDevice->obsRx->orxProfile->iqRate_kHz, 245760);
	AD9371_OF_PROP("adi,obs-profile-rf-bandwidth_hz", &phy->mykDevice->obsRx->orxProfile->rfBandwidth_Hz, 200000000);
	AD9371_OF_PROP("adi,obs-profile-rx-bbf-3db-corner_khz", &phy->mykDevice->obsRx->orxProfile->rxBbf3dBCorner_kHz, 100000);

	AD9371_GET_FIR("adi,obs-profile-rx-fir", phy->mykDevice->obsRx->orxProfile->rxFir);
	AD9371_GET_PROFILE("adi,obs-profile-custom-adc-profile", phy->mykDevice->obsRx->orxProfile->customAdcProfile);

	AD9371_OF_PROP("adi,sniffer-profile-adc-div", &phy->mykDevice->obsRx->snifferProfile->adcDiv, 1);
	AD9371_OF_PROP("adi,sniffer-profile-rx-fir-decimation", &phy->mykDevice->obsRx->snifferProfile->rxFirDecimation, 4);
	AD9371_OF_PROP("adi,sniffer-profile-rx-dec5-decimation", &phy->mykDevice->obsRx->snifferProfile->rxDec5Decimation, 5);
	AD9371_OF_PROP("adi,sniffer-profile-en-high-rej-dec5", &phy->mykDevice->obsRx->snifferProfile->enHighRejDec5, 0);
	AD9371_OF_PROP("adi,sniffer-profile-rhb1-decimation", &phy->mykDevice->obsRx->snifferProfile->rhb1Decimation, 2);
	AD9371_OF_PROP("adi,sniffer-profile-iq-rate_khz", &phy->mykDevice->obsRx->snifferProfile->iqRate_kHz, 30720);
	AD9371_OF_PROP("adi,sniffer-profile-rf-bandwidth_hz", &phy->mykDevice->obsRx->snifferProfile->rfBandwidth_Hz, 20000000);
	AD9371_OF_PROP("adi,sniffer-profile-rx-bbf-3db-corner_khz", &phy->mykDevice->obsRx->snifferProfile->rxBbf3dBCorner_kHz, 100000);

	AD9371_GET_FIR("adi,sniffer-profile-rx-fir", phy->mykDevice->obsRx->snifferProfile->rxFir);
	AD9371_GET_PROFILE("adi,sniffer-profile-custom-adc-profile", phy->mykDevice->obsRx->snifferProfile->customAdcProfile);

	AD9371_OF_PROP("adi,tx-profile-dac-div", &phy->mykDevice->tx->txProfile->dacDiv, 1);
	AD9371_OF_PROP("adi,tx-profile-tx-fir-interpolation", &phy->mykDevice->tx->txProfile->txFirInterpolation, 1);
	AD9371_OF_PROP("adi,tx-profile-thb1-interpolation", &phy->mykDevice->tx->txProfile->thb1Interpolation, 2);
	AD9371_OF_PROP("adi,tx-profile-thb2-interpolation", &phy->mykDevice->tx->txProfile->thb2Interpolation, 1);
	AD9371_OF_PROP("adi,tx-profile-tx-input-hb-interpolation", &phy->mykDevice->tx->txProfile->txInputHbInterpolation, 1);
	AD9371_OF_PROP("adi,tx-profile-iq-rate_khz", &phy->mykDevice->tx->txProfile->iqRate_kHz, 245760);
	AD9371_OF_PROP("adi,tx-profile-primary-sig-bandwidth_hz", &phy->mykDevice->tx->txProfile->primarySigBandwidth_Hz, 75000000);
	AD9371_OF_PROP("adi,tx-profile-rf-bandwidth_hz", &phy->mykDevice->tx->txProfile->rfBandwidth_Hz, 200000000);
	AD9371_OF_PROP("adi,tx-profile-tx-dac-3db-corner_khz", &phy->mykDevice->tx->txProfile->txDac3dBCorner_kHz, 189477);
	AD9371_OF_PROP("adi,tx-profile-tx-bbf-3db-corner_khz", &phy->mykDevice->tx->txProfile->txBbf3dBCorner_kHz, 100000);
	if (IS_AD9375(phy))
		AD9371_OF_PROP("adi,tx-profile-enable-dpd-data-path", &phy->mykDevice->tx->txProfile->enableDpdDataPath, 1);

	AD9371_GET_FIR("adi,tx-profile-tx-fir", phy->mykDevice->tx->txProfile->txFir);

	AD9371_OF_PROP("adi,clocks-device-clock_khz", &phy->mykDevice->clocks->deviceClock_kHz, 122880);
	AD9371_OF_PROP("adi,clocks-clk-pll-vco-freq_khz", &phy->mykDevice->clocks->clkPllVcoFreq_kHz, 9830400);
	AD9371_OF_PROP("adi,clocks-clk-pll-vco-div", &phy->mykDevice->clocks->clkPllVcoDiv, 2);
	AD9371_OF_PROP("adi,clocks-clk-pll-hs-div", &phy->mykDevice->clocks->clkPllHsDiv, 4);

	AD9371_OF_PROP("adi,tx-settings-tx-channels-enable", &phy->mykDevice->tx->txChannels, TX1_TX2);
	AD9371_OF_PROP("adi,tx-settings-tx-pll-use-external-lo", &phy->mykDevice->tx->txPllUseExternalLo, 0);
	AD9371_OF_PROP("adi,tx-settings-tx-pll-lo-frequency_hz", &phy->mykDevice->tx->txPllLoFrequency_Hz, 2500000000U);
	AD9371_OF_PROP("adi,tx-settings-tx-atten-step-size", &phy->mykDevice->tx->txAttenStepSize, 0);
	AD9371_OF_PROP("adi,tx-settings-tx1-atten_mdb", &phy->mykDevice->tx->tx1Atten_mdB, 10000);
	AD9371_OF_PROP("adi,tx-settings-tx2-atten_mdb", &phy->mykDevice->tx->tx2Atten_mdB, 10000);

	if (IS_AD9375(phy)) {
		AD9371_OF_PROP("adi,dpd-damping", &phy->mykDevice->tx->dpdConfig->damping, 5);
		AD9371_OF_PROP("adi,dpd-num-weights", &phy->mykDevice->tx->dpdConfig->numWeights, 1);
		AD9371_OF_PROP("adi,dpd-model-version", &phy->mykDevice->tx->dpdConfig->modelVersion, 2);
		AD9371_OF_PROP("adi,dpd-high-power-model-update", &phy->mykDevice->tx->dpdConfig->highPowerModelUpdate, 1);
		AD9371_OF_PROP("adi,dpd-model-prior-weight", &phy->mykDevice->tx->dpdConfig->modelPriorWeight, 20);
		AD9371_OF_PROP("adi,dpd-robust-modeling", &phy->mykDevice->tx->dpdConfig->robustModeling, 0);
		AD9371_OF_PROP("adi,dpd-samples", &phy->mykDevice->tx->dpdConfig->samples, 512);
		AD9371_OF_PROP("adi,dpd-outlier-threshold", &phy->mykDevice->tx->dpdConfig->outlierThreshold, 4096);
		AD9371_OF_PROP("adi,dpd-additional-delay-offset", &phy->mykDevice->tx->dpdConfig->additionalDelayOffset, 0);
		AD9371_OF_PROP("adi,dpd-path-delay-pn-seq-level", &phy->mykDevice->tx->dpdConfig->pathDelayPnSeqLevel, 255);
		AD9371_OF_PROP("adi,dpd-weights0-real", &phy->mykDevice->tx->dpdConfig->weights[0].real, 64);
		AD9371_OF_PROP("adi,dpd-weights0-imag", &phy->mykDevice->tx->dpdConfig->weights[0].imag, 0);
		AD9371_OF_PROP("adi,dpd-weights1-real", &phy->mykDevice->tx->dpdConfig->weights[1].real, 0);
		AD9371_OF_PROP("adi,dpd-weights1-imag", &phy->mykDevice->tx->dpdConfig->weights[1].imag, 0);
		AD9371_OF_PROP("adi,dpd-weights2-real", &phy->mykDevice->tx->dpdConfig->weights[2].real, 0);
		AD9371_OF_PROP("adi,dpd-weights2-imag", &phy->mykDevice->tx->dpdConfig->weights[2].imag, 0);

		AD9371_OF_PROP("adi,clgc-tx1-desired-gain", &phy->mykDevice->tx->clgcConfig->tx1DesiredGain, -2000);
		AD9371_OF_PROP("adi,clgc-tx2-desired-gain", &phy->mykDevice->tx->clgcConfig->tx2DesiredGain, -2000);
		AD9371_OF_PROP("adi,clgc-tx1-atten-limit", &phy->mykDevice->tx->clgcConfig->tx1AttenLimit, 0);
		AD9371_OF_PROP("adi,clgc-tx2-atten-limit", &phy->mykDevice->tx->clgcConfig->tx2AttenLimit, 0);
		AD9371_OF_PROP("adi,clgc-tx1-control-ratio", &phy->mykDevice->tx->clgcConfig->tx1ControlRatio, 75);
		AD9371_OF_PROP("adi,clgc-tx2-control-ratio", &phy->mykDevice->tx->clgcConfig->tx2ControlRatio, 75);
		AD9371_OF_PROP("adi,clgc-allow-tx1-atten-updates", &phy->mykDevice->tx->clgcConfig->allowTx1AttenUpdates, 0);
		AD9371_OF_PROP("adi,clgc-allow-tx2-atten-updates", &phy->mykDevice->tx->clgcConfig->allowTx2AttenUpdates, 0);
		AD9371_OF_PROP("adi,clgc-additional-delay-offset", &phy->mykDevice->tx->clgcConfig->additionalDelayOffset, 0);
		AD9371_OF_PROP("adi,clgc-path-delay-pn-seq-level", &phy->mykDevice->tx->clgcConfig->pathDelayPnSeqLevel, 255);
		AD9371_OF_PROP("adi,clgc-tx1-rel-threshold", &phy->mykDevice->tx->clgcConfig->tx1RelThreshold, 600);
		AD9371_OF_PROP("adi,clgc-tx2-rel-threshold", &phy->mykDevice->tx->clgcConfig->tx2RelThreshold, 600);
		AD9371_OF_PROP("adi,clgc-tx1-rel-threshold-en", &phy->mykDevice->tx->clgcConfig->tx1RelThresholdEn, 0);
		AD9371_OF_PROP("adi,clgc-tx2-rel-threshold-en", &phy->mykDevice->tx->clgcConfig->tx2RelThresholdEn, 0);

		AD9371_OF_PROP("adi,vswr-additional-delay-offset", &phy->mykDevice->tx->vswrConfig->additionalDelayOffset, 0);
		AD9371_OF_PROP("adi,vswr-path-delay-pn-seq-level", &phy->mykDevice->tx->vswrConfig->pathDelayPnSeqLevel, 255);
		AD9371_OF_PROP("adi,vswr-tx1-vswr-switch-gpio3p3-pin", &phy->mykDevice->tx->vswrConfig->tx1VswrSwitchGpio3p3Pin, 0);
		AD9371_OF_PROP("adi,vswr-tx2-vswr-switch-gpio3p3-pin", &phy->mykDevice->tx->vswrConfig->tx2VswrSwitchGpio3p3Pin, 1);
		AD9371_OF_PROP("adi,vswr-tx1-vswr-switch-polarity", &phy->mykDevice->tx->vswrConfig->tx1VswrSwitchPolarity, 0);
		AD9371_OF_PROP("adi,vswr-tx2-vswr-switch-polarity", &phy->mykDevice->tx->vswrConfig->tx2VswrSwitchPolarity, 0);
		AD9371_OF_PROP("adi,vswr-tx1-vswr-switch-delay_us", &phy->mykDevice->tx->vswrConfig->tx1VswrSwitchDelay_us, 50);
		AD9371_OF_PROP("adi,vswr-tx2-vswr-switch-delay_us", &phy->mykDevice->tx->vswrConfig->tx2VswrSwitchDelay_us, 50);
	}

	AD9371_OF_PROP("adi,rx-settings-rx-channels-enable", &phy->mykDevice->rx->rxChannels, RX1_RX2);
	AD9371_OF_PROP("adi,rx-settings-rx-pll-use-external-lo", &phy->mykDevice->rx->rxPllUseExternalLo, 0);
	AD9371_OF_PROP("adi,rx-settings-rx-pll-lo-frequency_hz", &phy->mykDevice->rx->rxPllLoFrequency_Hz, 2500000000U);
	AD9371_OF_PROP("adi,rx-settings-real-if-data", &phy->mykDevice->rx->realIfData, 0);

	AD9371_OF_PROP("adi,obs-settings-obs-rx-channels-enable", &phy->mykDevice->obsRx->obsRxChannelsEnable, MYK_ORX1_ORX2 | MYK_SNRXA_B_C);
	AD9371_OF_PROP("adi,obs-settings-obs-rx-lo-source", &phy->mykDevice->obsRx->obsRxLoSource, 0);
	AD9371_OF_PROP("adi,obs-settings-sniffer-pll-lo-frequency_hz", &phy->mykDevice->obsRx->snifferPllLoFrequency_Hz, 2600000000U);
	AD9371_OF_PROP("adi,obs-settings-real-if-data", &phy->mykDevice->obsRx->realIfData, 0);
	AD9371_OF_PROP("adi,obs-settings-default-obs-rx-channel", &phy->mykDevice->obsRx->defaultObsRxChannel, OBS_INTERNALCALS);

	AD9371_GET_PROFILE("adi,obs-settings-custom-loopback-adc-profile", phy->mykDevice->obsRx->customLoopbackAdcProfile);

	AD9371_OF_PROP("adi,arm-gpio-use-rx2-enable-pin", &phy->mykDevice->auxIo->armGpio->useRx2EnablePin, 0);
	AD9371_OF_PROP("adi,arm-gpio-use-tx2-enable-pin", &phy->mykDevice->auxIo->armGpio->useTx2EnablePin, 0);
	AD9371_OF_PROP("adi,arm-gpio-tx-rx-pin-mode", &phy->mykDevice->auxIo->armGpio->txRxPinMode, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx-pin-mode", &phy->mykDevice->auxIo->armGpio->orxPinMode, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx-trigger-pin", &phy->mykDevice->auxIo->armGpio->orxTriggerPin, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx-mode2-pin", &phy->mykDevice->auxIo->armGpio->orxMode2Pin, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx-mode1-pin", &phy->mykDevice->auxIo->armGpio->orxMode1Pin, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx-mode0-pin", &phy->mykDevice->auxIo->armGpio->orxMode0Pin, 0);
	AD9371_OF_PROP("adi,arm-gpio-rx1-enable-ack", &phy->mykDevice->auxIo->armGpio->rx1EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-rx2-enable-ack", &phy->mykDevice->auxIo->armGpio->rx2EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-tx1-enable-ack", &phy->mykDevice->auxIo->armGpio->tx1EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-tx2-enable-ack", &phy->mykDevice->auxIo->armGpio->tx2EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx1-enable-ack", &phy->mykDevice->auxIo->armGpio->orx1EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-orx2-enable-ack", &phy->mykDevice->auxIo->armGpio->orx2EnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-srx-enable-ack", &phy->mykDevice->auxIo->armGpio->srxEnableAck, 0);
	AD9371_OF_PROP("adi,arm-gpio-tx-obs-select", &phy->mykDevice->auxIo->armGpio->txObsSelect, 0);

	AD9371_OF_PROP("adi,gpio-3v3-oe-mask", &phy->mykDevice->auxIo->gpio3v3->gpio3v3Oe, 0);
	AD9371_OF_PROP("adi,gpio-3v3-src-ctrl3_0", &phy->mykDevice->auxIo->gpio3v3->gpio3v3SrcCtrl3_0, 3);
	AD9371_OF_PROP("adi,gpio-3v3-src-ctrl7_4", &phy->mykDevice->auxIo->gpio3v3->gpio3v3SrcCtrl7_4, 3);
	AD9371_OF_PROP("adi,gpio-3v3-src-ctrl11_8", &phy->mykDevice->auxIo->gpio3v3->gpio3v3SrcCtrl11_8, 3);

	AD9371_OF_PROP("adi,gpio-oe-mask", &phy->mykDevice->auxIo->gpio->gpioOe, 0);
	AD9371_OF_PROP("adi,gpio-src-ctrl3_0", &phy->mykDevice->auxIo->gpio->gpioSrcCtrl3_0, 0);
	AD9371_OF_PROP("adi,gpio-src-ctrl7_4", &phy->mykDevice->auxIo->gpio->gpioSrcCtrl7_4, 0);
	AD9371_OF_PROP("adi,gpio-src-ctrl11_8", &phy->mykDevice->auxIo->gpio->gpioSrcCtrl11_8, 0);
	AD9371_OF_PROP("adi,gpio-src-ctrl15_12", &phy->mykDevice->auxIo->gpio->gpioSrcCtrl15_12, 0);
	AD9371_OF_PROP("adi,gpio-src-ctrl18_16", &phy->mykDevice->auxIo->gpio->gpioSrcCtrl18_16, 0);

	AD9371_OF_PROP("adi,aux-dac-enable-mask", &phy->mykDevice->auxIo->auxDacEnable, 0);
	AD9371_OF_PROP("adi,aux-dac-value0", &phy->mykDevice->auxIo->auxDacValue[0], 0);
	AD9371_OF_PROP("adi,aux-dac-slope0", &phy->mykDevice->auxIo->auxDacSlope[0], 0);
	AD9371_OF_PROP("adi,aux-dac-vref0", &phy->mykDevice->auxIo->auxDacVref[0], 0);
	AD9371_OF_PROP("adi,aux-dac-value1", &phy->mykDevice->auxIo->auxDacValue[1], 0);
	AD9371_OF_PROP("adi,aux-dac-slope1", &phy->mykDevice->auxIo->auxDacSlope[1], 0);
	AD9371_OF_PROP("adi,aux-dac-vref1", &phy->mykDevice->auxIo->auxDacVref[1], 0);
	AD9371_OF_PROP("adi,aux-dac-value2", &phy->mykDevice->auxIo->auxDacValue[2], 0);
	AD9371_OF_PROP("adi,aux-dac-slope2", &phy->mykDevice->auxIo->auxDacSlope[2], 0);
	AD9371_OF_PROP("adi,aux-dac-vref2", &phy->mykDevice->auxIo->auxDacVref[2], 0);
	AD9371_OF_PROP("adi,aux-dac-value3", &phy->mykDevice->auxIo->auxDacValue[3], 0);
	AD9371_OF_PROP("adi,aux-dac-slope3", &phy->mykDevice->auxIo->auxDacSlope[3], 0);
	AD9371_OF_PROP("adi,aux-dac-vref3", &phy->mykDevice->auxIo->auxDacVref[3], 0);
	AD9371_OF_PROP("adi,aux-dac-value4", &phy->mykDevice->auxIo->auxDacValue[4], 0);
	AD9371_OF_PROP("adi,aux-dac-slope4", &phy->mykDevice->auxIo->auxDacSlope[4], 0);
	AD9371_OF_PROP("adi,aux-dac-vref4", &phy->mykDevice->auxIo->auxDacVref[4], 0);
	AD9371_OF_PROP("adi,aux-dac-value5", &phy->mykDevice->auxIo->auxDacValue[5], 0);
	AD9371_OF_PROP("adi,aux-dac-slope5", &phy->mykDevice->auxIo->auxDacSlope[5], 0);
	AD9371_OF_PROP("adi,aux-dac-vref5", &phy->mykDevice->auxIo->auxDacVref[5], 0);
	AD9371_OF_PROP("adi,aux-dac-value6", &phy->mykDevice->auxIo->auxDacValue[6], 0);
	AD9371_OF_PROP("adi,aux-dac-slope6", &phy->mykDevice->auxIo->auxDacSlope[6], 0);
	AD9371_OF_PROP("adi,aux-dac-vref6", &phy->mykDevice->auxIo->auxDacVref[6], 0);
	AD9371_OF_PROP("adi,aux-dac-value7", &phy->mykDevice->auxIo->auxDacValue[7], 0);
	AD9371_OF_PROP("adi,aux-dac-slope7", &phy->mykDevice->auxIo->auxDacSlope[7], 0);
	AD9371_OF_PROP("adi,aux-dac-vref7", &phy->mykDevice->auxIo->auxDacVref[7], 0);
	AD9371_OF_PROP("adi,aux-dac-value8", &phy->mykDevice->auxIo->auxDacValue[8], 0);
	AD9371_OF_PROP("adi,aux-dac-slope8", &phy->mykDevice->auxIo->auxDacSlope[8], 0);
	AD9371_OF_PROP("adi,aux-dac-vref8", &phy->mykDevice->auxIo->auxDacVref[8], 0);
	AD9371_OF_PROP("adi,aux-dac-value9", &phy->mykDevice->auxIo->auxDacValue[9], 0);
	AD9371_OF_PROP("adi,aux-dac-slope9", &phy->mykDevice->auxIo->auxDacSlope[9], 0);
	AD9371_OF_PROP("adi,aux-dac-vref9", &phy->mykDevice->auxIo->auxDacVref[9], 0);
	return pdata;
}
#else
static
struct ad9371_phy_platform_data *ad9371_phy_parse_dt(struct iio_dev *iodev,
						     struct device *dev)
{
	return NULL;
}

static int ad9371_register_debugfs(struct iio_dev *indio_dev)
{
	return -ENODEV;
}
#endif

static int ad9371_parse_profile(struct ad9371_rf_phy *phy,
				 char *data, u32 size)
{
	mykonosDevice_t *mykDevice = phy->mykDevice;
	mykonosRxProfile_t *rx_profile = NULL;
	mykonosTxProfile_t *tx_profile = NULL;
	mykonosFir_t *fir = NULL;
	struct device *dev = &phy->spi->dev;
	char clocks = 0, tx = 0, rx = 0,
	     filter = 0, adcprof = 0, lpbkadcprof = 0, header = 0,
	     dpdconfig = 0, clgcconfig = 0, vswrconfig = 0;

	char *line, *ptr = data;
	unsigned int int32, int32_2;
	int ret, num = 0, version = 0, type, max, sint32, retval = 0;

#define GET_TOKEN(x, n) \
	{ret = sscanf(line, " <" #n "=%u>", &int32);\
	if (ret == 1) { \
		x->n = int32;\
		continue;\
	}}

#define GET_STOKEN(x, n) \
	{ret = sscanf(line, " <" #n "=%d>", &sint32);\
		if (ret == 1) { \
			x->n = sint32;\
			continue;\
		}}

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size) {
			break;
		}

		line = skip_spaces(line);

		if (line[0] == '#' || line[0] == '\r' ||  line[0] == '\0')
			continue;

		if (!header && strstr(line, "<profile AD937")) {
			ret = sscanf(line, " <profile AD%d version=%d", &type, &version);

			if (ret == 2 && version == 0 && (type == 9371 || type == 9375))
				header = 1;
			else
				dev_err(dev, "%s: Invalid Version %d or Model %d",
					__func__, version, type);

			continue;
		}

		if (!clocks && strstr(line, "<clocks>")) {
			clocks = 1;
			continue;
		}

		if (clocks && strstr(line, "</clocks>")) {
			clocks = 0;
			retval = 0;
			continue;
		}

		if (!dpdconfig && strstr(line, "<DpdConfig>")) {
			dpdconfig = IS_AD9375(phy);
			continue;
		}

		if (dpdconfig && strstr(line, "</DpdConfig>")) {
			dpdconfig = 0;
			retval = 1;
			continue;
		}

		if (!clgcconfig && strstr(line, "<ClgcConfig>")) {
			clgcconfig = IS_AD9375(phy);
			continue;
		}

		if (clgcconfig && strstr(line, "</ClgcConfig>")) {
			clgcconfig = 0;
			retval = 1;
			continue;
		}

		if (!vswrconfig && strstr(line, "<VswrConfig>")) {
			vswrconfig = IS_AD9375(phy);
			continue;
		}

		if (vswrconfig && strstr(line, "</VswrConfig>")) {
			vswrconfig = 0;
			retval = 1;
			continue;
		}

		if (!rx && strstr(line, "<rx>")) {
			rx = 1;
			rx_profile = mykDevice->rx->rxProfile;
			fir = rx_profile->rxFir;
			continue;
		}

		if (rx && strstr(line, "</rx>")) {
			rx = 0;
			continue;
		}

		if (!rx && strstr(line, "<obs>")) {
			rx = 1;
			rx_profile = mykDevice->obsRx->orxProfile;
			fir = rx_profile->rxFir;
			continue;
		}

		if (rx && strstr(line, "</obs>")) {
			rx = 0;
			continue;
		}

		if (!rx && strstr(line, "<sniffer>")) {
			rx = 1;
			rx_profile = mykDevice->obsRx->snifferProfile;
			fir = rx_profile->rxFir;
			continue;
		}

		if (rx && strstr(line, "</sniffer>")) {
			rx = 0;
			continue;
		}

		if (!tx && strstr(line, "<tx>")) {
			tx = 1;
			tx_profile = mykDevice->tx->txProfile;
			fir = tx_profile->txFir;
			continue;
		}

		if (tx && strstr(line, "</tx>")) {
			tx = 0;
			continue;
		}

		if (!filter && (sscanf(line, " <filter FIR gain=%d num=%d>", &ret, &max) == 2)) {
			fir->gain_dB = ret;
			filter = 1;
			num = 0;
			continue;
		}

		if (filter && strstr(line, "</filter>")) {
			filter = 0;
			fir->numFirCoefs = num;
			continue;
		}

		if (!adcprof && (sscanf(line, " <adc-profile num=%d>", &max) == 1)) {
			adcprof = 1;
			num = 0;
			continue;
		}

		if (adcprof && strstr(line, "</adc-profile>")) {
			adcprof = 0;
			if (num != 16 && num != 37)
				dev_err(dev, "%s:%d: Invalid number (%d) of coefficients",
					__func__, __LINE__, num);

			num = 0;
			continue;
		}

		if (!lpbkadcprof && (sscanf(line, " <lpbk-adc-profile num=%d>", &max) == 1)) {
			lpbkadcprof = 1;
			num = 0;
			continue;
		}

		if (lpbkadcprof && strstr(line, "</lpbk-adc-profile>")) {
			lpbkadcprof = 0;
			if (num != 16 && num != 37)
				dev_err(dev, "%s:%d: Invalid number (%d) of coefficients",
					__func__, __LINE__, num);

			num = 0;
			continue;
		}

		if (dpdconfig) {
			GET_TOKEN(mykDevice->tx->dpdConfig, damping);
			GET_TOKEN(mykDevice->tx->dpdConfig, numWeights);
			GET_TOKEN(mykDevice->tx->dpdConfig, modelVersion);
			GET_TOKEN(mykDevice->tx->dpdConfig, robustModeling);
			GET_TOKEN(mykDevice->tx->dpdConfig, samples);
			GET_TOKEN(mykDevice->tx->dpdConfig, outlierThreshold);
			GET_STOKEN(mykDevice->tx->dpdConfig, additionalDelayOffset);
			GET_TOKEN(mykDevice->tx->dpdConfig, pathDelayPnSeqLevel);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[0].real);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[0].imag);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[1].real);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[1].imag);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[2].real);
			GET_STOKEN(mykDevice->tx->dpdConfig, weights[2].imag);
		}

		if (clgcconfig) {
			GET_STOKEN(mykDevice->tx->clgcConfig, tx1DesiredGain);
			GET_STOKEN(mykDevice->tx->clgcConfig, tx2DesiredGain);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx1AttenLimit);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx2AttenLimit);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx1ControlRatio);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx2ControlRatio);
			GET_TOKEN(mykDevice->tx->clgcConfig, allowTx1AttenUpdates);
			GET_TOKEN(mykDevice->tx->clgcConfig, allowTx2AttenUpdates);
			GET_STOKEN(mykDevice->tx->clgcConfig, additionalDelayOffset);
			GET_TOKEN(mykDevice->tx->clgcConfig, pathDelayPnSeqLevel);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx1RelThreshold);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx2RelThreshold);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx1RelThresholdEn);
			GET_TOKEN(mykDevice->tx->clgcConfig, tx2RelThresholdEn);
		}

		if (vswrconfig) {
			GET_STOKEN(mykDevice->tx->vswrConfig, additionalDelayOffset);
			GET_TOKEN(mykDevice->tx->vswrConfig, pathDelayPnSeqLevel);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx1VswrSwitchGpio3p3Pin);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx2VswrSwitchGpio3p3Pin);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx1VswrSwitchPolarity);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx2VswrSwitchPolarity);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx1VswrSwitchDelay_us);
			GET_TOKEN(mykDevice->tx->vswrConfig, tx2VswrSwitchDelay_us);
		}

		if (clocks) {
			GET_TOKEN(mykDevice->clocks, deviceClock_kHz);
			GET_TOKEN(mykDevice->clocks, clkPllVcoFreq_kHz);
			GET_TOKEN(mykDevice->clocks, clkPllVcoDiv);
			GET_TOKEN(mykDevice->clocks, clkPllHsDiv);
		}

		if (rx && !filter && !adcprof && !lpbkadcprof) {
			GET_TOKEN(rx_profile, adcDiv);
			GET_TOKEN(rx_profile, rxFirDecimation);
			GET_TOKEN(rx_profile, rxDec5Decimation);
			GET_TOKEN(rx_profile, enHighRejDec5);
			GET_TOKEN(rx_profile, rhb1Decimation);
			GET_TOKEN(rx_profile, iqRate_kHz);
			GET_TOKEN(rx_profile, rfBandwidth_Hz);
			GET_TOKEN(rx_profile, rxBbf3dBCorner_kHz);
		}

		if (tx && !filter) {
			GET_TOKEN(tx_profile, txFirInterpolation);
			GET_TOKEN(tx_profile, thb1Interpolation);
			GET_TOKEN(tx_profile, thb2Interpolation);
			GET_TOKEN(tx_profile, txInputHbInterpolation);
			GET_TOKEN(tx_profile, iqRate_kHz);
			GET_TOKEN(tx_profile, primarySigBandwidth_Hz);
			GET_TOKEN(tx_profile, rfBandwidth_Hz);
			GET_TOKEN(tx_profile, txDac3dBCorner_kHz);
			GET_TOKEN(tx_profile, txBbf3dBCorner_kHz);

			ret = sscanf(line, " <dacDiv=%u.%u>", &int32, &int32_2);
			if (ret > 0) {
				if (ret == 1) {
					if (int32 == 2)
						tx_profile->dacDiv = DACDIV_2;
					else if (int32 == 4)
						tx_profile->dacDiv = DACDIV_4;
				} else if (ret == 2) {
					if (int32 == 2 && int32_2 == 5)
						tx_profile->dacDiv = DACDIV_2p5;
					else if (int32 == 4)
						tx_profile->dacDiv = DACDIV_4;
					else
						tx_profile->dacDiv = DACDIV_2;
				}
				continue;
			}
		}

		if (filter) {
			if (sscanf(line, " %d", &ret) == 1) {
				if (num >= max)
					return -EINVAL;

				fir->coefs[num++] = ret;
				continue;
			}
		}

		if (adcprof && rx) {
			if (rx_profile->customAdcProfile == NULL) {
				rx_profile->customAdcProfile =
					devm_kzalloc(dev, 37 * sizeof(*rx_profile->customAdcProfile), GFP_KERNEL);
			}

			if (sscanf(line, " %d", &ret) == 1) {
				if (num >= 37 || num > max)
					return -EINVAL;

				rx_profile->customAdcProfile[num++] = ret;
				continue;
			}
		}

		if (lpbkadcprof && rx) {
			if (mykDevice->obsRx->customLoopbackAdcProfile == NULL) {
				mykDevice->obsRx->customLoopbackAdcProfile =
				devm_kzalloc(dev, 37 * sizeof(*mykDevice->obsRx->customLoopbackAdcProfile), GFP_KERNEL);
			}

			if (sscanf(line, " %d", &ret) == 1) {
				if (num >= 37 || num > max)
					return -EINVAL;

				mykDevice->obsRx->customLoopbackAdcProfile[num++] = ret;
				continue;
			}
		}

		if (header && strstr(line, "</profile>")) {
			return retval;
		}

		/* We should never end up here */
		dev_err(dev, "%s: Malformed profile entry was %s",
			__func__, line);

		return -EINVAL;

	}

	return -EINVAL;
}

static ssize_t
ad9371_profile_bin_write(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	int ret;

	if (off == 0) {
		if (phy->bin_attr_buf == NULL) {
			phy->bin_attr_buf = devm_kzalloc(&phy->spi->dev,
						bin_attr->size, GFP_KERNEL);
			if (!phy->bin_attr_buf)
				return -ENOMEM;
		} else {
			memset(phy->bin_attr_buf, 0, bin_attr->size);
		}
	}

	memcpy(phy->bin_attr_buf + off, buf, count);

	if (strnstr(phy->bin_attr_buf, "</profile>", off + count) == NULL)
		return count;

	ret = ad9371_parse_profile(phy, phy->bin_attr_buf, off + count);
	if (ret < 0)
		return ret;


	mutex_lock(&phy->indio_dev->mlock);

	if (IS_AD9375(phy) && ret == 1) {
		ad9371_set_radio_state(phy, RADIO_FORCE_OFF);

		ret = MYKONOS_configDpd(phy->mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out_unlock;
		}

		ret = MYKONOS_configClgc(phy->mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out_unlock;
		}

		ret = MYKONOS_configVswr(phy->mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			ret = -EFAULT;
			goto out_unlock;
		}

		ad9371_set_radio_state(phy, RADIO_RESTORE_STATE);
	} else {
		ret = ad9371_reinit(phy);
	}

out_unlock:
	mutex_unlock(&phy->indio_dev->mlock);

	return (ret < 0) ? ret : count;
}

static ssize_t
ad9371_profile_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

//	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
//	struct ad9371_rf_phy *phy = iio_priv(indio_dev);

	if (off)
		return 0;

	return sprintf(buf, "TBD");
}

static void ad9371_free_gt(struct ad9371_rf_phy *phy, struct gain_table_info *table)
{
	int i;

	if (!table)
		return;

	for (i = RX1_GT; i <= LOOPBACK_GT; i++)
		if (table[i].abs_gain_tbl) {
			devm_kfree(&phy->spi->dev, table[i].abs_gain_tbl);
			table[i].abs_gain_tbl = NULL;
			table[i].dest = 0;
		}
}

static int ad9371_load_all_gt(struct ad9371_rf_phy *phy, struct gain_table_info *table)
{
	int i, ret;

	if (!table)
		return -ENODEV;

	for (i = RX1_GT; i <= LOOPBACK_GT; i++)
		if (table[i].dest && table[i].abs_gain_tbl) {
			ret = MYKONOS_programRxGainTable(phy->mykDevice, (u8 *) table[i].tab,
						   table[i].max_index, table[i].dest);
			if (ret < 0)
				return ret;
		}

	return 0;
}

static struct gain_table_info * ad9371_parse_gt(struct ad9371_rf_phy *phy,
						char *data, u32 size)
{
	struct gain_table_info *table = phy->gt_info;
	bool header_found;
	int i = 0, ret, dest, table_num = 0;
	char *line, *ptr = data;
	u8 *p;

	header_found = false;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size) {
			break;
		}

		if (line[0] == '#') /* skip comment lines */
			continue;

		if (strstr(line, "list>")) /* skip <[/]list> */
			continue;

		if (!header_found) {
			char type[40];
			unsigned model;
			u64 start;
			u64 end;

			ret = sscanf(line, " <gaintable AD%i type=%s dest=%i start=%lli end=%lli>",
				     &model , type, &dest, &start, &end);

			if (ret == 5) {
				if (!(model == 9371 || model == 9370)) {
					ret = -EINVAL;
					goto out;
				}
				if (start >= end) {
					ret = -EINVAL;
					goto out;
				}

				if (dest < RX1_GT || dest > LOOPBACK_GT) {
					ret = -EINVAL;
					goto out;
				}

				p = devm_kzalloc(&phy->spi->dev,
					sizeof(s32[MAX_RX_GAIN_TABLE_NUMINDEXES]) +
					sizeof(u8[MAX_RX_GAIN_TABLE_NUMINDEXES][4]),
					GFP_KERNEL);
				if (!p) {
					ret = -ENOMEM;
					goto out;
				}

				table[dest].dest = dest;
				table[dest].abs_gain_tbl = (s32 *) p;
				table[dest].tab = (u8 (*) [4]) (p +
					sizeof(s32[MAX_RX_GAIN_TABLE_NUMINDEXES]));

				table[dest].start = start;
				table[dest].end = end;

				header_found = true;
				i = 0;

				continue;
			} else {
				header_found = false;
			}
		}

		if (header_found) {
			int a,b,c,d,e;
			ret = sscanf(line, " %i,%i,%i,%i,%i", &a, &b, &c, &d, &e);
			if (ret == 5) {
				if (i >= MAX_RX_GAIN_TABLE_NUMINDEXES)
					goto out;

				if ((i > 0) && (a > table[dest].abs_gain_tbl[i - 1]))
					dev_warn(&phy->spi->dev,
						"Gain table must be monotonic");

				table[dest].abs_gain_tbl[i] = a;
				table[dest].tab[i][0] = b;
				table[dest].tab[i][1] = c;
				table[dest].tab[i][2] = d;
				table[dest].tab[i][3] = e;
				i++;
				continue;
			} else if (strstr(line, "</gaintable>")) {
				table[dest].max_index = i;
				header_found = false;
				table_num++;

				continue;
			} else {
				dev_err(&phy->spi->dev,
						"ERROR: Malformed gain table");
				goto out_free_tables;
			}
		}
	}

	dev_dbg(&phy->spi->dev, "%s: table_num %d header_found %d",
		__func__, table_num, header_found);

	if (table_num > 0 && !header_found)
		return table;
	else
		return  ERR_PTR(-EFAULT);

out_free_tables:
	ad9371_free_gt(phy, table);

out:
	return ERR_PTR(ret);
}

static ssize_t
ad9371_gt_bin_write(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct ad9371_rf_phy *phy = iio_priv(indio_dev);
	struct gain_table_info *table;
	int ret;

	if (off == 0) {
		if (phy->bin_gt_attr_buf == NULL) {
			phy->bin_gt_attr_buf = devm_kzalloc(&phy->spi->dev,
						bin_attr->size, GFP_KERNEL);
			if (!phy->bin_gt_attr_buf)
				return -ENOMEM;
		} else {
			memset(phy->bin_gt_attr_buf, 0, bin_attr->size);
		}
	}

	memcpy(phy->bin_gt_attr_buf + off, buf, count);

	if (strnstr(phy->bin_gt_attr_buf, "</list>", off + count) == NULL)
		return count;

	table = ad9371_parse_gt(phy, phy->bin_gt_attr_buf, off + count);
	if (IS_ERR_OR_NULL(table))
		return PTR_ERR(table);

	mutex_lock(&phy->indio_dev->mlock);

	ret = ad9371_load_all_gt(phy, table);

	mutex_unlock(&phy->indio_dev->mlock);

	return (ret < 0) ? ret : count;
}

static unsigned long ad9371_bb_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct ad9371_clock *clk_priv = to_clk_priv(hw);
	return clk_priv->rate;
}

static int ad9371_bb_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct ad9371_clock *clk_priv = to_clk_priv(hw);
	clk_priv->rate = rate;

	return 0;
}

static long ad9371_bb_round_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long *prate)
{
	struct ad9371_clock *clk_priv = to_clk_priv(hw);
	dev_dbg(&clk_priv->spi->dev, "%s: Rate %lu Hz", __func__, rate);

	return rate;
}

static const struct clk_ops bb_clk_ops = {
	.round_rate = ad9371_bb_round_rate,
	.set_rate = ad9371_bb_set_rate,
	.recalc_rate = ad9371_bb_recalc_rate,
};

#define AD9371_MAX_CLK_NAME 79

static char *ad9371_clk_set_dev_name(struct ad9371_rf_phy *phy,
				     char *dest, const char *name)
{
	size_t len = 0;

	if (name == NULL)
		return NULL;

	if (*name == '-')
		len = strlcpy(dest, dev_name(&phy->spi->dev),
			      AD9371_MAX_CLK_NAME);
	else
		*dest = '\0';

	return strncat(dest, name, AD9371_MAX_CLK_NAME - len);
}

static int ad9371_clk_register(struct ad9371_rf_phy *phy,
			       const char *name, const char *parent_name,
			       const char *parent_name2, unsigned long flags,
			       u32 source)
{
	struct ad9371_clock *clk_priv = &phy->clk_priv[source];
	struct clk_init_data init;
	struct clk *clk;
	char c_name[AD9371_MAX_CLK_NAME + 1], p_name[2][AD9371_MAX_CLK_NAME + 1];
	const char *_parent_name[2];

	/* struct ad9371_clock assignments */
	clk_priv->source = source;
	clk_priv->hw.init = &init;
	clk_priv->spi = phy->spi;
	clk_priv->phy = phy;

	_parent_name[0] = ad9371_clk_set_dev_name(phy, p_name[0], parent_name);
	_parent_name[1] = ad9371_clk_set_dev_name(phy, p_name[1], parent_name2);

	init.name = ad9371_clk_set_dev_name(phy, c_name, name);;
	init.flags = flags;
	init.parent_names = &_parent_name[0];
	init.num_parents = _parent_name[1] ? 2 : _parent_name[0] ? 1 : 0;

	switch (source) {
	case RX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->mykDevice->rx->rxProfile->iqRate_kHz;
		break;
	case OBS_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->mykDevice->obsRx->orxProfile->iqRate_kHz;
		break;
	case TX_SAMPL_CLK:
		init.ops = &bb_clk_ops;
		clk_priv->rate = phy->mykDevice->tx->txProfile->iqRate_kHz;
		break;
	default:
		return -EINVAL;
	}

	clk_priv->rate *= 1000;

	clk = devm_clk_register(&phy->spi->dev, &clk_priv->hw);
	phy->clks[source] = clk;

	return 0;
}

/**** JESD204-FSM ****/


struct ad9371_jesd204_priv {
	struct ad9371_rf_phy *phy;
	bool has_tx;
	bool has_rx;
	bool has_orx;
	u8 pllLock;
};

static int ad9371_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct spi_device *spi = to_spi_device(dev);
	struct ad9371_rf_phy *phy = ad9371_spi_to_phy(spi);
	mykonosJesd204bFramerConfig_t *framer = NULL;
	mykonosJesd204bDeframerConfig_t *deframer = NULL;

	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	priv->phy = phy; /* FIXME : move to probe */

	switch (lnk->link_id) {
	case DEFRAMER_LINK_TX:
		deframer = phy->mykDevice->tx->deframer;
		lnk->sample_rate = phy->mykDevice->tx->txProfile->iqRate_kHz * 1000;
		priv->has_tx = true;
		phy->tracking_cal_mask |= TRACK_TX1_QEC | TRACK_TX2_QEC;
		break;
	case FRAMER_LINK_RX:
		framer = phy->mykDevice->rx->framer;
		lnk->sample_rate = phy->mykDevice->rx->rxProfile->iqRate_kHz * 1000;
		priv->has_rx = true;
		phy->tracking_cal_mask |= TRACK_RX1_QEC | TRACK_RX2_QEC;
		break;
	case FRAMER_LINK_ORX:
		framer = phy->mykDevice->obsRx->framer;
		lnk->sample_rate = phy->mykDevice->obsRx->orxProfile->iqRate_kHz * 1000;
		priv->has_orx = true;
		break;
	default:
		return -EINVAL;
	}

	if (framer) {
		lnk->num_converters = framer->M;
		lnk->num_lanes = hweight8(framer->serializerLanesEnabled);
		lnk->octets_per_frame = (2 * lnk->num_converters) / lnk->num_lanes;
		lnk->frames_per_multiframe = framer->K;
		lnk->device_id = framer->deviceId;
		lnk->bank_id = framer->bankId;
		lnk->scrambling = framer->scramble;
		lnk->bits_per_sample = 16;
		lnk->converter_resolution = 14;
		lnk->ctrl_bits_per_sample = 2;
		lnk->jesd_version = JESD204_VERSION_B;
		lnk->subclass = framer->externalSysref ? JESD204_SUBCLASS_1 : JESD204_SUBCLASS_0;
		lnk->is_transmit = false;
	} else if (deframer) {
		lnk->num_converters = deframer->M;
		lnk->num_lanes = hweight8(deframer->deserializerLanesEnabled);
		lnk->octets_per_frame = (2 * lnk->num_converters) / lnk->num_lanes;
		lnk->frames_per_multiframe = deframer->K;
		lnk->device_id = deframer->deviceId;
		lnk->bank_id = deframer->bankId;
		lnk->scrambling = deframer->scramble;
		lnk->bits_per_sample = 16;
		lnk->converter_resolution = 14;
		lnk->ctrl_bits_per_sample = 2;
		lnk->jesd_version = JESD204_VERSION_B;
		lnk->subclass = deframer->externalSysref ? JESD204_SUBCLASS_1 : JESD204_SUBCLASS_0;
		lnk->is_transmit = true;
	};

	return JESD204_STATE_CHANGE_DONE;
}

static void ad9371_info(struct ad9371_rf_phy *phy)
{
	u8 vers[3], rev;
	mykonosBuild_t buildType;
	u32 api_vers[4];

	MYKONOS_getArmVersion(phy->mykDevice, &vers[0], &vers[1], &vers[2], &buildType);
	MYKONOS_getApiVersion(phy->mykDevice, &api_vers[0], &api_vers[1], &api_vers[2], &api_vers[3]);
	MYKONOS_getDeviceRev(phy->mykDevice, &rev);

	dev_info(&phy->spi->dev,
		"AD937%d Rev %d, Firmware %u.%u.%u API version: %u.%u.%u.%u successfully initialized%s",
		 AD937x_PARTID(phy), rev, vers[0], vers[1], vers[2],
		 api_vers[0], api_vers[1], api_vers[2], api_vers[3],
		 phy->jdev ? " via jesd204-fsm" : "");
}

int ad9371_jesd204_link_setup(struct jesd204_dev *jdev,
				enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	u8 pllLockStatus = 0;
	int ret = MYKONOS_ERR_OK;
	long dev_clk;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	/*******************************/
	/**** Mykonos Initialization ***/
	/*******************************/

	dev_clk = clk_round_rate(phy->dev_clk, mykDevice->clocks->deviceClock_kHz * 1000);

	if (dev_clk > 0 && (dev_clk / 1000) == mykDevice->clocks->deviceClock_kHz) {
		clk_set_rate(phy->dev_clk, (unsigned long) dev_clk);
	} else {
		dev_err(&phy->spi->dev, "Requesting device clock %u failed got %ld",
			mykDevice->clocks->deviceClock_kHz * 1000, dev_clk);
		return -EINVAL;
	}

	/* Toggle RESETB pin on Mykonos device */

	ret = ad9371_reset(phy);
	if (ret) {
		dev_err(&phy->spi->dev, "RESET Failed");
		return ret;
	}

	/* MYKONOS_initialize() loads the Mykonos device data structure
	 * settings for the Rx/Tx/ORx/Sniffer profiles, digital
	 * filter enables, calibrates the CLKPLL, and loads the user provided Rx
	 * gain tables.
	 */
	ret = MYKONOS_initialize(mykDevice);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	MYKONOS_getProductId(phy->mykDevice, &phy->device_id);
	if (phy->device_id != AD937x_PRODID(phy)) {
		if (!(IS_AD9375(phy) && (phy->device_id == (ID_AD9375_ALT & 0xFF)))) {
			dev_err(&phy->spi->dev, "Failed product ID, expected 0x%X got 0x%X",
				AD937x_PRODID(phy), phy->device_id );
			return -ENODEV;
		}
	}

	/*******************************/
	/***** CLKPLL Status Check *****/
	/*******************************/
	ret = MYKONOS_checkPllsLockStatus(mykDevice, &pllLockStatus);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;

	}

	priv->pllLock = BIT(0); /* CLKPLL Locked */

	if (!(pllLockStatus & priv->pllLock)) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	/*******************************************************/
	/**** Perform MultiChip Sync (MCS) on Mykonos Device ***/
	/*******************************************************/
	ret = MYKONOS_enableMultichipSync(mykDevice, 1, NULL);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;

	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9371_jesd204_setup_stage1(struct jesd204_dev *jdev,
					 enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	uint8_t mcsStatus = 0;
	uint8_t pllLockStatus = 0;
	int ret, retry = 3;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	/*******************/
	/***** MCS ****/
	/*******************/

	do {
		msleep(1);
		ret = MYKONOS_enableMultichipSync(mykDevice, 0, &mcsStatus);
	} while (((mcsStatus & 0x0B) != 0x0B) && retry--);

	if (((mcsStatus & 0x0B) != 0x0B) || ret) {
		/*** < MCS failed - ensure MCS before proceeding - user code here > ***/
		dev_err(&phy->spi->dev, "MCS failed (0x%X)", mcsStatus);
		return -EFAULT;
	}

	/*************************/
	/**** Load Mykonos ARM ***/
	/*************************/

	ret = MYKONOS_initArm(mykDevice);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = MYKONOS_loadArmFromBinary(mykDevice,
					     (u8 *) phy->fw->data,
					     phy->fw->size);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	/*******************************/
	/**** Set RF PLL Frequencies ***/
	/*******************************/
	if (priv->has_rx) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, RX_PLL,
						mykDevice->rx->rxPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		priv->pllLock |= BIT(1); /* RX_PLL Locked */
	}
	if (priv->has_tx) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, TX_PLL,
						mykDevice->tx->txPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		priv->pllLock |= BIT(2); /* TX_PLL Locked */
	}
	if (priv->has_orx) {
		ret = MYKONOS_setRfPllFrequency(mykDevice, SNIFFER_PLL,
						mykDevice->obsRx->snifferPllLoFrequency_Hz);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		priv->pllLock |= BIT(3); /* SNIFFER_PLL Locked */
	}

	msleep(4);

	ret = MYKONOS_checkPllsLockStatus(mykDevice, &pllLockStatus);
	if ((pllLockStatus & priv->pllLock) != priv->pllLock) {
		dev_err(&phy->spi->dev, "PLLs unlocked %x", pllLockStatus & 0x0F);
		return -EFAULT;
	}

	if (IS_AD9375(phy) && priv->has_tx) {
		ret = MYKONOS_configDpd(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}

		ret = MYKONOS_configClgc(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}

		ret = MYKONOS_configVswr(mykDevice);
		if (ret != MYKONOS_ERR_OK) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}

		phy->dpd_actuator_en[0] = true;
		phy->dpd_actuator_en[1] = true;

	}

	/*****************************************************/
	/*** Mykonos ARM Initialization Calibrations       ***/
	/*****************************************************/

	phy->init_cal_mask |= TX_BB_FILTER | ADC_TUNER | TIA_3DB_CORNER | DC_OFFSET |
			       TX_ATTENUATION_DELAY | RX_GAIN_DELAY | FLASH_CAL |
			       PATH_DELAY | LOOPBACK_RX_LO_DELAY | LOOPBACK_RX_RX_QEC_INIT |
			       RX_LO_DELAY;

	ret = ad9371_init_cal(phy, phy->init_cal_mask);
	if (ret != MYKONOS_ERR_OK) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	return JESD204_STATE_CHANGE_DONE;
}


static int ad9371_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	switch (lnk->link_id) {
	case DEFRAMER_LINK_TX:
		ret = MYKONOS_enableSysrefToDeframer(mykDevice, 0);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		ret = MYKONOS_resetDeframer(mykDevice);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	case FRAMER_LINK_RX:
		ret = MYKONOS_enableSysrefToRxFramer(mykDevice, 0);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	case FRAMER_LINK_ORX:
		ret = MYKONOS_enableSysrefToObsRxFramer(mykDevice, 0);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	default:
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9371_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	switch (lnk->link_id) {
	case DEFRAMER_LINK_TX:
		ret = MYKONOS_enableSysrefToDeframer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	case FRAMER_LINK_RX:
		ret = MYKONOS_enableSysrefToRxFramer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	case FRAMER_LINK_ORX:
		ret = MYKONOS_enableSysrefToObsRxFramer(mykDevice, 1);
		if (ret) {
			dev_err(&phy->spi->dev, "%s (%d)",
				getMykonosErrorMessage(ret), ret);
			return -EFAULT;
		}
		break;
	default:
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9371_jesd204_link_running(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	int i, ret;
	uint8_t deframerStatus = 0;
	uint8_t obsFramerStatus = 0;
	uint16_t mismatch = 0;
	uint8_t framerStatus = 0;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (!lnk->num_converters)
		return JESD204_STATE_CHANGE_DONE;

	switch (lnk->link_id) {
	case DEFRAMER_LINK_TX:
		ret = MYKONOS_readDeframerStatus(mykDevice, &deframerStatus);
		if (deframerStatus != 0x28)
			dev_warn(&phy->spi->dev, "deframerStatus (0x%X)", deframerStatus);

		ret = MYKONOS_jesd204bIlasCheck(mykDevice, &mismatch);
		if (mismatch) {
			dev_warn(&phy->spi->dev, "ILAS mismatch: %04x\n", mismatch);
			for (i = 0; i < ARRAY_SIZE(ad9371_ilas_mismatch_table); i++) {
				if (mismatch & BIT(i))
					dev_warn(&phy->spi->dev, "ILAS %s did not match\n",
						ad9371_ilas_mismatch_table[i]);
			}
		}
		break;
	case FRAMER_LINK_RX:
		ret = MYKONOS_readRxFramerStatus(mykDevice, &framerStatus);
		if (framerStatus != 0x3E)
			dev_warn(&phy->spi->dev, "framerStatus (0x%X)", framerStatus);
		break;
	case FRAMER_LINK_ORX:
		ret = MYKONOS_readOrxFramerStatus(mykDevice, &obsFramerStatus);
		if (obsFramerStatus != 0x3E)
			dev_warn(&phy->spi->dev, "obsFramerStatus (0x%X)", obsFramerStatus);
		break;
	default:
		return -EINVAL;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9371_jesd204_post_running_stage(struct jesd204_dev *jdev,
					       enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9371_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9371_rf_phy *phy = priv->phy;
	mykonosDevice_t *mykDevice = phy->mykDevice;
	int ret;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	/* Allow Rx1/2 QEC tracking and Tx1/2 QEC tracking to run when in the radioOn state         */
	/* Tx calibrations will only run if radioOn and the obsRx path is set to OBS_INTERNAL_CALS  */

	ret = MYKONOS_enableTrackingCals(mykDevice, phy->tracking_cal_mask);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	/* Function to turn radio on, Enables transmitters and receivers */
	/* that were setup during MYKONOS_initialize() */

	ret = MYKONOS_setupObsRxAgc(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = MYKONOS_setupRxAgc(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = ad9371_set_radio_state(phy, RADIO_ON);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	/* Allow TxQEC to run when user is not actively using ORx receive path */
	ret = MYKONOS_setObsRxPathSource(mykDevice, OBS_INTERNALCALS);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = MYKONOS_setupAuxAdcs(mykDevice, 4, 1);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = MYKONOS_setupAuxDacs(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	ret = MYKONOS_setupGpio(mykDevice);
	if (ret) {
		dev_err(&phy->spi->dev, "%s (%d)",
			getMykonosErrorMessage(ret), ret);
		return -EFAULT;
	}

	clk_set_rate(phy->clks[RX_SAMPL_CLK],
		     mykDevice->rx->rxProfile->iqRate_kHz * 1000);
	clk_set_rate(phy->clks[OBS_SAMPL_CLK],
		     mykDevice->obsRx->orxProfile->iqRate_kHz * 1000);
	clk_set_rate(phy->clks[TX_SAMPL_CLK],
		     mykDevice->tx->txProfile->iqRate_kHz * 1000);

	phy->rf_bandwith[0] = mykDevice->rx->rxProfile->rfBandwidth_Hz;
	phy->rf_bandwith[1] = mykDevice->obsRx->orxProfile->rfBandwidth_Hz;
	phy->rf_bandwith[2] = mykDevice->tx->txProfile->primarySigBandwidth_Hz;

	ad9371_info(phy);

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9371_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9371_jesd204_link_init,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9371_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = ad9371_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9371_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9371_jesd204_link_running,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = ad9371_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = ad9371_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.num_retries = 3,
	.max_num_links = 3,
	.sizeof_priv = sizeof(struct ad9371_jesd204_priv),
};

static int ad9371_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9371_rf_phy *phy;
	struct jesd204_dev *jdev;
	struct clk *clk = NULL;
	int ret;
	bool clk_is_tx = 0;

	dev_info(&spi->dev, "%s : enter", __func__);

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9371_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	if (!jdev) {
		clk = devm_clk_get(&spi->dev, "jesd_rx_clk");
		if (IS_ERR(clk) && PTR_ERR(clk) == -ENOENT) {
			clk = devm_clk_get(&spi->dev, "jesd_tx_clk");
			clk_is_tx = true;
		}

		if (IS_ERR(clk))
			return PTR_ERR(clk);
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*phy));
	if (indio_dev == NULL)
		return -ENOMEM;

	phy = iio_priv(indio_dev);
	phy->indio_dev = indio_dev;
	phy->spi = spi;
	phy->jdev = jdev;

	ret = ad9371_alloc_mykonos_device(phy);
	if (ret < 0)
		return ret;

	phy->pdata = ad9371_phy_parse_dt(indio_dev, &spi->dev);
	if (phy->pdata == NULL)
		return -EINVAL;

	phy->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	ad9371_reset(phy);

	phy->sysref_req_gpio = devm_gpiod_get(&spi->dev, "sysref_req",
					      GPIOD_OUT_HIGH);

	phy->test_gpio = devm_gpiod_get_optional(&spi->dev, "test",
						 GPIOD_OUT_LOW);

	phy->mykDevice->spiSettings->spi 		 = spi;
	phy->mykDevice->spiSettings->writeBitPolarity    = 0;
	phy->mykDevice->spiSettings->longInstructionWord = 1;
	phy->mykDevice->spiSettings->MSBFirst            = 1;
	phy->mykDevice->spiSettings->enSpiStreaming      = 0;
	phy->mykDevice->spiSettings->autoIncAddrUp       = 1;
	phy->mykDevice->spiSettings->fourWireMode        = 1;


	phy->dev_clk = devm_clk_get(&spi->dev, "dev_clk");
	if (IS_ERR(phy->dev_clk))
		return PTR_ERR(phy->dev_clk);

	ret = clk_prepare_enable(phy->dev_clk);
	if (ret)
		return ret;

	if (!phy->jdev) {
		if (clk_is_tx) {
			phy->jesd_tx_clk = clk;
		} else {
			phy->jesd_rx_clk = clk;

			phy->jesd_tx_clk = devm_clk_get_optional(&spi->dev, "jesd_tx_clk");
			if (IS_ERR(phy->jesd_tx_clk))
				return PTR_ERR(phy->jesd_tx_clk);
		}

		phy->jesd_rx_os_clk = devm_clk_get_optional(&spi->dev, "jesd_rx_os_clk");
		if (IS_ERR(phy->jesd_rx_os_clk))
			return PTR_ERR(phy->jesd_rx_os_clk);

		phy->fmc_clk = devm_clk_get(&spi->dev, "fmc_clk");
		if (IS_ERR(phy->fmc_clk))
			return PTR_ERR(phy->fmc_clk);

		ret = clk_prepare_enable(phy->fmc_clk);
		if (ret)
			return ret;

		phy->sysref_dev_clk = devm_clk_get_optional(&spi->dev, "sysref_dev_clk");
		if (IS_ERR(phy->sysref_dev_clk))
			return PTR_ERR(phy->sysref_dev_clk);

		phy->sysref_fmc_clk = devm_clk_get_optional(&spi->dev, "sysref_fmc_clk");
		if (IS_ERR(phy->sysref_fmc_clk))
			return PTR_ERR(phy->sysref_fmc_clk);

		ret = clk_prepare_enable(phy->sysref_dev_clk);
		if (ret)
			return ret;

		ret = clk_prepare_enable(phy->sysref_fmc_clk);
		if (ret)
			return ret;
	}

	ret = request_firmware(&phy->fw, FIRMWARE, &spi->dev);
	if (ret) {
		dev_err(&spi->dev,
			"request_firmware() failed with %d\n", ret);
		return ret;
	}

	if (!phy->jdev) {
		ret = ad9371_setup(phy);
		if (ret < 0) {
			/* Try once more */
			ret = ad9371_setup(phy);
			if (ret < 0)
				goto out_unregister_notifier;
		}
	}

	ad9371_clk_register(phy, "-rx_sampl_clk", NULL, NULL,
			    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED , RX_SAMPL_CLK);

	ad9371_clk_register(phy, "-obs_sampl_clk", NULL, NULL,
			    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED, OBS_SAMPL_CLK);

	ad9371_clk_register(phy, "-tx_sampl_clk", NULL, NULL,
			    CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED, TX_SAMPL_CLK);

	phy->clk_data.clks = phy->clks;
	phy->clk_data.clk_num = NUM_AD9371_CLKS;

	ret = of_clk_add_provider(spi->dev.of_node,
				  of_clk_src_onecell_get, &phy->clk_data);
	if (ret)
		goto out_disable_clocks;

	if (!IS_AD9375(phy)) {
		int i;

		for (i = 0; i < ARRAY_SIZE(ad9371_phy_tx_ext_info) &&
			ad9371_phy_tx_ext_info[i].private != TX_DPD ; i++);

		ad9371_phy_tx_ext_info[i] = (struct iio_chan_spec_ext_info){ };
	}
	sysfs_bin_attr_init(&phy->bin);
	phy->bin.attr.name = "profile_config";
	phy->bin.attr.mode = S_IWUSR | S_IRUGO;
	phy->bin.write = ad9371_profile_bin_write;
	phy->bin.read = ad9371_profile_bin_read;
	phy->bin.size = 8192;

	sysfs_bin_attr_init(&phy->bin_gt);
	phy->bin_gt.attr.name = "gain_table_config";
	phy->bin_gt.attr.mode = S_IWUSR;
	phy->bin_gt.write = ad9371_gt_bin_write;
	phy->bin_gt.size = 32768;

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = "ad9371-phy";

	indio_dev->info = IS_AD9375(phy) ? &ad9375_phy_info : &ad9371_phy_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9371_phy_chan;
	indio_dev->num_channels = ARRAY_SIZE(ad9371_phy_chan);

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto out_clk_del_provider;

	ret = ad9371_register_axi_converter(phy);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = sysfs_create_bin_file(&indio_dev->dev.kobj, &phy->bin_gt);
	if (ret < 0)
		goto out_iio_device_unregister;

	ret = ad9371_register_debugfs(indio_dev);
	if (ret < 0)
		dev_warn(&spi->dev, "%s: failed to register debugfs", __func__);

	if (!phy->jdev)
		ad9371_info(phy);

	ret = jesd204_fsm_start(phy->jdev, JESD204_LINKS_ALL);
	if (ret)
		goto out_iio_device_unregister;

	return 0;

out_iio_device_unregister:
	iio_device_unregister(indio_dev);
out_clk_del_provider:
	of_clk_del_provider(spi->dev.of_node);
out_disable_clocks:
	//ad9371_clks_disable(phy);
out_unregister_notifier:
	release_firmware(phy->fw);

	return ret;
}

static int ad9371_remove(struct spi_device *spi)
{
	struct ad9371_rf_phy *phy = ad9371_spi_to_phy(spi);

	release_firmware(phy->fw);
	sysfs_remove_bin_file(&phy->indio_dev->dev.kobj, &phy->bin);
	sysfs_remove_bin_file(&phy->indio_dev->dev.kobj, &phy->bin_gt);
	iio_device_unregister(phy->indio_dev);
 	of_clk_del_provider(spi->dev.of_node);

	return 0;
}

static const struct spi_device_id ad9371_id[] = {
	{"ad9371", ID_AD9371},
	{"ad9375", ID_AD9375},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9371_id);

static struct spi_driver ad9371_driver = {
	.driver = {
		.name	= "ad9371",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9371_probe,
	.remove		= ad9371_remove,
	.id_table	= ad9371_id,
};
module_spi_driver(ad9371_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9371 ADC");
MODULE_LICENSE("GPL v2");
