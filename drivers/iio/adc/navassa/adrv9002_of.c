// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9002 RF Transceiver - OF interface
 *
 * Copyright 2022 Analog Devices Inc.
 */
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#include "adrv9002.h"
#include "adi_adrv9001_dpd_types.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_mcs_types.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_radio.h"

#define ADRV9002_OF_U32_GET_VALIDATE(dev, node, key, def, min, max,		\
				     val, mandatory) ({				\
	u32 tmp;								\
	int ret, __ret = 0;							\
	const char *__key = key;						\
										\
	val = def;								\
	ret = of_property_read_u32(node, __key, &tmp);				\
	if (!ret) {								\
		if (tmp < (min) || tmp > (max)) {				\
			dev_err(dev, "Invalid value(%d) for \"%s\"\n",		\
				tmp, __key);					\
			__ret = -EINVAL;					\
		} else {							\
			val = tmp;						\
		}								\
	} else if (mandatory) {							\
		dev_err(dev, "Failed to get mandatory prop: \"%s\", ret=%d\n",	\
			__key, ret);						\
		__ret = ret;							\
	}									\
										\
	__ret;									\
})

#define OF_ADRV9002_PINCTL(phy, node, key, def, min, max, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&(phy)->spi->dev, node, key, def, min, \
				     max, val, mandatory)

#define OF_ADRV9002_DGPIO(phy, node, key, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&(phy)->spi->dev, node, key, ADI_ADRV9001_GPIO_UNASSIGNED, \
				     ADI_ADRV9001_GPIO_DIGITAL_00, ADI_ADRV9001_GPIO_DIGITAL_15, \
				     val, mandatory)

#define OF_ADRV9002_FH(phy, node, key, def, min, max, val, mandatory) \
	ADRV9002_OF_U32_GET_VALIDATE(&(phy)->spi->dev, node, key, def, min, max, val, mandatory)

static int adrv9002_parse_hop_signal_ports(struct adrv9002_rf_phy *phy,
					   const struct device_node *node, int hop, bool tx)
{
	const char *prop = tx ? "adi,fh-hop-tx-ports" : "adi,fh-hop-rx-ports";
	const enum adi_adrv9001_FhHopSignal hop_signals[] = {
		ADI_ADRV9001_FH_HOP_SIGNAL_1, ADI_ADRV9001_FH_HOP_SIGNAL_2
	};
	int p, nports;

	nports = of_property_count_elems_of_size(node, prop, sizeof(u32));
	if (nports > ADRV9002_CHANN_MAX) {
		dev_err(&phy->spi->dev, "More than %d elements in %s\n", ADRV9002_CHANN_MAX, prop);
		return -EINVAL;
	}

	for (p = 0; p < nports; p++) {
		u32 port;

		of_property_read_u32_index(node, prop, p, &port);
		if (port > ADRV9002_CHANN_2) {
			dev_err(&phy->spi->dev, "Invalid port:%d given in %s\n", port, prop);
			return -EINVAL;
		}

		if (tx)
			phy->fh.txPortHopSignals[port] = hop_signals[hop];
		else
			phy->fh.rxPortHopSignals[port] = hop_signals[hop];
	}

	return 0;
}

static int adrv9002_parse_hop_signal(struct adrv9002_rf_phy *phy,
				     const struct device_node *node, int hop)
{
	struct device_node *child;
	adi_adrv9001_FhhopTableSelectCfg_t *hop_tbl = &phy->fh.hopTableSelectConfig;
	int ret;
	const char *hop_str = hop ? "adi,fh-hop-signal-2" : "adi,fh-hop-signal-1";

	child = of_get_child_by_name(node, hop_str);
	if (!child)
		return 0;

	/* check that hop2 is actually supported and valid */
	if (hop && phy->fh.mode != ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP) {
		dev_err(&phy->spi->dev, "adi,fh-hop-signal-2 given but adi,fh-mode not in dual hop");
		ret = -EINVAL;
		goto of_put;
	}

	ret = OF_ADRV9002_DGPIO(phy, child, "adi,fh-hop-pin",
				phy->fh.hopSignalGpioConfig[hop].pin, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_DGPIO(phy, child, "adi,fh-hop-table-select-pin",
				hop_tbl->hopTableSelectGpioConfig[hop].pin, false);
	if (ret)
		goto of_put;

	/*
	 * On commom mode, gpio[0] is used to control both hop tables select. Thus,
	 * error out already if we try to assign a gpio to hop_2
	 */
	if (hop_tbl->hopTableSelectMode == ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON && hop) {
		if (hop_tbl->hopTableSelectGpioConfig[hop].pin != ADI_ADRV9001_GPIO_UNASSIGNED) {
			dev_err(&phy->spi->dev,
				"Table select mode set to common. Cannot assign gpio for hop signal 2\n");
			ret = -EINVAL;
			goto of_put;
		}
	}

	ret = adrv9002_parse_hop_signal_ports(phy, child, hop, false);
	if (ret)
		goto of_put;

	ret = adrv9002_parse_hop_signal_ports(phy, child, hop, true);
of_put:
	of_node_put(child);
	return ret;
}

static int adrv9002_fh_parse_table_control(struct adrv9002_rf_phy *phy,
					   const struct device_node *fh)
{
	int ret;
	u32 pin, p;

	if (phy->fh.tableIndexCtrl != ADI_ADRV9001_TABLEINDEXCTRL_GPIO)
		return 0;

	ret = OF_ADRV9002_DGPIO(phy, fh, "adi,fh-table-control-pin", pin, true);
	if (ret)
		return ret;

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-table-control-npins", 0, 1,
			     ADI_ADRV9001_FH_MAX_NUM_FREQ_SELECT_PINS,
			     phy->fh.numTableIndexPins, true);
	if (ret)
		return ret;

	/* assign gpios now */
	for (p = 0; p < phy->fh.numTableIndexPins; p++) {
		if ((pin + p) > ADI_ADRV9001_GPIO_DIGITAL_15) {
			dev_err(&phy->spi->dev, "Invalid DGPIO given for tbl ctl: %d\n", pin + p);
			return -EINVAL;
		}
		phy->fh.tableIndexGpioConfig[p].pin = pin + p;
	}

	return 0;
}

#define assign_fh_gpio_ctl_table(node, prop, sz, tbl) {			\
	typeof(tbl) __tbl = (tbl);					\
	u32 p;								\
									\
	for (p = 0; p < (sz); p++) {					\
		u32 temp;						\
									\
		of_property_read_u32_index(node, (prop), p, &temp);	\
		__tbl[p] = temp;					\
	}								\
}

static int adrv9002_fh_parse_chan_gpio_gain_control_chan(struct adrv9002_rf_phy *phy,
							 const struct device_node *fh, int chan)
{
	int ret;
	adi_adrv9001_FhGainSetupByPinCfg_t *gpio_gain = &phy->fh.gainSetupByPinConfig[chan];
	u32 pin, p;
	int size;

	ret = OF_ADRV9002_DGPIO(phy, fh, "adi,fh-gain-select-pin", pin, true);
	if (ret)
		return ret;

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-gain-select-npins", 0, 1,
			     ADI_ADRV9001_FH_MAX_NUM_GAIN_SELECT_PINS,
			     gpio_gain->numGainCtrlPins, true);
	if (ret)
		return ret;

	/* assign gpios now */
	for (p = 0; p < gpio_gain->numGainCtrlPins; p++) {
		if ((pin + p) > ADI_ADRV9001_GPIO_DIGITAL_15) {
			dev_err(&phy->spi->dev, "Invalid DGPIO given for gain ctl: %d\n", pin + p);
			return -EINVAL;
		}
		gpio_gain->gainSelectGpioConfig[p].pin = pin + p;
	}

	/*
	 * The table size depends on the number of gpios used to control it. Hence, it cannot
	 * be bigger than 1 << gpio_gain->numGainCtrlPins
	 */
	size = of_property_count_elems_of_size(fh, "adi,fh-rx-gain-table", sizeof(u32));
	if (size <= 0 || size > (1 << gpio_gain->numGainCtrlPins)) {
		dev_err(&phy->spi->dev, "Invalid size:%d for fh rx gain table\n", size);
		return -EINVAL;
	}

	assign_fh_gpio_ctl_table(fh, "adi,fh-rx-gain-table", size, &gpio_gain->rxGainTable[0]);
	gpio_gain->numRxGainTableEntries = size;

	size = of_property_count_elems_of_size(fh, "adi,fh-tx-atten-table", sizeof(u32));
	if (size <= 0 || size > (1 << gpio_gain->numGainCtrlPins)) {
		dev_err(&phy->spi->dev, "Invalid size:%d for fh tx gain table\n", size);
		return -EINVAL;
	}

	assign_fh_gpio_ctl_table(fh, "adi,fh-tx-atten-table", size, &gpio_gain->txAttenTable[0]);
	gpio_gain->numTxAttenTableEntries = size;

	return 0;
}

static int adrv9002_fh_parse_gpio_gain_control(struct adrv9002_rf_phy *phy,
					       const struct device_node *node)
{
	struct device_node *gain, *child;
	int ret = 0, n_chann;

	gain = of_get_child_by_name(node, "adi,fh-gain-setup-by-pin");
	if (!gain)
		return 0;

	n_chann = of_get_available_child_count(gain);
	if (n_chann != ADRV9002_CHANN_MAX) {
		dev_err(&phy->spi->dev, "If set, Gain setup by pin must be set for both channels!\n");
		of_node_put(gain);
		return -EINVAL;
	}

	for_each_available_child_of_node(gain, child) {
		u32 chann;

		ret = of_property_read_u32(child, "reg", &chann);
		if (ret) {
			dev_err(&phy->spi->dev,
				"No reg property defined for gain pin setup channel\n");
			goto of_child_put;
		}

		if (chann > ADRV9002_CHANN_2) {
			dev_err(&phy->spi->dev,
				"Invalid value for gain pin setup channel: %d\n", chann);
			ret = -EINVAL;
			goto of_child_put;
		}

		ret = adrv9002_fh_parse_chan_gpio_gain_control_chan(phy, child, chann);
		if (ret)
			goto of_child_put;
	}

	phy->fh.gainSetupByPin = true;

	of_node_put(gain);
	return 0;

of_child_put:
	of_node_put(child);
	of_node_put(gain);
	return ret;
}

static int adrv9002_parse_fh_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	adi_adrv9001_FhhopTableSelectCfg_t *hop_tbl = &phy->fh.hopTableSelectConfig;
	struct device_node *fh;
	int ret;
	u64 lo;
	int hop;

	fh = of_get_child_by_name(node, "adi,frequency-hopping");
	if (!fh) {
		/* set default parameters */
		phy->fh.minRxGainIndex = ADI_ADRV9001_RX_GAIN_INDEX_MIN;
		phy->fh.maxRxGainIndex = ADI_ADRV9001_RX_GAIN_INDEX_MAX;
		phy->fh.maxTxAtten_mdB = ADRV9001_TX_MAX_ATTENUATION_MDB;
		phy->fh.minOperatingFrequency_Hz = ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ;
		phy->fh.maxOperatingFrequency_Hz = ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ;
		return 0;
	}

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-mode", ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS,
			     ADI_ADRV9001_FHMODE_LO_MUX_PREPROCESS,
			     ADI_ADRV9001_FHMODE_LO_RETUNE_REALTIME_PROCESS_DUAL_HOP,
			     phy->fh.mode, false);
	if (ret)
		goto of_put;

	if (of_property_read_bool(fh, "adi,fh-hop-table-select-common-en"))
		hop_tbl->hopTableSelectMode = ADI_ADRV9001_FHHOPTABLESELECTMODE_COMMON;

	for (hop = 0; hop < ADRV9002_FH_HOP_SIGNALS_NR; hop++) {
		ret = adrv9002_parse_hop_signal(phy, fh, hop);
		if (ret)
			goto of_put;
	}

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-min-rx-gain-idx", ADI_ADRV9001_RX_GAIN_INDEX_MIN,
			     ADI_ADRV9001_RX_GAIN_INDEX_MIN, ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.minRxGainIndex, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-max-rx-gain-idx", ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.minRxGainIndex, ADI_ADRV9001_RX_GAIN_INDEX_MAX,
			     phy->fh.maxRxGainIndex, false);
	if (ret)
		goto of_put;

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-min-tx-atten-mdb", 0, 0,
			     ADRV9001_TX_MAX_ATTENUATION_MDB, phy->fh.minTxAtten_mdB, false);
	if (ret)
		goto of_put;

	if (phy->fh.minTxAtten_mdB % ADRV9001_TX_ATTENUATION_RESOLUTION_MDB) {
		dev_err(&phy->spi->dev, "adi,fh-min-tx-atten-mdb must have %d resolution\n",
			ADRV9001_TX_ATTENUATION_RESOLUTION_MDB);
		ret = -EINVAL;
		goto of_put;
	}

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-max-tx-atten-mdb", ADRV9001_TX_MAX_ATTENUATION_MDB,
			     phy->fh.minTxAtten_mdB, ADRV9001_TX_MAX_ATTENUATION_MDB,
			     phy->fh.maxTxAtten_mdB, false);
	if (ret)
		goto of_put;

	if (phy->fh.maxTxAtten_mdB % ADRV9001_TX_ATTENUATION_RESOLUTION_MDB) {
		dev_err(&phy->spi->dev, "adi,fh-max-tx-atten-mdb must have %d resolution\n",
			ADRV9001_TX_ATTENUATION_RESOLUTION_MDB);
		ret = -EINVAL;
		goto of_put;
	}

	ret = of_property_read_u64(fh, "adi,fh-min-frequency-hz", &lo);
	if (!ret) {
		if (lo < ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid val(%llu) for adi,fh-min-frequency-hz\n",
				lo);
			ret = -EINVAL;
			goto of_put;
		}
		phy->fh.minOperatingFrequency_Hz = lo;
	} else {
		phy->fh.minOperatingFrequency_Hz = ADI_ADRV9001_FH_MIN_CARRIER_FREQUENCY_HZ;
	}

	ret = of_property_read_u64(fh, "adi,fh-max-frequency-hz", &lo);
	if (!ret) {
		if (lo < phy->fh.minOperatingFrequency_Hz ||
		    lo > ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ) {
			dev_err(&phy->spi->dev, "Invalid val(%llu) for adi,fh-max-frequency-hz\n",
				lo);
			ret = -EINVAL;
			goto of_put;
		}
		phy->fh.maxOperatingFrequency_Hz = lo;
	} else {
		phy->fh.maxOperatingFrequency_Hz = ADI_ADRV9001_FH_MAX_CARRIER_FREQUENCY_HZ;
	}

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-tx-analog-power-o-frame-delay", 0, 0,
			     ADI_ADRV9001_FH_MAX_TX_FE_POWERON_FRAME_DELAY,
			     phy->fh.txAnalogPowerOnFrameDelay, false);
	if (ret)
		goto of_put;

	phy->fh.rxZeroIfEnable = of_property_read_bool(fh, "adi-fh-rx-zero-if-en");
	of_property_read_u32(fh, "adi,fh-min-frame-us", &phy->fh.minFrameDuration_us);

	ret = OF_ADRV9002_FH(phy, fh, "adi,fh-table-control", ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP,
			     ADI_ADRV9001_TABLEINDEXCTRL_AUTO_LOOP,
			     ADI_ADRV9001_TABLEINDEXCTRL_GPIO, phy->fh.tableIndexCtrl, false);
	if (ret)
		goto of_put;

	ret = adrv9002_fh_parse_table_control(phy, fh);
	if (ret)
		goto of_put;

	ret = adrv9002_fh_parse_gpio_gain_control(phy, fh);
of_put:
	of_node_put(fh);
	return ret;
}

/*
 * We need to know the arm clock to validate some delays which means we
 * would need to have a profile loaded at this point where we still don't know
 * which one we should load. Hence we just validate the paremeters we can
 * at this point (so we can fail early) and "blindly" read the others. This is
 * not a problem since __all__ the parameters will be validated again when calling
 * adi_adrv9001_Radio_ChannelEnablementDelays_Configure() (assuming parameter
 * validations are enabled);
 */
static int adrv9002_parse_en_delays(const struct adrv9002_rf_phy *phy,
				    const struct device_node *node,
				    struct adrv9002_chan *chan)
{
	int ret;
	struct device_node *en_delay;
	struct adi_adrv9001_ChannelEnablementDelays *delays = &chan->en_delays_ns;

	en_delay = of_parse_phandle(node, "adi,en-delays", 0);
	if (!en_delay)
		return 0;

#define OF_ADRV9002_EN_DELAY(key, min, max, val) \
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, en_delay, key, 0, min, \
				     max, val, false)

	of_property_read_u32(en_delay, "adi,rise-to-on-delay-ns", &delays->riseToOnDelay);

	ret = OF_ADRV9002_EN_DELAY("adi,rise-to-analog-on-delay-ns", 0, delays->riseToOnDelay,
				   delays->riseToAnalogOnDelay);
	if (ret)
		goto of_en_delay_put;

	if (chan->port == ADI_TX) {
		of_property_read_u32(en_delay, "adi,fall-to-off-delay-ns", &delays->fallToOffDelay);

		ret = OF_ADRV9002_EN_DELAY("adi,hold-delay-ns", 0, delays->fallToOffDelay,
					   delays->holdDelay);
		if (ret)
			goto of_en_delay_put;
	} else {
		of_property_read_u32(en_delay, "adi,hold-delay-ns", &delays->holdDelay);

		ret = OF_ADRV9002_EN_DELAY("adi,fall-to-off-delay-ns", 0, delays->holdDelay,
					   delays->fallToOffDelay);
		if (ret)
			goto of_en_delay_put;
	}

	of_property_read_u32(en_delay, "adi,guard-delay-ns", &delays->guardDelay);

of_en_delay_put:
	of_node_put(en_delay);

	return ret;
}

static int adrv9002_parse_dpd_pre_calib(const struct adrv9002_rf_phy *phy,
					const struct device_node *dpd, struct adrv9002_tx_chan *tx)
{
	u32 val;
	int ret;

#define OF_ADRV9002_DPD_INIT(key, def, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, dpd, key, def, min, max, val, false)

	/* values are scaled by 1000 for fractional values */
	ret = OF_ADRV9002_DPD_INIT("adi,pre-lut", 2000, 1000, 3750, val);
	if (ret)
		return ret;

	/* U2.2 value */
	tx->dpd_init->preLutScale = DIV_ROUND_CLOSEST(val << 2, 1000);

	ret = OF_ADRV9002_DPD_INIT("adi,lut-size", ADI_ADRV9001_DPDLUTSIZE_512,
				   ADI_ADRV9001_DPDLUTSIZE_256, ADI_ADRV9001_DPDLUTSIZE_512,
				   tx->dpd_init->lutSize);
	if (ret)
		return ret;

	tx->dpd_init->clgcEnable = of_property_read_bool(dpd, "adi,close-loop-gain");

	ret = of_property_count_u32_elems(dpd, "adi,model-tap-orders");
	if (ret < 0)
		return 0;
	if (ret != ARRAY_SIZE(tx->dpd_init->modelOrdersForEachTap)) {
		dev_err(&phy->spi->dev, "Invalid number of taps(%u) for tx(%u)\n", ret,
			tx->channel.number);
		return -EINVAL;
	}

	tx->dpd_init->changeModelTapOrders = true;
	return of_property_read_u32_array(dpd, "adi,model-tap-orders",
					  tx->dpd_init->modelOrdersForEachTap,
					  ARRAY_SIZE(tx->dpd_init->modelOrdersForEachTap));
}

static int adrv9002_parse_dpd_config(const struct adrv9002_rf_phy *phy,
				     const struct device_node *dpd, struct adrv9002_tx_chan *tx)
{
	u64 gain_target;
	u32 val;
	int ret;

#define OF_ADRV9002_DPD(key, def, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, dpd, key, def, min, max, val, false)

	ret = OF_ADRV9002_DPD("adi,samples-number", 4096, 1024, 4096, tx->dpd->numberOfSamples);
	if (ret)
		return ret;

	ret = OF_ADRV9002_DPD("adi,additional-power-scale", 4, 0, UINT_MAX,
			      tx->dpd->additionalPowerScale);
	if (ret)
		return ret;

	/*
	 * Ideally, the next four properties would be given in dBFS but since we do not
	 * have floating point in the kernel, deriving the raw values would not be possible
	 * (without huge approximations).
	 * Some help can be given on how to get here though... Assuming we know the dBFS value:
	 *	dBFS = 10 * log(x), where x = signal / signal_peak
	 *
	 *	x = 10^(dBFS/10)
	 *	raw = x * 2^30 (as these are U2.30 values)
	 */
	ret = OF_ADRV9002_DPD("adi,rxtx-normalization-lower-threshold", 3395469, 0, 1 << 30,
			      tx->dpd->rxTxNormalizationLowerThreshold);
	if (ret)
		return ret;

	ret = OF_ADRV9002_DPD("adi,rxtx-normalization-upper-threshold", 33954696, 0, 1 << 30,
			      tx->dpd->rxTxNormalizationUpperThreshold);
	if (ret)
		return ret;

	/*
	 * The next two values is the same story as the above with the difference that the
	 * values are U1.31.
	 */
	ret = OF_ADRV9002_DPD("adi,detection-power-threshold", 0, 0, 1 << 31,
			      tx->dpd->detectionPowerThreshold);
	if (ret)
		return ret;

	ret = OF_ADRV9002_DPD("adi,detection-peak-threshold", 0, 0, 1 << 31,
			      tx->dpd->detectionPeakThreshold);
	if (ret)
		return ret;

	ret = OF_ADRV9002_DPD("adi,counts-less-power-threshold", 4096, 0, USHRT_MAX,
			      tx->dpd->detectionPowerThreshold);
	if (ret)
		return ret;

	ret = OF_ADRV9002_DPD("adi,counts-greater-peak-threshold", 0, 0, USHRT_MAX,
			      tx->dpd->detectionPeakThreshold);
	if (ret)
		return ret;

	/* U1.31 value. Value given in dts with micro granularity */
	ret = OF_ADRV9002_DPD("adi,time-filter-coefficient", 0, 0, 1000000, val);
	if (ret)
		return ret;

	tx->dpd->timeFilterCoefficient = DIV_ROUND_CLOSEST_ULL((u64)val << 31, 1000000);
	/*
	 * for the max value we get 2147483648 but for the API the max is 2147483647
	 * (which also gives 1.0 when doing the reverse math) so decrement one in that case.
	 * Yeahh, it does look bad...
	 */
	if (tx->dpd->timeFilterCoefficient == 1 << 31)
		tx->dpd->timeFilterCoefficient--;

	tx->dpd->clgcLoopOpen = of_property_read_bool(dpd, "adi,clgc-open-loop");
	tx->dpd->immediateLutSwitching = of_property_read_bool(dpd, "adi,immediate_lut_switch");

	ret = of_property_read_u64(dpd, "adi,clgc-gain-target-mdB", &gain_target);
	if (!ret) {
		if ((s64)gain_target < INT_MIN * 10LL || (s64)gain_target > INT_MAX * 10LL) {
			dev_err(&phy->spi->dev, "Invalid value(%lld) for adi,clgc-gain-target-mdB\n",
				gain_target);
			return -EINVAL;
		}

		tx->dpd->clgcGainTarget_HundredthdB = div_s64(gain_target, 10);
	}

	/* U1.31 value. Value given in dts with micro granularity */
	ret = OF_ADRV9002_DPD("adi,clg-filter-alpha", 0, 0, 1000000, val);
	if (ret)
		return ret;

	tx->dpd->clgcFilterAlpha = DIV_ROUND_CLOSEST_ULL((u64)val << 31, 1000000);

	return OF_ADRV9002_DPD("adi,capture-delay-us", 0, 0, 1000000, val);
}

static int adrv9002_parse_dpd(const struct adrv9002_rf_phy *phy,
			      const struct device_node *node, struct adrv9002_tx_chan *tx)
{
	struct device_node *dpd;
	int ret;

	if (!of_property_read_bool(node, "adi,dpd"))
		return 0;

	tx->dpd_init = devm_kzalloc(&phy->spi->dev, sizeof(*tx->dpd_init), GFP_KERNEL);
	if (!tx->dpd_init)
		return -ENOMEM;

	tx->dpd = devm_kzalloc(&phy->spi->dev, sizeof(*tx->dpd), GFP_KERNEL);
	if (!tx->dpd)
		return -ENOMEM;

	tx->dpd_init->enable = true;
	/* not configurable */
	tx->dpd_init->amplifierType = ADI_ADRV9001_DPDAMPLIFIER_DEFAULT;
	tx->dpd_init->model = ADI_ADRV9001_DPDMODEL_4;

	dpd = of_parse_phandle(node, "adi,dpd-config", 0);
	if (!dpd) {
		/* set default parameters */
		tx->dpd_init->lutSize = ADI_ADRV9001_DPDLUTSIZE_512;
		tx->dpd_init->preLutScale = 8;
		tx->dpd->numberOfSamples = 4096;
		tx->dpd->additionalPowerScale = 4;
		/* -25dBFS*/
		tx->dpd->rxTxNormalizationLowerThreshold = 3395470;
		/* -15dBFS */
		tx->dpd->rxTxNormalizationUpperThreshold = 33954698;
		/* disable it */
		tx->dpd->countsLessThanPowerThreshold = 4096;
		/*
		 * Should be set for FDD and in fact, TDD is only properly working
		 * when this is enabled.
		 */
		tx->dpd->immediateLutSwitching = true;
		return 0;
	}

	tx->ext_path_calib = of_property_read_bool(dpd, "adi,external-path-delay-calibrate");

	ret = adrv9002_parse_dpd_pre_calib(phy, dpd, tx);
	if (ret)
		goto of_dpd_put;

	ret = adrv9002_parse_dpd_config(phy, dpd, tx);
of_dpd_put:
	of_node_put(dpd);
	return ret;
}

/* there's no optional variant for this, so we need to check for -ENOENT */
static struct gpio_desc *devm_fwnode_gpiod_get_optional(struct device *dev,
							struct device_node *node,
							const char *con_id,
							enum gpiod_flags flags, const char *label)
{
	struct gpio_desc *desc;

	desc = devm_fwnode_gpiod_get(dev, of_fwnode_handle(node), con_id, flags, label);
	if (IS_ERR(desc) && PTR_ERR(desc) == -ENOENT)
		return NULL;

	return desc;
}

static int adrv9002_parse_tx_pin_dt(const struct adrv9002_rf_phy *phy,
				    struct device_node *node,
				    struct adrv9002_tx_chan *tx)
{
	struct device_node *pinctlr;
	int ret;

	pinctlr = of_parse_phandle(node, "adi,pinctrl", 0);
	if (!pinctlr)
		return 0;

	tx->pin_cfg = devm_kzalloc(&phy->spi->dev, sizeof(*tx->pin_cfg),
				   GFP_KERNEL);
	if (!tx->pin_cfg) {
		of_node_put(pinctlr);
		return -ENOMEM;
	}

	ret = OF_ADRV9002_DGPIO(phy, pinctlr, "adi,increment-pin", tx->pin_cfg->incrementPin, true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_DGPIO(phy, pinctlr, "adi,decrement-pin", tx->pin_cfg->decrementPin, true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_PINCTL(phy, pinctlr, "adi,step-size-mdB", 50, 50, 1550,
				 tx->pin_cfg->stepSize_mdB, false);
	/* extra validation since the value needs to be multiple of 50 */
	if (tx->pin_cfg->stepSize_mdB % 50 != 0) {
		dev_err(&phy->spi->dev, "adi,step-size-mdB must be multiple of 50\n");
		ret = -EINVAL;
	}

of_pinctrl_put:
	of_node_put(pinctlr);
	return ret;
}

static int adrv9002_parse_tx_dt(struct adrv9002_rf_phy *phy,
				struct device_node *node, const int channel)
{
	const char *mux_label_2 = channel ? "tx2-mux-ctl2" : "tx1-mux-ctl2";
	const char *mux_label = channel ? "tx2-mux-ctl" : "tx1-mux-ctl";
	struct adrv9002_tx_chan *tx = &phy->tx_channels[channel];
	int ret;

	if (channel >= phy->chip->n_tx) {
		dev_err(&phy->spi->dev, "TX%d not supported for this device\n", channel + 1);
		return -EINVAL;
	}

	if (of_property_read_bool(node, "adi,dac-full-scale-boost"))
		tx->dac_boost_en = true;

	ret = of_property_read_u64(node, "adi,carrier-hz", &tx->channel.carrier);
	if (!ret) {
		u64 min = ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ;
		u64 max = ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ;

		if (tx->channel.carrier < min || tx->channel.carrier > max) {
			dev_err(&phy->spi->dev, "Invalid carrier(%llu) for TX%d\n",
				tx->channel.carrier, channel + 1);
			return -EINVAL;
		}
	}

	ret = adrv9002_parse_en_delays(phy, node, &tx->channel);
	if (ret)
		return ret;

	tx->channel.mux_ctl = devm_fwnode_gpiod_get_optional(&phy->spi->dev, node, "mux-ctl",
							     GPIOD_OUT_HIGH, mux_label);
	if (IS_ERR(tx->channel.mux_ctl))
		return PTR_ERR(tx->channel.mux_ctl);

	tx->channel.mux_ctl_2 = devm_fwnode_gpiod_get_optional(&phy->spi->dev, node, "mux-ctl2",
							       GPIOD_OUT_HIGH, mux_label_2);
	if (IS_ERR(tx->channel.mux_ctl_2))
		return PTR_ERR(tx->channel.mux_ctl_2);

	ret = adrv9002_parse_dpd(phy, node, tx);
	if (ret)
		return ret;

	ret = adrv9002_parse_tx_pin_dt(phy, node, tx);
	if (ret)
		return ret;

	ret = ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, "adi,mcs-read-delay", 0,
					   0, 15, tx->channel.mcs_delay.readDelay, false);
	if (ret)
		return ret;

	return ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, "adi,mcs-sample-delay", 0, 0,
					    65535, tx->channel.mcs_delay.sampleDelay, false);
}

static int adrv9002_parse_rx_pinctl_dt(const struct adrv9002_rf_phy *phy,
				       const struct device_node *node,
				       struct adrv9002_rx_chan *rx)
{
	struct device_node *pinctlr;
	int ret;

	/* get pinctrl properties if any */
	pinctlr = of_parse_phandle(node, "adi,pinctrl", 0);
	if (!pinctlr)
		return 0;

	rx->pin_cfg = devm_kzalloc(&phy->spi->dev, sizeof(*rx->pin_cfg),
				   GFP_KERNEL);
	if (!rx->pin_cfg) {
		ret = -ENOMEM;
		goto of_pinctrl_put;
	}

	ret = OF_ADRV9002_PINCTL(phy, pinctlr, "adi,increment-step-size", 1, 1, 7,
				 rx->pin_cfg->incrementStepSize, false);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_PINCTL(phy, pinctlr, "adi,decrement-step-size", 1, 1, 7,
				 rx->pin_cfg->decrementStepSize, false);
	if (ret)
		goto of_pinctrl_put;

	/*
	 * Get gpios. This are mandatory properties. It makes no sense to
	 * pin crtl gain and no pins congigured to control it!
	 */
	ret = OF_ADRV9002_DGPIO(phy, pinctlr, "adi,increment-pin", rx->pin_cfg->incrementPin, true);
	if (ret)
		goto of_pinctrl_put;

	ret = OF_ADRV9002_DGPIO(phy, pinctlr, "adi,decrement-pin", rx->pin_cfg->decrementPin, true);
	if (ret)
		goto of_pinctrl_put;

of_pinctrl_put:
	of_node_put(pinctlr);
	return ret;
}

#define AGC_OFFSETOF(member)	\
	offsetof(struct adi_adrv9001_GainControlCfg, member)

static const struct {
	const char *key;
	const u32 __off;
	const u32 def;
	const u32 min;
	const u32 max;
	const u8 size;
} of_agc_props[] = {
	{"adi,peak-wait-time", AGC_OFFSETOF(peakWaitTime), 4, 0, 31, 1},
	{"adi,gain-update-counter",
	 AGC_OFFSETOF(gainUpdateCounter), 11520, 0, 4194303, 4},
	{"adi,attack-delax-us",
	 AGC_OFFSETOF(attackDelay_us), 10, 0, 63, 1},
	{"adi,slow-loop-settling-delay",
	 AGC_OFFSETOF(slowLoopSettlingDelay), 16, 0, 127, 1},
	{"adi,change-gain-threshold-high",
	 AGC_OFFSETOF(changeGainIfThreshHigh), 3, 0, 3, 1},
	{"adi,agc-mode", AGC_OFFSETOF(agcMode), 1, 0, 1, 1},
	{"adi,reset-on-rx-on-gain-index",
	 AGC_OFFSETOF(resetOnRxonGainIndex), ADRV9002_RX_MAX_GAIN_IDX, ADRV9002_RX_MIN_GAIN_IDX,
	 ADRV9002_RX_MAX_GAIN_IDX, 1},
	/* power detector */
	{"adi,power-under-range-high-threshold",
	 AGC_OFFSETOF(power.underRangeHighPowerThresh), 10, 0, 127, 1},
	{"adi,power-under-range-low-threshold",
	 AGC_OFFSETOF(power.underRangeLowPowerThresh), 4, 0, 15, 1},
	{"adi,power-under-range-high-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeHighPowerGainStepRecovery), 2, 0, 31, 1},
	{"adi,power-under-range-low-gain-step-recovery",
	 AGC_OFFSETOF(power.underRangeLowPowerGainStepRecovery), 4, 0, 31, 1},
	{"adi,power-measurement-duration",
	 AGC_OFFSETOF(power.powerMeasurementDuration), 10, 0, 31, 1},
	{"adi,power-measurement-delay",
	 AGC_OFFSETOF(power.powerMeasurementDelay), 2, 0, 255, 1},
	{"adi,power-rx-tdd-measurement-duration",
	 AGC_OFFSETOF(power.rxTddPowerMeasDuration), 0, 0, 65535, 2},
	{"adi,power-rx-tdd-measurement-delay",
	 AGC_OFFSETOF(power.rxTddPowerMeasDelay), 0, 0, 65535, 2},
	{"adi,power-over-range-high-threshold",
	 AGC_OFFSETOF(power.overRangeHighPowerThresh), 0, 0, 15, 1},
	{"adi,power-over-range-low-threshold",
	 AGC_OFFSETOF(power.overRangeLowPowerThresh), 7, 0, 127, 1},
	{"adi,power-over-range-high-gain-step-attack",
	 AGC_OFFSETOF(power.overRangeHighPowerGainStepAttack), 4, 0, 31, 1},
	{"adi,power-over-range-low-gain-step-attack",
	 AGC_OFFSETOF(power.overRangeLowPowerGainStepAttack), 4, 0, 31, 1},
	/* peak detector */
	{"adi,peak-agc-under-range-low-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeLowInterval), 50, 0, 65535, 2},
	{"adi,peak-agc-under-range-mid-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeMidInterval), 2, 0, 63, 1},
	{"adi,peak-agc-under-range-high-interval",
	 AGC_OFFSETOF(peak.agcUnderRangeHighInterval), 4, 0, 63, 1},
	{"adi,peak-apd-high-threshold",
	 AGC_OFFSETOF(peak.apdHighThresh), 21, 0, 63, 1},
	{"adi,peak-apd-low-threshold",
	 AGC_OFFSETOF(peak.apdLowThresh), 12, 0, 63, 1},
	{"adi,peak-apd-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-apd-lower-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.apdLowerThreshPeakExceededCount), 3, 0, 255, 1},
	{"adi,peak-apd-gain-step-attack",
	 AGC_OFFSETOF(peak.apdGainStepAttack), 2, 0, 31, 1},
	{"adi,peak-apd-gain-step-recovery",
	 AGC_OFFSETOF(peak.apdGainStepRecovery), 0, 0, 31, 1},
	{"adi,peak-hb-overload-duration-count",
	 AGC_OFFSETOF(peak.hbOverloadDurationCount), 1, 0, 7, 1},
	{"adi,peak-hb-overload-threshold-count",
	 AGC_OFFSETOF(peak.hbOverloadThreshCount), 1, 1, 15, 1},
	{"adi,peak-hb-high-threshold",
	 AGC_OFFSETOF(peak.hbHighThresh), 13044, 0, 16383, 2},
	{"adi,peak-hb-under-range-low-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeLowThresh), 5826, 0, 16383, 2},
	{"adi,peak-hb-under-range-mid-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeMidThresh), 8230, 0, 16383, 2},
	{"adi,peak-hb-under-range-high-threshold",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThresh), 7335, 0, 16383, 2},
	{"adi,peak-hb-upper-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUpperThreshPeakExceededCount), 6, 0, 255, 1},
	{"adi,peak-hb-under-range-high-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeHighThreshExceededCount), 3, 0, 255, 1},
	{"adi,peak-hb-gain-step-high-recover",
	 AGC_OFFSETOF(peak.hbGainStepHighRecovery), 2, 0, 31, 1},
	{"adi,peak-hb-gain-step-low-recovery",
	 AGC_OFFSETOF(peak.hbGainStepLowRecovery), 6, 0, 31, 1},
	{"adi,peak-hb-gain-step-mid-recovery",
	 AGC_OFFSETOF(peak.hbGainStepMidRecovery), 4, 0, 31, 1},
	{"adi,peak-hb-gain-step-attack",
	 AGC_OFFSETOF(peak.hbGainStepAttack), 2, 0, 31, 1},
	{"adi,peak-hb-overload-power-mode",
	 AGC_OFFSETOF(peak.hbOverloadPowerMode), 0, 0, 1, 1},
	{"adi,peak-hb-under-range-mid-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeMidThreshExceededCount), 3, 0, 255, 1},
	{"adi,peak-hb-under-range-low-threshold-exceeded-count",
	 AGC_OFFSETOF(peak.hbUnderRangeLowThreshExceededCount), 3, 0, 255, 1},
};

#define agc_assign_value(agc, prop, val) {			\
	typeof(prop) __prop = (prop);				\
	u32 __off = of_agc_props[__prop].__off;			\
								\
	switch (of_agc_props[__prop].size) {			\
	case 1:							\
		*(u8 *)((void *)(agc) + __off) = (val);		\
		break;						\
	case 2:							\
		*(u16 *)((void *)(agc) + __off) = (val);	\
		break;						\
	case 4:							\
		*(u32 *)((void *)(agc) + __off) = (val);	\
		break;						\
	};							\
}

static void adrv9002_set_agc_defaults(struct adi_adrv9001_GainControlCfg *agc)
{
	int prop;

	for (prop = 0; prop < ARRAY_SIZE(of_agc_props); prop++) {
		u32 temp = of_agc_props[prop].def;

		agc_assign_value(agc, prop, temp);
	}
	/*
	 * Since the enum is 0 (for now), we could just skipp this but I'm being paranoid and not
	 * trusting that this can't ever change...
	 */
	agc->power.feedback_apd_high_apd_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->power.feedback_inner_high_inner_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_low_hb_low = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_high_hb_high = ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED;
}

static int adrv9002_parse_rx_agc_dt(const struct adrv9002_rf_phy *phy,
				    const struct device_node *node,
				    struct adrv9002_rx_chan *rx)
{
	struct device_node *agc;
	int ret, prop;

#define ADRV9002_OF_AGC_PIN(key, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, key, \
				     ADI_ADRV9001_GPIO_PIN_CRUMB_UNASSIGNED, \
				     min, max, val, false)

	/* set boolean settings that are enabled by default */
	rx->agc.power.powerEnableMeasurement = true;
	rx->agc.peak.enableHbOverload = true;

	agc = of_parse_phandle(node, "adi,agc", 0);
	if (!agc) {
		adrv9002_set_agc_defaults(&rx->agc);
		return 0;
	}

	for (prop = 0; prop < ARRAY_SIZE(of_agc_props); prop++) {
		u32 temp;

		ret = of_property_read_u32(agc, of_agc_props[prop].key, &temp);
		if (ret) {
			temp = of_agc_props[prop].def;
		} else if (temp < of_agc_props[prop].min ||
			   temp > of_agc_props[prop].max) {
			dev_err(&phy->spi->dev, "%s not in valid range [%d %d]\n",
				of_agc_props[prop].key, of_agc_props[prop].min,
				of_agc_props[prop].max);
			of_node_put(agc);
			return -EINVAL;
		}

		agc_assign_value(&rx->agc, prop, temp);
	}

	/* boolean properties */
	if (of_property_read_bool(agc, "adi,low-threshold-prevent-gain-inc"))
		rx->agc.lowThreshPreventGainInc = true;

	if (of_property_read_bool(agc, "adi,sync-pulse-gain-counter-en"))
		rx->agc.enableSyncPulseForGainCounter = true;

	if (of_property_read_bool(agc, "adi,fast-recovery-loop-en"))
		rx->agc.enableFastRecoveryLoop = true;

	if (of_property_read_bool(agc, "adi,reset-on-rx-on"))
		rx->agc.resetOnRxon = true;

	if (of_property_read_bool(agc, "adi,no-power-measurement-en"))
		rx->agc.power.powerEnableMeasurement = false;

	if (of_property_read_bool(agc, "adi,no-peak-hb-overload-en"))
		rx->agc.peak.enableHbOverload = false;

	/* check if there are any gpios */
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-high-thres-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.power.feedback_inner_high_inner_low);
	if (ret)
		goto out;

	/* feedback pins*/
	ret = ADRV9002_OF_AGC_PIN("adi,agc-power-feedback-low-thres-gain-change",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.power.feedback_apd_high_apd_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-high-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.peak.feedback_apd_low_hb_low);
	if (ret)
		goto out;

	ret = ADRV9002_OF_AGC_PIN("adi,agc-peak-feedback-low-thres-counter-exceeded",
				  ADI_ADRV9001_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV9001_GPIO_PIN_CRUMB_15_14,
				  rx->agc.peak.feedback_apd_high_hb_high);
out:
	of_node_put(agc);
	return ret;
}

static int adrv9002_parse_rx_dt(struct adrv9002_rf_phy *phy,
				struct device_node *node,
				const int channel)
{
	const char *rxb_mux_label = channel ? "rx2b-mux-ctl" : "rx1b-mux-ctl";
	const char *mux_label = channel ? "rx2a-mux-ctl" : "rx1a-mux-ctl";
	const char *gpio_label = channel ? "orx2" : "orx1";
	struct adrv9002_rx_chan *rx = &phy->rx_channels[channel];
	int ret;
	u32 min_gain, max_gain;

	ret = adrv9002_parse_rx_agc_dt(phy, node, rx);
	if (ret)
		return ret;

	ret = adrv9002_parse_rx_pinctl_dt(phy, node, rx);
	if (ret)
		return ret;

	ret = adrv9002_parse_en_delays(phy, node, &rx->channel);
	if (ret)
		return ret;

	ret = of_property_read_u64(node, "adi,carrier-hz", &rx->channel.carrier);
	if (!ret) {
		u64 min = ADI_ADRV9001_CARRIER_FREQUENCY_MIN_HZ;
		u64 max = ADI_ADRV9001_CARRIER_FREQUENCY_MAX_HZ;

		if (rx->channel.carrier < min || rx->channel.carrier > max) {
			dev_err(&phy->spi->dev, "Invalid carrier(%llu) for RX%d\n",
				rx->channel.carrier, channel + 1);
			return -EINVAL;
		}
	}

	/* check min/max gain and assign to pinctrl and agc if there */
#define ADRV9002_OF_RX_OPTIONAL(key, def, min, max, val)	\
	ADRV9002_OF_U32_GET_VALIDATE(&phy->spi->dev, node, key, def, \
				     min, max, val, false)

	ret = ADRV9002_OF_RX_OPTIONAL("adi,min-gain-index", ADRV9002_RX_MIN_GAIN_IDX,
				      ADRV9002_RX_MIN_GAIN_IDX,
				      ADRV9002_RX_MAX_GAIN_IDX, min_gain);
	if (ret)
		return ret;

	ret = ADRV9002_OF_RX_OPTIONAL("adi,max-gain-index", ADRV9002_RX_MAX_GAIN_IDX,
				      min_gain, ADRV9002_RX_MAX_GAIN_IDX, max_gain);
	if (ret)
		return ret;

	rx->agc.maxGainIndex = max_gain;
	rx->agc.minGainIndex = min_gain;

	if (rx->pin_cfg) {
		rx->pin_cfg->maxGainIndex = max_gain;
		rx->pin_cfg->minGainIndex = min_gain;
	}

	/* there's no optional variant for this, so we need to check for -ENOENT */
	rx->orx_gpio = devm_fwnode_gpiod_get_optional(&phy->spi->dev, node, "orx", GPIOD_OUT_LOW,
						      gpio_label);
	if (IS_ERR(rx->orx_gpio))
		return PTR_ERR(rx->orx_gpio);

	rx->channel.mux_ctl = devm_fwnode_gpiod_get_optional(&phy->spi->dev, node, "mux-ctl",
							     GPIOD_OUT_HIGH, mux_label);
	if (IS_ERR(rx->channel.mux_ctl))
		return PTR_ERR(rx->channel.mux_ctl);

	rx->channel.mux_ctl_2 = devm_fwnode_gpiod_get_optional(&phy->spi->dev, node, "mux-ctl2",
							       GPIOD_OUT_HIGH, rxb_mux_label);
	if (IS_ERR(rx->channel.mux_ctl_2))
		return PTR_ERR(rx->channel.mux_ctl_2);

	ret = ADRV9002_OF_RX_OPTIONAL("adi,mcs-read-delay", 1, 1, 15,
				      rx->channel.mcs_delay.readDelay);
	if (ret)
		return ret;

	return ADRV9002_OF_RX_OPTIONAL("adi,mcs-sample-delay", 0, 0, 65535,
				       rx->channel.mcs_delay.sampleDelay);
}

static int adrv9002_parse_gpios_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	struct device_node *of_gpios, *child;
	struct device *dev = &phy->spi->dev;
	int ret = 0, idx = 0;

	of_gpios = of_get_child_by_name(node, "adi,gpios");
	if (!of_gpios)
		return 0;

	phy->ngpios = of_get_child_count(of_gpios);
	if (!phy->ngpios)
		goto of_gpio_put;

	phy->adrv9002_gpios = devm_kcalloc(&phy->spi->dev, phy->ngpios,
					   sizeof(*phy->adrv9002_gpios),
					   GFP_KERNEL);
	if (!phy->adrv9002_gpios) {
		ret = -ENOMEM;
		goto of_gpio_put;
	}

	for_each_available_child_of_node(of_gpios, child) {
		u32 gpio, polarity, master, signal;

		ret = of_property_read_u32(child, "reg", &gpio);
		if (ret) {
			dev_err(dev, "No reg property defined for gpio\n");
			goto of_child_put;
		}

		if (gpio < ADI_ADRV9001_GPIO_DIGITAL_00 ||
		    gpio > ADI_ADRV9001_GPIO_ANALOG_11) {
			dev_err(dev, "Invalid gpio number: %d\n", gpio);
			ret = -EINVAL;
			goto of_child_put;
		}

		phy->adrv9002_gpios[idx].gpio.pin = gpio;

		ret = of_property_read_u32(child, "adi,signal", &signal);
		if (ret) {
			dev_err(dev, "No adi,signal property defined for gpio%d\n", gpio);
			goto of_child_put;
		}

		if (signal >= ADI_ADRV9001_GPIO_NUM_SIGNALS) {
			dev_err(dev, "Invalid gpio signal: %d\n", signal);
			ret = -EINVAL;
			goto of_child_put;
		}

		phy->adrv9002_gpios[idx].signal = signal;

		ret = of_property_read_u32(child, "adi,polarity", &polarity);
		if (!ret) {
			if (polarity > ADI_ADRV9001_GPIO_POLARITY_INVERTED) {
				dev_err(dev, "Invalid gpio polarity: %d\n", polarity);
				ret = -EINVAL;
				goto of_child_put;
			}

			phy->adrv9002_gpios[idx].gpio.polarity = polarity;
		}

		ret = of_property_read_u32(child, "adi,master", &master);
		if (!ret) {
			if (master != ADI_ADRV9001_GPIO_MASTER_ADRV9001 &&
			    master != ADI_ADRV9001_GPIO_MASTER_BBIC) {
				dev_err(dev, "Invalid gpio master: %d\n", master);
				ret = -EINVAL;
				goto of_child_put;
			}

			phy->adrv9002_gpios[idx].gpio.master = master;
		}

		idx++;
	}

	of_node_put(of_gpios);
	return 0;

of_child_put:
	of_node_put(child);
of_gpio_put:
	of_node_put(of_gpios);
	return ret;
}

static int adrv9002_parse_channels_dt(struct adrv9002_rf_phy *phy, const struct device_node *node)
{
	struct device_node *of_channels, *child;
	struct device *dev = &phy->spi->dev;
	int ret;

	of_channels = of_get_child_by_name(node, "adi,channels");
	if (!of_channels)
		return 0;

	for_each_available_child_of_node(of_channels, child) {
		u32 chann, port;

		ret = of_property_read_u32(child, "reg", &chann);
		if (ret) {
			dev_err(dev, "No reg property defined for channel\n");
			goto of_error_put;
		}

		if (chann >= ADRV9002_CHANN_MAX) {
			dev_err(dev, "Invalid value for channel: %d\n", chann);
			ret = -EINVAL;
			goto of_error_put;
		}

		ret = of_property_read_u32(child, "adi,port", &port);
		if (ret) {
			dev_err(dev, "No port property defined for channel\n");
			goto of_error_put;
		}

		switch (port) {
		case ADI_TX:
			ret = adrv9002_parse_tx_dt(phy, child, chann);
			break;
		case ADI_RX:
			ret = adrv9002_parse_rx_dt(phy, child, chann);
			break;
		default:
			dev_err(dev, "Unknown port: %d\n", port);
			ret = -EINVAL;
			break;
		};

		if (ret)
			goto of_error_put;
	}

	of_node_put(of_channels);
	return 0;

of_error_put:
	of_node_put(child);
	of_node_put(of_channels);
	return ret;
}

int adrv9002_parse_dt(struct adrv9002_rf_phy *phy)
{
	struct device_node *parent = phy->spi->dev.of_node;
	int ret;

	ret = adrv9002_parse_channels_dt(phy, parent);
	if (ret)
		return ret;

	ret = adrv9002_parse_gpios_dt(phy, parent);
	if (ret)
		return ret;

	return adrv9002_parse_fh_dt(phy, parent);
}
