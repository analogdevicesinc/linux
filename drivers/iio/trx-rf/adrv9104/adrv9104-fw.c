// SPDX-License-Identifier: GPL-2.0
/*
 * ADRV9104 RF Transceiver - FW interface
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h>
#include <linux/types.h>

#include "adrv9104.h"
#include "adi_adrv910x_gpio_types.h"
#include "adi_adrv910x_rx_types.h"

static int adrv9104_fwnode_read_validate_u32(const struct device *dev,
					     const struct fwnode_handle *fwnode, const char *key,
					     u32 def, u32 min, u32 max, u32 *val, bool mandatory)
{
	int ret;

	*val = def;
	ret = fwnode_property_read_u32(fwnode, key, val);
	if (ret) {
		if (mandatory || ret != -EINVAL)
			return dev_err_probe(dev, ret, "Failed to get %s prop: %pfwP, ret=%d\n",
					     mandatory ? "mandatory" : "", fwnode, ret);

		return 0;
	}

	if (*val < min || *val > max)
		return dev_err_probe(dev, -EINVAL, "Value(%d) for %pfwP out of range [%d, %d]\n",
				     *val, fwnode, min, max);

	return 0;
}

static struct gpio_desc *devm_fwnode_gpiod_get_optional(struct device *dev,
							struct fwnode_handle *fwnode,
							const char *con_id, enum gpiod_flags flags,
							const char *label)
{
	struct gpio_desc *desc;

	desc = devm_fwnode_gpiod_get(dev, fwnode, con_id, flags, label);
	if (IS_ERR(desc) && PTR_ERR(desc) == -ENOENT)
		return NULL;

	return desc;
}

#define adrv9104_agc_offsetof(member)	\
	offsetof(struct adi_adrv910x_GainControlCfg, member)

static const struct {
	const char *key;
	u32 __off;
	u32 def;
	u32 min;
	u32 max;
	u8 size;
} adrv9104_agc_props[] = {
	{ "adi,peak-wait-time", adrv9104_agc_offsetof(peakWaitTime), 4, 0, 31, 1 },
	{ "adi,gain-update-counter",
	 adrv9104_agc_offsetof(gainUpdateCounter), 11520, 0, 4194303, 4 },
	{ "adi,attack-delay-us",
	 adrv9104_agc_offsetof(attackDelay_us), 10, 0, 63, 1 },
	{ "adi,slow-loop-settling-delay",
	 adrv9104_agc_offsetof(slowLoopSettlingDelay), 16, 0, 127, 1 },
	{ "adi,change-gain-threshold-high",
	 adrv9104_agc_offsetof(changeGainIfThreshHigh), 3, 0, 3, 1 },
	{ "adi,agc-mode", adrv9104_agc_offsetof(agcMode), 1, 0, 1, 1 },
	{ "adi,reset-on-rx-on-gain-index", adrv9104_agc_offsetof(resetOnRxonGainIndex),
	 ADI_ADRV910X_RX_GAIN_INDEX_MAX, ADI_ADRV910X_RX_GAIN_INDEX_MIN,
	 ADI_ADRV910X_RX_GAIN_INDEX_MAX, 1 },
	/* power detector */
	{ "adi,power-under-range-high-threshold",
	 adrv9104_agc_offsetof(power.underRangeHighPowerThresh), 10, 0, 127, 1 },
	{ "adi,power-under-range-low-threshold",
	 adrv9104_agc_offsetof(power.underRangeLowPowerThresh), 4, 0, 15, 1 },
	{ "adi,power-under-range-high-gain-step-recovery",
	 adrv9104_agc_offsetof(power.underRangeHighPowerGainStepRecovery), 2, 0, 31, 1 },
	{ "adi,power-under-range-low-gain-step-recovery",
	 adrv9104_agc_offsetof(power.underRangeLowPowerGainStepRecovery), 4, 0, 31, 1 },
	{ "adi,power-measurement-duration",
	 adrv9104_agc_offsetof(power.powerMeasurementDuration), 10, 0, 31, 1 },
	{ "adi,power-measurement-delay",
	 adrv9104_agc_offsetof(power.powerMeasurementDelay), 2, 0, 255, 1 },
	{ "adi,power-rx-tdd-measurement-duration",
	 adrv9104_agc_offsetof(power.rxTddPowerMeasDuration), 0, 0, 65535, 2 },
	{ "adi,power-rx-tdd-measurement-delay",
	 adrv9104_agc_offsetof(power.rxTddPowerMeasDelay), 0, 0, 65535, 2 },
	{ "adi,power-over-range-high-threshold",
	 adrv9104_agc_offsetof(power.overRangeHighPowerThresh), 0, 0, 15, 1 },
	{ "adi,power-over-range-low-threshold",
	 adrv9104_agc_offsetof(power.overRangeLowPowerThresh), 7, 0, 127, 1 },
	{ "adi,power-over-range-high-gain-step-attack",
	 adrv9104_agc_offsetof(power.overRangeHighPowerGainStepAttack), 4, 0, 31, 1 },
	{ "adi,power-over-range-low-gain-step-attack",
	 adrv9104_agc_offsetof(power.overRangeLowPowerGainStepAttack), 4, 0, 31, 1 },
	/* peak detector */
	{ "adi,peak-agc-under-range-low-interval",
	 adrv9104_agc_offsetof(peak.agcUnderRangeLowInterval), 50, 0, 65535, 2 },
	{ "adi,peak-agc-under-range-mid-interval",
	 adrv9104_agc_offsetof(peak.agcUnderRangeMidInterval), 2, 0, 63, 1 },
	{ "adi,peak-agc-under-range-high-interval",
	 adrv9104_agc_offsetof(peak.agcUnderRangeHighInterval), 4, 0, 63, 1 },
	{ "adi,peak-apd-high-threshold",
	 adrv9104_agc_offsetof(peak.apdHighThresh), 21, 0, 63, 1 },
	{ "adi,peak-apd-low-threshold",
	 adrv9104_agc_offsetof(peak.apdLowThresh), 12, 0, 63, 1 },
	{ "adi,peak-apd-upper-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.apdUpperThreshPeakExceededCount), 6, 0, 255, 1 },
	{ "adi,peak-apd-lower-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.apdLowerThreshPeakExceededCount), 3, 0, 255, 1 },
	{ "adi,peak-apd-gain-step-attack",
	 adrv9104_agc_offsetof(peak.apdGainStepAttack), 2, 0, 31, 1 },
	{ "adi,peak-apd-gain-step-recovery",
	 adrv9104_agc_offsetof(peak.apdGainStepRecovery), 0, 0, 31, 1 },
	{ "adi,peak-hb-overload-duration-count",
	 adrv9104_agc_offsetof(peak.hbOverloadDurationCount), 1, 0, 7, 1 },
	{ "adi,peak-hb-overload-threshold-count",
	 adrv9104_agc_offsetof(peak.hbOverloadThreshCount), 1, 1, 15, 1 },
	{ "adi,peak-hb-high-threshold",
	 adrv9104_agc_offsetof(peak.hbHighThresh), 13044, 0, 16383, 2 },
	{ "adi,peak-hb-under-range-low-threshold",
	 adrv9104_agc_offsetof(peak.hbUnderRangeLowThresh), 5826, 0, 16383, 2 },
	{ "adi,peak-hb-under-range-mid-threshold",
	 adrv9104_agc_offsetof(peak.hbUnderRangeMidThresh), 8230, 0, 16383, 2 },
	{ "adi,peak-hb-under-range-high-threshold",
	 adrv9104_agc_offsetof(peak.hbUnderRangeHighThresh), 7335, 0, 16383, 2 },
	{ "adi,peak-hb-upper-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.hbUpperThreshPeakExceededCount), 6, 0, 255, 1 },
	{ "adi,peak-hb-under-range-high-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.hbUnderRangeHighThreshExceededCount), 3, 0, 255, 1 },
	{ "adi,peak-hb-gain-step-high-recover",
	 adrv9104_agc_offsetof(peak.hbGainStepHighRecovery), 2, 0, 31, 1 },
	{ "adi,peak-hb-gain-step-low-recovery",
	 adrv9104_agc_offsetof(peak.hbGainStepLowRecovery), 6, 0, 31, 1 },
	{ "adi,peak-hb-gain-step-mid-recovery",
	 adrv9104_agc_offsetof(peak.hbGainStepMidRecovery), 4, 0, 31, 1 },
	{ "adi,peak-hb-gain-step-attack",
	 adrv9104_agc_offsetof(peak.hbGainStepAttack), 2, 0, 31, 1 },
	{ "adi,peak-hb-overload-power-mode",
	 adrv9104_agc_offsetof(peak.hbOverloadPowerMode), 0, 0, 1, 1 },
	{ "adi,peak-hb-under-range-mid-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.hbUnderRangeMidThreshExceededCount), 3, 0, 255, 1 },
	{ "adi,peak-hb-under-range-low-threshold-exceeded-count",
	 adrv9104_agc_offsetof(peak.hbUnderRangeLowThreshExceededCount), 3, 0, 255, 1 },
};

#define adrv9104_agc_assign_value(agc, prop, val) {		\
	typeof(prop) __prop = (prop);				\
	u32 __off = adrv9104_agc_props[__prop].__off;		\
								\
	switch (adrv9104_agc_props[__prop].size) {		\
	case 1:							\
		*(u8 *)((void *)(agc) + __off) = (val);		\
		break;						\
	case 2:							\
		*(u16 *)((void *)(agc) + __off) = (val);	\
		break;						\
	case 4:							\
		*(u32 *)((void *)(agc) + __off) = (val);	\
		break;						\
	}							\
}

static void adrv9104_set_agc_defaults(struct adi_adrv910x_GainControlCfg *agc)
{
	int prop;

	for (prop = 0; prop < ARRAY_SIZE(adrv9104_agc_props); prop++) {
		u32 temp = adrv9104_agc_props[prop].def;

		adrv9104_agc_assign_value(agc, prop, temp);
	}

	agc->power.feedback_apd_high_apd_low = ADI_ADRV910X_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->power.feedback_inner_high_inner_low = ADI_ADRV910X_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_low_hb_low = ADI_ADRV910X_GPIO_PIN_CRUMB_UNASSIGNED;
	agc->peak.feedback_apd_high_hb_high = ADI_ADRV910X_GPIO_PIN_CRUMB_UNASSIGNED;
}

static int adrv9104_parse_agc(struct adrv9104_rf_phy *phy, const struct fwnode_handle *fwnode,
			      struct adrv9104_rx *rx)
{
	int ret, prop;

	struct fwnode_handle *agc __free(fwnode_handle) = fwnode_find_reference(fwnode,
										"adi,agc", 0);
	if (!agc) {
		adrv9104_set_agc_defaults(&rx->agc);
		return 0;
	}

	/* set boolean settings that are enabled by default */
	rx->agc.power.powerEnableMeasurement = true;
	rx->agc.peak.enableHbOverload = true;

	for (prop = 0; prop < ARRAY_SIZE(adrv9104_agc_props); prop++) {
		u32 temp;

		ret = adrv9104_fwnode_read_validate_u32(phy->dev, agc, adrv9104_agc_props[prop].key,
							adrv9104_agc_props[prop].def,
							adrv9104_agc_props[prop].min,
							adrv9104_agc_props[prop].max, &temp, false);
		if (ret)
			return ret;

		adrv9104_agc_assign_value(&rx->agc, prop, temp);
	}

	/* boolean properties */
	if (fwnode_property_read_bool(agc, "adi,low-threshold-prevent-gain-inc"))
		rx->agc.lowThreshPreventGainInc = true;

	if (fwnode_property_read_bool(agc, "adi,sync-pulse-gain-counter-en"))
		rx->agc.enableSyncPulseForGainCounter = true;

	if (fwnode_property_read_bool(agc, "adi,fast-recovery-loop-en"))
		rx->agc.enableFastRecoveryLoop = true;

	if (fwnode_property_read_bool(agc, "adi,reset-on-rx-on"))
		rx->agc.resetOnRxon = true;

	if (fwnode_property_read_bool(agc, "adi,no-power-measurement-en"))
		rx->agc.power.powerEnableMeasurement = false;

	if (fwnode_property_read_bool(agc, "adi,no-peak-hb-overload-en"))
		rx->agc.peak.enableHbOverload = false;

	if (fwnode_property_read_bool(agc, "adi,rx-qec-freeze-enable"))
		rx->agc.rxQecFreezeEnable = true;

#define adrv9104_fw_agc_pin(key, min, max, val)	\
	adrv9104_fwnode_read_validate_u32(phy->dev, agc, key, \
					  ADI_ADRV910X_GPIO_PIN_CRUMB_UNASSIGNED, \
					  min, max, val, false)

	/* check if there are any gpios */
	ret = adrv9104_fw_agc_pin("adi,agc-power-feedback-high-thres-exceeded",
				  ADI_ADRV910X_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV910X_GPIO_PIN_CRUMB_15_14,
				  &rx->agc.power.feedback_inner_high_inner_low);
	if (ret)
		return ret;

	/* feedback pins*/
	ret = adrv9104_fw_agc_pin("adi,agc-power-feedback-low-thres-gain-change",
				  ADI_ADRV910X_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV910X_GPIO_PIN_CRUMB_15_14,
				  &rx->agc.power.feedback_apd_high_apd_low);
	if (ret)
		return ret;

	ret = adrv9104_fw_agc_pin("adi,agc-peak-feedback-high-thres-counter-exceeded",
				  ADI_ADRV910X_GPIO_PIN_CRUMB_01_00,
				  ADI_ADRV910X_GPIO_PIN_CRUMB_15_14,
				  &rx->agc.peak.feedback_apd_low_hb_low);
	if (ret)
		return ret;

	return adrv9104_fw_agc_pin("adi,agc-peak-feedback-low-thres-counter-exceeded",
				   ADI_ADRV910X_GPIO_PIN_CRUMB_01_00,
				   ADI_ADRV910X_GPIO_PIN_CRUMB_15_14,
				   &rx->agc.peak.feedback_apd_high_hb_high);
}

static int adrv9104_parse_rx_chan(struct adrv9104_rf_phy *phy, struct adrv9104_rx *rx,
				  const char *fw_node)
{
	u32 min_gain, max_gain;
	int ret;

	struct fwnode_handle *child __free(fwnode_handle) = device_get_named_child_node(phy->dev,
											fw_node);
	if (!child) {
		adrv9104_set_agc_defaults(&rx->agc);
		rx->agc.minGainIndex = ADI_ADRV910X_RX_GAIN_INDEX_MIN;
		rx->agc.maxGainIndex = ADI_ADRV910X_RX_GAIN_INDEX_MAX;
		return 0;
	}

	rx->channel.ensm = devm_fwnode_gpiod_get_optional(phy->dev, child, "enable",
							  GPIOD_OUT_LOW, "rx-enable");
	if (IS_ERR(rx->channel.ensm))
		return PTR_ERR(rx->channel.ensm);

	/*
	 * Read min/max gains to the global rx node since it's the same property for both AGC and
	 * pinctrl whenever that is available. Ask the BU about it.
	 */
	ret = adrv9104_fwnode_read_validate_u32(phy->dev, child, "adi,min-gain-index",
						ADI_ADRV910X_RX_GAIN_INDEX_MIN,
						ADI_ADRV910X_RX_GAIN_INDEX_MIN,
						ADI_ADRV910X_RX_GAIN_INDEX_MAX,
						&min_gain, false);
	if (ret)
		return ret;

	ret = adrv9104_fwnode_read_validate_u32(phy->dev, child, "adi,min-gain-index",
						ADI_ADRV910X_RX_GAIN_INDEX_MAX,
						min_gain, ADI_ADRV910X_RX_GAIN_INDEX_MAX,
						&max_gain, false);
	if (ret)
		return ret;

	rx->agc.minGainIndex = min_gain;
	rx->agc.maxGainIndex = max_gain;

	ret = fwnode_property_read_u64(child, "adi,carrier-hz", &rx->channel.carrier_hz);
	if (ret && ret != -EINVAL)
		return ret;

	return adrv9104_parse_agc(phy, child, rx);
}

static int adrv9104_parse_tx_chan(struct adrv9104_rf_phy *phy)
{
	struct adrv9104_tx *tx = &phy->tx_channel;
	int ret;

	struct fwnode_handle *child __free(fwnode_handle) = device_get_named_child_node(phy->dev,
											"adi,tx");
	if (!child)
		return 0;

	tx->channel.ensm = devm_fwnode_gpiod_get_optional(phy->dev, child, "enable",
							  GPIOD_OUT_LOW, "tx-enable");
	if (IS_ERR(tx->channel.ensm))
		return PTR_ERR(tx->channel.ensm);

	tx->dac_boost_en = fwnode_property_read_bool(child, "adi,dac-full-scale-boost");

	ret = fwnode_property_read_u64(child, "adi,carrier-hz", &tx->channel.carrier_hz);
	if (ret && ret != -EINVAL)
		return ret;

	return 0;
}

static const char * const adrv9104_rx_fw_node[] = {
	"adi,rx1",
	"adi,rx2",
};

int adrv9104_fw_parse(struct adrv9104_rf_phy *phy)
{
	unsigned int i;
	int ret;

	phy->rerun_calls = device_property_read_bool(phy->dev, "adi,rerun-calls-on-lo-retune");

	ret = adrv9104_parse_tx_chan(phy);
	if (ret)
		return ret;

	static_assert(ARRAY_SIZE(adrv9104_rx_fw_node) == ARRAY_SIZE(phy->rx_channels));
	for (i = 0; i < ARRAY_SIZE(adrv9104_rx_fw_node); i++) {
		ret = adrv9104_parse_rx_chan(phy, &phy->rx_channels[i], adrv9104_rx_fw_node[i]);
		if (ret)
			return ret;
	}

	return 0;
}
