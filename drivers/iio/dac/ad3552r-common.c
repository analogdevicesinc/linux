// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2010-2024 Analog Devices Inc.
// Copyright (c) 2024 Baylibre, SAS

#include <linux/device.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>

#include "ad3552r.h"

static const s32 ad3552r_ch_ranges[][2] = {
	[AD3552R_CH_OUTPUT_RANGE_0__2P5V]	= {0, 2500},
	[AD3552R_CH_OUTPUT_RANGE_0__5V]		= {0, 5000},
	[AD3552R_CH_OUTPUT_RANGE_0__10V]	= {0, 10000},
	[AD3552R_CH_OUTPUT_RANGE_NEG_5__5V]	= {-5000, 5000},
	[AD3552R_CH_OUTPUT_RANGE_NEG_10__10V]	= {-10000, 10000}
};

static const s32 ad3542r_ch_ranges[][2] = {
	[AD3542R_CH_OUTPUT_RANGE_0__2P5V]	= {0, 2500},
	[AD3542R_CH_OUTPUT_RANGE_0__3V]		= {0, 3000},
	[AD3542R_CH_OUTPUT_RANGE_0__5V]		= {0, 5000},
	[AD3542R_CH_OUTPUT_RANGE_0__10V]	= {0, 10000},
	[AD3542R_CH_OUTPUT_RANGE_NEG_2P5__7P5V]	= {-2500, 7500},
	[AD3542R_CH_OUTPUT_RANGE_NEG_5__5V]	= {-5000, 5000}
};

void ad3552r_calc_custom_gain(u8 p, u8 n, s16 goffs, u16 *reg)
{
	*reg = FIELD_PREP(AD3552R_MASK_CH_RANGE_OVERRIDE, 1);
	*reg |= FIELD_PREP(AD3552R_MASK_CH_GAIN_SCALING_P, p);
	*reg |= FIELD_PREP(AD3552R_MASK_CH_GAIN_SCALING_N, n);
	*reg |= FIELD_PREP(AD3552R_MASK_CH_OFFSET_BIT_8, abs((s32)goffs) >> 8);
	*reg |= FIELD_PREP(AD3552R_MASK_CH_OFFSET_POLARITY, (s32)goffs < 0);
}

int ad3552r_get_ref_voltage(struct device *dev, u32 *val)
{
	int voltage, delta = 100000;

	voltage = devm_regulator_get_enable_read_voltage(dev, "vref");
	if (voltage < 0 && voltage != -ENODEV)
		return dev_err_probe(dev, voltage,
				     "Error getting vref voltage\n");

	if (voltage == -ENODEV) {
		if (device_property_read_bool(dev, "adi,vref-out-en"))
			*val = AD3552R_INTERNAL_VREF_PIN_2P5V;
		else
			*val = AD3552R_INTERNAL_VREF_PIN_FLOATING;
	} else {
		if (voltage > 2500000 + delta || voltage < 2500000 - delta) {
			dev_warn(dev, "vref-supply must be 2.5V");
			return -EINVAL;
		}
		*val = AD3552R_EXTERNAL_VREF_PIN_INPUT;
	}

	return 0;
}

int ad3552r_get_drive_strength(struct device *dev, u32 *val)
{
	int err;

	err = device_property_read_u32(dev, "adi,sdo-drive-strength", val);
	if (!err && *val > 3) {
		dev_err(dev,
			"adi,sdo-drive-strength must be less than 4\n");
		return -EINVAL;
	}

	return err;
}

int ad3552r_get_custom_gain(struct device *dev, struct fwnode_handle *child,
			    u8 *gs_p, u8 *gs_n, u16 *rfb, s16 *goffs)
{
	int err;
	u32 val;
	struct fwnode_handle *gain_child __free(fwnode_handle)
		= fwnode_get_named_child_node(child,
				      "custom-output-range-config");

	if (!gain_child)
		return dev_err_probe(dev, -EINVAL,
				     "custom-output-range-config mandatory\n");

	err = fwnode_property_read_u32(gain_child, "adi,gain-scaling-p", &val);
	if (err)
		return dev_err_probe(dev, err,
				     "adi,gain-scaling-p mandatory\n");
	*gs_p = val;

	err = fwnode_property_read_u32(gain_child, "adi,gain-scaling-n", &val);
	if (err)
		return dev_err_probe(dev, err,
				     "adi,gain-scaling-n property mandatory\n");
	*gs_n = val;

	err = fwnode_property_read_u32(gain_child, "adi,rfb-ohms", &val);
	if (err)
		return dev_err_probe(dev, err,
				     "adi,rfb-ohms mandatoryn");
	*rfb = val;

	err = fwnode_property_read_u32(gain_child, "adi,gain-offset", &val);
	if (err)
		return dev_err_probe(dev, err,
				     "adi,gain-offset mandatory\n");
	*goffs = val;

	return 0;
}

static int ad3552r_find_range(u16 id, s32 *vals)
{
	int i, len;
	const s32 (*ranges)[2];

	if (id == AD3542R_ID) {
		len = ARRAY_SIZE(ad3542r_ch_ranges);
		ranges = ad3542r_ch_ranges;
	} else {
		len = ARRAY_SIZE(ad3552r_ch_ranges);
		ranges = ad3552r_ch_ranges;
	}

	for (i = 0; i < len; i++)
		if (vals[0] == ranges[i][0] * 1000 &&
		    vals[1] == ranges[i][1] * 1000)
			return i;

	return -EINVAL;
}

int ad3552r_get_output_range(struct device *dev, enum ad3552r_id chip_id,
			     struct fwnode_handle *child, u32 *val)
{
	int ret;
	s32 vals[2];

	if (!fwnode_property_present(child, "adi,output-range-microvolt"))
		return -ENOENT;

	ret = fwnode_property_read_u32_array(child,
					     "adi,output-range-microvolt",
					     vals, 2);
	if (ret)
		return dev_err_probe(dev, ret,
				"invalid adi,output-range-microvolt\n");

	ret = ad3552r_find_range(chip_id, vals);
	if (ret < 0)
		return dev_err_probe(dev, ret,
			"invalid adi,output-range-microvolt value\n");

	*val = ret;

	return 0;
}
