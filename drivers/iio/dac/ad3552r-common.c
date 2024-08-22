// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2010-2024 Analog Devices Inc.
// Copyright (c) 2024 Baylibre, SAS

#include <linux/module.h>
#include <linux/device.h>
#include <linux/property.h>

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
