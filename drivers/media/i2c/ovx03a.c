// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for OVX03A CMOS Image Sensor from Sony
 *
 * Copyright 2023 Bogdan Togorean <bogdan.togorean@analog.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define OVX03A_WIDTH	1920
#define OVX03A_HEIGHT	1280

#define OX03A_PID		0x300A
#define OX03A_VER		0x300B

static const char * const ovx03a_supply_names[] = {
	"avdd",
	"dvdd",
	"iovdd"
};

struct ovx03a {
	struct device *dev;
	struct clk *clk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ovx03a_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	bool streaming;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
};

/*
 * This table is extracted from vendor data that is entirely undocumented. The
 * first register write is required to disable the CSI-2 output. The other
 * entries may or may not be optional?
 */
static const struct {
	unsigned int reg;
	unsigned int value;
} ovx03a_init_table[] = {
	{ 0x0100, 0x00 }, // Stream off
	{ 0x0103, 0x01 }, // SW reset
	{ 0x4d07, 0x21 },
	{ 0x4d0e, 0x80 },
	{ 0x4d11, 0x7d },
	{ 0x0303, 0x02 },
	{ 0x0305, 0x30 },
	{ 0x0306, 0x03 },
	{ 0x0307, 0x01 },
	{ 0x0316, 0x00 },
	{ 0x0317, 0x42 },
	{ 0x0323, 0x02 },
	{ 0x0325, 0x68 },
	{ 0x0326, 0x00 },
	{ 0x032b, 0x00 },
	{ 0x0400, 0xe7 },
	{ 0x0401, 0xff },
	{ 0x0404, 0x2b },
	{ 0x0405, 0x32 },
	{ 0x0406, 0x33 },
	{ 0x0407, 0x8f },
	{ 0x0408, 0x0c },
	{ 0x0410, 0xe7 },
	{ 0x0411, 0xff },
	{ 0x0414, 0x2b },
	{ 0x0415, 0x32 },
	{ 0x0416, 0x33 },
	{ 0x0417, 0x8f },
	{ 0x0418, 0x0c },
	{ 0x3002, 0x03 },
	{ 0x3012, 0x41 },
	{ 0x301e, 0xb0 },
	{ 0x3706, 0x39 },
	{ 0x370a, 0x00 },
	{ 0x370b, 0xd1 },
	{ 0x370f, 0x40 },
	{ 0x3711, 0x22 },
	{ 0x3712, 0x12 },
	{ 0x3713, 0x00 },
	{ 0x3718, 0x04 },
	{ 0x3719, 0x3c },
	{ 0x371a, 0x06 },
	{ 0x371b, 0x14 },
	{ 0x372c, 0x17 },
	{ 0x3733, 0x41 },
	{ 0x3741, 0x44 },
	{ 0x3742, 0x34 },
	{ 0x3746, 0x03 },
	{ 0x374b, 0x03 },
	{ 0x3755, 0x09 },
	{ 0x376c, 0x04 },
	{ 0x376d, 0x08 },
	{ 0x376f, 0x08 },
	{ 0x3770, 0x91 },
	{ 0x3771, 0x08 },
	{ 0x3774, 0x8a },
	{ 0x3777, 0x99 },
	{ 0x3779, 0x22 },
	{ 0x377a, 0x00 },
	{ 0x377b, 0x00 },
	{ 0x377c, 0x48 },
	{ 0x3785, 0x08 },
	{ 0x3790, 0x10 },
	{ 0x3793, 0x00 },
	{ 0x379c, 0x01 },
	{ 0x37a1, 0x80 },
	{ 0x37b3, 0x0a },
	{ 0x37be, 0x01 },
	{ 0x37bf, 0x01 },
	{ 0x37c6, 0x48 },
	{ 0x37c7, 0x38 },
	{ 0x37c9, 0x00 },
	{ 0x37ca, 0x39 },
	{ 0x37cb, 0x00 },
	{ 0x37cc, 0xa3 },
	{ 0x37d1, 0x39 },
	{ 0x37d2, 0x00 },
	{ 0x37d3, 0xa3 },
	{ 0x37d5, 0x39 },
	{ 0x37d6, 0x00 },
	{ 0x37d7, 0xa3 },
	{ 0x3c06, 0x29 },
	{ 0x3c0b, 0xa8 },
	{ 0x3c53, 0x68 },
	{ 0x3192, 0x00 },
	{ 0x3193, 0x00 },
	{ 0x3206, 0x80 },
	{ 0x3216, 0x01 },
	{ 0x3400, 0x08 },
	{ 0x3409, 0x02 },
	{ 0x3501, 0x00 },
	{ 0x3502, 0x40 },
	{ 0x3508, 0x01 },
	{ 0x3509, 0x00 },
	{ 0x350a, 0x01 },
	{ 0x350b, 0x00 },
	{ 0x350c, 0x00 },
	{ 0x3548, 0x01 },
	{ 0x3549, 0x00 },
	{ 0x354a, 0x01 },
	{ 0x354b, 0x00 },
	{ 0x354c, 0x00 },
	{ 0x3581, 0x00 },
	{ 0x3582, 0x40 },
	{ 0x3588, 0x01 },
	{ 0x3589, 0x00 },
	{ 0x358a, 0x01 },
	{ 0x358b, 0x00 },
	{ 0x358c, 0x00 },
	{ 0x3600, 0x00 },
	{ 0x3602, 0x42 },
	{ 0x3603, 0xf3 },
	{ 0x3604, 0x93 },
	{ 0x3605, 0xff },
	{ 0x3606, 0xc0 },
	{ 0x3607, 0x4a },
	{ 0x360a, 0xd0 },
	{ 0x360b, 0x0b },
	{ 0x360e, 0x88 },
	{ 0x3611, 0x4b },
	{ 0x3612, 0x4e },
	{ 0x3614, 0x8a },
	{ 0x3615, 0x98 },
	{ 0x3619, 0x00 },
	{ 0x3620, 0x02 },
	{ 0x3626, 0x0e },
	{ 0x362c, 0x0e },
	{ 0x362d, 0x12 },
	{ 0x362e, 0x0b },
	{ 0x362f, 0x18 },
	{ 0x3630, 0x30 },
	{ 0x3631, 0x57 },
	{ 0x3632, 0x99 },
	{ 0x3633, 0x99 },
	{ 0x3643, 0x0c },
	{ 0x3644, 0x00 },
	{ 0x3645, 0x0e },
	{ 0x3646, 0x0f },
	{ 0x3647, 0x0e },
	{ 0x3648, 0x00 },
	{ 0x3649, 0x11 },
	{ 0x364a, 0x12 },
	{ 0x364c, 0x0e },
	{ 0x364d, 0x0e },
	{ 0x364e, 0x12 },
	{ 0x364f, 0x0e },
	{ 0x3652, 0xc5 },
	{ 0x3657, 0x88 },
	{ 0x3658, 0x08 },
	{ 0x365a, 0x57 },
	{ 0x365b, 0x30 },
	{ 0x365c, 0x18 },
	{ 0x365d, 0x0b },
	{ 0x3660, 0x01 },
	{ 0x3661, 0x07 },
	{ 0x3662, 0x00 },
	{ 0x3665, 0x92 },
	{ 0x3666, 0x13 },
	{ 0x3667, 0x2c },
	{ 0x3668, 0x95 },
	{ 0x3669, 0x2c },
	{ 0x366f, 0xc4 },
	{ 0x3671, 0x0b },
	{ 0x3673, 0x6a },
	{ 0x3678, 0x88 },
	{ 0x3800, 0x00 },
	{ 0x3801, 0x00 },
	{ 0x3802, 0x00 },
	{ 0x3803, 0x04 },
	{ 0x3804, 0x07 },
	{ 0x3805, 0x8f },
	{ 0x3806, 0x05 },
	{ 0x3807, 0x0b },
	{ 0x3808, 0x07 },
	{ 0x3809, 0x80 },
	{ 0x380a, 0x05 },
	{ 0x380b, 0x00 },
	{ 0x380c, 0x0a },
	{ 0x380d, 0x90 },
	{ 0x380e, 0x05 },
	{ 0x380f, 0x37 },
	{ 0x3810, 0x00 },
	{ 0x3811, 0x08 },
	{ 0x3813, 0x04 },
	{ 0x381c, 0x00 },
	{ 0x3820, 0x44 },
	{ 0x3821, 0x00 },
	{ 0x3822, 0x14 },
	{ 0x3832, 0x10 },
	{ 0x3833, 0x01 },
	{ 0x3834, 0xf0 },
	{ 0x383d, 0x20 },
	{ 0x384c, 0x02 },
	{ 0x384d, 0x14 },
	{ 0x384e, 0x00 },
	{ 0x384f, 0x40 },
	{ 0x3850, 0x00 },
	{ 0x3851, 0x42 },
	{ 0x3852, 0x00 },
	{ 0x3853, 0x40 },
	{ 0x3854, 0x00 },
	{ 0x3855, 0x05 },
	{ 0x3856, 0x05 },
	{ 0x3857, 0x33 },
	{ 0x3858, 0x3c },
	{ 0x3859, 0x00 },
	{ 0x385a, 0x03 },
	{ 0x385b, 0x05 },
	{ 0x385c, 0x32 },
	{ 0x385f, 0x00 },
	{ 0x3860, 0x10 },
	{ 0x3861, 0x00 },
	{ 0x3862, 0x40 },
	{ 0x3863, 0x00 },
	{ 0x3864, 0x40 },
	{ 0x3865, 0x00 },
	{ 0x3866, 0x40 },
	{ 0x3b40, 0x3e },
	{ 0x3b41, 0x00 },
	{ 0x3b42, 0x02 },
	{ 0x3b43, 0x00 },
	{ 0x3b44, 0x00 },
	{ 0x3b45, 0x20 },
	{ 0x3b46, 0x00 },
	{ 0x3b47, 0x20 },
	{ 0x3b84, 0x05 },
	{ 0x3b85, 0x00 },
	{ 0x3b86, 0x00 },
	{ 0x3b87, 0x10 },
	{ 0x3b88, 0x00 },
	{ 0x3b89, 0x10 },
	{ 0x3b8a, 0x00 },
	{ 0x3b8b, 0x08 },
	{ 0x3b8e, 0x03 },
	{ 0x3b8f, 0xe8 },
	{ 0x3d85, 0x0b },
	{ 0x3d8c, 0x70 },
	{ 0x3d8d, 0x26 },
	{ 0x3d97, 0x70 },
	{ 0x3d98, 0x24 },
	{ 0x3d99, 0x70 },
	{ 0x3d9a, 0x6d },
	{ 0x3d9b, 0x70 },
	{ 0x3d9c, 0x6e },
	{ 0x3d9d, 0x73 },
	{ 0x3d9e, 0xff },
	{ 0x3f00, 0x04 },
	{ 0x4001, 0x2b },
	{ 0x4004, 0x00 },
	{ 0x4005, 0x80 },
	{ 0x4008, 0x02 },
	{ 0x4009, 0x0d },
	{ 0x400a, 0x10 },
	{ 0x400b, 0x80 },
	{ 0x400f, 0x80 },
	{ 0x4010, 0x10 },
	{ 0x4011, 0xbb },
	{ 0x4016, 0x00 },
	{ 0x4017, 0x10 },
	{ 0x402e, 0x00 },
	{ 0x402f, 0x80 },
	{ 0x4030, 0x00 },
	{ 0x4031, 0x80 },
	{ 0x4032, 0x9f },
	{ 0x4033, 0x00 },
	{ 0x4308, 0x00 },
	{ 0x4502, 0x00 },
	{ 0x4507, 0x16 },
	{ 0x4580, 0xf8 },
	{ 0x4602, 0x02 },
	{ 0x4603, 0x00 },
	{ 0x460a, 0x50 },
	{ 0x460c, 0x60 },
	{ 0x4800, 0x04 },
	{ 0x480e, 0x04 },
	{ 0x4813, 0x24 },
	{ 0x4815, 0x2b },
	{ 0x4837, 0x35 },
	{ 0x484b, 0x27 },
	{ 0x484c, 0x00 },
	{ 0x4886, 0x00 },
	{ 0x4903, 0x80 },
	{ 0x4f00, 0xff },
	{ 0x4f01, 0xff },
	{ 0x4f05, 0x01 },
	{ 0x5180, 0x02 },
	{ 0x5181, 0x00 },
	{ 0x5182, 0x01 },
	{ 0x5183, 0x00 },
	{ 0x5184, 0x01 },
	{ 0x5185, 0x00 },
	{ 0x5186, 0x01 },
	{ 0x5187, 0x86 },
	{ 0x51a0, 0x04 },
	{ 0x51a1, 0x00 },
	{ 0x51a2, 0x04 },
	{ 0x51a3, 0x00 },
	{ 0x51a4, 0x04 },
	{ 0x51a5, 0x00 },
	{ 0x51a6, 0x04 },
	{ 0x51a7, 0x00 },
	{ 0x51c0, 0x04 },
	{ 0x51c1, 0x00 },
	{ 0x51c2, 0x04 },
	{ 0x51c3, 0x00 },
	{ 0x51c4, 0x04 },
	{ 0x51c5, 0x00 },
	{ 0x51c6, 0x04 },
	{ 0x51c7, 0x00 },
	{ 0x5380, 0x19 },
	{ 0x5382, 0x2e },
	{ 0x53a0, 0x41 },
	{ 0x53a2, 0x04 },
	{ 0x53a3, 0x00 },
	{ 0x53a4, 0x04 },
	{ 0x53a5, 0x00 },
	{ 0x53a7, 0x00 },
	{ 0x5400, 0x19 },
	{ 0x5402, 0x2e },
	{ 0x5420, 0x41 },
	{ 0x5422, 0x04 },
	{ 0x5423, 0x00 },
	{ 0x5424, 0x04 },
	{ 0x5425, 0x00 },
	{ 0x5427, 0x00 },
	{ 0x5480, 0x19 },
	{ 0x5482, 0x2e },
	{ 0x54a0, 0x41 },
	{ 0x54a2, 0x04 },
	{ 0x54a3, 0x00 },
	{ 0x54a4, 0x04 },
	{ 0x54a5, 0x00 },
	{ 0x54a7, 0x00 },
	{ 0x5800, 0x38 },
	{ 0x5801, 0x03 },
	{ 0x5802, 0xc0 },
	{ 0x5804, 0x00 },
	{ 0x5805, 0x80 },
	{ 0x5806, 0x01 },
	{ 0x5807, 0x00 },
	{ 0x580e, 0x10 },
	{ 0x5812, 0x34 },
	{ 0x5000, 0x89 },
	{ 0x5001, 0x42 },
	{ 0x5002, 0x19 },
	{ 0x5003, 0x16 },
	{ 0x5004, 0x00 },
	{ 0x5005, 0x40 },
	{ 0x5006, 0x00 },
	{ 0x5007, 0x40 },
	{ 0x503e, 0x00 },
	{ 0x503f, 0x00 },
	{ 0x5602, 0x02 },
	{ 0x5603, 0x58 },
	{ 0x5604, 0x03 },
	{ 0x5605, 0x20 },
	{ 0x5606, 0x02 },
	{ 0x5607, 0x58 },
	{ 0x5608, 0x03 },
	{ 0x5609, 0x20 },
	{ 0x560a, 0x02 },
	{ 0x560b, 0x58 },
	{ 0x560c, 0x03 },
	{ 0x560d, 0x20 },
	{ 0x560e, 0x02 },
	{ 0x560f, 0x58 },
	{ 0x5610, 0x03 },
	{ 0x5611, 0x20 },
	{ 0x5612, 0x02 },
	{ 0x5613, 0x58 },
	{ 0x5614, 0x03 },
	{ 0x5615, 0x20 },
	{ 0x5616, 0x02 },
	{ 0x5617, 0x58 },
	{ 0x5618, 0x03 },
	{ 0x5619, 0x20 },
	{ 0x5642, 0x02 },
	{ 0x5643, 0x58 },
	{ 0x5644, 0x03 },
	{ 0x5645, 0x20 },
	{ 0x5646, 0x02 },
	{ 0x5647, 0x58 },
	{ 0x5648, 0x03 },
	{ 0x5649, 0x20 },
	{ 0x564a, 0x02 },
	{ 0x564b, 0x58 },
	{ 0x564c, 0x03 },
	{ 0x564d, 0x20 },
	{ 0x564e, 0x02 },
	{ 0x564f, 0x58 },
	{ 0x5650, 0x03 },
	{ 0x5651, 0x20 },
	{ 0x5652, 0x02 },
	{ 0x5653, 0x58 },
	{ 0x5654, 0x03 },
	{ 0x5655, 0x20 },
	{ 0x5656, 0x02 },
	{ 0x5657, 0x58 },
	{ 0x5658, 0x03 },
	{ 0x5659, 0x20 },
	{ 0x5682, 0x02 },
	{ 0x5683, 0x58 },
	{ 0x5684, 0x03 },
	{ 0x5685, 0x20 },
	{ 0x5686, 0x02 },
	{ 0x5687, 0x58 },
	{ 0x5688, 0x03 },
	{ 0x5689, 0x20 },
	{ 0x568a, 0x02 },
	{ 0x568b, 0x58 },
	{ 0x568c, 0x03 },
	{ 0x568d, 0x20 },
	{ 0x568e, 0x02 },
	{ 0x568f, 0x58 },
	{ 0x5690, 0x03 },
	{ 0x5691, 0x20 },
	{ 0x5692, 0x02 },
	{ 0x5693, 0x58 },
	{ 0x5694, 0x03 },
	{ 0x5695, 0x20 },
	{ 0x5696, 0x02 },
	{ 0x5697, 0x58 },
	{ 0x5698, 0x03 },
	{ 0x5699, 0x20 },
	{ 0x5709, 0x0f },
	{ 0x5749, 0x0f },
	{ 0x5789, 0x0f },
	{ 0x5200, 0x70 },
	{ 0x5201, 0x70 },
	{ 0x5202, 0x73 },
	{ 0x5203, 0xff },
	{ 0x5205, 0x6f },
	{ 0x5209, 0x18 },
	{ 0x520b, 0x04 },
	{ 0x5285, 0x6f },
	{ 0x5289, 0x18 },
	{ 0x528b, 0x04 },
	{ 0x5305, 0x6f },
	{ 0x5309, 0x18 },
	{ 0x530b, 0x04 },
	{ 0x3501, 0x01 },
	{ 0x3502, 0x90 },
};

static inline struct ovx03a *to_ovx03a(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ovx03a, subdev);
}

static int ovx03a_reg_read(struct ovx03a *sensor, u16 addr, u8 *value)
{
	u32 reg_val = 0;
	int ret;

	ret = regmap_read(sensor->regmap, addr, &reg_val);
	*value = reg_val & 0xFF;
	if(ret)
		dev_err(sensor->dev, "[%s] : Failed at 0x%x\n", __func__, addr);

	return ret;
}

static int ovx03a_reg_write(struct ovx03a *sensor, u32 addr, u32 value)
{
	int ret = 0;

	ret = regmap_write(sensor->regmap, addr, value);
	if(ret)
		dev_err(sensor->dev, "[%s] : Failed at 0x%x\n", __func__, addr);

	return ret;
}

static u32 ovx03a_mbus_code(const struct ovx03a *sensor)
{
	return MEDIA_BUS_FMT_SBGGR12_1X12;
}

static int ovx03a_power_on(struct ovx03a *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	udelay(1);

	ret = gpiod_direction_output(sensor->reset, 0);
	if (ret < 0)
		goto err_supply;

	udelay(1);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;

	return 0;

err_reset:
	gpiod_direction_output(sensor->reset, 1);
err_supply:
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return ret;
}

static void ovx03a_power_off(struct ovx03a *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_direction_output(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int ox03a_check_id(struct ovx03a *sensor)
{
	u8 reg_val[2];
	int err;

	/* Probe sensor model id registers */
	err = ovx03a_reg_read(sensor, OX03A_PID, &reg_val[0]);
	if (err)
		return err;

	err = ovx03a_reg_read(sensor, OX03A_VER, &reg_val[1]);
	if (err)
		return err;

	if (!((reg_val[0] == 0x58) && reg_val[1] == 0x03)) {
		dev_err(sensor->dev, "%s: invalid sensor model id: %x%x\n",
			__func__, reg_val[0], reg_val[1]);
		return -EINVAL;
	}

	return 0;
}

/* ------------------------------------------------------------------------- */

/* -----------------------------------------------------------------------------
 * Controls
 */

static int ovx03a_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ovx03a *sensor = container_of(ctrl->handler, struct ovx03a, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (!sensor->streaming)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);

	switch (ctrl->id) {
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ovx03a_ctrl_ops = {
	.s_ctrl = ovx03a_s_ctrl,
};

static int ovx03a_ctrls_init(struct ovx03a *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 9);

	/*
	 * The sensor calculates the MIPI timings internally to achieve a bit
	 * rate between 1122 and 1198 Mbps. The exact value is unfortunately not
	 * reported, at least according to the documentation. Report a nominal
	 * rate of 1188 Mbps as that is used by the datasheet in multiple
	 * examples.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  1122000000 / 16, 1296000000 / 16, 1, 1188000000 / 16);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &ovx03a_ctrl_ops,
					&props);

	if (sensor->ctrls.error) {
		dev_err(sensor->dev, "failed to add controls (%d)\n",
			sensor->ctrls.error);
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return sensor->ctrls.error;
	}

	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */
static int ovx03a_setup(struct ovx03a *sensor, struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	unsigned int i;
	int ret = 0;

	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);

	for (i = 0; i < ARRAY_SIZE(ovx03a_init_table); ++i)
		ret = ovx03a_reg_write(sensor, ovx03a_init_table[i].reg,
			     ovx03a_init_table[i].value);

	return ret;
}

static int ovx03a_stream_on(struct ovx03a *sensor)
{
	int ret = 0;

	ret = ovx03a_reg_write(sensor, 0x100, 1);

	return ret;
}

static int ovx03a_stream_off(struct ovx03a *sensor)
{
	int ret = 0;

	ret = ovx03a_reg_write(sensor, 0x100, 0);

	return ret;
}

static int ovx03a_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ovx03a *sensor = to_ovx03a(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (sensor->streaming == enable)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = ovx03a_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		sensor->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = ovx03a_setup(sensor, state);
	if (ret < 0)
		goto err_pm;

	/*
	 * Set streaming to true to ensure __v4l2_ctrl_handler_setup() will set
	 * the controls. The flag is reset to false further down if an error
	 * occurs.
	 */
	sensor->streaming = true;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = ovx03a_stream_on(sensor);
	if (ret)
		goto err_pm;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(sensor->dev);
	sensor->streaming = false;

	goto unlock;
}

static int ovx03a_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int ovx03a_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ovx03a *sensor = to_ovx03a(sd);
	int ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	if (sensor->streaming)
		return -EBUSY;

	return ret;
}

static int ovx03a_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ovx03a *sensor = to_ovx03a(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = ovx03a_mbus_code(sensor);

	return 0;
}

static int ovx03a_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct ovx03a *sensor = to_ovx03a(sd);
	const struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_pad_format(sd, state, fse->pad);

	if (fse->index >= 1 || fse->code != ovx03a_mbus_code(sensor))
		return -EINVAL;

	fse->min_width = OVX03A_WIDTH / (fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = OVX03A_HEIGHT / (fse->index + 1);
	fse->max_height = fse->min_height;

	return 0;
}

static int ovx03a_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= 1)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = 30;

	fie->interval = tpf;
	return 0;
}

static int ovx03a_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	fmt->format = *v4l2_subdev_get_pad_format(sd, state, fmt->pad);

	return 0;
}

static int ovx03a_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct ovx03a *sensor = to_ovx03a(sd);
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_pad_format(sd, state, fmt->pad);

	format->width = OVX03A_WIDTH;
	format->height = OVX03A_HEIGHT;

	format->code = ovx03a_mbus_code(sensor);
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_RAW;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	return 0;
}

static int ovx03a_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = OVX03A_WIDTH,
			.height = OVX03A_HEIGHT,
		},
	};
	ovx03a_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_video_ops ovx03a_subdev_video_ops = {
	.g_frame_interval = ovx03a_g_frame_interval,
	.s_frame_interval = ovx03a_s_frame_interval,
	.s_stream = ovx03a_s_stream,
};

static const struct v4l2_subdev_pad_ops ovx03a_subdev_pad_ops = {
	.enum_mbus_code = ovx03a_enum_mbus_code,
	.enum_frame_size = ovx03a_enum_frame_size,
	.enum_frame_interval = ovx03a_enum_frame_interval,
	.get_fmt = ovx03a_get_format,
	.set_fmt = ovx03a_set_format,
	.init_cfg = ovx03a_init_cfg,
};

static const struct v4l2_subdev_ops ovx03a_subdev_ops = {
	.video = &ovx03a_subdev_video_ops,
	.pad = &ovx03a_subdev_pad_ops,
};

static int ovx03a_subdev_init(struct ovx03a *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ovx03a_subdev_ops);

	ret = ovx03a_ctrls_init(sensor);
	if (ret < 0)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;

	v4l2_subdev_init_finalize(&sensor->subdev);

	return ret;
}

static void ovx03a_subdev_cleanup(struct ovx03a *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int __maybe_unused ovx03a_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ovx03a *sensor = to_ovx03a(subdev);

	return ovx03a_power_on(sensor);
}

static int __maybe_unused ovx03a_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ovx03a *sensor = to_ovx03a(subdev);

	ovx03a_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops ovx03a_pm_ops = {
	SET_RUNTIME_PM_OPS(ovx03a_runtime_suspend, ovx03a_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static const struct regmap_config ovx03a_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static int ovx03a_probe(struct i2c_client *client)
{
	struct ovx03a *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	/* Acquire resources. */
	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = ovx03a_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret) {
		dev_err_probe(sensor->dev, ret, "failed to get supplies\n");
		return ret;
	}

	sensor->reset = devm_gpiod_get_optional(sensor->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_clk_get(sensor->dev, "xclk");
	if (IS_ERR(sensor->clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
				     "failed to get clock\n");

	sensor->regmap = devm_regmap_init_i2c(client, &ovx03a_regmap_config);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = ovx03a_power_on(sensor);
	if (ret < 0)
		return ret;

	ret = ox03a_check_id(sensor);
	if (ret < 0)
		return ret;

	/* Initialize the V4L2 subdev. */
	ret = ovx03a_subdev_init(sensor);
	if (ret < 0)
		goto err_power;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	/* Register the V4L2 subdev. */
	ret = v4l2_async_register_subdev(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	ovx03a_subdev_cleanup(sensor);
err_power:
	ovx03a_power_off(sensor);
	return ret;
}

static void ovx03a_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct ovx03a *sensor = to_ovx03a(subdev);

	v4l2_async_unregister_subdev(subdev);

	ovx03a_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		ovx03a_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id ovx03a_of_match[] = {
	{ .compatible = "ov,ox03a", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ovx03a_of_match);

static struct i2c_driver ovx03a_i2c_driver = {
	.driver = {
		.of_match_table = ovx03a_of_match,
		.name = "ovx03a",
		.pm = &ovx03a_pm_ops
	},
	.probe_new = ovx03a_probe,
	.remove = ovx03a_remove,
};

module_i2c_driver(ovx03a_i2c_driver);

MODULE_DESCRIPTION("Omni Vision OVX03A sensor driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
