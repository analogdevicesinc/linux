// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#ifndef MAX_SERDES_H
#define MAX_SERDES_H

#define v4l2_subdev_state v4l2_subdev_pad_config
#define v4l2_subdev_alloc_state v4l2_subdev_alloc_pad_config
#define v4l2_subdev_free_state v4l2_subdev_free_pad_config
#undef v4l2_async_notifier_add_fwnode_subdev
#define v4l2_async_notifier_add_fwnode_subdev(__notifier, __fwnode, __type)	\
	((__type *)__v4l2_async_notifier_add_fwnode_subdev(__notifier, __fwnode,\
							   sizeof(__type)))

struct pingroup {
	const char *name;
	const unsigned int *pins;
	size_t npins;
};

#define PINCTRL_PINGROUP(_name, _pins, _npins)	\
(struct pingroup) {				\
	.name = _name,				\
	.pins = _pins,				\
	.npins = _npins,			\
}

struct pinfunction {
	const char *name;
	const char * const *groups;
	size_t ngroups;
};

#define PINCTRL_PINFUNCTION(_name, _groups, _ngroups)	\
(struct pinfunction) {					\
		.name = (_name),			\
		.groups = (_groups),			\
		.ngroups = (_ngroups),			\
	}

#define MAX_SERDES_STREAMS_NUM     4
#define MAX_SERDES_VC_ID_NUM	   4

struct max_i2c_xlate {
	u8 src;
	u8 dst;
};

struct max_format {
	u32 code;
	u8 dt;
	u8 bpp;
	bool dbl;
};

#define MAX_DT_FS			0x00
#define MAX_DT_FE			0x01
#define MAX_DT_EMB8			0x12
#define MAX_DT_YUV422_8B		0x1e
#define MAX_DT_YUV422_10B		0x1f
#define MAX_DT_RGB565			0x22
#define MAX_DT_RGB666			0x23
#define MAX_DT_RGB888			0x24
#define MAX_DT_RAW8			0x2a
#define MAX_DT_RAW10			0x2b
#define MAX_DT_RAW12			0x2c
#define MAX_DT_RAW14			0x2d
#define MAX_DT_RAW16			0x2e
#define MAX_DT_RAW20			0x2f

const struct max_format *max_format_by_index(unsigned int index);
const struct max_format *max_format_by_code(u32 code);
const struct max_format *max_format_by_dt(u8 dt);

#endif // MAX_SERDES_H
