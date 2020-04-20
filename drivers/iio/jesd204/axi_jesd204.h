/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AXI_JESD204_H_
#define _AXI_JESD204_H_

#include <linux/bitfield.h>

#define JESD204_REG_SYNTH_REG_1		0x18
#define JESD204_ENCODER_MASK		GENMASK(9, 8)
#define JESD204_ENCODER_GET(x)		FIELD_GET(JESD204_ENCODER_MASK, x)

/* JESD204C Supported encoding scheme */
enum jesd204_encoder {
	JESD204_ENCODER_UNKNOWN,
	JESD204_ENCODER_8B10B,
	JESD204_ENCODER_64B66B,
	JESD204_ENCODER_MAX,
};

static const char *axi_jesd204_encoder_label[JESD204_ENCODER_MAX] = {
	"unknown",
	"8b10b",
	"64b66b"
};

#endif
