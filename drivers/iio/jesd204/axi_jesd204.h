/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _AXI_JESD204_H_
#define _AXI_JESD204_H_

#include <linux/bitfield.h>

#define JESD204_REG_SYNTH_REG_1		0x18
#define JESD204_ENCODER_MASK		GENMASK(9, 8)
#define JESD204_ENCODER_GET(x)		FIELD_GET(JESD204_ENCODER_MASK, x)

#endif
