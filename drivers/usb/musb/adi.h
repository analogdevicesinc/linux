/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef __ADI_H__
#define __ADI_H__

static int get_int_prop(struct device_node *node, const char *s);

#define REG_USB_VBUS_CTL	0x380
#define REG_USB_ID_CTL		0x382
#define REG_USB_PHY_CTL		0x394
#define REG_USB_PLL_OSC		0x398
#define REG_USB_UTMI_CTL	0x39c

#endif
