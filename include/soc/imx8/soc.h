/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef __SOC_IMX8_SOC_H__
#define __SOC_IMX8_SOC_H__

#define IMX_SOC_IMX8QM		0x01
#define IMX_SOC_IMX8QXP		0x02
#define IMX_SOC_IMX8MQ		0x82

bool cpu_is_imx8qm(void);
bool cpu_is_imx8mq(void);
bool cpu_is_imx8qxp(void);

extern bool TKT340553_SW_WORKAROUND;
unsigned int imx8_get_soc_revision(void);

#endif
