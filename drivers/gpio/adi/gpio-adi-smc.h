/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2023, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __DRIVERS_GPIO_ADI_SMC_H
#define __DRIVERS_GPIO_ADI_SMC_H

#include <linux/types.h>

bool adi_adrv906x_pintmux_map(unsigned int gpio, bool polarity, unsigned int *irq, uintptr_t base_addr);
bool adi_adrv906x_pintmux_unmap(unsigned int gpio, uintptr_t base_addr);

#endif /* __DRIVERS_GPIO_ADI_SMC_H */
