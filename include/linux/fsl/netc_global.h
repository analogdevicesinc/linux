/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2024 NXP
 */
#ifndef __NETC_GLOBAL_H
#define __NETC_GLOBAL_H

#include <linux/io.h>
#include <linux/platform_device.h>

static inline u32 netc_read(void __iomem *reg)
{
	return ioread32(reg);
}

#ifdef ioread64
static inline u64 netc_read64(void __iomem *reg)
{
	return ioread64(reg);
}
#else
static inline u64 netc_read64(void __iomem *reg)
{
	u32 low, high;
	u64 val;

	low = ioread32(reg);
	high = ioread32(reg + 4);

	val = (u64)high << 32 | low;

	return val;
}
#endif

static inline void netc_write(void __iomem *reg, u32 val)
{
	iowrite32(val, reg);
}

#if IS_ENABLED(CONFIG_NXP_NETC_BLK_CTRL)
void netc_emdio_supplier_register(struct device *supplier);
int netc_emdio_consumer_register(struct device *consumer);
int netc_check_emdio_state(void);
void netc_ierb_enable_wakeonlan(void);
void netc_ierb_disable_wakeonlan(void);
int netc_ierb_may_wakeonlan(void);
#else
static inline void netc_emdio_supplier_register(struct device *emdio)
{
}

static inline int netc_emdio_consumer_register(struct device *consumer)
{
	return 0;
}

static inline int netc_check_emdio_state(void)
{
	return 0;
}

static inline void netc_ierb_enable_wakeonlan(void)
{
}

static inline void netc_ierb_disable_wakeonlan(void)
{
}

static inline int netc_ierb_may_wakeonlan(void)
{
	return -EINVAL;
}
#endif

#if IS_ENABLED(CONFIG_PTP_1588_CLOCK_NETC)
int netc_timer_get_phc_index(int domain, unsigned int bus, unsigned int devfn);
#else
static inline int netc_timer_get_phc_index(int domain, unsigned int bus,
					   unsigned int devfn)
{
	return -1;
}
#endif

#endif
