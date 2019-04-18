/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2004-2007, 2010-2015 Freescale Semiconductor, Inc.
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 */

#ifndef __ASM_ARCH_MXC_H__
#define __ASM_ARCH_MXC_H__

#include <linux/types.h>
#include <soc/imx/cpu.h>

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#define IMX_DDR_TYPE_DDR3		0
#define IMX_DDR_TYPE_LPDDR2		1
#define IMX_DDR_TYPE_LPDDR3		2
#define IMX_MMDC_DDR_TYPE_LPDDR3	3

#define IMX_LPDDR2_1CH_MODE            0
#define IMX_LPDDR2_2CH_MODE            1

#ifndef __ASSEMBLY__

#ifdef CONFIG_SOC_IMX6SL
static inline bool cpu_is_imx6sl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SL;
}
#else
static inline bool cpu_is_imx6sl(void)
{
	return false;
}
#endif

static inline bool cpu_is_imx6dl(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6DL;
}

static inline bool cpu_is_imx6sx(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SX;
}

static inline bool cpu_is_imx6ul(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6UL;
}

static inline bool cpu_is_imx6ull(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULL;
}

static inline bool cpu_is_imx6ulz(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6ULZ;
}

static inline bool cpu_is_imx6sll(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6SLL;
}

static inline bool cpu_is_imx6q(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6Q;
}

static inline bool cpu_is_imx6(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX6Q ||
		__mxc_cpu_type == MXC_CPU_IMX6DL ||
		__mxc_cpu_type == MXC_CPU_IMX6SL ||
		__mxc_cpu_type == MXC_CPU_IMX6SX ||
		__mxc_cpu_type == MXC_CPU_IMX6UL ||
		__mxc_cpu_type == MXC_CPU_IMX6ULL ||
		__mxc_cpu_type == MXC_CPU_IMX6SLL ||
		__mxc_cpu_type == MXC_CPU_IMX6ULZ;
}

static inline bool cpu_is_imx7d(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7D;
}

static inline bool cpu_is_imx7ulp(void)
{
	return __mxc_cpu_type == MXC_CPU_IMX7ULP;
}

struct cpu_op {
	u32 cpu_rate;
};

int tzic_enable_wake(void);

extern struct cpu_op *(*get_cpu_op)(int *op);
#endif

#define imx_readl	readl_relaxed
#define imx_readw	readw_relaxed
#define imx_writel	writel_relaxed
#define imx_writew	writew_relaxed

#endif /*  __ASM_ARCH_MXC_H__ */
