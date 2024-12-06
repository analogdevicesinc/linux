/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2019 NXP
 */
#ifndef _MXCFB_EPDC_KERNEL
#define _MXCFB_EPDC_KERNEL

struct imx_epdc_fb_mode {
    struct fb_videomode *vmode;
    int vscan_holdoff;
    int sdoed_width;
    int sdoed_delay;
    int sdoez_width;
    int sdoez_delay;
    int gdclk_hp_offs;
    int gdsp_offs;
    int gdoe_offs;
    int gdclk_offs;
    int num_ce;
};

struct imx_epdc_fb_platform_data {
    struct imx_epdc_fb_mode *epdc_mode;
    int num_modes;
    int (*get_pins) (void);
    void (*put_pins) (void);
    void (*enable_pins) (void);
    void (*disable_pins) (void);
};

#endif
