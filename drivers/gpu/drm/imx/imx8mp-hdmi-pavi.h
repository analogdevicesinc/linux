// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 * PAI/PVI Head file
 *
 */
#ifndef _IMX8MP_HDMI_AV_CTL_H_
#define _IMX8MP_HDMI_AV_CTL_H_

#include <linux/clk.h>
#include <linux/reset.h>
#include <drm/drm_modes.h>

struct imx8mp_hdmi_pavi {
	struct device *dev;

	void __iomem *base;
	atomic_t rpm_suspended;

	struct clk *clk_pai;
	struct clk *clk_pvi;
	struct reset_control *reset_pai;
	struct reset_control *reset_pvi;
};

void imx8mp_hdmi_pai_enable(int channel, int width, int rate, int non_pcm);
void imx8mp_hdmi_pai_disable(void);

void imx8mp_hdmi_pvi_enable(const struct drm_display_mode *mode);
void imx8mp_hdmi_pvi_disable(void);

void imx8mp_hdmi_pavi_powerup(void);
void imx8mp_hdmi_pavi_powerdown(void);

struct imx8mp_hdmi_pavi *imx8mp_hdmi_pavi_init(void);

#endif /* _IMX8MP_HDMI_PAVI_H_ */
