/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2020,2021 NXP
 */

#ifndef __DCNANO_DRV_H__
#define __DCNANO_DRV_H__

#include <linux/clk.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/reset.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>
#include <drm/drm_plane.h>
#include <drm/drm_vblank.h>

enum dcnano_port {
	DCNANO_DPI_PORT,
	DCNANO_DBI_PORT,
	DCNANO_PORT_NUM,
};

struct dcnano_dev {
	struct drm_device base;
	void __iomem *mmio_base;

	unsigned int irq;

	struct clk *axi_clk;
	struct clk *ahb_clk;
	struct clk *pixel_clk;
	struct clk *pll_clk;

	struct reset_control *tied_resets;

	struct drm_crtc crtc;
	struct drm_plane primary;
	struct drm_encoder encoder;

	struct drm_pending_vblank_event *event;

	enum dcnano_port port;
};

static inline struct dcnano_dev *to_dcnano_dev(struct drm_device *drm)
{
	return container_of(drm, struct dcnano_dev, base);
}

irqreturn_t dcnano_irq_handler(int irq, void *data);

int dcnano_crtc_init(struct dcnano_dev *dcnano);

int dcnano_plane_init(struct dcnano_dev *dcnano);

int dcnano_kms_prepare(struct dcnano_dev *dcnano);

#endif
