// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2020,2021 NXP
 */

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "dcnano-drv.h"
#include "dcnano-reg.h"

#define DCNANO_CRTC_PLL_MIN_RATE	271500000
#define DCNANO_CRTC_PLL_MAX_RATE	792000000

#define DCNANO_CRTC_PLL_MIN_DIV		1
#define DCNANO_CRTC_PLL_MAX_DIV		64

#define dcnano_crtc_dbg(crtc, fmt, ...)					\
	drm_dbg_kms((crtc)->dev, "[CRTC:%d:%s] " fmt,			\
		    (crtc)->base.id, (crtc)->name, ##__VA_ARGS__)

#define dcnano_crtc_err(crtc, fmt, ...)					\
	drm_err((crtc)->dev, "[CRTC:%d:%s] " fmt,			\
		(crtc)->base.id, (crtc)->name, ##__VA_ARGS__)

static inline struct dcnano_dev *crtc_to_dcnano_dev(struct drm_crtc *crtc)
{
	return to_dcnano_dev(crtc->dev);
}

static void dcnano_crtc_mode_set_nofb_dpi(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);
	struct drm_display_mode *adj = &crtc->state->adjusted_mode;
	u32 val;

	/* select output bus */
	dcnano_write(dcnano, DCNANO_DBICONFIG, DBICFG_BUS_OUTPUT_SEL_DPI);

	/* set bus format */
	dcnano_write(dcnano, DCNANO_DPICONFIG, DPICFG_DATA_FORMAT_D24);

	/* horizontal timing */
	val = HDISPLAY_END(adj->crtc_hdisplay) |
	      HDISPLAY_TOTAL(adj->crtc_htotal);
	dcnano_write(dcnano, DCNANO_HDISPLAY, val);

	val = HSYNC_START(adj->crtc_hsync_start) |
	      HSYNC_END(adj->crtc_hsync_end) | HSYNC_PULSE_ENABLE;
	if (adj->flags & DRM_MODE_FLAG_PHSYNC)
		val |= HSYNC_POL_POSITIVE;
	else
		val |= HSYNC_POL_NEGATIVE;
	dcnano_write(dcnano, DCNANO_HSYNC, val);

	/* vertical timing */
	val = VDISPLAY_END(adj->crtc_vdisplay) |
	      VDISPLAY_TOTAL(adj->crtc_vtotal);
	dcnano_write(dcnano, DCNANO_VDISPLAY, val);

	val = VSYNC_START(adj->crtc_vsync_start) |
	      VSYNC_END(adj->crtc_vsync_end) | VSYNC_PULSE_ENABLE;
	if (adj->flags & DRM_MODE_FLAG_PVSYNC)
		val |= VSYNC_POL_POSITIVE;
	else
		val |= VSYNC_POL_NEGATIVE;
	dcnano_write(dcnano, DCNANO_VSYNC, val);

	/* panel configuration */
	val = PANELCFG_DE_ENABLE | PANELCFG_DE_POL_POSITIVE |
	      PANELCFG_DATA_ENABLE | PANELCFG_DATA_POL_POSITIVE |
	      PANELCFG_CLOCK_ENABLE | PANELCFG_CLOCK_POL_POSITIVE |
	      PANELCFG_SEQUENCING_SOFTWARE;
	dcnano_write(dcnano, DCNANO_PANELCONFIG, val);
}

static bool dcnano_crtc_pll_clock_rate_is_valid(unsigned long pll_clk_rate)
{
	return pll_clk_rate >= DCNANO_CRTC_PLL_MIN_RATE &&
	       pll_clk_rate <= DCNANO_CRTC_PLL_MAX_RATE;
}

static unsigned long
dcnano_crtc_find_pll_clock_rate(struct drm_crtc *crtc,
				const struct drm_display_mode *mode)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);
	unsigned long pll_clk_rate, rounded_pll_clk_rate;
	int i;

	for (i = DCNANO_CRTC_PLL_MIN_DIV; i <= DCNANO_CRTC_PLL_MAX_DIV; i++) {
		pll_clk_rate = mode->clock * 1000 * i;

		if (!dcnano_crtc_pll_clock_rate_is_valid(pll_clk_rate))
			continue;

		rounded_pll_clk_rate = clk_round_rate(dcnano->pll_clk,
						      pll_clk_rate);
		if (rounded_pll_clk_rate != pll_clk_rate) {
			dcnano_crtc_dbg(crtc,
					"rounded pll clock rate %lu, expected %lu\n",
					rounded_pll_clk_rate, pll_clk_rate);
			continue;
		}

		dcnano_crtc_dbg(crtc, "find pll clock rate %lu with div %d\n",
				pll_clk_rate, i);

		return pll_clk_rate;
	}

	dcnano_crtc_dbg(crtc, "failed to find pll clock rate\n");

	return 0;
}

static void dcnano_crtc_set_pixel_clock(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);
	struct drm_display_mode *adj = &crtc->state->adjusted_mode;
	struct clk *parent;
	unsigned long pixel_clk_rate = adj->crtc_clock * 1000;
	unsigned long pll_clk_rate;
	int ret;

	parent = clk_get_parent(dcnano->pixel_clk);
	if (!parent) {
		dcnano_crtc_err(crtc, "%s: no pixel clock's parent\n", __func__);
	} else if (IS_ERR(parent)) {
		ret = PTR_ERR(parent);
		dcnano_crtc_err(crtc,
				"%s: failed to get pixel clock's parent: %d\n",
				__func__, ret);
		return;
	}

	pll_clk_rate = dcnano_crtc_find_pll_clock_rate(crtc, adj);
	if (pll_clk_rate == 0)
		dcnano_crtc_err(crtc, "%s: failed to find pll clock rate\n",
				__func__);

	ret = clk_set_rate(dcnano->pll_clk, pll_clk_rate);
	if (ret)
		dcnano_crtc_err(crtc, "%s: failed to set pll clock rate: %d\n",
				__func__, ret);

	/* FIXME: The rate of pixel clock's parent is pixel clock rate. */
	ret = clk_set_rate(parent, pixel_clk_rate);
	if (ret)
		dcnano_crtc_err(crtc,
				"%s: failed to set pixel clock's parent rate: %d\n",
				__func__, ret);

	ret = clk_set_rate(dcnano->pixel_clk, pixel_clk_rate);
	if (ret)
		dcnano_crtc_err(crtc, "%s: failed to set pixel clock rate: %d\n",
				__func__, ret);

	ret = clk_prepare_enable(dcnano->pixel_clk);
	if (ret)
		dcnano_crtc_err(crtc, "%s: failed to enable pixel clock: %d\n",
				__func__, ret);

	dcnano_crtc_dbg(crtc, "%s: get pll clock rate: %lu\n",
			__func__, clk_get_rate(dcnano->pll_clk));

	dcnano_crtc_dbg(crtc, "%s: get rate of pixel clock's parent: %lu\n",
			__func__, clk_get_rate(parent));

	dcnano_crtc_dbg(crtc, "%s: get pixel clock rate %lu\n",
			__func__, clk_get_rate(dcnano->pixel_clk));
}

static enum drm_mode_status
dcnano_crtc_mode_valid(struct drm_crtc *crtc,
		       const struct drm_display_mode *mode)
{
	dcnano_crtc_dbg(crtc, "validating mode " DRM_MODE_FMT "\n",
			DRM_MODE_ARG(mode));

	if (dcnano_crtc_find_pll_clock_rate(crtc, mode) == 0)
		return MODE_NOCLOCK;

	return MODE_OK;
}

static void dcnano_crtc_queue_state_event(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		WARN_ON(dcnano->event);
		dcnano->event = crtc->state->event;
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static int dcnano_crtc_atomic_check(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	bool has_primary = crtc_state->plane_mask &
			   drm_plane_mask(crtc->primary);

	if (crtc_state->active && !has_primary)
		return -EINVAL;

	if (crtc_state->active_changed && crtc_state->active)
		crtc_state->mode_changed = true;

	return 0;
}

static void dcnano_crtc_atomic_flush(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state,
									      crtc);

	if (!crtc->state->active && !old_crtc_state->active)
		return;

	if (!drm_atomic_crtc_needs_modeset(crtc->state))
		dcnano_crtc_queue_state_event(crtc);
}

static void dcnano_crtc_atomic_enable(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct drm_device *drm = crtc->dev;
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);
	struct drm_plane *plane;
	struct drm_plane_state *new_plane_state;
	struct drm_display_mode *adj = &crtc->state->adjusted_mode;
	int i;
	u32 primary_fb_fmt = 0;
	u32 val;

	dcnano_crtc_dbg(crtc, "mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(adj));

	dcnano_crtc_set_pixel_clock(crtc);

	/* enable power when we start to set mode for CRTC */
	pm_runtime_get_sync(drm->dev);

	if (dcnano->port == DCNANO_DPI_PORT)
		dcnano_crtc_mode_set_nofb_dpi(crtc);

	drm_crtc_vblank_on(crtc);

	for_each_new_plane_in_state(state, plane, new_plane_state, i) {
		if (!new_plane_state->fb)
			continue;

		if (plane->type != DRM_PLANE_TYPE_PRIMARY)
			continue;

		switch (new_plane_state->fb->format->format) {
		case DRM_FORMAT_RGB565:
			primary_fb_fmt = FBCFG_FORMAT_R5G6B5;
			break;
		case DRM_FORMAT_XRGB8888:
			primary_fb_fmt = FBCFG_FORMAT_R8G8B8;
			break;
		}
	}

	val = FBCFG_OUTPUT_ENABLE | primary_fb_fmt;

	/* enable DPI timing and start a DPI transfer, if needed */
	if (dcnano->port == DCNANO_DPI_PORT)
		val |= FBCFG_RESET_ENABLE;

	dcnano_write(dcnano, DCNANO_FRAMEBUFFERCONFIG, val);

	dcnano_crtc_queue_state_event(crtc);
}

static void dcnano_crtc_atomic_disable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct drm_device *drm = crtc->dev;
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);

	/* simply write '0' to the framebuffer and timing control register */
	dcnano_write(dcnano, DCNANO_FRAMEBUFFERCONFIG, 0);

	drm_crtc_vblank_off(crtc);

	/* disable power when CRTC is disabled */
	pm_runtime_put_sync(drm->dev);

	clk_disable_unprepare(dcnano->pixel_clk);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static bool
dcnano_crtc_get_scanout_position(struct drm_crtc *crtc,
				 bool in_vblank_irq,
				 int *vpos, int *hpos,
				 ktime_t *stime, ktime_t *etime,
				 const struct drm_display_mode *mode)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);
	int hdisplay = mode->crtc_hdisplay;
	int htotal = mode->crtc_htotal;
	int vdisplay = mode->crtc_vdisplay;
	int vtotal = mode->crtc_vtotal;
	int x, y;
	u32 val;
	bool reliable;

	if (stime)
		*stime = ktime_get();

	val = dcnano_read(dcnano, DCNANO_DISPLAYCURRENTLOCATION);

	x = CURRENTLOCATION_X_GET(val);
	y = CURRENTLOCATION_Y_GET(val);

	if (x < hdisplay)
		*hpos = x + 1; /* active scanout area - positive */
	else
		*hpos = x - (htotal - 1); /* inside vblank - negative */

	if (y < vdisplay)
		*vpos = y + 1; /* active scanout area - positive */
	else
		*vpos = y - (vtotal - 1); /* inside vblank - negative */

	reliable = true;

	if (etime)
		*etime = ktime_get();

	return reliable;
}

static const struct drm_crtc_helper_funcs dcnano_crtc_helper_funcs = {
	.mode_valid		= dcnano_crtc_mode_valid,
	.atomic_check		= dcnano_crtc_atomic_check,
	.atomic_flush		= dcnano_crtc_atomic_flush,
	.atomic_enable		= dcnano_crtc_atomic_enable,
	.atomic_disable		= dcnano_crtc_atomic_disable,
	.get_scanout_position	= dcnano_crtc_get_scanout_position,
};

static u32 dcnano_crtc_get_vblank_counter(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);

	dcnano_write(dcnano, DCNANO_DEBUGCOUNTERSELECT, TOTAL_FRAME_CNT);
	return dcnano_read(dcnano, DCNANO_DEBUGCOUNTERVALUE);
}

static int dcnano_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);

	dcnano_write(dcnano, DCNANO_DISPLAYINTRENABLE, DISPLAYINTR_DISP0);

	return 0;
}

static void dcnano_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct dcnano_dev *dcnano = crtc_to_dcnano_dev(crtc);

	dcnano_write(dcnano, DCNANO_DISPLAYINTRENABLE, 0);
}

static const struct drm_crtc_funcs dcnano_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.get_vblank_counter	= dcnano_crtc_get_vblank_counter,
	.enable_vblank		= dcnano_crtc_enable_vblank,
	.disable_vblank		= dcnano_crtc_disable_vblank,
	.get_vblank_timestamp	= drm_crtc_vblank_helper_get_vblank_timestamp,
};

irqreturn_t dcnano_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct dcnano_dev *dcnano = to_dcnano_dev(drm);
	unsigned long flags;

	/* DCNANO_DISPLAYINTR will automatically clear after a read. */
	dcnano_read(dcnano, DCNANO_DISPLAYINTR);

	drm_crtc_handle_vblank(&dcnano->crtc);

	spin_lock_irqsave(&drm->event_lock, flags);
	if (dcnano->event) {
		drm_crtc_send_vblank_event(&dcnano->crtc, dcnano->event);
		dcnano->event = NULL;
		drm_crtc_vblank_put(&dcnano->crtc);
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);

	return IRQ_HANDLED;
}

static int dcnano_get_pll_clock(struct dcnano_dev *dcnano)
{
	int ret, i;

	/*
	 * There are one divider clock and gate clock between the pixel
	 * clock and the pll clock, so walk through the clock tree 3 levels
	 * up to get the pll clock.
	 */
	dcnano->pll_clk = dcnano->pixel_clk;
	for (i = 0; i < 3; i++) {
		dcnano->pll_clk = clk_get_parent(dcnano->pll_clk);
		if (IS_ERR(dcnano->pll_clk)) {
			ret = PTR_ERR(dcnano->pll_clk);
			drm_err(&dcnano->base,
				"failed to get pll clock: %d\n", ret);
			return ret;
		} else if (!dcnano->pll_clk) {
			drm_err(&dcnano->base, "no pll clock\n");
			return -ENODEV;
		}
	}

	return 0;
}

static void dcnano_reset_all_debug_counters(struct dcnano_dev *dcnano)
{
	struct drm_device *drm = &dcnano->base;

	pm_runtime_get_sync(drm->dev);
	dcnano_write(dcnano, DCNANO_DEBUGCOUNTERSELECT, RESET_ALL_CNTS);
	pm_runtime_put_sync(drm->dev);
}

int dcnano_crtc_init(struct dcnano_dev *dcnano)
{
	int ret;

	ret = dcnano_plane_init(dcnano);
	if (ret)
		return ret;

	drm_crtc_helper_add(&dcnano->crtc, &dcnano_crtc_helper_funcs);
	ret = drm_crtc_init_with_planes(&dcnano->base, &dcnano->crtc,
					&dcnano->primary, NULL,
					&dcnano_crtc_funcs, NULL);
	if (ret) {
		drm_err(&dcnano->base, "failed to initialize CRTC: %d\n", ret);
		return ret;
	}

	ret = dcnano_get_pll_clock(dcnano);
	if (ret)
		return ret;

	dcnano_reset_all_debug_counters(dcnano);

	return 0;
}
