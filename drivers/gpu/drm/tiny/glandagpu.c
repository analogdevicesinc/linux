// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>		/* GFP_KERNEL */
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/iosys-map.h>

#include <drm/drm_drv.h>
#include <drm/drm_device.h>
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>
#include <drm/drm_vblank_helper.h>

#include <drm/drm_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_plane.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_print.h>

/* Hardware Constants */
#define GLANDA_WIDTH      640
#define GLANDA_HEIGHT     480
#define GLANDA_VRAM_SIZE  (GLANDA_WIDTH * GLANDA_HEIGHT * 4)
#define GLANDA_MMIO_SIZE  32
#define GLANDA_MMIO_OFFSET 0x00200000

/* QEMU test device ID, from the range reserved for experimental use (docs/specs/pci-ids.rst). */
#define PCI_DEVICE_ID_GLANDA_GPU 0x10f0

/* Register Offsets */
#define REG_STATUS  0x00
#define REG_CTRL    0x04
#define REG_COORD0  0x08
#define REG_COORD1  0x0C
#define REG_COLOR   0x10
#define REG_ISR     0x14
#define REG_IER     0x18

/* Bit Masks */
#define INT_DONE    BIT(0)
#define INT_VSYNC   BIT(1)

#define STATUS_BUSY BIT(0)
#define CMD_CLEAR   (0x1)
#define CMD_RECT    (0x2)
#define CMD_LINE    (0x3)
#define CTRL_START  BIT(4)

struct glanda_device {
	struct drm_device drm;

	/* hw */
	void __iomem *mmio_base;
	void __iomem *vram_base;
	phys_addr_t vram_phys;

	int irq;

	/* drm */
	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

#define to_glanda(dev) container_of_const(dev, struct glanda_device, drm)

static const u32 glanda_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static void glanda_blit_rect(struct glanda_device *gdev,
			     const struct drm_rect *dst_clip,
			     const struct iosys_map *src,
			     struct drm_framebuffer *fb,
			     int dst_off_x, int dst_off_y,
				 struct drm_format_conv_state *fmtcnv_state)
{
	unsigned int src_pitch = fb->pitches[0];
	unsigned int width = drm_rect_width(dst_clip);
	unsigned int height = drm_rect_height(dst_clip);
	unsigned int x, y;
	size_t len = width * sizeof(u32);
	u32 *sbuf;

	sbuf = drm_format_conv_state_reserve(fmtcnv_state, len, GFP_KERNEL);
	if (!sbuf)
		return;

	for (y = 0; y < height; y++) {
		unsigned int dst_y = dst_clip->y1 + y;
		unsigned int src_y = dst_y - dst_off_y;
		u32 __iomem *dst = (u32 __iomem *)gdev->vram_base +
				   (size_t)dst_y * GLANDA_WIDTH + dst_clip->x1;
		size_t src_off = (size_t)src_y * src_pitch +
				 (size_t)(dst_clip->x1 - dst_off_x) * sizeof(u32);

		iosys_map_memcpy_from(sbuf, src, src_off, len);

		for (x = 0; x < width; x++) {
			u32 pixel = le32_to_cpu((__force __le32)sbuf[x]);
			u32 packed = ((pixel >> 12) & 0x0F00) |
				((pixel >> 8) & 0x00F0) |
				((pixel >> 4) & 0x000F);

			writel_relaxed(packed, &dst[x]);
		}
	}
}

static void glanda_plane_atomic_update(struct drm_plane *plane,
				       struct drm_atomic_commit *state)
{
	struct drm_plane_state *old_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_plane_state *new_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_state = to_drm_shadow_plane_state(new_state);
	struct drm_rect vram_clip = DRM_RECT_INIT(0, 0, GLANDA_WIDTH, GLANDA_HEIGHT);
	struct glanda_device *gdev = to_glanda(plane->dev);
	struct drm_framebuffer *fb = new_state->fb;
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int dst_off_x, dst_off_y;
	int ret, idx;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return;

	if (!drm_dev_enter(plane->dev, &idx))
		goto out_drm_gem_fb_end_cpu_access;

	dst_off_x = new_state->dst.x1 - (new_state->src.x1 >> 16);
	dst_off_y = new_state->dst.y1 - (new_state->src.y1 >> 16);

	drm_atomic_helper_damage_iter_init(&iter, old_state, new_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		struct drm_rect dst_clip = new_state->dst;

		drm_rect_translate(&damage, dst_off_x, dst_off_y);

		if (!drm_rect_intersect(&dst_clip, &damage))
			continue;
		if (!drm_rect_intersect(&dst_clip, &vram_clip))
			continue;

		glanda_blit_rect(gdev, &dst_clip, &shadow_state->data[0], fb,
				 dst_off_x, dst_off_y, &shadow_state->fmtcnv_state);
	}

	drm_dev_exit(idx);
out_drm_gem_fb_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
}

static void glanda_plane_atomic_disable(struct drm_plane *plane,
					struct drm_atomic_commit *state)
{
	struct drm_device *dev = plane->dev;
	struct glanda_device *gdev = to_glanda(dev);
	int idx;

	if (!drm_dev_enter(dev, &idx))
		return;

	memset_io(gdev->vram_base, 0, GLANDA_WIDTH * sizeof(u32) * GLANDA_HEIGHT);
	drm_dev_exit(idx);
}

static int glanda_plane_atomic_check(struct drm_plane *plane,
				     struct drm_atomic_commit *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_crtc_state *new_crtc_state = NULL;
	int ret;

	if (new_plane_state->crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_plane_state->crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  DRM_PLANE_NO_SCALING, DRM_PLANE_NO_SCALING,
		false,	/* can_position */
		false); /* can_update_disabled */
	if (ret)
		return ret;

	return 0;
}

static const struct drm_plane_helper_funcs glanda_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_update = glanda_plane_atomic_update,
	.atomic_check = glanda_plane_atomic_check,
	.atomic_disable = glanda_plane_atomic_disable,
};

static const struct drm_plane_funcs glanda_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static int glanda_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		dev_err(connector->dev->dev, "GlandaGPU: failed to create display mode\n");
		return 0;
	}

	/* Standard VGA timing: 640x480 @ 60 Hz. */
	mode->hdisplay = 640;
	mode->hsync_start = 656;
	mode->hsync_end = 752;
	mode->htotal = 800;

	mode->vdisplay = 480;
	mode->vsync_start = 490;
	mode->vsync_end = 492;
	mode->vtotal = 525;

	mode->clock = 25175;	/* 25.175 MHz pixel clock */

	mode->flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC;
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	return 1;
}

static int glanda_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct glanda_device *gdev = to_glanda(crtc->dev);
	u32 ier;
	int idx;

	if (!drm_dev_enter(crtc->dev, &idx))
		return -ENODEV;

	ier = readl(gdev->mmio_base + REG_IER);
	writel(ier | INT_VSYNC, gdev->mmio_base + REG_IER);

	drm_dev_exit(idx);
	return 0;
}

static void glanda_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct glanda_device *gdev = to_glanda(crtc->dev);
	u32 ier;
	int idx;

	if (!drm_dev_enter(crtc->dev, &idx))
		return;

	ier = readl(gdev->mmio_base + REG_IER);
	writel(ier & ~INT_VSYNC, gdev->mmio_base + REG_IER);

	drm_dev_exit(idx);
}

static const struct drm_crtc_funcs glanda_crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_create_state = drm_atomic_helper_crtc_create_state,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = glanda_crtc_enable_vblank,
	.disable_vblank = glanda_crtc_disable_vblank,
};

static const struct drm_crtc_helper_funcs glanda_crtc_helper_funcs = {
	.atomic_enable = drm_crtc_vblank_atomic_enable,
	.atomic_disable = drm_crtc_vblank_atomic_disable,
	.atomic_flush = drm_crtc_vblank_atomic_flush,
};

static const struct drm_connector_helper_funcs glanda_connector_helper_funcs = {
	.get_modes = glanda_connector_get_modes,
};

static const struct drm_encoder_funcs glanda_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_connector_funcs glanda_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_create_state = drm_atomic_helper_connector_create_state,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs glanda_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_FOPS(glanda_drm_fops);

static const struct drm_driver glanda_drm_driver = {
	.driver_features =
	    DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.name = "glandagpu",
	.desc = "GlandaGPU DRM Driver",
	.major = 1,
	.minor = 0,
	.fops = &glanda_drm_fops,
	.dumb_create = drm_gem_shmem_dumb_create,
};

static irqreturn_t glanda_irq_handler(int irq, void *dev_id)
{
	struct glanda_device *gdev = dev_id;
	u32 isr, ier;

	if (!gdev || !gdev->mmio_base)
		return IRQ_NONE;

	isr = readl(gdev->mmio_base + REG_ISR);
	if (unlikely(isr == 0xFFFFFFFF))
		return IRQ_NONE;

	ier = readl(gdev->mmio_base + REG_IER);

	if (!(isr & ier))
		return IRQ_NONE;

	if (isr & INT_VSYNC)
		drm_crtc_handle_vblank(&gdev->crtc);

	/* Clear interrupt(W1C) */
	writel(isr, gdev->mmio_base + REG_ISR);
	return IRQ_HANDLED;
}

/* Common DRM setup once MMIO/VRAM/IRQ are known(used by both probe paths) */
static int glanda_drm_init(struct glanda_device *gdev, int irq)
{
	int ret;

	writel(0, gdev->mmio_base + REG_IER);
	writel(0xFFFFFFFF, gdev->mmio_base + REG_ISR);	/* clear flags */

	/* DRM mode config */
	ret = drmm_mode_config_init(&gdev->drm);
	if (ret)
		return ret;

	gdev->drm.mode_config.min_width = 640;
	gdev->drm.mode_config.min_height = 480;
	gdev->drm.mode_config.max_width = DRM_SHADOW_PLANE_MAX_WIDTH;
	gdev->drm.mode_config.max_height = DRM_SHADOW_PLANE_MAX_HEIGHT;
	gdev->drm.mode_config.funcs = &glanda_mode_config_funcs;

	ret = drm_universal_plane_init(&gdev->drm, &gdev->primary_plane, 1 << 0,
				       &glanda_plane_funcs,
				       glanda_plane_formats,
				       ARRAY_SIZE(glanda_plane_formats), NULL,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret) {
		drm_err(&gdev->drm, "Failed to initialize primary plane\n");
		return ret;
	}
	drm_plane_helper_add(&gdev->primary_plane, &glanda_plane_helper_funcs);

	drm_plane_enable_fb_damage_clips(&gdev->primary_plane);

	/* CRTC init */
	ret = drm_crtc_init_with_planes(&gdev->drm, &gdev->crtc,
					&gdev->primary_plane, NULL,
					&glanda_crtc_funcs, NULL);
	if (ret) {
		drm_err(&gdev->drm, "Failed to initialize CRTC with planes\n");
		return ret;
	}
	drm_crtc_helper_add(&gdev->crtc, &glanda_crtc_helper_funcs);

	ret = drm_encoder_init(&gdev->drm, &gdev->encoder, &glanda_encoder_funcs,
			       DRM_MODE_ENCODER_DAC, NULL);
	if (ret) {
		drm_err(&gdev->drm, "Failed to initialize encoder\n");
		return ret;
	}
	gdev->encoder.possible_crtcs = 1;

	ret = drm_connector_init(&gdev->drm, &gdev->connector,
				 &glanda_connector_funcs, DRM_MODE_CONNECTOR_VGA);
	if (ret) {
		drm_err(&gdev->drm, "Failed to initialize connector\n");
		return ret;
	}
	drm_connector_helper_add(&gdev->connector, &glanda_connector_helper_funcs);

	drm_connector_attach_encoder(&gdev->connector, &gdev->encoder);

	/* VBlank init */
	ret = drm_vblank_init(&gdev->drm, 1);
	if (ret) {
		drm_err(&gdev->drm, "Failed to initialize vblank\n");
		return ret;
	}

	drm_mode_config_reset(&gdev->drm);

	gdev->irq = irq;
	ret = devm_request_irq(gdev->drm.dev, gdev->irq, glanda_irq_handler,
			       IRQF_SHARED, "glandagpu", gdev);
	if (ret) {
		drm_err(&gdev->drm, "Failed to request IRQ %d\n",
			gdev->irq);
		return ret;
	}

	ret = drm_dev_register(&gdev->drm, 0);
	if (ret)
		return ret;

	return 0;
}

/* Shared teardown, mirrors glanda_drm_init() */
static void glanda_drm_fini(struct glanda_device *gdev)
{
	drm_dev_unplug(&gdev->drm);
	drm_atomic_helper_shutdown(&gdev->drm);
}

static int glandagpu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct glanda_device *gdev;
	int irq;

	gdev = devm_drm_dev_alloc(&pdev->dev, &glanda_drm_driver, struct glanda_device, drm);
	if (IS_ERR(gdev))
		return PTR_ERR(gdev);

	platform_set_drvdata(pdev, gdev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (resource_size(res) < GLANDA_MMIO_OFFSET + GLANDA_MMIO_SIZE) {
		dev_err(&pdev->dev, "MMIO region too small: %llu bytes, need at least %u\n",
			(unsigned long long)resource_size(res),
			GLANDA_MMIO_OFFSET + GLANDA_MMIO_SIZE);
		return -EINVAL;
	}

	gdev->vram_phys = res->start;
	gdev->vram_base = devm_ioremap_wc(&pdev->dev, res->start, GLANDA_VRAM_SIZE);
	gdev->mmio_base = devm_ioremap(&pdev->dev, res->start + GLANDA_MMIO_OFFSET,
				       GLANDA_MMIO_SIZE);
	if (!gdev->vram_base || !gdev->mmio_base) {
		drm_err(&gdev->drm, "failed to ioremap\n");
		return -ENOMEM;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	return glanda_drm_init(gdev, irq);
}

static void glandagpu_remove(struct platform_device *pdev)
{
	glanda_drm_fini(platform_get_drvdata(pdev));
}

static void glandagpu_shutdown(struct platform_device *pdev)
{
	struct glanda_device *gdev = platform_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&gdev->drm);
}

/* Device Tree match table. */
static const struct of_device_id glanda_of_match[] = {
	{ .compatible = "kieweg,gpu-1.0" },
	{ }
};

MODULE_DEVICE_TABLE(of, glanda_of_match);

static struct platform_driver glandagpu_driver = {
	.driver = {
		.name = "glandagpu",
		.of_match_table = glanda_of_match,
	},
	.probe = glandagpu_probe,
	.remove = glandagpu_remove,
	.shutdown = glandagpu_shutdown,
};

/* PCI probe path for the QEMU test device, real hardware uses platform_driver */
#ifdef CONFIG_PCI
static int glandagpu_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct glanda_device *gdev;
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;
	pci_set_master(pdev);

	if (pci_resource_len(pdev, 0) < GLANDA_MMIO_SIZE ||
	    pci_resource_len(pdev, 1) < GLANDA_VRAM_SIZE) {
		dev_err(&pdev->dev, "BAR too small: BAR0=%llu (need %u), BAR1=%llu (need %u)\n",
			(unsigned long long)pci_resource_len(pdev, 0), GLANDA_MMIO_SIZE,
			(unsigned long long)pci_resource_len(pdev, 1), GLANDA_VRAM_SIZE);
		return -EINVAL;
	}

	ret = pcim_iomap_regions(pdev, BIT(0), "glandagpu");
	if (ret)
		return ret;

	ret = pcim_request_region(pdev, 1, "glandagpu");
	if (ret)
		return ret;

	gdev = devm_drm_dev_alloc(&pdev->dev, &glanda_drm_driver, struct glanda_device, drm);
	if (IS_ERR(gdev))
		return PTR_ERR(gdev);

	pci_set_drvdata(pdev, gdev);

	gdev->mmio_base = pcim_iomap_table(pdev)[0];
	gdev->vram_phys = pci_resource_start(pdev, 1);

	gdev->vram_base = devm_ioremap_wc(&pdev->dev, gdev->vram_phys, GLANDA_VRAM_SIZE);
	if (!gdev->vram_base)
		return -ENOMEM;

	return glanda_drm_init(gdev, pdev->irq);
}

static void glandagpu_pci_remove(struct pci_dev *pdev)
{
	glanda_drm_fini(pci_get_drvdata(pdev));
}

static void glandagpu_pci_shutdown(struct pci_dev *pdev)
{
	struct glanda_device *gdev = pci_get_drvdata(pdev);

	drm_atomic_helper_shutdown(&gdev->drm);
}

static const struct pci_device_id glanda_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_REDHAT_QUMRANET, PCI_DEVICE_ID_GLANDA_GPU) },
	{ }
};

MODULE_DEVICE_TABLE(pci, glanda_pci_ids);

static struct pci_driver glandagpu_pci_driver = {
	.name = "glandagpu-pci",
	.id_table = glanda_pci_ids,
	.probe = glandagpu_pci_probe,
	.remove = glandagpu_pci_remove,
	.shutdown = glandagpu_pci_shutdown,
};
#endif /* CONFIG_PCI */

static int __init glandagpu_init(void)
{
	int ret;

	ret = platform_driver_register(&glandagpu_driver);
	if (ret) {
		pr_err("GlandaGPU: Failed to register platform driver\n");
		return ret;
	}

#ifdef CONFIG_PCI
	ret = pci_register_driver(&glandagpu_pci_driver);
	if (ret) {
		pr_err("GlandaGPU: Failed to register PCI driver\n");
		platform_driver_unregister(&glandagpu_driver);
		return ret;
	}
#endif

	return 0;
}

static void __exit glandagpu_exit(void)
{
#ifdef CONFIG_PCI
	pci_unregister_driver(&glandagpu_pci_driver);
#endif
	platform_driver_unregister(&glandagpu_driver);
}

module_init(glandagpu_init);
module_exit(glandagpu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Leander Kieweg <kieweg.leander@gmail.com>");
MODULE_DESCRIPTION("DRM driver for GlandaGPU, an FPGA-based 2D GPU with VGA output");
