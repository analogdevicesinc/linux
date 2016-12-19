/*
 * Copyright (C) 2017 Intel Corporation.
 *
 * Intel Video and Image Processing(VIP) Frame Buffer II driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This is based on a driver made by Thomas Chou <thomas@wytron.com.tw> and
 * Walter Goossens <waltergoossens@home.nl> This driver supports the Intel VIP
 * Frame Buffer II component. A large portion of this file was derived from
 * altvipfb2.c which was created by Chris Rauer <christopher.rauer@intel.com>.
 * More info on the hardware can be found in the Intel Video and Image
 * Processing Suite User Guide at this address
 * http://www.altera.com/literature/ug/ug_vip.pdf.
 *
 */

#include "altvipfb2.h"
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>

static int altvipfb2_setcolreg(unsigned int regno, unsigned int red,
			       unsigned int green, unsigned int blue,
			       unsigned int transp, struct fb_info *info)
{
	/*
	 *  Set a single color register. The values supplied have a 32 bit
	 *  magnitude.
	 *  Return != 0 for invalid regno.
	 */

	if (regno > 255)
		return 1;

	red >>= 8;
	green >>= 8;
	blue >>= 8;

	if (regno < 255) {
		((u32 *)info->pseudo_palette)[regno] =
		((red & 255) << 16) | ((green & 255) << 8) | (blue & 255);
	}

	return 0;
}

static struct fb_ops altvipfb2_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_setcolreg = altvipfb2_setcolreg,
};

static void altvipfb2_start_hw(void __iomem *base, struct fb_info *info)
{
	/*
	 * The frameinfo variable has to correspond to the size of the VIP Suite
	 * Frame Reader register 7 which will determine the maximum size used
	 * in this frameinfo
	 */
	u32 frameinfo =
		readl(base + ALTVIPFB2_FRAME_READER) & 0x00ffffff;

	writel(frameinfo, base + ALTVIPFB2_FRAME_INFO);

	writel(info->fix.smem_start, base + ALTVIPFB2_FRAME_START);
	/* Finally set the control register to 1 to start streaming */
	writel(1, base + ALTVIPFB2_CONTROL);
}

static void altvipfb2_disable_hw(void __iomem *base)
{
	/* set the control register to 0 to stop streaming */
	writel(0, base + ALTVIPFB2_CONTROL);
}

static void altvipfb2_setup_fb_info(struct altvipfb2_priv *fbpriv)
{
	struct fb_info *info = &fbpriv->info;

	strncpy(info->fix.id, DRIVER_NAME, sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;

	info->fbops = &altvipfb2_ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = -1;
	info->var.width = -1;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->var.pixclock = 6734;
	info->var.left_margin = 148;
	info->var.right_margin = 88;
	info->var.upper_margin = 36;
	info->var.lower_margin = 4;
	info->var.hsync_len = 44;
	info->var.vsync_len = 5;

	/* settings for 32bit pixels */
	info->var.red.offset = 16;
	info->var.red.length = 8;
	info->var.red.msb_right = 0;
	info->var.green.offset = 8;
	info->var.green.length = 8;
	info->var.green.msb_right = 0;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;
	info->var.blue.msb_right = 0;
	info->pseudo_palette = fbpriv->pseudo_palette;

	info->flags = FBINFO_FLAG_DEFAULT;
}

int altvipfb2_probe(struct device *dev, void __iomem *base)
{
	int retval;
	void *fbmem_virt;
	struct altvipfb2_priv *fbpriv = dev_get_drvdata(dev);

	fbmem_virt = dma_alloc_coherent(NULL,
					fbpriv->info.fix.smem_len,
					(void *)&fbpriv->info.fix.smem_start,
					GFP_KERNEL);
	if (!fbmem_virt) {
		dev_err(dev,
			"altvipfb2: unable to allocate %d Bytes fb memory\n",
			fbpriv->info.fix.smem_len);
		return -ENOMEM;
	}

	fbpriv->info.screen_base = (char *)fbmem_virt;

	retval = fb_alloc_cmap(&fbpriv->info.cmap, PALETTE_SIZE, 0);
	if (retval < 0)
		goto err_dma_free;

	altvipfb2_setup_fb_info(fbpriv);

	altvipfb2_start_hw(base, &fbpriv->info);

	dev_info(dev, "fb%d: %s frame buffer device at 0x%x+0x%x\n",
		 fbpriv->info.node, fbpriv->info.fix.id,
		 (unsigned int)fbpriv->info.fix.smem_start,
		 fbpriv->info.fix.smem_len);

	return register_framebuffer(&fbpriv->info);

err_dma_free:
	fb_dealloc_cmap(&fbpriv->info.cmap);
	dma_free_coherent(NULL, fbpriv->info.fix.smem_len, fbmem_virt,
			  fbpriv->info.fix.smem_start);
	return retval;
}
EXPORT_SYMBOL_GPL(altvipfb2_probe);

int altvipfb2_remove(struct device *dev)
{
	struct altvipfb2_priv *fbpriv = dev_get_drvdata(dev);

	altvipfb2_disable_hw(fbpriv->base);
	dma_free_coherent(NULL, fbpriv->info.fix.smem_len,
			(void *)&fbpriv->info.screen_base,
			fbpriv->info.fix.smem_start);

	return unregister_framebuffer(&fbpriv->info);
}
EXPORT_SYMBOL_GPL(altvipfb2_remove);

MODULE_AUTHOR("Ong Hean Loong <hean.loong.ong@intel.com>");
MODULE_DESCRIPTION("Altera VIP Frame Buffer II driver");
MODULE_LICENSE("GPL v2");
