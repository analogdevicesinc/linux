/*
 * Copyright (C) 2017 Intel Corporation
 *
 * Intel Video and Image Processing(VIP) Frame Buffer II driver
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
 */

#include "altvipfb2.h"
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static int altvipfb2_of_setup(struct altvipfb2_priv *fbdev,
			      struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	int mem_word_width;
	u32 bits_per_color;

	ret = of_property_read_u32(np, "altr,max-width", &fbdev->info.var.xres);
	if (ret) {
		dev_err(&pdev->dev,
			"Missing required parameter 'altr,max-width'");
		return ret;
	}
	fbdev->info.var.xres_virtual = fbdev->info.var.xres,

	ret = of_property_read_u32(np, "altr,max-height",
				   &fbdev->info.var.yres);
	if (ret) {
		dev_err(&pdev->dev,
			"Missing required parameter 'altr,max-height'");
		return ret;
	}
	fbdev->info.var.yres_virtual = fbdev->info.var.yres;

	ret = of_property_read_u32(np, "altr,bits-per-symbol", &bits_per_color);
	if (ret) {
		dev_err(&pdev->dev,
			"Missing required parameter 'altr,bits-per-symbol'");
		return ret;
	}
	if (bits_per_color != 8) {
		dev_err(&pdev->dev,
			"bits-per-color is set to %i. Currently only 8 is supported.",
			bits_per_color);
		return -ENODEV;
	}
	fbdev->info.var.bits_per_pixel = bits_per_color * BYTES_PER_PIXEL;

	ret = of_property_read_u32(np, "altr,mem-port-width", &mem_word_width);
	if (ret) {
		dev_err(&pdev->dev,
			"Missing required parameter 'altr,mem-port-width '");
		return ret;
	}
	if (!(mem_word_width >= 32 && mem_word_width % 32 == 0)) {
		dev_err(&pdev->dev,
			"mem-word-width is set to %i. must be >= 32 and multiple of 32.",
			 mem_word_width);
		return -ENODEV;
	}

	fbdev->info.fix.line_length = (fbdev->info.var.xres *
		(fbdev->info.var.bits_per_pixel >> 3));
	fbdev->info.fix.smem_len =
		fbdev->info.fix.line_length * fbdev->info.var.yres;

	return 0;
}

static int altvipfb2_plat_probe(struct platform_device *pdev)
{
	int retval;

	struct device *dev = &pdev->dev;
	struct resource *reg_res;
	struct altvipfb2_priv *fbdev;

	fbdev = devm_kzalloc(dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)
		return -ENOMEM;

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!reg_res)
		return -ENODEV;

	fbdev->base = devm_ioremap_resource(dev, reg_res);
	if (IS_ERR(fbdev->base)) {
		dev_err(dev, "devm_ioremap_resource failed\n");
		retval = PTR_ERR(fbdev->base);
		return -ENOMEM;
	}

	altvipfb2_of_setup(fbdev, pdev);

	platform_set_drvdata(pdev, fbdev);

	return altvipfb2_probe(dev, fbdev->base);
}

static int altvipfb2_plat_remove(struct platform_device *pdev)
{
	return altvipfb2_remove(&pdev->dev);
}

static const struct of_device_id altvipfb2_match[] = {
	{ .compatible = "altr,vip-frame-buffer-2.0" },
	{},
};
MODULE_DEVICE_TABLE(of, altvipfb2_match);

static struct platform_driver altvipfb2_driver = {
	.probe = altvipfb2_plat_probe,
	.remove = altvipfb2_plat_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = altvipfb2_match,
	},
};

module_platform_driver(altvipfb2_driver);
