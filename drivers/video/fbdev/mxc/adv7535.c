/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <video/mxc_edid.h>

#include "mipi_dsi.h"

/* main registers addr*/
#define ADV7535_CHIP_REVISION		0x0
#define ADV7535_DSI_CEC_ADDR		0xE1

#define TEST_PATTERN_ENABLE    0

struct adv7535_info {
	int rev;
	struct device *dev;
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_dsi_cec;
	struct fb_videomode *fb_vmode;
	unsigned int bpp;
};

static int adv7535_probe(struct i2c_client *client);
static void adv7535_remove(struct i2c_client *client);
static int adv7535_detect(struct i2c_client *client,
			  struct i2c_board_info *info);

static const struct i2c_device_id adv7535_id[] = {
	{"adv7535", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, adv7535_id);

static const struct of_device_id adv7535_dt_ids[] = {
	{ .compatible = "adi,adv7535", .data = NULL, },
	{ /* sentinel */ }
};

static struct i2c_driver adv7535_i2c_driver = {
	.driver = {
		.name = "adv7535",
		.owner = THIS_MODULE,
		.of_match_table = adv7535_dt_ids,
	},
	.probe  = adv7535_probe,
	.remove = adv7535_remove,
	.id_table = adv7535_id,
	.detect = adv7535_detect,
};

#define adv7535_write_reg(reg, val) {				\
	int err;						\
								\
	err = i2c_smbus_write_byte_data(client, reg, val);	\
	if (err < 0) {						\
		dev_err(&client->dev,				\
			"%s:write reg error:reg=%2x, val=%2x\n",\
			__func__, reg, val);			\
		return err;					\
	}							\
}								\

static int adv7535_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	s8 ret;
	u8 chip_rev;
	struct i2c_adapter *adapter = client->adapter;

	/* check i2c functionality */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C |
					     I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* i2c detection */
	ret = i2c_smbus_read_byte_data(client,
				       ADV7535_CHIP_REVISION);
	if (ret < 0) {
		dev_err(&adapter->dev, "ADV7535 not found\n");
		return -ENODEV;
	}

	chip_rev = ret;
	if (chip_rev != 0x14) {
		dev_info(&adapter->dev, "Unsupported chip id: 0x%2x\n",
			 chip_rev);
		return -ENODEV;
	}

	dev_info(&adapter->dev, "chip_rev = 0x%x\n", ret);
	return 0;
}

static int adv7535_setup_cfg(struct adv7535_info *info)
{
	struct i2c_client *client = NULL;

	client = info->i2c_main;
	adv7535_write_reg(0xd6, 0x48); /* HPD Override */
	adv7535_write_reg(0x41, 0x10); /* Power Up   */

	adv7535_write_reg(0x16, 0x20); /* Fixed Init */
	adv7535_write_reg(0x9A, 0xE0);
	adv7535_write_reg(0xBA, 0x70);
	adv7535_write_reg(0xDE, 0x82);
	adv7535_write_reg(0xE4, 0x40);
	adv7535_write_reg(0xE5, 0x80);

	client = info->i2c_dsi_cec;
	adv7535_write_reg(0x15, 0xD0);
	adv7535_write_reg(0x17, 0xD0);
	adv7535_write_reg(0x24, 0x20);
	adv7535_write_reg(0x57, 0x11);

	return 0;
}

static int adv7535_vmode_cfg(struct adv7535_info *info)
{
	struct i2c_client *client = NULL;
	struct fb_videomode *fb_vmode = info->fb_vmode;
	u32 line_length, frame_height;
	u8 low, high;

	line_length  = fb_vmode->xres + fb_vmode->left_margin +
		       fb_vmode->right_margin + fb_vmode->hsync_len;
	frame_height = fb_vmode->yres + fb_vmode->upper_margin +
		       fb_vmode->lower_margin + fb_vmode->vsync_len;

	client = info->i2c_dsi_cec;
#ifdef CONFIG_FB_IMX64
	adv7535_write_reg(0x1C, 0x40); /* 4 Data Lanes */
#else
	adv7535_write_reg(0x1C, 0x20); /* 2 Data Lanes */
#endif

#if TEST_PATTERN_ENABLE
	adv7535_write_reg(0x55, 0x80);
	adv7535_write_reg(0x16, 0x1C);
#else
	adv7535_write_reg(0x16, 0x00); /* Pixel Clock  */
#endif
	adv7535_write_reg(0x27, 0xCB); /* INT_TIMING_GEN */

	/* video mode settings */
	low  = (line_length << 4);
	high = (line_length >> 4);
	adv7535_write_reg(0x28, high); /* Total Line Length */
	adv7535_write_reg(0x29, low);

	low  = (fb_vmode->hsync_len << 4);
	high = (fb_vmode->hsync_len >> 4);
	adv7535_write_reg(0x2A, high); /* Hsync Active Width */
	adv7535_write_reg(0x2B, low);

	low  = (fb_vmode->right_margin << 4);
	high = (fb_vmode->right_margin >> 4);
	adv7535_write_reg(0x2C, high); /* Horizontal FP Width */
	adv7535_write_reg(0x2D, low);

	low  = (fb_vmode->left_margin << 4);
	high = (fb_vmode->left_margin >> 4);
	adv7535_write_reg(0x2E, high); /* Horizontal BP Width */
	adv7535_write_reg(0x2F, low);

	low  = (frame_height << 4);
	high = (frame_height >> 4);
	adv7535_write_reg(0x30, high); /* Total Frame Height */
	adv7535_write_reg(0x31, low);

	low  = (fb_vmode->vsync_len << 4);
	high = (fb_vmode->vsync_len >> 4);
	adv7535_write_reg(0x32, high); /* Vsync Active Height */
	adv7535_write_reg(0x33, low);

	low  = (fb_vmode->lower_margin << 4);
	high = (fb_vmode->lower_margin >> 4);
	adv7535_write_reg(0x34, high); /* Vertical FP Height */
	adv7535_write_reg(0x35, low);

	low  = (fb_vmode->upper_margin << 4);
	high = (fb_vmode->upper_margin >> 4);
	adv7535_write_reg(0x36, high); /* Vertical BP Height */
	adv7535_write_reg(0x37, low);

	/* Reset Internal Timing Generator */
	adv7535_write_reg(0x27, 0xCB);
	adv7535_write_reg(0x27, 0x8B);
	adv7535_write_reg(0x27, 0xCB);

	client = info->i2c_main;
	adv7535_write_reg(0xAF, 0x16); /* HDMI Output */
	adv7535_write_reg(0x55, 0x10); /* AVI Info-frame */
#ifdef CONFIG_FB_IMX64
	adv7535_write_reg(0x56, 0x28); /* 16:9 */
#else
	adv7535_write_reg(0x56, 0x18);
#endif
	adv7535_write_reg(0x40, 0x80); /* GCP Enable */
	adv7535_write_reg(0x4C, 0x04); /* 24bpp */
	adv7535_write_reg(0x49, 0x00);

#ifdef CONFIG_FB_IMX64
	/* low refresh rate */
	if (fb_vmode->refresh < 50)
		adv7535_write_reg(0x4A, 0x8C);
#else
	adv7535_write_reg(0x17, 0x60); /* VS & HS Low Polarity, DE disabled */
#endif

	/*TODO Audio Setup */

	client = info->i2c_dsi_cec;
	adv7535_write_reg(0xBE, 0x3D); /* CEC Power Mode */

	adv7535_write_reg(0x03, 0x89);

	return 0;
}

static int adv7535_probe(struct i2c_client *client)
{
	u32 vmode_index;
	int ret = 0, addr;
	struct adv7535_info *info;
	struct device *dev = &client->dev;
	struct device_node *endpoint = NULL;

	pr_info("adv7535 probing phase\n");
	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);

	if (!info)
		return -ENOMEM;

	info->dev = &client->dev;
	info->i2c_main = client;
	i2c_set_clientdata(client, info);

	dev_info(dev, "main addr = 0x%x\n", info->i2c_main->addr);
	/* get dsi source endpoint */
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "DSI source endpoint not exsit\n");
		ret = -ENODEV;
		goto err1;
	}

	info->fb_vmode = devm_kzalloc(dev, sizeof(struct fb_videomode),
				      GFP_KERNEL);
	if (!info->fb_vmode) {
		ret = -ENOMEM;
		goto err1;
	}

	ret = of_property_read_u32(dev->of_node, "video-mode", &vmode_index);
	if (ret < 0) {
		dev_err(dev, "failed to get required video mode\n");
		goto err2;
	}
	if (vmode_index >= ARRAY_SIZE(mxc_cea_mode))
		goto err2;

	memcpy(info->fb_vmode, &mxc_cea_mode[vmode_index],
	       sizeof(struct fb_videomode));

	ret = of_property_read_u32(dev->of_node, "bpp", &info->bpp);
	if (ret < 0) {
		dev_err(dev, "failed to get bpp\n");
		goto err2;
	}

	if ((ret = adv7535_detect(client, NULL)))
		goto err2;

	addr = i2c_smbus_read_byte_data(client,
					ADV7535_DSI_CEC_ADDR);
	if (addr < 0) {
		dev_err(dev, "Cannot get dsi_cec addr\n");
		ret = addr;
		goto err2;
	}

	dev_info(dev, "dsi cec addr = 0x%x\n", addr);
	info->i2c_dsi_cec = i2c_new_dummy_device(client->adapter, addr >> 1);
	if (!info->i2c_dsi_cec) {
		dev_err(dev, "Failed to allocate I2C device for dsi_cec\n");
		ret = -ENOMEM;
		goto err2;
	}

	i2c_set_clientdata(info->i2c_dsi_cec, info);

	/*TODO interrupts */

	if ((ret = adv7535_setup_cfg(info)))
		goto err3;

	if ((ret = adv7535_vmode_cfg(info)))
		goto err3;

#ifndef CONFIG_FB_IMX64
	pm_runtime_enable(info->dev);
#endif

	dev_info(dev, "adv7535 probe finished\n");

	return 0;
err3:
	i2c_unregister_device(info->i2c_dsi_cec);
err2:
	devm_kfree(dev, info->fb_vmode);
err1:
	devm_kfree(dev, info);

	return ret;
}

static void adv7535_remove(struct i2c_client *client)
{
	struct adv7535_info *info;

	info = i2c_get_clientdata(client);
	i2c_set_clientdata(client, NULL);

#ifndef CONFIG_FB_IMX64
	pm_runtime_disable(info->dev);
#endif

	i2c_unregister_device(info->i2c_dsi_cec);

	kfree(info->fb_vmode);
	kfree(info);
}

static __init int adv7535_init(void)
{
	u8 err = 0;


	err = i2c_add_driver(&adv7535_i2c_driver);
	if (err != 0)
		pr_err("%s: i2c driver register failed, error = %d\n",
			__func__, err);

	pr_debug("%s (ret=%d)\n", __func__, err);
	return err;
}

static void __exit adv7535_exit(void)
{
	i2c_del_driver(&adv7535_i2c_driver);
}

module_init(adv7535_init);
module_exit(adv7535_exit);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("ADV7535 video encoder driver");
MODULE_LICENSE("GPL");
