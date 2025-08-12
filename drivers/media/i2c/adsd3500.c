// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Analog Devices ADSD3500 chip.
 *
 * Copyright (C) 2022 Analog Devices, All Rights Reserved.
 *
 */

#include "adsd3500_regs.h"
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pwm.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/sched/signal.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

static bool fw_load = false;
module_param(fw_load, bool, 0644);
MODULE_PARM_DESC(fw_load, "Boolean enabling/disbaling firmware loading by driver");

struct adsd3500_mode_info {
	uint32_t width;
	uint32_t height;
	uint32_t pixel_rate;
	uint32_t code;
	uint32_t link_freq_idx;
};

struct adsd3500_config_info {
	uint8_t nr_depth_bits;
	uint8_t nr_ab_bits;
	uint8_t nr_confidence_bits;
	uint8_t nr_mipi_lanes;
	bool use_vc;
};

struct adsd3500 {
	struct regmap *regmap;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;

	const struct adsd3500_mode_info *current_mode;
	struct adsd3500_config_info current_config;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	/* custom controls */
	struct v4l2_ctrl *operating_mode;
	struct v4l2_ctrl *set_chip_config;
	struct v4l2_ctrl *depth_bits;
	struct v4l2_ctrl *ab_bits;
	struct v4l2_ctrl *confidence_bits;
	struct v4l2_ctrl *ab_avg;
	struct v4l2_ctrl *depth_en;

	struct mutex lock;
	bool streaming;

	const struct firmware  *main_fw;
	struct gpio_desc *rst_gpio;
	struct gpio_desc *irq_gpio;
	int    irq;
	struct dentry *debugfs;
	struct task_struct *task;
	int signalnum;
};

static inline struct adsd3500 *to_adsd3500(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adsd3500, sd);
}

#define V4L2_CID_ADSD3500_OPERATING_MODE  (V4L2_CID_USER_ADSD_BASE + 0)
#define V4L2_CID_ADSD3500_CHIP_CONFIG (V4L2_CID_USER_ADSD_BASE + 1)
#define V4L2_CID_ADSD3500_DEPTH_BITS (V4L2_CID_USER_ADSD_BASE + 2)
#define V4L2_CID_ADSD3500_AB_BITS (V4L2_CID_USER_ADSD_BASE + 3)
#define V4L2_CID_ADSD3500_CONFIDENCE_BITS (V4L2_CID_USER_ADSD_BASE + 4)
#define V4L2_CID_ADSD3500_AB_AVG (V4L2_CID_USER_ADSD_BASE + 5)
#define V4L2_CID_ADSD3500_DEPTH_EN (V4L2_CID_USER_ADSD_BASE + 6)

static const struct reg_sequence adsd3500_powerup_setting[] = {
};

static const struct reg_sequence adsd3500_powerdown_setting[] = {
};

static const struct reg_sequence adsd3500_standby_setting[] = {
};

static const s64 link_freq_tbl[] = {
	732000000
};

static int debug_open(struct inode *inode, struct file *file)
{
	struct adsd3500 *adsd3500;

	if (inode->i_private)
		file->private_data = inode->i_private;

	adsd3500 = (struct adsd3500 *) file->private_data;

	dev_dbg(adsd3500->dev, "Entered debugfs file open\n");

	return 0;
}

static int debug_release(struct inode *inode, struct file *file)
{
	struct adsd3500 *adsd3500;
	struct task_struct *release_task = get_current();

	if (inode->i_private)
		file->private_data = inode->i_private;

	adsd3500 = (struct adsd3500 *) file->private_data;

	dev_dbg(adsd3500->dev, "Entered debugfs file close\n");
	if(release_task == adsd3500->task) {
		adsd3500->task = NULL;
	}

	return 0;
}

ssize_t debug_read(struct file *file, char __user *buff, size_t count, loff_t *offset){

	struct adsd3500 *adsd3500 = file->private_data;
	unsigned int read_val;
	unsigned int len;
	int ret;
	char data[16];

	dev_dbg(adsd3500->dev, "Entered debugfs file read\n");
	ret = regmap_read(adsd3500->regmap, GET_IMAGER_STATUS_CMD, &read_val);
	if (ret < 0) {
		dev_err(adsd3500->dev, "Read of get status cmd failed.\n");
		len = snprintf(data, sizeof(data), "Read failed\n");
	}
	else{
		dev_dbg(adsd3500->dev, "Read the error status: %.4X\n", read_val);
		len = snprintf(data, sizeof(data), "0x%.4X\n",read_val);
	}
	return simple_read_from_buffer(buff, count, offset, data, len);

}

ssize_t debug_write(struct file *file, const char __user *buff, size_t count, loff_t *offset){

	struct adsd3500 *adsd3500 = file->private_data;

	dev_dbg(adsd3500->dev, "Entered debugfs file write\n");

	return count;
}

static long debug_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adsd3500 *adsd3500;

	adsd3500 = (struct adsd3500 *) file->private_data;

	dev_dbg(adsd3500->dev, "Entered debugfs ioctl\n");
	if (cmd == USER_TASK) {
		dev_dbg(adsd3500->dev, "Registered user task\n");
		adsd3500->task = get_current();
		adsd3500->signalnum = SIGETX;
	}

	return 0;
}

static const struct file_operations adsd3500_debug_fops = {
	.owner 	= THIS_MODULE,
	.open   = debug_open,
	.read 	= debug_read,
	.write  = debug_write,
	.unlocked_ioctl = debug_ioctl,
	.release= debug_release,
};

static irqreturn_t adsd3500_irq_handler(int irq,void *priv)
{

	struct adsd3500 *adsd3500 = (struct adsd3500 *) priv;

	dev_dbg(adsd3500-> dev, "Entered ADSD3500 IRQ handler\n");

	if (adsd3500->task != NULL) {
		dev_dbg(adsd3500->dev, "Sending signal to app\n");
		if(send_sig_info(SIGETX, SEND_SIG_PRIV, adsd3500->task) < 0) {
			dev_err(adsd3500->dev, "Unable to send signal\n");
		}
	}

	return IRQ_HANDLED;

}

/* Elements of the structure must be ordered ascending by width & height */
static const struct adsd3500_mode_info adsd3500_mode_info_data[] = {

	/* --- RAW 8 MEDIA_BUS_FMT_SBGGR8_1X8 --- */

	{	/* RAW8 8BPP ADSD3100 QMP RESOLUTION */
		.width = 512,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 16BPP ADSD3100 QMP RESOLUTION */
		.width = 1024,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 20BPP ADSD3100 QMP RESOLUTION */
		.width = 1280,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 24BPP ADSD3100 QMP RESOLUTION */
		.width = 1536,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 28BPP ADSD3100 QMP RESOLUTION */
		.width = 1792,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 32BPP ADSD3100 QMP RESOLUTION */
		.width = 2048,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 36BPP ADSD3100 QMP RESOLUTION */
		.width = 2304,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 40BPP ADSD3100 QMP RESOLUTION */
		.width = 2560,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{   /* RAW8 8BPP ADSD3030 VGA RESOLUTION */
		.width = 512,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 16BPP ADSD3030 VGA RESOLUTION */
		.width = 1024,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 24BPP ADSD3030 VGA RESOLUTION */
		.width = 1536,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 32BPP ADSD3030 VGA RESOLUTION */
		.width = 2048,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 40BPP ADSD3030 VGA RESOLUTION */
		.width = 2560,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{   /* RAW8 8BPP ADSD3030 QVGA RESOLUTION */
		.width =  256,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 16BPP ADSD3030 QVGA RESOLUTION */
		.width =  512,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 24BPP ADSD3030 QVGA RESOLUTION */
		.width =  768,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 32BPP ADSD3030 QVGA RESOLUTION */
		.width = 1024,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW8 40BPP ADSD3030 QVGA RESOLUTION */
		.width = 1280,
		.height = 320,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 3 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x3.25)*/
		.width = 2048,
		.height = 3328,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 3 phase + 16BPP * 1 AB Superframe (512x2 512x3.25) */
		.width = 1024,
		.height = 1664,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	/* RAW8 ADSD3030 VGA 12BPP * 3 phase + 16BPP * 1 AB  SuperFrame (512x2 640x3.25)*/
		.width = 1024,
		.height = 2080,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 3 phase + 16BPP * 1 AB  SuperFrame (256x2 320x3.25)*/
		.width = 512,
		.height = 1040,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x2.5)*/
		.width = 2048,
		.height = 2560,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	/* RAW8 ADSD3100 QMP 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 512x2.5)*/
		.width = 1024,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 640x2.5)*/
		.width = 1024,
		.height = 1600,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 2 phase + 16BPP * 1 AB SuperFrame (256x2 320x2.5)*/
		.width = 512,
		.height = 800,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x4)*/
		.width = 2048,
		.height = 4096,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (512x2 512x4)*/
		.width = 1024,
		.height = 2048,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (512x2 640x4)*/
		.width = 1024,
		.height = 2560,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 3 phase + 16BPP * 1 AB SuperFrame (256x2 320x4)*/
		.width = 512,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (1024x2 1024x3)*/
		.width = 2048,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 512x3)*/
		.width = 1024,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (512x2 640x3)*/
		.width = 1024,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 2 phase + 16BPP * 1 AB SuperFrame (256x2 320x3)*/
		.width = 512,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 3 phase + 16BPP * 3 AB Interleaved (1024x3.5 1024x3) */
		.width = 3584,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 3 phase + 16BPP * 3 AB Interleaved (512x3.5 512x3) */
		.width = 1792,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 3 phase + 16BPP * 3 AB Interleaved (512x3.5 640x3) */
		.width = 1792,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 3 phase + 16BPP * 3 AB Interleaved (256x3.5 320x3) */
		.width = 896,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 12BPP * 2 phase + 16BPP * 2 AB Interleaved (1024x3.5 1024x2) */
		.width = 3584,
		.height = 2048,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 12BPP * 2 phase + 16BPP * 2 AB Interleaved (512x3.5 512x2) */
		.width = 1792,
		.height = 1024,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 12BPP * 2 phase + 16BPP * 2 AB Interleaved (512x3.5 640x2) */
		.width = 1792,
		.height = 1280,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 12BPP * 2 phase + 16BPP * 2 AB Interleaved (256x3.5 320x2) */
		.width = 896,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{	/* RAW8 ADSD3100 MP 16BPP * 3 phase + 16BPP * 3 AB Interleaved (1024x4 1024x3) */
		.width = 4096,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3100 QMP 16BPP * 3 phase + 16BPP * 3 AB Interleaved (512x4 512x3) */
		.width = 2048,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 VGA 16BPP * 3 phase + 16BPP * 3 AB Interleaved (512x4 640x3) */
		.width = 2048,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	/* RAW8 ADSD3030 QVGA 16BPP * 3 phase + 16BPP * 3 AB Interleaved (256x4 320x3) */
		.width = 1024,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 8 MEDIA_BUS_FMT_SBGGR8_1X8  Miscellaneous --- */

	{ 	//RAW8 MP 16BPP * 3 phase + 16BPP * 3 AB
		.width = 2048,
		.height = 6144,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QMP 16BPP * 3 phase + 16BPP * 3 AB
		.width = 3072,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 VGA 16BPP * 3 phase + 16BPP * 3 AB
		.width = 3072,
		.height = 3840,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QVGA 16BPP * 3 phase + 16BPP * 1 AB
		.width = 512,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW8 QVGA 16BPP * 3 phase + 16BPP * 3 AB
		.width = 1536,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 12 MEDIA_BUS_FMT_SBGGR12_1X12 --- */

	{   /* RAW12 12BPP ADSD3100 MP - 9 subframes - (1024x2 1024x4.5) */
		.width = 2048,
		.height = 4608,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3100 QMP - 9 subframes - (512x2 512x4.5) */
		.width = 1024,
		.height = 2304,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 VGA - 9 subframes - (512x2 640x4.5) */
		.width = 1024,
		.height = 2880,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 QVGA - 9 subframes - (256x2 320x4.5) */
		.width = 512,
		.height = 1440,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	{   /* RAW12 12BPP ADSD3100 MP - 6 subframes - (1024x2 1024x3) */
		.width = 2048,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3100 QMP - 6 subframes - (512x2 512x3) */
		.width = 1024,
		.height = 1536,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 VGA - 6 subframes - (512x2 640x3) */
		.width = 1024,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 QVGA - 6 subframes - (256x2 320x3) */
		.width = 512,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 12 MEDIA_BUS_FMT_SBGGR12_1X12  Miscellaneous --- */

	{ 	//RAW12 ADSD3100 QMP 12BPP Depth only/ AB Only
		.width = 512,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 ADSD3030 VGA 12BPP Depth Only/ AB Only
		.width = 512,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP AB test
		.width = 2048,
		.height = 512,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	//RAW12 1 Phase / Frame
		.width = 1024,
		.height = 1024,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{	//RAW12 12BPP * 3 phase
		.width = 1024,
		.height = 3072,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 3 phase
		.width = 1024,
		.height = 4096,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 3 phase + 12BPP * 3 AB
		.width = 2048,
		.height = 5376,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{ 	//RAW12 12BPP * 2 phase + 12BPP * 2 AB
		.width = 2048,
		.height = 3584,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

	/* --- RAW 12 MEDIA_BUS_FMT_Y12_1X12 --- */

	{   /* RAW12 12BPP ADSD3030 VGA ---- */
		.width = 512,
		.height = 640,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 256x320x6 - 6 subframes - (256x4 320x1.5) */
		.width = 1024,
		.height = 480,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 256x320x9 - 9 subframes - (256x4 320x2.25) */
		.width = 1024,
		.height = 720,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 512x640x3 - 3 subframes - (512x2 640x1.5) */
		.width = 1024,
		.height = 960,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 512x640x6 - 6 subframes - (512x2 640x3) */
		.width = 1024,
		.height = 1920,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},
	{   /* RAW12 12BPP ADSD3030 512x640x9 - 9 subframes - (512x2 640x4.5) */
		.width = 1024,
		.height = 2880,
		.pixel_rate = 488000000,
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.link_freq_idx = 0 /* an index in link_freq_tbl[] */
	},

};

static bool adsd3500_regmap_accessible_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case GET_CHIP_ID_CMD:
		case GET_IMAGER_MODE_CMD:
		case GET_IMAGER_AB_INVLD_TRSHLD:
		case GET_IMAGER_CONFIDENCE_TRSHLD:
		case GET_IMAGER_JBLF_STATE:
		case GET_IMAGER_JBLF_FILT_SIZE:
		case GET_FRAMERATE_CMD:
		case GET_IMAGER_STATUS_CMD:
			return 1;
		default:
			return 0;
	}
}

static const struct regmap_config adsd3500_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.cache_type = REGCACHE_NONE,
	.readable_reg = adsd3500_regmap_accessible_reg,
};

static int adsd3500_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct v4l2_ctrl *operating_mode = adsd3500->operating_mode;
	struct v4l2_ctrl *ab_avg = adsd3500->ab_avg;
	struct v4l2_ctrl *depth_en = adsd3500->depth_en;
	struct adsd3500_config_info config = adsd3500->current_config;
	unsigned int write_cmd, write_val = 0;
	unsigned int read_val;
	int ret;

	dev_dbg(adsd3500->dev, "Entered addicmos_power_on\n");

	ret = regmap_read(adsd3500->regmap, GET_CHIP_ID_CMD, &read_val);
	if (ret < 0) {
		dev_err(adsd3500->dev, "Read of Chip ID register failed.\n");
	}

	if (read_val != ADSD3500_CHIP_ID) {
		dev_err(adsd3500->dev, "Chip ID: %.4X is wrong.\n", read_val);
	}
	dev_dbg(adsd3500->dev, "Read Chip ID: %.4X\n", read_val);

	write_cmd = SET_IMAGER_MODE_CMD | SET_IMAGER_MODE(operating_mode->val);

	write_val |= SET_IMAGER_MODE_DEPTH_EN(config.nr_depth_bits ? depth_en->val : 0);
	write_val |= SET_IMAGER_MODE_DEPTH_BITS(config.nr_depth_bits ? 6 - config.nr_depth_bits : 7);
	
	write_val |= SET_IMAGER_MODE_AB_EN(config.nr_ab_bits ? 1 : 0);
	write_val |= SET_IMAGER_MODE_AB_BITS(config.nr_ab_bits ? 6 - config.nr_ab_bits : 0);

	write_val |= SET_IMAGER_MODE_CONF_BITS(config.nr_confidence_bits);

	write_val |= SET_IMAGER_MODE_VC_EN(!config.use_vc);
	write_val |= SET_IMAGER_MODE_AB_AVG_EN(ab_avg->val);
	write_val |= SET_IMAGER_MODE_MIPI_LANES_NR(config.nr_mipi_lanes);

	ret = regmap_write(adsd3500->regmap, write_cmd, write_val);
	if (ret) {
		dev_err(adsd3500->dev, "Could not set mode register\n");
		return ret;
	}

	return 0;
}

static int adsd3500_power_off(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int adsd3500_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	unsigned int read_val;
	int ret;

	reg->size = 2;
	ret = regmap_read(adsd3500->regmap, reg->reg, &read_val);
	reg->val = read_val;

	return ret;
}

static int adsd3500_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);

	return regmap_write(adsd3500->regmap, reg->reg, reg->val);
}
#endif

static int adsd3500_s_power(struct v4l2_subdev *sd, int on)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);

	dev_dbg(adsd3500->dev, "%s: %d\n", __func__, on);
	return 0;
}

static int adsd3500_bpp_config(struct adsd3500 *priv,
				    struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_ADSD3500_DEPTH_BITS:
		priv->current_config.nr_depth_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_AB_BITS:
		priv->current_config.nr_ab_bits = ctrl->val;
		break;
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		priv->current_config.nr_confidence_bits = ctrl->val;
		break;
	default:
		break;
	}

	return 0;
}

static int adsd3500_chip_config(struct adsd3500 *priv,
				    struct v4l2_ctrl *ctrl)
{
	struct device *dev = priv->dev;
	struct i2c_client *client = to_i2c_client(dev);
	uint16_t pld_size;
	uint8_t r_w, *data;
	int ret;

	r_w = *ctrl->p_new.p_u8;
	pld_size = (uint16_t)(*(ctrl->p_new.p_u8 + 1) << 8 | *(ctrl->p_new.p_u8 + 2));
	data = ctrl->p_new.p_u8 + 3;

	dev_dbg(dev, "Entered adsd3500_chip_config. R/W: %d, PLD_SIZE: %d\n", r_w, pld_size);

	if ((pld_size > 4096) || (pld_size < 2))
		return -EINVAL;
	
	if (r_w > 1)
		return -EINVAL;

	if (r_w) {
		ret = i2c_master_send(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Write burst transfer failed\n");
			return -EIO;
		}
	} else {
		ret = i2c_master_recv(client, data, pld_size);
		if (ret < 0) {
			dev_warn(dev, "Read burst transfer failed\n");
			return -EIO;
		}
	}
	memset(ctrl->p_new.p_u8, 0xFF, 1);
	return 0;
}

static int adsd3500_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct adsd3500 *adsd3500 = container_of(ctrl->handler,
						 struct adsd3500, ctrls);
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_ADSD3500_OPERATING_MODE:
	case V4L2_CID_ADSD3500_AB_AVG:
	case V4L2_CID_ADSD3500_DEPTH_EN:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		break;
	case V4L2_CID_ADSD3500_CHIP_CONFIG:
		ret = adsd3500_chip_config(adsd3500, ctrl);
		break;
	case V4L2_CID_ADSD3500_DEPTH_BITS:
	case V4L2_CID_ADSD3500_AB_BITS:
	case V4L2_CID_ADSD3500_CONFIDENCE_BITS:
		ret = adsd3500_bpp_config(adsd3500, ctrl);
		break;
	default:
		dev_err(adsd3500->dev, "%s > Unhandled: %x  param=%x\n",
			__func__, ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const s64 nr_bits_qmenu[] = {
	0, 4, 8, 10, 12, 14, 16
};

static const struct v4l2_ctrl_ops adsd3500_ctrl_ops = {
	.s_ctrl = adsd3500_s_ctrl,
};

static const struct v4l2_ctrl_config adsd3500_ctrl_operating_mode = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_OPERATING_MODE,
	.name		= "Operating Mode",
	.type		= V4L2_CTRL_TYPE_INTEGER,
	.def		= 0,
	.min		= 0,
	.max		= 10,
	.step		= 1
};

static const struct v4l2_ctrl_config adsd3500_ctrl_chip_config = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_CHIP_CONFIG,
	.name		= "Chip Config",
	.type		= V4L2_CTRL_TYPE_U8,
	.def		= 0x00,
	.min		= 0x00,
	.max		= 0xFF,
	.step		= 1,
	.dims		= { 4099 }
};

static const struct v4l2_ctrl_config adsd3500_ctrl_depth_bits = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_DEPTH_BITS,
	.name		= "Phase / Depth Bits",
	.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
	.def		= 0,
	.min		= 0,
	.max		= 6,
	.menu_skip_mask = 0x02,
	.qmenu_int	= nr_bits_qmenu,
};

static const struct v4l2_ctrl_config adsd3500_ab_depth_bits = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_AB_BITS,
	.name		= "AB Bits",
	.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
	.def		= 0,
	.min		= 0,
	.max		= 6,
	.menu_skip_mask = 0x02,
	.qmenu_int	= nr_bits_qmenu,
};

static const struct v4l2_ctrl_config adsd3500_confidence_bits = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_CONFIDENCE_BITS,
	.name		= "Confidence Bits",
	.type		= V4L2_CTRL_TYPE_INTEGER_MENU,
	.def		= 0,
	.min		= 0,
	.max		= 2,
	.qmenu_int	= nr_bits_qmenu,
};

static const struct v4l2_ctrl_config adsd3500_ab_avg = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_AB_AVG,
	.name		= "AB Averaging",
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.def		= 1,
	.min		= 0,
	.max		= 1,
	.step		= 1,
};

static const struct v4l2_ctrl_config adsd3500_depth_en = {
	.ops		= &adsd3500_ctrl_ops,
	.id			= V4L2_CID_ADSD3500_DEPTH_EN,
	.name		= "Depth enable",
	.type		= V4L2_CTRL_TYPE_BOOLEAN,
	.def		= 1,
	.min		= 0,
	.max		= 1,
	.step		= 1,
};

static int adsd3500_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;

	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_SBGGR8_1X8;
		break;
	case 1:
		code->code = MEDIA_BUS_FMT_SBGGR12_1X12;
		break;
	case 2:
		code->code = MEDIA_BUS_FMT_SBGGR16_1X16;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int adsd3500_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	int i, j = 0;

	if (fse->index >= ARRAY_SIZE(adsd3500_mode_info_data))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(adsd3500_mode_info_data); i++)
	{
		if(adsd3500_mode_info_data[i].code == fse->code)
			j++;
		if (j > fse->index)
			break;
	}

	if (i < ARRAY_SIZE(adsd3500_mode_info_data)) {
		fse->min_width = adsd3500_mode_info_data[i].width;
		fse->max_width = adsd3500_mode_info_data[i].width;
		fse->min_height = adsd3500_mode_info_data[i].height;
		fse->max_height = adsd3500_mode_info_data[i].height;
	} else {
		return -EINVAL;
	}

	return 0;
}

static struct v4l2_mbus_framefmt *
adsd3500_get_pad_format(struct adsd3500 *adsd3500,
			struct v4l2_subdev_pad_config *cfg, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&adsd3500->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &adsd3500->fmt;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int adsd3500_get_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct v4l2_mbus_framefmt *pad_format;

	pad_format = adsd3500_get_pad_format(adsd3500, cfg, format->pad,
					     format->which);
	if (IS_ERR(pad_format))
		return PTR_ERR(pad_format);

	format->format = *pad_format;

	return 0;
}

static struct v4l2_rect *
adsd3500_get_pad_crop(struct adsd3500 *adsd3500,
		      struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&adsd3500->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &adsd3500->crop;
	default:
		return ERR_PTR(-EINVAL);
	}
}

static int adsd3500_set_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct v4l2_mbus_framefmt *framefmt;
	struct v4l2_rect *crop;
	const struct adsd3500_mode_info *new_mode;
	int ret;

	dev_dbg(adsd3500->dev, "set_fmt: %x %dx%d %d\n",
		format->format.code, format->format.width,
		format->format.height, format->which);

	mutex_lock(&adsd3500->lock);

	if (format->pad != 0)
		return -EINVAL;

	crop = adsd3500_get_pad_crop(adsd3500, cfg, format->pad,
				     format->which);
	if (IS_ERR(crop))
		return PTR_ERR(crop);

	new_mode = v4l2_find_nearest_size(adsd3500_mode_info_data,
					  ARRAY_SIZE(adsd3500_mode_info_data),
					  width, height, format->format.width,
					  format->format.height);
	crop->width = new_mode->width;
	crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(adsd3500->pixel_rate,
					     new_mode->pixel_rate);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(adsd3500->link_freq,
				       new_mode->link_freq_idx);
		if (ret < 0)
			return ret;

		adsd3500->current_mode = new_mode;
	}

	framefmt = adsd3500_get_pad_format(adsd3500, cfg, format->pad,
					   format->which);
	if (IS_ERR(framefmt))
		return PTR_ERR(framefmt);

	framefmt->width = crop->width;
	framefmt->height = crop->height;
	framefmt->code = format->format.code;
	framefmt->field = V4L2_FIELD_NONE;
	framefmt->colorspace = V4L2_COLORSPACE_RAW;

	format->format = *framefmt;
	adsd3500->fmt = *framefmt;

	mutex_unlock(&adsd3500->lock);

	return 0;
}

static int adsd3500_entity_init_cfg(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = adsd3500_mode_info_data[0].width;
	fmt.format.height = adsd3500_mode_info_data[0].height;
	fmt.format.code = adsd3500_mode_info_data[0].code;

	return adsd3500_set_format(subdev, cfg, &fmt);
}

static int adsd3500_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct v4l2_rect *crop;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	crop = adsd3500_get_pad_crop(adsd3500, cfg, sel->pad, sel->which);
	if (IS_ERR(crop))
		return PTR_ERR(crop);

	sel->r = *crop;

	return 0;
}

static int adsd3500_start_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	ret = regmap_write(adsd3500->regmap, STREAM_ON_CMD, STREAM_ON_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-ON command failed.\n");

	return ret;
}

static int adsd3500_stop_streaming(struct adsd3500 *adsd3500)
{
	int ret;

	ret = regmap_write(adsd3500->regmap, STREAM_OFF_CMD, STREAM_OFF_VAL);
	if (ret < 0)
		dev_err(adsd3500->dev, "Write of STREAM-OFF command failed.\n");

	return ret;
}

static int adsd3500_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct adsd3500 *adsd3500 = to_adsd3500(subdev);
	int ret = 0;

	dev_dbg(adsd3500->dev, "s_stream: %d\n", enable);

	mutex_lock(&adsd3500->lock);
	if (adsd3500->streaming == enable) {
		mutex_unlock(&adsd3500->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(adsd3500->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(adsd3500->dev);
			goto err_unlock;
		}

		ret = adsd3500_start_streaming(adsd3500);
		if (ret)
			goto err_rpm_put;
	} else {
		adsd3500_stop_streaming(adsd3500);
		pm_runtime_put(adsd3500->dev);
	}

	adsd3500->streaming = enable;
	mutex_unlock(&adsd3500->lock);

	return ret;

err_rpm_put:
	pm_runtime_put(adsd3500->dev);
err_unlock:
	mutex_unlock(&adsd3500->lock);

	return ret;
}

static int adsd3500_g_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct adsd3500 *adsd3500 = to_adsd3500(subdev);
	uint32_t val;
	int ret;

	fi->interval.numerator = 1;
	fi->interval.denominator = 10;

	ret = regmap_read(adsd3500->regmap, GET_FRAMERATE_CMD, &val);
	if (ret < 0)
		dev_err(adsd3500->dev, "Get FRAMERATE COMMAND failed.\n");
	else
		fi->interval.denominator = val;

	return ret;
}

static int adsd3500_s_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct adsd3500 *adsd3500 = to_adsd3500(subdev);
	uint32_t val;
	int ret;

	val = DIV_ROUND_UP(fi->interval.denominator,  fi->interval.numerator);

	ret = regmap_write(adsd3500->regmap, SET_FRAMERATE_CMD, val);
	if (ret < 0)
		dev_err(adsd3500->dev, "Set FRAMERATE COMMAND failed.\n");

	dev_dbg(adsd3500->dev, "Set frame interval to %u / %u\n",
		fi->interval.numerator, fi->interval.denominator);

	return ret;
}

static int adsd3500_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}



static const struct v4l2_subdev_core_ops adsd3500_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= adsd3500_g_register,
	.s_register	= adsd3500_s_register,
#endif
	.s_power = adsd3500_s_power,
};

static const struct v4l2_subdev_video_ops adsd3500_video_ops = {
	.s_stream			= adsd3500_s_stream,
	.g_frame_interval	= adsd3500_g_frame_interval,
	.s_frame_interval	= adsd3500_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops adsd3500_subdev_pad_ops = {
	.init_cfg			= adsd3500_entity_init_cfg,
	.enum_mbus_code		= adsd3500_enum_mbus_code,
	.enum_frame_size	= adsd3500_enum_frame_size,
	.get_fmt			= adsd3500_get_format,
	.set_fmt			= adsd3500_set_format,
	.get_selection		= adsd3500_get_selection,
};

static const struct v4l2_subdev_ops adsd3500_subdev_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.core	= &adsd3500_core_ops,
#endif
	.video	= &adsd3500_video_ops,
	.pad	= &adsd3500_subdev_pad_ops,
};

static const struct media_entity_operations adsd3500_subdev_entity_ops = {
	.link_setup = adsd3500_link_setup,
};

static int adsd3500_init_ctrls(struct adsd3500 *priv){
	struct device *dev = priv->dev;
	int ret;

	v4l2_ctrl_handler_init(&priv->ctrls, 3);

	priv->pixel_rate = v4l2_ctrl_new_std(&priv->ctrls,
						  &adsd3500_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  1, INT_MAX, 1, 1);
	priv->link_freq = v4l2_ctrl_new_int_menu(&priv->ctrls,
						     &adsd3500_ctrl_ops,
						     V4L2_CID_LINK_FREQ,
						     ARRAY_SIZE(
							     link_freq_tbl) - 1,
						     0, link_freq_tbl);
	if (priv->link_freq)
		priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	priv->operating_mode = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_ctrl_operating_mode,
						NULL);
	priv->set_chip_config = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_ctrl_chip_config,
						NULL);

	priv->depth_bits = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_ctrl_depth_bits,
						NULL);

	priv->ab_bits = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_ab_depth_bits,
						NULL);

	priv->confidence_bits = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_confidence_bits,
						NULL);

	priv->ab_avg = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_ab_avg,
						NULL);

	priv->depth_en = v4l2_ctrl_new_custom(&priv->ctrls,
						&adsd3500_depth_en,
						NULL);

	ret = priv->ctrls.error;
	if (ret) {
		dev_err(dev, "%s: control initialization error %d\n",
			__func__, ret);
		return ret;
	}
	priv->sd.ctrl_handler = &priv->ctrls;

	//Initialize by default to 4 (RAW12, 12 bpp)
	v4l2_ctrl_s_ctrl(priv->depth_bits, 4);

	return 0;
}

static int adsd3500_debugfs_init(struct adsd3500 *priv){

	priv->debugfs = debugfs_create_dir("adsd3500", NULL);
	if(!priv->debugfs)
		return -ENOMEM;

	debugfs_create_file("value", 0660, priv->debugfs, priv, &adsd3500_debug_fops);

	return 0;
}

static int adsd3500_parse_dt(struct adsd3500 *priv){
	struct v4l2_fwnode_endpoint bus_cfg = {.bus_type = V4L2_MBUS_CSI2_DPHY};
	struct fwnode_handle *endpoint;
	struct device *dev = priv->dev;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	priv->current_config.nr_mipi_lanes = bus_cfg.bus.mipi_csi2.num_data_lanes;

	priv->rst_gpio = gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(priv->rst_gpio)) {
		dev_err(dev, "Unable to get \"reset\" gpio\n");
		return PTR_ERR(priv->rst_gpio);
	}

	priv->irq_gpio = gpiod_get_optional(dev, "interrupt", GPIOD_IN);
	if (IS_ERR(priv->irq_gpio)) {
		dev_err(dev, "Unable to get \"interrupt\" gpio\n");
		return PTR_ERR(priv->irq_gpio);
	}

	priv->irq = gpiod_to_irq(priv->irq_gpio);
	if(priv->irq < 0){
		dev_err(dev, "Unable to find the valid irq\n");
	}

	priv->current_config.use_vc = of_property_read_bool(dev->of_node, "adi,use-vc");

	return 0;
}

static int adsd3500_read_ack(struct v4l2_subdev *sd){

	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct device *dev = adsd3500->dev;
	struct i2c_client *client = to_i2c_client(dev);
	uint8_t read_val[4];
	int ret;

	ret = i2c_master_recv(client, read_val, sizeof(read_val));
	if (ret < 0) {
		dev_err(adsd3500->dev, "Read ACK cmd failed.\n");
		return -EIO;
	}
	else{
		// Verify the acknowledgement success command op-code from the received response
		if (read_val[0] != 0x0B) {
			dev_err(adsd3500->dev, "ACK ERROR response d0: %02X  d1: %02X d2: %02X d3: %02X\n", read_val[0], read_val[1], read_val[2], read_val[3]);
			return -ENXIO;
		}
		dev_dbg(adsd3500->dev, "ACK Received\n");
	}

	return 0;
}

static int adsd3500_send_host_boot_data(struct v4l2_subdev *sd, uint8_t* data, int size)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	struct device *dev = adsd3500->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int n_segments = 0, prev_seg_end = 0, curr_segment= 0;
	int position = 0, location = 0;
	int segment_start[100], segment_end[100], segment_size[100];
	int header_start[100], header_end[100];
	uint8_t header_packet[16];
	uint8_t* data_packet = NULL;
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	data_packet = (uint8_t *)kvzalloc(PAYLOAD_MAX_CHUNK_SIZE * sizeof(uint8_t),GFP_KERNEL);

	while (1) {
		segment_size[n_segments] = (data[prev_seg_end+2]<<8)|data[prev_seg_end+1];
		segment_start[n_segments] = prev_seg_end + HEADER_SIZE_IN_BYTES;
		segment_end[n_segments] = segment_start[n_segments] + segment_size[n_segments];
		header_start[n_segments] = segment_start[n_segments] - HEADER_SIZE_IN_BYTES;
		header_end[n_segments] = segment_start[n_segments] - 1;
		prev_seg_end = segment_end[n_segments];
		n_segments++;
		if (prev_seg_end >= size) {
			break;
		}
	}

	dev_info(adsd3500->dev, "No of headers =%02d\n", n_segments);

	while (curr_segment < n_segments){
		location = header_start[curr_segment];
		memcpy(header_packet, &data[location], HEADER_SIZE_IN_BYTES);
		dev_dbg(adsd3500->dev,"Current Segment = %d header_start = %d\n Size= %04X", curr_segment + 1, header_start[curr_segment], segment_size[curr_segment]);
		dev_dbg(adsd3500->dev, "Send Header data\n");
		ret = i2c_master_send(client, header_packet, HEADER_SIZE_IN_BYTES);
		if (ret < 0) {
			dev_err(adsd3500->dev, "Failed to write the header packet of segment %d\n", curr_segment + 1);
			return -EIO;
		}
		// Check for the RESET command op-code and skip the read acknowledgement
		if (header_packet[03] == 0x55) {
			dev_info(adsd3500->dev, "Firmware transfer Compelete\n");
			break;
		}
		ret = adsd3500_read_ack(sd);
		if(ret < 0) {
			dev_err(adsd3500->dev, "Failed to read the acknowledgement header packet segment: %d\n", curr_segment + 1);
			return -ENXIO;
		}
		msleep(5);

		if(segment_size[curr_segment] != 0){
			location = segment_start[curr_segment];
			memcpy(data_packet, &data[location], segment_size[curr_segment]);
			dev_dbg(adsd3500->dev, "Send Payload data\n");
			ret = i2c_master_send(client, data_packet, segment_size[curr_segment]);
			if (ret < 0) {
				dev_err(adsd3500->dev, "Failed to write the payload data of segment %d\n",curr_segment + 1);
				return -EIO;
			}
			msleep(5);
			ret = adsd3500_read_ack(sd);
			if(ret < 0) {
				dev_err(adsd3500->dev, "Failed to read the acknowledgement payload data segment: %d\n", curr_segment + 1);
				return -ENXIO;
			}
		}
		msleep(1);

		memset(header_packet, 0, sizeof(header_packet));
		memset(data_packet, 0, PAYLOAD_MAX_CHUNK_SIZE * sizeof(uint8_t));
		position=0;
		curr_segment++;
	}

	kfree(data_packet);

	return 0;
}

static int adsd3500_parse_fw(struct v4l2_subdev *sd)
{

	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	const struct firmware *fw = adsd3500->main_fw;
	uint8_t* data_fw_3500 = NULL;
	size_t data_fw_3500_size = 0;
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	data_fw_3500_size = fw->size;
	dev_info(adsd3500->dev, "Firmware size = %ld\n",data_fw_3500_size);
	data_fw_3500 = (uint8_t *)kvzalloc(data_fw_3500_size * sizeof(uint8_t),GFP_KERNEL);
	if(!data_fw_3500){
		dev_err(adsd3500->dev, "Failed to allocate memory for FW data\n");
		return -ENOMEM;
	}

	memcpy(data_fw_3500, fw->data, data_fw_3500_size);
	dev_dbg(adsd3500->dev, "FW data 1:%02X  2:%02X  3:%02X  4:%02X\n",data_fw_3500[0], data_fw_3500[1], data_fw_3500[2], data_fw_3500[3]);
	ret = adsd3500_send_host_boot_data(sd, data_fw_3500, data_fw_3500_size);
	if(ret != 0){
		dev_err(adsd3500->dev, "Failed to send the host boot firmware\n");
		kfree(data_fw_3500);
		return ret;
	};

	kfree(data_fw_3500);

	return 0;
}


static int adsd3500_load_firmware(struct v4l2_subdev *sd)
{
	struct adsd3500 *adsd3500 = to_adsd3500(sd);
	int ret;

	dev_dbg(adsd3500->dev, "Entered: %s\n",__func__);

	if(fw_load){
		dev_dbg(adsd3500->dev, "Request ADSD3500 firmware file\n");
		ret = request_firmware(&adsd3500->main_fw, ADSD3500_FIRMWARE, adsd3500->dev);
		if(ret < 0) {
			dev_err(adsd3500->dev, "Failed to read firmware\n");
			goto release_firmware;
		}
		else {
			ret = adsd3500_parse_fw(sd);
			if(ret < 0){
				dev_err(adsd3500->dev, "Failed to parse the firmware\n");
				goto release_firmware;
				return ret;
			}
		}
	}

	release_firmware(adsd3500->main_fw);

	return 0;

release_firmware:
	release_firmware(adsd3500->main_fw);
	return ret;

}

static int adsd3500_probe(struct i2c_client *client)
{
	
	struct device *dev = &client->dev;
	
	struct adsd3500 *adsd3500;
	int ret;

	adsd3500 = devm_kzalloc(&client->dev, sizeof(struct adsd3500), GFP_KERNEL);
	if (!adsd3500)
		return -ENOMEM;
	adsd3500->dev = dev;

	adsd3500->regmap = devm_regmap_init_i2c(client,
						 &adsd3500_regmap_config);
	if (IS_ERR(adsd3500->regmap)) {
		dev_err(dev, "Error initializing I2C regmap\n");
		return PTR_ERR(adsd3500->regmap);
	}

	mutex_init(&adsd3500->lock);

	adsd3500_parse_dt(adsd3500);

	if(adsd3500->rst_gpio)
		gpiod_set_value(adsd3500->rst_gpio, 1);

	ret = devm_request_irq(dev, adsd3500->irq, adsd3500_irq_handler,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       client->name, adsd3500);
	if(ret < 0){
		dev_err(dev, "Failed to request IRQ %d\n",adsd3500->irq);
	}

	ret= adsd3500_debugfs_init(adsd3500);

	ret = adsd3500_init_ctrls(adsd3500);
	if (ret < 0)
		goto release_gpio;

	v4l2_i2c_subdev_init(&adsd3500->sd, client, &adsd3500_subdev_ops);
	adsd3500->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	adsd3500->pad.flags = MEDIA_PAD_FL_SOURCE;
	adsd3500->sd.dev = &client->dev;
	adsd3500->sd.entity.ops = &adsd3500_subdev_entity_ops;
	adsd3500->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&adsd3500->sd.entity, 1, &adsd3500->pad);
	if (ret < 0) {
		dev_err(dev, "Could not register media entity\n");
		goto free_ctrl;
	}

	if(fw_load){
		ret = adsd3500_load_firmware(&adsd3500->sd);
		if(ret < 0){
			dev_err(dev, "Failed load to the adsd3500 firmware\n");
			goto release_gpio;
			return ret;
		}
	}

	ret = adsd3500_entity_init_cfg(&adsd3500->sd, NULL);
	if (ret) {
		dev_err(dev, "Could not init v4l2 device\n");
		goto free_entity;
	}

	ret = v4l2_async_register_subdev(&adsd3500->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 device\n");
		goto free_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

free_entity:
	media_entity_cleanup(&adsd3500->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&adsd3500->ctrls);
	mutex_destroy(&adsd3500->lock);
release_gpio:
	gpiod_put(adsd3500->rst_gpio);
	gpiod_put(adsd3500->irq_gpio);
	debugfs_remove(adsd3500->debugfs);

	return ret;
}

static int adsd3500_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adsd3500 *adsd3500 = to_adsd3500(sd);

	v4l2_async_unregister_subdev(&adsd3500->sd);
	media_entity_cleanup(&adsd3500->sd.entity);
	devm_free_irq(adsd3500->dev, adsd3500->irq, adsd3500);
	gpiod_put(adsd3500->rst_gpio);
	gpiod_put(adsd3500->irq_gpio);
	debugfs_remove(adsd3500->debugfs);
	v4l2_ctrl_handler_free(&adsd3500->ctrls);
	mutex_destroy(&adsd3500->lock);

	pm_runtime_disable(adsd3500->dev);
	if (!pm_runtime_status_suspended(adsd3500->dev))
		adsd3500_power_off(adsd3500->dev);
	pm_runtime_set_suspended(adsd3500->dev);

	return 0;
}

static const struct of_device_id adsd3500_of_match[] = {
	{ .compatible = "adi,adsd3500" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adsd3500_of_match);

static const struct dev_pm_ops adsd3500_pm_ops = {
	SET_RUNTIME_PM_OPS(adsd3500_power_off, adsd3500_power_on, NULL)
};

static struct i2c_driver adsd3500_i2c_driver = {
	.driver			= {
		.of_match_table = adsd3500_of_match,
		.name		= "adsd3500",
		.pm		= &adsd3500_pm_ops,
	},
	.probe_new		= adsd3500_probe,
	.remove			= adsd3500_remove,
};

module_i2c_driver(adsd3500_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADI ADSD3500 Driver");
MODULE_AUTHOR("Bogdan Togorean");
MODULE_LICENSE("GPL v2");
