/*
 * ADI AXI-ADXCVR Module
 *
 * Copyright 2018 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * https://wiki.analog.com/resources/fpga/docs/axi_adxcvr
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/sysfs.h>

#include "axi_adxcvr_eyescan.h"

static inline u32 adxcvr_eyescan_read(struct adxcvr_state *st, u32 reg)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
		 reg, ioread32(st->regs + reg));

	return ioread32(st->regs + reg);
}

static inline void adxcvr_eyescan_write(struct adxcvr_state *st,
					u32 reg, u32 val)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, val);

	iowrite32(val, st->regs + reg);
}

static int adxcvr_get_eyescan_es_hsize(struct adxcvr_state *st, u32 *hsize)
{
	u32 out_div;
	int ret;

	if (!hsize)
		return -EINVAL;

	ret = xilinx_xcvr_read_out_div(&st->xcvr, ADXCVR_DRP_PORT_CHANNEL(0),
				       &out_div, NULL);
	if (ret < 0)
		return ret;


	switch (out_div) {
	case 0x1:
		*hsize = ES_HSIZE_FULL;
		break;
	case 0x2:
		*hsize = ES_HSIZE_HALF;
		break;
	case 0x4:
		*hsize = ES_HSIZE_QRTR;
		break;
	case 0x8:
		*hsize = ES_HSIZE_OCT;
		break;
	case 0x10:
		*hsize = ES_HSIZE_HEX;
		break;
	default:
		dev_err(st->dev, "Failed get EYESCAN_RATE/RXOUT_DIV\n");
		return -EINVAL;
	}

	return 0;
}

static int adxcvr_eyescan_es(struct adxcvr_state *st, u32 lane)
{
	u32 stat, hsize;
	int ret;

	adxcvr_eyescan_write(st, ADXCVR_REG_ES_REQ, 0);

	ret = adxcvr_get_eyescan_es_hsize(st, &hsize);
	if (ret < 0)
		return ret;

	adxcvr_eyescan_write(st, ADXCVR_REG_ES_SEL, ADXCVR_ES_SEL(lane));
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_CONTROL_1,
			     ADXCVR_ES_PRESCALE(st->eye->prescale));
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_CONTROL_2,
			     ADXCVR_ES_VOFFSET_RANGE(0) |
			     ADXCVR_ES_VOFFSET_STEP(1) |
			     ADXCVR_ES_VOFFSET_MAX(ES_VSIZE / 2) |
			     ADXCVR_ES_VOFFSET_MIN(-1 * (ES_VSIZE / 2)));
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_CONTROL_3,
			     ADXCVR_ES_HOFFSET_MAX(hsize / 2) |
			     ADXCVR_ES_HOFFSET_MIN(-1 * (hsize / 2)));
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_CONTROL_4,
			     ADXCVR_ES_HOFFSET_STEP(1));
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_CONTROL_5, st->eye->buf_phys);
	adxcvr_eyescan_write(st, ADXCVR_REG_ES_REQ, ADXCVR_ES_REQ);

	do {
		msleep(50 * ((st->eye->prescale & 0x1F) + 1));
		stat = adxcvr_eyescan_read(st, ADXCVR_REG_ES_STATUS);
		if (stat & ADXCVR_ES_STATUS)
			return -EIO;
		stat = adxcvr_eyescan_read(st, ADXCVR_REG_ES_REQ);

	} while (stat & ADXCVR_ES_REQ);

	return 0;
}

static void adxcvr_eyescan_work_func(struct work_struct *work)
{
	struct adxcvr_eyescan *eye =
		container_of(work, struct adxcvr_eyescan, work);
	int ret;

	ret = adxcvr_eyescan_es(eye->st, eye->lane);
	if (ret)
		dev_warn(eye->st->dev, "Eye Scan failed (%d)\n", ret);


	complete_all(&eye->complete);
}

static ssize_t
adxcvr_eyescan_bin_read(struct file *filp, struct kobject *kobj,
		     struct bin_attribute *bin_attr,
		     char *buf, loff_t off, size_t count)
{
	struct adxcvr_state *st;
	struct device *dev;

	dev = container_of(kobj, struct device, kobj);
	st = dev_get_drvdata(dev);

	if (unlikely(off >= st->eye->bin.size))
		return 0;
	if ((off + count) > st->eye->bin.size)
		count = st->eye->bin.size - off;
	if (unlikely(!count))
		return count;

	if (wait_for_completion_interruptible(&st->eye->complete)) {
		adxcvr_eyescan_write(st, ADXCVR_REG_ES_REQ, 0);
		return -EINTR;
	}

	memcpy(buf, st->eye->buf_virt + off, count);

	return count;
}

static ssize_t adxcvr_eyescan_set_enable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	int ret;

	ret = kstrtoint(buf, 0, &st->eye->lane);
	if (ret)
		return ret;

	if (st->eye->lane >= 0xFF || st->eye->lane < 0)
		return -EINVAL;

	if (!completion_done(&st->eye->complete)) {
		adxcvr_eyescan_write(st, ADXCVR_REG_ES_REQ, 0);
		cancel_work_sync(&st->eye->work);
		complete_all(&st->eye->complete);
	}

	reinit_completion(&st->eye->complete);
	schedule_work(&st->eye->work);

	return count;
}

static ssize_t adxcvr_eyescan_get_enable(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);

	if (!completion_done(&st->eye->complete) && (st->eye->lane != -1))
		return -EBUSY;

	return sprintf(buf, "%d\n", st->eye->lane);
}

static DEVICE_ATTR(enable, 0644, adxcvr_eyescan_get_enable,
		   adxcvr_eyescan_set_enable);

static ssize_t adxcvr_eyescan_set_prescale(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	int ret;

	ret = kstrtoint(buf, 0,  &st->eye->prescale);
	if (ret)
		return ret;

	return count;
}

static ssize_t adxcvr_eyescan_get_prescale(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", st->eye->prescale);
}

static DEVICE_ATTR(prescale, 0644, adxcvr_eyescan_get_prescale,
		   adxcvr_eyescan_set_prescale);

static ssize_t adxcvr_eyescan_info_read(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	u32 hsize;
	int ret;

	ret = adxcvr_get_eyescan_es_hsize(st, &hsize);
	if (ret < 0)
		return ret;

	return sprintf(buf, "x%d,y%d CDRDW: %d LPM: %d NL: %d LR: %lu\n",
		       hsize, ES_VSIZE, 40, st->lpm_enable,
		       st->num_lanes, st->lane_rate);
}

static DEVICE_ATTR(eyescan_info, 0444, adxcvr_eyescan_info_read, NULL);

int adxcvr_eyescan_register(struct adxcvr_state *st)
{
	struct adxcvr_eyescan *eye;
	int ret;

	if (st->tx_enable)
		return 0;

	eye = devm_kzalloc(st->dev, sizeof(*eye), GFP_KERNEL);
	if (!eye)
		return -ENOMEM;

	st->eye = eye;
	eye->st = st;
	eye->lane = -1;

	sysfs_bin_attr_init(&eye->bin);
	eye->bin.attr.name = "eye_data";
	eye->bin.attr.mode = 0444;
	eye->bin.read = adxcvr_eyescan_bin_read;
	eye->bin.size = ES_HSIZE_HEX * ES_VSIZE * sizeof(u64);

	eye->buf_virt = dma_alloc_coherent(st->dev, PAGE_ALIGN(eye->bin.size),
					  &eye->buf_phys, GFP_KERNEL);

	if (eye->buf_virt == NULL) {
		dev_err(st->dev, "Not enough dma memory for device\n");
		return -ENOMEM;
	}

	memset(eye->buf_virt, 0, PAGE_ALIGN(eye->bin.size));

	ret = sysfs_create_bin_file(&st->dev->kobj, &eye->bin);
	if (ret) {
		dev_err(st->dev, "Failed to create sysfs bin file\n");
		goto err_dma_free;
	}

	device_create_file(st->dev, &dev_attr_enable);
	device_create_file(st->dev, &dev_attr_prescale);
	device_create_file(st->dev, &dev_attr_eyescan_info);

	INIT_WORK(&eye->work, adxcvr_eyescan_work_func);
	init_completion(&eye->complete);

	return 0;

err_dma_free:
	dma_free_coherent(st->dev, PAGE_ALIGN(st->eye->bin.size),
		  st->eye->buf_virt, st->eye->buf_phys);

	return ret;

}

int adxcvr_eyescan_unregister(struct adxcvr_state *st)
{
	if (st->tx_enable)
		return 0;

	cancel_work_sync(&st->eye->work);
	complete_all(&st->eye->complete);

	sysfs_remove_bin_file(&st->dev->kobj, &st->eye->bin);
	device_remove_file(st->dev, &dev_attr_enable);
	device_remove_file(st->dev, &dev_attr_prescale);
	device_remove_file(st->dev, &dev_attr_eyescan_info);

	dma_free_coherent(st->dev, PAGE_ALIGN(st->eye->bin.size),
			  st->eye->buf_virt, st->eye->buf_phys);

	return 0;
}
