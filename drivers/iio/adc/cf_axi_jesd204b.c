/*
 * ADI AXI-JESD204B Interface Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/device.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/clk.h>

#include "cf_axi_jesd204b.h"

#define AXI_JESD204B_ES_HSIZE_FULL 	65
#define AXI_JESD204B_ES_HSIZE_HALF 	129
#define AXI_JESD204B_ES_HSIZE_QRTR 	257
#define AXI_JESD204B_ES_HSIZE_OCT 	513
#define AXI_JESD204B_ES_HSIZE_HEX 	1025

#define AXI_JESD204B_ES_VSIZE		255

struct jesd204b_state {
	struct device 		*dev;
	void __iomem		*regs;
	struct clk 		*clk;
	void			*buf_virt;
	dma_addr_t		buf_phys;
	struct bin_attribute 	bin;
	unsigned			size;
	int			lane;
	int			prescale;
	unsigned			vers_id;
	struct work_struct 	work;
	struct completion       complete;
	unsigned long		flags;
	unsigned long		rate;
	unsigned			es_hsize;
};

/*
 * IO accessors
 */

static inline void jesd204b_write(struct jesd204b_state *st,
				  unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int jesd204b_read(struct jesd204b_state *st,
					 unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int jesd204b_set_lane(struct jesd204b_state *st, unsigned lane)
{
	jesd204b_write(st, AXI_JESD204B_REG_RX_LANESEL,
		       AXI_JESD204B_RX_LANESEL(lane));

	return 0;
}

static int jesd204b_es(struct jesd204b_state *st, unsigned lane)
{
	unsigned stat;

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_CNTRL,
		AXI_JESD204B_EYESCAN_STOP);

	jesd204b_set_lane(st, lane);

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_PRESCALE,
		AXI_JESD204B_EYESCAN_PRESCALE(st->prescale));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_VOFFSET,
		AXI_JESD204B_EYESCAN_VOFFSET_STEP(1) |
		AXI_JESD204B_EYESCAN_VOFFSET_MAX(AXI_JESD204B_ES_VSIZE / 2) |
		AXI_JESD204B_EYESCAN_VOFFSET_MIN(-1 * (AXI_JESD204B_ES_VSIZE / 2)));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_HOFFSET_1,
		AXI_JESD204B_EYESCAN_HOFFSET_MAX(st->es_hsize / 2) |
		AXI_JESD204B_EYESCAN_HOFFSET_MIN(-1 * (st->es_hsize / 2)));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_HOFFSET_2,
		AXI_JESD204B_EYESCAN_HOFFSET_STEP(1));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_DMA_STARTADDR, st->buf_phys);

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_SDATA_1_0,
		AXI_JESD204B_EYESCAN_SDATA1(0) |
		AXI_JESD204B_EYESCAN_SDATA0(0));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_SDATA_3_2,
		AXI_JESD204B_EYESCAN_SDATA3(0xFFFF) |
		AXI_JESD204B_EYESCAN_SDATA2(0xFFFF));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_SDATA_4,
		AXI_JESD204B_EYESCAN_SDATA4(0xFFFF));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_QDATA_1_0,
		AXI_JESD204B_EYESCAN_QDATA1(0xFFFF) |
		AXI_JESD204B_EYESCAN_QDATA0(0xFFFF));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_QDATA_3_2,
		AXI_JESD204B_EYESCAN_QDATA3(0xFFFF) |
		AXI_JESD204B_EYESCAN_QDATA2(0xFFFF));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_QDATA_4,
		AXI_JESD204B_EYESCAN_QDATA4(0xFFFF));

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_CNTRL,
		AXI_JESD204B_EYESCAN_INIT |
		AXI_JESD204B_EYESCAN_START);

	do {
		msleep(50 * ((st->prescale & 0x1F) + 1));
		stat = jesd204b_read(st, AXI_JESD204B_REG_EYESCAN_STATUS);
		if (stat & AXI_JESD204B_EYESCAN_DMAERR)
			return -EIO;

	} while (stat & AXI_JESD204B_EYESCAN_STATUS);

	return 0;
}

static void jesd204b_work_func(struct work_struct *work)
{
	struct jesd204b_state *st =
		container_of(work, struct jesd204b_state, work);
	int ret;

	set_bit(0, &st->flags);
		ret = jesd204b_es(st, st->lane);
	if (ret)
		dev_warn(st->dev, "Eye Scan failed (%d)\n", ret);

	complete_all(&st->complete);
	clear_bit(0, &st->flags);
}

static ssize_t
jesd204b_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{
	struct jesd204b_state *st;
	struct device *dev;
	int ret = 0;

	dev = container_of(kobj, struct device, kobj);
	st = dev_get_drvdata(dev);

	if (unlikely(off >= st->bin.size))
		return 0;
	if ((off + count) > st->bin.size)
		count = st->bin.size - off;
	if (unlikely(!count))
		return count;

	wait_for_completion(&st->complete);

	memcpy(buf, st->buf_virt + off, count);

	return ret ? : count;
}

static ssize_t jesd204b_laneinfo_read(struct device *dev,
			struct device_attribute *attr,
			char *buf, unsigned lane)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);
	int ret;
	unsigned val;

	if (test_bit(0, &st->flags) && !completion_done(&st->complete))
		return -EBUSY;

	jesd204b_set_lane(st, lane);

	val = jesd204b_read(st, AXI_JESD204B_REG_RX_INIT_DATA_0);
	ret = sprintf(buf,
		"DID: %d, BID: %d, LID: %d, L: %d, SCR: %d, F: %d\n",
		AXI_JESD204B_INIT0_DID(val),
		AXI_JESD204B_INIT0_BID(val),
		AXI_JESD204B_INIT0_LID(val),
		AXI_JESD204B_INIT0_L(val),
		AXI_JESD204B_INIT0_SCR(val),
		AXI_JESD204B_INIT0_F(val));

	val = jesd204b_read(st, AXI_JESD204B_REG_RX_INIT_DATA_1);
	ret += sprintf(buf + ret,
		"K: %d, M: %d, N: %d, CS: %d, S: %d, N': %d, HD: %d\n",
		AXI_JESD204B_INIT1_K(val),
		AXI_JESD204B_INIT1_M(val),
		AXI_JESD204B_INIT1_N(val),
		AXI_JESD204B_INIT1_CS(val),
		AXI_JESD204B_INIT1_S(val),
		AXI_JESD204B_INIT1_ND(val),
		AXI_JESD204B_INIT1_HD(val));

	val = jesd204b_read(st, AXI_JESD204B_REG_RX_INIT_DATA_2);
	ret += sprintf(buf + ret, "FCHK: 0x%X, CF: %d\n",
		AXI_JESD204B_INIT2_FCHK(val),
		AXI_JESD204B_INIT2_CF(val));

	val = jesd204b_read(st, AXI_JESD204B_REG_RX_INIT_DATA_3);
	ret += sprintf(buf + ret,
		"ADJCNT: %d, PHYADJ: %d, ADJDIR: %d, JESDV: %d, SUBCLASS: %d\n",
		AXI_JESD204B_INIT3_ADJCNT(val),
		AXI_JESD204B_INIT3_PHYADJ(val),
		AXI_JESD204B_INIT3_ADJDIR(val),
		AXI_JESD204B_INIT3_JESDV(val),
		AXI_JESD204B_INIT3_SUBCLASSV(val));

	ret += sprintf(buf + ret, "MFCNT : 0x%X\n",
		       jesd204b_read(st, AXI_JESD204B_REG_RX_TEST_MFCNT));
	ret += sprintf(buf + ret, "ILACNT: 0x%X\n",
		       jesd204b_read(st, AXI_JESD204B_REG_RX_TEST_ILACNT));
	ret += sprintf(buf + ret, "ERRCNT: 0x%X\n",
		       jesd204b_read(st, AXI_JESD204B_REG_RX_TEST_ERRCNT));
	ret += sprintf(buf + ret, "BUFCNT: 0x%X\n",
		       jesd204b_read(st, AXI_JESD204B_REG_RX_BUFCNT));

	ret += sprintf(buf + ret, "FC: %lu\n", st->rate);
	ret += sprintf(buf + ret, "x%d,y%d CDRDW: %d\n", st->es_hsize, AXI_JESD204B_ES_VSIZE, 40);

	return ret;
}

#define JESD_LANE(_x) 						 \
static ssize_t jesd204b_lane##_x##_info_read(struct device *dev,	\
			struct device_attribute *attr,		\
			char *buf)				\
{								\
	return jesd204b_laneinfo_read(dev, attr, buf, _x);	\
}								\
static DEVICE_ATTR(lane##_x##_info, S_IRUSR, jesd204b_lane##_x##_info_read, NULL);

JESD_LANE(0);
JESD_LANE(1);
JESD_LANE(2);
JESD_LANE(3);
JESD_LANE(4);
JESD_LANE(5);
JESD_LANE(6);
JESD_LANE(7);

static ssize_t jesd204b_enable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);

	sscanf(buf, "%d", &st->lane);

	if (!completion_done(&st->complete)) {
		cancel_work_sync(&st->work);
		complete_all(&st->complete);
	}

	INIT_COMPLETION(st->complete);
	schedule_work(&st->work);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, jesd204b_enable);

static ssize_t jesd204b_set_prescale(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);

	sscanf(buf, "%d", &st->prescale);

	return count;
}
static DEVICE_ATTR(prescale, S_IWUSR, NULL, jesd204b_set_prescale);

/* Match table for of_platform binding */
static const struct of_device_id jesd204b_of_match[] = {
	{ .compatible = "xlnx,axi-jesd204b-rx2-1.01.a", .data = (void*) 2},
	{ .compatible = "xlnx,axi-jesd204b-rx4-1.01.a", .data = (void*) 4},
	{ .compatible = "xlnx,axi-jesd204b-rx2-1.00.a", .data = (void*) 2},
	{ .compatible = "xlnx,axi-jesd204b-rx4-1.00.a", .data = (void*) 4},
	{ .compatible = "xlnx,axi-jesd204b-rx1-1.00.a", .data = (void*) 1},
	{ .compatible = "xlnx,axi-jesd204b-rx8-1.00.a", .data = (void*) 8},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, jesd204b_of_match);

static int jesd204b_of_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct jesd204b_state *st;
	struct resource r_mem; /* IO mem resources */
	struct clk *clk;
	resource_size_t remap_size, phys_addr;
	unsigned frmcnt, bytecnt;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(jesd204b_of_match, &op->dev);

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	clk = devm_clk_get(&op->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	st = devm_kzalloc(&op->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	st->dev = dev;
	st->clk = clk;
	dev_set_drvdata(dev, st);

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	st->rate = clk_get_rate(clk);

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		goto err_clk_disable;
	}

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto err_clk_disable;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto err_release_mem_region;
	}

	ret = of_property_read_u32(op->dev.of_node,
				   "jesd,frames-per-multiframe", &frmcnt);
	if (ret)
		goto err_release_mem_region;

	ret = of_property_read_u32(op->dev.of_node,
				   "jesd,bytes-per-frame", &bytecnt);
	if (ret)
		goto err_release_mem_region;

	jesd204b_write(st, AXI_JESD204B_REG_RX_CNTRL_1,
		       (of_property_read_bool(op->dev.of_node,
			"jesd,scramble_en") ?
		       AXI_JESD204B_RX_DESCR_ENB : 0) |
		       (of_property_read_bool(op->dev.of_node,
			"jesd,lanesync_en") ?
		       AXI_JESD204B_RX_LANESYNC_ENB : 0) |
		      (of_property_read_bool(op->dev.of_node,
			"jesd,sysref_en") ?
		       AXI_JESD204B_RX_SYSREF_ENB : 0) |
			AXI_JESD204B_RX_MFRM_FRMCNT(frmcnt - 1) |
			AXI_JESD204B_RX_FRM_BYTECNT(bytecnt - 1));

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN);

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN |
		       AXI_JESD204B_IP_RSTN);

	jesd204b_write(st, AXI_JESD204B_REG_SYSREF,
		       AXI_JESD204B_SYSREF);

	mdelay(1);

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN |
		       AXI_JESD204B_IP_RSTN |
		       AXI_JESD204B_RSTN);

	jesd204b_write(st, AXI_JESD204B_REG_SYSREF,
		       AXI_JESD204B_SYSREF |
		       AXI_JESD204B_IP_SYSREF);

	jesd204b_write(st, AXI_JESD204B_REG_SYNC,
		       AXI_JESD204B_SYNC);

	mdelay(1);

	if (!jesd204b_read(st, AXI_JESD204B_REG_RX_STATUS));
		dev_warn(dev, "JESD Link/Lane Errors");

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN);

	dev_info(dev, "AXI-JESD204B (0x%X) at 0x%08llX mapped to 0x%p,",
		 jesd204b_read(st, ADI_REG_VERSION),
		 (unsigned long long)phys_addr, st->regs);

	/* TEMP workaround - this needs to come from DRP: RXOUT_DIV */
	switch (st->vers_id) {
	case 8:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_FULL;
		break;
	default:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_HALF;
		break;
	}

	st->size = st->es_hsize * AXI_JESD204B_ES_VSIZE *
		sizeof(unsigned long long);
	st->prescale = 0;
	st->buf_virt = dma_alloc_coherent(dev, PAGE_ALIGN(st->size),
					  &st->buf_phys, GFP_KERNEL);

	memset(st->buf_virt, 0, PAGE_ALIGN(st->size));

	sysfs_bin_attr_init(&st->bin);
	st->bin.attr.name = "eye_data";
	st->bin.attr.mode = S_IRUSR;
	st->bin.read = jesd204b_bin_read;
	st->bin.size = st->size;

	ret = sysfs_create_bin_file(&op->dev.kobj, &st->bin);
	if (ret)
		goto err_release_mem_region;

	device_create_file(dev, &dev_attr_enable);
	device_create_file(dev, &dev_attr_prescale);

	switch (st->vers_id) {
	case 1:
		device_create_file(dev, &dev_attr_lane0_info);
		break;
	case 2:
		device_create_file(dev, &dev_attr_lane0_info);
		device_create_file(dev, &dev_attr_lane1_info);
		break;
	case 4:
		device_create_file(dev, &dev_attr_lane0_info);
		device_create_file(dev, &dev_attr_lane1_info);
		device_create_file(dev, &dev_attr_lane2_info);
		device_create_file(dev, &dev_attr_lane3_info);
		break;
	case 8:
		device_create_file(dev, &dev_attr_lane0_info);
		device_create_file(dev, &dev_attr_lane1_info);
		device_create_file(dev, &dev_attr_lane2_info);
		device_create_file(dev, &dev_attr_lane3_info);
		device_create_file(dev, &dev_attr_lane4_info);
		device_create_file(dev, &dev_attr_lane5_info);
		device_create_file(dev, &dev_attr_lane6_info);
		device_create_file(dev, &dev_attr_lane7_info);
		break;
	default:

		break;
	}

	INIT_WORK(&st->work, jesd204b_work_func);
	init_completion(&st->complete);

	return 0;

err_release_mem_region:
	release_mem_region(phys_addr, remap_size);
err_clk_disable:
	clk_disable_unprepare(clk);
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * jesd204b_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int jesd204b_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct jesd204b_state *st = dev_get_drvdata(dev);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	clk_disable_unprepare(st->clk);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static struct platform_driver jesd204b_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_of_match,
	},
	.probe		= jesd204b_of_probe,
	.remove		= jesd204b_of_remove,
};

module_platform_driver(jesd204b_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-JESD204B Interface Module");
MODULE_LICENSE("GPL v2");
