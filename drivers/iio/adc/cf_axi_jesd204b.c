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
#include <linux/platform_device.h>
#include <linux/of_device.h>

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
	unsigned			addr;
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

	jesd204b_read(st, AXI_JESD204B_REG_RX_LANESEL);

	return 0;
}

static int jesd204b_es(struct jesd204b_state *st, unsigned lane)
{
	unsigned stat;

	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_CNTRL,
		AXI_JESD204B_EYESCAN_STOP);
	jesd204b_write(st, AXI_JESD204B_REG_EYESCAN_CNTRL, 0);

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

	reinit_completion(&st->complete);
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

static ssize_t jesd204b_reg_write(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);
	unsigned val;
	int ret;

	ret = sscanf(buf, "%i %i", &st->addr, &val);

	if (ret == 2)
		jesd204b_write(st, st->addr, val);

	return count;
}
static ssize_t jesd204b_reg_read(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct jesd204b_state *st = dev_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", jesd204b_read(st, st->addr));
}

static DEVICE_ATTR(reg_access, S_IWUSR | S_IRUSR, jesd204b_reg_read, jesd204b_reg_write);

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

static int jesd204b_probe(struct platform_device *pdev)
{
	struct jesd204b_state *st;
	unsigned frmcnt, bytecnt;
	struct resource *mem;
	struct clk *clk;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(jesd204b_of_match, &pdev->dev);

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
		 pdev->dev.of_node->name);

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return -EPROBE_DEFER;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev = &pdev->dev;
	st->clk = clk;
	platform_set_drvdata(pdev, st);

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	st->rate = clk_get_rate(clk);

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "jesd,frames-per-multiframe", &frmcnt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read required dt property\n");
		goto err_clk_disable;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "jesd,bytes-per-frame", &bytecnt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to read required dt property\n");
		goto err_clk_disable;
	}

	jesd204b_write(st, AXI_JESD204B_REG_RX_CNTRL_1,
		       (of_property_read_bool(pdev->dev.of_node,
			"jesd,scramble_en") ?
		       AXI_JESD204B_RX_DESCR_ENB : 0) |
		       (of_property_read_bool(pdev->dev.of_node,
			"jesd,lanesync_en") ?
		       AXI_JESD204B_RX_LANESYNC_ENB : 0) |
		      (of_property_read_bool(pdev->dev.of_node,
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

	mdelay(10);

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN |
		       AXI_JESD204B_IP_RSTN |
		       AXI_JESD204B_RSTN | AXI_JESD204B_DRP_RSTN);

	jesd204b_write(st, AXI_JESD204B_REG_SYSREF,
		       AXI_JESD204B_SYSREF |
		       AXI_JESD204B_IP_SYSREF);

	jesd204b_write(st, AXI_JESD204B_REG_SYNC,
		       AXI_JESD204B_SYNC | 0x2);

	mdelay(10);


	if (!jesd204b_read(st, AXI_JESD204B_REG_RX_STATUS))
		dev_warn(&pdev->dev, "JESD Link/Lane Errors");

	st->es_hsize = jesd204b_read(st, AXI_JESD204B_REG_EYESCAN_RATE);

	switch (st->es_hsize) {
	case 0x1:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_FULL;
		break;
	case 0x2:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_HALF;
		break;
	case 0x4:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_QRTR;
		break;
	case 0x8:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_OCT;
		break;
	case 0x10:
		st->es_hsize = AXI_JESD204B_ES_HSIZE_HEX;
		break;
	default:
		ret = -EINVAL;
		dev_err(&pdev->dev, "Failed get EYESCAN_RATE/RXOUT_DIV\n");
		goto err_clk_disable;
	}

	st->size = st->es_hsize * AXI_JESD204B_ES_VSIZE *
		sizeof(unsigned long long);
	st->prescale = 0;
	st->buf_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(st->size),
					  &st->buf_phys, GFP_KERNEL);

	if (st->buf_virt == NULL) {
		dev_err(&pdev->dev, "Not enough dma memory for device\n");
		ret = -ENOMEM;
		goto err_clk_disable;
	}

	memset(st->buf_virt, 0, PAGE_ALIGN(st->size));

	sysfs_bin_attr_init(&st->bin);
	st->bin.attr.name = "eye_data";
	st->bin.attr.mode = S_IRUSR;
	st->bin.read = jesd204b_bin_read;
	st->bin.size = st->size;

	ret = sysfs_create_bin_file(&pdev->dev.kobj, &st->bin);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs bin file\n");
		goto err_dma_free;
	}

	device_create_file(&pdev->dev, &dev_attr_enable);
	device_create_file(&pdev->dev, &dev_attr_prescale);
	device_create_file(&pdev->dev, &dev_attr_reg_access);

	switch (st->vers_id) {
	case 8:
		device_create_file(&pdev->dev, &dev_attr_lane7_info);
		device_create_file(&pdev->dev, &dev_attr_lane6_info);
		device_create_file(&pdev->dev, &dev_attr_lane5_info);
		device_create_file(&pdev->dev, &dev_attr_lane4_info);
		/* fallthrough */
	case 4:
		device_create_file(&pdev->dev, &dev_attr_lane2_info);
		device_create_file(&pdev->dev, &dev_attr_lane3_info);
		/* fallthrough */
	case 2:
		device_create_file(&pdev->dev, &dev_attr_lane1_info);
		/* fallthrough */
	case 1:
		device_create_file(&pdev->dev, &dev_attr_lane0_info);
		break;
	default:

		break;
	}

	INIT_WORK(&st->work, jesd204b_work_func);
	init_completion(&st->complete);

	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN |
		       AXI_JESD204B_IP_RSTN |
		       AXI_JESD204B_DRP_RSTN);
	mdelay(10);
	jesd204b_write(st, AXI_JESD204B_REG_RSTN,
		       AXI_JESD204B_GT_RSTN |
		       AXI_JESD204B_IP_RSTN |
		       AXI_JESD204B_RSTN | AXI_JESD204B_DRP_RSTN);

	mdelay(10);

	dev_info(&pdev->dev, "AXI-JESD204B (0x%X) at 0x%08llX mapped to 0x%p,",
		 jesd204b_read(st, ADI_REG_VERSION),
		 (unsigned long long)mem->start, st->regs);

	return 0;

err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->size),
			  st->buf_virt, st->buf_phys);
err_clk_disable:
	clk_disable_unprepare(clk);

	return ret;
}

/**
 * jesd204b_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int jesd204b_remove(struct platform_device *pdev)
{
	struct jesd204b_state *st = platform_get_drvdata(pdev);

	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->size),
			  st->buf_virt, st->buf_phys);

	clk_disable_unprepare(st->clk);
	clk_put(st->clk);

	return 0;
}

static struct platform_driver jesd204b_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_of_match,
	},
	.probe		= jesd204b_probe,
	.remove		= jesd204b_remove,
};

module_platform_driver(jesd204b_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-JESD204B Interface Module");
MODULE_LICENSE("GPL v2");
