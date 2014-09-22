/*
 * ADI AXI-JESD204B GT Interface Module
 *
 * Copyright 2014 Analog Devices Inc.
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
#include <linux/clk-provider.h>

#include "cf_axi_jesd204b_gt.h"

#define ADI_ES_HSIZE_FULL 	65
#define ADI_ES_HSIZE_HALF 	129
#define ADI_ES_HSIZE_QRTR 	257
#define ADI_ES_HSIZE_OCT 	513
#define ADI_ES_HSIZE_HEX 	1025

#define ADI_ES_VSIZE		255

#define JESD204B_GT_RX		0
#define JESD204B_GT_TX		1


struct child_clk {
	struct device 		*dev;
	struct clk_hw		hw;
	unsigned long 		rate;
	bool			enabled;
	unsigned			num;
};

struct jesd204b_gt_state {
	struct device 		*dev;
	void __iomem		*regs;
	struct clk 		*clk;
	struct clk 		*adc_clk;
	struct clk 		*adc_sysref;
	struct clk 		*dac_clk;
	struct clk 		*dac_sysref;
	struct clk_onecell_data		clk_data;
	struct child_clk		output[2];


	void			*buf_virt;
	dma_addr_t		buf_phys;
	struct bin_attribute 	bin;
	unsigned			size;
	int			lane;
	int			prescale;
	unsigned			vers_id;
	unsigned			version;
	struct work_struct 	work;
	struct completion       complete;
	unsigned long		flags;
	unsigned long		rate;
	unsigned			es_hsize;
	unsigned			addr;
	unsigned			rx_sys_clk_sel;
	unsigned			rx_out_clk_sel;
	unsigned			tx_sys_clk_sel;
	unsigned			tx_out_clk_sel;
	bool			use_cpll;

	struct delayed_work	sync_work;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

/*
 * IO accessors
 */

static inline void jesd204b_gt_write(struct jesd204b_gt_state *st,
				  unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int jesd204b_gt_read(struct jesd204b_gt_state *st,
					 unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int jesd204b_gt_set_lane(struct jesd204b_gt_state *st, unsigned lane)
{
	jesd204b_gt_write(st, ADI_REG_RX_LANESEL,
		       ADI_RX_LANESEL(lane));

	jesd204b_gt_read(st, ADI_REG_RX_LANESEL);

	return 0;
}

static int jesd204b_gt_es(struct jesd204b_gt_state *st, unsigned lane)
{
	unsigned stat;

	jesd204b_gt_write(st, ADI_REG_EYESCAN_CNTRL,
		ADI_EYESCAN_STOP);
	jesd204b_gt_write(st, ADI_REG_EYESCAN_CNTRL, 0);

	jesd204b_gt_set_lane(st, lane);

	jesd204b_gt_write(st, ADI_REG_EYESCAN_PRESCALE,
		ADI_EYESCAN_PRESCALE(st->prescale));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_VOFFSET,
		ADI_EYESCAN_VOFFSET_STEP(1) |
		ADI_EYESCAN_VOFFSET_MAX(ADI_ES_VSIZE / 2) |
		ADI_EYESCAN_VOFFSET_MIN(-1 * (ADI_ES_VSIZE / 2)));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_HOFFSET_1,
		ADI_EYESCAN_HOFFSET_MAX(st->es_hsize / 2) |
		ADI_EYESCAN_HOFFSET_MIN(-1 * (st->es_hsize / 2)));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_HOFFSET_2,
		ADI_EYESCAN_HOFFSET_STEP(1));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_DMA_STARTADDR, st->buf_phys);

	jesd204b_gt_write(st, ADI_REG_EYESCAN_SDATA_1_0,
		ADI_EYESCAN_SDATA1(0) |
		ADI_EYESCAN_SDATA0(0));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_SDATA_3_2,
		ADI_EYESCAN_SDATA3(0xFFFF) |
		ADI_EYESCAN_SDATA2(0xFF00));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_SDATA_4,
		ADI_EYESCAN_SDATA4(0xFFFF));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_QDATA_1_0,
		ADI_EYESCAN_QDATA1(0xFFFF) |
		ADI_EYESCAN_QDATA0(0xFFFF));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_QDATA_3_2,
		ADI_EYESCAN_QDATA3(0xFFFF) |
		ADI_EYESCAN_QDATA2(0xFFFF));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_QDATA_4,
		ADI_EYESCAN_QDATA4(0xFFFF));

	jesd204b_gt_write(st, ADI_REG_EYESCAN_CNTRL,
		ADI_EYESCAN_INIT |
		ADI_EYESCAN_START);

	do {
		msleep(50 * ((st->prescale & 0x1F) + 1));
		stat = jesd204b_gt_read(st, ADI_REG_EYESCAN_STATUS);
		if (stat & ADI_EYESCAN_DMAERR)
			return -EIO;

	} while (stat & ADI_EYESCAN_STATUS);

	return 0;
}

static void jesd204b_gt_work_func(struct work_struct *work)
{
	struct jesd204b_gt_state *st =
		container_of(work, struct jesd204b_gt_state, work);
	int ret;

	set_bit(0, &st->flags);
		ret = jesd204b_gt_es(st, st->lane);
	if (ret)
		dev_warn(st->dev, "Eye Scan failed (%d)\n", ret);

	complete_all(&st->complete);
	clear_bit(0, &st->flags);
}

static ssize_t
jesd204b_gt_bin_read(struct file *filp, struct kobject *kobj,
		       struct bin_attribute *bin_attr,
		       char *buf, loff_t off, size_t count)
{
	struct jesd204b_gt_state *st;
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

static ssize_t jesd204b_gt_enable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);

	sscanf(buf, "%d", &st->lane);

	if (!completion_done(&st->complete)) {
		cancel_work_sync(&st->work);
		complete_all(&st->complete);
	}

	reinit_completion(&st->complete);
	schedule_work(&st->work);

	return count;
}
static DEVICE_ATTR(enable, S_IWUSR, NULL, jesd204b_gt_enable);

static ssize_t jesd204b_gt_set_prescale(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);

	sscanf(buf, "%d", &st->prescale);

	return count;
}
static DEVICE_ATTR(prescale, S_IWUSR, NULL, jesd204b_gt_set_prescale);

static ssize_t jesd204b_gt_reg_write(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);
	unsigned val;
	int ret;

	ret = sscanf(buf, "%i %i", &st->addr, &val);

	if (ret == 2)
		jesd204b_gt_write(st, st->addr, val);

	return count;
}
static ssize_t jesd204b_gt_reg_read(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", jesd204b_gt_read(st, st->addr));
}

static DEVICE_ATTR(reg_access, S_IWUSR | S_IRUSR, jesd204b_gt_reg_read,
		   jesd204b_gt_reg_write);


static ssize_t jesd204b_gt_info_read(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);

	return sprintf(buf, "x%d,y%d CDRDW: %d\n", st->es_hsize, ADI_ES_VSIZE, 40);
}

static DEVICE_ATTR(info, S_IRUSR, jesd204b_gt_info_read, NULL);


static unsigned long jesd204b_gt_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	return to_clk_priv(hw)->rate;
}

static int jesd204b_gt_status_error(struct device *dev,
				   unsigned offs, unsigned mask)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);
	unsigned val = jesd204b_gt_read(st, ADI_REG_RX_STATUS + offs);

	if ((val & mask) != mask) {
		dev_err(dev, "%s Error: %s%s%s\n",
			offs ? "TX" : "RX",
			(ADI_TO_RX_RST_DONE(val) != 0xFF) ? "RESET failed " : "",
			(ADI_TO_RX_PLL_LOCKED(val) != 0xFF) ? "PLL Unlocked " : "",
			(ADI_RX_STATUS & val) ? "" : "Interface Error"
       		);

		return -EIO;
	}

	return 0;
}

static void jesd204b_gt_clk_synchronize(struct child_clk *clk)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(clk->dev);
	unsigned offs = (clk->num == JESD204B_GT_TX ?
		ADI_REG_TX_GT_RSTN - ADI_REG_RX_GT_RSTN : 0);
	int ret;

	if (!clk->enabled)
		return;

	ret = jesd204b_gt_read(st, ADI_REG_RX_STATUS + offs);
	while (ret != 0x1ffff) {
		jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL + offs, ADI_RX_SYSREF);
		jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL + offs, 0);

		msleep(100);
		ret = jesd204b_gt_read(st, ADI_REG_RX_STATUS + offs);
		dev_dbg(st->dev, "Resynchronizing\n");
	}
}

static void jesd204b_gt_sync_work_func(struct work_struct *work)
{
	struct jesd204b_gt_state *st =
		container_of(work, struct jesd204b_gt_state, sync_work.work);
	unsigned int i;

	for (i = 0; i < 2; i++)
		jesd204b_gt_clk_synchronize(&st->output[i]);

	queue_delayed_work(system_freezable_wq, &st->sync_work, HZ);
}

static int jesd204b_gt_clk_enable(struct clk_hw *hw)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(to_clk_priv(hw)->dev);
	unsigned offs = (to_clk_priv(hw)->num == JESD204B_GT_TX ?
		ADI_REG_TX_GT_RSTN - ADI_REG_RX_GT_RSTN : 0);
	int ret = 0;

	jesd204b_gt_write(st, ADI_REG_RX_GT_RSTN + offs, 0);
	jesd204b_gt_write(st, ADI_REG_RX_RSTN + offs, 0);

	mdelay(10);

	ret = jesd204b_gt_read(st, ADI_REG_RX_STATUS + offs);
	if (ADI_TO_RX_PLL_LOCKED(ret) != 0xFF)
		dev_err(to_clk_priv(hw)->dev, "RX PLL NOT locked! (0x%X)\n", ret);

	jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL + offs, 0);
	jesd204b_gt_write(st, ADI_REG_RX_SYNC_CTL + offs, ADI_RX_SYNC);

	jesd204b_gt_write(st, ADI_REG_RX_GT_RSTN + offs, ADI_RX_GT_RSTN);
	jesd204b_gt_write(st, ADI_REG_RX_RSTN + offs, ADI_RX_RSTN);

	mdelay(40);

	jesd204b_gt_status_error(to_clk_priv(hw)->dev, offs,
		ADI_RX_RST_DONE(~0) | ADI_RX_PLL_LOCKED(~0));

	jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL + offs, ADI_RX_SYSREF);
	jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL + offs, 0);

	mdelay(50);

	ret = jesd204b_gt_status_error(to_clk_priv(hw)->dev, offs,
		ADI_RX_RST_DONE(~0) | ADI_RX_PLL_LOCKED(~0) | ADI_RX_STATUS);

	to_clk_priv(hw)->enabled = true;

	return ret;
}

static void jesd204b_gt_clk_disable(struct clk_hw *hw)
{
	to_clk_priv(hw)->enabled = false;
}

static int jesd204b_gt_clk_is_enabled(struct clk_hw *hw)
{
	return to_clk_priv(hw)->enabled;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = jesd204b_gt_clk_recalc_rate,
	.enable = jesd204b_gt_clk_enable,
	.disable = jesd204b_gt_clk_disable,
	.is_enabled = jesd204b_gt_clk_is_enabled,
};

static struct clk *jesd204b_gt_clk_register(struct device *dev, unsigned num,
				char *name)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);
	struct clk_init_data init;
	struct child_clk *output = &st->output[num];
	struct clk *clk;

	init.name = name;
	init.ops = &clkout_ops;

	init.num_parents = 0;
	init.flags = CLK_IS_ROOT;
	output->hw.init = &init;
	output->dev = dev;
	output->num = num;
	output->rate = st->rate;

	/* register the clock */
	clk = clk_register(dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

/* Match table for of_platform binding */
static const struct of_device_id jesd204b_gt_of_match[] = {
	{ .compatible = "xlnx,axi-jesd-gt-1.0", .data = (void*) 1},
	{ .compatible = "adi,axi-jesd-gt-1.0", .data = (void*) 1},
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, jesd204b_gt_of_match);

static int jesd204b_gt_probe(struct platform_device *pdev)
{
	struct jesd204b_gt_state *st;
	struct resource *mem; /* IO mem resources */
	struct clk *clk;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(jesd204b_gt_of_match, &pdev->dev);

	clk = devm_clk_get(&pdev->dev, "adc_clk");
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	st->adc_clk = clk;

	ret = clk_prepare_enable(st->adc_clk);
	if (ret < 0)
		return ret;

	st->adc_sysref = devm_clk_get(&pdev->dev, "adc_sysref");
	if (!IS_ERR(st->adc_sysref))
		clk_prepare_enable(st->adc_sysref);

	st->dac_clk = devm_clk_get(&pdev->dev, "dac_clk");
	if (!IS_ERR(st->dac_clk))
		clk_prepare_enable(st->dac_clk);

	st->dac_sysref = devm_clk_get(&pdev->dev, "dac_sysref");
	if (!IS_ERR(st->dac_sysref))
		clk_prepare_enable(st->dac_sysref);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev = &pdev->dev;
		platform_set_drvdata(pdev, st);

	st->rate = clk_get_rate(clk);

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "adi,rx-sys-clk-select", &st->rx_sys_clk_sel);
	ret = of_property_read_u32(pdev->dev.of_node,
				   "adi,rx-out-clk-select", &st->rx_out_clk_sel);
	ret = of_property_read_u32(pdev->dev.of_node,
				   "adi,tx-sys-clk-select", &st->tx_sys_clk_sel);
	ret = of_property_read_u32(pdev->dev.of_node,
				   "adi,tx-out-clk-select", &st->tx_out_clk_sel);

	st->use_cpll = of_property_read_bool(pdev->dev.of_node, "adi,use-cpll-enable");

	jesd204b_gt_write(st, ADI_REG_CPLL_PD, st->use_cpll ? 0 : ADI_CPLL_PD);

	if (st->rx_sys_clk_sel || st->rx_out_clk_sel)
		jesd204b_gt_write(st, ADI_REG_RX_CLK_SEL,
				ADI_RX_SYS_CLK_SEL(st->rx_sys_clk_sel) |
				ADI_RX_OUT_CLK_SEL(st->rx_out_clk_sel));

	if (st->tx_sys_clk_sel || st->tx_out_clk_sel)
		jesd204b_gt_write(st, ADI_REG_TX_CLK_SEL,
			  ADI_RX_SYS_CLK_SEL(st->tx_sys_clk_sel) |
			  ADI_RX_OUT_CLK_SEL(st->tx_out_clk_sel));

	jesd204b_gt_write(st, ADI_REG_RSTN_1, 0); /* resets (drp, pll) */


	jesd204b_gt_write(st, ADI_REG_RX_GT_RSTN, 0);
	jesd204b_gt_write(st, ADI_REG_RX_RSTN, 0);

	jesd204b_gt_write(st, ADI_REG_TX_GT_RSTN, 0);
	jesd204b_gt_write(st, ADI_REG_TX_RSTN, 0);

	jesd204b_gt_write(st, ADI_REG_RSTN_1,
			  ADI_DRP_RSTN | ADI_GT_PLL_RSTN); /* enable (drp, pll) */

	mdelay(50);

	if (st->rx_sys_clk_sel || st->rx_out_clk_sel) {
		ret = jesd204b_gt_read(st, ADI_REG_RX_STATUS);
		if (ADI_TO_RX_PLL_LOCKED(ret) != 0xFF)
			dev_err(&pdev->dev, "RX PLL NOT locked! (0x%X)\n", ret);
	}

	if (st->tx_sys_clk_sel || st->tx_out_clk_sel) {
		ret = jesd204b_gt_read(st, ADI_REG_TX_STATUS);
		if (ADI_TO_TX_PLL_LOCKED(ret) != 0xFF)
			dev_err(&pdev->dev, "TX PLL NOT locked! (0x%X)\n", ret);
	}

	jesd204b_gt_write(st, ADI_REG_RX_SYSREF_CTL, 0);
	jesd204b_gt_write(st, ADI_REG_RX_SYNC_CTL, ADI_RX_SYNC);

	jesd204b_gt_write(st, ADI_REG_TX_SYSREF_CTL, 0);
	jesd204b_gt_write(st, ADI_REG_TX_SYNC_CTL, ADI_TX_SYNC);


	st->es_hsize = jesd204b_gt_read(st, ADI_REG_EYESCAN_RATE);

	switch (st->es_hsize) {
	case 0x1:
		st->es_hsize = ADI_ES_HSIZE_FULL;
		break;
	case 0x2:
		st->es_hsize = ADI_ES_HSIZE_HALF;
		break;
	case 0x4:
		st->es_hsize = ADI_ES_HSIZE_QRTR;
		break;
	case 0x8:
		st->es_hsize = ADI_ES_HSIZE_OCT;
		break;
	case 0x10:
		st->es_hsize = ADI_ES_HSIZE_HEX;
		break;
	default:
		ret = -EINVAL;
		dev_err(&pdev->dev, "Failed get EYESCAN_RATE/RXOUT_DIV\n");
		return ret;
	}

	st->size = st->es_hsize * ADI_ES_VSIZE *
		sizeof(unsigned long long);
	st->prescale = 0;
	st->buf_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(st->size),
					  &st->buf_phys, GFP_KERNEL);

	if (st->buf_virt == NULL) {
		dev_err(&pdev->dev, "Not enough dma memory for device\n");
		ret = -ENOMEM;
		return ret;
	}

	memset(st->buf_virt, 0, PAGE_ALIGN(st->size));

	sysfs_bin_attr_init(&st->bin);
	st->bin.attr.name = "eye_data";
	st->bin.attr.mode = S_IRUSR;
	st->bin.read = jesd204b_gt_bin_read;
	st->bin.size = st->size;

	ret = sysfs_create_bin_file(&pdev->dev.kobj, &st->bin);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs bin file\n");
		goto err_dma_free;
	}

	device_create_file(&pdev->dev, &dev_attr_enable);
	device_create_file(&pdev->dev, &dev_attr_prescale);
	device_create_file(&pdev->dev, &dev_attr_reg_access);
	device_create_file(&pdev->dev, &dev_attr_info);

	INIT_WORK(&st->work, jesd204b_gt_work_func);
	init_completion(&st->complete);

	INIT_DELAYED_WORK(&st->sync_work, jesd204b_gt_sync_work_func);

	st->clk_data.clks = devm_kzalloc(&pdev->dev,
					 sizeof(*st->clk_data.clks) *
					 2, GFP_KERNEL);
	if (!st->clk_data.clks) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}
	st->clk_data.clk_num = 2;

	schedule_delayed_work(&st->sync_work, HZ);

	clk = jesd204b_gt_clk_register(&pdev->dev, JESD204B_GT_RX, "adc_gt_clk");
	if (IS_ERR(clk))
			return PTR_ERR(clk);

	clk = jesd204b_gt_clk_register(&pdev->dev, JESD204B_GT_TX, "dac_gt_clk");
	if (IS_ERR(clk))
			return PTR_ERR(clk);

	of_clk_add_provider(pdev->dev.of_node,
			    of_clk_src_onecell_get, &st->clk_data);

	st->version = jesd204b_gt_read(st, ADI_REG_VERSION);

	dev_info(&pdev->dev, "AXI-JESD204B (%d.%.2d.%c) at 0x%08llX mapped to 0x%p,",
		PCORE_VERSION_MAJOR(st->version),
		PCORE_VERSION_MINOR(st->version),
		PCORE_VERSION_LETTER(st->version),
		(unsigned long long)mem->start, st->regs);

	return 0;

err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->size),
			  st->buf_virt, st->buf_phys);
	clk_disable_unprepare(clk);

	return ret;
}

/**
 * jesd204b_gt_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int jesd204b_gt_remove(struct platform_device *pdev)
{
	struct jesd204b_gt_state *st = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&st->sync_work);

	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->size),
			  st->buf_virt, st->buf_phys);

	clk_disable_unprepare(st->clk);
	clk_put(st->clk);

	return 0;
}

static struct platform_driver jesd204b_gt_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_gt_of_match,
	},
	.probe		= jesd204b_gt_probe,
	.remove		= jesd204b_gt_remove,
};

module_platform_driver(jesd204b_gt_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AXI-JESD204B Interface Module");
MODULE_LICENSE("GPL v2");
