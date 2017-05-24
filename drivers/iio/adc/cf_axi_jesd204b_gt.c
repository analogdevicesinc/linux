/*
 * ADI AXI-JESD204B GT Interface Module
 *
 * Copyright 2014-2015 Analog Devices Inc.
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

#define JESD204B_GT_ES_HSIZE_FULL 	65
#define JESD204B_GT_ES_HSIZE_HALF 	129
#define JESD204B_GT_ES_HSIZE_QRTR 	257
#define JESD204B_GT_ES_HSIZE_OCT 	513
#define JESD204B_GT_ES_HSIZE_HEX 	1025

#define JESD204B_GT_ES_VSIZE		255

#define MAX_NUM_LINKS		6

#define for_each_link_of_gt(_st, _i, _gt_link) \
	for (_i = 0, _gt_link = &_st->gt_link[0]; _i < _st->num_links; \
		_i++, _gt_link = &_st->gt_link[_i])

#define for_each_lane_of_link(_st, _gt_link, _lane) \
	for (_lane = _gt_link->first_lane; \
		_lane < ((PCORE_VERSION_MAJOR(_st->version) < 7) ? 1 : \
		_gt_link->first_lane + _gt_link->num_lanes); _lane++)

#define for_each_lane_of_all_links(_st, _i, _gt_link, _lane) \
		for_each_link_of_gt(_st, _i, _gt_link) \
			for_each_lane_of_link(_st, _gt_link, _lane)

struct child_clk {
	struct device 		*dev;
	struct clk_hw		hw;
	struct jesd204b_gt_link *link;
	bool			enabled;
};

enum refclk_ppm {
	PM_200,
	PM_700,
	PM_1250,
};

struct jesd204b_gt_link {
	struct device 	*dev;
	struct clk 	*conv_clk;
	struct clk 	*sysref_clk;
	struct clk 	*lane_rate_div40_clk;
	struct device_node *node;
	struct work_struct work;

	unsigned long 	lane_rate;

	u32		tx_offset;
	u32		first_lane;
	u32		num_lanes;
	u32  		sys_clk_sel;
	u32  		out_clk_sel;
	u32		es_hsize;

	bool		cpll_enable;
	bool 		lpm_enable;
	bool 		sysref_ext_enable;
	bool 		gth_enable;
	bool		encoding;

	u32 		qpll;

	enum refclk_ppm ppm;
};

struct jesd204b_gt_state {
	struct device 		*dev;
	void __iomem		*regs;
	struct clk_onecell_data	clk_data;
	struct child_clk	output[MAX_NUM_LINKS];
	struct jesd204b_gt_link gt_link[MAX_NUM_LINKS];
	struct work_struct 	work;
	struct bin_attribute 	bin;
	struct delayed_work	sync_work;
	struct completion       complete;

	void			*buf_virt;
	dma_addr_t		buf_phys;

	unsigned		num_links;
	int			lane;
	int			es_last_lane;
	int			prescale;
	unsigned		vers_id;
	unsigned		version;
	bool			legacy;

	unsigned		addr;
};

#define to_clk_priv(_hw) container_of(_hw, struct child_clk, hw)

/*
 * IO accessors
 */

static inline unsigned jesd204b_gt_pll_sel(struct jesd204b_gt_link *gt_link)
{
	if (gt_link->cpll_enable)
		return CPLL;
	else if (gt_link->qpll == 0)
		return QPLL0;
	else
		return QPLL1;
}

static inline unsigned int jesd204b_gt_read(struct jesd204b_gt_state *st,
					 unsigned reg)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__,
		 reg, ioread32(st->regs + reg));

	return ioread32(st->regs + reg);
}

static inline void jesd204b_gt_write(struct jesd204b_gt_state *st,
				  unsigned reg, unsigned val)
{
	dev_vdbg(st->dev, "%s: reg 0x%X val 0x%X\n", __func__, reg, val);

	iowrite32(val, st->regs + reg);
}

static struct jesd204b_gt_link* jesd204b_get_rx_link_by_lane(struct jesd204b_gt_state *st, unsigned lane)
{
	struct jesd204b_gt_link *gt_link;
	u32 tmp;

	for_each_link_of_gt(st, tmp, gt_link) {
		if (gt_link->tx_offset)
			continue;
		if (st->lane >= gt_link->first_lane &&
			st->lane < (gt_link->first_lane + gt_link->num_lanes))
			return gt_link;
	}

	return NULL;
}

static unsigned int jesd204b_gt_drp_read(struct jesd204b_gt_state *st, unsigned lane, unsigned dest,
					 unsigned reg)
{
	int timeout = 20;
	unsigned val;

	if (dest)
		lane = 0;

	jesd204b_gt_write(st, JESD204B_GT_REG_DRP_CNTRL(lane) + dest,
		JESD204B_GT_DRP_RWN | JESD204B_GT_DRP_ADDRESS(reg) | JESD204B_GT_DRP_WDATA(0xFFFF));

	do {
		val = jesd204b_gt_read(st, JESD204B_GT_REG_DRP_STATUS(lane) + dest);

		if (val & JESD204B_GT_DRP_STATUS) {
			mdelay(1);
			continue;
		}

		dev_vdbg(st->dev, "%s: lane %d dest 0x%X reg 0x%X val 0x%X\n",
			 __func__, lane, dest, reg, JESD204B_GT_TO_DRP_RDATA(val));

		return JESD204B_GT_TO_DRP_RDATA(val);

	} while (timeout--);

	dev_err(st->dev, "%s: Timeout!", __func__);
	return -ETIMEDOUT;
}

static int jesd204b_gt_drp_write(struct jesd204b_gt_state *st, unsigned lane, unsigned dest,
				  unsigned reg, unsigned val)
{
	int timeout = 20;

	if (dest)
		lane = 0;

	dev_vdbg(st->dev, "%s: lane %d dest 0x%X reg 0x%X val 0x%X\n",
		 __func__, lane, dest, reg, val);

	jesd204b_gt_write(st, JESD204B_GT_REG_DRP_CNTRL(lane) + dest,
			  JESD204B_GT_DRP_ADDRESS(reg) | JESD204B_GT_DRP_WDATA(val));

	do {
		if (!(jesd204b_gt_read(st, JESD204B_GT_REG_DRP_STATUS(lane) + dest) & JESD204B_GT_DRP_STATUS)) {
			if (val != jesd204b_gt_drp_read(st, lane, dest, reg))
					dev_err(st->dev, "%s: MISMATCH lane %d dest 0x%X reg 0x%X val 0x%X\n",
				__func__, lane, dest, reg, val);

			return 0;
		}

		mdelay(1);
	} while (timeout--);

	dev_err(st->dev, "%s: Timeout!", __func__);

	return -ETIMEDOUT;
}

static int __jesd204b_gt_drp_writef(struct jesd204b_gt_state *st, unsigned lane, unsigned dest, u32 reg,
				 u32 mask, u32 offset, u32 val)
{
	u32 tmp;
	int ret;

	if (!mask)
		return -EINVAL;

	ret = jesd204b_gt_drp_read(st, lane, dest, reg);
	if (ret < 0)
		return ret;

	tmp = ret;

	tmp &= ~mask;
	tmp |= ((val << offset) & mask);

	return jesd204b_gt_drp_write(st, lane, dest, reg, tmp);
}

#define jesd204b_gt_drp_writef(st, lane, dest, reg, mask, val) \
	__jesd204b_gt_drp_writef(st, lane, dest, reg, mask, __ffs(mask), val)

static int jesd204b_gt_set_lane(struct jesd204b_gt_state *st, unsigned lane)
{
	if ((PCORE_VERSION_MAJOR(st->version) < 7)) {
		jesd204b_gt_write(st, JESD204B_GT_REG_LANESEL(lane),
			JESD204B_GT_LANESEL(lane));

		jesd204b_gt_read(st, JESD204B_GT_REG_LANESEL(lane));
	}

	return 0;
}

static int jesd204b_gt_set_lpm_dfe_mode(struct jesd204b_gt_state *st,
					unsigned lpm, unsigned lane, unsigned dest)
{
	u32 type;

	type = jesd204b_gt_read(st, JESD204B_GT_REG_TRANSCEIVER_TYPE(lane));

	jesd204b_gt_set_lane(st, lane);

	if (type == JESD204B_GT_TRANSCEIVER_GTH) {
		if (lpm) {
			jesd204b_gt_drp_write(st, lane, dest, 0x036, 0x0032);
			jesd204b_gt_drp_write(st, lane, dest, 0x039, 0x1000);
			jesd204b_gt_drp_write(st, lane, dest, 0x062, 0x1980);
		} else {
			jesd204b_gt_drp_write(st, lane, dest, 0x036, 0x0002);
			jesd204b_gt_drp_write(st, lane, dest, 0x039, 0x0000);
			jesd204b_gt_drp_write(st, lane, dest, 0x062, 0x0000);
		}
	} else {
		if (lpm) {
			jesd204b_gt_drp_write(st, lane, dest, 0x029, 0x0104);
		} else {
			jesd204b_gt_drp_write(st, lane, dest, 0x029, 0x0954);
		}
	}

	return 0;
}

static int jesd204b_gt_es(struct jesd204b_gt_state *st, unsigned lane)
{
	struct jesd204b_gt_link *gt_link = jesd204b_get_rx_link_by_lane(st, lane);
	unsigned stat;

	if (gt_link == NULL)
		return -ENODEV;

	if ((PCORE_VERSION_MAJOR(st->version) < 7)) {
		jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_CNTRL(0),
			JESD204B_GT_EYESCAN_STOP);
		jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_CNTRL(0), 0);
		jesd204b_gt_set_lane(st, lane);
		lane = 0;
	} else {
		jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_CNTRL(st->es_last_lane),
			JESD204B_GT_EYESCAN_STOP);
		jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_CNTRL(st->es_last_lane), 0);
		st->es_last_lane = lane;
	}

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_PRESCALE(lane),
		JESD204B_GT_EYESCAN_PRESCALE(st->prescale));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_VOFFSET(lane),
		JESD204B_GT_EYESCAN_VOFFSET_STEP(1) |
		JESD204B_GT_EYESCAN_VOFFSET_MAX(JESD204B_GT_ES_VSIZE / 2) |
		JESD204B_GT_EYESCAN_VOFFSET_MIN(-1 * (JESD204B_GT_ES_VSIZE / 2)));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_HOFFSET_1(lane),
		JESD204B_GT_EYESCAN_HOFFSET_MAX(gt_link->es_hsize / 2) |
		JESD204B_GT_EYESCAN_HOFFSET_MIN(-1 * (gt_link->es_hsize / 2)));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_HOFFSET_2(lane),
		JESD204B_GT_EYESCAN_HOFFSET_STEP(1));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_DMA_STARTADDR(lane), st->buf_phys);

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_SDATA_1_0(lane),
		JESD204B_GT_EYESCAN_SDATA1(0) |
		JESD204B_GT_EYESCAN_SDATA0(0));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_SDATA_3_2(lane),
		JESD204B_GT_EYESCAN_SDATA3(0xFFFF) |
		JESD204B_GT_EYESCAN_SDATA2(0xFF00));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_SDATA_4(lane),
		JESD204B_GT_EYESCAN_SDATA4(0xFFFF));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_QDATA_1_0(lane),
		JESD204B_GT_EYESCAN_QDATA1(0xFFFF) |
		JESD204B_GT_EYESCAN_QDATA0(0xFFFF));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_QDATA_3_2(lane),
		JESD204B_GT_EYESCAN_QDATA3(0xFFFF) |
		JESD204B_GT_EYESCAN_QDATA2(0xFFFF));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_QDATA_4(lane),
		JESD204B_GT_EYESCAN_QDATA4(0xFFFF));

	jesd204b_gt_write(st, JESD204B_GT_REG_EYESCAN_CNTRL(lane),
		JESD204B_GT_EYESCAN_INIT |
		JESD204B_GT_EYESCAN_START);

	do {
		msleep(50 * ((st->prescale & 0x1F) + 1));
		stat = jesd204b_gt_read(st, JESD204B_GT_REG_EYESCAN_STATUS(lane));
		if (stat & JESD204B_GT_EYESCAN_DMAERR)
			return -EIO;

	} while (stat & JESD204B_GT_EYESCAN_STATUS);

	return 0;
}

static void jesd204b_gt_work_func(struct work_struct *work)
{
	struct jesd204b_gt_state *st =
		container_of(work, struct jesd204b_gt_state, work);
	int ret;

	ret = jesd204b_gt_es(st, st->lane);
	if (ret)
		dev_warn(st->dev, "Eye Scan failed (%d)\n", ret);

	complete_all(&st->complete);
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

	if (ret == 2) {

		if (st->addr & BIT(31))

		jesd204b_gt_write(st, st->addr, val);

	}

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
	struct jesd204b_gt_link *gt_link = jesd204b_get_rx_link_by_lane(st, st->lane);

	if (gt_link)
		return sprintf(buf, "x%d,y%d CDRDW: %d LPM: %d\n",
			gt_link->es_hsize, JESD204B_GT_ES_VSIZE, 40, gt_link->lpm_enable);


	return -EINVAL;
}

static DEVICE_ATTR(info, S_IRUSR, jesd204b_gt_info_read, NULL);

static int jesd204b_gt_status_error(struct device *dev,
				   unsigned offs, unsigned lane, unsigned mask)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);
	unsigned val = jesd204b_gt_read(st, JESD204B_GT_REG_STATUS(lane) + offs);

	if ((val & mask) != mask) {
		dev_err(dev, "%s Error lane-%d: %s%s%s\n",
			offs ? "TX" : "RX", lane,
			(JESD204B_GT_TO_RST_DONE(val) != 0xFF) ? "RESET failed " : "",
			(JESD204B_GT_TO_PLL_LOCKED(val) != 0xFF) ? "PLL Unlocked " : "",
			(JESD204B_GT_STATUS & val) ? "" : "Interface Error"
       		);

		return -EIO;
	}

	return 0;
}

static void jesd204b_gt_sysref(struct jesd204b_gt_state *st,
			struct jesd204b_gt_link *gt_link, unsigned lane)
{
	if (gt_link->sysref_ext_enable) {
		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) +
			gt_link->tx_offset, JESD204B_GT_SYSREF_EXTERNAL);
	} else {
		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) +
			gt_link->tx_offset, JESD204B_GT_SYSREF_ON);
		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) +
			gt_link->tx_offset, JESD204B_GT_SYSREF_OFF);
	}
}

static void jesd204b_gt_clk_synchronize(struct child_clk *clk)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(clk->dev);
	struct jesd204b_gt_link *gt_link = clk->link;
	int ret, lane;

	if (!clk->enabled)
		return;

	for_each_lane_of_link(st, gt_link, lane) {
		ret = jesd204b_gt_read(st, JESD204B_GT_REG_STATUS(lane) + gt_link->tx_offset);
		while (ret != JESD204B_GT_STATUS_SYNC) {
			jesd204b_gt_sysref(st, gt_link, lane);
			msleep(100);
			ret = jesd204b_gt_read(st, JESD204B_GT_REG_STATUS(lane) + gt_link->tx_offset);
			dev_vdbg(st->dev, "Resynchronizing\n");
		}
	}
}

static void jesd204b_gt_sync_work_func(struct work_struct *work)
{
	struct jesd204b_gt_state *st =
		container_of(work, struct jesd204b_gt_state, sync_work.work);
	unsigned int i;

	for (i = 0; i < st->clk_data.clk_num; i++)
		jesd204b_gt_clk_synchronize(&st->output[i]);

	queue_delayed_work(system_freezable_wq, &st->sync_work, HZ);
}

static void jesd204b_gt_link_work_func(struct work_struct *work)
{
	struct jesd204b_gt_link *gt_link =
		container_of(work, struct jesd204b_gt_link, work);
	unsigned long div40_rate;
	int ret;

	if (!IS_ERR(gt_link->lane_rate_div40_clk)) {

		div40_rate = gt_link->lane_rate * (1000 / 40);

		dev_dbg(gt_link->dev, "%s: setting MMCM on %s rate %lu\n",
			__func__, gt_link->tx_offset ? "TX" : "RX", div40_rate);

		if (__clk_is_enabled(gt_link->lane_rate_div40_clk))
			clk_disable_unprepare(gt_link->lane_rate_div40_clk);

		ret = clk_set_rate(gt_link->lane_rate_div40_clk, div40_rate);
		if (ret < 0)
			dev_err(gt_link->dev, "%s: setting MMCM on %s rate %lu failed (%d)\n",
				__func__, gt_link->tx_offset ? "TX" : "RX", div40_rate, ret);

		ret = clk_prepare_enable(gt_link->lane_rate_div40_clk);
		if (ret < 0)
			dev_err(gt_link->dev, "%s: enabling MMCM rate %lu failed (%d)\n",
				__func__, div40_rate, ret);
	}
}

static int jesd204b_gt_clk_enable(struct clk_hw *hw)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(to_clk_priv(hw)->dev);
	struct jesd204b_gt_link *gt_link = to_clk_priv(hw)->link;
	unsigned offs = gt_link->tx_offset;
	int ret;
	unsigned lane;

	dev_dbg(st->dev, "%s: %s fist lane %d number of lanes %d", __func__,
		offs ? "TX" : "RX", gt_link->first_lane, gt_link->num_lanes);

	for_each_lane_of_link(st, gt_link, lane) {
		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + offs, 0); /* resets (drp, pll) */
		jesd204b_gt_write(st, JESD204B_GT_REG_GT_RSTN(lane) + offs, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN(lane) + offs, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_USER_READY(lane) + offs, 0); /* MH */


		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				JESD204B_GT_DRP_RSTN); /* enable (drp) FIXME */

		if (!gt_link->tx_offset) {
			jesd204b_gt_set_lpm_dfe_mode(st, gt_link->lpm_enable, lane,
						     jesd204b_gt_pll_sel(gt_link));
		}

		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				JESD204B_GT_DRP_RSTN | JESD204B_GT_GT_PLL_RSTN); /* enable (drp, pll) */

		jesd204b_gt_write(st, JESD204B_GT_REG_PLL_RSTN(lane) + gt_link->tx_offset,
				JESD204B_GT_PLL_RSTN); /* enable (pll) */
	}

	mdelay(50);

	for_each_lane_of_link(st, gt_link, lane) {
		jesd204b_gt_status_error(to_clk_priv(hw)->dev, offs, lane, JESD204B_GT_PLL_LOCKED(~0));

		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) + offs, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_SYNC_CTL(lane) + offs, JESD204B_GT_SYNC);

		jesd204b_gt_write(st, JESD204B_GT_REG_USER_READY(lane) + offs, JESD204B_GT_USER_READY);
		jesd204b_gt_write(st, JESD204B_GT_REG_GT_RSTN(lane) + offs, JESD204B_GT_GT_RSTN);
		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN(lane) + offs, JESD204B_GT_RSTN);
	}

	mdelay(40);

	for_each_lane_of_link(st, gt_link, lane) {

		jesd204b_gt_status_error(to_clk_priv(hw)->dev, offs, lane,
			JESD204B_GT_RST_DONE(~0) | JESD204B_GT_PLL_LOCKED(~0));

		jesd204b_gt_sysref(st, gt_link, lane);
	}

	mdelay(50);

	ret = 0;
	for_each_lane_of_link(st, gt_link, lane) {
		ret += jesd204b_gt_status_error(to_clk_priv(hw)->dev, offs, lane,
			JESD204B_GT_RST_DONE(~0) | JESD204B_GT_PLL_LOCKED(~0) | JESD204B_GT_STATUS);
	}
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

static long jesd204b_gt_gth_rxcdr_settings(struct jesd204b_gt_state *st,
			struct jesd204b_gt_link *gt_link, unsigned lane,
			u32 rxout_div)
{
	u16 cfg0, cfg1, cfg2, cfg3, cfg4;
	u32 dest;

	if (gt_link->tx_offset) {
		return 0; /* Do Nothing */
	}

	switch (gt_link->ppm) {
	case PM_200:
		cfg0 = 0x0018;
		break;
	case PM_700:
	case PM_1250:
		cfg0 = 0x8018;
		break;
	default:
		return -EINVAL;
	}

	if (gt_link->encoding == ENC_8B10B) {

		cfg1 = 0xC208;
		cfg3 = 0x07FE;
		cfg3 = 0x0020;

		switch (rxout_div) {
		case 0: /* 1 */
			cfg2 = 0x2000;
			break;
		case 1: /* 2 */
			cfg2 = 0x1000;
			break;
		case 2: /* 4 */
			cfg2 = 0x0800;
			break;
		case 3: /* 8 */
			cfg2 = 0x0400;
			break;
		default:
			return -EINVAL;
		}

	} else {
		dev_warn(st->dev, "%s: GTH PRBS CDR not implemented\n",__func__);
	}

	dest = jesd204b_gt_pll_sel(gt_link);

	jesd204b_gt_drp_write(st, lane, dest, RXCDR_CFG0_ADDR, cfg0);
	jesd204b_gt_drp_write(st, lane, dest, RXCDR_CFG1_ADDR, cfg1);
	jesd204b_gt_drp_write(st, lane, dest, RXCDR_CFG2_ADDR, cfg2);
	jesd204b_gt_drp_write(st, lane, dest, RXCDR_CFG3_ADDR, cfg3);
	jesd204b_gt_drp_write(st, lane, dest, RXCDR_CFG4_ADDR, cfg4);

	return 0;
}

static long jesd204b_gt_rxcdr_settings(struct jesd204b_gt_state *st,
			struct jesd204b_gt_link *gt_link, unsigned lane,
			u32 rxout_div)
{
	u16 cfg0, cfg1, cfg2, cfg3, cfg4;

	if (gt_link->tx_offset) {
		return 0; /* Do Nothing */
	}

	if (gt_link->gth_enable)
		return jesd204b_gt_gth_rxcdr_settings(st, gt_link,
						      lane, rxout_div);

	cfg2 = 0x23FF;
	cfg0 = 0x0020;

	switch (gt_link->ppm) {
	case PM_200:
		cfg3 = 0x0000;
		break;
	case PM_700:
	case PM_1250:
		cfg3 = 0x8000;
		break;
	default:
		return -EINVAL;
	}

	if (gt_link->encoding == ENC_8B10B) {

		cfg4 = 0x03;

		switch (rxout_div) {
		case 0: /* 1 */
			cfg1 = 0x1040;
			break;
		case 1: /* 2 */
			cfg1 = 0x1020;
			break;
		case 2: /* 4 */
			cfg1 = 0x1010;
			break;
		case 3: /* 8 */
			cfg1 = 0x1008;
			break;
		default:
			return -EINVAL;
		}

	} else {

		if (gt_link->lane_rate > 6600000 && rxout_div == 1) {
			cfg4 = 0x0B;
		} else {
			cfg4 = 0x03;
		}

		switch (rxout_div) {
		case 0: /* 1 */
			if (gt_link->lpm_enable) {
				if (gt_link->lane_rate  > 6600000) {
					if (gt_link->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x1040;
				} else {
					cfg1 = 0x1020;
				}
			} else { /* DFE */
				if (gt_link->lane_rate  > 6600000) {
					if (gt_link->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x1040;
				} else {
					if (gt_link->ppm == PM_1250)
						cfg1 = 0x1020;
					else
						cfg1 = 0x2040;
				}
			}
			break;
		case 1: /* 2 */
			cfg1 = 0x4020;
			break;
		case 2: /* 4 */
			cfg1 = 0x4010;
			break;
		case 3: /* 8 */
			cfg1 = 0x4008;
			break;
		default:
			return -EINVAL;
		}
	}


	jesd204b_gt_drp_write(st, lane, 0, RXCDR_CFG0_ADDR, cfg0);
	jesd204b_gt_drp_write(st, lane, 0, RXCDR_CFG1_ADDR, cfg1);
	jesd204b_gt_drp_write(st, lane, 0, RXCDR_CFG2_ADDR, cfg2);
	jesd204b_gt_drp_write(st, lane, 0, RXCDR_CFG3_ADDR, cfg3);
	jesd204b_gt_drp_writef(st, lane, 0, RXCDR_CFG4_ADDR, RXCDR_CFG4_MASK, cfg4);

	return 0;
}

static long jesd204b_gt_calc_cpll_settings(struct jesd204b_gt_state *st,
					   unsigned long refclk_kHz,
					   unsigned long laneRate_kHz,
					   u32 *refclk_div, u32 *out_div,
					   u32 *fbdiv_45, u32 *fbdiv)
{
	u32 n1, n2, d, m;
	u32 pllFreq_kHz;

	//possible Xilinx GTX PLL parameters for Virtex 7 CPLL.  Find one that works for the desired laneRate.
	/* Attribute encoding, DRP encoding */
	const u8 _N1[][2] = {{5, 1}, {4, 0}};
	const u8 _N2[][2] = {{5, 3}, {4, 2}, {3, 1}, {2, 0}, {1, 16}};
	const u8  _D[][2] = {{1, 0}, {2, 1}, {4, 2}, {8, 3}};
	const u8  _M[][2] = {{1, 16}, {2, 0}};

	for (m = 0; m < ARRAY_SIZE(_M); m++) {
		for (d = 0; d < ARRAY_SIZE(_D); d++) {
			for (n1 = 0; n1 < ARRAY_SIZE(_N1); n1++) {
				for (n2 = 0; n2 < ARRAY_SIZE(_N2); n2++) {
					pllFreq_kHz = refclk_kHz * _N1[n1][0] * _N2[n2][0] / _M[m][0];

					if ((pllFreq_kHz > 3300000) || (pllFreq_kHz < 1600000)) /* GTH 3.75 GHz */
						continue;

					if ((pllFreq_kHz * 2 / _D[d][0]) == laneRate_kHz) {
						if (refclk_div && out_div && fbdiv_45 && fbdiv) {
							*refclk_div = _M[m][1];
							*out_div = _D[d][1];
							*fbdiv_45 = _N1[n1][1];
							*fbdiv = _N2[n2][1];
						}

						dev_dbg(st->dev, "%s: M %d, D %d, N1 %d, N2 %d\n",
							__func__, _M[m][0], _D[d][0],
							_N1[n1][0], _N2[n2][0]);

						return laneRate_kHz;
					}
				}
			}
		}
	}

	dev_dbg(st->dev, "%s: Failed to find matching dividers for %lu kHz rate\n",
		__func__, laneRate_kHz);

	return -EINVAL;
}

static long jesd204b_gt_calc_qpll_settings(struct jesd204b_gt_state *st,
					   unsigned long refclk_kHz,
					   unsigned long laneRate_kHz,
					   u32 *refclk_div, u32 *out_div,
					   u32 *fbdiv, u32 *fbdiv_ratio,
					   u32 *lowband)
{
	// calculate the FPGA GTX PLL settings M, D, N1, N2
	u32 n, d, m;
	u32 pllVcoFreq_kHz;
	u32 pllOutFreq_kHz;

	//possible Xilinx GTX QPLL parameters for Virtex 7 QPLL.  Find one that works for the desired laneRate.
	/* Attribute encoding, DRP encoding */
	const u16 _N[][2] = {{16, 32}, {20, 48}, {32, 96}, {40, 128},
			     {64, 224}, {66, 320}, {80, 288}, {100, 368}};
	const u8 _D[][2] = {{1, 0}, {2, 1}, {4, 2}, {8, 3}, {16, 4}};
	const u8 _M[][2] = {{1, 16}, {2, 0}, {3, 1}, {4, 2}};
	u8 _lowBand = 0;

	for (m = 0; m < ARRAY_SIZE(_M); m++) {
		for (d = 0; d < ARRAY_SIZE(_D); d++) {
			for (n = 0; n < ARRAY_SIZE(_N); n++) {

				pllVcoFreq_kHz = refclk_kHz * _N[n][0] / _M[m][0];
				pllOutFreq_kHz = pllVcoFreq_kHz / 2;

				if ((pllVcoFreq_kHz >= 5930000) && (pllVcoFreq_kHz <= 8000000)) {
					// low band = 5.93G to 8.0GHz VCO
					_lowBand = 1;
				} else if ((pllVcoFreq_kHz >= 9800000) && (pllVcoFreq_kHz <= 12500000)) {
					//high band = 9.8G to 12.5GHz VCO
					_lowBand = 0;
				} else {
					continue; //if Pll out of range, not valid case, keep trying
				}

				if ((pllOutFreq_kHz * 2 / _D[d][0]) == laneRate_kHz) {
					if (refclk_div && out_div && fbdiv_ratio && fbdiv && lowband) {
						*refclk_div = _M[m][1];
						*out_div = _D[d][1];
						*fbdiv = _N[n][1];
						*fbdiv_ratio = (_N[n][0] == 66) ? 0 : 1;
						*lowband = _lowBand;
					}
						dev_dbg(st->dev, "%s: M %d, D %d, N %d, ratio %d, lowband %d\n",
							__func__, _M[m][0], _D[d][0],
							_N[n][0], (_N[n][0] == 66) ? 0 : 1,
							_lowBand);

					return laneRate_kHz;
				}

			}
		}
	}

	dev_dbg(st->dev, "%s: Failed to find matching dividers for %lu kHz rate\n",
		__func__, laneRate_kHz);

	return -EINVAL;
}

static unsigned long jesd204b_gt_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(to_clk_priv(hw)->dev);
	struct jesd204b_gt_link *gt_link = to_clk_priv(hw)->link;
	unsigned pll_type;

	dev_dbg(st->dev, "%s: Parent Rate %lu Hz, (%s, lane-%d ... lane-%d)",
		__func__, parent_rate, gt_link->tx_offset ? "TX" : "RX", gt_link->first_lane, gt_link->num_lanes);

	if (!IS_ERR(gt_link->lane_rate_div40_clk))
		return gt_link->lane_rate;

	pll_type = jesd204b_gt_pll_sel(gt_link);

	if (gt_link->cpll_enable) {
		unsigned refclk_div_m, out_div, N1, N2, M;
		refclk_div_m = jesd204b_gt_drp_read(st, gt_link->first_lane, pll_type, CPLL_REFCLK_DIV_M_ADDR);
		out_div = jesd204b_gt_drp_read(st, gt_link->first_lane, CPLL, RXOUT_DIV_ADDR);

		switch ((refclk_div_m & CPLL_FB_DIV_45_N1_MASK) >> CPLL_FB_DIV_45_N1_OFFSET) {
			case 0:
				N1 = 4;
				break;
			case 1:
				N1 = 5;
				break;
		}

		switch ((refclk_div_m & CPLL_REFCLK_DIV_M_MASK) >> CPLL_REFCLK_DIV_M_OFFSET) {
			case 0:
				M = 2;
				break;
			case 16:
				M = 1;
				break;
		}

		switch (refclk_div_m & CPLL_FBDIV_N2_MASK) {
			case 3:
				N2 = 5;
				break;
			case 2:
				N2 = 4;
				break;
			case 1:
				N2 = 3;
				break;
			case 0:
				N2 = 2;
				break;
			case 16:
				N2 = 1;
				break;
		}


		if (gt_link->tx_offset)
			out_div = (out_div & TXOUT_DIV_MASK) >> TXOUT_DIV_OFFSET;
		else
			out_div = (out_div & RXOUT_DIV_MASK) >> RXOUT_DIV_OFFSET;

		out_div = (1 << out_div);

		dev_dbg(st->dev, "%s  CPLL %lu   %lu\n", __func__, gt_link->lane_rate,
			((parent_rate / 1000) * N1 * N2 * 2) / (M * out_div));

		dev_dbg(st->dev, "%s  CPLL N1=%d N2=%d M=%d out_div=%d\n", __func__, N1, N2, M, out_div);


		return ((parent_rate / 1000) * N1 * N2 * 2) / (M * out_div);
	} else {
		unsigned refclk_div_m, fb_div, out_div, N, M;
		refclk_div_m = jesd204b_gt_drp_read(st, gt_link->first_lane, pll_type, QPLL_REFCLK_DIV_M_ADDR);
		fb_div = jesd204b_gt_drp_read(st, gt_link->first_lane, pll_type, QPLL_FBDIV_N_ADDR);
		out_div = jesd204b_gt_drp_read(st, gt_link->first_lane, CPLL, RXOUT_DIV_ADDR);

		switch ((refclk_div_m & QPLL_REFCLK_DIV_M_MASK) >> QPLL_REFCLK_DIV_M_OFFSET) {
			case 16:
				M = 1;
				break;
			case 0:
				M = 2;
				break;
			case 1:
				M = 3;
				break;
			case 2:
				M = 4;
				break;

		}

		switch (fb_div & QPLL_FBDIV_N_MASK) {
			case 32:
				N = 16;
				break;
			case 48:
				N = 20;
				break;
			case 96:
				N = 32;
				break;
			case 128:
				N = 40;
				break;
			case 224:
				N = 64;
				break;
			case 320:
				N = 66;
				break;
			case 288:
				N = 80;
				break;
			case 368:
				N = 100;
				break;
		}

		if (gt_link->tx_offset)
			out_div = (out_div & TXOUT_DIV_MASK) >> TXOUT_DIV_OFFSET;
		else
			out_div = (out_div & RXOUT_DIV_MASK) >> RXOUT_DIV_OFFSET;

		out_div = (1 << out_div);

		dev_dbg(st->dev, "%s QPLL  %lu %lu\n", __func__, gt_link->lane_rate,
			((parent_rate / 1000) * N) / (M * out_div));

		dev_dbg(st->dev, "%s QPLL N=%d M=%d out_div=%d\n", __func__, N, M, out_div);

		return ((parent_rate / 1000) * N) / (M * out_div);

	}
}


static long jesd204b_gt_clk_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(to_clk_priv(hw)->dev);
	struct jesd204b_gt_link *gt_link = to_clk_priv(hw)->link;
	int ret;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz",
		__func__, rate, *prate);

	if (gt_link->cpll_enable)
		ret = jesd204b_gt_calc_cpll_settings(st, *prate / 1000, rate,
						      NULL, NULL, NULL, NULL);
	else
		ret = jesd204b_gt_calc_qpll_settings(st, *prate / 1000, rate,
						     NULL, NULL, NULL, NULL,
						     NULL);

	return ret;
}

static int jesd204b_gt_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(to_clk_priv(hw)->dev);
	struct jesd204b_gt_link *gt_link = to_clk_priv(hw)->link;
	unsigned offs = gt_link->tx_offset;
	u32 lane, dest;
	u32 refclk_div, out_div, fbdiv_45, fbdiv, fbdiv_ratio, lowband;
	int ret, pll_done = 0;

	dev_dbg(st->dev, "%s: Rate %lu Hz Parent Rate %lu Hz (%s, lane-%d ... lane-%d)",
		__func__, rate, parent_rate, gt_link->tx_offset ? "TX" : "RX",
		gt_link->first_lane, gt_link->num_lanes);


	if (gt_link->cpll_enable)
		ret = jesd204b_gt_calc_cpll_settings(st, parent_rate / 1000, rate,
						     &refclk_div, &out_div,
						     &fbdiv_45, &fbdiv);
	else
		ret = jesd204b_gt_calc_qpll_settings(st, parent_rate / 1000, rate,
						     &refclk_div, &out_div,
						     &fbdiv, &fbdiv_ratio,
						     &lowband);
	if (ret < 0)
		return ret;

	gt_link->lane_rate = rate;

	for_each_lane_of_link(st, gt_link, lane) {

		jesd204b_gt_write(st, JESD204B_GT_REG_USER_READY(lane) + offs, 0); /* MH */

		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				  2); /* resets (pll) */
		jesd204b_gt_write(st, JESD204B_GT_REG_PLL_RSTN(lane) + gt_link->tx_offset,
				  0); /* resets (pll) */

		dest = jesd204b_gt_pll_sel(gt_link);

		if (gt_link->cpll_enable) {
			jesd204b_gt_drp_writef(st, lane, dest, CPLL_REFCLK_DIV_M_ADDR,
				CPLL_REFCLK_DIV_M_MASK |
				CPLL_FB_DIV_45_N1_MASK |
				CPLL_FBDIV_N2_MASK,
				(refclk_div << 8) | (fbdiv_45 << 7) | fbdiv);
		} else {

			if (!pll_done) {
				jesd204b_gt_drp_writef(st, lane, dest, QPLL_CFG0_ADDR,
					QPLL_CFG0_BAND_MASK, lowband);

				jesd204b_gt_drp_writef(st, lane, dest, QPLL_REFCLK_DIV_M_ADDR,
					QPLL_REFCLK_DIV_M_MASK, refclk_div);

				jesd204b_gt_drp_writef(st, lane, dest, QPLL_FBDIV_N_ADDR,
					QPLL_FBDIV_N_MASK, fbdiv);

				jesd204b_gt_drp_writef(st, lane, dest, QPLL_FBDIV_RATIO_ADDR,
					QPLL_FBDIV_RATIO_MASK, fbdiv_ratio);

				pll_done = 1;
			}
		}

		ret = jesd204b_gt_drp_writef(st, lane, CPLL, RXOUT_DIV_ADDR,
				offs ? TXOUT_DIV_MASK : RXOUT_DIV_MASK, out_div);

		jesd204b_gt_rxcdr_settings(st, gt_link, lane, out_div);

		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				  JESD204B_GT_GT_PLL_RSTN | JESD204B_GT_GT_PLL_RSTN); /* resets (pll) */
		jesd204b_gt_write(st, JESD204B_GT_REG_PLL_RSTN(lane) + gt_link->tx_offset,
				  JESD204B_GT_PLL_RSTN); /* resets (pll) */
	}

	if (!IS_ERR(gt_link->lane_rate_div40_clk))
		schedule_work(&gt_link->work);

	return ret;
}

static const struct clk_ops clkout_ops = {
	.recalc_rate = jesd204b_gt_clk_recalc_rate,
	.enable = jesd204b_gt_clk_enable,
	.disable = jesd204b_gt_clk_disable,
	.is_enabled = jesd204b_gt_clk_is_enabled,
	.round_rate = jesd204b_gt_clk_round_rate,
	.set_rate = jesd204b_gt_clk_set_rate,

};

static struct clk *jesd204b_gt_clk_register(struct device *dev, struct device_node *node,
					    const char *parent_name, unsigned num,
					    struct jesd204b_gt_link *gt_link)
{
	struct jesd204b_gt_state *st = dev_get_drvdata(dev);
	struct clk_init_data init;
	struct child_clk *output = &st->output[num];
	struct clk *clk;
	const char *clk_name;
	int ret;

	ret = of_property_read_string_index(node, "clock-output-names",
		st->legacy ? num : 0, &clk_name);
	if (ret < 0)
		return ERR_PTR(ret);

	init.name = clk_name;
	init.ops = &clkout_ops;
	init.flags = 0;

	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents = (parent_name ? 1 : 0);

	output->hw.init = &init;
	output->dev = dev;
	output->link = gt_link;

	/* register the clock */
	clk = clk_register(dev, &output->hw);
	st->clk_data.clks[num] = clk;

	return clk;
}

static int jesd204b_gt_parse_link_node(struct jesd204b_gt_link *gt_link,
				       struct device_node *np)
{
	int ret;

	gt_link->conv_clk = of_clk_get_by_name(np, "conv");
	if (IS_ERR(gt_link->conv_clk)) {
		return -EPROBE_DEFER;
	}
	ret = clk_prepare_enable(gt_link->conv_clk);
	if (ret < 0)
		return ret;

	gt_link->sysref_clk = of_clk_get_by_name(np, "sysref");
	if (!IS_ERR(gt_link->sysref_clk)) {
		ret = clk_prepare_enable(gt_link->sysref_clk);
		if (ret < 0)
			return ret;
	}

	gt_link->lane_rate_div40_clk = of_clk_get_by_name(np, "div40");

	of_property_read_u32(np, "adi,lanes", &gt_link->num_lanes);
	of_property_read_u32(np, "adi,first-lane", &gt_link->first_lane);

	if (of_property_read_bool(np, "adi,link-is-transmit-enable"))
		gt_link->tx_offset = JESD204B_GT_REG_TX_OFFSET;
	else
		gt_link->tx_offset = 0;

	of_property_read_u32(np, "adi,sys-clk-select",
				&gt_link->sys_clk_sel);
	of_property_read_u32(np, "adi,out-clk-select",
				&gt_link->out_clk_sel);

	gt_link->cpll_enable = of_property_read_bool(np,
				"adi,use-cpll-enable");
	gt_link->lpm_enable = of_property_read_bool(np,
				"adi,use-lpm-enable");
	gt_link->sysref_ext_enable = of_property_read_bool(np,
				"adi,sysref-external-enable");
	gt_link->node = np;

	gt_link->encoding = ENC_8B10B;
	gt_link->ppm = PM_200; /* TODO use clock accuracy */

	INIT_WORK(&gt_link->work, jesd204b_gt_link_work_func);

	return 0;
}

static void jesd204b_gt_disable_unprepare_clocks(struct jesd204b_gt_state *st)
{
	struct jesd204b_gt_link *gt_link;
	int i;

	for_each_link_of_gt(st, i, gt_link) {
		if (!IS_ERR(gt_link->conv_clk))
			clk_disable_unprepare(gt_link->conv_clk);

		if (!IS_ERR(gt_link->sysref_clk))
			clk_disable_unprepare(gt_link->sysref_clk);
	}
}

static void jesd204b_gt_unregister_clocks(struct jesd204b_gt_state *st)
{
	int i;

	for (i = 0; i <	st->clk_data.clk_num; i++) {
		if (!IS_ERR(st->clk_data.clks[i]))
			clk_unregister(st->clk_data.clks[i]);
	}
}

static void jesd204b_gt_unregister_clock_provider(struct platform_device *pdev)
{
	struct jesd204b_gt_state *st = platform_get_drvdata(pdev);

	if (st->legacy) {
		of_clk_del_provider(pdev->dev.of_node);
	} else {
		struct device_node *np;
		for_each_child_of_node(pdev->dev.of_node, np) {
			of_clk_del_provider(np);
		}
	}
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
	struct device_node *np = pdev->dev.of_node, *link_np;
	struct jesd204b_gt_state *st;
	struct resource *mem; /* IO mem resources */
	struct jesd204b_gt_link *gt_link;
	int ret;
	u32 lane, tmp;

	const struct of_device_id *of_id =
			of_match_device(jesd204b_gt_of_match, &pdev->dev);

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	ret = of_get_child_count(np);
	if (ret > MAX_NUM_LINKS)
		return -EINVAL;

	if (ret == 0) { /* Flat - for backward compatibility */
		st->legacy = true;
		st->gt_link[st->num_links].conv_clk = devm_clk_get(&pdev->dev, "adc_clk");
		if (IS_ERR(st->gt_link[st->num_links].conv_clk)) {
			return -EPROBE_DEFER;
		}

		ret = clk_prepare_enable(st->gt_link[st->num_links].conv_clk);
		if (ret < 0)
			return ret;

		st->gt_link[st->num_links].sysref_clk = devm_clk_get(&pdev->dev, "adc_sysref");
		if (!IS_ERR(st->gt_link[st->num_links].sysref_clk))
			clk_prepare_enable(st->gt_link[st->num_links].sysref_clk);

		ret = of_property_read_u32(np, "adi,rx-sys-clk-select",
					&st->gt_link[st->num_links].sys_clk_sel);
		ret += of_property_read_u32(np, "adi,rx-out-clk-select",
					&st->gt_link[st->num_links].out_clk_sel);

		if (ret == 0 && !IS_ERR(st->gt_link[st->num_links].conv_clk))
			st->num_links++;

		/* TX */

		st->gt_link[st->num_links].conv_clk = devm_clk_get(&pdev->dev, "dac_clk");
		if (!IS_ERR(st->gt_link[st->num_links].conv_clk))
			clk_prepare_enable(st->gt_link[st->num_links].conv_clk);

		st->gt_link[st->num_links].sysref_clk = devm_clk_get(&pdev->dev, "dac_sysref");
		if (!IS_ERR(st->gt_link[st->num_links].sysref_clk))
			clk_prepare_enable(st->gt_link[st->num_links].sysref_clk);

		ret = of_property_read_u32(np, "adi,tx-sys-clk-select",
					&st->gt_link[st->num_links].sys_clk_sel);
		ret += of_property_read_u32(np, "adi,tx-out-clk-select",
					&st->gt_link[st->num_links].out_clk_sel);

		if (ret == 0 && !IS_ERR(st->gt_link[st->num_links].conv_clk)) {
			st->gt_link[st->num_links].tx_offset = JESD204B_GT_REG_TX_OFFSET;
			st->num_links++;
		}

		/* Common */

		st->gt_link[0].cpll_enable = of_property_read_bool(np, "adi,use-cpll-enable");
		st->gt_link[0].lpm_enable = of_property_read_bool(np, "adi,use-lpm-enable");
		st->gt_link[0].sysref_ext_enable = of_property_read_bool(np, "adi,sysref-external-enable");

		of_property_read_u32(np, "adi,lanes", &st->gt_link[0].num_lanes);
		st->gt_link[0].node = np;

		st->gt_link[0].encoding = ENC_8B10B;
		st->gt_link[0].ppm = PM_200;

		if (st->num_links > 1) {
			st->gt_link[1].cpll_enable = st->gt_link[0].cpll_enable;
			st->gt_link[1].lpm_enable = st->gt_link[0].lpm_enable;
			st->gt_link[1].num_lanes = st->gt_link[0].num_lanes;
			st->gt_link[1].sysref_ext_enable = st->gt_link[0].sysref_ext_enable;
			st->gt_link[1].node = np;
			st->gt_link[1].encoding = ENC_8B10B;
			st->gt_link[1].ppm = PM_200;
		}

	} else {
		for_each_child_of_node(np, link_np) {
			ret = jesd204b_gt_parse_link_node(&st->gt_link[st->num_links++], link_np);
			if (ret < 0)
				goto disable_unprepare;


		}
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs)) {
		ret = PTR_ERR(st->regs);
		goto disable_unprepare;
	}

	st->dev = &pdev->dev;
	st->version = jesd204b_gt_read(st, JESD204B_GT_REG_VERSION);
	platform_set_drvdata(pdev, st);

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;

	for_each_lane_of_all_links(st, tmp, gt_link, lane) {
		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset, 0); /* resets (drp, pll) */
		jesd204b_gt_write(st, JESD204B_GT_REG_GT_RSTN(lane) + gt_link->tx_offset, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN(lane) + gt_link->tx_offset, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_USER_READY(lane) + gt_link->tx_offset, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) + gt_link->tx_offset, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_SYNC_CTL(lane) + gt_link->tx_offset, 0);

		jesd204b_gt_write(st, JESD204B_GT_REG_LPM_CPLL_PD(lane) + gt_link->tx_offset,
				(gt_link->cpll_enable ? 0 : JESD204B_GT_CPLL_PD) |
				JESD204B_GT_LPM_DFE(gt_link->lpm_enable));

		jesd204b_gt_write(st, JESD204B_GT_REG_CLK_SEL(lane) + gt_link->tx_offset,
				JESD204B_GT_SYS_CLK_SEL(gt_link->sys_clk_sel) |
				JESD204B_GT_OUT_CLK_SEL(gt_link->out_clk_sel));

		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				JESD204B_GT_DRP_RSTN); /* enable (drp) */

		if (!gt_link->tx_offset) {
			jesd204b_gt_set_lpm_dfe_mode(st, gt_link->lpm_enable, lane,
						     jesd204b_gt_pll_sel(gt_link));
		}

		jesd204b_gt_write(st, JESD204B_GT_REG_RSTN_1(lane) + gt_link->tx_offset,
				JESD204B_GT_DRP_RSTN | JESD204B_GT_GT_PLL_RSTN); /* enable (drp, pll) */

		jesd204b_gt_write(st, JESD204B_GT_REG_PLL_RSTN(lane) + gt_link->tx_offset,
				JESD204B_GT_PLL_RSTN); /* enable (pll) */

		gt_link->es_hsize = jesd204b_gt_read(st, JESD204B_GT_REG_EYESCAN_RATE(0));

		switch (gt_link->es_hsize) {
		case 0x1:
			gt_link->es_hsize = JESD204B_GT_ES_HSIZE_FULL;
			break;
		case 0x2:
			gt_link->es_hsize = JESD204B_GT_ES_HSIZE_HALF;
			break;
		case 0x4:
			gt_link->es_hsize = JESD204B_GT_ES_HSIZE_QRTR;
			break;
		case 0x8:
			gt_link->es_hsize = JESD204B_GT_ES_HSIZE_OCT;
			break;
		case 0x10:
			gt_link->es_hsize = JESD204B_GT_ES_HSIZE_HEX;
			break;
		default:
			ret = -EINVAL;
			dev_err(&pdev->dev, "Failed get EYESCAN_RATE/RXOUT_DIV\n");
			goto disable_unprepare;
		}
	}

	mdelay(10);

	for_each_lane_of_all_links(st, tmp, gt_link, lane) {
		ret = jesd204b_gt_read(st, JESD204B_GT_REG_STATUS(lane) + gt_link->tx_offset);

		if (JESD204B_GT_TO_PLL_LOCKED(ret) != JESD204B_GT_STATUS_PLL_LOCKED)
			dev_warn(&pdev->dev, "%s PLL NOT locked! (0x%X)\n", gt_link->tx_offset ? "TX" : "RX", ret);

		jesd204b_gt_write(st, JESD204B_GT_REG_SYSREF_CTL(lane) + gt_link->tx_offset, 0);
		jesd204b_gt_write(st, JESD204B_GT_REG_SYNC_CTL(lane) + gt_link->tx_offset, JESD204B_GT_SYNC);
		jesd204b_gt_write(st, JESD204B_GT_REG_USER_READY(lane) + gt_link->tx_offset, JESD204B_GT_USER_READY);

	}

	st->prescale = 0;

	sysfs_bin_attr_init(&st->bin);
	st->bin.attr.name = "eye_data";
	st->bin.attr.mode = S_IRUSR;
	st->bin.read = jesd204b_gt_bin_read;
	st->bin.size = JESD204B_GT_ES_HSIZE_HEX * JESD204B_GT_ES_VSIZE * sizeof(u64);

	st->buf_virt = dma_alloc_coherent(&pdev->dev, PAGE_ALIGN(st->bin.size),
					  &st->buf_phys, GFP_KERNEL);

	if (st->buf_virt == NULL) {
		dev_err(&pdev->dev, "Not enough dma memory for device\n");
		ret = -ENOMEM;
		goto disable_unprepare;
	}

	memset(st->buf_virt, 0, PAGE_ALIGN(st->bin.size));

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
					 MAX_NUM_LINKS, GFP_KERNEL);
	if (!st->clk_data.clks) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		ret = -ENOMEM;
		goto remove_sys_files;
	}
	st->clk_data.clk_num = 0;

	for_each_link_of_gt(st, tmp, gt_link) {
		struct clk *clk;

		gt_link->dev = &pdev->dev;

		clk = jesd204b_gt_clk_register(&pdev->dev, gt_link->node,
					       __clk_get_name(gt_link->conv_clk),
					       st->clk_data.clk_num, gt_link);
		if (IS_ERR(clk)) {
			ret = PTR_ERR(clk);
			goto unregister_clocks;
		}
		st->clk_data.clk_num++;
	}

	if (st->legacy) {
		of_clk_add_provider(np, of_clk_src_onecell_get, &st->clk_data);
	} else {
		for_each_link_of_gt(st, tmp, gt_link) {
	        if (!IS_ERR(st->clk_data.clks[tmp]))
			of_clk_add_provider(gt_link->node, of_clk_src_simple_get,
					    st->clk_data.clks[tmp]);
		}
	}

	dev_info(&pdev->dev, "AXI-JESD204B (%d.%.2d.%c) at 0x%08llX mapped to 0x%p,",
		PCORE_VERSION_MAJOR(st->version),
		PCORE_VERSION_MINOR(st->version),
		PCORE_VERSION_LETTER(st->version),
		(unsigned long long)mem->start, st->regs);

	if (!of_property_read_bool(np,"adi,no-auto-resync-enable"))
		schedule_delayed_work(&st->sync_work, HZ);

	return 0;

unregister_clocks:
	jesd204b_gt_unregister_clocks(st);

remove_sys_files:
	sysfs_remove_bin_file(&pdev->dev.kobj, &st->bin);
	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_prescale);
	device_remove_file(&pdev->dev, &dev_attr_reg_access);
	device_remove_file(&pdev->dev, &dev_attr_info);

err_dma_free:
	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->bin.size),
			  st->buf_virt, st->buf_phys);

disable_unprepare:
	jesd204b_gt_disable_unprepare_clocks(st);

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

	sysfs_remove_bin_file(&pdev->dev.kobj, &st->bin);
	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_prescale);
	device_remove_file(&pdev->dev, &dev_attr_reg_access);
	device_remove_file(&pdev->dev, &dev_attr_info);

	dma_free_coherent(&pdev->dev, PAGE_ALIGN(st->bin.size),
			  st->buf_virt, st->buf_phys);

	jesd204b_gt_unregister_clock_provider(pdev);
	jesd204b_gt_unregister_clocks(st);
	jesd204b_gt_disable_unprepare_clocks(st);

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
