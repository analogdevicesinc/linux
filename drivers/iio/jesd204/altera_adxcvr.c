/*
 * Altera ADXCVR Configuration Driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/jesd204/jesd204.h>

/* Registers Description */

/* ADXCVR Registers */

#define ADXCVR_REG_VERSION			0x0000
#define VERSION(major, minor, letter)		((major << 16) | (minor << 8) | letter)
#define VERSION_MAJOR(version)			(version >> 16)
#define VERSION_MINOR(version)			((version >> 8) & 0xff)
#define VERSION_LETTER(version)			(version & 0xff)

#define ADXCVR_REG_ID				0x0004

#define ADXCVR_REG_SCRATCH			0x0008

#define ADXCVR_REG_RESETN			0x0010
#define ADXCVR_RESETN				(1 << 0)

#define ADXCVR_REG_STATUS			0x0014
#define ADXCVR_STATUS				(1 << 0)

#define ADXCVR_REG_STATUS2			0x0018
#define ADXCVR_STATUS2_XCVR(x)			BIT(x)

#define ADXCVR_REG_SYNTH_CONF			0x0024

/* XCVR Registers */

#define XCVR_REG_ARBITRATION			0x000
#define XCVR_ARBITRATION_MASK			0xFF
#define XCVR_ARBITRATION_GET_AVMM		0x02
#define XCVR_ARBITRATION_RELEASE_AVMM_CALIB	0x01
#define XCVR_ARBITRATION_RELEASE_AVMM		0x03

#define XCVR_REG_CALIB_ATX_PLL_EN		0x100
#define XCVR_CALIB_ATX_PLL_EN_MASK		0x01
#define XCVR_CALIB_ATX_PLL_EN			0x01

#define XCVR_REG_CAPAB_ATX_PLL_STAT		0x280
#define XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK		0x02
#define XCVR_CAPAB_ATX_PLL_CAL_DONE		0x00



#define XCVR_REG_CALIB_PMA_EN			0x100
#define XCVR_CALIB_TX_TERM_VOD_MASK		0x20
#define XCVR_CALIB_TX_TERM_VOD_EN		0x20
#define XCVR_CALIB_CMU_CDR_PLL_EN_MASK		0x02
#define XCVR_CALIB_CMU_CDR_PLL_EN		0x02

#define XCVR_REG_CAPAB_PMA			0x281
#define XCVR_CAPAB_RX_CAL_BUSY_EN_MASK		0x20
#define XCVR_CAPAB_RX_CAL_BUSY_EN		0x20
#define XCVR_CAPAB_RX_CAL_BUSY_DIS		0x00
#define XCVR_CAPAB_RX_CAL_BUSY_MASK		0x02
#define XCVR_CAPAB_RX_CAL_DONE			0x00
#define XCVR_CAPAB_TX_CAL_BUSY_EN_MASK		0x10
#define XCVR_CAPAB_TX_CAL_BUSY_EN		0x10
#define XCVR_CAPAB_TX_CAL_BUSY_DIS		0x00
#define XCVR_CAPAB_TX_CAL_BUSY_MASK		0x01
#define XCVR_CAPAB_TX_CAL_DONE			0x00

#define XCVR_REG_RATE_SWITCH_FLAG		0x166
#define XCVR_RATE_SWITCH_FLAG_MASK		0x80
#define XCVR_RATE_SWITCH_FLAG_RATE_SWITCH	0x00
#define XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH	0x80

static DEFINE_MUTEX(adxcfg_global_lock);

struct adxcvr_state {
	struct device		*dev;
	void __iomem		*adxcvr_regs;
	void __iomem		*atx_pll_regs;
	void __iomem		*adxcfg_regs[32];
	struct jesd204_dev	*jdev;
	unsigned int 		version;
	bool			is_transmit;
	u32			lanes_per_link;

	unsigned int reset_counter;

	struct clk *ref_clk;

	struct clk_hw lane_clk;
	bool initial_recalc;

	unsigned int lane_rate;
	struct clk *link_clk;
	struct work_struct link_clk_work;
};

static void adxcvr_write(struct adxcvr_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->adxcvr_regs + reg);
}

static unsigned int adxcvr_read(struct adxcvr_state *st, unsigned reg)
{
	return ioread32(st->adxcvr_regs + reg);
}

static void adxcvr_acquire_arbitration(struct adxcvr_state *st,
	void __iomem *addr, unsigned int status_reg)
{
	unsigned int timeout = 0;
	unsigned int val;

	iowrite32(XCVR_ARBITRATION_GET_AVMM, addr);

	do {
		val = ioread32(addr + status_reg * 4);
		if ((val & BIT(2)) == 0) {
			dev_dbg(st->dev, "Acquired arbitration: %d us\n",
				timeout * 10);
			return;
		}
		udelay(10);
	} while (timeout++ < 10000);

	dev_err(st->dev, "Failed to acquire arbitration\n");
}

static void adxcvr_release_arbitration(void __iomem *addr, bool calibrate)
{
	unsigned int val;

	if (calibrate)
		val = XCVR_ARBITRATION_RELEASE_AVMM_CALIB;
	else
		val = XCVR_ARBITRATION_RELEASE_AVMM;
	iowrite32(val, addr);
}

static void atx_pll_write(struct adxcvr_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->atx_pll_regs + reg * 4);
}

static unsigned int atx_pll_read(struct adxcvr_state *st, unsigned reg)
{
	return ioread32(st->atx_pll_regs + reg * 4);
}

static void atx_pll_update(struct adxcvr_state *st, unsigned int reg,
	unsigned int mask, unsigned int val)
{
	unsigned int rval;

	rval = atx_pll_read(st, reg);
	rval &= ~mask;
	rval |= val;
	atx_pll_write(st, reg, rval);
}

static void atx_pll_acquire_arbitration(struct adxcvr_state *st)
{
	adxcvr_acquire_arbitration(st, st->atx_pll_regs,
		XCVR_REG_CAPAB_ATX_PLL_STAT);
}

static void atx_pll_release_arbitration(struct adxcvr_state *st,
	bool calibrate)
{
	adxcvr_release_arbitration(st->atx_pll_regs, calibrate);
}

static void adxcfg_write(struct adxcvr_state *st, unsigned lane, unsigned reg,
	unsigned val)
{
	iowrite32(val, st->adxcfg_regs[lane] + reg * 4);
}

static unsigned int adxcfg_read(struct adxcvr_state *st, unsigned lane,
	unsigned reg)
{
	return ioread32(st->adxcfg_regs[lane] + reg * 4);
}

static void adxcfg_update(struct adxcvr_state *st, unsigned int lane,
	unsigned int reg, unsigned int mask, unsigned int val)
{
	unsigned int rval;

	rval = adxcfg_read(st, lane, reg);
	rval &= ~mask;
	rval |= val;
	adxcfg_write(st, lane, reg, rval);
}

static void adxcfg_acquire_arbitration(struct adxcvr_state *st,
	unsigned int lane)
{
	adxcvr_acquire_arbitration(st, st->adxcfg_regs[lane],
		XCVR_REG_CAPAB_PMA);
}

static void adxcfg_release_arbitration(struct adxcvr_state *st,
	unsigned int lane, bool calibrate)
{
	adxcvr_release_arbitration(st->adxcfg_regs[lane], calibrate);
}

static void adxcfg_lock(struct adxcvr_state *st)
{
	mutex_lock(&adxcfg_global_lock);
}

static void adxcfg_unlock(struct adxcvr_state *st)
{
	mutex_unlock(&adxcfg_global_lock);
}

static int atx_pll_calibration_check(struct adxcvr_state *st)
{
	unsigned int timeout = 0;
	unsigned int val;

	/* Wait max 100ms for cal_busy to de-assert */
	do {
		mdelay(10);

		/* Read ATX PLL calibration status from capability register */
		val = atx_pll_read(st, XCVR_REG_CAPAB_ATX_PLL_STAT);
		if ((val & XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK) ==
				XCVR_CAPAB_ATX_PLL_CAL_DONE) {
			dev_info(st->dev, "ATX PLL calibration OK (%d ms)\n",
				timeout * 10);
			return 0;
		}
	} while (timeout++ < 10);

	dev_err(st->dev, "ATX PLL calibration FAILED\n");

	return 1;
}

static int adxcfg_calibration_check(struct adxcvr_state *st, unsigned int lane,
	bool tx)
{
	unsigned int timeout = 0;
	unsigned int mask;
	unsigned int val;
	const char *msg;

	if (tx) {
		mask = XCVR_CAPAB_TX_CAL_BUSY_MASK;
		msg = "TX termination and VOD calibration";
	} else {
		mask = XCVR_CAPAB_RX_CAL_BUSY_MASK;
		msg = "CDR/CMU PLL & RX offset calibration";
	}

	/* Wait max 100ms for cal_busy to de-assert */
	do {
		udelay(100);

		/* Read PMA calibration status from capability register */
		val = adxcfg_read(st, lane, XCVR_REG_CAPAB_PMA);
		if ((val & mask) == 0) {
			dev_info(st->dev, "Lane %d %s OK (%d us)\n", lane, msg,
				timeout * 100);
			return 0;
		}
	} while (timeout++ < 1000);

	dev_err(st->dev, "Lane %d %s FAILED\n", lane, msg);

	return 1;
}

static int xcvr_calib_tx(struct adxcvr_state *st)
{
	unsigned lane;
	unsigned err = 0;

	for (lane = 0; lane < st->lanes_per_link; lane++) {
		adxcfg_acquire_arbitration(st, lane);

		/* Perform TX termination & Vod calibration through
		   PMA calibration enable register */
		adxcfg_update(st, lane, XCVR_REG_CALIB_PMA_EN,
			XCVR_CALIB_TX_TERM_VOD_MASK,
			XCVR_CALIB_TX_TERM_VOD_EN);

		/* Disable rx_cal_busy and enable tx_cal_busy output through
		   capability register */
		adxcfg_update(st, lane, XCVR_REG_CAPAB_PMA,
			XCVR_CAPAB_RX_CAL_BUSY_EN_MASK |
			XCVR_CAPAB_TX_CAL_BUSY_EN_MASK,
			XCVR_CAPAB_RX_CAL_BUSY_DIS |
			XCVR_CAPAB_TX_CAL_BUSY_EN);

		adxcfg_release_arbitration(st, lane, true);

		err |= adxcfg_calibration_check(st, lane, true);
	}

	return err;
}

static ssize_t adxcvr_sysfs_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf);
static ssize_t adxcvr_sysfs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count);

static DEVICE_ATTR(adxcvr_resetn, S_IWUSR | S_IRUGO,
		   adxcvr_sysfs_show, adxcvr_sysfs_store);
static DEVICE_ATTR(adxcvr_status, S_IRUGO,
		   adxcvr_sysfs_show, NULL);

static struct attribute *adxcvr_sysfs_attrs[] = {
	&dev_attr_adxcvr_resetn.attr,
	&dev_attr_adxcvr_status.attr,
	NULL
};

static const struct attribute_group adxcvr_sysfs_group = {
	.attrs = adxcvr_sysfs_attrs,
};

static ssize_t adxcvr_sysfs_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned val = 0;

	if (attr == &dev_attr_adxcvr_resetn)
		reg = ADXCVR_REG_RESETN;
	if (attr == &dev_attr_adxcvr_status)
		reg = ADXCVR_REG_STATUS;
	if (reg == -1)
		return -EINVAL;

	val = adxcvr_read(st, reg);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t adxcvr_sysfs_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct adxcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (attr == &dev_attr_adxcvr_resetn)
		reg = ADXCVR_REG_RESETN;
	if (reg == -1)
		return -EINVAL;

	adxcvr_write(st, reg, val);

	return count;
}

static void adxcvr_pre_lane_rate_change(struct adxcvr_state *st)
{
	adxcfg_lock(st);
	/*
	 * Multiple re-configuration requests can be active at the same time.
	 * Make sure that the reset stays asserted until all of them have
	 * completed.
	 */
	if (st->reset_counter++ == 0)
		adxcvr_write(st, ADXCVR_REG_RESETN, 0);
}

static void adxcvr_finalize_lane_rate_change(struct adxcvr_state *st)
{
	unsigned int status;
	int timeout = 1000;
	unsigned int i;

	if (--st->reset_counter != 0)
		return;

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);
	do {
		status = adxcvr_read(st, ADXCVR_REG_STATUS);
		if (status == ADXCVR_STATUS)
			break;
		mdelay(1);
	} while (timeout--);

	if (timeout < 0) {
		status = adxcvr_read(st, ADXCVR_REG_STATUS2);
		dev_err(st->dev, "Link activation error:\n");
		dev_err(st->dev, "\tLink PLL %slocked\n",
			(status & ADXCVR_STATUS2_XCVR(st->lanes_per_link)) ?
				"" : "not ");
		for (i = 0; i < st->lanes_per_link; i++) {
			dev_err(st->dev, "\tLane %d transceiver %sready\n", i,
				(status & ADXCVR_STATUS2_XCVR(i)) ?
					"" : "not ");
		}
	}
}

static void adxcvr_link_clk_work(struct work_struct *work)
{
	struct adxcvr_state *st =
		container_of(work, struct adxcvr_state, link_clk_work);
	unsigned int link_rate;
	int ret;

	link_rate = READ_ONCE(st->lane_rate) * (1000 / 40);

	clk_disable_unprepare(st->link_clk);

	/*
	 * Due to rounding errors link_rate might not contain the exact rate.
	 * Using clk_round_rate() to compute the exact rate first before calling
	 * clk_set_rate() will make sure that clk_set_rate() will not attempt to
	 * re-configure the clock if already has the correct rate.
	 */
	link_rate = clk_round_rate(st->link_clk, link_rate);

	dev_info(st->dev, "Setting link rate to %u (lane rate: %u)\n",
		link_rate, st->lane_rate);

	ret = clk_set_rate(st->link_clk, link_rate);
	if (ret < 0)
		dev_err(st->dev, "Setting link rate failed: %d\n", ret);

	ret = clk_prepare_enable(st->link_clk);
	if (ret < 0)
		dev_err(st->dev, "Enabling link clock failed: %d\n", ret);

	adxcfg_lock(st);
	adxcvr_finalize_lane_rate_change(st);
	adxcfg_unlock(st);
}

static void adxcvr_post_lane_rate_change(struct adxcvr_state *st,
	unsigned int lane_rate)
{
	bool changed = st->lane_rate != lane_rate;

	st->lane_rate = lane_rate;

	/*
	 * Can't change the clock rate of another clock from within the
	 * set_rate callback. So we have to defer to a work item to change the
	 * link clock.
	 */
	if (changed && lane_rate && !IS_ERR(st->link_clk))
		schedule_work(&st->link_clk_work);
	else
		adxcvr_finalize_lane_rate_change(st);

	adxcfg_unlock(st);
}

static struct adxcvr_state *clk_hw_to_adxcvr(struct clk_hw *clk_hw)
{
	return container_of(clk_hw, struct adxcvr_state, lane_clk);
}

#include "altera_a10_atx_pll.c"
#include "altera_a10_cdr_pll.c"

static int adxcvr_register_lane_clk(struct adxcvr_state *st)
{
	struct clk_init_data init;
	const char *parent_name;
	const char *clk_name;
	struct clk *clk;

	st->initial_recalc = true;

	parent_name = of_clk_get_parent_name(st->dev->of_node, 0);
	if (!parent_name)
		return -EINVAL;

	clk_name = st->dev->of_node->name;
	of_property_read_string(st->dev->of_node, "clock-output-names",
		&clk_name);

	init.name = clk_name;
	if (st->atx_pll_regs)
		init.ops = &adxcvr_atx_pll_ops;
	else
		init.ops = &adxcvr_cdr_pll_ops;
	init.flags = CLK_SET_RATE_GATE | CLK_SET_PARENT_GATE;
	init.num_parents = 1;
	init.parent_names = &parent_name;

	st->lane_clk.init = &init;
	clk = devm_clk_register(st->dev, &st->lane_clk);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(st->dev->of_node, of_clk_src_simple_get,
		clk);
}

static const struct jesd204_dev_data adxcvr_jesd204_data = {
};

static int adxcvr_probe(struct platform_device *pdev)
{
	struct resource *mem_adxcvr;
	struct resource *mem_atx_pll;
	struct resource *mem_adxcfg;
	struct adxcvr_state *st;
	unsigned int synth_conf;
	char adxcfg_name[16];
	int lane;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->jdev = devm_jesd204_dev_register(&pdev->dev, &adxcvr_jesd204_data);
	if (IS_ERR(st->jdev))
		return PTR_ERR(st->jdev);

	st->ref_clk = devm_clk_get(&pdev->dev, "ref");
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	mem_adxcvr = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "adxcvr");
	st->adxcvr_regs = devm_ioremap_resource(&pdev->dev, mem_adxcvr);
	if (IS_ERR(st->adxcvr_regs))
		return PTR_ERR(st->adxcvr_regs);

	st->version = adxcvr_read(st, ADXCVR_REG_VERSION);

	synth_conf = adxcvr_read(st, ADXCVR_REG_SYNTH_CONF);

	st->is_transmit = (bool)(synth_conf & 0x100);
	st->lanes_per_link = synth_conf & 0xff;

	if (st->lanes_per_link > ARRAY_SIZE(st->adxcfg_regs)) {
		dev_err(&pdev->dev, "Only up to %d lanes supported.\n",
			ARRAY_SIZE(st->adxcfg_regs));
		return -EINVAL;
	}

	for (lane = 0; lane < st->lanes_per_link; lane++) {
		sprintf(adxcfg_name, "adxcfg-%d", lane);
		mem_adxcfg = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, adxcfg_name);
		st->adxcfg_regs[lane] = devm_ioremap_resource(&pdev->dev,
							      mem_adxcfg);
		if (IS_ERR(st->adxcfg_regs[lane]))
			return PTR_ERR(st->adxcfg_regs[lane]);
	}

	st->link_clk = devm_clk_get(&pdev->dev, "link");
	if (IS_ERR(st->link_clk) && PTR_ERR(st->link_clk) != -ENOENT)
		return PTR_ERR(st->link_clk);

	if (st->is_transmit) {
		mem_atx_pll = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "atx-pll");
		st->atx_pll_regs = devm_ioremap_resource(&pdev->dev,
							 mem_atx_pll);
		if (IS_ERR(st->atx_pll_regs))
			return PTR_ERR(st->atx_pll_regs);
	}

	st->dev = &pdev->dev;
	platform_set_drvdata(pdev, st);

	INIT_WORK(&st->link_clk_work, adxcvr_link_clk_work);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		return ret;

	ret = adxcvr_register_lane_clk(st);
	if (ret)
		return ret;

	if (!IS_ERR(st->link_clk)) {
		ret = clk_prepare_enable(st->link_clk);
		if (ret)
			return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &adxcvr_sysfs_group);
	if (ret)
		dev_err(&pdev->dev, "Can't create the sysfs group\n");

	ret = jesd204_fsm_start(st->jdev, JESD204_LINKS_ALL);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Altera ADXCVR (%d.%.2d.%c) probed\n",
			VERSION_MAJOR(st->version),
			VERSION_MINOR(st->version),
			VERSION_LETTER(st->version));

	return 0;
}

static int adxcvr_remove(struct platform_device *pdev)
{
	struct adxcvr_state *st = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);
	sysfs_remove_group(&pdev->dev.kobj, &adxcvr_sysfs_group);

	clk_disable_unprepare(st->link_clk);
	clk_disable_unprepare(st->ref_clk);

	return 0;
}

static const struct of_device_id adxcvr_of_match[] = {
	{ .compatible = "adi,altera-adxcvr-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adxcvr_of_match);

static struct platform_driver adxcvr_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = adxcvr_of_match,
	},
	.probe	= adxcvr_probe,
	.remove	= adxcvr_remove,
};

module_platform_driver(adxcvr_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Altera ADXCVR Configuration Driver");
MODULE_LICENSE("GPL v2");
