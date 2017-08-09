/*
 * Altera ADXCVR Configuration Driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

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

/* XCVR Registers */

#define XCVR_REG_ARBITRATION			0x000
#define XCVR_ARBITRATION_MASK			0xFF
#define XCVR_ARBITRATION_GET_AVMM		0x02
#define XCVR_ARBITRATION_RELEASE_AVMM		0x01

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
#define XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH	0x80

struct adxcvr_state {
	struct device		*dev;
	void __iomem		*adxcvr_regs;
	void __iomem		*atx_pll_regs;
	void __iomem		*adxcfg_regs[4];
	unsigned int 		version;
	bool			tx_en;
	bool			rx_en;
	u32			lanes_per_link;
	u32			delay_work_seconds;
	struct delayed_work	delayed_work;
};

static void adxcvr_write(struct adxcvr_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->adxcvr_regs + reg);
}

static unsigned int adxcvr_read(struct adxcvr_state *st, unsigned reg)
{
	return ioread32(st->adxcvr_regs + reg);
}

static void atx_pll_write(struct adxcvr_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->atx_pll_regs + reg * 4);
}

static unsigned int atx_pll_read(struct adxcvr_state *st, unsigned reg)
{
	return ioread32(st->atx_pll_regs + reg * 4);
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

static int atx_pll_calib(struct adxcvr_state *st)
{
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;

	/* Get AVMM Interface */
	addr = XCVR_REG_ARBITRATION;
	mask = XCVR_ARBITRATION_MASK;
	val = XCVR_ARBITRATION_GET_AVMM;
	write_val = (atx_pll_read(st, addr) & ~mask) | (val & mask);
	atx_pll_write(st, addr, write_val);

	/* Initiate re-calibration of ATX_PLL */
	addr = XCVR_REG_CALIB_ATX_PLL_EN;
	mask = XCVR_CALIB_ATX_PLL_EN_MASK;
	val = XCVR_CALIB_ATX_PLL_EN;
	write_val = (atx_pll_read(st, addr) & ~mask) | (val & mask);
	atx_pll_write(st, addr, write_val);

	/* Release AVMM Interface to PreSICE */
	addr = XCVR_REG_ARBITRATION;
	mask = XCVR_ARBITRATION_MASK;
	val = XCVR_ARBITRATION_RELEASE_AVMM;
	write_val = (atx_pll_read(st, addr) & ~mask) | (val & mask);
	atx_pll_write(st, addr, write_val);

	mdelay(100);	// Wait 100ms for cal_busy to de-assert

	/* Read ATX PLL calibration status from capability register */
	addr = XCVR_REG_CAPAB_ATX_PLL_STAT;
	read_val = atx_pll_read(st, addr);
	if ((read_val & XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK) ==
			XCVR_CAPAB_ATX_PLL_CAL_DONE) {
		dev_info(st->dev, "ATX PLL calibration OK\n");
		return 0;
	}
	else {
		dev_err(st->dev, "ATX PLL calibration error\n");
		return 1;
	}
}

static int xcvr_calib_tx(struct adxcvr_state *st)
{
	unsigned lane;
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;
	unsigned err = 0;

	for (lane = 0; lane < st->lanes_per_link; lane++) {
		/* Get AVMM Interface from PreSICE through arbitration register */
		addr = XCVR_REG_ARBITRATION;
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_GET_AVMM;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Perform TX termination & Vod calibration through
		   PMA calibration enable register */
		addr = XCVR_REG_CALIB_PMA_EN;
		mask = XCVR_CALIB_TX_TERM_VOD_MASK;
		val = XCVR_CALIB_TX_TERM_VOD_EN;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Disable rx_cal_busy and enable tx_cal_busy output through
		   capability register */
		addr = XCVR_REG_CAPAB_PMA;
		mask = XCVR_CAPAB_RX_CAL_BUSY_EN_MASK | XCVR_CAPAB_TX_CAL_BUSY_EN_MASK;
		val = XCVR_CAPAB_RX_CAL_BUSY_DIS | XCVR_CAPAB_TX_CAL_BUSY_EN;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Release AVMM Interface to PreSICE */
		addr = XCVR_REG_ARBITRATION;
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_RELEASE_AVMM;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		mdelay(100);	// Wait 100ms for cal_busy to de-assert

		/* Read PMA calibration status from capability register */
		addr = XCVR_REG_CAPAB_PMA;
		read_val = adxcfg_read(st, lane, addr);
		if ((read_val & XCVR_CAPAB_TX_CAL_BUSY_MASK) == XCVR_CAPAB_TX_CAL_DONE) {
			dev_info(st->dev, "Lane %d TX termination and VOD calib OK\n",
					lane);
		} else {
			dev_err(st->dev, "Lane %d TX termination and VOD calib error\n",
					lane);
			err |= 1;
		}
	}

	return err;
}

static int xcvr_calib_rx(struct adxcvr_state *st)
{
	unsigned lane;
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;
	unsigned err = 0;

	for (lane = 0; lane < st->lanes_per_link; lane++) {
		/* Get AVMM Interface from PreSICE through arbitration register */
		addr = XCVR_REG_ARBITRATION;
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_GET_AVMM;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Perform CDR/CMU PLL and RX offset cancellation calibration through
		   PMA calibration enable register */
		addr = XCVR_REG_CALIB_PMA_EN;
		mask = XCVR_CALIB_CMU_CDR_PLL_EN_MASK;
		val = XCVR_CALIB_CMU_CDR_PLL_EN;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Set rate switch flag register for CDR charge pump calibration */
		addr = XCVR_REG_RATE_SWITCH_FLAG;
		mask = XCVR_RATE_SWITCH_FLAG_MASK;
		val = XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Disable tx_cal_busy and enable rx_cal_busy output through
		   capability register */
		addr = XCVR_REG_CAPAB_PMA;
		mask = XCVR_CAPAB_RX_CAL_BUSY_EN_MASK | XCVR_CAPAB_TX_CAL_BUSY_EN_MASK;
		val = XCVR_CAPAB_RX_CAL_BUSY_EN | XCVR_CAPAB_TX_CAL_BUSY_DIS;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		/* Release AVMM Interface to PreSICE */
		addr = XCVR_REG_ARBITRATION;
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_RELEASE_AVMM;
		write_val = (adxcfg_read(st, lane, addr) & ~mask) | (val & mask);
		adxcfg_write(st, lane, addr, write_val);

		mdelay(100);	// Wait 100ms for cal_busy to de-assert

		/* Read PMA calibration status from capability register */
		addr = XCVR_REG_CAPAB_PMA;
		read_val = adxcfg_read(st, lane, addr);
		if ((read_val & XCVR_CAPAB_RX_CAL_BUSY_MASK) == XCVR_CAPAB_RX_CAL_DONE) {
			dev_info(st->dev, "Lane %d CDR/CMU PLL & RX offset calib OK\n",
					 lane);
		} else {
			dev_err(st->dev, "Lane %d CDR/CMU PLL & RX offset calib error\n",
					lane);
			err |= 1;
		}
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

static void adxcvr_work_func(struct work_struct *work)
{

	struct adxcvr_state *st =
		container_of(work, struct adxcvr_state, delayed_work.work);
	unsigned status = 0;
	int timeout = 1000;
	unsigned int err = 0;

	adxcvr_write(st, ADXCVR_REG_RESETN, 0);

	if (st->tx_en) {
		if (atx_pll_calib(st)) {
			dev_err(st->dev, "ATX PLL NOT ready\n");
			err = 1;
		}
		if (xcvr_calib_tx(st)) {
			dev_err(st->dev, "TX calib error\n");
			err = 1;
		}
	}

	if (st->rx_en)
		if (xcvr_calib_rx(st)) {
			dev_err(st->dev, "RX calib error\n");
			err = 1;
		}

	adxcvr_write(st, ADXCVR_REG_RESETN, ADXCVR_RESETN);
	do {
		status = adxcvr_read(st, ADXCVR_REG_STATUS);
		if (status == ADXCVR_STATUS)
			break;
		mdelay(1);
	} while (timeout--);

	if (timeout < 0) {
		dev_err(st->dev, "Link activation error\n");
		err = 1;
	}

	if (err)
		schedule_delayed_work(&st->delayed_work, HZ * 10);
}

static int adxcvr_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *mem_adxcvr;
	struct resource *mem_atx_pll;
	struct resource *mem_adxcfg[4];
	struct adxcvr_state *st;
	char adxcfg_name[16];
	int lane;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mem_adxcvr = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "adxcvr");
	st->adxcvr_regs = devm_ioremap_resource(&pdev->dev, mem_adxcvr);
	if (IS_ERR(st->adxcvr_regs))
		return PTR_ERR(st->adxcvr_regs);

	st->version = adxcvr_read(st, ADXCVR_REG_VERSION);

	of_property_read_u32(np, "adi,lanes-per-link",
			     &st->lanes_per_link);

	for (lane = 0; lane < st->lanes_per_link; lane++) {
		sprintf(adxcfg_name, "adxcfg-%d", lane);
		mem_adxcfg[lane] = platform_get_resource_byname(pdev,
						IORESOURCE_MEM, adxcfg_name);
		st->adxcfg_regs[lane] = devm_ioremap_resource(&pdev->dev,
						mem_adxcfg[lane]);
		if (IS_ERR(st->adxcfg_regs[lane]))
			return PTR_ERR(st->adxcfg_regs[lane]);
	}

	st->tx_en = of_property_read_bool(np, "adi,tx-enable");
	st->rx_en = of_property_read_bool(np, "adi,rx-enable");

	if (st->tx_en) {
		mem_atx_pll = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "atx-pll");
		st->atx_pll_regs = devm_ioremap_resource(&pdev->dev,
							 mem_atx_pll);
		if (IS_ERR(st->atx_pll_regs))
			return PTR_ERR(st->atx_pll_regs);
	}

	of_property_read_u32(np, "adi,delay-work-seconds",
			&st->delay_work_seconds);

	st->dev = &pdev->dev;
	platform_set_drvdata(pdev, st);

	INIT_DELAYED_WORK(&st->delayed_work, adxcvr_work_func);
	schedule_delayed_work(&st->delayed_work, HZ * st->delay_work_seconds);

	ret = sysfs_create_group(&pdev->dev.kobj, &adxcvr_sysfs_group);
	if (ret)
		dev_err(&pdev->dev, "Can't create the sysfs group\n");

	dev_info(&pdev->dev, "Altera ADXCVR (%d.%.2d.%c) probed\n",
			VERSION_MAJOR(st->version),
			VERSION_MINOR(st->version),
			VERSION_LETTER(st->version));

	return 0;
}

static int adxcvr_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &adxcvr_sysfs_group);

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
