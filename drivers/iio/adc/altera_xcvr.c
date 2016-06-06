/*
 * Altera XCVR Configuration Driver
 *
 * Copyright 2015-2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

/* Registers Description */

#define XCVR_REG_VERSION				0x0000
#define XCVR_REG_ID						0x0004
#define XCVR_REG_SCRATCH				0x0008

#define XCVR_REG_RESETN					0x000C
#define XCVR_RESETN						(1 << 0)

#define XCVR_REG_RX_SYSREF				0x0040
#define XCVR_RX_SYSREF_SEL				(1 << 1)
#define XCVR_RX_SYSREF					(1 << 0)

#define XCVR_REG_RX_SYNC				0x0044
#define XCVR_RX_SYNC					(1 << 0)

#define XCVR_REG_RX_STATUS				0x0048

#define XCVR_REG_RX_RESETN				0x004C
#define XCVR_RX_RESETN					(1 << 0)

#define XCVR_REG_TX_SYSREF				0x0080
#define XCVR_TX_SYSREF_SEL				(1 << 1)
#define XCVR_TX_SYSREF					(1 << 0)

#define XCVR_REG_TX_SYNC				0x0084
#define XCVR_TX_SYNC					(1 << 0)

#define XCVR_REG_TX_STATUS				0x0088

#define XCVR_REG_TX_RESETN				0x008C
#define XCVR_TX_RESETN					(1 << 0)

#define XCVR_REG_DEVICE_TYPE			0x00C0

#define XCVR_REG_ARBITRATION(link)				(0x000 | ((link) << 18))
#define XCVR_ARBITRATION_MASK					0xFF
#define XCVR_ARBITRATION_GET_AVMM				0x02
#define XCVR_ARBITRATION_RELEASE_AVMM			0x01

#define XCVR_REG_CALIB_ATX_PLL_EN(link)			(0x400 | ((link) << 18))
#define XCVR_CALIB_ATX_PLL_EN_MASK				0x01
#define XCVR_CALIB_ATX_PLL_EN					0x01

#define XCVR_REG_CAPAB_ATX_PLL_STAT(link)		(0xA00 | ((link) << 18))
#define XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK			0x02
#define XCVR_CAPAB_ATX_PLL_CAL_DONE				0x00

#define XCVR_REG_CALIB_PMA_EN(link)				(0x400 | ((link) << 18))
#define XCVR_CALIB_TX_TERM_VOD_MASK				0x20
#define XCVR_CALIB_TX_TERM_VOD_EN				0x20
#define XCVR_CALIB_CMU_CDR_PLL_EN_MASK			0x02
#define XCVR_CALIB_CMU_CDR_PLL_EN				0x02

#define XCVR_REG_CAPAB_PMA(link)				(0xA04 | ((link) << 18))
#define XCVR_CAPAB_RX_CAL_BUSY_EN_MASK			0x20
#define XCVR_CAPAB_RX_CAL_BUSY_EN				0x20
#define XCVR_CAPAB_RX_CAL_BUSY_DIS				0x00
#define XCVR_CAPAB_RX_CAL_BUSY_MASK				0x02
#define XCVR_CAPAB_RX_CAL_DONE					0x00
#define XCVR_CAPAB_TX_CAL_BUSY_EN_MASK			0x10
#define XCVR_CAPAB_TX_CAL_BUSY_EN				0x10
#define XCVR_CAPAB_TX_CAL_BUSY_DIS				0x00
#define XCVR_CAPAB_TX_CAL_BUSY_MASK				0x01
#define XCVR_CAPAB_TX_CAL_DONE					0x00

#define XCVR_REG_RATE_SWITCH_FLAG(link)			(0x598 | ((link) << 18))
#define XCVR_RATE_SWITCH_FLAG_MASK				0x80
#define XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH	0x80

#define XCVR_CFG_DPRIO_ADDR_WIDTH				12

struct xcvr_state {
	struct device 		*dev;
	void __iomem		*jesd_xcvr_regs;
	void __iomem		*xcvr_reconfig_avmm_regs;
	void __iomem		*xcvr_atx_pll_regs;
	bool				ext_sysref_en;
	bool				tx_en;
	u32					tx_link_num;
	u32					tx_lanes_per_link;
	bool				rx_en;
	u32					rx_link_num;
	u32					rx_lanes_per_link;
	struct delayed_work	delayed_work;
};

static inline void jesd_xcvr_write(struct xcvr_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->jesd_xcvr_regs + reg);
}

static inline unsigned int jesd_xcvr_read(struct xcvr_state *st,
	unsigned reg)
{
	return ioread32(st->jesd_xcvr_regs + reg);
}

static inline void xcvr_reconfig_avmm_write(struct xcvr_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->xcvr_reconfig_avmm_regs + reg);
}

static inline unsigned int xcvr_reconfig_avmm_read(struct xcvr_state *st,
	unsigned reg)
{
	return ioread32(st->xcvr_reconfig_avmm_regs + reg);
}

static inline void xcvr_atx_pll_write(struct xcvr_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->xcvr_atx_pll_regs + reg);
}

static inline unsigned int xcvr_atx_pll_read(struct xcvr_state *st,
	unsigned reg)
{
	return ioread32(st->xcvr_atx_pll_regs + reg);
}

static int xcvr_calib_atx_pll(struct xcvr_state *st)
{
	unsigned link = st->tx_link_num;
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;

	/* Get AVMM Interface */
	addr = XCVR_REG_ARBITRATION(link);
	mask = XCVR_ARBITRATION_MASK;
	val = XCVR_ARBITRATION_GET_AVMM;
	write_val = (xcvr_atx_pll_read(st, addr) & ~mask) | (val & mask);
	xcvr_atx_pll_write(st, addr, write_val);

	/* Initiate re-calibration of ATX_PLL */
	addr = XCVR_REG_CALIB_ATX_PLL_EN(link);
	mask = XCVR_CALIB_ATX_PLL_EN_MASK;
	val = XCVR_CALIB_ATX_PLL_EN;
	write_val = (xcvr_atx_pll_read(st, addr) & ~mask) | (val & mask);
	xcvr_atx_pll_write(st, addr, write_val);

	/* Release AVMM Interface to PreSICE */
	addr = XCVR_REG_ARBITRATION(link);
	mask = XCVR_ARBITRATION_MASK;
	val = XCVR_ARBITRATION_RELEASE_AVMM;
	write_val = (xcvr_atx_pll_read(st, addr) & ~mask) | (val & mask);
	xcvr_atx_pll_write(st, addr, write_val);

	mdelay(100);	// Wait 100ms for cal_busy to de-assert

	/* Read ATX PLL calibration status from capability register */
	addr = XCVR_REG_CAPAB_ATX_PLL_STAT(link);
	read_val = xcvr_atx_pll_read(st, addr);
	if ((read_val & XCVR_CAPAB_ATX_PLL_CAL_BSY_MASK) ==
			XCVR_CAPAB_ATX_PLL_CAL_DONE) {
		dev_info(st->dev, "Link %d ATX PLL calibration OK\n", link);
		return 0;
	}
	else {
		dev_err(st->dev, "Link %d ATX PLL calibration error\n", link);
		return 1;
	}
}

static int xcvr_calib_tx(struct xcvr_state *st)
{
	unsigned link = st->tx_link_num;
	unsigned lane;
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;
	unsigned err = 0;

    for (lane = 0; lane < st->tx_lanes_per_link; lane++)
    {
        /* Get AVMM Interface from PreSICE through arbitration register */
		addr = XCVR_REG_ARBITRATION(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_GET_AVMM;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Perform TX termination & Vod calibration through
		   PMA calibration enable register */
		addr = XCVR_REG_CALIB_PMA_EN(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_CALIB_TX_TERM_VOD_MASK;
		val = XCVR_CALIB_TX_TERM_VOD_EN;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Disable rx_cal_busy and enable tx_cal_busy output through
		   capability register */
		addr = XCVR_REG_CAPAB_PMA(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_CAPAB_RX_CAL_BUSY_EN_MASK | XCVR_CAPAB_TX_CAL_BUSY_EN_MASK;
		val = XCVR_CAPAB_RX_CAL_BUSY_DIS | XCVR_CAPAB_TX_CAL_BUSY_EN;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Release AVMM Interface to PreSICE */
		addr = XCVR_REG_ARBITRATION(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_RELEASE_AVMM;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

		mdelay(100);	// Wait 100ms for cal_busy to de-assert

        /* Read PMA calibration status from capability register */
		addr = XCVR_REG_CAPAB_PMA(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		read_val = xcvr_reconfig_avmm_read(st, addr);
		if ((read_val & XCVR_CAPAB_TX_CAL_BUSY_MASK) == XCVR_CAPAB_TX_CAL_DONE) {
			dev_info(st->dev, "Link %d ch %d TX termination and VOD calib OK\n",
					 link, lane);
		} else {
			dev_err(st->dev, "Link %d ch %d TX termination and VOD calib error\n",
					link, lane);
			err |= 1;
		}
	}

	return err;
}

static int xcvr_calib_rx(struct xcvr_state *st)
{
	unsigned link = st->rx_link_num;
	unsigned lane;
	unsigned addr;
	unsigned mask;
	unsigned val;
	unsigned write_val;
	unsigned read_val;
	unsigned err = 0;

    for (lane = 0; lane < st->tx_lanes_per_link; lane++)
    {
        /* Get AVMM Interface from PreSICE through arbitration register */
		addr = XCVR_REG_ARBITRATION(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_GET_AVMM;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Perform CDR/CMU PLL and RX offset cancellation calibration through
		   PMA calibration enable register */
		addr = XCVR_REG_CALIB_PMA_EN(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_CALIB_CMU_CDR_PLL_EN_MASK;
		val = XCVR_CALIB_CMU_CDR_PLL_EN;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

		/* Set rate switch flag register for CDR charge pump calibration */
		addr = XCVR_REG_RATE_SWITCH_FLAG(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_RATE_SWITCH_FLAG_MASK;
		val = XCVR_RATE_SWITCH_FLAG_NO_RATE_SWITCH;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Disable tx_cal_busy and enable rx_cal_busy output through
		   capability register */
		addr = XCVR_REG_CAPAB_PMA(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_CAPAB_RX_CAL_BUSY_EN_MASK | XCVR_CAPAB_TX_CAL_BUSY_EN_MASK;
		val = XCVR_CAPAB_RX_CAL_BUSY_EN | XCVR_CAPAB_TX_CAL_BUSY_DIS;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

        /* Release AVMM Interface to PreSICE */
		addr = XCVR_REG_ARBITRATION(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		mask = XCVR_ARBITRATION_MASK;
		val = XCVR_ARBITRATION_RELEASE_AVMM;
		write_val = (xcvr_reconfig_avmm_read(st, addr) & ~mask) | (val & mask);
		xcvr_reconfig_avmm_write(st, addr, write_val);

		mdelay(100);	// Wait 100ms for cal_busy to de-assert

        /* Read PMA calibration status from capability register */
		addr = XCVR_REG_CAPAB_PMA(link) | (lane << XCVR_CFG_DPRIO_ADDR_WIDTH);
		read_val = xcvr_reconfig_avmm_read(st, addr);
		if ((read_val & XCVR_CAPAB_RX_CAL_BUSY_MASK) == XCVR_CAPAB_RX_CAL_DONE) {
			dev_info(st->dev, "Link %d ch %d CDR/CMU PLL & RX offset calib OK\n",
					 link, lane);
		} else {
			dev_err(st->dev, "Link %d ch %d CDR/CMU PLL & RX offset calib error\n",
					link, lane);
			err |= 1;
		}
	}

	return err;
}

static ssize_t altera_xcvr_sysfs_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf);
static ssize_t altera_xcvr_sysfs_store(struct device *dev,
									   struct device_attribute *attr,
									   const char *buf, size_t count);

static DEVICE_ATTR(altera_xcvr_version, S_IRUGO,
				   altera_xcvr_sysfs_show, NULL);
static DEVICE_ATTR(altera_xcvr_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_sysref, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_sync, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_status, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_sysref, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_sync, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_status, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);

static struct attribute *altera_xcvr_sysfs_attrs[] = {
	&dev_attr_altera_xcvr_version.attr,
	&dev_attr_altera_xcvr_reset.attr,
	&dev_attr_altera_xcvr_rx_reset.attr,
	&dev_attr_altera_xcvr_rx_sysref.attr,
	&dev_attr_altera_xcvr_rx_sync.attr,
	&dev_attr_altera_xcvr_rx_status.attr,
	&dev_attr_altera_xcvr_tx_reset.attr,
	&dev_attr_altera_xcvr_tx_sysref.attr,
	&dev_attr_altera_xcvr_tx_sync.attr,
	&dev_attr_altera_xcvr_tx_status.attr,
	NULL
};

static const struct attribute_group altera_xcvr_sysfs_group = {
	.attrs = altera_xcvr_sysfs_attrs,
};

static ssize_t altera_xcvr_sysfs_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf)
{
	struct xcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned val = 0;

	if (attr == &dev_attr_altera_xcvr_version)
		reg = XCVR_REG_VERSION;
	if (attr == &dev_attr_altera_xcvr_reset)
		reg = XCVR_REG_RESETN;
	if (attr == &dev_attr_altera_xcvr_rx_reset)
		reg = XCVR_REG_RX_RESETN;
	if (attr == &dev_attr_altera_xcvr_rx_sysref)
		reg = XCVR_REG_RX_SYSREF;
	if (attr == &dev_attr_altera_xcvr_rx_sync)
		reg = XCVR_REG_RX_SYNC;
	if (attr == &dev_attr_altera_xcvr_rx_status)
		reg = XCVR_REG_RX_STATUS;
	if (attr == &dev_attr_altera_xcvr_tx_reset)
		reg = XCVR_REG_TX_RESETN;
	if (attr == &dev_attr_altera_xcvr_tx_sysref)
		reg = XCVR_REG_TX_SYSREF;
	if (attr == &dev_attr_altera_xcvr_tx_sync)
		reg = XCVR_REG_TX_SYNC;
	if (attr == &dev_attr_altera_xcvr_tx_status)
		reg = XCVR_REG_TX_STATUS;
	if (reg == -1)
		return -EINVAL;

	val = jesd_xcvr_read(st, reg);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t altera_xcvr_sysfs_store(struct device *dev,
									   struct device_attribute *attr,
									   const char *buf, size_t count)
{
	struct xcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (attr == &dev_attr_altera_xcvr_reset)
		reg = XCVR_REG_RESETN;
	else if (attr == &dev_attr_altera_xcvr_rx_reset)
		reg = XCVR_REG_RX_RESETN;
	else if (attr == &dev_attr_altera_xcvr_rx_sysref)
		reg = XCVR_REG_RX_SYSREF;
	else if (attr == &dev_attr_altera_xcvr_rx_sync)
		reg = XCVR_REG_RX_SYNC;
	else if (attr == &dev_attr_altera_xcvr_rx_status)
		reg = XCVR_REG_RX_STATUS;
	else if (attr == &dev_attr_altera_xcvr_tx_reset)
		reg = XCVR_REG_TX_RESETN;
	else if (attr == &dev_attr_altera_xcvr_tx_sysref)
		reg = XCVR_REG_TX_SYSREF;
	else if (attr == &dev_attr_altera_xcvr_tx_sync)
		reg = XCVR_REG_TX_SYNC;
	else if (attr == &dev_attr_altera_xcvr_tx_status)
		reg = XCVR_REG_TX_STATUS;
	if (reg == -1)
		return -EINVAL;

	jesd_xcvr_write(st, reg, val);

	return count;
}

static void altera_xcvr_work_func(struct work_struct *work)
{
	struct xcvr_state *st =
			container_of(work, struct xcvr_state, delayed_work.work);
	unsigned status;
	unsigned int err = 0;

	jesd_xcvr_write(st, XCVR_REG_RESETN, 0);
	mdelay(10);
	jesd_xcvr_write(st, XCVR_REG_RESETN, XCVR_RESETN);
	mdelay(10);

	/* RX */
	if (st->rx_en) {
		jesd_xcvr_write(st, XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? XCVR_RX_SYSREF_SEL : 0));
		mdelay(10);
		jesd_xcvr_write(st, XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? XCVR_RX_SYSREF_SEL : XCVR_RX_SYSREF));
		mdelay(500);

		if (xcvr_calib_rx(st)) {
			dev_err(st->dev, "RX calib error\n");
			err = 1;
		}

		status = jesd_xcvr_read(st, XCVR_REG_RX_STATUS);
		if ((status & 0x1ff) != 0x1ff) {
			dev_err(st->dev, "RX transceiver NOT ready [%04x]\n", status);
			err = 1;
		} else
			dev_info(st->dev, "RX transceiver ready\n");
	}

	/* TX */
	if (st->tx_en) {
		if (xcvr_calib_atx_pll(st)) {
			dev_err(st->dev, "ATX PLL NOT ready\n");
			err = 1;
		}

		jesd_xcvr_write(st, XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? XCVR_TX_SYSREF_SEL : 0));
		mdelay(10);
		jesd_xcvr_write(st, XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? XCVR_TX_SYSREF_SEL : XCVR_TX_SYSREF));
		mdelay(500);

		if (xcvr_calib_tx(st)) {
			dev_err(st->dev, "TX calib error\n");
			err = 1;
		}

		status = jesd_xcvr_read(st, XCVR_REG_TX_STATUS);
		if ((status & 0x1ff) != 0x1ff) {
			dev_err(st->dev, "TX transceiver NOT ready [%04x]\n", status);
			err = 1;
		} else
			dev_info(st->dev, "TX transceiver ready\n");
	}

	if (err)
		schedule_delayed_work(&st->delayed_work, HZ * 1);
}

static int altera_xcvr_probe(struct platform_device *pdev)
{
	struct xcvr_state *st;
	struct resource *mem_jesd_xcvr;
	struct resource *mem_xcvr_reconfig_avmm;
	struct resource *mem_xcvr_atx_pll;
	int ret;
	struct device_node *np = pdev->dev.of_node;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	st->ext_sysref_en = of_property_read_bool(np, "adi,external-sysref-enable");
	st->rx_en = of_property_read_bool(np, "adi,rx-enable");
	st->tx_en = of_property_read_bool(np, "adi,tx-enable");

	mem_jesd_xcvr = platform_get_resource_byname(pdev, IORESOURCE_MEM, "jesd-xcvr");
	st->jesd_xcvr_regs = devm_ioremap_resource(&pdev->dev, mem_jesd_xcvr);
	if (IS_ERR(st->jesd_xcvr_regs))
		return PTR_ERR(st->jesd_xcvr_regs);

	mem_xcvr_reconfig_avmm = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xcvr-reconfig-avmm");
	st->xcvr_reconfig_avmm_regs = devm_ioremap_resource(&pdev->dev, mem_xcvr_reconfig_avmm);
	if (IS_ERR(st->xcvr_reconfig_avmm_regs))
		return PTR_ERR(st->xcvr_reconfig_avmm_regs);

	if (st->tx_en) {
		of_property_read_u32(np, "adi,tx-link-number", &st->tx_link_num);
		of_property_read_u32(np, "adi,tx-lanes-per-link", &st->tx_lanes_per_link);
		mem_xcvr_atx_pll = platform_get_resource_byname(pdev, IORESOURCE_MEM,
														"xcvr-atx-pll");
		st->xcvr_atx_pll_regs = devm_ioremap_resource(&pdev->dev, mem_xcvr_atx_pll);
		if (IS_ERR(st->xcvr_atx_pll_regs))
			return PTR_ERR(st->xcvr_atx_pll_regs);
	}

	if (st->rx_en) {
		of_property_read_u32(np, "adi,rx-link-number", &st->rx_link_num);
		of_property_read_u32(np, "adi,rx-lanes-per-link", &st->rx_lanes_per_link);
	}

	st->dev = &pdev->dev;
	platform_set_drvdata(pdev, st);

	jesd_xcvr_write(st, XCVR_REG_RESETN, XCVR_RESETN);
	mdelay(10);

	if (st->rx_en) {
		jesd_xcvr_write(st, XCVR_REG_RX_RESETN, XCVR_RX_RESETN);
		mdelay(10);

		jesd_xcvr_write(st, XCVR_REG_RX_SYNC, XCVR_RX_SYNC);
		jesd_xcvr_write(st, XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? XCVR_RX_SYSREF_SEL : 0));
	}

	if (st->tx_en) {
		jesd_xcvr_write(st, XCVR_REG_TX_RESETN, XCVR_TX_RESETN);
		mdelay(10);

		jesd_xcvr_write(st, XCVR_REG_TX_SYNC, XCVR_TX_SYNC);
		jesd_xcvr_write(st, XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? XCVR_TX_SYSREF_SEL : 0));
	}

	INIT_DELAYED_WORK(&st->delayed_work, altera_xcvr_work_func);
	schedule_delayed_work(&st->delayed_work, HZ * 5);

	ret = sysfs_create_group(&pdev->dev.kobj, &altera_xcvr_sysfs_group);
	if (ret)
		dev_err(&pdev->dev, "Can't create the sysfs group\n");

	dev_info(&pdev->dev, "Altera XCVR probed\n");

	return 0;
}

static int altera_xcvr_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &altera_xcvr_sysfs_group);

	return 0;
}

static const struct of_device_id altera_xcvr_of_match[] = {
	{ .compatible = "adi,altera-xcvr-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, altera_xcvr_of_match);

static struct platform_driver altera_xcvr_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = altera_xcvr_of_match,
	},
	.probe	= altera_xcvr_probe,
	.remove	= altera_xcvr_remove,
};

module_platform_driver(altera_xcvr_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Altera XCVR Configuration Driver");
MODULE_LICENSE("GPL v2");
