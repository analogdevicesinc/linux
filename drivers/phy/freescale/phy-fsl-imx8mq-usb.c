// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2017 NXP. */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/typec_mux.h>

#define PHY_CTRL0			0x0
#define PHY_CTRL0_REF_SSP_EN		BIT(2)
#define PHY_CTRL0_FSEL_MASK		GENMASK(10, 5)
#define PHY_CTRL0_FSEL_24M		0x2a
#define PHY_CTRL0_FSEL_100M		0x27
#define PHY_CTRL0_SSC_RANGE_MASK	GENMASK(23, 21)
#define PHY_CTRL0_SSC_RANGE_4003PPM	(0x2 << 21)

#define PHY_CTRL1			0x4
#define PHY_CTRL1_RESET			BIT(0)
#define PHY_CTRL1_COMMONONN		BIT(1)
#define PHY_CTRL1_ATERESET		BIT(3)
#define PHY_CTRL1_DCDENB		BIT(17)
#define PHY_CTRL1_CHRGSEL		BIT(18)
#define PHY_CTRL1_VDATSRCENB0		BIT(19)
#define PHY_CTRL1_VDATDETENB0		BIT(20)

#define PHY_CTRL2			0x8
#define PHY_CTRL2_TXENABLEN0		BIT(8)
#define PHY_CTRL2_OTG_DISABLE		BIT(9)

#define PHY_CTRL3			0xc
#define PHY_CTRL3_COMPDISTUNE_MASK	GENMASK(2, 0)
#define PHY_CTRL3_TXPREEMP_TUNE_MASK	GENMASK(16, 15)
#define PHY_CTRL3_TXRISE_TUNE_MASK	GENMASK(21, 20)
#define PHY_CTRL3_TXVREF_TUNE_MASK	GENMASK(25, 22)
#define PHY_CTRL3_TX_VBOOST_LEVEL_MASK	GENMASK(31, 29)

#define PHY_CTRL4			0x10
#define PHY_CTRL4_PCS_TX_DEEMPH_3P5DB_MASK	GENMASK(20, 15)

#define PHY_CTRL5			0x14
#define PHY_CTRL5_DMPWD_OVERRIDE_SEL	BIT(23)
#define PHY_CTRL5_DMPWD_OVERRIDE	BIT(22)
#define PHY_CTRL5_DPPWD_OVERRIDE_SEL	BIT(21)
#define PHY_CTRL5_DPPWD_OVERRIDE	BIT(20)
#define PHY_CTRL5_PCS_TX_SWING_FULL_MASK	GENMASK(6, 0)

#define PHY_CTRL6			0x18
#define PHY_CTRL6_RXTERM_OVERRIDE_SEL	BIT(29)
#define PHY_CTRL6_ALT_CLK_EN		BIT(1)
#define PHY_CTRL6_ALT_CLK_SEL		BIT(0)

#define PHY_TUNE_DEFAULT		0xffffffff
/* PHY control register access */
#define PHY_CTRL_REG_COUNT_MAX		0x42
#define PHY_CTRL_REG_OFFSET_MAX		0x201f

#define PHY_CRCTL                      0x30
#define PHY_CRCTL_DATA_IN_MASK         GENMASK(15, 0)
#define PHY_CRCTL_CAP_ADDR             BIT(16)
#define PHY_CRCTL_CAP_DATA             BIT(17)
#define PHY_CRCTL_CR_WRITE             BIT(18)
#define PHY_CRCTL_CR_READ              BIT(19)

#define PHY_CRSR			0x34
#define PHY_CRSR_DATA_OUT_MASK		GENMASK(15, 0)
#define PHY_CRSR_CR_ACK			BIT(16)

#define PHY_STS0			0x40
#define PHY_STS0_OTGSESSVLD		BIT(7)
#define PHY_STS0_CHGDET			BIT(4)
#define PHY_STS0_FSVPLUS		BIT(3)
#define PHY_STS0_FSVMINUS		BIT(2)

/*
 *  ##############  TCA Block ################
 */

#define TCA_CLK_RST			0x00
#define TCA_CLK_RST_SW			BIT(9)
#define TCA_CLK_RST_REF_CLK_EN		BIT(1)
#define TCA_CLK_RST_SUSPEND_CLK_EN	BIT(0)

#define TCA_INTR_EN			0x04
#define TCA_INTR_STS			0x08

#define TCA_GCFG			0x10
#define TCA_GCFG_ROLE_HSTDEV		BIT(4)
#define TCA_GCFG_OP_MODE		GENMASK(1, 0)
#define TCA_GCFG_OP_MODE_SYSMODE	0
#define TCA_GCFG_OP_MODE_SYNCMODE	1

#define TCA_TCPC			0x14
#define TCA_TCPC_VALID			BIT(4)
#define TCA_TCPC_LOW_POWER_EN		BIT(3)
#define TCA_TCPC_ORIENTATION_NORMAL	BIT(2)
#define TCA_TCPC_MUX_CONTRL		GENMASK(1, 0)
#define TCA_TCPC_MUX_CONTRL_NO_CONN	0
#define TCA_TCPC_MUX_CONTRL_USB_CONN	1

#define TCA_SYSMODE_CFG			0x18
#define TCA_SYSMODE_TCPC_DISABLE	BIT(3)
#define TCA_SYSMODE_TCPC_FLIP		BIT(2)

#define TCA_CTRLSYNCMODE_CFG0		0x20
#define TCA_CTRLSYNCMODE_CFG1           0x20

#define TCA_PSTATE			0x30
#define TCA_PSTATE_CM_STS		BIT(4)
#define TCA_PSTATE_TX_STS		BIT(3)
#define TCA_PSTATE_RX_PLL_STS		BIT(2)
#define TCA_PSTATE_PIPE0_POWER_DOWN	GENMASK(1, 0)

#define TCA_GEN_STATUS			0x34
#define TCA_GEN_DEV_POR			BIT(12)
#define TCA_GEN_REF_CLK_SEL		BIT(8)
#define TCA_GEN_TYPEC_FLIP_INVERT	BIT(4)
#define TCA_GEN_PHY_TYPEC_DISABLE	BIT(3)
#define TCA_GEN_PHY_TYPEC_FLIP		BIT(2)

#define TCA_VBUS_CTRL			0x40
#define TCA_VBUS_STATUS			0x44

#define TCA_INFO			0xFC

struct tca_blk {
	struct typec_switch_dev *sw;
	void __iomem *base;
	struct mutex mutex;
	enum typec_orientation orientation;
};

struct imx8mq_usb_phy {
	struct phy *phy;
	struct clk *clk;
	void __iomem *base;
	struct regulator *vbus;
	struct tca_blk *tca;
	u32 pcs_tx_swing_full;
	u32 pcs_tx_deemph_3p5db;
	u32 tx_vref_tune;
	u32 tx_rise_tune;
	u32 tx_preemp_amp_tune;
	u32 tx_vboost_level;
	u32 comp_dis_tune;
	struct notifier_block chg_det_nb;
	struct power_supply *vbus_power_supply;
	enum power_supply_usb_type chg_type;
	struct dentry *debugfs;
	u16	cr_access_base;
	u16	cr_read_count;
};

static void tca_blk_orientation_set(struct tca_blk *tca,
				enum typec_orientation orientation);

#ifdef CONFIG_TYPEC

static int tca_blk_typec_switch_set(struct typec_switch_dev *sw,
				enum typec_orientation orientation)
{
	struct imx8mq_usb_phy *imx_phy = typec_switch_get_drvdata(sw);
	struct tca_blk *tca = imx_phy->tca;
	int ret;

	if (tca->orientation == orientation)
		return 0;

	ret = clk_prepare_enable(imx_phy->clk);
	if (ret)
		return ret;

	tca_blk_orientation_set(tca, orientation);
	clk_disable_unprepare(imx_phy->clk);

	return 0;
}

static struct typec_switch_dev *tca_blk_get_typec_switch(struct platform_device *pdev,
					struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &pdev->dev;
	struct typec_switch_dev *sw;
	struct typec_switch_desc sw_desc = { };

	sw_desc.drvdata = imx_phy;
	sw_desc.fwnode = dev->fwnode;
	sw_desc.set = tca_blk_typec_switch_set;
	sw_desc.name = NULL;

	sw = typec_switch_register(dev, &sw_desc);
	if (IS_ERR(sw)) {
		dev_err(dev, "Error register tca orientation switch: %ld",
				PTR_ERR(sw));
		return NULL;
	}

	return sw;
}

static void tca_blk_put_typec_switch(struct typec_switch_dev *sw)
{
	typec_switch_unregister(sw);
}

#else

static struct typec_switch_dev *tca_blk_get_typec_switch(struct platform_device *pdev,
			struct imx8mq_usb_phy *imx_phy)
{
	return NULL;
}

static void tca_blk_put_typec_switch(struct typec_switch_dev *sw) {}

#endif /* CONFIG_TYPEC */

static void tca_blk_orientation_set(struct tca_blk *tca,
				enum typec_orientation orientation)
{
	u32 val;

	mutex_lock(&tca->mutex);

	if (orientation == TYPEC_ORIENTATION_NONE) {
		/*
		 * use Controller Synced Mode for TCA low power enable and
		 * put PHY to USB safe state.
		 */
		val = readl(tca->base + TCA_GCFG);
		val = FIELD_PREP(TCA_GCFG_OP_MODE, TCA_GCFG_OP_MODE_SYNCMODE);
		writel(val, tca->base + TCA_GCFG);

		val = readl(tca->base + TCA_TCPC);
		val = TCA_TCPC_VALID | TCA_TCPC_LOW_POWER_EN;
		writel(val, tca->base + TCA_TCPC);

		goto out;
	}

	/* use System Configuration Mode for TCA mux control. */
	val = readl(tca->base + TCA_GCFG);
	val = FIELD_PREP(TCA_GCFG_OP_MODE, TCA_GCFG_OP_MODE_SYSMODE);
	writel(val, tca->base + TCA_GCFG);

	/* Disable TCA module */
	val = readl(tca->base + TCA_SYSMODE_CFG);
	val |= TCA_SYSMODE_TCPC_DISABLE;
	writel(val, tca->base + TCA_SYSMODE_CFG);

	if (orientation == TYPEC_ORIENTATION_REVERSE)
		val |= TCA_SYSMODE_TCPC_FLIP;
	else if (orientation == TYPEC_ORIENTATION_NORMAL)
		val &= ~TCA_SYSMODE_TCPC_FLIP;

	writel(val, tca->base + TCA_SYSMODE_CFG);

	/* Enable TCA module */
	val &= ~TCA_SYSMODE_TCPC_DISABLE;
	writel(val, tca->base + TCA_SYSMODE_CFG);

out:
	tca->orientation = orientation;
	mutex_unlock(&tca->mutex);
}

static void tca_blk_init(struct tca_blk *tca)
{
	u32 val;

	/* reset XBar block */
	val = readl(tca->base + TCA_CLK_RST);
	val &= ~TCA_CLK_RST_SW;
	writel(val, tca->base + TCA_CLK_RST);

	udelay(100);

	/* clear reset */
	val |= TCA_CLK_RST_SW;
	writel(val, tca->base + TCA_CLK_RST);

	tca_blk_orientation_set(tca, tca->orientation);
}

static int imx95_usb_phy_get_tca(struct platform_device *pdev,
				struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &pdev->dev;
	struct tca_blk *tca;

	tca = devm_kzalloc(dev, sizeof(*tca), GFP_KERNEL);
	if (!tca)
		return -ENOMEM;

	tca->base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(tca->base))
		return PTR_ERR(tca->base);

	mutex_init(&tca->mutex);

	tca->orientation = TYPEC_ORIENTATION_NORMAL;
	tca->sw = tca_blk_get_typec_switch(pdev, imx_phy);
	imx_phy->tca = tca;

	return 0;
}

static void imx95_usb_phy_put_tca(struct imx8mq_usb_phy *imx_phy)
{
	struct tca_blk *tca = imx_phy->tca;

	if (!tca)
		return;

	tca_blk_put_typec_switch(tca->sw);
}

static u32 phy_tx_vref_tune_from_property(u32 percent)
{
	percent = clamp(percent, 94U, 124U);

	return DIV_ROUND_CLOSEST(percent - 94U, 2);
}

static u32 phy_tx_rise_tune_from_property(u32 percent)
{
	switch (percent) {
	case 0 ... 98:
		return 3;
	case 99:
		return 2;
	case 100 ... 101:
		return 1;
	default:
		return 0;
	}
}

static u32 phy_tx_preemp_amp_tune_from_property(u32 microamp)
{
	microamp = min(microamp, 1800U);

	return microamp / 600;
}

static u32 phy_tx_vboost_level_from_property(u32 microvolt)
{
	switch (microvolt) {
	case 0 ... 960:
		return 0;
	case 961 ... 1160:
		return 2;
	default:
		return 3;
	}
}

static u32 phy_pcs_tx_deemph_3p5db_from_property(u32 decibel)
{
	return min(decibel, 36U);
}

static u32 phy_comp_dis_tune_from_property(u32 percent)
{
	switch (percent) {
	case 0 ... 92:
		return 0;
	case 93 ... 95:
		return 1;
	case 96 ... 97:
		return 2;
	case 98 ... 102:
		return 3;
	case 103 ... 105:
		return 4;
	case 106 ... 109:
		return 5;
	case 110 ... 113:
		return 6;
	default:
		return 7;
	}
}
static u32 phy_pcs_tx_swing_full_from_property(u32 percent)
{
	percent = min(percent, 100U);

	return (percent * 127) / 100;
};

#define IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT 500000
static int imx8mq_phy_ctrl_reg_addr(struct imx8mq_usb_phy *imx_phy, u16 offset)
{
	void __iomem	*cr_ctrl = imx_phy->base + PHY_CRCTL;
	void __iomem	*cr_sr = imx_phy->base + PHY_CRSR;
	struct device	*dev = &imx_phy->phy->dev;
	int i;

	/* Address Phrase */
	writel(offset, cr_ctrl);
	writel(readl(cr_ctrl) | PHY_CRCTL_CAP_ADDR, cr_ctrl);

	/* Wait CRSR[16] == 1 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while (!(readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_addr_err;

	writel(readl(cr_ctrl) & (~PHY_CRCTL_CAP_ADDR), cr_ctrl);

	/* Wait CRSR[16] == 0 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while ((readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_addr_err;

	return 0;

cr_addr_err:
	dev_err(dev, "Failed to address reg 0x%x.\n", offset);
	return -EIO;
}

static int imx8mq_phy_ctrl_reg_read(struct imx8mq_usb_phy *imx_phy, u16 offset)
{
	void __iomem	*cr_ctrl = imx_phy->base + PHY_CRCTL;
	void __iomem	*cr_sr = imx_phy->base + PHY_CRSR;
	struct device	*dev = &imx_phy->phy->dev;
	int val, i;

	/* Address Phrase */
	val = imx8mq_phy_ctrl_reg_addr(imx_phy, offset);
	if (val)
		return val;

	/* Read Phrase */
	writel(readl(cr_ctrl) | PHY_CRCTL_CR_READ, cr_ctrl);

	/* Wait CRSR[16] == 1 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while (!(readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_read_err;

	val = readl(cr_sr);
	writel(val & ~PHY_CRCTL_CR_READ, cr_ctrl);

	/* Wait CRSR[16] == 0 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while (!(readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_read_err;

	return val;

cr_read_err:
	dev_err(dev, "Failed to read reg 0x%x.\n", offset);
	return -EIO;
}

static int imx8mq_phy_ctrl_reg_write(struct imx8mq_usb_phy *imx_phy,
				u16 offset, u16 val)
{
	void __iomem	*cr_ctrl = imx_phy->base + PHY_CRCTL;
	void __iomem	*cr_sr = imx_phy->base + PHY_CRSR;
	struct device	*dev = &imx_phy->phy->dev;
	int i, ret;

	/* Address Phrase */
	ret = imx8mq_phy_ctrl_reg_addr(imx_phy, offset);
	if (ret)
		return ret;

	writel(val, cr_ctrl);

	/* Set cr_cap_data to be 1'b1 */
	writel(readl(cr_ctrl) | PHY_CRCTL_CAP_DATA, cr_ctrl);
	/* Wait CRSR[16] == 1 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while (!(readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_write_err;

	/* Clear cr_cap_data to be 1'br0 */
	writel(readl(cr_ctrl) & ~PHY_CRCTL_CAP_DATA, cr_ctrl);
	/* Wait CRSR[16] == 0 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while ((readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_write_err;

	/* Set cr_write to be 1'b1 */
	writel(readl(cr_ctrl) | PHY_CRCTL_CR_WRITE, cr_ctrl);
	/* Wait CRSR[16] == 1 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while (!(readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_write_err;

	/* Clear cr_write to be 1'br0 */
	writel(readl(cr_ctrl) & ~PHY_CRCTL_CR_WRITE, cr_ctrl);
	/* Wait CRSR[16] == 0 */
	i = IMX8M_PHY_DEBUG_PORT_LOOP_TIMEOUT;
	while ((readl(cr_sr) & PHY_CRSR_CR_ACK) && i > 0)
		i--;
	if (i == 0)
		goto cr_write_err;

	return 0;

cr_write_err:
	dev_err(dev, "Failed to write reg 0x%x.\n", offset);

	return -EIO;

}

static int ctrl_reg_value_show(struct seq_file *s, void *unused)
{
	struct imx8mq_usb_phy	*imx_phy = s->private;
	u16			i, val, base = imx_phy->cr_access_base;

	for (i = 0; i < imx_phy->cr_read_count; i++) {
		val = imx8mq_phy_ctrl_reg_read(imx_phy, base + i);
		seq_printf(s, "Control Register 0x%x value is 0x%4x\n",
			   base + i, val);
	}

	return 0;
}

static int ctrl_reg_value_open(struct inode *inode, struct file *file)
{
	return single_open(file, ctrl_reg_value_show, inode->i_private);
}

static ssize_t ctrl_reg_value_write(struct file *file, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct seq_file		*s = file->private_data;
	struct imx8mq_usb_phy	*imx_phy = s->private;
	u16			cr_value;
	int			ret;

	ret = kstrtou16_from_user(ubuf, count, 16, &cr_value);
	if (ret)
		return ret;

	imx8mq_phy_ctrl_reg_write(imx_phy, imx_phy->cr_access_base, cr_value);

	return count;
}

static const struct file_operations ctrl_reg_value_fops = {
	.open		= ctrl_reg_value_open,
	.write		= ctrl_reg_value_write,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void debug_create_files(struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &imx_phy->phy->dev;

	imx_phy->debugfs = debugfs_create_dir(dev_name(dev),
					      imx_phy->phy->debugfs);

	debugfs_create_x16("ctrl_reg_base", 0600, imx_phy->debugfs,
			   &imx_phy->cr_access_base);
	debugfs_create_x16("ctrl_reg_count", 0600, imx_phy->debugfs,
			   &imx_phy->cr_read_count);
	debugfs_create_file("ctrl_reg_value", 0600, imx_phy->debugfs,
			    imx_phy, &ctrl_reg_value_fops);

	imx_phy->cr_access_base = 0;
	imx_phy->cr_read_count = 1;
}

static void debug_remove_files(struct imx8mq_usb_phy *imx_phy)
{
	debugfs_remove_recursive(imx_phy->debugfs);
}

static void imx8m_get_phy_tuning_data(struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = imx_phy->phy->dev.parent;

	if (device_property_read_u32(dev, "fsl,phy-tx-vref-tune-percent",
				     &imx_phy->tx_vref_tune))
		imx_phy->tx_vref_tune = PHY_TUNE_DEFAULT;
	else
		imx_phy->tx_vref_tune =
			phy_tx_vref_tune_from_property(imx_phy->tx_vref_tune);

	if (device_property_read_u32(dev, "fsl,phy-tx-rise-tune-percent",
				     &imx_phy->tx_rise_tune))
		imx_phy->tx_rise_tune = PHY_TUNE_DEFAULT;
	else
		imx_phy->tx_rise_tune =
			phy_tx_rise_tune_from_property(imx_phy->tx_rise_tune);

	if (device_property_read_u32(dev, "fsl,phy-tx-preemp-amp-tune-microamp",
				     &imx_phy->tx_preemp_amp_tune))
		imx_phy->tx_preemp_amp_tune = PHY_TUNE_DEFAULT;
	else
		imx_phy->tx_preemp_amp_tune =
			phy_tx_preemp_amp_tune_from_property(imx_phy->tx_preemp_amp_tune);

	if (device_property_read_u32(dev, "fsl,phy-tx-vboost-level-microvolt",
				     &imx_phy->tx_vboost_level))
		imx_phy->tx_vboost_level = PHY_TUNE_DEFAULT;
	else
		imx_phy->tx_vboost_level =
			phy_tx_vboost_level_from_property(imx_phy->tx_vboost_level);

	if (device_property_read_u32(dev, "fsl,phy-comp-dis-tune-percent",
				     &imx_phy->comp_dis_tune))
		imx_phy->comp_dis_tune = PHY_TUNE_DEFAULT;
	else
		imx_phy->comp_dis_tune =
			phy_comp_dis_tune_from_property(imx_phy->comp_dis_tune);

	if (device_property_read_u32(dev, "fsl,phy-pcs-tx-deemph-3p5db-attenuation-db",
				     &imx_phy->pcs_tx_deemph_3p5db))
		imx_phy->pcs_tx_deemph_3p5db = PHY_TUNE_DEFAULT;
	else
		imx_phy->pcs_tx_deemph_3p5db =
			phy_pcs_tx_deemph_3p5db_from_property(imx_phy->pcs_tx_deemph_3p5db);

	if (device_property_read_u32(dev, "fsl,phy-pcs-tx-swing-full-percent",
				     &imx_phy->pcs_tx_swing_full))
		imx_phy->pcs_tx_swing_full = PHY_TUNE_DEFAULT;
	else
		imx_phy->pcs_tx_swing_full =
			phy_pcs_tx_swing_full_from_property(imx_phy->pcs_tx_swing_full);
}

static void imx8m_phy_tune(struct imx8mq_usb_phy *imx_phy)
{
	u32 value;

	/* PHY tuning */
	if (imx_phy->pcs_tx_deemph_3p5db != PHY_TUNE_DEFAULT) {
		value = readl(imx_phy->base + PHY_CTRL4);
		value &= ~PHY_CTRL4_PCS_TX_DEEMPH_3P5DB_MASK;
		value |= FIELD_PREP(PHY_CTRL4_PCS_TX_DEEMPH_3P5DB_MASK,
				   imx_phy->pcs_tx_deemph_3p5db);
		writel(value, imx_phy->base + PHY_CTRL4);
	}

	if (imx_phy->pcs_tx_swing_full != PHY_TUNE_DEFAULT) {
		value = readl(imx_phy->base + PHY_CTRL5);
		value |= FIELD_PREP(PHY_CTRL5_PCS_TX_SWING_FULL_MASK,
				   imx_phy->pcs_tx_swing_full);
		writel(value, imx_phy->base + PHY_CTRL5);
	}

	if ((imx_phy->tx_vref_tune & imx_phy->tx_rise_tune &
	     imx_phy->tx_preemp_amp_tune & imx_phy->comp_dis_tune &
	     imx_phy->tx_vboost_level) == PHY_TUNE_DEFAULT)
		/* If all are the default values, no need update. */
		return;

	value = readl(imx_phy->base + PHY_CTRL3);

	if (imx_phy->tx_vref_tune != PHY_TUNE_DEFAULT) {
		value &= ~PHY_CTRL3_TXVREF_TUNE_MASK;
		value |= FIELD_PREP(PHY_CTRL3_TXVREF_TUNE_MASK,
				   imx_phy->tx_vref_tune);
	}

	if (imx_phy->tx_rise_tune != PHY_TUNE_DEFAULT) {
		value &= ~PHY_CTRL3_TXRISE_TUNE_MASK;
		value |= FIELD_PREP(PHY_CTRL3_TXRISE_TUNE_MASK,
				    imx_phy->tx_rise_tune);
	}

	if (imx_phy->tx_preemp_amp_tune != PHY_TUNE_DEFAULT) {
		value &= ~PHY_CTRL3_TXPREEMP_TUNE_MASK;
		value |= FIELD_PREP(PHY_CTRL3_TXPREEMP_TUNE_MASK,
				imx_phy->tx_preemp_amp_tune);
	}

	if (imx_phy->comp_dis_tune != PHY_TUNE_DEFAULT) {
		value &= ~PHY_CTRL3_COMPDISTUNE_MASK;
		value |= FIELD_PREP(PHY_CTRL3_COMPDISTUNE_MASK,
				    imx_phy->comp_dis_tune);
	}

	if (imx_phy->tx_vboost_level != PHY_TUNE_DEFAULT) {
		value &= ~PHY_CTRL3_TX_VBOOST_LEVEL_MASK;
		value |= FIELD_PREP(PHY_CTRL3_TX_VBOOST_LEVEL_MASK,
				    imx_phy->tx_vboost_level);
	}

	writel(value, imx_phy->base + PHY_CTRL3);
}

static int imx8mq_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		   PHY_CTRL1_COMMONONN);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	value &= ~PHY_CTRL0_SSC_RANGE_MASK;
	value |= PHY_CTRL0_SSC_RANGE_4003PPM;
	writel(value, imx_phy->base + PHY_CTRL0);

	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0;
	writel(value, imx_phy->base + PHY_CTRL2);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	return 0;
}

static int imx8mp_usb_phy_init(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* USB3.0 PHY signal fsel for 24M ref */
	value = readl(imx_phy->base + PHY_CTRL0);
	value &= ~PHY_CTRL0_FSEL_MASK;
	value |= FIELD_PREP(PHY_CTRL0_FSEL_MASK, PHY_CTRL0_FSEL_24M);
	writel(value, imx_phy->base + PHY_CTRL0);

	/* Disable alt_clk_en and use internal MPLL clocks */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~(PHY_CTRL6_ALT_CLK_SEL | PHY_CTRL6_ALT_CLK_EN);
	writel(value, imx_phy->base + PHY_CTRL6);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0);
	value |= PHY_CTRL1_RESET | PHY_CTRL1_ATERESET;
	writel(value, imx_phy->base + PHY_CTRL1);

	value = readl(imx_phy->base + PHY_CTRL0);
	value |= PHY_CTRL0_REF_SSP_EN;
	writel(value, imx_phy->base + PHY_CTRL0);

	value = readl(imx_phy->base + PHY_CTRL2);
	value |= PHY_CTRL2_TXENABLEN0 | PHY_CTRL2_OTG_DISABLE;
	writel(value, imx_phy->base + PHY_CTRL2);

	udelay(10);

	value = readl(imx_phy->base + PHY_CTRL1);
	value &= ~(PHY_CTRL1_RESET | PHY_CTRL1_ATERESET);
	writel(value, imx_phy->base + PHY_CTRL1);

	imx8m_phy_tune(imx_phy);

	if (imx_phy->tca)
		tca_blk_init(imx_phy->tca);

	return 0;
}

static int imx8mq_phy_power_on(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;
	int ret;

	ret = regulator_enable(imx_phy->vbus);
	if (ret)
		return ret;

	ret = clk_prepare_enable(imx_phy->clk);
	if (ret)
		return ret;

	/* Disable rx term override */
	value = readl(imx_phy->base + PHY_CTRL6);
	value &= ~PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	return 0;
}

static int imx8mq_phy_power_off(struct phy *phy)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);
	u32 value;

	/* Override rx term to be 0 */
	value = readl(imx_phy->base + PHY_CTRL6);
	value |= PHY_CTRL6_RXTERM_OVERRIDE_SEL;
	writel(value, imx_phy->base + PHY_CTRL6);

	clk_disable_unprepare(imx_phy->clk);
	regulator_disable(imx_phy->vbus);

	return 0;
}

static int imx8mq_chg_data_contact_det(struct imx8mq_usb_phy *imx_phy)
{
	int i, data_pin_contact_count = 0;
	u32 val;

	/* Set DMPULLDOWN<#> = 1'b1 (to enable RDM_DWN) */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Set DPPULLDOWN<#> = 1'b0 */
	val = readl(imx_phy->base + PHY_CTRL5);
	val |= PHY_CTRL5_DMPWD_OVERRIDE_SEL | PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	/* Enable Data Contact Detect (DCD) per the USB BC 1.2 */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_DCDENB, imx_phy->base + PHY_CTRL1);

	for (i = 0; i < 100; i = i + 1) {
		val = readl(imx_phy->base + PHY_STS0);
		/* DP is low */
		if (!(val & PHY_STS0_FSVPLUS)) {
			if (data_pin_contact_count++ > 5)
				/* Data pin makes contact */
				break;
			usleep_range(5000, 10000);
		} else {
			data_pin_contact_count = 0;
			usleep_range(5000, 6000);
		}
	}

	/* Disable DCD after finished data contact check */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_DCDENB;
	writel(val, imx_phy->base + PHY_CTRL1);

	if (i == 100) {
		dev_err(&imx_phy->phy->dev,
			"VBUS is coming from a dedicated power supply.\n");

		/* disable override before finish */
		val = readl(imx_phy->base + PHY_CTRL5);
		val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
		writel(val, imx_phy->base + PHY_CTRL5);

		return -ENXIO;
	}

	/* Set DMPULLDOWN<#> to 1'b0 when DCD is completed */
	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~PHY_CTRL5_DMPWD_OVERRIDE_SEL;
	val |= PHY_CTRL5_DMPWD_OVERRIDE;
	writel(val, imx_phy->base + PHY_CTRL5);

	return 0;
}

static int imx8mq_chg_primary_detect(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDP_SRC is connected to D+ and IDM_SINK is connected to D- */
	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~PHY_CTRL1_CHRGSEL;
	val |= PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0;
	writel(val, imx_phy->base + PHY_CTRL1);

	msleep(40);

	/* Check if D- is less than VDAT_REF to determine an SDP per BC 1.2 */
	val = readl(imx_phy->base + PHY_STS0);
	if (!(val & PHY_STS0_CHGDET)) {
		dev_dbg(&imx_phy->phy->dev, "It is a SDP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_SDP;
	}

	return 0;
}

static int imx8mq_phy_chg_secondary_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	/* VDM_SRC is connected to D- and IDP_SINK is connected to D+ */
	val = readl(imx_phy->base + PHY_CTRL1);
	writel(val | PHY_CTRL1_VDATSRCENB0 | PHY_CTRL1_VDATDETENB0 |
		PHY_CTRL1_CHRGSEL, imx_phy->base + PHY_CTRL1);

	msleep(40);

	/*
	 * Per BC 1.2, check voltage of D+:
	 * DCP: if greater than VDAT_REF;
	 * CDP: if less than VDAT0_REF.
	 */
	val = readl(imx_phy->base + PHY_STS0);
	if (val & PHY_STS0_CHGDET) {
		dev_dbg(&imx_phy->phy->dev, "It is a DCP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_DCP;
	} else {
		dev_dbg(&imx_phy->phy->dev, "It is a CDP.\n");
		imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_CDP;
	}

	return 0;
}

static void imx8mq_phy_disable_chg_det(struct imx8mq_usb_phy *imx_phy)
{
	u32 val;

	val = readl(imx_phy->base + PHY_CTRL5);
	val &= ~(PHY_CTRL5_DMPWD_OVERRIDE | PHY_CTRL5_DPPWD_OVERRIDE);
	writel(val, imx_phy->base + PHY_CTRL5);

	val = readl(imx_phy->base + PHY_CTRL1);
	val &= ~(PHY_CTRL1_DCDENB | PHY_CTRL1_VDATSRCENB0 |
		 PHY_CTRL1_VDATDETENB0 | PHY_CTRL1_CHRGSEL);
	writel(val, imx_phy->base + PHY_CTRL1);
}

static int imx8mq_phy_charger_detect(struct imx8mq_usb_phy *imx_phy)
{
	struct device *dev = &imx_phy->phy->dev;
	struct device_node *np = dev->parent->of_node;
	union power_supply_propval propval;
	u32 value;
	int ret = 0;

	if (!np)
		return 0;

	imx_phy->vbus_power_supply = power_supply_get_by_phandle(np,
						"vbus-power-supply");
	if (IS_ERR_OR_NULL(imx_phy->vbus_power_supply))
		return 0;

	if (imx_phy->chg_type != POWER_SUPPLY_USB_TYPE_UNKNOWN)
		goto put_psy;

	ret = power_supply_get_property(imx_phy->vbus_power_supply,
					POWER_SUPPLY_PROP_ONLINE,
					&propval);
	if (ret || propval.intval == 0) {
		dev_err(dev, "failed to get psy online infor\n");
		ret = -EINVAL;
		goto put_psy;
	}

	/* Check if vbus is valid */
	value = readl(imx_phy->base + PHY_STS0);
	if (!(value & PHY_STS0_OTGSESSVLD)) {
		dev_err(&imx_phy->phy->dev, "vbus is error\n");
		ret = -EINVAL;
		goto put_psy;
	}

	imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	ret = imx8mq_chg_data_contact_det(imx_phy);
	if (ret)
		goto put_psy;

	ret = imx8mq_chg_primary_detect(imx_phy);
	if (!ret && imx_phy->chg_type != POWER_SUPPLY_USB_TYPE_SDP)
		ret = imx8mq_phy_chg_secondary_det(imx_phy);

	imx8mq_phy_disable_chg_det(imx_phy);

	if (!ret) {
		propval.intval = imx_phy->chg_type;
		power_supply_set_property(imx_phy->vbus_power_supply,
					  POWER_SUPPLY_PROP_USB_TYPE,
					  &propval);
	}

put_psy:
	power_supply_put(imx_phy->vbus_power_supply);

	return ret;
}

static int imx8mq_phy_usb_vbus_notify(struct notifier_block *nb,
				      unsigned long val, void *v)
{
	struct imx8mq_usb_phy *imx_phy = container_of(nb, struct imx8mq_usb_phy,
						      chg_det_nb);
	struct device *dev = &imx_phy->phy->dev;
	struct device_node *np = dev->parent->of_node;
	union power_supply_propval propval;
	struct power_supply *psy = v;
	int ret;

	if (!np)
		return NOTIFY_DONE;

	imx_phy->vbus_power_supply = power_supply_get_by_phandle(np,
						"vbus-power-supply");
	if (IS_ERR_OR_NULL(imx_phy->vbus_power_supply)) {
		dev_err(dev, "failed to get power supply\n");
		return NOTIFY_DONE;
	}

	if (val == PSY_EVENT_PROP_CHANGED && psy == imx_phy->vbus_power_supply) {
		ret = power_supply_get_property(imx_phy->vbus_power_supply,
						POWER_SUPPLY_PROP_ONLINE,
						&propval);
		if (ret) {
			power_supply_put(imx_phy->vbus_power_supply);
			dev_err(dev, "failed to get psy online info\n");
			return NOTIFY_DONE;
		}

		if (propval.intval == 0)
			imx_phy->chg_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	}
	power_supply_put(imx_phy->vbus_power_supply);

	return NOTIFY_OK;
}

static int imx8mq_phy_set_mode(struct phy *phy, enum phy_mode mode,
			       int submode)
{
	struct imx8mq_usb_phy *imx_phy = phy_get_drvdata(phy);

	if (mode == PHY_MODE_USB_DEVICE)
		return imx8mq_phy_charger_detect(imx_phy);

	return 0;
}

static const struct phy_ops imx8mq_usb_phy_ops = {
	.init		= imx8mq_usb_phy_init,
	.power_on	= imx8mq_phy_power_on,
	.power_off	= imx8mq_phy_power_off,
	.set_mode	= imx8mq_phy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct phy_ops imx8mp_usb_phy_ops = {
	.init		= imx8mp_usb_phy_init,
	.power_on	= imx8mq_phy_power_on,
	.power_off	= imx8mq_phy_power_off,
	.set_mode	= imx8mq_phy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct of_device_id imx8mq_usb_phy_of_match[] = {
	{.compatible = "fsl,imx8mq-usb-phy",
	 .data = &imx8mq_usb_phy_ops,},
	{.compatible = "fsl,imx8mp-usb-phy",
	 .data = &imx8mp_usb_phy_ops,},
	{.compatible = "fsl,imx95-usb-phy",
	 .data = &imx8mp_usb_phy_ops,},
	{ }
};
MODULE_DEVICE_TABLE(of, imx8mq_usb_phy_of_match);

static int imx8mq_usb_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct imx8mq_usb_phy *imx_phy;
	const struct phy_ops *phy_ops;

	imx_phy = devm_kzalloc(dev, sizeof(*imx_phy), GFP_KERNEL);
	if (!imx_phy)
		return -ENOMEM;

	imx_phy->clk = devm_clk_get(dev, "phy");
	if (IS_ERR(imx_phy->clk)) {
		dev_err(dev, "failed to get imx8mq usb phy clock\n");
		return PTR_ERR(imx_phy->clk);
	}

	imx_phy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(imx_phy->base))
		return PTR_ERR(imx_phy->base);

	phy_ops = of_device_get_match_data(dev);
	if (!phy_ops)
		return -EINVAL;

	imx_phy->phy = devm_phy_create(dev, NULL, phy_ops);
	if (IS_ERR(imx_phy->phy))
		return PTR_ERR(imx_phy->phy);

	imx_phy->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(imx_phy->vbus))
		return dev_err_probe(dev, PTR_ERR(imx_phy->vbus), "failed to get vbus\n");

	phy_set_drvdata(imx_phy->phy, imx_phy);
	platform_set_drvdata(pdev, imx_phy);

	if (device_property_present(dev, "vbus-power-supply")) {
		imx_phy->chg_det_nb.notifier_call = imx8mq_phy_usb_vbus_notify;
		power_supply_reg_notifier(&imx_phy->chg_det_nb);
	}

	if (device_is_compatible(dev, "fsl,imx95-usb-phy") &&
		imx95_usb_phy_get_tca(pdev, imx_phy) < 0) {
		dev_err(dev, "failed to get tca\n");
		return -ENODEV;
	}

	imx8m_get_phy_tuning_data(imx_phy);

	debug_create_files(imx_phy);

	device_set_wakeup_capable(dev, true);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static void imx8mq_usb_phy_remove(struct platform_device *pdev)
{
	struct imx8mq_usb_phy *imx_phy = platform_get_drvdata(pdev);

	imx95_usb_phy_put_tca(imx_phy);

	if (device_property_present(&pdev->dev, "vbus-power-supply"))
		power_supply_unreg_notifier(&imx_phy->chg_det_nb);

	debug_remove_files(imx_phy);
}

static struct platform_driver imx8mq_usb_phy_driver = {
	.probe	= imx8mq_usb_phy_probe,
	.remove = imx8mq_usb_phy_remove,
	.driver = {
		.name	= "imx8mq-usb-phy",
		.of_match_table	= imx8mq_usb_phy_of_match,
	}
};
module_platform_driver(imx8mq_usb_phy_driver);

MODULE_DESCRIPTION("FSL IMX8MQ USB PHY driver");
MODULE_LICENSE("GPL");
