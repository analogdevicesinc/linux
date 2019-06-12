// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Freescale QorIQ AHCI SATA platform driver
 *
 * Copyright 2015 Freescale, Inc.
 *   Tang Yuantian <Yuantian.Tang@freescale.com>
 */

#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/ahci_platform.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/libata.h>
#include "ahci.h"

#define DRV_NAME "ahci-qoriq"

/* port register definition */
#define PORT_PHY1	0xA8
#define PORT_PHY2	0xAC
#define PORT_PHY3	0xB0
#define PORT_PHY4	0xB4
#define PORT_PHY5	0xB8
#define PORT_AXICC	0xBC
#define PORT_TRANS	0xC8

/* port register default value */
#define AHCI_PORT_PHY_1_CFG	0xa003fffe
#define AHCI_PORT_PHY2_CFG	0x28184d1f
#define AHCI_PORT_PHY3_CFG	0x0e081509
#define AHCI_PORT_TRANS_CFG	0x08000029
#define AHCI_PORT_AXICC_CFG	0x3fffffff

/* for ls1021a */
#define LS1021A_PORT_PHY2	0x28183414
#define LS1021A_PORT_PHY3	0x0e080e06
#define LS1021A_PORT_PHY4	0x064a080b
#define LS1021A_PORT_PHY5	0x2aa86470
#define LS1021A_AXICC_ADDR	0xC0

#define SATA_ECC_DISABLE	0x00020000
#define ECC_DIS_ARMV8_CH2	0x80000000
#define ECC_DIS_LS1088A		0x40000000

/* errata for lx2160 */
#define RCWSR29_BASE			0x1E00170
#define SERDES2_BASE			0x1EB0000
#define DEVICE_CONFIG_REG_BASE		0x1E00000
#define SERDES2_LNAX_RX_CR(x)		(0x840 + (0x100 * (x)))
#define SERDES2_LNAX_RX_CBR(x)		(0x8C0 + (0x100 * (x)))
#define SYS_VER_REG			0xA4
#define LN_RX_RST			0x80000010
#define LN_RX_RST_DONE			0x3
#define LN_RX_MASK			0xf
#define LX2160A_VER1			0x1

#define SERDES2_LNAA 0
#define SERDES2_LNAB 1
#define SERDES2_LNAC 2
#define SERDES2_LNAD 3
#define SERDES2_LNAE 4
#define SERDES2_LNAF 5
#define SERDES2_LNAG 6
#define SERDES2_LNAH 7

enum ahci_qoriq_type {
	AHCI_LS1021A,
	AHCI_LS1028A,
	AHCI_LS1043A,
	AHCI_LS2080A,
	AHCI_LS1046A,
	AHCI_LS1088A,
	AHCI_LS2088A,
	AHCI_LX2160A,
};

struct ahci_qoriq_priv {
	struct ccsr_ahci *reg_base;
	enum ahci_qoriq_type type;
	void __iomem *ecc_addr;
	bool is_dmacoherent;
};

static bool ecc_initialized;

static const struct of_device_id ahci_qoriq_of_match[] = {
	{ .compatible = "fsl,ls1021a-ahci", .data = (void *)AHCI_LS1021A},
	{ .compatible = "fsl,ls1028a-ahci", .data = (void *)AHCI_LS1028A},
	{ .compatible = "fsl,ls1043a-ahci", .data = (void *)AHCI_LS1043A},
	{ .compatible = "fsl,ls2080a-ahci", .data = (void *)AHCI_LS2080A},
	{ .compatible = "fsl,ls1046a-ahci", .data = (void *)AHCI_LS1046A},
	{ .compatible = "fsl,ls1088a-ahci", .data = (void *)AHCI_LS1088A},
	{ .compatible = "fsl,ls2088a-ahci", .data = (void *)AHCI_LS2088A},
	{ .compatible = "fsl,lx2160a-ahci", .data = (void *)AHCI_LX2160A},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ahci_qoriq_of_match);

static const struct acpi_device_id ahci_qoriq_acpi_match[] = {
	{"NXP0004", .driver_data = (kernel_ulong_t)AHCI_LX2160A},
	{ }
};
MODULE_DEVICE_TABLE(acpi, ahci_qoriq_acpi_match);

static void fsl_sata_errata_379364(bool select)
{
	int val = 0;
	void __iomem *rcw_base = NULL;
	void __iomem *serdes_base = NULL;
	void __iomem *dev_con_base = NULL;

	if (select) {
		dev_con_base = ioremap(DEVICE_CONFIG_REG_BASE, PAGE_SIZE);
		if (!dev_con_base)
			return;

		val = (readl(dev_con_base + SYS_VER_REG) & GENMASK(7, 4)) >> 4;
		if (val != LX2160A_VER1)
			goto dev_unmap;

		/*
		 * Add few msec delay.
		 * Check for corresponding serdes lane RST_DONE .
		 * apply lane reset.
		 */

		serdes_base = ioremap(SERDES2_BASE, PAGE_SIZE);
		if (!serdes_base)
			goto dev_unmap;

		rcw_base = ioremap(RCWSR29_BASE, PAGE_SIZE);
		if (!rcw_base)
			goto serdes_unmap;

		msleep(20);

		val = (readl(rcw_base) & GENMASK(25, 21)) >> 21;

		switch (val) {
		case 1:
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAC)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAC));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAD)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAD));
			break;

		case 4:
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAG)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAG));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAH)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAH));
			break;

		case 5:
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAE)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAE));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAF)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAF));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAG)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAG));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAH)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAH));
			break;

		case 8:
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAC)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAC));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAD)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAD));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAE)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAE));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAF)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAF));
			break;

		case 12:
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAG)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAG));
			if ((readl(serdes_base + SERDES2_LNAX_RX_CBR(SERDES2_LNAH)) &
				LN_RX_MASK) != LN_RX_RST_DONE)
				writel(LN_RX_RST, serdes_base +
					SERDES2_LNAX_RX_CR(SERDES2_LNAH));
			break;

		default:
			break;
		}
	} else {
		return;
	}

	iounmap(rcw_base);
serdes_unmap:
	iounmap(serdes_base);
dev_unmap:
	iounmap(dev_con_base);
}

static int ahci_qoriq_hardreset(struct ata_link *link, unsigned int *class,
			  unsigned long deadline)
{
	const unsigned int *timing = sata_ehc_deb_timing(&link->eh_context);
	void __iomem *port_mmio = ahci_port_base(link->ap);
	u32 px_cmd, px_is, px_val;
	struct ata_port *ap = link->ap;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	struct ahci_qoriq_priv *qoriq_priv = hpriv->plat_data;
	u8 *d2h_fis = pp->rx_fis + RX_FIS_D2H_REG;
	struct ata_taskfile tf;
	bool online;
	int rc;
	bool ls1021a_workaround = (qoriq_priv->type == AHCI_LS1021A);
	bool lx2160a_workaround = (qoriq_priv->type == AHCI_LX2160A);

	hpriv->stop_engine(ap);

	/*
	 * There is a errata on ls1021a Rev1.0 and Rev2.0 which is:
	 * A-009042: The device detection initialization sequence
	 * mistakenly resets some registers.
	 *
	 * Workaround for this is:
	 * The software should read and store PxCMD and PxIS values
	 * before issuing the device detection initialization sequence.
	 * After the sequence is complete, software should restore the
	 * PxCMD and PxIS with the stored values.
	 */
	if (ls1021a_workaround) {
		px_cmd = readl(port_mmio + PORT_CMD);
		px_is = readl(port_mmio + PORT_IRQ_STAT);
	}

	/* clear D2H reception area to properly wait for D2H FIS */
	ata_tf_init(link->device, &tf);
	tf.status = ATA_BUSY;
	ata_tf_to_fis(&tf, 0, 0, d2h_fis);

	fsl_sata_errata_379364(lx2160a_workaround);

	rc = sata_link_hardreset(link, timing, deadline, &online,
				 ahci_check_ready);

	/* restore the PxCMD and PxIS on ls1021 */
	if (ls1021a_workaround) {
		px_val = readl(port_mmio + PORT_CMD);
		if (px_val != px_cmd)
			writel(px_cmd, port_mmio + PORT_CMD);

		px_val = readl(port_mmio + PORT_IRQ_STAT);
		if (px_val != px_is)
			writel(px_is, port_mmio + PORT_IRQ_STAT);
	}

	hpriv->start_engine(ap);

	if (online)
		*class = ahci_dev_classify(ap);
	return rc;
}

static struct ata_port_operations ahci_qoriq_ops = {
	.inherits	= &ahci_ops,
	.hardreset	= ahci_qoriq_hardreset,
};

static const struct ata_port_info ahci_qoriq_port_info = {
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_NCQ,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_qoriq_ops,
};

static const struct scsi_host_template ahci_qoriq_sht = {
	AHCI_SHT(DRV_NAME),
};

static int ahci_qoriq_phy_init(struct ahci_host_priv *hpriv)
{
	struct ahci_qoriq_priv *qpriv = hpriv->plat_data;
	void __iomem *reg_base = hpriv->mmio;

	switch (qpriv->type) {
	case AHCI_LS1021A:
		if (!(qpriv->ecc_addr || ecc_initialized))
			return -EINVAL;
		else if (qpriv->ecc_addr && !ecc_initialized)
			writel(SATA_ECC_DISABLE, qpriv->ecc_addr);
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(LS1021A_PORT_PHY2, reg_base + PORT_PHY2);
		writel(LS1021A_PORT_PHY3, reg_base + PORT_PHY3);
		writel(LS1021A_PORT_PHY4, reg_base + PORT_PHY4);
		writel(LS1021A_PORT_PHY5, reg_base + PORT_PHY5);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG,
					reg_base + LS1021A_AXICC_ADDR);
		break;

	case AHCI_LS1043A:
		if (!(qpriv->ecc_addr || ecc_initialized))
			return -EINVAL;
		else if (qpriv->ecc_addr && !ecc_initialized)
			writel(readl(qpriv->ecc_addr) |
			       ECC_DIS_ARMV8_CH2,
			       qpriv->ecc_addr);
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(AHCI_PORT_PHY2_CFG, reg_base + PORT_PHY2);
		writel(AHCI_PORT_PHY3_CFG, reg_base + PORT_PHY3);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG, reg_base + PORT_AXICC);
		break;

	case AHCI_LS2080A:
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(AHCI_PORT_PHY2_CFG, reg_base + PORT_PHY2);
		writel(AHCI_PORT_PHY3_CFG, reg_base + PORT_PHY3);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG, reg_base + PORT_AXICC);
		break;

	case AHCI_LS1046A:
		if (!(qpriv->ecc_addr || ecc_initialized))
			return -EINVAL;
		else if (qpriv->ecc_addr && !ecc_initialized)
			writel(readl(qpriv->ecc_addr) |
			       ECC_DIS_ARMV8_CH2,
			       qpriv->ecc_addr);
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(AHCI_PORT_PHY2_CFG, reg_base + PORT_PHY2);
		writel(AHCI_PORT_PHY3_CFG, reg_base + PORT_PHY3);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG, reg_base + PORT_AXICC);
		break;

	case AHCI_LS1028A:
	case AHCI_LS1088A:
	case AHCI_LX2160A:
		if (!(qpriv->ecc_addr || ecc_initialized))
			return -EINVAL;
		else if (qpriv->ecc_addr && !ecc_initialized)
			writel(readl(qpriv->ecc_addr) |
			       ECC_DIS_LS1088A,
			       qpriv->ecc_addr);
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(AHCI_PORT_PHY2_CFG, reg_base + PORT_PHY2);
		writel(AHCI_PORT_PHY3_CFG, reg_base + PORT_PHY3);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG, reg_base + PORT_AXICC);
		break;

	case AHCI_LS2088A:
		writel(AHCI_PORT_PHY_1_CFG, reg_base + PORT_PHY1);
		writel(AHCI_PORT_PHY2_CFG, reg_base + PORT_PHY2);
		writel(AHCI_PORT_PHY3_CFG, reg_base + PORT_PHY3);
		writel(AHCI_PORT_TRANS_CFG, reg_base + PORT_TRANS);
		if (qpriv->is_dmacoherent)
			writel(AHCI_PORT_AXICC_CFG, reg_base + PORT_AXICC);
		break;
	}

	ecc_initialized = true;
	return 0;
}

static int ahci_qoriq_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct acpi_device_id *acpi_id;
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv;
	struct ahci_qoriq_priv *qoriq_priv;
	const struct of_device_id *of_id;
	struct resource *res;
	int rc;

	hpriv = ahci_platform_get_resources(pdev, 0);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	of_id = of_match_node(ahci_qoriq_of_match, np);
	acpi_id = acpi_match_device(ahci_qoriq_acpi_match, &pdev->dev);
	if (!(of_id || acpi_id))
		return -ENODEV;

	qoriq_priv = devm_kzalloc(dev, sizeof(*qoriq_priv), GFP_KERNEL);
	if (!qoriq_priv)
		return -ENOMEM;

	if (of_id)
		qoriq_priv->type = (unsigned long)of_id->data;
	else
		qoriq_priv->type = (enum ahci_qoriq_type)acpi_id->driver_data;

	if (unlikely(!ecc_initialized)) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (res) {
			qoriq_priv->ecc_addr =
				devm_ioremap_resource(dev, res);
			if (IS_ERR(qoriq_priv->ecc_addr))
				return PTR_ERR(qoriq_priv->ecc_addr);
		}
	}

	if (device_get_dma_attr(&pdev->dev) == DEV_DMA_COHERENT)
		qoriq_priv->is_dmacoherent = true;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	hpriv->plat_data = qoriq_priv;
	rc = ahci_qoriq_phy_init(hpriv);
	if (rc)
		goto disable_resources;

	rc = ahci_platform_init_host(pdev, hpriv, &ahci_qoriq_port_info,
				     &ahci_qoriq_sht);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);

	return rc;
}

#ifdef CONFIG_PM_SLEEP
static int ahci_qoriq_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	int rc;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	rc = ahci_qoriq_phy_init(hpriv);
	if (rc)
		goto disable_resources;

	rc = ahci_platform_resume_host(dev);
	if (rc)
		goto disable_resources;

	/* We resumed so update PM runtime state */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);

	return rc;
}
#endif

static SIMPLE_DEV_PM_OPS(ahci_qoriq_pm_ops, ahci_platform_suspend,
			 ahci_qoriq_resume);

static struct platform_driver ahci_qoriq_driver = {
	.probe = ahci_qoriq_probe,
	.remove_new = ata_platform_remove_one,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ahci_qoriq_of_match,
		.acpi_match_table = ahci_qoriq_acpi_match,
		.pm = &ahci_qoriq_pm_ops,
	},
};
module_platform_driver(ahci_qoriq_driver);

MODULE_DESCRIPTION("Freescale QorIQ AHCI SATA platform driver");
MODULE_AUTHOR("Tang Yuantian <Yuantian.Tang@freescale.com>");
MODULE_LICENSE("GPL");
