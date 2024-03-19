// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include "adrv906x-macsec-ext.h"

struct cco_macsec_priv *cco_macsec_get_priv(struct net_device *netdev)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);

	return &macsec->priv;
}

void *cco_macsec_get_base(struct net_device *netdev)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);

	return macsec->base;
}

void cco_macsec_reg_wr(struct net_device *netdev, unsigned long addr, u32 value)
{
//	void __iomem *reg_ptr = (void __iomem *)cco_macsec_get_base(netdev);
//	printk("adi- write reg %s(0x%lx, 0x%08x) reg_ptr=0x%lx\n",
	//	__func__, addr, value, reg_ptr);
//	iowrite32(value, reg_ptr + addr);
}

u32  cco_macsec_reg_rd(struct net_device *netdev, unsigned long addr)
{
	u32 val = 0;

//	void __iomem *reg_ptr = (void __iomem *)cco_macsec_get_base(netdev);
//	val = ioread32(reg_ptr + addr );
//	printk("adi- read reg %s(0x%lx), return 0x%08x (reg_ptr=0x%lx)\n",
	//	__func__, addr, val, reg_ptr);

	// simulate values for some of the register reads here:
	switch (addr) {
	case (MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_ID_BASE_ADDR):
		val = CCO_MACSEC_IP_ID;
		break;
	case (MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_VERSION_BASE_ADDR):
		val = (CCO_MACSEC_MAJOR_VER << MACSEC_CORE_IP_VERSION_MAJOR_SHIFT);
		break;
	case (MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CAPABILITIES_1_BASE_ADDR):
		val = ((4 << MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_PEERS_SHIFT) |
		       (16 << MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_RX_SHIFT) |
		       (16 << MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_TX_SHIFT) |
		       (4 << MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_SECYS_SHIFT));
		break;
	case (MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CAPABILITIES_2_BASE_ADDR):
		val = ((CCO_CS_AES_GCM_128 << MACSEC_CORE_IP_CAPABILITIES_2_AVAILABLE_CIPHERSUITES_SHIFT) |
		       (1 << MACSEC_CORE_IP_CAPABILITIES_2_VLAN_IN_CLEAR_SHIFT) |
		       (8 << MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_RX_SHIFT) |
		       (8 << MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_TX_SHIFT));
		break;
	case (MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CS_CAPABILITY_BASE_ADDR):
		val = (16 << MACSEC_CORE_IP_CS_CAPABILITY_ICVLENGTH_SHIFT);
		break;
	case (SECY_CONFIG_BASE_ADDR + SECY_CONFIG_TX_CONFIG_BASE_ADDR):
		val = ((8 << SECY_CONFIG_TX_CONFIG_MAXTRANSMITKEYS_SHIFT) |
		       (8 << SECY_CONFIG_TX_CONFIG_MAXTRANSMITCHANNELS_SHIFT));
		break;
	case (SECY_CONFIG_BASE_ADDR + SECY_CONFIG_RX_CONFIG_BASE_ADDR):
		val = ((8 << SECY_CONFIG_RX_CONFIG_MAXRECEIVEKEYS_SHIFT) |
		       (8 << SECY_CONFIG_RX_CONFIG_MAXRECEIVECHANNELS_SHIFT));
		break;
	default:
		break;
	}
	return val;
}

void cco_macsec_commonport_status_get(struct net_device *netdev, u8 *operational, u8 *enabled)
{
	// just always return up
	*operational = 1;
	*enabled = 1;
}

void cco_macsec_max_framesize_get(struct net_device *netdev, u32 *max_framesize)
{
	*max_framesize = netdev->mtu + 14 + 8; // DMAC, SMAC, EthType, 2 VLAN tags
}

irqreturn_t adi_macsec_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

void adrv906x_macsec_commonport_status_update(struct net_device *netdev)
{
	cco_macsec_commonport_status_update(netdev, 1, 1);
}

int adrv906x_macsec_probe(struct platform_device *pdev, struct net_device *netdev,
			  struct device_node *np)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);
	struct device *dev = &pdev->dev;
	u32 reg, len;
	int ret;

	macsec->dev = dev;

	if (of_property_read_u32_index(np, "macsec", 0, &reg))
		goto error;
	if (of_property_read_u32_index(np, "macsec", 1, &len))
		goto error;

	macsec->base = devm_ioremap(dev, reg, len);
	if (!macsec->base) {
		dev_err(dev, "ioremap macsec membase failed!");
		goto error;
	}

	macsec->irq = of_irq_get_byname(np, "ts_event");
	if (macsec->irq <= 0) {
		dev_err(dev, "cannot obtain MACsec ts_event IRQ");
		goto error;
	}

	dev_info(dev, "macsec irq %d", macsec->irq);


/* TODO turn it on after basic MACsec functinality is tested
 *	ret = devm_request_irq(dev, macsec_irq, adi_macsec_isr, IRQF_SHARED, "macsec", dev);
 *	if (val) {
 *		dev_err(dev, "unable to register macsec interrupt %d", macsec_irq);
 *		return -EFAULT;
 *	}
 */

	macsec->enabled = 1;

	ret = cco_macsec_init(netdev);
	if (ret)
		dev_err(dev, "Failed to initialize macsec HW offload driver");
	else
		dev_info(dev, "macsec initialized, enabled: %d, base: 0x%px, irq: %d",
			 macsec->enabled, macsec->base, macsec->irq);
	return ret;

error:
	macsec->enabled = 0;
	return 0;
}

MODULE_LICENSE("GPL");
