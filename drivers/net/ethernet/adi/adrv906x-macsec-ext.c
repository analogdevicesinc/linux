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

#define ADRV906X_MAX_TX_KEYS            4
#define ADRV906X_MAX_TX_CHANNELS        1
#define ADRV906X_MAX_RX_KEYS            32
#define ADRV906X_MAX_RX_CHANNELS        8
#define ADRV906X_NO_OF_KEY_ENTRIES_RX   255

struct cco_macsec_priv *cco_macsec_get_priv(struct net_device *netdev)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);

	return &macsec->priv;
}

static void *cco_macsec_get_base(struct net_device *netdev)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);

	return macsec->base;
}

void cco_macsec_reg_wr(struct net_device *netdev, unsigned long addr, u32 value)
{
	void __iomem *reg_ptr = (void __iomem *)cco_macsec_get_base(netdev);

	iowrite32(value, reg_ptr + addr);
}

u32  cco_macsec_reg_rd(struct net_device *netdev, unsigned long addr)
{
	void __iomem *reg_ptr = (void __iomem *)cco_macsec_get_base(netdev);

	return ioread32(reg_ptr + addr);
}

void cco_macsec_max_framesize_get(struct net_device *netdev, u32 *max_framesize)
{
	*max_framesize = netdev->mtu + 14 + 8; // DMAC, SMAC, EthType, 2 VLAN tags
}

/*
 * static irqreturn_t adi_macsec_isr(int irq, void *dev_id)
 * {
 * 	return IRQ_HANDLED;
 * }
 */
void adrv906x_macsec_commonport_status_update(struct net_device *netdev)
{
	cco_macsec_commonport_status_update(netdev, 1, 1);
}

int adrv906x_macsec_probe(struct platform_device *pdev, struct net_device *netdev,
			  struct device_node *np, struct adrv906x_macsec_priv *macsec)
{
	struct device *dev = &pdev->dev;
	struct cco_macsec_capabilities *p;
	u32 reg, len;
	int ret;

	macsec->dev = dev;

	if (of_property_read_u32_index(np, "reg", 0, &reg))
		return -EINVAL;
	if (of_property_read_u32_index(np, "reg", 1, &len))
		return -EINVAL;

	macsec->base = devm_ioremap(dev, reg, len);
	if (!macsec->base) {
		dev_err(dev, "ioremap macsec membase failed!");
		return -ENOMEM;
	}

	macsec->irq = of_irq_get_byname(np, "ts_event");
	if (macsec->irq <= 0) {
		dev_err(dev, "cannot obtain macsec ts_event irq");
		return -EINVAL;
	}

	dev_info(dev, "macsec irq %d", macsec->irq);

/* TODO turn it on after basic MACsec functinality is tested
 *	ret = devm_request_irq(dev, macsec_irq, adi_macsec_isr, IRQF_SHARED, "macsec", dev);
 *	if (val) {
 *		dev_err(dev, "unable to register macsec interrupt %d", macsec_irq);
 *		return -EFAULT;
 *	}
 */

	ret = cco_macsec_init(netdev);
	if (ret) {
		dev_err(dev, "failed to initialize macsec hw offload driver");
		return ret;
	}

	/* Both SecY tx_config and rx_config registers can only be accessed when
	 * the macsec_bypass_mode is cleared in digital_control_0 register.
	 * Accessing SecY rx_config register also requires Ethernet RX recovered
	 * clock is available. Thus, cco_macsec_init() cannot get the correct
	 * values from these two registers. The following hack fix the values.
	 */
	p = &macsec->priv.capabilities;
	p->maxTxKeys = ADRV906X_MAX_TX_KEYS;
	p->maxTxChannels = ADRV906X_MAX_TX_CHANNELS;
	p->maxRxKeys = ADRV906X_MAX_RX_KEYS;
	p->maxRxChannels = ADRV906X_MAX_RX_CHANNELS;
	netdev_info(netdev, "%s: maxTxKeys=%u maxTxChannels=%u maxRxKeys=%u maxRxChannels=%u (corrected)\n",
		    __func__, p->maxTxKeys, p->maxTxChannels, p->maxRxKeys, p->maxRxChannels);

	/* MACsec IP in ADRV906x is configured with 256 keys in ingress.
	 * But both the ip_capabilities_1 register bitfield and Comcores'
	 * MACsec driver can only support up to 255 keys. The following hack
	 * fixes this issue.
	 */
	p->no_of_key_entries_rx = ADRV906X_NO_OF_KEY_ENTRIES_RX;
	netdev_info(netdev, "%s: no_of_key_entries_rx=%u (corrected)\n",
		    __func__, p->no_of_key_entries_rx);

	return 0;
}

void adrv906x_macsec_remove(struct net_device *netdev)
{
	struct adrv906x_macsec_priv *macsec = adrv906x_macsec_get(netdev);

	if (macsec) {
		cco_macsec_exit(netdev);
		macsec = NULL;
	}
}

MODULE_LICENSE("GPL");
