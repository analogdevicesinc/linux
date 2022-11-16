// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * interface to ADI SC5xx CANs
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#include <linux/can/dev.h>
#include <linux/can/error.h>
#include "sc5xx_can.h"
#include <mach/portmux.h>
#include <mach/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#define DRV_NAME			"sc5xx_can"
#define ADI_CAN_TIMEOUT		100
#define TX_ECHO_SKB_MAX		1

/* sc5xx can private data
 */
struct sc5xx_can_priv {
	struct can_priv can;	/* must be the first member */
	struct net_device *dev;
	struct resource *res_mem;
	void __iomem *membase;
	int rx_irq;
	int tx_irq;
	int err_irq;
	unsigned int enable_pin;
	unsigned int enable_pin_active_low;
	unsigned short *pin_list;
	struct spi_device *spi;
};

/* sc5xx can timing parameters
 */
static const struct can_bittiming_const sc5xx_can_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max = 4,
	/* Although the BRP field can be set to any value, it is recommended
	 * that the value be greater than or equal to 4, as restrictions
	 * apply to the bit timing configuration when BRP is less than 4.
	 */
	.brp_min = 4,
	.brp_max = 1024,
	.brp_inc = 1,
};

static int sc5xx_can_set_bittiming(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	struct can_bittiming *bt = &priv->can.bittiming;
	u16 clk, timing;

	clk = bt->brp - 1;
	timing = ((bt->sjw - 1) << 8) | (bt->prop_seg + bt->phase_seg1 - 1) |
		((bt->phase_seg2 - 1) << 4);

	/* If the SAM bit is set, the input signal is oversampled three times
	 * at the SCLK rate.
	 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		timing |= SAM;

	writew(clk, &reg->clock);
	writew(timing, &reg->timing);

	netdev_info(dev, "setting CLOCK=0x%04x TIMING=0x%04x\n", clk, timing);

	return 0;
}

static void sc5xx_can_set_reset_mode(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	int timeout = ADI_CAN_TIMEOUT;
	int i;

	/* disable interrupts */
	writew(0, &reg->mbim1);
	writew(0, &reg->mbim2);
	writew(0, &reg->gim);

	/* reset can and enter configuration mode */
	writew(SRS | CCR, &reg->control);
	writew(CCR, &reg->control);
	while (!(readw(&reg->control) & CCA)) {
		udelay_range(10, 100);
		if (--timeout == 0) {
			netdev_err(dev, "fail to enter configuration mode\n");
			WARN(1, "SC5XX CAN timeout took too long\n");
			return;
		}
	}

	/* All mailbox configurations are marked as inactive
	 * by writing to CAN Mailbox Configuration Registers 1 and 2
	 * For all bits: 0 - Mailbox disabled, 1 - Mailbox enabled
	 */
	writew(0, &reg->mc1);
	writew(0, &reg->mc2);

	/* Set Mailbox Direction */
	writew(0xFFFF, &reg->md1);   /* mailbox 1-16 are RX */
	writew(0, &reg->md2);   /* mailbox 17-32 are TX */

	/* RECEIVE_STD_CHL */
	for (i = 0; i < 2; i++) {
		writew(0, &reg->chl[RECEIVE_STD_CHL + i].id0);
		writew(AME, &reg->chl[RECEIVE_STD_CHL + i].id1);
		writew(0, &reg->chl[RECEIVE_STD_CHL + i].dlc);
		writew(0x1FFF, &reg->msk[RECEIVE_STD_CHL + i].amh);
		writew(0xFFFF, &reg->msk[RECEIVE_STD_CHL + i].aml);
	}

	/* RECEIVE_EXT_CHL */
	for (i = 0; i < 2; i++) {
		writew(0, &reg->chl[RECEIVE_EXT_CHL + i].id0);
		writew(AME | IDE, &reg->chl[RECEIVE_EXT_CHL + i].id1);
		writew(0, &reg->chl[RECEIVE_EXT_CHL + i].dlc);
		writew(0x1FFF, &reg->msk[RECEIVE_EXT_CHL + i].amh);
		writew(0xFFFF, &reg->msk[RECEIVE_EXT_CHL + i].aml);
	}

	writew(BIT(TRANSMIT_CHL - 16), &reg->mc2);
	writew(BIT(RECEIVE_STD_CHL) + BIT(RECEIVE_EXT_CHL), &reg->mc1);

	priv->can.state = CAN_STATE_STOPPED;
}

static void sc5xx_can_set_normal_mode(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	int timeout = ADI_CAN_TIMEOUT;

	/* leave configuration mode
	 */
	writew(readw(&reg->control) & ~CCR, &reg->control);

	while (readw(&reg->status) & CCA) {
		usleep_range(10, 100);
		if (--timeout == 0) {
			netdev_err(dev, "fail to leave configuration mode\n");
			WARN(1, "SC5XX CAN timeout took too long\n");
			return;
		}
	}

	/* clear _All_  tx and rx interrupts
	 */
	writew(0xFFFF, &reg->mbtif1);
	writew(0xFFFF, &reg->mbtif2);
	writew(0xFFFF, &reg->mbrif1);
	writew(0xFFFF, &reg->mbrif2);

	/* clear global interrupt status register
	 */
	writew(0x7FF, &reg->gis); /* overwrites with '1' */

	/* Initialize Interrupts
	 * - set bits in the mailbox interrupt mask register
	 * - global interrupt mask
	 */
	writew(BIT(RECEIVE_STD_CHL) + BIT(RECEIVE_EXT_CHL), &reg->mbim1);
	writew(BIT(TRANSMIT_CHL - 16), &reg->mbim2);

	writew(EPIM | BOIM | RMLIM, &reg->gim);
}

static void sc5xx_can_start(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);

	/* enter reset mode */
	if (priv->can.state != CAN_STATE_STOPPED)
		sc5xx_can_set_reset_mode(dev);

	/* leave reset mode */
	sc5xx_can_set_normal_mode(dev);
}

static int sc5xx_can_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		sc5xx_can_start(dev);
		if (netif_queue_stopped(dev))
			netif_wake_queue(dev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int sc5xx_can_get_berr_counter(const struct net_device *dev,
				      struct can_berr_counter *bec)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;

	u16 cec = readw(&reg->cec);

	bec->txerr = cec >> 8;
	bec->rxerr = cec;

	return 0;
}

static int sc5xx_can_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	struct can_frame *cf = (struct can_frame *)skb->data;
	u8 dlc = cf->can_dlc;
	canid_t id = cf->can_id;
	u8 *data = cf->data;
	u16 val;
	int i;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);

	/* fill id */
	if (id & CAN_EFF_FLAG) {
		writew(id, &reg->chl[TRANSMIT_CHL].id0);
		val = ((id & 0x1FFF0000) >> 16) | IDE;
	} else {
		val = (id << 2);
	}

	if (id & CAN_RTR_FLAG)
		val |= RTR;
	writew(val | AME, &reg->chl[TRANSMIT_CHL].id1);

	/* fill payload */
	for (i = 0; i < 8; i += 2) {
		val = ((7 - i) < dlc ? (data[7 - i]) : 0) +
			((6 - i) < dlc ? (data[6 - i] << 8) : 0);
		writew(val, &reg->chl[TRANSMIT_CHL].data[i]);
	}

	/* fill data length code */
	writew(dlc, &reg->chl[TRANSMIT_CHL].dlc);

	can_put_echo_skb(skb, dev, 0);

	/* set transmit request */
	writew(BIT(TRANSMIT_CHL - 16), &reg->trs2);

	return 0;
}

static void sc5xx_can_rx(struct net_device *dev, u16 isrc)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	struct can_frame *cf;
	struct sk_buff *skb;
	int obj;
	int i;
	u16 val;

	skb = alloc_can_skb(dev, &cf);
	if (!skb)
		return;

	/* get id */
	if (isrc & BIT(RECEIVE_EXT_CHL)) {
		/* extended frame format (EFF) */
		cf->can_id = ((readw(&reg->chl[RECEIVE_EXT_CHL].id1)
			     & 0x1FFF) << 16)
			     + readw(&reg->chl[RECEIVE_EXT_CHL].id0);
		cf->can_id |= CAN_EFF_FLAG;
		obj = RECEIVE_EXT_CHL;
	} else {
		/* standard frame format (SFF) */
		cf->can_id = (readw(&reg->chl[RECEIVE_STD_CHL].id1)
			     & 0x1ffc) >> 2;
		obj = RECEIVE_STD_CHL;
	}
	if (readw(&reg->chl[obj].id1) & RTR)
		cf->can_id |= CAN_RTR_FLAG;

	/* get data length code */
	cf->can_dlc = get_can_dlc(readw(&reg->chl[obj].dlc) & 0xF);

	/* get payload */
	for (i = 0; i < 8; i += 2) {
		val = readw(&reg->chl[obj].data[i]);
		cf->data[7 - i] = (7 - i) < cf->can_dlc ? val : 0;
		cf->data[6 - i] = (6 - i) < cf->can_dlc ? (val >> 8) : 0;
	}

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);
}

static int sc5xx_can_err(struct net_device *dev, u16 isrc, u16 status)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	enum can_state state = priv->can.state;

	skb = alloc_can_err_skb(dev, &cf);
	if (!skb)
		return -ENOMEM;

	if (isrc & RMLIS) {
		/* data overrun interrupt */
		netdev_dbg(dev, "data overrun interrupt\n");
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
		stats->rx_over_errors++;
		stats->rx_errors++;
	}

	if (isrc & BOIS) {
		netdev_dbg(dev, "bus-off mode interrupt\n");
		state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		priv->can.can_stats.bus_off++;
		can_bus_off(dev);
	}

	if (isrc & EPIS) {
		/* error passive interrupt */
		netdev_dbg(dev, "error passive interrupt\n");
		state = CAN_STATE_ERROR_PASSIVE;
	}

	if ((isrc & EWTIS) || (isrc & EWRIS)) {
		netdev_dbg(dev, "Error Warning Transmit/Receive Interrupt\n");
		state = CAN_STATE_ERROR_WARNING;
	}

	if (state != priv->can.state && (state == CAN_STATE_ERROR_WARNING ||
					 state == CAN_STATE_ERROR_PASSIVE)) {
		u16 cec = readw(&reg->cec);
		u8 rxerr = cec;
		u8 txerr = cec >> 8;

		cf->can_id |= CAN_ERR_CRTL;
		if (state == CAN_STATE_ERROR_WARNING) {
			priv->can.can_stats.error_warning++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		} else {
			priv->can.can_stats.error_passive++;
			cf->data[1] = (txerr > rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
	}

	if (status) {
		priv->can.can_stats.bus_error++;

		cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

		if (status & BEF)
			cf->data[2] |= CAN_ERR_PROT_BIT;
		else if (status & FER)
			cf->data[2] |= CAN_ERR_PROT_FORM;
		else if (status & SER)
			cf->data[2] |= CAN_ERR_PROT_STUFF;
	}

	priv->can.state = state;

	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;
	netif_rx(skb);

	return 0;
}

static irqreturn_t sc5xx_can_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	struct net_device_stats *stats = &dev->stats;
	u16 status, isrc;

	if (irq == priv->tx_irq && readw(&reg->mbtif2)) {
		/* transmission complete interrupt */
		writew(0xFFFF, &reg->mbtif2);
		stats->tx_packets++;
		stats->tx_bytes += readw(&reg->chl[TRANSMIT_CHL].dlc);
		can_get_echo_skb(dev, 0);
		netif_wake_queue(dev);
	} else if (irq == priv->rx_irq && readw(&reg->mbrif1)) {
		/* receive interrupt */
		isrc = readw(&reg->mbrif1);
		writew(0xFFFF, &reg->mbrif1);
		sc5xx_can_rx(dev, isrc);
	} else if (irq == priv->err_irq && readw(&reg->gis)) {
		/* error interrupt */
		isrc = readw(&reg->gis);
		status = readw(&reg->esr);
		writew(0x7FF, &reg->gis);
		sc5xx_can_err(dev, isrc, status);
	} else {
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int sc5xx_can_open(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	int err;

	/* set chip into reset mode */
	sc5xx_can_set_reset_mode(dev);

	/* common open */
	err = open_candev(dev);
	if (err)
		goto exit_open;

	/* register interrupt handler */
	err = request_irq(priv->rx_irq, &sc5xx_can_interrupt, 0,
			  "sc5xx-can-rx", dev);
	if (err)
		goto exit_rx_irq;
	err = request_irq(priv->tx_irq, &sc5xx_can_interrupt, 0,
			  "sc5xx-can-tx", dev);
	if (err)
		goto exit_tx_irq;
	err = request_irq(priv->err_irq, &sc5xx_can_interrupt, 0,
			  "sc5xx-can-err", dev);
	if (err)
		goto exit_err_irq;

	sc5xx_can_start(dev);

	netif_start_queue(dev);

	return 0;

exit_err_irq:
	free_irq(priv->tx_irq, dev);
exit_tx_irq:
	free_irq(priv->rx_irq, dev);
exit_rx_irq:
	close_candev(dev);
exit_open:
	return err;
}

static int sc5xx_can_close(struct net_device *dev)
{
	struct sc5xx_can_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	sc5xx_can_set_reset_mode(dev);

	close_candev(dev);

	free_irq(priv->rx_irq, dev);
	free_irq(priv->tx_irq, dev);
	free_irq(priv->err_irq, dev);

	return 0;
}

static struct net_device *alloc_sc5xx_candev(void)
{
	struct net_device *dev;
	struct sc5xx_can_priv *priv;

	dev = alloc_candev(sizeof(*priv), TX_ECHO_SKB_MAX);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &sc5xx_can_bittiming_const;
	priv->can.do_set_bittiming = sc5xx_can_set_bittiming;
	priv->can.do_set_mode = sc5xx_can_set_mode;
	priv->can.do_get_berr_counter = sc5xx_can_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;

	return dev;
}

static const struct net_device_ops sc5xx_can_netdev_ops = {
	.ndo_open               = sc5xx_can_open,
	.ndo_stop               = sc5xx_can_close,
	.ndo_start_xmit         = sc5xx_can_start_xmit,
	.ndo_change_mtu         = can_change_mtu,
};

#ifdef CONFIG_OF
static const struct of_device_id adi_can_of_match[] = {
	{
		.compatible = "adi,can",
	},
	{},
};

MODULE_DEVICE_TABLE(of, adi_can_of_match);
#endif

static int phy_set_normal_mode(struct spi_device *spi)
{
	struct spi_message msg;
	struct spi_transfer x;
	int err;

	u8 data[2];
	u8 rcv[2];

	data[0] = (0x1 << 1);
	data[1] = 0x7;

	spi_message_init(&msg);
	memset(&x, 0, sizeof(x));

	spi->mode |= SPI_MODE_1;
	spi->bits_per_word = 8;
	err = spi_setup(spi);
	x.tx_buf = data;
	x.rx_buf = rcv;
	x.len = 2;

	spi_message_add_tail(&x, &msg);
	return spi_sync(spi, &msg);
}

static int dummy_probe(struct spi_device *spi)
{
	return 0;
}

static int dummy_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver dummy_driver = {
	.driver.name = "dummy",
	.probe       = dummy_probe,
	.remove      = dummy_remove,
};

static int phy1145_startup(struct sc5xx_can_priv *priv, u16 spi_bus,
			   u16 spi_cs, u32 spi_clk)
{
	int ret;
	struct spi_master *master;
	struct spi_board_info board = {
		.modalias               = "dummy",
		.max_speed_hz           = spi_clk,
		.bus_num                = spi_bus,
		.chip_select            = spi_cs,
	};

	master = spi_busnum_to_master(0);
	if (!master)
		return -EPROBE_DEFER;
	priv->spi = spi_new_device(master, &board);
	if (!priv->spi)
		return -EINVAL;
	ret = spi_register_driver(&dummy_driver);
	if (ret < 0) {
		spi_unregister_device(priv->spi);
		return ret;
	}

	phy_set_normal_mode(priv->spi);

	return 0;
}

static int sc5xx_can_probe(struct platform_device *pdev)
{
	int err;
	struct device_node *np = pdev->dev.of_node;
	struct net_device *dev;
	struct sc5xx_can_priv *priv;
	struct resource *res_mem, *rx_irq, *tx_irq, *err_irq;
	unsigned short *pdata;
	struct clk *clk_source;

	if (!np)
		return -ENODEV;

	if (!of_match_device(of_match_ptr(adi_can_of_match), &pdev->dev)) {
		dev_err(&pdev->dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	pdata = (unsigned short *)pdev->dev.platform_data;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}

	rx_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	tx_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
	err_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
	if (!res_mem || !rx_irq || !tx_irq || !err_irq) {
		err = -EINVAL;
		goto exit;
	}

	/* request peripheral pins */
	err = peripheral_request_list(pdata, dev_name(&pdev->dev));
	if (err)
		goto exit_mem_release;

	dev = alloc_sc5xx_candev();
	if (!dev) {
		err = -ENOMEM;
		goto exit_peri_pin_free;
	}

	priv = netdev_priv(dev);

	priv->membase = devm_ioremap_resource(&pdev->dev, res_mem);
	if (IS_ERR((void *)priv->membase)) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		return PTR_ERR((void *)priv->membase);
	}

	priv->res_mem = res_mem;
	priv->rx_irq = rx_irq->start;
	priv->tx_irq = tx_irq->start;
	priv->err_irq = err_irq->start;
	priv->pin_list = pdata;

	clk_source = devm_clk_get(&pdev->dev, "can");
	if (IS_ERR(clk_source)) {
		dev_err(&pdev->dev, "can not get can clock\n");
		return PTR_ERR(clk_source);
	}
	priv->can.clock.freq = clk_get_rate(clk_source);

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->flags |= IFF_ECHO;	/* we support local echo */
	dev->netdev_ops = &sc5xx_can_netdev_ops;

	sc5xx_can_set_reset_mode(dev);

	err = register_candev(dev);
	if (err) {
		dev_err(&pdev->dev, "registering failed (err=%d)\n", err);
		goto exit_candev_free;
	}

	dev_info(&pdev->dev,
		 "%s device registered (&reg_base=%p, rx_irq=%d, tx_irq=%d, err_irq=%d, sclk=%d)\n",
		 DRV_NAME, priv->membase, priv->rx_irq,
		 priv->tx_irq, priv->err_irq, priv->can.clock.freq);

	if (likely(of_count_phandle_with_args(np, "enable-pin", NULL) > 0)) {
		if (softconfig_of_set_active_pin_output(&pdev->dev,
							np, "enable-pin", 0, &priv->enable_pin,
							&priv->enable_pin_active_low, true))
			goto exit_candev_free;
	}

	if (of_property_match_string(np, "phy-name", "tja1055") >= 0) {
		struct gpio_desc *en_gpio, *stb_gpio;

		en_gpio = devm_gpiod_get_index(&pdev->dev, "phy", 0, GPIOD_OUT_LOW);
		if (IS_ERR(en_gpio)) {
			err = PTR_ERR(en_gpio);
			if (err != -ENOENT)
				goto exit_candev_unreg;
		}
		gpiod_direction_output(en_gpio, 1);

		stb_gpio = devm_gpiod_get_index(&pdev->dev, "phy", 1, GPIOD_OUT_HIGH);
		if (IS_ERR(stb_gpio)) {
			err = PTR_ERR(stb_gpio);
			if (err != -ENOENT)
				goto exit_candev_unreg;
		}
		gpiod_direction_output(stb_gpio, 0);
	}

	if (of_property_match_string(np, "phy-name", "tja1145") >= 0) {
		u16 spi_bus, spi_cs;
		u32 spi_clk;

		of_property_read_u16(np, "phy-spibus", &spi_bus);
		of_property_read_u16(np, "phy-spics", &spi_cs);
		of_property_read_u32(np, "phy-spiclk", &spi_clk);

		err = phy1145_startup(priv, spi_bus, spi_cs, spi_clk);
		if (err < 0) {
			dev_err(&pdev->dev, "Can't start can phy\n");
			goto exit_candev_unreg;
		}
	}

	return 0;

exit_candev_unreg:
	unregister_candev(dev);
exit_candev_free:
	free_candev(dev);
exit_peri_pin_free:
	peripheral_free_list(pdata);
exit_mem_release:
	release_mem_region(priv->res_mem->start, resource_size(priv->res_mem));
exit:
	return err;
}

static int sc5xx_can_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct resource *res = priv->res_mem;

	sc5xx_can_set_reset_mode(dev);
	unregister_candev(dev);
	release_mem_region(res->start, resource_size(res));
	peripheral_free_list(priv->pin_list);
	free_candev(dev);
	if (priv->enable_pin && gpio_is_valid(priv->enable_pin))
		gpio_direction_output(priv->enable_pin,
				      priv->enable_pin_active_low ? 1 : 0);

	return 0;
}

static int __maybe_unused sc5xx_can_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;
	int timeout = ADI_CAN_TIMEOUT;

	if (netif_running(dev)) {
		/* enter sleep mode */
		writew(readw(&reg->control) | SMR, &reg->control);
		while (!(readw(&reg->intr) & SMACK)) {
			udelay_range(10, 100);
			if (--timeout == 0) {
				netdev_err(dev, "fail to enter sleep mode\n");
				WARN(1, "SC5XX CAN timeout took too long\n");
				return -1;
			}
		}
	}

	return 0;
}

static int __maybe_unused sc5xx_can_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct sc5xx_can_priv *priv = netdev_priv(dev);
	struct sc5xx_can_regs __iomem *reg = priv->membase;

	if (netif_running(dev)) {
		/* leave sleep mode */
		writew(0, &reg->intr);
	}

	return 0;
}

static struct platform_driver sc5xx_can_driver = {
	.probe = sc5xx_can_probe,
	.remove = sc5xx_can_remove,
	.suspend = sc5xx_can_suspend,
	.resume = sc5xx_can_resume,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(adi_can_of_match),
	},
};

module_platform_driver(sc5xx_can_driver);

MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ADI SC5xx on-chip CAN netdevice driver");
MODULE_ALIAS("platform:" DRV_NAME);
