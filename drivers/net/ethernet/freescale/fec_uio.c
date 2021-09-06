// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 NXP
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/uio_driver.h>
#include <linux/list.h>

#include <linux/string.h>
#include <linux/pm_runtime.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <net/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/icmp.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/fec.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/regulator/consumer.h>
#include <linux/if_vlan.h>
#include <linux/pinctrl/consumer.h>
#include <linux/busfreq-imx.h>
#include <linux/prefetch.h>
#include <soc/imx/cpuidle.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "fec.h"

struct fec_dev *fec_dev;
static const char fec_uio_version[] = "FEC UIO driver v1.0";
dma_addr_t bd_dma;
int bd_size;
struct bufdesc *cbd_base;

#define EXTEND_BUF		0

#define NAME_LENGTH		30
#define DRIVER_NAME		"fec-uio"

#define FEC_ATIME_CTRL          0x400
#define FEC_ATIME               0x404
#define FEC_ATIME_EVT_OFFSET    0x408
#define FEC_ATIME_EVT_PERIOD    0x40c
#define FEC_ATIME_CORR          0x410
#define FEC_ATIME_INC           0x414
#define FEC_TS_TIMESTAMP        0x418

#define FEC_TGSR                0x604
#define FEC_TCSR(n)             (0x608 + (n) * 0x08)
#define FEC_TCCR(n)             (0x60C + (n) * 0x08)
#define MAX_TIMER_CHANNEL       3
#define FEC_TMODE_TOGGLE        0x05
#define FEC_HIGH_PULSE          0x0F

#define FEC_CC_MULT		BIT(31)
#define FEC_COUNTER_PERIOD	BIT(31)
#define PPS_OUTPUT_RELOAD_PERIOD NSEC_PER_SEC
#define FEC_CHANNEL_0		0
#define DEFAULT_PPS_CHANNEL	FEC_CHANNEL_0

/* FEC 1588 register bits */
#define FEC_T_CTRL_CAPTURE              0x00000800
#define FEC_T_CTRL_PERIOD_RST           0x00000030
#define FEC_T_CTRL_ENABLE               0x00000001

#define FEC_T_INC_MASK                  0x0000007f
#define FEC_T_INC_OFFSET                0
#define FEC_T_INC_CORR_OFFSET           8

#define FEC_T_CTRL_PINPER               0x00000080
#define FEC_T_TDRE_OFFSET               0
#define FEC_T_TMODE_MASK                0x0000003C
#define FEC_T_TMODE_OFFSET              2
#define FEC_T_TIE_OFFSET                6
#define FEC_T_TF_MASK                   0x00000080
#define FEC_T_TF_OFFSET                 7

/* By default, set the copybreak to 1518,
 * then the RX path always keep DMA memory unchanged, and
 * allocate one new skb and copy DMA memory data to the new skb
 * buffer, which can improve the performance when SMMU is enabled.
 *
 * The driver support .set_tunable() interface for ethtool, user
 * can dynamicly change the copybreak value.
 */
#define COPYBREAK_DEFAULT	1518

/* Pause frame feild and FIFO threshold */
#define FEC_ENET_FCE	BIT(5)
#define FEC_ENET_RSEM_V 0x84
#define FEC_ENET_RSFL_V 16
#define FEC_ENET_RAEM_V 0x8
#define FEC_ENET_RAFL_V 0x8
#define FEC_ENET_OPD_V  0xFFF0
#define FEC_MDIO_PM_TIMEOUT  100 /* ms */

/* FEC MII MMFR bits definition */
#define FEC_MMFR_ST             BIT(30)
#define FEC_MMFR_ST_C45         (0)
#define FEC_MMFR_OP_READ        (2 << 28)
#define FEC_MMFR_OP_READ_C45    (3 << 28)
#define FEC_MMFR_OP_WRITE       BIT(28)
#define FEC_MMFR_OP_ADDR_WRITE  (0)
#define FEC_MMFR_PA(v)          (((v) & 0x1f) << 23)
#define FEC_MMFR_RA(v)          (((v) & 0x1f) << 18)
#define FEC_MMFR_TA             (2 << 16)
#define FEC_MMFR_DATA(v)        ((v) & 0xffff)

/* FEC ECR bits definition */
#define FEC_ECR_MAGICEN         BIT(2)
#define FEC_ECR_SLEEP           BIT(3)

#define FEC_MII_TIMEOUT         30000 /* us */

#define FEC_PAUSE_FLAG_AUTONEG  0x1
#define FEC_PAUSE_FLAG_ENABLE   0x2
#define FEC_WOL_HAS_MAGIC_PACKET        (0x1 << 0)
#define FEC_WOL_FLAG_ENABLE             (0x1 << 1)
#define FEC_WOL_FLAG_SLEEP_ON           (0x1 << 2)

/* Pause frame feild and FIFO threshold */
#define FEC_MDIO_PM_TIMEOUT  100 /* ms */

/* The 5270/5271/5280/5282/532x RX control register also contains maximum frame
 * size bits. Other FEC hardware does not, so we need to take that into
 * account when setting it.
 */
#if defined(CONFIG_M523x) || defined(CONFIG_M527x) || defined(CONFIG_M528x) || \
	defined(CONFIG_M520x) || defined(CONFIG_M532x) || \
	defined(CONFIG_ARM) || defined(CONFIG_ARM64)
#define OPT_FRAME_SIZE  (PKT_MAXBUF_SIZE << 16)
#else
#define OPT_FRAME_SIZE  0
#endif

static const char uio_device_name[] = "imx-fec-uio";
static int mii_cnt;
struct fec_uio_info {
	atomic_t ref; /* exclusive, only one open() at a time */
	struct uio_info uio_info;
	char name[NAME_LENGTH];
};

struct fec_dev {
	u32 index;
	struct device *dev;
	struct resource *res;
	struct fec_uio_info info;
};

struct fec_uio_devinfo {
	u32 quirks;
};

static const struct fec_uio_devinfo fec_imx8_info = {
	.quirks = FEC_QUIRK_ENET_MAC | FEC_QUIRK_HAS_GBIT |
		  FEC_QUIRK_HAS_BUFDESC_EX | FEC_QUIRK_HAS_CSUM |
		  FEC_QUIRK_HAS_VLAN | FEC_QUIRK_ERR007885 |
		  FEC_QUIRK_BUG_CAPTURE | FEC_QUIRK_HAS_RACC |
		  FEC_QUIRK_HAS_COALESCE | FEC_QUIRK_CLEAR_SETUP_MII,
};

static const struct fec_uio_devinfo fec_imx8mm_info = {
	.quirks = FEC_QUIRK_ENET_MAC | FEC_QUIRK_HAS_GBIT |
		  FEC_QUIRK_HAS_BUFDESC_EX | FEC_QUIRK_HAS_CSUM |
		  FEC_QUIRK_HAS_VLAN | FEC_QUIRK_HAS_AVB |
		  FEC_QUIRK_ERR007885 | FEC_QUIRK_BUG_CAPTURE |
		  FEC_QUIRK_HAS_RACC | FEC_QUIRK_HAS_COALESCE |
		  FEC_QUIRK_CLEAR_SETUP_MII | FEC_QUIRK_HAS_EEE,
};

static struct platform_device_id fec_enet_uio_devtype[] = {
	{
		/* keep it for coldfire */
		.name = DRIVER_NAME,
		.driver_data = 0,
	}, {
		.name = "imx8-fec",
		.driver_data = (kernel_ulong_t)&fec_imx8_info,
	}, {
		.name = "imx8mm-fec",
		.driver_data = (kernel_ulong_t)&fec_imx8mm_info,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(platform, fec_enet_uio_devtype);

enum imx_fec_uio_type {
	IMX8_FEC,
	IMX8MM_FEC,
};

static const struct of_device_id fec_enet_uio_ids[] = {
	{ .compatible = "fsl,imx8-fec-uio", .data = &fec_enet_uio_devtype[IMX8_FEC], },
	{ .compatible = "fsl,imx8mm-fec-uio", .data = &fec_enet_uio_devtype[IMX8MM_FEC], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fec_enet_uio_ids);

static unsigned char macaddr[ETH_ALEN];
module_param_array(macaddr, byte, NULL, 0);
MODULE_PARM_DESC(macaddr, "FEC Ethernet MAC address");

/* The FEC stores dest/src/type/vlan, data, and checksum for receive packets.
 *
 * 2048 byte skbufs are allocated. However, alignment requirements
 * varies between FEC variants. Worst case is 64, so round down by 64.
 */
#define PKT_MAXBUF_SIZE         (round_down(2048 - 64, 64))
#define PKT_MINBUF_SIZE         64

/* FEC receive acceleration */
#define FEC_RACC_IPDIS          BIT(1)
#define FEC_RACC_PRODIS         BIT(2)
#define FEC_RACC_SHIFT16        BIT(7)
#define FEC_RACC_OPTIONS        (FEC_RACC_IPDIS | FEC_RACC_PRODIS)

static const struct fec_uio_stat {
	char name[ETH_GSTRING_LEN];
	u16 offset;
} fec_uio_stats[] = {
	/* RMON TX */
	{ "tx_dropped", RMON_T_DROP },
	{ "tx_packets", RMON_T_PACKETS },
	{ "tx_broadcast", RMON_T_BC_PKT },
	{ "tx_multicast", RMON_T_MC_PKT },
	{ "tx_crc_errors", RMON_T_CRC_ALIGN },
	{ "tx_undersize", RMON_T_UNDERSIZE },
	{ "tx_oversize", RMON_T_OVERSIZE },
	{ "tx_fragment", RMON_T_FRAG },
	{ "tx_jabber", RMON_T_JAB },
	{ "tx_collision", RMON_T_COL },
	{ "tx_64byte", RMON_T_P64 },
	{ "tx_65to127byte", RMON_T_P65TO127 },
	{ "tx_128to255byte", RMON_T_P128TO255 },
	{ "tx_256to511byte", RMON_T_P256TO511 },
	{ "tx_512to1023byte", RMON_T_P512TO1023 },
	{ "tx_1024to2047byte", RMON_T_P1024TO2047 },
	{ "tx_GTE2048byte", RMON_T_P_GTE2048 },
	{ "tx_octets", RMON_T_OCTETS },

	/* IEEE TX */
	{ "IEEE_tx_drop", IEEE_T_DROP },
	{ "IEEE_tx_frame_ok", IEEE_T_FRAME_OK },
	{ "IEEE_tx_1col", IEEE_T_1COL },
	{ "IEEE_tx_mcol", IEEE_T_MCOL },
	{ "IEEE_tx_def", IEEE_T_DEF },
	{ "IEEE_tx_lcol", IEEE_T_LCOL },
	{ "IEEE_tx_excol", IEEE_T_EXCOL },
	{ "IEEE_tx_macerr", IEEE_T_MACERR },
	{ "IEEE_tx_cserr", IEEE_T_CSERR },
	{ "IEEE_tx_sqe", IEEE_T_SQE },
	{ "IEEE_tx_fdxfc", IEEE_T_FDXFC },
	{ "IEEE_tx_octets_ok", IEEE_T_OCTETS_OK },

	/* RMON RX */
	{ "rx_packets", RMON_R_PACKETS },
	{ "rx_broadcast", RMON_R_BC_PKT },
	{ "rx_multicast", RMON_R_MC_PKT },
	{ "rx_crc_errors", RMON_R_CRC_ALIGN },
	{ "rx_undersize", RMON_R_UNDERSIZE },
	{ "rx_oversize", RMON_R_OVERSIZE },
	{ "rx_fragment", RMON_R_FRAG },
	{ "rx_jabber", RMON_R_JAB },
	{ "rx_64byte", RMON_R_P64 },
	{ "rx_65to127byte", RMON_R_P65TO127 },
	{ "rx_128to255byte", RMON_R_P128TO255 },
	{ "rx_256to511byte", RMON_R_P256TO511 },
	{ "rx_512to1023byte", RMON_R_P512TO1023 },
	{ "rx_1024to2047byte", RMON_R_P1024TO2047 },
	{ "rx_GTE2048byte", RMON_R_P_GTE2048 },
	{ "rx_octets", RMON_R_OCTETS },

	/* IEEE RX */
	{ "IEEE_rx_drop", IEEE_R_DROP },
	{ "IEEE_rx_frame_ok", IEEE_R_FRAME_OK },
	{ "IEEE_rx_crc", IEEE_R_CRC },
	{ "IEEE_rx_align", IEEE_R_ALIGN },
	{ "IEEE_rx_macerr", IEEE_R_MACERR },
	{ "IEEE_rx_fdxfc", IEEE_R_FDXFC },
	{ "IEEE_rx_octets_ok", IEEE_R_OCTETS_OK },
};

#define FEC_STATS_SIZE          (ARRAY_SIZE(fec_uio_stats) * sizeof(u64))

#ifdef CONFIG_OF
static int fec_enet_uio_reset_phy(struct platform_device *pdev)
{
	int err, phy_reset;
	bool active_high = false;
	int msec = 1, phy_post_delay = 0;
	struct device_node *np = pdev->dev.of_node;

	if (!np)
		return 0;

	err = of_property_read_u32(np, "phy-reset-duration", &msec);
	/* A sane reset duration should not be longer than 1s */
	if (!err && msec > 1000)
		msec = 1;

	phy_reset = of_get_named_gpio(np, "phy-reset-gpios", 0);
	if (phy_reset == -EPROBE_DEFER)
		return phy_reset;
	else if (!gpio_is_valid(phy_reset))
		return 0;

	err = of_property_read_u32(np, "phy-reset-post-delay", &phy_post_delay);
	/* valid reset duration should be less than 1s */
	if (!err && phy_post_delay > 1000)
		return -EINVAL;

	active_high = of_property_read_bool(np, "phy-reset-active-high");

	err = devm_gpio_request_one(&pdev->dev, phy_reset,
				    active_high ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
				    "phy-reset");
	if (err) {
		dev_err(&pdev->dev, "failed to get phy-reset-gpios: %d\n", err);
		return err;
	}

	if (msec > 20)
		msleep(msec);
	else
		usleep_range(msec * 1000, msec * 1000 + 1000);

	gpio_set_value_cansleep(phy_reset, !active_high);

	if (!phy_post_delay)
		return 0;
	if (phy_post_delay > 20)
		msleep(phy_post_delay);
	else
		usleep_range(phy_post_delay * 1000,
			     phy_post_delay * 1000 + 1000);

	return 0;
}

#else /* CONFIG_OF */
static int fec_enet_uio_reset_phy(struct platform_device *pdev)
{
	/* In case of platform probe, the reset has been done
	 * by machine code.
	 */
	return 0;
}
#endif /* CONFIG_OF */

static int fec_enet_uio_get_irq_cnt(struct platform_device *pdev)
{
	int irq_cnt = platform_irq_count(pdev);

	if (irq_cnt > FEC_IRQ_NUM)
		irq_cnt = FEC_IRQ_NUM;  /* last for pps */
	else if (irq_cnt == 2)
		irq_cnt = 1;    /* last for pps */
	else if (irq_cnt <= 0)
		irq_cnt = 1;    /* At least 1 irq is needed */
	return irq_cnt;
}

static void fec_enet_uio_phy_reset_after_clk_enable(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct phy_device *phy_dev = ndev->phydev;

	if (phy_dev) {
		phy_reset_after_clk_enable(phy_dev);
	} else if (fep->phy_node) {
		/* If the PHY still is not bound to the MAC, but there is
		 * OF PHY node and a matching PHY device instance already,
		 * use the OF PHY node to obtain the PHY device instance,
		 * and then use that PHY device instance when triggering
		 * the PHY reset.
		 */
		phy_dev = of_phy_find_device(fep->phy_node);
		phy_reset_after_clk_enable(phy_dev);
		put_device(&phy_dev->mdio.dev);
	}
}

static int fec_enet_uio_clk_enable(struct net_device *ndev, bool enable)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	int ret;

	if (enable) {
		ret = clk_prepare_enable(fep->clk_enet_out);
		if (ret)
			return ret;

		if (fep->clk_ptp) {
			mutex_lock(&fep->ptp_clk_mutex);
			ret = clk_prepare_enable(fep->clk_ptp);
			if (ret) {
				mutex_unlock(&fep->ptp_clk_mutex);
				goto failed_clk_ptp;
			} else {
				fep->ptp_clk_on = true;
			}
			mutex_unlock(&fep->ptp_clk_mutex);
		}

		ret = clk_prepare_enable(fep->clk_ref);
		if (ret)
			goto failed_clk_ref;

		fec_enet_uio_phy_reset_after_clk_enable(ndev);
	} else {
		clk_disable_unprepare(fep->clk_enet_out);
		if (fep->clk_ptp) {
			mutex_lock(&fep->ptp_clk_mutex);
			clk_disable_unprepare(fep->clk_ptp);
			fep->ptp_clk_on = false;
			mutex_unlock(&fep->ptp_clk_mutex);
		}
		clk_disable_unprepare(fep->clk_ref);
	}

	return 0;

failed_clk_ref:
	if (fep->clk_ptp) {
		mutex_lock(&fep->ptp_clk_mutex);
		clk_disable_unprepare(fep->clk_ptp);
		fep->ptp_clk_on = false;
		mutex_unlock(&fep->ptp_clk_mutex);
	}
failed_clk_ptp:
	clk_disable_unprepare(fep->clk_enet_out);

	return ret;
}

static bool fec_enet_uio_collect_events(struct fec_enet_private *fep)
{
	uint int_events;

	int_events = readl(fep->hwp + FEC_IEVENT);
	/* Don't clear MDIO events, we poll for those */
	int_events &= ~FEC_ENET_MII;

	writel(int_events, fep->hwp + FEC_IEVENT);

	return int_events != 0;
}

static irqreturn_t
fec_enet_uio_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct fec_enet_private *fep = netdev_priv(ndev);
	irqreturn_t ret = IRQ_NONE;

	if (fec_enet_uio_collect_events(fep) && fep->link) {
		ret = IRQ_HANDLED;

		if (napi_schedule_prep(&fep->napi)) {
			/* Disable interrupts */
			writel(0, fep->hwp + FEC_IMASK);
			__napi_schedule(&fep->napi);
		}
	}

	return ret;
}

static int fec_enet_uio_mdio_wait(struct fec_enet_private *fep)
{
	uint ievent;
	int ret;

	ret = readl_poll_timeout_atomic(fep->hwp + FEC_IEVENT, ievent,
					ievent & FEC_ENET_MII, 2, 30000);

	if (!ret)
		writel(FEC_ENET_MII, fep->hwp + FEC_IEVENT);
	return ret;
}

static int fec_enet_uio_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret = 0, frame_start, frame_addr, frame_op;
	bool is_c45 = !!(regnum & MII_ADDR_C45);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	if (is_c45) {
		frame_start = FEC_MMFR_ST_C45;

		/* write address */
		frame_addr = (regnum >> 16);
		writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
			FEC_MMFR_TA | (regnum & 0xFFFF),
			fep->hwp + FEC_MII_DATA);

		/* wait for end of transfer */
		ret = fec_enet_uio_mdio_wait(fep);
		if (ret) {
			netdev_err(fep->netdev, "MDIO address write timeout\n");
			goto out;
		}
		frame_op = FEC_MMFR_OP_READ_C45;
	} else {
		/* C22 read */
		frame_op = FEC_MMFR_OP_READ;
		frame_start = FEC_MMFR_ST;
		frame_addr = regnum;
	}
	/* start a read op */
	writel(frame_start | frame_op |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
		FEC_MMFR_TA, fep->hwp + FEC_MII_DATA);
	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret) {
		netdev_err(fep->netdev, "MDIO read timeout\n");
		goto out;
	}
	ret = FEC_MMFR_DATA(readl(fep->hwp + FEC_MII_DATA));

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				   u16 value)
{
	struct fec_enet_private *fep = bus->priv;
	struct device *dev = &fep->pdev->dev;
	int ret, frame_start, frame_addr;
	bool is_c45 = !!(regnum & MII_ADDR_C45);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	if (is_c45) {
		frame_start = FEC_MMFR_ST_C45;

		/* write address */
		frame_addr = (regnum >> 16);
		writel(frame_start | FEC_MMFR_OP_ADDR_WRITE |
			FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
			FEC_MMFR_TA | (regnum & 0xFFFF),
			fep->hwp + FEC_MII_DATA);

		/* wait for end of transfer */
		ret = fec_enet_uio_mdio_wait(fep);
		if (ret) {
			netdev_err(fep->netdev, "MDIO address write timeout\n");
			goto out;
		}
	} else {
		/* C22 write */
		frame_start = FEC_MMFR_ST;
		frame_addr = regnum;
	}

	/* start a write op */
	writel(frame_start | FEC_MMFR_OP_WRITE |
		FEC_MMFR_PA(mii_id) | FEC_MMFR_RA(frame_addr) |
		FEC_MMFR_TA | FEC_MMFR_DATA(value),
		fep->hwp + FEC_MII_DATA);

	/* wait for end of transfer */
	ret = fec_enet_uio_mdio_wait(fep);
	if (ret)
		netdev_err(fep->netdev, "MDIO write timeout\n");

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int fec_enet_uio_mii_init(struct platform_device *pdev)
{
	static struct mii_bus *fec0_mii_bus;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	bool suppress_preamble = false;
	struct device_node *node;
	int err = -ENXIO;
	u32 mii_speed, holdtime;
	u32 bus_freq;

	/* The i.MX28 dual fec interfaces are not equal.
	 * Here are the differences:
	 *
	 *  - fec0 supports MII & RMII modes while fec1 only supports RMII
	 *  - fec0 acts as the 1588 time master while fec1 is slave
	 *  - external phys can only be configured by fec0
	 *
	 * That is to say fec1 can not work independently. It only works
	 * when fec0 is working. The reason behind this design is that the
	 * second interface is added primarily for Switch mode.
	 *
	 * Because of the last point above, both phys are attached on fec0
	 * mdio interface in board design, and need to be configured by
	 * fec0 mii_bus.
	 */
	if ((fep->quirks & FEC_QUIRK_SINGLE_MDIO) && fep->dev_id > 0) {
		/* fec1 uses fec0 mii_bus */
		if (mii_cnt && fec0_mii_bus) {
			fep->mii_bus = fec0_mii_bus;
			mii_cnt++;
			return 0;
		}
		return -ENOENT;
	}

	bus_freq = 2500000; /* 2.5MHz by default */
	node = of_get_child_by_name(pdev->dev.of_node, "mdio");
	if (node) {
		of_property_read_u32(node, "clock-frequency", &bus_freq);
		suppress_preamble = of_property_read_bool(node,
							  "suppress-preamble");
	}

	/* Set MII speed to 2.5 MHz (= clk_get_rate() / 2 * phy_speed)
	 *
	 * The formula for FEC MDC is 'ref_freq / (MII_SPEED x 2)' while
	 * for ENET-MAC is 'ref_freq / ((MII_SPEED + 1) x 2)'.  The i.MX28
	 * Reference Manual has an error on this, and gets fixed on i.MX6Q
	 * document.
	 */
	mii_speed = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 5000000);
	if (fep->quirks & FEC_QUIRK_ENET_MAC)
		mii_speed--;
	if (mii_speed > 63) {
		dev_err(&pdev->dev,
			"fec clock (%lu) too fast to get right mii speed\n",
			clk_get_rate(fep->clk_ipg));
		err = -EINVAL;
		goto err_out;
	}

	/* The i.MX28 and i.MX6 types have another filed in the MSCR (aka
	 * MII_SPEED) register that defines the MDIO output hold time. Earlier
	 * versions are RAZ there, so just ignore the difference and write the
	 * register always.
	 * The minimal hold time according to IEE802.3 (clause 22) is 10 ns.
	 * HOLDTIME + 1 is the number of clk cycles the fec is holding the
	 * output.
	 * The HOLDTIME bitfield takes values between 0 and 7 (inclusive).
	 * Given that ceil(clkrate / 5000000) <= 64, the calculation for
	 * holdtime cannot result in a value greater than 3.
	 */
	holdtime = DIV_ROUND_UP(clk_get_rate(fep->clk_ipg), 100000000) - 1;

	fep->phy_speed = mii_speed << 1 | holdtime << 8;

	if (suppress_preamble)
		fep->phy_speed |= BIT(7);

	if (fep->quirks & FEC_QUIRK_CLEAR_SETUP_MII) {
		/* Clear MMFR to avoid to generate MII event by writing MSCR.
		 * MII event generation condition:
		 * - writing MSCR:
		 *      - mmfr[31:0]_not_zero & mscr[7:0]_is_zero &
		 *        mscr_reg_data_in[7:0] != 0
		 * - writing MMFR:
		 *      - mscr[7:0]_not_zero
		 */
		writel(0, fep->hwp + FEC_MII_DATA);
	}

	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	/* Clear any pending transaction complete indication */
	writel(FEC_ENET_MII, fep->hwp + FEC_IEVENT);

	fep->mii_bus = mdiobus_alloc();
	if (!fep->mii_bus) {
		err = -ENOMEM;
		goto err_out;
	}

	fep->mii_bus->name = "fec_enet_mii_bus";
	fep->mii_bus->read = fec_enet_uio_mdio_read;
	fep->mii_bus->write = fec_enet_uio_mdio_write;
	snprintf(fep->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 pdev->name, fep->dev_id + 1);
	fep->mii_bus->priv = fep;
	fep->mii_bus->parent = &pdev->dev;

	err = of_mdiobus_register(fep->mii_bus, node);
	of_node_put(node);
	if (err)
		goto err_out_free_mdiobus;

	mii_cnt++;

	/* save fec0 mii_bus */
	if (fep->quirks & FEC_QUIRK_SINGLE_MDIO)
		fec0_mii_bus = fep->mii_bus;

	return 0;

err_out_free_mdiobus:
	mdiobus_free(fep->mii_bus);
err_out:
	return err;
}

/* This function is called to start or restart the FEC during a link
 * change, transmit timeout, or to reconfigure the FEC.  The network
 * packet processing for this device must be stopped before this call.
 */
static void
fec_enet_uio_restart(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	u32 val;
	u32 rcntl = OPT_FRAME_SIZE | 0x04;
	u32 ecntl = 0x2; /* ETHEREN */

	/* Whack a reset.  We should wait for this.
	 * For i.MX6SX SOC, enet use AXI bus, we use disable MAC
	 * instead of reset MAC itself.
	 */
	if (fep->quirks & FEC_QUIRK_HAS_AVB) {
		writel(0, fep->hwp + FEC_ECNTRL);
	} else {
		writel(1, fep->hwp + FEC_ECNTRL);
		udelay(10);
	}

	/* Clear any outstanding interrupt, except MDIO. */
	writel((0xffffffff & ~FEC_ENET_MII), fep->hwp + FEC_IEVENT);

	/* Enable MII mode */
	if (fep->full_duplex == DUPLEX_FULL) {
		/* FD enable */
		writel(0x04, fep->hwp + FEC_X_CNTRL);
	} else {
		/* No Rcv on Xmit */
		rcntl |= 0x02;
		writel(0x0, fep->hwp + FEC_X_CNTRL);
	}

	/* Set MII speed */
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

#if !defined(CONFIG_M5272)
	if (fep->quirks & FEC_QUIRK_HAS_RACC) {
		val = readl(fep->hwp + FEC_RACC);
		/* align IP header */
		val |= FEC_RACC_SHIFT16;
		if (fep->csum_flags & FLAG_RX_CSUM_ENABLED)
			/* set RX checksum */
			val |= FEC_RACC_OPTIONS;
		else
			val &= ~FEC_RACC_OPTIONS;
		writel(val, fep->hwp + FEC_RACC);
		writel(PKT_MAXBUF_SIZE, fep->hwp + FEC_FTRL);
	}
#endif

	/* The phy interface and speed need to get configured
	 * differently on enet-mac.
	 */
	if (fep->quirks & FEC_QUIRK_ENET_MAC) {
		/* Enable flow control and length check */
		rcntl |= 0x40000000 | 0x00000020;

		/* RGMII, RMII or MII */
		if (fep->phy_interface == PHY_INTERFACE_MODE_RGMII ||
		    fep->phy_interface == PHY_INTERFACE_MODE_RGMII_ID ||
		    fep->phy_interface == PHY_INTERFACE_MODE_RGMII_RXID ||
		    fep->phy_interface == PHY_INTERFACE_MODE_RGMII_TXID)
			rcntl |= (1 << 6);
		else if (fep->phy_interface == PHY_INTERFACE_MODE_RMII)
			rcntl |= (1 << 8);
		else
			rcntl &= ~(1 << 8);

		/* 1G, 100M or 10M */
		if (ndev->phydev) {
			if (ndev->phydev->speed == SPEED_1000)
				ecntl |= (1 << 5);
			else if (ndev->phydev->speed == SPEED_100)
				rcntl &= ~(1 << 9);
			else
				rcntl |= (1 << 9);
		}
	} else {
#ifdef FEC_MIIGSK_ENR
		if (fep->quirks & FEC_QUIRK_USE_GASKET) {
			u32 cfgr;
			/* disable the gasket and wait */
			writel(0, fep->hwp + FEC_MIIGSK_ENR);
			while (readl(fep->hwp + FEC_MIIGSK_ENR) & 4)
				udelay(1);

			/* configure the gasket:
			 * RMII, 50 MHz, no loopback, no echo
			 * MII, 25 MHz, no loopback, no echo
			 */
			cfgr = (fep->phy_interface == PHY_INTERFACE_MODE_RMII)
				? BM_MIIGSK_CFGR_RMII : BM_MIIGSK_CFGR_MII;
			if (ndev->phydev && ndev->phydev->speed == SPEED_10)
				cfgr |= BM_MIIGSK_CFGR_FRCONT_10M;
			writel(cfgr, fep->hwp + FEC_MIIGSK_CFGR);

			/* re-enable the gasket */
			writel(2, fep->hwp + FEC_MIIGSK_ENR);
		}
#endif
	}
#if !defined(CONFIG_M5272)
	/* enable pause frame*/
	if ((fep->pause_flag & FEC_PAUSE_FLAG_ENABLE) ||
	    ((fep->pause_flag & FEC_PAUSE_FLAG_AUTONEG) &&
	     ndev->phydev && ndev->phydev->pause)) {
		rcntl |= FEC_ENET_FCE;

		/* set FIFO threshold parameter to reduce overrun */
		writel(FEC_ENET_RSEM_V, fep->hwp + FEC_R_FIFO_RSEM);
		writel(FEC_ENET_RSFL_V, fep->hwp + FEC_R_FIFO_RSFL);
		writel(FEC_ENET_RAEM_V, fep->hwp + FEC_R_FIFO_RAEM);
		writel(FEC_ENET_RAFL_V, fep->hwp + FEC_R_FIFO_RAFL);

		/* OPD */
		writel(FEC_ENET_OPD_V, fep->hwp + FEC_OPD);
	} else {
		rcntl &= ~FEC_ENET_FCE;
	}
#endif /* !defined(CONFIG_M5272) */

	writel(rcntl, fep->hwp + FEC_R_CNTRL);
#ifndef CONFIG_M5272
	writel(0, fep->hwp + FEC_HASH_TABLE_HIGH);
	writel(0, fep->hwp + FEC_HASH_TABLE_LOW);
#endif

	if (fep->quirks & FEC_QUIRK_ENET_MAC) {
		/* enable ENET endian swap */
		ecntl |= (1 << 8);
		/* enable ENET store and forward mode */
		writel(1 << 8, fep->hwp + FEC_X_WMRK);
	}

	if (fep->bufdesc_ex)
		ecntl |= (1 << 4);

#ifndef CONFIG_M5272
	/* Enable the MIB statistic event counters */
	writel(0 << 31, fep->hwp + FEC_MIB_CTRLSTAT);
#endif

	/* And last, enable the transmit and receive processing */
	writel(ecntl, fep->hwp + FEC_ECNTRL);
}

static void fec_enet_uio_stop_mode(struct fec_enet_private *fep, bool enabled)
{
	struct fec_platform_data *pdata = fep->pdev->dev.platform_data;
	struct fec_stop_mode_gpr *stop_gpr = &fep->stop_gpr;

	if (stop_gpr->gpr) {
		if (enabled)
			regmap_update_bits(stop_gpr->gpr, stop_gpr->reg,
					   BIT(stop_gpr->bit),
					   BIT(stop_gpr->bit));
		else
			regmap_update_bits(stop_gpr->gpr, stop_gpr->reg,
					   BIT(stop_gpr->bit), 0);
	} else if (pdata && pdata->sleep_mode_enable) {
		pdata->sleep_mode_enable(enabled);
	}
}

static inline void fec_enet_uio_irqs_disable(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);

	writel(0, fep->hwp + FEC_IMASK);
}

static void
fec_uio_stop(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	u32 rmii_mode = readl(fep->hwp + FEC_R_CNTRL) & (1 << 8);
	u32 val;

	/* We cannot expect a graceful transmit stop without link !!! */
	if (fep->link) {
		writel(1, fep->hwp + FEC_X_CNTRL); /* Graceful transmit stop */
		udelay(10);
		if (!(readl(fep->hwp + FEC_IEVENT) & FEC_ENET_GRA))
			netdev_err(ndev, "Graceful transmit stop did not complete!\n");
	}

	/* Whack a reset.  We should wait for this.
	 * For i.MX6SX SOC, enet use AXI bus, we use disable MAC
	 * instead of reset MAC itself.
	 */
	if (!(fep->wol_flag & FEC_WOL_FLAG_SLEEP_ON)) {
		if (fep->quirks & FEC_QUIRK_HAS_AVB) {
			writel(0, fep->hwp + FEC_ECNTRL);
		} else {
			writel(1, fep->hwp + FEC_ECNTRL);
			udelay(10);
		}
		writel(FEC_DEFAULT_IMASK, fep->hwp + FEC_IMASK);
	} else {
		writel(FEC_DEFAULT_IMASK | FEC_ENET_WAKEUP,
		       fep->hwp + FEC_IMASK);
		val = readl(fep->hwp + FEC_ECNTRL);
		val |= (FEC_ECR_MAGICEN | FEC_ECR_SLEEP);
		writel(val, fep->hwp + FEC_ECNTRL);
	}
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	/* We have to keep ENET enabled to have MII interrupt stay working */
	if (fep->quirks & FEC_QUIRK_ENET_MAC &&
	    !(fep->wol_flag & FEC_WOL_FLAG_SLEEP_ON)) {
		writel(2, fep->hwp + FEC_ECNTRL);
		writel(rmii_mode, fep->hwp + FEC_R_CNTRL);
	}
}

/* Phy section */
static void fec_enet_adjust_link(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct phy_device *phy_dev = ndev->phydev;
	int status_change = 0;

	/* If the netdev is down, or is going down, we're not interested
	 * in link state events, so just mark our idea of the link as down
	 * and ignore the event.
	 */
	if (!netif_running(ndev) || !netif_device_present(ndev)) {
		fep->link = 0;
	} else if (phy_dev->link) {
		if (!fep->link) {
			fep->link = phy_dev->link;
			status_change = 1;
		}

		if (fep->full_duplex != phy_dev->duplex) {
			fep->full_duplex = phy_dev->duplex;
			status_change = 1;
		}

		if (phy_dev->speed != fep->speed) {
			fep->speed = phy_dev->speed;
			status_change = 1;
		}

		/* if any of the above changed restart the FEC */
		if (status_change) {
			netif_tx_lock_bh(ndev);
			fec_enet_uio_restart(ndev);
			netif_tx_wake_all_queues(ndev);
			netif_tx_unlock_bh(ndev);
		}
	} else {
		if (fep->link) {
			netif_tx_lock_bh(ndev);
			fec_uio_stop(ndev);
			netif_tx_unlock_bh(ndev);
			fep->link = phy_dev->link;
			status_change = 1;
		}
	}

	if (status_change)
		phy_print_status(phy_dev);
}

static int fec_restore_mii_bus(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	int ret;

	ret = pm_runtime_get_sync(&fep->pdev->dev);
	if (ret < 0)
		return ret;

	writel(0xffc00000, fep->hwp + FEC_IEVENT);
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);
	writel(FEC_ENET_MII, fep->hwp + FEC_IMASK);
	writel(FEC_ENET_ETHEREN, fep->hwp + FEC_ECNTRL);

	pm_runtime_mark_last_busy(&fep->pdev->dev);
	pm_runtime_put_autosuspend(&fep->pdev->dev);
	return 0;
}

static int fec_enet_uio_mii_probe(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct phy_device *phy_dev = NULL;
	char mdio_bus_id[MII_BUS_ID_SIZE];
	char phy_name[MII_BUS_ID_SIZE + 3];
	int phy_id;
	int dev_id = fep->dev_id;

	if (fep->phy_node) {
		phy_dev = of_phy_connect(ndev, fep->phy_node,
					 &fec_enet_adjust_link, 0,
					 fep->phy_interface);
		if (!phy_dev) {
			netdev_err(ndev, "Unable to connect to phy\n");
			return -ENODEV;
		}
	} else {
		/* check for attached phy */
		for (phy_id = 0; (phy_id < PHY_MAX_ADDR); phy_id++) {
			if (!mdiobus_is_registered_device(fep->mii_bus, phy_id))
				continue;
			if (dev_id--)
				continue;
			strscpy(mdio_bus_id, fep->mii_bus->id, MII_BUS_ID_SIZE);
			break;
		}

		if (phy_id >= PHY_MAX_ADDR) {
			netdev_info(ndev, "no PHY, assuming direct connection to switch\n");
			strscpy(mdio_bus_id, "fixed-0", MII_BUS_ID_SIZE);
			phy_id = 0;
		}

		snprintf(phy_name, sizeof(phy_name),
			 PHY_ID_FMT, mdio_bus_id, phy_id);
		phy_dev = phy_connect(ndev, phy_name, &fec_enet_adjust_link,
				      fep->phy_interface);
	}

	if (IS_ERR(phy_dev)) {
		netdev_err(ndev, "could not attach to PHY\n");
		return PTR_ERR(phy_dev);
	}
	/* mask with MAC supported features */
	if (fep->quirks & FEC_QUIRK_HAS_GBIT) {
		phy_set_max_speed(phy_dev, 1000);
		phy_remove_link_mode(phy_dev,
				     ETHTOOL_LINK_MODE_1000baseT_Half_BIT);
#if !defined(CONFIG_M5272)
		phy_support_sym_pause(phy_dev);
#endif
	} else {
		phy_set_max_speed(phy_dev, 100);
	}
	fep->link = 0;
	fep->full_duplex = 0;

	phy_attached_info(phy_dev);

	return 0;
}

static void fec_enet_uio_timeout_work(struct work_struct *work)
{
	struct fec_enet_private *fep =
		container_of(work, struct fec_enet_private, tx_timeout_work);
	struct net_device *ndev = fep->netdev;

	rtnl_lock();
	if (netif_device_present(ndev) || netif_running(ndev)) {
		netif_tx_lock_bh(ndev);
		fec_enet_uio_restart(ndev);
		netif_tx_wake_all_queues(ndev);
		netif_tx_unlock_bh(ndev);
	}
	rtnl_unlock();
}

static int fec_uio_open(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int fec_uio_release(struct uio_info *info, struct inode *inode)
{
	return 0;
}

static int fec_uio_mmap(struct uio_info *info, struct vm_area_struct *vma)
{
	u32 ret;
	u32 pfn;

	pfn = (info->mem[vma->vm_pgoff].addr) >> PAGE_SHIFT;

	if (vma->vm_pgoff)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start, pfn,
			      vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (ret) {
		/* Error Handle */
		pr_info("remap_pfn_range failed");
	}
	return ret;
}

static int __init uio_init(struct fec_dev *fec_dev)
{
	int ret;
	struct fec_uio_info *fec_uio_info;

	fec_uio_info = &fec_dev->info;
	atomic_set(&fec_uio_info->ref, 0);
	fec_uio_info->uio_info.version = fec_uio_version;
	fec_uio_info->uio_info.name = fec_dev->info.name;

	fec_uio_info->uio_info.mem[0].name = "FEC_REG_SPACE";
	fec_uio_info->uio_info.mem[0].addr = fec_dev->res->start;
	fec_uio_info->uio_info.mem[0].size = 0x1000;
	fec_uio_info->uio_info.mem[0].internal_addr = 0;
	fec_uio_info->uio_info.mem[0].memtype = UIO_MEM_PHYS;

	fec_uio_info->uio_info.mem[1].name = "FEC_BD_SPACE";
	fec_uio_info->uio_info.mem[1].addr = bd_dma;
	fec_uio_info->uio_info.mem[1].size = bd_size;
	fec_uio_info->uio_info.mem[1].memtype = UIO_MEM_PHYS;

	fec_uio_info->uio_info.open = fec_uio_open;
	fec_uio_info->uio_info.release = fec_uio_release;
	/* Custom mmap function. */
	fec_uio_info->uio_info.mmap = fec_uio_mmap;
	fec_uio_info->uio_info.priv = fec_dev;

	ret = uio_register_device(fec_dev->dev, &fec_uio_info->uio_info);
	if (ret == -517)
		return ret;
	if (ret) {
		dev_err(fec_dev->dev, "fec_uio: UIO registration failed\n");
		return ret;
	}
	return 0;
}

/**
 * fec_ptp_adjfreq - adjust ptp cycle frequency
 * @ptp: the ptp clock structure
 * @ppb: parts per billion adjustment from base
 *
 * Adjust the frequency of the ptp cycle counter by the
 * indicated ppb from the base frequency.
 *
 * Because ENET hardware frequency adjust is complex,
 * using software method to do that.
 */
static int fec_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	unsigned long flags;
	int neg_adj = 0;
	u32 i, tmp;
	u32 corr_inc, corr_period;
	u32 corr_ns;
	u64 lhs, rhs;

	struct fec_enet_private *fep =
		container_of(ptp, struct fec_enet_private, ptp_caps);

	if (ppb == 0)
		return 0;

	if (ppb < 0) {
		ppb = -ppb;
		neg_adj = 1;
	}

	/* In theory, corr_inc/corr_period = ppb/NSEC_PER_SEC;
	 * Try to find the corr_inc  between 1 to fep->ptp_inc to
	 * meet adjustment requirement.
	 */
	lhs = NSEC_PER_SEC;
	rhs = (u64)ppb * (u64)fep->ptp_inc;
	for (i = 1; i <= fep->ptp_inc; i++) {
		if (lhs >= rhs) {
			corr_inc = i;
			corr_period = div_u64(lhs, rhs);
			break;
		}
		lhs += NSEC_PER_SEC;
	}
	/* Not found? Set it to high value - double speed
	 * correct in every clock step.
	 */
	if (i > fep->ptp_inc) {
		corr_inc = fep->ptp_inc;
		corr_period = 1;
	}

	if (neg_adj)
		corr_ns = fep->ptp_inc - corr_inc;
	else
		corr_ns = fep->ptp_inc + corr_inc;

	spin_lock_irqsave(&fep->tmreg_lock, flags);

	tmp = readl(fep->hwp + FEC_ATIME_INC) & FEC_T_INC_MASK;
	tmp |= corr_ns << FEC_T_INC_CORR_OFFSET;
	writel(tmp, fep->hwp + FEC_ATIME_INC);
	corr_period = corr_period > 1 ? corr_period - 1 : corr_period;
	writel(corr_period, fep->hwp + FEC_ATIME_CORR);
	/* dummy read to update the timer. */
	timecounter_read(&fep->tc);

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_adjtime
 * @ptp: the ptp clock structure
 * @delta: offset to adjust the cycle counter by
 *
 * adjust the timer by resetting the timecounter structure.
 */
static int fec_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct fec_enet_private *fep =
		container_of(ptp, struct fec_enet_private, ptp_caps);
	unsigned long flags;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	timecounter_adjtime(&fep->tc, delta);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_gettime
 * @ptp: the ptp clock structure
 * @ts: timespec structure to hold the current time value
 *
 * read the timecounter and return the correct value on ns,
 * after converting it into a struct timespec.
 */
static int fec_ptp_gettime(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct fec_enet_private *adapter =
		container_of(ptp, struct fec_enet_private, ptp_caps);
	u64 ns;
	unsigned long flags;

	spin_lock_irqsave(&adapter->tmreg_lock, flags);
	ns = timecounter_read(&adapter->tc);
	spin_unlock_irqrestore(&adapter->tmreg_lock, flags);

	*ts = ns_to_timespec64(ns);

	return 0;
}

/**
 * fec_ptp_settime
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time for the cycle counter
 *
 * reset the timecounter to use a new base value instead of the kernel
 * wall timer value.
 */
static int fec_ptp_settime(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct fec_enet_private *fep =
		container_of(ptp, struct fec_enet_private, ptp_caps);

	u64 ns;
	unsigned long flags;
	u32 counter;

	mutex_lock(&fep->ptp_clk_mutex);
	/* Check the ptp clock */
	if (!fep->ptp_clk_on) {
		mutex_unlock(&fep->ptp_clk_mutex);
		return -EINVAL;
	}

	ns = timespec64_to_ns(ts);
	/* Get the timer value based on timestamp.
	 * Update the counter with the masked value.
	 */
	counter = ns & fep->cc.mask;

	spin_lock_irqsave(&fep->tmreg_lock, flags);
	writel(counter, fep->hwp + FEC_ATIME);
	timecounter_init(&fep->tc, &fep->cc, ns);
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
	mutex_unlock(&fep->ptp_clk_mutex);
	return 0;
}

/**
 * fec_ptp_enable_pps
 * @fep: the fec_enet_private structure handle
 * @enable: enable the channel pps output
 *
 * This function enable the PPS output on the timer channel.
 */
static int fec_ptp_enable_pps(struct fec_enet_private *fep, uint enable)
{
	unsigned long flags;
	u32 val, tempval;
	struct timespec64 ts;
	u64 ns;

	val = 0;
	if (!(fep->hwts_tx_en || fep->hwts_rx_en)) {
		dev_err(&fep->pdev->dev, "No ptp stack is running\n");
		return -EINVAL;
	}

	if (fep->pps_enable == enable)
		return 0;

	fep->pps_channel = DEFAULT_PPS_CHANNEL;
	fep->reload_period = PPS_OUTPUT_RELOAD_PERIOD;

	spin_lock_irqsave(&fep->tmreg_lock, flags);

	if (enable) {
		/* clear capture or output compare interrupt status if have */
		writel(FEC_T_TF_MASK, fep->hwp + FEC_TCSR(fep->pps_channel));

		/* It is recommended to double check the TMODE field in the
		 * TCSR register to be cleared before the first compare counter
		 * is written into TCCR register. Just add a double check.
		 */
		val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		do {
			val &= ~(FEC_T_TMODE_MASK);
			writel(val, fep->hwp + FEC_TCSR(fep->pps_channel));
			val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		} while (val & FEC_T_TMODE_MASK);

		/* Dummy read counter to update the counter */
		timecounter_read(&fep->tc);
		/* We want to find the first compare event in the next
		 * second point. So we need to know what the ptp time
		 * is now and how many nanoseconds is ahead to get next second.
		 * The remaining nanosecond ahead before the next second would
		 * be NSEC_PER_SEC - ts.tv_nsec. Add the remaining nanoseconds
		 * to current timer would be next second.
		 */
		tempval = readl(fep->hwp + FEC_ATIME_CTRL);
		tempval |= FEC_T_CTRL_CAPTURE;
		writel(tempval, fep->hwp + FEC_ATIME_CTRL);
		tempval = readl(fep->hwp + FEC_ATIME);
		/* Convert the ptp local counter to 1588 timestamp */
		ns = timecounter_cyc2time(&fep->tc, tempval);
		ts = ns_to_timespec64(ns);

		/* The tempval is  less than 3 seconds, and  so val is less than
		 * 4 seconds. No overflow for 32bit calculation.
		 */
		val = NSEC_PER_SEC - (u32)ts.tv_nsec + tempval;

		/* Need to consider the situation that the current time is
		 * very close to the second point, which means NSEC_PER_SEC
		 * - ts.tv_nsec is close to be zero(For example 20ns); Since
		 * the timer is still running when we calculate the first
		 * compare event, it is possible that the remaining nanoseonds
		 * run out before the compare counter is calculated and
		 * written into TCCR register. To avoid this possibility,
		 * we will set the compare event to be the next of next second.
		 * The current setting is 31-bit timer and wrap around over 2
		 * seconds. So it is okay to set the next of next
		 * seond for the timer.
		 */
		val += NSEC_PER_SEC;

		/* We add (2 * NSEC_PER_SEC - (u32)ts.tv_nsec) to current
		 * ptp counter, which maybe cause 32-bit wrap. Since the
		 * (NSEC_PER_SEC - (u32)ts.tv_nsec) is less than 2 second.
		 * We can ensure the wrap will not cause issue. If the offset
		 * is bigger than fep->cc.mask would be a error.
		 */
		val &= fep->cc.mask;
		writel(val, fep->hwp + FEC_TCCR(fep->pps_channel));

		/* Calculate the second the compare event timestamp */
		fep->next_counter = (val + fep->reload_period) & fep->cc.mask;

		/* Enable compare event when overflow */
		val = readl(fep->hwp + FEC_ATIME_CTRL);
		val |= FEC_T_CTRL_PINPER;
		writel(val, fep->hwp + FEC_ATIME_CTRL);

		/* Compare channel setting. */
		val = readl(fep->hwp + FEC_TCSR(fep->pps_channel));
		val |= (1 << FEC_T_TF_OFFSET | 1 << FEC_T_TIE_OFFSET);
		val &= ~(1 << FEC_T_TDRE_OFFSET);
		val &= ~(FEC_T_TMODE_MASK);
		val |= (FEC_HIGH_PULSE << FEC_T_TMODE_OFFSET);
		writel(val, fep->hwp + FEC_TCSR(fep->pps_channel));

		/* Write the second compare event timestamp and calculate
		 * the third timestamp. Refer the TCCR register detail in
		 * the spec.
		 */
		writel(fep->next_counter,
		       fep->hwp + FEC_TCCR(fep->pps_channel));
		fep->next_counter = (fep->next_counter + fep->reload_period)
				& fep->cc.mask;
	} else {
		writel(0, fep->hwp + FEC_TCSR(fep->pps_channel));
	}

	fep->pps_enable = enable;
	spin_unlock_irqrestore(&fep->tmreg_lock, flags);

	return 0;
}

/**
 * fec_ptp_enable
 * @ptp: the ptp clock structure
 * @rq: the requested feature to change
 * @on: whether to enable or disable the feature
 *
 */
static int fec_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	struct fec_enet_private *fep =
		container_of(ptp, struct fec_enet_private, ptp_caps);
	int ret = 0;

	if (rq->type == PTP_CLK_REQ_PPS) {
		ret = fec_ptp_enable_pps(fep, on);

		return ret;
	}
	return -EOPNOTSUPP;
}

static void
fec_enet_uio_get_queue_num(struct platform_device *pdev,
			   int *num_tx, int *num_rx)
{
	struct device_node *np = pdev->dev.of_node;

	*num_tx = *num_rx = 1;

	if (!np || !of_device_is_available(np))
		return;

	/* parse the num of tx and rx queues */
	of_property_read_u32(np, "fsl,num-tx-queues", num_tx);
	of_property_read_u32(np, "fsl,num-rx-queues", num_rx);

	if (*num_tx < 1 || *num_tx > FEC_ENET_MAX_TX_QS) {
		dev_warn(&pdev->dev, "Invalid num_tx(=%d), fall back to 1\n",
			 *num_tx);
		*num_tx = 1;
		return;
	}

	if (*num_rx < 1 || *num_rx > FEC_ENET_MAX_RX_QS) {
		dev_warn(&pdev->dev, "Invalid num_rx(=%d), fall back to 1\n",
			 *num_rx);
		*num_rx = 1;
		return;
	}
}

/**
 * fec_uio_time_keep - call timecounter_read every second to avoid timer overrun
 * because ENET just support 32bit counter, will timeout in 4s
 */
static void fec_uio_time_keep(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct fec_enet_private *fep =
			container_of(dwork, struct fec_enet_private, time_keep);
	u64 ns;
	unsigned long flags;

	mutex_lock(&fep->ptp_clk_mutex);
	if (fep->ptp_clk_on) {
		spin_lock_irqsave(&fep->tmreg_lock, flags);
		ns = timecounter_read(&fep->tc);
		spin_unlock_irqrestore(&fep->tmreg_lock, flags);
	}
	mutex_unlock(&fep->ptp_clk_mutex);

	schedule_delayed_work(&fep->time_keep, HZ);
}

/* This function checks the pps event and reloads the timer compare counter. */
static irqreturn_t fec_uio_pps_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct fec_enet_private *fep = netdev_priv(ndev);
	u32 val;
	u8 channel = fep->pps_channel;
	struct ptp_clock_event event;

	val = readl(fep->hwp + FEC_TCSR(channel));
	if (val & FEC_T_TF_MASK) {
		/* Write the next compare(not the next according the spec)
		 * value to the register
		 */
		writel(fep->next_counter, fep->hwp + FEC_TCCR(channel));
		do {
			writel(val, fep->hwp + FEC_TCSR(channel));
		} while (readl(fep->hwp + FEC_TCSR(channel)) & FEC_T_TF_MASK);

		/* Update the counter; */
		fep->next_counter = (fep->next_counter + fep->reload_period) &
			fep->cc.mask;

		event.type = PTP_CLOCK_PPS;
		ptp_clock_event(fep->ptp_clock, &event);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/**
 * fec_ptp_read - read raw cycle counter (to be used by time counter)
 * @cc: the cyclecounter structure
 *
 * this function reads the cyclecounter registers and is called by the
 * cyclecounter structure used to construct a ns counter from the
 * arbitrary fixed point registers
 */
static u64 fec_uio_ptp_read(const struct cyclecounter *cc)
{
	struct fec_enet_private *fep =
		container_of(cc, struct fec_enet_private, cc);
	const struct platform_device_id *id_entry =
		platform_get_device_id(fep->pdev);
	u32 tempval;

	tempval = readl(fep->hwp + FEC_ATIME_CTRL);
	tempval |= FEC_T_CTRL_CAPTURE;
	writel(tempval, fep->hwp + FEC_ATIME_CTRL);

	if (id_entry->driver_data & FEC_QUIRK_BUG_CAPTURE)
		udelay(1);

	return readl(fep->hwp + FEC_ATIME);
}

/**
 * fec_uio_ptp_start_cyclecounter - create the cycle counter from hw
 * @ndev: network device
 *
 * this function initializes the timecounter and cyclecounter
 * structures for use in generated a ns counter from the arbitrary
 * fixed point cycles registers in the hardware.
 */
void fec_uio_ptp_start_cyclecounter(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned long flags;
	int inc;

	inc = 1000000000 / fep->cycle_speed;

	/* grab the ptp lock */
	spin_lock_irqsave(&fep->tmreg_lock, flags);

	/* 1ns counter */
	writel(inc << FEC_T_INC_OFFSET, fep->hwp + FEC_ATIME_INC);

	/* use 31-bit timer counter */
	writel(FEC_COUNTER_PERIOD, fep->hwp + FEC_ATIME_EVT_PERIOD);

	writel(FEC_T_CTRL_ENABLE | FEC_T_CTRL_PERIOD_RST,
	       fep->hwp + FEC_ATIME_CTRL);

	memset(&fep->cc, 0, sizeof(fep->cc));
	fep->cc.read = fec_uio_ptp_read;
	fep->cc.mask = CLOCKSOURCE_MASK(31);
	fep->cc.shift = 31;
	fep->cc.mult = FEC_CC_MULT;

	/* reset the ns time counter */
	timecounter_init(&fep->tc, &fep->cc, ktime_to_ns(ktime_get_real()));

	spin_unlock_irqrestore(&fep->tmreg_lock, flags);
}

/**
 * fec_enet_uio_ptp_init
 * @ndev: The FEC network adapter
 *
 * This function performs the required steps for enabling ptp
 * support. If ptp support has already been loaded it simply calls the
 * cyclecounter init routine and exits.
 */

void fec_enet_uio_ptp_init(struct platform_device *pdev, int irq_idx)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	int irq;
	int ret;

	fep->ptp_caps.owner = THIS_MODULE;
	snprintf(fep->ptp_caps.name, 16, "fec ptp");

	fep->ptp_caps.max_adj = 250000000;
	fep->ptp_caps.n_alarm = 0;
	fep->ptp_caps.n_ext_ts = 0;
	fep->ptp_caps.n_per_out = 0;
	fep->ptp_caps.n_pins = 0;
	fep->ptp_caps.pps = 1;

	fep->ptp_caps.adjfreq = fec_ptp_adjfreq;
	fep->ptp_caps.adjtime = fec_ptp_adjtime;
	fep->ptp_caps.gettime64 = fec_ptp_gettime;
	fep->ptp_caps.settime64 = fec_ptp_settime;
	fep->ptp_caps.enable = fec_ptp_enable;

	fep->cycle_speed = clk_get_rate(fep->clk_ptp);
	if (!fep->cycle_speed) {
		fep->cycle_speed = NSEC_PER_SEC;
		dev_err(&fep->pdev->dev, "clk_ptp clock rate is zero\n");
	}
	fep->ptp_inc = NSEC_PER_SEC / fep->cycle_speed;

	spin_lock_init(&fep->tmreg_lock);

	fec_uio_ptp_start_cyclecounter(ndev);

	INIT_DELAYED_WORK(&fep->time_keep, fec_uio_time_keep);

	irq = platform_get_irq_byname_optional(pdev, "pps");
	if (irq < 0)
		irq = platform_get_irq_optional(pdev, irq_idx);
	/* Failure to get an irq is not fatal,
	 * only the PTP_CLOCK_PPS clock events should stop
	 */
	if (irq >= 0) {
		ret = devm_request_irq(&pdev->dev, irq, fec_uio_pps_interrupt,
				       0, pdev->name, ndev);
		if (ret < 0)
			dev_warn(&pdev->dev, "request for pps irq failed(%d)\n",
				 ret);
	}

	fep->ptp_clock = ptp_clock_register(&fep->ptp_caps, &pdev->dev);
	if (IS_ERR(fep->ptp_clock)) {
		fep->ptp_clock = NULL;
		dev_err(&pdev->dev, "ptp_clock_register failed\n");
	}

	schedule_delayed_work(&fep->time_keep, HZ);
}

void fec_enet_uio_ptp_stop(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	cancel_delayed_work_sync(&fep->time_keep);
	if (fep->ptp_clock)
		ptp_clock_unregister(fep->ptp_clock);
}

static void fec_enet_uio_mii_remove(struct fec_enet_private *fep)
{
	if (--mii_cnt == 0) {
		mdiobus_unregister(fep->mii_bus);
		mdiobus_free(fep->mii_bus);
	}
}

static const unsigned short offset_des_active_rxq[] = {
	FEC_R_DES_ACTIVE_0, FEC_R_DES_ACTIVE_1, FEC_R_DES_ACTIVE_2
};

static const unsigned short offset_des_active_txq[] = {
	FEC_X_DES_ACTIVE_0, FEC_X_DES_ACTIVE_1, FEC_X_DES_ACTIVE_2
};

static int fec_enet_uio_init(struct net_device *ndev)
{
	struct fec_enet_private *fep = netdev_priv(ndev);
	unsigned int dsize = fep->bufdesc_ex ? sizeof(struct bufdesc_ex) :
				sizeof(struct bufdesc);
	int ret, i;
	unsigned short tx_ring_size, rx_ring_size;
	unsigned int total_tx_ring_size = 0, total_rx_ring_size = 0;

	/* Check mask of the streaming and coherent API */
	ret = dma_set_mask_and_coherent(&fep->pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_warn(&fep->pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	tx_ring_size = TX_RING_SIZE;
	rx_ring_size = RX_RING_SIZE;

	for (i = 0; i < fep->num_tx_queues; i++)
		total_tx_ring_size += tx_ring_size;
	for (i = 0; i < fep->num_rx_queues; i++)
		total_rx_ring_size += rx_ring_size;

	bd_size = (total_tx_ring_size + total_rx_ring_size) * dsize;

	/* Allocate memory for buffer descriptors. */
	cbd_base = dma_alloc_coherent(&fep->pdev->dev, bd_size, &bd_dma,
				      GFP_KERNEL);
	if (!cbd_base) {
		ret = -ENOMEM;
		goto free_mem;
	}

	return 0;
free_mem:
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);
	return ret;
}

static int
fec_enet_uio_probe(struct platform_device *pdev)
{
	struct fec_enet_private *fep;
	struct fec_platform_data *pdata;
	phy_interface_t interface;
	struct net_device *ndev;
	int i, irq, ret = 0;
	const struct of_device_id *of_id;
	static int dev_id;
	struct device_node *np = pdev->dev.of_node, *phy_node;
	int num_tx_qs;
	int num_rx_qs;
	char irq_name[8];
	int irq_cnt;
	bool reset_again;
	struct fec_uio_devinfo *dev_info;

	fec_enet_uio_get_queue_num(pdev, &num_tx_qs, &num_rx_qs);

	/* Init network device */
	ndev = alloc_etherdev_mqs(sizeof(struct fec_enet_private) +
				FEC_STATS_SIZE, num_tx_qs, num_rx_qs);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	/* setup board info structure */
	fep = netdev_priv(ndev);

	of_id = of_match_device(fec_enet_uio_ids, &pdev->dev);
	if (of_id)
		pdev->id_entry = of_id->data;

	dev_info = (struct fec_uio_devinfo *)pdev->id_entry->driver_data;
	if (dev_info)
		fep->quirks = dev_info->quirks;

	fep->netdev = ndev;
	fep->num_rx_queues = num_rx_qs;
	fep->num_tx_queues = num_tx_qs;

#if !defined(CONFIG_M5272)
	/* default enable pause frame auto negotiation */
	if (fep->quirks & FEC_QUIRK_HAS_GBIT)
		fep->pause_flag |= FEC_PAUSE_FLAG_AUTONEG;
#endif
	/* Select default pin state */
	pinctrl_pm_select_default_state(&pdev->dev);

	/* allocate memory for uio structure */
	fec_dev = kzalloc(sizeof(*fec_dev), GFP_KERNEL);
	if (!fec_dev)
		return -ENOMEM;

	snprintf(fec_dev->info.name, sizeof(fec_dev->info.name) - 1,
		 "%s", uio_device_name);

	fec_dev->dev = &pdev->dev;

	fec_dev->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fep->hwp = ioremap(fec_dev->res->start, 0x1000);
	if (IS_ERR(fep->hwp)) {
		ret = PTR_ERR(fep->hwp);
		goto failed_ioremap;
	}
	fep->pdev = pdev;
	fep->dev_id = dev_id++;

	platform_set_drvdata(pdev, ndev);

	if (of_get_property(np, "fsl,magic-packet", NULL))
		fep->wol_flag |= FEC_WOL_HAS_MAGIC_PACKET;

	phy_node = of_parse_phandle(np, "phy-handle", 0);
	if (!phy_node && of_phy_is_fixed_link(np)) {
		ret = of_phy_register_fixed_link(np);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"broken fixed-link specification\n");
			goto failed_phy;
		}
		phy_node = of_node_get(np);
	}
	fep->phy_node = phy_node;

	ret = of_get_phy_mode(pdev->dev.of_node, &interface);
	if (ret) {
		pdata = dev_get_platdata(&pdev->dev);
		if (pdata)
			fep->phy_interface = pdata->phy;
		else
			fep->phy_interface = PHY_INTERFACE_MODE_MII;
	} else {
		fep->phy_interface = interface;
	}

	request_bus_freq(BUS_FREQ_HIGH);

	fep->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(fep->clk_ipg)) {
		ret = PTR_ERR(fep->clk_ipg);
		goto failed_clk;
	}

	fep->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(fep->clk_ahb)) {
		ret = PTR_ERR(fep->clk_ahb);
		goto failed_clk;
	}

	fep->itr_clk_rate = clk_get_rate(fep->clk_ahb);

	/* enet_out is optional, depends on board */
	fep->clk_enet_out = devm_clk_get(&pdev->dev, "enet_out");
	if (IS_ERR(fep->clk_enet_out))
		fep->clk_enet_out = NULL;

	fep->ptp_clk_on = false;
	mutex_init(&fep->ptp_clk_mutex);

	/* clk_ref is optional, depends on board */
	fep->clk_ref = devm_clk_get(&pdev->dev, "enet_clk_ref");
	if (IS_ERR(fep->clk_ref))
		fep->clk_ref = NULL;

#if EXTEND_BUF
	fep->bufdesc_ex = fep->quirks & FEC_QUIRK_HAS_BUFDESC_EX;
	fep->clk_ptp = devm_clk_get(&pdev->dev, "ptp");
	if (IS_ERR(fep->clk_ptp)) {
		fep->clk_ptp = NULL;
		fep->bufdesc_ex = false;
	}
#endif
	ret = fec_enet_uio_clk_enable(ndev, true);
	if (ret)
		goto failed_clk;
	ret = clk_prepare_enable(fep->clk_ipg);
	if (ret)
		goto failed_clk_ipg;
	ret = clk_prepare_enable(fep->clk_ahb);
	if (ret)
		goto failed_clk_ahb;

	fep->reg_phy = devm_regulator_get_optional(&pdev->dev, "phy");
	if (!IS_ERR(fep->reg_phy)) {
		ret = regulator_enable(fep->reg_phy);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to enable phy regulator: %d\n", ret);
			goto failed_regulator;
		}
	} else {
		if (PTR_ERR(fep->reg_phy) == -EPROBE_DEFER) {
			ret = -EPROBE_DEFER;
			goto failed_regulator;
		}
		fep->reg_phy = NULL;
	}

	pm_runtime_set_autosuspend_delay(&pdev->dev, FEC_MDIO_PM_TIMEOUT);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = fec_enet_uio_reset_phy(pdev);
	if (ret)
		goto failed_reset;
	irq_cnt = fec_enet_uio_get_irq_cnt(pdev);
#if EXTEND_BUF
	if (fep->bufdesc_ex)
		fec_enet_uio_ptp_init(pdev, irq_cnt);
#endif
	ret = fec_enet_uio_init(ndev);
	if (ret)
		goto failed_init;

	/* Register UIO */
	ret = uio_init(fec_dev);
	if (ret) {
		if (ret == -517) {
			dev_info(&pdev->dev,
				 "Driver request probe retry: %s\n", __func__);
			goto out_unmap;
		} else {
			dev_err(&pdev->dev, "UIO init Failed\n");
			goto abort;
		}
	}
	dev_info(fec_dev->dev, "UIO device full name %s initialized\n",
		 fec_dev->info.name);

	fec_enet_uio_restart(ndev);

	for (i = 0; i < irq_cnt; i++) {
		snprintf(irq_name, sizeof(irq_name), "int%d", i);
		irq = platform_get_irq_byname_optional(pdev, irq_name);
		if (irq < 0)
			irq = platform_get_irq(pdev, i);
		if (irq < 0) {
			ret = irq;
			goto failed_irq;
		}
		ret = devm_request_irq(&pdev->dev, irq, fec_enet_uio_interrupt,
				       0, pdev->name, ndev);
		if (ret)
			goto failed_irq;

		fep->irq[i] = irq;
	}

	ret = fec_enet_uio_mii_init(pdev);
	if (ret)
		goto failed_mii_init;

	/* Carrier starts down, phylib will bring it up */
	netif_carrier_off(ndev);

	device_init_wakeup(&ndev->dev, fep->wol_flag &
			FEC_WOL_HAS_MAGIC_PACKET);

	if (fep->bufdesc_ex && fep->ptp_clock)
		netdev_info(ndev, "registered PHC device %d\n", fep->dev_id);

	fep->rx_copybreak = COPYBREAK_DEFAULT;
	INIT_WORK(&fep->tx_timeout_work, fec_enet_uio_timeout_work);

	pm_runtime_get_sync(&fep->pdev->dev);
	if (ndev->phydev && ndev->phydev->drv)
		reset_again = false;
	else
		reset_again = true;

	/* Set MII speed */
	writel(fep->phy_speed, fep->hwp + FEC_MII_SPEED);

	ret = fec_enet_uio_mii_probe(ndev);
	if (ret) {
		netdev_err(ndev, "%s: fec_phy_init() failed\n",
			   __func__);
		goto failed_ioremap;
	}
	if (reset_again)
		phy_reset_after_clk_enable(ndev->phydev);
	phy_start(ndev->phydev);
	device_set_wakeup_enable(&ndev->dev, fep->wol_flag &
			FEC_WOL_FLAG_ENABLE);
	return 0;

failed_mii_init:
failed_irq:
failed_init:
	fec_enet_uio_ptp_stop(pdev);
failed_reset:
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);
failed_regulator:
	clk_disable_unprepare(fep->clk_ahb);
failed_clk_ahb:
	clk_disable_unprepare(fep->clk_ipg);
failed_clk_ipg:
	fec_enet_uio_clk_enable(ndev, false);
failed_clk:
	release_bus_freq(BUS_FREQ_HIGH);
	if (of_phy_is_fixed_link(np))
		of_phy_deregister_fixed_link(np);
	of_node_put(phy_node);
failed_phy:
	dev_id--;
failed_ioremap:
	free_netdev(ndev);

	return ret;
out_unmap:
	dev_id--;
	kfree(fec_dev);
	iounmap(fep->hwp);
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);
	free_netdev(ndev);
	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	fec_enet_uio_clk_enable(ndev, false);
	if (of_phy_is_fixed_link(np))
		of_phy_deregister_fixed_link(np);
	of_node_put(phy_node);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return -EPROBE_DEFER;
abort:
	return ret;
}

static int
fec_enet_uio_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	struct device_node *np = pdev->dev.of_node;
	int ret;

	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0)
		return ret;

	cancel_work_sync(&fep->tx_timeout_work);
	fec_enet_uio_ptp_stop(pdev);
	kfree(fec_dev);
	iounmap(fep->hwp);
	dma_free_coherent(&fep->pdev->dev, bd_size, cbd_base, bd_dma);

	uio_unregister_device(&fec_dev->info.uio_info);

	fec_enet_uio_mii_remove(fep);
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);

	if (of_phy_is_fixed_link(np))
		of_phy_deregister_fixed_link(np);
	of_node_put(fep->phy_node);
	free_netdev(ndev);

	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static int __maybe_unused fec_enet_uio_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	rtnl_lock();
	if (netif_running(ndev)) {
		int ret;

		if (fep->wol_flag & FEC_WOL_FLAG_ENABLE)
			fep->wol_flag |= FEC_WOL_FLAG_SLEEP_ON;
		phy_stop(ndev->phydev);
		netif_tx_lock_bh(ndev);
		netif_device_detach(ndev);
		netif_tx_unlock_bh(ndev);
		fec_uio_stop(ndev);
		fec_enet_uio_clk_enable(ndev, false);
		if (!(fep->wol_flag & FEC_WOL_FLAG_ENABLE)) {
			fec_enet_uio_irqs_disable(ndev);
			pinctrl_pm_select_sleep_state(&fep->pdev->dev);
		} else {
			fec_enet_uio_stop_mode(fep, true);
			disable_irq(fep->wake_irq);
			enable_irq_wake(fep->wake_irq);
		}
		fec_enet_uio_clk_enable(ndev, false);

		fep->rpm_active = !pm_runtime_status_suspended(dev);
		if (fep->rpm_active) {
			ret = pm_runtime_force_suspend(dev);
			if (ret < 0)
				return ret;
		} else if (fep->mii_bus_share && !ndev->phydev) {
			pinctrl_pm_select_sleep_state(&fep->pdev->dev);
		}
	}
	rtnl_unlock();

	if (fep->reg_phy && !(fep->wol_flag & FEC_WOL_FLAG_ENABLE))
		regulator_disable(fep->reg_phy);

	/* SOC supply clock to phy, when clock is disabled, phy link down
	 * SOC control phy regulator, when regulator is disabled, phy link down
	 */
	if (fep->clk_enet_out || fep->reg_phy)
		fep->link = 0;

	return 0;
}

static int __maybe_unused fec_enet_uio_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	int ret = 0;
	int val;

	if (fep->reg_phy && !(fep->wol_flag & FEC_WOL_FLAG_ENABLE)) {
		ret = regulator_enable(fep->reg_phy);
		if (ret)
			return ret;
	}

	rtnl_lock();
	if (netif_running(ndev)) {
		if (fep->rpm_active)
			pm_runtime_force_resume(dev);

		ret = fec_enet_uio_clk_enable(ndev, true);
		if (ret) {
			rtnl_unlock();
			goto failed_clk;
		}
		if (fep->wol_flag & FEC_WOL_FLAG_ENABLE) {
			disable_irq_wake(fep->wake_irq);
			fec_enet_uio_stop_mode(fep, false);
			enable_irq(fep->wake_irq);
			val = readl(fep->hwp + FEC_ECNTRL);
			val &= ~(FEC_ECR_MAGICEN | FEC_ECR_SLEEP);
			writel(val, fep->hwp + FEC_ECNTRL);
			fep->wol_flag &= ~FEC_WOL_FLAG_SLEEP_ON;
		} else {
			pinctrl_pm_select_default_state(&fep->pdev->dev);
		}
		fec_enet_uio_restart(ndev);
		netif_tx_lock_bh(ndev);
		netif_device_attach(ndev);
		netif_tx_unlock_bh(ndev);
		phy_start(ndev->phydev);
	} else if (fep->mii_bus_share && !ndev->phydev) {
		pinctrl_pm_select_default_state(&fep->pdev->dev);
		/* And then recovery mii bus */
		ret = fec_restore_mii_bus(ndev);
	}
	rtnl_unlock();

	return ret;

failed_clk:
	if (fep->reg_phy)
		regulator_disable(fep->reg_phy);
	return ret;
}

static int __maybe_unused fec_enet_uio_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fec_enet_private *fep = netdev_priv(ndev);

	clk_disable_unprepare(fep->clk_ahb);
	clk_disable_unprepare(fep->clk_ipg);
	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int __maybe_unused fec_enet_uio_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct fec_enet_private *fep = netdev_priv(ndev);
	int ret;

	request_bus_freq(BUS_FREQ_HIGH);

	ret = clk_prepare_enable(fep->clk_ahb);
	if (ret)
		return ret;
	ret = clk_prepare_enable(fep->clk_ipg);
	if (ret)
		goto failed_clk_ipg;

	return 0;

failed_clk_ipg:
	clk_disable_unprepare(fep->clk_ahb);
	release_bus_freq(BUS_FREQ_HIGH);
	return ret;
}

static const struct dev_pm_ops fec_enet_uio_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fec_enet_uio_suspend, fec_enet_uio_resume)
	SET_RUNTIME_PM_OPS(fec_enet_uio_runtime_suspend,
			   fec_enet_uio_runtime_resume, NULL)
};

static struct platform_driver fec_enet_uio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.pm     = &fec_enet_uio_pm_ops,
		.of_match_table = fec_enet_uio_ids,
		.suppress_bind_attrs = true,
	},
	.id_table = fec_enet_uio_devtype,
	.prevent_deferred_probe = false,
	.probe = fec_enet_uio_probe,
	.remove = fec_enet_uio_remove,
};

module_platform_driver(fec_enet_uio_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("i.MX FEC UIO Driver");
