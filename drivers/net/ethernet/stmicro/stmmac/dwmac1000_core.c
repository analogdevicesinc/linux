// SPDX-License-Identifier: GPL-2.0-only
/*******************************************************************************
  This is the driver for the GMAC on-chip Ethernet controller for ST SoCs.
  DWC Ether MAC 10/100/1000 Universal version 3.41a  has been used for
  developing this code.

  This only implements the mac core functions for this chip.

  Copyright (C) 2007-2009  STMicroelectronics Ltd


  Author: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/crc32.h>
#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/string_choices.h>
#include "stmmac.h"
#include "stmmac_pcs.h"
#include "stmmac_ptp.h"
#include "dwmac1000.h"

static int dwmac1000_pcs_init(struct stmmac_priv *priv)
{
	if (!priv->dma_cap.pcs)
		return 0;

	return stmmac_integrated_pcs_init(priv, GMAC_PCS_BASE);
}

static void dwmac1000_core_init(struct mac_device_info *hw,
				struct net_device *dev)
{
	void __iomem *ioaddr = hw->pcsr;
	int mtu = dev->mtu;
	u32 value;

	/* Configure GMAC core */
	value = readl(ioaddr + GMAC_CONTROL);

	if (mtu > 1500)
		value |= GMAC_CONTROL_2K;
	if (mtu > 2000)
		value |= GMAC_CONTROL_JE;

	writel(value | GMAC_CORE_INIT, ioaddr + GMAC_CONTROL);

	/* Mask GMAC interrupts */
	value = GMAC_INT_DEFAULT_MASK;

	if (hw->pcs)
		value &= ~GMAC_INT_DISABLE_PCS;

	writel(value, ioaddr + GMAC_INT_MASK);

#ifdef STMMAC_VLAN_TAG_USED
	/* Tag detection without filtering */
	writel(0x0, ioaddr + GMAC_VLAN_TAG);
#endif
}

static int dwmac1000_rx_ipc_enable(struct mac_device_info *hw)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value = readl(ioaddr + GMAC_CONTROL);

	if (hw->rx_csum)
		value |= GMAC_CONTROL_IPC;
	else
		value &= ~GMAC_CONTROL_IPC;

	writel(value, ioaddr + GMAC_CONTROL);

	value = readl(ioaddr + GMAC_CONTROL);

	return !!(value & GMAC_CONTROL_IPC);
}

static void dwmac1000_dump_regs(struct mac_device_info *hw, u32 *reg_space)
{
	void __iomem *ioaddr = hw->pcsr;
	int i;

	for (i = 0; i < 55; i++)
		reg_space[i] = readl(ioaddr + i * 4);
}

static void dwmac1000_set_umac_addr(struct mac_device_info *hw,
				    const unsigned char *addr,
				    unsigned int reg_n)
{
	void __iomem *ioaddr = hw->pcsr;
	stmmac_set_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
			    GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_get_umac_addr(struct mac_device_info *hw,
				    unsigned char *addr,
				    unsigned int reg_n)
{
	void __iomem *ioaddr = hw->pcsr;
	stmmac_get_mac_addr(ioaddr, addr, GMAC_ADDR_HIGH(reg_n),
			    GMAC_ADDR_LOW(reg_n));
}

static void dwmac1000_set_mchash(void __iomem *ioaddr, u32 *mcfilterbits,
				 int mcbitslog2)
{
	int numhashregs, regs;

	switch (mcbitslog2) {
	case 6:
		writel(mcfilterbits[0], ioaddr + GMAC_HASH_LOW);
		writel(mcfilterbits[1], ioaddr + GMAC_HASH_HIGH);
		return;
	case 7:
		numhashregs = 4;
		break;
	case 8:
		numhashregs = 8;
		break;
	default:
		pr_debug("STMMAC: err in setting multicast filter\n");
		return;
	}
	for (regs = 0; regs < numhashregs; regs++)
		writel(mcfilterbits[regs],
		       ioaddr + GMAC_EXTHASH_BASE + regs * 4);
}

static void dwmac1000_set_filter(struct mac_device_info *hw,
				 struct net_device *dev)
{
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	unsigned int value = 0;
	unsigned int perfect_addr_number = hw->unicast_filter_entries;
	u32 mc_filter[8];
	int mcbitslog2 = hw->mcast_bits_log2;

	pr_debug("%s: # mcasts %d, # unicast %d\n", __func__,
		 netdev_mc_count(dev), netdev_uc_count(dev));

	memset(mc_filter, 0, sizeof(mc_filter));

	if (dev->flags & IFF_PROMISC) {
		value = GMAC_FRAME_FILTER_PR | GMAC_FRAME_FILTER_PCF;
	} else if (dev->flags & IFF_ALLMULTI) {
		value = GMAC_FRAME_FILTER_PM;	/* pass all multi */
	} else if (!netdev_mc_empty(dev) && (mcbitslog2 == 0)) {
		/* Fall back to all multicast if we've no filter */
		value = GMAC_FRAME_FILTER_PM;
	} else if (!netdev_mc_empty(dev)) {
		struct netdev_hw_addr *ha;

		/* Hash filter for multicast */
		value = GMAC_FRAME_FILTER_HMC;

		netdev_for_each_mc_addr(ha, dev) {
			/* The upper n bits of the calculated CRC are used to
			 * index the contents of the hash table. The number of
			 * bits used depends on the hardware configuration
			 * selected at core configuration time.
			 */
			int bit_nr = bitrev32(~crc32_le(~0, ha->addr,
					      ETH_ALEN)) >>
					      (32 - mcbitslog2);
			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register.
			 */
			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
	}

	value |= GMAC_FRAME_FILTER_HPF;
	dwmac1000_set_mchash(ioaddr, mc_filter, mcbitslog2);

	/* Handle multiple unicast addresses (perfect filtering) */
	if (netdev_uc_count(dev) > perfect_addr_number)
		/* Switch to promiscuous mode if more than unicast
		 * addresses are requested than supported by hardware.
		 */
		value |= GMAC_FRAME_FILTER_PR;
	else {
		int reg = 1;
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, dev) {
			stmmac_set_mac_addr(ioaddr, ha->addr,
					    GMAC_ADDR_HIGH(reg),
					    GMAC_ADDR_LOW(reg));
			reg++;
		}

		while (reg < perfect_addr_number) {
			writel(0, ioaddr + GMAC_ADDR_HIGH(reg));
			writel(0, ioaddr + GMAC_ADDR_LOW(reg));
			reg++;
		}
	}

#ifdef FRAME_FILTER_DEBUG
	/* Enable Receive all mode (to debug filtering_fail errors) */
	value |= GMAC_FRAME_FILTER_RA;
#endif
	writel(value, ioaddr + GMAC_FRAME_FILTER);
}


static void dwmac1000_flow_ctrl(struct mac_device_info *hw, unsigned int duplex,
				unsigned int fc, unsigned int pause_time,
				u32 tx_cnt)
{
	void __iomem *ioaddr = hw->pcsr;
	/* Set flow such that DZPQ in Mac Register 6 is 0,
	 * and unicast pause detect is enabled.
	 */
	unsigned int flow = GMAC_FLOW_CTRL_UP;

	pr_debug("GMAC Flow-Control:\n");
	if (fc & FLOW_RX) {
		pr_debug("\tReceive Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_RFE;
	}
	if (fc & FLOW_TX) {
		pr_debug("\tTransmit Flow-Control ON\n");
		flow |= GMAC_FLOW_CTRL_TFE;
	}

	if (duplex) {
		pr_debug("\tduplex mode: PAUSE %d\n", pause_time);
		flow |= (pause_time << GMAC_FLOW_CTRL_PT_SHIFT);
	}

	writel(flow, ioaddr + GMAC_FLOW_CTRL);
}

static void dwmac1000_pmt(struct mac_device_info *hw, unsigned long mode)
{
	void __iomem *ioaddr = hw->pcsr;
	unsigned int pmt = 0;

	if (mode & WAKE_MAGIC) {
		pr_debug("GMAC: WOL Magic frame\n");
		pmt |= power_down | magic_pkt_en;
	}
	if (mode & WAKE_UCAST) {
		pr_debug("GMAC: WOL on global unicast\n");
		pmt |= power_down | global_unicast | wake_up_frame_en;
	}

	writel(pmt, ioaddr + GMAC_PMT);
}

static int dwmac1000_irq_status(struct mac_device_info *hw,
				struct stmmac_extra_stats *x)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 intr_status = readl(ioaddr + GMAC_INT_STATUS);
	u32 intr_mask = readl(ioaddr + GMAC_INT_MASK);
	int ret = 0;

	/* Discard masked bits */
	intr_status &= ~intr_mask;

	/* Not used events (e.g. MMC interrupts) are not handled. */
	if ((intr_status & GMAC_INT_STATUS_MMCTIS))
		x->mmc_tx_irq_n++;
	if (unlikely(intr_status & GMAC_INT_STATUS_MMCRIS))
		x->mmc_rx_irq_n++;
	if (unlikely(intr_status & GMAC_INT_STATUS_MMCCSUM))
		x->mmc_rx_csum_offload_irq_n++;
	if (unlikely(intr_status & GMAC_INT_DISABLE_PMT)) {
		/* clear the PMT bits 5 and 6 by reading the PMT status reg */
		readl(ioaddr + GMAC_PMT);
		x->irq_receive_pmt_irq_n++;
	}

	/* MAC tx/rx EEE LPI entry/exit interrupts */
	if (intr_status & GMAC_INT_STATUS_LPIIS) {
		/* Clean LPI interrupt by reading the Reg 12 */
		ret = readl(ioaddr + LPI_CTRL_STATUS);

		if (ret & LPI_CTRL_STATUS_TLPIEN)
			x->irq_tx_path_in_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_TLPIEX)
			x->irq_tx_path_exit_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_RLPIEN)
			x->irq_rx_path_in_lpi_mode_n++;
		if (ret & LPI_CTRL_STATUS_RLPIEX)
			x->irq_rx_path_exit_lpi_mode_n++;
	}

	dwmac_pcs_isr(ioaddr, GMAC_PCS_BASE, intr_status, x);

	return ret;
}

static int dwmac1000_set_lpi_mode(struct mac_device_info *hw,
				  enum stmmac_lpi_mode mode,
				  bool en_tx_lpi_clockgating, u32 et)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	if (mode == STMMAC_LPI_TIMER)
		return -EOPNOTSUPP;

	value = readl(ioaddr + LPI_CTRL_STATUS);
	if (mode == STMMAC_LPI_FORCED)
		value |= LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA;
	else
		value &= ~(LPI_CTRL_STATUS_LPIEN | LPI_CTRL_STATUS_LPITXA);
	writel(value, ioaddr + LPI_CTRL_STATUS);

	return 0;
}

static void dwmac1000_set_eee_pls(struct mac_device_info *hw, int link)
{
	void __iomem *ioaddr = hw->pcsr;
	u32 value;

	value = readl(ioaddr + LPI_CTRL_STATUS);

	if (link)
		value |= LPI_CTRL_STATUS_PLS;
	else
		value &= ~LPI_CTRL_STATUS_PLS;

	writel(value, ioaddr + LPI_CTRL_STATUS);
}

static void dwmac1000_set_eee_timer(struct mac_device_info *hw, int ls, int tw)
{
	void __iomem *ioaddr = hw->pcsr;
	int value = ((tw & 0xffff)) | ((ls & 0x7ff) << 16);

	/* Program the timers in the LPI timer control register:
	 * LS: minimum time (ms) for which the link
	 *  status from PHY should be ok before transmitting
	 *  the LPI pattern.
	 * TW: minimum time (us) for which the core waits
	 *  after it has stopped transmitting the LPI pattern.
	 */
	writel(value, ioaddr + LPI_TIMER_CTRL);
}

static void dwmac1000_ctrl_ane(struct stmmac_priv *priv, bool ane,
			       bool srgmi_ral)
{
	dwmac_ctrl_ane(priv->ioaddr, GMAC_PCS_BASE, ane, srgmi_ral);
}

static void dwmac1000_debug(struct stmmac_priv *priv, void __iomem *ioaddr,
			    struct stmmac_extra_stats *x,
			    u32 rx_queues, u32 tx_queues)
{
	u32 value = readl(ioaddr + GMAC_DEBUG);

	if (value & GMAC_DEBUG_TXSTSFSTS)
		x->mtl_tx_status_fifo_full++;
	if (value & GMAC_DEBUG_TXFSTS)
		x->mtl_tx_fifo_not_empty++;
	if (value & GMAC_DEBUG_TWCSTS)
		x->mmtl_fifo_ctrl++;
	if (value & GMAC_DEBUG_TRCSTS_MASK) {
		u32 trcsts = (value & GMAC_DEBUG_TRCSTS_MASK)
			     >> GMAC_DEBUG_TRCSTS_SHIFT;
		if (trcsts == GMAC_DEBUG_TRCSTS_WRITE)
			x->mtl_tx_fifo_read_ctrl_write++;
		else if (trcsts == GMAC_DEBUG_TRCSTS_TXW)
			x->mtl_tx_fifo_read_ctrl_wait++;
		else if (trcsts == GMAC_DEBUG_TRCSTS_READ)
			x->mtl_tx_fifo_read_ctrl_read++;
		else
			x->mtl_tx_fifo_read_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_TXPAUSED)
		x->mac_tx_in_pause++;
	if (value & GMAC_DEBUG_TFCSTS_MASK) {
		u32 tfcsts = (value & GMAC_DEBUG_TFCSTS_MASK)
			      >> GMAC_DEBUG_TFCSTS_SHIFT;

		if (tfcsts == GMAC_DEBUG_TFCSTS_XFER)
			x->mac_tx_frame_ctrl_xfer++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_GEN_PAUSE)
			x->mac_tx_frame_ctrl_pause++;
		else if (tfcsts == GMAC_DEBUG_TFCSTS_WAIT)
			x->mac_tx_frame_ctrl_wait++;
		else
			x->mac_tx_frame_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_TPESTS)
		x->mac_gmii_tx_proto_engine++;
	if (value & GMAC_DEBUG_RXFSTS_MASK) {
		u32 rxfsts = (value & GMAC_DEBUG_RXFSTS_MASK)
			     >> GMAC_DEBUG_RRCSTS_SHIFT;

		if (rxfsts == GMAC_DEBUG_RXFSTS_FULL)
			x->mtl_rx_fifo_fill_level_full++;
		else if (rxfsts == GMAC_DEBUG_RXFSTS_AT)
			x->mtl_rx_fifo_fill_above_thresh++;
		else if (rxfsts == GMAC_DEBUG_RXFSTS_BT)
			x->mtl_rx_fifo_fill_below_thresh++;
		else
			x->mtl_rx_fifo_fill_level_empty++;
	}
	if (value & GMAC_DEBUG_RRCSTS_MASK) {
		u32 rrcsts = (value & GMAC_DEBUG_RRCSTS_MASK) >>
			     GMAC_DEBUG_RRCSTS_SHIFT;

		if (rrcsts == GMAC_DEBUG_RRCSTS_FLUSH)
			x->mtl_rx_fifo_read_ctrl_flush++;
		else if (rrcsts == GMAC_DEBUG_RRCSTS_RSTAT)
			x->mtl_rx_fifo_read_ctrl_read_data++;
		else if (rrcsts == GMAC_DEBUG_RRCSTS_RDATA)
			x->mtl_rx_fifo_read_ctrl_status++;
		else
			x->mtl_rx_fifo_read_ctrl_idle++;
	}
	if (value & GMAC_DEBUG_RWCSTS)
		x->mtl_rx_fifo_ctrl_active++;
	if (value & GMAC_DEBUG_RFCFCSTS_MASK)
		x->mac_rx_frame_ctrl_fifo = (value & GMAC_DEBUG_RFCFCSTS_MASK)
					    >> GMAC_DEBUG_RFCFCSTS_SHIFT;
	if (value & GMAC_DEBUG_RPESTS)
		x->mac_gmii_rx_proto_engine++;
}

static void dwmac1000_set_mac_loopback(void __iomem *ioaddr, bool enable)
{
	u32 value = readl(ioaddr + GMAC_CONTROL);

	if (enable)
		value |= GMAC_CONTROL_LM;
	else
		value &= ~GMAC_CONTROL_LM;

	writel(value, ioaddr + GMAC_CONTROL);
}

const struct stmmac_ops dwmac1000_ops = {
	.pcs_init = dwmac1000_pcs_init,
	.core_init = dwmac1000_core_init,
	.set_mac = stmmac_set_mac,
	.rx_ipc = dwmac1000_rx_ipc_enable,
	.dump_regs = dwmac1000_dump_regs,
	.host_irq_status = dwmac1000_irq_status,
	.set_filter = dwmac1000_set_filter,
	.flow_ctrl = dwmac1000_flow_ctrl,
	.pmt = dwmac1000_pmt,
	.set_umac_addr = dwmac1000_set_umac_addr,
	.get_umac_addr = dwmac1000_get_umac_addr,
	.set_lpi_mode = dwmac1000_set_lpi_mode,
	.set_eee_timer = dwmac1000_set_eee_timer,
	.set_eee_pls = dwmac1000_set_eee_pls,
	.debug = dwmac1000_debug,
	.pcs_ctrl_ane = dwmac1000_ctrl_ane,
	.set_mac_loopback = dwmac1000_set_mac_loopback,
};

int dwmac1000_setup(struct stmmac_priv *priv)
{
	struct mac_device_info *mac = priv->hw;

	dev_info(priv->device, "\tDWMAC1000\n");

	priv->dev->priv_flags |= IFF_UNICAST_FLT;
	mac->pcsr = priv->ioaddr;
	mac->multicast_filter_bins = priv->plat->multicast_filter_bins;
	mac->unicast_filter_entries = priv->plat->unicast_filter_entries;
	mac->mcast_bits_log2 = 0;

	if (mac->multicast_filter_bins)
		mac->mcast_bits_log2 = ilog2(mac->multicast_filter_bins);

	mac->link.caps = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
			 MAC_10 | MAC_100 | MAC_1000;
	mac->link.duplex = GMAC_CONTROL_DM;
	mac->link.speed10 = GMAC_CONTROL_PS;
	mac->link.speed100 = GMAC_CONTROL_PS | GMAC_CONTROL_FES;
	mac->link.speed1000 = 0;
	mac->link.speed_mask = GMAC_CONTROL_PS | GMAC_CONTROL_FES;
	mac->mii.addr = GMAC_MII_ADDR;
	mac->mii.data = GMAC_MII_DATA;
	mac->mii.addr_shift = 11;
	mac->mii.addr_mask = 0x0000F800;
	mac->mii.reg_shift = 6;
	mac->mii.reg_mask = 0x000007C0;
	mac->mii.clk_csr_shift = 2;
	mac->mii.clk_csr_mask = GENMASK(5, 2);

	return 0;
}

/* DWMAC 1000 HW Timestaming ops */

void dwmac1000_get_ptptime(void __iomem *ptpaddr, u64 *ptp_time)
{
	u64 ns;

	ns = readl(ptpaddr + GMAC_PTP_ATNR);
	ns += (u64)readl(ptpaddr + GMAC_PTP_ATSR) * NSEC_PER_SEC;

	*ptp_time = ns;
}

void dwmac1000_timestamp_interrupt(struct stmmac_priv *priv)
{
	struct ptp_clock_event event;
	u32 ts_status, num_snapshot;
	unsigned long flags;
	u64 ptp_time;
	int i;

	/* Clears the timestamp interrupt */
	ts_status = readl(priv->ptpaddr + GMAC3_X_TIMESTAMP_STATUS);

	if (!(priv->plat->flags & STMMAC_FLAG_EXT_SNAPSHOT_EN))
		return;

	num_snapshot = (ts_status & GMAC3_X_ATSNS) >> GMAC3_X_ATSNS_SHIFT;

	for (i = 0; i < num_snapshot; i++) {
		read_lock_irqsave(&priv->ptp_lock, flags);
		stmmac_get_ptptime(priv, priv->ptpaddr, &ptp_time);
		read_unlock_irqrestore(&priv->ptp_lock, flags);

		event.type = PTP_CLOCK_EXTTS;
		event.index = 0;
		event.timestamp = ptp_time;
		ptp_clock_event(priv->ptp_clock, &event);
	}
}

/* DWMAC 1000 ptp_clock_info ops */

static void dwmac1000_timestamp_interrupt_cfg(struct stmmac_priv *priv, bool en)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 intr_mask = readl(ioaddr + GMAC_INT_MASK);

	if (en)
		intr_mask &= ~GMAC_INT_DISABLE_TIMESTAMP;
	else
		intr_mask |= GMAC_INT_DISABLE_TIMESTAMP;

	writel(intr_mask, ioaddr + GMAC_INT_MASK);
}

int dwmac1000_ptp_enable(struct ptp_clock_info *ptp,
			 struct ptp_clock_request *rq, int on)
{
	struct stmmac_priv *priv =
	    container_of(ptp, struct stmmac_priv, ptp_clock_ops);
	void __iomem *ptpaddr = priv->ptpaddr;
	int ret = -EOPNOTSUPP;
	u32 tcr_val;

	switch (rq->type) {
	case PTP_CLK_REQ_EXTTS:
		mutex_lock(&priv->aux_ts_lock);
		tcr_val = readl(ptpaddr + PTP_TCR);

		if (on) {
			tcr_val |= GMAC_PTP_TCR_ATSEN0;
			tcr_val |= GMAC_PTP_TCR_ATSFC;
			priv->plat->flags |= STMMAC_FLAG_EXT_SNAPSHOT_EN;
		} else {
			tcr_val &= ~GMAC_PTP_TCR_ATSEN0;
			priv->plat->flags &= ~STMMAC_FLAG_EXT_SNAPSHOT_EN;
		}

		netdev_dbg(priv->dev, "Auxiliary Snapshot %s.\n",
			   str_enabled_disabled(on));
		writel(tcr_val, ptpaddr + PTP_TCR);

		/* wait for auxts fifo clear to finish */
		ret = readl_poll_timeout(ptpaddr + PTP_TCR, tcr_val,
					 !(tcr_val & GMAC_PTP_TCR_ATSFC),
					 10, 10000);

		mutex_unlock(&priv->aux_ts_lock);

		dwmac1000_timestamp_interrupt_cfg(priv, on);
		break;

	default:
		break;
	}

	return ret;
}
