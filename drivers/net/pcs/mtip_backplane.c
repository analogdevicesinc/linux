// SPDX-License-Identifier: GPL-2.0
/* Driver for MoreThanIP copper backplane AN/LT (Auto-Negotiation and
 * Link Training) core
 *
 * Copyright 2023 NXP
 */

#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/phy/phy.h>

#include "mtip_backplane.h"

#define BP_ETH_STAT_ALWAYS_1		BIT(0)
#define BP_ETH_STAT_1GKX		BIT(1)
#define BP_ETH_STAT_10GKX4		BIT(2)
#define BP_ETH_STAT_10GKR		BIT(3)
#define BP_ETH_STAT_FC_FEC		BIT(4)
#define BP_ETH_STAT_40GKR4		BIT(5)
#define BP_ETH_STAT_40GCR4		BIT(6)
#define BP_ETH_STAT_RS_FEC		BIT(7)
#define BP_ETH_STAT_100GCR10		BIT(8)
#define BP_ETH_STAT_100GKP4		BIT(9)
#define BP_ETH_STAT_100GKR4		BIT(10)
#define BP_ETH_STAT_100GCR4		BIT(11)
#define BP_ETH_STAT_25GKR_S		BIT(12)
#define BP_ETH_STAT_25GKR		BIT(13)

#define BP_ETH_STAT_PARALLEL_DETECT	(BP_ETH_STAT_ALWAYS_1 | \
					 BP_ETH_STAT_1GKX | \
					 BP_ETH_STAT_10GKX4)

#define LT_CTRL_RESTART_TRAINING	BIT(0)
#define LT_CTRL_TRAINING_ENABLE		BIT(1)

#define LT_STAT_RX_STATUS		BIT(0)
#define LT_STAT_FRAME_LOCK		BIT(1)
#define LT_STAT_STARTUP_PROTOCOL_STATUS	BIT(2)
#define LT_STAT_TRAINING_FAILURE	BIT(3)
#define LT_STAT_SIGNAL_DETECT		BIT(15)

#define LT_COM1_MASK			GENMASK(1, 0)
#define LT_COZ_MASK			GENMASK(3, 2)
#define LT_COP1_MASK			GENMASK(5, 4)
#define LT_COM1(x)			((x) & LT_COM1_MASK)
#define LT_COM1_X(x)			((x) & LT_COM1_MASK)
#define LT_COZ(x)			(((x) << 2) & LT_COZ_MASK)
#define LT_COZ_X(x)			(((x) & LT_COZ_MASK) >> 2)
#define LT_COP1(x)			(((x) << 4) & LT_COP1_MASK)
#define LT_COP1_X(x)			(((x) & LT_COP1_MASK) >> 4)

#define LT_COEF_STAT_MASK		(LT_COM1_MASK | LT_COZ_MASK | LT_COP1_MASK)
#define LT_COEF_STAT_ALL_NOT_UPDATED(x)	(((x) & LT_COEF_STAT_MASK) == 0)
#define LT_COEF_STAT_ANY_UPDATED(x)	(((x) & LT_COEF_STAT_MASK) != 0)

#define LT_COEF_UPD_MASK		(LT_COM1_MASK | LT_COZ_MASK | LT_COP1_MASK)
#define LT_COEF_UPD_ALL_HOLD		(LT_COM1(COEF_UPD_HOLD) | \
					LT_COZ(COEF_UPD_HOLD) | \
					LT_COP1(COEF_UPD_HOLD))

#define LT_COEF_UPD_ANYTHING(x)		((x) != 0)
#define LT_COEF_UPD_NOTHING(x)		((x) == 0)

#define LT_COEF_UPD_INIT		BIT(12)
#define LT_COEF_UPD_PRESET		BIT(13)

#define LT_COEF_STAT_RX_READY		BIT(15)

#define C73_ADV_0(x)			(u16)((x) & GENMASK(15, 0))
#define C73_ADV_1(x)			(u16)(((x) & GENMASK(31, 16)) >> 16)
#define C73_ADV_2(x)			(u16)(((x) & GENMASK_ULL(47, 32)) >> 32)

#define VEND_REVISION_CFG_VER_X(x)	(((x) & GENMASK(15, 8)) >> 8)
#define VEND_REVISION_IP_VER_X(x)	(((x) & GENMASK(7, 4)) >> 4)
#define VEND_REVISION_IP_REV_X(x)	((x) & GENMASK(3, 0))

#define IRQ_PCS_TX_LP_IDLE		BIT(9)
#define IRQ_PCS_RX_LP_IDLE		BIT(8)
#define IRQ_PCS_TX_LOC_FAULT		BIT(7)
#define IRQ_PCS_TX_REM_FAULT		BIT(6)
#define IRQ_PCS_RX_LOC_FAULT		BIT(5)
#define IRQ_PCS_RX_REM_FAULT		BIT(4)
#define IRQ_PCS_HI_BER			BIT(3)
#define IRQ_PCS_BLK_LOCK_LOSS		BIT(2)
#define IRQ_PCS_TX_BUF_FAULT		BIT(1)
#define IRQ_PCS_RX_BUF_FAULT		BIT(0)

/* TODO: these seem to be missing on LX2160A, which means that a common driver
 * can't use them. Need to triple-check.
 */
#define IRQ_AN_NEXT_PAGE_RECEIVED	BIT(4)
#define IRQ_AN_BASE_PAGE_RECEIVED	BIT(3)
#define IRQ_AN_PARDET_FAULT		BIT(2)
#define IRQ_AN_PAGE_RECEIVED		BIT(1)
#define IRQ_AN_DONE			BIT(0)
#define IRQ_AN_MASK			(IRQ_AN_NEXT_PAGE_RECEIVED | \
					 IRQ_AN_BASE_PAGE_RECEIVED | \
					 IRQ_AN_PARDET_FAULT | \
					 IRQ_AN_PAGE_RECEIVED | \
					 IRQ_AN_DONE)

#define IRQPOLL_INTERVAL		(HZ / 4)

#define MTIP_CDR_SLEEP_US		100
#define MTIP_CDR_TIMEOUT_US		500000

#define MTIP_LT_END_SLEEP_US		10
#define MTIP_LT_END_TIMEOUT_US		300000

#define MTIP_LT_RESTART_SLEEP_US	10
#define MTIP_LT_RESTART_TIMEOUT_US	1000000

#define MTIP_FRAME_LOCK_SLEEP_US	10
#define MTIP_FRAME_LOCK_TIMEOUT_US	1000000

#define MTIP_RESET_SLEEP_US		100
#define MTIP_RESET_TIMEOUT_US		100000

#define MTIP_BP_ETH_STAT_SLEEP_US	10
#define MTIP_BP_ETH_STAT_TIMEOUT_US	100

#define MTIP_COEF_STAT_SLEEP_US		10
#define MTIP_COEF_STAT_TIMEOUT_US	500000

#define MTIP_LT_TIMEOUT_MS		1000
#define MTIP_AN_TIMEOUT_MS		10000

#define MTIP_MAX_NUM_SUBORDINATES	(MTIP_MAX_NUM_LANES - 1)

enum mtip_an_reg {
	AN_CTRL,
	AN_STAT,
	AN_DEVID1,
	AN_DEVID2,
	AN_DEVS1,
	AN_DEVS2,
	AN_ADV_0,
	AN_ADV_1,
	AN_ADV_2,
	AN_LPA_0,
	AN_LPA_1,
	AN_LPA_2,
	AN_MS_CNT,
	AN_ADV_XNP_0,
	AN_ADV_XNP_1,
	AN_ADV_XNP_2,
	AN_LPA_XNP_0,
	AN_LPA_XNP_1,
	AN_LPA_XNP_2,
	AN_BP_ETH_STAT,
};

enum mtip_lt_reg {
	LT_CTRL,
	LT_STAT,
	LT_LP_COEF,
	LT_LP_STAT,
	LT_LD_COEF,
	LT_LD_STAT,
};

enum mtip_vendor_reg {
	VEND_REVISION,
	VEND_SCRATCH,
	VEND_PCS_IEVENT,
	VEND_PCS_IMASK,
	VEND_AN_IEVENT,
	VEND_AN_IMASK,
};

enum an_restart_reason {
	AN_RESTART_REASON_NONE,
	AN_RESTART_REASON_BOGUS_AN_WA,
	AN_RESTART_REASON_CDR,
	AN_RESTART_REASON_FRAME_LOCK,
	AN_RESTART_REASON_LT,
	AN_RESTART_REASON_NO_HCD,
	AN_RESTART_REASON_PCS_LINK_DROP,
	AN_RESTART_REASON_RECONFIG,
	AN_RESTART_REASON_TIMEOUT_AN,
	AN_RESTART_REASON_USER,
};

enum state_reset_reason {
	STATE_RESET_REASON_AN_RESTART,
	STATE_RESET_REASON_PAGE_RECEIVED,
};

struct mtip_irqpoll {
	struct mutex lock;
	struct delayed_work work;
	u16 old_an_stat;
	u16 old_pcs_stat;
	bool link;
	bool link_ack;
	bool cdr_locked;
	bool run_once;
};

struct mtip_lt_work {
	struct mtip_backplane *priv;
	struct kthread_work work;
};

struct mtip_backplane;

struct mtip_backplane {
	struct mdio_device *mdiodev;
	struct mdio_device *pcs_mdiodev;
	struct mii_bus *bus;
	union {
		struct mtip_backplane *subordinate[MTIP_MAX_NUM_SUBORDINATES];
		struct mtip_backplane *coordinator;
	};
	enum mtip_model model;
	bool is_subordinate;
	int num_subordinates;
	struct phylink_pcs *pcs;
	struct phy *serdes;
	const u16 *an_regs;
	const u16 *lt_regs;
	int lt_mmd;
	enum ethtool_link_mode_bit_indices cfg_link_mode;
	enum ethtool_link_mode_bit_indices link_mode;
	bool link_mode_resolved;
	bool lane_powered_on;
	bool any_request_received;
	unsigned long last_lt_done;
	unsigned long last_an_restart;
	struct mtip_irqpoll irqpoll;
	struct kthread_worker *local_tx_lt_worker;
	struct kthread_worker *remote_tx_lt_worker;
	/* Serialized by an_restart_lock */
	bool an_restart_pending;
	bool an_enabled;
	/* Used for orderly shutdown of LT threads. Modified without any
	 * locking. Set to true only by the irqpoll thread, set to false
	 * by irqpoll and by the LT threads.
	 */
	bool lt_stop_request;
	bool lt_enabled;
	bool local_tx_lt_done;
	bool remote_tx_lt_done;
	/* Serialize concurrent attempts from the local TX and remote TX
	 * kthreads to finalize their side of the link training
	 */
	struct mutex lt_lock;
	struct mutex an_restart_lock;
};

/* Auto-Negotiation Control and Status Registers are on page 0: 0x0 */
static const u16 mtip_lx2160a_an_regs[] = {
	[AN_CTRL] = 0,
	[AN_STAT] = 1,
	[AN_DEVID1] = 0xffff, /* not implemented */
	[AN_DEVID2] = 0xffff, /* not implemented */
	[AN_DEVS1] = 0xffff, /* not implemented */
	[AN_DEVS2] = 0xffff, /* not implemented */
	[AN_ADV_0] = 2,
	[AN_ADV_1] = 3,
	[AN_ADV_2] = 4,
	[AN_LPA_0] = 5,
	[AN_LPA_1] = 6,
	[AN_LPA_2] = 7,
	[AN_MS_CNT] = 8,
	[AN_ADV_XNP_0] = 9,
	[AN_ADV_XNP_1] = 10,
	[AN_ADV_XNP_2] = 11,
	[AN_LPA_XNP_0] = 12,
	[AN_LPA_XNP_1] = 13,
	[AN_LPA_XNP_2] = 14,
	[AN_BP_ETH_STAT] = 15,
};

/* Link Training Control and Status Registers are on page 1: 256 = 0x100 */
static const u16 mtip_lx2160a_lt_regs[] = {
	[LT_CTRL] = 0x100,
	[LT_STAT] = 0x101,
	[LT_LP_COEF] = 0x102,
	[LT_LP_STAT] = 0x103,
	[LT_LD_COEF] = 0x104,
	[LT_LD_STAT] = 0x105,
};

static const u16 mtip_an_regs[] = {
	[AN_CTRL] = MDIO_CTRL1,
	[AN_STAT] = MDIO_STAT1,
	[AN_DEVID1] = MDIO_DEVID1,
	[AN_DEVID2] = MDIO_DEVID2,
	[AN_DEVS1] = MDIO_DEVS1,
	[AN_DEVS2] = MDIO_DEVS2,
	[AN_ADV_0] = MDIO_AN_ADVERTISE,
	[AN_ADV_1] = MDIO_AN_ADVERTISE + 1,
	[AN_ADV_2] = MDIO_AN_ADVERTISE + 2,
	[AN_LPA_0] = MDIO_AN_LPA,
	[AN_LPA_1] = MDIO_AN_LPA + 1,
	[AN_LPA_2] = MDIO_AN_LPA + 2,
	[AN_MS_CNT] = 0x8000,
	[AN_ADV_XNP_0] = 0x16,
	[AN_ADV_XNP_1] = 0x17,
	[AN_ADV_XNP_2] = 0x18,
	[AN_LPA_XNP_0] = 0x19,
	[AN_LPA_XNP_1] = 0x20,
	[AN_LPA_XNP_2] = 0x21,
	[AN_BP_ETH_STAT] = 0x30,
};

static const u16 mtip_lt_regs[] = {
	[LT_CTRL] = 0x96,
	[LT_STAT] = 0x97,
	[LT_LP_COEF] = 0x98,
	[LT_LP_STAT] = 0x99,
	[LT_LD_COEF] = 0x9a,
	[LT_LD_STAT] = 0x9b,
};

/* TODO: determine if this MMD exists at all on LX2160A, and thus if it can
 * be used for interrupts. Reading the registers shows all zeroes.
 */
static const u16 mtip_vendor_regs[] = {
	[VEND_REVISION] = 0,
	[VEND_SCRATCH] = 1,
	[VEND_PCS_IEVENT] = 2,
	[VEND_PCS_IMASK] = 3,
	[VEND_AN_IEVENT] = 4,
	[VEND_AN_IMASK] = 5,
};

/* Keep sorted in order of decreasing link speeds */
static const enum ethtool_link_mode_bit_indices mtip_backplane_link_modes[] = {
	ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT,
	ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT,
	ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_25000baseKR_S_Full_BIT,
	ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
	ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
};

static bool
link_mode_needs_training(enum ethtool_link_mode_bit_indices link_mode)
{
	if (link_mode == ETHTOOL_LINK_MODE_1000baseKX_Full_BIT)
		return false;

	return true;
}

static int for_each_lane(int (*cb)(struct mtip_backplane *priv),
			 struct mtip_backplane *priv)
{
	int i, err;

	err = cb(priv);
	if (err)
		return err;

	for (i = 0; i < priv->num_subordinates; i++) {
		err = cb(priv->subordinate[i]);
		if (err)
			return err;
	}

	return 0;
}

static int for_each_lane_args(int (*cb)(struct mtip_backplane *priv,
					void *args),
			      struct mtip_backplane *priv, void *args)
{
	int i, err;

	err = cb(priv, args);
	if (err)
		return err;

	for (i = 0; i < priv->num_subordinates; i++) {
		err = cb(priv->subordinate[i], args);
		if (err)
			return err;
	}

	return 0;
}

static int mtip_read_an(struct mtip_backplane *priv, enum mtip_an_reg reg)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_read(mdiodev, MDIO_MMD_AN, priv->an_regs[reg]);
}

static int mtip_write_an(struct mtip_backplane *priv, enum mtip_an_reg reg,
			 u16 val)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_write(mdiodev, MDIO_MMD_AN, priv->an_regs[reg],
				 val);
}

static int mtip_modify_an(struct mtip_backplane *priv, enum mtip_an_reg reg,
			  u16 mask, u16 set)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_modify(mdiodev, MDIO_MMD_AN, priv->an_regs[reg],
				  mask, set);
}

static int mtip_read_lt(struct mtip_backplane *priv, enum mtip_lt_reg reg)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_read(mdiodev, priv->lt_mmd, priv->lt_regs[reg]);
}

static int mtip_write_lt(struct mtip_backplane *priv, enum mtip_lt_reg reg,
			 u16 val)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_write(mdiodev, priv->lt_mmd, priv->lt_regs[reg],
				 val);
}

static int mtip_modify_lt(struct mtip_backplane *priv, enum mtip_lt_reg reg,
			  u16 mask, u16 set)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_modify(mdiodev, priv->lt_mmd, priv->lt_regs[reg],
				  mask, set);
}

static int mtip_read_vendor(struct mtip_backplane *priv, enum mtip_vendor_reg reg)
{
	struct mdio_device *mdiodev = priv->mdiodev;

	return mdiodev_c45_read(mdiodev, MDIO_MMD_VEND1, mtip_vendor_regs[reg]);
}

static int mtip_read_pcs(struct mtip_backplane *priv, int reg)
{
	struct mdio_device *mdiodev = priv->pcs_mdiodev;

	return mdiodev_c45_read(mdiodev, MDIO_MMD_PCS, reg);
}

static int mtip_reset_pcs(struct mtip_backplane *priv)
{
	struct mdio_device *mdiodev = priv->pcs_mdiodev;
	int err, val;

	err = mdiodev_c45_modify(mdiodev, MDIO_MMD_PCS, MDIO_CTRL1,
				 MDIO_CTRL1_RESET, MDIO_CTRL1_RESET);
	if (err < 0)
		return err;

	err = read_poll_timeout(mdiodev_c45_read, val,
				val < 0 || !(val & MDIO_CTRL1_RESET),
				MTIP_RESET_SLEEP_US, MTIP_RESET_TIMEOUT_US,
				false, mdiodev, MDIO_MMD_PCS, MDIO_CTRL1);

	return (val < 0) ? val : err;
}

static int mtip_reset_an(struct mtip_backplane *priv)
{
	int err, val;

	err = mtip_modify_an(priv, AN_CTRL, MDIO_CTRL1_RESET,
			     MDIO_CTRL1_RESET);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_an, val,
				val < 0 || !(val & MDIO_CTRL1_RESET),
				MTIP_RESET_SLEEP_US, MTIP_RESET_TIMEOUT_US,
				false, priv, AN_CTRL);

	return (val < 0) ? val : err;
}

static int mtip_check_cdr_lock(struct mtip_backplane *priv,
			       bool *all_lanes_have_cdr_lock)
{
	union phy_status_opts opts = {};
	int i, err;

	err = phy_get_status(priv->serdes, PHY_STATUS_CDR_LOCK, &opts);
	if (err)
		return err;

	*all_lanes_have_cdr_lock = opts.cdr.cdr_locked;

	/* Until C73 resolves a link mode, only the primary lane needs
	 * to have CDR lock. The others may even be powered off.
	 */
	if (!priv->link_mode_resolved || !*all_lanes_have_cdr_lock)
		return 0;

	for (i = 0; i < priv->num_subordinates; i++) {
		err = phy_get_status(priv->subordinate[i]->serdes,
				     PHY_STATUS_CDR_LOCK, &opts);
		if (err)
			return err;

		if (!opts.cdr.cdr_locked) {
			*all_lanes_have_cdr_lock = false;
			return 0;
		}
	}

	return 0;
}

static int mtip_wait_for_cdr_lock(struct mtip_backplane *priv)
{
	bool cdr_locked;
	int err, val;

	err = read_poll_timeout(mtip_check_cdr_lock, val,
				val < 0 || cdr_locked,
				MTIP_CDR_SLEEP_US, MTIP_CDR_TIMEOUT_US,
				false, priv, &cdr_locked);

	return (val < 0) ? val : err;
}

/* May be called before mtip_backplane_create(), which comes from
 * phylink_pcs_ops :: pcs_config()
 */
int mtip_backplane_validate(struct phy *serdes, unsigned long *supported)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mtip_supported) = {};
	const enum ethtool_link_mode_bit_indices *link_modes;
	int i, err;

	linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, mtip_supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Backplane_BIT, mtip_supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Pause_BIT, mtip_supported);
	linkmode_set_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, mtip_supported);

	link_modes = mtip_backplane_link_modes;

	/* Ask the SerDes driver what link modes are supported,
	 * based on the current PLL configuration.
	 */
	for (i = 0; i < ARRAY_SIZE(mtip_backplane_link_modes); i++) {
		err = phy_validate(serdes, PHY_MODE_ETHERNET_LINKMODE,
				   link_modes[i], NULL);
		if (err)
			continue;

		linkmode_set_bit(link_modes[i], mtip_supported);
		dev_dbg(&serdes->dev, "Discovered backplane link mode %s\n",
			ethtool_link_mode_str(link_modes[i]));
	}

	linkmode_and(supported, supported, mtip_supported);

	return 0;
}
EXPORT_SYMBOL(mtip_backplane_validate);

static void mtip_start_irqpoll(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;

	if (priv->is_subordinate)
		return;

	schedule_delayed_work(&irqpoll->work, 0);
}

static void mtip_stop_irqpoll(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;

	if (priv->is_subordinate)
		return;

	cancel_delayed_work_sync(&irqpoll->work);
}

static void mtip_run_irqpoll_once(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;

	if (priv->is_subordinate)
		return;

	irqpoll->run_once = true;
	schedule_delayed_work(&irqpoll->work, 0);
	flush_delayed_work(&irqpoll->work);
}

int mtip_backplane_suspend(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	mtip_stop_irqpoll(priv);

	err = phy_power_off(priv->serdes);
	if (err) {
		dev_err(dev, "Failed to power off SerDes: %pe\n",
			ERR_PTR(err));
		return err;
	}

	priv->lane_powered_on = false;

	/* The link will drop via the CDR lock check after we power off the
	 * SerDes lane, and that is not latched low. So we need to schedule
	 * the irqpoll thread once more, so that we don't miss the event.
	 */
	mtip_run_irqpoll_once(priv);

	return 0;
}
EXPORT_SYMBOL(mtip_backplane_suspend);

int mtip_backplane_resume(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	err = phy_power_on(priv->serdes);
	if (err) {
		dev_err(dev, "Failed to power on SerDes: %pe\n", ERR_PTR(err));
		return err;
	}

	priv->lane_powered_on = true;

	if (!priv->is_subordinate)
		mtip_start_irqpoll(priv);

	return 0;
}
EXPORT_SYMBOL(mtip_backplane_resume);

/* Our LT_LP_STAT register updates only after receiving training frames, so
 * unless we wait for it to lock, there is a risk that after a renegotiation,
 * we act upon information from the previous link training process.
 */
static int mtip_lt_frame_lock(struct mtip_backplane *priv)
{
	int err, val;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || (val & LT_STAT_FRAME_LOCK),
				MTIP_FRAME_LOCK_SLEEP_US, MTIP_FRAME_LOCK_TIMEOUT_US,
				false, priv, LT_STAT);

	return (val < 0) ? val : err;
}

static int mtip_restart_lt(struct mtip_backplane *priv, bool enable)
{
	u16 mask = LT_CTRL_RESTART_TRAINING | LT_CTRL_TRAINING_ENABLE;
	u16 set = LT_CTRL_RESTART_TRAINING;
	int err, val;

	if (enable)
		set |= LT_CTRL_TRAINING_ENABLE;

	err = mtip_modify_lt(priv, LT_CTRL, mask, set);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || !(val & LT_CTRL_RESTART_TRAINING),
				MTIP_LT_RESTART_SLEEP_US, MTIP_LT_RESTART_TIMEOUT_US,
				false, priv, LT_CTRL);

	return (val < 0) ? val : err;
}

/* Enable the lane datapath by disconnecting it from the AN/LT block
 * and connecting it to the PCS. This is called both from the irqpoll thread,
 * as well as from the last link training kthread to finish.
 */
static int mtip_finish_lt(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	if (!priv->lt_enabled)
		return 0;

	err = mtip_restart_lt(priv, false);
	if (err) {
		dev_err(dev, "Failed to disable link training: %pe\n",
			ERR_PTR(err));
		return err;
	}

	/* Subsequent attempts to disable LT will time out, so stop them */
	priv->lt_enabled = false;

	return 0;
}

static struct mtip_irqpoll *mtip_get_irqpoll(struct mtip_backplane *priv)
{
	if (priv->is_subordinate)
		return &priv->coordinator->irqpoll;

	return &priv->irqpoll;
}

static int mtip_stop_lt(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = mtip_get_irqpoll(priv);
	struct device *dev = &priv->mdiodev->dev;
	int err;

	lockdep_assert_held(&irqpoll->lock);

	WRITE_ONCE(priv->lt_stop_request, true);

	kthread_flush_worker(priv->remote_tx_lt_worker);
	kthread_flush_worker(priv->local_tx_lt_worker);

	err = mtip_finish_lt(priv);
	if (err)
		return err;

	dev_dbg(dev, "Link training disabled\n");

	return 0;
}

static int mtip_reset_lt(struct mtip_backplane *priv)
{
	int err;

	/* Don't allow AN to complete without training */
	err = mtip_modify_lt(priv, LT_STAT, LT_STAT_RX_STATUS, 0);
	if (err < 0)
		return err;

	err = mtip_write_lt(priv, LT_LD_COEF, 0);
	if (err < 0)
		return err;

	err = mtip_write_lt(priv, LT_LD_STAT, 0);
	if (err < 0)
		return err;

	priv->any_request_received = false;

	return 0;
}

static const char *mtip_an_restart_reason_str(enum an_restart_reason reason)
{
	switch (reason) {
	case AN_RESTART_REASON_NONE:
		return "none";
	case AN_RESTART_REASON_BOGUS_AN_WA:
		return "workaround for bogus AN";
	case AN_RESTART_REASON_CDR:
		return "CDR not locked";
	case AN_RESTART_REASON_FRAME_LOCK:
		return "no link training frame lock";
	case AN_RESTART_REASON_LT:
		return "link training failed";
	case AN_RESTART_REASON_NO_HCD:
		return "no highest common denominator link mode";
	case AN_RESTART_REASON_PCS_LINK_DROP:
		return "PCS link dropped";
	case AN_RESTART_REASON_RECONFIG:
		return "AN/LT block needs reconfiguration";
	case AN_RESTART_REASON_TIMEOUT_AN:
		return "AN timeout workaround";
	case AN_RESTART_REASON_USER:
		return "requested by user";
	default:
		return "unknown";
	}
}

static const char *mtip_state_reset_reason_str(enum state_reset_reason reason)
{
	switch (reason) {
	case STATE_RESET_REASON_AN_RESTART:
		return "AN restart";
	case STATE_RESET_REASON_PAGE_RECEIVED:
		return "Page received";
	default:
		return "unknown";
	}
}

static int mtip_lt_state_reset(struct mtip_backplane *priv)
{
	priv->local_tx_lt_done = false;
	priv->remote_tx_lt_done = false;
	priv->lt_stop_request = false;

	return 0;
}

/* Reset state when detecting that the previously determined link mode
 * is no longer valid
 */
static int mtip_state_reset(struct mtip_backplane *priv,
			    enum state_reset_reason reason)
{
	int i, err;

	dev_dbg(&priv->mdiodev->dev, "Resetting state: %s\n",
		mtip_state_reset_reason_str(reason));

	priv->link_mode_resolved = false;

	err = for_each_lane(mtip_stop_lt, priv);
	if (err)
		return err;

	err = for_each_lane(mtip_reset_lt, priv);
	if (err < 0)
		return err;

	/* 802.3-2018 clause 73.5.1 recommends: "For any multi-lane PHY, DME
	 * pages shall be transmitted only on lane 0. The transmitters on other
	 * lanes should be disabled". Let's do that and only keep them enabled
	 * when the link mode has been resolved.
	 */
	for (i = 0; i < priv->num_subordinates; i++) {
		if (priv->subordinate[i]->lane_powered_on) {
			err = mtip_backplane_suspend(priv->subordinate[i]);
			if (err)
				return err;
		}
	}

	for_each_lane(mtip_lt_state_reset, priv);

	/* If we received a new base page and the local link training threads
	 * also requested an autoneg restart, it's pointless to fulfill it now.
	 * Clear the request here, after the link training was stopped, so that
	 * we don't race with it.
	 */
	priv->an_restart_pending = false;

	return 0;
}

/* Make sure we don't act on old event bits from previous runs when
 * we restart autoneg.
 */
static int mtip_unlatch_an_stat(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	int val;

	lockdep_assert_held(&irqpoll->lock);

	val = mtip_read_an(priv, AN_STAT);
	if (val < 0)
		return val;

	/* Discard the current AN status, it will become invalid soon */
	irqpoll->old_an_stat = 0;

	return 0;
}

/* Suppress a "PCS link dropped, restarting autoneg" event when initiating
 * an autoneg restart locally.
 */
static int mtip_unlatch_pcs_stat(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;

	irqpoll->old_pcs_stat = 0;

	return 0;
}

static int mtip_read_adv(struct mtip_backplane *priv, u64 *base_page)
{
	int val;

	val = mtip_read_an(priv, AN_ADV_0);
	if (val < 0)
		return val;

	*base_page = (u64)val;

	val = mtip_read_an(priv, AN_ADV_1);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 16;

	val = mtip_read_an(priv, AN_ADV_2);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 32;

	return 0;
}

static int mtip_read_lpa_xnp(struct mtip_backplane *priv, u64 *base_page)
{
	int val;

	val = mtip_read_an(priv, AN_LPA_XNP_0);
	if (val < 0)
		return val;

	*base_page = (u64)val;

	val = mtip_read_an(priv, AN_LPA_XNP_1);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 16;

	val = mtip_read_an(priv, AN_LPA_XNP_2);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 32;

	return 0;
}

static int mtip_write_adv(struct mtip_backplane *priv, u64 base_page)
{
	int val;

	val = mtip_write_an(priv, AN_ADV_0, C73_ADV_0(base_page));
	if (val < 0)
		return val;

	val = mtip_write_an(priv, AN_ADV_1, C73_ADV_1(base_page));
	if (val < 0)
		return val;

	val = mtip_write_an(priv, AN_ADV_2, C73_ADV_2(base_page));
	if (val < 0)
		return val;

	return 0;
}

static int mtip_read_lpa(struct mtip_backplane *priv, u64 *base_page)
{
	int val;

	val = mtip_read_an(priv, AN_LPA_0);
	if (val < 0)
		return val;

	*base_page = (u64)val;

	val = mtip_read_an(priv, AN_LPA_1);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 16;

	val = mtip_read_an(priv, AN_LPA_2);
	if (val < 0)
		return val;

	*base_page |= (u64)val << 32;

	return 0;
}

static int mtip_config_an_adv(struct mtip_backplane *priv,
			      const unsigned long *advertising)
{
	u64 base_page = linkmode_adv_to_c73_base_page(advertising);
	u64 old_base_page;
	bool changed;
	u8 nonce;
	int err;

	err = mtip_read_adv(priv, &old_base_page);
	if (err < 0)
		return err;

	/* The transmitted nonce must not be equal with the one transmitted by
	 * the link partner, otherwise AN will not complete (nonce_match=true).
	 */
	get_random_bytes(&nonce, sizeof(nonce));

	base_page |= C73_BASE_PAGE_TRANSMITTED_NONCE(nonce);
	/* According to Annex 28A, set Selector to "IEEE 802.3" */
	base_page |= C73_BASE_PAGE_SELECTOR(1);
	/* C73_BASE_PAGE_ACK and C73_BASE_PAGE_ECHOED_NONCE seem to have
	 * a life of their own, regardless of what we set them to.
	 */

	changed = base_page != old_base_page;
	if (!changed)
		return 0;

	dev_dbg(&priv->mdiodev->dev,
		"Advertisement change: %04x:%04x:%04x -> %04x:%04x:%04x\n",
		C73_ADV_2(old_base_page), C73_ADV_1(old_base_page),
		C73_ADV_0(old_base_page), C73_ADV_2(base_page),
		C73_ADV_1(base_page), C73_ADV_0(base_page));

	err = mtip_write_adv(priv, base_page);
	if (err)
		return err;

	return 1; /* restart autoneg */
}

static void mtip_c73_decode_next_page(struct mtip_backplane *priv)
{
	u64 xnp;

	if (mtip_read_lpa_xnp(priv, &xnp))
		return;

	dev_info(&priv->mdiodev->dev,
		 "Next page: message code 0x%llx, toggle %d, U 0x%llx, %s%s%s%s\n",
		 C73_NEXT_PAGE_MESSAGE_CODE(xnp),
		 !!(xnp & C73_NEXT_PAGE_TOGGLE),
		 C73_NEXT_PAGE_U_X(xnp),
		 !!(xnp & C73_NEXT_PAGE_ACK2) ? "ack2 " : "",
		 !!(xnp & C73_NEXT_PAGE_MP) ? "mp " : "",
		 !!(xnp & C73_NEXT_PAGE_ACK) ? "ack " : "",
		 !!(xnp & C73_NEXT_PAGE_NP) ? "np " : "");
}

static int mtip_an_restart(struct mtip_backplane *priv,
			   enum an_restart_reason reason)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	if (!priv->an_enabled)
		return 0;

	dev_dbg(dev, "Restarting auto-negotiation, reason: %s\n",
		mtip_an_restart_reason_str(reason));

	err = mtip_state_reset(priv, STATE_RESET_REASON_AN_RESTART);
	if (err)
		return err;

	/* Make sure AN is temporarily disabled, so that we can safely
	 * unlatch the previous status without losing real events
	 */
	err = mtip_reset_an(priv);
	if (err < 0)
		return err;

	err = mtip_unlatch_an_stat(priv);
	if (err)
		return err;

	err = mtip_unlatch_pcs_stat(priv);
	if (err)
		return err;

	err = mtip_modify_an(priv, AN_CTRL,
			     MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART,
			     MDIO_AN_CTRL1_ENABLE | MDIO_AN_CTRL1_RESTART);
	if (err < 0)
		return err;

	priv->last_an_restart = jiffies;
	priv->an_restart_pending = false;

	return 0;
}

void mtip_backplane_an_restart(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	struct device *dev = &priv->mdiodev->dev;
	int err;

	mutex_lock(&irqpoll->lock);

	if (!priv->an_enabled)
		goto out_unlock;

	err = mtip_an_restart(priv, AN_RESTART_REASON_USER);
	if (err)
		dev_err(dev, "Failed to restart backplane autoneg: %pe\n",
			ERR_PTR(err));

out_unlock:
	mutex_unlock(&irqpoll->lock);
}
EXPORT_SYMBOL(mtip_backplane_an_restart);

/* The reason for deferral is that the irqpoll thread waits for the LT kthreads
 * to finish with irqpoll->lock held, and AN restart also requires holding the
 * irqpoll->lock. So the kthreads cannot directly restart autoneg without
 * deadlocking with the irqpoll thread, they must signal to the irqpoll thread
 * to do so.
 */
static void mtip_an_restart_from_lt(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	struct mtip_backplane *coordinator;

	dev_dbg(dev, "Link training requests autoneg restart\n");

	coordinator = priv->is_subordinate ? priv->coordinator : priv;

	mutex_lock(&coordinator->an_restart_lock);
	coordinator->an_restart_pending = true;
	mutex_unlock(&coordinator->an_restart_lock);
}

static int mtip_wait_for_lt_end(struct mtip_backplane *priv)
{
	int err, val;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || !(val & LT_STAT_STARTUP_PROTOCOL_STATUS),
				MTIP_LT_END_SLEEP_US, MTIP_LT_END_TIMEOUT_US,
				false, priv, LT_STAT);

	return (val < 0) ? val : err;
}

static int mtip_finalize_lt(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	union phy_configure_opts opts = {
		.ethernet = {
			.type = C72_LT_DONE,
		},
	};
	int err, val;

	lockdep_assert_held(&priv->lt_lock);

	if (!priv->local_tx_lt_done || !priv->remote_tx_lt_done)
		return 0;

	priv->last_lt_done = jiffies;

	/* Let the local state machine know we're done */
	err = mtip_modify_lt(priv, LT_STAT, LT_STAT_RX_STATUS,
			     LT_STAT_RX_STATUS);
	if (err < 0) {
		dev_err(dev, "Failed to update LT_STAT: %pe\n", ERR_PTR(err));
		return err;
	}

	/* Give some time for the LP to see our training frames
	 * with "RX ready", before disabling link training.
	 */
	err = mtip_wait_for_lt_end(priv);
	if (err) {
		/* With 25G, this can often be seen to fail, but it seems
		 * inconsequential, so ignore it
		 */
		dev_warn(dev,
			 "Failed to wait for the Start-up Protocol Status bit to clear: %pe, LT_STAT = 0x%x\n",
			 ERR_PTR(err), mtip_read_lt(priv, LT_STAT));
	}

	err = mtip_finish_lt(priv);
	if (err)
		return err;

	/* The SIGNAL_DETECT bit is apparently reserved in the 10GBASE-KR PMD
	 * Status Register for the other AN/LT models
	 */
	if (priv->model == MTIP_MODEL_LX2160A) {
		val = mtip_read_lt(priv, LT_STAT);
		if (val < 0)
			return val;

		if (!(val & LT_STAT_SIGNAL_DETECT)) {
			dev_err(dev, "Link training did not succeed: LT_STAT = 0x%x\n",
				val);
			return -ENETDOWN;
		}
	}

	return phy_configure(priv->serdes, &opts);
}

static int mtip_tx_c72_coef_update(struct mtip_backplane *priv,
				   const struct c72_coef_update *upd,
				   struct c72_coef_status *stat)
{
	char upd_buf[C72_COEF_UPDATE_BUFSIZ], stat_buf[C72_COEF_STATUS_BUFSIZ];
	struct device *dev = &priv->mdiodev->dev;
	int err, val;
	u16 ld_coef;

	c72_coef_update_print(upd, upd_buf);

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_STAT_ALL_NOT_UPDATED(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, priv, LT_LP_STAT);
	if (val < 0)
		return val;
	if (err) {
		dev_err(dev,
			"LP did not set coefficient status to NOT_UPDATED, couldn't send %s; LP_STAT = 0x%x\n",
			upd_buf, val);
		return err;
	}

	ld_coef = LT_COM1(upd->com1) | LT_COZ(upd->coz) | LT_COP1(upd->cop1);
	if (upd->init)
		ld_coef |= LT_COEF_UPD_INIT;
	if (upd->preset)
		ld_coef |= LT_COEF_UPD_PRESET;

	err = mtip_write_lt(priv, LT_LD_COEF, ld_coef);
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_STAT_ANY_UPDATED(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, priv, LT_LP_STAT);
	if (val < 0)
		return val;
	if (err) {
		dev_err(dev,
			"LP did not update coefficient status for %s; LP_STAT = 0x%x\n",
			upd_buf, val);
		return err;
	}

	stat->com1 = LT_COM1_X(val);
	stat->coz = LT_COZ_X(val);
	stat->cop1 = LT_COP1_X(val);
	c72_coef_status_print(stat, stat_buf);

	ld_coef = LT_COM1(COEF_UPD_HOLD) | LT_COZ(COEF_UPD_HOLD) |
		  LT_COP1(COEF_UPD_HOLD);
	err = mtip_write_lt(priv, LT_LD_COEF, ld_coef);
	if (err < 0)
		return err;

	dev_dbg(dev, "Sent update: %s, got status: %s\n", upd_buf, stat_buf);

	return 0;
}

static int mtip_c72_process_request(struct mtip_backplane *priv,
				    const struct c72_coef_update *upd,
				    struct c72_coef_status *stat)
{
	union phy_configure_opts opts = {
		.ethernet = {
			.type = C72_LOCAL_TX,
			.local_tx = {
				.update = *upd,
			},
		},
	};
	int err;

	err = phy_configure(priv->serdes, &opts);
	if (err)
		return err;

	*stat = opts.ethernet.local_tx.status;
	priv->any_request_received = true;

	return 0;
}

static int mtip_read_lt_lp_coef_if_not_ready(struct mtip_backplane *priv,
					     bool *rx_ready)
{
	int val;

	val = mtip_read_lt(priv, LT_LP_STAT);
	if (val < 0)
		return val;

	*rx_ready = !!(val & LT_COEF_STAT_RX_READY);
	if (*rx_ready)
		return 0;

	return mtip_read_lt(priv, LT_LP_COEF);
}

static int mtip_rx_c72_coef_update(struct mtip_backplane *priv,
				   struct c72_coef_update *upd,
				   bool *rx_ready)
{
	char upd_buf[C72_COEF_UPDATE_BUFSIZ], stat_buf[C72_COEF_STATUS_BUFSIZ];
	struct device *dev = &priv->mdiodev->dev;
	struct c72_coef_status stat = {};
	int err, val;

	err = read_poll_timeout(mtip_read_lt_lp_coef_if_not_ready,
				val, val < 0 || *rx_ready || LT_COEF_UPD_ANYTHING(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, priv, rx_ready);
	if (err) {
		dev_err(dev,
			"LP did not request coefficient updates; LP_COEF = 0x%x\n",
			val);
		return err;
	}
	if (val < 0)
		return val;
	if (*rx_ready) {
		if (!priv->any_request_received)
			dev_warn(dev,
				 "LP says its RX is ready, but there was no coefficient request (LP_STAT = 0x%x, LD_STAT = 0x%x)\n",
				 mtip_read_lt(priv, LT_LP_STAT),
				 mtip_read_lt(priv, LT_LD_STAT));
		else
			dev_dbg(dev, "LP says its RX is ready\n");
		return 0;
	}

	upd->com1 = LT_COM1_X(val);
	upd->coz = LT_COZ_X(val);
	upd->cop1 = LT_COP1_X(val);
	upd->init = !!(val & LT_COEF_UPD_INIT);
	upd->preset = !!(val & LT_COEF_UPD_PRESET);
	c72_coef_update_print(upd, upd_buf);

	if ((upd->com1 || upd->coz || upd->cop1) + upd->init + upd->preset > 1) {
		dev_warn(dev, "Ignoring illegal request: %s (LP_COEF = 0x%x)\n",
			 upd_buf, val);
		return 0;
	}

	err = mtip_c72_process_request(priv, upd, &stat);
	if (err)
		return err;

	c72_coef_status_print(&stat, stat_buf);
	dev_dbg(dev, "Received update: %s, responded with status: %s\n",
		upd_buf, stat_buf);

	err = mtip_modify_lt(priv, LT_LD_STAT, LT_COEF_STAT_MASK,
			     LT_COM1(stat.com1) | LT_COZ(stat.coz) |
			     LT_COP1(stat.cop1));
	if (err < 0)
		return err;

	err = read_poll_timeout(mtip_read_lt, val,
				val < 0 || LT_COEF_UPD_NOTHING(val),
				MTIP_COEF_STAT_SLEEP_US, MTIP_COEF_STAT_TIMEOUT_US,
				false, priv, LT_LP_COEF);
	if (err) {
		dev_err(dev, "LP did not revert to HOLD; LP_COEF = 0x%x\n",
			val);
		return err;
	}
	if (val < 0)
		return val;

	err = mtip_modify_lt(priv, LT_LD_STAT, LT_COEF_STAT_MASK,
			     LT_COM1(COEF_STAT_NOT_UPDATED) |
			     LT_COZ(COEF_STAT_NOT_UPDATED) |
			     LT_COP1(COEF_STAT_NOT_UPDATED));
	if (err < 0)
		return err;

	return 0;
}

static int mtip_local_tx_lt_done(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	mutex_lock(&priv->lt_lock);

	priv->local_tx_lt_done = true;

	err = mtip_finalize_lt(priv);
	if (!err)
		dev_dbg(dev, "Link training for local TX done\n");

	mutex_unlock(&priv->lt_lock);

	return err;
}

static int mtip_remote_tx_lt_done(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	mutex_lock(&priv->lt_lock);

	priv->remote_tx_lt_done = true;

	err = mtip_finalize_lt(priv);
	if (!err)
		dev_dbg(dev, "Link training for remote TX done\n");

	mutex_unlock(&priv->lt_lock);

	return err;
}

/* This is our hardware-based 500 ms timer for the link training */
static int mtip_lt_in_progress(struct mtip_backplane *priv)
{
	int val;

	val = mtip_read_lt(priv, LT_STAT);
	if (val < 0)
		return val;

	return !!(val & LT_STAT_TRAINING_FAILURE) ? -ETIMEDOUT : 0;
}

/* Make adjustments to the local TX according to link partner requests,
 * so that its RX improves
 */
static void mtip_local_tx_lt_work(struct kthread_work *work)
{
	struct mtip_lt_work *lt_work = container_of(work, struct mtip_lt_work,
						    work);
	struct mtip_backplane *priv = lt_work->priv;
	struct device *dev = &priv->mdiodev->dev;
	int err = 0;

	while (!READ_ONCE(priv->lt_stop_request)) {
		struct c72_coef_update upd = {};
		bool rx_ready;

		err = mtip_lt_in_progress(priv);
		if (err) {
			dev_err(dev, "Local TX LT failed: %pe\n", ERR_PTR(err));
			break;
		}

		err = mtip_rx_c72_coef_update(priv, &upd, &rx_ready);
		if (err)
			goto out;

		if (rx_ready) {
			err = mtip_local_tx_lt_done(priv);
			if (err) {
				dev_err(dev, "Failed to finalize local LT: %pe\n",
					ERR_PTR(err));
				goto out;
			}
			break;
		}
	}

out:
	if (err && !READ_ONCE(priv->lt_stop_request))
		mtip_an_restart_from_lt(priv);

	kfree(lt_work);
}

/* Train the link partner TX, so that the local RX quality improves */
static void mtip_remote_tx_lt_work(struct kthread_work *work)
{
	struct mtip_lt_work *lt_work = container_of(work, struct mtip_lt_work,
						    work);
	struct mtip_backplane *priv = lt_work->priv;
	struct device *dev = &priv->mdiodev->dev;
	int i = 0, err = 0;

	while (true) {
		struct c72_coef_status status = {};
		union phy_configure_opts opts = {
			.ethernet = {
				.type = C72_REMOTE_TX,
			},
		};

		if (READ_ONCE(priv->lt_stop_request)) {
			dev_dbg(dev, "LT stop request after %d steps\n", i);
			goto out;
		}

		err = mtip_lt_in_progress(priv);
		if (err) {
			dev_err(dev, "Remote TX LT failed: %pe\n", ERR_PTR(err));
			goto out;
		}

		err = phy_configure(priv->serdes, &opts);
		if (err) {
			dev_err(dev,
				"Failed to get remote TX training request from SerDes: %pe\n",
				ERR_PTR(err));
			goto out;
		}

		if (opts.ethernet.remote_tx.rx_ready)
			break;

		err = mtip_tx_c72_coef_update(priv, &opts.ethernet.remote_tx.update,
					      &status);
		if (opts.ethernet.remote_tx.cb)
			opts.ethernet.remote_tx.cb(opts.ethernet.remote_tx.cb_priv,
						   err, opts.ethernet.remote_tx.update,
						   status);
		if (err)
			goto out;

		i++;
	}

	/* Let the link partner know we're done */
	err = mtip_modify_lt(priv, LT_LD_STAT, LT_COEF_STAT_RX_READY,
			     LT_COEF_STAT_RX_READY);
	if (err < 0) {
		dev_err(dev, "Failed to update LT_LD_STAT: %pe\n",
			ERR_PTR(err));
		goto out;
	}

	err = mtip_remote_tx_lt_done(priv);
	if (err) {
		dev_err(dev, "Failed to finalize remote LT: %pe\n",
			ERR_PTR(err));
		goto out;
	}

out:
	if (err && !READ_ONCE(priv->lt_stop_request))
		mtip_an_restart_from_lt(priv);

	kfree(lt_work);
}

static int mtip_start_lt(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = mtip_get_irqpoll(priv);
	struct device *dev = &priv->mdiodev->dev;
	struct mtip_lt_work *remote_tx_lt_work;
	struct mtip_lt_work *local_tx_lt_work;
	int err;

	lockdep_assert_held(&irqpoll->lock);

	local_tx_lt_work = kzalloc(sizeof(*local_tx_lt_work), GFP_KERNEL);
	if (!local_tx_lt_work) {
		err = -ENOMEM;
		goto out;
	}

	remote_tx_lt_work = kzalloc(sizeof(*remote_tx_lt_work), GFP_KERNEL);
	if (!remote_tx_lt_work) {
		err = -ENOMEM;
		goto out_free_local_tx_lt;
	}

	err = mtip_reset_lt(priv);
	if (err)
		goto out_free_remote_tx_lt;

	err = mtip_restart_lt(priv, true);
	if (err)
		goto out_free_remote_tx_lt;

	priv->lt_enabled = true;

	dev_dbg(dev, "Link training enabled\n");

	err = mtip_lt_frame_lock(priv);
	if (err) {
		dev_err(dev,
			"Failed to acquire training frame delineation: %pe\n",
			ERR_PTR(err));
		goto out_stop_lt;
	}

	local_tx_lt_work->priv = priv;
	kthread_init_work(&local_tx_lt_work->work, mtip_local_tx_lt_work);
	kthread_queue_work(priv->local_tx_lt_worker, &local_tx_lt_work->work);

	remote_tx_lt_work->priv = priv;
	kthread_init_work(&remote_tx_lt_work->work, mtip_remote_tx_lt_work);
	kthread_queue_work(priv->remote_tx_lt_worker, &remote_tx_lt_work->work);

	return 0;

out_stop_lt:
	mtip_finish_lt(priv);
out_free_remote_tx_lt:
	kfree(remote_tx_lt_work);
out_free_local_tx_lt:
	kfree(local_tx_lt_work);
out:
	dev_err(dev, "Failed to start link training: %pe\n", ERR_PTR(err));
	return err;
}

/* Allow the datapath to come up without link training */
static int mtip_bypass_lt(struct mtip_backplane *priv)
{
	struct device *dev = &priv->mdiodev->dev;
	int err;

	err = mtip_modify_lt(priv, LT_STAT, LT_STAT_RX_STATUS,
			     LT_STAT_RX_STATUS);
	if (err < 0) {
		dev_err(dev, "Failed to bypass link training: %pe\n",
			ERR_PTR(err));
		return err;
	}

	return 0;
}

static void mtip_update_link_latch(struct mtip_backplane *priv,
				   bool cdr_locked, bool phy_los,
				   bool an_complete, bool pcs_lstat)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	struct device *dev = &priv->mdiodev->dev;

	lockdep_assert_held(&irqpoll->lock);

	/* Update irqpoll->link if true, or if false
	 * and mtip_read_status() saw that already.
	 */
	if (irqpoll->link || irqpoll->link_ack) {
		irqpoll->link = phy_los && cdr_locked && an_complete && pcs_lstat;
		irqpoll->link_ack = false;
	}

	dev_dbg(dev, "PCS link%s: %d, PHY LOS: %d, CDR locked: %d, AN complete: %d\n",
		priv->link_mode_resolved ? "" : " (ignored)",
		pcs_lstat, phy_los, cdr_locked, an_complete);
}

static bool mtip_cached_an_complete(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;

	lockdep_assert_held(&irqpoll->lock);

	return !!(irqpoll->old_an_stat & MDIO_AN_STAT1_COMPLETE);
}

static bool mtip_read_link_unlatch(struct mtip_backplane *priv)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	bool old_link = irqpoll->link;

	lockdep_assert_held(&irqpoll->lock);

	/* A change to the link status may have occurred while a link
	 * loss was latched, so update it again after reading it
	 */
	irqpoll->link = !!(irqpoll->old_an_stat & MDIO_STAT1_LSTATUS) &&
			!!(irqpoll->old_an_stat & MDIO_AN_STAT1_COMPLETE) &&
			!!(irqpoll->old_pcs_stat & MDIO_STAT1_LSTATUS) &&
			irqpoll->cdr_locked;
	irqpoll->link_ack = true;

	return old_link;
}

static struct mdio_device *
mtip_get_mdiodev_for_link_mode(struct mii_bus *bus, struct phy *serdes,
			       enum ethtool_link_mode_bit_indices link_mode)
{
	union phy_status_opts opts = {
		.pcvt = {
			.type = PHY_PCVT_ETHERNET_ANLT,
		},
	};
	struct mdio_device *mdiodev;
	int err;

	err = phy_set_mode_ext(serdes, PHY_MODE_ETHERNET_LINKMODE, link_mode);
	if (err)
		return ERR_PTR(err);

	err = phy_get_status(serdes, PHY_STATUS_PCVT_ADDR, &opts);
	if (err)
		return ERR_PTR(err);

	mdiodev = mdio_device_create(bus, opts.pcvt.addr.mdio);
	if (!mdiodev)
		return ERR_PTR(-ENODEV);

	return mdiodev;
}

/* TODO: untested code path, see if we need to apply the configuration of the
 * old AN/LT block to the new one
 */
static int mtip_reconfigure(struct mtip_backplane *priv,
			    enum ethtool_link_mode_bit_indices resolved)
{
	struct device *dev = &priv->mdiodev->dev;
	struct mdio_device *mdiodev;

	dev_info(dev, "Resolved link mode %s but configured for %s, reconfiguring...\n",
		 ethtool_link_mode_str(resolved),
		 ethtool_link_mode_str(priv->cfg_link_mode));

	mdiodev = mtip_get_mdiodev_for_link_mode(priv->bus, priv->serdes,
						 resolved);
	if (IS_ERR(mdiodev))
		return PTR_ERR(mdiodev);

	mdio_device_put(priv->mdiodev);
	priv->mdiodev = mdiodev;
	priv->cfg_link_mode = resolved;

	return 0;
}

static int mtip_apply_serdes_protocol(struct mtip_backplane *priv, void *args)
{
	const enum ethtool_link_mode_bit_indices *resolved = args;
	struct device *dev = &priv->mdiodev->dev;
	int err;

	err = phy_set_mode_ext(priv->serdes, PHY_MODE_ETHERNET_LINKMODE,
			       *resolved);
	if (err) {
		dev_err(dev, "phy_set_mode_ext(%s) returned %pe\n",
			ethtool_link_mode_str(*resolved), ERR_PTR(err));
		return err;
	}

	return 0;
}

static int mtip_c73_page_received(struct mtip_backplane *priv,
				  enum an_restart_reason *an_restart_reason)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(lp_advertising) = {};
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising) = {};
	enum ethtool_link_mode_bit_indices resolved;
	struct device *dev = &priv->mdiodev->dev;
	__ETHTOOL_DECLARE_LINK_MODE_MASK(common);
	u64 base_page, lpa;
	int i, err;

	err = mtip_state_reset(priv, STATE_RESET_REASON_PAGE_RECEIVED);
	if (err)
		return err;

	err = mtip_read_adv(priv, &base_page);
	if (err < 0)
		return err;

	err = mtip_read_lpa(priv, &lpa);
	if (err < 0)
		return err;

	if (lpa & C73_BASE_PAGE_NP)
		mtip_c73_decode_next_page(priv);

	mii_c73_mod_linkmode_lpa_t(advertising, base_page);
	mii_c73_mod_linkmode_lpa_t(lp_advertising, lpa);
	linkmode_and(common, advertising, lp_advertising);

	err = linkmode_c73_priority_resolution(common, &resolved);
	if (err) {
		*an_restart_reason = AN_RESTART_REASON_NO_HCD;
		return 0;
	}

	dev_dbg(dev,
		"C73 page received, LD %04x:%04x:%04x, LP %04x:%04x:%04x, resolved link mode %s\n",
		C73_ADV_2(base_page), C73_ADV_1(base_page), C73_ADV_0(base_page),
		C73_ADV_2(lpa), C73_ADV_1(lpa), C73_ADV_0(lpa),
		ethtool_link_mode_str(resolved));

	err = for_each_lane_args(mtip_apply_serdes_protocol, priv, &resolved);
	if (err)
		return err;

	if (resolved != priv->cfg_link_mode) {
		*an_restart_reason = AN_RESTART_REASON_RECONFIG;
		return mtip_reconfigure(priv, resolved);
	}

	for (i = 0; i < priv->num_subordinates; i++) {
		if (!priv->subordinate[i]->lane_powered_on) {
			err = mtip_backplane_resume(priv->subordinate[i]);
			if (err)
				return err;
		}
	}

	err = mtip_wait_for_cdr_lock(priv);
	if (err) {
		dev_warn(dev, "Failed to reacquire CDR lock after protocol change: %pe\n",
			 ERR_PTR(err));
		*an_restart_reason = AN_RESTART_REASON_CDR;
		return 0;
	}

	if (link_mode_needs_training(resolved)) {
		err = for_each_lane(mtip_start_lt, priv);
		if (err) { // FIXME return err
			/* mtip_an_restart() -> mtip_state_reset()
			 * will clean up and stop the kthreads for the lanes
			 * where link training may have already started
			 */
			*an_restart_reason = AN_RESTART_REASON_FRAME_LOCK;
			return 0;
		}
	} else {
		err = for_each_lane(mtip_bypass_lt, priv);
		if (err)
			return err;
	}

	priv->link_mode = resolved;
	priv->link_mode_resolved = true;

	return 0;
}

static void mtip_c73_remote_fault(struct mtip_backplane *priv, bool fault)
{
	struct device *dev = &priv->mdiodev->dev;

	dev_err(dev, "Remote fault: %d\n", fault);
}

static bool mtip_are_all_lanes_trained(struct mtip_backplane *priv)
{
	int i;

	if (!priv->local_tx_lt_done || !priv->remote_tx_lt_done)
		return false;

	for (i = 0; i < priv->num_subordinates; i++) {
		if (!priv->subordinate[i]->local_tx_lt_done ||
		    !priv->subordinate[i]->remote_tx_lt_done)
			return false;
	}

	return true;
}

static void mtip_irqpoll_work(struct work_struct *work)
{
	struct mtip_irqpoll *irqpoll = container_of(work, struct mtip_irqpoll, work.work);
	struct mtip_backplane *priv = container_of(irqpoll, struct mtip_backplane, irqpoll);
	enum an_restart_reason an_restart_reason = AN_RESTART_REASON_NONE;
	struct device *dev = &priv->mdiodev->dev;
	bool all_lanes_have_cdr_lock;
	bool new_page = false;
	int val, err = 0;
	int pcs_stat = 0;

	/* Check for AN restart requests from the link training kthreads */
	mutex_lock(&priv->an_restart_lock);
	if (priv->an_restart_pending) {
		an_restart_reason = AN_RESTART_REASON_LT;
		priv->an_restart_pending = false;
	}
	mutex_unlock(&priv->an_restart_lock);

	/* Then enter the irqpoll logic per se
	 * (PCS MDIO_STAT1, AN/LT MDIO_STAT1 and CDR lock)
	 */
	mutex_lock(&irqpoll->lock);

	err = mtip_check_cdr_lock(priv, &all_lanes_have_cdr_lock);
	if (err)
		goto out_unlock;

	if (priv->link_mode_resolved) {
		pcs_stat = mtip_read_pcs(priv, MDIO_STAT1);
		if (pcs_stat < 0) {
			err = pcs_stat;
			goto out_unlock;
		}
	}

	val = mtip_read_an(priv, AN_STAT);
	if (val < 0) {
		err = val;
		goto out_unlock;
	}

	if ((irqpoll->cdr_locked != all_lanes_have_cdr_lock) ||
	    ((irqpoll->old_an_stat ^ val) & (MDIO_STAT1_LSTATUS |
					     MDIO_AN_STAT1_COMPLETE)) ||
	    ((irqpoll->old_pcs_stat ^ pcs_stat) & MDIO_STAT1_LSTATUS)) {
		mtip_update_link_latch(priv, all_lanes_have_cdr_lock,
				       !!(val & MDIO_STAT1_LSTATUS),
				       !!(val & MDIO_AN_STAT1_COMPLETE),
				       !!(pcs_stat & MDIO_STAT1_LSTATUS));
	}

	if ((val & MDIO_AN_STAT1_PAGE) && !an_restart_reason) {
		/* FIXME: I don't know how to make the AN/LT block stop from
		 * sending base pages during link training and even after
		 * training has finished, and the reception of those will
		 * trigger a new resolve and link training process, which will
		 * make the link never come up. A fragile hack which works
		 * most of the time is to only act upon bit transitions, to
		 * avoid processing the "page received" events during link
		 * training.
		 */
		if (!((irqpoll->old_an_stat ^ val) & MDIO_AN_STAT1_PAGE)) {
			dev_dbg(dev, "Skipping the processing of one C73 base page\n");
			goto skip_c73_page;
		}

		/* When we had a link and the LP retriggers autoneg, we first
		 * detect a new base page and resolve the link mode properly,
		 * then we see that the PCS link dropped and we retrigger
		 * autoneg again. Avoid that.
		 */
		new_page = true;

		err = mtip_c73_page_received(priv, &an_restart_reason);
		if (err)
			goto out_unlock;
	}

skip_c73_page:
	if (val & MDIO_AN_STAT1_RFAULT)
		mtip_c73_remote_fault(priv, val & MDIO_AN_STAT1_RFAULT);

	/* Checks that result in AN restart should go at the end */

	/* Make sure the lane goes back into DME page exchange mode
	 * after a link drop
	 */
	if (priv->link_mode_resolved && !new_page &&
	    (irqpoll->old_pcs_stat & MDIO_STAT1_LSTATUS) &&
	    !(pcs_stat & MDIO_STAT1_LSTATUS))
		an_restart_reason = AN_RESTART_REASON_PCS_LINK_DROP;

	/* Paranoid workaround for undetermined issue */
	if (!priv->link_mode_resolved && (val & MDIO_AN_STAT1_COMPLETE) &&
	    priv->an_enabled && time_after(jiffies, priv->last_an_restart +
					   msecs_to_jiffies(MTIP_AN_TIMEOUT_MS))) {
		dev_err(dev,
			"Hardware says AN has completed, but we never saw a base page, and that's bogus\n");
		an_restart_reason = AN_RESTART_REASON_BOGUS_AN_WA;
	}

	/* Sometimes, after a renegotiation, it can be seen that link training
	 * failures on one side trigger an autoneg restart (as they should),
	 * but that does not get acted upon by the other side. It appears that
	 * the other side is in a strange state where it has completed link
	 * training and it's waiting for something. As seen in the
	 * MDIO_AN_STAT1_PAGE workaround above, we will fail to detect new base
	 * pages received during link training, so we won't be able to exit
	 * that state. Detect it and exit it if 1 second has passed since link
	 * training has completed, but the 'autoneg done' bit hasn't asserted.
	 */
	if (mtip_are_all_lanes_trained(priv) && !(val & MDIO_AN_STAT1_COMPLETE) &&
	    time_after(jiffies, priv->last_lt_done +
		       msecs_to_jiffies(MTIP_LT_TIMEOUT_MS))) {
		dev_err(dev, "AN did not complete after link training completed\n");
		an_restart_reason = AN_RESTART_REASON_TIMEOUT_AN;
	}

	if (an_restart_reason) {
		err = mtip_an_restart(priv, an_restart_reason);
		if (err)
			goto out_unlock;

		/* don't overwrite what was set by mtip_unlatch_an_stat() */
		goto ignore_an_and_pcs_stat;
	}

	irqpoll->old_an_stat = val;
	irqpoll->old_pcs_stat = pcs_stat;
ignore_an_and_pcs_stat:
	irqpoll->cdr_locked = all_lanes_have_cdr_lock;

out_unlock:
	mutex_unlock(&irqpoll->lock);

	if (err) {
		dev_err(dev,
			"Error detected from irqpoll thread: %pe, exiting...\n",
			ERR_PTR(err));
		return;
	}

	if (!irqpoll->run_once)
		schedule_delayed_work(&irqpoll->work, IRQPOLL_INTERVAL);

	irqpoll->run_once = false;
}

int mtip_backplane_config_aneg(struct mtip_backplane *priv, bool autoneg,
			       const unsigned long *advertising)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	int err;

	if (!autoneg) {
		dev_err(&priv->mdiodev->dev,
			"Disabling C73 autoneg not supported\n");
		return -EOPNOTSUPP;
	}

	mutex_lock(&irqpoll->lock);

	err = mtip_config_an_adv(priv, advertising);
	if (err < 0)
		goto out_unlock;

	priv->an_enabled = true;

out_unlock:
	mutex_unlock(&irqpoll->lock);

	return err;
}
EXPORT_SYMBOL(mtip_backplane_config_aneg);

static int mtip_resolve_aneg_linkmode(struct mtip_backplane *priv,
				      struct phylink_link_state *state)
{
	u64 base_page;
	int err;

	linkmode_zero(state->lp_advertising);

	err = mtip_read_lpa(priv, &base_page);
	if (err)
		return err;

	mii_c73_mod_linkmode_lpa_t(state->lp_advertising, base_page);
	phylink_resolve_c73(state);

	return 0;
}

void mtip_backplane_get_state(struct mtip_backplane *priv,
			      struct phylink_link_state *state)
{
	struct mtip_irqpoll *irqpoll = &priv->irqpoll;
	struct device *dev = &priv->mdiodev->dev;
	u64 base_page;
	int err;

	mutex_lock(&irqpoll->lock);

	state->speed = SPEED_UNKNOWN;
	state->duplex = DUPLEX_UNKNOWN;
	state->pause = 0;

	err = mtip_read_adv(priv, &base_page);
	if (err)
		goto out_unlock;

	state->link = mtip_read_link_unlatch(priv);
	if (!state->link)
		goto out_unlock;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			      state->advertising)) {
		state->an_complete = mtip_cached_an_complete(priv);

		if (state->an_complete)
			err = mtip_resolve_aneg_linkmode(priv, state);

		state->link = state->link && mtip_are_all_lanes_trained(priv);
	}

out_unlock:
	mutex_unlock(&irqpoll->lock);

	if (err)
		dev_err(dev, "Failed to get backplane state: %pe\n",
			ERR_PTR(err));
}
EXPORT_SYMBOL(mtip_backplane_get_state);

void mtip_backplane_add_subordinate(struct mtip_backplane *priv,
				    struct mtip_backplane *subordinate)
{
	subordinate->is_subordinate = true;
	subordinate->coordinator = priv;
	priv->subordinate[priv->num_subordinates++] = subordinate;
}
EXPORT_SYMBOL(mtip_backplane_add_subordinate);

static struct mdio_device *
mtip_get_mdiodev(struct mii_bus *bus, struct phy *serdes,
		 enum ethtool_link_mode_bit_indices *cfg_link_mode)
{
	const enum ethtool_link_mode_bit_indices *link_modes;
	int i, err;

	link_modes = mtip_backplane_link_modes;

	/* Preconfigure the SerDes lane for the highest supported link mode,
	 * make sure the backplane AN/LT + PCS are enabled, and get the MDIO
	 * address of our device so that we can access its registers.
	 */
	for (i = 0; i < ARRAY_SIZE(mtip_backplane_link_modes); i++) {
		err = phy_validate(serdes, PHY_MODE_ETHERNET_LINKMODE,
				   link_modes[i], NULL);
		if (err)
			continue;

		*cfg_link_mode = link_modes[i];

		return mtip_get_mdiodev_for_link_mode(bus, serdes,
						      link_modes[i]);
	}

	dev_err(&serdes->dev, "No backplane link modes supported!\n");

	return ERR_PTR(-ENODEV);
}

static void mtip_irqpoll_init(struct mtip_backplane *priv,
			      struct mtip_irqpoll *irqpoll)
{
	mutex_init(&irqpoll->lock);
	INIT_DELAYED_WORK(&irqpoll->work, mtip_irqpoll_work);
}

static int mtip_detect(struct mtip_backplane *priv)
{
	int val, rev, phy_id;

	if (priv->model != MTIP_MODEL_AUTODETECT)
		return 0;

	val = mtip_read_vendor(priv, VEND_REVISION);
	if (val < 0)
		return val;

	rev = val;

	val = mtip_read_an(priv, AN_DEVID1);
	if (val < 0)
		return val;

	phy_id = val << 16;

	val = mtip_read_an(priv, AN_DEVID2);
	if (val < 0)
		return val;

	phy_id |= val;

	dev_info(&priv->mdiodev->dev,
		 "MTIP Backplane AN/LT: IP version %lu.%lu, configuration version %lu, PHY ID %04x:%04x, preconfigured for %s\n",
		 VEND_REVISION_IP_VER_X(rev), VEND_REVISION_IP_REV_X(rev),
		 VEND_REVISION_CFG_VER_X(rev),
		 phy_id >> 16, (u16)(phy_id & GENMASK(15, 0)),
		 ethtool_link_mode_str(priv->cfg_link_mode));

	return 0;
}

struct mtip_backplane *mtip_backplane_create(struct mdio_device *pcs_mdiodev,
					     struct phy *serdes,
					     enum mtip_model model)
{
	enum ethtool_link_mode_bit_indices cfg_link_mode;
	struct mii_bus *bus = pcs_mdiodev->bus;
	struct mtip_backplane *priv;
	struct mdio_device *mdiodev;
	int err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto out;
	}

	mdiodev = mtip_get_mdiodev(bus, serdes, &cfg_link_mode);
	if (IS_ERR(mdiodev)) {
		err = PTR_ERR(mdiodev);
		goto out_free_priv;
	}

	priv->pcs_mdiodev = pcs_mdiodev;
	priv->mdiodev = mdiodev;
	priv->serdes = serdes;
	priv->model = model;
	priv->bus = bus;
	priv->cfg_link_mode = cfg_link_mode;

	switch (model) {
	case MTIP_MODEL_LX2160A:
		priv->an_regs = mtip_lx2160a_an_regs;
		priv->lt_regs = mtip_lx2160a_lt_regs;
		priv->lt_mmd = MDIO_MMD_AN;
		break;
	default:
		priv->an_regs = mtip_an_regs;
		priv->lt_regs = mtip_lt_regs;
		priv->lt_mmd = MDIO_MMD_PMAPMD;
	}

	err = mtip_detect(priv);
	if (err)
		goto out_free_mdiodev;

	err = mtip_reset_pcs(priv);
	if (err < 0)
		goto out_free_mdiodev;

	mtip_irqpoll_init(priv, &priv->irqpoll);
	mutex_init(&priv->an_restart_lock);
	mutex_init(&priv->lt_lock);

	priv->local_tx_lt_worker = kthread_create_worker(0, "%d_local_tx_lt",
							 mdiodev->addr);
	if (IS_ERR(priv->local_tx_lt_worker)) {
		err = PTR_ERR(priv->local_tx_lt_worker);
		goto out_free_priv;
	}

	priv->remote_tx_lt_worker = kthread_create_worker(0, "%d_remote_tx_lt",
							  mdiodev->addr);
	if (IS_ERR(priv->remote_tx_lt_worker)) {
		err = PTR_ERR(priv->remote_tx_lt_worker);
		goto out_destroy_local_tx_lt;
	}

	return priv;

out_destroy_local_tx_lt:
	kthread_destroy_worker(priv->local_tx_lt_worker);
out_free_mdiodev:
	mdio_device_put(priv->mdiodev);
out_free_priv:
	kfree(priv);
out:
	return ERR_PTR(err);
}
EXPORT_SYMBOL(mtip_backplane_create);

void mtip_backplane_destroy(struct mtip_backplane *priv)
{
	kthread_destroy_worker(priv->remote_tx_lt_worker);
	kthread_destroy_worker(priv->local_tx_lt_worker);
	mdio_device_put(priv->mdiodev);
	kfree(priv);
}
EXPORT_SYMBOL(mtip_backplane_destroy);

MODULE_AUTHOR("Vladimir Oltean <vladimir.oltean@nxp.com>");
MODULE_DESCRIPTION("MTIP Backplane PHY driver");
MODULE_LICENSE("GPL");
