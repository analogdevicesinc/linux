// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2020 NXP
 * Lynx PCS MDIO helpers
 */

#include <linux/mdio.h>
#include <linux/phy/phy.h>
#include <linux/phylink.h>
#include <linux/pcs-lynx.h>
#include <linux/property.h>

#include "mtip_backplane.h"

#define PRIMARY_LANE			0
#define MAX_NUM_LANES			4

#define SGMII_CLOCK_PERIOD_NS		8 /* PCS is clocked at 125 MHz */
#define LINK_TIMER_VAL(ns)		((u32)((ns) / SGMII_CLOCK_PERIOD_NS))

#define LINK_TIMER_LO			0x12
#define LINK_TIMER_HI			0x13
#define IF_MODE				0x14
#define IF_MODE_SGMII_EN		BIT(0)
#define IF_MODE_USE_SGMII_AN		BIT(1)
#define IF_MODE_SPEED(x)		(((x) << 2) & GENMASK(3, 2))
#define IF_MODE_SPEED_MSK		GENMASK(3, 2)
#define IF_MODE_HALF_DUPLEX		BIT(4)

struct lynx_pcs {
	struct phylink_pcs pcs;
	struct mdio_device *mdio;
	size_t num_phys;
	struct phy *serdes[MAX_NUM_LANES];
	struct mtip_backplane *anlt[MAX_NUM_LANES];
	enum mtip_model model;
};

enum sgmii_speed {
	SGMII_SPEED_10		= 0,
	SGMII_SPEED_100		= 1,
	SGMII_SPEED_1000	= 2,
	SGMII_SPEED_2500	= 2,
};

#define phylink_pcs_to_lynx(pl_pcs) container_of((pl_pcs), struct lynx_pcs, pcs)
#define lynx_to_phylink_pcs(lynx) (&(lynx)->pcs)

static void lynx_pcs_get_state_usxgmii(struct mdio_device *pcs,
				       struct phylink_link_state *state)
{
	struct mii_bus *bus = pcs->bus;
	int addr = pcs->addr;
	int status, lpa;

	status = mdiobus_c45_read(bus, addr, MDIO_MMD_VEND2, MII_BMSR);
	if (status < 0)
		return;

	state->link = !!(status & MDIO_STAT1_LSTATUS);
	state->an_complete = !!(status & MDIO_AN_STAT1_COMPLETE);
	if (!state->link || !state->an_complete)
		return;

	lpa = mdiobus_c45_read(bus, addr, MDIO_MMD_VEND2, MII_LPA);
	if (lpa < 0)
		return;

	phylink_decode_usxgmii_word(state, lpa);
}

static void lynx_pcs_get_state_2500basex(struct mdio_device *pcs,
					 struct phylink_link_state *state)
{
	int bmsr;

	bmsr = mdiodev_read(pcs, MII_BMSR);
	if (bmsr < 0) {
		state->link = false;
		return;
	}

	state->link = !!(bmsr & BMSR_LSTATUS);
	state->an_complete = !!(bmsr & BMSR_ANEGCOMPLETE);
	if (!state->link)
		return;

	state->speed = SPEED_2500;
	state->pause |= MLO_PAUSE_TX | MLO_PAUSE_RX;
	state->duplex = DUPLEX_FULL;
}

static void lynx_pcs_get_state(struct phylink_pcs *pcs,
			       struct phylink_link_state *state)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);

	if (phylink_autoneg_c73(pcs->cfg_link_an_mode))
		return mtip_backplane_get_state(lynx->anlt[PRIMARY_LANE], state);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		phylink_mii_c22_pcs_get_state(lynx->mdio, state);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		lynx_pcs_get_state_2500basex(lynx->mdio, state);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		lynx_pcs_get_state_usxgmii(lynx->mdio, state);
		break;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_25GBASER:
		phylink_mii_c45_pcs_get_state(lynx->mdio, state);
		break;
	default:
		break;
	}

	dev_dbg(&lynx->mdio->dev,
		"mode=%s/%s/%s link=%u an_complete=%u\n",
		phy_modes(state->interface),
		phy_speed_to_str(state->speed),
		phy_duplex_to_str(state->duplex),
		state->link, state->an_complete);
}

static int lynx_pcs_config_giga(struct mdio_device *pcs,
				phy_interface_t interface,
				const unsigned long *advertising,
				unsigned int neg_mode)
{
	int link_timer_ns;
	u32 link_timer;
	u16 if_mode;
	int err;

	link_timer_ns = phylink_get_link_timer_ns(interface);
	if (link_timer_ns > 0) {
		link_timer = LINK_TIMER_VAL(link_timer_ns);

		mdiodev_write(pcs, LINK_TIMER_LO, link_timer & 0xffff);
		mdiodev_write(pcs, LINK_TIMER_HI, link_timer >> 16);
	}

	if (interface == PHY_INTERFACE_MODE_1000BASEX) {
		if_mode = 0;
	} else {
		/* SGMII and QSGMII */
		if_mode = IF_MODE_SGMII_EN;
		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
			if_mode |= IF_MODE_USE_SGMII_AN;
	}

	err = mdiodev_modify(pcs, IF_MODE,
			     IF_MODE_SGMII_EN | IF_MODE_USE_SGMII_AN,
			     if_mode);
	if (err)
		return err;

	return phylink_mii_c22_pcs_config(pcs, interface, advertising,
					  neg_mode);
}

static int lynx_pcs_config_usxgmii(struct mdio_device *pcs,
				   phy_interface_t interface,
				   const unsigned long *advertising,
				   unsigned int neg_mode)
{
	struct mii_bus *bus = pcs->bus;
	int addr = pcs->addr;

	if (neg_mode != PHYLINK_PCS_NEG_INBAND_ENABLED) {
		dev_err(&pcs->dev, "%s only supports in-band AN for now\n",
			phy_modes(interface));
		return -EOPNOTSUPP;
	}

	/* Configure device ability for the USXGMII Replicator */
	return mdiobus_c45_write(bus, addr, MDIO_MMD_VEND2, MII_ADVERTISE,
				 MDIO_USXGMII_10G | MDIO_USXGMII_LINK |
				 MDIO_USXGMII_FULL_DUPLEX |
				 ADVERTISE_SGMII | ADVERTISE_LPACK);
}

static int lynx_pcs_config_c73(struct phylink_pcs *pcs, unsigned int neg_mode,
			       const unsigned long *advertising)
{
	bool autoneg = !!(neg_mode & PHYLINK_PCS_NEG_ENABLED);
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);

	if (!lynx->num_phys) {
		dev_err(&lynx->mdio->dev, "C73 autoneg requires SerDes\n");
		return -ENODEV;
	}

	return mtip_backplane_config_aneg(lynx->anlt[PRIMARY_LANE], autoneg,
					  advertising);
}

static int lynx_pcs_config(struct phylink_pcs *pcs, unsigned int neg_mode,
			   phy_interface_t ifmode,
			   const unsigned long *advertising, bool permit)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;
	int err;

	if (neg_mode & PHYLINK_PCS_NEG_C73)
		return lynx_pcs_config_c73(pcs, neg_mode, advertising);

	for (i = 0; i < lynx->num_phys; i++) {
		err = phy_set_mode_ext(lynx->serdes[i], PHY_MODE_ETHERNET,
				       ifmode);
		if (err) {
			dev_err(&lynx->mdio->dev,
				"phy_set_mode_ext() failed: %pe\n",
				ERR_PTR(err));
			return err;
		}
	}

	switch (ifmode) {
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		return lynx_pcs_config_giga(lynx->mdio, ifmode, advertising,
					    neg_mode);
	case PHY_INTERFACE_MODE_2500BASEX:
		if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED) {
			dev_err(&lynx->mdio->dev,
				"AN not supported on 3.125GHz SerDes lane\n");
			return -EOPNOTSUPP;
		}
		break;
	case PHY_INTERFACE_MODE_USXGMII:
	case PHY_INTERFACE_MODE_10G_QXGMII:
		return lynx_pcs_config_usxgmii(lynx->mdio, ifmode, advertising,
					       neg_mode);
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_25GBASER:
		/* Nothing to do here for 10GBASER and 25GBASER */
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static void lynx_pcs_an_restart(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);

	if (phylink_autoneg_c73(pcs->cfg_link_an_mode))
		return mtip_backplane_an_restart(lynx->anlt[PRIMARY_LANE]);

	phylink_mii_c22_pcs_an_restart(lynx->mdio);
}

static void lynx_pcs_link_up_sgmii(struct mdio_device *pcs,
				   unsigned int neg_mode,
				   int speed, int duplex)
{
	u16 if_mode = 0, sgmii_speed;

	/* The PCS needs to be configured manually only
	 * when not operating on in-band mode
	 */
	if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED)
		return;

	if (duplex == DUPLEX_HALF)
		if_mode |= IF_MODE_HALF_DUPLEX;

	switch (speed) {
	case SPEED_1000:
		sgmii_speed = SGMII_SPEED_1000;
		break;
	case SPEED_100:
		sgmii_speed = SGMII_SPEED_100;
		break;
	case SPEED_10:
		sgmii_speed = SGMII_SPEED_10;
		break;
	case SPEED_UNKNOWN:
		/* Silently don't do anything */
		return;
	default:
		dev_err(&pcs->dev, "Invalid PCS speed %d\n", speed);
		return;
	}
	if_mode |= IF_MODE_SPEED(sgmii_speed);

	mdiodev_modify(pcs, IF_MODE,
		       IF_MODE_HALF_DUPLEX | IF_MODE_SPEED_MSK,
		       if_mode);
}

/* 2500Base-X is SerDes protocol 7 on Felix and 6 on ENETC. It is a SerDes lane
 * clocked at 3.125 GHz which encodes symbols with 8b/10b and does not have
 * auto-negotiation of any link parameters. Electrically it is compatible with
 * a single lane of XAUI.
 * The hardware reference manual wants to call this mode SGMII, but it isn't
 * really, since the fundamental features of SGMII:
 * - Downgrading the link speed by duplicating symbols
 * - Auto-negotiation
 * are not there.
 * The speed is configured at 1000 in the IF_MODE because the clock frequency
 * is actually given by a PLL configured in the Reset Configuration Word (RCW).
 * Since there is no difference between fixed speed SGMII w/o AN and 802.3z w/o
 * AN, we call this PHY interface type 2500Base-X. In case a PHY negotiates a
 * lower link speed on line side, the system-side interface remains fixed at
 * 2500 Mbps and we do rate adaptation through pause frames.
 */
static void lynx_pcs_link_up_2500basex(struct mdio_device *pcs,
				       unsigned int neg_mode,
				       int speed, int duplex)
{
	u16 if_mode = 0;

	if (neg_mode == PHYLINK_PCS_NEG_INBAND_ENABLED) {
		dev_err(&pcs->dev, "AN not supported for 2500BaseX\n");
		return;
	}

	if (duplex == DUPLEX_HALF)
		if_mode |= IF_MODE_HALF_DUPLEX;
	if_mode |= IF_MODE_SPEED(SGMII_SPEED_2500);

	mdiodev_modify(pcs, IF_MODE,
		       IF_MODE_HALF_DUPLEX | IF_MODE_SPEED_MSK,
		       if_mode);
}

static void lynx_pcs_link_up(struct phylink_pcs *pcs, unsigned int neg_mode,
			     phy_interface_t interface,
			     int speed, int duplex)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);

	switch (interface) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_QSGMII:
		lynx_pcs_link_up_sgmii(lynx->mdio, neg_mode, speed, duplex);
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		lynx_pcs_link_up_2500basex(lynx->mdio, neg_mode, speed, duplex);
		break;
	case PHY_INTERFACE_MODE_USXGMII:
		/* At the moment, only in-band AN is supported for USXGMII
		 * so nothing to do in link_up
		 */
		break;
	default:
		break;
	}
}

static int lynx_pcs_validate(struct phylink_pcs *pcs, unsigned long *supported,
			     const struct phylink_link_state *state)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);

	if (!phylink_autoneg_c73(pcs->cfg_link_an_mode))
		return 0;

	if (!lynx->num_phys) {
		linkmode_zero(supported);
		dev_err(&lynx->mdio->dev, "C73 autoneg requires SerDes\n");
		return -EINVAL;
	}

	return mtip_backplane_validate(lynx->serdes[PRIMARY_LANE], supported);
}

static int lynx_pcs_c73_init(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;
	int err;

	if (!lynx->num_phys) {
		dev_err(&lynx->mdio->dev, "C73 autoneg requires SerDes\n");
		return -ENODEV;
	}

	for (i = 0; i < lynx->num_phys; i++) {
		lynx->anlt[i] = mtip_backplane_create(lynx->mdio,
						      lynx->serdes[i],
						      lynx->model);
		if (IS_ERR(lynx->anlt[i])) {
			err = PTR_ERR(lynx->anlt[i]);
			while (i--)
				mtip_backplane_destroy(lynx->anlt[i]);
			return err;
		}
	}

	for (i = 1; i < lynx->num_phys; i++)
		mtip_backplane_add_subordinate(lynx->anlt[PRIMARY_LANE],
					       lynx->anlt[i]);

	return mtip_backplane_resume(lynx->anlt[PRIMARY_LANE]);
}

static void lynx_pcs_c73_teardown(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;
	int err;

	err = mtip_backplane_suspend(lynx->anlt[PRIMARY_LANE]);
	if (err)
		dev_warn(&lynx->mdio->dev, "Failed to suspend: %pe\n",
			 ERR_PTR(err));

	for (i = 0; i < lynx->num_phys; i++)
		if (lynx->anlt[i])
			mtip_backplane_destroy(lynx->anlt[i]);
}

static int lynx_pcs_enable(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;
	int err;

	if (phylink_autoneg_c73(pcs->cfg_link_an_mode))
		return lynx_pcs_c73_init(pcs);

	/* The backplane AN/LT deals with lane power management */
	for (i = 0; i < lynx->num_phys; i++) {
		err = phy_power_on(lynx->serdes[i]);
		if (err)
			return err;
	}

	return 0;
}

static void lynx_pcs_disable(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;

	if (phylink_autoneg_c73(pcs->cfg_link_an_mode))
		return lynx_pcs_c73_teardown(pcs);

	/* The backplane AN/LT deals with lane power management */
	for (i = 0; i < lynx->num_phys; i++)
		phy_power_off(lynx->serdes[i]);
}

static const struct phylink_pcs_ops lynx_pcs_phylink_ops = {
	.pcs_get_state = lynx_pcs_get_state,
	.pcs_config = lynx_pcs_config,
	.pcs_an_restart = lynx_pcs_an_restart,
	.pcs_link_up = lynx_pcs_link_up,
	.pcs_validate = lynx_pcs_validate,
	.pcs_enable = lynx_pcs_enable,
	.pcs_disable = lynx_pcs_disable,
};

void lynx_pcs_set_supported_interfaces(struct phylink_pcs *pcs,
				       phy_interface_t default_interface,
				       unsigned long *supported_interfaces)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	phy_interface_t iface;
	int err;

	__set_bit(default_interface, supported_interfaces);

	if (default_interface == PHY_INTERFACE_MODE_1000BASEX ||
	    default_interface == PHY_INTERFACE_MODE_SGMII) {
		__set_bit(PHY_INTERFACE_MODE_1000BASEX, supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_SGMII, supported_interfaces);
	}

	if (!lynx->num_phys)
		return;

	/* In case we have access to the SerDes phy/lane, then ask the SerDes
	 * driver what interfaces are supported based on the current PLL
	 * configuration.
	 */
	for (iface = 0; iface < PHY_INTERFACE_MODE_MAX; iface++) {
		if (iface == PHY_INTERFACE_MODE_NA)
			continue;

		err = phy_validate(lynx->serdes[PRIMARY_LANE],
				   PHY_MODE_ETHERNET, iface, NULL);
		if (err)
			continue;

		__set_bit(iface, supported_interfaces);
	}
}
EXPORT_SYMBOL(lynx_pcs_set_supported_interfaces);

static int lynx_pcs_validate_addr(struct mdio_device *mdiodev,
				  struct phy *serdes)
{
	union phy_status_opts opts1 = {
		.pcvt_count = {
			.type = PHY_PCVT_ETHERNET_PCS,
		},
	};
	int i, err;

	err = phy_get_status(serdes, PHY_STATUS_PCVT_COUNT, &opts1);
	if (err)
		return err;

	for (i = 0; i < opts1.pcvt_count.num_pcvt; i++) {
		union phy_status_opts opts2 = {
			.pcvt = {
				.type = PHY_PCVT_ETHERNET_PCS,
				.index = i,
			},
		};

		err = phy_get_status(serdes, PHY_STATUS_PCVT_ADDR, &opts2);
		if (err)
			return err;

		/* For a multi-port protocol converter, the match is
		 * approximate, since for full confidence, we'd have to
		 * know which port within the PCS is the consumer (MAC)
		 * mapped to.
		 */
		if (opts2.pcvt.addr.mdio == mdiodev->addr)
			return 0;
	}

	dev_err(&mdiodev->dev,
		"Own MDIO address not found among %zu protocol converters reported by SerDes lane %s\n",
		opts1.pcvt_count.num_pcvt,
		dev_name(&serdes->dev));

	return -ENODEV;
}

static struct phylink_pcs *lynx_pcs_create(struct mdio_device *mdio,
					   struct phy **phys, size_t num_phys,
					   enum mtip_model model)
{
	struct lynx_pcs *lynx;
	size_t i;
	int err;

	if (num_phys > MAX_NUM_LANES)
		return ERR_PTR(-ERANGE);

	lynx = kzalloc(sizeof(*lynx), GFP_KERNEL);
	if (!lynx)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_phys; i++) {
		lynx->serdes[i] = phys[i];
		err = phy_init(lynx->serdes[i]);
		if (err) {
			dev_err(&mdio->dev,
				"Failed to initialize SerDes: %pe\n",
				ERR_PTR(err));

			while (i--)
				phy_exit(lynx->serdes[i]);
			goto out_free;
		}
	}

	if (num_phys) {
		err = lynx_pcs_validate_addr(mdio, lynx->serdes[PRIMARY_LANE]);
		if (err && err != -EOPNOTSUPP)
			goto out_phy_exit;
	}

	mdio_device_get(mdio);
	lynx->mdio = mdio;
	lynx->pcs.ops = &lynx_pcs_phylink_ops;
	lynx->pcs.neg_mode = true;
	lynx->pcs.poll = true;
	lynx->num_phys = num_phys;
	lynx->model = model;

	return lynx_to_phylink_pcs(lynx);

out_phy_exit:
	for (i = 0; i < num_phys; i++)
		phy_exit(lynx->serdes[i]);
out_free:
	kfree(lynx);
	return ERR_PTR(err);
}

struct phylink_pcs *lynx_pcs_create_mdiodev(struct mii_bus *bus, int addr,
					    struct phy **phys, size_t num_phys)
{
	struct mdio_device *mdio;
	struct phylink_pcs *pcs;

	mdio = mdio_device_create(bus, addr);
	if (IS_ERR(mdio))
		return ERR_CAST(mdio);

	pcs = lynx_pcs_create(mdio, phys, num_phys, MTIP_MODEL_AUTODETECT);

	/* lynx_create() has taken a refcount on the mdiodev if it was
	 * successful. If lynx_create() fails, this will free the mdio
	 * device here. In any case, we don't need to hold our reference
	 * anymore, and putting it here will allow mdio_device_put() in
	 * lynx_destroy() to automatically free the mdio device.
	 */
	mdio_device_put(mdio);

	return pcs;
}
EXPORT_SYMBOL(lynx_pcs_create_mdiodev);

/*
 * lynx_pcs_create_fwnode() creates a lynx PCS instance from the fwnode
 * device indicated by node.
 *
 * Returns:
 *  -ENODEV if the fwnode is marked unavailable
 *  -EPROBE_DEFER if we fail to find the device
 *  -ENOMEM if we fail to allocate memory
 *  pointer to a phylink_pcs on success
 */
struct phylink_pcs *lynx_pcs_create_fwnode(struct fwnode_handle *node,
					   struct phy **phys, size_t num_phys)
{
	enum mtip_model model = MTIP_MODEL_AUTODETECT;
	struct mdio_device *mdio;
	struct phylink_pcs *pcs;

	if (!fwnode_device_is_available(node))
		return ERR_PTR(-ENODEV);

	if (fwnode_device_is_compatible(node, "fsl,lx2160a-lynx-pcs"))
		model = MTIP_MODEL_LX2160A;

	mdio = fwnode_mdio_find_device(node);
	if (!mdio)
		return ERR_PTR(-EPROBE_DEFER);

	pcs = lynx_pcs_create(mdio, phys, num_phys, model);

	/* lynx_create() has taken a refcount on the mdiodev if it was
	 * successful. If lynx_create() fails, this will free the mdio
	 * device here. In any case, we don't need to hold our reference
	 * anymore, and putting it here will allow mdio_device_put() in
	 * lynx_destroy() to automatically free the mdio device.
	 */
	mdio_device_put(mdio);

	return pcs;
}
EXPORT_SYMBOL_GPL(lynx_pcs_create_fwnode);

void lynx_pcs_destroy(struct phylink_pcs *pcs)
{
	struct lynx_pcs *lynx = phylink_pcs_to_lynx(pcs);
	size_t i;

	for (i = 0; i < lynx->num_phys; i++)
		phy_exit(lynx->serdes[i]);

	mdio_device_put(lynx->mdio);

	kfree(lynx);
}
EXPORT_SYMBOL(lynx_pcs_destroy);

MODULE_DESCRIPTION("NXP Lynx PCS phylink library");
MODULE_LICENSE("Dual BSD/GPL");
