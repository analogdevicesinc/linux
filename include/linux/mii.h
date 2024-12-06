/* SPDX-License-Identifier: GPL-2.0 */
/*
 * linux/mii.h: definitions for MII-compatible transceivers
 * Originally drivers/net/sunhme.h.
 *
 * Copyright (C) 1996, 1999, 2001 David S. Miller (davem@redhat.com)
 */
#ifndef __LINUX_MII_H__
#define __LINUX_MII_H__


#include <linux/if.h>
#include <linux/linkmode.h>
#include <uapi/linux/mii.h>

/* 802.3-2018 clause 73.6 Link codeword encoding */
#define C73_BASE_PAGE_SELECTOR(x)		((x) & GENMASK(4, 0))
#define C73_BASE_PAGE_ECHOED_NONCE(x)		(((x) << 5) & GENMASK(9, 5))
#define C73_BASE_PAGE_ECHOED_NONCE_X(x)		(((x) & GENMASK(9, 5)) >> 5)
#define C73_BASE_PAGE_ECHOED_NONCE_MSK		GENMASK(9, 5)
#define C73_BASE_PAGE_PAUSE			BIT(10)
#define C73_BASE_PAGE_ASM_DIR			BIT(11)
#define C73_BASE_PAGE_RF			BIT(13)
#define C73_BASE_PAGE_ACK			BIT(14)
#define C73_BASE_PAGE_NP			BIT(15)
#define C73_BASE_PAGE_TRANSMITTED_NONCE(x)	(((x) << 16) & GENMASK(20, 16))
#define C73_BASE_PAGE_TRANSMITTED_NONCE_X(x)	(((x) & GENMASK(20, 16)) >> 16)
#define C73_BASE_PAGE_TRANSMITTED_NONCE_MSK	GENMASK(20, 16)
#define C73_BASE_PAGE_A(x)			BIT(21 + (x))
#define C73_BASE_PAGE_TECH_ABL_1000BASEKX	C73_BASE_PAGE_A(0)
#define C73_BASE_PAGE_TECH_ABL_10GBASEKX4	C73_BASE_PAGE_A(1)
#define C73_BASE_PAGE_TECH_ABL_10GBASEKR	C73_BASE_PAGE_A(2)
#define C73_BASE_PAGE_TECH_ABL_40GBASEKR4	C73_BASE_PAGE_A(3)
#define C73_BASE_PAGE_TECH_ABL_40GBASECR4	C73_BASE_PAGE_A(4)
#define C73_BASE_PAGE_TECH_ABL_100GBASECR10	C73_BASE_PAGE_A(5)
#define C73_BASE_PAGE_TECH_ABL_100GBASEKP4	C73_BASE_PAGE_A(6)
#define C73_BASE_PAGE_TECH_ABL_100GBASEKR4	C73_BASE_PAGE_A(7)
#define C73_BASE_PAGE_TECH_ABL_100GBASECR4	C73_BASE_PAGE_A(8)
#define C73_BASE_PAGE_TECH_ABL_25GBASEKRS	C73_BASE_PAGE_A(9)
#define C73_BASE_PAGE_TECH_ABL_25GBASEKR	C73_BASE_PAGE_A(10)
#define C73_BASE_PAGE_25G_RS_FEC_REQ		BIT_ULL(44)
#define C73_BASE_PAGE_25G_BASER_FEC_REQ		BIT_ULL(45)
#define C73_BASE_PAGE_10G_BASER_FEC_ABL		BIT_ULL(46)
#define C73_BASE_PAGE_10G_BASER_FEC_REQ		BIT_ULL(47)

#define C73_NEXT_PAGE_MESSAGE_CODE(x)		((x) & GENMASK(10, 0))
#define C73_NEXT_PAGE_TOGGLE			BIT(11)
#define C73_NEXT_PAGE_ACK2			BIT(12)
#define C73_NEXT_PAGE_MP			BIT(13)
#define C73_NEXT_PAGE_ACK			BIT(14)
#define C73_NEXT_PAGE_NP			BIT(15)
#define C73_NEXT_PAGE_U_MSK			GENMASK(47, 16)
#define C73_NEXT_PAGE_U_X(x)			(((x) & C73_NEXT_PAGE_U_MSK) >> 16)
#define C73_NEXT_PAGE_U(x)			(((x) << 16) & C73_NEXT_PAGE_U_MSK)

struct ethtool_cmd;

struct mii_if_info {
	int phy_id;
	int advertising;
	int phy_id_mask;
	int reg_num_mask;

	unsigned int full_duplex : 1;	/* is full duplex? */
	unsigned int force_media : 1;	/* is autoneg. disabled? */
	unsigned int supports_gmii : 1; /* are GMII registers supported? */

	struct net_device *dev;
	int (*mdio_read) (struct net_device *dev, int phy_id, int location);
	void (*mdio_write) (struct net_device *dev, int phy_id, int location, int val);
};

extern int mii_link_ok (struct mii_if_info *mii);
extern int mii_nway_restart (struct mii_if_info *mii);
extern void mii_ethtool_gset(struct mii_if_info *mii, struct ethtool_cmd *ecmd);
extern void mii_ethtool_get_link_ksettings(
	struct mii_if_info *mii, struct ethtool_link_ksettings *cmd);
extern int mii_ethtool_sset(struct mii_if_info *mii, struct ethtool_cmd *ecmd);
extern int mii_ethtool_set_link_ksettings(
	struct mii_if_info *mii, const struct ethtool_link_ksettings *cmd);
extern int mii_check_gmii_support(struct mii_if_info *mii);
extern void mii_check_link (struct mii_if_info *mii);
extern unsigned int mii_check_media (struct mii_if_info *mii,
				     unsigned int ok_to_print,
				     unsigned int init_media);
extern int generic_mii_ioctl(struct mii_if_info *mii_if,
			     struct mii_ioctl_data *mii_data, int cmd,
			     unsigned int *duplex_changed);

extern int
linkmode_c73_priority_resolution(const unsigned long *modes,
				 enum ethtool_link_mode_bit_indices *resolved);
extern void linkmode_support_c73(unsigned long *modes);


static inline struct mii_ioctl_data *if_mii(struct ifreq *rq)
{
	return (struct mii_ioctl_data *) &rq->ifr_ifru;
}

/**
 * mii_nway_result
 * @negotiated: value of MII ANAR and'd with ANLPAR
 *
 * Given a set of MII abilities, check each bit and returns the
 * currently supported media, in the priority order defined by
 * IEEE 802.3u.  We use LPA_xxx constants but note this is not the
 * value of LPA solely, as described above.
 *
 * The one exception to IEEE 802.3u is that 100baseT4 is placed
 * between 100T-full and 100T-half.  If your phy does not support
 * 100T4 this is fine.  If your phy places 100T4 elsewhere in the
 * priority order, you will need to roll your own function.
 */
static inline unsigned int mii_nway_result (unsigned int negotiated)
{
	unsigned int ret;

	if (negotiated & LPA_100FULL)
		ret = LPA_100FULL;
	else if (negotiated & LPA_100BASE4)
		ret = LPA_100BASE4;
	else if (negotiated & LPA_100HALF)
		ret = LPA_100HALF;
	else if (negotiated & LPA_10FULL)
		ret = LPA_10FULL;
	else
		ret = LPA_10HALF;

	return ret;
}

/**
 * mii_duplex
 * @duplex_lock: Non-zero if duplex is locked at full
 * @negotiated: value of MII ANAR and'd with ANLPAR
 *
 * A small helper function for a common case.  Returns one
 * if the media is operating or locked at full duplex, and
 * returns zero otherwise.
 */
static inline unsigned int mii_duplex (unsigned int duplex_lock,
				       unsigned int negotiated)
{
	if (duplex_lock)
		return 1;
	if (mii_nway_result(negotiated) & LPA_DUPLEX)
		return 1;
	return 0;
}

/**
 * ethtool_adv_to_mii_adv_t
 * @ethadv: the ethtool advertisement settings
 *
 * A small helper function that translates ethtool advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_ADVERTISE register.
 */
static inline u32 ethtool_adv_to_mii_adv_t(u32 ethadv)
{
	u32 result = 0;

	if (ethadv & ADVERTISED_10baseT_Half)
		result |= ADVERTISE_10HALF;
	if (ethadv & ADVERTISED_10baseT_Full)
		result |= ADVERTISE_10FULL;
	if (ethadv & ADVERTISED_100baseT_Half)
		result |= ADVERTISE_100HALF;
	if (ethadv & ADVERTISED_100baseT_Full)
		result |= ADVERTISE_100FULL;
	if (ethadv & ADVERTISED_Pause)
		result |= ADVERTISE_PAUSE_CAP;
	if (ethadv & ADVERTISED_Asym_Pause)
		result |= ADVERTISE_PAUSE_ASYM;

	return result;
}

/**
 * linkmode_adv_to_mii_adv_t
 * @advertising: the linkmode advertisement settings
 *
 * A small helper function that translates linkmode advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_ADVERTISE register.
 */
static inline u32 linkmode_adv_to_mii_adv_t(const unsigned long *advertising)
{
	u32 result = 0;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT, advertising))
		result |= ADVERTISE_10HALF;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT, advertising))
		result |= ADVERTISE_10FULL;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT, advertising))
		result |= ADVERTISE_100HALF;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT, advertising))
		result |= ADVERTISE_100FULL;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT, advertising))
		result |= ADVERTISE_PAUSE_CAP;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, advertising))
		result |= ADVERTISE_PAUSE_ASYM;

	return result;
}

/**
 * mii_adv_to_ethtool_adv_t
 * @adv: value of the MII_ADVERTISE register
 *
 * A small helper function that translates MII_ADVERTISE bits
 * to ethtool advertisement settings.
 */
static inline u32 mii_adv_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_10HALF)
		result |= ADVERTISED_10baseT_Half;
	if (adv & ADVERTISE_10FULL)
		result |= ADVERTISED_10baseT_Full;
	if (adv & ADVERTISE_100HALF)
		result |= ADVERTISED_100baseT_Half;
	if (adv & ADVERTISE_100FULL)
		result |= ADVERTISED_100baseT_Full;
	if (adv & ADVERTISE_PAUSE_CAP)
		result |= ADVERTISED_Pause;
	if (adv & ADVERTISE_PAUSE_ASYM)
		result |= ADVERTISED_Asym_Pause;

	return result;
}

/**
 * ethtool_adv_to_mii_ctrl1000_t
 * @ethadv: the ethtool advertisement settings
 *
 * A small helper function that translates ethtool advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_CTRL1000 register when in 1000T mode.
 */
static inline u32 ethtool_adv_to_mii_ctrl1000_t(u32 ethadv)
{
	u32 result = 0;

	if (ethadv & ADVERTISED_1000baseT_Half)
		result |= ADVERTISE_1000HALF;
	if (ethadv & ADVERTISED_1000baseT_Full)
		result |= ADVERTISE_1000FULL;

	return result;
}

/**
 * linkmode_adv_to_mii_ctrl1000_t
 * @advertising: the linkmode advertisement settings
 *
 * A small helper function that translates linkmode advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_CTRL1000 register when in 1000T mode.
 */
static inline u32
linkmode_adv_to_mii_ctrl1000_t(const unsigned long *advertising)
{
	u32 result = 0;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
			      advertising))
		result |= ADVERTISE_1000HALF;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
			      advertising))
		result |= ADVERTISE_1000FULL;

	return result;
}

/**
 * mii_ctrl1000_to_ethtool_adv_t
 * @adv: value of the MII_CTRL1000 register
 *
 * A small helper function that translates MII_CTRL1000
 * bits, when in 1000Base-T mode, to ethtool
 * advertisement settings.
 */
static inline u32 mii_ctrl1000_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_1000HALF)
		result |= ADVERTISED_1000baseT_Half;
	if (adv & ADVERTISE_1000FULL)
		result |= ADVERTISED_1000baseT_Full;

	return result;
}

/**
 * mii_lpa_to_ethtool_lpa_t
 * @adv: value of the MII_LPA register
 *
 * A small helper function that translates MII_LPA
 * bits, when in 1000Base-T mode, to ethtool
 * LP advertisement settings.
 */
static inline u32 mii_lpa_to_ethtool_lpa_t(u32 lpa)
{
	u32 result = 0;

	if (lpa & LPA_LPACK)
		result |= ADVERTISED_Autoneg;

	return result | mii_adv_to_ethtool_adv_t(lpa);
}

/**
 * mii_stat1000_to_ethtool_lpa_t
 * @adv: value of the MII_STAT1000 register
 *
 * A small helper function that translates MII_STAT1000
 * bits, when in 1000Base-T mode, to ethtool
 * advertisement settings.
 */
static inline u32 mii_stat1000_to_ethtool_lpa_t(u32 lpa)
{
	u32 result = 0;

	if (lpa & LPA_1000HALF)
		result |= ADVERTISED_1000baseT_Half;
	if (lpa & LPA_1000FULL)
		result |= ADVERTISED_1000baseT_Full;

	return result;
}

/**
 * mii_stat1000_mod_linkmode_lpa_t
 * @advertising: target the linkmode advertisement settings
 * @adv: value of the MII_STAT1000 register
 *
 * A small helper function that translates MII_STAT1000 bits, when in
 * 1000Base-T mode, to linkmode advertisement settings. Other bits in
 * advertising are not changes.
 */
static inline void mii_stat1000_mod_linkmode_lpa_t(unsigned long *advertising,
						   u32 lpa)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT,
			 advertising, lpa & LPA_1000HALF);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
			 advertising, lpa & LPA_1000FULL);
}

/**
 * ethtool_adv_to_mii_adv_x
 * @ethadv: the ethtool advertisement settings
 *
 * A small helper function that translates ethtool advertisement
 * settings to phy autonegotiation advertisements for the
 * MII_CTRL1000 register when in 1000Base-X mode.
 */
static inline u32 ethtool_adv_to_mii_adv_x(u32 ethadv)
{
	u32 result = 0;

	if (ethadv & ADVERTISED_1000baseT_Half)
		result |= ADVERTISE_1000XHALF;
	if (ethadv & ADVERTISED_1000baseT_Full)
		result |= ADVERTISE_1000XFULL;
	if (ethadv & ADVERTISED_Pause)
		result |= ADVERTISE_1000XPAUSE;
	if (ethadv & ADVERTISED_Asym_Pause)
		result |= ADVERTISE_1000XPSE_ASYM;

	return result;
}

/**
 * mii_adv_to_ethtool_adv_x
 * @adv: value of the MII_CTRL1000 register
 *
 * A small helper function that translates MII_CTRL1000
 * bits, when in 1000Base-X mode, to ethtool
 * advertisement settings.
 */
static inline u32 mii_adv_to_ethtool_adv_x(u32 adv)
{
	u32 result = 0;

	if (adv & ADVERTISE_1000XHALF)
		result |= ADVERTISED_1000baseT_Half;
	if (adv & ADVERTISE_1000XFULL)
		result |= ADVERTISED_1000baseT_Full;
	if (adv & ADVERTISE_1000XPAUSE)
		result |= ADVERTISED_Pause;
	if (adv & ADVERTISE_1000XPSE_ASYM)
		result |= ADVERTISED_Asym_Pause;

	return result;
}

/**
 * mii_adv_mod_linkmode_adv_t
 * @advertising:pointer to destination link mode.
 * @adv: value of the MII_ADVERTISE register
 *
 * A small helper function that translates MII_ADVERTISE bits to
 * linkmode advertisement settings. Leaves other bits unchanged.
 */
static inline void mii_adv_mod_linkmode_adv_t(unsigned long *advertising,
					      u32 adv)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Half_BIT,
			 advertising, adv & ADVERTISE_10HALF);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_10baseT_Full_BIT,
			 advertising, adv & ADVERTISE_10FULL);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Half_BIT,
			 advertising, adv & ADVERTISE_100HALF);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
			 advertising, adv & ADVERTISE_100FULL);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Pause_BIT, advertising,
			 adv & ADVERTISE_PAUSE_CAP);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT,
			 advertising, adv & ADVERTISE_PAUSE_ASYM);
}

/**
 * mii_adv_to_linkmode_adv_t
 * @advertising:pointer to destination link mode.
 * @adv: value of the MII_ADVERTISE register
 *
 * A small helper function that translates MII_ADVERTISE bits
 * to linkmode advertisement settings. Clears the old value
 * of advertising.
 */
static inline void mii_adv_to_linkmode_adv_t(unsigned long *advertising,
					     u32 adv)
{
	linkmode_zero(advertising);

	mii_adv_mod_linkmode_adv_t(advertising, adv);
}

/**
 * mii_lpa_to_linkmode_lpa_t
 * @adv: value of the MII_LPA register
 *
 * A small helper function that translates MII_LPA bits, when in
 * 1000Base-T mode, to linkmode LP advertisement settings. Clears the
 * old value of advertising
 */
static inline void mii_lpa_to_linkmode_lpa_t(unsigned long *lp_advertising,
					     u32 lpa)
{
	mii_adv_to_linkmode_adv_t(lp_advertising, lpa);

	if (lpa & LPA_LPACK)
		linkmode_set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
				 lp_advertising);

}

/**
 * mii_lpa_mod_linkmode_lpa_t
 * @adv: value of the MII_LPA register
 *
 * A small helper function that translates MII_LPA bits, when in
 * 1000Base-T mode, to linkmode LP advertisement settings. Leaves
 * other bits unchanged.
 */
static inline void mii_lpa_mod_linkmode_lpa_t(unsigned long *lp_advertising,
					      u32 lpa)
{
	mii_adv_mod_linkmode_adv_t(lp_advertising, lpa);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			 lp_advertising, lpa & LPA_LPACK);
}

static inline void mii_ctrl1000_mod_linkmode_adv_t(unsigned long *advertising,
						   u32 ctrl1000)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Half_BIT, advertising,
			 ctrl1000 & ADVERTISE_1000HALF);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT, advertising,
			 ctrl1000 & ADVERTISE_1000FULL);
}

/**
 * linkmode_adv_to_lcl_adv_t
 * @advertising:pointer to linkmode advertising
 *
 * A small helper function that translates linkmode advertising to LVL
 * pause capabilities.
 */
static inline u32 linkmode_adv_to_lcl_adv_t(const unsigned long *advertising)
{
	u32 lcl_adv = 0;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT,
			      advertising))
		lcl_adv |= ADVERTISE_PAUSE_CAP;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT,
			      advertising))
		lcl_adv |= ADVERTISE_PAUSE_ASYM;

	return lcl_adv;
}

/**
 * mii_lpa_mod_linkmode_x - decode the link partner's config_reg to linkmodes
 * @linkmodes: link modes array
 * @lpa: config_reg word from link partner
 * @fd_bit: link mode for 1000XFULL bit
 */
static inline void mii_lpa_mod_linkmode_x(unsigned long *linkmodes, u16 lpa,
					 int fd_bit)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, linkmodes,
			 lpa & LPA_LPACK);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Pause_BIT, linkmodes,
			 lpa & LPA_1000XPAUSE);
	linkmode_mod_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, linkmodes,
			 lpa & LPA_1000XPAUSE_ASYM);
	linkmode_mod_bit(fd_bit, linkmodes,
			 lpa & LPA_1000XFULL);
}

/**
 * linkmode_adv_to_mii_adv_x - encode a linkmode to config_reg
 * @linkmodes: linkmodes
 * @fd_bit: full duplex bit
 */
static inline u16 linkmode_adv_to_mii_adv_x(const unsigned long *linkmodes,
					    int fd_bit)
{
	u16 adv = 0;

	if (linkmode_test_bit(fd_bit, linkmodes))
		adv |= ADVERTISE_1000XFULL;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT, linkmodes))
		adv |= ADVERTISE_1000XPAUSE;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, linkmodes))
		adv |= ADVERTISE_1000XPSE_ASYM;

	return adv;
}

static inline u64 linkmode_adv_to_c73_base_page(const unsigned long *advertising)
{
	u64 result = 0;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_1000baseKX_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_1000BASEKX;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_10GBASEKX4;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_10000baseKR_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_10GBASEKR;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_40GBASEKR4;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_40GBASECR4;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_100GBASEKR4;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_100GBASECR4;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_25000baseKR_Full_BIT,
			      advertising))
		result |= C73_BASE_PAGE_TECH_ABL_25GBASEKR;

	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Pause_BIT, advertising))
		result |= C73_BASE_PAGE_PAUSE;
	if (linkmode_test_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, advertising))
		result |= C73_BASE_PAGE_ASM_DIR;

	return result;
}

static inline void mii_c73_mod_linkmode_lpa_t(unsigned long *advertising,
					      u64 base_page)
{
	linkmode_mod_bit(ETHTOOL_LINK_MODE_1000baseKX_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_1000BASEKX);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseKX4_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_10GBASEKX4);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_10000baseKR_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_10GBASEKR);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_40GBASEKR4);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_40000baseCR4_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_40GBASECR4);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_100000baseKR4_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_100GBASEKR4);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_100000baseCR4_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_100GBASECR4);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_25000baseKR_Full_BIT, advertising,
			 base_page & C73_BASE_PAGE_TECH_ABL_25GBASEKR);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Pause_BIT, advertising,
			 base_page & C73_BASE_PAGE_PAUSE);

	linkmode_mod_bit(ETHTOOL_LINK_MODE_Asym_Pause_BIT, advertising,
			 base_page & C73_BASE_PAGE_ASM_DIR);
}

/**
 * mii_advertise_flowctrl - get flow control advertisement flags
 * @cap: Flow control capabilities (FLOW_CTRL_RX, FLOW_CTRL_TX or both)
 */
static inline u16 mii_advertise_flowctrl(int cap)
{
	u16 adv = 0;

	if (cap & FLOW_CTRL_RX)
		adv = ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;
	if (cap & FLOW_CTRL_TX)
		adv ^= ADVERTISE_PAUSE_ASYM;

	return adv;
}

/**
 * mii_resolve_flowctrl_fdx
 * @lcladv: value of MII ADVERTISE register
 * @rmtadv: value of MII LPA register
 *
 * Resolve full duplex flow control as per IEEE 802.3-2005 table 28B-3
 */
static inline u8 mii_resolve_flowctrl_fdx(u16 lcladv, u16 rmtadv)
{
	u8 cap = 0;

	if (lcladv & rmtadv & ADVERTISE_PAUSE_CAP) {
		cap = FLOW_CTRL_TX | FLOW_CTRL_RX;
	} else if (lcladv & rmtadv & ADVERTISE_PAUSE_ASYM) {
		if (lcladv & ADVERTISE_PAUSE_CAP)
			cap = FLOW_CTRL_RX;
		else if (rmtadv & ADVERTISE_PAUSE_CAP)
			cap = FLOW_CTRL_TX;
	}

	return cap;
}

/**
 * mii_bmcr_encode_fixed - encode fixed speed/duplex settings to a BMCR value
 * @speed: a SPEED_* value
 * @duplex: a DUPLEX_* value
 *
 * Encode the speed and duplex to a BMCR value. 2500, 1000, 100 and 10 Mbps are
 * supported. 2500Mbps is encoded to 1000Mbps. Other speeds are encoded as 10
 * Mbps. Unknown duplex values are encoded to half-duplex.
 */
static inline u16 mii_bmcr_encode_fixed(int speed, int duplex)
{
	u16 bmcr;

	switch (speed) {
	case SPEED_2500:
	case SPEED_1000:
		bmcr = BMCR_SPEED1000;
		break;

	case SPEED_100:
		bmcr = BMCR_SPEED100;
		break;

	case SPEED_10:
	default:
		bmcr = BMCR_SPEED10;
		break;
	}

	if (duplex == DUPLEX_FULL)
		bmcr |= BMCR_FULLDPLX;

	return bmcr;
}

#endif /* __LINUX_MII_H__ */
