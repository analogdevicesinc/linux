
/* Intentionally not adding header guards.
 * This is code that should be removed as we upgrade the kernel.
 * Some things should be removed after each upgrade.
 * Most of the following bits of code have been taken from the latest PHY code.
 */

#define PHY_ID_MATCH_MODEL(id) .phy_id = (id), .phy_id_mask = GENMASK(31, 4)

#define phydev_warn(_phydev, format, args...)	\
	dev_warn(&_phydev->mdio.dev, format, ##args)

#define phydev_info(_phydev, format, args...)	\
	dev_info(&_phydev->mdio.dev, format, ##args)

/* __mdiobus_read & __mdiobus_write are unlocked functions in upstream kernel,
 * but we shouldn't implement them as unlocked in 4.14, as the locks that
 * protect them, don't exist; this way, it should fail with compilation error
 * and we can remove them, as we know that the kernel implements correct locking
 */
static inline int __mdiobus_read(struct mii_bus *bus, int addr, u32 regnum)
{
	return mdiobus_read(bus, addr, regnum);
}

static inline int __mdiobus_write(struct mii_bus *bus, int addr,
				  u32 regnum, u16 val)
{
	return mdiobus_write(bus, addr, regnum, val);
}

static int adin_read_mmd(struct phy_device *phydev, int devad, u16 regnum);
static int adin_write_mmd(struct phy_device *phydev, int devad, u16 regnum,
			  u16 val);

static inline int phy_modify_mmd_changed(struct phy_device *phydev, int devad,
					 u32 regnum, u16 mask, u16 set)
{
	int new, ret;

	ret = adin_read_mmd(phydev, devad, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = adin_write_mmd(phydev, devad, regnum, new);

	return ret < 0 ? ret : 1;
}

static inline int phy_modify_mmd(struct phy_device *phydev, int devad,
				 u32 regnum, u16 mask, u16 set)
{
	int ret;

	ret = phy_modify_mmd_changed(phydev, devad, regnum, mask, set);

	return ret < 0 ? ret : 0;
}

static inline int phy_clear_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, val, 0);
}

static inline int phy_modify_changed(struct phy_device *phydev, u32 regnum,
				     u16 mask, u16 set)
{
	int new, ret;

	ret = phy_read(phydev, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = phy_write(phydev, regnum, new);

	return ret < 0 ? ret : 1;
}
static inline int phy_modify(struct phy_device *phydev, u32 regnum,
			     u16 mask, u16 set)
{
	int ret;

	ret = phy_modify_changed(phydev, regnum, mask, set);

	return ret < 0 ? ret : 0;
}
static inline int phy_set_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, 0, val);
}

static inline int phy_clear_bits(struct phy_device *phydev, u32 regnum, u16 val)
{
	return phy_modify(phydev, regnum, val, 0);
}

static inline int phy_set_bits(struct phy_device *phydev, u32 regnum, u16 val)
{
	return phy_modify(phydev, regnum, 0, val);
}

/**
 * Below are parts for ADIN1100 T1 PHY, for Clause 45 that are missing in 4.19
 * Defining them here as static-inline is a trick to get them working until
 * the kernel is updated, and they are present.
 * Same as with other parts from this file, these are backports re-implemented
 * or adapted for this kernel version.
 */

/*
 * Careful with genphy_c45_read_link() : return value
 * semantic differs slightly in 4.19 vs newer kernels.
 * In newer kernels, it returns 0 or negative for error.
 * In 4.19-ish, it also eturns true if the link is active.
 */
static inline int genphy_c45_read_link_compat(struct phy_device *phydev)
{
	int ret = genphy_c45_read_link(phydev, MDIO_DEVS_PMAPMD);

	if (ret < 0) {
		phydev->link = 0;
		return ret;
	}

	phydev->link = (ret == true);

	return 0;
}

#define genphy_c45_read_link	genphy_c45_read_link_compat

static inline int genphy_c45_check_and_restart_aneg(struct phy_device *phydev, bool restart)
{
	int ret;

	if (!restart) {
		/* Configure and restart aneg if it wasn't set before */
		ret = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_CTRL1);
		if (ret < 0)
			return ret;

		if (!(ret & MDIO_AN_CTRL1_ENABLE))
			restart = true;
	}

	if (restart)
		return genphy_c45_restart_aneg(phydev);

	return 0;
}

static inline void phy_resolve_aneg_linkmode(struct phy_device *phydev)
{
	u32 common = phydev->lp_advertising & phydev->advertising;

	if (common & ADVERTISED_10000baseT_Full) {
		phydev->speed = SPEED_10000;
		phydev->duplex = DUPLEX_FULL;
	} else if (common & ADVERTISED_1000baseT_Full) {
		phydev->speed = SPEED_1000;
		phydev->duplex = DUPLEX_FULL;
	} else if (common & ADVERTISED_1000baseT_Half) {
		phydev->speed = SPEED_1000;
		phydev->duplex = DUPLEX_HALF;
	} else if (common & ADVERTISED_100baseT_Full) {
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_FULL;
	} else if (common & ADVERTISED_100baseT_Half) {
		phydev->speed = SPEED_100;
		phydev->duplex = DUPLEX_HALF;
	} else if (common & ADVERTISED_10baseT_Full) {
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_FULL;
	} else if (common & ADVERTISED_10baseT_Half) {
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_HALF;
	}

	if (phydev->duplex == DUPLEX_FULL) {
		phydev->pause = !!(phydev->lp_advertising & ADVERTISED_Pause);
		phydev->asym_pause = !!(phydev->lp_advertising &
					ADVERTISED_Asym_Pause);
	}
}
