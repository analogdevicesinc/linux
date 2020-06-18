
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

#include <linux/version.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,19,999)
#error "Please check this compat layer and see what needs to be removed. After that, please adjust the KERNEL_VERSION(x,y,z) until all things are un-needed. Somewhere around version 5.3, all these should go way."
#endif

/* FIXME: These go away starting at kernel 5.0.
 *        Unfortunately, these need to be macros/renames, because there are
 *        already __mdiobus_{read,write} in 4.19, but no phy_modify_mmd_*()
 *        functions yet. And MMD hooks need to be locked, because the MDIO
 *        lock re-work isn't present in this kernel version.
 */
#define __mdiobus_read		mdiobus_read
#define __mdiobus_write		mdiobus_write

static inline int phy_modify_mmd_changed(struct phy_device *phydev, int devad,
					 u32 regnum, u16 mask, u16 set)
{
	int new, ret;

	ret = phy_read_mmd(phydev, devad, regnum);
	if (ret < 0)
		return ret;

	new = (ret & ~mask) | set;
	if (new == ret)
		return 0;

	ret = phy_write_mmd(phydev, devad, regnum, new);

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

static inline int phy_set_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, 0, val);
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
