
/* Intentionally not adding header guards.
 * This is code that should be removed as we upgrade the kernel.
 * Some things should be removed after each upgrade.
 * Most of the following bits of code have been taken from the latest PHY code.
 */

#define PHY_ID_MATCH_MODEL(id) .phy_id = (id), .phy_id_mask = GENMASK(31, 4)

#define phydev_warn(_phydev, format, args...)	\
	dev_warn(&_phydev->mdio.dev, format, ##args)


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

static inline int phy_set_bits_mmd(struct phy_device *phydev, int devad,
		u32 regnum, u16 val)
{
	return phy_modify_mmd(phydev, devad, regnum, 0, val);
}
