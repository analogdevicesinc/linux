/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright 2023 NXP
 */
#ifndef _MTIP_BACKPLANE_H
#define _MTIP_BACKPLANE_H

struct mdio_device;
struct mtip_backplane;
struct phy;
struct phylink_link_state;

enum mtip_model {
	MTIP_MODEL_AUTODETECT,
	MTIP_MODEL_LX2160A,
};

#define MTIP_PRIMARY_LANE	0
#define MTIP_MAX_NUM_LANES	4

#if IS_ENABLED(CONFIG_MTIP_BACKPLANE_PHY)

int mtip_backplane_config_aneg(struct mtip_backplane *priv, bool autoneg,
			       const unsigned long *advertising);
void mtip_backplane_an_restart(struct mtip_backplane *priv);
void mtip_backplane_get_state(struct mtip_backplane *priv,
			      struct phylink_link_state *state);
int mtip_backplane_suspend(struct mtip_backplane *priv);
int mtip_backplane_resume(struct mtip_backplane *priv);
int mtip_backplane_validate(struct phy *serdes, unsigned long *supported);
void mtip_backplane_add_subordinate(struct mtip_backplane *priv,
				    struct mtip_backplane *subordinate);
struct mtip_backplane *mtip_backplane_create(struct mdio_device *pcs_mdiodev,
					     struct phy *serdes,
					     enum mtip_model model);
void mtip_backplane_destroy(struct mtip_backplane *priv);

#else

static inline int mtip_backplane_config_aneg(struct mtip_backplane *priv,
					     bool autoneg,
					     const unsigned long *advertising)
{
	return -ENODEV;
}

static inline void mtip_backplane_an_restart(struct mtip_backplane *priv)
{
}

static inline void mtip_backplane_get_state(struct mtip_backplane *priv,
					    struct phylink_link_state *state)
{
}

static inline int mtip_backplane_suspend(struct mtip_backplane *priv)
{
	return -ENODEV;
}

static inline int mtip_backplane_resume(struct mtip_backplane *priv)
{
	return -ENODEV;
}

static inline int mtip_backplane_validate(struct phy *serdes,
					  unsigned long *supported)
{
	return -ENODEV;
}

static inline void mtip_backplane_add_subordinate(struct mtip_backplane *priv,
						  struct mtip_backplane *subordinate)
{
}

static inline struct mtip_backplane *mtip_backplane_create(struct mdio_device *pcs_mdiodev,
							   struct phy *serdes,
							   enum mtip_model model)
{
	return ERR_PTR(-ENODEV);
}

static inline void mtip_backplane_destroy(struct mtip_backplane *priv)
{
}

#endif

#endif
