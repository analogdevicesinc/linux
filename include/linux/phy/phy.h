/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * phy.h -- generic phy header file
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#ifndef __DRIVERS_PHY_H
#define __DRIVERS_PHY_H

#include <linux/err.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <linux/phy/phy-dp.h>
#include <linux/phy/phy-ethernet.h>
#include <linux/phy/phy-lvds.h>
#include <linux/phy/phy-mipi-dphy.h>

struct phy;

enum phy_mode {
	PHY_MODE_INVALID,
	PHY_MODE_USB_HOST,
	PHY_MODE_USB_HOST_LS,
	PHY_MODE_USB_HOST_FS,
	PHY_MODE_USB_HOST_HS,
	PHY_MODE_USB_HOST_SS,
	PHY_MODE_USB_DEVICE,
	PHY_MODE_USB_DEVICE_LS,
	PHY_MODE_USB_DEVICE_FS,
	PHY_MODE_USB_DEVICE_HS,
	PHY_MODE_USB_DEVICE_SS,
	PHY_MODE_USB_OTG,
	PHY_MODE_UFS_HS_A,
	PHY_MODE_UFS_HS_B,
	PHY_MODE_PCIE,
	PHY_MODE_ETHERNET,
	PHY_MODE_ETHERNET_LINKMODE,
	PHY_MODE_MIPI_DPHY,
	PHY_MODE_SATA,
	PHY_MODE_LVDS,
	PHY_MODE_DP
};

enum phy_media {
	PHY_MEDIA_DEFAULT,
	PHY_MEDIA_SR,
	PHY_MEDIA_DAC,
};

enum phy_status_type {
	/* Valid for PHY_MODE_ETHERNET and PHY_MODE_ETHERNET_LINKMODE */
	PHY_STATUS_CDR_LOCK,
	PHY_STATUS_PCVT_COUNT,
	PHY_STATUS_PCVT_ADDR,
};

/* enum phy_pcvt_type - PHY protocol converter type
 *
 * @PHY_PCVT_ETHERNET_PCS: Ethernet Physical Coding Sublayer, top-most layer of
 *			   an Ethernet PHY. Connects through MII to the MAC,
 *			   and handles link status detection and the conversion
 *			   of MII signals to link-specific code words (8b/10b,
 *			   64b/66b etc).
 * @PHY_PCVT_ETHERNET_ANLT: Ethernet Auto-Negotiation and Link Training,
 *			    bottom-most layer of an Ethernet PHY, beneath the
 *			    PMA and PMD. Its activity is only visible on the
 *			    physical medium, and it is responsible for
 *			    selecting the most adequate PCS/PMA/PMD set that
 *			    can operate on that medium.
 */
enum phy_pcvt_type {
	PHY_PCVT_ETHERNET_PCS,
	PHY_PCVT_ETHERNET_ANLT,
};

struct phy_status_opts_pcvt {
	enum phy_pcvt_type type;
	size_t index;
	union {
		unsigned int mdio;
	} addr;
};

struct phy_status_opts_pcvt_count {
	enum phy_pcvt_type type;
	size_t num_pcvt;
};

/* If the CDR (Clock and Data Recovery) block is able to lock onto the RX bit
 * stream, it means that the stream contains valid bit transitions for the
 * configured protocol. This indicates that a link partner is physically
 * present and powered on.
 */
struct phy_status_opts_cdr {
	bool cdr_locked;
};

/**
 * union phy_status_opts - Opaque generic phy status
 *
 * @cdr:	Configuration set applicable for PHY_STATUS_CDR_LOCK.
 * @pcvt_count:	Configuration set applicable for PHY_STATUS_PCVT_COUNT.
 * @pcvt:	Configuration set applicable for PHY_STATUS_PCVT_ADDR.
 */
union phy_status_opts {
	struct phy_status_opts_cdr		cdr;
	struct phy_status_opts_pcvt_count	pcvt_count;
	struct phy_status_opts_pcvt		pcvt;
};

/**
 * union phy_configure_opts - Opaque generic phy configuration
 *
 * @mipi_dphy:	Configuration set applicable for phys supporting
 *		the MIPI_DPHY phy mode.
 * @dp:		Configuration set applicable for phys supporting
 *		the DisplayPort protocol.
 * @lvds:	Configuration set applicable for phys supporting
 *		the LVDS phy mode.
 * @ethernet:	Configuration set applicable for phys supporting
 *		the ethernet and ethtool phy mode.
 */
union phy_configure_opts {
	struct phy_configure_opts_mipi_dphy	mipi_dphy;
	struct phy_configure_opts_dp		dp;
	struct phy_configure_opts_lvds		lvds;
	struct phy_configure_opts_ethernet	ethernet;
};

/**
 * struct phy_ops - set of function pointers for performing phy operations
 * @init: operation to be performed for initializing phy
 * @exit: operation to be performed while exiting
 * @power_on: powering on the phy
 * @power_off: powering off the phy
 * @set_mode: set the mode of the phy
 * @set_media: set the media type of the phy (optional)
 * @set_speed: set the speed of the phy (optional)
 * @reset: resetting the phy
 * @calibrate: calibrate the phy
 * @get_status: get the mode-specific status of the phy
 * @release: ops to be performed while the consumer relinquishes the PHY
 * @owner: the module owner containing the ops
 */
struct phy_ops {
	int	(*init)(struct phy *phy);
	int	(*exit)(struct phy *phy);
	int	(*power_on)(struct phy *phy);
	int	(*power_off)(struct phy *phy);
	int	(*set_mode)(struct phy *phy, enum phy_mode mode, int submode);
	int	(*set_media)(struct phy *phy, enum phy_media media);
	int	(*set_speed)(struct phy *phy, int speed);

	/**
	 * @configure:
	 *
	 * Optional.
	 *
	 * Used to change the PHY parameters. phy_init() must have
	 * been called on the phy.
	 *
	 * Returns: 0 if successful, an negative error code otherwise
	 */
	int	(*configure)(struct phy *phy, union phy_configure_opts *opts);

	/**
	 * @validate:
	 *
	 * Optional.
	 *
	 * Used to check that the current set of parameters can be
	 * handled by the phy. Implementations are free to tune the
	 * parameters passed as arguments if needed by some
	 * implementation detail or constraints. It must not change
	 * any actual configuration of the PHY, so calling it as many
	 * times as deemed fit by the consumer must have no side
	 * effect.
	 *
	 * Returns: 0 if the configuration can be applied, an negative
	 * error code otherwise
	 */
	int	(*validate)(struct phy *phy, enum phy_mode mode, int submode,
			    union phy_configure_opts *opts);
	int	(*reset)(struct phy *phy);
	int	(*calibrate)(struct phy *phy);

	/* notify phy connect status change */
	int	(*connect)(struct phy *phy, int port);
	int	(*disconnect)(struct phy *phy, int port);

	/**
	 * @get_status:
	 *
	 * Optional.
	 *
	 * Used to query the mode-specific status of the phy. Must have no side
	 * effects.
	 *
	 * Returns: 0 if the operation was successful, negative error code
	 * otherwise.
	 */
	int	(*get_status)(struct phy *phy, enum phy_status_type type,
			      union phy_status_opts *opts);
	void	(*release)(struct phy *phy);
	struct module *owner;
};

/**
 * struct phy_attrs - represents phy attributes
 * @bus_width: Data path width implemented by PHY
 * @max_link_rate: Maximum link rate supported by PHY (units to be decided by producer and consumer)
 * @mode: PHY mode
 */
struct phy_attrs {
	u32			bus_width;
	u32			max_link_rate;
	enum phy_mode		mode;
};

/**
 * struct phy - represents the phy device
 * @dev: phy device
 * @id: id of the phy device
 * @ops: function pointers for performing phy operations
 * @mutex: mutex to protect phy_ops
 * @init_count: used to protect when the PHY is used by multiple consumers
 * @power_count: used to protect when the PHY is used by multiple consumers
 * @attrs: used to specify PHY specific attributes
 * @pwr: power regulator associated with the phy
 * @debugfs: debugfs directory
 */
struct phy {
	struct device		dev;
	int			id;
	const struct phy_ops	*ops;
	struct mutex		mutex;
	int			init_count;
	int			power_count;
	struct phy_attrs	attrs;
	struct regulator	*pwr;
	struct dentry		*debugfs;
};

/**
 * struct phy_provider - represents the phy provider
 * @dev: phy provider device
 * @children: can be used to override the default (dev->of_node) child node
 * @owner: the module owner having of_xlate
 * @list: to maintain a linked list of PHY providers
 * @of_xlate: function pointer to obtain phy instance from phy pointer
 */
struct phy_provider {
	struct device		*dev;
	struct device_node	*children;
	struct module		*owner;
	struct list_head	list;
	struct phy * (*of_xlate)(struct device *dev,
				 const struct of_phandle_args *args);
};

/**
 * struct phy_lookup - PHY association in list of phys managed by the phy driver
 * @node: list node
 * @dev_id: the device of the association
 * @con_id: connection ID string on device
 * @phy: the phy of the association
 */
struct phy_lookup {
	struct list_head node;
	const char *dev_id;
	const char *con_id;
	struct phy *phy;
};

#define	to_phy(a)	(container_of((a), struct phy, dev))

#define	of_phy_provider_register(dev, xlate)	\
	__of_phy_provider_register((dev), NULL, THIS_MODULE, (xlate))

#define	devm_of_phy_provider_register(dev, xlate)	\
	__devm_of_phy_provider_register((dev), NULL, THIS_MODULE, (xlate))

#define of_phy_provider_register_full(dev, children, xlate) \
	__of_phy_provider_register(dev, children, THIS_MODULE, xlate)

#define devm_of_phy_provider_register_full(dev, children, xlate) \
	__devm_of_phy_provider_register(dev, children, THIS_MODULE, xlate)

static inline void phy_set_drvdata(struct phy *phy, void *data)
{
	dev_set_drvdata(&phy->dev, data);
}

static inline void *phy_get_drvdata(struct phy *phy)
{
	return dev_get_drvdata(&phy->dev);
}

#if IS_ENABLED(CONFIG_GENERIC_PHY)
int phy_pm_runtime_get(struct phy *phy);
int phy_pm_runtime_get_sync(struct phy *phy);
int phy_pm_runtime_put(struct phy *phy);
int phy_pm_runtime_put_sync(struct phy *phy);
void phy_pm_runtime_allow(struct phy *phy);
void phy_pm_runtime_forbid(struct phy *phy);
int phy_init(struct phy *phy);
int phy_exit(struct phy *phy);
int phy_power_on(struct phy *phy);
int phy_power_off(struct phy *phy);
int phy_set_mode_ext(struct phy *phy, enum phy_mode mode, int submode);
#define phy_set_mode(phy, mode) \
	phy_set_mode_ext(phy, mode, 0)
int phy_set_media(struct phy *phy, enum phy_media media);
int phy_set_speed(struct phy *phy, int speed);
int phy_configure(struct phy *phy, union phy_configure_opts *opts);
int phy_validate(struct phy *phy, enum phy_mode mode, int submode,
		 union phy_configure_opts *opts);
int phy_get_status(struct phy *phy, enum phy_status_type type,
		   union phy_status_opts *opts);

static inline enum phy_mode phy_get_mode(struct phy *phy)
{
	return phy->attrs.mode;
}
int phy_reset(struct phy *phy);
int phy_calibrate(struct phy *phy);
int phy_notify_connect(struct phy *phy, int port);
int phy_notify_disconnect(struct phy *phy, int port);
static inline int phy_get_bus_width(struct phy *phy)
{
	return phy->attrs.bus_width;
}
static inline void phy_set_bus_width(struct phy *phy, int bus_width)
{
	phy->attrs.bus_width = bus_width;
}
struct phy *phy_get(struct device *dev, const char *string);
struct phy *devm_phy_get(struct device *dev, const char *string);
struct phy *devm_phy_optional_get(struct device *dev, const char *string);
struct phy *devm_of_phy_get(struct device *dev, struct device_node *np,
			    const char *con_id);
struct phy *devm_of_phy_optional_get(struct device *dev, struct device_node *np,
				     const char *con_id);
struct phy *devm_of_phy_get_by_index(struct device *dev, struct device_node *np,
				     int index);
void of_phy_put(struct phy *phy);
void phy_put(struct device *dev, struct phy *phy);
void devm_phy_put(struct device *dev, struct phy *phy);
struct phy *of_phy_get(struct device_node *np, const char *con_id);
struct phy *of_phy_simple_xlate(struct device *dev,
				const struct of_phandle_args *args);
struct phy *phy_create(struct device *dev, struct device_node *node,
		       const struct phy_ops *ops);
struct phy *devm_phy_create(struct device *dev, struct device_node *node,
			    const struct phy_ops *ops);
void phy_destroy(struct phy *phy);
void devm_phy_destroy(struct device *dev, struct phy *phy);
struct phy_provider *__of_phy_provider_register(struct device *dev,
	struct device_node *children, struct module *owner,
	struct phy * (*of_xlate)(struct device *dev,
				 const struct of_phandle_args *args));
struct phy_provider *__devm_of_phy_provider_register(struct device *dev,
	struct device_node *children, struct module *owner,
	struct phy * (*of_xlate)(struct device *dev,
				 const struct of_phandle_args *args));
void of_phy_provider_unregister(struct phy_provider *phy_provider);
void devm_of_phy_provider_unregister(struct device *dev,
	struct phy_provider *phy_provider);
int phy_create_lookup(struct phy *phy, const char *con_id, const char *dev_id);
void phy_remove_lookup(struct phy *phy, const char *con_id, const char *dev_id);
#else
static inline int phy_pm_runtime_get(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_pm_runtime_get_sync(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_pm_runtime_put(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_pm_runtime_put_sync(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline void phy_pm_runtime_allow(struct phy *phy)
{
	return;
}

static inline void phy_pm_runtime_forbid(struct phy *phy)
{
	return;
}

static inline int phy_init(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_exit(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_power_on(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_power_off(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_set_mode_ext(struct phy *phy, enum phy_mode mode,
				   int submode)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

#define phy_set_mode(phy, mode) \
	phy_set_mode_ext(phy, mode, 0)

static inline int phy_set_media(struct phy *phy, enum phy_media media)
{
	if (!phy)
		return 0;
	return -ENODEV;
}

static inline int phy_set_speed(struct phy *phy, int speed)
{
	if (!phy)
		return 0;
	return -ENODEV;
}

static inline enum phy_mode phy_get_mode(struct phy *phy)
{
	return PHY_MODE_INVALID;
}

static inline int phy_reset(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_calibrate(struct phy *phy)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_notify_connect(struct phy *phy, int index)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_notify_disconnect(struct phy *phy, int index)
{
	if (!phy)
		return 0;
	return -ENOSYS;
}

static inline int phy_configure(struct phy *phy,
				union phy_configure_opts *opts)
{
	if (!phy)
		return 0;

	return -ENOSYS;
}

static inline int phy_validate(struct phy *phy, enum phy_mode mode, int submode,
			       union phy_configure_opts *opts)
{
	if (!phy)
		return 0;

	return -ENOSYS;
}

static inline int phy_get_status(struct phy *phy, enum phy_status_type type,
				 union phy_status_opts *opts)
{
	if (!phy)
		return 0;

	return -ENOSYS;
}

static inline int phy_get_bus_width(struct phy *phy)
{
	return -ENOSYS;
}

static inline void phy_set_bus_width(struct phy *phy, int bus_width)
{
	return;
}

static inline struct phy *phy_get(struct device *dev, const char *string)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *devm_phy_get(struct device *dev, const char *string)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *devm_phy_optional_get(struct device *dev,
						const char *string)
{
	return NULL;
}

static inline struct phy *devm_of_phy_get(struct device *dev,
					  struct device_node *np,
					  const char *con_id)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *devm_of_phy_optional_get(struct device *dev,
						   struct device_node *np,
						   const char *con_id)
{
	return NULL;
}

static inline struct phy *devm_of_phy_get_by_index(struct device *dev,
						   struct device_node *np,
						   int index)
{
	return ERR_PTR(-ENOSYS);
}

static inline void of_phy_put(struct phy *phy)
{
}

static inline void phy_put(struct device *dev, struct phy *phy)
{
}

static inline void devm_phy_put(struct device *dev, struct phy *phy)
{
}

static inline struct phy *of_phy_get(struct device_node *np, const char *con_id)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *of_phy_simple_xlate(struct device *dev,
					      const struct of_phandle_args *args)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *phy_create(struct device *dev,
				     struct device_node *node,
				     const struct phy_ops *ops)
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy *devm_phy_create(struct device *dev,
					  struct device_node *node,
					  const struct phy_ops *ops)
{
	return ERR_PTR(-ENOSYS);
}

static inline void phy_destroy(struct phy *phy)
{
}

static inline void devm_phy_destroy(struct device *dev, struct phy *phy)
{
}

static inline struct phy_provider *__of_phy_provider_register(
	struct device *dev, struct device_node *children, struct module *owner,
	struct phy * (*of_xlate)(struct device *dev,
				 const struct of_phandle_args *args))
{
	return ERR_PTR(-ENOSYS);
}

static inline struct phy_provider *__devm_of_phy_provider_register(struct device
	*dev, struct device_node *children, struct module *owner,
	struct phy * (*of_xlate)(struct device *dev,
				 const struct of_phandle_args *args))
{
	return ERR_PTR(-ENOSYS);
}

static inline void of_phy_provider_unregister(struct phy_provider *phy_provider)
{
}

static inline void devm_of_phy_provider_unregister(struct device *dev,
	struct phy_provider *phy_provider)
{
}
static inline int
phy_create_lookup(struct phy *phy, const char *con_id, const char *dev_id)
{
	return 0;
}
static inline void phy_remove_lookup(struct phy *phy, const char *con_id,
				     const char *dev_id) { }
#endif

#endif /* __DRIVERS_PHY_H */
