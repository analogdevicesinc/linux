// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2024 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>

#include <linux/dsa/ksz_common.h>
#include <net/dsa.h>
#include <net/dscp.h>
#include <net/ieee8021q.h>

#include "ksz_common.h"
#include "ksz_dcb.h"
#include "ksz8.h"

#define KSZ8_REG_PORT_1_CTRL_0			0x10
#define KSZ8_PORT_DIFFSERV_ENABLE		BIT(6)
#define KSZ8_PORT_802_1P_ENABLE			BIT(5)
#define KSZ8_PORT_BASED_PRIO_M			GENMASK(4, 3)

#define KSZ88X3_REG_TOS_DSCP_CTRL		0x60
#define KSZ8765_REG_TOS_DSCP_CTRL		0x90

#define KSZ9477_REG_SW_MAC_TOS_CTRL		0x033e
#define KSZ9477_SW_TOS_DSCP_REMAP		BIT(0)
#define KSZ9477_SW_TOS_DSCP_DEFAULT_PRIO_M	GENMASK(5, 3)

#define KSZ9477_REG_DIFFSERV_PRIO_MAP		0x0340

#define KSZ9477_REG_PORT_MRI_PRIO_CTRL		0x0801
#define KSZ9477_PORT_HIGHEST_PRIO		BIT(7)
#define KSZ9477_PORT_OR_PRIO			BIT(6)
#define KSZ9477_PORT_MAC_PRIO_ENABLE		BIT(4)
#define KSZ9477_PORT_VLAN_PRIO_ENABLE		BIT(3)
#define KSZ9477_PORT_802_1P_PRIO_ENABLE		BIT(2)
#define KSZ9477_PORT_DIFFSERV_PRIO_ENABLE	BIT(1)
#define KSZ9477_PORT_ACL_PRIO_ENABLE		BIT(0)

#define KSZ9477_REG_PORT_MRI_MAC_CTRL		0x0802
#define KSZ9477_PORT_BASED_PRIO_M		GENMASK(2, 0)

struct ksz_apptrust_map {
	u8 apptrust;
	u8 bit;
};

static const struct ksz_apptrust_map ksz8_apptrust_map_to_bit[] = {
	{ DCB_APP_SEL_PCP, KSZ8_PORT_802_1P_ENABLE },
	{ IEEE_8021QAZ_APP_SEL_DSCP, KSZ8_PORT_DIFFSERV_ENABLE },
};

static const struct ksz_apptrust_map ksz9477_apptrust_map_to_bit[] = {
	{ DCB_APP_SEL_PCP, KSZ9477_PORT_802_1P_PRIO_ENABLE },
	{ IEEE_8021QAZ_APP_SEL_DSCP, KSZ9477_PORT_DIFFSERV_PRIO_ENABLE },
};

/* ksz_supported_apptrust[] - Supported apptrust selectors and Priority Order
 *			      of Internal Priority Map (IPM) sources.
 *
 * This array defines the apptrust selectors supported by the hardware, where
 * the index within the array indicates the priority of the selector - lower
 * indices correspond to higher priority. This fixed priority scheme is due to
 * the hardware's design, which does not support configurable priority among
 * different priority sources.
 *
 * The priority sources, including Tail Tag, ACL, VLAN PCP and DSCP are ordered
 * by the hardware's fixed logic, as detailed below. The order reflects a
 * non-configurable precedence where certain types of priority information
 * override others:
 *
 * 1. Tail Tag - Highest priority, overrides ACL, VLAN PCP, and DSCP priorities.
 * 2. ACL - Overrides VLAN PCP and DSCP priorities.
 * 3. VLAN PCP - Overrides DSCP priority.
 * 4. DSCP - Lowest priority, does not override any other priority source.
 *
 * In this context, the array's lower index (higher priority) for
 * 'DCB_APP_SEL_PCP' suggests its relative priority over
 * 'IEEE_8021QAZ_APP_SEL_DSCP' within the system's fixed priority scheme.
 *
 * DCB_APP_SEL_PCP - Priority Code Point selector
 * IEEE_8021QAZ_APP_SEL_DSCP - Differentiated Services Code Point selector
 */
static const u8 ksz_supported_apptrust[] = {
	DCB_APP_SEL_PCP,
	IEEE_8021QAZ_APP_SEL_DSCP,
};

static const char * const ksz_supported_apptrust_variants[] = {
	"empty", "dscp", "pcp", "dscp pcp"
};

static void ksz_get_default_port_prio_reg(struct ksz_device *dev, int *reg,
					  u8 *mask, int *shift)
{
	if (is_ksz8(dev)) {
		*reg = KSZ8_REG_PORT_1_CTRL_0;
		*mask = KSZ8_PORT_BASED_PRIO_M;
		*shift = __bf_shf(KSZ8_PORT_BASED_PRIO_M);
	} else {
		*reg = KSZ9477_REG_PORT_MRI_MAC_CTRL;
		*mask = KSZ9477_PORT_BASED_PRIO_M;
		*shift = __bf_shf(KSZ9477_PORT_BASED_PRIO_M);
	}
}

/**
 * ksz_get_dscp_prio_reg - Retrieves the DSCP-to-priority-mapping register
 * @dev: Pointer to the KSZ switch device structure
 * @reg: Pointer to the register address to be set
 * @per_reg: Pointer to the number of DSCP values per register
 * @mask: Pointer to the mask to be set
 *
 * This function retrieves the DSCP to priority mapping register, the number of
 * DSCP values per register, and the mask to be set.
 */
static void ksz_get_dscp_prio_reg(struct ksz_device *dev, int *reg,
				  int *per_reg, u8 *mask)
{
	if (ksz_is_ksz87xx(dev)) {
		*reg = KSZ8765_REG_TOS_DSCP_CTRL;
		*per_reg = 4;
		*mask = GENMASK(1, 0);
	} else if (ksz_is_ksz88x3(dev)) {
		*reg = KSZ88X3_REG_TOS_DSCP_CTRL;
		*per_reg = 4;
		*mask = GENMASK(1, 0);
	} else {
		*reg = KSZ9477_REG_DIFFSERV_PRIO_MAP;
		*per_reg = 2;
		*mask = GENMASK(2, 0);
	}
}

/**
 * ksz_get_apptrust_map_and_reg - Retrieves the apptrust map and register
 * @dev: Pointer to the KSZ switch device structure
 * @map: Pointer to the apptrust map to be set
 * @reg: Pointer to the register address to be set
 * @mask: Pointer to the mask to be set
 *
 * This function retrieves the apptrust map and register address for the
 * apptrust configuration.
 */
static void ksz_get_apptrust_map_and_reg(struct ksz_device *dev,
					 const struct ksz_apptrust_map **map,
					 int *reg, u8 *mask)
{
	if (is_ksz8(dev)) {
		*map = ksz8_apptrust_map_to_bit;
		*reg = KSZ8_REG_PORT_1_CTRL_0;
		*mask = KSZ8_PORT_DIFFSERV_ENABLE | KSZ8_PORT_802_1P_ENABLE;
	} else {
		*map = ksz9477_apptrust_map_to_bit;
		*reg = KSZ9477_REG_PORT_MRI_PRIO_CTRL;
		*mask = KSZ9477_PORT_802_1P_PRIO_ENABLE |
			KSZ9477_PORT_DIFFSERV_PRIO_ENABLE;
	}
}

/**
 * ksz_port_get_default_prio - Retrieves the default priority for a port on a
 *			       KSZ switch
 * @ds: Pointer to the DSA switch structure
 * @port: Port number from which to get the default priority
 *
 * This function fetches the default priority for the specified port on a KSZ
 * switch.
 *
 * Return: The default priority of the port on success, or a negative error
 * code on failure.
 */
int ksz_port_get_default_prio(struct dsa_switch *ds, int port)
{
	struct ksz_device *dev = ds->priv;
	int ret, reg, shift;
	u8 data, mask;

	ksz_get_default_port_prio_reg(dev, &reg, &mask, &shift);

	ret = ksz_pread8(dev, port, reg, &data);
	if (ret)
		return ret;

	return (data & mask) >> shift;
}

/**
 * ksz88x3_port_set_default_prio_quirks - Quirks for default priority
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to set the default priority
 * @prio: Priority value to set
 *
 * This function implements quirks for setting the default priority on KSZ88x3
 * devices. On Port 2, no other priority providers are working
 * except of PCP. So, configuring default priority on Port 2 is not possible.
 * On Port 1, it is not possible to configure port priority if PCP
 * apptrust on Port 2 is disabled. Since we disable multiple queues on the
 * switch to disable PCP on Port 2, we need to ensure that the default priority
 * configuration on Port 1 is in agreement with the configuration on Port 2.
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz88x3_port_set_default_prio_quirks(struct ksz_device *dev, int port,
						u8 prio)
{
	if (!prio)
		return 0;

	if (port == KSZ_PORT_2) {
		dev_err(dev->dev, "Port priority configuration is not working on Port 2\n");
		return -EINVAL;
	} else if (port == KSZ_PORT_1) {
		u8 port2_data;
		int ret;

		ret = ksz_pread8(dev, KSZ_PORT_2, KSZ8_REG_PORT_1_CTRL_0,
				 &port2_data);
		if (ret)
			return ret;

		if (!(port2_data & KSZ8_PORT_802_1P_ENABLE)) {
			dev_err(dev->dev, "Not possible to configure port priority on Port 1 if PCP apptrust on Port 2 is disabled\n");
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * ksz_port_set_default_prio - Sets the default priority for a port on a KSZ
 *			       switch
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to set the default priority
 * @prio: Priority value to set
 *
 * This function sets the default priority for the specified port on a KSZ
 * switch.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
int ksz_port_set_default_prio(struct dsa_switch *ds, int port, u8 prio)
{
	struct ksz_device *dev = ds->priv;
	int reg, shift, ret;
	u8 mask;

	if (prio >= dev->info->num_ipms)
		return -EINVAL;

	if (ksz_is_ksz88x3(dev)) {
		ret = ksz88x3_port_set_default_prio_quirks(dev, port, prio);
		if (ret)
			return ret;
	}

	ksz_get_default_port_prio_reg(dev, &reg, &mask, &shift);

	return ksz_prmw8(dev, port, reg, mask, (prio << shift) & mask);
}

/**
 * ksz_port_get_dscp_prio - Retrieves the priority for a DSCP value on a KSZ
 *			    switch
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to get the priority
 * @dscp: DSCP value for which to get the priority
 *
 * This function fetches the priority value from switch global DSCP-to-priorty
 * mapping table for the specified DSCP value.
 *
 * Return: The priority value for the DSCP on success, or a negative error
 * code on failure.
 */
int ksz_port_get_dscp_prio(struct dsa_switch *ds, int port, u8 dscp)
{
	struct ksz_device *dev = ds->priv;
	int reg, per_reg, ret, shift;
	u8 data, mask;

	ksz_get_dscp_prio_reg(dev, &reg, &per_reg, &mask);

	/* If DSCP remapping is disabled, DSCP bits 3-5 are used as Internal
	 * Priority Map (IPM)
	 */
	if (!is_ksz8(dev)) {
		ret = ksz_read8(dev, KSZ9477_REG_SW_MAC_TOS_CTRL, &data);
		if (ret)
			return ret;

		/* If DSCP remapping is disabled, DSCP bits 3-5 are used as
		 * Internal Priority Map (IPM)
		 */
		if (!(data & KSZ9477_SW_TOS_DSCP_REMAP))
			return FIELD_GET(KSZ9477_SW_TOS_DSCP_DEFAULT_PRIO_M,
					 dscp);
	}

	/* In case DSCP remapping is enabled, we need to write the DSCP to
	 * priority mapping table.
	 */
	reg += dscp / per_reg;
	ret = ksz_read8(dev, reg, &data);
	if (ret)
		return ret;

	shift = (dscp % per_reg) * (8 / per_reg);

	return (data >> shift) & mask;
}

/**
 * ksz_set_global_dscp_entry - Sets the global DSCP-to-priority mapping entry
 * @dev: Pointer to the KSZ switch device structure
 * @dscp: DSCP value for which to set the priority
 * @ipm: Priority value to set
 *
 * This function sets the global DSCP-to-priority mapping entry for the
 * specified DSCP value.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ksz_set_global_dscp_entry(struct ksz_device *dev, u8 dscp, u8 ipm)
{
	int reg, per_reg, shift;
	u8 mask;

	ksz_get_dscp_prio_reg(dev, &reg, &per_reg, &mask);

	shift = (dscp % per_reg) * (8 / per_reg);

	return ksz_rmw8(dev, reg + (dscp / per_reg), mask << shift,
			ipm << shift);
}

/**
 * ksz_init_global_dscp_map - Initializes the global DSCP-to-priority mapping
 * @dev: Pointer to the KSZ switch device structure
 *
 * This function initializes the global DSCP-to-priority mapping table for the
 * switch.
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz_init_global_dscp_map(struct ksz_device *dev)
{
	int ret, dscp;

	/* On KSZ9xxx variants, DSCP remapping is disabled by default.
	 * Enable to have, predictable and reproducible behavior across
	 * different devices.
	 */
	if (!is_ksz8(dev)) {
		ret = ksz_rmw8(dev, KSZ9477_REG_SW_MAC_TOS_CTRL,
			       KSZ9477_SW_TOS_DSCP_REMAP,
			       KSZ9477_SW_TOS_DSCP_REMAP);
		if (ret)
			return ret;
	}

	for (dscp = 0; dscp < DSCP_MAX; dscp++) {
		int ipm, tt;

		/* Map DSCP to Traffic Type, which is corresponding to the
		 * Internal Priority Map (IPM) in the switch.
		 */
		if (!is_ksz8(dev)) {
			ipm = ietf_dscp_to_ieee8021q_tt(dscp);
		} else {
			/* On KSZ8xxx variants we do not have IPM to queue
			 * remapping table. We need to convert DSCP to Traffic
			 * Type and then to queue.
			 */
			tt = ietf_dscp_to_ieee8021q_tt(dscp);
			if (tt < 0)
				return tt;

			ipm = ieee8021q_tt_to_tc(tt, dev->info->num_tx_queues);
		}

		if (ipm < 0)
			return ipm;

		ret = ksz_set_global_dscp_entry(dev, dscp, ipm);
	}

	return 0;
}

/**
 * ksz_port_add_dscp_prio - Adds a DSCP-to-priority mapping entry for a port on
 *			    a KSZ switch.
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to add the DSCP-to-priority mapping entry
 * @dscp: DSCP value for which to add the priority
 * @prio: Priority value to set
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_port_add_dscp_prio(struct dsa_switch *ds, int port, u8 dscp, u8 prio)
{
	struct ksz_device *dev = ds->priv;

	if (prio >= dev->info->num_ipms)
		return -ERANGE;

	return ksz_set_global_dscp_entry(dev, dscp, prio);
}

/**
 * ksz_port_del_dscp_prio - Deletes a DSCP-to-priority mapping entry for a port
 *			    on a KSZ switch.
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to delete the DSCP-to-priority mapping entry
 * @dscp: DSCP value for which to delete the priority
 * @prio: Priority value to delete
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_port_del_dscp_prio(struct dsa_switch *ds, int port, u8 dscp, u8 prio)
{
	struct ksz_device *dev = ds->priv;
	int ipm;

	if (ksz_port_get_dscp_prio(ds, port, dscp) != prio)
		return 0;

	if (is_ksz8(dev)) {
		ipm = ieee8021q_tt_to_tc(IEEE8021Q_TT_BE,
					 dev->info->num_tx_queues);
		if (ipm < 0)
			return ipm;
	} else {
		ipm = IEEE8021Q_TT_BE;
	}

	return ksz_set_global_dscp_entry(dev, dscp, ipm);
}

/**
 * ksz_apptrust_error - Prints an error message for an invalid apptrust selector
 * @dev: Pointer to the KSZ switch device structure
 *
 * This function prints an error message when an invalid apptrust selector is
 * provided.
 */
static void ksz_apptrust_error(struct ksz_device *dev)
{
	char supported_apptrust_variants[64];
	int i;

	supported_apptrust_variants[0] = '\0';
	for (i = 0; i < ARRAY_SIZE(ksz_supported_apptrust_variants); i++) {
		if (i > 0)
			strlcat(supported_apptrust_variants, ", ",
				sizeof(supported_apptrust_variants));
		strlcat(supported_apptrust_variants,
			ksz_supported_apptrust_variants[i],
			sizeof(supported_apptrust_variants));
	}

	dev_err(dev->dev, "Invalid apptrust selector or priority order. Supported: %s\n",
		supported_apptrust_variants);
}

/**
 * ksz_port_set_apptrust_validate - Validates the apptrust selectors
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to set the apptrust selectors
 * @sel: Array of apptrust selectors to validate
 * @nsel: Number of apptrust selectors in the array
 *
 * This function validates the apptrust selectors provided and ensures that
 * they are in the correct order.
 *
 * This family of switches supports two apptrust selectors: DCB_APP_SEL_PCP and
 * IEEE_8021QAZ_APP_SEL_DSCP. The priority order of the selectors is fixed and
 * cannot be changed. The order is as follows:
 * 1. DCB_APP_SEL_PCP - Priority Code Point selector (highest priority)
 * 2. IEEE_8021QAZ_APP_SEL_DSCP - Differentiated Services Code Point selector
 *   (lowest priority)
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz_port_set_apptrust_validate(struct ksz_device *dev, int port,
					  const u8 *sel, int nsel)
{
	int i, j, found;
	int j_prev = 0;

	/* Iterate through the requested selectors */
	for (i = 0; i < nsel; i++) {
		found = 0;

		/* Check if the current selector is supported by the hardware */
		for (j = 0; j < sizeof(ksz_supported_apptrust); j++) {
			if (sel[i] != ksz_supported_apptrust[j])
				continue;

			found = 1;

			/* Ensure that no higher priority selector (lower index)
			 * precedes a lower priority one
			 */
			if (i > 0 && j <= j_prev)
				goto err_sel_not_vaild;

			j_prev = j;
			break;
		}

		if (!found)
			goto err_sel_not_vaild;
	}

	return 0;

err_sel_not_vaild:
	ksz_apptrust_error(dev);

	return -EINVAL;
}

/**
 * ksz88x3_port1_apptrust_quirk - Quirk for apptrust configuration on Port 1
 *				  of KSZ88x3 devices
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to set the apptrust selectors
 * @reg: Register address for the apptrust configuration
 * @port1_data: Data to set for the apptrust configuration
 *
 * This function implements a quirk for apptrust configuration on Port 1 of
 * KSZ88x3 devices. It ensures that apptrust configuration on Port 1 is not
 * possible if PCP apptrust on Port 2 is disabled. This is because the Port 2
 * seems to be permanently hardwired to PCP classification, so we need to
 * do Port 1 configuration always in agreement with Port 2 configuration.
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz88x3_port1_apptrust_quirk(struct ksz_device *dev, int port,
					int reg, u8 port1_data)
{
	u8 port2_data;
	int ret;

	/* If no apptrust is requested for Port 1, no need to care about Port 2
	 * configuration.
	 */
	if (!(port1_data & (KSZ8_PORT_802_1P_ENABLE | KSZ8_PORT_DIFFSERV_ENABLE)))
		return 0;

	/* We got request to enable any apptrust on Port 1. To make it possible,
	 * we need to enable multiple queues on the switch. If we enable
	 * multiqueue support, PCP classification on Port 2 will be
	 * automatically activated by HW.
	 */
	ret = ksz_pread8(dev, KSZ_PORT_2, reg, &port2_data);
	if (ret)
		return ret;

	/* If KSZ8_PORT_802_1P_ENABLE bit is set on Port 2, the driver showed
	 * the interest in PCP classification on Port 2. In this case,
	 * multiqueue support is enabled and we can enable any apptrust on
	 * Port 1.
	 * If KSZ8_PORT_802_1P_ENABLE bit is not set on Port 2, the PCP
	 * classification on Port 2 is still active, but the driver disabled
	 * multiqueue support and made frame prioritization inactive for
	 * all ports. In this case, we can't enable any apptrust on Port 1.
	 */
	if (!(port2_data & KSZ8_PORT_802_1P_ENABLE)) {
		dev_err(dev->dev, "Not possible to enable any apptrust on Port 1 if PCP apptrust on Port 2 is disabled\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * ksz88x3_port2_apptrust_quirk - Quirk for apptrust configuration on Port 2
 *				  of KSZ88x3 devices
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to set the apptrust selectors
 * @reg: Register address for the apptrust configuration
 * @port2_data: Data to set for the apptrust configuration
 *
 * This function implements a quirk for apptrust configuration on Port 2 of
 * KSZ88x3 devices. It ensures that DSCP apptrust is not working on Port 2 and
 * that it is not possible to disable PCP on Port 2. The only way to disable PCP
 * on Port 2 is to disable multiple queues on the switch.
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz88x3_port2_apptrust_quirk(struct ksz_device *dev, int port,
					int reg, u8 port2_data)
{
	struct dsa_switch *ds = dev->ds;
	u8 port1_data;
	int ret;

	/* First validate Port 2 configuration. DiffServ/DSCP is not working
	 * on this port.
	 */
	if (port2_data & KSZ8_PORT_DIFFSERV_ENABLE) {
		dev_err(dev->dev, "DSCP apptrust is not working on Port 2\n");
		return -EINVAL;
	}

	/* If PCP support is requested, we need to enable all queues on the
	 * switch to make PCP priority working on Port 2.
	 */
	if (port2_data & KSZ8_PORT_802_1P_ENABLE)
		return ksz8_all_queues_split(dev, dev->info->num_tx_queues);

	/* We got request to disable PCP priority on Port 2.
	 * Now, we need to compare Port 2 configuration with Port 1
	 * configuration.
	 */
	ret = ksz_pread8(dev, KSZ_PORT_1, reg, &port1_data);
	if (ret)
		return ret;

	/* If Port 1 has any apptrust enabled, we can't disable multiple queues
	 * on the switch, so we can't disable PCP on Port 2.
	 */
	if (port1_data & (KSZ8_PORT_802_1P_ENABLE | KSZ8_PORT_DIFFSERV_ENABLE)) {
		dev_err(dev->dev, "Not possible to disable PCP on Port 2 if any apptrust is enabled on Port 1\n");
		return -EINVAL;
	}

	/* Now we need to ensure that default priority on Port 1 is set to 0
	 * otherwise we can't disable multiqueue support on the switch.
	 */
	ret = ksz_port_get_default_prio(ds, KSZ_PORT_1);
	if (ret < 0) {
		return ret;
	} else if (ret) {
		dev_err(dev->dev, "Not possible to disable PCP on Port 2 if non zero default priority is set on Port 1\n");
		return -EINVAL;
	}

	/* Port 1 has no apptrust or default priority set and we got request to
	 * disable PCP on Port 2. We can disable multiqueue support to disable
	 * PCP on Port 2.
	 */
	return ksz8_all_queues_split(dev, 1);
}

/**
 * ksz88x3_port_apptrust_quirk - Quirk for apptrust configuration on KSZ88x3
 *			       devices
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to set the apptrust selectors
 * @reg: Register address for the apptrust configuration
 * @data: Data to set for the apptrust configuration
 *
 * This function implements a quirk for apptrust configuration on KSZ88x3
 * devices. It ensures that apptrust configuration on Port 1 and
 * Port 2 is done in agreement with each other.
 *
 * Return: 0 on success, or a negative error code on failure
 */
static int ksz88x3_port_apptrust_quirk(struct ksz_device *dev, int port,
				       int reg, u8 data)
{
	if (port == KSZ_PORT_1)
		return ksz88x3_port1_apptrust_quirk(dev, port, reg, data);
	else if (port == KSZ_PORT_2)
		return ksz88x3_port2_apptrust_quirk(dev, port, reg, data);

	return 0;
}

/**
 * ksz_port_set_apptrust - Sets the apptrust selectors for a port on a KSZ
 *			   switch
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to set the apptrust selectors
 * @sel: Array of apptrust selectors to set
 * @nsel: Number of apptrust selectors in the array
 *
 * This function sets the apptrust selectors for the specified port on a KSZ
 * switch.
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_port_set_apptrust(struct dsa_switch *ds, int port,
			  const u8 *sel, int nsel)
{
	const struct ksz_apptrust_map *map;
	struct ksz_device *dev = ds->priv;
	int reg, i, ret;
	u8 data = 0;
	u8 mask;

	ret = ksz_port_set_apptrust_validate(dev, port, sel, nsel);
	if (ret)
		return ret;

	ksz_get_apptrust_map_and_reg(dev, &map, &reg, &mask);

	for (i = 0; i < nsel; i++) {
		int j;

		for (j = 0; j < ARRAY_SIZE(ksz_supported_apptrust); j++) {
			if (sel[i] != ksz_supported_apptrust[j])
				continue;

			data |= map[j].bit;
			break;
		}
	}

	if (ksz_is_ksz88x3(dev)) {
		ret = ksz88x3_port_apptrust_quirk(dev, port, reg, data);
		if (ret)
			return ret;
	}

	return ksz_prmw8(dev, port, reg, mask, data);
}

/**
 * ksz_port_get_apptrust - Retrieves the apptrust selectors for a port on a KSZ
 *			   switch
 * @ds: Pointer to the DSA switch structure
 * @port: Port number for which to get the apptrust selectors
 * @sel: Array to store the apptrust selectors
 * @nsel: Number of apptrust selectors in the array
 *
 * This function fetches the apptrust selectors for the specified port on a KSZ
 * switch.
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_port_get_apptrust(struct dsa_switch *ds, int port, u8 *sel, int *nsel)
{
	const struct ksz_apptrust_map *map;
	struct ksz_device *dev = ds->priv;
	int reg, i, ret;
	u8 data;
	u8 mask;

	ksz_get_apptrust_map_and_reg(dev, &map, &reg, &mask);

	ret = ksz_pread8(dev, port, reg, &data);
	if (ret)
		return ret;

	*nsel = 0;
	for (i = 0; i < ARRAY_SIZE(ksz_supported_apptrust); i++) {
		if (data & map[i].bit)
			sel[(*nsel)++] = ksz_supported_apptrust[i];
	}

	return 0;
}

/**
 * ksz_dcb_init_port - Initializes the DCB configuration for a port on a KSZ
 * @dev: Pointer to the KSZ switch device structure
 * @port: Port number for which to initialize the DCB configuration
 *
 * This function initializes the DCB configuration for the specified port on a
 * KSZ switch. Particular DCB configuration is set for the port, including the
 * default priority and apptrust selectors.
 * The default priority is set to Best Effort, and the apptrust selectors are
 * set to all supported selectors.
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_dcb_init_port(struct ksz_device *dev, int port)
{
	const u8 ksz_default_apptrust[] = { DCB_APP_SEL_PCP };
	int ret, ipm;

	if (is_ksz8(dev)) {
		ipm = ieee8021q_tt_to_tc(IEEE8021Q_TT_BE,
					 dev->info->num_tx_queues);
		if (ipm < 0)
			return ipm;
	} else {
		ipm = IEEE8021Q_TT_BE;
	}

	/* Set the default priority for the port to Best Effort */
	ret = ksz_port_set_default_prio(dev->ds, port, ipm);
	if (ret)
		return ret;

	return ksz_port_set_apptrust(dev->ds, port, ksz_default_apptrust,
				     ARRAY_SIZE(ksz_default_apptrust));
}

/**
 * ksz_dcb_init - Initializes the DCB configuration for a KSZ switch
 * @dev: Pointer to the KSZ switch device structure
 *
 * This function initializes the DCB configuration for a KSZ switch. The global
 * DSCP-to-priority mapping table is initialized.
 *
 * Return: 0 on success, or a negative error code on failure
 */
int ksz_dcb_init(struct ksz_device *dev)
{
	int ret;

	ret = ksz_init_global_dscp_map(dev);
	if (ret)
		return ret;

	/* Enable 802.1p priority control on Port 2 during switch initialization.
	 * This setup is critical for the apptrust functionality on Port 1, which
	 * relies on the priority settings of Port 2. Note: Port 1 is naturally
	 * configured before Port 2, necessitating this configuration order.
	 */
	if (ksz_is_ksz88x3(dev))
		return ksz_prmw8(dev, KSZ_PORT_2, KSZ8_REG_PORT_1_CTRL_0,
				 KSZ8_PORT_802_1P_ENABLE,
				 KSZ8_PORT_802_1P_ENABLE);

	return 0;
}
