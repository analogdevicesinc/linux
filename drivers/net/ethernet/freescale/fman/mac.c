// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-or-later
/*
 * Copyright 2008 - 2015 Freescale Semiconductor Inc.
 * Copyright 2020 Puresoftware Ltd.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/device.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/phy_fixed.h>
#include <linux/phylink.h>
#include <linux/etherdevice.h>
#include <linux/libfdt_env.h>
#include <linux/platform_device.h>

#include "mac.h"
#include "fman_mac.h"
#include "fman_dtsec.h"
#include "fman_tgec.h"
#include "fman_memac.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL FMan MAC API based driver");

struct mac_priv_s {
	u8				cell_index;
	struct fman			*fman;
	/* List of multicast addresses */
	struct list_head		mc_addr_list;
	struct platform_device		*eth_dev;
	u16				speed;
};

struct mac_address {
	u8 addr[ETH_ALEN];
	struct list_head list;
};

static void mac_exception(struct mac_device *mac_dev,
			  enum fman_mac_exceptions ex)
{
	if (ex == FM_MAC_EX_10G_RX_FIFO_OVFL) {
		/* don't flag RX FIFO after the first */
		mac_dev->set_exception(mac_dev->fman_mac,
				       FM_MAC_EX_10G_RX_FIFO_OVFL, false);
		dev_err(mac_dev->dev, "10G MAC got RX FIFO Error = %x\n", ex);
	}

	dev_dbg(mac_dev->dev, "%s:%s() -> %d\n", KBUILD_BASENAME ".c",
		__func__, ex);
}

int fman_set_multi(struct net_device *net_dev, struct mac_device *mac_dev)
{
	struct mac_priv_s	*priv;
	struct mac_address	*old_addr, *tmp;
	struct netdev_hw_addr	*ha;
	int			err;
	enet_addr_t		*addr;

	priv = mac_dev->priv;

	/* Clear previous address list */
	list_for_each_entry_safe(old_addr, tmp, &priv->mc_addr_list, list) {
		addr = (enet_addr_t *)old_addr->addr;
		err = mac_dev->remove_hash_mac_addr(mac_dev->fman_mac, addr);
		if (err < 0)
			return err;

		list_del(&old_addr->list);
		kfree(old_addr);
	}

	/* Add all the addresses from the new list */
	netdev_for_each_mc_addr(ha, net_dev) {
		addr = (enet_addr_t *)ha->addr;
		err = mac_dev->add_hash_mac_addr(mac_dev->fman_mac, addr);
		if (err < 0)
			return err;

		tmp = kmalloc(sizeof(*tmp), GFP_ATOMIC);
		if (!tmp)
			return -ENOMEM;

		ether_addr_copy(tmp->addr, ha->addr);
		list_add(&tmp->list, &priv->mc_addr_list);
	}
	return 0;
}

static DEFINE_MUTEX(eth_lock);

static struct platform_device *dpaa_eth_add_device(int fman_id,
						   struct mac_device *mac_dev)
{
	struct platform_device *pdev;
	struct dpaa_eth_data data;
	struct mac_priv_s	*priv;
	static int dpaa_eth_dev_cnt;
	int ret;

	priv = mac_dev->priv;

	data.mac_dev = mac_dev;
	data.mac_hw_id = priv->cell_index;
	data.fman_hw_id = fman_id;

	mutex_lock(&eth_lock);
	pdev = platform_device_alloc("dpaa-ethernet", dpaa_eth_dev_cnt);
	if (!pdev) {
		ret = -ENOMEM;
		goto no_mem;
	}

	pdev->dev.parent = mac_dev->dev;

	ret = platform_device_add_data(pdev, &data, sizeof(data));
	if (ret)
		goto err;

	ret = platform_device_add(pdev);
	if (ret)
		goto err;

	dpaa_eth_dev_cnt++;
	mutex_unlock(&eth_lock);

	return pdev;

err:
	platform_device_put(pdev);
no_mem:
	mutex_unlock(&eth_lock);

	return ERR_PTR(ret);
}

static const struct of_device_id mac_match[] = {
	{ .compatible   = "fsl,fman-dtsec", .data = dtsec_initialization },
	{ .compatible   = "fsl,fman-xgec", .data = tgec_initialization },
	{ .compatible	= "fsl,fman-memac", .data = memac_initialization },
	{}
};
MODULE_DEVICE_TABLE(of, mac_match);

static int fwnode_match_devnode(struct device *dev, const void *fwnode)
{
	return dev->fwnode == fwnode;
}

static int acpi_mac_probe(struct platform_device *pdev)
{
	int			err, i, nph;
	int (*init)(struct mac_device *mac_dev, struct fman_mac_params *params);
	struct device		*dev;
	struct mac_device	*mac_dev;
	struct mac_priv_s	*priv;
	u32			val;
	u8			fman_id;
	phy_interface_t		phy_if;
	struct device		*fman_dev = NULL;
	struct fwnode_handle	*fman_fwnode = NULL;
	struct device		*fman_port_dev = NULL;
	/* firmware node references */
	struct fwnode_reference_args args;
	struct fman_mac_params	 params;

	dev = &pdev->dev;

	init = device_get_match_data(dev);

	mac_dev = devm_kzalloc(dev, sizeof(*mac_dev), GFP_KERNEL);
	if (!mac_dev) {
		err = -ENOMEM;
		goto _return;
	}
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto _return;
	}

	/* Save private information */
	mac_dev->priv = priv;
	mac_dev->dev = dev;

	INIT_LIST_HEAD(&priv->mc_addr_list);

	/* Get the FM node */
	fman_fwnode = fwnode_get_parent(dev->fwnode);
	if (!fman_fwnode) {
		err = -EINVAL;
		dev_err(dev, "%s : fetch fman node failed\n", __func__);
		goto _return;
	}
	if (fwnode_property_read_u32(fman_fwnode, "cell-index", &val)) {
		err = -EINVAL;
		goto _return;
	}

	/* cell-index 0 => FMan id 1 */
	fman_id = (u8)(val + 1);

	fman_dev = bus_find_device(&platform_bus_type, NULL, fman_fwnode,
				   fwnode_match_devnode);
	if (!fman_dev) {
		dev_err(dev, "%s : bus_find_device failed\n", __func__);
		err = -ENODEV;
		goto _return;
	}

	priv->fman = fman_bind(fman_dev);
	if (!priv->fman) {
		dev_err(dev, "%s : fman_bind failed\n", __func__);
		err = -ENODEV;
		goto _return;
	}

	/* Get the address of the memory mapped registers */
	mac_dev->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mac_dev->res) {
		dev_err(dev, "%s: Can't get MAC memory resource\n",  __func__);
		err = -EINVAL;
		goto _return;
	}
	dev_dbg(dev, "MAC : %s : IORESOURCE [%llx] size [%llx]\n",
		__func__, mac_dev->res->start, resource_size(mac_dev->res));

	mac_dev->vaddr = ioremap(mac_dev->res->start, resource_size(mac_dev->res));
	if (!mac_dev->vaddr) {
		dev_err(dev, "%s : ioremap() failed\n", __func__);
		err = -EIO;
		goto _return;
	}

	if (!fwnode_device_is_available(dev->fwnode)) {
		err = -ENODEV;
		goto _return;
	}

	/* Get the cell-index */
	if (device_property_read_u32(dev, "cell-index", &val)) {
		dev_err(dev, "%s : failed to read cell-index\n", __func__);
		err = -EINVAL;
		goto _return;
	}
	priv->cell_index = (u8)val;

	/* Get the MAC address */
	device_get_mac_address(dev, mac_dev->addr);

	/* Get the port handles */
	nph = device_property_count_u32(dev, "fsl,fman-ports");

	if (unlikely(nph < 0)) {
		dev_err(dev, "%s : reading port count failed\n", __func__);
		err = nph;
		goto _return;
	}

	if (nph != ARRAY_SIZE(mac_dev->port)) {
		dev_err(dev, "Not supported number of fman-ports handles of mac node from DSD property\n");
		err = -EINVAL;
		goto _return;
	}

	for (i = 0; i < ARRAY_SIZE(mac_dev->port); i++) {
		/* Find the port node */
		struct fwnode_handle *fw_node =
			acpi_fwnode_handle(ACPI_COMPANION(dev));
		err = acpi_node_get_property_reference(fw_node, "fsl,fman-ports",
						       i, &args);
		if (ACPI_FAILURE(err) ||
		    !is_acpi_device_node(args.fwnode)) {
			dev_err(dev, "%s : reading fsl,fman-ports handle failed\n",
				__func__);
			goto _return;
		}

		/* Bind to a specific FMan Port */
		fman_port_dev = bus_find_device(&platform_bus_type, NULL,
						args.fwnode,
						fwnode_match_devnode);
		if (!fman_port_dev) {
			dev_err(dev, "%s : bus_find_device failed\n", __func__);
			err = -ENODEV;
			goto _return;
		}

		mac_dev->port[i] = fman_port_bind(fman_port_dev);
	}

	/* Get the PHY connection type */
	phy_if = fwnode_get_phy_mode(dev->fwnode);
	if (phy_if < 0) {
		dev_warn(dev, "fwnode_get_phy_mode failed. Defaulting to SGMII\n");
		phy_if = PHY_INTERFACE_MODE_SGMII;
	}
	mac_dev->phy_if = phy_if;

	params.mac_id		= priv->cell_index;
	params.fm		= (void *)priv->fman;
	params.exception_cb	= mac_exception;
	params.event_cb		= mac_exception;

	err = init(mac_dev, &params);
	if (err < 0) {
		dev_err(dev, "%s: mac_dev->init() = %d\n", __func__, err);
		goto _return;
	}

	if (!is_zero_ether_addr(mac_dev->addr))
		dev_info(dev, "FMan MAC address: %pM\n", mac_dev->addr);

	priv->eth_dev = dpaa_eth_add_device(fman_id, mac_dev);
	if (IS_ERR(priv->eth_dev)) {
		dev_err(dev, "%s : failed to add Ethernet platform device for MAC %d\n",
			__func__, priv->cell_index);
		priv->eth_dev = NULL;
	}

	goto _return;

_return:
	return err;
}

static const struct acpi_device_id mac_acpi_match[] = {
	{ .id = "NXP0025",
	  .driver_data = (kernel_ulong_t)memac_initialization },
	{},
};
MODULE_DEVICE_TABLE(acpi, mac_acpi_match);

static int mac_probe(struct platform_device *_of_dev)
{
	int			 err, i, nph;
	int (*init)(struct mac_device *mac_dev, struct fman_mac_params *params);
	struct device		*dev;
	struct device_node	*mac_node, *dev_node;
	struct mac_device	*mac_dev;
	struct platform_device	*of_dev;
	struct mac_priv_s	*priv;
	struct fman_mac_params	 params;
	u32			 val;
	u8			fman_id;
	phy_interface_t          phy_if;
	const char		*managed;

	phy_if = PHY_INTERFACE_MODE_NA;
	dev = &_of_dev->dev;
	mac_node = dev->of_node;
	init = of_device_get_match_data(dev);

	if (is_acpi_node(dev->fwnode))
		return acpi_mac_probe(_of_dev);

	mac_dev = devm_kzalloc(dev, sizeof(*mac_dev), GFP_KERNEL);
	if (!mac_dev)
		return -ENOMEM;
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	platform_set_drvdata(_of_dev, mac_dev);

	/* Save private information */
	mac_dev->priv = priv;
	mac_dev->dev = dev;

	INIT_LIST_HEAD(&priv->mc_addr_list);

	/* Get the FM node */
	dev_node = of_get_parent(mac_node);
	if (!dev_node) {
		dev_err(dev, "of_get_parent(%pOF) failed\n",
			mac_node);
		return -EINVAL;
	}

	of_dev = of_find_device_by_node(dev_node);
	if (!of_dev) {
		dev_err(dev, "of_find_device_by_node(%pOF) failed\n", dev_node);
		err = -EINVAL;
		goto _return_of_node_put;
	}
	mac_dev->fman_dev = &of_dev->dev;

	/* Get the FMan cell-index */
	err = of_property_read_u32(dev_node, "cell-index", &val);
	if (err) {
		dev_err(dev, "failed to read cell-index for %pOF\n", dev_node);
		err = -EINVAL;
		goto _return_dev_put;
	}
	/* cell-index 0 => FMan id 1 */
	fman_id = (u8)(val + 1);

	priv->fman = fman_bind(mac_dev->fman_dev);
	if (!priv->fman) {
		dev_err(dev, "fman_bind(%pOF) failed\n", dev_node);
		err = -ENODEV;
		goto _return_dev_put;
	}

	/* Two references have been taken in of_find_device_by_node()
	 * and fman_bind(). Release one of them here. The second one
	 * will be released in mac_remove().
	 */
	put_device(mac_dev->fman_dev);
	of_node_put(dev_node);
	dev_node = NULL;

	/* Get the address of the memory mapped registers */
	mac_dev->res = platform_get_mem_or_io(_of_dev, 0);
	if (!mac_dev->res) {
		dev_err(dev, "could not get registers\n");
		err = -EINVAL;
		goto _return_dev_put;
	}

	err = devm_request_resource(dev, fman_get_mem_region(priv->fman),
				    mac_dev->res);
	if (err) {
		dev_err_probe(dev, err, "could not request resource\n");
		goto _return_dev_put;
	}

	mac_dev->vaddr = devm_ioremap(dev, mac_dev->res->start,
				      resource_size(mac_dev->res));
	if (!mac_dev->vaddr) {
		dev_err(dev, "devm_ioremap() failed\n");
		err = -EIO;
		goto _return_dev_put;
	}

	if (!of_device_is_available(mac_node)) {
		err = -ENODEV;
		goto _return_dev_put;
	}

	/* Get the cell-index */
	err = of_property_read_u32(mac_node, "cell-index", &val);
	if (err) {
		dev_err(dev, "failed to read cell-index for %pOF\n", mac_node);
		err = -EINVAL;
		goto _return_dev_put;
	}
	priv->cell_index = (u8)val;

	/* Get the MAC address */
	err = of_get_mac_address(mac_node, mac_dev->addr);
	if (err)
		dev_warn(dev, "of_get_mac_address(%pOF) failed\n", mac_node);

	/* Get the port handles */
	nph = of_count_phandle_with_args(mac_node, "fsl,fman-ports", NULL);
	if (unlikely(nph < 0)) {
		dev_err(dev, "of_count_phandle_with_args(%pOF, fsl,fman-ports) failed\n",
			mac_node);
		err = nph;
		goto _return_dev_put;
	}

	if (nph != ARRAY_SIZE(mac_dev->port)) {
		dev_err(dev, "Not supported number of fman-ports handles of mac node %pOF from device tree\n",
			mac_node);
		err = -EINVAL;
		goto _return_dev_put;
	}

	/* PORT_NUM determines the size of the port array */
	for (i = 0; i < PORT_NUM; i++) {
		/* Find the port node */
		dev_node = of_parse_phandle(mac_node, "fsl,fman-ports", i);
		if (!dev_node) {
			dev_err(dev, "of_parse_phandle(%pOF, fsl,fman-ports) failed\n",
				mac_node);
			err = -EINVAL;
			goto _return_dev_arr_put;
		}

		of_dev = of_find_device_by_node(dev_node);
		if (!of_dev) {
			dev_err(dev, "of_find_device_by_node(%pOF) failed\n",
				dev_node);
			err = -EINVAL;
			goto _return_dev_arr_put;
		}
		mac_dev->fman_port_devs[i] = &of_dev->dev;

		mac_dev->port[i] = fman_port_bind(mac_dev->fman_port_devs[i]);
		if (!mac_dev->port[i]) {
			dev_err(dev, "dev_get_drvdata(%pOF) failed\n",
				dev_node);
			err = -EINVAL;
			goto _return_dev_arr_put;
		}
		/* Two references have been taken in of_find_device_by_node()
		 * and fman_port_bind(). Release one of them here. The second
		 * one will be released in mac_remove().
		 */
		put_device(mac_dev->fman_port_devs[i]);
		of_node_put(dev_node);
		dev_node = NULL;
	}

	/* Get the PHY connection type, except for C73 managed links where we
	 * let phylink select state->interface, regardless of what's in the
	 * device tree.
	 *
	 * Although, U-Boot's board_ft_fman_fixup_port() will "fix up" the
	 * device tree and force a fixed-link on 10G ports, unless we use
	 * phy-connection-type = "10gbase-kr". So we do expect to find device
	 * trees with this phy-connection-type value, yet we still deliberately
	 * ignore it.
	 */
	if (of_property_read_string(mac_node, "managed", &managed) != 0 ||
	    strcmp(managed, "c73") != 0) {
		err = of_get_phy_mode(mac_node, &phy_if);
		if (err) {
			dev_warn(dev,
				 "of_get_phy_mode() for %pOF failed. Defaulting to SGMII\n",
				 mac_node);
			phy_if = PHY_INTERFACE_MODE_SGMII;
		}
	}
	mac_dev->phy_if = phy_if;

	params.mac_id		= priv->cell_index;
	params.fm		= (void *)priv->fman;
	params.exception_cb	= mac_exception;
	params.event_cb		= mac_exception;

	err = init(mac_dev, &params);
	if (err < 0)
		goto _return_dev_arr_put;

	if (!is_zero_ether_addr(mac_dev->addr))
		dev_info(dev, "FMan MAC address: %pM\n", mac_dev->addr);

	priv->eth_dev = dpaa_eth_add_device(fman_id, mac_dev);
	if (IS_ERR(priv->eth_dev)) {
		err = PTR_ERR(priv->eth_dev);
		dev_err(dev, "failed to add Ethernet platform device for MAC %d\n",
			priv->cell_index);
		priv->eth_dev = NULL;
	}

	return err;

_return_dev_arr_put:
	/* mac_dev is kzalloc'ed */
	for (i = 0; i < PORT_NUM; i++)
		put_device(mac_dev->fman_port_devs[i]);
_return_dev_put:
	put_device(mac_dev->fman_dev);
_return_of_node_put:
	of_node_put(dev_node);
	return err;
}

static void mac_remove(struct platform_device *pdev)
{
	struct mac_device *mac_dev = platform_get_drvdata(pdev);
	int		   i;

	for (i = 0; i < PORT_NUM; i++)
		put_device(mac_dev->fman_port_devs[i]);
	put_device(mac_dev->fman_dev);

	platform_device_unregister(mac_dev->priv->eth_dev);
}

static struct platform_driver mac_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= mac_match,
		.acpi_match_table = ACPI_PTR(mac_acpi_match),
	},
	.probe		= mac_probe,
	.remove_new	= mac_remove,
};

builtin_platform_driver(mac_driver);
