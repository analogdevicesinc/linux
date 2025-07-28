.. SPDX-License-Identifier: GPL-2.0

=======
phylink
=======

Overview
========

phylink is a mechanism to support hot-pluggable networking modules
directly connected to a MAC without needing to re-initialise the
adapter on hot-plug events.

phylink supports conventional phylib-based setups, fixed link setups
and SFP (Small Formfactor Pluggable) modules at present.

Modes of operation
==================

phylink has several modes of operation, which depend on the firmware
settings.

1. PHY mode

   In PHY mode, we use phylib to read the current link settings from
   the PHY, and pass them to the MAC driver.  We expect the MAC driver
   to configure exactly the modes that are specified without any
   negotiation being enabled on the link.

2. Fixed mode

   Fixed mode is the same as PHY mode as far as the MAC driver is
   concerned.

3. In-band mode

   In-band mode is used with 802.3z, SGMII and similar interface modes,
   and we are expecting to use and honor the in-band negotiation or
   control word sent across the serdes channel.

By example, what this means is that:

.. code-block:: none

  &eth {
    phy = <&phy>;
    phy-mode = "sgmii";
  };

does not use in-band SGMII signalling.  The PHY is expected to follow
exactly the settings given to it in its :c:func:`mac_config` function.
The link should be forced up or down appropriately in the
:c:func:`mac_link_up` and :c:func:`mac_link_down` functions.

.. code-block:: none

  &eth {
    managed = "in-band-status";
    phy = <&phy>;
    phy-mode = "sgmii";
  };

uses in-band mode, where results from the PHY's negotiation are passed
to the MAC through the SGMII control word, and the MAC is expected to
acknowledge the control word.  The :c:func:`mac_link_up` and
:c:func:`mac_link_down` functions must not force the MAC side link
up and down.

Rough guide to converting a network driver to sfp/phylink
=========================================================

This guide briefly describes how to convert a network driver from
phylib to the sfp/phylink support.  Please send patches to improve
this documentation.

1. Optionally split the network driver's phylib update function into
   two parts dealing with link-down and link-up. This can be done as
   a separate preparation commit.

   An older example of this preparation can be found in git commit
   fc548b991fb0, although this was splitting into three parts; the
   link-up part now includes configuring the MAC for the link settings.
   Please see :c:func:`mac_link_up` for more information on this.

2. Replace::

	select FIXED_PHY
	select PHYLIB

   with::

	select PHYLINK

   in the driver's Kconfig stanza.

3. Add::

	#include <linux/phylink.h>

   to the driver's list of header files.

4. Add::

	struct phylink *phylink;
	struct phylink_config phylink_config;

   to the driver's private data structure.  We shall refer to the
   driver's private data pointer as ``priv`` below, and the driver's
   private data structure as ``struct foo_priv``.

5. Replace the following functions:

   .. flat-table::
    :header-rows: 1
    :widths: 1 1
    :stub-columns: 0

    * - Original function
      - Replacement function
    * - phy_start(phydev)
      - phylink_start(priv->phylink)
    * - phy_stop(phydev)
      - phylink_stop(priv->phylink)
    * - phy_mii_ioctl(phydev, ifr, cmd)
      - phylink_mii_ioctl(priv->phylink, ifr, cmd)
    * - phy_ethtool_get_wol(phydev, wol)
      - phylink_ethtool_get_wol(priv->phylink, wol)
    * - phy_ethtool_set_wol(phydev, wol)
      - phylink_ethtool_set_wol(priv->phylink, wol)
    * - phy_disconnect(phydev)
      - phylink_disconnect_phy(priv->phylink)

   Please note that some of these functions must be called under the
   rtnl lock, and will warn if not. This will normally be the case,
   except if these are called from the driver suspend/resume paths.

6. Add/replace ksettings get/set methods with:

   .. code-block:: c

	static int foo_ethtool_set_link_ksettings(struct net_device *dev,
						  const struct ethtool_link_ksettings *cmd)
	{
		struct foo_priv *priv = netdev_priv(dev);
	
		return phylink_ethtool_ksettings_set(priv->phylink, cmd);
	}

	static int foo_ethtool_get_link_ksettings(struct net_device *dev,
						  struct ethtool_link_ksettings *cmd)
	{
		struct foo_priv *priv = netdev_priv(dev);
	
		return phylink_ethtool_ksettings_get(priv->phylink, cmd);
	}

7. Replace the call to::

	phy_dev = of_phy_connect(dev, node, link_func, flags, phy_interface);

   and associated code with a call to::

	err = phylink_of_phy_connect(priv->phylink, node, flags);

   For the most part, ``flags`` can be zero; these flags are passed to
   the phy_attach_direct() inside this function call if a PHY is specified
   in the DT node ``node``.

   ``node`` should be the DT node which contains the network phy property,
   fixed link properties, and will also contain the sfp property.

   The setup of fixed links should also be removed; these are handled
   internally by phylink.

   of_phy_connect() was also passed a function pointer for link updates.
   This function is replaced by a different form of MAC updates
   described below in (8).

   Manipulation of the PHY's supported/advertised happens within phylink
   based on the validate callback, see below in (8).

   Note that the driver no longer needs to store the ``phy_interface``,
   and also note that ``phy_interface`` becomes a dynamic property,
   just like the speed, duplex etc. settings.

   Finally, note that the MAC driver has no direct access to the PHY
   anymore; that is because in the phylink model, the PHY can be
   dynamic.

8. Add a :c:type:`struct phylink_mac_ops <phylink_mac_ops>` instance to
   the driver, which is a table of function pointers, and implement
   these functions. The old link update function for
   :c:func:`of_phy_connect` becomes three methods: :c:func:`mac_link_up`,
   :c:func:`mac_link_down`, and :c:func:`mac_config`. If step 1 was
   performed, then the functionality will have been split there.

   It is important that if in-band negotiation is used,
   :c:func:`mac_link_up` and :c:func:`mac_link_down` do not prevent the
   in-band negotiation from completing, since these functions are called
   when the in-band link state changes - otherwise the link will never
   come up.

   The :c:func:`mac_get_caps` method is optional, and if provided should
   return the phylink MAC capabilities that are supported for the passed
   ``interface`` mode. In general, there is no need to implement this method.
   Phylink will use these capabilities in combination with permissible
   capabilities for ``interface`` to determine the allowable ethtool link
   modes.

   The :c:func:`mac_link_state` method is used to read the link state
   from the MAC, and report back the settings that the MAC is currently
   using. This is particularly important for in-band negotiation
   methods such as 1000base-X and SGMII.

   The :c:func:`mac_link_up` method is used to inform the MAC that the
   link has come up. The call includes the negotiation mode and interface
   for reference only. The finalised link parameters are also supplied
   (speed, duplex and flow control/pause enablement settings) which
   should be used to configure the MAC when the MAC and PCS are not
   tightly integrated, or when the settings are not coming from in-band
   negotiation.

   The :c:func:`mac_config` method is used to update the MAC with the
   requested state, and must avoid unnecessarily taking the link down
   when making changes to the MAC configuration.  This means the
   function should modify the state and only take the link down when
   absolutely necessary to change the MAC configuration.  An example
   of how to do this can be found in :c:func:`mvneta_mac_config` in
   ``drivers/net/ethernet/marvell/mvneta.c``.

   For further information on these methods, please see the inline
   documentation in :c:type:`struct phylink_mac_ops <phylink_mac_ops>`.

9. Fill-in the :c:type:`struct phylink_config <phylink_config>` fields with
   a reference to the :c:type:`struct device <device>` associated to your
   :c:type:`struct net_device <net_device>`:

   .. code-block:: c

	priv->phylink_config.dev = &dev.dev;
	priv->phylink_config.type = PHYLINK_NETDEV;

   Fill-in the various speeds, pause and duplex modes your MAC can handle:

   .. code-block:: c

        priv->phylink_config.mac_capabilities = MAC_SYM_PAUSE | MAC_10 | MAC_100 | MAC_1000FD;

10. Some Ethernet controllers work in pair with a PCS (Physical Coding Sublayer)
    block, that can handle among other things the encoding/decoding, link
    establishment detection and autonegotiation. While some MACs have internal
    PCS whose operation is transparent, some other require dedicated PCS
    configuration for the link to become functional. In that case, phylink
    provides a PCS abstraction through :c:type:`struct phylink_pcs <phylink_pcs>`.

    Identify if your driver has one or more internal PCS blocks, and/or if
    your controller can use an external PCS block that might be internally
    connected to your controller.

    If your controller doesn't have any internal PCS, you can go to step 11.

    If your Ethernet controller contains one or several PCS blocks, create
    one :c:type:`struct phylink_pcs <phylink_pcs>` instance per PCS block within
    your driver's private data structure:

    .. code-block:: c

        struct phylink_pcs pcs;

    Populate the relevant :c:type:`struct phylink_pcs_ops <phylink_pcs_ops>` to
    configure your PCS. Create a :c:func:`pcs_get_state` function that reports
    the inband link state, a :c:func:`pcs_config` function to configure your
    PCS according to phylink-provided parameters, and a :c:func:`pcs_validate`
    function that report to phylink all accepted configuration parameters for
    your PCS:

    .. code-block:: c

        struct phylink_pcs_ops foo_pcs_ops = {
                .pcs_validate = foo_pcs_validate,
                .pcs_get_state = foo_pcs_get_state,
                .pcs_config = foo_pcs_config,
        };

    Arrange for PCS link state interrupts to be forwarded into
    phylink, via:

    .. code-block:: c

        phylink_pcs_change(pcs, link_is_up);

    where ``link_is_up`` is true if the link is currently up or false
    otherwise. If a PCS is unable to provide these interrupts, then
    it should set ``pcs->pcs_poll = true;`` when creating the PCS.

11. If your controller relies on, or accepts the presence of an external PCS
    controlled through its own driver, add a pointer to a phylink_pcs instance
    in your driver private data structure:

    .. code-block:: c

        struct phylink_pcs *pcs;

    The way of getting an instance of the actual PCS depends on the platform,
    some PCS sit on an MDIO bus and are grabbed by passing a pointer to the
    corresponding :c:type:`struct mii_bus <mii_bus>` and the PCS's address on
    that bus. In this example, we assume the controller attaches to a Lynx PCS
    instance:

    .. code-block:: c

        priv->pcs = lynx_pcs_create_mdiodev(bus, 0);

    Some PCS can be recovered based on firmware information:

    .. code-block:: c

        priv->pcs = lynx_pcs_create_fwnode(of_fwnode_handle(node));

12. Populate the :c:func:`mac_select_pcs` callback and add it to your
    :c:type:`struct phylink_mac_ops <phylink_mac_ops>` set of ops. This function
    must return a pointer to the relevant :c:type:`struct phylink_pcs <phylink_pcs>`
    that will be used for the requested link configuration:

    .. code-block:: c

        static struct phylink_pcs *foo_select_pcs(struct phylink_config *config,
                                                  phy_interface_t interface)
        {
                struct foo_priv *priv = container_of(config, struct foo_priv,
                                                     phylink_config);

                if ( /* 'interface' needs a PCS to function */ )
                        return priv->pcs;

                return NULL;
        }

    See :c:func:`mvpp2_select_pcs` for an example of a driver that has multiple
    internal PCS.

13. Fill-in all the :c:type:`phy_interface_t <phy_interface_t>` (i.e. all MAC to
    PHY link modes) that your MAC can output. The following example shows a
    configuration for a MAC that can handle all RGMII modes, SGMII and 1000BaseX.
    You must adjust these according to what your MAC and all PCS associated
    with this MAC are capable of, and not just the interface you wish to use:

    .. code-block:: c

       phy_interface_set_rgmii(priv->phylink_config.supported_interfaces);
        __set_bit(PHY_INTERFACE_MODE_SGMII,
                  priv->phylink_config.supported_interfaces);
        __set_bit(PHY_INTERFACE_MODE_1000BASEX,
                  priv->phylink_config.supported_interfaces);

14. Remove calls to of_parse_phandle() for the PHY,
    of_phy_register_fixed_link() for fixed links etc. from the probe
    function, and replace with:

    .. code-block:: c

	struct phylink *phylink;

	phylink = phylink_create(&priv->phylink_config, node, phy_mode, &phylink_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		fail probe;
	}

	priv->phylink = phylink;

    and arrange to destroy the phylink in the probe failure path as
    appropriate and the removal path too by calling:

    .. code-block:: c

	phylink_destroy(priv->phylink);

15. Arrange for MAC link state interrupts to be forwarded into
    phylink, via:

    .. code-block:: c

	phylink_mac_change(priv->phylink, link_is_up);

    where ``link_is_up`` is true if the link is currently up or false
    otherwise.

16. Verify that the driver does not call::

	netif_carrier_on()
	netif_carrier_off()

    as these will interfere with phylink's tracking of the link state,
    and cause phylink to omit calls via the :c:func:`mac_link_up` and
    :c:func:`mac_link_down` methods.

Network drivers should call phylink_stop() and phylink_start() via their
suspend/resume paths, which ensures that the appropriate
:c:type:`struct phylink_mac_ops <phylink_mac_ops>` methods are called
as necessary.

For information describing the SFP cage in DT, please see the binding
documentation in the kernel source tree
``Documentation/devicetree/bindings/net/sff,sfp.yaml``.
