# Linux Network Driver Template

Reference driver: `drivers/net/ethernet/adi/adin1110.c` (SPI-based 10BASE-T1L
MAC-PHY)

This template covers every file needed to add a new Linux network driver for an
Analog Devices Ethernet MAC, PHY, or SPI-Ethernet adapter. Replace `<devname>`
with the part number (e.g., `adin1110`) and `<DEVNAME>` with its uppercase form
(e.g., `ADIN1110`) throughout.

---

## 1. Purpose & Subsystem Mapping

Linux network drivers integrate with the **net** subsystem through the
`net_device` and `ethtool_ops` interfaces. The core abstractions are:

| Concept | Kernel Interface | Header |
|---|---|---|
| Network device | `struct net_device` | `<linux/netdevice.h>` |
| Device operations | `struct net_device_ops` | `<linux/netdevice.h>` |
| ethtool interface | `struct ethtool_ops` | `<linux/ethtool.h>` |
| Socket buffers | `struct sk_buff` | `<linux/skbuff.h>` |
| PHY abstraction | `struct phy_device` | `<linux/phy.h>` |
| PHY link mgmt | phylink / phylib | `<linux/phylink.h>`, `<linux/phy.h>` |
| NAPI polling | `struct napi_struct` | `<linux/netdevice.h>` |
| MDIO bus | `struct mii_bus` | `<linux/mii.h>` |

**Device categories covered:**

- **Ethernet MAC** -- standalone MAC requiring an external PHY (connected via
  MDIO/RGMII/RMII). Uses phylink or phylib for PHY management.
- **MAC-PHY combo** -- integrated MAC + PHY on a single chip (e.g., ADIN1110).
  The driver registers an internal MDIO bus and connects to the embedded PHY.
- **SPI-Ethernet adapter** -- MAC-PHY accessed entirely over SPI. Frame TX/RX
  and register access all go through SPI transfers. Cannot use DMA-based ring
  buffers; typically uses threaded IRQs and work queues.
- **PHY driver** -- standalone PHY driver under `drivers/net/phy/`. Implements
  `struct phy_driver` with config/read_status/suspend/resume callbacks.

---

## 2. File Checklist

```
drivers/net/ethernet/adi/
    Kconfig                         # Kconfig entry (or extend existing)
    Makefile                        # Build rule
    <devname>.c                     # Driver source

drivers/net/phy/
    <devname>.c                     # PHY driver (if standalone PHY)

Documentation/devicetree/bindings/net/
    adi,<devname>.yaml              # DT binding schema

MAINTAINERS                        # Add entry for new files
```

For a MAC-PHY combo (like ADIN1110), the driver lives under
`drivers/net/ethernet/adi/` and the PHY portion is a separate driver under
`drivers/net/phy/`.

---

## 3. Devicetree Binding (`.yaml`)

Path: `Documentation/devicetree/bindings/net/adi,<devname>.yaml`

```yaml
# SPDX-License-Identifier: (GPL-2.0 OR BSD-2-Clause)
%YAML 1.2
---
$id: http://devicetree.org/schemas/net/adi,<devname>.yaml#
$schema: http://devicetree.org/meta-schemas/core.yaml#

title: ADI <DEVNAME> Ethernet MAC-PHY

maintainers:
  - Your Name <your.name@analog.com>

description: |
  The <DEVNAME> is a low power 10BASE-T1L Ethernet MAC-PHY designed
  for industrial Ethernet applications. It integrates an Ethernet PHY
  core with a MAC and all associated analog circuitry.

  The device has a 4-wire SPI interface for communication between
  the MAC and host processor.

allOf:
  - $ref: ethernet-controller.yaml#
  - $ref: /schemas/spi/spi-peripheral-props.yaml#

properties:
  compatible:
    enum:
      - adi,<devname>

  reg:
    maxItems: 1
    description: SPI chip-select number.

  interrupts:
    maxItems: 1
    description: Active-low interrupt from the device to the host.

  reset-gpios:
    maxItems: 1
    description: GPIO connected to active low reset pin.

  local-mac-address:
    description: 6-byte MAC address assigned to this interface.

  adi,spi-crc:
    description: Enable CRC8 checks on SPI read/writes.
    type: boolean

  # For devices with an external PHY:
  phy-handle:
    description: Phandle to the external PHY node.

  phy-mode:
    description: |
      PHY connection type. Typical values for ADI devices:
      "internal" (MAC-PHY combo), "rgmii", "rmii", "mii", "sgmii".

  # SPI bus properties (inherited from spi-peripheral-props.yaml):
  spi-max-frequency:
    description: Maximum SPI clock frequency in Hz.

  spi-cpha:
    description: SPI clock phase.

  spi-cpol:
    description: SPI clock polarity.

required:
  - compatible
  - reg
  - interrupts

unevaluatedProperties: false

examples:
  - |
    #include <dt-bindings/interrupt-controller/irq.h>

    spi {

        #address-cells = <1>;
        #size-cells = <0>;

        ethernet@0 {
            compatible = "adi,<devname>";
            reg = <0>;
            spi-max-frequency = <24500000>;

            adi,spi-crc;

            interrupt-parent = <&gpio>;
            interrupts = <25 IRQ_TYPE_LEVEL_LOW>;

            local-mac-address = [ 00 11 22 33 44 55 ];
        };
    };
```

### Binding notes

- Always reference `ethernet-controller.yaml#` via `allOf` for standard
  ethernet properties (`local-mac-address`, `mac-address`, `phy-handle`,
  `phy-mode`).
- For SPI-attached devices, also reference `spi-peripheral-props.yaml#` to
  inherit `spi-max-frequency`, `spi-cpha`, `spi-cpol`.
- Use `phy-handle` when the device connects to an external PHY. For MAC-PHY
  combos with an internal PHY, omit `phy-handle` and set
  `phy-mode = "internal"` (or omit it).
- Validate bindings with `make dt_binding_check` and
  `make dtbs_check DT_SCHEMA_FILES=...`.

---

## 4. Kconfig

Path: `drivers/net/ethernet/adi/Kconfig`

The ADI vendor Kconfig is sourced from `drivers/net/ethernet/Kconfig` via:

```
source "drivers/net/ethernet/adi/Kconfig"
```

A new device entry is added under the `NET_VENDOR_ADI` menu:

```kconfig
# SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
#
# Analog Devices device configuration
#

config NET_VENDOR_ADI
	bool "Analog Devices devices"
	default y
	depends on SPI
	help
	  If you have a network (Ethernet) card belonging to this class, say Y.

	  Note that the answer to this question doesn't directly affect the
	  kernel: saying N will just cause the configurator to skip all
	  the questions about ADI devices. If you say Y, you will be asked
	  for your specific card in the following questions.

if NET_VENDOR_ADI

config <DEVNAME>
	tristate "Analog Devices <DEVNAME> MAC-PHY"
	depends on SPI && NET_SWITCHDEV
	select CRC8
	select PHYLIB
	help
	  Say yes here to build support for Analog Devices <DEVNAME>
	  Low Power 10BASE-T1L Ethernet MAC-PHY.

endif # NET_VENDOR_ADI
```

### Kconfig conventions

- `tristate` allows the driver to be built as a module (`=m`) or built-in
  (`=y`).
- `depends on SPI` -- use the appropriate bus dependency (`SPI`, `PCI`,
  `PLATFORM`).
- `select CRC8` / `select PHYLIB` -- select library dependencies rather than
  depending on them, so the user does not need to manually enable them.
- If the device does not use switchdev, omit `NET_SWITCHDEV`.
- For a standalone PHY driver, place the config in
  `drivers/net/phy/Kconfig` and depend on `PHYLIB`.

---

## 5. Makefile

Path: `drivers/net/ethernet/adi/Makefile`

```makefile
# SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
#
# Makefile for the Analog Devices network device drivers.
#

obj-$(CONFIG_<DEVNAME>) += <devname>.o
```

For multi-file drivers:

```makefile
obj-$(CONFIG_<DEVNAME>) += <devname>-drv.o
<devname>-drv-y := <devname>_main.o <devname>_ethtool.o <devname>_hw.o
```

The parent `drivers/net/ethernet/Makefile` must include:

```makefile
obj-$(CONFIG_NET_VENDOR_ADI) += adi/
```

---

## 6. Driver Source (`.c`)

Path: `drivers/net/ethernet/adi/<devname>.c`

### 6.1 Header and includes

```c
// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* <DEVNAME> Low Power 10BASE-T1L Ethernet MAC-PHY
 *
 * Copyright YYYY Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
```

### 6.2 Private data structures

```c
struct <devname>_priv {
	struct spi_device	*spidev;
	struct net_device	*netdev;
	struct mii_bus		*mii_bus;
	struct phy_device	*phydev;
	struct mutex		lock;		/* protect SPI access */
	struct work_struct	tx_work;
	struct sk_buff_head	txq;
	u8			data[2048] ____cacheline_aligned;
	int			irq;
};
```

### 6.3 `net_device_ops`

The `net_device_ops` structure defines the core callbacks for the network
interface:

```c
static int <devname>_net_open(struct net_device *netdev)
{
	struct <devname>_priv *priv = netdev_priv(netdev);

	/* Enable device interrupts. */
	/* Start PHY: */
	phy_start(priv->phydev);

	netif_start_queue(netdev);

	return 0;
}

static int <devname>_net_stop(struct net_device *netdev)
{
	struct <devname>_priv *priv = netdev_priv(netdev);

	netif_stop_queue(netdev);
	flush_work(&priv->tx_work);
	phy_stop(priv->phydev);

	return 0;
}

static netdev_tx_t <devname>_start_xmit(struct sk_buff *skb,
					 struct net_device *netdev)
{
	struct <devname>_priv *priv = netdev_priv(netdev);

	/* Validate frame size, check TX space. */
	/* Queue SKB for transmission: */
	skb_queue_tail(&priv->txq, skb);
	schedule_work(&priv->tx_work);

	return NETDEV_TX_OK;
}

static int <devname>_set_mac_address(struct net_device *netdev, void *addr)
{
	struct sockaddr *sa = addr;
	int ret;

	ret = eth_prepare_mac_addr_change(netdev, addr);
	if (ret < 0)
		return ret;

	eth_hw_addr_set(netdev, sa->sa_data);

	/* Program the new MAC address into the hardware filter. */
	return 0;
}

static const struct net_device_ops <devname>_netdev_ops = {
	.ndo_open		= <devname>_net_open,
	.ndo_stop		= <devname>_net_stop,
	.ndo_start_xmit		= <devname>_start_xmit,
	.ndo_set_mac_address	= <devname>_set_mac_address,
	.ndo_set_rx_mode	= <devname>_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_eth_ioctl		= phy_do_ioctl,
	.ndo_get_stats64	= <devname>_get_stats64,
};
```

### 6.4 `ethtool_ops`

```c
static void <devname>_get_drvinfo(struct net_device *dev,
				  struct ethtool_drvinfo *di)
{
	strscpy(di->driver, "<DEVNAME>", sizeof(di->driver));
	strscpy(di->bus_info, dev_name(dev->dev.parent),
		sizeof(di->bus_info));
}

static const struct ethtool_ops <devname>_ethtool_ops = {
	.get_drvinfo		= <devname>_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};
```

### 6.5 SKB handling (TX work queue)

For SPI-based devices, frame transmission must happen in process context
(SPI transfers can sleep). Use a work queue:

```c
static void <devname>_tx_work(struct work_struct *work)
{
	struct <devname>_priv *priv = container_of(work,
						   struct <devname>_priv,
						   tx_work);
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&priv->txq))) {
		/* Write frame to device TX FIFO via SPI. */
		/* On success: */
		priv->netdev->stats.tx_packets++;
		priv->netdev->stats.tx_bytes += skb->len;
		dev_kfree_skb(skb);
	}
}
```

### 6.6 NAPI polling (RX)

For high-throughput MACs, use NAPI to batch receive processing:

```c
static int <devname>_poll(struct napi_struct *napi, int budget)
{
	struct <devname>_priv *priv = container_of(napi,
						   struct <devname>_priv,
						   napi);
	int work_done = 0;

	while (work_done < budget) {
		struct sk_buff *skb;
		int frame_len;

		/* Read frame from device RX FIFO. */
		/* If no more frames: */
		break;

		skb = netdev_alloc_skb(priv->netdev, frame_len + NET_IP_ALIGN);
		if (!skb)
			break;

		skb_reserve(skb, NET_IP_ALIGN);
		/* Copy frame data into skb->data. */
		skb_put(skb, frame_len);
		skb->protocol = eth_type_trans(skb, priv->netdev);
		netif_receive_skb(skb);

		priv->netdev->stats.rx_packets++;
		priv->netdev->stats.rx_bytes += frame_len;
		work_done++;
	}

	if (work_done < budget)
		napi_complete_done(napi, work_done);

	return work_done;
}
```

For simpler SPI-based MAC-PHYs (like ADIN1110), a threaded IRQ handler is
used instead of NAPI:

```c
static irqreturn_t <devname>_irq(int irq, void *dev_id)
{
	struct <devname>_priv *priv = dev_id;

	/* Read status register. */
	/* Process RX frames. */
	/* Process TX completion. */

	return IRQ_HANDLED;
}
```

### 6.7 Network device allocation and registration (probe)

```c
static int <devname>_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct <devname>_priv *priv;
	struct net_device *netdev;
	int ret;

	/* Allocate net_device with private data. */
	netdev = devm_alloc_etherdev(dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);
	priv->spidev = spi;
	priv->netdev = netdev;
	SET_NETDEV_DEV(netdev, dev);

	mutex_init(&priv->lock);
	INIT_WORK(&priv->tx_work, <devname>_tx_work);
	skb_queue_head_init(&priv->txq);

	/* Read MAC address from DT or EEPROM. */
	ret = device_get_ethdev_address(dev, netdev);
	if (ret < 0) {
		/* Generate random MAC if none available: */
		eth_hw_addr_random(netdev);
	}

	/* Assign operations. */
	netdev->netdev_ops = &<devname>_netdev_ops;
	netdev->ethtool_ops = &<devname>_ethtool_ops;

	/* Hardware initialization (reset, verify chip ID). */

	/* Register MDIO bus (for MAC-PHY combos). */
	ret = <devname>_register_mdiobus(priv, dev);
	if (ret < 0)
		return ret;

	/* Connect PHY. */
	priv->phydev = phy_connect(netdev, phydev_name(priv->phydev),
				   <devname>_adjust_link,
				   PHY_INTERFACE_MODE_INTERNAL);
	if (IS_ERR(priv->phydev))
		return PTR_ERR(priv->phydev);

	/* Request IRQ (threaded for SPI). */
	ret = devm_request_threaded_irq(dev, spi->irq, NULL,
					<devname>_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					dev_name(dev), priv);
	if (ret < 0)
		return ret;

	netif_carrier_off(netdev);

	/* Register network device. */
	ret = devm_register_netdev(dev, netdev);
	if (ret < 0) {
		dev_err(dev, "Failed to register network device\n");
		return ret;
	}

	return 0;
}
```

### 6.8 Module boilerplate

```c
static const struct of_device_id <devname>_match_table[] = {
	{ .compatible = "adi,<devname>" },
	{ }
};
MODULE_DEVICE_TABLE(of, <devname>_match_table);

static const struct spi_device_id <devname>_spi_id[] = {
	{ .name = "<devname>", .driver_data = 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, <devname>_spi_id);

static struct spi_driver <devname>_driver = {
	.driver = {
		.name = "<devname>",
		.of_match_table = <devname>_match_table,
	},
	.probe = <devname>_probe,
	.id_table = <devname>_spi_id,
};
module_spi_driver(<devname>_driver);

MODULE_DESCRIPTION("<DEVNAME> Ethernet MAC-PHY driver");
MODULE_AUTHOR("Your Name <your.name@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");
```

For platform-bus (non-SPI) devices, use `platform_driver` and
`module_platform_driver()` instead.

---

## 7. PHY Integration

### 7.1 phylib (simple PHY management)

phylib is the simpler option, suitable when the MAC connects to a single PHY
with fixed interface type:

```c
/* In probe: */
priv->phydev = phy_connect(netdev, phydev_name(phydev),
			   <devname>_adjust_link,
			   PHY_INTERFACE_MODE_INTERNAL);

/* Link change callback: */
static void <devname>_adjust_link(struct net_device *dev)
{
	struct phy_device *phydev = dev->phydev;

	if (!phydev->link)
		phy_print_status(phydev);
}

/* In ndo_open: */
phy_start(priv->phydev);

/* In ndo_stop: */
phy_stop(priv->phydev);

/* Cleanup: */
phy_disconnect(priv->phydev);
```

### 7.2 phylink (advanced PHY/SFP management)

phylink is preferred for MACs that support multiple PHY interface modes or
SFP cages:

```c
static const struct phylink_mac_ops <devname>_phylink_ops = {
	.mac_config	= <devname>_mac_config,
	.mac_link_up	= <devname>_mac_link_up,
	.mac_link_down	= <devname>_mac_link_down,
};

/* In probe: */
priv->phylink = phylink_create(&priv->phylink_config, dev_fwnode(dev),
			       phy_mode, &<devname>_phylink_ops);

phylink_connect_phy(priv->phylink, priv->phydev);

/* In ndo_open: */
phylink_start(priv->phylink);

/* In ndo_stop: */
phylink_stop(priv->phylink);
```

### 7.3 Internal MDIO bus (MAC-PHY combos)

When the PHY is embedded in the MAC (like ADIN1110), the driver must register
an MDIO bus to allow the PHY subsystem to discover the internal PHY:

```c
static int <devname>_register_mdiobus(struct <devname>_priv *priv,
				      struct device *dev)
{
	struct mii_bus *mii_bus;
	int ret;

	mii_bus = devm_mdiobus_alloc(dev);
	if (!mii_bus)
		return -ENOMEM;

	mii_bus->name = "<devname>-mdio";
	mii_bus->read = <devname>_mdio_read;
	mii_bus->write = <devname>_mdio_write;
	mii_bus->priv = priv;
	mii_bus->parent = dev;
	mii_bus->phy_mask = ~((u32)GENMASK(2, 0));
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s",
		 dev_name(dev));

	ret = devm_mdiobus_register(dev, mii_bus);
	if (ret)
		return ret;

	priv->mii_bus = mii_bus;

	return 0;
}
```

---

## 8. Devicetree Parsing

### 8.1 MAC address

The kernel provides helpers that check DT properties (`local-mac-address`,
`mac-address`) and fall back to NVMEM/EEPROM cells:

```c
ret = device_get_ethdev_address(dev, netdev);
if (ret < 0) {
	/* No MAC in DT or EEPROM -- generate a random one: */
	eth_hw_addr_random(netdev);
	dev_info(dev, "Using random MAC address\n");
}
```

### 8.2 PHY connection

```c
/* For external PHY via phy-handle in DT: */
priv->phydev = of_phy_connect(netdev, phy_node,
			      <devname>_adjust_link, 0,
			      PHY_INTERFACE_MODE_RGMII);

/* For internal PHY in a MAC-PHY combo: */
priv->phydev = get_phy_device(priv->mii_bus, phy_addr, false);
if (IS_ERR(priv->phydev))
	return PTR_ERR(priv->phydev);

priv->phydev = phy_connect(netdev, phydev_name(priv->phydev),
			   <devname>_adjust_link,
			   PHY_INTERFACE_MODE_INTERNAL);
```

### 8.3 Other DT properties

```c
/* Boolean property: */
priv->use_crc = device_property_read_bool(dev, "adi,spi-crc");

/* Integer property: */
ret = device_property_read_u32(dev, "adi,rx-fifo-depth",
			       &priv->rx_fifo_depth);

/* GPIO: */
priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
					    GPIOD_OUT_LOW);
```

---

## 9. Test & Debug

### 9.1 ethtool

```bash
# Show driver info:
ethtool -i eth0

# Show link status and speed:
ethtool eth0

# Show NIC statistics:
ethtool -S eth0

# Show link settings:
ethtool -s eth0 speed 10 duplex full autoneg off
```

### 9.2 ip / iproute2

```bash
# Bring interface up:
ip link set eth0 up

# Assign IP address:
ip addr add 192.168.1.10/24 dev eth0

# Show interface details:
ip -d link show eth0

# Set MAC address:
ip link set eth0 address 00:11:22:33:44:55
```

### 9.3 netdev sysfs

```bash
# Carrier state:
cat /sys/class/net/eth0/carrier

# Operational state:
cat /sys/class/net/eth0/operstate

# Speed:
cat /sys/class/net/eth0/speed

# Statistics:
cat /sys/class/net/eth0/statistics/rx_packets
cat /sys/class/net/eth0/statistics/tx_packets
cat /sys/class/net/eth0/statistics/rx_errors
```

### 9.4 Dynamic debug

```bash
# Enable debug messages for the driver:
echo 'module <devname> +p' > /sys/kernel/debug/dynamic_debug/control

# Enable netdev debug messages:
echo 'file drivers/net/ethernet/adi/<devname>.c +p' > \
    /sys/kernel/debug/dynamic_debug/control
```

### 9.5 Common test scenarios

- **Link up/down**: connect/disconnect cable, verify `carrier` changes.
- **Ping test**: verify basic connectivity with `ping`.
- **iperf**: measure throughput with `iperf3 -s` / `iperf3 -c <ip>`.
- **Stress**: run `ping -f` (flood) and check for dropped packets.
- **PHY loopback**: enable loopback via `ethtool --set-phy-tunable` or
  driver-specific register writes, then verify TX frames are received back.
- **Suspend/resume**: test `echo mem > /sys/power/state` and verify the
  interface recovers.

---

## 10. Key Conventions

1. **License** -- use `GPL-2.0` or `GPL-2.0 OR BSD-2-Clause` (dual license).
   The SPDX identifier goes in the first line of every source file. Module
   license must be `"GPL"` or `"Dual BSD/GPL"`.

2. **NAPI** -- use NAPI for receive-side processing on high-throughput MACs.
   For low-speed SPI-based MAC-PHYs, a threaded IRQ handler reading frames
   directly is acceptable.

3. **Logging** -- use `netdev_err()`, `netdev_warn()`, `netdev_info()`,
   `netdev_dbg()` instead of `dev_err()` for messages associated with a
   `net_device`. Use `dev_err()` / `dev_info()` in probe before the netdev
   is registered.

4. **Managed resources (`devm_*`)** -- prefer `devm_alloc_etherdev()`,
   `devm_register_netdev()`, `devm_request_threaded_irq()`,
   `devm_mdiobus_alloc()`, `devm_mdiobus_register()`, and
   `devm_gpiod_get_optional()`. These free resources automatically when the
   device is unbound.

5. **SKB lifecycle** -- `dev_kfree_skb()` after successful TX.
   `netdev_alloc_skb()` for RX. Never leak SKBs.

6. **Locking** -- use a mutex to serialize SPI transfers (SPI can sleep).
   Use `netif_stop_queue()` / `netif_wake_queue()` for TX flow control.
   Use a spinlock only for state that is accessed from hard IRQ context.

7. **Carrier management** -- call `netif_carrier_off()` during probe (link
   is unknown). The PHY state machine or `adjust_link` callback will call
   `netif_carrier_on()` / `netif_carrier_off()` as the link changes.

8. **MAC address** -- use `device_get_ethdev_address()` to read from DT,
   fall back to `eth_hw_addr_random()`. Use `eth_hw_addr_set()` to program
   a new address.

9. **Statistics** -- implement `ndo_get_stats64` for 64-bit packet/byte
   counters. Maintain per-CPU or per-port counters as needed.

10. **Error handling** -- all kernel functions should check return values.
    Use managed resources to avoid complex error cleanup in probe. Log
    errors with `netdev_err()` / `dev_err()` before returning.

---

## 11. Commit Message Format

Network driver commits use the `net:` or `net: ethernet:` prefix. Follow
the netdev commit message conventions:

```
net: ethernet: adi: add <DEVNAME> driver support

Add support for the Analog Devices <DEVNAME>, a low power 10BASE-T1L
Ethernet MAC-PHY with SPI interface designed for industrial Ethernet
applications.

The driver implements net_device_ops for frame TX/RX, registers an
internal MDIO bus for the embedded PHY, and uses threaded IRQs for
SPI-based interrupt handling.

Signed-off-by: Your Name <your.name@analog.com>
```

### Prefix conventions

| Change type | Prefix |
|---|---|
| New MAC driver | `net: ethernet: adi: add ...` |
| MAC driver fix | `net: ethernet: adi: fix ...` |
| New PHY driver | `net: phy: add ...` |
| PHY driver fix | `net: phy: fix ...` |
| DT binding | `dt-bindings: net: add ...` |
| ethtool change | `net: ethtool: ...` |

### Submitting to netdev

- Network patches go to `netdev@vger.kernel.org`.
- Use `net` tree for bug fixes and `net-next` tree for new features.
- Include DT binding, Kconfig, Makefile, and driver in a patch series with
  the binding patch first.
- Run `scripts/checkpatch.pl` on all patches before submitting.
- Test with `make W=1` to catch extra warnings.
