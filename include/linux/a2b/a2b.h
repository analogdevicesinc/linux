/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef A2B_H
#define A2B_H

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/poll.h>

/**
 * enum a2b_pin_drive_strength - Drive strength for the output pins
 * @A2B_PIN_DRIVE_STRENGTH_LOW: Use low current output driver
 * @A2B_PIN_DRIVE_STRENGTH_HIGH: Use high current output driver
 */
enum a2b_pin_drive_strength {
	A2B_PIN_DRIVE_STRENGTH_LOW,
	A2B_PIN_DRIVE_STRENGTH_HIGH
};

/**
 * struct a2b_transceiver_config - Configuration for the transceivers in the A2B chip
 * @a_power: Transmit power setting for the A port
 * @b_power: Transmit power setting for the B port
 * @invert: Whether a non-crossover cable is used
 */
struct a2b_transceiver_config {
	uint8_t a_power;
	uint8_t b_power;
	bool invert;
};

/**
 * struct a2b_slot_config - Slot configuration for an A2B subnode
 * @dn_rx_slots: Downstream receive mask
 * @dn_n_tx_slots: Number of transmitted downstream slots
 * @dn_n_forward_slots: Number of forwarded downstream slots
 * @up_rx_slots: Upstream receive mask
 * @up_n_tx_slots: Number of transmitted upstream slots
 * @up_n_forward_slots: Number of forwarded upstream slots
 */
struct a2b_slot_config {
	unsigned int dn_rx_slots;
	unsigned int dn_n_tx_slots;
	unsigned int dn_n_forward_slots;
	unsigned int up_rx_slots;
	unsigned int up_n_tx_slots;
	unsigned int up_n_forward_slots;
};

/**
 * struct a2b_tdm_config - A2B node TDM interface configuration
 * @tdm_channels: Number of TDM channels per frame
 * @tdm_slot_size: TDM interface slot size in bits (either 16 or 32)
 * @alt_sync: Whether to 50/50 SYNC signal or pulsed SYNC signal
 * @early_sync: Whether to assert SYNC 1 clock cycle before BCLK (I2S mode)
 * @invert_sync: Whether SYNC is active low
 */
struct a2b_tdm_config {
	unsigned int tdm_channels;
	unsigned int tdm_slot_size;

	bool alt_sync : 1;
	bool early_sync : 1;
	bool invert_sync : 1;
};

/**
 * enum a2b_spread_spectrum_mode - A2B node PLL spread spectrum mode config
 * @A2B_SPREAD_SPECTRUM_NONE: No spread spectrum clocking
 * @A2B_SPREAD_SPECTRUM_A2B: Spread spectrum clocking for the A2B bus
 * @A2B_SPREAD_SPECTRUM_A2B_I2S: Spread spectrum clocking for the A2B and I2S
 *							   buses
 */
enum a2b_spread_spectrum_mode {
	A2B_SPREAD_SPECTRUM_NONE,
	A2B_SPREAD_SPECTRUM_A2B,
	A2B_SPREAD_SPECTRUM_A2B_I2S,
};

/**
 * struct a2b_pll_config - A2B node PLL configuration
 * @spread_spectrum_mode: Spread spectrum mode
 * @spread_spectrum_high: Whether to use high or low spectrum depth of
 *						modulation
 * @spread_spectrum_frequency: Frequency of the spread spectrum modulation
 */
struct a2b_pll_config {
	enum a2b_spread_spectrum_mode spread_spectrum_mode;
	bool spread_spectrum_high;
	uint8_t spread_spectrum_frequency;
};

/**
 * struct a2b_node - A2B node state struct
 * @id: ID of the device on the bus, A2B_MAINNODE_ID for mainnodes
 * @dev: Device driver framework base struct
 * @regmap: Regmap that can be used to access the node's registers
 * @label: Unique symbolic name to identify the node
 * @transceiver_config: A2B transceiver configuration for the node
 * @tdm_config: TDM interface configuration for the node
 * @pll_config: PLL configuration for the node
 * @pin_config: Pin output strength configuration for the node
 * @vendor_id: Cached value of the VENDOR_ID register
 * @product_id: Cached value of the PRODUCT_ID register
 * @version_id: Cached value of the VERSION_ID register
 * @capability_id: Cached value of the CAPABILITY_ID register
 */
struct a2b_node {
	unsigned int id;

	struct device dev;

	struct regmap *regmap;

	const char *label;

	struct a2b_transceiver_config transceiver_config;
	struct a2b_tdm_config tdm_config;
	struct a2b_pll_config pll_config;
	enum a2b_pin_drive_strength pin_config;

	uint8_t vendor_id;
	uint8_t product_id;
	uint8_t version_id;
	uint8_t capability_id;

	wait_queue_head_t mbox_pollq;
};

#define A2B_MAX_NODES 9
#define A2B_MAIN_NODE_ID 0xff

struct a2b_subnode;

/**
 * struct a2b_bus_config - A2B bus slot configuration
 * @up_slot_size: A2B upstream slot size in bits (8, 12, 16, 20, 24, 28, 32)
 * @dn_slot_size: A2B downstream slot size in bits (8, 12, 16, 20, 24, 28, 32)
 * @up_slot_alt_fmt: Use alternative format for A2B upstream slots
 * @dn_slot_alt_fmt: Use alternative format for A2B downstream slots
 */
struct a2b_bus_config {
	unsigned int up_slot_size;
	unsigned int dn_slot_size;
	bool up_slot_alt_fmt;
	bool dn_slot_alt_fmt;
};

/**
 * struct a2b_mainnode - A2B mainnode state struct
 * @node: A2B node base structure
 * @bus_id: Globally unique ID for the bus
 * @bus_lock: Lock for accessing bus_client
 * @bus_client: I2C client to access the bus interface of the A2B node
 * @num_subnodes: Number of subnodes attached to this node
 * @subnodes: Array of attached subnodes
 * @run_completion: Will be triggered when the mainnode is running
 * @discover_completion: Will be triggered when the discovery of a node is
 *					   complete
 * @irq: IRQ used by the node, 0 if no IRQ is available
 * @sync_clk: SYNC clock
 * @bus_config: A2B bus slot config
 */
struct a2b_mainnode {
	struct a2b_node node;
	unsigned int bus_id;

	struct mutex bus_lock;
	struct i2c_client *bus_client;

	unsigned int num_subnodes;
	struct a2b_subnode *subnodes[A2B_MAX_NODES];

	struct completion run_completion;
	struct completion discover_completion;

	int irq;

	struct clk *sync_clk;

	struct a2b_bus_config bus_config;
};

/**
 * struct a2b_subnode - A2B subnode state struct
 * @node: A2B node base structure
 * @slot_config: Slot configuration for the A2B subnode
 * @mainnode: A2B mainnode that the subnode is attached to
 */
struct a2b_subnode {
	struct a2b_node node;
	struct a2b_slot_config slot_config;

	struct a2b_mainnode *mainnode;
};

/**
 * a2b_node_to_mainnode() - Cast a struct a2b_node to struct a2b_mainnode
 * @node: The node to cast
 *
 * Return: The a2b_mainnode struct
 */
static inline struct a2b_mainnode *a2b_node_to_mainnode(struct a2b_node *node)
{
	return container_of(node, struct a2b_mainnode, node);
}

/**
 * a2b_node_to_subnode() - Cast a struct a2b_node to struct a2b_subnode
 * @node: The node to cast
 *
 * Return: The a2b_subnode struct
 */
static inline struct a2b_subnode *a2b_node_to_subnode(struct a2b_node *node)
{
	return container_of(node, struct a2b_subnode, node);
}

/**
 * dev_to_a2b_node() - Cast a struct device to a struct a2b_node
 * @dev: The device to cast
 *
 * Return: The a2b_node struct
 */
static inline struct a2b_node *dev_to_a2b_node(struct device *dev)
{
	return container_of(dev, struct a2b_node, dev);
}

/**
 * struct a2b_device_id_table - A2B device ID table
 * @vendor: VENDOR_ID register value to match
 * @product: PRODUCT_ID register value to match
 * @product_mask: Mask to apply to the PRODUCT_ID register when matching
 *
 * This struct is used to match an A2B node device to a driver based on the
 * VENDOR_ID and PRODUCT_ID registers.
 */
struct a2b_device_id_table {
	u8 vendor;
	u8 product;
	u8 product_mask;
};

/**
 * struct a2b_subnode_driver - A2B subnode driver
 * @id_table: Device ID match table for this driver
 * @regmap_config: Regmap config that will be used by subnodes bound to this
 *				 driver
 * @cells: Array of MFD cells that will be instantiated when a subnode is bound
 *		 to the driver
 * @num_cells: Number of entries in the @cells array
 */
struct a2b_subnode_driver {
	struct device_driver driver;

	const struct a2b_device_id_table *id_table;

	const struct regmap_config *regmap_config;
	const struct mfd_cell *cells;
	unsigned int num_cells;
};

int a2b_driver_register(struct module *owner,
			struct a2b_subnode_driver *driver);
void a2b_driver_unregister(struct a2b_subnode_driver *driver);

#define __a2b_driver_register(driver) a2b_driver_register(THIS_MODULE, driver)

#define module_a2b_driver(__a2b_driver)                                        \
	module_driver(__a2b_driver, __a2b_driver_register,                     \
		      a2b_driver_unregister)

struct a2b_subnode *a2b_subnode_alloc(struct a2b_mainnode *mainnode,
				      uint8_t id);
void a2b_subnode_free(struct a2b_subnode *subnode);
struct regmap *a2b_subnode_regmap_init(struct a2b_subnode *subnode,
				       const struct regmap_config *config);

int a2b_node_read(struct a2b_node *node, uint8_t reg);
int a2b_node_write(struct a2b_node *node, uint8_t reg, uint8_t val);

int a2b_init_irq(struct a2b_mainnode *mainnode);
int a2b_wait_for_irq(struct a2b_mainnode *mainnode,
		     struct completion *completion, unsigned int timeout);

int a2b_of_mainnode_read_config(struct a2b_mainnode *mainnode);
int a2b_of_bus_enumerate(struct a2b_mainnode *mainnode);

int a2b_of_read_transceiver_config(struct a2b_node *node);
int a2b_of_read_tdm_config(struct a2b_node *node);
int a2b_of_read_pll_config(struct a2b_node *node);
int a2b_of_read_pin_config(struct a2b_node *node);

int a2b_node_init_extra_regmap(struct a2b_node *node);

struct a2b_mainnode *devm_a2b_mainnode_register(
	struct i2c_client *parent, const struct regmap_config *regmap_config,
	const struct mfd_cell *cells, unsigned int num_cells);

extern const struct attribute_group *a2b_attr_groups[];
extern const struct attribute_group *a2b_mainnode_attr_groups[];

/**
 * a2b_node_is_main() - Whether the node is a mainnode
 * @node: The node
 *
 * Return: true if the node is mainnode, false otherwise
 */
static inline bool a2b_node_is_main(struct a2b_node *node)
{
	return node->id == A2B_MAIN_NODE_ID;
}

/**
 * a2b_node_has_capability() - Check whether a node supports a certain
 *							 capability
 * @node: The node
 *
 * Return: true if the capability is supported, false otherwise
 */
static inline bool a2b_node_has_capability(struct a2b_node *node,
					   unsigned int cap)
{
	return node->capability_id & cap;
}

/**
 * a2b_node_get_mainnode() - Get the mainnode for an A2B node
 * @node: The node
 *
 * For mainnodes this will return the node itself, for subnodes this will return
 * the mainnode that the subnode is attached to.
 *
 * Return: The mainnode for the node
 */
static struct a2b_mainnode *a2b_node_get_mainnode(struct a2b_node *node)
{
	if (a2b_node_is_main(node))
		return a2b_node_to_mainnode(node);
	else
		return a2b_node_to_subnode(node)->mainnode;
}

/**
 * a2b_node_get_bus_id() - Get the bus ID for a node
 * @node: The node
 *
 * Return: The bus ID for the node
 */
static inline int a2b_node_get_bus_id(struct a2b_node *node)
{
	return a2b_node_get_mainnode(node)->bus_id;
}

/**
 * a2b_node_has_irq() - Check whether IRQs are supported for a node
 * @node: The node
 *
 * Return: true if the node has IRQ support, false otherwise
 */
static inline bool a2b_node_has_irq(struct a2b_node *node)
{
	return a2b_node_get_mainnode(node)->irq > 0;
}

/**
 * a2b_node_get_sync_clk() - Get the SYNC clock for a node
 * @node: The node
 *
 * Return: The SYNC clock for the node
 */
static inline struct clk *a2b_node_get_sync_clk(struct a2b_node *node)
{
	return a2b_node_get_mainnode(node)->sync_clk;
}

#endif
