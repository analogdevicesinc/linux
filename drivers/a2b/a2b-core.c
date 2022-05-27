// SPDX-License-Identifier: GPL-2.0-only

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/i2c.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

static DEFINE_IDA(a2b_mainnode_ida);

static struct a2b_mainnode *dev_to_a2b_mainnode(struct device *dev)
{
	return container_of(dev, struct a2b_mainnode, node.dev);
}

static struct a2b_subnode *dev_to_a2b_subnode(struct device *dev)
{
	return container_of(dev, struct a2b_subnode, node.dev);
}

static struct a2b_subnode_driver *
driver_to_a2b_subnode_driver(struct device_driver *driver)
{
	return container_of(driver, struct a2b_subnode_driver, driver);
}

static void a2b_subnode_release(struct device *dev)
{
	struct a2b_subnode *subnode = dev_to_a2b_subnode(dev);

	kfree(subnode);
}

static struct device_type a2b_subnode_type = {
	.name = "a2b-sub-node",
	.release = a2b_subnode_release,
};

static int a2b_match(struct device *dev, struct device_driver *driver)
{
	struct a2b_subnode_driver *subnode_driver;
	const struct a2b_device_id_table *id;
	struct a2b_subnode *subnode;

	if (dev->type != &a2b_subnode_type)
		return 0;

	subnode = dev_to_a2b_subnode(dev);
	subnode_driver = driver_to_a2b_subnode_driver(driver);

	if (!subnode_driver->id_table)
		return 0;

	for (id = subnode_driver->id_table; id->vendor != 0; id++) {
		if (id->vendor == subnode->node.vendor_id &&
		    (id->product & id->product_mask) ==
			    (subnode->node.product_id & id->product_mask))
			return 1;
	}

	return 0;
}

static int a2b_probe(struct device *dev)
{
	struct a2b_subnode_driver *subnode_driver;
	struct a2b_subnode *subnode;

	if (dev->type != &a2b_subnode_type)
		return 0;

	subnode = dev_to_a2b_subnode(dev);
	subnode_driver = driver_to_a2b_subnode_driver(dev->driver);

	subnode->node.regmap =
		a2b_subnode_regmap_init(subnode, subnode_driver->regmap_config);
	if (IS_ERR(subnode->node.regmap))
		return PTR_ERR(subnode->node.regmap);

	return mfd_add_hotplug_devices(dev, subnode_driver->cells,
				       subnode_driver->num_cells);
}

static int a2b_remove(struct device *dev)
{
	struct a2b_subnode *subnode;

	if (dev->type != &a2b_subnode_type)
		return 0;

	subnode = dev_to_a2b_subnode(dev);

	mfd_remove_devices(dev);
	regmap_exit(subnode->node.regmap);
	subnode->node.regmap = NULL;

	return 0;
}

static struct bus_type a2b_bus = {
	.name = "a2b",
	.match = a2b_match,
	.probe = a2b_probe,
	.remove = a2b_remove,
	.dev_groups = a2b_attr_groups,
};

/**
 * a2b_subnode_alloc() - Allocates a new subnode
 * @mainnode: The mainnode the subnode is attached to
 * @id: Subnode ID
 *
 * a2b_subnode_free() must be used to free a subnode allocated by this function.
 *
 * Return: The newly allocated subnode or NULL on allocation failure
 */
struct a2b_subnode *a2b_subnode_alloc(struct a2b_mainnode *mainnode, uint8_t id)
{
	struct a2b_subnode *subnode;

	subnode = kzalloc(sizeof(*subnode), GFP_KERNEL);
	if (!subnode)
		return NULL;

	subnode->mainnode = mainnode;
	subnode->node.id = id;

	device_initialize(&subnode->node.dev);
	dev_set_name(&subnode->node.dev, "a2b-%d.%d", mainnode->bus_id, id);

	subnode->node.dev.parent = &mainnode->node.dev;
	subnode->node.dev.bus = &a2b_bus;
	subnode->node.dev.type = &a2b_subnode_type;
	init_waitqueue_head(&subnode->node.mbox_pollq);

	return subnode;
}

/**
 * a2b_subnode_free() - Free an A2B subnode
 * @subnode: The subnode to free
 *
 * Free an A2B subnode that has been allocated using a2b_subnode_alloc()
 */
void a2b_subnode_free(struct a2b_subnode *subnode)
{
	put_device(&subnode->node.dev);
}

static void a2b_mainnode_release(struct device *dev)
{
	struct a2b_mainnode *mainnode = dev_to_a2b_mainnode(dev);

	ida_simple_remove(&a2b_mainnode_ida, mainnode->bus_id);
	kfree(mainnode);
}

static struct device_type a2b_mainnode_type = {
	.name = "a2b-main-node",
	.release = a2b_mainnode_release,
	.groups = a2b_mainnode_attr_groups,
};

/**
 * struct a2b_mainnode_slot_config - Mainnode A2B slot configuration
 * @respcycs_up_min: Response cycle minimum on the upstream bus
 * @respcycs_dn_max: Response cycle maximum on the downstream bus
 * @up_slots: Number of upstream slots transmitted by the mainnode
 * @dn_slots: Number of downstream slots received by the mainnode
 * @up_enabled: Whether the upstream bus is enabled
 * @dn_enabled: Whether the downstream bus is enabled
 */
struct a2b_mainnode_slot_config {
	unsigned int respcycs_up_min;
	unsigned int respcycs_dn_max;

	unsigned int up_slots;
	unsigned int dn_slots;

	bool up_enabled;
	bool dn_enabled;
};

/* See Table 3-2 in the datasheet */
static unsigned int a2b_bus_bits(unsigned int slot_size, bool alt_fmt)
{
	int alt_bits[8] = { 0, 13, 17, 21, 30, 0, 39, 0 };
	int idx = A2B_SLOTFMT_DNSIZE(slot_size);

	return alt_fmt ? alt_bits[idx] : slot_size + 1;
}

static void
a2b_calc_mainnode_slot_config(struct a2b_mainnode *mainnode,
			      struct a2b_mainnode_slot_config *config)
{
	struct a2b_tdm_config *tdm_config = &mainnode->node.tdm_config;
	unsigned int respcycs_up_min, respcycs_dn_max;
	struct a2b_slot_config *slot_config;
	unsigned int respoffs;
	unsigned int i;

	respcycs_up_min = UINT_MAX;
	respcycs_dn_max = 0;

	/* See Table 9-1 in the TRM */
	if (tdm_config->tdm_channels == 2 && tdm_config->tdm_slot_size == 16)
		respoffs = 238;
	else if ((tdm_config->tdm_channels == 2 &&
		  tdm_config->tdm_slot_size == 32) ||
		 (tdm_config->tdm_channels == 4 &&
		  tdm_config->tdm_slot_size == 16))
		respoffs = 245;
	else
		respoffs = 248;

	config->dn_enabled = false;
	config->up_enabled = false;
	config->up_slots = 0;
	config->dn_slots = 0;

	for (i = 0; i < mainnode->num_subnodes; i++) {
		unsigned int dnslot_activity, upslot_activity;
		unsigned int respcycs_dn, respcycs_up;
		unsigned int dn_slots, up_slots;

		slot_config = &mainnode->subnodes[i]->slot_config;

		/* See section 3-18 in the TRM */
		dn_slots = slot_config->dn_n_forward_slots +
			   slot_config->dn_n_tx_slots;
		up_slots = slot_config->up_n_forward_slots +
			   slot_config->up_n_tx_slots;

		/* See Appendix B in the TRM */
		dnslot_activity =
			dn_slots *
			a2b_bus_bits(mainnode->bus_config.dn_slot_size,
				     mainnode->bus_config.dn_slot_alt_fmt);
		upslot_activity =
			up_slots *
			a2b_bus_bits(mainnode->bus_config.up_slot_size,
				     mainnode->bus_config.up_slot_alt_fmt);

		respcycs_dn = DIV_ROUND_UP(64 + dnslot_activity, 4) + 4 * i + 2;
		respcycs_up =
			respoffs - (DIV_ROUND_UP(64 + upslot_activity, 4) + 1);

		respcycs_dn_max = max(respcycs_dn_max, respcycs_dn);
		respcycs_up_min = min(respcycs_up_min, respcycs_up);

		if (dn_slots > 0)
			config->dn_enabled = true;

		if (up_slots > 0)
			config->up_enabled = true;

		if (i == 0) {
			config->up_slots = up_slots;
			config->dn_slots = dn_slots;
		}
	}

	config->respcycs_dn_max = respcycs_dn_max;
	config->respcycs_up_min = respcycs_up_min;
}

static int a2b_node_read_id(struct a2b_node *node)
{
	int val;

	val = a2b_node_read(node, A2B_PRODUCT);
	if (val < 0)
		return val;
	node->product_id = val;

	val = a2b_node_read(node, A2B_VENDOR);
	if (val < 0)
		return val;
	node->vendor_id = val;

	val = a2b_node_read(node, A2B_VERSION);
	if (val < 0)
		return val;
	node->version_id = val;

	val = a2b_node_read(node, A2B_CAPABILITY);
	if (val < 0)
		return val;
	node->capability_id = val;

	return 0;
}
/*
 * Sets the configuration that needs to be written before the bus is enabled.
 */
static int a2b_subnode_set_config(struct a2b_subnode *node)
{
	struct a2b_slot_config *cfg = &node->slot_config;
	unsigned int up_rx_slots;
	unsigned int dn_rx_slots;
	unsigned int i;
	int ret;

	/* Slot config */
	ret = a2b_node_write(&node->node, A2B_UPSLOTS, cfg->up_n_forward_slots);
	if (ret < 0)
		return ret;

	ret = a2b_node_write(&node->node, A2B_DNSLOTS, cfg->dn_n_forward_slots);
	if (ret < 0)
		return ret;

	ret = a2b_node_write(&node->node, A2B_LUPSLOTS, cfg->up_n_tx_slots);
	if (ret < 0)
		return ret;

	ret = a2b_node_write(&node->node, A2B_LDNSLOTS,
			     cfg->dn_n_tx_slots | A2B_LDNSLOTS_DNMASKEN);
	if (ret < 0)
		return ret;

	up_rx_slots = cfg->up_rx_slots;
	dn_rx_slots = cfg->dn_rx_slots;
	for (i = 0; i < 4; i++) {
		ret = a2b_node_write(&node->node, A2B_UPMASK(i),
				     up_rx_slots & 0xff);
		if (ret < 0)
			return ret;

		ret = a2b_node_write(&node->node, A2B_DNMASK(i),
				     dn_rx_slots & 0xff);
		if (ret < 0)
			return ret;

		up_rx_slots >>= 8;
		dn_rx_slots >>= 8;
	}

	return 0;
}

/* See Table 7-43 in the datasheet */
static int a2b_tdmmode_index(unsigned int channels, bool is_mainnode)
{
	switch (channels) {
	case 2:
		return 0;
	case 4:
		return 1;
	case 8:
		return 2;
	case 12:
		return is_mainnode ? 3 : -EINVAL;
	case 16:
		return 4;
	case 20:
		return is_mainnode ? 5 : -EINVAL;
	case 24:
		return is_mainnode ? 6 : -EINVAL;
	case 32:
		return 7;
	default:
		return -EINVAL;
	}
}

static int a2b_node_set_tdm_config(struct a2b_node *node)
{
	struct a2b_tdm_config *cfg = &node->tdm_config;
	struct device *dev = &node->dev;
	unsigned int val;
	int ret;

	ret = a2b_tdmmode_index(cfg->tdm_channels, a2b_node_is_main(node));
	if (ret < 0) {
		dev_err(dev, "Invalid number of TDM channels %d\n",
			cfg->tdm_channels);
		return -EINVAL;
	}

	val = A2B_I2SGCTL_TDMMODE(ret);
	if (cfg->tdm_slot_size == 16) {
		val |= A2B_I2SGCTL_TDMSS;
	} else if (cfg->tdm_slot_size != 32) {
		dev_err(dev, "invalid TDM slot size %d\n", cfg->tdm_slot_size);
		return -EINVAL;
	}

	if (cfg->alt_sync)
		val |= A2B_I2SGCTL_ALT;

	if (cfg->early_sync)
		val |= A2B_I2SGCTL_EARLY;

	if (cfg->invert_sync)
		val |= A2B_I2SGCTL_INV;

	return a2b_node_write(node, A2B_I2SGCTL, val);
}

static int a2b_node_set_transceiver_config(struct a2b_node *node)
{
	uint8_t val;
	int ret;

	ret = a2b_node_write(node, A2B_TXACTL,
			     node->transceiver_config.a_power);
	if (ret < 0)
		return ret;

	ret = a2b_node_write(node, A2B_TXBCTL,
			     node->transceiver_config.b_power);
	if (ret < 0)
		return ret;

	if (node->transceiver_config.invert)
		val = A2B_CONTROL_XCVRBINV;
	else
		val = 0;

	return a2b_node_write(node, A2B_CONTROL, val);
}

static int a2b_node_set_pin_config(struct a2b_node *node)
{
	uint8_t val;

	if (node->pin_config == A2B_PIN_DRIVE_STRENGTH_HIGH)
		val = A2B_PINCFG_DRVSTR_HIGH;
	else
		val = 0;

	return a2b_node_write(node, A2B_PINCFG, val);
}

static int a2b_node_set_pll_config(struct a2b_node *node)
{
	struct a2b_pll_config *pll_config = &node->pll_config;
	uint8_t val = 0;

	switch (pll_config->spread_spectrum_mode) {
	case A2B_SPREAD_SPECTRUM_A2B:
		val |= A2B_PLLCTL_SSMODE_AB;
		break;
	case A2B_SPREAD_SPECTRUM_A2B_I2S:
		val |= A2B_PLLCTL_SSMODE_AB_I2S;
		break;
	default:
		break;
	}

	if (pll_config->spread_spectrum_high)
		val |= A2B_PLLCTL_SSDEPTH;

	return a2b_node_write(node, A2B_PLLCTL, val);
}

static int a2b_bus_discover(struct a2b_mainnode *mainnode)
{
	struct a2b_mainnode_slot_config mainnode_config;
	struct regmap *regmap = mainnode->node.regmap;
	struct device *dev = &mainnode->node.dev;
	struct a2b_subnode *subnode;
	struct a2b_node *node;
	unsigned int respcycs;
	unsigned int val;
	int ret;
	int i;

	if (mainnode->num_subnodes == 0)
		return 0;

	a2b_calc_mainnode_slot_config(mainnode, &mainnode_config);

	if (mainnode_config.respcycs_dn_max > mainnode_config.respcycs_up_min) {
		dev_err(dev, "Unsupported bus topology\n");
		return -EINVAL;
	}

	ret = a2b_node_set_tdm_config(&mainnode->node);
	if (ret)
		return ret;

	ret = a2b_node_set_transceiver_config(&mainnode->node);
	if (ret)
		return ret;

	ret = regmap_write(regmap, A2B_SWCTL, A2B_SWCTL_ENSW);
	if (ret < 0)
		return ret;

	respcycs = (mainnode_config.respcycs_dn_max +
		    mainnode_config.respcycs_up_min) /
		   2;
	ret = regmap_write(regmap, A2B_RESPCYCS, respcycs);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_CONTROL, A2B_CONTROL_NEWSTRCT,
				 A2B_CONTROL_NEWSTRCT);
	if (ret < 0)
		return ret;

	node = &mainnode->node;

	for (i = 0; i < mainnode->num_subnodes; i++) {
		ret = regmap_write(regmap, A2B_DISCVRY, respcycs - (4 * i));
		if (ret < 0)
			return ret;

		ret = a2b_wait_for_irq(mainnode, &mainnode->discover_completion,
				       35);
		if (ret < 0) {
			dev_err(dev, "Discovery of node %d timed out\n", i);
			return ret;
		}

		ret = a2b_node_write(node, A2B_SWCTL,
				     A2B_SWCTL_MODE(2) | A2B_SWCTL_ENSW);
		if (ret < 0)
			return ret;

		subnode = mainnode->subnodes[i];
		node = &subnode->node;

		ret = a2b_node_read_id(node);
		if (ret)
			return ret;

		dev_info(dev, "Node %d discovered\n", i);

		ret = a2b_subnode_set_config(subnode);
		if (ret)
			return ret;

		ret = a2b_node_set_transceiver_config(node);
		if (ret)
			return ret;

		/* Last node? */
		if (i == mainnode->num_subnodes - 1)
			break;

		ret = a2b_node_write(node, A2B_SWCTL, A2B_SWCTL_ENSW);
		if (ret < 0)
			return ret;

		reinit_completion(&mainnode->discover_completion);
	}

	for (; i >= 0; i--) {
		subnode = mainnode->subnodes[i];

		/* Skip the last node, it's B port will remain off */
		if (i != mainnode->num_subnodes - 1) {
			ret = a2b_node_write(&subnode->node, A2B_SWCTL,
					     A2B_SWCTL_ENSW);
			if (ret < 0)
				return ret;
		}

		ret = a2b_node_set_pll_config(&subnode->node);
		if (ret)
			return ret;

		ret = a2b_node_set_pin_config(&subnode->node);
		if (ret)
			return ret;

		ret = a2b_node_set_tdm_config(&subnode->node);
		if (ret)
			return ret;

		ret = a2b_node_init_extra_regmap(&subnode->node);
		if (ret < 0) {
			dev_err(dev, "Failed to program extra registers: %d\n",
				ret);
			return ret;
		}
	}

	ret = regmap_write(regmap, A2B_SWCTL, A2B_SWCTL_ENSW);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, A2B_DNSLOTS, mainnode_config.dn_slots);
	if (ret < 0)
		return ret;

	ret = regmap_write(regmap, A2B_UPSLOTS, mainnode_config.up_slots);
	if (ret < 0)
		return ret;

	ret = a2b_node_init_extra_regmap(&mainnode->node);
	if (ret < 0) {
		dev_err(dev, "Failed to program extra registers: %d\n", ret);
		return ret;
	}

	val = A2B_SLOTFMT_DNSIZE(mainnode->bus_config.dn_slot_size) |
	      A2B_SLOTFMT_UPSIZE(mainnode->bus_config.up_slot_size);

	if (mainnode->bus_config.dn_slot_alt_fmt)
		val |= A2B_SLOTFMT_DNFMT;

	if (mainnode->bus_config.up_slot_alt_fmt)
		val |= A2B_SLOTFMT_UPFMT;

	ret = regmap_write(regmap, A2B_SLOTFMT, val);
	if (ret < 0)
		return ret;

	val = 0;
	if (mainnode_config.dn_enabled)
		val |= A2B_DATCTL_DNS;

	if (mainnode_config.up_enabled)
		val |= A2B_DATCTL_UPS;

	ret = regmap_write(regmap, A2B_DATCTL, val);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(regmap, A2B_CONTROL, A2B_CONTROL_NEWSTRCT,
				 A2B_CONTROL_NEWSTRCT);
	if (ret < 0)
		return ret;

	for (i = 0; i < mainnode->num_subnodes; i++) {
		ret = device_add(&mainnode->subnodes[i]->node.dev);
		if (ret)
			return ret;
	}

	return 0;
}

static int a2b_mainnode_startup(struct a2b_mainnode *mainnode)
{
	struct regmap *regmap = mainnode->node.regmap;
	struct device *dev = &mainnode->node.dev;
	long sync_clk_rate;
	int ret;

	mainnode->sync_clk = clk_get(dev, "sync");
	if (IS_ERR(mainnode->sync_clk)) {
		ret = PTR_ERR(mainnode->sync_clk);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get SYNC clock: %d\n", ret);

		return ret;
	}

	sync_clk_rate = clk_get_rate(mainnode->sync_clk);
	if (sync_clk_rate < 0) {
		dev_err(dev, "Failed to query SYNC clock rate: %ld\n",
			sync_clk_rate);
		return sync_clk_rate;
	}

	/* Accept values that are within +-5% */
	sync_clk_rate = ((sync_clk_rate + 5) / 10) * 10;

	if (sync_clk_rate != 44100 && sync_clk_rate != 48000) {
		dev_err(dev, "SYNC clock rate %ld is invalid\n", sync_clk_rate);
		return -EINVAL;
	}

	ret = clk_prepare_enable(mainnode->sync_clk);
	if (ret) {
		dev_err(dev, "failed to enable sync clk: %d\n", ret);
		return ret;
	}

	ret = regmap_write(regmap, A2B_CONTROL,
			   A2B_CONTROL_MSTR | A2B_CONTROL_SOFTRST);
	if (ret < 0)
		return ret;

	ret = a2b_wait_for_irq(mainnode, &mainnode->run_completion, 10);
	if (ret < 0) {
		dev_err(dev, "timeout waiting for PLL sync: %d\n", ret);
		return ret;
	}

	ret = regmap_update_bits(regmap, A2B_CONTROL, A2B_CONTROL_SOFTRST, 0);
	if (ret)
		return ret;

	ret = a2b_node_set_pll_config(&mainnode->node);
	if (ret)
		return ret;

	ret = a2b_node_set_pin_config(&mainnode->node);
	if (ret)
		return ret;

	ret = a2b_init_irq(mainnode);
	if (ret)
		return ret;

	return 0;
}

static int a2b_unregister_subnodes(struct device *dev, void *data)
{
	if (dev->type == &a2b_subnode_type)
		device_unregister(dev);

	return 0;
}

static void a2b_mainnode_unregister(struct a2b_mainnode *mainnode)
{
	mfd_remove_devices(&mainnode->node.dev);
	device_for_each_child(&mainnode->node.dev, NULL,
			      a2b_unregister_subnodes);

	i2c_unregister_device(mainnode->bus_client);
	regmap_exit(mainnode->node.regmap);
	mainnode->node.regmap = NULL;

	device_unregister(&mainnode->node.dev);
}

static struct a2b_mainnode *
a2b_mainnode_register(struct i2c_client *parent,
		      const struct regmap_config *regmap_config,
		      const struct mfd_cell *cells, unsigned int num_cells)
{
	struct a2b_mainnode *mainnode;
	int ret;
	int id;

	id = ida_simple_get(&a2b_mainnode_ida, 0, 0, GFP_KERNEL);
	if (id < 0)
		return ERR_PTR(id);

	mainnode = kzalloc(sizeof(*mainnode), GFP_KERNEL);
	if (!mainnode)
		return ERR_PTR(-ENOMEM);

	mutex_init(&mainnode->bus_lock);
	init_completion(&mainnode->run_completion);
	init_completion(&mainnode->discover_completion);

	mainnode->node.regmap = regmap_init_i2c(parent, regmap_config);
	if (IS_ERR(mainnode->node.regmap)) {
		dev_err(&parent->dev, "Failed to create regmap: %ld\n",
			PTR_ERR(mainnode->node.regmap));
		ret = PTR_ERR(mainnode->node.regmap);
		goto err_free;
	}

	/*
	 * The A2B deviecs consume two I2C addresses on the bus:
	 * b1101xx0: The I2C address for the local device
	 * b1101xx1: The I2C address to send transactions out on the A2B bus
	 *
	 * Create an additional I2C client to communicate with devices
	 * on the A2B bus
	 */
	mainnode->bus_client =
		i2c_new_dummy(parent->adapter, parent->addr | 0x1);
	if (!mainnode->bus_client) {
		dev_err(&parent->dev, "Failed to allocate bus I2C client\n");
		ret = -EBUSY;
		goto err_regmap_exit;
	}

	mainnode->bus_id = id;
	mainnode->irq = parent->irq;

	mainnode->node.id = A2B_MAIN_NODE_ID;

	dev_set_name(&mainnode->node.dev, "a2b-%d", mainnode->bus_id);
	mainnode->node.dev.parent = &parent->dev;
	mainnode->node.dev.bus = &a2b_bus;
	mainnode->node.dev.type = &a2b_mainnode_type;
	mainnode->node.dev.of_node = parent->dev.of_node;

	ret = a2b_of_mainnode_read_config(mainnode);
	if (ret)
		goto err_bus_client_unregister;

	ret = a2b_node_read_id(&mainnode->node);
	if (ret)
		goto err_bus_client_unregister;

	ret = device_register(&mainnode->node.dev);
	if (ret) {
		put_device(&mainnode->node.dev);
		goto err_bus_client_unregister;
	}

	ret = a2b_of_bus_enumerate(mainnode);
	if (ret)
		goto err_unregister;

	ret = a2b_mainnode_startup(mainnode);
	if (ret)
		goto err_unregister;

	ret = a2b_bus_discover(mainnode);
	if (ret)
		goto err_unregister;

	ret = mfd_add_hotplug_devices(&mainnode->node.dev, cells, num_cells);
	if (ret)
		goto err_unregister;

	return mainnode;

err_unregister:
	a2b_mainnode_unregister(mainnode);
	return ERR_PTR(ret);

err_bus_client_unregister:
	i2c_unregister_device(mainnode->bus_client);
err_regmap_exit:
	regmap_exit(mainnode->node.regmap);
err_free:
	kfree(mainnode);
	return ERR_PTR(ret);
}

static void devm_a2b_mainnode_unregister(void *mainnode)
{
	a2b_mainnode_unregister(mainnode);
}

/**
 * devm_a2b_mainnode_register - Register a new A2B mainnode
 * @parent: Parent I2C client device
 * @regmap_config: Regmap config to use for the A2B mainnode
 *
 * Return: The newly created A2B mainnode on success. On error returns a
 * PTR_ERR().
 */
struct a2b_mainnode *
devm_a2b_mainnode_register(struct i2c_client *parent,
			   const struct regmap_config *regmap_config,
			   const struct mfd_cell *cells, unsigned int num_cells)
{
	struct a2b_mainnode *mainnode;
	int ret;

	mainnode =
		a2b_mainnode_register(parent, regmap_config, cells, num_cells);
	if (IS_ERR(mainnode))
		return mainnode;

	ret = devm_add_action_or_reset(&parent->dev,
				       devm_a2b_mainnode_unregister, mainnode);
	if (ret)
		return ERR_PTR(ret);

	return mainnode;
}
EXPORT_SYMBOL_GPL(devm_a2b_mainnode_register);

/**
 * a2b_driver_register - Register an A2B subnode driver with the framework
 * @owner: Module that contains the driver
 * @driver: The driver to be registered
 *
 * Use the module_a2b_driver() macro instead of calling this function if
 * possible.
 *
 * Return: 0 on success, a negative error code otherwise
 */
int a2b_driver_register(struct module *owner, struct a2b_subnode_driver *driver)
{
	driver->driver.owner = owner;
	driver->driver.bus = &a2b_bus;

	return driver_register(&driver->driver);
}
EXPORT_SYMBOL_GPL(a2b_driver_register);

/**
 * a2b_driver_unregister - Unregister an A2B subnode driver from the framework
 * @driver: The driver to unregister
 */
void a2b_driver_unregister(struct a2b_subnode_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(a2b_driver_unregister);

static int __init a2b_init(void)
{
	return bus_register(&a2b_bus);
}
subsys_initcall(a2b_init);

static void __exit a2b_exit(void)
{
	bus_unregister(&a2b_bus);
	ida_destroy(&a2b_mainnode_ida);
}
module_exit(a2b_exit);

MODULE_LICENSE("GPL v2");
