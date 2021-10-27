// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices Data Offload Engine Driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * Wiki: https://wiki.analog.com/resources/fpga/docs/data_offload
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fpga/adi-axi-common.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stringify.h>

#include "adi-axi-data-offload.h"

/* Register map */

#define AXI_DO_REG_VERSION			0x0000
#define AXI_DO_REG_IDENTIFICATION		0x0004
#define AXI_DO_REG_SCRATCH			0x0008
#define AXI_DO_REG_MAGIC			0x000c
#define   AXI_DO_VAL_MAGIC			0x44414f46
#define AXI_DO_REG_SYNTHESIS_CONFIG		0x0010
#define   AXI_DO_BIT_SYNTHESIS_CONFIG_MEM_TYPE	BIT(0)
#define   AXI_DO_BIT_SYNTHESIS_CONFIG_TX_RXN	BIT(1)
#define AXI_DO_REG_MEMORY_SIZE_LSB		0x0014
#define AXI_DO_REG_MEMORY_SIZE_MSB		0x0018
#define AXI_DO_REG_TRANSFER_LENGTH		0x001c

#define AXI_DO_REG_MEM_PHY_STATE		0x0080
#define AXI_DO_REG_RESETN_OFFLOAD		0x0084
#define AXI_DO_REG_CONTROL			0x0088
#define   AXI_DO_BIT_CONTROL_BYPASS		BIT(0)
#define   AXI_DO_BIT_CONTROL_ONESHOT_EN		BIT(1)

#define AXI_DO_REG_SYNC_TRIGGER			0x100
#define AXI_DO_REG_SYNC_CONFIG			0x104

#define AXI_DO_REG_FSM_DEBUG			0x0200
#define   AXI_DO_BIT_FSM_DEBUG_FSM_STATE_WRITE	GENMASK(1, 0)
#define   AXI_DO_BIT_FSM_DEBUG_FSM_STATE_READ	GENMASK(5, 4)

#define AXI_DO_REG_SAMPLE_COUNT_LSB		0x0204
#define AXI_DO_REG_SAMPLE_COUNT_MSB		0x0208


/* Private driver data */

struct axi_data_offload_state {
	struct list_head			entry;
	void __iomem				*regs;
	struct dentry				*debugdir;

	/*
	 * The update lock is used to prevent races when updating
	 * partial registers, see axi_data_offload_write_mask.
	 * Additionally, lifecycle changes are managed by this
	 * mutex.
	 */
	struct mutex				update_lock;
	u64					mem_size;
	u32					version;
	struct kref				kref;
	bool					initialized;
	struct device_node			*of_node;
};

static LIST_HEAD(probed_devices);
static DEFINE_MUTEX(probed_devices_lock);

static void axi_data_offload_write(struct axi_data_offload_state *st, u32 reg, u32 val)
{
	iowrite32(val, st->regs + reg);
}

static u32 axi_data_offload_read(struct axi_data_offload_state *st, u32 reg)
{
	return ioread32(st->regs + reg);
}

static void axi_data_offload_write_mask(struct axi_data_offload_state *st, u32 reg, u32 val,
					u32 mask)
{
	u32 data;

	data = axi_data_offload_read(st, reg) & ~mask;
	data |= val & mask;

	axi_data_offload_write(st, reg, data);
}

int axi_data_offload_ctrl_bypass(struct axi_data_offload_state *st, bool en)
{
	mutex_lock(&st->update_lock);
	if (!st->initialized) {
		mutex_unlock(&st->update_lock);
		return -ENODEV;
	}

	axi_data_offload_write_mask(st, AXI_DO_REG_CONTROL, en,
			AXI_DO_BIT_CONTROL_BYPASS);

	mutex_unlock(&st->update_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(axi_data_offload_ctrl_bypass);

int axi_data_offload_ctrl_oneshot(struct axi_data_offload_state *st, bool en)
{
	mutex_lock(&st->update_lock);
	if (!st->initialized) {
		mutex_unlock(&st->update_lock);
		return -ENODEV;
	}

	axi_data_offload_write_mask(st, AXI_DO_REG_CONTROL, en << 1,
			AXI_DO_BIT_CONTROL_ONESHOT_EN);

	mutex_unlock(&st->update_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(axi_data_offload_ctrl_oneshot);

static int axi_data_offload_ctrl_transfer_length(struct axi_data_offload_state *st, u64 len)
{
	/* Only accept values which are divisible by 64 */
	if (len & 0x3fULL)
		return -EINVAL;

	/* Requested more memory than we have available! */
	if (len > st->mem_size)
		return -EINVAL;

	axi_data_offload_write(st, AXI_DO_REG_TRANSFER_LENGTH, len >> 6);
	return 0;
}

static int axi_data_offload_ctrl_sync_config(struct axi_data_offload_state *st, u32 cfg)
{
	/* len doesn't fit 32-bit when decremented */
	if (!(cfg == 0 || cfg == 1 || cfg == 2))
		return -EINVAL;

	axi_data_offload_write(st, AXI_DO_REG_SYNC_CONFIG, cfg);
	return 0;
}

static int axi_data_offload_ctrl_enable(struct axi_data_offload_state *st, bool en)
{
	axi_data_offload_write(st, AXI_DO_REG_RESETN_OFFLOAD, en);
	return 0;
}

struct axi_data_offload_info {
	u32 version;
};

static const struct axi_data_offload_info axi_data_offload_1_0_a_info = {
	.version = ADI_AXI_PCORE_VER(1, 0, 'a')
};

/* Match table for of_platform binding */
static const struct of_device_id axi_data_offload_of_match[] = {
	{ .compatible = "adi,axi-data-offload-1.0.a", .data = &axi_data_offload_1_0_a_info },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, axi_data_offload_of_match);

static int axi_data_offload_dbg_get(void *data, u64 *val, size_t offset, u32 mask,
				    int shift, bool l)
{
	struct axi_data_offload_state *st = data;
	u64 rdata;

	mutex_lock(&st->update_lock);

	if (l) {
		rdata = axi_data_offload_read(st, offset);
		rdata |= ((u64)(axi_data_offload_read(st, offset + 4) & mask)) << 32;
	} else {
		rdata = (axi_data_offload_read(st, offset) & mask) >> shift;
	}

	mutex_unlock(&st->update_lock);

	*val = rdata;
	return 0;
}

static int axi_data_offload_dbg_set(void *data, u64 val, size_t offset, u32 mask,
				    int shift, bool l)
{
	struct axi_data_offload_state *st = data;
	u32 n = val;

	/* Input value didn't fit into 32 bit */
	if (val > UINT_MAX)
		return -EINVAL;

	mutex_lock(&st->update_lock);
	axi_data_offload_write_mask(st, offset, n << shift, mask);
	mutex_unlock(&st->update_lock);

	return 0;
}

struct axi_do_dbg_attr {
	char				*name;
	umode_t				mode;
	const struct file_operations	*fops;
};

static struct dentry *axi_data_offload_dbg_parent;

#define ADI_REG_DEVICE_ATTR(_name, _mode, _off, _mask, _shift, _fmt, _l)	\
static int axi_data_offload_dbg_ ## _name ## _get(void *data, u64 *val)		\
{										\
	return axi_data_offload_dbg_get(data, val, _off, _mask, _shift, _l);	\
}										\
static int axi_data_offload_dbg_ ## _name ## _set(void *data, u64 val)		\
{										\
	return axi_data_offload_dbg_set(data, val, _off, _mask, _shift, _l);	\
}										\
DEFINE_DEBUGFS_ATTRIBUTE(axi_data_offload_dbg_ ## _name,			\
			 axi_data_offload_dbg_ ## _name ## _get,		\
			 axi_data_offload_dbg_ ## _name ## _set,		\
			 _fmt);							\
static struct axi_do_dbg_attr axi_data_offload_dbg_ ## _name ## _attr = {	\
	.name	= __stringify(_name),						\
	.mode	= _mode,							\
	.fops	= &axi_data_offload_dbg_ ## _name				\
}

ADI_REG_DEVICE_ATTR(version, 0444, AXI_DO_REG_VERSION, ~0u, 0, "0x%08llx\n", false);
ADI_REG_DEVICE_ATTR(identification, 0444, AXI_DO_REG_IDENTIFICATION, ~0u, 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(scratch, 0644, AXI_DO_REG_SCRATCH, ~0u, 0, "0x%08llx\n", false);
ADI_REG_DEVICE_ATTR(magic, 0444, AXI_DO_REG_MAGIC, ~0u, 0, "0x%08llx\n", false);

ADI_REG_DEVICE_ATTR(synthesis_mem_type, 0444, AXI_DO_REG_SYNTHESIS_CONFIG,
		    BIT(0), 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(synthesis_tx_rxn, 0444, AXI_DO_REG_SYNTHESIS_CONFIG,
		    BIT(1), 1, "%lld\n", false);
ADI_REG_DEVICE_ATTR(memory_size, 0444, AXI_DO_REG_MEMORY_SIZE_LSB,
		    GENMASK(1, 0), 0, "%lld\n", true);
ADI_REG_DEVICE_ATTR(transfer_length, 0644, AXI_DO_REG_TRANSFER_LENGTH,
		    ~0u, 0, "0x%llx\n", false);

ADI_REG_DEVICE_ATTR(phy_calib_complete, 0444, AXI_DO_REG_MEM_PHY_STATE, BIT(0), 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(resetn, 0644, AXI_DO_REG_RESETN_OFFLOAD, BIT(0), 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(control_bypass, 0644, AXI_DO_REG_CONTROL, BIT(0), 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(control_oneshot, 0644, AXI_DO_REG_CONTROL, BIT(1), 1, "%lld\n", false);

ADI_REG_DEVICE_ATTR(sync_trigger, 0644, AXI_DO_REG_SYNC_TRIGGER, BIT(0), 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(sync_config, 0644, AXI_DO_REG_SYNC_CONFIG, GENMASK(1, 0), 0, "%lld\n", false);

ADI_REG_DEVICE_ATTR(fsm_debug_state_write, 0444, AXI_DO_REG_FSM_DEBUG,
		AXI_DO_BIT_FSM_DEBUG_FSM_STATE_WRITE, 0, "%lld\n", false);
ADI_REG_DEVICE_ATTR(fsm_debug_state_read, 0444, AXI_DO_REG_FSM_DEBUG,
		AXI_DO_BIT_FSM_DEBUG_FSM_STATE_READ, 4, "%lld\n", false);

/* May encounter race condition - maybe read the value multiple times? */
ADI_REG_DEVICE_ATTR(sample_count, 0444, AXI_DO_REG_SAMPLE_COUNT_LSB, ~0u, 0, "%llu\n", true);

static struct axi_do_dbg_attr *axi_data_offload_dbg_attrs[] = {
	&axi_data_offload_dbg_version_attr,
	&axi_data_offload_dbg_identification_attr,
	&axi_data_offload_dbg_scratch_attr,
	&axi_data_offload_dbg_magic_attr,
	&axi_data_offload_dbg_synthesis_mem_type_attr,
	&axi_data_offload_dbg_synthesis_tx_rxn_attr,
	&axi_data_offload_dbg_memory_size_attr,
	&axi_data_offload_dbg_transfer_length_attr,
	&axi_data_offload_dbg_phy_calib_complete_attr,
	&axi_data_offload_dbg_resetn_attr,
	&axi_data_offload_dbg_control_bypass_attr,
	&axi_data_offload_dbg_control_oneshot_attr,
	&axi_data_offload_dbg_sync_trigger_attr,
	&axi_data_offload_dbg_sync_config_attr,
	&axi_data_offload_dbg_fsm_debug_state_write_attr,
	&axi_data_offload_dbg_fsm_debug_state_read_attr,
	&axi_data_offload_dbg_sample_count_attr,
	NULL /* END OF LIST MARKER */
};

static void axi_data_offload_dbg_cleanup(void *data)
{
	struct axi_data_offload_state *st = data;

	debugfs_remove_recursive(st->debugdir);
}

#define kref_to_data_offload(x) container_of(x, struct axi_data_offload_state, kref)

static void axi_data_offload_release(struct kref *kref)
{
	struct axi_data_offload_state *st = kref_to_data_offload(kref);

	of_node_put(st->of_node);
	kfree(st);
}

static void axi_data_offload_put(void *data)
{
	struct axi_data_offload_state *st = data;

	kref_put(&st->kref, axi_data_offload_release);
}

static void axi_data_offload_unregister(void *data)
{
	struct axi_data_offload_state *st = data;

	mutex_lock(&probed_devices_lock);
	list_del(&st->entry);
	mutex_unlock(&probed_devices_lock);

	mutex_lock(&st->update_lock);
	st->initialized = false;
	mutex_unlock(&st->update_lock);
}

static int axi_data_offload_register(struct device *dev, struct axi_data_offload_state *st)
{
	mutex_lock(&probed_devices_lock);
	list_add(&st->entry, &probed_devices);
	mutex_unlock(&probed_devices_lock);

	return devm_add_action_or_reset(dev, axi_data_offload_unregister, st);
}

struct axi_data_offload_state *devm_axi_data_offload_get_optional(struct device *dev)
{
	struct axi_data_offload_state *st;
	int ret;
	struct device_node *of_node;

	of_node = of_parse_phandle(dev->of_node, "adi,axi-data-offload-connected", 0);
	if (!of_node)
		return NULL;

	mutex_lock(&probed_devices_lock);

	list_for_each_entry(st, &probed_devices, entry) {
		if (st->of_node != of_node)
			continue;

		kref_get(&st->kref);

		mutex_unlock(&probed_devices_lock);
		of_node_put(of_node);

		ret = devm_add_action_or_reset(dev, axi_data_offload_put, st);
		if (ret)
			return ERR_PTR(ret);

		return st;
	}

	dev_dbg(dev, "Failed to find requested data_offload \"%s\", try again later!\n", of_node->name);

	mutex_unlock(&probed_devices_lock);
	of_node_put(of_node);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(devm_axi_data_offload_get_optional);

static int axi_data_offload_probe(struct platform_device *pdev)
{
	int ret;
	struct axi_data_offload_state *st;
	const struct axi_data_offload_info *info;
	struct device_node *np = pdev->dev.of_node;
	struct axi_do_dbg_attr **i;
	u32 magic;
	u32 sync_config;
	u64 transfer_length;

	info = of_device_get_match_data(&pdev->dev);
	if (!info)
		return -ENODEV;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n", np->name);

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->of_node = of_node_get(np);
	mutex_init(&st->update_lock);
	kref_init(&st->kref);

	ret = devm_add_action_or_reset(&pdev->dev, axi_data_offload_put, st);
	if (ret)
		return ret;

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	magic = axi_data_offload_read(st, AXI_DO_REG_MAGIC);
	if (magic != AXI_DO_VAL_MAGIC) {
		dev_err(&pdev->dev, "Unexpected peripheral identifier %.08x != %.08x\n", magic,
				AXI_DO_VAL_MAGIC);
		return -ENODEV;
	}

	st->version = axi_data_offload_read(st, AXI_DO_REG_VERSION);
	if (info->version > st->version) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.%d.%c, Reported %d.%d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(info->version),
			ADI_AXI_PCORE_VER_MINOR(info->version),
			ADI_AXI_PCORE_VER_PATCH(info->version),
			ADI_AXI_PCORE_VER_MAJOR(st->version),
			ADI_AXI_PCORE_VER_MINOR(st->version),
			ADI_AXI_PCORE_VER_PATCH(st->version));
		return -ENODEV;
	}

	st->mem_size = axi_data_offload_read(st, AXI_DO_REG_MEMORY_SIZE_LSB)
		| ((u64)(axi_data_offload_read(st, AXI_DO_REG_MEMORY_SIZE_MSB) & GENMASK(1, 0)) << 32);

	if (!IS_ERR(axi_data_offload_dbg_parent)) {
		st->debugdir = debugfs_create_dir(np->name, axi_data_offload_dbg_parent);
		if (!IS_ERR(st->debugdir)) {
			for (i = axi_data_offload_dbg_attrs; *i != NULL; i++)
				debugfs_create_file_unsafe((*i)->name, (*i)->mode,
						st->debugdir, st, (*i)->fops);

			ret = devm_add_action_or_reset(&pdev->dev, axi_data_offload_dbg_cleanup, st);
			if (ret)
				return ret;
		}
	}

	st->initialized = true;

	/*
	 * Initialization done. Device access is now allowed and we can proceed with applying
	 * the device tree configuration.
	 */
	ret = of_property_read_u64(st->of_node, "adi,transfer-length", &transfer_length);
	if (!ret) {
		ret = axi_data_offload_ctrl_transfer_length(st, transfer_length);
		if (ret) {
			dev_err(&pdev->dev,
				"Invalid device tree attribute value: adi,transfer-length = 0x%llx\n",
				transfer_length);
			return ret;
		}
	}

	ret = of_property_read_u32(st->of_node, "adi,sync-config", &sync_config);
	if (!ret) {
		ret = axi_data_offload_ctrl_sync_config(st, sync_config);
		if (ret) {
			dev_err(&pdev->dev,
				"Invalid device tree attribute value: adi,sync-config = %u\n",
				sync_config);
			return ret;
		}
	}

	if (of_property_read_bool(st->of_node, "adi,oneshot"))
		axi_data_offload_ctrl_oneshot(st, true);

	if (of_property_read_bool(st->of_node, "adi,bypass"))
		axi_data_offload_ctrl_bypass(st, true);

	if (of_property_read_bool(st->of_node, "adi,bringup"))
		axi_data_offload_ctrl_enable(st, true);


	/* Register device for other drivers to access */
	ret = axi_data_offload_register(&pdev->dev, st);
	if (ret)
		return ret;

	/* Done */
	dev_dbg(&pdev->dev, "AXI IP core (%d.%d.%c) probed\n",
		ADI_AXI_PCORE_VER_MAJOR(st->version),
		ADI_AXI_PCORE_VER_MINOR(st->version),
		ADI_AXI_PCORE_VER_PATCH(st->version));

	return 0;
}

static struct platform_driver axi_data_offload_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = axi_data_offload_of_match,
	},
	.probe = axi_data_offload_probe
};

static int axi_data_offload_driver_register(struct platform_driver *driver)
{
	axi_data_offload_dbg_parent = debugfs_create_dir(KBUILD_MODNAME, NULL);
	return platform_driver_register(driver);
}

static void axi_data_offload_driver_unregister(struct platform_driver *driver)
{
	debugfs_remove_recursive(axi_data_offload_dbg_parent);
	platform_driver_unregister(driver);
}

module_driver(axi_data_offload_driver,
	      axi_data_offload_driver_register,
	      axi_data_offload_driver_unregister);

MODULE_AUTHOR("David Winter <david.winter@analog.com>");
MODULE_DESCRIPTION("Analog Devices data offload engine driver");
MODULE_LICENSE("GPL v2");
