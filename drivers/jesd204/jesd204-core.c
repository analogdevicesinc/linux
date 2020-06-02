// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - core logic
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#define pr_fmt(fmt) "jesd204: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/slab.h>

#include "jesd204-priv.h"

/* IDA to assign each registered device a unique id */
static DEFINE_IDA(jesd204_ida);

static struct bus_type jesd204_bus_type = {
	.name = "jesd204",
};

static DEFINE_MUTEX(jesd204_device_list_lock);
static LIST_HEAD(jesd204_device_list);
static LIST_HEAD(jesd204_topologies);

static unsigned int jesd204_device_count;
static unsigned int jesd204_topologies_count;

int jesd204_link_get_rate(struct jesd204_link *lnk, u64 *lane_rate_hz)
{
	u64 rate, encoding_n, encoding_d;

	if (!lnk->num_lanes || !lnk->num_converters ||
	    !lnk->bits_per_sample || !lnk->sample_rate) {
		/* FIXME: make this a bit more verbose */
		pr_err("%s: Invalid pramater", __func__);
		return -EINVAL;
	}

	switch (lnk->jesd_version) {
	case JESD204_VERSION_C:
		switch (lnk->jesd_encoder) {
		case JESD204_ENC_64B66B:
			encoding_n = 66; /* JESD 204C */
			encoding_d = 64;
			break;
		case JESD204_ENC_8B10B:
			encoding_n = 10; /* JESD 204C */
			encoding_d = 8;
			break;
		case JESD204_ENC_64B80B:
			encoding_n = 80; /* JESD 204C */
			encoding_d = 64;
			break;
		default:
			return -EINVAL;
		}
	default:
		encoding_n = 10; /* JESD 204AB */
		encoding_d = 8;
		break;
	}

	rate = lnk->num_converters * lnk->bits_per_sample *
		encoding_n * lnk->sample_rate;
	do_div(rate, lnk->num_lanes * encoding_d);

	*lane_rate_hz = rate;

	return 0;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_rate);

int jesd204_link_get_rate_khz(struct jesd204_link *lnk,
			      unsigned long *lane_rate_khz)
{
	u64 lane_rate_hz;
	int ret;

	ret = jesd204_link_get_rate(lnk, &lane_rate_hz);
	if (ret)
		return ret;

	do_div(lane_rate_hz, 1000);

	*lane_rate_khz = lane_rate_hz;

	return ret;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_rate_khz);

int jesd204_link_get_device_clock(struct jesd204_link *lnk,
				  unsigned long *device_clock)
{
	u64 lane_rate_hz;
	u32 encoding_n;
	int ret;

	ret = jesd204_link_get_rate(lnk, &lane_rate_hz);
	if (ret)
		return ret;

	switch (lnk->jesd_encoder) {
	case JESD204_ENC_64B66B:
		encoding_n = 66; /* JESD 204C */
		break;
	case JESD204_ENC_8B10B:
		encoding_n = 40; /* JESD 204ABC */
		break;
	case JESD204_ENC_64B80B:
		encoding_n = 80; /* JESD 204C */
		break;
	default:
		return -EINVAL;
	}

	do_div(lane_rate_hz, encoding_n);

	*device_clock = lane_rate_hz;

	return ret;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_device_clock);

int jesd204_link_get_lmfc_lemc_rate(struct jesd204_link *lnk,
				    unsigned long *rate_hz)
{
	u64 lane_rate_hz;
	u32 bkw;
	int ret;

	ret = jesd204_link_get_rate(lnk, &lane_rate_hz);
	if (ret)
		return ret;

	switch (lnk->jesd_encoder) {
	case JESD204_ENC_64B66B:
		bkw = 66; /* JESD 204C */
	case JESD204_ENC_64B80B:
		if (lnk->jesd_encoder == JESD204_ENC_64B80B)
			bkw = 80; /* JESD 204C */

		if (lnk->num_of_multiblocks_in_emb) {
			do_div(lane_rate_hz, bkw * 32 *
				lnk->num_of_multiblocks_in_emb);
		} else {
			lane_rate_hz *= 8;
			do_div(lane_rate_hz, bkw *
				lnk->octets_per_frame *
				lnk->frames_per_multiframe);
		}
		break;
	default:
		do_div(lane_rate_hz, 10 * lnk->octets_per_frame *
			lnk->frames_per_multiframe);
		break;
	}

	*rate_hz = lane_rate_hz;

	return 0;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_lmfc_lemc_rate);

struct list_head *jesd204_topologies_get(void)
{
	return &jesd204_topologies;
}

static inline bool dev_is_jesd204_dev(struct device *dev)
{
	return device_property_read_bool(dev, "jesd204-device");
}

void *jesd204_dev_priv(struct jesd204_dev *jdev)
{
	return jdev->priv;
}
EXPORT_SYMBOL(jesd204_dev_priv);

struct jesd204_dev *jesd204_dev_from_device(struct device *dev)
{
	struct jesd204_dev *jdev;

	if (!dev)
		return NULL;

	list_for_each_entry(jdev, &jesd204_device_list, entry) {
		if (jdev->parent && jdev->parent == dev)
			return jdev;
	}

	return NULL;
}
EXPORT_SYMBOL(jesd204_dev_from_device);

struct device *jesd204_dev_to_device(struct jesd204_dev *jdev)
{
	return jdev ? jdev->parent : NULL;
}
EXPORT_SYMBOL(jesd204_dev_to_device);

static int jesd204_dev_alloc_links(struct jesd204_dev_top *jdev_top)
{
	struct jesd204_link_opaque *links;
	size_t mem_size;
	unsigned int i;

	mem_size = jdev_top->num_links * sizeof(*links);

	links = kzalloc(mem_size, GFP_KERNEL);
	if (!links)
		return -ENOMEM;
	jdev_top->active_links = links;

	for (i = 0; i < jdev_top->num_links; i++) {
		links[i].jdev_top = jdev_top;
		links[i].link_idx = i;
		links[i].link.link_id = jdev_top->link_ids[i];
	}

	links = kzalloc(mem_size, GFP_KERNEL);
	if (!links) {
		kfree(jdev_top->active_links);
		return -ENOMEM;
	}
	jdev_top->staged_links = links;

	return 0;
}

static struct jesd204_dev *jesd204_dev_alloc(struct device_node *np)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_dev *jdev;
	unsigned int link_ids[JESD204_MAX_LINKS];
	u32 topo_id;
	int i, ret;

	if (of_property_read_u32(np, "jesd204-top-device", &topo_id) == 0) {
		ret = of_property_read_variable_u32_array(np,
							  "jesd204-link-ids",
							  link_ids,
							  1,
							  JESD204_MAX_LINKS);
		if (ret < 0) {
			pr_err("%pOF error getting 'jesd204-link-ids': %d\n",
			       np, ret);
			return ERR_PTR(ret);
		}

		jdev_top = kzalloc(sizeof(*jdev_top), GFP_KERNEL);
		if (!jdev_top)
			return ERR_PTR(-ENOMEM);

		jdev = &jdev_top->jdev;

		jdev_top->topo_id = topo_id;
		jdev_top->num_links = ret;
		for (i = 0; i < jdev_top->num_links; i++)
			jdev_top->link_ids[i] = link_ids[i];

		ret = jesd204_dev_alloc_links(jdev_top);
		if (ret) {
			kfree(jdev_top);
			return ERR_PTR(ret);
		}

		jdev->is_top = true;

		list_add(&jdev_top->entry, &jesd204_topologies);
		jesd204_topologies_count++;
	} else {
		jdev = kzalloc(sizeof(*jdev), GFP_KERNEL);
		if (!jdev)
			return ERR_PTR(-ENOMEM);
	}

	jdev->id = -1;
	jdev->np = of_node_get(np);
	kref_init(&jdev->ref);

	INIT_LIST_HEAD(&jdev->outputs);

	list_add(&jdev->entry, &jesd204_device_list);
	jesd204_device_count++;

	return jdev;
}

static struct jesd204_dev *jesd204_dev_find_by_of_node(struct device_node *np)
{
	struct jesd204_dev *jdev = NULL, *jdev_it;

	if (!np)
		return NULL;

	list_for_each_entry(jdev_it, &jesd204_device_list, entry) {
		if (jdev_it->np == np) {
			jdev = jdev_it;
			break;
		}
	}

	return jdev;
}

static struct jesd204_dev_con_out *jesd204_dev_find_output_con(
		struct jesd204_dev *jdev,
		struct of_phandle_args *args)
{
	struct jesd204_dev_con_out *con;
	unsigned int i;

	/* find an existing output connection for the current of args */
	list_for_each_entry(con, &jdev->outputs, entry) {
		if (args->np != con->of.np)
			continue;
		if (args->args_count != con->of.args_count)
			continue;
		for (i = 0; i < args->args_count; i++) {
			if (args->args[i] != con->of.args[i])
				break;
		}
		if (i != args->args_count)
			continue;
		return con;
	}

	return NULL;
}

static int jesd204_dev_create_con(struct jesd204_dev *jdev,
				  struct of_phandle_args *args)
{
	struct jesd204_dev_con_out *con;
	struct jesd204_dev *jdev_in;
	struct jesd204_dev_list_entry *e;

	if (args->args_count < 2) {
		pr_err("connection %pOF->%pOF requires 2 args minimum\n",
		       args->np, jdev->np);
		return -EINVAL;
	}

	jdev_in = jesd204_dev_find_by_of_node(args->np);
	if (!jdev_in) {
		pr_err("connection %pOF->%pOF invalid\n", args->np, jdev->np);
		return -ENOENT;
	}

	e = kzalloc(sizeof(*e), GFP_KERNEL);
	if (!e)
		return -ENOMEM;

	con = jesd204_dev_find_output_con(jdev_in, args);
	if (!con) {
		con = kzalloc(sizeof(*con), GFP_KERNEL);
		if (!con) {
			kfree(e);
			return -ENOMEM;
		}

		con->topo_id = args->args[0];
		con->link_id = args->args[1];
		con->link_idx = JESD204_LINKS_ALL;
		con->owner = jdev_in;
		INIT_LIST_HEAD(&con->dests);

		memcpy(&con->of, args, sizeof(con->of));
		list_add(&con->entry, &jdev_in->outputs);
		jdev_in->outputs_count++;
	}

	e->jdev = jdev;
	list_add(&e->entry, &con->dests);
	con->dests_count++;

	jdev->inputs[jdev->inputs_count] = con;
	jdev->inputs_count++;

	return 0;
}

static int jesd204_of_device_create_cons(struct jesd204_dev *jdev)
{
	struct device_node *np = jdev->np;
	struct of_phandle_args args;
	int i, c, ret;

	c = of_count_phandle_with_args(np, "jesd204-inputs", "#jesd204-cells");
	if (c == -ENOENT || c == 0)
		return 0;
	if (c < 0)
		return c;

	jdev->inputs = kcalloc(c, sizeof(*jdev->inputs), GFP_KERNEL);
	if (!jdev->inputs)
		return -ENOMEM;

	for (i = 0; i < c; i++) {
		ret = of_parse_phandle_with_args(np,
						 "jesd204-inputs",
						 "#jesd204-cells",
						 i, &args);
		/**
		 * If one bad/non-existing connection is found, then all
		 * JESD204 topologies won't be initialized. We may
		 * improve this later, to allow the good configs
		 */
		if (ret < 0)
			return ret;

		ret = jesd204_dev_create_con(jdev, &args);
		of_node_put(args.np);
		if (ret)
			return ret;
	}

	return 0;
}

static int jesd204_of_create_devices(void)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_dev *jdev;
	struct device_node *np;
	int ret;

	mutex_lock(&jesd204_device_list_lock);

	ret = 0;
	for_each_node_with_property(np, "jesd204-device") {
		jdev = jesd204_dev_alloc(np);
		if (IS_ERR(jdev)) {
			ret = PTR_ERR(jdev);
			goto unlock;
		}
	}

	list_for_each_entry(jdev, &jesd204_device_list, entry) {
		ret = jesd204_of_device_create_cons(jdev);
		if (ret)
			goto unlock;
	}

	list_for_each_entry(jdev_top, &jesd204_topologies, entry) {
		ret = jesd204_init_topology(jdev_top);
		if (ret)
			goto unlock;
	}

unlock:
	mutex_unlock(&jesd204_device_list_lock);

	return ret;
}

static int jesd204_dev_init_link_lane_ids(struct jesd204_dev_top *jdev_top,
					  unsigned int link_idx,
					  struct jesd204_link *jlink)
{
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct device *dev = jdev->parent;
	u8 id;

	if (!jlink->num_lanes) {
		dev_err(dev, "JESD204 link [%d] number of lanes is 0\n",
			link_idx);
		jlink->lane_ids = NULL;
		return -EINVAL;
	}

	/* We have some lane IDs provided statically for this link ID; exit */
	if (jdev_top->init_links &&
	    jdev_top->init_links[link_idx].lane_ids)
		return 0;

	if (jlink->lane_ids)
		devm_kfree(dev, jlink->lane_ids);

	jlink->lane_ids = devm_kmalloc_array(dev, jlink->num_lanes,
					     sizeof(*jlink->lane_ids),
					     GFP_KERNEL);
	if (!jlink->lane_ids)
		return -ENOMEM;

	/* Assign lane IDs, based on lane index */
	for (id = 0; id < jlink->num_lanes; id++)
		jlink->lane_ids[id] = id;

	return 0;
}

static int __jesd204_dev_init_link_data(struct jesd204_dev_top *jdev_top,
					unsigned int link_idx)
{
	struct jesd204_link_opaque *ol;
	int ret;

	ol = &jdev_top->active_links[link_idx];
	ol->link.link_id = jdev_top->link_ids[link_idx];
	ol->jdev_top = jdev_top;
	ol->link_idx = link_idx;
	ret = jesd204_dev_init_link_lane_ids(jdev_top, link_idx, &ol->link);
	if (ret)
		return ret;

	memcpy(&jdev_top->staged_links[link_idx], ol,
	       sizeof(jdev_top->staged_links[link_idx]));

	return 0;
}

/* FIXME: see about maybe handling lane IDs assigned via the link_op for init links */
int jesd204_dev_init_link_data(struct jesd204_dev_top *jdev_top,
			       unsigned int link_idx)
{
	int ret;

	if (link_idx != JESD204_LINKS_ALL)
		return __jesd204_dev_init_link_data(jdev_top, link_idx);

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ret = __jesd204_dev_init_link_data(jdev_top, link_idx);
		if (ret)
			return ret;
	}

	return 0;
}

static int jesd204_dev_init_links_data(struct jesd204_dev *jdev,
				       const struct jesd204_dev_data *init)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	struct device *dev = jdev->parent;
	unsigned int i;

	if (!jdev_top)
		return 0;

	if (!init->num_links) {
		dev_err(dev, "num_links shouldn't be zero\n");
		return -EINVAL;
	}

	/* FIXME: should we just do a minimum? for now we error out if these mismatch */
	if (init->num_links != jdev_top->num_links) {
		dev_err(dev,
			"Driver and DT mismatch for number of links %u vs %u\n",
			init->num_links, jdev_top->num_links);
		return -EINVAL;
	}

	/**
	 * Framework users should provide at least initial JESD204 link data,
	 * or a link init op/callback which should do JESD204 link init.
	 */
	if (!init->links &&
	    !init->state_ops &&
	    !init->state_ops[JESD204_OP_LINK_INIT].per_link) {
		dev_err(dev,
			"num_links is non-zero, but no links data provided\n");
		return -EINVAL;
	}

	if (!init->links)
		return 0;

	for (i = 0; i < jdev_top->num_links; i++) {
		memcpy(&jdev_top->active_links[i].link, &init->links[i],
		       sizeof(struct jesd204_link));
		memcpy(&jdev_top->staged_links[i].link, &init->links[i],
		       sizeof(struct jesd204_link));
	}

	jdev_top->init_links = init->links;

	return 0;
}

struct jesd204_dev *jesd204_dev_register(struct device *dev,
					 const struct jesd204_dev_data *init)
{
	struct jesd204_dev *jdev;
	int ret, id;

	if (!dev || !init) {
		dev_err(dev, "Invalid register arguments\n");
		return ERR_PTR(-EINVAL);
	}

	if (!dev_is_jesd204_dev(dev))
		return NULL;

	id = ida_simple_get(&jesd204_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		dev_err(dev, "Unable to get unique ID for device\n");
		return ERR_PTR(id);
	}

	mutex_lock(&jesd204_device_list_lock);

	jdev = jesd204_dev_from_device(dev);
	if (jdev) {
		dev_err(dev, "Device already registered with framework\n");
		ret = -EEXIST;
		goto err_free_id;
	}

	jdev = jesd204_dev_find_by_of_node(dev->of_node);
	if (!jdev) {
		dev_err(dev, "Device has no configuration node\n");
		ret = -ENODEV;
		goto err_free_id;
	}

	jdev->state_ops = init->state_ops;
	jdev->parent = get_device(dev);

	ret = jesd204_dev_init_links_data(jdev, init);
	if (ret)
		goto err_put_device;

	jdev->id = id;

	jdev->dev.parent = dev;
	jdev->dev.bus = &jesd204_bus_type;
	device_initialize(&jdev->dev);
	dev_set_name(&jdev->dev, "jesd204:%d", id);

	ret = device_add(&jdev->dev);
	if (ret) {
		put_device(&jdev->dev);
		goto err_put_device;
	}
	jdev->id = id;

	ret = jesd204_fsm_probe(jdev);
	if (ret)
		goto err_device_del;

	if (init->sizeof_priv) {
		jdev->priv = devm_kzalloc(jdev->parent, init->sizeof_priv,
					  GFP_KERNEL);
		if (!jdev->priv) {
			ret = -ENOMEM;
			goto err_device_del;
		}
	}

	mutex_unlock(&jesd204_device_list_lock);

	return jdev;

err_device_del:
	device_del(&jdev->dev);
err_put_device:
	put_device(dev);
err_free_id:
	ida_simple_remove(&jesd204_ida, id);
	mutex_unlock(&jesd204_device_list_lock);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL(jesd204_dev_register);

static void jesd204_dev_destroy_cons(struct jesd204_dev *jdev)
{
	struct jesd204_dev_con_out *c, *c1;
	struct jesd204_dev_list_entry *e, *e1;

	kfree(jdev->inputs);

	list_for_each_entry_safe(c, c1, &jdev->outputs, entry) {
		list_del(&c->entry);
		list_for_each_entry_safe(e, e1, &c->dests, entry) {
			list_del(&e->entry);
			kfree(e);
		}
		kfree(c);
	}
}

static void jesd204_of_unregister_devices(void)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_dev *jdev, *j;

	list_for_each_entry_safe(jdev, j, &jesd204_device_list, entry) {
		jesd204_dev_unregister(jdev);
		jesd204_dev_destroy_cons(jdev);
		of_node_put(jdev->np);
		jesd204_device_count--;
		if (!jdev->is_top) {
			kfree(jdev);
			continue;
		}
		jdev_top = jesd204_dev_top_dev(jdev);
		list_del(&jdev_top->entry);
		kfree(jdev_top);
		jesd204_topologies_count--;
	}
}

/* Free memory allocated. */
static void __jesd204_dev_release(struct kref *ref)
{
	struct jesd204_dev *jdev = container_of(ref, struct jesd204_dev, ref);
	int id = jdev->id;

	mutex_lock(&jesd204_device_list_lock);

	if (jdev->parent) {
		put_device(jdev->parent);
		jdev->parent = NULL;
	}

	mutex_unlock(&jesd204_device_list_lock);

	if (id > -1)
		ida_simple_remove(&jesd204_ida, id);
}

static void jesd204_dev_kref_put(struct jesd204_dev *jdev)
{
	kref_put(&jdev->ref, __jesd204_dev_release);
}

/**
 * jesd204_dev_unregister() - unregister a device from the JESD204 subsystem
 * @jdev:		Device structure representing the device.
 **/
void jesd204_dev_unregister(struct jesd204_dev *jdev)
{
	if (IS_ERR_OR_NULL(jdev))
		return;

	if (jdev->id > -1) {
		jdev->id = -1;
		device_del(&jdev->dev);
	}

	jesd204_fsm_uninit_device(jdev);
	jesd204_dev_kref_put(jdev);
}
EXPORT_SYMBOL(jesd204_dev_unregister);

static void devm_jesd204_dev_unreg(struct device *dev, void *res)
{
	jesd204_dev_unregister(*(struct jesd204_dev **)res);
}

struct jesd204_dev *devm_jesd204_dev_register(struct device *dev,
					      const struct jesd204_dev_data *i)
{
	struct jesd204_dev **jdevp, *jdev;

	if (!dev_is_jesd204_dev(dev))
		return NULL;

	jdevp = devres_alloc(devm_jesd204_dev_unreg, sizeof(*jdevp),
			     GFP_KERNEL);
	if (!jdevp)
		return ERR_PTR(-ENOMEM);

	jdev = jesd204_dev_register(dev, i);
	if (!IS_ERR(jdev)) {
		*jdevp = jdev;
		devres_add(dev, jdevp);
	} else {
		devres_free(jdevp);
	}

	return jdev;
}
EXPORT_SYMBOL_GPL(devm_jesd204_dev_register);

static int __init jesd204_init(void)
{
	int ret;

	mutex_init(&jesd204_device_list_lock);

	ret  = bus_register(&jesd204_bus_type);
	if (ret < 0) {
		pr_err("could not register bus type\n");
		return ret;
	}

	ret = jesd204_of_create_devices();
	if (ret < 0)
		goto error_unreg_devices;

	pr_info("found %u devices and %u topologies\n",
		jesd204_device_count, jesd204_topologies_count);

	return 0;

error_unreg_devices:
	jesd204_of_unregister_devices();
	pr_err("framework error: %d\n", ret);
	return ret;
}

static void __exit jesd204_exit(void)
{
	jesd204_of_unregister_devices();
	mutex_destroy(&jesd204_device_list_lock);
	bus_unregister(&jesd204_bus_type);
}

subsys_initcall(jesd204_init);
module_exit(jesd204_exit);

MODULE_AUTHOR("Alexandru Ardelean <alexandru.ardelean@analog.com>");
MODULE_DESCRIPTION("JESD204 framework core");
MODULE_LICENSE("GPL");
