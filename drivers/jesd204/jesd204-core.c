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

static LIST_HEAD(jesd204_device_list);
static LIST_HEAD(jesd204_topologies);

static unsigned int jesd204_device_count;
static unsigned int jesd204_topologies_count;

static unsigned int jesd204_con_id_counter;

static void jesd204_dev_unregister(struct jesd204_dev *jdev);

int jesd204_get_active_links_num(struct jesd204_dev *jdev)
{
	struct jesd204_dev_top *jdev_top;

	if (!jdev)
		return -EINVAL;

	jdev_top = jesd204_dev_get_topology_top_dev(jdev);
	if (!jdev_top) {
		jesd204_err(jdev, "Could not find top-level device\n");
		return -EFAULT;
	}

	return jdev_top->num_links;
}

int jesd204_get_links_data(struct jesd204_dev *jdev,
			   struct jesd204_link ** const links,
			   const unsigned int num_links)
{
	struct jesd204_dev_top *jdev_top;
	unsigned int i;

	if (!jdev || !links || !num_links)
		return -EINVAL;

	jdev_top = jesd204_dev_get_topology_top_dev(jdev);
	if (!jdev_top) {
		jesd204_err(jdev, "Could not find top-level device\n");
		return -EFAULT;
	}

	if (jdev_top->num_links != num_links) {
		jesd204_err(jdev,
			    "Link number mismatch: top-level has %u, call has %u\n",
			    jdev_top->num_links, num_links);
		return -EINVAL;
	}

	for (i = 0; i < jdev_top->num_links; i++)
		links[i] = &jdev_top->active_links[i].link;

	return 0;
}
EXPORT_SYMBOL_GPL(jesd204_get_links_data);

const char *jesd204_link_get_state_str(const struct jesd204_link *lnk)
{
	struct jesd204_link_opaque *ol =
		container_of(lnk, struct jesd204_link_opaque, link);

	return jesd204_state_str(ol->state);
}
EXPORT_SYMBOL_GPL(jesd204_link_get_state_str);

bool jesd204_link_get_paused(const struct jesd204_link *lnk)
{
	struct jesd204_link_opaque *ol =
		container_of(lnk, struct jesd204_link_opaque, link);

	return ol->fsm_paused;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_paused);

int jesd204_device_count_get()
{
	return jesd204_device_count;
}

static bool jesd204_dev_has_con_in_topology(struct jesd204_dev *jdev,
					    struct jesd204_dev_top *jdev_top)
{
	struct jesd204_dev_con_out *c;
	int i;

	list_for_each_entry(c, &jdev->outputs, entry) {
		if (c->jdev_top == jdev_top)
			return true;
	}

	for (i = 0; i < jdev->inputs_count; i++) {
		c = jdev->inputs[i];
		if (c->jdev_top == jdev_top)
			return true;
	}

	return false;
}

static int jesd204_link_validate_params(const struct jesd204_link *lnk)
{
	if (!lnk->num_lanes) {
		pr_err("link[%u], number of lanes is zero\n", lnk->link_id);
		return -EINVAL;
	}

	if (!lnk->num_converters) {
		pr_err("link[%u], number of converters is zero\n", lnk->link_id);
		return -EINVAL;
	}

	if (!lnk->bits_per_sample) {
		pr_err("link[%u], bits-per-sample is zero\n", lnk->link_id);
		return -EINVAL;
	}

	if (!lnk->sample_rate) {
		pr_err("link[%u], sample rate is zero\n", lnk->link_id);
		return -EINVAL;
	}

	return 0;
}

int jesd204_link_get_rate(struct jesd204_link *lnk, u64 *lane_rate_hz)
{
	u64 rate, encoding_n, encoding_d;
	u32 sample_rate_div;
	int ret;

	ret = jesd204_link_validate_params(lnk);
	if (ret)
		return ret;

	switch (lnk->jesd_version) {
	case JESD204_VERSION_C:
		switch (lnk->jesd_encoder) {
		case JESD204_ENCODER_64B66B:
			encoding_n = 66; /* JESD 204C */
			encoding_d = 64;
			break;
		case JESD204_ENCODER_8B10B:
			encoding_n = 10; /* JESD 204C */
			encoding_d = 8;
			break;
		case JESD204_ENCODER_64B80B:
			encoding_n = 80; /* JESD 204C */
			encoding_d = 64;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		encoding_n = 10; /* JESD 204AB */
		encoding_d = 8;
		break;
	}

	sample_rate_div = lnk->sample_rate_div ? lnk->sample_rate_div : 1;

	rate = lnk->num_converters * lnk->bits_per_sample *
		encoding_n * lnk->sample_rate;
	do_div(rate, lnk->num_lanes * encoding_d * sample_rate_div);

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

	*lane_rate_khz = DIV_ROUND_CLOSEST_ULL(lane_rate_hz, 1000);

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

	switch (lnk->jesd_version) {
	case JESD204_VERSION_C:
		switch (lnk->jesd_encoder) {
		case JESD204_ENCODER_64B66B:
			encoding_n = 66; /* JESD 204C */
			break;
		case JESD204_ENCODER_8B10B:
			encoding_n = 40; /* JESD 204ABC */
			break;
		case JESD204_ENCODER_64B80B:
			encoding_n = 80; /* JESD 204C */
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		encoding_n = 40; /* JESD 204AB */
		break;
	}

	do_div(lane_rate_hz, encoding_n);

	*device_clock = lane_rate_hz;

	return ret;
}
EXPORT_SYMBOL_GPL(jesd204_link_get_device_clock);

void jesd204_copy_link_params(struct jesd204_link *dst,
			      const struct jesd204_link *src)
{
	dst->is_transmit = src->is_transmit;
	dst->num_lanes = src->num_lanes;
	dst->num_converters = src->num_converters;
	dst->octets_per_frame = src->octets_per_frame;
	dst->frames_per_multiframe = src->frames_per_multiframe;
	dst->num_of_multiblocks_in_emb = src->num_of_multiblocks_in_emb;
	dst->bits_per_sample = src->bits_per_sample;
	dst->converter_resolution = src->converter_resolution;
	dst->jesd_version = src->jesd_version;
	dst->jesd_encoder = src->jesd_encoder;
	dst->subclass = src->subclass;
	dst->device_id = src->device_id;
	dst->bank_id = src->bank_id;
	dst->scrambling = src->scrambling;
	dst->high_density = src->high_density;
	dst->ctrl_words_per_frame_clk = src->ctrl_words_per_frame_clk;
	dst->ctrl_bits_per_sample = src->ctrl_bits_per_sample;
	dst->samples_per_conv_frame = src->samples_per_conv_frame;
	dst->dac_adj_resolution_steps = src->dac_adj_resolution_steps;
	dst->dac_adj_direction = src->dac_adj_direction;
	dst->dac_phase_adj = src->dac_phase_adj;
}
EXPORT_SYMBOL_GPL(jesd204_copy_link_params);

int jesd204_link_get_lmfc_lemc_rate(struct jesd204_link *lnk,
				    unsigned long *rate_hz)
{
	u64 lane_rate_hz;
	u32 bkw;
	int ret;

	ret = jesd204_link_get_rate(lnk, &lane_rate_hz);
	if (ret)
		return ret;

	switch (lnk->jesd_version) {
	case JESD204_VERSION_C:
		switch (lnk->jesd_encoder) {
		case JESD204_ENCODER_64B66B:
			bkw = 66; /* JESD 204C */
			/* fall-through */
		case JESD204_ENCODER_64B80B:
			if (lnk->jesd_encoder == JESD204_ENCODER_64B80B)
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
		case JESD204_ENCODER_8B10B:
			do_div(lane_rate_hz, 10 * lnk->octets_per_frame *
				lnk->frames_per_multiframe);
			break;
		default:
			return -EINVAL;
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

struct jesd204_dev_top *jesd204_dev_get_topology_top_dev(struct jesd204_dev *jdev)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);

	if (jdev_top)
		return jdev_top;

	/* FIXME: enforce that one jdev object can only be in one topology */
	list_for_each_entry(jdev_top, &jesd204_topologies, entry) {
		if (!jesd204_dev_has_con_in_topology(jdev, jdev_top))
			continue;
		return jdev_top;
	}

	jesd204_warn(jdev, "Device isn't a top-device, nor does it belong to topology with top-device\n");

	return NULL;
}

int jesd204_sysref_async(struct jesd204_dev *jdev)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_get_topology_top_dev(jdev);
	const struct jesd204_dev_data *dev_data;

	if (!jdev_top)
		return -EFAULT;

	/* No SYSREF registered for this topology */
	if (!jdev_top->jdev_sysref)
		return 0;

	if (!jdev_top->jdev_sysref->dev_data)
		return -EFAULT;

	dev_data = jdev_top->jdev_sysref->dev_data;

	/* By now, this should have been validated to have sysref_cb() */
	return dev_data->sysref_cb(jdev_top->jdev_sysref);
}
EXPORT_SYMBOL(jesd204_sysref_async);

int jesd204_sysref_async_force(struct jesd204_dev *jdev)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_get_topology_top_dev(jdev);
	const struct jesd204_dev_data *dev_data;

	if (!jdev_top)
		return -EFAULT;

	/* Primary SYSREF registered for this topology? */
	if (jdev_top->jdev_sysref)
		return jesd204_sysref_async(jdev);

	/* No SYSREF registered for this topology */
	if (!jdev_top->jdev_sysref_sec)
		return 0;

	if (!jdev_top->jdev_sysref_sec->dev_data)
		return -EFAULT;

	dev_data = jdev_top->jdev_sysref_sec->dev_data;

	/* By now, this should have been validated to have sysref_cb() */
	return dev_data->sysref_cb(jdev_top->jdev_sysref_sec);
}
EXPORT_SYMBOL(jesd204_sysref_async_force);

bool jesd204_dev_is_top(struct jesd204_dev *jdev)
{
	return jdev && jdev->is_top;
}
EXPORT_SYMBOL(jesd204_dev_is_top);

static inline bool dev_is_jesd204_dev(struct device *dev)
{
	return device_property_read_bool(dev, "jesd204-device");
}

void *jesd204_dev_priv(struct jesd204_dev *jdev)
{
	return jdev->priv;
}
EXPORT_SYMBOL_GPL(jesd204_dev_priv);

struct jesd204_dev *jesd204_dev_from_device(struct device *dev)
{
	struct jesd204_dev *jdev;

	if (!dev)
		return NULL;

	list_for_each_entry(jdev, &jesd204_device_list, entry) {
		if (jdev->dev.parent && jdev->dev.parent == dev)
			return jdev;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(jesd204_dev_from_device);

struct device *jesd204_dev_to_device(struct jesd204_dev *jdev)
{
	return jdev ? jdev->dev.parent : NULL;
}
EXPORT_SYMBOL_GPL(jesd204_dev_to_device);

static void __jesd204_printk(const char *level, const struct jesd204_dev *jdev,
			     struct va_format *vaf)
{
	const struct device *dev;

	if (!jdev) {
		printk("%sjesd204: (NULL jesd204 device *): %pV", level, vaf);
		return;
	}

	if (!jdev->dev.parent) {
		printk("%sjesd204: %pOF: %pV", level, jdev->np, vaf);
		return;
	}

	dev = &jdev->dev;
	dev_printk_emit(level[1] - '0', dev, "jesd204: %pOF,%s,parent=%s: %pV",
			jdev->np,
			dev_name(dev),
			dev_name(dev->parent),
			vaf);
}

void jesd204_printk(const char *level, const struct jesd204_dev *jdev,
		    const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	__jesd204_printk(level, jdev, &vaf);

	va_end(args);
}

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

static int jesd204_dev_init_stop_states(struct jesd204_dev *jdev,
					struct device_node *np)
{
	unsigned int stop_states[JESD204_FSM_STATES_NUM];
	int fsm_state;
	int i, ret;

	ret = of_property_read_variable_u32_array(np, "jesd204-stop-states",
						  stop_states,
						  1, JESD204_FSM_STATES_NUM);
	if (ret <= 0)
		return 0;

	for (i = 0; i < ret; i++) {
		if (stop_states[i] >= JESD204_FSM_STATES_NUM) {
			pr_err("%pOF: Invalid state ID %u\n", np,
			       stop_states[i]);
			return -EINVAL;
		}
		jdev->stop_states[stop_states[i]] = true;
		fsm_state = stop_states[i] + JESD204_STATE_FSM_OFFSET;
		pr_info("%pOF: stop state: '%s'\n", np,
			jesd204_state_str(fsm_state));
	}

	return 0;
}

static struct jesd204_dev *jesd204_dev_alloc(struct device_node *np)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_dev *jdev;
	unsigned int link_ids[JESD204_MAX_LINKS];
	u32 topo_id;
	int i, ret, id;

	id = ida_simple_get(&jesd204_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		pr_err("%pOF: Unable to get unique ID for device\n", np);
		return ERR_PTR(id);
	}

	if (of_property_read_u32(np, "jesd204-top-device", &topo_id) == 0) {
		ret = of_property_read_variable_u32_array(np,
							  "jesd204-link-ids",
							  link_ids,
							  1,
							  JESD204_MAX_LINKS);
		if (ret < 0) {
			pr_err("%pOF error getting 'jesd204-link-ids': %d\n",
			       np, ret);
			goto err_free_id;
		}

		jdev_top = kzalloc(sizeof(*jdev_top), GFP_KERNEL);
		if (!jdev_top) {
			ret = -ENOMEM;
			goto err_free_id;
		}

		jdev = &jdev_top->jdev;

		jdev_top->topo_id = topo_id;
		jdev_top->num_links = ret;
		for (i = 0; i < jdev_top->num_links; i++)
			jdev_top->link_ids[i] = link_ids[i];

		ret = jesd204_dev_alloc_links(jdev_top);
		if (ret) {
			kfree(jdev_top);
			goto err_free_id;
		}

		jdev->is_top = true;

		list_add(&jdev_top->entry, &jesd204_topologies);
		jesd204_topologies_count++;
	} else {
		jdev = kzalloc(sizeof(*jdev), GFP_KERNEL);
		if (!jdev) {
			ret = -ENOMEM;
			goto err_free_id;
		}
	}

	jdev->is_sysref_provider = of_property_read_bool(np,
		"jesd204-sysref-provider");
	jdev->is_sec_sysref_provider = of_property_read_bool(np,
		"jesd204-secondary-sysref-provider");

	ret = jesd204_dev_init_stop_states(jdev, np);
	if (ret)
		goto err_free_id;

	jdev->id = id;
	jdev->np = of_node_get(np);

	INIT_LIST_HEAD(&jdev->outputs);

	list_add(&jdev->entry, &jesd204_device_list);
	jesd204_device_count++;

	return jdev;

err_free_id:
	ida_simple_remove(&jesd204_ida, id);

	return ERR_PTR(ret);
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

		con->id = jesd204_con_id_counter++;
		con->topo_id = args->args[0];
		con->link_id = args->args[1];
		con->link_idx = JESD204_LINKS_ALL;
		con->owner = jdev_in;
		INIT_LIST_HEAD(&con->dests);

		memcpy(&con->of, args, sizeof(con->of));
		list_add(&con->entry, &jdev_in->outputs);
		jdev_in->outputs_count++;
	}

	pr_info("created con: id=%u, topo=%u, link=%u, %pOF <-> %pOF\n",
		con->id, con->topo_id, con->link_id, con->owner->np, jdev->np);

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

	for_each_node_with_property(np, "jesd204-device") {
		jdev = jesd204_dev_alloc(np);
		if (IS_ERR(jdev))
			return PTR_ERR(jdev);
	}

	list_for_each_entry(jdev, &jesd204_device_list, entry) {
		ret = jesd204_of_device_create_cons(jdev);
		if (ret)
			return ret;
	}

	list_for_each_entry(jdev_top, &jesd204_topologies, entry) {
		ret = jesd204_init_topology(jdev_top);
		if (ret)
			return ret;
	}

	return 0;
}

static int jesd204_dev_init_link_lane_ids(struct jesd204_dev_top *jdev_top,
					  unsigned int link_idx,
					  struct jesd204_link *jlink)
{
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct device *dev = jdev->dev.parent;
	u8 id;

	if (!jlink->num_lanes) {
		jesd204_err(jdev, "JESD204 link [%u] number of lanes is 0\n",
			    jlink->link_id);
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
	ol->link.sysref.lmfc_offset = JESD204_LMFC_OFFSET_UNINITIALIZED;
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

static int jesd204_dev_init_links_data(struct device *parent,
				       struct jesd204_dev *jdev,
				       const struct jesd204_dev_data *init)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	unsigned int i;

	if (!jdev_top)
		return 0;

	if (!init->max_num_links) {
		jesd204_err(jdev, "max_num_links shouldn't be zero\n");
		return -EINVAL;
	}

	if (init->max_num_links < jdev_top->num_links) {
		jesd204_err(jdev,
			"Driver supports %u number of links, DT specified %u\n",
			init->max_num_links, jdev_top->num_links);
		return -EINVAL;
	}

	/**
	 * Framework users should provide at least initial JESD204 link data,
	 * or a link init op/callback which should do JESD204 link init.
	 */
	if (!init->links &&
	    !init->state_ops &&
	    !init->state_ops[JESD204_OP_LINK_INIT].per_link) {
		jesd204_err(jdev,
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

static void jesd204_dev_init_top_data(struct jesd204_dev *jdev,
				      const struct jesd204_dev_data *init)
{
	struct jesd204_dev_top *jdev_top;

	jdev_top = jesd204_dev_top_dev(jdev);
	if (!jdev_top)
		return;

	jdev_top->num_retries = init->num_retries;
}

static struct jesd204_dev *jesd204_dev_register(struct device *dev,
						const struct jesd204_dev_data *init)
{
	struct jesd204_dev *jdev;
	int ret;

	if (!dev || !init) {
		dev_err(dev, "Invalid register arguments\n");
		return ERR_PTR(-EINVAL);
	}

	jdev = jesd204_dev_from_device(dev);
	if (jdev) {
		dev_err(dev, "Device already registered with framework\n");
		return ERR_PTR(-EEXIST);
	}

	jdev = jesd204_dev_find_by_of_node(dev->of_node);
	if (!jdev) {
		dev_err(dev, "Device has no configuration node\n");
		return ERR_PTR(-ENODEV);
	}

	ret = jesd204_dev_init_links_data(dev, jdev, init);
	if (ret)
		return ERR_PTR(ret);

	jesd204_dev_init_top_data(jdev, init);

	jdev->dev.parent = dev;
	jdev->dev_data = init;
	jdev->dev.bus = &jesd204_bus_type;
	device_initialize(&jdev->dev);
	dev_set_name(&jdev->dev, "jesd204:%d", jdev->id);

	ret = device_add(&jdev->dev);
	if (ret) {
		put_device(&jdev->dev);
		goto err_uninit_device;
	}

	if (init->sizeof_priv) {
		jdev->priv = devm_kzalloc(dev, init->sizeof_priv,
					  GFP_KERNEL);
		if (!jdev->priv) {
			ret = -ENOMEM;
			goto err_device_del;
		}
	}

	ret = jesd204_dev_create_sysfs(jdev);
	if (ret)
		goto err_device_del;

	return jdev;

err_device_del:
	device_del(&jdev->dev);
err_uninit_device:
	memset(&jdev->dev, 0, sizeof(jdev->dev));

	return ERR_PTR(ret);
}

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
		list_del(&jdev->entry);
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

/**
 * jesd204_dev_unregister() - unregister a device from the JESD204 subsystem
 * @jdev:		Device structure representing the device.
 **/
static void jesd204_dev_unregister(struct jesd204_dev *jdev)
{
	if (IS_ERR_OR_NULL(jdev))
		return;

	jesd204_dev_destroy_sysfs(jdev);

	if (jdev->dev.parent)
		device_del(&jdev->dev);

	jesd204_fsm_stop(jdev, JESD204_LINKS_ALL);

	memset(&jdev->dev, 0, sizeof(jdev->dev));
}

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

	BUILD_BUG_ON(__JESD204_MAX_OPS != JESD204_FSM_STATES_NUM);

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
	bus_unregister(&jesd204_bus_type);
}

subsys_initcall(jesd204_init);
module_exit(jesd204_exit);

MODULE_AUTHOR("Alexandru Ardelean <alexandru.ardelean@analog.com>");
MODULE_DESCRIPTION("JESD204 framework core");
MODULE_LICENSE("GPL");
