// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim SerDes devices' glue layer
 *
 * Copyright 2023 NXP
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/workqueue.h>

struct maxim_serdes_late_setup {
	void (*cb)(void *data);
	void *data;
};

/* A SerDes chain is formed of one local device and multiple remote devices. */
struct maxim_serdes_chain {
	struct list_head entry;
	struct work_struct late_setup_work;

	bool setup_finished;

	struct device_node *local_dev_node;
	struct maxim_serdes_late_setup local_late_setup;

	int remotes_no;
	struct maxim_serdes_late_setup remote_late_setup[];
};

static LIST_HEAD(maxim_serdes_chains_list);

static void maxim_serdes_worker(struct work_struct *work)
{
	struct maxim_serdes_chain *chain;
	int i;

	chain = container_of(work, struct maxim_serdes_chain, late_setup_work);

	chain->local_late_setup.cb(chain->local_late_setup.data);

	for (i = 0; i < chain->remotes_no; i++)
		if (chain->remote_late_setup[i].cb)
			chain->remote_late_setup[i].cb(chain->remote_late_setup[i].data);

	chain->setup_finished = true;
}

static struct maxim_serdes_chain *maxim_serdes_get_chain(struct device_node *local_dev_node)
{
	struct maxim_serdes_chain *serdes_chain;

	list_for_each_entry(serdes_chain, &maxim_serdes_chains_list, entry) {
		if (serdes_chain->local_dev_node == local_dev_node)
			return serdes_chain;
	}

	return NULL;
}

int maxim_serdes_chain_register_remote(struct device *dev, int link, void (*cb)(void *), void *data)
{
	struct maxim_serdes_chain *serdes_chain;
	struct device_node *dn;
	int i;

	dn = of_node_get(dev->of_node);

	while (dn) {
		serdes_chain = maxim_serdes_get_chain(dn);
		if (serdes_chain) {
			of_node_put(dn);
			break;
		}

		dn = of_get_next_parent(dn);
	}

	if (!dn) {
		dev_err(dev, "could not find the local device of_node in the chains list\n");
		return -ENODEV;
	}

	if (link > serdes_chain->remotes_no - 1) {
		dev_err(dev, "invalid remote link id provided\n");
		return -EINVAL;
	}

	serdes_chain->remote_late_setup[link].cb = cb;
	serdes_chain->remote_late_setup[link].data = data;

	/* let's find out if all remotes registered their late setup routines */
	for (i = 0; i < serdes_chain->remotes_no; i++) {
		if (!serdes_chain->remote_late_setup[i].cb)
			return 0;
	}

	/* we're good to go, let's queue the late setup work */
	INIT_WORK(&serdes_chain->late_setup_work, maxim_serdes_worker);

	queue_work(system_unbound_wq, &serdes_chain->late_setup_work);

	return 0;
};
EXPORT_SYMBOL(maxim_serdes_chain_register_remote);

int maxim_serdes_chain_register_local(struct device *dev, int remote_links_used,
				      void (*cb)(void *), void *data)
{
	struct maxim_serdes_chain *chain;
	int extra_space;

	extra_space = remote_links_used * sizeof(struct maxim_serdes_late_setup);
	chain = devm_kzalloc(dev, sizeof(*chain) + extra_space, GFP_KERNEL);
	if (!chain)
		return -ENOMEM;

	chain->local_dev_node = dev->of_node;
	chain->local_late_setup.cb = cb;
	chain->local_late_setup.data = data;
	chain->remotes_no = remote_links_used;

	list_add_tail(&chain->entry, &maxim_serdes_chains_list);

	return 0;
}
EXPORT_SYMBOL(maxim_serdes_chain_register_local);

MODULE_DESCRIPTION("Maxim SerDes glue layer");
MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@oss.nxp.com>");
MODULE_LICENSE("GPL v2");
