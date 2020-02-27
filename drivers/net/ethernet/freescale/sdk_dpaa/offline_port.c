/* Copyright 2011-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Offline Parsing / Host Command port driver for FSL QorIQ FMan.
 * Validates device-tree configuration and sets up the offline ports.
 */

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__
#else
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": " fmt
#endif


#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/fsl_qman.h>

#include "offline_port.h"
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"

#define OH_MOD_DESCRIPTION	"FSL FMan Offline Parsing port driver"
/* Manip extra space and data alignment for fragmentation */
#define FRAG_MANIP_SPACE 128
#define FRAG_DATA_ALIGN 64


MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Bogdan Hamciuc <bogdan.hamciuc@freescale.com>");
MODULE_DESCRIPTION(OH_MOD_DESCRIPTION);


static const struct of_device_id oh_port_match_table[] = {
	{
		.compatible	= "fsl,dpa-oh"
	},
	{
		.compatible	= "fsl,dpa-oh-shared"
	},
	{}
};
MODULE_DEVICE_TABLE(of, oh_port_match_table);

#ifdef CONFIG_PM

static int oh_suspend(struct device *dev)
{
	struct dpa_oh_config_s	*oh_config;

	oh_config = dev_get_drvdata(dev);
	return fm_port_suspend(oh_config->oh_port);
}

static int oh_resume(struct device *dev)
{
	struct dpa_oh_config_s	*oh_config;

	oh_config = dev_get_drvdata(dev);
	return fm_port_resume(oh_config->oh_port);
}

static const struct dev_pm_ops oh_pm_ops = {
	.suspend = oh_suspend,
	.resume = oh_resume,
};

#define OH_PM_OPS (&oh_pm_ops)

#else /* CONFIG_PM */

#define OH_PM_OPS NULL

#endif /* CONFIG_PM */

/* Creates Frame Queues */
static uint32_t oh_fq_create(struct qman_fq *fq,
	uint32_t fq_id, uint16_t channel,
	uint16_t wq_id)
{
	struct qm_mcc_initfq fq_opts;
	uint32_t create_flags, init_flags;
	uint32_t ret = 0;

	if (fq == NULL)
		return 1;

	/* Set flags for FQ create */
	create_flags = QMAN_FQ_FLAG_LOCKED | QMAN_FQ_FLAG_TO_DCPORTAL;

	/* Create frame queue */
	ret = qman_create_fq(fq_id, create_flags, fq);
	if (ret != 0)
		return 1;

	/* Set flags for FQ init */
	init_flags = QMAN_INITFQ_FLAG_SCHED;

	/* Set FQ init options. Specify destination WQ ID and channel */
	fq_opts.we_mask = QM_INITFQ_WE_DESTWQ;
	fq_opts.fqd.dest.wq = wq_id;
	fq_opts.fqd.dest.channel = channel;

	/* Initialize frame queue */
	ret = qman_init_fq(fq, init_flags, &fq_opts);
	if (ret != 0) {
		qman_destroy_fq(fq, 0);
		return 1;
	}

	return 0;
}

static void dump_fq(struct device *dev, int fqid, uint16_t channel)
{
	if (channel) {
		/* display fqs with a valid (!= 0) destination channel */
		dev_info(dev, "FQ ID:%d Channel ID:%d\n", fqid, channel);
	}
}

static void dump_fq_duple(struct device *dev, struct qman_fq *fqs,
		int fqs_count, uint16_t channel_id)
{
	int i;
	for (i = 0; i < fqs_count; i++)
		dump_fq(dev, (fqs + i)->fqid, channel_id);
}

static void dump_oh_config(struct device *dev, struct dpa_oh_config_s *conf)
{
	struct list_head *fq_list;
	struct fq_duple *fqd;
	int i;

	dev_info(dev, "Default egress frame queue: %d\n", conf->default_fqid);
	dev_info(dev, "Default error frame queue: %d\n", conf->error_fqid);

	/* TX queues (old initialization) */
	dev_info(dev, "Initialized queues:");
	for (i = 0; i < conf->egress_cnt; i++)
		dump_fq_duple(dev, conf->egress_fqs, conf->egress_cnt,
				conf->channel);

	/* initialized ingress queues */
	list_for_each(fq_list, &conf->fqs_ingress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);
		dump_fq_duple(dev, fqd->fqs, fqd->fqs_count, fqd->channel_id);
	}

	/* initialized egress queues */
	list_for_each(fq_list, &conf->fqs_egress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);
		dump_fq_duple(dev, fqd->fqs, fqd->fqs_count, fqd->channel_id);
	}
}

/* Destroys Frame Queues */
static void oh_fq_destroy(struct qman_fq *fq)
{
	int _errno = 0;

	_errno = qman_retire_fq(fq, NULL);
	if (unlikely(_errno < 0))
		pr_err(KBUILD_MODNAME": %s:%hu:%s(): qman_retire_fq(%u)=%d\n",
			KBUILD_BASENAME".c", __LINE__, __func__,
			qman_fq_fqid(fq), _errno);

	_errno = qman_oos_fq(fq);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME": %s:%hu:%s(): qman_oos_fq(%u)=%d\n",
			KBUILD_BASENAME".c", __LINE__, __func__,
			qman_fq_fqid(fq), _errno);
	}

	qman_destroy_fq(fq, 0);
}

/* Allocation code for the OH port's PCD frame queues */
static int __cold oh_alloc_pcd_fqids(struct device *dev,
	uint32_t num,
	uint8_t alignment,
	uint32_t *base_fqid)
{
	dev_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

static int __cold oh_free_pcd_fqids(struct device *dev, uint32_t base_fqid)
{
	dev_crit(dev, "callback not implemented!\n");
	BUG();

	return 0;
}

static void oh_set_buffer_layout(struct fm_port *port,
				 struct dpa_buffer_layout_s *layout)
{
	struct fm_port_params params;

	layout->priv_data_size = DPA_TX_PRIV_DATA_SIZE;
	layout->parse_results = true;
	layout->hash_results = true;
	layout->time_stamp = false;

	fm_port_get_buff_layout_ext_params(port, &params);
	layout->manip_extra_space = params.manip_extra_space;
	layout->data_align = params.data_align;
}

static int
oh_port_probe(struct platform_device *_of_dev)
{
	struct device *dpa_oh_dev;
	struct device_node *dpa_oh_node;
	int lenp, _errno = 0, fq_idx, duple_idx;
	int n_size, i, j, ret, duples_count;
	struct platform_device *oh_of_dev;
	struct device_node *oh_node, *bpool_node = NULL, *root_node;
	struct device *oh_dev;
	struct dpa_oh_config_s *oh_config = NULL;
	const __be32 *oh_all_queues;
	const __be32 *channel_ids;
	const __be32 *oh_tx_queues;
	uint32_t queues_count;
	uint32_t crt_fqid_base;
	uint32_t crt_fq_count;
	bool frag_enabled = false;
	struct fm_port_params oh_port_tx_params;
	struct fm_port_pcd_param oh_port_pcd_params;
	struct dpa_buffer_layout_s buf_layout;

	/* True if the current partition owns the OH port. */
	bool init_oh_port;

	const struct of_device_id *match;
	int crt_ext_pools_count;
	u32 ext_pool_size;
	u32 port_id;
	u32 channel_id;

	int channel_ids_count;
	int channel_idx;
	struct fq_duple *fqd;
	struct list_head *fq_list, *fq_list_tmp;

	const __be32 *bpool_cfg;
	uint32_t bpid;

	memset(&oh_port_tx_params, 0, sizeof(oh_port_tx_params));
	dpa_oh_dev = &_of_dev->dev;
	dpa_oh_node = dpa_oh_dev->of_node;
	BUG_ON(dpa_oh_node == NULL);

	match = of_match_device(oh_port_match_table, dpa_oh_dev);
	if (!match)
		return -EINVAL;

	dev_dbg(dpa_oh_dev, "Probing OH port...\n");

	/* Find the referenced OH node */
	oh_node = of_parse_phandle(dpa_oh_node, "fsl,fman-oh-port", 0);
	if (oh_node == NULL) {
		dev_err(dpa_oh_dev,
			"Can't find OH node referenced from node %s\n",
			dpa_oh_node->full_name);
		return -EINVAL;
	}
	dev_info(dpa_oh_dev, "Found OH node handle compatible with %s\n",
		match->compatible);

	_errno = of_property_read_u32(oh_node, "cell-index", &port_id);
	if (_errno) {
		dev_err(dpa_oh_dev, "No port id found in node %s\n",
			dpa_oh_node->full_name);
		goto return_kfree;
	}

	_errno = of_property_read_u32(oh_node, "fsl,qman-channel-id",
			&channel_id);
	if (_errno) {
		dev_err(dpa_oh_dev, "No channel id found in node %s\n",
			dpa_oh_node->full_name);
		goto return_kfree;
	}

	oh_of_dev = of_find_device_by_node(oh_node);
	BUG_ON(oh_of_dev == NULL);
	oh_dev = &oh_of_dev->dev;

	/* The OH port must be initialized exactly once.
	 * The following scenarios are of interest:
	 *	- the node is Linux-private (will always initialize it);
	 *	- the node is shared between two Linux partitions
	 *	  (only one of them will initialize it);
	 *	- the node is shared between a Linux and a LWE partition
	 *	  (Linux will initialize it) - "fsl,dpa-oh-shared"
	 */

	/* Check if the current partition owns the OH port
	 * and ought to initialize it. It may be the case that we leave this
	 * to another (also Linux) partition.
	 */
	init_oh_port = strcmp(match->compatible, "fsl,dpa-oh-shared");

	/* If we aren't the "owner" of the OH node, we're done here. */
	if (!init_oh_port) {
		dev_dbg(dpa_oh_dev,
			"Not owning the shared OH port %s, will not initialize it.\n",
			oh_node->full_name);
		of_node_put(oh_node);
		return 0;
	}

	/* Allocate OH dev private data */
	oh_config = devm_kzalloc(dpa_oh_dev, sizeof(*oh_config), GFP_KERNEL);
	if (oh_config == NULL) {
		dev_err(dpa_oh_dev,
			"Can't allocate private data for OH node %s referenced from node %s!\n",
			oh_node->full_name, dpa_oh_node->full_name);
		_errno = -ENOMEM;
		goto return_kfree;
	}

	INIT_LIST_HEAD(&oh_config->fqs_ingress_list);
	INIT_LIST_HEAD(&oh_config->fqs_egress_list);

	/* FQs that enter OH port */
	lenp = 0;
	oh_all_queues = of_get_property(dpa_oh_node,
		"fsl,qman-frame-queues-ingress", &lenp);
	if (lenp % (2 * sizeof(*oh_all_queues))) {
		dev_warn(dpa_oh_dev,
			"Wrong ingress queues format for OH node %s referenced from node %s!\n",
			oh_node->full_name, dpa_oh_node->full_name);
		/* just ignore the last unpaired value */
	}

	duples_count = lenp / (2 * sizeof(*oh_all_queues));
	dev_err(dpa_oh_dev, "Allocating %d ingress frame queues duples\n",
			duples_count);
	for (duple_idx = 0; duple_idx < duples_count; duple_idx++) {
		crt_fqid_base = be32_to_cpu(oh_all_queues[2 * duple_idx]);
		crt_fq_count = be32_to_cpu(oh_all_queues[2 * duple_idx + 1]);

		fqd = devm_kzalloc(dpa_oh_dev,
				sizeof(struct fq_duple), GFP_KERNEL);
		if (!fqd) {
			dev_err(dpa_oh_dev, "Can't allocate structures for ingress frame queues for OH node %s referenced from node %s!\n",
					oh_node->full_name,
					dpa_oh_node->full_name);
			_errno = -ENOMEM;
			goto return_kfree;
		}

		fqd->fqs = devm_kzalloc(dpa_oh_dev,
				crt_fq_count * sizeof(struct qman_fq),
				GFP_KERNEL);
		if (!fqd->fqs) {
			dev_err(dpa_oh_dev, "Can't allocate structures for ingress frame queues for OH node %s referenced from node %s!\n",
					oh_node->full_name,
					dpa_oh_node->full_name);
			_errno = -ENOMEM;
			goto return_kfree;
		}

		for (j = 0; j < crt_fq_count; j++)
			(fqd->fqs + j)->fqid = crt_fqid_base + j;
		fqd->fqs_count = crt_fq_count;
		fqd->channel_id = (uint16_t)channel_id;
		list_add(&fqd->fq_list, &oh_config->fqs_ingress_list);
	}

	/* create the ingress queues */
	list_for_each(fq_list, &oh_config->fqs_ingress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);

		for (j = 0; j < fqd->fqs_count; j++) {
			ret = oh_fq_create(fqd->fqs + j,
					(fqd->fqs + j)->fqid,
					fqd->channel_id, 3);
			if (ret != 0) {
				dev_err(dpa_oh_dev, "Unable to create ingress frame queue %d for OH node %s referenced from node %s!\n",
						(fqd->fqs + j)->fqid,
						oh_node->full_name,
						dpa_oh_node->full_name);
				_errno = -EINVAL;
				goto return_kfree;
			}
		}
	}

	/* FQs that exit OH port */
	lenp = 0;
	oh_all_queues = of_get_property(dpa_oh_node,
		"fsl,qman-frame-queues-egress", &lenp);
	if (lenp % (2 * sizeof(*oh_all_queues))) {
		dev_warn(dpa_oh_dev,
			"Wrong egress queues format for OH node %s referenced from node %s!\n",
			oh_node->full_name, dpa_oh_node->full_name);
		/* just ignore the last unpaired value */
	}

	duples_count = lenp / (2 * sizeof(*oh_all_queues));
	dev_dbg(dpa_oh_dev, "Allocating %d egress frame queues duples\n",
			duples_count);
	for (duple_idx = 0; duple_idx < duples_count; duple_idx++) {
		crt_fqid_base = be32_to_cpu(oh_all_queues[2 * duple_idx]);
		crt_fq_count = be32_to_cpu(oh_all_queues[2 * duple_idx + 1]);

		fqd = devm_kzalloc(dpa_oh_dev,
				sizeof(struct fq_duple), GFP_KERNEL);
		if (!fqd) {
			dev_err(dpa_oh_dev, "Can't allocate structures for egress frame queues for OH node %s referenced from node %s!\n",
					oh_node->full_name,
					dpa_oh_node->full_name);
			_errno = -ENOMEM;
			goto return_kfree;
		}

		fqd->fqs = devm_kzalloc(dpa_oh_dev,
				crt_fq_count * sizeof(struct qman_fq),
				GFP_KERNEL);
		if (!fqd->fqs) {
			dev_err(dpa_oh_dev,
					"Can't allocate structures for egress frame queues for OH node %s referenced from node %s!\n",
					oh_node->full_name,
					dpa_oh_node->full_name);
			_errno = -ENOMEM;
			goto return_kfree;
		}

		for (j = 0; j < crt_fq_count; j++)
			(fqd->fqs + j)->fqid = crt_fqid_base + j;
		fqd->fqs_count = crt_fq_count;
		/* channel ID is specified in another attribute */
		fqd->channel_id = 0;
		list_add_tail(&fqd->fq_list, &oh_config->fqs_egress_list);

		/* allocate the queue */

	}

	/* channel_ids for FQs that exit OH port */
	lenp = 0;
	channel_ids = of_get_property(dpa_oh_node,
		"fsl,qman-channel-ids-egress", &lenp);

	channel_ids_count = lenp / (sizeof(*channel_ids));
	if (channel_ids_count != duples_count) {
		dev_warn(dpa_oh_dev,
			"Not all egress queues have a channel id for OH node %s referenced from node %s!\n",
			oh_node->full_name, dpa_oh_node->full_name);
		/* just ignore the queues that do not have a Channel ID */
	}

	channel_idx = 0;
	list_for_each(fq_list, &oh_config->fqs_egress_list) {
		if (channel_idx + 1 > channel_ids_count)
			break;
		fqd = list_entry(fq_list, struct fq_duple, fq_list);
		fqd->channel_id =
			(uint16_t)be32_to_cpu(channel_ids[channel_idx++]);
	}

	/* create egress queues */
	list_for_each(fq_list, &oh_config->fqs_egress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);

		if (fqd->channel_id == 0) {
			/* missing channel id in dts */
			continue;
		}

		for (j = 0; j < fqd->fqs_count; j++) {
			ret = oh_fq_create(fqd->fqs + j,
					(fqd->fqs + j)->fqid,
					fqd->channel_id, 3);
			if (ret != 0) {
				dev_err(dpa_oh_dev, "Unable to create egress frame queue %d for OH node %s referenced from node %s!\n",
						(fqd->fqs + j)->fqid,
						oh_node->full_name,
						dpa_oh_node->full_name);
				_errno = -EINVAL;
				goto return_kfree;
			}
		}
	}

	/* Read FQ ids/nums for the DPA OH node */
	oh_all_queues = of_get_property(dpa_oh_node,
		"fsl,qman-frame-queues-oh", &lenp);
	if (oh_all_queues == NULL) {
		dev_err(dpa_oh_dev,
			"No frame queues have been defined for OH node %s referenced from node %s\n",
			oh_node->full_name, dpa_oh_node->full_name);
		_errno = -EINVAL;
		goto return_kfree;
	}

	/* Check that the OH error and default FQs are there */
	BUG_ON(lenp % (2 * sizeof(*oh_all_queues)));
	queues_count = lenp / (2 * sizeof(*oh_all_queues));
	if (queues_count != 2) {
		dev_err(dpa_oh_dev,
			"Error and Default queues must be defined for OH node %s referenced from node %s\n",
			oh_node->full_name, dpa_oh_node->full_name);
		_errno = -EINVAL;
		goto return_kfree;
	}

	/* Read the FQIDs defined for this OH port */
	dev_dbg(dpa_oh_dev, "Reading %d queues...\n", queues_count);
	fq_idx = 0;

	/* Error FQID - must be present */
	crt_fqid_base = be32_to_cpu(oh_all_queues[fq_idx++]);
	crt_fq_count = be32_to_cpu(oh_all_queues[fq_idx++]);
	if (crt_fq_count != 1) {
		dev_err(dpa_oh_dev,
			"Only 1 Error FQ allowed in OH node %s referenced from node %s (read: %d FQIDs).\n",
			oh_node->full_name, dpa_oh_node->full_name,
			crt_fq_count);
		_errno = -EINVAL;
		goto return_kfree;
	}
	oh_config->error_fqid = crt_fqid_base;
	dev_dbg(dpa_oh_dev, "Read Error FQID 0x%x for OH port %s.\n",
		oh_config->error_fqid, oh_node->full_name);

	/* Default FQID - must be present */
	crt_fqid_base = be32_to_cpu(oh_all_queues[fq_idx++]);
	crt_fq_count = be32_to_cpu(oh_all_queues[fq_idx++]);
	if (crt_fq_count != 1) {
		dev_err(dpa_oh_dev,
			"Only 1 Default FQ allowed in OH node %s referenced from %s (read: %d FQIDs).\n",
			oh_node->full_name, dpa_oh_node->full_name,
			crt_fq_count);
		_errno = -EINVAL;
		goto return_kfree;
	}
	oh_config->default_fqid = crt_fqid_base;
	dev_dbg(dpa_oh_dev, "Read Default FQID 0x%x for OH port %s.\n",
		oh_config->default_fqid, oh_node->full_name);

	/* TX FQID - presence is optional */
	oh_tx_queues = of_get_property(dpa_oh_node, "fsl,qman-frame-queues-tx",
			&lenp);
	if (oh_tx_queues == NULL) {
		dev_dbg(dpa_oh_dev,
			"No tx queues have been defined for OH node %s referenced from node %s\n",
			oh_node->full_name, dpa_oh_node->full_name);
		goto config_port;
	}

	/* Check that queues-tx has only a base and a count defined */
	BUG_ON(lenp % (2 * sizeof(*oh_tx_queues)));
	queues_count = lenp / (2 * sizeof(*oh_tx_queues));
	if (queues_count != 1) {
		dev_err(dpa_oh_dev,
			"TX queues must be defined in only one <base count> tuple for OH node %s referenced from node %s\n",
			oh_node->full_name, dpa_oh_node->full_name);
		_errno = -EINVAL;
		goto return_kfree;
	}

	fq_idx = 0;
	crt_fqid_base = be32_to_cpu(oh_tx_queues[fq_idx++]);
	crt_fq_count = be32_to_cpu(oh_tx_queues[fq_idx++]);
	oh_config->egress_cnt = crt_fq_count;

	/* Allocate TX queues */
	dev_dbg(dpa_oh_dev, "Allocating %d queues for TX...\n", crt_fq_count);
	oh_config->egress_fqs = devm_kzalloc(dpa_oh_dev,
		crt_fq_count * sizeof(struct qman_fq), GFP_KERNEL);
	if (oh_config->egress_fqs == NULL) {
		dev_err(dpa_oh_dev,
			"Can't allocate private data for TX queues for OH node %s referenced from node %s!\n",
			oh_node->full_name, dpa_oh_node->full_name);
		_errno = -ENOMEM;
		goto return_kfree;
	}

	/* Create TX queues */
	for (i = 0; i < crt_fq_count; i++) {
		ret = oh_fq_create(oh_config->egress_fqs + i,
			crt_fqid_base + i, (uint16_t)channel_id, 3);
		if (ret != 0) {
			dev_err(dpa_oh_dev,
				"Unable to create TX frame queue %d for OH node %s referenced from node %s!\n",
				crt_fqid_base + i, oh_node->full_name,
				dpa_oh_node->full_name);
			_errno = -EINVAL;
			goto return_kfree;
		}
	}

config_port:
	/* Get a handle to the fm_port so we can set
	 * its configuration params
	 */
	oh_config->oh_port = fm_port_bind(oh_dev);
	if (oh_config->oh_port == NULL) {
		dev_err(dpa_oh_dev, "NULL drvdata from fm port dev %s!\n",
			oh_node->full_name);
		_errno = -EINVAL;
		goto return_kfree;
	}

	oh_set_buffer_layout(oh_config->oh_port, &buf_layout);

	/* read the pool handlers */
	crt_ext_pools_count = of_count_phandle_with_args(dpa_oh_node,
			"fsl,bman-buffer-pools", NULL);
	if (crt_ext_pools_count <= 0) {
		dev_info(dpa_oh_dev,
			 "OH port %s has no buffer pool. Fragmentation will not be enabled\n",
			oh_node->full_name);
		goto init_port;
	}

	/* used for reading ext_pool_size*/
	root_node = of_find_node_by_path("/");
	if (root_node == NULL) {
		dev_err(dpa_oh_dev, "of_find_node_by_path(/) failed\n");
		_errno = -EINVAL;
		goto return_kfree;
	}

	n_size = of_n_size_cells(root_node);
	of_node_put(root_node);

	dev_dbg(dpa_oh_dev, "OH port number of pools = %d\n",
			crt_ext_pools_count);

	oh_port_tx_params.num_pools = (uint8_t)crt_ext_pools_count;

	for (i = 0; i < crt_ext_pools_count; i++) {
		bpool_node = of_parse_phandle(dpa_oh_node,
				"fsl,bman-buffer-pools", i);
		if (bpool_node == NULL) {
			dev_err(dpa_oh_dev, "Invalid Buffer pool node\n");
			_errno = -EINVAL;
			goto return_kfree;
		}

		_errno = of_property_read_u32(bpool_node, "fsl,bpid", &bpid);
		if (_errno) {
			dev_err(dpa_oh_dev, "Invalid Buffer Pool ID\n");
			_errno = -EINVAL;
			goto return_kfree;
		}

		oh_port_tx_params.pool_param[i].id = (uint8_t)bpid;
		dev_dbg(dpa_oh_dev, "OH port bpool id = %u\n", bpid);

		bpool_cfg = of_get_property(bpool_node,
				"fsl,bpool-ethernet-cfg", &lenp);
		if (bpool_cfg == NULL) {
			dev_err(dpa_oh_dev, "Invalid Buffer pool config params\n");
			_errno = -EINVAL;
			goto return_kfree;
		}

		ext_pool_size = of_read_number(bpool_cfg + n_size, n_size);
		oh_port_tx_params.pool_param[i].size = (uint16_t)ext_pool_size;
		dev_dbg(dpa_oh_dev, "OH port bpool size = %u\n",
			ext_pool_size);
		of_node_put(bpool_node);

	}

	if (buf_layout.data_align != FRAG_DATA_ALIGN ||
	    buf_layout.manip_extra_space != FRAG_MANIP_SPACE)
		goto init_port;

	frag_enabled = true;
	dev_info(dpa_oh_dev, "IP Fragmentation enabled for OH port %d",
			port_id);

init_port:
	of_node_put(oh_node);
	/* Set Tx params */
	dpaa_eth_init_port(tx, oh_config->oh_port, oh_port_tx_params,
		oh_config->error_fqid, oh_config->default_fqid, (&buf_layout),
		frag_enabled);
	/* Set PCD params */
	oh_port_pcd_params.cba = oh_alloc_pcd_fqids;
	oh_port_pcd_params.cbf = oh_free_pcd_fqids;
	oh_port_pcd_params.dev = dpa_oh_dev;
	fm_port_pcd_bind(oh_config->oh_port, &oh_port_pcd_params);

	dev_set_drvdata(dpa_oh_dev, oh_config);

	/* Enable the OH port */
	_errno = fm_port_enable(oh_config->oh_port);
	if (_errno)
		goto return_kfree;

	dev_info(dpa_oh_dev, "OH port %s enabled.\n", oh_node->full_name);

	/* print of all referenced & created queues */
	dump_oh_config(dpa_oh_dev, oh_config);

	return 0;

return_kfree:
	if (bpool_node)
		of_node_put(bpool_node);
	if (oh_node)
		of_node_put(oh_node);
	if (oh_config && oh_config->egress_fqs)
		devm_kfree(dpa_oh_dev, oh_config->egress_fqs);

	list_for_each_safe(fq_list, fq_list_tmp, &oh_config->fqs_ingress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);
		list_del(fq_list);
		devm_kfree(dpa_oh_dev, fqd->fqs);
		devm_kfree(dpa_oh_dev, fqd);
	}

	list_for_each_safe(fq_list, fq_list_tmp, &oh_config->fqs_egress_list) {
		fqd = list_entry(fq_list, struct fq_duple, fq_list);
		list_del(fq_list);
		devm_kfree(dpa_oh_dev, fqd->fqs);
		devm_kfree(dpa_oh_dev, fqd);
	}

	devm_kfree(dpa_oh_dev, oh_config);
	return _errno;
}

static int __cold oh_port_remove(struct platform_device *_of_dev)
{
	int _errno = 0, i;
	struct dpa_oh_config_s *oh_config;

	pr_info("Removing OH port...\n");

	oh_config = dev_get_drvdata(&_of_dev->dev);
	if (oh_config == NULL) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): No OH config in device private data!\n",
			KBUILD_BASENAME".c", __LINE__, __func__);
		_errno = -ENODEV;
		goto return_error;
	}

	if (oh_config->egress_fqs)
		for (i = 0; i < oh_config->egress_cnt; i++)
			oh_fq_destroy(oh_config->egress_fqs + i);

	if (oh_config->oh_port == NULL) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): No fm port in device private data!\n",
			KBUILD_BASENAME".c", __LINE__, __func__);
		_errno = -EINVAL;
		goto free_egress_fqs;
	}

	_errno = fm_port_disable(oh_config->oh_port);

free_egress_fqs:
	if (oh_config->egress_fqs)
		devm_kfree(&_of_dev->dev, oh_config->egress_fqs);
	devm_kfree(&_of_dev->dev, oh_config);
	dev_set_drvdata(&_of_dev->dev, NULL);

return_error:
	return _errno;
}

static struct platform_driver oh_port_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= oh_port_match_table,
		.owner		= THIS_MODULE,
		.pm		= OH_PM_OPS,
	},
	.probe		= oh_port_probe,
	.remove		= oh_port_remove
};

static int __init __cold oh_port_load(void)
{
	int _errno;

	pr_info(OH_MOD_DESCRIPTION "\n");

	_errno = platform_driver_register(&oh_port_driver);
	if (_errno < 0) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
	return _errno;
}
module_init(oh_port_load);

static void __exit __cold oh_port_unload(void)
{
	pr_debug(KBUILD_MODNAME ": -> %s:%s()\n",
		KBUILD_BASENAME".c", __func__);

	platform_driver_unregister(&oh_port_driver);

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
}
module_exit(oh_port_unload);
