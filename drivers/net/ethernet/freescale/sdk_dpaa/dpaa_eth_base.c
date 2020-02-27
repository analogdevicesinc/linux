/* Copyright 2008-2013 Freescale Semiconductor, Inc.
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
#include <linux/io.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/highmem.h>
#include <linux/sort.h>
#include <linux/fsl_qman.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "dpaa_eth_base.h"

#define DPA_DESCRIPTION "FSL DPAA Advanced drivers:"

MODULE_LICENSE("Dual BSD/GPL");

uint8_t advanced_debug = -1;
module_param(advanced_debug, byte, S_IRUGO);
MODULE_PARM_DESC(advanced_debug, "Module/Driver verbosity level");
EXPORT_SYMBOL(advanced_debug);

static int dpa_bp_cmp(const void *dpa_bp0, const void *dpa_bp1)
{
	return ((struct dpa_bp *)dpa_bp0)->size -
			((struct dpa_bp *)dpa_bp1)->size;
}

struct dpa_bp * __cold __must_check /* __attribute__((nonnull)) */
dpa_bp_probe(struct platform_device *_of_dev, size_t *count)
{
	int			 i, lenp, na, ns, err;
	struct device		*dev;
	struct device_node	*dev_node;
	const __be32		*bpool_cfg;
	struct dpa_bp		*dpa_bp;
	u32			bpid;

	dev = &_of_dev->dev;

	*count = of_count_phandle_with_args(dev->of_node,
			"fsl,bman-buffer-pools", NULL);
	if (*count < 1) {
		dev_err(dev, "missing fsl,bman-buffer-pools device tree entry\n");
		return ERR_PTR(-EINVAL);
	}

	dpa_bp = devm_kzalloc(dev, *count * sizeof(*dpa_bp), GFP_KERNEL);
	if (dpa_bp == NULL) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return ERR_PTR(-ENOMEM);
	}

	dev_node = of_find_node_by_path("/");
	if (unlikely(dev_node == NULL)) {
		dev_err(dev, "of_find_node_by_path(/) failed\n");
		return ERR_PTR(-EINVAL);
	}

	na = of_n_addr_cells(dev_node);
	ns = of_n_size_cells(dev_node);

	for (i = 0; i < *count; i++) {
		of_node_put(dev_node);

		dev_node = of_parse_phandle(dev->of_node,
				"fsl,bman-buffer-pools", i);
		if (dev_node == NULL) {
			dev_err(dev, "of_find_node_by_phandle() failed\n");
			return ERR_PTR(-EFAULT);
		}

		if (unlikely(!of_device_is_compatible(dev_node, "fsl,bpool"))) {
			dev_err(dev,
				"!of_device_is_compatible(%s, fsl,bpool)\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}

		err = of_property_read_u32(dev_node, "fsl,bpid", &bpid);
		if (err) {
			dev_err(dev, "Cannot find buffer pool ID in the device tree\n");
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
		dpa_bp[i].bpid = (uint8_t)bpid;

		bpool_cfg = of_get_property(dev_node, "fsl,bpool-ethernet-cfg",
					&lenp);
		if (bpool_cfg && (lenp == (2 * ns + na) * sizeof(*bpool_cfg))) {
			const uint32_t *seed_pool;

			dpa_bp[i].config_count =
				(int)of_read_number(bpool_cfg, ns);
			dpa_bp[i].size	=
				(size_t)of_read_number(bpool_cfg + ns, ns);
			dpa_bp[i].paddr	=
				of_read_number(bpool_cfg + 2 * ns, na);

			seed_pool = of_get_property(dev_node,
					"fsl,bpool-ethernet-seeds", &lenp);
			dpa_bp[i].seed_pool = !!seed_pool;

		} else {
			dev_err(dev,
				"Missing/invalid fsl,bpool-ethernet-cfg device tree entry for node %s\n",
				dev_node->full_name);
			dpa_bp = ERR_PTR(-EINVAL);
			goto _return_of_node_put;
		}
	}

	sort(dpa_bp, *count, sizeof(*dpa_bp), dpa_bp_cmp, NULL);

	return dpa_bp;

_return_of_node_put:
	if (dev_node)
		of_node_put(dev_node);

	return dpa_bp;
}
EXPORT_SYMBOL(dpa_bp_probe);

int dpa_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
		size_t count)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	int i;

	priv->dpa_bp = dpa_bp;
	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;
		err = dpa_bp_alloc(&dpa_bp[i], net_dev->dev.parent);
		if (err < 0) {
			dpa_bp_free(priv);
			priv->dpa_bp = NULL;
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL(dpa_bp_create);

static int __init __cold dpa_advanced_load(void)
{
	pr_info(DPA_DESCRIPTION "\n");

	return 0;
}
module_init(dpa_advanced_load);

static void __exit __cold dpa_advanced_unload(void)
{
	pr_debug(KBUILD_MODNAME ": -> %s:%s()\n",
		 KBUILD_BASENAME".c", __func__);

}
module_exit(dpa_advanced_unload);
