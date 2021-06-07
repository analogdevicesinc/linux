/* Copyright 2008-2012 Freescale Semiconductor, Inc.
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

#include "qman_private.h"

#include <asm/smp.h>	/* hard_smp_processor_id() if !CONFIG_SMP */
#ifdef CONFIG_HOTPLUG_CPU
#include <linux/cpu.h>
#endif

/* Global variable containing revision id (even on non-control plane systems
 * where CCSR isn't available) */
u16 qman_ip_rev;
EXPORT_SYMBOL(qman_ip_rev);
u8 qman_ip_cfg;
EXPORT_SYMBOL(qman_ip_cfg);
u16 qm_channel_pool1 = QMAN_CHANNEL_POOL1;
EXPORT_SYMBOL(qm_channel_pool1);
u16 qm_channel_caam = QMAN_CHANNEL_CAAM;
EXPORT_SYMBOL(qm_channel_caam);
u16 qm_channel_pme = QMAN_CHANNEL_PME;
EXPORT_SYMBOL(qm_channel_pme);
u16 qm_channel_dce = QMAN_CHANNEL_DCE;
EXPORT_SYMBOL(qm_channel_dce);
u16 qman_portal_max;
EXPORT_SYMBOL(qman_portal_max);

u32 qman_clk;
struct qm_ceetm qman_ceetms[QMAN_CEETM_MAX];
/* the qman ceetm instances on the given SoC */
u8 num_ceetms;

/* For these variables, and the portal-initialisation logic, the
 * comments in bman_driver.c apply here so won't be repeated. */
static struct qman_portal *shared_portals[NR_CPUS];
static int num_shared_portals;
static int shared_portals_idx;
static LIST_HEAD(unused_pcfgs);
static DEFINE_SPINLOCK(unused_pcfgs_lock);

/* A SDQCR mask comprising all the available/visible pool channels */
static u32 pools_sdqcr;

#define STR_ERR_NOPROP	    "No '%s' property in node %s\n"
#define STR_ERR_CELL	    "'%s' is not a %d-cell range in node %s\n"
#define STR_FQID_RANGE	    "fsl,fqid-range"
#define STR_POOL_CHAN_RANGE "fsl,pool-channel-range"
#define STR_CGRID_RANGE	     "fsl,cgrid-range"

/* A "fsl,fqid-range" node; release the given range to the allocator */
static __init int fsl_fqid_range_init(struct device_node *node)
{
	int ret;
	const u32 *range = of_get_property(node, STR_FQID_RANGE, &ret);
	if (!range) {
		pr_err(STR_ERR_NOPROP, STR_FQID_RANGE, node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err(STR_ERR_CELL, STR_FQID_RANGE, 2, node->full_name);
		return -EINVAL;
	}
	qman_seed_fqid_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	pr_info("Qman: FQID allocator includes range %d:%d\n",
		be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	return 0;
}

/* A "fsl,pool-channel-range" node; add to the SDQCR mask only */
static __init int fsl_pool_channel_range_sdqcr(struct device_node *node)
{
	int ret;
	const u32 *chanid = of_get_property(node, STR_POOL_CHAN_RANGE, &ret);
	if (!chanid) {
		pr_err(STR_ERR_NOPROP, STR_POOL_CHAN_RANGE, node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err(STR_ERR_CELL, STR_POOL_CHAN_RANGE, 1, node->full_name);
		return -EINVAL;
	}
	for (ret = 0; ret < be32_to_cpu(chanid[1]); ret++)
		pools_sdqcr |= QM_SDQCR_CHANNELS_POOL_CONV(be32_to_cpu(chanid[0]) + ret);
	return 0;
}

/* A "fsl,pool-channel-range" node; release the given range to the allocator */
static __init int fsl_pool_channel_range_init(struct device_node *node)
{
	int ret;
	const u32 *chanid = of_get_property(node, STR_POOL_CHAN_RANGE, &ret);
	if (!chanid) {
		pr_err(STR_ERR_NOPROP, STR_POOL_CHAN_RANGE, node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err(STR_ERR_CELL, STR_POOL_CHAN_RANGE, 1, node->full_name);
		return -EINVAL;
	}
	qman_seed_pool_range(be32_to_cpu(chanid[0]), be32_to_cpu(chanid[1]));
	pr_info("Qman: pool channel allocator includes range %d:%d\n",
		be32_to_cpu(chanid[0]), be32_to_cpu(chanid[1]));
	return 0;
}

/* A "fsl,cgrid-range" node; release the given range to the allocator */
static __init int fsl_cgrid_range_init(struct device_node *node)
{
	struct qman_cgr cgr;
	int ret, errors = 0;
	const u32 *range = of_get_property(node, STR_CGRID_RANGE, &ret);
	if (!range) {
		pr_err(STR_ERR_NOPROP, STR_CGRID_RANGE, node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err(STR_ERR_CELL, STR_CGRID_RANGE, 2, node->full_name);
		return -EINVAL;
	}
	qman_seed_cgrid_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	pr_info("Qman: CGRID allocator includes range %d:%d\n",
		be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	for (cgr.cgrid = 0; cgr.cgrid < __CGR_NUM; cgr.cgrid++) {
		ret = qman_modify_cgr(&cgr, QMAN_CGR_FLAG_USE_INIT, NULL);
		if (ret)
			errors++;
	}
	if (errors)
		pr_err("Warning: %d error%s while initialising CGRs %d:%d\n",
			errors, (errors > 1) ? "s" : "", range[0], range[1]);
	return 0;
}

static __init int fsl_ceetm_init(struct device_node *node)
{
	enum qm_dc_portal dcp_portal;
	struct qm_ceetm_sp *sp;
	struct qm_ceetm_lni *lni;
	int ret, i;
	const u32 *range;

	/* Find LFQID range */
	range = of_get_property(node, "fsl,ceetm-lfqid-range", &ret);
	if (!range) {
		pr_err("No fsl,ceetm-lfqid-range in node %s\n",
							node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("fsl,ceetm-lfqid-range is not a 2-cell range in node"
					" %s\n", node->full_name);
		return -EINVAL;
	}

	dcp_portal = (be32_to_cpu(range[0]) & 0x0F0000) >> 16;
	if (dcp_portal > qm_dc_portal_fman1) {
		pr_err("The DCP portal %d doesn't support CEETM\n", dcp_portal);
		return -EINVAL;
	}

	if (dcp_portal == qm_dc_portal_fman0)
		qman_seed_ceetm0_lfqid_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	if (dcp_portal == qm_dc_portal_fman1)
		qman_seed_ceetm1_lfqid_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	pr_debug("Qman: The lfqid allocator of CEETM %d includes range"
		 " 0x%x:0x%x\n", dcp_portal, be32_to_cpu(range[0]), be32_to_cpu(range[1]));

	qman_ceetms[dcp_portal].idx = dcp_portal;
	INIT_LIST_HEAD(&qman_ceetms[dcp_portal].sub_portals);
	INIT_LIST_HEAD(&qman_ceetms[dcp_portal].lnis);

	/* Find Sub-portal range */
	range = of_get_property(node, "fsl,ceetm-sp-range", &ret);
	if (!range) {
		pr_err("No fsl,ceetm-sp-range in node %s\n", node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("fsl,ceetm-sp-range is not a 2-cell range in node %s\n",
							node->full_name);
		return -EINVAL;
	}

	for (i = 0; i < be32_to_cpu(range[1]); i++) {
		sp = kzalloc(sizeof(*sp), GFP_KERNEL);
		if (!sp) {
			pr_err("Can't alloc memory for sub-portal %d\n",
							range[0] + i);
			return -ENOMEM;
		}
		sp->idx = be32_to_cpu(range[0]) + i;
		sp->dcp_idx = dcp_portal;
		sp->is_claimed = 0;
		list_add_tail(&sp->node, &qman_ceetms[dcp_portal].sub_portals);
		sp++;
	}
	pr_debug("Qman: Reserve sub-portal %d:%d for CEETM %d\n",
		 be32_to_cpu(range[0]), be32_to_cpu(range[1]), dcp_portal);
	qman_ceetms[dcp_portal].sp_range[0] = be32_to_cpu(range[0]);
	qman_ceetms[dcp_portal].sp_range[1] = be32_to_cpu(range[1]);

	/* Find LNI range */
	range = of_get_property(node, "fsl,ceetm-lni-range", &ret);
	if (!range) {
		pr_err("No fsl,ceetm-lni-range in node %s\n", node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("fsl,ceetm-lni-range is not a 2-cell range in node %s\n",
							node->full_name);
		return -EINVAL;
	}

	for (i = 0; i < be32_to_cpu(range[1]); i++) {
		lni = kzalloc(sizeof(*lni), GFP_KERNEL);
		if (!lni) {
			pr_err("Can't alloc memory for LNI %d\n",
							range[0] + i);
			return -ENOMEM;
		}
		lni->idx = be32_to_cpu(range[0]) + i;
		lni->dcp_idx = dcp_portal;
		lni->is_claimed = 0;
		INIT_LIST_HEAD(&lni->channels);
		list_add_tail(&lni->node, &qman_ceetms[dcp_portal].lnis);
		lni++;
	}
	pr_debug("Qman: Reserve LNI %d:%d for CEETM %d\n",
		 be32_to_cpu(range[0]), be32_to_cpu(range[1]), dcp_portal);
	qman_ceetms[dcp_portal].lni_range[0] = be32_to_cpu(range[0]);
	qman_ceetms[dcp_portal].lni_range[1] = be32_to_cpu(range[1]);

	/* Find CEETM channel range */
	range = of_get_property(node, "fsl,ceetm-channel-range", &ret);
	if (!range) {
		pr_err("No fsl,ceetm-channel-range in node %s\n",
							node->full_name);
		return -EINVAL;
	}
	if (ret != 8) {
		pr_err("fsl,ceetm-channel-range is not a 2-cell range in node"
						"%s\n", node->full_name);
		return -EINVAL;
	}

	if (dcp_portal == qm_dc_portal_fman0)
		qman_seed_ceetm0_channel_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	if (dcp_portal == qm_dc_portal_fman1)
		qman_seed_ceetm1_channel_range(be32_to_cpu(range[0]), be32_to_cpu(range[1]));
	pr_debug("Qman: The channel allocator of CEETM %d includes"
		 " range %d:%d\n", dcp_portal, be32_to_cpu(range[0]), be32_to_cpu(range[1]));

	/* Set CEETM PRES register */
	ret = qman_ceetm_set_prescaler(dcp_portal);
	if (ret)
		return ret;
	return 0;
}

static void qman_get_ip_revision(struct device_node *dn)
{
	u16 ip_rev = 0;
	u8 ip_cfg = QMAN_REV_CFG_0;
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		if (!of_device_is_available(dn))
			continue;
		if (of_device_is_compatible(dn, "fsl,qman-portal-1.0") ||
			of_device_is_compatible(dn, "fsl,qman-portal-1.0.0")) {
			pr_err("QMAN rev1.0 on P4080 rev1 is not supported!\n");
			BUG_ON(1);
		} else if (of_device_is_compatible(dn, "fsl,qman-portal-1.1") ||
			of_device_is_compatible(dn, "fsl,qman-portal-1.1.0")) {
			ip_rev = QMAN_REV11;
			qman_portal_max = 10;
		} else if (of_device_is_compatible(dn, "fsl,qman-portal-1.2") ||
			of_device_is_compatible(dn, "fsl,qman-portal-1.2.0")) {
			ip_rev = QMAN_REV12;
			qman_portal_max = 10;
		} else if (of_device_is_compatible(dn, "fsl,qman-portal-2.0") ||
			of_device_is_compatible(dn, "fsl,qman-portal-2.0.0")) {
			ip_rev = QMAN_REV20;
			qman_portal_max = 3;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.0.0")) {
			ip_rev = QMAN_REV30;
			qman_portal_max = 50;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.0.1")) {
			ip_rev = QMAN_REV30;
			qman_portal_max = 25;
			ip_cfg = QMAN_REV_CFG_1;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.1.0")) {
			ip_rev = QMAN_REV31;
			qman_portal_max = 50;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.1.1")) {
			ip_rev = QMAN_REV31;
			qman_portal_max = 25;
			ip_cfg = QMAN_REV_CFG_1;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.1.2")) {
			ip_rev = QMAN_REV31;
			qman_portal_max = 18;
			ip_cfg = QMAN_REV_CFG_2;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.1.3")) {
			ip_rev = QMAN_REV31;
			qman_portal_max = 10;
			ip_cfg = QMAN_REV_CFG_3;
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.2.0")) {
			ip_rev = QMAN_REV32;
			qman_portal_max = 10;
			ip_cfg = QMAN_REV_CFG_3; // TODO: Verify for ls1043
		} else if (of_device_is_compatible(dn,
						"fsl,qman-portal-3.2.1")) {
			ip_rev = QMAN_REV32;
			qman_portal_max = 10;
			ip_cfg = QMAN_REV_CFG_3;
		} else {
			pr_warn("unknown QMan version in portal node,"
				"default to rev1.1\n");
			ip_rev = QMAN_REV11;
			qman_portal_max = 10;
		}

		if (!qman_ip_rev) {
			if (ip_rev) {
				qman_ip_rev = ip_rev;
				qman_ip_cfg = ip_cfg;
			} else {
				pr_warn("unknown Qman version,"
					" default to rev1.1\n");
				qman_ip_rev = QMAN_REV11;
				qman_ip_cfg = QMAN_REV_CFG_0;
			}
		} else if (ip_rev && (qman_ip_rev != ip_rev))
			pr_warn("Revision=0x%04x, but portal '%s' has"
							" 0x%04x\n",
			qman_ip_rev, dn->full_name, ip_rev);
		if (qman_ip_rev == ip_rev)
			break;
	}
}

/* Parse a portal node, perform generic mapping duties and return the config. It
 * is not known at this stage for what purpose (or even if) the portal will be
 * used. */
static struct qm_portal_config * __init parse_pcfg(struct device_node *node)
{
	struct qm_portal_config *pcfg;
	const u32 *index_p;
	u32 index, channel;
	int irq, ret;
	resource_size_t len;

	pcfg = kmalloc(sizeof(*pcfg), GFP_KERNEL);
	if (!pcfg) {
		pr_err("can't allocate portal config");
		return NULL;
	}

	/*
	 * This is a *horrible hack*, but the IOMMU/PAMU driver needs a
	 * 'struct device' in order to get the PAMU stashing setup and the QMan
	 * portal [driver] won't function at all without ring stashing
	 *
	 * Making the QMan portal driver nice and proper is part of the
	 * upstreaming effort
	 */
	pcfg->dev.bus = &platform_bus_type;
	pcfg->dev.of_node = node;

	ret = of_address_to_resource(node, DPA_PORTAL_CE,
				&pcfg->addr_phys[DPA_PORTAL_CE]);
	if (ret) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"reg::CE");
		goto err;
	}
	ret = of_address_to_resource(node, DPA_PORTAL_CI,
				&pcfg->addr_phys[DPA_PORTAL_CI]);
	if (ret) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"reg::CI");
		goto err;
	}
	index_p = of_get_property(node, "cell-index", &ret);
	if (!index_p || (ret != 4)) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"cell-index");
		goto err;
	}
	index = be32_to_cpu(*index_p);
	if (index >= qman_portal_max) {
		pr_err("QMan portal index %d is beyond max (%d)\n",
		       index, qman_portal_max);
		goto err;
	}

	channel = index + QM_CHANNEL_SWPORTAL0;
	pcfg->public_cfg.channel = channel;
	pcfg->public_cfg.cpu = -1;
	irq = irq_of_parse_and_map(node, 0);
	if (irq == 0) {
		pr_err("Can't get %s property '%s'\n", node->full_name,
			"interrupts");
		goto err;
	}
	pcfg->public_cfg.irq = irq;
	pcfg->public_cfg.index = index;
#ifdef CONFIG_FSL_QMAN_CONFIG
	/* We need the same LIODN offset for all portals */
	qman_liodn_fixup(pcfg->public_cfg.channel);
#endif

	len = resource_size(&pcfg->addr_phys[DPA_PORTAL_CE]);
	if (len != (unsigned long)len)
		goto err;

#if defined(CONFIG_ARM) || defined(CONFIG_ARM64)
	pcfg->addr_virt[DPA_PORTAL_CE] = ioremap_cache_ns(
                                pcfg->addr_phys[DPA_PORTAL_CE].start,
                                resource_size(&pcfg->addr_phys[DPA_PORTAL_CE]));

        pcfg->addr_virt[DPA_PORTAL_CI] = ioremap(
                                pcfg->addr_phys[DPA_PORTAL_CI].start,
                                resource_size(&pcfg->addr_phys[DPA_PORTAL_CI]));
#else

	pcfg->addr_virt[DPA_PORTAL_CE] =
		memremap(pcfg->addr_phys[DPA_PORTAL_CE].start,
			 (unsigned long)len, MEMREMAP_WB);

	pcfg->addr_virt[DPA_PORTAL_CI] =
		ioremap(pcfg->addr_phys[DPA_PORTAL_CI].start,
			resource_size(&pcfg->addr_phys[DPA_PORTAL_CI]));

#endif
	return pcfg;
err:
	kfree(pcfg);
	return NULL;
}

static struct qm_portal_config *get_pcfg(struct list_head *list)
{
	struct qm_portal_config *pcfg;
	if (list_empty(list))
		return NULL;
	pcfg = list_entry(list->prev, struct qm_portal_config, list);
	list_del(&pcfg->list);
	return pcfg;
}

static struct qm_portal_config *get_pcfg_idx(struct list_head *list, u32 idx)
{
	struct qm_portal_config *pcfg;
	if (list_empty(list))
		return NULL;
	list_for_each_entry(pcfg, list, list) {
		if (pcfg->public_cfg.index == idx) {
			list_del(&pcfg->list);
			return pcfg;
		}
	}
	return NULL;
}

static void portal_set_cpu(struct qm_portal_config *pcfg, int cpu)
{
#ifdef CONFIG_FSL_PAMU
	int ret;
	int window_count = 1;
	struct iommu_domain_geometry geom_attr;
	struct pamu_stash_attribute stash_attr;

	pcfg->iommu_domain = iommu_domain_alloc(&platform_bus_type);
	if (!pcfg->iommu_domain) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_alloc() failed",
			   __func__);
		goto _no_iommu;
	}
	geom_attr.aperture_start = 0;
	geom_attr.aperture_end =
		((dma_addr_t)1 << min(8 * sizeof(dma_addr_t), (size_t)36)) - 1;
	geom_attr.force_aperture = true;
	ret = iommu_domain_set_attr(pcfg->iommu_domain, DOMAIN_ATTR_GEOMETRY,
				    &geom_attr);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_set_attr() = %d",
			   __func__, ret);
		goto _iommu_domain_free;
	}
	ret = iommu_domain_set_attr(pcfg->iommu_domain, DOMAIN_ATTR_WINDOWS,
				    &window_count);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_set_attr() = %d",
			   __func__, ret);
		goto _iommu_domain_free;
	}
	stash_attr.cpu = cpu;
	stash_attr.cache = PAMU_ATTR_CACHE_L1;
	/* set stash information for the window */
	ret = iommu_domain_set_attr(pcfg->iommu_domain,
				    DOMAIN_ATTR_FSL_PAMU_STASH,
				    &stash_attr);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_set_attr() = %d",
			   __func__, ret);
		goto _iommu_domain_free;
	}
	ret = iommu_domain_window_enable(pcfg->iommu_domain, 0, 0, 1ULL << 36,
					 IOMMU_READ | IOMMU_WRITE);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_window_enable() = %d",
			   __func__, ret);
		goto _iommu_domain_free;
	}
	ret = iommu_attach_device(pcfg->iommu_domain, &pcfg->dev);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_device_attach() = %d",
			   __func__, ret);
		goto _iommu_domain_free;
	}
	ret = iommu_domain_set_attr(pcfg->iommu_domain,
				    DOMAIN_ATTR_FSL_PAMU_ENABLE,
				    &window_count);
	if (ret < 0) {
		pr_err(KBUILD_MODNAME ":%s(): iommu_domain_set_attr() = %d",
			   __func__, ret);
		goto _iommu_detach_device;
	}

_no_iommu:
#endif
#ifdef CONFIG_FSL_QMAN_CONFIG
	if (qman_set_sdest(pcfg->public_cfg.channel, cpu))
#endif
		pr_warn("Failed to set QMan portal's stash request queue\n");

	return;

#ifdef CONFIG_FSL_PAMU
_iommu_detach_device:
	iommu_detach_device(pcfg->iommu_domain, NULL);
_iommu_domain_free:
	iommu_domain_free(pcfg->iommu_domain);
#endif
}

struct qm_portal_config *qm_get_unused_portal_idx(u32 idx)
{
	struct qm_portal_config *ret;
	spin_lock(&unused_pcfgs_lock);
	if (idx == QBMAN_ANY_PORTAL_IDX)
		ret = get_pcfg(&unused_pcfgs);
	else
		ret = get_pcfg_idx(&unused_pcfgs, idx);
	spin_unlock(&unused_pcfgs_lock);
	/* Bind stashing LIODNs to the CPU we are currently executing on, and
	 * set the portal to use the stashing request queue corresonding to the
	 * cpu as well. The user-space driver assumption is that the pthread has
	 * to already be affine to one cpu only before opening a portal. If that
	 * check is circumvented, the only risk is a performance degradation -
	 * stashing will go to whatever cpu they happened to be running on when
	 * opening the device file, and if that isn't the cpu they subsequently
	 * bind to and do their polling on, tough. */
	if (ret)
		portal_set_cpu(ret, hard_smp_processor_id());
	return ret;
}

struct qm_portal_config *qm_get_unused_portal(void)
{
	return qm_get_unused_portal_idx(QBMAN_ANY_PORTAL_IDX);
}

void qm_put_unused_portal(struct qm_portal_config *pcfg)
{
	spin_lock(&unused_pcfgs_lock);
	list_add(&pcfg->list, &unused_pcfgs);
	spin_unlock(&unused_pcfgs_lock);
}

static struct qman_portal *init_pcfg(struct qm_portal_config *pcfg)
{
	struct qman_portal *p;

	pcfg->iommu_domain = NULL;
	portal_set_cpu(pcfg, pcfg->public_cfg.cpu);
	p = qman_create_affine_portal(pcfg, NULL);
	if (p) {
		u32 irq_sources = 0;
		/* Determine what should be interrupt-vs-poll driven */
#ifdef CONFIG_FSL_DPA_PIRQ_SLOW
		irq_sources |= QM_PIRQ_EQCI | QM_PIRQ_EQRI | QM_PIRQ_MRI |
			       QM_PIRQ_CSCI | QM_PIRQ_CCSCI;
#endif
#ifdef CONFIG_FSL_DPA_PIRQ_FAST
		irq_sources |= QM_PIRQ_DQRI;
#endif
		qman_p_irqsource_add(p, irq_sources);
		pr_info("Qman portal %sinitialised, cpu %d\n",
			pcfg->public_cfg.is_shared ? "(shared) " : "",
			pcfg->public_cfg.cpu);
	} else
		pr_crit("Qman portal failure on cpu %d\n",
			pcfg->public_cfg.cpu);
	return p;
}

static void init_slave(int cpu)
{
	struct qman_portal *p;
	struct cpumask oldmask = current->cpus_mask;
	set_cpus_allowed_ptr(current, get_cpu_mask(cpu));
	p = qman_create_affine_slave(shared_portals[shared_portals_idx++], cpu);
	if (!p)
		pr_err("Qman slave portal failure on cpu %d\n", cpu);
	else
		pr_info("Qman portal %sinitialised, cpu %d\n", "(slave) ", cpu);
	set_cpus_allowed_ptr(current, &oldmask);
	if (shared_portals_idx >= num_shared_portals)
		shared_portals_idx = 0;
}

static struct cpumask want_unshared __initdata;
static struct cpumask want_shared __initdata;

static int __init parse_qportals(char *str)
{
	return parse_portals_bootarg(str, &want_shared, &want_unshared,
				     "qportals");
}
__setup("qportals=", parse_qportals);

static void qman_portal_update_sdest(const struct qm_portal_config *pcfg,
							unsigned int cpu)
{
#ifdef CONFIG_FSL_PAMU
	struct pamu_stash_attribute stash_attr;
	int ret;

	if (pcfg->iommu_domain) {
		stash_attr.cpu = cpu;
		stash_attr.cache = PAMU_ATTR_CACHE_L1;
		/* set stash information for the window */
		ret = iommu_domain_set_attr(pcfg->iommu_domain,
				DOMAIN_ATTR_FSL_PAMU_STASH, &stash_attr);
		if (ret < 0) {
			pr_err("Failed to update pamu stash setting\n");
			return;
		}
	}
#endif
#ifdef CONFIG_FSL_QMAN_CONFIG
	if (qman_set_sdest(pcfg->public_cfg.channel, cpu))
		pr_warn("Failed to update portal's stash request queue\n");
#endif
}

static int qman_offline_cpu(unsigned int cpu)
{
	struct qman_portal *p;
	const struct qm_portal_config *pcfg;
	p = (struct qman_portal *)affine_portals[cpu];
	if (p) {
		pcfg = qman_get_qm_portal_config(p);
		if (pcfg) {
			irq_set_affinity(pcfg->public_cfg.irq, cpumask_of(0));
			qman_portal_update_sdest(pcfg, 0);
		}
	}
	return 0;
}

#ifdef CONFIG_HOTPLUG_CPU
static int qman_online_cpu(unsigned int cpu)
{
	struct qman_portal *p;
	const struct qm_portal_config *pcfg;
	p = (struct qman_portal *)affine_portals[cpu];
	if (p) {
		pcfg = qman_get_qm_portal_config(p);
		if (pcfg) {
			irq_set_affinity(pcfg->public_cfg.irq, cpumask_of(cpu));
			qman_portal_update_sdest(pcfg, cpu);
		}
	}
	return 0;
}

#endif /* CONFIG_HOTPLUG_CPU */

__init int qman_init(void)
{
	struct cpumask slave_cpus;
	struct cpumask unshared_cpus = *cpu_none_mask;
	struct cpumask shared_cpus = *cpu_none_mask;
	LIST_HEAD(unshared_pcfgs);
	LIST_HEAD(shared_pcfgs);
	struct device_node *dn;
	struct qm_portal_config *pcfg;
	struct qman_portal *p;
	int cpu, ret;
	const u32 *clk;
	struct cpumask offline_cpus;

	/* Initialise the Qman (CCSR) device */
	for_each_compatible_node(dn, NULL, "fsl,qman") {
		if (!qman_init_ccsr(dn))
			pr_info("Qman err interrupt handler present\n");
		else
			pr_err("Qman CCSR setup failed\n");

		clk = of_get_property(dn, "clock-frequency", NULL);
		if (!clk)
			pr_warn("Can't find Qman clock frequency\n");
		else
			qman_clk = be32_to_cpu(*clk);
	}
#ifdef CONFIG_FSL_QMAN_FQ_LOOKUP
	/* Setup lookup table for FQ demux */
	ret = qman_setup_fq_lookup_table(get_qman_fqd_size()/64);
	if (ret)
		return ret;
#endif

	/* Get qman ip revision */
	qman_get_ip_revision(dn);
	if ((qman_ip_rev & 0xff00) >= QMAN_REV30) {
		qm_channel_pool1 = QMAN_CHANNEL_POOL1_REV3;
		qm_channel_caam = QMAN_CHANNEL_CAAM_REV3;
		qm_channel_pme = QMAN_CHANNEL_PME_REV3;
	}

	if ((qman_ip_rev == QMAN_REV31) && (qman_ip_cfg == QMAN_REV_CFG_2))
		qm_channel_dce = QMAN_CHANNEL_DCE_QMANREV312;

	/*
	 * Parse the ceetm node to get how many ceetm instances are supported
	 * on the current silicon. num_ceetms must be confirmed before portals
	 * are intiailized.
	 */
	num_ceetms = 0;
	for_each_compatible_node(dn, NULL, "fsl,qman-ceetm")
		num_ceetms++;

	/* Parse pool channels into the SDQCR mask. (Must happen before portals
	 * are initialised.) */
	for_each_compatible_node(dn, NULL, "fsl,pool-channel-range") {
		ret = fsl_pool_channel_range_sdqcr(dn);
		if (ret)
			return ret;
	}

	memset(affine_portals, 0, sizeof(void *) * num_possible_cpus());
	/* Initialise portals. See bman_driver.c for comments */
	for_each_compatible_node(dn, NULL, "fsl,qman-portal") {
		if (!of_device_is_available(dn))
			continue;
		pcfg = parse_pcfg(dn);
		if (pcfg) {
			pcfg->public_cfg.pools = pools_sdqcr;
			list_add_tail(&pcfg->list, &unused_pcfgs);
		}
	}
	for_each_possible_cpu(cpu) {
		if (cpumask_test_cpu(cpu, &want_shared)) {
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &shared_pcfgs);
			cpumask_set_cpu(cpu, &shared_cpus);
		}
		if (cpumask_test_cpu(cpu, &want_unshared)) {
			if (cpumask_test_cpu(cpu, &shared_cpus))
				continue;
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &unshared_pcfgs);
			cpumask_set_cpu(cpu, &unshared_cpus);
		}
	}
	if (list_empty(&shared_pcfgs) && list_empty(&unshared_pcfgs)) {
		for_each_online_cpu(cpu) {
			pcfg = get_pcfg(&unused_pcfgs);
			if (!pcfg)
				break;
			pcfg->public_cfg.cpu = cpu;
			list_add_tail(&pcfg->list, &unshared_pcfgs);
			cpumask_set_cpu(cpu, &unshared_cpus);
		}
	}
	cpumask_andnot(&slave_cpus, cpu_possible_mask, &shared_cpus);
	cpumask_andnot(&slave_cpus, &slave_cpus, &unshared_cpus);
	if (cpumask_empty(&slave_cpus)) {
		if (!list_empty(&shared_pcfgs)) {
			cpumask_or(&unshared_cpus, &unshared_cpus,
				   &shared_cpus);
			cpumask_clear(&shared_cpus);
			list_splice_tail(&shared_pcfgs, &unshared_pcfgs);
			INIT_LIST_HEAD(&shared_pcfgs);
		}
	} else {
		if (list_empty(&shared_pcfgs)) {
			pcfg = get_pcfg(&unshared_pcfgs);
			if (!pcfg) {
				pr_crit("No QMan portals available!\n");
				return 0;
			}
			cpumask_clear_cpu(pcfg->public_cfg.cpu, &unshared_cpus);
			cpumask_set_cpu(pcfg->public_cfg.cpu, &shared_cpus);
			list_add_tail(&pcfg->list, &shared_pcfgs);
		}
	}
	list_for_each_entry(pcfg, &unshared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 0;
		p = init_pcfg(pcfg);
		if (!p) {
			pr_crit("Unable to configure portals\n");
			return 0;
		}
	}
	list_for_each_entry(pcfg, &shared_pcfgs, list) {
		pcfg->public_cfg.is_shared = 1;
		p = init_pcfg(pcfg);
		if (p)
			shared_portals[num_shared_portals++] = p;
	}
	if (!cpumask_empty(&slave_cpus))
		for_each_cpu(cpu, &slave_cpus)
			init_slave(cpu);
	pr_info("Qman portals initialised\n");
	cpumask_andnot(&offline_cpus, cpu_possible_mask, cpu_online_mask);
	for_each_cpu(cpu, &offline_cpus)
		qman_offline_cpu(cpu);
#ifdef CONFIG_HOTPLUG_CPU
	ret = cpuhp_setup_state_nocalls(CPUHP_AP_ONLINE_DYN,
					"soc/qman_portal:online",
					qman_online_cpu, qman_offline_cpu);
	if (ret < 0) {
		pr_err("qman: failed to register hotplug callbacks.\n");
		return ret;
	}
#endif
	return 0;
}

__init int qman_resource_init(void)
{
	struct device_node *dn;
	int ret;

	/* Initialise FQID allocation ranges */
	for_each_compatible_node(dn, NULL, "fsl,fqid-range") {
		ret = fsl_fqid_range_init(dn);
		if (ret)
			return ret;
	}
	/* Initialise CGRID allocation ranges */
	for_each_compatible_node(dn, NULL, "fsl,cgrid-range") {
		ret = fsl_cgrid_range_init(dn);
		if (ret)
			return ret;
	}
	/* Parse pool channels into the allocator. (Must happen after portals
	 * are initialised.) */
	for_each_compatible_node(dn, NULL, "fsl,pool-channel-range") {
		ret = fsl_pool_channel_range_init(dn);
		if (ret)
			return ret;
	}

	/* Parse CEETM */
	for_each_compatible_node(dn, NULL, "fsl,qman-ceetm") {
		ret = fsl_ceetm_init(dn);
		if (ret)
			return ret;
	}
	return 0;
}

#ifdef CONFIG_SUSPEND
void suspend_unused_qportal(void)
{
	struct qm_portal_config *pcfg;

	if (list_empty(&unused_pcfgs))
		return;

	list_for_each_entry(pcfg, &unused_pcfgs, list) {
#ifdef CONFIG_PM_DEBUG
		pr_info("Need to save qportal %d\n", pcfg->public_cfg.index);
#endif
		/* save isdr, disable all via isdr, clear isr */
		pcfg->saved_isdr =
			__raw_readl(pcfg->addr_virt[DPA_PORTAL_CI] + 0xe08);
		__raw_writel(0xffffffff, pcfg->addr_virt[DPA_PORTAL_CI] +
					0xe08);
		__raw_writel(0xffffffff, pcfg->addr_virt[DPA_PORTAL_CI] +
					0xe00);
	}
	return;
}

void resume_unused_qportal(void)
{
	struct qm_portal_config *pcfg;

	if (list_empty(&unused_pcfgs))
		return;

	list_for_each_entry(pcfg, &unused_pcfgs, list) {
#ifdef CONFIG_PM_DEBUG
		pr_info("Need to resume qportal %d\n", pcfg->public_cfg.index);
#endif
		/* restore isdr */
		__raw_writel(pcfg->saved_isdr,
				pcfg->addr_virt[DPA_PORTAL_CI] + 0xe08);
	}
	return;
}
#endif
