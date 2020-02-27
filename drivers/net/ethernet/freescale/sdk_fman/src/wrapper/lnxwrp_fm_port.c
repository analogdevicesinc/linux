/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

/*
 @File          lnxwrp_fm_port.c

 @Description   FMD wrapper - FMan port functions.

*/

#include <linux/version.h>
#if defined(CONFIG_MODVERSIONS) && !defined(MODVERSIONS)
#define MODVERSIONS
#endif
#ifdef MODVERSIONS
#include <config/modversions.h>
#endif /* MODVERSIONS */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#ifndef CONFIG_FMAN_ARM
#include <linux/fsl/svr.h>
#endif
#include <linux/io.h>

#include "sprint_ext.h"
#include "fm_common.h"
#include "lnxwrp_fsl_fman.h"
#include "fm_port_ext.h"
#if (DPAA_VERSION >= 11)
#include "fm_vsp_ext.h"
#endif /* DPAA_VERSION >= 11 */
#include "fm_ioctls.h"
#include "lnxwrp_resources.h"
#include "lnxwrp_sysfs_fm_port.h"

#define __ERR_MODULE__  MODULE_FM

extern struct device_node *GetFmAdvArgsDevTreeNode (uint8_t fmIndx);

/* TODO: duplicated, see lnxwrp_fm.c */
#define ADD_ADV_CONFIG_NO_RET(_func, _param)\
do {\
	if (i < max) {\
		p_Entry = &p_Entrys[i];\
		p_Entry->p_Function = _func;\
		_param\
		i++;\
	} else {\
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,\
		("Number of advanced-configuration entries exceeded"));\
	} \
} while (0)

#ifndef CONFIG_FMAN_ARM
#define IS_T1023_T1024	(SVR_SOC_VER(mfspr(SPRN_SVR)) == SVR_T1024 || \
			SVR_SOC_VER(mfspr(SPRN_SVR)) == SVR_T1023)
#endif

static volatile int hcFrmRcv/* = 0 */;
static spinlock_t lock;

static enum qman_cb_dqrr_result qm_tx_conf_dqrr_cb(struct qman_portal *portal,
						   struct qman_fq *fq,
						   const struct qm_dqrr_entry
						   *dq)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev = ((t_FmTestFq *) fq)->h_Arg;
	unsigned long flags;

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
{
	/* extract the HC frame address */
	uint32_t *hcf_va = XX_PhysToVirt(qm_fd_addr((struct qm_fd *)&dq->fd));
	int hcf_l = ((struct qm_fd *)&dq->fd)->length20;
	int i;

	/* 32b byteswap of all data in the HC Frame */
	for(i = 0; i < hcf_l / 4; ++i)
		hcf_va[i] =
			___constant_swab32(hcf_va[i]);
}
#endif
	FM_PCD_HcTxConf(p_LnxWrpFmDev->h_PcdDev, (t_DpaaFD *)&dq->fd);
	spin_lock_irqsave(&lock, flags);
	hcFrmRcv--;
	spin_unlock_irqrestore(&lock, flags);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result qm_tx_dqrr_cb(struct qman_portal *portal,
					      struct qman_fq *fq,
					      const struct qm_dqrr_entry *dq)
{
	WARN(1, "FMD: failure at %s:%d/%s()!\n", __FILE__, __LINE__,
	     __func__);
	return qman_cb_dqrr_consume;
}

static void qm_err_cb(struct qman_portal *portal,
		      struct qman_fq *fq, const struct qm_mr_entry *msg)
{
	WARN(1, "FMD: failure at %s:%d/%s()!\n", __FILE__, __LINE__,
	     __func__);
}

static struct qman_fq *FqAlloc(t_LnxWrpFmDev * p_LnxWrpFmDev,
			       uint32_t fqid,
			       uint32_t flags, uint16_t channel, uint8_t wq)
{
	int _errno;
	struct qman_fq *fq = NULL;
	t_FmTestFq *p_FmtFq;
	struct qm_mcc_initfq initfq;

	p_FmtFq = (t_FmTestFq *) XX_Malloc(sizeof(t_FmTestFq));
	if (!p_FmtFq) {
		REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FQ obj!!!"));
		return NULL;
	}

	p_FmtFq->fq_base.cb.dqrr = ((flags & QMAN_FQ_FLAG_NO_ENQUEUE)
			? qm_tx_conf_dqrr_cb
			: qm_tx_dqrr_cb);
	p_FmtFq->fq_base.cb.ern = qm_err_cb;
	/* p_FmtFq->fq_base.cb.fqs = qm_err_cb; */
	/* qm_err_cb wrongly called when the FQ is parked */
	p_FmtFq->fq_base.cb.fqs = NULL;
	p_FmtFq->h_Arg = (t_Handle) p_LnxWrpFmDev;
	if (fqid == 0) {
		flags |= QMAN_FQ_FLAG_DYNAMIC_FQID;
		flags &= ~QMAN_FQ_FLAG_NO_MODIFY;
	} else {
		flags &= ~QMAN_FQ_FLAG_DYNAMIC_FQID;
	}

	if (qman_create_fq(fqid, flags, &p_FmtFq->fq_base)) {
		REPORT_ERROR(MAJOR, E_NO_MEMORY, ("FQ obj - qman_new_fq!!!"));
		XX_Free(p_FmtFq);
		return NULL;
	}
	fq = &p_FmtFq->fq_base;

	if (!(flags & QMAN_FQ_FLAG_NO_MODIFY)) {
		initfq.we_mask = QM_INITFQ_WE_DESTWQ;
		initfq.fqd.dest.channel = channel;
		initfq.fqd.dest.wq = wq;

		_errno = qman_init_fq(fq, QMAN_INITFQ_FLAG_SCHED, &initfq);
		if (unlikely(_errno < 0)) {
			REPORT_ERROR(MAJOR, E_NO_MEMORY,
				     ("FQ obj - qman_init_fq!!!"));
			qman_destroy_fq(fq, 0);
			XX_Free(p_FmtFq);
			return NULL;
		}
	}

	DBG(TRACE,
	    ("fqid %d, flags 0x%08x, channel %d, wq %d", qman_fq_fqid(fq),
	     flags, channel, wq));

	return fq;
}

static void FqFree(struct qman_fq *fq)
{
	int _errno;

	_errno = qman_retire_fq(fq, NULL);
	if (unlikely(_errno < 0))
		printk(KERN_WARNING "qman_retire_fq(%u) = %d\n", qman_fq_fqid(fq), _errno);

	_errno = qman_oos_fq(fq);
	if (unlikely(_errno < 0))
		printk(KERN_WARNING "qman_oos_fq(%u) = %d\n", qman_fq_fqid(fq), _errno);

	qman_destroy_fq(fq, 0);
	XX_Free((t_FmTestFq *) fq);
}

static t_Error QmEnqueueCB(t_Handle h_Arg, void *p_Fd)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev = (t_LnxWrpFmDev *) h_Arg;
	int _errno, timeout = 1000000;
	unsigned long flags;

	ASSERT_COND(p_LnxWrpFmDev);

	spin_lock_irqsave(&lock, flags);
	hcFrmRcv++;
	spin_unlock_irqrestore(&lock, flags);

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
{
	/* extract the HC frame address */
	uint32_t *hcf_va = XX_PhysToVirt(qm_fd_addr((struct qm_fd *) p_Fd));
	int hcf_l = ((struct qm_fd *)p_Fd)->length20;
	int i;

	/* 32b byteswap of all data in the HC Frame */
	for(i = 0; i < hcf_l / 4; ++i)
		hcf_va[i] =
			___constant_swab32(hcf_va[i]);
}
#endif

	_errno = qman_enqueue(p_LnxWrpFmDev->hc_tx_fq, (struct qm_fd *) p_Fd,
			      0);
	if (_errno)
		RETURN_ERROR(MINOR, E_INVALID_STATE,
			     ("qman_enqueue() failed"));

	while (hcFrmRcv && --timeout) {
		udelay(1);
		cpu_relax();
	}
	if (timeout == 0) {
		dump_stack();
		RETURN_ERROR(MINOR, E_WRITE_FAILED,
			     ("timeout waiting for Tx confirmation"));
		return E_WRITE_FAILED;
	}

	return E_OK;
}

static t_LnxWrpFmPortDev *ReadFmPortDevTreeNode(struct platform_device
						*of_dev)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev;
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
	struct device_node *fm_node, *port_node;
	struct resource res;
	const uint32_t *uint32_prop;
	int _errno = 0, lenp;
	uint32_t tmp_prop;

#ifdef CONFIG_FMAN_P1023
	static unsigned char have_oh_port/* = 0 */;
#endif

	port_node = of_node_get(of_dev->dev.of_node);

	/* Get the FM node */
	fm_node = of_get_parent(port_node);
	if (unlikely(fm_node == NULL)) {
		REPORT_ERROR(MAJOR, E_NO_DEVICE,
			     ("of_get_parent() = %d", _errno));
		return NULL;
	}

	p_LnxWrpFmDev =
		dev_get_drvdata(&of_find_device_by_node(fm_node)->dev);
	of_node_put(fm_node);

	/* if fm_probe() failed, no point in going further with port probing */
	if (p_LnxWrpFmDev == NULL)
		return NULL;

	uint32_prop =
		(uint32_t *) of_get_property(port_node, "cell-index", &lenp);
	if (unlikely(uint32_prop == NULL)) {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,
			     ("of_get_property(%s, cell-index) failed",
			      port_node->full_name));
		return NULL;
	}
	tmp_prop = be32_to_cpu(*uint32_prop);
	if (WARN_ON(lenp != sizeof(uint32_t)))
		return NULL;
	if (of_device_is_compatible(port_node, "fsl,fman-port-oh") ||
	    of_device_is_compatible(port_node, "fsl,fman-v2-port-oh") ||
	    of_device_is_compatible(port_node, "fsl,fman-v3-port-oh")) {
#ifndef CONFIG_FMAN_ARM
#ifdef CONFIG_FMAN_P3040_P4080_P5020
		/* On PPC FMan v2, OH ports start from cell-index 0x1 */
		tmp_prop -= 0x1;
#else
		/* On PPC FMan v3 (Low and High), OH ports start from
		 * cell-index 0x2
		 */
		tmp_prop -= 0x2;
#endif // CONFIG_FMAN_P3040_P4080_P5020
#endif // CONFIG_FMAN_ARM

		if (unlikely(tmp_prop >= FM_MAX_NUM_OF_OH_PORTS)) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
				     ("of_get_property(%s, cell-index) failed",
				      port_node->full_name));
			return NULL;
		}

#ifdef CONFIG_FMAN_P1023
		/* Beware, this can be done when there is only
		   one FMan to be initialized */
		if (!have_oh_port) {
			have_oh_port = 1; /* first OP/HC port
					     is used for host command */
#else
		/* Here it is hardcoded the use of the OH port 1
		   (with cell-index 0) */
		if (tmp_prop == 0) {
#endif
			p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
			p_LnxWrpFmPortDev->id = 0;
			/*
			p_LnxWrpFmPortDev->id = *uint32_prop-1;
			p_LnxWrpFmPortDev->id = *uint32_prop;
			*/
			p_LnxWrpFmPortDev->settings.param.portType =
				e_FM_PORT_TYPE_OH_HOST_COMMAND;
		} else {
			p_LnxWrpFmPortDev =
				&p_LnxWrpFmDev->opPorts[tmp_prop - 1];
			p_LnxWrpFmPortDev->id = tmp_prop- 1;
			p_LnxWrpFmPortDev->settings.param.portType =
				e_FM_PORT_TYPE_OH_OFFLINE_PARSING;
		}
		p_LnxWrpFmPortDev->settings.param.portId = tmp_prop;

		uint32_prop =
			(uint32_t *) of_get_property(port_node,
						     "fsl,qman-channel-id",
						     &lenp);
		if (uint32_prop == NULL) {
						/*
 REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("missing fsl,qman-channel-id"));
 */
			XX_Print("FM warning: missing fsl,qman-channel-id"
					" for OH port.\n");
			return NULL;
		}
		tmp_prop = be32_to_cpu(*uint32_prop);
		if (WARN_ON(lenp != sizeof(uint32_t)))
			return NULL;
		p_LnxWrpFmPortDev->txCh = tmp_prop;

		p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.
			qmChannel = p_LnxWrpFmPortDev->txCh;
	} else if (of_device_is_compatible(port_node, "fsl,fman-port-1g-tx")) {
		tmp_prop -= 0x28;
		if (unlikely(tmp_prop >= FM_MAX_NUM_OF_1G_TX_PORTS)) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("of_get_property(%s, cell-index) failed",
					 port_node->full_name));
			return NULL;
		}
		p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[tmp_prop];

		p_LnxWrpFmPortDev->id = tmp_prop;
		p_LnxWrpFmPortDev->settings.param.portId =
			p_LnxWrpFmPortDev->id;
		p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_TX;

		uint32_prop = (uint32_t *) of_get_property(port_node,
				"fsl,qman-channel-id", &lenp);
		if (uint32_prop == NULL) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("missing fsl,qman-channel-id"));
			return NULL;
		}
		tmp_prop = be32_to_cpu(*uint32_prop);
		if (WARN_ON(lenp != sizeof(uint32_t)))
			return NULL;
		p_LnxWrpFmPortDev->txCh = tmp_prop;
		p_LnxWrpFmPortDev->
			settings.param.specificParams.nonRxParams.qmChannel =
			p_LnxWrpFmPortDev->txCh;
	} else if (of_device_is_compatible(port_node, "fsl,fman-port-10g-tx")) {
#ifndef CONFIG_FMAN_ARM
		/* On T102x, the 10G TX port IDs start from 0x28 */
		if (IS_T1023_T1024)
			tmp_prop -= 0x28;
		else
#endif
		tmp_prop -= 0x30;

		if (unlikely(tmp_prop>= FM_MAX_NUM_OF_10G_TX_PORTS)) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("of_get_property(%s, cell-index) failed",
					 port_node->full_name));
			return NULL;
		}
		p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[tmp_prop +
			FM_MAX_NUM_OF_1G_TX_PORTS];
#ifndef CONFIG_FMAN_ARM
		if (IS_T1023_T1024)
			p_LnxWrpFmPortDev = &p_LnxWrpFmDev->txPorts[tmp_prop];
#endif

		p_LnxWrpFmPortDev->id = tmp_prop;
		p_LnxWrpFmPortDev->settings.param.portId =
			p_LnxWrpFmPortDev->id;
		p_LnxWrpFmPortDev->settings.param.portType =
			e_FM_PORT_TYPE_TX_10G;
		uint32_prop = (uint32_t *) of_get_property(port_node,
				"fsl,qman-channel-id", &lenp);
		if (uint32_prop == NULL) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("missing fsl,qman-channel-id"));
			return NULL;
		}
		tmp_prop = be32_to_cpu(*uint32_prop);
		if (WARN_ON(lenp != sizeof(uint32_t)))
			return NULL;
		p_LnxWrpFmPortDev->txCh = tmp_prop;
		p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.
			qmChannel = p_LnxWrpFmPortDev->txCh;
	} else if (of_device_is_compatible(port_node, "fsl,fman-port-1g-rx")) {
		tmp_prop -= 0x08;
		if (unlikely(tmp_prop >= FM_MAX_NUM_OF_1G_RX_PORTS)) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("of_get_property(%s, cell-index) failed",
					 port_node->full_name));
			return NULL;
		}
		p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[tmp_prop];

		p_LnxWrpFmPortDev->id = tmp_prop;
		p_LnxWrpFmPortDev->settings.param.portId =
			p_LnxWrpFmPortDev->id;
		p_LnxWrpFmPortDev->settings.param.portType = e_FM_PORT_TYPE_RX;
		if (p_LnxWrpFmDev->pcdActive)
			p_LnxWrpFmPortDev->defPcd = p_LnxWrpFmDev->defPcd;
	} else if (of_device_is_compatible(port_node, "fsl,fman-port-10g-rx")) {
#ifndef CONFIG_FMAN_ARM
		/* On T102x, the 10G RX port IDs start from 0x08 */
		if (IS_T1023_T1024)
			tmp_prop -= 0x8;
		else
#endif
		tmp_prop -= 0x10;

		if (unlikely(tmp_prop >= FM_MAX_NUM_OF_10G_RX_PORTS)) {
			REPORT_ERROR(MAJOR, E_INVALID_VALUE,
					("of_get_property(%s, cell-index) failed",
					 port_node->full_name));
			return NULL;
		}
		p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[tmp_prop +
			FM_MAX_NUM_OF_1G_RX_PORTS];

#ifndef CONFIG_FMAN_ARM
		if (IS_T1023_T1024)
			p_LnxWrpFmPortDev = &p_LnxWrpFmDev->rxPorts[tmp_prop];
#endif

		p_LnxWrpFmPortDev->id = tmp_prop;
		p_LnxWrpFmPortDev->settings.param.portId =
			p_LnxWrpFmPortDev->id;
		p_LnxWrpFmPortDev->settings.param.portType =
			e_FM_PORT_TYPE_RX_10G;
		if (p_LnxWrpFmDev->pcdActive)
			p_LnxWrpFmPortDev->defPcd = p_LnxWrpFmDev->defPcd;
	} else {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE, ("Illegal port type"));
		return NULL;
	}

	_errno = of_address_to_resource(port_node, 0, &res);
	if (unlikely(_errno < 0)) {
		REPORT_ERROR(MAJOR, E_INVALID_VALUE,
			     ("of_address_to_resource() = %d", _errno));
		return NULL;
	}

	p_LnxWrpFmPortDev->dev = &of_dev->dev;
	p_LnxWrpFmPortDev->baseAddr = 0;
	p_LnxWrpFmPortDev->phys_baseAddr = res.start;
	p_LnxWrpFmPortDev->memSize = res.end + 1 - res.start;
	p_LnxWrpFmPortDev->settings.param.h_Fm = p_LnxWrpFmDev->h_Dev;
	p_LnxWrpFmPortDev->h_LnxWrpFmDev = (t_Handle) p_LnxWrpFmDev;

	of_node_put(port_node);

	p_LnxWrpFmPortDev->active = TRUE;

#if defined(CONFIG_FMAN_DISABLE_OH_TO_REUSE_RESOURCES)
	/* for performance mode no OH port available. */
	if (p_LnxWrpFmPortDev->settings.param.portType ==
	    e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
		p_LnxWrpFmPortDev->active = FALSE;
#endif

	return p_LnxWrpFmPortDev;
}

struct device_node * GetFmPortAdvArgsDevTreeNode (struct device_node *fm_node,
                                                         e_FmPortType       portType,
                                                         uint8_t            portId)
{
    struct device_node  *port_node;
    const uint32_t      *uint32_prop;
    int                 lenp;
    char                *portTypeString;
    uint32_t            tmp_prop;

    switch(portType) {
        case e_FM_PORT_TYPE_OH_OFFLINE_PARSING:
            portTypeString = "fsl,fman-port-op-extended-args";
            break;
        case e_FM_PORT_TYPE_TX:
            portTypeString = "fsl,fman-port-1g-tx-extended-args";
            break;
        case e_FM_PORT_TYPE_TX_10G:
            portTypeString = "fsl,fman-port-10g-tx-extended-args";
            break;
        case e_FM_PORT_TYPE_RX:
            portTypeString = "fsl,fman-port-1g-rx-extended-args";
            break;
        case e_FM_PORT_TYPE_RX_10G:
            portTypeString = "fsl,fman-port-10g-rx-extended-args";
            break;
        default:
            return NULL;
    }

    for_each_child_of_node(fm_node, port_node) {
        uint32_prop = (uint32_t *)of_get_property(port_node, "cell-index", &lenp);
        if (unlikely(uint32_prop == NULL)) {
            REPORT_ERROR(MAJOR, E_INVALID_VALUE,
                         ("of_get_property(%s, cell-index) failed",
                          port_node->full_name));
            return NULL;
        }
        tmp_prop = be32_to_cpu(*uint32_prop);
        if (WARN_ON(lenp != sizeof(uint32_t)))
            return NULL;
    	if ((portId == tmp_prop) &&
    	    (of_device_is_compatible(port_node, portTypeString))) {
            return port_node;
    	}
    }

    return NULL;
}

static t_Error CheckNConfigFmPortAdvArgs (t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
    struct device_node      *fm_node, *port_node;
    t_Error                 err;
    t_FmPortRsrc            portRsrc;
    const uint32_t          *uint32_prop;
    /*const char              *str_prop;*/
    int                     lenp;
#ifdef CONFIG_FMAN_PFC
    uint8_t i, id, num_pools;
    t_FmBufPoolDepletion poolDepletion;

    if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX ||
            p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G) {
        memset(&poolDepletion, 0, sizeof(t_FmBufPoolDepletion));
        poolDepletion.singlePoolModeEnable = true;
        num_pools = p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.
                extBufPools.numOfPoolsUsed;
        for (i = 0; i < num_pools; i++) {
            id = p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.
                    extBufPools.extBufPool[i].id;
            poolDepletion.poolsToConsiderForSingleMode[id] = true;
        }

        for (i = 0; i < CONFIG_FMAN_PFC_COS_COUNT; i++)
            poolDepletion.pfcPrioritiesEn[i] = true;

        err = FM_PORT_ConfigPoolDepletion(p_LnxWrpFmPortDev->h_Dev,
                &poolDepletion);
        if (err != E_OK)
            RETURN_ERROR(MAJOR, err, ("FM_PORT_ConfigPoolDepletion() failed"));
    }
#endif

    fm_node = GetFmAdvArgsDevTreeNode(((t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev)->id);
    if (!fm_node) /* no advance parameters for FMan */
        return E_OK;

    port_node = GetFmPortAdvArgsDevTreeNode(fm_node,
                                            p_LnxWrpFmPortDev->settings.param.portType,
                                            p_LnxWrpFmPortDev->settings.param.portId);
    if (!port_node) /* no advance parameters for FMan-Port */
        return E_OK;

    uint32_prop = (uint32_t *)of_get_property(port_node, "num-tnums", &lenp);
    if (uint32_prop) {
    	if (WARN_ON(lenp != sizeof(uint32_t)*2))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

        portRsrc.num   = be32_to_cpu(uint32_prop[0]);
        portRsrc.extra = be32_to_cpu(uint32_prop[1]);

        if ((err = FM_PORT_ConfigNumOfTasks(p_LnxWrpFmPortDev->h_Dev,
                                            &portRsrc)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }

    uint32_prop = (uint32_t *)of_get_property(port_node, "num-dmas", &lenp);
    if (uint32_prop) {
    	if (WARN_ON(lenp != sizeof(uint32_t)*2))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

        portRsrc.num   = be32_to_cpu(uint32_prop[0]);
        portRsrc.extra = be32_to_cpu(uint32_prop[1]);

        if ((err = FM_PORT_ConfigNumOfOpenDmas(p_LnxWrpFmPortDev->h_Dev,
                                            &portRsrc)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }

    uint32_prop = (uint32_t *)of_get_property(port_node, "fifo-size", &lenp);
    if (uint32_prop) {
    	if (WARN_ON(lenp != sizeof(uint32_t)*2))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

        portRsrc.num   = be32_to_cpu(uint32_prop[0]);
        portRsrc.extra = be32_to_cpu(uint32_prop[1]);

        if ((err = FM_PORT_ConfigSizeOfFifo(p_LnxWrpFmPortDev->h_Dev,
                                            &portRsrc)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }

    uint32_prop = (uint32_t *)of_get_property(port_node, "errors-to-discard", &lenp);
    if (uint32_prop) {
    	if (WARN_ON(lenp != sizeof(uint32_t)))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);
        if ((err = FM_PORT_ConfigErrorsToDiscard(p_LnxWrpFmPortDev->h_Dev,
                                                 be32_to_cpu(uint32_prop[0]))) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }

    uint32_prop = (uint32_t *)of_get_property(port_node, "ar-tables-sizes",
	&lenp);
    if (uint32_prop) {

    	if (WARN_ON(lenp != sizeof(uint32_t)*8))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);
    	if (WARN_ON(p_LnxWrpFmPortDev->settings.param.portType !=
		e_FM_PORT_TYPE_RX) &&
		(p_LnxWrpFmPortDev->settings.param.portType !=
		e_FM_PORT_TYPE_RX_10G))
            RETURN_ERROR(MINOR, E_INVALID_VALUE,
		("Auto Response is an Rx port atribute."));

        memset(&p_LnxWrpFmPortDev->dsar_table_sizes, 0, sizeof(struct auto_res_tables_sizes));

        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_arp_entries        =
		(uint16_t)be32_to_cpu(uint32_prop[0]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_echo_ipv4_entries  =
		(uint16_t)be32_to_cpu(uint32_prop[1]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_ndp_entries        =
		(uint16_t)be32_to_cpu(uint32_prop[2]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_echo_ipv6_entries  =
		(uint16_t)be32_to_cpu(uint32_prop[3]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_snmp_ipv4_entries   =
		(uint16_t)be32_to_cpu(uint32_prop[4]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_snmp_ipv6_entries   =
		(uint16_t)be32_to_cpu(uint32_prop[5]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_snmp_oid_entries   =
		(uint16_t)be32_to_cpu(uint32_prop[6]);
        p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_snmp_char          =
		(uint16_t)be32_to_cpu(uint32_prop[7]);

	uint32_prop = (uint32_t *)of_get_property(port_node,
		"ar-filters-sizes", &lenp);
        if (uint32_prop) {
        	if (WARN_ON(lenp != sizeof(uint32_t)*3))
                RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

            p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_ip_prot_filtering  =
		(uint16_t)be32_to_cpu(uint32_prop[0]);
            p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_tcp_port_filtering =
		(uint16_t)be32_to_cpu(uint32_prop[1]);
            p_LnxWrpFmPortDev->dsar_table_sizes.max_num_of_udp_port_filtering =
		(uint16_t)be32_to_cpu(uint32_prop[2]);
        }

        if ((err = FM_PORT_ConfigDsarSupport(p_LnxWrpFmPortDev->h_Dev,
		(t_FmPortDsarTablesSizes*)&p_LnxWrpFmPortDev->dsar_table_sizes)) != E_OK)
		RETURN_ERROR(MINOR, err, NO_MSG);
    }

    of_node_put(port_node);
    of_node_put(fm_node);

    return E_OK;
}

static t_Error CheckNSetFmPortAdvArgs (t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
    struct device_node      *fm_node, *port_node;
    t_Error                 err;
    const uint32_t          *uint32_prop;
    /*const char              *str_prop;*/
    int                     lenp;

    fm_node = GetFmAdvArgsDevTreeNode(((t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev)->id);
    if (!fm_node) /* no advance parameters for FMan */
        return E_OK;

    port_node = GetFmPortAdvArgsDevTreeNode(fm_node,
                                            p_LnxWrpFmPortDev->settings.param.portType,
                                            p_LnxWrpFmPortDev->settings.param.portId);
    if (!port_node) /* no advance parameters for FMan-Port */
        return E_OK;

#if (DPAA_VERSION >= 11)
    uint32_prop = (uint32_t *)of_get_property(port_node, "vsp-window", &lenp);
    if (uint32_prop) {
        t_FmPortVSPAllocParams  portVSPAllocParams;
        t_FmVspParams           fmVspParams;
        t_LnxWrpFmDev           *p_LnxWrpFmDev;
        uint8_t                 portId;

        p_LnxWrpFmDev = ((t_LnxWrpFmDev *)p_LnxWrpFmPortDev->h_LnxWrpFmDev);

    	if (WARN_ON(lenp != sizeof(uint32_t)*2))
            RETURN_ERROR(MINOR, E_INVALID_VALUE, NO_MSG);

        if ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX) ||
            (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_TX_10G) ||
            ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
             p_LnxWrpFmPortDev->settings.frag_enabled))
            return E_OK;

        memset(&portVSPAllocParams, 0, sizeof(portVSPAllocParams));
        memset(&fmVspParams, 0, sizeof(fmVspParams));

        portVSPAllocParams.numOfProfiles = (uint8_t)be32_to_cpu(uint32_prop[0]);
        portVSPAllocParams.dfltRelativeId = (uint8_t)be32_to_cpu(uint32_prop[1]);
        fmVspParams.h_Fm = p_LnxWrpFmDev->h_Dev;

        fmVspParams.portParams.portType = p_LnxWrpFmPortDev->settings.param.portType;
        fmVspParams.portParams.portId   = p_LnxWrpFmPortDev->settings.param.portId;
        fmVspParams.relativeProfileId   = portVSPAllocParams.dfltRelativeId;

        if (p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_OH_OFFLINE_PARSING)
        {
            portId = fmVspParams.portParams.portId;
            if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX_10G){
#ifndef CONFIG_FMAN_ARM
		if (!(IS_T1023_T1024))
#endif
                    portId += FM_MAX_NUM_OF_1G_RX_PORTS;
	    }
	    portVSPAllocParams.h_FmTxPort =
                p_LnxWrpFmDev->txPorts[portId].h_Dev;
            fmVspParams.liodnOffset =
                p_LnxWrpFmDev->rxPorts[portId].settings.param.specificParams.rxParams.liodnOffset;
            memcpy(&fmVspParams.extBufPools,
                   &p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.extBufPools,
                   sizeof(t_FmExtPools));
        }
        else
        {
            memcpy(&fmVspParams.extBufPools,
                   &p_LnxWrpFmPortDev->opExtPools,
                   sizeof(t_FmExtPools));
        }

        if ((err = FM_PORT_VSPAlloc(p_LnxWrpFmPortDev->h_Dev,
                                    &portVSPAllocParams)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);

        /* We're initializing only the default VSP that are being used by the Linux-Ethernet-driver */
        if ((p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
            !p_LnxWrpFmPortDev->opExtPools.numOfPoolsUsed)
            return E_OK;

        p_LnxWrpFmPortDev->h_DfltVsp = FM_VSP_Config(&fmVspParams);
        if (!p_LnxWrpFmPortDev->h_DfltVsp)
            RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("default-VSP for port!"));

        if ((err = FM_VSP_ConfigBufferPrefixContent(p_LnxWrpFmPortDev->h_DfltVsp,
                                                    &p_LnxWrpFmPortDev->buffPrefixContent)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);

        if ((err = FM_VSP_Init(p_LnxWrpFmPortDev->h_DfltVsp)) != E_OK)
            RETURN_ERROR(MINOR, err, NO_MSG);
    }
#else
UNUSED(err); UNUSED(uint32_prop); UNUSED(lenp);
#endif /* (DPAA_VERSION >= 11) */

    of_node_put(port_node);
    of_node_put(fm_node);

    return E_OK;
}

static t_Error ConfigureFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev =
		(t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev;
	struct resource *dev_res;

	if (!p_LnxWrpFmPortDev->active)
		RETURN_ERROR(MAJOR, E_INVALID_STATE,
			     ("FM port not configured!!!"));

	dev_res =
		__devm_request_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res,
				      p_LnxWrpFmPortDev->phys_baseAddr,
				      p_LnxWrpFmPortDev->memSize,
				      "fman-port-hc");
	if (unlikely(dev_res == NULL))
		RETURN_ERROR(MAJOR, E_INVALID_STATE,
			     ("__devm_request_region() failed"));
	p_LnxWrpFmPortDev->baseAddr =
		PTR_TO_UINT(devm_ioremap
			    (p_LnxWrpFmDev->dev,
			     p_LnxWrpFmPortDev->phys_baseAddr,
			     p_LnxWrpFmPortDev->memSize));
	if (unlikely(p_LnxWrpFmPortDev->baseAddr == 0))
		REPORT_ERROR(MAJOR, E_INVALID_STATE,
			     ("devm_ioremap() failed"));

	p_LnxWrpFmPortDev->settings.param.baseAddr =
		p_LnxWrpFmPortDev->baseAddr;

	return E_OK;
}

static t_Error InitFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
#define MY_ADV_CONFIG_CHECK_END \
		RETURN_ERROR(MAJOR, E_INVALID_SELECTION,\
			("Advanced configuration routine"));\
		if (errCode != E_OK)\
			RETURN_ERROR(MAJOR, errCode, NO_MSG);\
	}

	int i = 0;

	if (!p_LnxWrpFmPortDev->active || p_LnxWrpFmPortDev->h_Dev)
		return E_INVALID_STATE;

	p_LnxWrpFmPortDev->h_Dev =
		     FM_PORT_Config(&p_LnxWrpFmPortDev->settings.param);
	if (p_LnxWrpFmPortDev->h_Dev == NULL)
		RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM-port"));

#ifndef  FM_QMI_NO_DEQ_OPTIONS_SUPPORT
	if ((p_LnxWrpFmPortDev->settings.param.portType ==
	     e_FM_PORT_TYPE_TX_10G)
	    || (p_LnxWrpFmPortDev->settings.param.portType ==
		e_FM_PORT_TYPE_TX)) {
		t_Error errCode = E_OK;
		errCode =
		     FM_PORT_ConfigDeqHighPriority(p_LnxWrpFmPortDev->h_Dev,
								   TRUE);
		if (errCode != E_OK)
			RETURN_ERROR(MAJOR, errCode, NO_MSG);
		errCode =
		FM_PORT_ConfigDeqPrefetchOption(p_LnxWrpFmPortDev->h_Dev,
						e_FM_PORT_DEQ_FULL_PREFETCH);
		if (errCode
		    != E_OK)
			RETURN_ERROR(MAJOR, errCode, NO_MSG);
	}
#endif  /* !FM_QMI_NO_DEQ_OPTIONS_SUPPORT */

#ifndef CONFIG_FMAN_ARM
#ifdef FM_BCB_ERRATA_BMI_SW001
/* Configure BCB workaround on Rx ports, only for B4860 rev1 */
#define SVR_SECURITY_MASK    0x00080000
#define SVR_PERSONALITY_MASK 0x0000FF00
#define SVR_VER_IGNORE_MASK (SVR_SECURITY_MASK | SVR_PERSONALITY_MASK)
#define SVR_B4860_REV1_VALUE 0x86800010

	if ((p_LnxWrpFmPortDev->settings.param.portType ==
		e_FM_PORT_TYPE_RX_10G) ||
		(p_LnxWrpFmPortDev->settings.param.portType ==
		e_FM_PORT_TYPE_RX)) {
		unsigned int svr;

		svr = mfspr(SPRN_SVR);

		if ((svr & ~SVR_VER_IGNORE_MASK) == SVR_B4860_REV1_VALUE)
			FM_PORT_ConfigBCBWorkaround(p_LnxWrpFmPortDev->h_Dev);
	}
#endif /* FM_BCB_ERRATA_BMI_SW001 */
#endif /* CONFIG_FMAN_ARM */
/* Call the driver's advanced configuration routines, if requested:
   Compare the function pointer of each entry to the available routines,
   and invoke the matching routine with proper casting of arguments. */
	while (p_LnxWrpFmPortDev->settings.advConfig[i].p_Function
	       && (i < FM_MAX_NUM_OF_ADV_SETTINGS)) {

/* TODO: Change this MACRO */
			ADV_CONFIG_CHECK_START(
				&(p_LnxWrpFmPortDev->settings.advConfig[i]))

			ADV_CONFIG_CHECK(p_LnxWrpFmPortDev->h_Dev,
					 FM_PORT_ConfigBufferPrefixContent,
					 NCSW_PARAMS(1,
						(t_FmBufferPrefixContent *)))

			if ((p_LnxWrpFmPortDev->settings.param.portType ==
				    e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
				   (p_LnxWrpFmPortDev->settings.frag_enabled == TRUE)) {

				ADV_CONFIG_CHECK(p_LnxWrpFmPortDev->h_Dev,
					FM_PORT_ConfigExtBufPools,
					NCSW_PARAMS(1, (t_FmExtPools *)))

		/* this define contains an else */
		MY_ADV_CONFIG_CHECK_END
		}

			/* Advance to next advanced configuration entry */
			i++;
	}


    if ((p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_TX) &&
        (p_LnxWrpFmPortDev->settings.param.portType != e_FM_PORT_TYPE_TX_10G)) {
            if (FM_PORT_ConfigErrorsToDiscard(p_LnxWrpFmPortDev->h_Dev, (FM_PORT_FRM_ERR_IPRE |
                                                                         FM_PORT_FRM_ERR_IPR_NCSP |
                                                                         FM_PORT_FRM_ERR_CLS_DISCARD)) !=E_OK)
            RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
    }

    if (CheckNConfigFmPortAdvArgs(p_LnxWrpFmPortDev) != E_OK)
		RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    if (FM_PORT_Init(p_LnxWrpFmPortDev->h_Dev) != E_OK)
		RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

    if (CheckNSetFmPortAdvArgs(p_LnxWrpFmPortDev) != E_OK)
		RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);

/* FMan Fifo sizes behind the scene":
 * Using the following formulae (*), under a set of simplifying assumptions (.):
 *  . all ports are configured in Normal Mode (rather than Independent Mode)
 *  . the DPAA Eth driver allocates buffers of size:
 *      . MAXFRM + NET_IP_ALIGN + DPA_PRIV_DATA_SIZE + DPA_PARSE_RESULTS_SIZE
 *		 + DPA_HASH_RESULTS_SIZE, i.e.:
 *        MAXFRM + 2 + 16 + sizeof(t_FmPrsResult) + 16, i.e.:
 *        MAXFRM + 66
 *  . excessive buffer pools not accounted for
 *
 *  * for Rx ports on P4080:
 *      . IFSZ = ceil(max(FMBM_EBMPI[PBS]) / 256) * 256 + 7 * 256
 *      . no internal frame offset (FMBM_RIM[FOF] == 0) - otherwise,
 *      add up to 256 to the above
 *
 *  * for Rx ports on P1023:
 *      . IFSZ = ceil(second_largest(FMBM_EBMPI[PBS] / 256)) * 256 + 7 * 256,
 *      if at least 2 bpools are configured
 *      . IFSZ = 8 * 256, if only a single bpool is configured
 *
 *  * for Tx ports:
 *      . IFSZ = ceil(frame_size / 256) * 256 + 3 * 256
 *			+ FMBM_TFP[DPDE] * 256, i.e.:
 *        IFSZ = ceil(MAXFRM / 256) * 256 + 3 x 256 + FMBM_TFP[DPDE] * 256
 *
 *  * for OH ports on P4080:
 *      . IFSZ = ceil(frame_size / 256) * 256 + 1 * 256 + FMBM_PP[MXT] * 256
 *  * for OH ports on P1023:
 *      . IFSZ = ceil(frame_size / 256) * 256 + 3 * 256 + FMBM_TFP[DPDE] * 256
 *  * for both P4080 and P1023:
 *      . (conservative decisions, assuming that BMI must bring the entire
 *      frame, not only the frame header)
 *      . no internal frame offset (FMBM_OIM[FOF] == 0) - otherwise,
 *      add up to 256 to the above
 *
 *  . for P4080/P5020/P3041/P2040, DPDE is:
 *              > 0 or 1, for 1Gb ports, HW default: 0
 *              > 2..7 (recommended: 3..7) for 10Gb ports, HW default: 3
 *  . for P1023, DPDE should be 1
 *
 *  . for P1023, MXT is in range (0..31)
 *  . for P4080, MXT is in range (0..63)
 *
 */
#if 0
	if ((p_LnxWrpFmPortDev->defPcd != e_NO_PCD) &&
	    (InitFmPort3TupleDefPcd(p_LnxWrpFmPortDev) != E_OK))
		RETURN_ERROR(MAJOR, E_INVALID_STATE, NO_MSG);
#endif
	return E_OK;
}

void fm_set_rx_port_params(struct fm_port *port,
			   struct fm_port_params *params)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *) port;
	int i;

	p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.errFqid =
		params->errq;
	p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.dfltFqid =
		params->defq;
	p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.extBufPools.
		numOfPoolsUsed = params->num_pools;
	for (i = 0; i < params->num_pools; i++) {
		p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.
			extBufPools.extBufPool[i].id =
			params->pool_param[i].id;
		p_LnxWrpFmPortDev->settings.param.specificParams.rxParams.
			extBufPools.extBufPool[i].size =
			params->pool_param[i].size;
	}

	p_LnxWrpFmPortDev->buffPrefixContent.privDataSize =
		params->priv_data_size;
	p_LnxWrpFmPortDev->buffPrefixContent.passPrsResult =
		params->parse_results;
	p_LnxWrpFmPortDev->buffPrefixContent.passHashResult =
		params->hash_results;
	p_LnxWrpFmPortDev->buffPrefixContent.passTimeStamp =
		params->time_stamp;
	p_LnxWrpFmPortDev->buffPrefixContent.dataAlign =
		params->data_align;
	p_LnxWrpFmPortDev->buffPrefixContent.manipExtraSpace =
		params->manip_extra_space;

	ADD_ADV_CONFIG_START(p_LnxWrpFmPortDev->settings.advConfig,
			     FM_MAX_NUM_OF_ADV_SETTINGS)

		ADD_ADV_CONFIG_NO_RET(FM_PORT_ConfigBufferPrefixContent,
				      ARGS(1,
					   (&p_LnxWrpFmPortDev->
					    buffPrefixContent)));

	ADD_ADV_CONFIG_END InitFmPortDev(p_LnxWrpFmPortDev);
}
EXPORT_SYMBOL(fm_set_rx_port_params);

/* this function is called from oh_probe as well, thus it contains oh port
 * specific parameters (make sure everything is checked) */
void fm_set_tx_port_params(struct fm_port *port,
			   struct fm_port_params *params)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = (t_LnxWrpFmPortDev *) port;

	p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.errFqid =
		params->errq;
	p_LnxWrpFmPortDev->settings.param.specificParams.nonRxParams.
		dfltFqid = params->defq;

	p_LnxWrpFmPortDev->buffPrefixContent.privDataSize =
		params->priv_data_size;
	p_LnxWrpFmPortDev->buffPrefixContent.passPrsResult =
		params->parse_results;
	p_LnxWrpFmPortDev->buffPrefixContent.passHashResult =
		params->hash_results;
	p_LnxWrpFmPortDev->buffPrefixContent.passTimeStamp =
		params->time_stamp;
	p_LnxWrpFmPortDev->settings.frag_enabled =
		params->frag_enable;
	p_LnxWrpFmPortDev->buffPrefixContent.dataAlign =
		params->data_align;
	p_LnxWrpFmPortDev->buffPrefixContent.manipExtraSpace =
		params->manip_extra_space;

	ADD_ADV_CONFIG_START(p_LnxWrpFmPortDev->settings.advConfig,
			     FM_MAX_NUM_OF_ADV_SETTINGS)

	ADD_ADV_CONFIG_NO_RET(FM_PORT_ConfigBufferPrefixContent,
			      ARGS(1,
				   (&p_LnxWrpFmPortDev->
				    buffPrefixContent)));

	/* oh port specific parameter (for fragmentation only) */
	if ((p_LnxWrpFmPortDev->settings.param.portType ==
	     e_FM_PORT_TYPE_OH_OFFLINE_PARSING) &&
	     params->num_pools) {
		int i;

		p_LnxWrpFmPortDev->opExtPools.numOfPoolsUsed = params->num_pools;
		for (i = 0; i < params->num_pools; i++) {
			p_LnxWrpFmPortDev->opExtPools.extBufPool[i].id = params->pool_param[i].id;
			p_LnxWrpFmPortDev->opExtPools.extBufPool[i].size = params->pool_param[i].size;
		}

		if (p_LnxWrpFmPortDev->settings.frag_enabled)
		ADD_ADV_CONFIG_NO_RET(FM_PORT_ConfigExtBufPools,
				      ARGS(1, (&p_LnxWrpFmPortDev->opExtPools)));
	}

	ADD_ADV_CONFIG_END InitFmPortDev(p_LnxWrpFmPortDev);
}
EXPORT_SYMBOL(fm_set_tx_port_params);

void fm_mac_set_handle(t_Handle h_lnx_wrp_fm_dev,
        t_Handle h_fm_mac,
        int mac_id)
{
    t_LnxWrpFmDev *p_lnx_wrp_fm_dev = (t_LnxWrpFmDev *)h_lnx_wrp_fm_dev;

    p_lnx_wrp_fm_dev->macs[mac_id].h_Dev = h_fm_mac;
    p_lnx_wrp_fm_dev->macs[mac_id].h_LnxWrpFmDev = h_lnx_wrp_fm_dev;
}
EXPORT_SYMBOL(fm_mac_set_handle);

static void LnxwrpFmPcdDevExceptionsCb(t_Handle h_App,
				       e_FmPcdExceptions exception)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev = (t_LnxWrpFmDev *) h_App;

	ASSERT_COND(p_LnxWrpFmDev);

	DBG(INFO, ("got fm-pcd exception %d", exception));

	/* do nothing */
	UNUSED(exception);
}

static void LnxwrpFmPcdDevIndexedExceptionsCb(t_Handle h_App,
					      e_FmPcdExceptions exception,
					      uint16_t index)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev = (t_LnxWrpFmDev *) h_App;

	ASSERT_COND(p_LnxWrpFmDev);

	DBG(INFO,
	    ("got fm-pcd-indexed exception %d, indx %d", exception, index));

	/* do nothing */
	UNUSED(exception);
	UNUSED(index);
}

static t_Error InitFmPcdDev(t_LnxWrpFmDev *p_LnxWrpFmDev)
{
	spin_lock_init(&lock);

	if (p_LnxWrpFmDev->pcdActive) {
		t_LnxWrpFmPortDev *p_LnxWrpFmPortDev = &p_LnxWrpFmDev->hcPort;
		t_FmPcdParams fmPcdParams;
		t_Error err;

		memset(&fmPcdParams, 0, sizeof(fmPcdParams));
		fmPcdParams.h_Fm = p_LnxWrpFmDev->h_Dev;
		fmPcdParams.prsSupport = p_LnxWrpFmDev->prsActive;
		fmPcdParams.kgSupport = p_LnxWrpFmDev->kgActive;
		fmPcdParams.plcrSupport = p_LnxWrpFmDev->plcrActive;
		fmPcdParams.ccSupport = p_LnxWrpFmDev->ccActive;
		fmPcdParams.numOfSchemes = FM_PCD_KG_NUM_OF_SCHEMES;

#ifndef CONFIG_GUEST_PARTITION
		fmPcdParams.f_Exception = LnxwrpFmPcdDevExceptionsCb;
		if (fmPcdParams.kgSupport)
			fmPcdParams.f_ExceptionId =
				LnxwrpFmPcdDevIndexedExceptionsCb;
		fmPcdParams.h_App = p_LnxWrpFmDev;
#endif /* !CONFIG_GUEST_PARTITION */

#ifdef CONFIG_MULTI_PARTITION_SUPPORT
		fmPcdParams.numOfSchemes = 0;
		fmPcdParams.numOfClsPlanEntries = 0;
		fmPcdParams.partitionId = 0;
#endif /* CONFIG_MULTI_PARTITION_SUPPORT */
		fmPcdParams.useHostCommand = TRUE;

		p_LnxWrpFmDev->hc_tx_fq =
			FqAlloc(p_LnxWrpFmDev,
				0,
				QMAN_FQ_FLAG_TO_DCPORTAL,
				p_LnxWrpFmPortDev->txCh, 0);
		if (!p_LnxWrpFmDev->hc_tx_fq)
			RETURN_ERROR(MAJOR, E_NULL_POINTER,
				     ("Frame queue allocation failed..."));

		p_LnxWrpFmDev->hc_tx_conf_fq =
			FqAlloc(p_LnxWrpFmDev,
				0,
				QMAN_FQ_FLAG_NO_ENQUEUE,
				p_LnxWrpFmDev->hcCh, 1);
		if (!p_LnxWrpFmDev->hc_tx_conf_fq)
			RETURN_ERROR(MAJOR, E_NULL_POINTER,
				     ("Frame queue allocation failed..."));

		p_LnxWrpFmDev->hc_tx_err_fq =
			FqAlloc(p_LnxWrpFmDev,
				0,
				QMAN_FQ_FLAG_NO_ENQUEUE,
				p_LnxWrpFmDev->hcCh, 2);
		if (!p_LnxWrpFmDev->hc_tx_err_fq)
			RETURN_ERROR(MAJOR, E_NULL_POINTER,
				     ("Frame queue allocation failed..."));

		fmPcdParams.hc.portBaseAddr = p_LnxWrpFmPortDev->baseAddr;
		fmPcdParams.hc.portId =
			p_LnxWrpFmPortDev->settings.param.portId;
		fmPcdParams.hc.liodnBase =
			p_LnxWrpFmPortDev->settings.param.liodnBase;
		fmPcdParams.hc.errFqid =
			qman_fq_fqid(p_LnxWrpFmDev->hc_tx_err_fq);
		fmPcdParams.hc.confFqid =
			qman_fq_fqid(p_LnxWrpFmDev->hc_tx_conf_fq);
		fmPcdParams.hc.qmChannel = p_LnxWrpFmPortDev->txCh;
		fmPcdParams.hc.f_QmEnqueue = QmEnqueueCB;
		fmPcdParams.hc.h_QmArg = (t_Handle) p_LnxWrpFmDev;

		p_LnxWrpFmDev->h_PcdDev = FM_PCD_Config(&fmPcdParams);
		if (!p_LnxWrpFmDev->h_PcdDev)
			RETURN_ERROR(MAJOR, E_INVALID_HANDLE, ("FM PCD!"));

		err =
		FM_PCD_ConfigPlcrNumOfSharedProfiles(p_LnxWrpFmDev->h_PcdDev,
				LNXWRP_FM_NUM_OF_SHARED_PROFILES);
		if (err != E_OK)
			RETURN_ERROR(MAJOR, err, NO_MSG);

		err = FM_PCD_Init(p_LnxWrpFmDev->h_PcdDev);
		if (err != E_OK)
			RETURN_ERROR(MAJOR, err, NO_MSG);

		if (p_LnxWrpFmDev->err_irq == 0) {
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_KG_EXCEPTION_DOUBLE_ECC,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_KG_EXCEPTION_KEYSIZE_OVERFLOW,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_PLCR_EXCEPTION_INIT_ENTRY_ERROR,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_PLCR_EXCEPTION_DOUBLE_ECC,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_PRS_EXCEPTION_DOUBLE_ECC,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
			    e_FM_PCD_PLCR_EXCEPTION_PRAM_SELF_INIT_COMPLETE,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_PLCR_EXCEPTION_ATOMIC_ACTION_COMPLETE,
				FALSE);
			FM_PCD_SetException(p_LnxWrpFmDev->h_PcdDev,
				e_FM_PCD_PRS_EXCEPTION_SINGLE_ECC,
				FALSE);
		}
	}

	return E_OK;
}

void FreeFmPcdDev(t_LnxWrpFmDev *p_LnxWrpFmDev)
{

	if (p_LnxWrpFmDev->h_PcdDev)
		FM_PCD_Free(p_LnxWrpFmDev->h_PcdDev);

	if (p_LnxWrpFmDev->hc_tx_err_fq)
		FqFree(p_LnxWrpFmDev->hc_tx_err_fq);

	if (p_LnxWrpFmDev->hc_tx_conf_fq)
		FqFree(p_LnxWrpFmDev->hc_tx_conf_fq);

	if (p_LnxWrpFmDev->hc_tx_fq)
		FqFree(p_LnxWrpFmDev->hc_tx_fq);
}

static void FreeFmPortDev(t_LnxWrpFmPortDev *p_LnxWrpFmPortDev)
{
	t_LnxWrpFmDev *p_LnxWrpFmDev =
		(t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev;

	if (!p_LnxWrpFmPortDev->active)
		return;

	if (p_LnxWrpFmPortDev->h_Dev)
		FM_PORT_Free(p_LnxWrpFmPortDev->h_Dev);

	devm_iounmap(p_LnxWrpFmDev->dev,
		     UINT_TO_PTR(p_LnxWrpFmPortDev->baseAddr));
	__devm_release_region(p_LnxWrpFmDev->dev, p_LnxWrpFmDev->res,
			      p_LnxWrpFmPortDev->phys_baseAddr,
			      p_LnxWrpFmPortDev->memSize);
}

static int /*__devinit*/ fm_port_probe(struct platform_device *of_dev)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
	t_LnxWrpFmDev *p_LnxWrpFmDev;
	struct device *dev;

	dev = &of_dev->dev;

	p_LnxWrpFmPortDev = ReadFmPortDevTreeNode(of_dev);
	if (p_LnxWrpFmPortDev == NULL)
		return -EIO;
	/* Port can be inactive, thus will not be probed:
	   - in performance mode, OH ports are disabled
	   ...
	 */
	if (!p_LnxWrpFmPortDev->active)
		return 0;

	if (ConfigureFmPortDev(p_LnxWrpFmPortDev) != E_OK)
		return -EIO;

	dev_set_drvdata(dev, p_LnxWrpFmPortDev);

	if (p_LnxWrpFmPortDev->settings.param.portType ==
		e_FM_PORT_TYPE_OH_HOST_COMMAND)
		InitFmPcdDev((t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev);

	p_LnxWrpFmDev = (t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev;

	if (p_LnxWrpFmPortDev->settings.param.portType == e_FM_PORT_TYPE_RX) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-rx%d",
		       p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + DEV_FM_RX_PORTS_MINOR_BASE;
	} else if (p_LnxWrpFmPortDev->settings.param.portType ==
		 e_FM_PORT_TYPE_RX_10G) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-rx%d",
		       p_LnxWrpFmDev->name,
		       p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_RX_PORTS);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_RX_PORTS +
			DEV_FM_RX_PORTS_MINOR_BASE;
#ifndef CONFIG_FMAN_ARM
		if (IS_T1023_T1024) {
			Sprint(p_LnxWrpFmPortDev->name, "%s-port-rx%d",
				p_LnxWrpFmDev->name,
				p_LnxWrpFmPortDev->id);
			p_LnxWrpFmPortDev->minor =
				p_LnxWrpFmPortDev->id +
				DEV_FM_RX_PORTS_MINOR_BASE;
		}
#endif
	} else if (p_LnxWrpFmPortDev->settings.param.portType ==
		 e_FM_PORT_TYPE_TX) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-tx%d",
		       p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + DEV_FM_TX_PORTS_MINOR_BASE;
	} else if (p_LnxWrpFmPortDev->settings.param.portType ==
		 e_FM_PORT_TYPE_TX_10G) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-tx%d",
		       p_LnxWrpFmDev->name,
		       p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_TX_PORTS);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + FM_MAX_NUM_OF_1G_TX_PORTS +
			DEV_FM_TX_PORTS_MINOR_BASE;
#ifndef CONFIG_FMAN_ARM
		if (IS_T1023_T1024) {
			Sprint(p_LnxWrpFmPortDev->name, "%s-port-tx%d",
				p_LnxWrpFmDev->name,
				p_LnxWrpFmPortDev->id);
			p_LnxWrpFmPortDev->minor =
				p_LnxWrpFmPortDev->id +
				DEV_FM_TX_PORTS_MINOR_BASE;
		}
#endif
	} else if (p_LnxWrpFmPortDev->settings.param.portType ==
		 e_FM_PORT_TYPE_OH_HOST_COMMAND) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-oh%d",
		       p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + DEV_FM_OH_PORTS_MINOR_BASE;
	} else if (p_LnxWrpFmPortDev->settings.param.portType ==
		 e_FM_PORT_TYPE_OH_OFFLINE_PARSING) {
		Sprint(p_LnxWrpFmPortDev->name, "%s-port-oh%d",
		       p_LnxWrpFmDev->name, p_LnxWrpFmPortDev->id + 1);
		p_LnxWrpFmPortDev->minor =
			p_LnxWrpFmPortDev->id + 1 +
			DEV_FM_OH_PORTS_MINOR_BASE;
	}

	device_create(p_LnxWrpFmDev->fm_class, NULL,
		      MKDEV(p_LnxWrpFmDev->major, p_LnxWrpFmPortDev->minor),
		      NULL, p_LnxWrpFmPortDev->name);

	/* create sysfs entries for stats and regs */

	if (fm_port_sysfs_create(dev) != 0) {
		FreeFmPortDev(p_LnxWrpFmPortDev);
		REPORT_ERROR(MAJOR, E_INVALID_STATE,
			     ("Unable to create sys entry - fm port!!!"));
		return -EIO;
	}

#ifdef FM_TX_INVALID_ECC_ERRATA_10GMAC_A009
	FM_DisableRamsEcc(p_LnxWrpFmDev->h_Dev);
#endif /* FM_TX_INVALID_ECC_ERRATA_10GMAC_A009 */

	DBG(TRACE, ("%s probed", p_LnxWrpFmPortDev->name));

	return 0;
}

static int fm_port_remove(struct platform_device *of_dev)
{
	t_LnxWrpFmPortDev *p_LnxWrpFmPortDev;
	t_LnxWrpFmDev *p_LnxWrpFmDev;
	struct device *dev;

	dev = &of_dev->dev;
	p_LnxWrpFmPortDev = dev_get_drvdata(dev);

	fm_port_sysfs_destroy(dev);

	p_LnxWrpFmDev = (t_LnxWrpFmDev *) p_LnxWrpFmPortDev->h_LnxWrpFmDev;
	device_destroy(p_LnxWrpFmDev->fm_class,
		       MKDEV(p_LnxWrpFmDev->major, p_LnxWrpFmPortDev->minor));

	FreeFmPortDev(p_LnxWrpFmPortDev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static const struct of_device_id fm_port_match[] = {
	{
	 .compatible = "fsl,fman-port-oh"},
	{
	 .compatible = "fsl,fman-v2-port-oh"},
	{
	 .compatible = "fsl,fman-v3-port-oh"},
	{
	 .compatible = "fsl,fman-port-1g-rx"},
	{
	 .compatible = "fsl,fman-port-10g-rx"},
	{
	 .compatible = "fsl,fman-port-1g-tx"},
	{
	 .compatible = "fsl,fman-port-10g-tx"},
	{}
};

#ifndef MODULE
MODULE_DEVICE_TABLE(of, fm_port_match);
#endif /* !MODULE */

static struct platform_driver fm_port_driver = {

	.driver = {
		   .name = "fsl-fman-port",
		   .of_match_table = fm_port_match,
		   .owner = THIS_MODULE,
		   },
	.probe = fm_port_probe,
	.remove = fm_port_remove
};


t_Error LNXWRP_FM_Port_Init(void)
{
	/* Register to the DTB for basic FM port API */
	if (platform_driver_register(&fm_port_driver))
		return E_NO_DEVICE;

	return E_OK;
}

void LNXWRP_FM_Port_Free(void)
{
	platform_driver_unregister(&fm_port_driver);
}

static int __init __cold fm_port_load(void)
{
	if (LNXWRP_FM_Port_Init() != E_OK) {
		printk(KERN_CRIT "Failed to init FM Ports wrapper!\n");
		return -ENODEV;
	}

	printk(KERN_CRIT "Freescale FM Ports module\n");

	return 0;
}

static void __exit __cold fm_port_unload(void)
{
	LNXWRP_FM_Port_Free();
}

module_init(fm_port_load);
module_exit(fm_port_unload);
