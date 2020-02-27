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

#include "lnxwrp_sysfs.h"
#include "lnxwrp_sysfs_fm.h"
#include "lnxwrp_fm.h"

#include "../../sdk_fman/Peripherals/FM/inc/fm_common.h"
#include "../../sdk_fman/Peripherals/FM/Pcd/fm_pcd.h"
#include "../../sdk_fman/Peripherals/FM/Pcd/fm_kg.h"
#include "../../sdk_fman/Peripherals/FM/Pcd/fm_plcr.h"

#if defined(__ERR_MODULE__)
#undef __ERR_MODULE__
#endif

#include "../../sdk_fman/Peripherals/FM/fm.h"
#include <linux/delay.h>


static int fm_get_counter(void *h_fm, e_FmCounters cnt_e, uint32_t *cnt_val);

enum fm_dma_match_stats {
	FM_DMA_COUNTERS_CMQ_NOT_EMPTY,
	FM_DMA_COUNTERS_BUS_ERROR,
	FM_DMA_COUNTERS_READ_BUF_ECC_ERROR,
	FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR,
	FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR
};

static const struct sysfs_stats_t fm_sysfs_stats[] = {
	/* FM statistics */
	{
	 .stat_name = "enq_total_frame",
	 .stat_counter = e_FM_COUNTERS_ENQ_TOTAL_FRAME,
	 },
	{
	 .stat_name = "deq_total_frame",
	 .stat_counter = e_FM_COUNTERS_DEQ_TOTAL_FRAME,
	 },
	{
	 .stat_name = "deq_0",
	 .stat_counter = e_FM_COUNTERS_DEQ_0,
	 },
	{
	 .stat_name = "deq_1",
	 .stat_counter = e_FM_COUNTERS_DEQ_1,
	 },
	{
	 .stat_name = "deq_2",
	 .stat_counter = e_FM_COUNTERS_DEQ_2,
	 },
	{
	 .stat_name = "deq_3",
	 .stat_counter = e_FM_COUNTERS_DEQ_3,
	 },
	{
	 .stat_name = "deq_from_default",
	 .stat_counter = e_FM_COUNTERS_DEQ_FROM_DEFAULT,
	 },
	{
	 .stat_name = "deq_from_context",
	 .stat_counter = e_FM_COUNTERS_DEQ_FROM_CONTEXT,
	 },
	{
	 .stat_name = "deq_from_fd",
	 .stat_counter = e_FM_COUNTERS_DEQ_FROM_FD,
	 },
	{
	 .stat_name = "deq_confirm",
	 .stat_counter = e_FM_COUNTERS_DEQ_CONFIRM,
	 },
	/* FM:DMA  statistics */
	{
	 .stat_name = "cmq_not_empty",
	 .stat_counter = FM_DMA_COUNTERS_CMQ_NOT_EMPTY,
	 },
	{
	 .stat_name = "bus_error",
	 .stat_counter = FM_DMA_COUNTERS_BUS_ERROR,
	 },
	{
	 .stat_name = "read_buf_ecc_error",
	 .stat_counter = FM_DMA_COUNTERS_READ_BUF_ECC_ERROR,
	 },
	{
	 .stat_name = "write_buf_ecc_sys_error",
	 .stat_counter = FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR,
	 },
	{
	 .stat_name = "write_buf_ecc_fm_error",
	 .stat_counter = FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR,
	 },
	/* FM:PCD  statistics */
	{
	 .stat_name = "pcd_kg_total",
	 .stat_counter = e_FM_PCD_KG_COUNTERS_TOTAL,
	 },
	{
	 .stat_name = "pcd_plcr_yellow",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_YELLOW,
	 },
	{
	 .stat_name = "pcd_plcr_red",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_RED,
	 },
	{
	 .stat_name = "pcd_plcr_recolored_to_red",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_RED,
	 },
	{
	 .stat_name = "pcd_plcr_recolored_to_yellow",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_RECOLORED_TO_YELLOW,
	 },
	{
	 .stat_name = "pcd_plcr_total",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_TOTAL,
	 },
	{
	 .stat_name = "pcd_plcr_length_mismatch",
	 .stat_counter = e_FM_PCD_PLCR_COUNTERS_LENGTH_MISMATCH,
	 },
	{
	 .stat_name = "pcd_prs_parse_dispatch",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_PARSE_DISPATCH,
	 },
	{
	 .stat_name = "pcd_prs_l2_parse_result_returned",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED,
	 },
	{
	 .stat_name = "pcd_prs_l3_parse_result_returned",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED,
	 },
	{
	 .stat_name = "pcd_prs_l4_parse_result_returned",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED,
	 },
	{
	 .stat_name = "pcd_prs_shim_parse_result_returned",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED,
	 },
	{
	 .stat_name = "pcd_prs_l2_parse_result_returned_with_err",
	 .stat_counter =
	 e_FM_PCD_PRS_COUNTERS_L2_PARSE_RESULT_RETURNED_WITH_ERR,
	 },
	{
	 .stat_name = "pcd_prs_l3_parse_result_returned_with_err",
	 .stat_counter =
	 e_FM_PCD_PRS_COUNTERS_L3_PARSE_RESULT_RETURNED_WITH_ERR,
	 },
	{
	 .stat_name = "pcd_prs_l4_parse_result_returned_with_err",
	 .stat_counter =
	 e_FM_PCD_PRS_COUNTERS_L4_PARSE_RESULT_RETURNED_WITH_ERR,
	 },
	{
	 .stat_name = "pcd_prs_shim_parse_result_returned_with_err",
	 .stat_counter =
	 e_FM_PCD_PRS_COUNTERS_SHIM_PARSE_RESULT_RETURNED_WITH_ERR,
	 },
	{
	 .stat_name = "pcd_prs_soft_prs_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_SOFT_PRS_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_soft_prs_stall_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_SOFT_PRS_STALL_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_hard_prs_cycle_incl_stall_cycles",
	 .stat_counter =
	 e_FM_PCD_PRS_COUNTERS_HARD_PRS_CYCLE_INCL_STALL_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_muram_read_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_MURAM_READ_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_muram_read_stall_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_MURAM_READ_STALL_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_muram_write_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_MURAM_WRITE_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_muram_write_stall_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_MURAM_WRITE_STALL_CYCLES,
	 },
	{
	 .stat_name = "pcd_prs_fpm_command_stall_cycles",
	 .stat_counter = e_FM_PCD_PRS_COUNTERS_FPM_COMMAND_STALL_CYCLES,
	 },
	{}
};


static ssize_t show_fm_risc_load(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	unsigned long flags;
	int m =0;
	int err =0;
	unsigned n = 0;
	t_FmCtrlMon     util;
	uint8_t         i =0 ;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;

	local_irq_save(flags);

	/* Calculate risc load */
	FM_CtrlMonStart(p_wrp_fm_dev->h_Dev);
	msleep(1000);
	FM_CtrlMonStop(p_wrp_fm_dev->h_Dev);

	for (i = 0; i < FM_NUM_OF_CTRL; i++) {
		err |= FM_CtrlMonGetCounters(p_wrp_fm_dev->h_Dev, i, &util);
		m = snprintf(&buf[n],PAGE_SIZE,"\tRisc%u: util-%u%%, efficiency-%u%%\n",
				i, util.percentCnt[0], util.percentCnt[1]);
		n=m+n;
	}

	local_irq_restore(flags);

	return n;
}

/* Fm stats and regs dumps via sysfs */
static ssize_t show_fm_dma_stats(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	t_FmDmaStatus dma_status;
	unsigned long flags = 0;
	unsigned n = 0;
	uint8_t counter_value = 0, counter = 0;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;

	counter = fm_find_statistic_counter_by_name(
			attr->attr.name,
			fm_sysfs_stats, NULL);

	local_irq_save(flags);

	memset(&dma_status, 0, sizeof(dma_status));
	FM_GetDmaStatus(p_wrp_fm_dev->h_Dev, &dma_status);

	switch (counter) {
	case FM_DMA_COUNTERS_CMQ_NOT_EMPTY:
		counter_value = dma_status.cmqNotEmpty;
		break;
	case FM_DMA_COUNTERS_BUS_ERROR:
		counter_value = dma_status.busError;
		break;
	case FM_DMA_COUNTERS_READ_BUF_ECC_ERROR:
		counter_value = dma_status.readBufEccError;
		break;
	case FM_DMA_COUNTERS_WRITE_BUF_ECC_SYS_ERROR:
		counter_value = dma_status.writeBufEccSysError;
		break;
	case FM_DMA_COUNTERS_WRITE_BUF_ECC_FM_ERROR:
		counter_value = dma_status.writeBufEccFmError;
		break;
	default:
		WARN(1, "FMD: failure at %s:%d/%s()!\n", __FILE__, __LINE__,
			__func__);
		break;
	};

	n = snprintf(buf, PAGE_SIZE, "\tFM %u counter: %c\n",
		p_wrp_fm_dev->id, counter_value ? 'T' : 'F');

	local_irq_restore(flags);

	return n;
}

static ssize_t show_fm_stats(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	unsigned long flags = 0;
	unsigned n = 0, cnt_e = 0;
	uint32_t cnt_val;
	int err;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;

	cnt_e = fm_find_statistic_counter_by_name(
			attr->attr.name,
			fm_sysfs_stats, NULL);

	err = fm_get_counter(p_wrp_fm_dev->h_Dev,
		(e_FmCounters) cnt_e, &cnt_val);

	if (err)
		return err;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "\tFM %d counter: %d\n",
			p_wrp_fm_dev->id, cnt_val);

	local_irq_restore(flags);

	return n;
}

static ssize_t show_fm_muram_free_sz(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	unsigned long flags = 0;
	unsigned n = 0;
	uint64_t muram_free_size = 0;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;

	muram_free_size = FM_MURAM_GetFreeMemSize(p_wrp_fm_dev->h_MuramDev);

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "\tFM %d muram_free_size: %lld\n",
			p_wrp_fm_dev->id, muram_free_size);

	local_irq_restore(flags);

	return n;
}

static ssize_t show_fm_ctrl_code_ver(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	unsigned long flags = 0;
	unsigned n = 0;
	t_FmCtrlCodeRevisionInfo rv_info;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;

	FM_GetFmanCtrlCodeRevision((t_Fm *)p_wrp_fm_dev->h_Dev, &rv_info);

	local_irq_save(flags);

	FM_DMP_LN(buf, n, "- FM %d ctrl code pkg info:\n", p_wrp_fm_dev->id);
	FM_DMP_LN(buf, n, "Package rev: %d\n", rv_info.packageRev);
	FM_DMP_LN(buf, n, "major rev: %d\n", rv_info.majorRev);
	FM_DMP_LN(buf, n, "minor rev: %d\n", rv_info.minorRev);

	local_irq_restore(flags);

	return n;
}

static ssize_t show_fm_pcd_stats(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
	unsigned long flags = 0;
	unsigned n = 0, counter = 0;

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev ||
			!p_wrp_fm_dev->h_PcdDev)
		return -EIO;

	counter = fm_find_statistic_counter_by_name(
			attr->attr.name,
			fm_sysfs_stats, NULL);

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "\tFM %d counter: %d\n",
			p_wrp_fm_dev->id,
			FM_PCD_GetCounter(p_wrp_fm_dev->h_PcdDev,
					(e_FmPcdCounters) counter));

	local_irq_restore(flags);

	return n;
}

static ssize_t show_fm_tnum_dbg(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	if (!p_wrp_fm_dev->active)
		return -EIO;
	else {
		int tn_s;

		if (!sscanf(attr->attr.name, "tnum_dbg_%d", &tn_s))
			return -EINVAL;

		n = fm_dump_tnum_dbg(p_wrp_fm_dev->h_Dev,
					tn_s, tn_s + 15, buf, n);
	}
	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_fm_cls_plan(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "\n FM-KG classification plan dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else {
		int cpn;

		if (!sscanf(attr->attr.name, "cls_plan_%d", &cpn))
			return -EINVAL;

		n = fm_dump_cls_plan(p_wrp_fm_dev->h_PcdDev, cpn, buf, n);
	}
	local_irq_restore(flags);
#else
	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_fm_profiles(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "FM policer profile dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else {
		int pn;

		if (!sscanf(attr->attr.name, "profile_%d", &pn))
			return -EINVAL;

		n = fm_profile_dump_regs(p_wrp_fm_dev->h_PcdDev, pn, buf, n);
	}
	local_irq_restore(flags);
#else
	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_fm_schemes(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "FM-KG driver schemes dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else {
		int sn;

		if (!sscanf(attr->attr.name, "scheme_%d", &sn))
			return -EINVAL;

		n = fm_dump_scheme(p_wrp_fm_dev->h_PcdDev, sn, buf, n);
	}
	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
		     "Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

/* FM */
static DEVICE_ATTR(enq_total_frame, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_total_frame, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(fm_risc_load_val, S_IRUGO, show_fm_risc_load, NULL);
static DEVICE_ATTR(deq_0, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_1, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_2, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_3, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_default, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_context, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_from_fd, S_IRUGO, show_fm_stats, NULL);
static DEVICE_ATTR(deq_confirm, S_IRUGO, show_fm_stats, NULL);
/* FM:DMA */
static DEVICE_ATTR(cmq_not_empty, S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(bus_error, S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(read_buf_ecc_error, S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(write_buf_ecc_sys_error, S_IRUGO, show_fm_dma_stats, NULL);
static DEVICE_ATTR(write_buf_ecc_fm_error, S_IRUGO, show_fm_dma_stats, NULL);
/* FM:PCD */
static DEVICE_ATTR(pcd_kg_total, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_yellow, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_red, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_recolored_to_red, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_plcr_recolored_to_yellow, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_plcr_total, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_plcr_length_mismatch, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_prs_parse_dispatch, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l2_parse_result_returned, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l3_parse_result_returned, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l4_parse_result_returned, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_shim_parse_result_returned, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l2_parse_result_returned_with_err, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l3_parse_result_returned_with_err, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_l4_parse_result_returned_with_err, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_shim_parse_result_returned_with_err, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_soft_prs_cycles, S_IRUGO, show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_soft_prs_stall_cycles, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_prs_hard_prs_cycle_incl_stall_cycles, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_read_cycles, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_prs_muram_read_stall_cycles, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_muram_write_cycles, S_IRUGO, show_fm_pcd_stats,
		   NULL);
static DEVICE_ATTR(pcd_prs_muram_write_stall_cycles, S_IRUGO,
		   show_fm_pcd_stats, NULL);
static DEVICE_ATTR(pcd_prs_fpm_command_stall_cycles, S_IRUGO,
		   show_fm_pcd_stats, NULL);

static DEVICE_ATTR(tnum_dbg_0, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_16, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_32, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_48, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_64, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_80, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_96, S_IRUGO, show_fm_tnum_dbg, NULL);
static DEVICE_ATTR(tnum_dbg_112, S_IRUGO, show_fm_tnum_dbg, NULL);

static DEVICE_ATTR(cls_plan_0, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_1, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_2, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_3, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_4, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_5, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_6, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_7, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_8, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_9, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_10, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_11, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_12, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_13, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_14, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_15, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_16, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_17, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_18, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_19, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_20, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_21, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_22, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_23, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_24, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_25, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_26, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_27, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_28, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_29, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_30, S_IRUGO, show_fm_cls_plan, NULL);
static DEVICE_ATTR(cls_plan_31, S_IRUGO, show_fm_cls_plan, NULL);

static DEVICE_ATTR(profile_0, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_1, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_2, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_3, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_4, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_5, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_6, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_7, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_8, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_9, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_10, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_11, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_12, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_13, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_14, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_15, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_16, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_17, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_18, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_19, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_20, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_21, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_22, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_23, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_24, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_25, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_26, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_27, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_28, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_29, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_30, S_IRUGO, show_fm_profiles, NULL);
static DEVICE_ATTR(profile_31, S_IRUGO, show_fm_profiles, NULL);

static DEVICE_ATTR(scheme_0, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_1, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_2, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_3, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_4, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_5, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_6, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_7, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_8, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_9, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_10, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_11, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_12, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_13, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_14, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_15, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_16, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_17, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_18, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_19, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_20, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_21, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_22, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_23, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_24, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_25, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_26, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_27, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_28, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_29, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_30, S_IRUGO, show_fm_schemes, NULL);
static DEVICE_ATTR(scheme_31, S_IRUGO, show_fm_schemes, NULL);


static struct attribute *fm_dev_stats_attributes[] = {
	&dev_attr_enq_total_frame.attr,
	&dev_attr_deq_total_frame.attr,
	&dev_attr_deq_0.attr,
	&dev_attr_deq_1.attr,
	&dev_attr_deq_2.attr,
	&dev_attr_deq_3.attr,
	&dev_attr_deq_from_default.attr,
	&dev_attr_deq_from_context.attr,
	&dev_attr_deq_from_fd.attr,
	&dev_attr_deq_confirm.attr,
	&dev_attr_cmq_not_empty.attr,
	&dev_attr_bus_error.attr,
	&dev_attr_read_buf_ecc_error.attr,
	&dev_attr_write_buf_ecc_sys_error.attr,
	&dev_attr_write_buf_ecc_fm_error.attr,
	&dev_attr_pcd_kg_total.attr,
	&dev_attr_pcd_plcr_yellow.attr,
	&dev_attr_pcd_plcr_red.attr,
	&dev_attr_pcd_plcr_recolored_to_red.attr,
	&dev_attr_pcd_plcr_recolored_to_yellow.attr,
	&dev_attr_pcd_plcr_total.attr,
	&dev_attr_pcd_plcr_length_mismatch.attr,
	&dev_attr_pcd_prs_parse_dispatch.attr,
	&dev_attr_pcd_prs_l2_parse_result_returned.attr,
	&dev_attr_pcd_prs_l3_parse_result_returned.attr,
	&dev_attr_pcd_prs_l4_parse_result_returned.attr,
	&dev_attr_pcd_prs_shim_parse_result_returned.attr,
	&dev_attr_pcd_prs_l2_parse_result_returned_with_err.attr,
	&dev_attr_pcd_prs_l3_parse_result_returned_with_err.attr,
	&dev_attr_pcd_prs_l4_parse_result_returned_with_err.attr,
	&dev_attr_pcd_prs_shim_parse_result_returned_with_err.attr,
	&dev_attr_pcd_prs_soft_prs_cycles.attr,
	&dev_attr_pcd_prs_soft_prs_stall_cycles.attr,
	&dev_attr_pcd_prs_hard_prs_cycle_incl_stall_cycles.attr,
	&dev_attr_pcd_prs_muram_read_cycles.attr,
	&dev_attr_pcd_prs_muram_read_stall_cycles.attr,
	&dev_attr_pcd_prs_muram_write_cycles.attr,
	&dev_attr_pcd_prs_muram_write_stall_cycles.attr,
	&dev_attr_pcd_prs_fpm_command_stall_cycles.attr,
	NULL
};

static struct attribute *fm_dev_tnums_dbg_attributes[] = {
	&dev_attr_tnum_dbg_0.attr,
	&dev_attr_tnum_dbg_16.attr,
	&dev_attr_tnum_dbg_32.attr,
	&dev_attr_tnum_dbg_48.attr,
	&dev_attr_tnum_dbg_64.attr,
	&dev_attr_tnum_dbg_80.attr,
	&dev_attr_tnum_dbg_96.attr,
	&dev_attr_tnum_dbg_112.attr,
	NULL
};

static struct attribute *fm_dev_cls_plans_attributes[] = {
	&dev_attr_cls_plan_0.attr,
	&dev_attr_cls_plan_1.attr,
	&dev_attr_cls_plan_2.attr,
	&dev_attr_cls_plan_3.attr,
	&dev_attr_cls_plan_4.attr,
	&dev_attr_cls_plan_5.attr,
	&dev_attr_cls_plan_6.attr,
	&dev_attr_cls_plan_7.attr,
	&dev_attr_cls_plan_8.attr,
	&dev_attr_cls_plan_9.attr,
	&dev_attr_cls_plan_10.attr,
	&dev_attr_cls_plan_11.attr,
	&dev_attr_cls_plan_12.attr,
	&dev_attr_cls_plan_13.attr,
	&dev_attr_cls_plan_14.attr,
	&dev_attr_cls_plan_15.attr,
	&dev_attr_cls_plan_16.attr,
	&dev_attr_cls_plan_17.attr,
	&dev_attr_cls_plan_18.attr,
	&dev_attr_cls_plan_19.attr,
	&dev_attr_cls_plan_20.attr,
	&dev_attr_cls_plan_21.attr,
	&dev_attr_cls_plan_22.attr,
	&dev_attr_cls_plan_23.attr,
	&dev_attr_cls_plan_24.attr,
	&dev_attr_cls_plan_25.attr,
	&dev_attr_cls_plan_26.attr,
	&dev_attr_cls_plan_27.attr,
	&dev_attr_cls_plan_28.attr,
	&dev_attr_cls_plan_29.attr,
	&dev_attr_cls_plan_30.attr,
	&dev_attr_cls_plan_31.attr,
	NULL
};

static struct attribute *fm_dev_profiles_attributes[] = {
	&dev_attr_profile_0.attr,
	&dev_attr_profile_1.attr,
	&dev_attr_profile_2.attr,
	&dev_attr_profile_3.attr,
	&dev_attr_profile_4.attr,
	&dev_attr_profile_5.attr,
	&dev_attr_profile_6.attr,
	&dev_attr_profile_7.attr,
	&dev_attr_profile_8.attr,
	&dev_attr_profile_9.attr,
	&dev_attr_profile_10.attr,
	&dev_attr_profile_11.attr,
	&dev_attr_profile_12.attr,
	&dev_attr_profile_13.attr,
	&dev_attr_profile_14.attr,
	&dev_attr_profile_15.attr,
	&dev_attr_profile_16.attr,
	&dev_attr_profile_17.attr,
	&dev_attr_profile_18.attr,
	&dev_attr_profile_19.attr,
	&dev_attr_profile_20.attr,
	&dev_attr_profile_21.attr,
	&dev_attr_profile_22.attr,
	&dev_attr_profile_23.attr,
	&dev_attr_profile_24.attr,
	&dev_attr_profile_25.attr,
	&dev_attr_profile_26.attr,
	&dev_attr_profile_27.attr,
	&dev_attr_profile_28.attr,
	&dev_attr_profile_29.attr,
	&dev_attr_profile_30.attr,
	&dev_attr_profile_31.attr,
	NULL
};

static struct attribute *fm_dev_schemes_attributes[] = {
	&dev_attr_scheme_0.attr,
	&dev_attr_scheme_1.attr,
	&dev_attr_scheme_2.attr,
	&dev_attr_scheme_3.attr,
	&dev_attr_scheme_4.attr,
	&dev_attr_scheme_5.attr,
	&dev_attr_scheme_6.attr,
	&dev_attr_scheme_7.attr,
	&dev_attr_scheme_8.attr,
	&dev_attr_scheme_9.attr,
	&dev_attr_scheme_10.attr,
	&dev_attr_scheme_11.attr,
	&dev_attr_scheme_12.attr,
	&dev_attr_scheme_13.attr,
	&dev_attr_scheme_14.attr,
	&dev_attr_scheme_15.attr,
	&dev_attr_scheme_16.attr,
	&dev_attr_scheme_17.attr,
	&dev_attr_scheme_18.attr,
	&dev_attr_scheme_19.attr,
	&dev_attr_scheme_20.attr,
	&dev_attr_scheme_21.attr,
	&dev_attr_scheme_22.attr,
	&dev_attr_scheme_23.attr,
	&dev_attr_scheme_24.attr,
	&dev_attr_scheme_25.attr,
	&dev_attr_scheme_26.attr,
	&dev_attr_scheme_27.attr,
	&dev_attr_scheme_28.attr,
	&dev_attr_scheme_29.attr,
	&dev_attr_scheme_30.attr,
	&dev_attr_scheme_31.attr,
	NULL
};

static const struct attribute_group fm_dev_stats_attr_grp = {
	.name = "statistics",
	.attrs = fm_dev_stats_attributes
};

static const struct attribute_group fm_dev_tnums_dbg_attr_grp = {
	.name = "tnums_dbg",
	.attrs = fm_dev_tnums_dbg_attributes
};

static const struct attribute_group fm_dev_cls_plans_attr_grp = {
	.name = "cls_plans",
	.attrs = fm_dev_cls_plans_attributes
};

static const struct attribute_group fm_dev_schemes_attr_grp = {
	.name = "schemes",
	.attrs = fm_dev_schemes_attributes
};

static const struct attribute_group fm_dev_profiles_attr_grp = {
	.name = "profiles",
	.attrs = fm_dev_profiles_attributes
};

static ssize_t show_fm_regs(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif
	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "FM driver registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;
	else
		n = fm_dump_regs(p_wrp_fm_dev->h_Dev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_fm_kg_pe_regs(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE,
			"\n FM-KG Port Partition Config registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else
		n = fm_kg_pe_dump_regs(p_wrp_fm_dev->h_PcdDev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_fm_kg_regs(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "FM-KG registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else
		n = fm_kg_dump_regs(p_wrp_fm_dev->h_PcdDev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}


static ssize_t show_fm_fpm_regs(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);

	n = snprintf(buf, PAGE_SIZE, "FM-FPM registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_Dev)
		return -EIO;
	else
		n = fm_fpm_dump_regs(p_wrp_fm_dev->h_Dev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);
#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_prs_regs(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE, "FM Policer registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else
		n = fm_prs_dump_regs(p_wrp_fm_dev->h_PcdDev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
		     "Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static ssize_t show_plcr_regs(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	unsigned long flags;
	unsigned n = 0;
#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;
#endif

	if (attr == NULL || buf == NULL || dev == NULL)
		return -EINVAL;

#if (defined(DEBUG_ERRORS) && (DEBUG_ERRORS > 0))
	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return -EINVAL;

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE, "FM Policer registers dump.\n");

	if (!p_wrp_fm_dev->active || !p_wrp_fm_dev->h_PcdDev)
		return -EIO;
	else
		n = fm_plcr_dump_regs(p_wrp_fm_dev->h_PcdDev, buf, n);

	local_irq_restore(flags);
#else

	local_irq_save(flags);
	n = snprintf(buf, PAGE_SIZE,
			"Debug level is too low to dump registers!!!\n");
	local_irq_restore(flags);

#endif /* (defined(DEBUG_ERRORS) && ... */

	return n;
}

static DEVICE_ATTR(fm_regs, S_IRUGO, show_fm_regs, NULL);
static DEVICE_ATTR(fm_fpm_regs, S_IRUGO, show_fm_fpm_regs, NULL);
static DEVICE_ATTR(fm_kg_regs, S_IRUGO, show_fm_kg_regs, NULL);
static DEVICE_ATTR(fm_kg_pe_regs, S_IRUGO, show_fm_kg_pe_regs, NULL);
static DEVICE_ATTR(fm_plcr_regs, S_IRUGO, show_plcr_regs, NULL);
static DEVICE_ATTR(fm_prs_regs, S_IRUGO, show_prs_regs, NULL);
static DEVICE_ATTR(fm_muram_free_size, S_IRUGO, show_fm_muram_free_sz, NULL);
static DEVICE_ATTR(fm_ctrl_code_ver, S_IRUGO, show_fm_ctrl_code_ver, NULL);

int fm_sysfs_create(struct device *dev)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;

	if (dev == NULL)
		return -EIO;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);

	/* store to remove them when module is disabled */
	p_wrp_fm_dev->dev_attr_regs = &dev_attr_fm_regs;
	p_wrp_fm_dev->dev_attr_risc_load = &dev_attr_fm_risc_load_val;
	p_wrp_fm_dev->dev_fm_fpm_attr_regs = &dev_attr_fm_fpm_regs;
	p_wrp_fm_dev->dev_fm_kg_attr_regs = &dev_attr_fm_kg_regs;
	p_wrp_fm_dev->dev_fm_kg_pe_attr_regs = &dev_attr_fm_kg_pe_regs;
	p_wrp_fm_dev->dev_plcr_attr_regs = &dev_attr_fm_plcr_regs;
	p_wrp_fm_dev->dev_prs_attr_regs = &dev_attr_fm_prs_regs;
	p_wrp_fm_dev->dev_attr_muram_free_size = &dev_attr_fm_muram_free_size;
	p_wrp_fm_dev->dev_attr_fm_ctrl_code_ver = &dev_attr_fm_ctrl_code_ver;

	/* Create sysfs statistics group for FM module */
	if (sysfs_create_group(&dev->kobj, &fm_dev_stats_attr_grp) != 0)
		return -EIO;

	if (sysfs_create_group(&dev->kobj, &fm_dev_schemes_attr_grp) != 0)
		return -EIO;

	if (sysfs_create_group(&dev->kobj, &fm_dev_profiles_attr_grp) != 0)
		return -EIO;

	if (sysfs_create_group(&dev->kobj, &fm_dev_tnums_dbg_attr_grp) != 0)
		return -EIO;

	if (sysfs_create_group(&dev->kobj, &fm_dev_cls_plans_attr_grp) != 0)
		return -EIO;

	/* Registers dump entry - in future will be moved to debugfs */
	if (device_create_file(dev, &dev_attr_fm_regs) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_risc_load_val) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_fpm_regs) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_kg_regs) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_kg_pe_regs) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_plcr_regs) != 0)
		return -EIO;

	if (device_create_file(dev, &dev_attr_fm_prs_regs) != 0)
		return -EIO;

	/* muram free size */
	if (device_create_file(dev, &dev_attr_fm_muram_free_size) != 0)
		return -EIO;

	/* fm ctrl code version */
	if (device_create_file(dev, &dev_attr_fm_ctrl_code_ver) != 0)
		return -EIO;

	return 0;
}

void fm_sysfs_destroy(struct device *dev)
{
	t_LnxWrpFmDev *p_wrp_fm_dev = NULL;

	if (WARN_ON(dev == NULL))
		return;

	p_wrp_fm_dev = (t_LnxWrpFmDev *) dev_get_drvdata(dev);
	if (WARN_ON(p_wrp_fm_dev == NULL))
		return;

	sysfs_remove_group(&dev->kobj, &fm_dev_stats_attr_grp);
	sysfs_remove_group(&dev->kobj, &fm_dev_schemes_attr_grp);
	sysfs_remove_group(&dev->kobj, &fm_dev_profiles_attr_grp);
	sysfs_remove_group(&dev->kobj, &fm_dev_cls_plans_attr_grp);
	sysfs_remove_group(&dev->kobj, &fm_dev_tnums_dbg_attr_grp);
	device_remove_file(dev, p_wrp_fm_dev->dev_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_fm_fpm_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_fm_kg_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_fm_kg_pe_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_plcr_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_prs_attr_regs);
	device_remove_file(dev, p_wrp_fm_dev->dev_attr_muram_free_size);
	device_remove_file(dev, p_wrp_fm_dev->dev_attr_fm_ctrl_code_ver);
}

int fm_dump_regs(void *h_fm, char *buf, int nn)
{
	t_Fm		*p_Fm = (t_Fm *)h_fm;
	uint8_t		i = 0;
	int		n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");

	FM_DMP_TITLE(buf, n, p_Fm->p_FmDmaRegs, "FM-DMA Regs");

	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmsr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmemsr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmmr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmtr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmhy);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmsetr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmtah);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmtal);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmtcid);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmra);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmrd);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmwcr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmebcr);
	FM_DMP_V32(buf, n, p_Fm->p_FmDmaRegs, fmdmdcr);

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmDmaRegs->fmdmplr, "fmdmplr");

	for (i = 0; i < FM_MAX_NUM_OF_HW_PORT_IDS / 2 ; ++i)
		FM_DMP_MEM_32(buf, n, &p_Fm->p_FmDmaRegs->fmdmplr[i]);

	FM_DMP_TITLE(buf, n, p_Fm->p_FmBmiRegs, "FM-BMI COMMON Regs");
	FM_DMP_V32(buf, n, p_Fm->p_FmBmiRegs, fmbm_init);
	FM_DMP_V32(buf, n, p_Fm->p_FmBmiRegs, fmbm_cfg1);
	FM_DMP_V32(buf, n, p_Fm->p_FmBmiRegs, fmbm_cfg2);
	FM_DMP_V32(buf, n, p_Fm->p_FmBmiRegs, fmbm_ievr);
	FM_DMP_V32(buf, n, p_Fm->p_FmBmiRegs, fmbm_ier);

	FM_DMP_TITLE(buf, n, &p_Fm->p_FmBmiRegs->fmbm_arb, "fmbm_arb");
	for (i = 0; i < 8 ; ++i)
		FM_DMP_MEM_32(buf, n, &p_Fm->p_FmBmiRegs->fmbm_arb[i]);

	FM_DMP_TITLE(buf, n, p_Fm->p_FmQmiRegs, "FM-QMI COMMON Regs");
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_gc);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_eie);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_eien);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_eif);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_ie);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_ien);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_if);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_gs);
	FM_DMP_V32(buf, n, p_Fm->p_FmQmiRegs, fmqm_etfc);

	return n;
}

int fm_dump_tnum_dbg(void *h_fm, int tn_s, int tn_e, char *buf, int nn)
{
	t_Fm		*p_Fm = (t_Fm *)h_fm;
	uint8_t		i, j = 0;
	int		n = nn;

	FM_DMP_TITLE(buf, n, NULL, "Tnums and Tnum dbg regs %d - %d",
			tn_s, tn_e);

	iowrite32be(tn_s << 24, &p_Fm->p_FmFpmRegs->fmfp_dra);

	mb();

	for (j = tn_s; j <= tn_e; j++) {
		FM_DMP_LN(buf, n, "> fmfp_ts[%d]\n", j);
		FM_DMP_MEM_32(buf, n, &p_Fm->p_FmFpmRegs->fmfp_ts[j]);
		FM_DMP_V32(buf, n, p_Fm->p_FmFpmRegs, fmfp_dra);
		FM_DMP_LN(buf, n, "> fmfp_drd[0-3]\n");

		for (i = 0; i < 4 ; ++i)
			FM_DMP_MEM_32(buf, n, &p_Fm->p_FmFpmRegs->fmfp_drd[i]);

		FM_DMP_LN(buf, n, "\n");

	}

	return n;
}

int fm_dump_cls_plan(void *h_fm_pcd, int cpn, char *buf, int nn)
{
	t_FmPcd				*p_pcd = (t_FmPcd *)h_fm_pcd;
	int				i = 0;
	uint32_t			tmp;
	unsigned long			i_flg;
	int				n = nn;
	u_FmPcdKgIndirectAccessRegs	*idac;
	spinlock_t			*p_lk;

	p_lk = (spinlock_t *)p_pcd->p_FmPcdKg->h_HwSpinlock;
	idac = p_pcd->p_FmPcdKg->p_IndirectAccessRegs;

	spin_lock_irqsave(p_lk, i_flg);

	/* Read ClsPlan Block Action Regs */
	tmp =  (uint32_t)(FM_KG_KGAR_GO |
		FM_KG_KGAR_READ |
		FM_PCD_KG_KGAR_SEL_CLS_PLAN_ENTRY |
		DUMMY_PORT_ID |
		((uint32_t)cpn << FM_PCD_KG_KGAR_NUM_SHIFT) |
		FM_PCD_KG_KGAR_WSEL_MASK);

	if (fman_kg_write_ar_wait(p_pcd->p_FmPcdKg->p_FmPcdKgRegs, tmp)) {
		FM_DMP_LN(buf, nn, "Keygen scheme access violation");
		spin_unlock_irqrestore(p_lk, i_flg);
		return nn;
	}
	FM_DMP_TITLE(buf, n, &idac->clsPlanRegs,
			"ClsPlan %d Indirect Access Regs", cpn);

	for (i = 0; i < 8; i++)
		FM_DMP_MEM_32(buf, n, &idac->clsPlanRegs.kgcpe[i]);

	spin_unlock_irqrestore(p_lk, i_flg);

	return n;
}

int fm_profile_dump_regs(void *h_fm_pcd, int ppn, char *buf, int nn)
{
	t_FmPcd				*p_pcd = (t_FmPcd *)h_fm_pcd;
	t_FmPcdPlcrProfileRegs		*p_prof_regs;
	t_FmPcdPlcrRegs			*p_plcr_regs;
	t_FmPcdPlcr			*p_plcr;
	uint32_t			tmp;
	unsigned long			i_flg;
	int				n = nn;
	int				toc = 10;
	spinlock_t			*p_lk;

	p_plcr = p_pcd->p_FmPcdPlcr;
	p_prof_regs = &p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs->profileRegs;
	p_plcr_regs = p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs;

	p_lk = (spinlock_t *)((t_FmPcdPlcr *)p_plcr)->h_HwSpinlock;

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_TITLE(buf, n, p_plcr_regs, "FM-PCD policer-profile regs");

	tmp =  (uint32_t)(FM_PCD_PLCR_PAR_GO |
			FM_PCD_PLCR_PAR_R |
			((uint32_t)ppn << FM_PCD_PLCR_PAR_PNUM_SHIFT) |
			FM_PCD_PLCR_PAR_PWSEL_MASK);

	spin_lock_irqsave(p_lk, i_flg);

	iowrite32be(tmp, &p_plcr_regs->fmpl_par);

	mb();

	/* wait for the porfile regs to be present */
	do {
		--toc;
		udelay(10);
		if (!toc) {
			/* looks like PLCR_PAR_GO refuses to clear */
			spin_unlock_irqrestore(p_lk, i_flg);
			FM_DMP_LN(buf, n, "Profile regs not accessible -");
			FM_DMP_LN(buf, n, " check profile init process\n");
			return n;
		}
	} while ((ioread32be(&p_plcr_regs->fmpl_par) & FM_PCD_PLCR_PAR_GO));

	FM_DMP_TITLE(buf, n, p_prof_regs, "Profile %d regs", ppn);

	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pemode);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pegnia);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_peynia);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pernia);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pecir);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pecbs);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pepepir_eir);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pepbs_ebs);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pelts);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pects);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pepts_ets);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_pegpc);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_peypc);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_perpc);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_perypc);
	FM_DMP_V32(buf, n, p_prof_regs, fmpl_perrpc);

	spin_unlock_irqrestore(p_lk, i_flg);

	return n;
}

int fm_dump_scheme(void *h_fm_pcd, int scnum, char *buf, int nn)
{
	t_FmPcd				*p_pcd = (t_FmPcd *)h_fm_pcd;
	uint32_t			tmp_ar;
	unsigned long			i_flg;
	int				i, n = nn;
	spinlock_t			*p_lk;
	u_FmPcdKgIndirectAccessRegs	*idac;

	idac = p_pcd->p_FmPcdKg->p_IndirectAccessRegs;
	p_lk = (spinlock_t *)p_pcd->p_FmPcdKg->h_HwSpinlock;

	spin_lock_irqsave(p_lk, i_flg);

	tmp_ar = FmPcdKgBuildReadSchemeActionReg((uint8_t)scnum);
	if (fman_kg_write_ar_wait(p_pcd->p_FmPcdKg->p_FmPcdKgRegs, tmp_ar)) {
		FM_DMP_LN(buf, nn,
			"Keygen scheme access violation or no such scheme");
		spin_unlock_irqrestore(p_lk, i_flg);
		return nn;
	}

	FM_DMP_TITLE(buf, n, &idac->schemeRegs,
			"Scheme %d Indirect Access Regs", scnum);

	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_mode);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_ekfc);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_ekdv);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_bmch);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_bmcl);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_fqb);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_hc);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_ppc);

	FM_DMP_TITLE(buf, n, &idac->schemeRegs.kgse_gec, "kgse_gec");

	for (i = 0; i < FM_KG_NUM_OF_GENERIC_REGS; i++)
		FM_DMP_MEM_32(buf, n, &idac->schemeRegs.kgse_gec[i]);

	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_spc);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_dv0);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_dv1);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_ccbs);
	FM_DMP_V32(buf, n, &idac->schemeRegs, kgse_mv);

	FM_DMP_SUBTITLE(buf, n, "\n");

	spin_unlock_irqrestore(p_lk, i_flg);

	return n;
}

int fm_kg_pe_dump_regs(void *h_fm_pcd, char *buf, int nn)
{
	t_FmPcd				*p_pcd = (t_FmPcd *)h_fm_pcd;
	int				i = 0;
	uint8_t				prt_id = 0;
	uint32_t			tmp_ar;
	unsigned long			i_flg;
	int				n = nn;
	u_FmPcdKgIndirectAccessRegs	*idac;
	t_FmPcdKg			*p_kg;
	spinlock_t			*p_lk;

	p_kg = p_pcd->p_FmPcdKg;
	idac = p_pcd->p_FmPcdKg->p_IndirectAccessRegs;
	p_lk = (spinlock_t *)p_kg->h_HwSpinlock;

	spin_lock_irqsave(p_lk, i_flg);

	FM_DMP_SUBTITLE(buf, n, "\n");

	for (i = 0; i < FM_MAX_NUM_OF_PORTS; i++) {
		SW_PORT_INDX_TO_HW_PORT_ID(prt_id, i);

		tmp_ar = FmPcdKgBuildReadPortSchemeBindActionReg(prt_id);

		if (fman_kg_write_ar_wait(p_kg->p_FmPcdKgRegs, tmp_ar)) {
			FM_DMP_LN(buf, nn, "Keygen scheme access violation");
			spin_unlock_irqrestore(p_lk, i_flg);
			return nn;
		}
		FM_DMP_TITLE(buf, n, &idac->portRegs, "Port %d regs", prt_id);
		FM_DMP_V32(buf, n, &idac->portRegs, fmkg_pe_sp);
		FM_DMP_V32(buf, n, &idac->portRegs, fmkg_pe_cpp);
	}

	FM_DMP_SUBTITLE(buf, n, "\n");

	spin_unlock_irqrestore(p_lk, i_flg);

	return n;
}

int fm_kg_dump_regs(void *h_fm_pcd, char *buf, int nn)
{
	t_FmPcd		*p_pcd = (t_FmPcd *)h_fm_pcd;
	int			n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_TITLE(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs,
			"FmPcdKgRegs Regs");

	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_gcr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_eer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_eeer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_seer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_seeer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_gsr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_tpc);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_serc);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_fdor);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_gdv0r);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_gdv1r);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_feer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdKg->p_FmPcdKgRegs, fmkg_ar);

	FM_DMP_SUBTITLE(buf, n, "\n");

	return n;
}


int fm_fpm_dump_regs(void *h_fm, char *buf, int nn)
{
	t_Fm		*p_fm = (t_Fm *)h_fm;
	uint8_t		i;
	int		n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");

	FM_DMP_TITLE(buf, n, p_fm->p_FmFpmRegs, "FM-FPM Regs");

	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_tnc);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_prc);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_brkc);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_mxd);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_dist1);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_dist2);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_epi);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_rie);

	FM_DMP_TITLE(buf, n, &p_fm->p_FmFpmRegs->fmfp_fcev, "fmfp_fcev");
	for (i = 0; i < 4; ++i)
		FM_DMP_MEM_32(buf, n, &p_fm->p_FmFpmRegs->fmfp_fcev[i]);

	FM_DMP_TITLE(buf, n, &p_fm->p_FmFpmRegs->fmfp_cee, "fmfp_cee");
	for (i = 0; i < 4; ++i)
		FM_DMP_MEM_32(buf, n, &p_fm->p_FmFpmRegs->fmfp_cee[i]);

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_tsc1);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_tsc2);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_tsp);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_tsf);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_rcr);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_extc);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_ext1);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_ext2);

	FM_DMP_SUBTITLE(buf, n, "\n");
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_ip_rev_1);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_ip_rev_2);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_rstc);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_cld);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fm_npi);
	FM_DMP_V32(buf, n, p_fm->p_FmFpmRegs, fmfp_ee);

	FM_DMP_TITLE(buf, n, &p_fm->p_FmFpmRegs->fmfp_cev, "fmfp_cev");
	for (i = 0; i < 4; ++i)
		FM_DMP_MEM_32(buf, n, &p_fm->p_FmFpmRegs->fmfp_cev[i]);

	FM_DMP_TITLE(buf, n, &p_fm->p_FmFpmRegs->fmfp_ps, "fmfp_ps");
	for (i = 0; i < 64; ++i)
		FM_DMP_MEM_32(buf, n, &p_fm->p_FmFpmRegs->fmfp_ps[i]);

	return n;
}

int fm_prs_dump_regs(void *h_fm_pcd, char *buf, int nn)
{
	t_FmPcd		*p_pcd = (t_FmPcd *)h_fm_pcd;
	int		n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");

	FM_DMP_TITLE(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs,
			"FM-PCD parser regs");

	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_rpclim);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_rpimac);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, pmeec);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_pevr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_pever);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_perr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_perer);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_ppsc);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_pds);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l2rrs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l3rrs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l4rrs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_srrs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l2rres);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l3rres);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_l4rres);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_srres);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_spcs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_spscs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_hxscs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_mrcs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_mwcs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_mrscs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_mwscs);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPrs->p_FmPcdPrsRegs, fmpr_fcscs);

	return n;
}

int fm_plcr_dump_regs(void *h_fm_pcd, char *buf, int nn)
{
	t_FmPcd		*p_pcd = (t_FmPcd *)h_fm_pcd;
	int		i = 0;
	int		n = nn;

	FM_DMP_SUBTITLE(buf, n, "\n");

	FM_DMP_TITLE(buf, n,
			p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs,
			"FM policer regs");

	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_gcr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_gsr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_evr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_ier);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_ifr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_eevr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_eier);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_eifr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_rpcnt);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_ypcnt);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_rrpcnt);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_rypcnt);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_tpcnt);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_flmcnt);

	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_serc);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_upcr);
	FM_DMP_V32(buf, n, p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs, fmpl_dpmr);

	FM_DMP_TITLE(buf, n,
			&p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs->fmpl_pmr,
			"fmpl_pmr");

	for (i = 0; i < 63; ++i)
		FM_DMP_MEM_32(buf, n,
			&p_pcd->p_FmPcdPlcr->p_FmPcdPlcrRegs->fmpl_pmr[i]);

	return n;
}

int fm_get_counter(void *h_fm, e_FmCounters cnt_e, uint32_t *cnt_val)
{
	t_Fm		*p_fm = (t_Fm *)h_fm;

	/* When applicable (when there is an "enable counters" bit),
	check that counters are enabled */

	switch (cnt_e) {
	case (e_FM_COUNTERS_DEQ_1):
	case (e_FM_COUNTERS_DEQ_2):
	case (e_FM_COUNTERS_DEQ_3):
		if (p_fm->p_FmStateStruct->revInfo.majorRev >= 6)
			return -EINVAL; /* counter not available */

	case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
	case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
	case (e_FM_COUNTERS_DEQ_0):
	case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
	case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
	case (e_FM_COUNTERS_DEQ_FROM_FD):
	case (e_FM_COUNTERS_DEQ_CONFIRM):
		if (!(ioread32be(&p_fm->p_FmQmiRegs->fmqm_gc) &
			QMI_CFG_EN_COUNTERS))
			return -EINVAL; /* Requested counter not available */
		break;
	default:
		break;
	}

	switch (cnt_e) {
	case (e_FM_COUNTERS_ENQ_TOTAL_FRAME):
		*cnt_val = ioread32be(&p_fm->p_FmQmiRegs->fmqm_etfc);
		return 0;
	case (e_FM_COUNTERS_DEQ_TOTAL_FRAME):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dtfc);
		return 0;
	case (e_FM_COUNTERS_DEQ_0):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dc0);
		return 0;
	case (e_FM_COUNTERS_DEQ_1):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dc1);
		return 0;
	case (e_FM_COUNTERS_DEQ_2):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dc2);
		return 0;
	case (e_FM_COUNTERS_DEQ_3):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dc3);
		return 0;
	case (e_FM_COUNTERS_DEQ_FROM_DEFAULT):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dfdc);
		return 0;
	case (e_FM_COUNTERS_DEQ_FROM_CONTEXT):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dfcc);
		return 0;
	case (e_FM_COUNTERS_DEQ_FROM_FD):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dffc);
		return 0;
	case (e_FM_COUNTERS_DEQ_CONFIRM):
		*cnt_val =  ioread32be(&p_fm->p_FmQmiRegs->fmqm_dcc);
		return 0;
	}
	/* should never get here */
	return -EINVAL; /* counter not available */
}
