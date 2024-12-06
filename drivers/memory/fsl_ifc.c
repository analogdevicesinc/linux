// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2011 Freescale Semiconductor, Inc
 *
 * Freescale Integrated Flash Controller
 *
 * Author: Dipen Dudhat <Dipen.Dudhat@freescale.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/fsl_ifc.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

struct fsl_ifc_ctrl *fsl_ifc_ctrl_dev;
EXPORT_SYMBOL(fsl_ifc_ctrl_dev);
#define FSL_IFC_V1_3_0	0x01030000
#define IFC_TIMEOUT_MSECS	1000 /* 1000ms */

/*
 * convert_ifc_address - convert the base address
 * @addr_base:	base address of the memory bank
 */
unsigned int convert_ifc_address(phys_addr_t addr_base)
{
	return addr_base & CSPR_BA;
}
EXPORT_SYMBOL(convert_ifc_address);

/*
 * fsl_ifc_find - find IFC bank
 * @addr_base:	base address of the memory bank
 *
 * This function walks IFC banks comparing "Base address" field of the CSPR
 * registers with the supplied addr_base argument. When bases match this
 * function returns bank number (starting with 0), otherwise it returns
 * appropriate errno value.
 */
int fsl_ifc_find(phys_addr_t addr_base)
{
	int i = 0;

	if (!fsl_ifc_ctrl_dev || !fsl_ifc_ctrl_dev->gregs)
		return -ENODEV;

	for (i = 0; i < fsl_ifc_ctrl_dev->banks; i++) {
		u32 cspr = ifc_in32(&fsl_ifc_ctrl_dev->gregs->cspr_cs[i].cspr);

		if (cspr & CSPR_V && (cspr & CSPR_BA) ==
				convert_ifc_address(addr_base))
			return i;
	}

	return -ENOENT;
}
EXPORT_SYMBOL(fsl_ifc_find);

static int fsl_ifc_ctrl_init(struct fsl_ifc_ctrl *ctrl)
{
	struct fsl_ifc_global __iomem *ifc = ctrl->gregs;

	/*
	 * Clear all the common status and event registers
	 */
	if (ifc_in32(&ifc->cm_evter_stat) & IFC_CM_EVTER_STAT_CSER)
		ifc_out32(IFC_CM_EVTER_STAT_CSER, &ifc->cm_evter_stat);

	/* enable all error and events */
	ifc_out32(IFC_CM_EVTER_EN_CSEREN, &ifc->cm_evter_en);

	/* enable all error and event interrupts */
	ifc_out32(IFC_CM_EVTER_INTR_EN_CSERIREN, &ifc->cm_evter_intr_en);
	ifc_out32(0x0, &ifc->cm_erattr0);
	ifc_out32(0x0, &ifc->cm_erattr1);

	return 0;
}

static void fsl_ifc_ctrl_remove(struct platform_device *dev)
{
	struct fsl_ifc_ctrl *ctrl = dev_get_drvdata(&dev->dev);

	of_platform_depopulate(&dev->dev);
	free_irq(ctrl->nand_irq, ctrl);
	free_irq(ctrl->irq, ctrl);

	irq_dispose_mapping(ctrl->nand_irq);
	irq_dispose_mapping(ctrl->irq);

	iounmap(ctrl->gregs);

	dev_set_drvdata(&dev->dev, NULL);
}

/*
 * NAND events are split between an operational interrupt which only
 * receives OPC, and an error interrupt that receives everything else,
 * including non-NAND errors.  Whichever interrupt gets to it first
 * records the status and wakes the wait queue.
 */
static DEFINE_SPINLOCK(nand_irq_lock);

static u32 check_nand_stat(struct fsl_ifc_ctrl *ctrl)
{
	struct fsl_ifc_runtime __iomem *ifc = ctrl->rregs;
	unsigned long flags;
	u32 stat;

	spin_lock_irqsave(&nand_irq_lock, flags);

	stat = ifc_in32(&ifc->ifc_nand.nand_evter_stat);
	if (stat) {
		ifc_out32(stat, &ifc->ifc_nand.nand_evter_stat);
		ctrl->nand_stat = stat;
		wake_up(&ctrl->nand_wait);
	}

	spin_unlock_irqrestore(&nand_irq_lock, flags);

	return stat;
}

static irqreturn_t fsl_ifc_nand_irq(int irqno, void *data)
{
	struct fsl_ifc_ctrl *ctrl = data;

	if (check_nand_stat(ctrl))
		return IRQ_HANDLED;

	return IRQ_NONE;
}

/*
 * NOTE: This interrupt is used to report ifc events of various kinds,
 * such as transaction errors on the chipselects.
 */
static irqreturn_t fsl_ifc_ctrl_irq(int irqno, void *data)
{
	struct fsl_ifc_ctrl *ctrl = data;
	struct fsl_ifc_global __iomem *ifc = ctrl->gregs;
	u32 err_axiid, err_srcid, status, cs_err, err_addr;
	irqreturn_t ret = IRQ_NONE;

	/* read for chip select error */
	cs_err = ifc_in32(&ifc->cm_evter_stat);
	if (cs_err) {
		dev_err(ctrl->dev, "transaction sent to IFC is not mapped to any memory bank 0x%08X\n",
			cs_err);
		/* clear the chip select error */
		ifc_out32(IFC_CM_EVTER_STAT_CSER, &ifc->cm_evter_stat);

		/* read error attribute registers print the error information */
		status = ifc_in32(&ifc->cm_erattr0);
		err_addr = ifc_in32(&ifc->cm_erattr1);

		if (status & IFC_CM_ERATTR0_ERTYP_READ)
			dev_err(ctrl->dev, "Read transaction error CM_ERATTR0 0x%08X\n",
				status);
		else
			dev_err(ctrl->dev, "Write transaction error CM_ERATTR0 0x%08X\n",
				status);

		err_axiid = (status & IFC_CM_ERATTR0_ERAID) >>
					IFC_CM_ERATTR0_ERAID_SHIFT;
		dev_err(ctrl->dev, "AXI ID of the error transaction 0x%08X\n",
			err_axiid);

		err_srcid = (status & IFC_CM_ERATTR0_ESRCID) >>
					IFC_CM_ERATTR0_ESRCID_SHIFT;
		dev_err(ctrl->dev, "SRC ID of the error transaction 0x%08X\n",
			err_srcid);

		dev_err(ctrl->dev, "Transaction Address corresponding to error ERADDR 0x%08X\n",
			err_addr);

		ret = IRQ_HANDLED;
	}

	if (check_nand_stat(ctrl))
		ret = IRQ_HANDLED;

	return ret;
}

/*
 * fsl_ifc_ctrl_probe
 *
 * called by device layer when it finds a device matching
 * one our driver can handled. This code allocates all of
 * the resources needed for the controller only.  The
 * resources for the NAND banks themselves are allocated
 * in the chip probe function.
 */
static int fsl_ifc_ctrl_probe(struct platform_device *dev)
{
	int ret = 0;
	int version, banks;
	void __iomem *addr;

	dev_info(&dev->dev, "Freescale Integrated Flash Controller\n");

	fsl_ifc_ctrl_dev = devm_kzalloc(&dev->dev, sizeof(*fsl_ifc_ctrl_dev),
					GFP_KERNEL);
	if (!fsl_ifc_ctrl_dev)
		return -ENOMEM;

	dev_set_drvdata(&dev->dev, fsl_ifc_ctrl_dev);

	/* IOMAP the entire IFC region */
	fsl_ifc_ctrl_dev->gregs = of_iomap(dev->dev.of_node, 0);
	if (!fsl_ifc_ctrl_dev->gregs) {
		dev_err(&dev->dev, "failed to get memory region\n");
		return -ENODEV;
	}

	if (of_property_read_bool(dev->dev.of_node, "little-endian")) {
		fsl_ifc_ctrl_dev->little_endian = true;
		dev_dbg(&dev->dev, "IFC REGISTERS are LITTLE endian\n");
	} else {
		fsl_ifc_ctrl_dev->little_endian = false;
		dev_dbg(&dev->dev, "IFC REGISTERS are BIG endian\n");
	}

	version = ifc_in32(&fsl_ifc_ctrl_dev->gregs->ifc_rev) &
			FSL_IFC_VERSION_MASK;

	banks = (version == FSL_IFC_VERSION_1_0_0) ? 4 : 8;
	dev_info(&dev->dev, "IFC version %d.%d, %d banks\n",
		version >> 24, (version >> 16) & 0xf, banks);

	fsl_ifc_ctrl_dev->version = version;
	fsl_ifc_ctrl_dev->banks = banks;

	addr = fsl_ifc_ctrl_dev->gregs;
	if (version >= FSL_IFC_VERSION_2_0_0)
		addr += PGOFFSET_64K;
	else
		addr += PGOFFSET_4K;
	fsl_ifc_ctrl_dev->rregs = addr;

	/* get the Controller level irq */
	fsl_ifc_ctrl_dev->irq = irq_of_parse_and_map(dev->dev.of_node, 0);
	if (fsl_ifc_ctrl_dev->irq == 0) {
		dev_err(&dev->dev, "failed to get irq resource for IFC\n");
		ret = -ENODEV;
		goto err;
	}

	/* get the nand machine irq */
	fsl_ifc_ctrl_dev->nand_irq =
			irq_of_parse_and_map(dev->dev.of_node, 1);

	fsl_ifc_ctrl_dev->dev = &dev->dev;

	ret = fsl_ifc_ctrl_init(fsl_ifc_ctrl_dev);
	if (ret < 0)
		goto err_unmap_nandirq;

	init_waitqueue_head(&fsl_ifc_ctrl_dev->nand_wait);

	ret = request_irq(fsl_ifc_ctrl_dev->irq, fsl_ifc_ctrl_irq, IRQF_SHARED,
			  "fsl-ifc", fsl_ifc_ctrl_dev);
	if (ret != 0) {
		dev_err(&dev->dev, "failed to install irq (%d)\n",
			fsl_ifc_ctrl_dev->irq);
		goto err_unmap_nandirq;
	}

	if (fsl_ifc_ctrl_dev->nand_irq) {
		ret = request_irq(fsl_ifc_ctrl_dev->nand_irq, fsl_ifc_nand_irq,
				0, "fsl-ifc-nand", fsl_ifc_ctrl_dev);
		if (ret != 0) {
			dev_err(&dev->dev, "failed to install irq (%d)\n",
				fsl_ifc_ctrl_dev->nand_irq);
			goto err_free_irq;
		}
	}

	/* legacy dts may still use "simple-bus" compatible */
	ret = of_platform_default_populate(dev->dev.of_node, NULL, &dev->dev);
	if (ret)
		goto err_free_nandirq;

	return 0;

err_free_nandirq:
	free_irq(fsl_ifc_ctrl_dev->nand_irq, fsl_ifc_ctrl_dev);
err_free_irq:
	free_irq(fsl_ifc_ctrl_dev->irq, fsl_ifc_ctrl_dev);
err_unmap_nandirq:
	irq_dispose_mapping(fsl_ifc_ctrl_dev->nand_irq);
	irq_dispose_mapping(fsl_ifc_ctrl_dev->irq);
err:
	iounmap(fsl_ifc_ctrl_dev->gregs);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
/* save ifc registers */
static int fsl_ifc_suspend(struct device *dev)
{
	struct fsl_ifc_ctrl *ctrl = dev_get_drvdata(dev);
	struct fsl_ifc_global __iomem *fcm = ctrl->gregs;
	struct fsl_ifc_runtime __iomem *runtime = ctrl->rregs;
	__be32 nand_evter_intr_en, cm_evter_intr_en, nor_evter_intr_en,
							 gpcm_evter_intr_en;
	uint32_t ifc_bank, i;

	ctrl->saved_gregs = kzalloc(sizeof(struct fsl_ifc_global), GFP_KERNEL);
	if (!ctrl->saved_gregs)
		return -ENOMEM;
	ctrl->saved_rregs = kzalloc(sizeof(struct fsl_ifc_runtime), GFP_KERNEL);
	if (!ctrl->saved_rregs)
		return -ENOMEM;

	cm_evter_intr_en = ifc_in32(&fcm->cm_evter_intr_en);
	nand_evter_intr_en = ifc_in32(&runtime->ifc_nand.nand_evter_intr_en);
	nor_evter_intr_en = ifc_in32(&runtime->ifc_nor.nor_evter_intr_en);
	gpcm_evter_intr_en = ifc_in32(&runtime->ifc_gpcm.gpcm_evter_intr_en);

/* IFC interrupts disabled */

	ifc_out32(0x0, &fcm->cm_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_nand.nand_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_nor.nor_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_gpcm.gpcm_evter_intr_en);

	if (ctrl->saved_gregs) {
		for (ifc_bank = 0; ifc_bank < FSL_IFC_BANK_COUNT; ifc_bank++) {
			ctrl->saved_gregs->cspr_cs[ifc_bank].cspr_ext =
				ifc_in32(&fcm->cspr_cs[ifc_bank].cspr_ext);
			ctrl->saved_gregs->cspr_cs[ifc_bank].cspr =
				ifc_in32(&fcm->cspr_cs[ifc_bank].cspr);
			ctrl->saved_gregs->amask_cs[ifc_bank].amask =
				ifc_in32(&fcm->amask_cs[ifc_bank].amask);
			ctrl->saved_gregs->csor_cs[ifc_bank].csor_ext =
				ifc_in32(&fcm->csor_cs[ifc_bank].csor_ext);
			ctrl->saved_gregs->csor_cs[ifc_bank].csor =
				ifc_in32(&fcm->csor_cs[ifc_bank].csor);
			for (i = 0; i < 4; i++) {
				ctrl->saved_gregs->ftim_cs[ifc_bank].ftim[i] =
					ifc_in32(
					&fcm->ftim_cs[ifc_bank].ftim[i]);
			}
		}

		ctrl->saved_gregs->rb_map = ifc_in32(&fcm->rb_map);
		ctrl->saved_gregs->wb_map = ifc_in32(&fcm->wb_map);
		ctrl->saved_gregs->ifc_gcr = ifc_in32(&fcm->ifc_gcr);
		ctrl->saved_gregs->ddr_ccr_low = ifc_in32(&fcm->ddr_ccr_low);
		ctrl->saved_gregs->cm_evter_en = ifc_in32(&fcm->cm_evter_en);
	}

	if (ctrl->saved_rregs) {
		/* IFC controller NAND machine registers */
		ctrl->saved_rregs->ifc_nand.ncfgr =
					ifc_in32(&runtime->ifc_nand.ncfgr);
		ctrl->saved_rregs->ifc_nand.nand_fcr0 =
					ifc_in32(&runtime->ifc_nand.nand_fcr0);
		ctrl->saved_rregs->ifc_nand.nand_fcr1 =
					ifc_in32(&runtime->ifc_nand.nand_fcr1);
		ctrl->saved_rregs->ifc_nand.row0 =
					ifc_in32(&runtime->ifc_nand.row0);
		ctrl->saved_rregs->ifc_nand.row1 =
					ifc_in32(&runtime->ifc_nand.row1);
		ctrl->saved_rregs->ifc_nand.col0 =
					ifc_in32(&runtime->ifc_nand.col0);
		ctrl->saved_rregs->ifc_nand.col1 =
					ifc_in32(&runtime->ifc_nand.col1);
		ctrl->saved_rregs->ifc_nand.row2 =
					ifc_in32(&runtime->ifc_nand.row2);
		ctrl->saved_rregs->ifc_nand.col2 =
					ifc_in32(&runtime->ifc_nand.col2);
		ctrl->saved_rregs->ifc_nand.row3 =
					ifc_in32(&runtime->ifc_nand.row3);
		ctrl->saved_rregs->ifc_nand.col3 =
					ifc_in32(&runtime->ifc_nand.col3);

		ctrl->saved_rregs->ifc_nand.nand_fbcr =
					ifc_in32(&runtime->ifc_nand.nand_fbcr);
		ctrl->saved_rregs->ifc_nand.nand_fir0 =
					ifc_in32(&runtime->ifc_nand.nand_fir0);
		ctrl->saved_rregs->ifc_nand.nand_fir1 =
					ifc_in32(&runtime->ifc_nand.nand_fir1);
		ctrl->saved_rregs->ifc_nand.nand_fir2 =
					ifc_in32(&runtime->ifc_nand.nand_fir2);
		ctrl->saved_rregs->ifc_nand.nand_csel =
					ifc_in32(&runtime->ifc_nand.nand_csel);
		ctrl->saved_rregs->ifc_nand.nandseq_strt =
					ifc_in32(
					&runtime->ifc_nand.nandseq_strt);
		ctrl->saved_rregs->ifc_nand.nand_evter_en =
					ifc_in32(
					&runtime->ifc_nand.nand_evter_en);
		ctrl->saved_rregs->ifc_nand.nanndcr =
					ifc_in32(&runtime->ifc_nand.nanndcr);
		ctrl->saved_rregs->ifc_nand.nand_dll_lowcfg0 =
					ifc_in32(
					&runtime->ifc_nand.nand_dll_lowcfg0);
		ctrl->saved_rregs->ifc_nand.nand_dll_lowcfg1 =
					ifc_in32(
					&runtime->ifc_nand.nand_dll_lowcfg1);

		/* IFC controller NOR machine registers */
		ctrl->saved_rregs->ifc_nor.nor_evter_en =
					ifc_in32(
					&runtime->ifc_nor.nor_evter_en);
		ctrl->saved_rregs->ifc_nor.norcr =
					ifc_in32(&runtime->ifc_nor.norcr);

		/* IFC controller GPCM Machine registers */
		ctrl->saved_rregs->ifc_gpcm.gpcm_evter_en =
					ifc_in32(
					&runtime->ifc_gpcm.gpcm_evter_en);
	}

/* save the interrupt values */
	ctrl->saved_gregs->cm_evter_intr_en = cm_evter_intr_en;
	ctrl->saved_rregs->ifc_nand.nand_evter_intr_en = nand_evter_intr_en;
	ctrl->saved_rregs->ifc_nor.nor_evter_intr_en = nor_evter_intr_en;
	ctrl->saved_rregs->ifc_gpcm.gpcm_evter_intr_en = gpcm_evter_intr_en;

	return 0;
}

/* restore ifc registers */
static int fsl_ifc_resume(struct device *dev)
{
	struct fsl_ifc_ctrl *ctrl = dev_get_drvdata(dev);
	struct fsl_ifc_global __iomem *fcm = ctrl->gregs;
	struct fsl_ifc_runtime __iomem *runtime = ctrl->rregs;
	struct fsl_ifc_global *savd_gregs = ctrl->saved_gregs;
	struct fsl_ifc_runtime *savd_rregs = ctrl->saved_rregs;
	uint32_t ver = 0, ncfgr, timeout, ifc_bank, i;

/*
 * IFC interrupts disabled
 */
	ifc_out32(0x0, &fcm->cm_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_nand.nand_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_nor.nor_evter_intr_en);
	ifc_out32(0x0, &runtime->ifc_gpcm.gpcm_evter_intr_en);


	if (ctrl->saved_gregs) {
		for (ifc_bank = 0; ifc_bank < FSL_IFC_BANK_COUNT; ifc_bank++) {
			ifc_out32(savd_gregs->cspr_cs[ifc_bank].cspr_ext,
					&fcm->cspr_cs[ifc_bank].cspr_ext);
			ifc_out32(savd_gregs->cspr_cs[ifc_bank].cspr,
					&fcm->cspr_cs[ifc_bank].cspr);
			ifc_out32(savd_gregs->amask_cs[ifc_bank].amask,
					&fcm->amask_cs[ifc_bank].amask);
			ifc_out32(savd_gregs->csor_cs[ifc_bank].csor_ext,
					&fcm->csor_cs[ifc_bank].csor_ext);
			ifc_out32(savd_gregs->csor_cs[ifc_bank].csor,
					&fcm->csor_cs[ifc_bank].csor);
			for (i = 0; i < 4; i++) {
				ifc_out32(savd_gregs->ftim_cs[ifc_bank].ftim[i],
					&fcm->ftim_cs[ifc_bank].ftim[i]);
			}
		}
		ifc_out32(savd_gregs->rb_map, &fcm->rb_map);
		ifc_out32(savd_gregs->wb_map, &fcm->wb_map);
		ifc_out32(savd_gregs->ifc_gcr, &fcm->ifc_gcr);
		ifc_out32(savd_gregs->ddr_ccr_low, &fcm->ddr_ccr_low);
		ifc_out32(savd_gregs->cm_evter_en, &fcm->cm_evter_en);
	}

	if (ctrl->saved_rregs) {
		/* IFC controller NAND machine registers */
		ifc_out32(savd_rregs->ifc_nand.ncfgr,
						&runtime->ifc_nand.ncfgr);
		ifc_out32(savd_rregs->ifc_nand.nand_fcr0,
						&runtime->ifc_nand.nand_fcr0);
		ifc_out32(savd_rregs->ifc_nand.nand_fcr1,
						&runtime->ifc_nand.nand_fcr1);
		ifc_out32(savd_rregs->ifc_nand.row0, &runtime->ifc_nand.row0);
		ifc_out32(savd_rregs->ifc_nand.row1, &runtime->ifc_nand.row1);
		ifc_out32(savd_rregs->ifc_nand.col0, &runtime->ifc_nand.col0);
		ifc_out32(savd_rregs->ifc_nand.col1, &runtime->ifc_nand.col1);
		ifc_out32(savd_rregs->ifc_nand.row2, &runtime->ifc_nand.row2);
		ifc_out32(savd_rregs->ifc_nand.col2, &runtime->ifc_nand.col2);
		ifc_out32(savd_rregs->ifc_nand.row3, &runtime->ifc_nand.row3);
		ifc_out32(savd_rregs->ifc_nand.col3, &runtime->ifc_nand.col3);
		ifc_out32(savd_rregs->ifc_nand.nand_fbcr,
						&runtime->ifc_nand.nand_fbcr);
		ifc_out32(savd_rregs->ifc_nand.nand_fir0,
						&runtime->ifc_nand.nand_fir0);
		ifc_out32(savd_rregs->ifc_nand.nand_fir1,
						&runtime->ifc_nand.nand_fir1);
		ifc_out32(savd_rregs->ifc_nand.nand_fir2,
						&runtime->ifc_nand.nand_fir2);
		ifc_out32(savd_rregs->ifc_nand.nand_csel,
						&runtime->ifc_nand.nand_csel);
		ifc_out32(savd_rregs->ifc_nand.nandseq_strt,
					&runtime->ifc_nand.nandseq_strt);
		ifc_out32(savd_rregs->ifc_nand.nand_evter_en,
					&runtime->ifc_nand.nand_evter_en);
		ifc_out32(savd_rregs->ifc_nand.nanndcr,
					&runtime->ifc_nand.nanndcr);
		ifc_out32(savd_rregs->ifc_nand.nand_dll_lowcfg0,
					&runtime->ifc_nand.nand_dll_lowcfg0);
		ifc_out32(savd_rregs->ifc_nand.nand_dll_lowcfg1,
					&runtime->ifc_nand.nand_dll_lowcfg1);

		/* IFC controller NOR machine registers */
		ifc_out32(savd_rregs->ifc_nor.nor_evter_en,
					&runtime->ifc_nor.nor_evter_en);
		ifc_out32(savd_rregs->ifc_nor.norcr, &runtime->ifc_nor.norcr);

		/* IFC controller GPCM Machine registers */
		ifc_out32(savd_rregs->ifc_gpcm.gpcm_evter_en,
					&runtime->ifc_gpcm.gpcm_evter_en);

		/* IFC interrupts enabled */
		ifc_out32(ctrl->saved_gregs->cm_evter_intr_en,
					&fcm->cm_evter_intr_en);
		ifc_out32(ctrl->saved_rregs->ifc_nand.nand_evter_intr_en,
					&runtime->ifc_nand.nand_evter_intr_en);
		ifc_out32(ctrl->saved_rregs->ifc_nor.nor_evter_intr_en,
					&runtime->ifc_nor.nor_evter_intr_en);
		ifc_out32(ctrl->saved_rregs->ifc_gpcm.gpcm_evter_intr_en,
					&runtime->ifc_gpcm.gpcm_evter_intr_en);

		kfree(ctrl->saved_gregs);
		kfree(ctrl->saved_rregs);
		ctrl->saved_gregs = NULL;
		ctrl->saved_rregs = NULL;
	}

	ver = ifc_in32(&fcm->ifc_rev);
	ncfgr = ifc_in32(&runtime->ifc_nand.ncfgr);
	if (ver >= FSL_IFC_V1_3_0) {

		ifc_out32(ncfgr | IFC_NAND_NCFGR_SRAM_INIT_EN,
					&runtime->ifc_nand.ncfgr);
		/* wait for  SRAM_INIT bit to be clear or timeout */
		timeout = 10;
		while ((ifc_in32(&runtime->ifc_nand.ncfgr) &
			IFC_NAND_NCFGR_SRAM_INIT_EN) && timeout) {
			mdelay(IFC_TIMEOUT_MSECS);
			timeout--;
		}

		if (!timeout)
			dev_err(ctrl->dev, "Timeout waiting for IFC SRAM INIT");
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id fsl_ifc_match[] = {
	{
		.compatible = "fsl,ifc",
	},
	{},
};

static const struct dev_pm_ops ifc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fsl_ifc_suspend, fsl_ifc_resume)
};

static struct platform_driver fsl_ifc_ctrl_driver = {
	.driver = {
		.name	= "fsl-ifc",
		.of_match_table = fsl_ifc_match,
		.pm = &ifc_pm_ops,
	},
	.probe       = fsl_ifc_ctrl_probe,
	.remove_new  = fsl_ifc_ctrl_remove,
};

static int __init fsl_ifc_init(void)
{
	return platform_driver_register(&fsl_ifc_ctrl_driver);
}
subsys_initcall(fsl_ifc_init);

MODULE_AUTHOR("Freescale Semiconductor");
MODULE_DESCRIPTION("Freescale Integrated Flash Controller driver");
