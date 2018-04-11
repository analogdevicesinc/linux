// SPDX-License-Identifier: GPL-2.0
/*
 * CAAM RNG instantiation driver backend
 *
 * Copyright 2017-2018 NXP
 */

#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/wait.h>
#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "jr.h"
#include "desc_constr.h"
#include "error.h"
#include "ctrl.h"
#include "inst_rng.h"

static DECLARE_WAIT_QUEUE_HEAD(wq_desc);
static int desc_completed;
static int desc_status;

/*
 * Descriptor to instantiate RNG State Handle 0 in normal mode and
 * load the JDKEK, TDKEK and TDSK registers
 */
static void build_instantiation_desc(u32 *desc, int handle, int do_sk)
{
	u32 *jump_cmd, op_flags;

	init_job_desc(desc, 0);

	op_flags = OP_TYPE_CLASS1_ALG | OP_ALG_ALGSEL_RNG |
			(handle << OP_ALG_AAI_SHIFT) | OP_ALG_AS_INIT;

	/* INIT RNG in non-test mode */
	append_operation(desc, op_flags);

	if (!handle && do_sk) {
		/*
		 * For SH0, Secure Keys must be generated as well
		 */

		/* wait for done */
		jump_cmd = append_jump(desc, JUMP_CLASS_CLASS1);
		set_jump_tgt_here(desc, jump_cmd);

		/*
		 * load 1 to clear written reg:
		 * resets the done interrupt and returns the RNG to idle.
		 */
		append_load_imm_u32(desc, 1, LDST_SRCDST_WORD_CLRW);

		/* Initialize State Handle  */
		append_operation(desc, OP_TYPE_CLASS1_ALG | OP_ALG_ALGSEL_RNG |
				 OP_ALG_AAI_RNG4_SK);
	}

	append_jump(desc, JUMP_CLASS_CLASS1 | JUMP_TYPE_HALT);
}

/* Descriptor for deinstantiation of State Handle 0 of the RNG block. */
static void build_deinstantiation_desc(u32 *desc, int handle)
{
	init_job_desc(desc, 0);

	/* Uninstantiate State Handle 0 */
	append_operation(desc, OP_TYPE_CLASS1_ALG | OP_ALG_ALGSEL_RNG |
			 (handle << OP_ALG_AAI_SHIFT) | OP_ALG_AS_INITFINAL);

	append_jump(desc, JUMP_CLASS_CLASS1 | JUMP_TYPE_HALT);
}

void cbk_jr_rng_inst(struct device *jrdev, u32 *desc,  u32 status, void *areq)
{
	if ((status & JRSTA_SSRC_JUMP_HALT_CC) == JRSTA_SSRC_JUMP_HALT_CC) {
		dev_info(jrdev, "Instantiated RNG4 SH%d.\n", *((int *)areq));
		desc_status = 0;
	} else {
		desc_status = -EAGAIN;
	}
	desc_completed = 1;
	wake_up(&wq_desc);
}

/*
 * run_descriptor_jr - runs a descriptor on first JR
 * @status - descriptor status, after being run
 *
 * Return: - 0 if no error occurred
 *	   - -ENODEV if the DECO couldn't be acquired
 *	   - -EAGAIN if an error occurred while executing the descriptor
 */
static int run_descriptor_jr(u32 *desc, int sh_idx)
{
	struct device *jrdev;
	int ret;

	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev)) {
		pr_err("Job Ring Device allocation for transform failed\n");
		return -ENODEV;
	}
	ret = caam_jr_enqueue(jrdev, desc, cbk_jr_rng_inst, &sh_idx);
	if (ret) {
		dev_err(jrdev, "caam_jr_enqueue() failed\n");
		return ret;
	}
	/* wait for job descriptor completion */
	wait_event(wq_desc, desc_completed != 0);
	desc_completed = 0;
	caam_jr_free(jrdev);
	return desc_status;
}

/*
 * instantiate_rng - builds and executes a descriptor on JR0,
 *		     which initializes the RNG block.
 * @state_handle_mask - bitmask containing the instantiation status
 *			for the RNG4 state handles which exist in
 *			the RNG4 block: 1 if it's been instantiated
 *			by an external entry, 0 otherwise.
 * @gen_sk  - generate data to be loaded into the JDKEK, TDKEK and TDSK;
 *	      Caution: this can be done only once; if the keys need to be
 *	      regenerated, a POR is required
 *
 * Return: - 0 if no error occurred
 *	   - -ENOMEM if there isn't enough memory to allocate the descriptor
 *	   - -ENODEV if DECO0 couldn't be acquired
 *	   - -EAGAIN if an error occurred when executing the descriptor
 *	      f.i. there was a RNG hardware error due to not "good enough"
 *	      entropy being acquired.
 */
static int instantiate_rng(int state_handle_mask, int gen_sk)
{
	u32 *desc;
	int sh_idx, ret = 0;

	desc = kmalloc(CAAM_CMD_SZ * 7, GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	for (sh_idx = 0; sh_idx < RNG4_MAX_HANDLES; sh_idx++) {
		/*
		 * If the corresponding bit is set, this state handle
		 * was initialized by somebody else, so it's left alone.
		 */
		if ((1 << sh_idx) & state_handle_mask)
			continue;

		/* Create the descriptor for instantiating RNG State Handle */
		build_instantiation_desc(desc, sh_idx, gen_sk);

		/* Try to run it through JR */
		ret = run_descriptor_jr(desc, sh_idx);
		if (ret)
			pr_debug("Failed to run desc  RNG4 SH%d status (0x%x)\n",
				 sh_idx, ret);
		/* Clear the contents before recreating the descriptor */
		memset(desc, 0x00, CAAM_CMD_SZ * 7);
	}

	kfree(desc);

	return ret;
}

/*
 * deinstantiate_rng - builds and executes a descriptor on JR0,
 *		       which deinitializes the RNG block.
 * @state_handle_mask - bitmask containing the instantiation status
 *			for the RNG4 state handles which exist in
 *			the RNG4 block: 1 if it's been instantiated
 *
 * Return: - 0 if no error occurred
 *	   - -ENOMEM if there isn't enough memory to allocate the descriptor
 *	   - -ENODEV if DECO0 couldn't be acquired
 *	   - -EAGAIN if an error occurred when executing the descriptor
 */
int deinstantiate_rng(int state_handle_mask)
{
	u32 *desc;
	int sh_idx, ret = 0;

	desc = kmalloc(CAAM_CMD_SZ * 3, GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	for (sh_idx = 0; sh_idx < RNG4_MAX_HANDLES; sh_idx++) {
		/*
		 * If the corresponding bit is set, then it means the state
		 * handle was initialized by us, and thus it needs to be
		 * deinitialized as well
		 */
		if ((1 << sh_idx) & state_handle_mask) {
			/*
			 * Create the descriptor for deinstantating this state
			 * handle
			 */
			build_deinstantiation_desc(desc, sh_idx);

			/* Try to run it through JR */
			ret = run_descriptor_jr(desc, sh_idx);
			if (ret)
				pr_debug("Failed to run desc to deinstantiate RNG4 SH%d\n",
					 sh_idx);
		}
	}

	kfree(desc);

	return ret;
}

/*
 * kick_trng - sets the various parameters for enabling the initialization
 *	       of the RNG4 block in CAAM
 * @ctrldev - pointer to the device
 * @ent_delay - Defines the length (in system clocks) of each entropy sample.
 */
static void kick_trng(struct device *ctrldev, int ent_delay)
{
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(ctrldev);
	struct caam_ctrl __iomem *ctrl;
	struct rng4tst __iomem *r4tst;
	u32 val;

	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;
	r4tst = &ctrl->r4tst[0];

	/* put RNG4 into program mode */
	clrsetbits_32(&r4tst->rtmctl, 0, RTMCTL_PRGM);

	/*
	 * Performance-wise, it does not make sense to
	 * set the delay to a value that is lower
	 * than the last one that worked (i.e. the state handles
	 * were instantiated properly. Thus, instead of wasting
	 * time trying to set the values controlling the sample
	 * frequency, the function simply returns.
	 */
	val = (rd_reg32(&r4tst->rtsdctl) & RTSDCTL_ENT_DLY_MASK)
	      >> RTSDCTL_ENT_DLY_SHIFT;
	if (ent_delay <= val) {
		/* put RNG4 into run mode */
		clrsetbits_32(&r4tst->rtmctl, RTMCTL_PRGM, 0);
		return;
	}

	val = rd_reg32(&r4tst->rtsdctl);
	val = (val & ~RTSDCTL_ENT_DLY_MASK) |
	      (ent_delay << RTSDCTL_ENT_DLY_SHIFT);
	wr_reg32(&r4tst->rtsdctl, val);
	/* min. freq. count, equal to 1/4 of the entropy sample length */
	wr_reg32(&r4tst->rtfrqmin, ent_delay >> 2);
	/* max. freq. count, equal to 16 times the entropy sample length */
	wr_reg32(&r4tst->rtfrqmax, ent_delay << 4);
	/* read the control register */
	val = rd_reg32(&r4tst->rtmctl);
	/*
	 * select raw sampling in both entropy shifter
	 * and statistical checker
	 */
	clrsetbits_32(&val, 0, RTMCTL_SAMP_MODE_RAW_ES_SC);
	/* put RNG4 into run mode */
	clrsetbits_32(&val, RTMCTL_PRGM, 0);
	/* write back the control register */
	wr_reg32(&r4tst->rtmctl, val);
}

/*
 * inst_rng_imx - RNG instantiation function for i.MX6/7/8m platforms
 * @pdev - pointer to the device
 */
int inst_rng_imx(struct platform_device *pdev)
{
	struct device *ctrldev, *dev;
	struct caam_drv_private *ctrlpriv;
	struct caam_ctrl __iomem *ctrl;
	int ret = 0, gen_sk, ent_delay = RTSDCTL_ENT_DLY_MIN;
	u32 cha_vid_ls;

	dev = &pdev->dev;
	ctrldev = pdev->dev.parent;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ctrl = (struct caam_ctrl __iomem *)ctrlpriv->ctrl;

#ifndef CONFIG_ARM64
	/*
	 * Check if the Secure Firmware is running,
	 * check only for i.MX6 and i.MX7
	 */
	if (of_find_compatible_node(NULL, NULL, "linaro,optee-tz")) {
		pr_info("RNG Instantation done by Secure Firmware\n");
		return ret;
	}
#endif

	cha_vid_ls = rd_reg32(&ctrl->perfmon.cha_id_ls);

	/*
	 * If SEC has RNG version >= 4 and RNG state handle has not been
	 * already instantiated, do RNG instantiation
	 * In case of DPAA 2.x, RNG is managed by MC firmware.
	 */
	if (!caam_dpaa2 &&
		(cha_vid_ls & CHA_ID_LS_RNG_MASK) >> CHA_ID_LS_RNG_SHIFT >= 4) {
		ctrlpriv->rng4_sh_init =
			rd_reg32(&ctrl->r4tst[0].rdsta);
		/*
		 * If the secure keys (TDKEK, JDKEK, TDSK), were already
		 * generated, signal this to the function that is instantiating
		 * the state handles. An error would occur if RNG4 attempts
		 * to regenerate these keys before the next POR.
		 */
		gen_sk = ctrlpriv->rng4_sh_init & RDSTA_SKVN ? 0 : 1;
		ctrlpriv->rng4_sh_init &= RDSTA_IFMASK;
		do {
			int inst_handles =
				rd_reg32(&ctrl->r4tst[0].rdsta) &
								RDSTA_IFMASK;
			/*
			 * If either SH were instantiated by somebody else
			 * (e.g. u-boot) then it is assumed that the entropy
			 * parameters are properly set and thus the function
			 * setting these (kick_trng(...)) is skipped.
			 * Also, if a handle was instantiated, do not change
			 * the TRNG parameters.
			 */
			if (!(ctrlpriv->rng4_sh_init || inst_handles)) {
				dev_info(dev,
					 "Entropy delay = %u\n",
					 ent_delay);
				kick_trng(ctrldev, ent_delay);
				ent_delay += ENT_DELAY_STEP;
			}
			/*
			 * if instantiate_rng(...) fails, the loop will rerun
			 * and the kick_trng(...) function will modfiy the
			 * upper and lower limits of the entropy sampling
			 * interval, leading to a sucessful initialization of
			 * the RNG.
			 */
			ret = instantiate_rng(inst_handles, gen_sk);
			if (ret == -EAGAIN)
				/*
				 * if here, the loop will rerun,
				 * so don't hog the CPU
				 */
				cpu_relax();
		} while ((ret == -EAGAIN) && (ent_delay < RTSDCTL_ENT_DLY_MAX));
		if (ret) {
			dev_err(dev, "failed to instantiate RNG");
			return ret;
		}
		/*
		 * Set handles init'ed by this module as the complement of the
		 * already initialized ones
		 */
		ctrlpriv->rng4_sh_init = ~ctrlpriv->rng4_sh_init & RDSTA_IFMASK;
		/* Enable RDB bit so that RNG works faster */
		clrsetbits_32(&ctrl->scfgr, 0, SCFGR_RDBENABLE);
	}
	return ret;
}

/*
 * deinst_rng - RNG de-instantiation function
 * @pdev - pointer to the device
 */
int deinst_rng(struct platform_device *pdev)
{
	struct device *ctrldev, *dev;
	struct caam_drv_private *ctrlpriv;
	int ret = 0;

	dev = &pdev->dev;
	ctrldev = pdev->dev.parent;
	ctrlpriv = dev_get_drvdata(ctrldev);

	ret = deinstantiate_rng(ctrlpriv->rng4_sh_init);
	return ret;
}
