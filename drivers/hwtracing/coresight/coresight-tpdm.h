/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _CORESIGHT_CORESIGHT_TPDM_H
#define _CORESIGHT_CORESIGHT_TPDM_H

/* The max number of the datasets that TPDM supports */
#define TPDM_DATASETS       7

/* CMB Subunit Registers */
#define TPDM_CMB_CR		(0xA00)
/* CMB subunit timestamp insertion enable register */
#define TPDM_CMB_TIER		(0xA04)
/* CMB subunit timestamp pattern registers */
#define TPDM_CMB_TPR(n)		(0xA08 + (n * 4))
/* CMB subunit timestamp pattern mask registers */
#define TPDM_CMB_TPMR(n)	(0xA10 + (n * 4))
/* CMB subunit trigger pattern registers */
#define TPDM_CMB_XPR(n)		(0xA18 + (n * 4))
/* CMB subunit trigger pattern mask registers */
#define TPDM_CMB_XPMR(n)	(0xA20 + (n * 4))
/* CMB MSR register */
#define TPDM_CMB_MSR(n)		(0xA80 + (n * 4))

/* Enable bit for CMB subunit */
#define TPDM_CMB_CR_ENA		BIT(0)
/* Trace collection mode for CMB subunit */
#define TPDM_CMB_CR_MODE	BIT(1)
/* Timestamp control for pattern match */
#define TPDM_CMB_TIER_PATT_TSENAB	BIT(0)
/* CMB CTI timestamp request */
#define TPDM_CMB_TIER_XTRIG_TSENAB	BIT(1)
/* For timestamp fo all trace */
#define TPDM_CMB_TIER_TS_ALL		BIT(2)

/* Patten register number */
#define TPDM_CMB_MAX_PATT		2

/* MAX number of DSB MSR */
#define TPDM_CMB_MAX_MSR 32

/* DSB Subunit Registers */
#define TPDM_DSB_CR		(0x780)
#define TPDM_DSB_TIER		(0x784)
#define TPDM_DSB_TPR(n)		(0x788 + (n * 4))
#define TPDM_DSB_TPMR(n)	(0x7A8 + (n * 4))
#define TPDM_DSB_XPR(n)		(0x7C8 + (n * 4))
#define TPDM_DSB_XPMR(n)	(0x7E8 + (n * 4))
#define TPDM_DSB_EDCR(n)	(0x808 + (n * 4))
#define TPDM_DSB_EDCMR(n)	(0x848 + (n * 4))
#define TPDM_DSB_MSR(n)		(0x980 + (n * 4))

/* Enable bit for DSB subunit */
#define TPDM_DSB_CR_ENA		BIT(0)
/* Enable bit for DSB subunit perfmance mode */
#define TPDM_DSB_CR_MODE		BIT(1)
/* Enable bit for DSB subunit trigger type */
#define TPDM_DSB_CR_TRIG_TYPE		BIT(12)
/* Data bits for DSB high performace mode */
#define TPDM_DSB_CR_HPSEL		GENMASK(6, 2)
/* Data bits for DSB test mode */
#define TPDM_DSB_CR_TEST_MODE		GENMASK(10, 9)

/* Enable bit for DSB subunit pattern timestamp */
#define TPDM_DSB_TIER_PATT_TSENAB		BIT(0)
/* Enable bit for DSB subunit trigger timestamp */
#define TPDM_DSB_TIER_XTRIG_TSENAB		BIT(1)
/* Bit for DSB subunit pattern type */
#define TPDM_DSB_TIER_PATT_TYPE			BIT(2)

/* DSB programming modes */
/* DSB mode bits mask */
#define TPDM_DSB_MODE_MASK			GENMASK(8, 0)
/* Test mode control bit*/
#define TPDM_DSB_MODE_TEST(val)	(val & GENMASK(1, 0))
/* Performance mode */
#define TPDM_DSB_MODE_PERF		BIT(3)
/* High performance mode */
#define TPDM_DSB_MODE_HPBYTESEL(val)	(val & GENMASK(8, 4))

#define EDCRS_PER_WORD			16
#define EDCR_TO_WORD_IDX(r)		((r) / EDCRS_PER_WORD)
#define EDCR_TO_WORD_SHIFT(r)		((r % EDCRS_PER_WORD) * 2)
#define EDCR_TO_WORD_VAL(val, r)	(val << EDCR_TO_WORD_SHIFT(r))
#define EDCR_TO_WORD_MASK(r)		EDCR_TO_WORD_VAL(0x3, r)

#define EDCMRS_PER_WORD				32
#define EDCMR_TO_WORD_IDX(r)		((r) / EDCMRS_PER_WORD)
#define EDCMR_TO_WORD_SHIFT(r)		((r) % EDCMRS_PER_WORD)

/* TPDM integration test registers */
#define TPDM_ITATBCNTRL		(0xEF0)
#define TPDM_ITCNTRL		(0xF00)

/* Register value for integration test */
#define ATBCNTRL_VAL_32		0xC00F1409
#define ATBCNTRL_VAL_64		0xC01F1409

/*
 * Number of cycles to write value when
 * integration test.
 */
#define INTEGRATION_TEST_CYCLE	10

/**
 * The bits of PERIPHIDR0 register.
 * The fields [6:0] of PERIPHIDR0 are used to determine what
 * interfaces and subunits are present on a given TPDM.
 *
 * PERIPHIDR0[0] : Fix to 1 if ImplDef subunit present, else 0
 * PERIPHIDR0[1] : Fix to 1 if DSB subunit present, else 0
 * PERIPHIDR0[2] : Fix to 1 if CMB subunit present, else 0
 */

#define TPDM_PIDR0_DS_IMPDEF	BIT(0)
#define TPDM_PIDR0_DS_DSB	BIT(1)
#define TPDM_PIDR0_DS_CMB	BIT(2)

#define TPDM_DSB_MAX_LINES	256
/* MAX number of EDCR registers */
#define TPDM_DSB_MAX_EDCR	16
/* MAX number of EDCMR registers */
#define TPDM_DSB_MAX_EDCMR	8
/* MAX number of DSB pattern */
#define TPDM_DSB_MAX_PATT	8
/* MAX number of DSB MSR */
#define TPDM_DSB_MAX_MSR 32

#define tpdm_simple_dataset_ro(name, mem, idx)			\
	(&((struct tpdm_dataset_attribute[]) {			\
	   {								\
		__ATTR(name, 0444, tpdm_simple_dataset_show, NULL),	\
		mem,							\
		idx,							\
	   }								\
	})[0].attr.attr)

#define tpdm_simple_dataset_rw(name, mem, idx)			\
	(&((struct tpdm_dataset_attribute[]) {			\
	   {								\
		__ATTR(name, 0644, tpdm_simple_dataset_show,		\
		tpdm_simple_dataset_store),		\
		mem,							\
		idx,							\
	   }								\
	})[0].attr.attr)

#define tpdm_patt_enable_ts(name, mem)				\
	(&((struct tpdm_dataset_attribute[]) {			\
	   {							\
		__ATTR(name, 0644, enable_ts_show,		\
		enable_ts_store),		\
		mem,						\
		0,						\
	   }							\
	})[0].attr.attr)

#define DSB_EDGE_CTRL_ATTR(nr)					\
		tpdm_simple_dataset_ro(edcr##nr,		\
		DSB_EDGE_CTRL, nr)

#define DSB_EDGE_CTRL_MASK_ATTR(nr)				\
		tpdm_simple_dataset_ro(edcmr##nr,		\
		DSB_EDGE_CTRL_MASK, nr)

#define DSB_TRIG_PATT_ATTR(nr)					\
		tpdm_simple_dataset_rw(xpr##nr,			\
		DSB_TRIG_PATT, nr)

#define DSB_TRIG_PATT_MASK_ATTR(nr)				\
		tpdm_simple_dataset_rw(xpmr##nr,		\
		DSB_TRIG_PATT_MASK, nr)

#define DSB_PATT_ATTR(nr)					\
		tpdm_simple_dataset_rw(tpr##nr,			\
		DSB_PATT, nr)

#define DSB_PATT_MASK_ATTR(nr)					\
		tpdm_simple_dataset_rw(tpmr##nr,		\
		DSB_PATT_MASK, nr)

#define DSB_PATT_ENABLE_TS					\
		tpdm_patt_enable_ts(enable_ts,			\
		DSB_PATT)

#define DSB_MSR_ATTR(nr)					\
		tpdm_simple_dataset_rw(msr##nr,			\
		DSB_MSR, nr)

#define CMB_TRIG_PATT_ATTR(nr)					\
		tpdm_simple_dataset_rw(xpr##nr,			\
		CMB_TRIG_PATT, nr)

#define CMB_TRIG_PATT_MASK_ATTR(nr)				\
		tpdm_simple_dataset_rw(xpmr##nr,		\
		CMB_TRIG_PATT_MASK, nr)

#define CMB_PATT_ATTR(nr)					\
		tpdm_simple_dataset_rw(tpr##nr,			\
		CMB_PATT, nr)

#define CMB_PATT_MASK_ATTR(nr)					\
		tpdm_simple_dataset_rw(tpmr##nr,		\
		CMB_PATT_MASK, nr)

#define CMB_PATT_ENABLE_TS					\
		tpdm_patt_enable_ts(enable_ts,			\
		CMB_PATT)

#define CMB_MSR_ATTR(nr)					\
		tpdm_simple_dataset_rw(msr##nr,			\
		CMB_MSR, nr)

/**
 * struct dsb_dataset - specifics associated to dsb dataset
 * @mode:             DSB programming mode
 * @edge_ctrl_idx     Index number of the edge control
 * @edge_ctrl:        Save value for edge control
 * @edge_ctrl_mask:   Save value for edge control mask
 * @patt_val:         Save value for pattern
 * @patt_mask:        Save value for pattern mask
 * @trig_patt:        Save value for trigger pattern
 * @trig_patt_mask:   Save value for trigger pattern mask
 * @msr               Save value for MSR
 * @patt_ts:          Enable/Disable pattern timestamp
 * @patt_type:        Set pattern type
 * @trig_ts:          Enable/Disable trigger timestamp.
 * @trig_type:        Enable/Disable trigger type.
 */
struct dsb_dataset {
	u32			mode;
	u32			edge_ctrl_idx;
	u32			edge_ctrl[TPDM_DSB_MAX_EDCR];
	u32			edge_ctrl_mask[TPDM_DSB_MAX_EDCMR];
	u32			patt_val[TPDM_DSB_MAX_PATT];
	u32			patt_mask[TPDM_DSB_MAX_PATT];
	u32			trig_patt[TPDM_DSB_MAX_PATT];
	u32			trig_patt_mask[TPDM_DSB_MAX_PATT];
	u32			msr[TPDM_DSB_MAX_MSR];
	bool			patt_ts;
	bool			patt_type;
	bool			trig_ts;
	bool			trig_type;
};

/**
 * struct cmb_dataset
 * @trace_mode:       Dataset collection mode
 * @patt_val:         Save value for pattern
 * @patt_mask:        Save value for pattern mask
 * @trig_patt:        Save value for trigger pattern
 * @trig_patt_mask:   Save value for trigger pattern mask
 * @msr               Save value for MSR
 * @patt_ts:          Indicates if pattern match for timestamp is enabled.
 * @trig_ts:          Indicates if CTI trigger for timestamp is enabled.
 * @ts_all:           Indicates if timestamp is enabled for all packets.
 */
struct cmb_dataset {
	u32			trace_mode;
	u32			patt_val[TPDM_CMB_MAX_PATT];
	u32			patt_mask[TPDM_CMB_MAX_PATT];
	u32			trig_patt[TPDM_CMB_MAX_PATT];
	u32			trig_patt_mask[TPDM_CMB_MAX_PATT];
	u32			msr[TPDM_CMB_MAX_MSR];
	bool			patt_ts;
	bool			trig_ts;
	bool			ts_all;
};

/**
 * struct tpdm_drvdata - specifics associated to an TPDM component
 * @base:       memory mapped base address for this component.
 * @dev:        The device entity associated to this component.
 * @csdev:      component vitals needed by the framework.
 * @spinlock:   lock for the drvdata value.
 * @enable:     enable status of the component.
 * @datasets:   The datasets types present of the TPDM.
 * @dsb         Specifics associated to TPDM DSB.
 * @cmb         Specifics associated to TPDM CMB.
 * @dsb_msr_num Number of MSR supported by DSB TPDM
 * @cmb_msr_num Number of MSR supported by CMB TPDM
 */

struct tpdm_drvdata {
	void __iomem		*base;
	struct device		*dev;
	struct coresight_device	*csdev;
	spinlock_t		spinlock;
	bool			enable;
	unsigned long		datasets;
	struct dsb_dataset	*dsb;
	struct cmb_dataset	*cmb;
	u32			dsb_msr_num;
	u32			cmb_msr_num;
};

/* Enumerate members of various datasets */
enum dataset_mem {
	DSB_EDGE_CTRL,
	DSB_EDGE_CTRL_MASK,
	DSB_TRIG_PATT,
	DSB_TRIG_PATT_MASK,
	DSB_PATT,
	DSB_PATT_MASK,
	DSB_MSR,
	CMB_TRIG_PATT,
	CMB_TRIG_PATT_MASK,
	CMB_PATT,
	CMB_PATT_MASK,
	CMB_MSR
};

/**
 * struct tpdm_dataset_attribute - Record the member variables and
 * index number of datasets that need to be operated by sysfs file
 * @attr:       The device attribute
 * @mem:        The member in the dataset data structure
 * @idx:        The index number of the array data
 */
struct tpdm_dataset_attribute {
	struct device_attribute attr;
	enum dataset_mem mem;
	u32 idx;
};

static bool tpdm_has_dsb_dataset(struct tpdm_drvdata *drvdata)
{
	return (drvdata->datasets & TPDM_PIDR0_DS_DSB);
}

static bool tpdm_has_cmb_dataset(struct tpdm_drvdata *drvdata)
{
	return (drvdata->datasets & TPDM_PIDR0_DS_CMB);
}
#endif  /* _CORESIGHT_CORESIGHT_TPDM_H */
