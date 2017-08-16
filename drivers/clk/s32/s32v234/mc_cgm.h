/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2017 NXP
 */
#ifndef _MC_CGM_H
#define _MC_CGM_H

#define ARMPLL_PLLDIG(mc_cgm)		(mc_cgm)
#define ARMPLL_PLLDIG_DFS(mc_cgm)	((mc_cgm) + 0x40)
#define ARMPLL_PLLDIG_PLLDV_MFD		(50)
#define ARMPLL_PLLDIG_PLLDV_MFN		(0)
#define ARMPLL_PLLDIG_PLLDV_RFDPHI0	(1)
#define ARMPLL_PLLDIG_PLLDV_RFDPHI1	(1)
#define ARMPLL_PLLDIG_DFS0_MFN		(195)
#define ARMPLL_PLLDIG_DFS1_MFN		(171)
#define ARMPLL_PLLDIG_DFS2_MFN		(171)

#define PERIPHPLL_PLLDIG(mc_cgm)	((mc_cgm) + 0x80)
#define PERIPHPLL_PLLDIG_PLLDV_MFD	(30)
#define PERIPHPLL_PLLDIG_PLLDV_MFN	(0)
#define PERIPHPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define PERIPHPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)

#define ENETPLL_PLLDIG(mc_cgm)		((mc_cgm) + 0x100)
#define ENETPLL_PLLDIG_DFS(mc_cgm)	((mc_cgm) + 0x100 + 0x40)
#define ENETPLL_PLLDIG_PLLDV_MFD	(50)
#define ENETPLL_PLLDIG_PLLDV_MFN	(0)
#define ENETPLL_PLLDIG_PLLDV_RFDPHI0	(0x1)
#define ENETPLL_PLLDIG_PLLDV_RFDPHI1	(0x1)
#define ENETPLL_PLLDIG_DFS0_MFN		(220)
#define ENETPLL_PLLDIG_DFS1_MFN		(220)
#define ENETPLL_PLLDIG_DFS2_MFN		(33)
#define ENETPLL_PLLDIG_DFS3_MFN		(1)

/* MC_CGM_SC_SS */
#define CGM_SC_SS(mc_cgm)		(((mc_cgm) + 0x7E4))

/* MC_CGM_SC_DCn */
#define CGM_SC_DCn(mc_cgm, dc)		(((mc_cgm) + 0x7E8) + ((dc) * 0x4))

#define MC_CGM_SC_DCn_PREDIV_OFFSET	(16)
#define MC_CGM_SC_DCn_PREDIV_SIZE	(3)
#define MC_CGM_SC_DCn_DE		(1 << 31)
#define MC_CGM_SC_SEL_OFFSET		(24)
#define MC_CGM_SC_SEL_SIZE		(4)

/* MC_CGM_ACn_DCm */
#define CGM_ACn_DCm(mc_cgm, ac, dc)	(((mc_cgm) + 0x808) + ((ac) * 0x20)\
					+ ((dc) * 0x4))

#define MC_CGM_ACn_DCm_PREDIV(val)	(MC_CGM_ACn_DCm_PREDIV_MASK & \
					((val) \
					<< MC_CGM_ACn_DCm_PREDIV_OFFSET))
#define MC_CGM_ACn_DCm_PREDIV_MASK	(0x001F0000)
#define MC_CGM_ACn_DCm_PREDIV_OFFSET	(16)
#define MC_CGM_ACn_DCm_PREDIV_SIZE	(5)
#define MC_CGM_ACn_DCm_DE		(1 << 31)

/* MC_CGM_ACn_SC/MC_CGM_ACn_SS */
#define CGM_ACn_SC(mc_cgm, ac)		(((mc_cgm) + 0x800) + ((ac) * 0x20))
#define CGM_ACn_SS(mc_cgm, ac)		(((mc_cgm) + 0x804) + ((ac) * 0x24))
#define MC_CGM_ACn_SEL_MASK		(0x07000000)
#define MC_CGM_ACn_SEL_SET(source)	(MC_CGM_ACn_SEL_MASK & \
					(((source) & 0x7) \
					<< MC_CGM_ACn_SEL_OFFSET))
#define MC_CGM_ACn_SEL_OFFSET		(24)
#define MC_CGM_ACn_SEL_SIZE		(4)

#endif
