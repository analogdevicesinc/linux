/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _DFS_S32V234_H
#define _DFS_S32V234_H

/* DFS Control Register (DFS_CTRL) */
#define DFS_CTRL(base)		((base) + 0x00000018)
#define DFS_CTRL_DLL_LOLIE	(1 << 0)
#define DFS_CTRL_DLL_RESET	(1 << 1)

/* DFS Port Status (DFS_PORTSR) */
#define DFS_PORTSR(base)	((base) + 0x0000000C)
#define DFS_PORTSR_MASK		(0x0000000F)
#define DFS_PORTSR_OFFSET	(28)

/* DFS Port Reset Register (DFS_PORTRESET) */
#define DFS_PORTRESET(base)	((base) + 0x00000014)
#define DFS_PORTRESET_PORTRESET_SET(val) \
		(DFS_PORTRESET_PORTRESET_MASK | \
		(((val) & DFS_PORTRESET_PORTRESET_MAXVAL) \
		<< DFS_PORTRESET_PORTRESET_OFFSET))
#define DFS_PORTRESET_PORTRESET_MAXVAL		(0xF)
#define DFS_PORTRESET_PORTRESET_MASK		(0x0000000F)
#define DFS_PORTRESET_PORTRESET_OFFSET		(28)

/* DFS Divide Register Portn (DFS_DVPORTn) */
#define DFS_DVPORTn(base, n)		((base) + (0x0000001C + \
					((n) * sizeof(u32))))
#define DFS_DVPORTn_MFI_SET(val)	(DFS_DVPORTn_MFI_MASK & \
					(((val) & DFS_DVPORTn_MFI_MAXVAL) \
					<< DFS_DVPORTn_MFI_OFFSET))
#define DFS_DVPORTn_MFN_SET(val)	(DFS_DVPORTn_MFN_MASK & \
					(((val) & DFS_DVPORTn_MFN_MAXVAL) \
					<< DFS_DVPORTn_MFN_OFFSET))
#define DFS_DVPORTn_MFI_MASK		(0x0000FF00)
#define DFS_DVPORTn_MFN_MASK		(0x000000FF)
#define DFS_DVPORTn_MFI_MAXVAL		(0xFF)
#define DFS_DVPORTn_MFN_MAXVAL		(0xFF)
#define DFS_DVPORTn_MFI_OFFSET		(8)
#define DFS_DVPORTn_MFN_OFFSET		(0)
#define DFS_MAXNUMBER			(4)

/*
 * Naming convention for PLL:
 * ARMPLL - PLL0
 * PERIPHPLL - PLL1
 * ENETPLL - PLL2
 * DDRPLL - PLL3
 * VIDEOPLL - PLL4
 */

/* The max values for PLL DFS is in Hz */
/* ARMPLL */
#define ARMPLL_DFS0_MAX_RATE		(266000000)
#define ARMPLL_DFS1_MAX_RATE		(600000000)
#define ARMPLL_DFS2_MAX_RATE		(600000000)
/* ENETPLL */
#define ENETPLL_DFS0_MAX_RATE		(350000000)
#define ENETPLL_DFS1_MAX_RATE		(350000000)
#define ENETPLL_DFS2_MAX_RATE		(416000000)
#define ENETPLL_DFS3_MAX_RATE		(104000000)
/* DDRPLL */
#define DDRPLL_DFS0_MAX_RATE		(500000000)
#define DDRPLL_DFS1_MAX_RATE		(500000000)
#define DDRPLL_DFS2_MAX_RATE		(350000000)

#define ARMPLL_DFS_NR			(3)
#define ENETPLL_DFS_NR			(4)
#define DDRPLL_DFS_NR			(3)

#endif
