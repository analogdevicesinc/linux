/*
 * Samsung MIPI DSIM PLL_1432X
 *
 * Copyright 2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SEC_DSIM_PLL_1432X_H__
#define __SEC_DSIM_PLL_1432X_H__

#include <drm/bridge/sec_mipi_dsim.h>
/*
 * DSIM PLL_1432X setting guide from spec:
 *
 * Fout(bitclk) = ((m + k / 65536) * Fin) / (p * 2^s), and
 * p = P[5:0], m = M[9:0], s = S[2:0], k = K[15:0];
 *
 * Fpref = Fin / p
 * Fin: [6MHz ~ 300MHz], Fpref: [2MHz ~ 30MHz]
 *
 * Fvco = ((m + k / 65536) * Fin) / p
 * Fvco: [1050MHz ~ 2100MHz]
 *
 * 1 <= P[5:0] <= 63, 64 <= M[9:0] <= 1023,
 * 0 <= S[2:0] <=  5, -32768 <= K[15:0] <= 32767
 *
 */

const struct sec_mipi_dsim_pll pll_1432x = {
	.p	= { .min = 1,		.max = 63,	},
	.m	= { .min = 64,		.max = 1023,	},
	.s	= { .min = 0,		.max = 5,	},
	.k	= { .min = 0,		.max = 32768,	},	/* abs(k) */
	.fin	= { .min = 6000,	.max = 300000,	},	/* in KHz */
	.fpref	= { .min = 2000,	.max = 30000,	},	/* in KHz */
	.fvco	= { .min = 1050000,	.max = 2100000,	},	/* in KHz */
};

#endif

