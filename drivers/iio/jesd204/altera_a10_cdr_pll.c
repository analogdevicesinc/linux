/*
 * Altera CDR/CMU PLL dynamic reconfiguration driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

static unsigned int altera_a10_cdr_lookup_vco_speed(unsigned int fvco)
{
	if (fvco < 7000000)
		return 3;
	else if (fvco < 12500000)
		return 2;
	else
		return 0;
}

static unsigned int altera_a10_cdr_lookup_vco_speed_fix(unsigned int fvco)
{
	if (fvco < 6060000)
		return 60;
	else if (fvco < 7060000)
		return 74;
	else if (fvco < 9060000)
		return 90;
	else if (fvco < 11350000)
		return 116;
	else if (fvco < 12760000)
		return 150;
	else if (fvco < 13460000)
		return 159;
	else
		return 174;
}

static unsigned int altera_a10_cdr_lookup_cp_current_pd(unsigned int fvco,
	unsigned int lpd)
{
	if (lpd == 1 && fvco < 12000000)
		return 3;
	else
		return 4;
}

static unsigned int altera_a10_cdr_lookup_clklow_div(unsigned int fref)
{
	if (fref > 400000)
		return 1;
	else
		return 0;
}

static unsigned int altera_a10_cdr_lookup_fastlock(unsigned int lpd)
{
	if (lpd == 16)
		return 1;
	else
		return 0;
}

static unsigned int altera_a10_cdr_lookup_lf_resistor_pd(unsigned int fvco,
	unsigned int lpd)
{
	if (fvco < 7000000) {
		switch (lpd) {
		case 1:
			return 2;
		case 2:
		case 4:
			return 3;
		default:
			return 0;
		}
	} else {
		switch (lpd) {
		case 1:
		case 2:
		case 4:
			return 3;
		default:
			return 0;
		}
	}
}

static unsigned int altera_a10_cdr_lookup_lf_resistor_pfd(unsigned int fvco,
	unsigned int m, unsigned int lpfd) {
	if (fvco < 7000000) {
		switch (m*lpfd) {
		case 8 ... 45:
			return 2;
		default:
			return 3;
		}
	} else if (fvco < 9000000) {
		switch (m*lpfd) {
		case 8 ... 57:
			return 2;
		case 58 ... 71:
			return 3;
		case 72 ... 85:
			return 2;
		default:
			return 3;
		}
	} else if (fvco < 12000000) {
		switch (m*lpfd) {
		case 8 ... 27:
			return 1;
		case 28 ... 71:
			return 2;
		case 72 ... 85:
			return 3;
		case 86 ... 129:
			return 2;
		default:
			return 3;
		}
	} else {
		switch (m*lpfd) {
		case 8 ... 27:
			return 1;
		case 28 ... 57:
			return 2;
		default:
			return 3;
		}
	}
}

static unsigned int altera_a10_cdr_lookup_cp_current_pfd(unsigned int fvco,
	unsigned int m, unsigned int lpfd)
{
	if (fvco < 7000000) {
		switch (m*lpfd) {
		case 0 ... 35:
			return 2;
		case 36 ... 45:
			return 3;
		case 46 ... 71:
			return 1;
		case 72 ... 111:
			return 2;
		default:
			return 3;
		}
	} else if (fvco < 9000000) {
		switch (m*lpfd) {
		case 0 ... 27:
			return 1;
		case 28 ... 45:
			return 2;
		case 46 ... 57:
			return 3;
		case 58 ... 71:
			return 1;
		case 72 ... 85:
			return 4;
		case 86 ... 129:
			return 2;
		case 130 ... 141:
			return 3;
		case 142 ... 181:
			return 2;
		default:
			return 3;
		}
	} else if (fvco < 12000000) {
		switch (m*lpfd) {
		case 0 ... 27:
			return 3;
		case 28 ... 35:
			return 1;
		case 36 ... 71:
			return 2;
		case 72 ... 85:
			return 1;
		case 86 ... 129:
			return 4;
		default:
			return 2;
		}
	} else {
		switch (m*lpfd) {
		case 0 ... 27:
			return 4;
		case 28 ... 35:
			return 1;
		case 36 ... 45:
			return 2;
		case 46 ... 57:
			return 3;
		case 58 ... 71:
			return 1;
		case 72 ... 95:
			return 2;
		default:
			return 3;
		}
	}
}

#define A10_CDR_PLL_PFD_MIN 50000    /*  50.00 Mhz */
#define A10_CDR_PLL_PFD_MAX 800000   /* 800.00 Mhz */
#define A10_CDR_PLL_VCO_MIN 4900000  /*   4.90 GHz */
#define A10_CDR_PLL_VCO_MAX 14150000 /*  14.15 GHz */

static void altera_a10_cdr_calc_params(unsigned long fref,
	unsigned long fout, unsigned int *best_n, unsigned int *best_m,
	unsigned int *best_lpfd, unsigned int *best_lpd,
	unsigned long *best_fvco)
{
	unsigned long m, m_min, m_max;
	unsigned long n, lpd, lpfd, lpfd_min;
	unsigned long fvco, target_fvco;
	unsigned long pfd;

	*best_n = *best_m = *best_lpfd = *best_lpd = *best_fvco = 0;


	fout /= 2;

	for (lpd = 1; lpd < 16; lpd *= 2) {
		if (fout >= A10_CDR_PLL_VCO_MIN / lpd)
			break;
	}

	target_fvco = fout * lpd;

	if (fout * lpd > 5200000)
		lpfd_min = 2;
	else
		lpfd_min = 1;

	m_min = max_t(unsigned long, DIV_ROUND_UP(A10_CDR_PLL_VCO_MIN / 2, fref), 8);
	m_max = min_t(unsigned long, A10_CDR_PLL_VCO_MAX * 8 / lpfd_min / fref, 127);

	for (n = 1; n <= 8; n *= 2) {
		pfd = fref / n;
		if (pfd < A10_CDR_PLL_PFD_MIN)
			break;
		if (pfd > A10_CDR_PLL_PFD_MAX)
			continue;

		for (lpfd = lpfd_min; lpfd <= 2; lpfd++) {
			for (m = m_min; m <= m_max; m++) {
				fvco = fref * m * lpfd / n;

				if (abs(fvco - target_fvco) < abs(*best_fvco - target_fvco)) {
					*best_n = n;
					*best_m = m;
					*best_lpfd = lpfd;
					*best_fvco = fvco;
					*best_lpd = lpd;
					if (target_fvco == fvco)
						return;
				}
			}
		}
	}
}

static long altera_a10_cdr_pll_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	unsigned int n, m, lpfd, lpd;
	unsigned long fvco;
	unsigned long long tmp;

	altera_a10_cdr_calc_params(*parent_rate / 1000, rate, &n, &m, &lpfd,
		&lpd, &fvco);

	if (n == 0 || m == 0 || lpfd == 0 || lpd == 0)
		return -EINVAL;

	tmp = (unsigned long long)*parent_rate * m * lpfd * 2;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, 1000 * n * lpd);

	return min_t(unsigned long long, tmp, LONG_MAX);
}

static int altera_a10_cdr_pll_set_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long parent_rate)
{
	struct adxcvr_state *st = clk_hw_to_adxcvr(clk_hw);
	unsigned int n, m, lpfd, lpd;
	unsigned long fvco;
	unsigned int i;
	unsigned int vco_speed, vco_speed_fix;
	unsigned int cp_current_pd, lfr_pd;
	unsigned int cp_current_pfd, lfr_pfd;
	unsigned int clkdiv_low;
	unsigned int fast_lock;

	parent_rate /= 1000;
	altera_a10_cdr_calc_params(parent_rate, rate, &n, &m, &lpfd, &lpd,
		&fvco);

	if (n == 0 || m == 0 || lpfd == 0 || lpd == 0)
		return -EINVAL;

	switch (n) {
	case 1:
		n = 0;
		break;
	case 2:
		n = 1;
		break;
	case 4:
		n = 2;
		break;
	default:
		n = 3;
		break;
	}

	switch (lpd) {
	case 1:
		lpd = 1;
		break;
	case 2:
		lpd = 3;
		break;
	case 4:
		lpd = 4;
		break;
	case 8:
		lpd = 5;
		break;
	default:
		lpd = 6;
		break;
	}

	switch (lpfd) {
	case 1:
		lpfd = 2;
		break;
	default:
		lpfd = 3;
		break;
	}

	vco_speed = altera_a10_cdr_lookup_vco_speed(fvco);
	vco_speed_fix = altera_a10_cdr_lookup_vco_speed_fix(fvco);
	clkdiv_low = altera_a10_cdr_lookup_clklow_div(parent_rate);
	fast_lock = altera_a10_cdr_lookup_fastlock(lpd);
	cp_current_pd = altera_a10_cdr_lookup_cp_current_pd(fvco, lpd);
	lfr_pd = altera_a10_cdr_lookup_lf_resistor_pd(fvco, lpd);
	cp_current_pfd = altera_a10_cdr_lookup_cp_current_pfd(fvco, m, lpfd);
	lfr_pfd = altera_a10_cdr_lookup_lf_resistor_pfd(fvco, m, lpfd);

	adxcvr_pre_lane_rate_change(st);

	for (i = 0; i < st->lanes_per_link; i++) {
		adxcfg_acquire_arbitration(st, i);

		adxcfg_update(st, i, 0x132, 0x05, ((vco_speed_fix & 0x80) >> 7) | ((vco_speed_fix & 0x40) >> 4));
		adxcfg_update(st, i, 0x133, 0xe0, cp_current_pd << 5);
		adxcfg_update(st, i, 0x134, 0x40, (vco_speed_fix & 0x10) << 2);
		adxcfg_update(st, i, 0x135, 0x4f, (vco_speed_fix & 0x20) << 1 | (lfr_pd << 2) | lfr_pfd);
		adxcfg_update(st, i, 0x136, 0x0f, vco_speed_fix & 0xf);
		adxcfg_update(st, i, 0x137, 0x7c, vco_speed << 2);
		adxcfg_update(st, i, 0x139, 0xbf, (fast_lock << 7) | (cp_current_pd << 3) | cp_current_pfd);

		adxcfg_write(st, i, 0x13a, (clkdiv_low << 6) | (lpd << 3) | lpfd);
		adxcfg_write(st, i, 0x13b, m);
		adxcfg_update(st, i, 0x13c, 0x0c, n << 2);

		adxcfg_update(st, i, XCVR_REG_CALIB_PMA_EN,
			XCVR_CALIB_CMU_CDR_PLL_EN_MASK,
			XCVR_CALIB_CMU_CDR_PLL_EN);

		adxcfg_update(st, i, XCVR_REG_RATE_SWITCH_FLAG,
			XCVR_RATE_SWITCH_FLAG_MASK,
			XCVR_RATE_SWITCH_FLAG_RATE_SWITCH);

		/*
		 * Disable tx_cal_busy and enable rx_cal_busy output through
		 * capability register.
		 */
		adxcfg_update(st, i, XCVR_REG_CAPAB_PMA,
			XCVR_CAPAB_RX_CAL_BUSY_EN_MASK |
			XCVR_CAPAB_TX_CAL_BUSY_EN_MASK,
			XCVR_CAPAB_RX_CAL_BUSY_EN |
			XCVR_CAPAB_TX_CAL_BUSY_DIS);

		adxcfg_release_arbitration(st, i, true);

		adxcfg_calibration_check(st, i, false);
	}

	adxcvr_post_lane_rate_change(st, rate);

	st->initial_recalc = false;

	return 0;
}

static unsigned long altera_a10_cdr_pll_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct adxcvr_state *st = clk_hw_to_adxcvr(clk_hw);
	unsigned int m, n, lpd, lpfd;
	unsigned long long tmp;
	unsigned int div0, div1;

	adxcfg_lock(st);
	adxcfg_acquire_arbitration(st, 0);

	div0 = adxcfg_read(st, 0, 0x13a);
	m = adxcfg_read(st, 0, 0x13b);
	div1 = adxcfg_read(st, 0, 0x13c);

	adxcfg_release_arbitration(st, 0, false);
	adxcfg_unlock(st);

	switch ((div0 >> 3) & 0x7) {
	case 1:
		lpd = 1;
		break;
	case 3:
		lpd = 2;
		break;
	case 4:
		lpd = 4;
		break;
	case 5:
		lpd = 8;
		break;
	default:
		lpd = 16;
		break;
	}

	switch (div0 & 0x7) {
	case 2:
		lpfd = 1;
		break;
	default:
		lpfd = 2;
		break;
	}

	n = 1 << ((div1 >> 2) & 0x3);

	tmp = (unsigned long long)parent_rate * m * lpfd * 2;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, n * lpd * 1000);

	if (tmp != 0 && st->initial_recalc)
		altera_a10_cdr_pll_set_rate(clk_hw, tmp, parent_rate);

	return min_t(unsigned long long, tmp, ULONG_MAX);
}

static const struct clk_ops adxcvr_cdr_pll_ops = {
	.recalc_rate = altera_a10_cdr_pll_recalc_rate,
	.round_rate = altera_a10_cdr_pll_round_rate,
	.set_rate = altera_a10_cdr_pll_set_rate,
};
