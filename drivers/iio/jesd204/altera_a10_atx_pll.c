/*
 * Altera ATX PLL dynamic reconfiguration driver
 *
 * Copyright 2017 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

static const unsigned int altera_a10_atx_bands[] = {
	14200000, 13650000, 13150000, 13110000, 12600000,
	11900000, 11550000, 11400000, 11100000, 10600000,
	10400000,  9750000,  9350000,  9100000,  8900000,
	 8800000,  8600000,  8200000,  7800000,  7400000,
};

static unsigned int altera_a10_atx_lookup_band(unsigned int fvco)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(altera_a10_atx_bands); i++) {
		if (fvco > altera_a10_atx_bands[i]) {
			i = i % 8;
			if (i != 7)
				i++;
			return i;
		}
	}

	return 5;
}

static unsigned int altera_a10_atx_lookup_tank(unsigned int fvco)
{
	if (fvco < 8800000)
		return 0;
	else if (fvco < 11400000)
		return 1;
	else
		return 2;
}

static unsigned int altera_a10_atx_lookup_cp_current(unsigned int m)
{
	switch (m) {
	case 0 ... 20:
		return 0x1a;
	case 21 ... 30:
		return 0x1c;
	case 31 ... 40:
		return 0x1d;
	case 41 ... 60:
		return 0x25;
	case 61 ... 70:
		return 0x26;
	case 71 ... 90:
		return 0x1d;
	case 91 ... 100:
		return 0x25;
	default:
		return 0x26;
	}
}

static unsigned int altera_a10_atx_lookup_lf_resistance(unsigned int m)
{
	switch (m) {
	case 0 ... 12:
		return 0;
	case 13 ... 20:
		return 1;
	case 21 ... 30:
		return 0;
	case 31 ... 70:
		return 1;
	default:
		return 2;
	}
}

#define A10_ATX_PLL_PFD_MIN 61440    /*  61.44 Mhz */
#define A10_ATX_PLL_PFD_MAX 800000   /* 800.00 Mhz */
#define A10_ATX_PLL_VCO_MIN 7200000  /*   7.20 GHz */
#define A10_ATX_PLL_VCO_MAX 14400000 /*  14.40 GHz */

static void altera_a10_atx_calc_params(unsigned long fref,
	unsigned long fout, unsigned int *best_n, unsigned int *best_m,
	unsigned int *best_l, unsigned long *best_fvco)
{
	unsigned long m, m_min, m_max;
	unsigned long n, l;
	unsigned long f, fvco, best_f;
	unsigned long pfd;

	*best_n = *best_m = *best_l = *best_fvco = 0;
	best_f = ULONG_MAX;

	m_min = max_t(unsigned long, DIV_ROUND_UP(A10_ATX_PLL_VCO_MIN / 2, fref), 8);
	m_max = min_t(unsigned long, A10_ATX_PLL_VCO_MAX / 2 * 8 / fref, 127);

	for (n = 1; n <= 8; n *= 2) {
		pfd = fref / n;
		if (pfd < A10_ATX_PLL_PFD_MIN)
			break;
		if (pfd > A10_ATX_PLL_VCO_MAX)
			continue;

		for (m = m_min; m <= m_max; m++) {
			fvco = fref * m * 2 / n;

			for (l = 1; l <= 16; l *= 2) {
				f = (fvco * 2) / l;

				if (abs(f - fout) < abs(best_f - fout)) {
					best_f = f;
					*best_n = n;
					*best_m = m;
					*best_l = l;
					*best_fvco = fvco;
					if (f == fout)
						return;
				}
			}
		}
	}
}

static long altera_a10_atx_pll_round_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long *parent_rate)
{
	unsigned int n, m, l;
	unsigned long fvco;
	unsigned long long tmp;

	altera_a10_atx_calc_params(*parent_rate / 1000, rate, &n, &m, &l, &fvco);

	if (n == 0 || m == 0 || l == 0)
		return -EINVAL;

	tmp = (unsigned long long)*parent_rate * m;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, l * n * 1000 / 4);

	return min_t(unsigned long long, tmp, LONG_MAX);
}

static int altera_a10_atx_pll_set_rate(struct clk_hw *clk_hw,
	unsigned long rate, unsigned long parent_rate)
{
	struct adxcvr_state *st = clk_hw_to_adxcvr(clk_hw);
	unsigned int n, m, l;
	unsigned long fvco;
	unsigned int lfr, cpc, band, tank;

	altera_a10_atx_calc_params(parent_rate / 1000, rate, &n, &m, &l, &fvco);

	if (n == 0 || m == 0 || l == 0)
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

	switch (l) {
	case 1:
		l = 0;
		break;
	case 2:
		l = 1;
		break;
	case 4:
		l = 2;
		break;
	case 8:
		l = 3;
		break;
	default:
		l = 4;
		break;
	}

	lfr = altera_a10_atx_lookup_lf_resistance(m);
	cpc = altera_a10_atx_lookup_cp_current(m);
	band = altera_a10_atx_lookup_band(fvco);
	tank = altera_a10_atx_lookup_tank(fvco);

	adxcvr_pre_lane_rate_change(st);
	atx_pll_acquire_arbitration(st);

	atx_pll_update(st, 0x102, 0x1f, band | (tank << 3));
	atx_pll_update(st, 0x104, 0x7f, ((cpc & 0x38) << 1) | 0x04 | lfr);
	atx_pll_update(st, 0x105, 0x07, cpc & 0x7);

	atx_pll_update(st, 0x107, 0x0c, n << 2);
	atx_pll_update(st, 0x108, 0x07, l);
	atx_pll_write(st, 0x109, m);

	atx_pll_update(st, XCVR_REG_CALIB_ATX_PLL_EN,
		XCVR_CALIB_ATX_PLL_EN_MASK, XCVR_CALIB_ATX_PLL_EN);
	atx_pll_release_arbitration(st, true);

	atx_pll_calibration_check(st);
	xcvr_calib_tx(st);
	adxcvr_post_lane_rate_change(st, rate);

	st->initial_recalc = false;

	return 0;
}

static unsigned long altera_a10_atx_pll_recalc_rate(struct clk_hw *clk_hw,
	unsigned long parent_rate)
{
	struct adxcvr_state *st = clk_hw_to_adxcvr(clk_hw);
	unsigned int m, n, l;
	unsigned long long tmp;
	unsigned int div0, div1;

	atx_pll_acquire_arbitration(st);

	div0 = atx_pll_read(st, 0x107);
	div1 = atx_pll_read(st, 0x108);
	m = atx_pll_read(st, 0x109);

	atx_pll_release_arbitration(st, false);

	n = 1 << ((div0 >> 2) & 0x3);
	l = 1 << (div1 & 0x7);

	tmp = (unsigned long long)parent_rate * m;
	tmp = DIV_ROUND_CLOSEST_ULL(tmp, l * n * 1000 / 4);

	if (tmp != 0 && st->initial_recalc)
		altera_a10_atx_pll_set_rate(clk_hw, tmp, parent_rate);

	return min_t(unsigned long long, tmp, ULONG_MAX);
}

static const struct clk_ops adxcvr_atx_pll_ops = {
	.recalc_rate = altera_a10_atx_pll_recalc_rate,
	.round_rate = altera_a10_atx_pll_round_rate,
	.set_rate = altera_a10_atx_pll_set_rate,
};
