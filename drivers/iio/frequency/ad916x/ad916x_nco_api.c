#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"

#define AD916x_MAX_NUM_NCO 1

static int ad916x_nco_set_configure_main(ad916x_handle_t *h,
								const int64_t carrier_freq_hz,
								const uint16_t amplitude,
								const int dc_test_en)
{
	uint64_t tmp_freq;
	int err;
	/* check if the desired frequency power of 2 */
	uint8_t is_pow2 = 0;

	if (carrier_freq_hz == 0)
		return API_ERROR_INVALID_PARAM;

	tmp_freq = carrier_freq_hz;
	while (tmp_freq <= h->dac_freq_hz) {
		if ((tmp_freq) == h->dac_freq_hz) {
			/* It is power of 2 */
			is_pow2 = 1;
			break;
		}
		tmp_freq *= 2;
	}

	if (is_pow2 == 1) {
		/* Integer NCO mode */
		/* As we are in Integer NCO mode it guranteed the
		   value is integer power of 2 */
		tmp_freq = DIV64_U64(h->dac_freq_hz, carrier_freq_hz);
		tmp_freq = DIV64_U64(ADI_POW2_48, tmp_freq);
		/* Disable modulus */
		err = ad916x_nco_set_enable(h, 0, 1);
		if (err != API_ERROR_OK) {
			return err;
		}
		if(dc_test_en == 0) {
			/* DC Test Disable */
			err = ad916x_dc_test_set_mode(h, amplitude, 0);
			if (err != API_ERROR_OK) {
				return err;
			}
		} else {
			/* Set interpolation mode - bypas */
			err = ad916x_register_write(h, AD916x_REG_LANE_INTPL_MODE,
											AD916x_JESD_LANES(8));
			if (err != API_ERROR_OK) {
				return err;
			}
			/* DC Test Enable */
			err = ad916x_dc_test_set_mode(h, amplitude, 1);
			if (err != API_ERROR_OK) {
				return err;
			}
		}
		/* Write FTW */
		err = ad916x_nco_set_ftw(h, 0, tmp_freq, 0, 0);
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		int gcd;
		uint64_t int_part;
		uint64_t frac_part_a;
		uint64_t frac_part_b;
		uint64_t M, N;
		uint64_t tmp_ah, tmp_al, tmp_bh, tmp_bl, tmp_fh, tmp_fl;
		/* Modulus NCO mode */

		gcd = adi_api_utils_gcd(carrier_freq_hz, h->dac_freq_hz);
		M = DIV64_U64(carrier_freq_hz, gcd);
		N = DIV64_U64(h->dac_freq_hz, gcd);

		if (M > INT16_MAX) {
			uint64_t mask = U64MSB;
			int i = 0;
			while (((mask & M) == 0) && (mask != 1)) {
				mask >>= 1;
				i++;
			}
			int_part = DIV64_U64(M*((uint64_t)1u << i), N);
			int_part *= ((uint64_t)1u << (48 - i));
		} else {
			int_part = DIV64_U64(M*(ADI_POW2_48), N);
		}

		adi_api_utils_mult_128(M, ADI_POW2_48, &tmp_ah, &tmp_al);
		adi_api_utils_mult_128(N, int_part, &tmp_bh, &tmp_bl);
		adi_api_utils_subt_128(tmp_ah, tmp_al, tmp_bh, tmp_bl, &tmp_fh, &tmp_fl);
		frac_part_a = tmp_fl;
		frac_part_b = N;

		gcd = adi_api_utils_gcd(frac_part_a, frac_part_b);
		frac_part_a = DIV64_U64(frac_part_a, gcd);
		frac_part_b = DIV64_U64(frac_part_b, gcd);

		if ((frac_part_a > ADI_MAXUINT48) || (frac_part_b > ADI_MAXUINT48)) {
			/* TODO: a better error */
			return API_ERROR_INVALID_PARAM;
		}
		/* Enable modulus */
		err = ad916x_nco_set_enable(h, 1, 1);
		if (err != API_ERROR_OK) {
			return err;
		}

		if(dc_test_en == 0) {
			/* DC Test Disable */
			err = ad916x_dc_test_set_mode(h, amplitude, 0);
			if (err != API_ERROR_OK) {
				return err;
			}
		} else {
			/* Set interpolation mode - bypas */
			err = ad916x_register_write(h, AD916x_REG_LANE_INTPL_MODE,
											AD916x_JESD_LANES(8));
			if (err != API_ERROR_OK) {
				return err;
			}
			/* DC Test Enable */
			err = ad916x_dc_test_set_mode(h, amplitude, 1);
			if (err != API_ERROR_OK) {
				return err;
			}
		}
		/* Write FTW, A and B */
		err = ad916x_nco_set_ftw(h, 0, int_part, frac_part_a, frac_part_b);
		if (err != API_ERROR_OK) {
			return err;
		}
	}
	return API_ERROR_OK;
}


static int ad916x_nco_calc_freq_int_main(ad916x_handle_t *h, uint64_t int_part,
										int64_t *carrier_freq_hz)
{
	uint64_t tmpa_lo, tmpa_hi;
	adi_api_utils_mult_128(int_part, h->dac_freq_hz, &tmpa_hi, &tmpa_lo);
	adi_api_utils_div_128(tmpa_hi, tmpa_lo, 0, ADI_POW2_48, &tmpa_hi, &tmpa_lo);
	*carrier_freq_hz = tmpa_lo;
	return API_ERROR_OK;
}

static int ad916x_nco_calc_freq_fract_main(ad916x_handle_t *h,
							uint64_t int_part, uint64_t frac_part_a,
							uint64_t frac_part_b, int64_t *carrier_freq_hz)
{
	uint64_t tmpa_lo, tmpa_hi;
	uint64_t tmpb_lo, tmpb_hi;
	adi_api_utils_mult_128(int_part, h->dac_freq_hz, &tmpa_hi, &tmpa_lo);
	adi_api_utils_mult_128(frac_part_a, h->dac_freq_hz, &tmpb_hi, &tmpb_lo);
	adi_api_utils_div_128(tmpb_hi, tmpb_lo, 0, frac_part_b, &tmpb_hi, &tmpb_lo);
	adi_api_utils_add_128(tmpa_hi, tmpa_lo, tmpb_hi, tmpb_lo, &tmpa_hi, &tmpa_lo);
	adi_api_utils_div_128(tmpa_hi, tmpa_lo, 0, ADI_POW2_48, &tmpa_hi, &tmpa_lo);
	*carrier_freq_hz = tmpa_lo;
	return API_ERROR_OK;
}


static int ad916x_nco_get_configure_main(ad916x_handle_t *h,
				int64_t *carrier_freq_hz, uint16_t *amplitude, int *dc_test_en)
{
	uint8_t tmp_reg;
	int err;
	/* Get amplitude */
	err = ad916x_dc_test_get_mode(h, amplitude, dc_test_en);
	if (err != API_ERROR_OK) {
		return err;
	}
	/* Check if modulus is used */
	err = ad916x_register_read(h, AD916x_REG_DATAPATH_CFG, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if(tmp_reg & BIT(2)) {
		/* modulus enabled */
		uint64_t int_part;
		uint64_t frac_part_a;
		uint64_t frac_part_b;
		err = ad916x_nco_get_ftw(h, 0, &int_part, &frac_part_a, &frac_part_b);
		if (err != API_ERROR_OK) {
			return err;
		}
		if((frac_part_a == 0) || (frac_part_b == 0)) {
			/* Division by 0 = Modulus is off. */
			err = ad916x_nco_calc_freq_int_main(h, int_part, carrier_freq_hz);
			if (err != API_ERROR_OK) {
				return err;
			}
			return API_ERROR_OK;
		}
		err = ad916x_nco_calc_freq_fract_main(h, int_part, frac_part_a,
											frac_part_b, carrier_freq_hz);
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		/* No modulus used */
		/* Get FTW */
		uint64_t int_part;
		err = ad916x_nco_get_ftw(h, 0, &int_part,
									INVALID_POINTER, INVALID_POINTER);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_nco_calc_freq_int_main(h, int_part, carrier_freq_hz);
		if (err != API_ERROR_OK) {
			return err;
		}
	}
	return API_ERROR_OK;
}


ADI_API int ad916x_nco_set_phase_offset(ad916x_handle_t *h, const uint16_t po)
{
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	err = ad916x_register_write(h, AD916x_REG_PHASE_OFFSET0, ADI_GET_BYTE(po, 0));
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_write(h, AD916x_REG_PHASE_OFFSET1, ADI_GET_BYTE(po, 8));
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_nco_get_phase_offset(ad916x_handle_t *h, uint16_t *po)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	err = ad916x_register_read(h, AD916x_REG_PHASE_OFFSET1, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*po = tmp_reg;
	*po <<= 8;
	err = ad916x_register_read(h, AD916x_REG_PHASE_OFFSET0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*po |= tmp_reg;
	return API_ERROR_OK;
}

ADI_API int ad916x_nco_set_enable(ad916x_handle_t *h,
									const int modulus_en, const int nco_en)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	err = ad916x_register_read(h, AD916x_REG_DATAPATH_CFG, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (nco_en == 0) {
		tmp_reg &= ~BIT(6);
	} else {
		tmp_reg |= BIT(6);
	}
	if (modulus_en == 0) {
		tmp_reg &= ~BIT(2);
	} else {
		tmp_reg |= BIT(2);
	}
	err = ad916x_register_write(h, AD916x_REG_DATAPATH_CFG, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_nco_get_enable(ad916x_handle_t *h,
									int *modulus_en, int *nco_en)
{
	uint8_t tmp_reg;
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	err = ad916x_register_read(h, AD916x_REG_DATAPATH_CFG, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*nco_en = 1;
	if((tmp_reg & BIT(6)) == 0) {
		*nco_en = 0;
	}
	*modulus_en = 1;
	if ((tmp_reg & BIT(2)) == 0) {
		*modulus_en = 0;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_nco_set_ftw(ad916x_handle_t *h, const unsigned int nco_nr,
								const uint64_t ftw, const uint64_t acc_modulus,
								const uint64_t acc_delta)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (nco_nr == 0) {
		/* Set FTW and MOD for the main NCO */
		err = ad916x_register_read(h, AD916x_REG_FTW_UPDATE, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= 0xFE;
		err = ad916x_register_write(h, AD916x_REG_FTW_UPDATE, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW0, ADI_GET_BYTE(ftw, 0));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW1, ADI_GET_BYTE(ftw, 8));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW2, ADI_GET_BYTE(ftw, 16));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW3, ADI_GET_BYTE(ftw, 24));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW4, ADI_GET_BYTE(ftw, 32));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_FTW5, ADI_GET_BYTE(ftw, 40));
		if (err != API_ERROR_OK) {
			return err;
		}
		if((acc_modulus != 0) && (acc_delta != 0)) {
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA0,
											ADI_GET_BYTE(acc_delta, 0));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA1,
											ADI_GET_BYTE(acc_delta, 8));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA2,
											ADI_GET_BYTE(acc_delta, 16));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA3,
											ADI_GET_BYTE(acc_delta, 24));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA4,
											ADI_GET_BYTE(acc_delta, 32));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_DELTA5,
											ADI_GET_BYTE(acc_delta, 40));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD0,
											ADI_GET_BYTE(acc_modulus, 0));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD1,
											ADI_GET_BYTE(acc_modulus, 8));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD2,
											ADI_GET_BYTE(acc_modulus, 16));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD3,
											ADI_GET_BYTE(acc_modulus, 24));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD4,
											ADI_GET_BYTE(acc_modulus, 32));
			if (err != API_ERROR_OK) {
				return err;
			}
			err = ad916x_register_write(h, AD916x_REG_ACC_MOD5,
											ADI_GET_BYTE(acc_modulus, 40));
			if (err != API_ERROR_OK) {
				return err;
			}
		}
		/* FTW_LOAD_REQ (rising edge )*/
		tmp_reg |= 0x01;
		err = ad916x_register_write(h, AD916x_REG_FTW_UPDATE, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		/* FTW_LOAD_ACK check */
		err = ad916x_register_read(h, AD916x_REG_FTW_UPDATE, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		if((tmp_reg & 0x02) != 0) {
			return API_ERROR_OK;
		}
		return API_ERROR_FTW_LOAD_ACK;
	} else if (nco_nr < 32) {
		/* Get the FTW for NCO 1..31 */
		err = ad916x_register_write(h, AD916x_REG_HOPF_FTWX_0(nco_nr),
										ADI_GET_BYTE(ftw, 0));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_HOPF_FTWX_1(nco_nr),
										ADI_GET_BYTE(ftw, 8));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_HOPF_FTWX_2(nco_nr),
										ADI_GET_BYTE(ftw, 16));
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_write(h, AD916x_REG_HOPF_FTWX_3(nco_nr),
										ADI_GET_BYTE(ftw, 24));
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		return API_ERROR_INVALID_PARAM;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_nco_get_ftw(ad916x_handle_t *h, const unsigned int nco_nr,
								uint64_t *ftw, uint64_t *acc_modulus,
								uint64_t *acc_delta)
{
	uint8_t tmp_reg;
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (ftw == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}
		
	if (nco_nr == 0){
		/* Get FTW for the main NCO */
		err = ad916x_register_read(h, AD916x_REG_FTW5, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw = tmp_reg;
		(*ftw) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_FTW4, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw |= tmp_reg;
		(*ftw) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_FTW3, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw |= tmp_reg;
		(*ftw) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_FTW2, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw |= tmp_reg;
		(*ftw) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_FTW1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw |= tmp_reg;
		(*ftw) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_FTW0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*ftw |= tmp_reg;
	} else {
		return API_ERROR_INVALID_PARAM;
	}

	/* Get the MODULUS if needed */
	if ((acc_modulus != INVALID_POINTER) && (acc_delta != INVALID_POINTER)) {
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA5, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta = tmp_reg;
		(*acc_delta) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA4, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta |= tmp_reg;
		(*acc_delta) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA3, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta |= tmp_reg;
		(*acc_delta) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA2, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta |= tmp_reg;
		(*acc_delta) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta |= tmp_reg;
		(*acc_delta) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_DELTA0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_delta |= tmp_reg;
			
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD5, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus = tmp_reg;
		(*acc_modulus) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD4, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus |= tmp_reg;
		(*acc_modulus) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD3, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus |= tmp_reg;
		(*acc_modulus) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD2, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus |= tmp_reg;
		(*acc_modulus) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus |= tmp_reg;
		(*acc_modulus) <<= 8;
		err = ad916x_register_read(h, AD916x_REG_ACC_MOD0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*acc_modulus |= tmp_reg;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_dc_test_set_mode(ad916x_handle_t *h,
								const uint16_t test_data, const int en)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	err = ad916x_register_read(h, AD916x_REG_PROD_ID_LSB, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	switch(tmp_reg) {
		case AD9162_PROD_ID_LSB:
		case AD9166_PROD_ID_LSB:
			break;
		case AD9163_PROD_ID_LSB:
		case AD9161_PROD_ID_LSB:
		default:
			return API_ERROR_NOT_SUPPORTED;
	}

	err = ad916x_register_read(h, AD916x_REG_DIG_TEST, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (en == 0) {
		tmp_reg &= ~BIT(1);
	} else {
		tmp_reg |= BIT(1);
	}
	err = ad916x_register_write(h, AD916x_REG_DIG_TEST, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_write(h, AD916x_REG_TEST_DC_DATA1,
									ADI_GET_BYTE(test_data , 8));
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_write(h, AD916x_REG_TEST_DC_DATA0,
									ADI_GET_BYTE(test_data , 0));
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_dc_test_get_mode(ad916x_handle_t *h,
										uint16_t *test_data, int *en)
{
	uint8_t tmp_reg;
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	err = ad916x_register_read(h, AD916x_REG_PROD_ID_LSB, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	switch(tmp_reg) {
		case AD9162_PROD_ID_LSB:
		case AD9166_PROD_ID_LSB:
			break;
		case AD9163_PROD_ID_LSB:
		case AD9161_PROD_ID_LSB:
		default:
			return API_ERROR_NOT_SUPPORTED;
	}
	err = ad916x_register_read(h, AD916x_REG_DIG_TEST, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*en = 0;
	if((tmp_reg & BIT(1)) != 0) {
		*en = 1;
	}
	err = ad916x_register_read(h, AD916x_REG_TEST_DC_DATA1, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*test_data = tmp_reg;
	(*test_data) <<= 8;
	err = ad916x_register_read(h, AD916x_REG_TEST_DC_DATA0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*test_data |= tmp_reg;
	return API_ERROR_OK;
}

/**
	amplitude - amplitude full scale. 0xFFFF = full scale.
*/
ADI_API int ad916x_nco_set(ad916x_handle_t *h, const unsigned int nco_nr,
							const int64_t carrier_freq_hz,
							const uint16_t amplitude, int dc_test_en)
{
	uint8_t tmp_reg;
	int err;

	/*todo: phase offset - as param */
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	/* check params */
	if (nco_nr > (AD916x_MAX_NUM_NCO-1)) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_DATAPATH_CFG, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if ((tmp_reg & 1) != 0) {
		/* FIR85 Enabled */
		/* TODO: Should we just check here, or add a parameter
			to force enable/disable FIR85. */
		if (!((carrier_freq_hz >= 0) && (carrier_freq_hz < h->dac_freq_hz))) {
			return API_ERROR_INVALID_PARAM;
		}
	} else {
		if (!((carrier_freq_hz >= (int64_t)(0ll-h->dac_freq_hz / 2)) &&
				(carrier_freq_hz < (h->dac_freq_hz / 2)))) {
			return API_ERROR_INVALID_PARAM;
		}
	}

	return ad916x_nco_set_configure_main(h, carrier_freq_hz, amplitude,
												dc_test_en);
}

ADI_API int ad916x_nco_get(ad916x_handle_t *h, const unsigned int nco_nr,
				int64_t *carrier_freq_hz, uint16_t *amplitude, int *dc_test_en)
{
	/*+ param nco nr - to 0 - 31 ; 0 -48bit nco; 1 - 31 -> 32bit nco*/
	/*todo: phase offset - as param */
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (nco_nr == 0) {
		return ad916x_nco_get_configure_main(h, carrier_freq_hz, amplitude,
												dc_test_en);
	}

	return API_ERROR_INVALID_PARAM;
}

ADI_API int ad916x_nco_reset(ad916x_handle_t *h)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_OK;
	}
	err = ad916x_register_read(h, AD916x_REG_HOPF_CTRL, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg &= ~(3u << 6);
	tmp_reg |= BIT(6);
	err = ad916x_register_write(h, AD916x_REG_HOPF_CTRL, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}
