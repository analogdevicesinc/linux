#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"

#define GET_LANE_MSK(lane) (0x1 << (lane))

#define LANE_MIN 1
#define LANE_MAX AD916x_JESD_NOF_LANES
#define K_MAX 32
#define M_DEFAULT 2
#define N_DEFAULT 16
#define NP_DEFAULT 16
#define S_MIN 1
#define S_MAX 4
#define CF_DEFAULT 0
#define CS_DEFAULT 0
#define HD_DEFAULT 0
#define LANE_RATE_MIN 750
#define LANE_RATE_MAX 12500
#define INTERPOLATION_MIN 0
#define INTERPOLATION_MAX 24
#define DAC_CLK_FREQ_MIN 850
#define DAC_CLK_FREQ_MAX 6000
#define SYSREF_JITTER_WIN_MAX 28
#define INTPL_MODE_INVALID    25

typedef enum {
	JESDMODE0 = 0,
	JESDMODE1,
	JESDMODE2,
	JESDMODE3,
	JESDMODE4,
	JESDMODE5,
	JESDMODE6,
	JESDMODEINVALID
} jesd_mode_t;

typedef struct {
	uint8_t L;
	uint8_t M;
	uint8_t F;
	uint8_t S;
	jesd_mode_t jesd_mode;
} jesd_if_mode_param_t;

typedef struct {
	uint8_t intpl;
	uint8_t intpl_cfg;
	jesd_mode_t jesd_mode_min;
	jesd_mode_t jesd_mode_max;
} jesd_intpl_mode_t;

/*DataSheet Table 15*/
jesd_if_mode_param_t SUPPORTED_JESD_MODES_TBL[] = {
	{8, 1, 1, 4, JESDMODE0},
	{8, 2, 1, 2, JESDMODE1},
	{6, 2, 2, 3, JESDMODE2},
	{4, 2, 1, 1, JESDMODE3},
	{3, 2, 4, 3, JESDMODE4},
	{2, 2, 2, 1, JESDMODE5},
	{1, 2, 4, 1, JESDMODE6}
};

/*DataSheet Table 25*/
jesd_intpl_mode_t SUPPORTED_JESD_INTPL_MODES_TBL[] = {
	{1, 0x0, JESDMODE0, JESDMODE0},
	{2, 0x1, JESDMODE1, JESDMODE2},
	{3, 0x2, JESDMODE1, JESDMODE2},
	{4, 0x3, JESDMODE1, JESDMODE4},
	{6, 0x4, JESDMODE1, JESDMODE4},
	{8, 0x5, JESDMODE1, JESDMODE5},
	{12, 0x6, JESDMODE1, JESDMODE5},
	{16, 0x7, JESDMODE1, JESDMODE6},
	{24, 0x8, JESDMODE1, JESDMODE6},
	{INTPL_MODE_INVALID,0xFF, JESDMODEINVALID, JESDMODEINVALID}
};

/*DataSheet Table 42 */
struct ad916x_reg_data ADI_REC_SERDES_COUNTERS_TBL[] = {
	{0x300, 0x00}, /*Disable Serdes Links for Configuration*/
	{0x480, 0x38}, /*Config Serdes Error Counters*/
	{0x481, 0x38}, /*Config Serdes Error Counters*/
	{0x482, 0x38}, /*Config Serdes Error Counters*/
	{0x483, 0x38}, /*Config Serdes Error Counters*/
	{0x484, 0x38}, /*Config Serdes Error Counters*/
	{0x485, 0x38}, /*Config Serdes Error Counters*/
	{0x486, 0x38}, /*Config Serdes Error Counters*/
	{0x487, 0x38} /*Config Serdes Error Counters*/
};

/*DataSheet Table 42 */
struct ad916x_reg_data ADI_REC_SERDES_INIT_TBL[] = {
	{0x29E, 0x1F} /*ADI INTERNAL Init Serdes PLL Settings*/
};

/*DataSheet Table 16 */
struct ad916x_reg_data ADI_REC_EQ_INIT_TBL[] = {
	{0x2AA, 0xC3}, /*ADI INTERNAL Equaliser Settings*/
	{0x2AB, 0x93}, /*ADI INTERNAL Equaliser Settings*/
	{0x2B1, 0xC3}, /*ADI INTERNAL Equaliser Settings*/
	{0x2B2, 0x93} /*ADI INTERNAL Equaliser Settings*/
};

/*Engineering Sample DataSheet Table 42 */
struct ad916x_reg_data ADI_REC_ES_SERDES_INIT_TBL_1[] = {
	{0x284, 0x62}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x285, 0xC9}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x286, 0x0E}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x287, 0x12}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x28B, 0x00}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2A7, 0x01}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2AE, 0x01}, /*ADI INTERNAL Init Serdes PLL Settings*/
};

struct ad916x_reg_data ADI_REC_ES_SERDES_INIT_TBL_2[] = {
	{0x28F, 0x3C}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x291, 0x49}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x290, 0x89}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x294, 0x24}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x296, 0x03}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x297, 0x0D}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x299, 0x02}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x29A, 0x8E}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x29B, 0x0E}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x29C, 0x28}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x29E, 0x1F}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2A0, 0x06}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2A4, 0x88}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x29F, 0x78}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2AA, 0xBA}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2AB, 0x8A}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2B1, 0xBB}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x2B2, 0x8A}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x218, 0x01}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x232, 0xFF}, /*ADI INTERNAL Init Serdes PLL Settings*/
	{0x268, 0x62} /*ADI INTERNAL Init Serdes PLL Settings*/
};


static int check_interpolation_range(uint8_t interpolation, uint16_t prod_id )
{
	uint8_t interpol_min;

	switch(prod_id) {
	case 0x9161:
		interpol_min = 2;
		break;
	case 0x9163:
		interpol_min = 6;
		break;
	case 0x9162:
	default:
		interpol_min = INTERPOLATION_MIN;
	}

	if ((interpolation == interpol_min) ||
			(interpolation > INTERPOLATION_MAX)) {
		return API_ERROR_INVALID_PARAM;
	}

	return API_ERROR_OK;
}

static int check_jesd_params_range(jesd_param_t jesd_param)
{
	/*Transport layer Parameter Ranges Table 22*/
	if ((jesd_param.jesd_L < LANE_MIN) ||
		(jesd_param.jesd_L > LANE_MAX) ||
		(jesd_param.jesd_L == 5)) {
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_param.jesd_F != 1) && (jesd_param.jesd_F != 2)
			                     && (jesd_param.jesd_F != 4)) {
		return API_ERROR_INVALID_PARAM;
	}

	if (jesd_param.jesd_M != M_DEFAULT){
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_param.jesd_S < S_MIN) || (jesd_param.jesd_S > S_MAX)) {
		return API_ERROR_INVALID_PARAM;
	}

	/*Check Device Parameters*/
	if (jesd_param.jesd_CF != CF_DEFAULT) {
		return API_ERROR_INVALID_PARAM;
	}

	if (jesd_param.jesd_CS != CS_DEFAULT) {
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_param.jesd_F == 1) && (jesd_param.jesd_HD != 1)) {
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_param.jesd_F != 1) && (jesd_param.jesd_HD != HD_DEFAULT)) {
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_param.jesd_N != N_DEFAULT) || (jesd_param.jesd_NP != NP_DEFAULT)) {
		return API_ERROR_INVALID_PARAM;
	}

	return API_ERROR_OK;
}

static int check_jesd_params_config(int interpolation, int L,
									int M, int F, int S, uint8_t *mode)
{
	uint8_t i = 0;
	jesd_mode_t jesd_mode_sel = JESDMODEINVALID;
	uint8_t intpl_sel = INTPL_MODE_INVALID;

	/*Get Valid LMFS Combo*/
	for ( i=0; i < ARRAY_SIZE(SUPPORTED_JESD_MODES_TBL); i++) {
		if (SUPPORTED_JESD_MODES_TBL[i].L == L) {
			if(SUPPORTED_JESD_MODES_TBL[i].M == M) {
				if(SUPPORTED_JESD_MODES_TBL[i].F == F) {
					if(SUPPORTED_JESD_MODES_TBL[i].S == S) {
						jesd_mode_sel = SUPPORTED_JESD_MODES_TBL[i].jesd_mode;
						break;
					}
				}
			}
		}
	}

	if (jesd_mode_sel == JESDMODEINVALID) {
		return API_ERROR_INVALID_PARAM;
	}

	/*Check Interpolation + LMFS Combo*/
	for (i=0; i< ARRAY_SIZE(SUPPORTED_JESD_INTPL_MODES_TBL); i++) {
			if (SUPPORTED_JESD_INTPL_MODES_TBL[i].intpl == interpolation) {
				intpl_sel = SUPPORTED_JESD_INTPL_MODES_TBL[i].intpl;
				break;
			}
	}
	if (intpl_sel == INTPL_MODE_INVALID) {
		return API_ERROR_INVALID_PARAM;
	}

	if ((jesd_mode_sel < SUPPORTED_JESD_INTPL_MODES_TBL[i].jesd_mode_min ) ||
			(jesd_mode_sel > SUPPORTED_JESD_INTPL_MODES_TBL[i].jesd_mode_max)) {
		return API_ERROR_INVALID_PARAM;
	}
	*mode = SUPPORTED_JESD_INTPL_MODES_TBL[i].intpl_cfg;
	return API_ERROR_OK;
}

static int get_cdr_serdes_cfg(uint64_t lane_rate_mhz, uint8_t *serializer_cfg)
{
	uint8_t cdr_en_half_rate = 0x0;
	uint8_t cdr_div_rate = 0x0;
	uint8_t serdes_pll_clk_rate = 0x0;
	uint8_t serdes_pll_div_fctr = 0x0;

	/*Get Clock Data Recovery & SERDES Settings*/
	if ((lane_rate_mhz > LANE_RATE_MIN) && 
			(lane_rate_mhz <= (LANE_RATE_MIN*2))) {
		cdr_en_half_rate = 0x0;
		cdr_div_rate = 0x2;
		serdes_pll_clk_rate = 0x1;
		serdes_pll_div_fctr = 0x2;
	} else if ((lane_rate_mhz > (LANE_RATE_MIN*2)) &&
			   (lane_rate_mhz <= (LANE_RATE_MIN*4))) {
		cdr_en_half_rate = 0x0;
		cdr_div_rate = 0x1;
		serdes_pll_clk_rate = 0x0;
		serdes_pll_div_fctr = 0x2;
	} else if ((lane_rate_mhz > LANE_RATE_MIN*4) &&
				(lane_rate_mhz < (LANE_RATE_MIN*8))) {
		cdr_en_half_rate = 0x0;
		cdr_div_rate = 0x0;
		serdes_pll_clk_rate = 0x0;
		serdes_pll_div_fctr = 0x1;
	} else if ((lane_rate_mhz >= (LANE_RATE_MIN*8)) &&
			 (lane_rate_mhz <= LANE_RATE_MAX)) {
		cdr_en_half_rate = 0x1;
		cdr_div_rate = 0x0;
		serdes_pll_clk_rate = 0x0;
		serdes_pll_div_fctr = 0x0;
	} else {
		return API_ERROR_INVALID_PARAM;
	}

	serializer_cfg[0] = cdr_en_half_rate;
	serializer_cfg[1] = cdr_div_rate;
	serializer_cfg[2] = serdes_pll_clk_rate;
	serializer_cfg[3] = serdes_pll_div_fctr;

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_config_datapath(ad916x_handle_t *h,
					jesd_param_t jesd_param, uint8_t interpolation,
					uint64_t *lane_rate_mbps)
{
	int err;
	uint8_t interpol_mode;
	uint8_t tmp_reg = 0x0;
	uint8_t M = 0x0;
	uint64_t dac_rate_mhz = 0x0, dac_clk_freq_mhz, tmp_lane_rate_mbps;
	uint8_t cdr_serdes_cfg[4] = {0x0, 0x0, 0x0, 0x0};
	ad916x_chip_id_t chip_id;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	err = ad916x_get_chip_id(h, &chip_id);
	if (err != API_ERROR_OK) {
		return err;
	}
	
	err = check_interpolation_range(interpolation, chip_id.prod_id);
	if (err != API_ERROR_OK) {
		return err;
	}

	err = check_jesd_params_range(jesd_param);
	if (err != API_ERROR_OK) {
		return err;
	}

	dac_clk_freq_mhz = DIV_U64(h->dac_freq_hz, 1000000);
	if((dac_clk_freq_mhz < DAC_CLK_FREQ_MIN) ||
			(dac_clk_freq_mhz > DAC_CLK_FREQ_MAX)) {
		return API_ERROR_INVALID_PARAM;
	}

	if (interpolation == 1) {
		M = 1;
	} else {
		M = jesd_param.jesd_M;
	}
	err = check_jesd_params_config(interpolation, jesd_param.jesd_L,
					M, jesd_param.jesd_F, jesd_param.jesd_S, &interpol_mode);
	if (err != API_ERROR_OK) {
		return err;
	}

	/*Calculate Lane Rate & Check Range*/
	dac_rate_mhz = DIV_U64(dac_clk_freq_mhz, interpolation);
	tmp_lane_rate_mbps = DIV_U64((dac_rate_mhz * (20 * M)), jesd_param.jesd_L);

	if(lane_rate_mbps != INVALID_POINTER) {
		*lane_rate_mbps = tmp_lane_rate_mbps;
	}

	/*Check LaneRate Range*/
	if ((tmp_lane_rate_mbps < LANE_RATE_MIN) ||
			(tmp_lane_rate_mbps > LANE_RATE_MAX)) {
		return API_ERROR_INVALID_PARAM;
	}

	/*Disable Serdes Links prior to configuration & Enable Error Counters*/
	/*Init DAC Based on Silicon Revision*/
	if (err != API_ERROR_OK) {
		return err;
	}
	if (chip_id.dev_revision == AD916x_REL_SI_REV) {
		/*Final Silicon Configuration*/
		err = ad916x_register_write_tbl(h,
					&ADI_REC_SERDES_COUNTERS_TBL[0],
					ARRAY_SIZE(ADI_REC_SERDES_COUNTERS_TBL));
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	/*Configure Lane and Interpolation Rate*/
	tmp_reg = ((jesd_param.jesd_L << 4) |
				((interpol_mode) & AD916x_FLD_INTPL_MODE));
	err = ad916x_register_write(h, AD916x_REG_LANE_INTPL_MODE, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	/*Look up and Configure CDR and Serdes PLL Settings for Lane Rate*/
	get_cdr_serdes_cfg(tmp_lane_rate_mbps, &cdr_serdes_cfg[0]);

	/*Apply CDR Setttings*/
	err = ad916x_register_read(h, AD916x_REG_CDR_OP_MODE_0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg = (tmp_reg & ~AD916x_FLD_ENHALFRATE) |
				((cdr_serdes_cfg[0] << 5) & AD916x_FLD_ENHALFRATE);
	tmp_reg = (tmp_reg & ~AD916x_FLD_DIVISION_RATE) |
				((cdr_serdes_cfg[1] << 1) & AD916x_FLD_DIVISION_RATE);
	err = ad916x_register_write(h, AD916x_REG_CDR_OP_MODE_0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	/*Apply SERDES PLL Settings*/
	err = ad916x_register_read(h, AD916x_REG_REF_CLK_DIVIDER_LDO, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg = (tmp_reg & ~AD916x_FLD_SERDES_PLL_DIV_FCTR) |
				((cdr_serdes_cfg[3] << 0) & AD916x_FLD_SERDES_PLL_DIV_FCTR);
	err = ad916x_register_write(h, AD916x_REG_REF_CLK_DIVIDER_LDO, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	tmp_reg = (tmp_reg & ~AD916x_FLD_LDO_BYPASS_FILT) |
				((0x0 << 2) & AD916x_FLD_LDO_BYPASS_FILT);
	err = ad916x_register_write(h, AD916x_REG_REF_CLK_DIVIDER_LDO, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	err = ad916x_register_read(h, AD916x_REG_PLL_REF_CLK_PD, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg = (tmp_reg & ~AD916x_FLD_PLL_REF_CLK_RATE) |
				((cdr_serdes_cfg[2] << 4) & AD916x_FLD_PLL_REF_CLK_RATE);
	err = ad916x_register_write(h, AD916x_REG_PLL_REF_CLK_PD, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_cfg_status(ad916x_handle_t *h,
										uint8_t *jesd_cfg_stat)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	
	if( jesd_cfg_stat == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_ILS_CFG_STATUS, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*jesd_cfg_stat = (tmp_reg & AD916x_FLD_CFG_STATUS);

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_cfg_param(ad916x_handle_t *h,
									jesd_param_t *jesd_param)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (jesd_param == INVALID_POINTER) {
			return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_ILS_SCR_L, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_L = ((tmp_reg & AD916x_FLD_L) + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_F, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_F = (tmp_reg + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_K, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_K = ((tmp_reg & AD916x_FLD_K) + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_M, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_M = (tmp_reg + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_CS_N, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_N = ((tmp_reg & AD916x_FLD_N) + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_NP, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_NP = ((tmp_reg & AD916x_FLD_NP) + 1);

	err = ad916x_register_read(h, AD916x_REG_ILS_S, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_S = ((tmp_reg & AD916x_FLD_S) + 1);
	jesd_param->jesd_JESDV = ((tmp_reg & AD916x_FLD_JESDV) >> 5);

	err = ad916x_register_read(h, AD916x_REG_ILS_HD_CF, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_HD = ((tmp_reg & AD916x_FLD_HD) >> 7);

	err = ad916x_register_read(h, AD916x_REG_ILS_DID, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_DID = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_ILS_BID, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_BID = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_ILS_LID0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	jesd_param->jesd_LID0 = (tmp_reg & AD916x_FLD_LID0);

	/*Fixed Parameters*/
	jesd_param->jesd_CF = CF_DEFAULT;
	jesd_param->jesd_CS = CS_DEFAULT;

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_set_sysref_mode(ad916x_handle_t *h,
					ad916x_jesd_sysref_mode_t mode, uint8_t jitter_window)
{
	int err;
	uint8_t tmp_reg = 0x0;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (mode >= SYSREF_MODE_INVLD) {
		return API_ERROR_INVALID_PARAM;
	}

	if (mode == SYSREF_NONE) {
		/*set Subclass to Version 0*/
		err = ad916x_register_read(h, AD916x_REG_GEN_JRX_CTRL_1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= (~AD916x_FLD_SUBCLASS | 0x0); /*subclass 0*/
		err = ad916x_register_write(h, AD916x_REG_GEN_JRX_CTRL_1, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		/*set Subclass to version 1*/
		err = ad916x_register_read(h, AD916x_REG_GEN_JRX_CTRL_1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= (~AD916x_FLD_SUBCLASS | 0x1); /*sublcass 1*/
		err = ad916x_register_write(h, AD916x_REG_GEN_JRX_CTRL_1, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	if (mode == SYSREF_MON) {
		mode = SYSREF_NONE; /*if monitor mode selected configu to SYSREF_NONE*/

		/*Set Sysref jitter window tolerance*/
		if(jitter_window > SYSREF_JITTER_WIN_MAX) {
			return API_ERROR_INVALID_PARAM;
		}

		err = ad916x_register_read(h, AD916x_REG_SYSREF_JITTER_WIN, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= (jitter_window & AD916x_FLD_SYSREF_JITTER_WIN);
		err = ad916x_register_write(h, AD916x_REG_SYSREF_JITTER_WIN, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	err = ad916x_register_read(h, AD916x_REG_SYNC_CTRL, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg |= (mode & AD916x_FLD_SYNC_MODE);
	err = ad916x_register_write(h, AD916x_REG_SYNC_CTRL, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_sysref_mode(ad916x_handle_t *h,
					ad916x_jesd_sysref_mode_t *mode, uint8_t *sysref_jttr_win)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if ( mode != INVALID_POINTER) {
		err = ad916x_register_read(h, AD916x_REG_SYNC_CTRL, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*mode = (tmp_reg & AD916x_FLD_SYNC_MODE);
	}

	if ( sysref_jttr_win != INVALID_POINTER) {
		err = ad916x_register_read(h, AD916x_REG_SYSREF_JITTER_WIN, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		*sysref_jttr_win = (tmp_reg & AD916x_FLD_SYSREF_JITTER_WIN);
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_sysref_status(ad916x_handle_t *h,
					uint8_t *sysref_count, uint16_t *sysref_phase,
					uint16_t *sync_lmfc_stat)
{
	int err;
	uint8_t tmp_reg[2] = {0x0, 0x0};
	uint16_t tmp_val;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	/*reset Counters*/
	err = ad916x_register_write(h, AD916x_REG_SYNC_CTRL, 0x0);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_write(h, AD916x_REG_SYNC_LMFC_ST0, 0x0);
	if (err != API_ERROR_OK) {
		return err;
	}

	if ( sysref_count != INVALID_POINTER) {
		err = ad916x_register_read(h, AD916x_REG_SYNC_CTRL, &tmp_reg[0]);
		if (err != API_ERROR_OK) {
			return err;
		}
		*sysref_count = (tmp_reg[0]);
	}

	if ( sync_lmfc_stat != INVALID_POINTER) {
		err = ad916x_register_read(h, AD916x_REG_SYNC_LMFC_ST0, &tmp_reg[0]);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_read(h, AD916x_REG_SYNC_LMFC_ST1, &tmp_reg[1]);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_val = (((tmp_reg[1] & AD916x_FLD_SYNC_LMFC_MSB) << 8) |
					(tmp_reg[0]));
		*sync_lmfc_stat = tmp_val;
	}

	if ( sysref_phase != INVALID_POINTER) {
		err = ad916x_register_read(h, AD916x_REG_SYSREF_PHASE0, &tmp_reg[0]);
		if (err != API_ERROR_OK) {
			return err;
		}
		err = ad916x_register_read(h, AD916x_REG_SYSREF_PHASE1, &tmp_reg[1]);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_val = (((tmp_reg[1] & AD916x_FLD_SYSREF_PHASE_MSB) << 8) |
					(tmp_reg[0]));
		*sysref_phase = tmp_val;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_dynamic_link_latency(ad916x_handle_t *h,
											uint8_t *latency)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if ( latency == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_DYN_LINK_LATENCY_0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	*latency = (tmp_reg & AD916x_FLD_DYN_LINK_LATENCY);

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_set_lmfc_delay(ad916x_handle_t *h,
										uint8_t delay, uint8_t var)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER){
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ((delay > AD916x_FLD_LMFC_DELAY) || (var > AD916x_FLD_LMFC_VAR)) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_LMFC_DELAY_0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg &= ~AD916x_FLD_LMFC_DELAY;
	tmp_reg |= (delay & AD916x_FLD_LMFC_DELAY);
	err = ad916x_register_write(h, AD916x_REG_LMFC_DELAY_0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	err = ad916x_register_read(h, AD916x_REG_LMFC_VAR_0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg &= ~AD916x_FLD_LMFC_VAR;
	tmp_reg |= (delay & AD916x_FLD_LMFC_VAR);
	err = ad916x_register_write(h, AD916x_REG_LMFC_VAR_0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_set_lane_xbar(ad916x_handle_t *h,
						uint8_t physical_lane, uint8_t logical_lane)
{
	int err;
	uint8_t tmp_reg_val;
	uint16_t tmp_reg_addr;
	uint8_t tmp_nibble;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if ((physical_lane > (LANE_MAX-1)) || (logical_lane > LANE_MAX-1)) {
		return API_ERROR_INVALID_PARAM;
	}

	switch(physical_lane) {
	case 0:
		tmp_reg_addr = AD916x_REG_XBAR_LN_0_1;
		tmp_nibble = 0x0;
		break;
	case 1:
		tmp_reg_addr = AD916x_REG_XBAR_LN_0_1;
		tmp_nibble = 0x1;
		break;
	case 2:
		tmp_reg_addr = AD916x_REG_XBAR_LN_2_3;
		tmp_nibble =0x0;
		break;
	case 3:
		tmp_reg_addr = AD916x_REG_XBAR_LN_2_3;
		tmp_nibble = 0x1;
		break;
	case 4:
		tmp_reg_addr = AD916x_REG_XBAR_LN_4_5;
		tmp_nibble = 0x0;
		break;
	case 5:
		tmp_reg_addr = AD916x_REG_XBAR_LN_4_5;
		tmp_nibble = 0x1;
		break;
	case 6:
		tmp_reg_addr = AD916x_REG_XBAR_LN_6_7;
		tmp_nibble = 0x0;
		break;
	case 7:
		tmp_reg_addr = AD916x_REG_XBAR_LN_6_7;
		tmp_nibble = 0x1;
		break;
	default:
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, tmp_reg_addr, &tmp_reg_val);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (tmp_nibble == 0) {
		tmp_reg_val = (tmp_reg_val & (~AD916x_FLD_XBAR_LN_EVEN)) |
						(logical_lane);
	} else {
		tmp_reg_val = (tmp_reg_val & (~AD916x_FLD_XBAR_LN_ODD)) |
						(logical_lane << 0x3);
	}
	err = ad916x_register_write(h, tmp_reg_addr, tmp_reg_val);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_invert_lane(ad916x_handle_t *h,
									uint8_t logical_lane, uint8_t invert)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (logical_lane > (LANE_MAX-1)) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_JESD_BIT_INVERT_CTRL, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	if (invert) {
		tmp_reg |= GET_LANE_MSK(logical_lane);
	} else {
		tmp_reg &= (~(GET_LANE_MSK(logical_lane)));
	}

	err = ad916x_register_write(h, AD916x_REG_JESD_BIT_INVERT_CTRL, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_lane_xbar(ad916x_handle_t *h, uint8_t *phy_log_map)
{
	int err;
	uint8_t tmp_xbar_A;
	uint8_t tmp_xbar_B;
	uint8_t tmp_xbar_C;
	uint8_t tmp_xbar_D;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if(phy_log_map == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_XBAR_LN_0_1, &tmp_xbar_A);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_read(h, AD916x_REG_XBAR_LN_2_3, &tmp_xbar_B);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_read(h, AD916x_REG_XBAR_LN_4_5, &tmp_xbar_C);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_read(h, AD916x_REG_XBAR_LN_6_7, &tmp_xbar_D);
	if (err != API_ERROR_OK) {
		return err;
	}

	phy_log_map[0] = tmp_xbar_A & AD916x_FLD_XBAR_LN_EVEN;
	phy_log_map[1] = ((tmp_xbar_A & AD916x_FLD_XBAR_LN_ODD) >> 3);
	phy_log_map[2] = tmp_xbar_B & AD916x_FLD_XBAR_LN_EVEN;
	phy_log_map[3] = ((tmp_xbar_B & AD916x_FLD_XBAR_LN_ODD) >> 3);
	phy_log_map[4] = tmp_xbar_C & AD916x_FLD_XBAR_LN_EVEN;
	phy_log_map[5] = ((tmp_xbar_C & AD916x_FLD_XBAR_LN_ODD) >> 3);
	phy_log_map[6] = tmp_xbar_D & AD916x_FLD_XBAR_LN_EVEN;
	phy_log_map[7] = ((tmp_xbar_D & AD916x_FLD_XBAR_LN_ODD) >> 3);

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_enable_datapath(ad916x_handle_t *h, uint8_t lanes_msk,
									uint8_t en, uint8_t run_cal)
{
	int err;
	uint8_t tmp_reg;
	ad916x_chip_id_t chip_id;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if ( (en > 1) || (run_cal > 1)) {
		return API_ERROR_INVALID_PARAM;
	}

	if (en == 0x1) {
		/*Enable JESD Block*/
		err = ad916x_register_read(h, AD916x_REG_MASTER_PD, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg = (tmp_reg & ~AD916x_FLD_PD_MASTER);
		err = ad916x_register_write(h, AD916x_REG_MASTER_PD, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Soft Reset JESD Deframer*/
		err = ad916x_register_read(h, AD916x_REG_JESD_CTRL0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= AD916x_FLD_DEFRAMER_RESET;
		err = ad916x_register_write(h, AD916x_REG_JESD_CTRL0, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		err = ad916x_register_read(h, AD916x_REG_JESD_CTRL0, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= ~AD916x_FLD_DEFRAMER_RESET;
		err = ad916x_register_write(h, AD916x_REG_JESD_CTRL0, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Power Down Any unused Lanes*/
		tmp_reg = ~lanes_msk;
		err = ad916x_register_write(h, AD916x_REG_PHY_PD, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		if (run_cal == 0x1) {
			/*Initialise Equaliser*/
			ad916x_register_write_tbl(h, &ADI_REC_EQ_INIT_TBL[0],
						ARRAY_SIZE(ADI_REC_EQ_INIT_TBL));

			/*Calibrate SERDES PHY Termination Block*/
			err = ad916x_register_read(h, AD916x_REG_TERM_BLK1_CTRL0, &tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}
			tmp_reg |=AD916x_FLD_TUNE_CALTERM_A;
			err = ad916x_register_write(h, AD916x_REG_TERM_BLK1_CTRL0, tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}

			err = ad916x_register_read(h, AD916x_REG_TERM_BLK2_CTRL0, &tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}
			tmp_reg |= AD916x_FLD_TUNE_CALTERM_B;
			err = ad916x_register_write(h, AD916x_REG_TERM_BLK2_CTRL0, tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}
		}

		err = ad916x_get_chip_id(h, &chip_id);
		if (err != API_ERROR_OK) {
			return err;
		}
		if (chip_id.dev_revision == AD916x_REL_SI_REV) {
			/*Final Silicon Configuration*/
			err = ad916x_register_write_tbl(h, &ADI_REC_SERDES_INIT_TBL[0],
										ARRAY_SIZE(ADI_REC_SERDES_INIT_TBL));
			if (err != API_ERROR_OK) {
				return err;
			}
		} else {
			/*Engineeing Sample Silicon Configuration*/
			/*Config SERDES PLL with ADI RECOMMENDED Settings*/
			err = ad916x_register_write_tbl(h, &ADI_REC_ES_SERDES_INIT_TBL_1[0],
									ARRAY_SIZE(ADI_REC_ES_SERDES_INIT_TBL_1));
			if (err != API_ERROR_OK) {
				return err;
			}

			/*Reset CDR*/
			err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			};
			tmp_reg &= ~AD916x_FLD_CDR_RESET;
			err = ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}

			err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}
			tmp_reg |= AD916x_FLD_CDR_RESET;
			err = ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);
			if (err != API_ERROR_OK) {
				return err;
			}

			/*Config SERDES PLL with ADI RECOMMENDED Settings*/
			err = ad916x_register_write_tbl(h, &ADI_REC_ES_SERDES_INIT_TBL_2[0],
								ARRAY_SIZE(ADI_REC_ES_SERDES_INIT_TBL_2));
			if (err != API_ERROR_OK) {
				return err;
			}
		}

		/*Reset CDR*/
		err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= ~AD916x_FLD_CDR_RESET;
		err = ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= AD916x_FLD_CDR_RESET;
		err = ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Configure JESD for ILAS*/
		err = ad916x_register_read(h, AD916x_REG_JESD_CTRL1, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= AD916x_FLD_CGS_SEL;
		err = ad916x_register_write(h, AD916x_REG_JESD_CTRL1, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Reset CDR*/
		err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= ~AD916x_FLD_CDR_RESET;
		err = ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		err = ad916x_register_read(h, AD916x_REG_CDR_RESET, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= AD916x_FLD_CDR_RESET;
		ad916x_register_write(h, AD916x_REG_CDR_RESET, tmp_reg);

		/*Enable SERDES PLL*/
		err = ad916x_register_read(h, AD916x_REG_SYNTH_ENABLE_CTRL, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= (AD916x_FLD_EN_CURRENTS | AD916x_FLD_EN_SYNTH);
		err = ad916x_register_write(h, AD916x_REG_SYNTH_ENABLE_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		/*TODO: Procedure to disable Reset JESD IF*/
		/*Disable SERDES PLL*/
		err = ad916x_register_read(h, AD916x_REG_SYNTH_ENABLE_CTRL, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg &= ~(AD916x_FLD_EN_CURRENTS | AD916x_FLD_EN_SYNTH);
		err = ad916x_register_write(h, AD916x_REG_SYNTH_ENABLE_CTRL, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Power Down ALL Lanes*/
		err = ad916x_register_write(h, AD916x_REG_PHY_PD, 0xFF);
		if (err != API_ERROR_OK) {
			return err;
		}

		/*Disable JESD Block*/
		err = ad916x_register_read(h, AD916x_REG_MASTER_PD, &tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
		tmp_reg |= AD916x_FLD_PD_MASTER ;
		err = ad916x_register_write(h, AD916x_REG_MASTER_PD, tmp_reg);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_enable_link(ad916x_handle_t *h, uint8_t en)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (en > 0x1) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_GEN_JRX_CTRL_0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg |= (0x01 & en);
	err = ad916x_register_write(h, AD916x_REG_GEN_JRX_CTRL_0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	/*Soft Reset JESD Deframer*/
	err = ad916x_register_read(h, AD916x_REG_JESD_CTRL0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg |= AD916x_FLD_DEFRAMER_RESET;
	err = ad916x_register_write(h, AD916x_REG_JESD_CTRL0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	err = ad916x_register_read(h, AD916x_REG_JESD_CTRL0, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg &= ~AD916x_FLD_DEFRAMER_RESET;
	err = ad916x_register_write(h, AD916x_REG_JESD_CTRL0, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_pll_status(ad916x_handle_t *h, uint8_t *pll_status)
{
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (pll_status == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_PLL_STATUS, pll_status);
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_get_link_status(ad916x_handle_t *h,
									ad916x_jesd_link_stat_t *link_status)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER ) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if(link_status == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_CODE_GRP_SYNC, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	link_status->code_grp_sync_stat = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_FRAME_SYNC, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	link_status->frame_sync_stat = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_GOOD_CHECKSUM, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	link_status->good_checksum_stat = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_INIT_LANE_SYNC, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	link_status->init_lane_sync_stat = tmp_reg;

	return API_ERROR_OK;
}

ADI_API int ad916x_jesd_enable_scrambler(ad916x_handle_t *h, uint8_t en)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (en > 0x1) {
		return API_ERROR_INVALID_PARAM;
	}

	err = ad916x_register_read(h, AD916x_REG_ILS_SCR_L, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (en == 0) {
		tmp_reg &= ~AD916x_FLD_SCR;
	} else {
		tmp_reg |= AD916x_FLD_SCR;
	}
	err = ad916x_register_write(h, AD916x_REG_ILS_SCR_L, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}
