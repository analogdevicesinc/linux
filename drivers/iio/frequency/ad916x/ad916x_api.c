
#include <linux/kernel.h>
#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"


#define FULL_SCALE_CURRENT_MIN 8
#define FULL_SCALE_CURRENT_MAX 40
#define HW_RESET_PERIOD_US 10
#define SPI_RESET_PERIOD_US 50
#define NVRAM_RESET_PERIOD_US 1000
#define DAC_CLK_FREQ_MHZ_MAX 6000
#define DAC_CLK_FREQ_MHZ_MIN 850

/*======================================
 * Revision Data
 *=====================================*/
static uint8_t api_revision[3] = {1,0,0};

/*DataSheet Table 41*/
/*Engineering Sample Data*/
struct ad916x_reg_data ADI_RECOMMENDED_BOOT_TBL[] = {
	{0x0D2, 0x52}, /*ADI INTERNAL:Internal Cal*/
	{0x0D2, 0xD2}, /*ADI INTERNAL:Internal Cal*/
	{0x606, 0x02}, /*ADI INTERNAL:Configure NVRAM*/
	{0x607, 0x00}, /*ADI INTERNAL:Configure NVRAM*/
	{0x604, 0x01} /*ADI INTERNAL:LOAD NVRAM FACTORY SETTINGS*/
};
/*TODO: To be removed for following Release*/
struct ad916x_reg_data ADI_RECOMMENDED_ES_INIT_TBL[] = {
	{0x058, 0x03}, /*Enable Bandgap Reference*/
	{0x090, 0x1E}, /*Power up the DACCLK DLL*/
	{0x080, 0x00}, /*Enable the Clock Receiver*/
	{0x040, 0x00}, /*Enable the DAC BIAS circuits*/
	{0x289, 0x04}, /*Enable the Serdes Reference Clock Divider*/
	{0x047, 0x01}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x049, 0x17}, /*ADI INTERNAL: Configure Dac analog Params*/
	{0x04A, 0x04}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x04E, 0x15}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x059, 0x0F}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x05A, 0x08}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x079, 0x81}, /*ADI INTERNAL: Configure DAC analog Params*/
	{0x480, 0x38}, /*Enable SERDES Error Counters*/
	{0x481, 0x38}, /*Enable SERDES Error Counters*/
	{0x482, 0x38}, /*Enable SERDES Error Counters*/
	{0x483, 0x38}, /*Enable SERDES Error Counters*/
	{0x484, 0x38}, /*Enable SERDES Error Counters*/
	{0x485, 0x38}, /*Enable SERDES Error Counters*/
	{0x486, 0x38}, /*Enable SERDES Error Counters*/
	{0x487, 0x38}, /*Enable SERDES Error Counters*/
	{0x093, 0x00}, /*ADI INTERNAL: Configure DACCLK DLL*/
	{0x094, 0x00}, /*ADI INTERNAL: Configure DACCLK DLL*/
	{0x096, 0x0D}, /*ADI INTERNAL: Configure DACCLK DLL*/
	{0x09E, 0x85}, /*ADI INTERNAL: Configure DACCLK DLL*/
	{0x091, 0xE9}, /*Enable DACCLK DLL*/
	{0x0E8, 0x20} /*ADI INTERNAL: Enable Calibration Factors*/
};

struct ad916x_reg_data ADI_RECOMMENDED_INIT_TBL[] = {
	{0x058, 0x03}, /*Enable Bandgap Reference*/
	{0x090, 0x1E}, /*Power up the DACCLK DLL*/
	{0x080, 0x00}, /*Enable the Clock Receiver*/
	{0x040, 0x00}, /*Enable the DAC BIAS circuits*/
	{0x09E, 0x85}, /*ADI INTERNAL: Configure DACCLK DLL*/
	{0x091, 0xE9}, /*Enable DACCLK DLL*/
	{0x0E8, 0x20} /*ADI INTERNAL: Enable Calibration Factors*/
};

static int spi_configure(ad916x_handle_t *h)
{
	switch (h->sdo) {
	case SPI_SDO:
		return ad916x_register_write(h, AD916x_REG_IF_CFG_A, 0x18);
	case SPI_SDIO:
		return ad916x_register_write(h, AD916x_REG_IF_CFG_A, 0x00);
	default:
		return API_ERROR_SPI_SDO;
		break;
	}
	return API_ERROR_SPI_SDO;
}

static int dac_init_sequence(ad916x_handle_t *h)
{
	int err;
	uint8_t tmp_reg = 0x0;
	ad916x_chip_id_t chip_id;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	/*Boot from NVRAM & Check Boot Status*/
	err = ad916x_register_write_tbl(h, &ADI_RECOMMENDED_BOOT_TBL[0],
				ARRAY_SIZE(ADI_RECOMMENDED_BOOT_TBL));
	if(err != API_ERROR_OK) {
		return err;
	}

	if (h->delay_us != INVALID_POINTER) {
		err = h->delay_us(h->user_data, NVRAM_RESET_PERIOD_US);
		if (err != 0) {
			return API_ERROR_US_DELAY;
		}
	}
	err = ad916x_register_read(h, AD916x_REG_NVM_LOADER, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (!(tmp_reg & AD916x_FLD_NVM_BLR_DONE)) {
		return API_ERROR_INIT_SEQ_FAIL;
	}
	/*Init DAC Based on Silicon Revision*/
	err = ad916x_get_chip_id(h, &chip_id);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (chip_id.dev_revision == AD916x_REL_SI_REV) {
		/*Final Silicon Configuration*/
		err = ad916x_register_write_tbl(h, &ADI_RECOMMENDED_INIT_TBL[0],
						ARRAY_SIZE(ADI_RECOMMENDED_INIT_TBL));
		if (err != API_ERROR_OK) {
			return err;
		}
	} else {
		/*Engineeing Sample Silicon Configuration*/
		err = ad916x_register_write_tbl(h, &ADI_RECOMMENDED_ES_INIT_TBL[0],
						ARRAY_SIZE(ADI_RECOMMENDED_ES_INIT_TBL));
		if (err != API_ERROR_OK) {
			return err;
		}
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_init(ad916x_handle_t *h)
{
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (h->dev_xfer == INVALID_POINTER) {
		return API_ERROR_INVALID_XFER_PTR;
	}
	if (h->delay_us == INVALID_POINTER) {
		return API_ERROR_INVALID_XFER_PTR;
	}
	if (h->sdo >= SPI_CONFIG_MAX) {
		return API_ERROR_SPI_SDO;
	}
	if (h->hw_open != INVALID_POINTER) {
		err = h->hw_open(h->user_data);
		if (err != 0) {
			return API_ERROR_HW_OPEN;
		}
	}
	err = spi_configure(h);
	if (err != API_ERROR_OK) {
		return err;
	}
	return dac_init_sequence(h);
}

ADI_API int ad916x_deinit(ad916x_handle_t *h)
{
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (h->hw_close != INVALID_POINTER) {
		err = h->hw_close(h->user_data);
		if (err != 0) {
			return API_ERROR_HW_CLOSE;
		}
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_get_chip_id(ad916x_handle_t *h, ad916x_chip_id_t *chip_id)
{
	int err;
	uint8_t tmp_reg;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if (chip_id == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_read(h, AD916x_REG_CHIP_TYPE, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	chip_id->chip_type = tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_PROD_ID_MSB, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	chip_id->prod_id = tmp_reg;
	chip_id->prod_id <<= 8;

	err = ad916x_register_read(h, AD916x_REG_PROD_ID_LSB, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	chip_id->prod_id |= tmp_reg;

	err = ad916x_register_read(h, AD916x_REG_CHIP_GRADE, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	chip_id->prod_grade = (tmp_reg >> 4);
	chip_id->dev_revision = (tmp_reg & 0x0F);

	return API_ERROR_OK;
}

ADI_API int ad916x_reset(ad916x_handle_t *h, uint8_t hw_reset)
{
	int err;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if (hw_reset) {
		if (h->reset_pin_ctrl == INVALID_POINTER) {
			return API_ERROR_INVALID_PARAM;
		}

		err = h->reset_pin_ctrl(h->user_data, 0x1);
		if (err != 0) {
			return API_ERROR_RESET_PIN_CTRL;
		}
		if (h->delay_us != INVALID_POINTER) {
			err = h->delay_us(h->user_data, HW_RESET_PERIOD_US);
			if (err != 0) {
				return API_ERROR_US_DELAY;
			}
		}
		err = h->reset_pin_ctrl(h->user_data, 0x0);
		if (err != 0) {
			return API_ERROR_RESET_PIN_CTRL;
		}
	}

	switch (h->sdo) {
	case SPI_SDO:
		err = ad916x_register_write(h, AD916x_REG_IF_CFG_A, 0x99);
		if (err != API_ERROR_OK) {
			return err;
		}
		break;
	case SPI_SDIO:
		err = ad916x_register_write(h, AD916x_REG_IF_CFG_A, 0x81);
		if (err != API_ERROR_OK) {
			return err;
		}
		break;
	default:
		return API_ERROR_SPI_SDO;
		break;
	}

	if (h->delay_us != INVALID_POINTER) {
		err = h->delay_us(h->user_data, SPI_RESET_PERIOD_US);
		if (err != 0) {
			return API_ERROR_US_DELAY;
		}
	}

	err = dac_init_sequence(h);
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_transmit_enable_pin(ad916x_handle_t *h,
						const ad916x_tx_enable_pin_mode_t tx_en_func)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	err = ad916x_register_read(h, AD916x_REG_TX_ENABLE, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	switch (tx_en_func) {
	case NONE:
		tmp_reg &= ~(0x0F);
		break;
	case RESET_NCO:
		tmp_reg &= ~(0x0F);
		tmp_reg |= AD916x_FLD_TX_EN_NCO_RESET;
		break;
	case ZERO_DATA_DAC:
		tmp_reg &= ~(0x0F);
		tmp_reg |= AD916x_FLD_TX_EN_ZERO_DATA_DAC;
		break;
	case ZERO_DATA_IN_PATH:	
		tmp_reg &= ~(0x0F);
		tmp_reg |= AD916x_FLD_TX_EN_ZERO_DATA_PATH;
		break;
	case CURRENT_CONTROL:
		tmp_reg &= ~(0x0F);
		tmp_reg |= AD916x_FLD_TX_EN_FULL_SCALE_CURR;
		break;
	default:
		return API_ERROR_INVALID_PARAM;
		break;
	}

	err = ad916x_register_write(h, AD916x_REG_TX_ENABLE, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_transmit_enable(ad916x_handle_t *h,
						const ad916x_transmit_mode_t mode)
{
	int err;
	uint8_t tmp_reg;
	uint8_t enable;
	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	err = ad916x_register_read(h, AD916x_REG_TX_ENABLE, &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	/*HW Transmit Enable Control*/
	if ((tmp_reg & AD916x_FLD_TX_EN_ZERO_DATA_DAC) ||
					(tmp_reg & AD916x_FLD_TX_EN_ZERO_DATA_PATH)) {
		if(h->tx_en_pin_ctrl != INVALID_POINTER) {
			if(mode == TX_ENABLE) {
				enable =0x1;
			}
			else {
				enable = 0x0;
			}
			err = h->tx_en_pin_ctrl(h->user_data, enable);
			if (err != 0) {
				return API_ERROR_TX_EN_PIN_CTRL;
			}
		}

		return API_ERROR_NOT_SUPPORTED;
	}

	/* SW Transmit Enable Control*/
	switch(mode) {
	case TX_ENABLE:
		tmp_reg &= ~(AD916x_FLD_DATAPATH_POST | AD916x_FLD_DATAPATH_PRE);
		break;
	case TX_ZERO_DATA_DAC:
		tmp_reg &= ~AD916x_FLD_DATAPATH_PRE;
		tmp_reg |= AD916x_FLD_DATAPATH_POST;
		break;
	case TX_ZERO_DATA_IN_PATH:
		tmp_reg &= ~AD916x_FLD_DATAPATH_POST;
		tmp_reg |= AD916x_FLD_DATAPATH_PRE;
		break;
	default:
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_write(h, AD916x_REG_TX_ENABLE, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_clk_adjust_dutycycle(ad916x_handle_t *h,
										uint8_t en, uint8_t offset)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ( (en > 1) || (offset > AD916x_FLD_CLK_DUTY)) {
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_read(h, AD916x_REG_CLK_DUTY , &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg &= ~AD916x_FLD_CLK_DUTY;
	if(!(tmp_reg & AD916x_FLD_CLK_DUTY_EN)) {
		tmp_reg |= AD916x_FLD_CLK_DUTY_EN;
	}

	if (en == 0){
		tmp_reg &= ~AD916x_FLD_CLK_DUTY_OFFSET_EN;
	} else {
		tmp_reg |= AD916x_FLD_CLK_DUTY_OFFSET_EN;
		tmp_reg |= (offset & AD916x_FLD_CLK_DUTY);
	}
	err = ad916x_register_write(h, AD916x_REG_CLK_DUTY, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_clk_adjust_phase(ad916x_handle_t *h,
									uint8_t pol, uint8_t phase)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ((pol > 1) || (phase > AD916x_FLD_CLK_PHASE)) {
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_read(h, AD916x_REG_CLK_PHASE_TUNE , &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (pol == 0) {
		tmp_reg &= ~(AD916x_FLD_CLK_PHASE_POL);
	} else {
		tmp_reg |= (AD916x_FLD_CLK_PHASE_POL);
	}
	tmp_reg &= ~AD916x_FLD_CLK_PHASE;
	tmp_reg |= (phase & AD916x_FLD_CLK_PHASE);
	err = ad916x_register_write(h, AD916x_REG_CLK_PHASE_TUNE, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_clk_set_cross_ctrl(ad916x_handle_t *h,
									uint8_t en, uint8_t cross_point)
{
	int err;
	uint8_t tmp_reg;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ((en > 1) || (cross_point > AD916x_FLD_CLK_CRS_ADJ)){
			return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_read(h, AD916x_REG_CLK_CRS_CTRL , &tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	if (en == 0) {
		tmp_reg &= ~AD916x_FLD_CLK_CRS_EN;
	} else {
		tmp_reg &= ~AD916x_FLD_CLK_CRS_ADJ;
		tmp_reg |= AD916x_FLD_CLK_CRS_EN;
		tmp_reg |= (cross_point & AD916x_FLD_CLK_CRS_ADJ);
	}
	err = ad916x_register_write(h, AD916x_REG_CLK_CRS_CTRL, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_dac_set_clk_frequency(ad916x_handle_t *h,
										uint64_t dac_clk_freq_hz)
{
	if(h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ((dac_clk_freq_hz >  MHZ_TO_HZ(DAC_CLK_FREQ_MHZ_MAX)) ||
			(dac_clk_freq_hz < MHZ_TO_HZ(DAC_CLK_FREQ_MHZ_MIN))) {
			return API_ERROR_INVALID_PARAM;
	}
	h->dac_freq_hz = dac_clk_freq_hz;
	return API_ERROR_OK;
}

ADI_API int ad916x_dac_get_clk_frequency(ad916x_handle_t *h,
					uint64_t *dac_clk_freq_hz)
{
	if(h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if(dac_clk_freq_hz == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	*dac_clk_freq_hz = h->dac_freq_hz;
	return API_ERROR_OK;
}

ADI_API int ad916x_dac_set_full_scale_current(ad916x_handle_t *h,
										uint8_t current)
{
	int err;
	uint8_t tmp_reg[2];
	uint16_t fsc_val = 0x0;

	if(h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}
	if ((current > FULL_SCALE_CURRENT_MAX) ||
			(current < FULL_SCALE_CURRENT_MIN)) {
			return API_ERROR_INVALID_PARAM;
	}
	fsc_val = (((current - 8) *1023)/32);
	err = ad916x_register_read(h, AD916x_REG_ANA_FSC0 , &tmp_reg[0]);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_read(h, AD916x_REG_ANA_FSC1 , &tmp_reg[1]);
	if (err != API_ERROR_OK) {
		return err;
	}
	tmp_reg[0] = ((tmp_reg[0] & ~AD916x_FLD_ANA_FSC_LSB )
					| (uint8_t) (fsc_val & AD916x_FLD_ANA_FSC_LSB));
	tmp_reg[1] = (uint8_t)((fsc_val >> 2) & AD916x_FLD_ANA_FSC_MSB);
	err = ad916x_register_write(h, AD916x_REG_ANA_FSC0, tmp_reg[0]);
	if (err != API_ERROR_OK) {
		return err;
	}
	err = ad916x_register_write(h, AD916x_REG_ANA_FSC1, tmp_reg[1]);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_fir85_set_enable(ad916x_handle_t *h, const int en)
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
	if (en == 0) {
		tmp_reg &= ~AD916x_FLD_FIR85_EN;
	} else {
		tmp_reg |= AD916x_FLD_FIR85_EN;
	}
	err = ad916x_register_write(h, AD916x_REG_DATAPATH_CFG, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}
	return API_ERROR_OK;
}

ADI_API int ad916x_fir85_get_enable(ad916x_handle_t *h, int *en)
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
	*en = (tmp_reg & AD916x_FLD_FIR85_EN);
	return API_ERROR_OK;
}

ADI_API int ad916x_filt_bw90_enable(ad916x_handle_t *h, const int en)
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
	if (en == 0) {
		tmp_reg &= ~AD916x_FLD_FILT_BW90_EN;
	} else if (en == 1) {
		tmp_reg |= AD916x_FLD_FILT_BW90_EN;
	} else {
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_write(h, AD916x_REG_DATAPATH_CFG, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_invsinc_enable(ad916x_handle_t *h, const int en)
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
	if (en == 0) {
		tmp_reg &= ~(AD916x_FLD_INVSINC_EN);
	} else if (en == 1) {
		tmp_reg |= (AD916x_FLD_INVSINC_EN);
	} else {
		return API_ERROR_INVALID_PARAM;
	}
	err = ad916x_register_write(h, AD916x_REG_DATAPATH_CFG, tmp_reg);
	if (err != API_ERROR_OK) {
		return err;
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_get_revision(ad916x_handle_t *h, uint8_t *rev_major,
								uint8_t *rev_minor, uint8_t *rev_rc)
{
	int err = API_ERROR_OK;

	if (rev_major != INVALID_POINTER) {
		*rev_major = api_revision[0];
	} else {
		err = API_ERROR_INVALID_PARAM;
	}
	if (rev_minor != INVALID_POINTER) {
		*rev_minor = api_revision[1];
	} else {
		err = API_ERROR_INVALID_PARAM;
	}
	if(rev_rc != INVALID_POINTER) {
		*rev_rc = api_revision[2];
	} else {
		err = API_ERROR_INVALID_PARAM;
	}

	return err;
}
