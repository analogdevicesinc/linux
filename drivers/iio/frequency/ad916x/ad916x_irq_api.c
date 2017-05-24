#include "api_config.h"
#include "AD916x.h"
#include "ad916x_reg.h"
#include "api_errors.h"
#include "utils.h"

#define NOF_LINK_STAT AD916x_JESD_NOF_LANES

enum {
	GLOBAL_MASK_A = 0,
	JESD_LANE_MASK_A,
	JESD_LANE_MASK_B,
	NOF_MASK_REGS
};

const uint16_t int_msk_regs[] = {
	AD916x_REG_IRQ_EN,
	AD916x_REG_JESD_IRQ_ENABLE_A,
	AD916x_REG_JESD_IRQ_ENABLE_B,
};

const uint16_t int_status_regs[] = {
	AD916x_REG_IRQ_STATUS,
	AD916x_REG_JESD_IRQ_STATUS_A,
	AD916x_REG_JESD_IRQ_STATUS_B,
};

typedef struct event_mask_info_t {
	ad916x_event_t event;
	uint8_t event_msk_reg;
	uint8_t event_bit_msk;
}event_mask_info_t;

const event_mask_info_t EVENT_MSK_INFO_TBL[] = {
	{EVENT_SYSREF_JITTER, GLOBAL_MASK_A, AD916x_BIT_MSK_IRQ_SYSREFJITTER},
	{EVENT_DATA_RDY,  GLOBAL_MASK_A, AD916x_BIT_MSK_IRQ_DATA_RDY},
	{EVENT_JESD_LANE_FIFO_ERR, GLOBAL_MASK_A, AD916x_BIT_MSK_IRQ_LANE_FIFO},
	{EVENT_JESD_PRBS_IMG_ERR,  GLOBAL_MASK_A, AD916x_BIT_MSK_IRQ_PRBSQ},
	{EVENT_JESD_PRBS_REAL_ERR, GLOBAL_MASK_A, AD916x_BIT_MSK_IRQ_PRBSI},
	{EVENT_JESD_BAD_DISPARITY_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_BDE},
	{EVENT_JESD_NOT_IN_TBL_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_NIT},
	{EVENT_JESD_K_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_UEK},
	{EVENT_JESD_ILD_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_ILD},
	{EVENT_JESD_ILS_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_ILS},
	{EVENT_JESD_CKSUM_ERR,JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_CKS},
	{EVENT_JESD_FS_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_FS},
	{EVENT_JESD_CGS_ERR, JESD_LANE_MASK_A, AD916x_BIT_MSK_IRQ_CGS},
	{EVENT_JESD_ILAS_ERR, JESD_LANE_MASK_B,AD916x_BIT_MSK_IRQ_ILAS}
};

ADI_API int ad916x_set_events(ad916x_handle_t *h,
				ad916x_event_t *event_list, uint8_t list_size, uint8_t en)
{
	int err;
	uint8_t i, j, mask_vals[NOF_MASK_REGS];

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	if(event_list == INVALID_POINTER) {
		return API_ERROR_INVALID_PARAM;
	}

	/* Read current Interrupt Mask config*/
	for (i=0; i<NOF_MASK_REGS; i++) {
			err = ad916x_register_read(h, int_msk_regs[i], &mask_vals[i]);
			if (err != API_ERROR_OK) {
				return err;
			}
	}

	/*Enable/Disable list of events*/
	for (i=0; i<list_size; i++) {
		for(j=0; j< ARRAY_SIZE(EVENT_MSK_INFO_TBL); j++) {
			if(EVENT_MSK_INFO_TBL[j].event == event_list[i]) {
				mask_vals[EVENT_MSK_INFO_TBL[j].event_msk_reg] &= 
						(~EVENT_MSK_INFO_TBL[j].event_bit_msk);
				if(en) {
					mask_vals[EVENT_MSK_INFO_TBL[j].event_msk_reg] |=
						EVENT_MSK_INFO_TBL[j].event_bit_msk;
				}
			}
		}
	}

	/*Write new configuration*/
	for (i=0; i< NOF_MASK_REGS; i++) {
		err = ad916x_register_write(h, int_msk_regs[i], mask_vals[i]);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_get_interrupt_status(ad916x_handle_t *h,
									uint8_t *int_pending, uint8_t *int_status)
{
	int err;
	uint8_t  int_reg_status[NOF_MASK_REGS], i= 0x0, tmp_reg= 0x0;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	for (i=0; int_status_regs[i]; i++) {
		err = ad916x_register_read(h, int_status_regs[i], &int_reg_status[i]);
		if (err != API_ERROR_OK) {
			return err;
		}

		tmp_reg |= int_reg_status[i];
	}

	if (tmp_reg > 0x0) {
		*int_pending = 0x1;
	}
	else {
		*int_pending = 0x0;
	}
	

	if(int_status != INVALID_POINTER) {
		for (i=0; int_msk_regs[i]; i++) {
			int_status[i] = int_reg_status[i];
		}
	}

	return API_ERROR_OK;
}

ADI_API int ad916x_isr(ad916x_handle_t *h)
{
	int err;
	uint8_t  link_status[NOF_LINK_STAT], int_status[NOF_MASK_REGS];
	uint8_t i = 0x0;

	if (h == INVALID_POINTER) {
		return API_ERROR_INVALID_HANDLE_PTR;
	}

	/*************************************************
	 * Get Trigger Interrupt Status
	 *
	 *************************************************/
	/*Get Interrupts Status*/
	for (i=0; int_status_regs[i]; i++) {
		err = ad916x_register_read(h, int_status_regs[i], &int_status[i]);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	/*Read Per Link Interrupts Status*/
	for(i =0; i<NOF_LINK_STAT; i++) {
		err = ad916x_register_read(h, (AD916x_REG_LINK_STATUS0 +i), &link_status[i]);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	/*************************************************
	 * Clear Interrupt Events
	 *
	 *************************************************/
	for (i=0; int_status_regs[i]; i++) {
		err = ad916x_register_write(h, int_status_regs[i], int_status[i]);
		if (err != API_ERROR_OK) {
			return err;
		}
	}

	/*************************************************
	 * Process Chip Level Interrupts
	 * if event_handler present.
	 * If not API exits at this point since interrupts
	 * have been cleared.
	 *************************************************/
	if (h->event_handler == INVALID_POINTER) {
		return err;
	}

	/*Sysref Jitter Interrupt*/
	if(int_status[0] & AD916x_BIT_MSK_IRQ_SYSREFJITTER) {
		err = h->event_handler(EVENT_SYSREF_JITTER, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}

	/*Data Ready Interrupt*/
	if(int_status[0] &  AD916x_BIT_MSK_IRQ_DATA_RDY) {
		err = h->event_handler(EVENT_DATA_RDY, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}

	/*Lane Fifo Interrupt*/
	if(int_status[0] & AD916x_BIT_MSK_IRQ_LANE_FIFO) {
		err = h->event_handler(EVENT_JESD_LANE_FIFO_ERR, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}

	/*PRBS-Q Interrupt*/
	if(int_status[0] & AD916x_BIT_MSK_IRQ_PRBSQ) {
		err = h->event_handler(EVENT_JESD_PRBS_IMG_ERR, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}

	/*PRBS-I Interrupt*/
	if(int_status[0] & AD916x_BIT_MSK_IRQ_PRBSI) {
		err = h->event_handler(EVENT_JESD_PRBS_REAL_ERR, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}

	/*ILAS  Interrupt*/
	if(int_status[2] & AD916x_BIT_MSK_IRQ_ILAS) {
		err = h->event_handler(EVENT_JESD_ILAS_ERR, 0, 0);
		if (err != 0) {
			return API_ERROR_EVENT_HNDL;
		}
	}
	/*************************************************
	 * Process Lane Level Interrupts Status
	 *
	 *************************************************/
	/*CGS Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_CGS) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] &AD916x_BIT_MSK_IRQ_CGS) {
				err = h->event_handler(EVENT_JESD_CGS_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Frame Sync Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_FS) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_FS) {
				err = h->event_handler(EVENT_JESD_FS_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Good Checksum Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_CKS) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_CKS) {
				err = h->event_handler(EVENT_JESD_CKSUM_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Initial Lane Sync Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_ILS) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_ILS) {
				err = h->event_handler(EVENT_JESD_ILS_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}
	

	/*Inter Lane De-skew Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_ILD) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_ILD) {
				err = h->event_handler(EVENT_JESD_ILD_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Un-expected K Character Error Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_UEK) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_UEK) {
				err = h->event_handler(EVENT_JESD_K_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Not in Table Error Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_NIT) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_NIT) {
				err =h->event_handler(EVENT_JESD_NOT_IN_TBL_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}

	/*Bad Disparity Error Interrupt*/
	if(int_status[1] & AD916x_BIT_MSK_IRQ_BDE) {
		for (i =0; i<NOF_LINK_STAT; i++) {
			if(link_status[i] & AD916x_BIT_MSK_IRQ_BDE) {
				err = h->event_handler(EVENT_JESD_BAD_DISPARITY_ERR, i, 0);
				if (err != 0) {
					return API_ERROR_EVENT_HNDL;
				}
			}
		}
	}
	return API_ERROR_OK;
}
