/** \file */
#ifndef __AD916XAPI_H__
#define __AD916XAPI_H__
#include <linux/kernel.h>
#include <linux/bug.h>

#include "api_def.h"
#include "api_config.h"


/** AD916x Device Identification Data */
typedef struct {
	uint8_t chip_type;      /*Chip Type Code */
	uint16_t prod_id;       /*Product ID Code */
	uint8_t prod_grade;     /*Product Grade Code*/
	uint8_t dev_revision;   /*Device Revision */
}ad916x_chip_id_t;

/** JESD Interface Link Status */
typedef struct {
	/*Bit wise Code Group Sync Status for all JESD Lanes*/
	uint8_t code_grp_sync_stat;
	/*Bit wise Frame Sync Status for all JESD Lanes*/
	uint8_t frame_sync_stat;
	/*Bit wise Good Checksum Status for all JESD Lanes*/
	uint8_t good_checksum_stat;
	/*Bit wise Initial Lane Sync Status for all JESD Lanes*/
	uint8_t init_lane_sync_stat;
}ad916x_jesd_link_stat_t;

/** Enumerates SYSREF Synchronization Mode */
typedef enum {
	SYSREF_NONE,     /**< No SYSREF Support */
	SYSREF_ONESHOT,  /**< ONE-SHOT SYSREF */
	SYSREF_CONT,     /**< Continuous Sysref Synchronisation */
	SYSREF_MON,      /**< SYSREF monitor Mode */
	SYSREF_MODE_INVLD
}ad916x_jesd_sysref_mode_t;

/** Enumerates JESD Configuration Error Flags*/
typedef enum {
	INTPL_FACTOR_INVLD = 0x1,   /**< Invalid Interpolation Factor Set*/
	SUBCLASS_V_INVLD = 0x2,     /**< Illegal Subclass V used*/
	K_INVLD = 0x4,              /**< Illegal K is used */
	LMFS_INPOL_INVLD = 0x8,     /**< Unsupported LMFS for Interpolation factor*/
	LMFC_DELAY_INVLD = 0x10,    /**< Illegal LMFC delay used*/
	SYSREF_JTTR_WIN_INVLD =0x20 /**< Illegal SYSREF Jitter window used*/
}ad916x_jesd_cfg_err_flg_t;

/** Enumerates SERDES PLL Status Flags*/
typedef enum {
	PLL_LOCK_STAT = 0x1,      /**< Serdes PLL lock Status Flag*/
	PLL_CAL_STAT = 0x8,       /**< Serdes PLL Calibration Status Flag*/
	PLL_UPP_CAL_THRES =0x10,  /**< Serdes PLL Upper Calibration Threshold flag*/
	PLL_LWR_CAL_THRES =0x20   /**< Serdes PLL lower Calibration Threshold flag*/
}ad916x_jesd_serdes_pll_flg_t;

/** AD916x Device Identification Data */
typedef enum {
	TX_ENABLE,               /*Data Transmit Mode*/
	TX_ZERO_DATA_DAC,        /*Zero Data at DAC*/
	TX_ZERO_DATA_IN_PATH     /*Zero Data at input to datapath*/
}ad916x_transmit_mode_t;

/** Enumerates all available functions for the TX_ENABLE pin */
typedef enum {
	/** No function is selected */
	NONE,
	/** Use TX_ENABLE to reset the NCO */
	RESET_NCO,
	/** Use TX_ENABLE pin to zero data to the DAC */
	ZERO_DATA_DAC,
	/** Use TX_ENABLE to zero data at input to the datapath */
	ZERO_DATA_IN_PATH,
	/** Use TX_ENABLE to control Full Scale Current*/
	CURRENT_CONTROL
}ad916x_tx_enable_pin_mode_t;

/** Results from the JESD PRBS test. */
typedef struct {
	uint32_t phy_prbs_err_cnt; /**< PRBS Test Error Count */
	uint8_t phy_prbs_pass;     /**< PRBS Test Status */
	uint8_t phy_src_err_cnt;   /**< PRBS Test Source Error Count */
}ad916x_prbs_test_t;

/** Enumerates Interrupts Available */
typedef enum {
	EVENT_SYSREF_JITTER,         /**< SYSREF JITTER Threshold Event */
	EVENT_DATA_RDY,              /**< DATA Ready Event */
	EVENT_JESD_LANE_FIFO_ERR,    /**< JESD LANE FIFO Underflow/overflow Event */
	EVENT_JESD_PRBS_IMG_ERR,      /**< PRBS Q Error Event */
	EVENT_JESD_PRBS_REAL_ERR,     /**< PRBS I Error Event */
	EVENT_JESD_BAD_DISPARITY_ERR, /**< Bad Disparity Error Event */
	EVENT_JESD_NOT_IN_TBL_ERR,    /**< Not-In-Table Error Event */
	EVENT_JESD_K_ERR,    /**< Unexpected K Error Event */
	EVENT_JESD_ILD_ERR,  /**< Inter-lane De-Skew Event */
	EVENT_JESD_ILS_ERR,  /**< Initiali Lane Sync Event */
	EVENT_JESD_CKSUM_ERR,/**< Good Checksum Event */
	EVENT_JESD_FS_ERR,   /**< Frame Synchronization Event */
	EVENT_JESD_CGS_ERR,  /**< Code Group Synchronization Event */
	EVENT_JESD_ILAS_ERR, /**< ILAS Configuration Mismatch Event */
	EVENT_NOF_EVENTS
}ad916x_event_t;

/** AD916x API handle */
typedef struct {
	void *user_data;        /**< Void pointer to user defined data for HAL initialization */
	spi_sdo_config_t sdo;   /**< DAC SPI interface configuration*/
	uint64_t dac_freq_hz;   /**< DAC Clock Frequency in Hz. Valid range 850MHz to 6GHz*/
	spi_xfer_t dev_xfer;    /**< Function Pointer to HAL SPI access function*/
	delay_us_t delay_us;    /**< Function Pointer to HAL delay function*/
	event_handler_t event_handler; /**< Function Pointer to Client Event Handler */
	tx_en_pin_ctrl_t tx_en_pin_ctrl; /**< Function Pointer to HAL TX_ENABLE Pin Ctrl function*/
	reset_pin_ctrl_t reset_pin_ctrl; /**< Function Point to HAL RESETB Pin Ctrl Function*/
	hw_open_t hw_open;      /**< Function Pointer to HAL initialization function*/
	hw_close_t hw_close;    /**< Function Pointer to HAL de-initialization function*/
}ad916x_handle_t;

/**
 * \brief Initialize AD916x Device
 * This API must be called first before any other API calls.
 * It performs internal API initialization of the memory and API states.
 * If AD916x handle member hw_open is not NULL it shall call the function
 * to which it points. This feature may be used to get and initialize the
 * hardware resources required by the API and the AD916x devices.
 * For example GPIO, SPI etc.
 *
 * Its is recommended to call the Reset API after this API to ensure all
 * spi registers are reset to ADI recommended defaults.
 *
 * \param h Pointer to the AD916x device reference handle.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_HANDLE_PTR  Invalid reference handle
 * \retval API_ERROR_INIT_SEQ_FAIL Init sequence failed
 * \retval API_ERROR_SPI_SDO    Invalid SPI configuration
 */
ADI_API int ad916x_init(ad916x_handle_t *h);

/**
 * \brief De-initialize the AD916x Device.
 *
 * This API must be called last. No other API should be called after this call.
 * It performs internal API de-initialization of the memory and API states.
 * If AD916x handle member hw_close, is not NULL it shall call the function
 * to which it points. This feature may be used to de-initialize and release
 * any hardware resources required by the API and the AD916x Device.
 * For example GPIO, SPI etc
 *
 * \param h Pointer to the AD916x device reference handle.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR   Invalid reference handle
 */
ADI_API int ad916x_deinit(ad916x_handle_t *h);

/**
 * \brief Reset the AD916x
 *
 * Issues a hard reset or soft reset of the device.
 * Performs a full reset of AD916x via the hardware pin (hard) or
 * via the spi register (soft).
 * Resetting all SPI registers to default and triggering the required
 * initialization sequence.
 *
 * \param h         Pointer to the AD916x device reference handle.
 * \param hw_reset  A parameter to indicate if the reset issues is to be via the
 *                  hardware pin or SPI register.
 *                  A value of 1 indicates a hardware reset is required.
 *                  A value of 0 indicates a software reset is required.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR   Invalid reference handle
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 * \retval API_ERROR_INIT_SEQ_FAIL Init sequence failed
 * \retval API_ERROR_SPI_SDO    Invalid SPI configuration
 */
ADI_API int ad916x_reset(ad916x_handle_t *h, uint8_t hw_reset);

/**
 * \brief Get Chip Identification Data
 *
 * read-back Product type, identification and revision data
 *
 * \param h          Pointer to the AD916x device reference handle.
 * \param chip_id    Pointer to a variable of type  ad916x_chip_id_t to
 *                   which the product data read-back from chip shall be stored.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_get_chip_id(ad916x_handle_t *h, ad916x_chip_id_t *chip_id);

/**
 * \brief Set Interrupt Events
 *
 * Configure the events that trigger the interrupt signal
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param event_list  Pointer to an array of AD916x events.
 *                    Events are enumerated by ad916x_event_t
 * \param en          Enable/Disable parameter to indicate whether to enable or
 *                    disable the events listed by the event_list parameter.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_set_events(ad916x_handle_t *h,
				ad916x_event_t *event_list, uint8_t list_size, uint8_t en);

/**
 * \brief Get Interrupt Pending Status
 *
 * Get interrupt status to indicate if any interrupts have triggered and
 * are currently pending.
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param int_pending   Pointer uint8 variable where the interrupt
 *                      status shall be stored.
 *                      1 indicates at lease one interrupt event has occurred.
 *                      0 indicates no interrupt events have occurred.
 * \param int_status    Pointer to an 3-deep array of uint8_t to which
 *                      the interrupt status registers shall be stored.
 *                      May be set to null if data not required.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_get_interrupt_status(ad916x_handle_t *h,
									uint8_t *int_pending, uint8_t *int_status);

/**
 * \brief Interrupt Service Routine
 *
 * Implements a simple interrupt Service Routine to check, clear
 * and handle interrupts. In addition it shall notify the client application
 * of events if an event_handler function has been provided.
 * If the event_handler function is not provided the API shall simply
 * clear any detected interrupts.
 *
 * \param h   Pointer to the AD916x device reference handle.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_isr(ad916x_handle_t *h);

/**
 * \brief Set DAC Clock Frequency
 *
 * Set the software dac clock frequency value for the API.
 * This should be equal to the hardware DAC clock provided to the DAC.
 *
 * \param h          Pointer to the AD916x device reference handle.
 * \param dac_clk_hz DAC clock Frequency in Hz. Valid range 850Mhz to 6GHz
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_dac_set_clk_frequency(ad916x_handle_t *h,
								uint64_t dac_clk_freq_hz);

/**
 * \brief Get DAC Clock Frequency
 *
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param dac_clk_hz  Pointer to a uint64_t variable where the current
 *                    DAC clock frequency configured shall be stored
 *                    The frequency value shall be returned Hz.
 *                    Valid range 850Mhz to 6GHz
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_dac_get_clk_frequency(ad916x_handle_t *h,
								uint64_t *dac_clk_freq_hz);

/**
 * \brief Adjust Full Scale Output Current
 *
 * Adjust the full scale current output.
 * Fullscale can be adjusted over a range of 8mA to 40mA
 *
 * \param h        pointer to the AD916x device reference handle.
 * \param current  Desired full scale output current. Valid range 8 to 40
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_dac_set_full_scale_current(ad916x_handle_t *h,
										uint8_t _current);

/**
 * \brief Configure the JESD Interface for AD916x
 *
 * Configure JESD interface parameters and digital datapath interpolation mode.
 * An error shall be returned if the input parameters define an
 * unsupported configuration. Determination of valid configuarion depends on the
 * DAC CLK Frequency configured. This should be correctly configured prior
 * to calling this API.
 *
 * \param h                Pointer to the AD916x device reference handle.
 * \param jesd_param       The desired JESD link parameters for which to
 *                         configure the JESD Interface to AD916x.
 *                         jesd_param_t structure defines the parameter their
 *                         ranges supported by the AD916x.
 * \param interpolation    The desired interpolation mode.
 *                         Valid interpolation modes are: 1,2,3,4,6,8,12,16,24
 * \param *lane_rate_mbps  Pointer to uint64_t to return the determined:
 *                         Lane Rate. Set to NULL if not required.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_config_datapath(ad916x_handle_t *h,
							jesd_param_t jesd_param, uint8_t interpolation,
							uint64_t *lane_rate_mbps);

/**
 * \brief Read back all current JESD parameter settings.
 *
 * Read-back all the currently configured JESD Interface parameters.
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param jesd_param  Pointer to a variable that will be set will the
 *                    current values of the JESD interface parameters.
 *
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_cfg_param(ad916x_handle_t *h,
										jesd_param_t *jesd_param);

/**
 * \brief Configure the SysRef Mode for the JESD Interface
 *
 * Configure AD916x for the desired type of SYSREF synchronization signal
 *
 * \param h               Pointer to the AD916x device reference handle.
 * \param mode            The format for the SYSREF synchronization signal for
 *                        the JESD link to the AD916x. The mode is defined by
 *                        the ad916x_jesd_sysref_mode_t enumeration.
 * \param jitter_window   SYSREF jitter Window Tolerance.
 *                        Valid values and their respective tolerance in
 *                        DAC clk cycles are as follows:
 *                        0  => +/- 0.5 DAC Clock Cycles
 *                        4  => +/- 4 DAC Clock Cycles
 *                        8  => +/- 8 DAC Clock Cycles
 *                        12 => +/- 16 DAC Clock Cycles
 *                        16 => +/- 16 DAC Clock Cycles
 *                        20 => +/- 20 DAC Clock Cycles
 *                        24 => +/- 24 DAC Clock Cycles
 *                        28 => +/- 28  DAC Clock Cycles
 *                        Valid only in Monitor Mode.
 *                        This parameter ignored in all other modes.
 *
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_set_sysref_mode(ad916x_handle_t *h,
						ad916x_jesd_sysref_mode_t mode, uint8_t jitter_window);

/**
 * \brief Get the currently configured SysRef Mode
 *
 * Get the current setting for SYSREF synchronization signal
 * for the JESD Interface
 *
 * \param h               Pointer to the AD916x device reference handle.
 * \param mode            Pointer to the variable where the format for
 *                        the SYSREF synchronization signal for the JESD link
 *                        shall be stored.
 * \param sysref_jttr_win Pointer to the variable where the SYSREF jitter
 *                        Window Tolerance value shall be stored.
 *                        Valid tolerances are enumerated by
 *                        jesd_sysref_jttr_win_tol.
 *
 *                        Valid values and their respective tolerance in
 *                        DAC clk cycles are as follows:
 *                        0  => +/- 0.5 DAC Clock Cycles
 *                        4  => +/- 4 DAC Clock Cycles
 *                        8  => +/- 8 DAC Clock Cycles
 *                        12 => +/- 16 DAC Clock Cycles
 *                        12 => +/- 16 DAC Clock Cycles
 *                        20 => +/- 20 DAC Clock Cycles
 *                        24 => +/- 24 DAC Clock Cycles
 *                        28 => +/- 28  DAC Clock Cycles
 *                        Valid only in Monitor Mode.
 *                        This parameter ignored in all other modes.
 *
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_sysref_mode(ad916x_handle_t *h,
					ad916x_jesd_sysref_mode_t *mode, uint8_t *sysref_jttr_win);

/**
 * \brief Read back SYSREF and LMFC Synchronization Status
 *
 * Get the current synchronization data for SYSREF synchronization.
 * This data can be used to determine Dynamic Link Latency Settings in
 * SYSREF Monitor Mode.
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param sysref_count  Pointer to the variable where the detected SYSREF
 *                      count shall be stored.
 *                      Set to NULL to ignore this read-back.
 * \param sysref_phase  Pointer to the variable where the SYSREF phase used
 *                      to sample sysref signals shall be stored.
 *                      Set to NULL to ignore this read-back.
 *
 * \param sync_lmfc_stat Pointer to the variable where the LMFC status
 *                       shall be stored. This value may be used to determine 
 *                       synchronization status.
 *                       A value of 0 to 4 indicated valid synchronization.
 *                       Set to NULL to ignore this read-back.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_sysref_status(ad916x_handle_t *h,
							uint8_t *sysref_count, uint16_t *sysref_phase,
							uint16_t *sync_lmfc_stat);

/**
 * \brief Read back Dynamic Link Latency
 *
 * Read Back the Dynamic Link Latency.
 *
 * \param h         Pointer to the AD916x device reference handle.
 * \param latency   Pointer to the variable where the dynamic latency
 *                  provided in units of pclks shall be stored.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_dynamic_link_latency(ad916x_handle_t *h,
													uint8_t *latency);

/**
 * \brief Adjust the LMFC Delay and LMFC Variance
 *
 *
 *
 * \param h         Pointer to the AD916x device reference handle.
 * \param delay     A 5-bit value to be set as the global LMFC delay
 *                  in units of Frame Clock Cycles.
 * \param var       A 5-bit value to be set as the variable LMFC delay value.
 *
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_set_lmfc_delay(ad916x_handle_t *h,
									uint8_t delay, uint8_t var);

/**
 * \brief Configure the Lane Cross Bar in the JESD datalink layer
 *
 * Configure AD916x Lane Cross Bar to route the physical JESD lanes
 * to the desired logical lanes.
 *
 * \param h              Pointer to the AD916x device reference handle.
 * \param physical_lane  uint8_t value indicating the Physical Lanes
 *                       to be routed to the serdes logical indicated
 *                       by the logical_lane parameter
 * \param logical_lane   uint8_t value indicating the corresponding logical
 *                       lane for the physical lane listed
 *                       in parameter physical_lane.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_set_lane_xbar(ad916x_handle_t *h,
								uint8_t physical_lane, uint8_t logical_lane);

/**
 * \brief Get current Lane Cross Bar configuration for the JESD datalink layer
 *
 * Return the physical to logical lane mapping set by the configured by the
 * current Lane Cross Bar configuration.
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param phy_log_map   Pointer a 8 deep uint8_t array.Each element of the array
 *                      represents the physical lane 0 - 7 and
 *                      the value represents the logical lane assigned to
 *                      that physical lane.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_lane_xbar(ad916x_handle_t *h, uint8_t *phy_log_map);

/**
 * \brief Invert or un-invert logical lanes
 *
 * Each logical lane can be inverted which can be used to ease
 * routing of SERDIN signals.
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param logical_lane  Logical lane ID to be inverted. 0 to 7
 * \param invert        Desired invert status for the logical lane
 *                      represented in logical_lane parameter.
 *                      Set to 1 to invert.
 *                      Set to 0 to de-invert.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_invert_lane(ad916x_handle_t *h,
									uint8_t logical_lane, uint8_t invert);

/**
 * \brief Enable the de-scrambler for the JESD Interface
 *
 * Enable or Disable the descrambler for the JESD Interface
 *
 * \param h   Pointer to the AD916x device reference handle.
 * \param en  Enable control for JESD Scrambler.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_enable_scrambler(ad916x_handle_t *h, uint8_t en);

/**
 * \brief Get JESD Configuration Error Status
 *
 * Return status of 6 JESD configuration Error Flags.
 *
 * \param h              Pointer to the AD916x device reference handle.
 * \param jesd_cfg_stat  JESD configuration status flag. Each bit in status byte
 *                       represents a status flag enumerated by jesd_cfg_err_flg
 *                       enumerations.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_cfg_status(ad916x_handle_t *h,
										uint8_t *jesd_cfg_stat);

/**
 * \brief Enable the JESD Interface
 *
 * Configure power up and enable the AD916x the JESD Interface
 *
 * \param h         Pointer to the AD916x device reference handle.
 * \param lanes_msk Lanes to be enabled on JESD Interface.
 *                  8-bit mask where bit 0 represents lane 0,
 *                  bit 1 represents lane 1 etc.
 *                  Set to 1 to enable JESD lane, set to 0 to disable JESD Lane.
 * \param run_cal   Run JESD interface calibration.
 * \param en        Enable control for the JESD Interface
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_enable_datapath(ad916x_handle_t *h,
						uint8_t lanes_msk, uint8_t run_cal, uint8_t en);

/**
 * \brief  Enable JESD Link
 *
 * Enable SERDES Link to start the bring up JESD Link procedure
 *
 * JESD Transmitter Link shall be enabled and ready to begin link bring
 * prior to calling this function
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param en    Enable control for the JESD Interface
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_enable_link(ad916x_handle_t *h, uint8_t en);

/**
 * \brief  Get SERDES PLL Status
 *
 * Read Serdes PLL status and return the status via the
 * pll_status parameter.
 *
 *
 * \param h            Pointer to the AD916x device reference handle.
 * \param *pll_status  Pointer to the variable that will be set with
 *                     the PLL status.
 *                     bit[0] => PLL Lock Status
 *                     bit[3] => PLL calibration Status
 *                     bit[4] => PLL Upper Threshold Status
 *                     bit[5] => PLL lwr Threshold Status
 *
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_pll_status(ad916x_handle_t *h, uint8_t *pll_status);

/**
 * \brief  Get JESD Link Status
 *
 * Read-back JESD Status for all lanes. JESD status include
 * Code Group Sync Status, Frame Sync Status, Checksum Status
 * Initial Lane Sync Status for the active JESD link.
 *
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param *link_status  Pointer to the variable of type jesd_link_status
 *                      that will be set with current jesd link read-back data.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_get_link_status(ad916x_handle_t *h,
										ad916x_jesd_link_stat_t *link_status);

/**
 * \brief  Run PRBS Test for JESD Receiver
 *
 * The JESD204B receiver on the AD916x includes a PRBS pattern
 * checker on the back end of its physical layer. This functionality
 * enables bit error rate (BER) testing of each physical lane of the
 * JESD204B link. The PHY PRBS pattern checker does not
 * require that the JESD204B link be established. It can synchronize
 * with a PRBS7, PRBS15, or PRBS31 data pattern. PRBS pattern
 * verification can be done on multiple lanes at once. The error
 * counts for failing lanes are reported for one JESD204B lane at a
 * time
 *
 * \param h              Pointer to the AD916x device reference handle.
 * \param prbs_pattern   PRBS pattern identifier, valid values are enumerated
 *                       by jesd_prbs_pattern_t
 * \param lanes_en       A 8-bit mask value to indicate the lanes enable
 *                       and on which to run the PRBS tests.
 *                       Bit 0 represents lane 0,
 *                       Bit 1 represents lane 1 etc.
 *                       Set to 1 to enable JESD lane.
 *                       Set to 0 to disable JESD Lane.
 * \param prbs_error_threshold
 *                       A 32-Bit value representing the error threshold pass
 *                       criteria for the test.
 * \param result         A point to a structure of type ad916x_prbs_test_t
 *                       in which the test result and data on the errors
 *                       shall be stored.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR Invalid reference handle.
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_jesd_phy_prbs_test(ad916x_handle_t *h,
                                 const jesd_prbs_pattern_t prbs_pattern,
                                 const uint8_t lanes_en,
                                 const uint32_t prbs_error_threshold,
                                 ad916x_prbs_test_t *result);
/**
 * \brief Configure TX_ENABLE pin functionality
 *
 * TX_ENABLE pin can be configured so influence the data for tx-disable mode
 * different points in the datapath.
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param tx_en_func
 *              Desired functionality of th TX_ENABLE Pin.
 *              Valid options are enumerated by ad916x_tx_enable_pin_mode_t
 *              NONE
 *                 No functionality is assigned to the TX_ENABLE pin
 *              RESET_NCO
 *                 NCO is reset based on status of TX_ENABLE pin
 *              ZERO_DATA_DAC
 *                 Data to DAC is zeroed based on status of TX_ENABLE pin
 *              ZERO_DATA_IN_PATH
 *                 Data in datapath is zeroed based on status of TX_ENABLE pin.
 *              CURRENT_CONTROL
 *                 Full scale current control is influenced by TX_ENABLE pin.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_transmit_enable_pin(ad916x_handle_t *h,
								const ad916x_tx_enable_pin_mode_t tx_en_func);

/**
 * \brief Configure the transmit data operation.
 *
 * Controls the Transmit Data operation mode. Configures how data from the 
 * DAC datapath is transmitted.
 *
 * \param h    Pointer to the AD916x device reference handle.
 * \param mode Enable DAC transmit function as per the desired mode
 *             Supported modes are enumerated by ad916x_transmit_mode_t
 *             TX_ENABLE Enable
 *                Transmit functionality. Transmit DAC data
 *             TX_ZERO_DATA_DAC
 *                Disable transmit by zeroing data at DAC,
 *             TX_ZERO_DATA_IN_PATH
 *                Disable transmit by zeroing data at input to datapath
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_transmit_enable(ad916x_handle_t *h,
							const ad916x_transmit_mode_t mode);

/**
 * \brief Enable and configure Clock Duty Cycle Adjustment
 *
 * \param h       Pointer to the AD916x device reference handle.
 * \param en      Enable Duty Cycle offset Adjustment
 *                Set to 1 to enable.Set to 0 to disable
 * \param offset  Duty cycle offset parameter. This parameter sould be set to a 
 *                5-bit value in two's complement format to indicate a positive 
 *                or negative skew.
 *                A larger value will result in a larger clock duty skew.
 *
 * \retval API_ERROR_OK                 API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR   SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM      Invalid Parameter
 */
ADI_API int ad916x_clk_adjust_dutycycle(ad916x_handle_t *h,
										uint8_t en, uint8_t offset);

/**
 * \brief Adjust Phase of the CLK + and CLK- inputs.
 *
 * \param h      Pointer to the AD916x device reference handle.
 * \param pol    Clock Polarity input.
 *               Set to 1 to apply phase adjustment to CLK- Input
 *               Set to 0 to apply phase adjustment to CLK+ Input
 * \param phase  Desired number of phase adjustments steps,
 *               The valid range fot the number of phase adjustment
 *               steps is 0 to 31.
 *               Each step adds a capacitance of 20fF
 *
 * \retval API_ERROR_OK                 API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR   SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM      Invalid Parameter
 */
ADI_API int ad916x_clk_adjust_phase(ad916x_handle_t *h,
									uint8_t pol, uint8_t phase);

/**
 * \brief Enable/Disable Clock Cross Control
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param en          Cross Control Enable.
 *                    1 = Enable Clock Cross Control
 *                    0 = Disable Clock Cross Control
 * \param cross_point A 3-bit value representing the the Clock cross control
 *                    crossing point.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_clk_set_cross_ctrl(ad916x_handle_t *h,
									 uint8_t en, uint8_t cross_point);
/**
 * \brief Enable FIR -85 for 2xNRZ mode operation.
 *
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param en    Enable FIR-85 filter.
 *              1 = Enable FIR 85.
 *              0 = Disable FIR 85
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_fir85_set_enable(ad916x_handle_t *h, const int en);

/**
 * \brief Get FIR -85 for 2xNRZ mode operation enable status.
 *
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param en    Pointer to a variable to which the FIR-85 Enable Filter status
 *              shall be stored.
 *              1 = Enable FIR 85.
 *              0 = Disable FIR 85
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_fir85_get_enable(ad916x_handle_t *h, int *en);

/**
 * \brief Enable Interpolation Filter BW mode to 90% BW
 *
 * By default the interpolation filters in the datapath are in a
 * low power 80% bandwidth mode. By enabling the High Bandwidth filter mode.
 * The Interpolation filter bandwidth will be set to 90% BW.
 *
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param en    Enable high bandwidth mode.
 *              1 = Set interpolation filters to 90% BW.
 *              0 = Set interpolation filters to 80% BW
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_filt_bw90_enable(ad916x_handle_t *h, const int en);

/**
 * \brief Enable the Inverse sin c FIR filter.
 *
 * API to enable to enable the inverse sin c filter and apply it to
 * the datapath.
 *
 * \param h     Pointer to the AD916x device reference handle.
 * \param en    Enable Inverse Sin c Filter paramter.
 *              0 = Disable the inverse sin c filter.
 *              1 = Enable the inverse sin c filter.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_invsinc_enable(ad916x_handle_t *h, const int en);

/**
 * \brief Set Frequency tuning word (FTW), MODULUS
 *        and DELTA parameters for a particular NCO
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param nco_nr      Always set to 0. All other values are invalid.
 * \param ftw         Frequency Tuning Word value. For the AD916x NCO
 *                    this parameter is 48 bit.
 * \param acc_modulus MODULUS value.
 * \param acc_delta   DELTA value.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 * \retval API_ERROR_FTW_LOAD_ACK FTW has not been loaded. This check is only for the main NCO (NCO 0).
 * \retval API_ERROR_INVALID_PARAM NCO number is invalid.
 */
ADI_API int ad916x_nco_set_ftw(ad916x_handle_t *h, const unsigned int nco_nr,
								const uint64_t ftw, const uint64_t acc_modulus,
								const uint64_t acc_delta);

/**
 * \brief Get the Frequency Tuning word (FTW), MODULUS
 *         and DELTA parameters for a particular NCO
 *
 * \param h           Pointer to the AD916x device reference handle.
 * \param nco_nr      Always set to 0. All other values are invalid.
 * \param ftw         Pointer to variable where the FTW value will be stored.
 * \param acc_modulus Pointer to variable where the MODULUS value will be stored.
 * \param acc_delta   Pointer to variable where the DELTA value will be stored.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 * \retval API_ERROR_INVALID_PARAM NCO number is invalid, or 'ftw' pointer is invalid.
 *
 */
ADI_API int ad916x_nco_get_ftw(ad916x_handle_t *h, const unsigned int nco_nr,
								uint64_t *ftw, uint64_t *acc_modulus,
								uint64_t *acc_delta);

/**
 * \brief Set the NCO phase offset.
 *
 * \param h  Pointer to the AD916x device reference handle.
 * \param po Phase offset.
 *
 * \retval API_ERROR_OK API Completed Successfully.
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 */
ADI_API int ad916x_nco_set_phase_offset(ad916x_handle_t *h, const uint16_t po);

/**
 * \brief Get the NCO phase offset.
 *
 * \param h  Pointer to the AD916x device reference handle.
 * \param po Pointer to uint16_t variable where the phase offset will be stored.
 *
 * \return API error code.
 * \retval API_ERROR_OK API Completed Successfully.
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 * \retval API_ERROR_INVALID_PARAM Provided parameter is invalid pointer.
 *
 */
ADI_API int ad916x_nco_get_phase_offset(ad916x_handle_t *h, uint16_t *po);

/**
 * \brief Set the Enable status of the main NCO and modulus if desired.
 *
 * \param h          Pointer to the AD916x device reference handle.
 * \param modulus_en Enable dual modulus NCO.
 *                   1 = Enable dual Modulus mode.
 *                   0 = Disable dual Modulus mode
 * \param nco_en     Enable NCO
 *                   1 = Enable NCO
 *                   0 = Disable NCO
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_nco_set_enable(ad916x_handle_t *h,
									const int modulus_en, const int nco_en);

/**
 * \brief Get the Enable status of the main NCO and modulus if desired.
 *
 * \param h          Pointer to the AD916x device reference handle.
 * \param modulus_en Pointer to int variable where the modulus status
 *                   will be stored.
 *                   1 = Enable dual Modulus mode.
 *                   0 = Disable dual Modulus mode
 * \param nco_en     Pointer to int variable where the main NCO enable
 *                   status will be stored.
 *                   1 = Enable NCO
 *                   0 = Disable NCO
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM Provided parameter is invalid pointer.
 */
ADI_API int ad916x_nco_get_enable(ad916x_handle_t *h,
									int *modulus_en, int *nco_en);

/**
 *  \brief Set DC TEST data and enable status.
 *
 *  Configures and enables the DC Test Mode. DC Test mode uses the
 *  NCO only operation without data from the jesd interface.
 *  The DC Test mode is only available on the AD9162.
 *
 *  \param h         Pointer to the AD916x device reference handle.
 *  \param test_data DC Test data value.
 *  \param en        DC Test mode enable.
 *                   0: Disable,
 *                   else: Enable
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 *
 */
ADI_API int ad916x_dc_test_set_mode(ad916x_handle_t *h,
									const uint16_t test_data, const int en);

/**
 * \brief Get DC TEST data and enable status.
 *
 *  This API gets the currently configured status of DC Test feature.
 *  The DC Test mode is only available on the AD9162.
 *
 * \param h         Pointer to the AD916x device reference handle.
 * \param test_data DC Test data value.
 * \param en        DC Test mode enable.
 *                  0: Disable,
 *                  else: Enable
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_HANDLE_PTR API handle pointer parameter is invalid.
 * \retval API_ERROR_INVALID_PARAM Provided parameter is invalid pointer.
 */
ADI_API int ad916x_dc_test_get_mode(ad916x_handle_t *h,
										uint16_t *test_data, int *en);

/**
 * \brief Configure NCO Parameters
 *
 * Configure NCO parameters. Set the carrier frequency.
 * For the AD916x family NCO reference number will always be 0.
 * Option to enable the DC TEST Mode is valid for AD9162 only.
 *
 * \param h               Pointer to the AD916x device reference handle.
 * \param nco_nr          NCO number. Always set to 0
 * \param carrier_freq_hz Desired Carrier Frequency in Hz
 * \param amplitude       Desired Amplitude for dc_test_mode data. AD9162 Only
 * \param dc_test_en      Enable for the DC TEST MODE feature. AD9162 Only.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_nco_set(ad916x_handle_t *h, const unsigned int nco_nr,
							const int64_t carrier_freq_hz,
							const uint16_t amplitude, int dc_test_en);

/**
 * \brief Get current NCO configuration
 *
 * Get current carrier frequency and amplitude for the choosen NCO.
 * For the AD916x family NCO reference number will always be 0.
 * Amplitude and dc_test_en data is only valid for AD9162.
 *
 * \param h                 Pointer to the AD916x device reference handle.
 * \param nco_nr            NCO number. Always set to 0
 * \param carrier_freq_hz   Carrier Frequency in Hz
 * \param amplitude         Amplitude for dc_test_mode data. AD9162 Only
 * \param dc_test_en        Enable status for the DC TEST MODE feature.
 *                          AD9162 Only.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_nco_get(ad916x_handle_t *h, const unsigned int nco_nr,
				int64_t *carrier_freq_hz, uint16_t *amplitude, int *dc_test_en);

/**
 * \brief Reset NCO
 *
 * Reset the NCO via SPI register. NCO accumulater resets and begins
 * counting at the latest FTW.
 *
 * \param h Pointer to the AD916x device reference handle.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_nco_reset(ad916x_handle_t *h);

/**
 * \brief Perform SPI register write access to AD916x Device
 *
 * \param h        Pointer to the AD916x device reference handle.
 * \param address  AD916x Device SPI address to which the value of data
 *                 parameter shall be written
 * \param data     8-bit value to be written to SPI register defined
 *                 by the address parameter.
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_register_write(ad916x_handle_t *h,
						const uint16_t address, const uint8_t data);
/**
 * \brief Perform SPI register read access to AD916x Device.
 *
 *
 * \param h        Pointer to the AD916x device reference handle.
 * \param address  AD916x Device SPI address from which the value of data
 *                 parameter shall be read,
 * \param data     Pointer to an 8-bit variable to which the value of the
 *                 SPI register at the address defined by address parameter
 *                 shall be stored.
 *
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_XFER_PTR SPI Access Failed
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_register_read(ad916x_handle_t *h,
                                const uint16_t address, uint8_t *data);
/**
 * \brief Get API Revision Data
 *
 * \param h             Pointer to the AD916x device reference handle.
 * \param rev_major     Pointer to variable to store the Major Revision Number
 * \param rev_minor     Pointer to variable to store the Minor Revision Number
 * \param rev_rc        Pointer to variable to store the RC Revision Number
 *
 * \retval API_ERROR_OK API Completed Successfully
 * \retval API_ERROR_INVALID_PARAM    Invalid Parameter
 */
ADI_API int ad916x_get_revision(ad916x_handle_t *h, uint8_t *rev_major,
								uint8_t *rev_minor, uint8_t *rev_rc);

#endif /* !__AD916XAPI_H__ */
