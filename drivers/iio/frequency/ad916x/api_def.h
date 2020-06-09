#ifndef __API_DEF_H__
#define __API_DEF_H__
#include <linux/kernel.h>

#include <linux/math64.h>

#if __GNUC__
#ifndef __UINT64_TYPE__
#error "This compiler doesn't support 64 bit integer type"
#endif
#else
#error "This platform is not supported"
#endif
#define ADI_POW2_48 ((uint64_t)1u<<48)
#define ADI_MAXUINT48 (ADI_POW2_48 - 1)

#define ADI_POW2_32 ((uint64_t)1u<<32)
#define ADI_MAXUINT32 (ADI_POW2_32 - 1)
#define ADI_MAXUINT24 (0xFFFFFF)
#define ADI_GET_BYTE(w, p) (uint8_t)(((w) >> (p)) & 0xFF)
#define DIV_U64(x, y) div_u64(x, y)
#define DIV64_U64(x, y) div64_u64(x, y)

#define INT16_MAX S16_MAX


/**
 * \brief Platform dependent SPI access functions.
 *
 *
 * \param indata  Pointer to array with the data to be sent on the SPI
 * \param outdata Pointer to array where the data to which the SPI will be written
 * \param size_bytes The size in bytes allocated for each of the indata and outdata arrays.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note indata and outdata arrays are of same size.
 */
typedef int(*spi_xfer_t)(void *user_data, uint8_t *indata, uint8_t *outdata, int size_bytes);

/**
 * \brief Delay for specified number of microseconds. Platform Dependant.
 *
 * Performs a blocking or sleep delay for the specified time in microseconds
 * The implementation of this function is platform dependant and
 * is required for correct operation of the API.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \param us - time to delay/sleep in microseconds.
 */
typedef int(*delay_us_t)(void *user_data, unsigned int us);

/**
 * \brief Platform hardware initialisation for the AD9164 Device
 *
 * This function shall initialize all external hardware resources required by
 * the ADI Device and API for correct functionatlity as per the 
 * target platform.
 * For example initialisation of SPI, GPIO resources, clocks etc.
 *
 *
 * \param *user_data - A void pointer to a client defined structure containing any
 *             parameters/settings that may be required by the function
 *             to initialise the hardware for the ADI Device.
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 */
typedef int(*hw_open_t)(void *user_data);

/**
 * \brief Closes any platform hardware resources for the AD9164 Device
 *
 * This function shall close or shutdown all external hardware resources 
 * required by the AD9164 Device and API for correct functionatlity 
 * as per the target platform.
 * For example initialisation of SPI, GPIO resources, clocks etc.
 * It should close and free any resources assigned in the hw_open_t function.
 *
 * \param *user_data - A void pointer to a client defined structure containing any
 * 				parameters/settings that may be required by the function
 * 				to close/shutdown the hardware for the ADI Device.
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 */
typedef int(*hw_close_t)(void *user_data);
/**
 * \brief Client  Event Handler
 *
 *
 * \param event	A uint16_t value representing the event that occurred.
 * \param ref   A uint8_t value indicating the reference for that event if any.
 * 				For example 0 if even occured on lane 0.
 * \param data  A void pointer to any user data that may pertain to that event.
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*event_handler_t)(uint16_t event, uint8_t ref, void* data);

/**
 * \brief TX_ENABLE PIN CONTROL FUNCTION
 *
 *
 * \param *user_data  A void pointer to a client defined structure containing
 * 					any parameters/settings that may be required by the function
 * 					to control the hardware for the ADI Device TX_ENABLE PIN.
 * \param enable   A uint8_t value indicating the desired enable/disable
 * 					setting for the tx_enable pin.
 * 					A value of 1 indicates TX_ENABLE pin is set HIGH
 * 					A value of 0 indicates TX_ENABLE pin is set LOW
 *
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*tx_en_pin_ctrl_t)(void *user_data, uint8_t enable);

/**
 * \brief RESETB PIN CONTROL FUNCTION
 *
 *
 * \param *user_data  A void pointer to a client defined structure containing
 * 					any parameters/settings that may be required by the function
 * 					to control the hardware for the ADI Device RESETB PIN.
 * \param enable   A uint8_t value indicating the desired enable/disable
 * 					reset via the ADI device RESETB pin.
 * 					A value of 1 indicates RESETB pin is set LOW
 * 					A value of 0 indicates RESETB pin is set HIGH
 *
 * \retval 0 for success
 * \retval Any non-zero value indicates an error
 *
 * \note
 */
typedef int(*reset_pin_ctrl_t)(void *user_data, uint8_t enable);


/** SPI Interface Parameters. */
typedef enum
{
	/* Keep this invalid value as 0, so the API can test for wrong setting.*/
	SPI_NONE = 0,
	/* Set SPI SDO (Serial Data Output) pin as active. 
	 * This is in case the 4-wire SPI is needed.*/
	SPI_SDO = 1,
	/* Set SPI SDIO (Serial Data Input-Output) pin as active. 
	 * This is in case the 3-wire SPI is needed.*/
	SPI_SDIO = 2,
	/* Keep it last. */
	SPI_CONFIG_MAX = 3
}spi_sdo_config_t;

/** JESD Interface Parameters. */
typedef struct {
	uint8_t jesd_L;     /**< JESD Lane Param L. Valid Range 1 to 8 */
	uint8_t jesd_F;     /**< JESD Octet Param F. Valid Range 1 to 4 */
	uint8_t jesd_M;     /**< JESD Converter Param M. Must be set to 2 */
	uint8_t jesd_S;     /**< JESD No of Sample Param S. Valid range to 1-4 */

	/** JESD High Density Param HD. Set to 1 if F = 1 else set to 0 */
	uint8_t jesd_HD;

	uint8_t jesd_K;     /**< JESD multiframe Param K. Must be Set to 32 */
	uint8_t jesd_N;     /**< JESD Converter Resolution Param N. Set to 16 */
	uint8_t jesd_NP;    /**< JESD Bit Packing Sample NP.Set to 16 */
	uint8_t jesd_CF;    /**< JESD Param CF. Set to 0 */
	uint8_t jesd_CS;    /**< JESD Param CS. Set to 0 */

	uint8_t jesd_DID;   /**< JESD Device ID Param DID. */
	uint8_t jesd_BID;   /**< JESD Bank ID. Param BID */
	uint8_t jesd_LID0;  /**< JESD Lane ID for Lane 0 Param LIDO*/
	uint8_t jesd_JESDV; /**< JESD Version */
}jesd_param_t;

/** Enumerates all available PRBS patterns */
typedef enum {
	PRBS_NONE, /**< PRBS OFF */
	PRBS7,     /**< PRBS7 pattern */
	PRBS15,    /**< PRBS15 pattern */
	PRBS31,    /**< PRBS31 pattern */
	PRBS_MAX   /**< Number of member in this enum */
}jesd_prbs_pattern_t;

#endif /* !__API_DEF_H__ */
