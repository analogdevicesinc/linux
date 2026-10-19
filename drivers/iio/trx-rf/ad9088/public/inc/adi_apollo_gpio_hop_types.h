/*!
 * \brief     API header file
 *            This file contains all the publicly exposed methods and data
 *            structures to interface with API.
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_GPIO_HOP
 * @{
 */
 
#ifndef ADI_APOLLO_GPIO_HOP_TYPES_H__
#define ADI_APOLLO_GPIO_HOP_TYPES_H__

/*============= I N C L U D E S ============*/
#include "adi_cms_api_common.h"
#include "adi_apollo_common.h"

/*============= D E F I N E S ==============*/

#define ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER 5U
#define ADI_APOLLO_GPIO_HOP_BLOCK_BIT_NUMBER 4U
#define ADI_APOLLO_GPIO_HOP_SIDE_BIT_NUMBER 1U
#define ADI_APOLLO_GPIO_HOP_SLICE_BIT_NUMBER 3U
#define ADI_APOLLO_GPIO_HOP_TERMINAL_BIT_NUMBER 2U

#define ADI_APOLLO_GPIO_HOP_IDX_NONE 0xFFU /*!< Use to indicate that a gpio bit is not to be used. I.e. pfilt profile needs only 2 bits, [26, 27, ADI_APOLLO_GPIO_HOP_IDX_NONE, ADI_APOLLO_GPIO_HOP_IDX_NONE, ADI_APOLLO_GPIO_HOP_IDX_NONE] */

/*!
* \brief Enumerates Tx/Rx hop parameter selection
*/
typedef enum {
    ADI_APOLLO_GPIO_HOP_TX     = 0x0,   /*!< TX TERMINAL SIGNAL PATH*/
    ADI_APOLLO_GPIO_HOP_RX     = 0x1,   /*!< RX TERMINAL SIGNAL PATH */
    ADI_APOLLO_GPIO_HOP_TXRX   = 0x2,   /*!< RX AND TX TERMINAL SIGNAL PATH */
} adi_apollo_gpio_hop_terminal_e;

/*!
* \brief Enumerates GPIO select blocks
*/
typedef enum {
    ADI_APOLLO_GPIO_BLOCK_FNCO           = 0,
    ADI_APOLLO_GPIO_BLOCK_CNCO           = 1,
    ADI_APOLLO_GPIO_BLOCK_FNCO_CNCO      = 2,
    ADI_APOLLO_GPIO_BLOCK_PFILT          = 3,
    ADI_APOLLO_GPIO_BLOCK_CNCO_PFILT     = 4,
    ADI_APOLLO_GPIO_BLOCK_FNCO_CFIR      = 5,
    ADI_APOLLO_GPIO_BLOCK_FDDC           = 6,
    ADI_APOLLO_GPIO_BLOCK_BMEM_DELAY     = 7,
    ADI_APOLLO_GPIO_BLOCK_CFIR           = 8,
    ADI_APOLLO_GPIO_BLOCK_FNCO_PFILT     = 9,
    ADI_APOLLO_GPIO_BLOCK_PFILT_CFIR     = 10,
    ADI_APOLLO_GPIO_BLOCK_CDDC           = 11,
    ADI_APOLLO_GPIO_BLOCK_LINX           = 12
} adi_apollo_gpio_hop_block_e;

/*!
* \brief Enumerates SPI/GPIO hop parameter selection
*/
typedef enum {
    ADI_APOLLO_GPIO_HOP_SELECT_GPIO = 0,
    ADI_APOLLO_GPIO_HOP_SELECT_SPI = 1
} adi_apollo_gpio_hop_select_e;

/*!
* \brief GPIO profile select 
*/
typedef struct {
    uint8_t index[ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER];  /*!< GPIO indexes for profile select bits */
} adi_apollo_gpio_hop_profile_t;

/*!
* \brief GPIO block select 
*/
typedef struct {
    uint8_t index[ADI_APOLLO_GPIO_HOP_BLOCK_BIT_NUMBER];  /*!< GPIO indexes for block select bits */
} adi_apollo_gpio_hop_block_t;

/*!
* \brief GPIO side select 
*/
typedef struct {
    uint8_t index[ADI_APOLLO_GPIO_HOP_SIDE_BIT_NUMBER];  /*!< GPIO indexes for side select bits */
} adi_apollo_gpio_hop_side_t;

/*!
* \brief GPIO slice select 
*/
typedef struct {
    uint8_t index[ADI_APOLLO_GPIO_HOP_SLICE_BIT_NUMBER];  /*!< GPIO indexes for slice select bits */
} adi_apollo_gpio_hop_slice_t;

/*!
* \brief GPIO terminal select 
*/
typedef struct {
    uint8_t index[ADI_APOLLO_GPIO_HOP_TERMINAL_BIT_NUMBER];  /*!< GPIO indexes for terminal select bits */
} adi_apollo_gpio_hop_terminal_t;

#endif /* ADI_APOLLO_GPIO_HOP_TYPES_H__ */
/*! @} */
