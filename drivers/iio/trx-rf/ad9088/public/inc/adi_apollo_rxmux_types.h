/*!
 * \brief     Rx Crossbar Mux functional block definitions
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_RXMUX
 * @{
 */
#ifndef ADI_APOLLO_RXMUX_TYPES_H__
#define ADI_APOLLO_RXMUX_TYPES_H__

/*============= I N C L U D E S ============*/
#include "adi_cms_api_common.h"
#include "adi_apollo_common.h"

/*============= D E F I N E S ==============*/
#define ADI_APOLLO_ADC_MUX_NUM 8U
#define ADI_APOLLO_ADC_MUX_PER_SIDE 4
#define ADI_APOLLO_ADC_PER_MUX 4


/*!
* \brief Enumerates Rx Sample Xbar select
*/
typedef enum {
    ADI_APOLLO_RXMUX_SAMPLE_FINE0_I =         0U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE0_Q =         1U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE1_I =         2U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE1_Q =         3U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE2_I =         4U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE2_Q =         5U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE3_I =         6U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE3_Q =         7U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE4_I =         8U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE4_Q =         9U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE5_I =         10U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE5_Q =         11U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE6_I =         12U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE6_Q =         13U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE7_I =         14U,
    ADI_APOLLO_RXMUX_SAMPLE_FINE7_Q =         15U,
} adi_apollo_rxmux_sample_sel_e;

/*!
* \brief Enumerates Rx Mux1 crossbar outputs (CBOUT). ADC inputs are routed to CBOUTx
*/
typedef enum {
    ADI_APOLLO_RXMUX_CBOUT_0 =         0U,  /*!< Rx Mux1 CBOUT-0 output */
    ADI_APOLLO_RXMUX_CBOUT_1 =         1U,  /*!< Rx Mux1 CBOUT-1 output */
    ADI_APOLLO_RXMUX_CBOUT_2 =         2U,  /*!< Rx Mux1 CBOUT-2 output */
    ADI_APOLLO_RXMUX_CBOUT_3 =         3U   /*!< Rx Mux1 CBOUT-3 output */
} adi_apollo_rxmux_cbout_sel_e;

#endif /* ADI_APOLLO_RXMUX_TYPES_H__ */
/*! @} */