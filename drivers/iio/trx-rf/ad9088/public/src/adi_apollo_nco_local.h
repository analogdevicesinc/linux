/*!
 * \brief     NCO local defines and prototypes
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_CNCO
 * @{
 */
#ifndef ADI_APOLLO_NCO_LOCAL_H__
#define ADI_APOLLO_NCO_LOCAL_H__

/*============= I N C L U D E S ============*/

/*============= D E F I N E S ==============*/

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

uint32_t calc_rx_cnco_base(uint32_t idx);
uint32_t calc_rx_fnco_base(uint32_t idx);
uint32_t calc_tx_cnco_base(uint32_t idx);
uint32_t calc_tx_fnco_base(uint32_t idx);

#ifndef CLIENT_IGNORE
#endif /* CLIENT_IGNORE*/

#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_NCO_LOCAL_H__ */
/*! @} */
