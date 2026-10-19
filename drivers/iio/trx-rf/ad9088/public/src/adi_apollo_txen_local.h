/*!
 * \brief     Apollo Tx En local defines and prototypes
 *
 * \copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * \addtogroup ADI_APOLLO_TXEN
 * @{
 */
#ifndef ADI_APOLLO_TXEN_LOCAL_H__
#define ADI_APOLLO_TXEN_LOCAL_H__


/*============= I N C L U D E S ============*/

/*============= D E F I N E S ==============*/

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

uint32_t tx_chan_id_to_txen_power_ctrl_base(int32_t chan_id);

#ifdef __cplusplus
}
#endif

#endif /* ADI_APOLLO_TXEN_LOCAL_H__ */
/*! @} */
