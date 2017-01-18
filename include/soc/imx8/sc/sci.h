/*==========================================================================*/
/*!
 * @file sci.h
 *
 * Header file containing the public System Controller Interface (SCI)
 * definitions.
 *
 *
 * @{
 */
/*==========================================================================*/

#ifndef _SC_SCI_H
#define _SC_SCI_H

/* Defines */

#define SC_NUM_IPC		5

/* Includes */

#include <soc/imx8/sc/ipc.h>
#include <soc/imx8/sc/svc/misc/api.h>
#include <soc/imx8/sc/svc/pad/api.h>
#include <soc/imx8/sc/svc/pm/api.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/timer/api.h>

/* Types */

/* Functions */
/*!
 * This function initializes the MU connection to SCU.
 *
 * @return  Returns an error code.
 */
int imx8_mu_init(void);

#endif /* _SC_SCI_H */

/**@}*/

