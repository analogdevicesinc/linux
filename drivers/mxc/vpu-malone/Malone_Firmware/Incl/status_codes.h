/***************************************************
  Copyright (c) 2015 Amphion Semiconductor Ltd 
                All rights reserved.               
 ***************************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************
  Filename    : status_codes.h
  Description : Public header file for FW & SW status codes
                including those used by relevant abstraction layers
  Author      : Kyle McAdoo
 
 *****************************************************/

#ifndef _STATUS_CODES_H_
#define _STATUS_CODES_H_

typedef enum
{
   /*   0  0x00  */   MEDIAIP_FW_STATUS_OK = 0,
   /*   1  0x01  */   MEDIAIP_FW_STATUS_ALREADY_INIT,
   /*   2  0x02  */   MEDIAIP_FW_STATUS_NOT_INIT,
   /*   3  0x03  */   MEDIAIP_FW_STATUS_INTERNAL_ERROR,
   /*   4  0x04  */   MEDIAIP_FW_STATUS_BAD_HANDLE,
   /*   5  0x05  */   MEDIAIP_FW_STATUS_BAD_PARAMETER,
   /*   6  0x06  */   MEDIAIP_FW_STATUS_BAD_LENGTH,
   /*   7  0x07  */   MEDIAIP_FW_STATUS_BAD_UNIT,
   /*   8  0x08  */   MEDIAIP_FW_STATUS_RESOURCE_ERROR,
   /*   9  0x09  */   MEDIAIP_FW_STATUS_CLOSED_HANDLE,
   /*  10  0x0A  */   MEDIAIP_FW_STATUS_TIMEOUT,
   /*  11  0x0B  */   MEDIAIP_FW_STATUS_NOT_ATTACHED,
   /*  12  0x0C  */   MEDIAIP_FW_STATUS_NOT_SUPPORTED,
   /*  13  0x0D  */   MEDIAIP_FW_STATUS_REOPENED_HANDLE,
   /*  14  0x0E  */   MEDIAIP_FW_STATUS_INVALID,
   /*  15  0x0F  */   MEDIAIP_FW_STATUS_DESTROYED,
   /*  16  0x10  */   MEDIAIP_FW_STATUS_DISCONNECTED,
   /*  17  0x11  */   MEDIAIP_FW_STATUS_BUSY,
   /*  18  0x12  */   MEDIAIP_FW_STATUS_IN_USE,
   /*  19  0x13  */   MEDIAIP_FW_STATUS_CANCELLED,
   /*  20  0x14  */   MEDIAIP_FW_STATUS_UNDEFINED,
   /*  21  0x15  */   MEDIAIP_FW_STATUS_UNKNOWN,
   /*  22  0x16  */   MEDIAIP_FW_STATUS_NOT_FOUND,
   /*  23  0x17  */   MEDIAIP_FW_STATUS_NOT_AVAILABLE,
   /*  24  0x18  */   MEDIAIP_FW_STATUS_NOT_COMPATIBLE,
   /*  25  0x19  */   MEDIAIP_FW_STATUS_NOT_IMPLEMENTED,
   /*  26  0x1A  */   MEDIAIP_FW_STATUS_EMPTY,
   /*  27  0x1B  */   MEDIAIP_FW_STATUS_FULL,
   /*  28  0x1C  */   MEDIAIP_FW_STATUS_FAILURE,
   /*  29  0x1D  */   MEDIAIP_FW_STATUS_ALREADY_ATTACHED,
   /*  30  0x1E  */   MEDIAIP_FW_STATUS_ALREADY_DONE,
   /*  31  0x1F  */   MEDIAIP_FW_STATUS_ASLEEP,
   /*  32  0x20  */   MEDIAIP_FW_STATUS_BAD_ATTACHMENT,
   /*  33  0x21  */   MEDIAIP_FW_STATUS_BAD_COMMAND,
   /*  34  0x22  */   MEDIAIP_FW_STATUS_INT_HANDLED,
   /*  35  0x23  */   MEDIAIP_FW_STATUS_INT_NOT_HANDLED,
   /*  36  0x24  */   MEDIAIP_FW_STATUS_NOT_SET,
   /*  37  0x25  */   MEDIAIP_FW_STATUS_NOT_HOOKED,
   /*  38  0x26  */   MEDIAIP_FW_STATUS_COMPLETE,
   /*  39  0x27  */   MEDIAIP_FW_STATUS_INVALID_NODE,
   /*  40  0x28  */   MEDIAIP_FW_STATUS_DUPLICATE_NODE,
   /*  41  0x29  */   MEDIAIP_FW_STATUS_HARDWARE_NOT_FOUND,
   /*  42  0x2A  */   MEDIAIP_FW_STATUS_ILLEGAL_OPERATION,
   /*  43  0x2B  */   MEDIAIP_FW_STATUS_INCOMPATIBLE_FORMATS,
   /*  44  0x2C  */   MEDIAIP_FW_STATUS_INVALID_DEVICE,
   /*  45  0x2D  */   MEDIAIP_FW_STATUS_INVALID_EDGE,
   /*  46  0x2E  */   MEDIAIP_FW_STATUS_INVALID_NUMBER,
   /*  47  0x2F  */   MEDIAIP_FW_STATUS_INVALID_STATE,
   /*  48  0x30  */   MEDIAIP_FW_STATUS_INVALID_TYPE,
   /*  49  0x31  */   MEDIAIP_FW_STATUS_STOPPED,
   /*  50  0x32  */   MEDIAIP_FW_STATUS_SUSPENDED,
   /*  51  0x33  */   MEDIAIP_FW_STATUS_TERMINATED,
   /*  52  0x34  */   MEDIAIP_FW_STATUS_FRAMESTORE_NOT_HANDLED,
   /* Last Entry */   MEDIAIP_FW_STATUS_CODE_LAST = MEDIAIP_FW_STATUS_FRAMESTORE_NOT_HANDLED
} MEDIAIP_FW_STATUS;

#if RTOS != NONE

#if OSAL == CNXT_KAL
#include "cnxt_kal_status_codes.h"
typedef MEDIAIP_OSAL_STATUS CNXT_IRQ_RETCODE;
typedef MEDIAIP_FW_STATUS MEDIAIP_IRQ_RETCODE;
#endif

#if OSAL == NXP_OSAL
#include "nxp_osal_status_codes.h"
typedef MEDIAIP_OSAL_STATUS MEDIAIP_IRQ_RETCODE; 
#endif

#else
typedef MEDIAIP_FW_STATUS MEDIAIP_IRQ_RETCODE; 
#endif

#endif /* _STATUS_CODES_H_ */

/* End of File */
