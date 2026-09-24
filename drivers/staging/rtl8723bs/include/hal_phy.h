/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#ifndef __HAL_PHY_H__
#define __HAL_PHY_H__
/*  */
/*  Antenna detection method, i.e., using single tone detection or RSSI reported from each antenna detected. */
/*  Added by Roger, 2013.05.22. */
/*  */
#define ANT_DETECT_BY_SINGLE_TONE	BIT(0)
#define ANT_DETECT_BY_RSSI				BIT(1)
#define IS_ANT_DETECT_SUPPORT_SINGLE_TONE(__Adapter)		((GET_HAL_DATA(__Adapter)->AntDetection) & ANT_DETECT_BY_SINGLE_TONE)
#define IS_ANT_DETECT_SUPPORT_RSSI(__Adapter)		((GET_HAL_DATA(__Adapter)->AntDetection) & ANT_DETECT_BY_RSSI)


/*--------------------------Define Parameters-------------------------------*/
enum rf_path {
	RF_PATH_A = 0,
	RF_PATH_B,
	RF_PATH_MAX
};

#define	TX_1S			0

enum wireless_mode {
	WIRELESS_MODE_UNKNOWN = 0x00,
	WIRELESS_MODE_B = 0x02,
	WIRELESS_MODE_G = 0x04,
	WIRELESS_MODE_AUTO = 0x08,
	WIRELESS_MODE_N_24G = 0x10,
	WIRELESS_MODE_AC_24G  = 0x80,
	WIRELESS_MODE_AC_ONLY  = 0x100,
};

enum SwChnlCmdID {
	CmdID_End,
	CmdID_SetTxPowerLevel,
	CmdID_BBRegWrite10,
	CmdID_WritePortUlong,
	CmdID_WritePortUshort,
	CmdID_WritePortUchar,
	CmdID_RF_WriteReg,
};

struct SwChnlCmd {
	enum SwChnlCmdID	CmdID;
	u32 			Para1;
	u32 			Para2;
	u32 			msDelay;
};

/*--------------------------Exported Function prototype---------------------*/

#endif /* __HAL_COMMON_H__ */
