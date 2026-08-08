// SPDX-License-Identifier: GPL-2.0
/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#include <drv_types.h>
#include <hal_data.h>
#include <linux/jiffies.h>

/*  11/16/2008 MH Add description. Get current efuse area enabled word!!. */
u8
rtw_efuse_calculate_word_counts(u8 word_en)
{
	u8 word_cnts = 0;

	if (!(word_en & BIT(0)))
		word_cnts++; /*  0 : write enable */
	if (!(word_en & BIT(1)))
		word_cnts++;
	if (!(word_en & BIT(2)))
		word_cnts++;
	if (!(word_en & BIT(3)))
		word_cnts++;
	return word_cnts;
}

/*-----------------------------------------------------------------------------
 * Function:	rtw_efuse_read_1_byte
 *
 * Overview:	Copy from WMAC fot EFUSE read 1 byte.
 *
 * Input:       NONE
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 * When			Who		Remark
 * 09/23/2008	MHC		Copy from WMAC.
 *
 */
u8 rtw_efuse_read_1_byte(struct adapter *Adapter, u16 Address)
{
	u8 Bytetemp = {0x00};
	u8 temp = {0x00};
	u32 k = 0;
	u16 contentLen = 0;

	Hal_GetEfuseDefinition(Adapter, EFUSE_WIFI, TYPE_EFUSE_REAL_CONTENT_LEN, &contentLen);

	if (Address < contentLen) {/* E-fuse 512Byte */
		/* Write E-fuse Register address bit0~7 */
		temp = Address & 0xFF;
		rtw_write8(Adapter, EFUSE_CTRL + 1, temp);
		Bytetemp = rtw_read8(Adapter, EFUSE_CTRL + 2);
		/* Write E-fuse Register address bit8~9 */
		temp = ((Address >> 8) & 0x03) | (Bytetemp & 0xFC);
		rtw_write8(Adapter, EFUSE_CTRL + 2, temp);

		/* Write 0x30[31]= 0 */
		Bytetemp = rtw_read8(Adapter, EFUSE_CTRL + 3);
		temp = Bytetemp & 0x7F;
		rtw_write8(Adapter, EFUSE_CTRL + 3, temp);

		/* Wait Write-ready (0x30[31]= 1) */
		Bytetemp = rtw_read8(Adapter, EFUSE_CTRL + 3);
		while (!(Bytetemp & 0x80)) {
			Bytetemp = rtw_read8(Adapter, EFUSE_CTRL + 3);
			k++;
			if (k == 1000)
				break;
		}
		return rtw_read8(Adapter, EFUSE_CTRL);
	} else {
		return 0xFF;
	}
} /* rtw_efuse_read_1_byte */

/*  11/16/2008 MH Read one byte from real Efuse. */
u8 rtw_efuse_one_byte_read(struct adapter *padapter, u16 addr, u8 *data)
{
	u32 tmpidx = 0;
	u8 bResult;
	u8 readbyte;

	/*  <20130121, Kordan> For SMIC EFUSE specificatoin. */
	/* 0x34[11]: SW force PGMEN input of efuse to high. (for the bank selected by 0x34[9:8]) */
	/* PHY_SetMacReg(padapter, 0x34, BIT11, 0); */
	rtw_write16(padapter, 0x34, rtw_read16(padapter, 0x34) & (~BIT(11)));

	/*  -----------------e-fuse reg ctrl --------------------------------- */
	/* address */
	rtw_write8(padapter, EFUSE_CTRL + 1, (u8)(addr & 0xff));
	rtw_write8(padapter, EFUSE_CTRL + 2, ((u8)((addr >> 8) & 0x03)) |
	(rtw_read8(padapter, EFUSE_CTRL + 2) & 0xFC));

	/* rtw_write8(padapter, EFUSE_CTRL+3,  0x72); read cmd */
	/* Write bit 32 0 */
	readbyte = rtw_read8(padapter, EFUSE_CTRL + 3);
	rtw_write8(padapter, EFUSE_CTRL + 3, (readbyte & 0x7f));

	while (!(0x80 & rtw_read8(padapter, EFUSE_CTRL + 3)) && (tmpidx < 1000)) {
		mdelay(1);
		tmpidx++;
	}
	if (tmpidx < 100) {
		*data = rtw_read8(padapter, EFUSE_CTRL);
		bResult = true;
	} else {
		*data = 0xff;
		bResult = false;
	}

	return bResult;
}

/*-----------------------------------------------------------------------------
 * Function:	Efuse_ReadAllMap
 *
 * Overview:	Read All Efuse content
 *
 * Input:       NONE
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 * When			Who		Remark
 * 11/11/2008	MHC		Create Version 0.
 *
 */
static void Efuse_ReadAllMap(struct adapter *padapter, u8 efuseType, u8 *Efuse)
{
	u16 mapLen = 0;

	Hal_EfusePowerSwitch(padapter, true);

	Hal_GetEfuseDefinition(padapter, efuseType, TYPE_EFUSE_MAP_LEN, &mapLen);

	Hal_ReadEFuse(padapter, efuseType, 0, mapLen, Efuse);

	Hal_EfusePowerSwitch(padapter, false);
}

/*-----------------------------------------------------------------------------
 * Function:	rtw_efuse_shadow_map_update
 *
 * Overview:	Transfer current EFUSE content to shadow init and modify map.
 *
 * Input:       NONE
 *
 * Output:      NONE
 *
 * Return:      NONE
 *
 * Revised History:
 * When			Who		Remark
 * 11/13/2008	MHC		Create Version 0.
 *
 */
void rtw_efuse_shadow_map_update(struct adapter *padapter, u8 efuseType)
{
	struct eeprom_priv *pEEPROM = GET_EEPROM_EFUSE_PRIV(padapter);
	u16 mapLen = 0;

	Hal_GetEfuseDefinition(padapter, efuseType, TYPE_EFUSE_MAP_LEN, &mapLen);

	if (pEEPROM->bautoload_fail_flag)
		memset(pEEPROM->efuse_eeprom_data, 0xFF, mapLen);
	else
		Efuse_ReadAllMap(padapter, efuseType, pEEPROM->efuse_eeprom_data);

	/* PlatformMoveMemory((void *)&pHalData->EfuseMap[EFUSE_MODIFY_MAP][0], */
	/* void *)&pHalData->EfuseMap[EFUSE_INIT_MAP][0], mapLen); */
} /*  rtw_efuse_shadow_map_update */
