// SPDX-License-Identifier: GPL-2.0
/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#include <drv_types.h>
#include <hal_data.h>

void rtw_hal_chip_configure(struct adapter *padapter)
{
	rtl8723bs_interface_configure(padapter);
}

void rtw_hal_read_chip_info(struct adapter *padapter)
{
	ReadAdapterInfo8723BS(padapter);
}

void rtw_hal_read_chip_version(struct adapter *padapter)
{
	rtl8723b_read_chip_version(padapter);
}

void rtw_hal_def_value_init(struct adapter *padapter)
{
	rtl8723bs_init_default_value(padapter);
}

void rtw_hal_free_data(struct adapter *padapter)
{
	/* free HAL Data */
	rtw_hal_data_deinit(padapter);
}

void rtw_hal_dm_init(struct adapter *padapter)
{
	rtl8723b_init_dm_priv(padapter);
}

static void rtw_hal_init_opmode(struct adapter *padapter)
{
	enum ndis_802_11_network_infrastructure networkType = Ndis802_11InfrastructureMax;
	struct  mlme_priv *pmlmepriv = &(padapter->mlmepriv);
	signed int fw_state;

	fw_state = get_fwstate(pmlmepriv);

	if (fw_state & WIFI_ADHOC_STATE)
		networkType = Ndis802_11IBSS;
	else if (fw_state & WIFI_STATION_STATE)
		networkType = Ndis802_11Infrastructure;
	else if (fw_state & WIFI_AP_STATE)
		networkType = Ndis802_11APMode;
	else
		return;

	rtw_setopmode_cmd(padapter, networkType, false);
}

uint rtw_hal_init(struct adapter *padapter)
{
	uint status;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);

	status = rtl8723bs_hal_init(padapter);

	if (status == _SUCCESS) {
		rtw_hal_init_opmode(padapter);

		dvobj->padapters->hw_init_completed = true;

		if (padapter->registrypriv.notch_filter == 1)
			rtw_hal_notch_filter(padapter, 1);

		rtw_sec_restore_wep_key(dvobj->padapters);

		init_hw_mlme_ext(padapter);

		rtw_bb_rf_gain_offset(padapter);
	} else {
		dvobj->padapters->hw_init_completed = false;
	}

	return status;
}

uint rtw_hal_deinit(struct adapter *padapter)
{
	uint status = _SUCCESS;
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);

	status = rtl8723bs_hal_deinit(padapter);

	if (status == _SUCCESS) {
		padapter = dvobj->padapters;
		padapter->hw_init_completed = false;
	}

	return status;
}

void rtw_hal_set_hwreg(struct adapter *padapter, u8 variable, u8 *val)
{
	SetHwReg8723BS(padapter, variable, val);
}

void rtw_hal_get_hwreg(struct adapter *padapter, u8 variable, u8 *val)
{
	GetHwReg8723BS(padapter, variable, val);
}

void rtw_hal_set_hwreg_with_buf(struct adapter *padapter, u8 variable, u8 *pbuf, int len)
{
	SetHwRegWithBuf8723B(padapter, variable, pbuf, len);
}

u8 rtw_hal_get_def_var(struct adapter *padapter, enum hal_def_variable eVariable, void *pValue)
{
	return GetHalDefVar8723BSDIO(padapter, eVariable, pValue);
}

void rtw_hal_set_odm_var(struct adapter *padapter, enum hal_odm_variable eVariable, void *pValue1, bool bSet)
{
	SetHalODMVar(padapter, eVariable, pValue1, bSet);
}

void rtw_hal_enable_interrupt(struct adapter *padapter)
{
	EnableInterrupt8723BSdio(padapter);
}

void rtw_hal_disable_interrupt(struct adapter *padapter)
{
	DisableInterrupt8723BSdio(padapter);
}

u8 rtw_hal_check_ips_status(struct adapter *padapter)
{
	return CheckIPSStatus(padapter);
}

s32	rtw_hal_xmitframe_enqueue(struct adapter *padapter, struct xmit_frame *pxmitframe)
{
	return rtl8723bs_hal_xmitframe_enqueue(padapter, pxmitframe);
}

s32	rtw_hal_xmit(struct adapter *padapter, struct xmit_frame *pxmitframe)
{
	return rtl8723bs_hal_xmit(padapter, pxmitframe);
}

/*
 * [IMPORTANT] This function would be run in interrupt context.
 */
s32	rtw_hal_mgnt_xmit(struct adapter *padapter, struct xmit_frame *pmgntframe)
{
	update_mgntframe_attrib_addr(padapter, pmgntframe);
	/* pframe = (u8 *)(pmgntframe->buf_addr) + TXDESC_OFFSET; */
	/* pwlanhdr = (struct rtw_ieee80211_hdr *)pframe; */
	/* memcpy(pmgntframe->attrib.ra, pwlanhdr->addr1, ETH_ALEN); */

	if (padapter->securitypriv.binstallBIPkey == true) {
		if (is_multicast_ether_addr(pmgntframe->attrib.ra)) {
			pmgntframe->attrib.encrypt = _BIP_;
			/* pmgntframe->attrib.bswenc = true; */
		} else {
			pmgntframe->attrib.encrypt = _AES_;
			pmgntframe->attrib.bswenc = true;
		}
		rtw_mgmt_xmitframe_coalesce(padapter, pmgntframe->pkt, pmgntframe);
	}

	return rtl8723bs_mgnt_xmit(padapter, pmgntframe);
}

s32	rtw_hal_init_xmit_priv(struct adapter *padapter)
{
	return rtl8723bs_init_xmit_priv(padapter);
}

void rtw_hal_free_xmit_priv(struct adapter *padapter)
{
	rtl8723bs_free_xmit_priv(padapter);
}

s32	rtw_hal_init_recv_priv(struct adapter *padapter)
{
	return rtl8723bs_init_recv_priv(padapter);
}

void rtw_hal_free_recv_priv(struct adapter *padapter)
{
	rtl8723bs_free_recv_priv(padapter);
}

void rtw_hal_update_ra_mask(struct sta_info *psta, u8 rssi_level)
{
	struct adapter *padapter;
	struct mlme_priv *pmlmepriv;

	if (!psta)
		return;

	padapter = psta->padapter;

	pmlmepriv = &(padapter->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == true)
		add_RATid(padapter, psta, rssi_level);
	else {
		UpdateHalRAMask8723B(padapter, psta->mac_id, rssi_level);
	}
}

void rtw_hal_add_ra_tid(struct adapter *padapter, u32 bitmap, u8 *arg, u8 rssi_level)
{
	rtl8723b_Add_RateATid(padapter, bitmap, arg, rssi_level);
}

/*Start specifical interface thread		*/
void rtw_hal_start_thread(struct adapter *padapter)
{
	rtl8723b_start_thread(padapter);
}
/*Start specifical interface thread		*/
void rtw_hal_stop_thread(struct adapter *padapter)
{
	rtl8723b_stop_thread(padapter);
}

u32 rtw_hal_read_bbreg(struct adapter *padapter, u32 RegAddr, u32 BitMask)
{
	return PHY_QueryBBReg_8723B(padapter, RegAddr, BitMask);
}
void rtw_hal_write_bbreg(struct adapter *padapter, u32 RegAddr, u32 BitMask, u32 Data)
{
	PHY_SetBBReg_8723B(padapter, RegAddr, BitMask, Data);
}

u32 rtw_hal_read_rfreg(struct adapter *padapter, u32 eRFPath, u32 RegAddr, u32 BitMask)
{
	return PHY_QueryRFReg_8723B(padapter, eRFPath, RegAddr, BitMask);
}
void rtw_hal_write_rfreg(struct adapter *padapter, u32 eRFPath, u32 RegAddr, u32 BitMask, u32 Data)
{
	PHY_SetRFReg_8723B(padapter, eRFPath, RegAddr, BitMask, Data);
}

void rtw_hal_set_chan(struct adapter *padapter, u8 channel)
{
	PHY_SwChnl8723B(padapter, channel);
}

void rtw_hal_set_chnl_bw(struct adapter *padapter, u8 channel,
			 enum channel_width Bandwidth, u8 Offset40, u8 Offset80)
{
	PHY_SetSwChnlBWMode8723B(padapter, channel, Bandwidth, Offset40, Offset80);
}

void rtw_hal_dm_watchdog(struct adapter *padapter)
{
	rtl8723b_HalDmWatchDog(padapter);
}

void rtw_hal_dm_watchdog_in_lps(struct adapter *padapter)
{
	if (adapter_to_pwrctl(padapter)->fw_current_in_ps_mode) {
		rtl8723b_HalDmWatchDog_in_LPS(padapter); /* this function caller is in interrupt context */
	}
}

void beacon_timing_control(struct adapter *padapter)
{
	rtl8723b_SetBeaconRelatedRegisters(padapter);
}


s32 rtw_hal_xmit_thread_handler(struct adapter *padapter)
{
	return rtl8723bs_xmit_buf_handler(padapter);
}

void rtw_hal_notch_filter(struct adapter *adapter, bool enable)
{
	hal_notch_filter_8723b(adapter, enable);
}

bool rtw_hal_c2h_valid(struct adapter *adapter, u8 *buf)
{
	return c2h_evt_valid((struct c2h_evt_hdr_88xx *)buf);
}

s32 rtw_hal_c2h_handler(struct adapter *adapter, u8 *c2h_evt)
{
	return c2h_handler_8723b(adapter, c2h_evt);
}

c2h_id_filter rtw_hal_c2h_id_filter_ccx(struct adapter *adapter)
{
	return c2h_id_filter_ccx_8723b;
}

s32 rtw_hal_macid_sleep(struct adapter *padapter, u32 macid)
{
	u8 support;

	support = false;
	rtw_hal_get_def_var(padapter, HAL_DEF_MACID_SLEEP, &support);
	if (false == support)
		return _FAIL;

	rtw_hal_set_hwreg(padapter, HW_VAR_MACID_SLEEP, (u8 *)&macid);

	return _SUCCESS;
}

s32 rtw_hal_macid_wakeup(struct adapter *padapter, u32 macid)
{
	u8 support;

	support = false;
	rtw_hal_get_def_var(padapter, HAL_DEF_MACID_SLEEP, &support);
	if (false == support)
		return _FAIL;

	rtw_hal_set_hwreg(padapter, HW_VAR_MACID_WAKEUP, (u8 *)&macid);

	return _SUCCESS;
}

s32 rtw_hal_fill_h2c_cmd(struct adapter *padapter, u8 ElementID, u32 CmdLen, u8 *pCmdBuffer)
{
	return FillH2CCmd8723B(padapter, ElementID, CmdLen, pCmdBuffer);
}
