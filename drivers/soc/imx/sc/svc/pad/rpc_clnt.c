/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the PAD service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup PAD_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/pad/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_pad_set_mux(sc_ipc_t ipc, sc_pad_t pad,
			uint8_t mux, sc_pad_config_t config, sc_pad_iso_t iso)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_MUX);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(mux);
	RPC_U8(&msg, 3U) = U8(config);
	RPC_U8(&msg, 4U) = U8(iso);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_mux(sc_ipc_t ipc, sc_pad_t pad,
			uint8_t *mux, sc_pad_config_t *config,
			sc_pad_iso_t *iso)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_MUX);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (mux != NULL) {
		*mux = RPC_U8(&msg, 0U);
	}

	if (config != NULL) {
		*config = RPC_U8(&msg, 1U);
	}

	if (iso != NULL) {
		*iso = RPC_U8(&msg, 2U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp(sc_ipc_t ipc, sc_pad_t pad, uint32_t ctrl)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP);
	RPC_U32(&msg, 0U) = U32(ctrl);
	RPC_U16(&msg, 4U) = U16(pad);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp(sc_ipc_t ipc, sc_pad_t pad, uint32_t *ctrl)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (ctrl != NULL) {
		*ctrl = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_wakeup(sc_ipc_t ipc, sc_pad_t pad, sc_pad_wakeup_t wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_WAKEUP);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(wakeup);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_wakeup(sc_ipc_t ipc, sc_pad_t pad, sc_pad_wakeup_t *wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_WAKEUP);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (wakeup != NULL) {
		*wakeup = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pad_set_all(sc_ipc_t ipc, sc_pad_t pad, uint8_t mux,
			sc_pad_config_t config, sc_pad_iso_t iso, uint32_t ctrl,
			sc_pad_wakeup_t wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_ALL);
	RPC_U32(&msg, 0U) = U32(ctrl);
	RPC_U16(&msg, 4U) = U16(pad);
	RPC_U8(&msg, 6U) = U8(mux);
	RPC_U8(&msg, 7U) = U8(config);
	RPC_U8(&msg, 8U) = U8(iso);
	RPC_U8(&msg, 9U) = U8(wakeup);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_all(sc_ipc_t ipc, sc_pad_t pad, uint8_t *mux,
			sc_pad_config_t *config, sc_pad_iso_t *iso,
			uint32_t *ctrl, sc_pad_wakeup_t *wakeup)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_ALL);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (ctrl != NULL) {
		*ctrl = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	if (mux != NULL) {
		*mux = RPC_U8(&msg, 4U);
	}

	if (config != NULL) {
		*config = RPC_U8(&msg, 5U);
	}

	if (iso != NULL) {
		*iso = RPC_U8(&msg, 6U);
	}

	if (wakeup != NULL) {
		*wakeup = RPC_U8(&msg, 7U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pad_set(sc_ipc_t ipc, sc_pad_t pad, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET);
	RPC_U32(&msg, 0U) = U32(val);
	RPC_U16(&msg, 4U) = U16(pad);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get(sc_ipc_t ipc, sc_pad_t pad, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (val != NULL) {
		*val = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28fdsoi(sc_ipc_t ipc, sc_pad_t pad,
			       sc_pad_28fdsoi_dse_t dse, sc_pad_28fdsoi_ps_t ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(dse);
	RPC_U8(&msg, 3U) = U8(ps);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28fdsoi(sc_ipc_t ipc, sc_pad_t pad,
			       sc_pad_28fdsoi_dse_t *dse,
			       sc_pad_28fdsoi_ps_t *ps)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (dse != NULL) {
		*dse = RPC_U8(&msg, 0U);
	}

	if (ps != NULL) {
		*ps = RPC_U8(&msg, 1U);
	}

	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28fdsoi_hsic(sc_ipc_t ipc, sc_pad_t pad,
				    sc_pad_28fdsoi_dse_t dse, sc_bool_t hys,
				    sc_pad_28fdsoi_pus_t pus, sc_bool_t pke,
				    sc_bool_t pue)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI_HSIC);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(dse);
	RPC_U8(&msg, 3U) = U8(pus);
	RPC_U8(&msg, 4U) = B2U8(hys);
	RPC_U8(&msg, 5U) = B2U8(pke);
	RPC_U8(&msg, 6U) = B2U8(pue);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28fdsoi_hsic(sc_ipc_t ipc, sc_pad_t pad,
				    sc_pad_28fdsoi_dse_t *dse, sc_bool_t *hys,
				    sc_pad_28fdsoi_pus_t *pus, sc_bool_t *pke,
				    sc_bool_t *pue)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI_HSIC);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (dse != NULL) {
		*dse = RPC_U8(&msg, 0U);
	}

	if (pus != NULL) {
		*pus = RPC_U8(&msg, 1U);
	}

	if (hys != NULL) {
		*hys = U2B(RPC_U8(&msg, 2U));
	}

	if (pke != NULL) {
		*pke = U2B(RPC_U8(&msg, 3U));
	}

	if (pue != NULL) {
		*pue = U2B(RPC_U8(&msg, 4U));
	}

	return (sc_err_t)result;
}

sc_err_t sc_pad_set_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pad_t pad,
				    uint8_t compen, sc_bool_t fastfrz,
				    uint8_t rasrcp, uint8_t rasrcn,
				    sc_bool_t nasrc_sel, sc_bool_t psw_ovr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_SET_GP_28FDSOI_COMP);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_U8(&msg, 2U) = U8(compen);
	RPC_U8(&msg, 3U) = U8(rasrcp);
	RPC_U8(&msg, 4U) = U8(rasrcn);
	RPC_U8(&msg, 5U) = B2U8(fastfrz);
	RPC_U8(&msg, 6U) = B2U8(nasrc_sel);
	RPC_U8(&msg, 7U) = B2U8(psw_ovr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_pad_get_gp_28fdsoi_comp(sc_ipc_t ipc, sc_pad_t pad,
				    uint8_t *compen, sc_bool_t *fastfrz,
				    uint8_t *rasrcp, uint8_t *rasrcn,
				    sc_bool_t *nasrc_sel, sc_bool_t *compok,
				    uint8_t *nasrc, sc_bool_t *psw_ovr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_PAD);
	RPC_FUNC(&msg) = U8(PAD_FUNC_GET_GP_28FDSOI_COMP);
	RPC_U16(&msg, 0U) = U16(pad);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (compen != NULL) {
		*compen = RPC_U8(&msg, 0U);
	}

	if (rasrcp != NULL) {
		*rasrcp = RPC_U8(&msg, 1U);
	}

	if (rasrcn != NULL) {
		*rasrcn = RPC_U8(&msg, 2U);
	}

	if (nasrc != NULL) {
		*nasrc = RPC_U8(&msg, 3U);
	}

	if (fastfrz != NULL) {
		*fastfrz = U2B(RPC_U8(&msg, 4U));
	}

	if (nasrc_sel != NULL) {
		*nasrc_sel = U2B(RPC_U8(&msg, 5U));
	}

	if (compok != NULL) {
		*compok = U2B(RPC_U8(&msg, 6U));
	}

	if (psw_ovr != NULL) {
		*psw_ovr = U2B(RPC_U8(&msg, 7U));
	}

	return (sc_err_t)result;
}

/**@}*/
