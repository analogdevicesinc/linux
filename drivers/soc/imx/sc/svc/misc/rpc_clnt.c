/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * File containing client-side RPC functions for the MISC service. These
 * functions are ported to clients that communicate to the SC.
 *
 * @addtogroup MISC_SVC
 * @{
 */

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>
#include <soc/imx8/sc/svc/misc/api.h>
#include "../../main/rpc.h"
#include "rpc.h"

/* Local Defines */

/* Local Types */

/* Local Functions */

sc_err_t sc_misc_set_control(sc_ipc_t ipc, sc_rsrc_t resource,
			     sc_ctrl_t ctrl, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SET_CONTROL;
	RPC_U32(&msg, 0) = ctrl;
	RPC_U32(&msg, 4) = val;
	RPC_U16(&msg, 8) = resource;
	RPC_SIZE(&msg) = 4;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_get_control(sc_ipc_t ipc, sc_rsrc_t resource,
			     sc_ctrl_t ctrl, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_GET_CONTROL;
	RPC_U32(&msg, 0) = ctrl;
	RPC_U16(&msg, 4) = resource;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	if (val != NULL) {
		*val = RPC_U32(&msg, 0);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_max_dma_group(sc_ipc_t ipc, sc_rm_pt_t pt,
				   sc_misc_dma_group_t max)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SET_MAX_DMA_GROUP;
	RPC_U8(&msg, 0) = pt;
	RPC_U8(&msg, 1) = max;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_dma_group(sc_ipc_t ipc, sc_rsrc_t resource,
			       sc_misc_dma_group_t group)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SET_DMA_GROUP;
	RPC_U16(&msg, 0) = resource;
	RPC_U8(&msg, 2) = group;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_image_load(sc_ipc_t ipc, uint32_t addr_src,
				 uint32_t addr_dst, uint32_t len, bool fw)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SECO_IMAGE_LOAD;
	RPC_U32(&msg, 0) = addr_src;
	RPC_U32(&msg, 4) = addr_dst;
	RPC_U32(&msg, 8) = len;
	RPC_U8(&msg, 12) = (uint8_t)fw;
	RPC_SIZE(&msg) = 5;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_authenticate(sc_ipc_t ipc,
				   sc_misc_seco_auth_cmd_t cmd,
				   uint32_t addr_meta)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SECO_AUTHENTICATE;
	RPC_U32(&msg, 0) = addr_meta;
	RPC_U8(&msg, 4) = cmd;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_debug_out(sc_ipc_t ipc, uint8_t ch)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_DEBUG_OUT;
	RPC_U8(&msg, 0) = ch;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	return;
}

sc_err_t sc_misc_waveform_capture(sc_ipc_t ipc, bool enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_WAVEFORM_CAPTURE;
	RPC_U8(&msg, 0) = (uint8_t)enable;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_build_info(sc_ipc_t ipc, uint32_t *build, uint32_t *commit)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_BUILD_INFO;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (build != NULL) {
		*build = RPC_U32(&msg, 0);
	}

	if (commit != NULL) {
		*commit = RPC_U32(&msg, 4);
	}

	return;
}

void sc_misc_unique_id(sc_ipc_t ipc, uint32_t *id_l, uint32_t *id_h)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_UNIQUE_ID;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (id_l != NULL) {
		*id_l = RPC_U32(&msg, 0);
	}

	if (id_h != NULL) {
		*id_h = RPC_U32(&msg, 4);
	}

	return;
}

sc_err_t sc_misc_set_ari(sc_ipc_t ipc, sc_rsrc_t resource,
			 sc_rsrc_t resource_mst, uint16_t ari, bool enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SET_ARI;
	RPC_U16(&msg, 0) = resource;
	RPC_U16(&msg, 2) = resource_mst;
	RPC_U16(&msg, 4) = ari;
	RPC_U8(&msg, 6) = (uint8_t)enable;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_boot_status(sc_ipc_t ipc, sc_misc_boot_status_t status)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_BOOT_STATUS;
	RPC_U8(&msg, 0) = status;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, true);

	return;
}

sc_err_t sc_misc_boot_done(sc_ipc_t ipc, sc_rsrc_t cpu)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_BOOT_DONE;
	RPC_U16(&msg, 0) = cpu;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_otp_fuse_read(sc_ipc_t ipc, uint32_t word, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_OTP_FUSE_READ;
	RPC_U32(&msg, 0) = word;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (val != NULL) {
		*val = RPC_U32(&msg, 0);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_otp_fuse_write(sc_ipc_t ipc, uint32_t word, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_OTP_FUSE_WRITE;
	RPC_U32(&msg, 0) = word;
	RPC_U32(&msg, 4) = val;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_temp(sc_ipc_t ipc, sc_rsrc_t resource,
			  sc_misc_temp_t temp, int16_t celsius, int8_t tenths)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_SET_TEMP;
	RPC_U16(&msg, 0) = resource;
	RPC_I16(&msg, 2) = celsius;
	RPC_U8(&msg, 4) = temp;
	RPC_I8(&msg, 5) = tenths;
	RPC_SIZE(&msg) = 3;

	sc_call_rpc(ipc, &msg, false);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_get_temp(sc_ipc_t ipc, sc_rsrc_t resource,
			  sc_misc_temp_t temp, int16_t * celsius,
			  int8_t * tenths)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_GET_TEMP;
	RPC_U16(&msg, 0) = resource;
	RPC_U8(&msg, 2) = temp;
	RPC_SIZE(&msg) = 2;

	sc_call_rpc(ipc, &msg, false);

	if (celsius != NULL) {
		*celsius = RPC_I16(&msg, 0);
	}

	result = RPC_R8(&msg);
	if (tenths != NULL) {
		*tenths = RPC_I8(&msg, 2);
	}

	return (sc_err_t)result;
}

void sc_misc_get_boot_dev(sc_ipc_t ipc, sc_rsrc_t *dev)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_GET_BOOT_DEV;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (dev != NULL) {
		*dev = RPC_U16(&msg, 0);
	}

	return;
}

void sc_misc_get_button_status(sc_ipc_t ipc, bool *status)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = (uint8_t)SC_RPC_SVC_MISC;
	RPC_FUNC(&msg) = (uint8_t)MISC_FUNC_GET_BUTTON_STATUS;
	RPC_SIZE(&msg) = 1;

	sc_call_rpc(ipc, &msg, false);

	if (status != NULL) {
		*status = RPC_U8(&msg, 0);
	}

	return;
}

/**@}*/
