/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SET_CONTROL);
	RPC_U32(&msg, 0U) = U32(ctrl);
	RPC_U32(&msg, 4U) = U32(val);
	RPC_U16(&msg, 8U) = U16(resource);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_get_control(sc_ipc_t ipc, sc_rsrc_t resource,
			     sc_ctrl_t ctrl, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_GET_CONTROL);
	RPC_U32(&msg, 0U) = U32(ctrl);
	RPC_U16(&msg, 4U) = U16(resource);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (val != NULL) {
		*val = RPC_U32(&msg, 0U);
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
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SET_MAX_DMA_GROUP);
	RPC_U8(&msg, 0U) = U8(pt);
	RPC_U8(&msg, 1U) = U8(max);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_dma_group(sc_ipc_t ipc, sc_rsrc_t resource,
			       sc_misc_dma_group_t group)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SET_DMA_GROUP);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(group);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_image_load(sc_ipc_t ipc, sc_faddr_t addr_src,
				 sc_faddr_t addr_dst, uint32_t len,
				 sc_bool_t fw)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_IMAGE_LOAD);
	RPC_U32(&msg, 0U) = U32(addr_src >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr_src);
	RPC_U32(&msg, 8U) = U32(addr_dst >> 32ULL);
	RPC_U32(&msg, 12U) = U32(addr_dst);
	RPC_U32(&msg, 16U) = U32(len);
	RPC_U8(&msg, 20U) = B2U8(fw);
	RPC_SIZE(&msg) = 7U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_authenticate(sc_ipc_t ipc,
				   sc_misc_seco_auth_cmd_t cmd, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_AUTHENTICATE);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_U8(&msg, 8U) = U8(cmd);
	RPC_SIZE(&msg) = 4U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_fuse_write(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_FUSE_WRITE);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_enable_debug(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_ENABLE_DEBUG);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_forward_lifecycle(sc_ipc_t ipc, uint32_t change)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_FORWARD_LIFECYCLE);
	RPC_U32(&msg, 0U) = U32(change);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_return_lifecycle(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_RETURN_LIFECYCLE);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_seco_build_info(sc_ipc_t ipc, uint32_t *version, uint32_t *commit)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_BUILD_INFO);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (version != NULL) {
		*version = RPC_U32(&msg, 0U);
	}

	if (commit != NULL) {
		*commit = RPC_U32(&msg, 4U);
	}

	return;
}

sc_err_t sc_misc_seco_chip_info(sc_ipc_t ipc, uint16_t *lc,
				uint16_t *monotonic, uint32_t *uid_l,
				uint32_t *uid_h)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_CHIP_INFO);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (uid_l != NULL) {
		*uid_l = RPC_U32(&msg, 0U);
	}

	if (uid_h != NULL) {
		*uid_h = RPC_U32(&msg, 4U);
	}

	if (lc != NULL) {
		*lc = RPC_U16(&msg, 8U);
	}

	if (monotonic != NULL) {
		*monotonic = RPC_U16(&msg, 10U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_attest_mode(sc_ipc_t ipc, uint32_t mode)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_ATTEST_MODE);
	RPC_U32(&msg, 0U) = U32(mode);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_attest(sc_ipc_t ipc, uint64_t nonce)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_ATTEST);
	RPC_U32(&msg, 0U) = U32(nonce >> 32ULL);
	RPC_U32(&msg, 4U) = U32(nonce);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_get_attest_pkey(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_GET_ATTEST_PKEY);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_get_attest_sign(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_GET_ATTEST_SIGN);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_attest_verify(sc_ipc_t ipc, sc_faddr_t addr)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_ATTEST_VERIFY);
	RPC_U32(&msg, 0U) = U32(addr >> 32ULL);
	RPC_U32(&msg, 4U) = U32(addr);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_seco_commit(sc_ipc_t ipc, uint32_t *info)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SECO_COMMIT);
	RPC_U32(&msg, 0U) = *PTR_U32(info);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	*info = RPC_U32(&msg, 0U);
	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_debug_out(sc_ipc_t ipc, uint8_t ch)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_DEBUG_OUT);
	RPC_U8(&msg, 0U) = U8(ch);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	return;
}

sc_err_t sc_misc_waveform_capture(sc_ipc_t ipc, sc_bool_t enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_WAVEFORM_CAPTURE);
	RPC_U8(&msg, 0U) = B2U8(enable);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_build_info(sc_ipc_t ipc, uint32_t *build, uint32_t *commit)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_BUILD_INFO);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (build != NULL) {
		*build = RPC_U32(&msg, 0U);
	}

	if (commit != NULL) {
		*commit = RPC_U32(&msg, 4U);
	}

	return;
}

void sc_misc_unique_id(sc_ipc_t ipc, uint32_t *id_l, uint32_t *id_h)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_UNIQUE_ID);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (id_l != NULL) {
		*id_l = RPC_U32(&msg, 0U);
	}

	if (id_h != NULL) {
		*id_h = RPC_U32(&msg, 4U);
	}

	return;
}

sc_err_t sc_misc_set_ari(sc_ipc_t ipc, sc_rsrc_t resource,
			 sc_rsrc_t resource_mst, uint16_t ari, sc_bool_t enable)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SET_ARI);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U16(&msg, 2U) = U16(resource_mst);
	RPC_U16(&msg, 4U) = U16(ari);
	RPC_U8(&msg, 6U) = B2U8(enable);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

void sc_misc_boot_status(sc_ipc_t ipc, sc_misc_boot_status_t status)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_BOOT_STATUS);
	RPC_U8(&msg, 0U) = U8(status);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_TRUE);

	return;
}

sc_err_t sc_misc_boot_done(sc_ipc_t ipc, sc_rsrc_t cpu)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_BOOT_DONE);
	RPC_U16(&msg, 0U) = U16(cpu);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_otp_fuse_read(sc_ipc_t ipc, uint32_t word, uint32_t *val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_OTP_FUSE_READ);
	RPC_U32(&msg, 0U) = U32(word);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (val != NULL) {
		*val = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_otp_fuse_write(sc_ipc_t ipc, uint32_t word, uint32_t val)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_OTP_FUSE_WRITE);
	RPC_U32(&msg, 0U) = U32(word);
	RPC_U32(&msg, 4U) = U32(val);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

sc_err_t sc_misc_set_temp(sc_ipc_t ipc, sc_rsrc_t resource,
			  sc_misc_temp_t temp, int16_t celsius, int8_t tenths)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_SET_TEMP);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_I16(&msg, 2U) = I16(celsius);
	RPC_U8(&msg, 4U) = U8(temp);
	RPC_I8(&msg, 5U) = I8(tenths);
	RPC_SIZE(&msg) = 3U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

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
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_GET_TEMP);
	RPC_U16(&msg, 0U) = U16(resource);
	RPC_U8(&msg, 2U) = U8(temp);
	RPC_SIZE(&msg) = 2U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (celsius != NULL) {
		*celsius = RPC_I16(&msg, 0U);
	}

	result = RPC_R8(&msg);
	if (tenths != NULL) {
		*tenths = RPC_I8(&msg, 2U);
	}

	return (sc_err_t)result;
}

void sc_misc_get_boot_dev(sc_ipc_t ipc, sc_rsrc_t *dev)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_GET_BOOT_DEV);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (dev != NULL) {
		*dev = RPC_U16(&msg, 0U);
	}

	return;
}

sc_err_t sc_misc_get_boot_type(sc_ipc_t ipc, sc_misc_bt_t *type)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_GET_BOOT_TYPE);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	result = RPC_R8(&msg);
	if (type != NULL) {
		*type = RPC_U8(&msg, 0U);
	}

	return (sc_err_t)result;
}

void sc_misc_get_button_status(sc_ipc_t ipc, sc_bool_t *status)
{
	sc_rpc_msg_t msg;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_GET_BUTTON_STATUS);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (status != NULL) {
		*status = U2B(RPC_U8(&msg, 0U));
	}

	return;
}

sc_err_t sc_misc_rompatch_checksum(sc_ipc_t ipc, uint32_t *checksum)
{
	sc_rpc_msg_t msg;
	uint8_t result;

	RPC_VER(&msg) = SC_RPC_VERSION;
	RPC_SVC(&msg) = U8(SC_RPC_SVC_MISC);
	RPC_FUNC(&msg) = U8(MISC_FUNC_ROMPATCH_CHECKSUM);
	RPC_SIZE(&msg) = 1U;

	sc_call_rpc(ipc, &msg, SC_FALSE);

	if (checksum != NULL) {
		*checksum = RPC_U32(&msg, 0U);
	}

	result = RPC_R8(&msg);
	return (sc_err_t)result;
}

/**@}*/
