/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2020, 2024 NXP
 *
 * Header file containing the public API for the System Controller (SC)
 * Security Controller (SECO) function.
 *
 * SECO_SVC (SVC) Security Controller Service
 *
 * Module for the Security Controller (SECO) service.
 */

#ifndef _SC_SECO_API_H
#define _SC_SECO_API_H

#include <linux/errno.h>
#include <linux/firmware/imx/sci.h>

#define SECURE_RAM_BASE_ADDRESS         (0x31800000ULL)
#define SECURE_RAM_SIZE                 (0x10000ULL)
#define IMX_SC_RM_PERM_FULL             7U  /* Full access */

/*
 * This type is used to indicate RPC RM function calls.
 */
enum imx_sc_seco_func {
	IMX_SC_SECO_FUNC_UNKNOWN = 0,
	IMX_SC_SECO_FUNC_BUILD_INFO = 16,
	IMX_SC_SECO_FUNC_SAB_MSG = 23,
	IMX_SC_SECO_FUNC_SECVIO_ENABLE = 25,
	IMX_SC_SECO_FUNC_SECVIO_CONFIG = 26,
	IMX_SC_SECO_FUNC_SECVIO_DGO_CONFIG = 27,
};

#if IS_ENABLED(CONFIG_IMX_SCU)
int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit);
int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr);
int imx_sc_seco_secvio_enable(struct imx_sc_ipc *ipc);
int imx_sc_seco_secvio_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
			      u32 *data0, u32 *data1, u32 *data2, u32 *data3,
			      u32 *data4, u8 size);
int imx_sc_seco_secvio_dgo_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
				  u32 *data);
#else /* IS_ENABLED(CONFIG_IMX_SCU) */
static inline
int imx_sc_seco_build_info(struct imx_sc_ipc *ipc, uint32_t *version,
			   uint32_t *commit)
{
	return -EOPNOTSUPP;
}

static inline
int imx_sc_seco_sab_msg(struct imx_sc_ipc *ipc, u64 smsg_addr)
{
	return -EOPNOTSUPP;
}

static inline
int imx_sc_seco_secvio_enable(struct imx_sc_ipc *ipc)
{
	return -EOPNOTSUPP;
}

static inline
int imx_sc_seco_secvio_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
			      u32 *data0, u32 *data1, u32 *data2, u32 *data3,
			      u32 *data4, u8 size)
{
	return -EOPNOTSUPP;
}

static inline
int imx_sc_seco_secvio_dgo_config(struct imx_sc_ipc *ipc, u8 id, u8 access,
				  u32 *data)
{
	return -EOPNOTSUPP;
}

#endif /* IS_ENABLED(CONFIG_IMX_SCU) */

#endif /* _SC_SECO_API_H */
