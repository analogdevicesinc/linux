/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#ifndef _SC_SCFW_H
#define _SC_SCFW_H

#include <linux/types.h>

/*!
 * This type is used to declare a handle for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef uint32_t sc_ipc_t;

/*!
 * This type is used to declare an ID for an IPC communication
 * channel. Its meaning is specific to the IPC implementation.
 */
typedef uint32_t sc_ipc_id_t;

/*!
 * This function returns the MU channel ID for this implementation
 *
 * @param[in]     ipc         pointer to Mu channel ID
 * @return Returns an error code (SC_ERR_NONE = success, SC_ERR_IPC
 *         otherwise).
 */
int sc_ipc_getMuID(uint32_t *mu_id);

#endif
