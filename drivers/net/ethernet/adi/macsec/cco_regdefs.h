// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2025, Analog Devices Incorporated, All Rights Reserved
 */
 
#ifndef _REGDEFS_H_
#define _REGDEFS_H_

#include "cco_ciphersuite_memmap.h"
#include "cco_macseccore_memmap.h"
#include "cco_receivesa_memmap.h"
#include "cco_receivesc_memmap.h"
#include "cco_secy_config_memmap.h"
#include "cco_statistics_memmap.h"
#include "cco_status_memmap.h"
#include "cco_traffic_map_memmap.h"
#include "cco_transmitsa_memmap.h"
#include "cco_transmitsc_memmap.h"

 /* IP_ID should be 0x4d435343 "MCSC" */
#define CCO_MACSEC_IP_ID     0x4d435343 // "MCSC"
#define CCO_MACSEC_MAJOR_VER 13

#endif // _REGDEFS_H_
