/*
 * Copyright 2015-2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _CBUS_H_
#define _CBUS_H_

#define EMAC1_BASE_ADDR	(CBUS_BASE_ADDR + 0x200000)
#define EGPI1_BASE_ADDR	(CBUS_BASE_ADDR + 0x210000)
#define EMAC2_BASE_ADDR	(CBUS_BASE_ADDR + 0x220000)
#define EGPI2_BASE_ADDR	(CBUS_BASE_ADDR + 0x230000)
#define BMU1_BASE_ADDR	(CBUS_BASE_ADDR + 0x240000)
#define BMU2_BASE_ADDR	(CBUS_BASE_ADDR + 0x250000)
#define ARB_BASE_ADDR	(CBUS_BASE_ADDR + 0x260000)
#define DDR_CONFIG_BASE_ADDR	(CBUS_BASE_ADDR + 0x270000)
#define HIF_BASE_ADDR	(CBUS_BASE_ADDR + 0x280000)
#define HGPI_BASE_ADDR	(CBUS_BASE_ADDR + 0x290000)
#define LMEM_BASE_ADDR	(CBUS_BASE_ADDR + 0x300000)
#define LMEM_SIZE	0x10000
#define LMEM_END	(LMEM_BASE_ADDR + LMEM_SIZE)
#define TMU_CSR_BASE_ADDR	(CBUS_BASE_ADDR + 0x310000)
#define CLASS_CSR_BASE_ADDR	(CBUS_BASE_ADDR + 0x320000)
#define HIF_NOCPY_BASE_ADDR	(CBUS_BASE_ADDR + 0x350000)
#define UTIL_CSR_BASE_ADDR	(CBUS_BASE_ADDR + 0x360000)
#define CBUS_GPT_BASE_ADDR	(CBUS_BASE_ADDR + 0x370000)

/*
 * defgroup XXX_MEM_ACCESS_ADDR PE memory access through CSR
 * XXX_MEM_ACCESS_ADDR register bit definitions.
 */
#define PE_MEM_ACCESS_WRITE	BIT(31)	/* Internal Memory Write. */
#define PE_MEM_ACCESS_IMEM	BIT(15)
#define PE_MEM_ACCESS_DMEM	BIT(16)

/* Byte Enables of the Internal memory access. These are interpred in BE */
#define PE_MEM_ACCESS_BYTE_ENABLE(offset, size)	\
	({ typeof(size) size_ = (size);		\
	(((BIT(size_) - 1) << (4 - (offset) - (size_))) & 0xf) << 24; })

#include "cbus/emac_mtip.h"
#include "cbus/gpi.h"
#include "cbus/bmu.h"
#include "cbus/hif.h"
#include "cbus/tmu_csr.h"
#include "cbus/class_csr.h"
#include "cbus/hif_nocpy.h"
#include "cbus/util_csr.h"

/* PFE cores states */
#define CORE_DISABLE	0x00000000
#define CORE_ENABLE	0x00000001
#define CORE_SW_RESET	0x00000002

/* LMEM defines */
#define LMEM_HDR_SIZE	0x0010
#define LMEM_BUF_SIZE_LN2	0x7
#define LMEM_BUF_SIZE	BIT(LMEM_BUF_SIZE_LN2)

/* DDR defines */
#define DDR_HDR_SIZE	0x0100
#define DDR_BUF_SIZE_LN2	0xb
#define DDR_BUF_SIZE	BIT(DDR_BUF_SIZE_LN2)

#endif /* _CBUS_H_ */
