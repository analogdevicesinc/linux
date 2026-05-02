/* SPDX-License-Identifier: GPL-2.0 */
/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *
 ******************************************************************************/
#ifndef __HAL_VERSION_DEF_H__
#define __HAL_VERSION_DEF_H__

/*  hal_ic_type_e */
enum hal_ic_type_e { /* tag_HAL_IC_Type_Definition */
	CHIP_8723B	=	8,
};

/* hal_chip_type_e */
enum hal_chip_type_e { /* tag_HAL_CHIP_Type_Definition */
	TEST_CHIP		=	0,
	NORMAL_CHIP	=	1,
	FPGA			=	2,
};

/* hal_cut_version_e */
enum hal_cut_version_e { /* tag_HAL_Cut_Version_Definition */
	A_CUT_VERSION		=	0,
	B_CUT_VERSION		=	1,
	C_CUT_VERSION		=	2,
	D_CUT_VERSION		=	3,
	E_CUT_VERSION		=	4,
	F_CUT_VERSION		=	5,
	G_CUT_VERSION		=	6,
	H_CUT_VERSION		=	7,
	I_CUT_VERSION		=	8,
	J_CUT_VERSION		=	9,
	K_CUT_VERSION		=	10,
};

/*  HAL_Manufacturer */
enum hal_vendor_e { /* tag_HAL_Manufacturer_Version_Definition */
	CHIP_VENDOR_TSMC	=	0,
	CHIP_VENDOR_UMC		=	1,
	CHIP_VENDOR_SMIC	=	2,
};

struct hal_version { /* tag_HAL_VERSION */
	bool chip_normal;	/* true - normal chip, false - test chip */
};

/* hal_version			VersionID; */


#endif
