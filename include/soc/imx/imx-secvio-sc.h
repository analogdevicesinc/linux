/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef _MISC_IMX_SECVIO_SC_H_
#define _MISC_IMX_SECVIO_SC_H_

#include <linux/kernel.h>
#include <linux/notifier.h>

/* Bitmask of the security violation status bit in the HPSVS register */
#define HPSVS__LP_SEC_VIO__MASK BIT(31)
#define HPSVS__SW_LPSV__MASK    BIT(15)
#define HPSVS__SW_FSV__MASK     BIT(14)
#define HPSVS__SW_SV__MASK      BIT(13)
#define HPSVS__SV5__MASK        BIT(5)
#define HPSVS__SV4__MASK        BIT(4)
#define HPSVS__SV3__MASK        BIT(3)
#define HPSVS__SV2__MASK        BIT(2)
#define HPSVS__SV1__MASK        BIT(1)
#define HPSVS__SV0__MASK        BIT(0)

/* Bitmask of all security violation status bit in the HPSVS register */
#define HPSVS__ALL_SV__MASK (HPSVS__LP_SEC_VIO__MASK | \
			     HPSVS__SW_LPSV__MASK | \
			     HPSVS__SW_FSV__MASK | \
			     HPSVS__SW_SV__MASK | \
			     HPSVS__SV5__MASK | \
			     HPSVS__SV4__MASK | \
			     HPSVS__SV3__MASK | \
			     HPSVS__SV2__MASK | \
			     HPSVS__SV1__MASK | \
			     HPSVS__SV0__MASK)

/*
 * Bitmask of the security violation and tampers status bit in the LPS register
 */
#define LPS__ESVD__MASK  BIT(16)
#define LPS__ET2D__MASK  BIT(10)
#define LPS__ET1D__MASK  BIT(9)
#define LPS__WMT2D__MASK BIT(8)
#define LPS__WMT1D__MASK BIT(7)
#define LPS__VTD__MASK   BIT(6)
#define LPS__TTD__MASK   BIT(5)
#define LPS__CTD__MASK   BIT(4)
#define LPS__PGD__MASK   BIT(3)
#define LPS__MCR__MASK   BIT(2)
#define LPS__SRTCR__MASK BIT(1)
#define LPS__LPTA__MASK  BIT(0)

/*
 * Bitmask of all security violation and tampers status bit in the LPS register
 */
#define LPS__ALL_TP__MASK (LPS__ESVD__MASK | \
			   LPS__ET2D__MASK | \
			   LPS__ET1D__MASK | \
			   LPS__WMT2D__MASK | \
			   LPS__WMT1D__MASK | \
			   LPS__VTD__MASK | \
			   LPS__TTD__MASK | \
			   LPS__CTD__MASK | \
			   LPS__PGD__MASK | \
			   LPS__MCR__MASK | \
			   LPS__SRTCR__MASK | \
			   LPS__LPTA__MASK)

/*
 * Bitmask of the security violation and tampers status bit in the LPTDS
 * register
 */
#define LPTDS__ET10D__MASK  BIT(7)
#define LPTDS__ET9D__MASK   BIT(6)
#define LPTDS__ET8D__MASK   BIT(5)
#define LPTDS__ET7D__MASK   BIT(4)
#define LPTDS__ET6D__MASK   BIT(3)
#define LPTDS__ET5D__MASK   BIT(2)
#define LPTDS__ET4D__MASK   BIT(1)
#define LPTDS__ET3D__MASK   BIT(0)

/*
 * Bitmask of all security violation and tampers status bit in the LPTDS
 * register
 */
#define LPTDS__ALL_TP__MASK (LPTDS__ET10D__MASK | \
			     LPTDS__ET9D__MASK | \
			     LPTDS__ET8D__MASK | \
			     LPTDS__ET7D__MASK | \
			     LPTDS__ET6D__MASK | \
			     LPTDS__ET5D__MASK | \
			     LPTDS__ET4D__MASK | \
			     LPTDS__ET3D__MASK)

/* Struct for notification */
/**
 * struct secvio_sc_notifier_info - Information about the status of the SNVS
 * @hpsvs:   status from register HPSVS
 * @lps: status from register LPS
 * @lptds: status from register LPTDS
 */
struct secvio_sc_notifier_info {
	u32 hpsvs;
	u32 lps;
	u32 lptds;
};

/**
 * register_imx_secvio_sc_notifier() - Register a notifier
 *
 * @nb: The notifier block structure
 *
 * Register a function to notify to the imx-secvio-sc module. The function
 * will be notified when a check of the state of the SNVS happens: called by
 * a user or triggered by an interruption form the SNVS.
 *
 * The struct secvio_sc_notifier_info is passed as data to the notifier.
 *
 * Return: 0 in case of success
 */
int register_imx_secvio_sc_notifier(struct notifier_block *nb);

/**
 * unregister_imx_secvio_sc_notifier() - Unregister a notifier
 *
 * @nb: The notifier block structure
 *
 * Return: 0 in case of success
 */
int unregister_imx_secvio_sc_notifier(struct notifier_block *nb);

/**
 * imx_secvio_sc_get_state() - Get the state of the SNVS
 *
 * @info: The structure containing the state of the SNVS
 *
 * Return: 0 in case of success
 */
int imx_secvio_sc_get_state(struct secvio_sc_notifier_info *info);

/**
 * imx_secvio_sc_check_state() - Check the state of the SNVS
 *
 * If a security violation or a tamper is detected, the list of notifier
 * (registered using register_imx_secvio_sc_notifier() ) will be called
 *
 * Return: 0 in case of success
 */
int imx_secvio_sc_check_state(void);

/**
 * imx_secvio_sc_clear_state() - Clear the state of the SNVS
 *
 * @hpsvs: Value to write to HPSVS register
 * @lps:   Value to write to LPS register
 * @lptds: Value to write to LPTDSregister
 *
 * The function will write the value provided to the corresponding register
 * which will clear the status of the bits set.
 *
 * Return: 0 in case of success
 */
int imx_secvio_sc_clear_state(u32 hpsvs, u32 lps, u32 lptds);

/* Commands of the ioctl interface */
enum ioctl_cmd_t {
	GET_STATE,
	CHECK_STATE,
	CLEAR_STATE,
};

/* Definition for the ioctl interface */
#define IMX_SECVIO_SC_GET_STATE   _IOR('S', GET_STATE, \
				struct secvio_sc_notifier_info)
#define IMX_SECVIO_SC_CHECK_STATE _IO('S', CHECK_STATE)
#define IMX_SECVIO_SC_CLEAR_STATE _IOW('S', CLEAR_STATE, \
				struct secvio_sc_notifier_info)

#endif /* _MISC_IMX_SECVIO_SC_H_ */
