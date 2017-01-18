/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file containing the public API for the System Controller (SC)
 * Power Management (PM) function. This includes functions for power state
 * control, clock control, reset control, and wake-up event control.
 *
 * @addtogroup PM_SVC (SVC) Power Management Service
 *
 * Module for the Power Management (PM) service.
 *
 * @{
 */

#ifndef _SC_PM_API_H
#define _SC_PM_API_H

/* Includes */

#include <soc/imx8/sc/types.h>
#include <soc/imx8/sc/svc/rm/api.h>

/* Defines */

/*!
 * @name Defines for type widths
 */
/*@{*/
#define SC_PM_POWER_MODE_W      2	/* Width of sc_pm_power_mode_t */
#define SC_PM_CLOCK_MODE_W      3	/* Width of sc_pm_clock_mode_t */
#define SC_PM_RESET_TYPE_W      2	/* Width of sc_pm_reset_type_t */
#define SC_PM_RESET_REASON_W    3	/* Width of sc_pm_reset_reason_t */
/*@}*/

/*!
 * @name Defines for clock indexes (sc_pm_clk_t)
 */
/*@{*/
/*@}*/

/*!
 * @name Defines for ALL parameters
 */
/*@{*/
#define SC_PM_CLK_ALL   UINT8_MAX	/* All clocks */
/*@}*/

/*!
 * @name Defines for sc_pm_power_mode_t
 */
/*@{*/
#define SC_PM_PW_MODE_OFF       0	/* Power off */
#define SC_PM_PW_MODE_STBY      1	/* Power in standby */
#define SC_PM_PW_MODE_LP        2	/* Power in low-power */
#define SC_PM_PW_MODE_ON        3	/* Power on */
/*@}*/

/*!
 * @name Defines for sc_pm_clk_t
 */
/*@{*/
#define SC_PM_CLK_SLV_BUS       0	/* Slave bus clock */
#define SC_PM_CLK_MST_BUS       1	/* Master bus clock */
#define SC_PM_CLK_PER           2	/* Peripheral clock */
#define SC_PM_CLK_PHY           3	/* Phy clock */
#define SC_PM_CLK_MISC          4	/* Misc clock */
#define SC_PM_CLK_MISC0         0	/* Misc 0 clock */
#define SC_PM_CLK_MISC1         1	/* Misc 1 clock */
#define SC_PM_CLK_MISC2         2	/* Misc 2 clock */
#define SC_PM_CLK_MISC3         3	/* Misc 3 clock */
#define SC_PM_CLK_MISC4         4	/* Misc 4 clock */
#define SC_PM_CLK_CPU           2	/* CPU clock */
#define SC_PM_CLK_PLL           4	/* PLL */
#define SC_PM_CLK_BYPASS        4	/* Bypass clock */
/*@}*/

/*!
 * @name Defines for sc_pm_clk_mode_t
 */
/*@{*/
#define SC_PM_CLK_MODE_ROM_INIT        0	/* Clock is initialized by ROM. */
#define SC_PM_CLK_MODE_OFF             1	/* Clock is disabled */
#define SC_PM_CLK_MODE_ON              2	/* Clock is enabled. */
#define SC_PM_CLK_MODE_AUTOGATE_SW     3	/* Clock is in SW autogate mode */
#define SC_PM_CLK_MODE_AUTOGATE_HW     4	/* Clock is in HW autogate mode */
#define SC_PM_CLK_MODE_AUTOGATE_SW_HW  5	/* Clock is in SW-HW autogate mode */
/*@}*/

/*!
 * @name Defines for sc_pm_clk_parent_t
 */
/*@{*/
#define SC_PM_PARENT_XTAL              0	/* Parent is XTAL. */
#define SC_PM_PARENT_PLL0              1	/* Parent is PLL0 */
#define SC_PM_PARENT_PLL1              2	/* Parent is PLL1 or PLL0/2 */
#define SC_PM_PARENT_PLL2              3	/* Parent in PLL2 or PLL0/4 */
#define SC_PM_PARENT_BYPS              4	/* Parent is a bypass clock. */
/*@}*/

/*!
 * @name Defines for sc_pm_reset_type_t
 */
/*@{*/
#define SC_PM_RESET_TYPE_COLD          0	/* Cold reset */
#define SC_PM_RESET_TYPE_WARM          1	/* Warm reset */
#define SC_PM_RESET_TYPE_BOARD         2	/* Board reset */
/*@}*/

/*!
 * @name Defines for sc_pm_reset_reason_t
 */
/*@{*/
#define SC_PM_RESET_REASON_POR         0	/* Power on reset */
#define SC_PM_RESET_REASON_WARM        1	/* Warm reset */
#define SC_PM_RESET_REASON_SW          2	/* Software reset */
#define SC_PM_RESET_REASON_WDOG        3	/* Watchdog reset */
#define SC_PM_RESET_REASON_LOCKUP      4	/* Lockup reset */
#define SC_PM_RESET_REASON_TAMPER      5	/* Tamper reset */
#define SC_PM_RESET_REASON_TEMP        6	/* Temp reset */
#define SC_PM_RESET_REASON_LOW_VOLT    7	/* Low voltage reset */
/*@}*/

/*!
 * @name Defines for sc_pm_sys_if_t
 */
/*@{*/
#define SC_PM_SYS_IF_INTERCONNECT       0	/* System interconnect */
#define SC_PM_SYS_IF_MU                 1	/* AP -> SCU message units */
#define SC_PM_SYS_IF_OCMEM              2	/* On-chip memory (ROM/OCRAM) */
#define SC_PM_SYS_IF_DDR                3	/* DDR memory */
/*@}*/

/* Types */

/*!
 * This type is used to declare a power mode. Note resources only use
 * SC_PM_PW_MODE_OFF and SC_PM_PW_MODE_ON. The other modes are used only
 * as system power modes.
 */
typedef uint8_t sc_pm_power_mode_t;

/*!
 * This type is used to declare a clock.
 */
typedef uint8_t sc_pm_clk_t;

/*!
 * This type is used to declare a clock mode.
 */
typedef uint8_t sc_pm_clk_mode_t;

/*!
 * This type is used to declare the clock parent.
 */
typedef uint8_t sc_pm_clk_parent_t;

/*!
 * This type is used to declare clock rates.
 */
typedef uint32_t sc_pm_clock_rate_t;

/*!
 * This type is used to declare a desired reset type.
 */
typedef uint8_t sc_pm_reset_type_t;

/*!
 * This type is used to declare a reason for a reset.
 */
typedef uint8_t sc_pm_reset_reason_t;

/*!
 * This type is used to specify a system-level interface to be power managed.
 */
typedef uint8_t sc_pm_sys_if_t;

/* Functions */

/*!
 * @name Power Functions
 * @{
 */

/*!
 * This function sets the system power mode. Only the owner of the
 * SC_R_SYSTEM resource can do this.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     mode        power mode to apply
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid mode,
 * - SC_ERR_NOACCESS if caller not the owner of SC_R_SYSTEM
 *
 * @see sc_pm_set_sys_power_mode().
 */
sc_err_t sc_pm_set_sys_power_mode(sc_ipc_t ipc, sc_pm_power_mode_t mode);

/*!
 * This function sets the power mode of a partition.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     pt          handle of partition
 * @param[in]     mode        power mode to apply
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid partition or mode,
 * - SC_ERR_NOACCESS if caller's partition is not the owner or
 *   parent of \a pt
 *
 * All resources owned by \a pt that are on will have their power
 * mode changed to \a mode.
 *
 * @see sc_pm_set_resource_power_mode().
 */
sc_err_t sc_pm_set_partition_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
					sc_pm_power_mode_t mode);

/*!
 * This function gets the power mode of a partition.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     pt          handle of partition
 * @param[out]    mode        pointer to return power mode
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid partition
 */
sc_err_t sc_pm_get_sys_power_mode(sc_ipc_t ipc, sc_rm_pt_t pt,
				  sc_pm_power_mode_t *mode);

/*!
 * This function sets the power mode of a resource.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     mode        power mode to apply
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or mode,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner
 *
 * Resources set to SC_PM_PW_MODE_ON will reflect the
 * power mode of the partition and will change as that changes.
 *
 * Note some resources are still not accessible even when powered up if bus
 * transactions go through a fabric not powered up. Examples of this are
 * resources in display and capture subsystems which require the display
 * controller or the imaging subsytem to be powered up first.
 *
 *  @see sc_pm_set_partition_power_mode().
 */
sc_err_t sc_pm_set_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t mode);

/*!
 * This function gets the power mode of a resource.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[out]    mode        pointer to return power mode
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Note only SC_PM_PW_MODE_OFF and SC_PM_PW_MODE_ON are valid. The value
 * returned does not reflect the power mode of the partition..
 */
sc_err_t sc_pm_get_resource_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				       sc_pm_power_mode_t *mode);

/*!
 * This function requests the low power mode some of the resources
 * can enter based on their state. This API is only valid for the
 * following resources : SC_R_A53, SC_R_A53_0, SC_R_A53_1, SC_A53_2,
 * SC_A53_3, SC_R_A72, SC_R_A72_0, SC_R_A72_1, SC_R_CC1, SC_R_A35,
 * SC_R_A35_0, SC_R_A35_1, SC_R_A35_2, SC_R_A35_3.
 * For all other resources it will return SC_ERR_PARAM.
 * This function will set the low power mode the cores, cluster
 * and cluster associated resources will enter when all the cores
 * in a given cluster execute WFI
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[out]    mode        pointer to return power mode
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 */
sc_err_t sc_pm_req_low_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				  sc_pm_power_mode_t mode);

/*!
 * This function is used to set the resume address of a CPU.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the CPU resource
 * @param[in]     address     64-bit resume address
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or address,
 * - SC_ERR_NOACCESS if caller's partition is not the parent of the
 *   resource (CPU) owner
 */
sc_err_t sc_pm_set_cpu_resume_addr(sc_ipc_t ipc, sc_rsrc_t resource,
				   sc_faddr_t address);

/*!
 * This function requests the power mode configuration for system-level
 * interfaces including messaging units, interconnect, and memories.  This API
 * is only valid for the following resources : SC_R_A53, SC_R_A72, and
 * SC_R_M4_x_PID_y.  For all other resources, it will return SC_ERR_PARAM.
 * The requested power mode will be captured and applied to system-level
 * resources as system conditions allow.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     sys_if      system-level interface to be configured
 * @param[in]     hpm         high-power mode for the system interface
 * @param[in]     lpm         low-power mode for the system interface
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 */
sc_err_t sc_pm_req_sys_if_power_mode(sc_ipc_t ipc, sc_rsrc_t resource,
				     sc_pm_sys_if_t sys_if,
				     sc_pm_power_mode_t hpm,
				     sc_pm_power_mode_t lpm);

/* @} */

/*!
 * @name Clock/PLL Functions
 * @{
 */

/*!
 * This function sets the rate of a resource's clock/PLL.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     clk         clock/PLL to affect
 * @param[in,out] rate        pointer to rate to set,
 *                            return actual rate
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or clock/PLL,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner,
 * - SC_ERR_UNAVAILABLE if clock/PLL not applicable to this resource,
 * - SC_ERR_LOCKED if rate locked (usually because shared clock/PLL)
 *
 * Refer to the [Clock List](@ref CLOCKS) for valid clock/PLL values.
 */
sc_err_t sc_pm_set_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate);

/*!
 * This function gets the rate of a resource's clock/PLL.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     clk         clock/PLL to affect
 * @param[out]    rate        pointer to return rate
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or clock/PLL,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner,
 * - SC_ERR_UNAVAILABLE if clock/PLL not applicable to this resource
 *
 * Refer to the [Clock List](@ref CLOCKS) for valid clock/PLL values.
 */
sc_err_t sc_pm_get_clock_rate(sc_ipc_t ipc, sc_rsrc_t resource,
			      sc_pm_clk_t clk, sc_pm_clock_rate_t *rate);

/*!
 * This function enables/disables a resource's clock.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     clk         clock to affect
 * @param[in]     enable      enable if true; otherwise disabled
 * @param[in]     autog       HW auto clock gating
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or clock,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner,
 * - SC_ERR_UNAVAILABLE if clock not applicable to this resource
 *
 * Refer to the [Clock List](@ref CLOCKS) for valid clock values.
 */
sc_err_t sc_pm_clock_enable(sc_ipc_t ipc, sc_rsrc_t resource,
			    sc_pm_clk_t clk, bool enable, bool autog);

/*!
 * This function sets the parent of a resource's clock.
 * This function should only be called when the clock is disabled.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     clk         clock to affect
 * @param[in]     parent      New parent of the clock.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or clock,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner,
 * - SC_ERR_UNAVAILABLE if clock not applicable to this resource
 * - SC_ERR_BUSY if clock is currently enabled.
 *
 * Refer to the [Clock List](@ref CLOCKS) for valid clock values.
 */
sc_err_t sc_pm_set_clock_parent(sc_ipc_t ipc, sc_rsrc_t resource,
				sc_pm_clk_t clk, sc_pm_clk_parent_t parent);

/*!
 * This function gets the parent of a resource's clock.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the resource
 * @param[in]     clk         clock to affect
 * @param[out]     parent     pointer to return parent of clock.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or clock,
 * - SC_ERR_NOACCESS if caller's partition is not the resource owner
 *   or parent of the owner,
 * - SC_ERR_UNAVAILABLE if clock not applicable to this resource
 *
 * Refer to the [Clock List](@ref CLOCKS) for valid clock values.
 */
sc_err_t sc_pm_get_clock_parent(sc_ipc_t ipc, sc_rsrc_t resource,
				sc_pm_clk_t clk, sc_pm_clk_parent_t * parent);

/* @} */

/*!
 * @name Reset Functions
 * @{
 */

/*!
 * This function is used to reset the system. Only the owner of the
 * SC_R_SYSTEM resource can do this.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     type        reset type
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid type,
 * - SC_ERR_NOACCESS if caller not the owner of SC_R_SYSTEM
 *
 * If this function returns, then the reset did not occur due to an
 * invalid parameter.
 */
sc_err_t sc_pm_reset(sc_ipc_t ipc, sc_pm_reset_type_t type);

/*!
 * This function gets a caller's reset reason.
 *
 * @param[in]     ipc         IPC handle
 * @param[out]    reason      pointer to return reset reason
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 */
sc_err_t sc_pm_reset_reason(sc_ipc_t ipc, sc_pm_reset_reason_t *reason);

/*!
 * This function is used to boot a partition.
 *
 * @param[in]     ipc          IPC handle
 * @param[in]     pt           handle of partition to boot
 * @param[in]     resource_cpu ID of the CPU resource to start
 * @param[in]     boot_addr    64-bit boot address
 * @param[in]     resource_mu  ID of the MU that must be powered
 * @param[in]     resource_dev ID of the boot device that must be powered
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid partition, resource, or addr,
 * - SC_ERR_NOACCESS if caller's partition is not the parent of the
 *   partition to boot
 */
sc_err_t sc_pm_boot(sc_ipc_t ipc, sc_rm_pt_t pt,
		    sc_rsrc_t resource_cpu, sc_faddr_t boot_addr,
		    sc_rsrc_t resource_mu, sc_rsrc_t resource_dev);

/*!
 * This function is used to reboot the caller's partition.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     type        reset type
 *
 * If \a type is SC_PM_RESET_TYPE_COLD, then most peripherals owned by
 * the calling partition will be reset if possible. SC state (partitions,
 * power, clocks, etc.) is reset. The boot SW of the booting CPU must be
 * able to handle peripherals that that are not reset.
 *
 * If \a type is SC_PM_RESET_TYPE_WARM, then only the boot CPU is reset.
 * SC state (partitions, power, clocks, etc.) are NOT reset. The boot SW
 * of the booting CPU must be able to handle peripherals and SC state that
 * that are not reset.
 *
 * If \a type is SC_PM_RESET_TYPE_BOARD, then return with no action.
 *
 * If this function returns, then the reset did not occur due to an
 * invalid parameter.
 */
void sc_pm_reboot(sc_ipc_t ipc, sc_pm_reset_type_t type);

/*!
 * This function is used to reboot a partition.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     pt          handle of partition to reboot
 * @param[in]     type        reset type
 *
 * If \a type is SC_PM_RESET_TYPE_COLD, then most peripherals owned by
 * the calling partition will be reset if possible. SC state (partitions,
 * power, clocks, etc.) is reset. The boot SW of the booting CPU must be
 * able to handle peripherals that that are not reset.
 *
 * If \a type is SC_PM_RESET_TYPE_WARM, then only the boot CPU is reset.
 * SC state (partitions, power, clocks, etc.) are NOT reset. The boot SW
 * of the booting CPU must be able to handle peripherals and SC state that
 * that are not reset.
 *
 * If \a type is SC_PM_RESET_TYPE_BOARD, then return with no action.
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid partition or type
 * - SC_ERR_NOACCESS if caller's partition is not the parent of \a pt,
 *
 * Most peripherals owned by the partition will be reset if
 * possible. SC state (partitions, power, clocks, etc.) is reset. The
 * boot SW of the booting CPU must be able to handle peripherals that
 * that are not reset.
 */
sc_err_t sc_pm_reboot_partition(sc_ipc_t ipc, sc_rm_pt_t pt,
				sc_pm_reset_type_t type);

/*!
 * This function is used to start/stop a CPU.
 *
 * @param[in]     ipc         IPC handle
 * @param[in]     resource    ID of the CPU resource
 * @param[in]     enable      start if true; otherwise stop
 * @param[in]     address     64-bit boot address
 *
 * @return Returns an error code (SC_ERR_NONE = success).
 *
 * Return errors:
 * - SC_ERR_PARM if invalid resource or address,
 * - SC_ERR_NOACCESS if caller's partition is not the parent of the
 *   resource (CPU) owner
 */
sc_err_t sc_pm_cpu_start(sc_ipc_t ipc, sc_rsrc_t resource, bool enable,
			 sc_faddr_t address);

/* @} */

#endif				/* _SC_PM_API_H */

/**@}*/
