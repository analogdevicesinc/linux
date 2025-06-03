// SPDX-License-Identifier: GPL-2.0
/*
 * Xilinx Zynq MPSoC Firmware layer
 *
 *  Copyright (C) 2014-2022 Xilinx, Inc.
 *  Copyright (C) 2022 - 2023, Advanced Micro Devices, Inc.
 *
 *  Michal Simek <michal.simek@amd.com>
 *  Davorin Mista <davorin.mista@aggios.com>
 *  Jolly Shah <jollys@xilinx.com>
 *  Rajan Vaja <rajanv@xilinx.com>
 */

#include <linux/arm-smccc.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/hashtable.h>

#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/firmware/xlnx-event-manager.h>
#include "zynqmp-debug.h"

/* Max HashMap Order for PM API feature check (1<<7 = 128) */
#define PM_API_FEATURE_CHECK_MAX_ORDER  7

/* CRL registers and bitfields */
#define CRL_APB_BASE			0xFF5E0000U
/* BOOT_PIN_CTRL- Used to control the mode pins after boot */
#define CRL_APB_BOOT_PIN_CTRL		(CRL_APB_BASE + (0x250U))
/* BOOT_PIN_CTRL_MASK- out_val[11:8], out_en[3:0] */
#define CRL_APB_BOOTPIN_CTRL_MASK	0xF0FU

/* IOCTL/QUERY feature payload size */
#define FEATURE_PAYLOAD_SIZE		2

static bool feature_check_enabled;
static DEFINE_HASHTABLE(pm_api_features_map, PM_API_FEATURE_CHECK_MAX_ORDER);
static u32 ioctl_features[FEATURE_PAYLOAD_SIZE];
static u32 query_features[FEATURE_PAYLOAD_SIZE];

static struct platform_device *em_dev;

/**
 * struct zynqmp_devinfo - Structure for Zynqmp device instance
 * @dev:		Device Pointer
 * @feature_conf_id:	Feature conf id
 */
struct zynqmp_devinfo {
	struct device *dev;
	u32 feature_conf_id;
};

/**
 * struct pm_api_feature_data - PM API Feature data
 * @pm_api_id:		PM API Id, used as key to index into hashmap
 * @feature_status:	status of PM API feature: valid, invalid
 * @hentry:		hlist_node that hooks this entry into hashtable
 */
struct pm_api_feature_data {
	u32 pm_api_id;
	int feature_status;
	struct hlist_node hentry;
};

static const struct mfd_cell firmware_devs[] = {
	{
		.name = "zynqmp_power_controller",
	},
};

/**
 * zynqmp_pm_ret_code() - Convert PMU-FW error codes to Linux error codes
 * @ret_status:		PMUFW return code
 *
 * Return: corresponding Linux error code
 */
static int zynqmp_pm_ret_code(u32 ret_status)
{
	switch (ret_status) {
	case XST_PM_SUCCESS:
	case XST_PM_DOUBLE_REQ:
		return 0;
	case XST_PM_NO_FEATURE:
		return -ENOTSUPP;
	case XST_PM_INVALID_VERSION:
		return -EOPNOTSUPP;
	case XST_PM_NO_ACCESS:
		return -EACCES;
	case XST_PM_ABORT_SUSPEND:
		return -ECANCELED;
	case XST_PM_MULT_USER:
		return -EUSERS;
	case XST_PM_INTERNAL:
	case XST_PM_CONFLICT:
	case XST_PM_INVALID_NODE:
	case XST_PM_INVALID_CRC:
	default:
		return -EINVAL;
	}
}

static noinline int do_fw_call_fail(u32 *ret_payload, u32 num_args, ...)
{
	return -ENODEV;
}

/*
 * PM function call wrapper
 * Invoke do_fw_call_smc or do_fw_call_hvc, depending on the configuration
 */
static int (*do_fw_call)(u32 *ret_payload, u32, ...) = do_fw_call_fail;

/**
 * do_fw_call_smc() - Call system-level platform management layer (SMC)
 * @num_args:		Number of variable arguments should be <= 8
 * @ret_payload:	Returned value array
 *
 * Invoke platform management function via SMC call (no hypervisor present).
 *
 * Return: Returns status, either success or error+reason
 */
static noinline int do_fw_call_smc(u32 *ret_payload, u32 num_args, ...)
{
	struct arm_smccc_res res;
	u64 args[8] = {0};
	va_list arg_list;
	u8 i;

	if (num_args > 8)
		return -EINVAL;

	va_start(arg_list, num_args);

	for (i = 0; i < num_args; i++)
		args[i] = va_arg(arg_list, u64);

	va_end(arg_list);

	arm_smccc_smc(args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], &res);

	if (ret_payload) {
		ret_payload[0] = lower_32_bits(res.a0);
		ret_payload[1] = upper_32_bits(res.a0);
		ret_payload[2] = lower_32_bits(res.a1);
		ret_payload[3] = upper_32_bits(res.a1);
	}

	return zynqmp_pm_ret_code((enum pm_ret_status)res.a0);
}

/**
 * do_fw_call_hvc() - Call system-level platform management layer (HVC)
 * @num_args:		Number of variable arguments should be <= 8
 * @ret_payload:	Returned value array
 *
 * Invoke platform management function via HVC
 * HVC-based for communication through hypervisor
 * (no direct communication with ATF).
 *
 * Return: Returns status, either success or error+reason
 */
static noinline int do_fw_call_hvc(u32 *ret_payload, u32 num_args, ...)
{
	struct arm_smccc_res res;
	u64 args[8] = {0};
	va_list arg_list;
	u8 i;

	if (num_args > 8)
		return -EINVAL;

	va_start(arg_list, num_args);

	for (i = 0; i < num_args; i++)
		args[i] = va_arg(arg_list, u64);

	va_end(arg_list);

	arm_smccc_hvc(args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], &res);

	if (ret_payload) {
		ret_payload[0] = lower_32_bits(res.a0);
		ret_payload[1] = upper_32_bits(res.a0);
		ret_payload[2] = lower_32_bits(res.a1);
		ret_payload[3] = upper_32_bits(res.a1);
	}

	return zynqmp_pm_ret_code((enum pm_ret_status)res.a0);
}

static int __do_feature_check_call(const u32 api_id, u32 *ret_payload)
{
	int ret;
	u64 smc_arg[2];
	u32 module_id;
	u32 feature_check_api_id;

	module_id = FIELD_GET(MODULE_ID_MASK, api_id);

	/*
	 * Feature check of APIs belonging to PM, XSEM, and TF-A are handled by calling
	 * PM_FEATURE_CHECK API. For other modules, call PM_API_FEATURES API.
	 */
	if (module_id == PM_MODULE_ID || module_id == XSEM_MODULE_ID || module_id == TF_A_MODULE_ID)
		feature_check_api_id = PM_FEATURE_CHECK;
	else
		feature_check_api_id = PM_API_FEATURES;

	/*
	 * Feature check of TF-A APIs is done in the TF-A layer and it expects for
	 * MODULE_ID_MASK bits of SMC's arg[0] to be the same as PM_MODULE_ID.
	 */
	if (module_id == TF_A_MODULE_ID)
		module_id = PM_MODULE_ID;

	smc_arg[0] = PM_SIP_SVC | FIELD_PREP(MODULE_ID_MASK, module_id) | feature_check_api_id;
	smc_arg[1] = (api_id & API_ID_MASK);

	ret = do_fw_call(ret_payload, 2, smc_arg[0], smc_arg[1]);
	if (ret)
		ret = -EOPNOTSUPP;
	else
		ret = ret_payload[1];

	return ret;
}

static int do_feature_check_call(const u32 api_id)
{
	int ret;
	u32 ret_payload[PAYLOAD_ARG_CNT];
	struct pm_api_feature_data *feature_data;

	/* Check for existing entry in hash table for given api */
	hash_for_each_possible(pm_api_features_map, feature_data, hentry,
			       api_id) {
		if (feature_data->pm_api_id == api_id)
			return feature_data->feature_status;
	}

	/* Add new entry if not present */
	feature_data = kmalloc(sizeof(*feature_data), GFP_ATOMIC);
	if (!feature_data)
		return -ENOMEM;

	feature_data->pm_api_id = api_id;
	ret = __do_feature_check_call(api_id, ret_payload);

	feature_data->feature_status = ret;
	hash_add(pm_api_features_map, &feature_data->hentry, api_id);

	if (api_id == PM_IOCTL)
		/* Store supported IOCTL IDs mask */
		memcpy(ioctl_features, &ret_payload[2], FEATURE_PAYLOAD_SIZE * 4);
	else if (api_id == PM_QUERY_DATA)
		/* Store supported QUERY IDs mask */
		memcpy(query_features, &ret_payload[2], FEATURE_PAYLOAD_SIZE * 4);

	return ret;
}

/**
 * zynqmp_pm_feature() - Check whether given feature is supported or not and
 *			 store supported IOCTL/QUERY ID mask
 * @api_id:		API ID to check
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_feature(const u32 api_id)
{
	int ret;

	if (!feature_check_enabled)
		return 0;

	ret = do_feature_check_call(api_id);

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_feature);

/**
 * zynqmp_pm_is_function_supported() - Check whether given IOCTL/QUERY function
 *				       is supported or not
 * @api_id:		PM_IOCTL or PM_QUERY_DATA
 * @id:			IOCTL or QUERY function IDs
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_is_function_supported(const u32 api_id, const u32 id)
{
	int ret;
	u32 *bit_mask;

	/* Input arguments validation */
	if (id >= 64 || (api_id != PM_IOCTL && api_id != PM_QUERY_DATA))
		return -EINVAL;

	/* Check feature check API version */
	ret = do_feature_check_call(PM_FEATURE_CHECK);
	if (ret < 0)
		return ret;

	/* Check if feature check version 2 is supported or not */
	if ((ret & FIRMWARE_VERSION_MASK) == PM_API_VERSION_2) {
		/*
		 * Call feature check for IOCTL/QUERY API to get IOCTL ID or
		 * QUERY ID feature status.
		 */
		ret = do_feature_check_call(api_id);
		if (ret < 0)
			return ret;

		bit_mask = (api_id == PM_IOCTL) ? ioctl_features : query_features;

		if ((bit_mask[(id / 32)] & BIT((id % 32))) == 0U)
			return -EOPNOTSUPP;
	} else {
		return -ENODATA;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_is_function_supported);

/**
 * zynqmp_pm_invoke_fn() - Invoke the system-level platform management layer
 *			   caller function depending on the configuration
 * @pm_api_id:		Requested PM-API call
 * @ret_payload:	Returned value array
 * @num_args:		Number of arguments to requested PM-API call
 *
 * Invoke platform management function for SMC or HVC call, depending on
 * configuration.
 * Following SMC Calling Convention (SMCCC) for SMC64:
 * Pm Function Identifier,
 * PM_SIP_SVC + PM_API_ID =
 *	((SMC_TYPE_FAST << FUNCID_TYPE_SHIFT)
 *	((SMC_64) << FUNCID_CC_SHIFT)
 *	((SIP_START) << FUNCID_OEN_SHIFT)
 *	((PM_API_ID) & FUNCID_NUM_MASK))
 *
 * PM_SIP_SVC	- Registered ZynqMP SIP Service Call.
 * PM_API_ID	- Platform Management API ID.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_invoke_fn(u32 pm_api_id, u32 *ret_payload, u32 num_args, ...)
{
	/*
	 * Added SIP service call Function Identifier
	 * Make sure to stay in x0 register
	 */
	u64 smc_arg[8];
	int ret, i;
	va_list arg_list;
	u32 args[14] = {0};

	if (num_args > 14)
		return -EINVAL;

	va_start(arg_list, num_args);

	/* Check if feature is supported or not */
	ret = zynqmp_pm_feature(pm_api_id);
	if (ret < 0)
		return ret;

	for (i = 0; i < num_args; i++)
		args[i] = va_arg(arg_list, u32);

	va_end(arg_list);

	smc_arg[0] = PM_SIP_SVC | pm_api_id;
	for (i = 0; i < 7; i++)
		smc_arg[i + 1] = ((u64)args[(i * 2) + 1] << 32) | args[i * 2];

	return do_fw_call(ret_payload, 8, smc_arg[0], smc_arg[1], smc_arg[2], smc_arg[3],
			  smc_arg[4], smc_arg[5], smc_arg[6], smc_arg[7]);
}

static u32 pm_api_version;
static u32 pm_tz_version;
static u32 pm_family_code;
static u32 pm_sub_family_code;

int zynqmp_pm_register_sgi(u32 sgi_num, u32 reset)
{
	int ret;

	ret = zynqmp_pm_invoke_fn(TF_A_PM_REGISTER_SGI, NULL, 2, sgi_num, reset);
	if (ret != -EOPNOTSUPP && !ret)
		return ret;

	/* try old implementation as fallback strategy if above fails */
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, IOCTL_REGISTER_SGI, sgi_num, reset);
}

/**
 * zynqmp_pm_get_api_version() - Get version number of PMU PM firmware
 * @version:	Returned version value
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_get_api_version(u32 *version)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!version)
		return -EINVAL;

	/* Check is PM API version already verified */
	if (pm_api_version > 0) {
		*version = pm_api_version;
		return 0;
	}
	ret = zynqmp_pm_invoke_fn(PM_GET_API_VERSION, ret_payload, 0);
	*version = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_api_version);

/**
 * zynqmp_pm_get_chipid - Get silicon ID registers
 * @idcode:     IDCODE register
 * @version:    version register
 *
 * Return:      Returns the status of the operation and the idcode and version
 *              registers in @idcode and @version.
 */
int zynqmp_pm_get_chipid(u32 *idcode, u32 *version)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!idcode || !version)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_GET_CHIPID, ret_payload, 0);
	*idcode = ret_payload[1];
	*version = ret_payload[2];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_chipid);

/**
 * zynqmp_pm_get_family_info() - Get family info of platform
 * @family:	Returned family code value
 * @subfamily:	Returned sub-family code value
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_get_family_info(u32 *family, u32 *subfamily)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	u32 idcode;
	int ret;

	/* Check is family or sub-family code already received */
	if (pm_family_code && pm_sub_family_code) {
		*family = pm_family_code;
		*subfamily = pm_sub_family_code;
		return 0;
	}

	ret = zynqmp_pm_invoke_fn(PM_GET_CHIPID, ret_payload, 0);
	if (ret < 0)
		return ret;

	idcode = ret_payload[1];
	pm_family_code = FIELD_GET(FAMILY_CODE_MASK, idcode);
	pm_sub_family_code = FIELD_GET(SUB_FAMILY_CODE_MASK, idcode);
	*family = pm_family_code;
	*subfamily = pm_sub_family_code;

	return 0;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_family_info);

/**
 * zynqmp_pm_get_trustzone_version() - Get secure trustzone firmware version
 * @version:	Returned version value
 *
 * Return: Returns status, either success or error+reason
 */
static int zynqmp_pm_get_trustzone_version(u32 *version)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!version)
		return -EINVAL;

	/* Check is PM trustzone version already verified */
	if (pm_tz_version > 0) {
		*version = pm_tz_version;
		return 0;
	}
	ret = zynqmp_pm_invoke_fn(PM_GET_TRUSTZONE_VERSION, ret_payload, 0);
	*version = ret_payload[1];

	return ret;
}

/**
 * get_set_conduit_method() - Choose SMC or HVC based communication
 * @np:		Pointer to the device_node structure
 *
 * Use SMC or HVC-based functions to communicate with EL2/EL3.
 *
 * Return: Returns 0 on success or error code
 */
static int get_set_conduit_method(struct device_node *np)
{
	const char *method;

	if (of_property_read_string(np, "method", &method)) {
		pr_warn("%s missing \"method\" property\n", __func__);
		return -ENXIO;
	}

	if (!strcmp("hvc", method)) {
		do_fw_call = do_fw_call_hvc;
	} else if (!strcmp("smc", method)) {
		do_fw_call = do_fw_call_smc;
	} else {
		pr_warn("%s Invalid \"method\" property: %s\n",
			__func__, method);
		return -EINVAL;
	}

	return 0;
}

/**
 * zynqmp_pm_query_data() - Get query data from firmware
 * @qdata:	Variable to the zynqmp_pm_query_data structure
 * @out:	Returned output value
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_query_data(struct zynqmp_pm_query_data qdata, u32 *out)
{
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_QUERY_DATA, out, 4, qdata.qid, qdata.arg1, qdata.arg2,
				  qdata.arg3);

	/*
	 * For clock name query, all bytes in SMC response are clock name
	 * characters and return code is always success. For invalid clocks,
	 * clock name bytes would be zeros.
	 */
	return qdata.qid == PM_QID_CLOCK_GET_NAME ? 0 : ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_query_data);

/**
 * zynqmp_pm_clock_enable() - Enable the clock for given id
 * @clock_id:	ID of the clock to be enabled
 *
 * This function is used by master to enable the clock
 * including peripherals and PLL clocks.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_enable(u32 clock_id)
{
	return zynqmp_pm_invoke_fn(PM_CLOCK_ENABLE, NULL, 1, clock_id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_enable);

/**
 * zynqmp_pm_clock_disable() - Disable the clock for given id
 * @clock_id:	ID of the clock to be disable
 *
 * This function is used by master to disable the clock
 * including peripherals and PLL clocks.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_disable(u32 clock_id)
{
	return zynqmp_pm_invoke_fn(PM_CLOCK_DISABLE, NULL, 1, clock_id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_disable);

/**
 * zynqmp_pm_clock_getstate() - Get the clock state for given id
 * @clock_id:	ID of the clock to be queried
 * @state:	1/0 (Enabled/Disabled)
 *
 * This function is used by master to get the state of clock
 * including peripherals and PLL clocks.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_getstate(u32 clock_id, u32 *state)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_CLOCK_GETSTATE, ret_payload, 1, clock_id);
	*state = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_getstate);

/**
 * zynqmp_pm_clock_setdivider() - Set the clock divider for given id
 * @clock_id:	ID of the clock
 * @divider:	divider value
 *
 * This function is used by master to set divider for any clock
 * to achieve desired rate.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_setdivider(u32 clock_id, u32 divider)
{
	return zynqmp_pm_invoke_fn(PM_CLOCK_SETDIVIDER, NULL, 2, clock_id, divider);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_setdivider);

/**
 * zynqmp_pm_clock_getdivider() - Get the clock divider for given id
 * @clock_id:	ID of the clock
 * @divider:	divider value
 *
 * This function is used by master to get divider values
 * for any clock.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_getdivider(u32 clock_id, u32 *divider)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_CLOCK_GETDIVIDER, ret_payload, 1, clock_id);
	*divider = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_getdivider);

/**
 * zynqmp_pm_clock_setparent() - Set the clock parent for given id
 * @clock_id:	ID of the clock
 * @parent_id:	parent id
 *
 * This function is used by master to set parent for any clock.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_setparent(u32 clock_id, u32 parent_id)
{
	return zynqmp_pm_invoke_fn(PM_CLOCK_SETPARENT, NULL, 2, clock_id, parent_id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_setparent);

/**
 * zynqmp_pm_clock_getparent() - Get the clock parent for given id
 * @clock_id:	ID of the clock
 * @parent_id:	parent id
 *
 * This function is used by master to get parent index
 * for any clock.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_clock_getparent(u32 clock_id, u32 *parent_id)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_CLOCK_GETPARENT, ret_payload, 1, clock_id);
	*parent_id = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_clock_getparent);

/**
 * zynqmp_pm_set_pll_frac_mode() - PM API for set PLL mode
 *
 * @clk_id:	PLL clock ID
 * @mode:	PLL mode (PLL_MODE_FRAC/PLL_MODE_INT)
 *
 * This function sets PLL mode
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_set_pll_frac_mode(u32 clk_id, u32 mode)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_SET_PLL_FRAC_MODE, clk_id, mode);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_pll_frac_mode);

/**
 * zynqmp_pm_get_pll_frac_mode() - PM API for get PLL mode
 *
 * @clk_id:	PLL clock ID
 * @mode:	PLL mode
 *
 * This function return current PLL mode
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_get_pll_frac_mode(u32 clk_id, u32 *mode)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, mode, 3, 0, IOCTL_GET_PLL_FRAC_MODE, clk_id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_pll_frac_mode);

/**
 * zynqmp_pm_set_pll_frac_data() - PM API for setting pll fraction data
 *
 * @clk_id:	PLL clock ID
 * @data:	fraction data
 *
 * This function sets fraction data.
 * It is valid for fraction mode only.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_set_pll_frac_data(u32 clk_id, u32 data)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_SET_PLL_FRAC_DATA, clk_id, data);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_pll_frac_data);

/**
 * zynqmp_pm_get_pll_frac_data() - PM API for getting pll fraction data
 *
 * @clk_id:	PLL clock ID
 * @data:	fraction data
 *
 * This function returns fraction data value.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_get_pll_frac_data(u32 clk_id, u32 *data)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, data, 3, 0, IOCTL_GET_PLL_FRAC_DATA, clk_id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_pll_frac_data);

/**
 * zynqmp_pm_set_sd_tapdelay() -  Set tap delay for the SD device
 *
 * @node_id:	Node ID of the device
 * @type:	Type of tap delay to set (input/output)
 * @value:	Value to set fot the tap delay
 *
 * This function sets input/output tap delay for the SD device.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_set_sd_tapdelay(u32 node_id, u32 type, u32 value)
{
	u32 reg = (type == PM_TAPDELAY_INPUT) ? SD_ITAPDLY : SD_OTAPDLYSEL;
	u32 mask = (node_id == NODE_SD_0) ? GENMASK(15, 0) : GENMASK(31, 16);

	if (value) {
		return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, node_id, IOCTL_SET_SD_TAPDELAY, type,
					   value);
	}

	/*
	 * Work around completely misdesigned firmware API on Xilinx ZynqMP.
	 * The IOCTL_SET_SD_TAPDELAY firmware call allows the caller to only
	 * ever set IOU_SLCR SD_ITAPDLY Register SD0_ITAPDLYENA/SD1_ITAPDLYENA
	 * bits, but there is no matching call to clear those bits. If those
	 * bits are not cleared, SDMMC tuning may fail.
	 *
	 * Luckily, there are PM_MMIO_READ/PM_MMIO_WRITE calls which seem to
	 * allow complete unrestricted access to all address space, including
	 * IOU_SLCR SD_ITAPDLY Register and all the other registers, access
	 * to which was supposed to be protected by the current firmware API.
	 *
	 * Use PM_MMIO_READ/PM_MMIO_WRITE to re-implement the missing counter
	 * part of IOCTL_SET_SD_TAPDELAY which clears SDx_ITAPDLYENA bits.
	 */
	return zynqmp_pm_invoke_fn(PM_MMIO_WRITE, NULL, 2, reg, mask);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_sd_tapdelay);

/**
 * zynqmp_pm_sd_dll_reset() - Reset DLL logic
 *
 * @node_id:	Node ID of the device
 * @type:	Reset type
 *
 * This function resets DLL logic for the SD device.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_sd_dll_reset(u32 node_id, u32 type)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, node_id, IOCTL_SD_DLL_RESET, type);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_sd_dll_reset);

/**
 * zynqmp_pm_ospi_mux_select() - OSPI Mux selection
 *
 * @dev_id:	Device Id of the OSPI device.
 * @select:	OSPI Mux select value.
 *
 * This function select the OSPI Mux.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_ospi_mux_select(u32 dev_id, u32 select)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, dev_id, IOCTL_OSPI_MUX_SELECT, select);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_ospi_mux_select);

/**
 * zynqmp_pm_write_ggs() - PM API for writing global general storage (ggs)
 * @index:	GGS register index
 * @value:	Register value to be written
 *
 * This function writes value to GGS register.
 *
 * Return:      Returns status, either success or error+reason
 */
int zynqmp_pm_write_ggs(u32 index, u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_WRITE_GGS, index, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_write_ggs);

/**
 * zynqmp_pm_read_ggs() - PM API for reading global general storage (ggs)
 * @index:	GGS register index
 * @value:	Register value to be written
 *
 * This function returns GGS register value.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_read_ggs(u32 index, u32 *value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, value, 3, 0, IOCTL_READ_GGS, index);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_read_ggs);

/**
 * zynqmp_pm_write_pggs() - PM API for writing persistent global general
 *			     storage (pggs)
 * @index:	PGGS register index
 * @value:	Register value to be written
 *
 * This function writes value to PGGS register.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_write_pggs(u32 index, u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_WRITE_PGGS, index, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_write_pggs);

/**
 * zynqmp_pm_read_pggs() - PM API for reading persistent global general
 *			     storage (pggs)
 * @index:	PGGS register index
 * @value:	Register value to be written
 *
 * This function returns PGGS register value.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_read_pggs(u32 index, u32 *value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, value, 3, 0, IOCTL_READ_PGGS, index);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_read_pggs);

int zynqmp_pm_set_tapdelay_bypass(u32 index, u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_SET_TAPDELAY_BYPASS, index, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_tapdelay_bypass);

/**
 * zynqmp_pm_set_boot_health_status() - PM API for setting healthy boot status
 * @value:	Status value to be written
 *
 * This function sets healthy bit value to indicate boot health status
 * to firmware.
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_set_boot_health_status(u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, 0, IOCTL_SET_BOOT_HEALTH_STATUS, value);
}

/**
 * zynqmp_pm_reset_assert - Request setting of reset (1 - assert, 0 - release)
 * @reset:		Reset to be configured
 * @assert_flag:	Flag stating should reset be asserted (1) or
 *			released (0)
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_reset_assert(const enum zynqmp_pm_reset reset,
			   const enum zynqmp_pm_reset_action assert_flag)
{
	return zynqmp_pm_invoke_fn(PM_RESET_ASSERT, NULL, 2, reset, assert_flag);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_reset_assert);

/**
 * zynqmp_pm_reset_get_status - Get status of the reset
 * @reset:      Reset whose status should be returned
 * @status:     Returned status
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_reset_get_status(const enum zynqmp_pm_reset reset, u32 *status)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!status)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_RESET_GET_STATUS, ret_payload, 1, reset);
	*status = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_reset_get_status);

/**
 * zynqmp_pm_fpga_load - Perform the fpga load
 * @address:	Address to write to
 * @size:	pl bitstream size
 * @flags:	Bitstream type
 *	-XILINX_ZYNQMP_PM_FPGA_FULL:  FPGA full reconfiguration
 *	-XILINX_ZYNQMP_PM_FPGA_PARTIAL: FPGA partial reconfiguration
 *
 * This function provides access to pmufw. To transfer
 * the required bitstream into PL.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_fpga_load(const u64 address, const u32 size, const u32 flags)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_FPGA_LOAD, ret_payload, 4, lower_32_bits(address),
				  upper_32_bits(address), size, flags);
	if (ret_payload[0])
		return -ret_payload[0];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_fpga_load);

/**
 * zynqmp_pm_fpga_get_status - Read value from PCAP status register
 * @value: Value to read
 *
 * This function provides access to the pmufw to get the PCAP
 * status
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_fpga_get_status(u32 *value)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!value)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_FPGA_GET_STATUS, ret_payload, 0);
	*value = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_fpga_get_status);

/**
 * zynqmp_pm_fpga_get_config_status - Get the FPGA configuration status.
 * @value: Buffer to store FPGA configuration status.
 *
 * This function provides access to the pmufw to get the FPGA configuration
 * status
 *
 * Return: 0 on success, a negative value on error
 */
int zynqmp_pm_fpga_get_config_status(u32 *value)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!value)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_FPGA_READ, ret_payload, 4,
				  XILINX_ZYNQMP_PM_FPGA_CONFIG_STAT_OFFSET, 0, 0,
				  XILINX_ZYNQMP_PM_FPGA_READ_CONFIG_REG);

	*value = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_fpga_get_config_status);

/**
 * zynqmp_pm_pinctrl_request - Request Pin from firmware
 * @pin: Pin number to request
 *
 * This function requests pin from firmware.
 *
 * Return: Returns status, either success or error+reason.
 */
int zynqmp_pm_pinctrl_request(const u32 pin)
{
	return zynqmp_pm_invoke_fn(PM_PINCTRL_REQUEST, NULL, 1, pin);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_pinctrl_request);

/**
 * zynqmp_pm_pinctrl_release - Inform firmware that Pin control is released
 * @pin: Pin number to release
 *
 * This function release pin from firmware.
 *
 * Return: Returns status, either success or error+reason.
 */
int zynqmp_pm_pinctrl_release(const u32 pin)
{
	return zynqmp_pm_invoke_fn(PM_PINCTRL_RELEASE, NULL, 1, pin);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_pinctrl_release);

/**
 * zynqmp_pm_pinctrl_set_function - Set requested function for the pin
 * @pin: Pin number
 * @id: Function ID to set
 *
 * This function sets requested function for the given pin.
 *
 * Return: Returns status, either success or error+reason.
 */
int zynqmp_pm_pinctrl_set_function(const u32 pin, const u32 id)
{
	return zynqmp_pm_invoke_fn(PM_PINCTRL_SET_FUNCTION, NULL, 2, pin, id);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_pinctrl_set_function);

/**
 * zynqmp_pm_pinctrl_get_config - Get configuration parameter for the pin
 * @pin: Pin number
 * @param: Parameter to get
 * @value: Buffer to store parameter value
 *
 * This function gets requested configuration parameter for the given pin.
 *
 * Return: Returns status, either success or error+reason.
 */
int zynqmp_pm_pinctrl_get_config(const u32 pin, const u32 param,
				 u32 *value)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!value)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_PINCTRL_CONFIG_PARAM_GET, ret_payload, 2, pin, param);
	*value = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_pinctrl_get_config);

/**
 * zynqmp_pm_pinctrl_set_config - Set configuration parameter for the pin
 * @pin: Pin number
 * @param: Parameter to set
 * @value: Parameter value to set
 *
 * This function sets requested configuration parameter for the given pin.
 *
 * Return: Returns status, either success or error+reason.
 */
int zynqmp_pm_pinctrl_set_config(const u32 pin, const u32 param,
				 u32 value)
{
	int ret;

	if (pm_family_code == ZYNQMP_FAMILY_CODE &&
	    param == PM_PINCTRL_CONFIG_TRI_STATE) {
		ret = zynqmp_pm_feature(PM_PINCTRL_CONFIG_PARAM_SET);
		if (ret < PM_PINCTRL_PARAM_SET_VERSION)
			return -EOPNOTSUPP;
	}

	return zynqmp_pm_invoke_fn(PM_PINCTRL_CONFIG_PARAM_SET, NULL, 3, pin, param, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_pinctrl_set_config);

/**
 * zynqmp_pm_bootmode_read() - PM Config API for read bootpin status
 * @ps_mode: Returned output value of ps_mode
 *
 * This API function is to be used for notify the power management controller
 * to read bootpin status.
 *
 * Return: status, either success or error+reason
 */
unsigned int zynqmp_pm_bootmode_read(u32 *ps_mode)
{
	unsigned int ret;
	u32 ret_payload[PAYLOAD_ARG_CNT];

	ret = zynqmp_pm_invoke_fn(PM_MMIO_READ, ret_payload, 1, CRL_APB_BOOT_PIN_CTRL);

	*ps_mode = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_bootmode_read);

/**
 * zynqmp_pm_bootmode_write() - PM Config API for Configure bootpin
 * @ps_mode: Value to be written to the bootpin ctrl register
 *
 * This API function is to be used for notify the power management controller
 * to configure bootpin.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_bootmode_write(u32 ps_mode)
{
	return zynqmp_pm_invoke_fn(PM_MMIO_WRITE, NULL, 3, CRL_APB_BOOT_PIN_CTRL,
				   CRL_APB_BOOTPIN_CTRL_MASK, ps_mode);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_bootmode_write);

/**
 * zynqmp_pm_init_finalize() - PM call to inform firmware that the caller
 *			       master has initialized its own power management
 *
 * Return: Returns status, either success or error+reason
 *
 * This API function is to be used for notify the power management controller
 * about the completed power management initialization.
 */
int zynqmp_pm_init_finalize(void)
{
	return zynqmp_pm_invoke_fn(PM_PM_INIT_FINALIZE, NULL, 0);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_init_finalize);

/**
 * zynqmp_pm_set_suspend_mode()	- Set system suspend mode
 * @mode:	Mode to set for system suspend
 *
 * This API function is used to set mode of system suspend.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_set_suspend_mode(u32 mode)
{
	return zynqmp_pm_invoke_fn(PM_SET_SUSPEND_MODE, NULL, 1, mode);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_suspend_mode);

/**
 * zynqmp_pm_request_node() - Request a node with specific capabilities
 * @node:		Node ID of the slave
 * @capabilities:	Requested capabilities of the slave
 * @qos:		Quality of service (not supported)
 * @ack:		Flag to specify whether acknowledge is requested
 *
 * This function is used by master to request particular node from firmware.
 * Every master must request node before using it.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_request_node(const u32 node, const u32 capabilities,
			   const u32 qos, const enum zynqmp_pm_request_ack ack)
{
	return zynqmp_pm_invoke_fn(PM_REQUEST_NODE, NULL, 4, node, capabilities, qos, ack);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_request_node);

/**
 * zynqmp_pm_release_node() - Release a node
 * @node:	Node ID of the slave
 *
 * This function is used by master to inform firmware that master
 * has released node. Once released, master must not use that node
 * without re-request.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_release_node(const u32 node)
{
	return zynqmp_pm_invoke_fn(PM_RELEASE_NODE, NULL, 1, node);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_release_node);

/**
 * zynqmp_pm_get_rpu_mode() - Get RPU mode
 * @node_id:	Node ID of the device
 * @rpu_mode:	return by reference value
 *		either split or lockstep
 *
 * Return:	return 0 on success or error+reason.
 *		if success, then  rpu_mode will be set
 *		to current rpu mode.
 */
int zynqmp_pm_get_rpu_mode(u32 node_id, enum rpu_oper_mode *rpu_mode)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	ret = zynqmp_pm_invoke_fn(PM_IOCTL, ret_payload, 2, node_id, IOCTL_GET_RPU_OPER_MODE);

	/* only set rpu_mode if no error */
	if (ret == XST_PM_SUCCESS)
		*rpu_mode = ret_payload[0];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_get_rpu_mode);

/**
 * zynqmp_pm_set_rpu_mode() - Set RPU mode
 * @node_id:	Node ID of the device
 * @rpu_mode:	Argument 1 to requested IOCTL call. either split or lockstep
 *
 *		This function is used to set RPU mode to split or
 *		lockstep
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_set_rpu_mode(u32 node_id, enum rpu_oper_mode rpu_mode)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, node_id, IOCTL_SET_RPU_OPER_MODE,
				   (u32)rpu_mode);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_rpu_mode);

/**
 * zynqmp_pm_set_tcm_config - configure TCM
 * @node_id:	Firmware specific TCM subsystem ID
 * @tcm_mode:	Argument 1 to requested IOCTL call
 *              either PM_RPU_TCM_COMB or PM_RPU_TCM_SPLIT
 *
 * This function is used to set RPU mode to split or combined
 *
 * Return: status: 0 for success, else failure
 */
int zynqmp_pm_set_tcm_config(u32 node_id, enum rpu_tcm_comb tcm_mode)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 3, node_id, IOCTL_TCM_COMB_CONFIG,
				   (u32)tcm_mode);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_tcm_config);

/**
 * zynqmp_pm_force_pwrdwn - PM call to request for another PU or subsystem to
 *             be powered down forcefully
 * @node:  Node ID of the targeted PU or subsystem
 * @ack:   Flag to specify whether acknowledge is requested
 *
 * Return: status, either success or error+reason
 */
int zynqmp_pm_force_pwrdwn(const u32 node,
			   const enum zynqmp_pm_request_ack ack)
{
	return zynqmp_pm_invoke_fn(PM_FORCE_POWERDOWN, NULL, 2, node, ack);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_force_pwrdwn);

/**
 * zynqmp_pm_request_wake - PM call to wake up selected master or subsystem
 * @node:  Node ID of the master or subsystem
 * @set_addr:  Specifies whether the address argument is relevant
 * @address:   Address from which to resume when woken up
 * @ack:   Flag to specify whether acknowledge requested
 *
 * Return: status, either success or error+reason
 */
int zynqmp_pm_request_wake(const u32 node,
			   const bool set_addr,
			   const u64 address,
			   const enum zynqmp_pm_request_ack ack)
{
	/* set_addr flag is encoded into 1st bit of address */
	return zynqmp_pm_invoke_fn(PM_REQUEST_WAKEUP, NULL, 4, node, address | set_addr,
				   address >> 32, ack);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_request_wake);

/**
 * zynqmp_pm_set_requirement() - PM call to set requirement for PM slaves
 * @node:		Node ID of the slave
 * @capabilities:	Requested capabilities of the slave
 * @qos:		Quality of service (not supported)
 * @ack:		Flag to specify whether acknowledge is requested
 *
 * This API function is to be used for slaves a PU already has requested
 * to change its capabilities.
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_set_requirement(const u32 node, const u32 capabilities,
			      const u32 qos,
			      const enum zynqmp_pm_request_ack ack)
{
	return zynqmp_pm_invoke_fn(PM_SET_REQUIREMENT, NULL, 4, node, capabilities, qos, ack);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_requirement);

/**
 * zynqmp_pm_load_pdi - Load and process PDI
 * @src:	Source device where PDI is located
 * @address:	PDI src address
 *
 * This function provides support to load PDI from linux
 *
 * Return: Returns status, either success or error+reason
 */
int zynqmp_pm_load_pdi(const u32 src, const u64 address)
{
	return zynqmp_pm_invoke_fn(PM_LOAD_PDI, NULL, 3, src, lower_32_bits(address),
				   upper_32_bits(address));
}
EXPORT_SYMBOL_GPL(zynqmp_pm_load_pdi);

/**
 * zynqmp_pm_aes_engine - Access AES hardware to encrypt/decrypt the data using
 * AES-GCM core.
 * @address:	Address of the AesParams structure.
 * @out:	Returned output value
 *
 * Return:	Returns status, either success or error code.
 */
int zynqmp_pm_aes_engine(const u64 address, u32 *out)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!out)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_SECURE_AES, ret_payload, 2, upper_32_bits(address),
				  lower_32_bits(address));
	*out = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_aes_engine);

/**
 * zynqmp_pm_efuse_access - Provides access to efuse memory.
 * @address:	Address of the efuse params structure
 * @out:		Returned output value
 *
 * Return:	Returns status, either success or error code.
 */
int zynqmp_pm_efuse_access(const u64 address, u32 *out)
{
	u32 ret_payload[PAYLOAD_ARG_CNT];
	int ret;

	if (!out)
		return -EINVAL;

	ret = zynqmp_pm_invoke_fn(PM_EFUSE_ACCESS, ret_payload, 2,
				  upper_32_bits(address),
				  lower_32_bits(address));
	*out = ret_payload[1];

	return ret;
}
EXPORT_SYMBOL_GPL(zynqmp_pm_efuse_access);

/**
 * zynqmp_pm_sha_hash - Access the SHA engine to calculate the hash
 * @address:	Address of the data/ Address of output buffer where
 *		hash should be stored.
 * @size:	Size of the data.
 * @flags:
 *	BIT(0) - for initializing csudma driver and SHA3(Here address
 *		 and size inputs can be NULL).
 *	BIT(1) - to call Sha3_Update API which can be called multiple
 *		 times when data is not contiguous.
 *	BIT(2) - to get final hash of the whole updated data.
 *		 Hash will be overwritten at provided address with
 *		 48 bytes.
 *
 * Return:	Returns status, either success or error code.
 */
int zynqmp_pm_sha_hash(const u64 address, const u32 size, const u32 flags)
{
	u32 lower_addr = lower_32_bits(address);
	u32 upper_addr = upper_32_bits(address);

	return zynqmp_pm_invoke_fn(PM_SECURE_SHA, NULL, 4, upper_addr, lower_addr, size, flags);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_sha_hash);

/**
 * zynqmp_pm_register_notifier() - PM API for register a subsystem
 *                                to be notified about specific
 *                                event/error.
 * @node:	Node ID to which the event is related.
 * @event:	Event Mask of Error events for which wants to get notified.
 * @wake:	Wake subsystem upon capturing the event if value 1
 * @enable:	Enable the registration for value 1, disable for value 0
 *
 * This function is used to register/un-register for particular node-event
 * combination in firmware.
 *
 * Return: Returns status, either success or error+reason
 */

int zynqmp_pm_register_notifier(const u32 node, const u32 event,
				const u32 wake, const u32 enable)
{
	return zynqmp_pm_invoke_fn(PM_REGISTER_NOTIFIER, NULL, 4, node, event, wake, enable);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_register_notifier);

/**
 * zynqmp_pm_system_shutdown - PM call to request a system shutdown or restart
 * @type:	Shutdown or restart? 0 for shutdown, 1 for restart
 * @subtype:	Specifies which system should be restarted or shut down
 *
 * Return:	Returns status, either success or error+reason
 */
int zynqmp_pm_system_shutdown(const u32 type, const u32 subtype)
{
	return zynqmp_pm_invoke_fn(PM_SYSTEM_SHUTDOWN, NULL, 2, type, subtype);
}

/**
 * zynqmp_pm_set_feature_config - PM call to request IOCTL for feature config
 * @id:         The config ID of the feature to be configured
 * @value:      The config value of the feature to be configured
 *
 * Return:      Returns 0 on success or error value on failure.
 */
int zynqmp_pm_set_feature_config(enum pm_feature_config_id id, u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, 0, IOCTL_SET_FEATURE_CONFIG, id, value);
}

/**
 * zynqmp_pm_get_feature_config - PM call to get value of configured feature
 * @id:         The config id of the feature to be queried
 * @payload:    Returned value array
 *
 * Return:      Returns 0 on success or error value on failure.
 */
int zynqmp_pm_get_feature_config(enum pm_feature_config_id id,
				 u32 *payload)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, payload, 3, 0, IOCTL_GET_FEATURE_CONFIG, id);
}

/**
 * zynqmp_pm_set_sd_config - PM call to set value of SD config registers
 * @node:	SD node ID
 * @config:	The config type of SD registers
 * @value:	Value to be set
 *
 * Return:	Returns 0 on success or error value on failure.
 */
int zynqmp_pm_set_sd_config(u32 node, enum pm_sd_config_type config, u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, node, IOCTL_SET_SD_CONFIG, config, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_sd_config);

/**
 * zynqmp_pm_set_gem_config - PM call to set value of GEM config registers
 * @node:	GEM node ID
 * @config:	The config type of GEM registers
 * @value:	Value to be set
 *
 * Return:	Returns 0 on success or error value on failure.
 */
int zynqmp_pm_set_gem_config(u32 node, enum pm_gem_config_type config,
			     u32 value)
{
	return zynqmp_pm_invoke_fn(PM_IOCTL, NULL, 4, node, IOCTL_SET_GEM_CONFIG, config, value);
}
EXPORT_SYMBOL_GPL(zynqmp_pm_set_gem_config);

/**
 * struct zynqmp_pm_shutdown_scope - Struct for shutdown scope
 * @subtype:	Shutdown subtype
 * @name:	Matching string for scope argument
 *
 * This struct encapsulates mapping between shutdown scope ID and string.
 */
struct zynqmp_pm_shutdown_scope {
	const enum zynqmp_pm_shutdown_subtype subtype;
	const char *name;
};

static struct zynqmp_pm_shutdown_scope shutdown_scopes[] = {
	[ZYNQMP_PM_SHUTDOWN_SUBTYPE_SUBSYSTEM] = {
		.subtype = ZYNQMP_PM_SHUTDOWN_SUBTYPE_SUBSYSTEM,
		.name = "subsystem",
	},
	[ZYNQMP_PM_SHUTDOWN_SUBTYPE_PS_ONLY] = {
		.subtype = ZYNQMP_PM_SHUTDOWN_SUBTYPE_PS_ONLY,
		.name = "ps_only",
	},
	[ZYNQMP_PM_SHUTDOWN_SUBTYPE_SYSTEM] = {
		.subtype = ZYNQMP_PM_SHUTDOWN_SUBTYPE_SYSTEM,
		.name = "system",
	},
};

static struct zynqmp_pm_shutdown_scope *selected_scope =
		&shutdown_scopes[ZYNQMP_PM_SHUTDOWN_SUBTYPE_SYSTEM];

/**
 * zynqmp_pm_is_shutdown_scope_valid - Check if shutdown scope string is valid
 * @scope_string:	Shutdown scope string
 *
 * Return:		Return pointer to matching shutdown scope struct from
 *			array of available options in system if string is valid,
 *			otherwise returns NULL.
 */
static struct zynqmp_pm_shutdown_scope*
		zynqmp_pm_is_shutdown_scope_valid(const char *scope_string)
{
	int count;

	for (count = 0; count < ARRAY_SIZE(shutdown_scopes); count++)
		if (sysfs_streq(scope_string, shutdown_scopes[count].name))
			return &shutdown_scopes[count];

	return NULL;
}

static ssize_t shutdown_scope_show(struct device *device,
				   struct device_attribute *attr,
				   char *buf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(shutdown_scopes); i++) {
		if (&shutdown_scopes[i] == selected_scope) {
			strcat(buf, "[");
			strcat(buf, shutdown_scopes[i].name);
			strcat(buf, "]");
		} else {
			strcat(buf, shutdown_scopes[i].name);
		}
		strcat(buf, " ");
	}
	strcat(buf, "\n");

	return strlen(buf);
}

static ssize_t shutdown_scope_store(struct device *device,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int ret;
	struct zynqmp_pm_shutdown_scope *scope;

	scope = zynqmp_pm_is_shutdown_scope_valid(buf);
	if (!scope)
		return -EINVAL;

	ret = zynqmp_pm_system_shutdown(ZYNQMP_PM_SHUTDOWN_TYPE_SETSCOPE_ONLY,
					scope->subtype);
	if (ret) {
		pr_err("unable to set shutdown scope %s\n", buf);
		return ret;
	}

	selected_scope = scope;

	return count;
}

static DEVICE_ATTR_RW(shutdown_scope);

static ssize_t health_status_store(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	unsigned int value;

	ret = kstrtouint(buf, 10, &value);
	if (ret)
		return ret;

	ret = zynqmp_pm_set_boot_health_status(value);
	if (ret) {
		dev_err(device, "unable to set healthy bit value to %u\n",
			value);
		return ret;
	}

	return count;
}

static DEVICE_ATTR_WO(health_status);

static ssize_t ggs_show(struct device *device,
			struct device_attribute *attr,
			char *buf,
			u32 reg)
{
	int ret;
	u32 ret_payload[PAYLOAD_ARG_CNT];

	ret = zynqmp_pm_read_ggs(reg, ret_payload);
	if (ret)
		return ret;

	return sprintf(buf, "0x%x\n", ret_payload[1]);
}

static ssize_t ggs_store(struct device *device,
			 struct device_attribute *attr,
			 const char *buf, size_t count,
			 u32 reg)
{
	long value;
	int ret;

	if (reg >= GSS_NUM_REGS)
		return -EINVAL;

	ret = kstrtol(buf, 16, &value);
	if (ret) {
		count = -EFAULT;
		goto err;
	}

	ret = zynqmp_pm_write_ggs(reg, value);
	if (ret)
		count = -EFAULT;
err:
	return count;
}

/* GGS register show functions */
#define GGS0_SHOW(N)						\
	ssize_t ggs##N##_show(struct device *device,		\
			      struct device_attribute *attr,	\
			      char *buf)			\
	{							\
		return ggs_show(device, attr, buf, N);		\
	}

static GGS0_SHOW(0);
static GGS0_SHOW(1);
static GGS0_SHOW(2);
static GGS0_SHOW(3);

/* GGS register store function */
#define GGS0_STORE(N)						\
	ssize_t ggs##N##_store(struct device *device,		\
			       struct device_attribute *attr,	\
			       const char *buf,			\
			       size_t count)			\
	{							\
		return ggs_store(device, attr, buf, count, N);	\
	}

static GGS0_STORE(0);
static GGS0_STORE(1);
static GGS0_STORE(2);
static GGS0_STORE(3);

static ssize_t pggs_show(struct device *device,
			 struct device_attribute *attr,
			 char *buf,
			 u32 reg)
{
	int ret;
	u32 ret_payload[PAYLOAD_ARG_CNT];

	ret = zynqmp_pm_read_pggs(reg, ret_payload);
	if (ret)
		return ret;

	return sprintf(buf, "0x%x\n", ret_payload[1]);
}

static ssize_t pggs_store(struct device *device,
			  struct device_attribute *attr,
			  const char *buf, size_t count,
			  u32 reg)
{
	long value;
	int ret;

	if (reg >= GSS_NUM_REGS)
		return -EINVAL;

	ret = kstrtol(buf, 16, &value);
	if (ret) {
		count = -EFAULT;
		goto err;
	}

	ret = zynqmp_pm_write_pggs(reg, value);
	if (ret)
		count = -EFAULT;

err:
	return count;
}

#define PGGS0_SHOW(N)						\
	ssize_t pggs##N##_show(struct device *device,		\
			       struct device_attribute *attr,	\
			       char *buf)			\
	{							\
		return pggs_show(device, attr, buf, N);		\
	}

#define PGGS0_STORE(N)						\
	ssize_t pggs##N##_store(struct device *device,		\
				struct device_attribute *attr,	\
				const char *buf,		\
				size_t count)			\
	{							\
		return pggs_store(device, attr, buf, count, N);	\
	}

/* PGGS register show functions */
static PGGS0_SHOW(0);
static PGGS0_SHOW(1);
static PGGS0_SHOW(2);
static PGGS0_SHOW(3);

/* PGGS register store functions */
static PGGS0_STORE(0);
static PGGS0_STORE(1);
static PGGS0_STORE(2);
static PGGS0_STORE(3);

/* GGS register attributes */
static DEVICE_ATTR_RW(ggs0);
static DEVICE_ATTR_RW(ggs1);
static DEVICE_ATTR_RW(ggs2);
static DEVICE_ATTR_RW(ggs3);

/* PGGS register attributes */
static DEVICE_ATTR_RW(pggs0);
static DEVICE_ATTR_RW(pggs1);
static DEVICE_ATTR_RW(pggs2);
static DEVICE_ATTR_RW(pggs3);

static ssize_t feature_config_id_show(struct device *device,
				      struct device_attribute *attr,
				      char *buf)
{
	struct zynqmp_devinfo *devinfo = dev_get_drvdata(device);

	return sysfs_emit(buf, "%d\n", devinfo->feature_conf_id);
}

static ssize_t feature_config_id_store(struct device *device,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	u32 config_id;
	int ret;
	struct zynqmp_devinfo *devinfo = dev_get_drvdata(device);

	if (!buf)
		return -EINVAL;

	ret = kstrtou32(buf, 10, &config_id);
	if (ret)
		return ret;

	devinfo->feature_conf_id = config_id;

	return count;
}

static DEVICE_ATTR_RW(feature_config_id);

static ssize_t feature_config_value_show(struct device *device,
					 struct device_attribute *attr,
					 char *buf)
{
	int ret;
	u32 ret_payload[PAYLOAD_ARG_CNT];
	struct zynqmp_devinfo *devinfo = dev_get_drvdata(device);

	ret = zynqmp_pm_get_feature_config(devinfo->feature_conf_id,
					   ret_payload);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", ret_payload[1]);
}

static ssize_t feature_config_value_store(struct device *device,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	u32 value;
	int ret;
	struct zynqmp_devinfo *devinfo = dev_get_drvdata(device);

	if (!buf)
		return -EINVAL;

	ret = kstrtou32(buf, 10, &value);
	if (ret)
		return ret;

	ret = zynqmp_pm_set_feature_config(devinfo->feature_conf_id,
					   value);
	if (ret)
		return ret;

	return count;
}

static DEVICE_ATTR_RW(feature_config_value);

static struct attribute *zynqmp_firmware_attrs[] = {
	&dev_attr_ggs0.attr,
	&dev_attr_ggs1.attr,
	&dev_attr_ggs2.attr,
	&dev_attr_ggs3.attr,
	&dev_attr_pggs0.attr,
	&dev_attr_pggs1.attr,
	&dev_attr_pggs2.attr,
	&dev_attr_pggs3.attr,
	&dev_attr_shutdown_scope.attr,
	&dev_attr_health_status.attr,
	&dev_attr_feature_config_id.attr,
	&dev_attr_feature_config_value.attr,
	NULL,
};

ATTRIBUTE_GROUPS(zynqmp_firmware);

static int zynqmp_firmware_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct zynqmp_devinfo *devinfo;
	int ret;

	ret = get_set_conduit_method(dev->of_node);
	if (ret)
		return ret;

	ret = do_feature_check_call(PM_FEATURE_CHECK);
	if (ret >= 0 && ((ret & FIRMWARE_VERSION_MASK) >= PM_API_VERSION_1))
		feature_check_enabled = true;

	devinfo = devm_kzalloc(dev, sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo)
		return -ENOMEM;

	devinfo->dev = dev;

	platform_set_drvdata(pdev, devinfo);

	/* Check PM API version number */
	ret = zynqmp_pm_get_api_version(&pm_api_version);
	if (ret)
		return ret;

	if (pm_api_version < ZYNQMP_PM_VERSION) {
		panic("%s Platform Management API version error. Expected: v%d.%d - Found: v%d.%d\n",
		      __func__,
		      ZYNQMP_PM_VERSION_MAJOR, ZYNQMP_PM_VERSION_MINOR,
		      pm_api_version >> 16, pm_api_version & 0xFFFF);
	}

	pr_info("%s Platform Management API v%d.%d\n", __func__,
		pm_api_version >> 16, pm_api_version & 0xFFFF);

	/* Get the Family code and sub family code of platform */
	ret = zynqmp_pm_get_family_info(&pm_family_code, &pm_sub_family_code);
	if (ret < 0)
		return ret;

	/* Check trustzone version number */
	ret = zynqmp_pm_get_trustzone_version(&pm_tz_version);
	if (ret)
		panic("Legacy trustzone found without version support\n");

	if (pm_tz_version < ZYNQMP_TZ_VERSION)
		panic("%s Trustzone version error. Expected: v%d.%d - Found: v%d.%d\n",
		      __func__,
		      ZYNQMP_TZ_VERSION_MAJOR, ZYNQMP_TZ_VERSION_MINOR,
		      pm_tz_version >> 16, pm_tz_version & 0xFFFF);

	pr_info("%s Trustzone version v%d.%d\n", __func__,
		pm_tz_version >> 16, pm_tz_version & 0xFFFF);

	ret = mfd_add_devices(&pdev->dev, PLATFORM_DEVID_NONE, firmware_devs,
			      ARRAY_SIZE(firmware_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to add MFD devices %d\n", ret);
		return ret;
	}

	zynqmp_pm_api_debugfs_init();

	if (pm_family_code == VERSAL_FAMILY_CODE) {
		em_dev = platform_device_register_data(&pdev->dev, "xlnx_event_manager",
						       -1, NULL, 0);
		if (IS_ERR(em_dev))
			dev_err_probe(&pdev->dev, PTR_ERR(em_dev), "EM register fail with error\n");
	}

	return of_platform_populate(dev->of_node, NULL, NULL, dev);
}

static void zynqmp_firmware_remove(struct platform_device *pdev)
{
	struct pm_api_feature_data *feature_data;
	struct hlist_node *tmp;
	int i;

	mfd_remove_devices(&pdev->dev);
	zynqmp_pm_api_debugfs_exit();

	hash_for_each_safe(pm_api_features_map, i, tmp, feature_data, hentry) {
		hash_del(&feature_data->hentry);
		kfree(feature_data);
	}

	platform_device_unregister(em_dev);
}

static const struct of_device_id zynqmp_firmware_of_match[] = {
	{.compatible = "xlnx,zynqmp-firmware"},
	{.compatible = "xlnx,versal-firmware"},
	{},
};
MODULE_DEVICE_TABLE(of, zynqmp_firmware_of_match);

static struct platform_driver zynqmp_firmware_driver = {
	.driver = {
		.name = "zynqmp_firmware",
		.of_match_table = zynqmp_firmware_of_match,
		.dev_groups = zynqmp_firmware_groups,
	},
	.probe = zynqmp_firmware_probe,
	.remove_new = zynqmp_firmware_remove,
};
module_platform_driver(zynqmp_firmware_driver);
