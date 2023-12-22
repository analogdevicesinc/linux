// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2011-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

/*
 * Base kernel property query APIs
 */

#include <mali_kbase.h>
#include <hw_access/mali_kbase_hw_access_regmap.h>

#include <mali_kbase_gpuprops_private_types.h>
#include <mali_kbase_gpuprops.h>
#include <mali_kbase_hwaccess_gpuprops.h>

#include <mali_kbase_config_defaults.h>
#include <uapi/gpu/arm/midgard/mali_kbase_ioctl.h>
#include <linux/clk.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <linux/of_platform.h>
#include <linux/moduleparam.h>

#define PRIV_DATA_REGDUMP(kbdev) \
	(((struct kbasep_gpuprops_priv_data *)((kbdev)->gpu_props.priv_data))->regdump)

/* Default values when registers are not supported by the implemented hardware */
#define THREAD_MT_DEFAULT 256
#define THREAD_MWS_DEFAULT 256
#define THREAD_MBS_DEFAULT 256
#define THREAD_MR_DEFAULT 1024
#define THREAD_MTQ_DEFAULT 4
#define THREAD_MTGS_DEFAULT 10

int kbase_gpuprops_get_curr_config_props(struct kbase_device *kbdev,
					 struct curr_config_props *const curr_config)
{
	struct kbase_current_config_regdump curr_config_regdump;
	int err;

	if (WARN_ON(!kbdev) || WARN_ON(!curr_config))
		return -EINVAL;

	/* If update not needed just return. */
	if (!curr_config->update_needed)
		return 0;

	/* Dump relevant registers */
	err = kbase_backend_gpuprops_get_curr_config(kbdev, &curr_config_regdump);
	if (err)
		return err;

	{
		curr_config->l2_slices = KBASE_UBFX64(curr_config_regdump.mem_features, 8U, 4) + 1;
	}

	curr_config->l2_present = curr_config_regdump.l2_present;

	curr_config->shader_present = curr_config_regdump.shader_present;

	curr_config->num_cores = hweight64(curr_config->shader_present);

	curr_config->update_needed = false;

	return 0;
}

int kbase_gpuprops_req_curr_config_update(struct kbase_device *kbdev)
{
	if (WARN_ON(!kbdev))
		return -EINVAL;

	kbdev->gpu_props.curr_config.update_needed = true;
	return 0;
}

/**
 * kbase_gpuprops_update_max_config_props - Updates the max config properties in
 * the kbase_gpu_props.
 * @kbdev:      The &struct kbase_device structure for the device
 *
 * Updates the &struct kbase_gpu_props structure with the max config properties.
 */
static void kbase_gpuprops_update_max_config_props(struct kbase_device *kbdev)
{
	struct kbase_gpu_props *gpu_props;
	int l2_n = 0;

	if (WARN_ON(!kbdev))
		return;

	/* return if the max_config is not set during arbif initialization */
	if (kbdev->gpu_props.max_config.core_mask == 0)
		return;

	gpu_props = &kbdev->gpu_props;

	/*
	 * Set the base_props with the maximum config values to ensure that the
	 * user space will always be based on the maximum resources available.
	 */
	gpu_props->num_l2_slices = gpu_props->max_config.l2_slices;
	gpu_props->shader_present = gpu_props->max_config.core_mask;
	/*
	 * Update l2_present in the raw data to be consistent with the
	 * max_config.l2_slices number.
	 */
	gpu_props->l2_present = 0;
	for (l2_n = 0; l2_n < gpu_props->num_l2_slices; l2_n++) {
		gpu_props->l2_present <<= 1;
		gpu_props->l2_present |= 0x1;
	}
	/*
	 * Update the coherency_info data using just one core group. For
	 * architectures where the max_config is provided by the arbiter it is
	 * not necessary to split the shader core groups in different coherent
	 * groups.
	 */
	gpu_props->num_core_groups = 1;
	gpu_props->coherency_info.group.core_mask = gpu_props->max_config.core_mask;
	gpu_props->coherency_info.group.num_cores = hweight32(gpu_props->max_config.core_mask);
}

void kbase_gpuprops_set_max_config(struct kbase_device *kbdev,
				   const struct max_config_props *max_config)
{
	if (WARN_ON(!kbdev) || WARN_ON(!max_config))
		return;

	kbdev->gpu_props.max_config.l2_slices = max_config->l2_slices;
	kbdev->gpu_props.max_config.core_mask = max_config->core_mask;
}

void kbase_gpuprops_update_composite_ids(struct kbase_gpu_id_props *props)
{
	props->product_id = GPU_ID_PRODUCT_ID_MAKE(props->arch_major, props->arch_minor,
						   props->arch_rev, props->product_major);
	props->product_model = GPU_ID_MODEL_MAKE(props->arch_major, props->product_major);
	props->version_id = GPU_ID_VERSION_MAKE(props->version_major, props->version_minor,
						props->version_status);
	props->arch_id = GPU_ID_ARCH_MAKE(props->arch_major, props->arch_minor, props->arch_rev);
}

void kbase_gpuprops_parse_gpu_id(struct kbase_gpu_id_props *props, u64 gpu_id)
{
	props->arch_major = GPU_ID2_ARCH_MAJOR_GET(gpu_id);
	props->version_status = gpu_id & GPU_ID2_VERSION_STATUS;
	props->version_minor = GPU_ID2_VERSION_MINOR_GET(gpu_id);
	props->version_major = GPU_ID2_VERSION_MAJOR_GET(gpu_id);
	props->product_major = GPU_ID2_PRODUCT_MAJOR_GET(gpu_id);
	props->arch_rev = GPU_ID2_ARCH_REV_GET(gpu_id);
	props->arch_minor = GPU_ID2_ARCH_MINOR_GET(gpu_id);

	kbase_gpuprops_update_composite_ids(props);
}
KBASE_EXPORT_TEST_API(kbase_gpuprops_parse_gpu_id);

static void kbase_gpuprops_parse_gpu_features(struct kbase_gpu_features_props *props,
					      u64 gpu_features)
{
	props->ray_intersection = KBASE_UBFX64(gpu_features, 2U, 1);
	props->cross_stream_sync = KBASE_UBFX64(gpu_features, 3U, 1);
}

static void kbase_gpuprops_parse_js_features(struct kbase_js_features_props *props, u32 js_features)
{
	props->null = KBASE_UBFX32(js_features, 1U, 1);
	props->write_value = KBASE_UBFX32(js_features, 2U, 1);
	props->cache_flush = KBASE_UBFX32(js_features, 3U, 1);
	props->compute_shader = KBASE_UBFX32(js_features, 4U, 1);
	props->tiler = KBASE_UBFX32(js_features, 7U, 1);
	props->fragment_shader = KBASE_UBFX32(js_features, 9U, 1);
}

/**
 * kbase_gpuprops_get_props - Get the GPU configuration
 * @kbdev: The &struct kbase_device structure for the device
 *
 * Fill the &struct base_gpu_props structure with values from the GPU
 * configuration registers. Only the raw properties are filled in this function.
 *
 * Return: Zero on success, Linux error code on failure
 */
static int kbase_gpuprops_get_props(struct kbase_device *kbdev)
{
	struct kbase_gpu_props *gpu_props;
	struct kbasep_gpuprops_regdump *regdump;

	int i, err;

	if (WARN_ON(kbdev == NULL) || WARN_ON(kbdev->gpu_props.priv_data == NULL))
		return -EINVAL;

	gpu_props = &kbdev->gpu_props;
	regdump = &PRIV_DATA_REGDUMP(kbdev);

	/* Dump relevant registers */
	err = kbase_backend_gpuprops_get(kbdev, regdump);
	if (err)
		return err;

	gpu_props->shader_present = regdump->shader_present;
	gpu_props->tiler_present = regdump->tiler_present;
	gpu_props->stack_present = regdump->stack_present;
	gpu_props->l2_present = regdump->l2_present;

	gpu_props->num_cores = hweight64(regdump->shader_present);
	gpu_props->num_core_groups = hweight64(regdump->l2_present);

	{
		gpu_props->num_address_spaces = hweight32(regdump->as_present);
	}

	gpu_props->num_job_slots = hweight32(regdump->js_present);

	gpu_props->log2_program_counter_size = KBASE_GPU_PC_SIZE_LOG2;

	if (regdump->thread_max_threads == 0)
		gpu_props->max_threads = THREAD_MT_DEFAULT;
	else
		gpu_props->max_threads = regdump->thread_max_threads;

#if MALI_USE_CSF
	gpu_props->impl_tech = KBASE_UBFX32(regdump->thread_features, 22U, 2);
#else /* MALI_USE_CSF */
	gpu_props->impl_tech = KBASE_UBFX32(regdump->thread_features, 30U, 2);
#endif /* MALI_USE_CSF */

	/* Features */
	kbase_gpuprops_parse_gpu_features(&gpu_props->gpu_features, regdump->gpu_features);

	gpu_props->coherency_info.coherent_core_group = KBASE_UBFX64(regdump->mem_features, 0U, 1);
	gpu_props->coherency_info.coherent_super_group = KBASE_UBFX64(regdump->mem_features, 1U, 1);
	gpu_props->coherency_info.group.core_mask = gpu_props->shader_present;
	gpu_props->coherency_info.group.num_cores = gpu_props->num_cores;

	gpu_props->mmu.va_bits = KBASE_UBFX64(regdump->mmu_features, 0U, 8);
	gpu_props->mmu.pa_bits = KBASE_UBFX64(regdump->mmu_features, 8U, 8);

	/*
	 * this will get turned into the selected coherency mode.
	 * Additionally, add non-coherent mode, as this is always supported.
	 */

	gpu_props->coherency_mode = regdump->coherency_features |
				    COHERENCY_FEATURE_BIT(COHERENCY_NONE);

	gpu_props->log2_line_size = KBASE_UBFX64(regdump->l2_features, 0U, 8);
	{
		gpu_props->num_l2_slices = KBASE_UBFX64(regdump->mem_features, 8U, 4) + 1;
	}

	for (i = 0; i < GPU_MAX_JOB_SLOTS; i++)
		kbase_gpuprops_parse_js_features(&gpu_props->js_features[i],
						 regdump->js_features[i]);

	if (gpu_props->max_config.core_mask)
		kbase_gpuprops_update_max_config_props(kbdev);

	return 0;
}

int kbase_gpuprops_init(struct kbase_device *kbdev)
{
	struct kbase_gpu_props *gpu_props;
	int err = 0;

	if (WARN_ON(!kbdev))
		return -EINVAL;

	gpu_props = &kbdev->gpu_props;

	/* Allocate private data for gpuprop backend */
	kbdev->gpu_props.priv_data = kzalloc(sizeof(struct kbasep_gpuprops_priv_data), GFP_KERNEL);

	if (!gpu_props->priv_data)
		return -ENOMEM;

	/* Get and populate kbase gpu properties */
	err = kbase_gpuprops_get_props(kbdev);
	if (err) {
		kbase_gpuprops_term(kbdev);
		return err;
	}

	/*
	 * Current configuration is used on HW interactions so that the maximum
	 * config is just used for user space avoiding interactions with parts
	 * of the hardware that might not be allocated to the kbase instance at
	 * that moment.
	 */
	kbase_gpuprops_req_curr_config_update(kbdev);
	kbase_gpuprops_get_curr_config_props(kbdev, &gpu_props->curr_config);
	return 0;
}

void kbase_gpuprops_term(struct kbase_device *kbdev)
{
	kfree(kbdev->gpu_props.priv_data);
	kbdev->gpu_props.priv_data = NULL;
}

/*
 * Module parameters to allow the L2 size and hash configuration to be
 * overridden.
 *
 * These parameters must be set on insmod to take effect, and are not visible
 * in sysfs.
 */
static u8 override_l2_size;
module_param(override_l2_size, byte, 0000);
MODULE_PARM_DESC(override_l2_size, "Override L2 size config for testing");
/* Minimum L2 cache size - LOG2(1KiB) */
#define OVERRIDE_L2_SIZE_MIN_LOG2 (10)

static u8 override_l2_hash;
module_param(override_l2_hash, byte, 0000);
MODULE_PARM_DESC(override_l2_hash, "Override L2 hash config for testing");

static u32 l2_hash_values[GPU_L2_SLICE_HASH_COUNT] = {
	0,
};
static unsigned int num_override_l2_hash_values;
module_param_array(l2_hash_values, uint, &num_override_l2_hash_values, 0000);
MODULE_PARM_DESC(l2_hash_values, "Override L2 hash values config for testing");

/* Definitions for range of supported user defined hash functions for GPUs
 * that support L2_CONFIG and not L2 cache slice hash features. Supported
 * hash function range from 0b1000-0b1111 inclusive. Selection of any other
 * values will lead to undefined behavior.
 */
#define USER_DEFINED_HASH_LO ((u8)0x08)
#define USER_DEFINED_HASH_HI ((u8)0x0F)

enum l2_config_override_result {
	L2_CONFIG_OVERRIDE_FAIL = -1,
	L2_CONFIG_OVERRIDE_NONE,
	L2_CONFIG_OVERRIDE_OK,
};

/**
 * kbase_read_l2_config_from_dt - Read L2 configuration
 * @kbdev: The kbase device for which to get the L2 configuration.
 *
 * Check for L2 configuration overrides in module parameters and device tree.
 * Override values in module parameters take priority over override values in
 * device tree.
 *
 * Return: L2_CONFIG_OVERRIDE_OK if either size or hash, or both was properly
 *         overridden, L2_CONFIG_OVERRIDE_NONE if no overrides are provided.
 *         L2_CONFIG_OVERRIDE_FAIL otherwise.
 */
static enum l2_config_override_result kbase_read_l2_config_from_dt(struct kbase_device *const kbdev)
{
	struct device_node *np = kbdev->dev->of_node;

	if (!np)
		return L2_CONFIG_OVERRIDE_NONE;

	if (override_l2_size)
		kbdev->l2_size_override = override_l2_size;
	else if (of_property_read_u8(np, "l2-size", &kbdev->l2_size_override))
		kbdev->l2_size_override = 0;

	if (kbdev->l2_size_override != 0 && kbdev->l2_size_override < OVERRIDE_L2_SIZE_MIN_LOG2)
		return L2_CONFIG_OVERRIDE_FAIL;

	/* Check overriding value is supported, if not will result in
	 * undefined behavior.
	 */
	if (override_l2_hash >= USER_DEFINED_HASH_LO && override_l2_hash <= USER_DEFINED_HASH_HI)
		kbdev->l2_hash_override = override_l2_hash;
	else if (of_property_read_u8(np, "l2-hash", &kbdev->l2_hash_override))
		kbdev->l2_hash_override = 0;

	kbdev->l2_hash_values_override = false;
	if (num_override_l2_hash_values) {
		unsigned int i;

		kbdev->l2_hash_values_override = true;
		for (i = 0; i < num_override_l2_hash_values; i++)
			kbdev->l2_hash_values[i] = l2_hash_values[i];
	} else if (!of_property_read_u32_array(np, "l2-hash-values", kbdev->l2_hash_values,
					       GPU_L2_SLICE_HASH_COUNT))
		kbdev->l2_hash_values_override = true;

	if (kbase_hw_has_l2_slice_hash_feature(kbdev) && (kbdev->l2_hash_override)) {
		dev_err(kbdev->dev, "l2-hash not supported\n");
		return L2_CONFIG_OVERRIDE_FAIL;
	}

	if (!kbase_hw_has_l2_slice_hash_feature(kbdev) && (kbdev->l2_hash_values_override)) {
		dev_err(kbdev->dev, "l2-hash-values not supported\n");
		return L2_CONFIG_OVERRIDE_FAIL;
	}

	if (kbdev->l2_hash_override && kbdev->l2_hash_values_override) {
		dev_err(kbdev->dev, "both l2-hash & l2-hash-values not supported\n");
		return L2_CONFIG_OVERRIDE_FAIL;
	}

	if (kbdev->l2_size_override || kbdev->l2_hash_override || kbdev->l2_hash_values_override)
		return L2_CONFIG_OVERRIDE_OK;

	return L2_CONFIG_OVERRIDE_NONE;
}

int kbase_gpuprops_update_l2_features(struct kbase_device *kbdev)
{
	int err = 0;

	if (kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_L2_CONFIG)) {
		struct kbasep_gpuprops_regdump *regdump = &PRIV_DATA_REGDUMP(kbdev);

		/* Check for L2 cache size & hash overrides */
		switch (kbase_read_l2_config_from_dt(kbdev)) {
		case L2_CONFIG_OVERRIDE_FAIL:
			err = -EIO;
			goto exit;
		case L2_CONFIG_OVERRIDE_NONE:
			goto exit;
		default:
			break;
		}

		/* pm.active_count is expected to be 1 here, which is set in
		 * kbase_hwaccess_pm_powerup().
		 */
		WARN_ON(kbdev->pm.active_count != 1);
		/* The new settings for L2 cache can only be applied when it is
		 * off, so first do the power down.
		 */
		kbase_pm_context_idle(kbdev);
		kbase_pm_wait_for_desired_state(kbdev);

		/* Need L2 to get powered to reflect to L2_FEATURES */
		kbase_pm_context_active(kbdev);

		/* Wait for the completion of L2 power transition */
		kbase_pm_wait_for_l2_powered(kbdev);

		/* Dump L2_FEATURES register */
		err = kbase_backend_gpuprops_get_l2_features(kbdev, regdump);
		if (err)
			goto exit;

		dev_info(kbdev->dev, "Reflected L2_FEATURES is 0x%llx\n", regdump->l2_features);
		dev_info(kbdev->dev, "Reflected L2_CONFIG is 0x%08x\n", regdump->l2_config);

		if (kbase_hw_has_l2_slice_hash_feature(kbdev)) {
			int idx;
			const bool enable = regdump->l2_config &
					    L2_CONFIG_L2_SLICE_HASH_ENABLE_MASK;

#if !IS_ENABLED(CONFIG_MALI_NO_MALI)
			if (!enable && kbdev->l2_hash_values_override) {
					dev_err(kbdev->dev,
						"Failed to use requested ASN_HASH, fallback to default");
			}
#endif
			for (idx = 0; idx < GPU_L2_SLICE_HASH_COUNT; idx++)
					dev_info(kbdev->dev, "%s ASN_HASH[%d] is [0x%08x]\n",
						 enable ? "Overridden" : "Default", idx,
						 regdump->l2_slice_hash[idx]);
		}
	}

exit:
	return err;
}

static struct {
	u32 type;
	size_t offset;
	int size;
} gpu_property_mapping[] = {
#define PROP(name, member)                                                          \
	{                                                                           \
		KBASE_GPUPROP_##name, offsetof(struct gpu_props_user_data, member), \
			sizeof(((struct gpu_props_user_data *)0)->member)           \
	}
	PROP(PRODUCT_ID, core_props.product_id),
	PROP(VERSION_STATUS, core_props.version_status),
	PROP(MINOR_REVISION, core_props.minor_revision),
	PROP(MAJOR_REVISION, core_props.major_revision),
	PROP(GPU_FREQ_KHZ_MAX, core_props.gpu_freq_khz_max),
	PROP(LOG2_PROGRAM_COUNTER_SIZE, core_props.log2_program_counter_size),
	PROP(TEXTURE_FEATURES_0, core_props.texture_features[0]),
	PROP(TEXTURE_FEATURES_1, core_props.texture_features[1]),
	PROP(TEXTURE_FEATURES_2, core_props.texture_features[2]),
	PROP(TEXTURE_FEATURES_3, core_props.texture_features[3]),
	PROP(GPU_AVAILABLE_MEMORY_SIZE, core_props.gpu_available_memory_size),
	PROP(NUM_EXEC_ENGINES, core_props.num_exec_engines),

	PROP(L2_LOG2_LINE_SIZE, l2_props.log2_line_size),
	PROP(L2_LOG2_CACHE_SIZE, l2_props.log2_cache_size),
	PROP(L2_NUM_L2_SLICES, l2_props.num_l2_slices),

	PROP(TILER_BIN_SIZE_BYTES, tiler_props.bin_size_bytes),
	PROP(TILER_MAX_ACTIVE_LEVELS, tiler_props.max_active_levels),

	PROP(MAX_THREADS, thread_props.max_threads),
	PROP(MAX_WORKGROUP_SIZE, thread_props.max_workgroup_size),
	PROP(MAX_BARRIER_SIZE, thread_props.max_barrier_size),
	PROP(MAX_REGISTERS, thread_props.max_registers),
	PROP(MAX_TASK_QUEUE, thread_props.max_task_queue),
	PROP(MAX_THREAD_GROUP_SPLIT, thread_props.max_thread_group_split),
	PROP(IMPL_TECH, thread_props.impl_tech),
	PROP(TLS_ALLOC, thread_props.tls_alloc),

	PROP(RAW_SHADER_PRESENT, raw_props.shader_present),
	PROP(RAW_TILER_PRESENT, raw_props.tiler_present),
	PROP(RAW_L2_PRESENT, raw_props.l2_present),
	PROP(RAW_STACK_PRESENT, raw_props.stack_present),
	PROP(RAW_L2_FEATURES, raw_props.l2_features),
	PROP(RAW_CORE_FEATURES, raw_props.core_features),
	PROP(RAW_MEM_FEATURES, raw_props.mem_features),
	PROP(RAW_MMU_FEATURES, raw_props.mmu_features),
	PROP(RAW_AS_PRESENT, raw_props.as_present),
	PROP(RAW_JS_PRESENT, raw_props.js_present),
	PROP(RAW_JS_FEATURES_0, raw_props.js_features[0]),
	PROP(RAW_JS_FEATURES_1, raw_props.js_features[1]),
	PROP(RAW_JS_FEATURES_2, raw_props.js_features[2]),
	PROP(RAW_JS_FEATURES_3, raw_props.js_features[3]),
	PROP(RAW_JS_FEATURES_4, raw_props.js_features[4]),
	PROP(RAW_JS_FEATURES_5, raw_props.js_features[5]),
	PROP(RAW_JS_FEATURES_6, raw_props.js_features[6]),
	PROP(RAW_JS_FEATURES_7, raw_props.js_features[7]),
	PROP(RAW_JS_FEATURES_8, raw_props.js_features[8]),
	PROP(RAW_JS_FEATURES_9, raw_props.js_features[9]),
	PROP(RAW_JS_FEATURES_10, raw_props.js_features[10]),
	PROP(RAW_JS_FEATURES_11, raw_props.js_features[11]),
	PROP(RAW_JS_FEATURES_12, raw_props.js_features[12]),
	PROP(RAW_JS_FEATURES_13, raw_props.js_features[13]),
	PROP(RAW_JS_FEATURES_14, raw_props.js_features[14]),
	PROP(RAW_JS_FEATURES_15, raw_props.js_features[15]),
	PROP(RAW_TILER_FEATURES, raw_props.tiler_features),
	PROP(RAW_TEXTURE_FEATURES_0, raw_props.texture_features[0]),
	PROP(RAW_TEXTURE_FEATURES_1, raw_props.texture_features[1]),
	PROP(RAW_TEXTURE_FEATURES_2, raw_props.texture_features[2]),
	PROP(RAW_TEXTURE_FEATURES_3, raw_props.texture_features[3]),
	PROP(RAW_GPU_ID, raw_props.gpu_id),
	PROP(RAW_THREAD_MAX_THREADS, raw_props.thread_max_threads),
	PROP(RAW_THREAD_MAX_WORKGROUP_SIZE, raw_props.thread_max_workgroup_size),
	PROP(RAW_THREAD_MAX_BARRIER_SIZE, raw_props.thread_max_barrier_size),
	PROP(RAW_THREAD_FEATURES, raw_props.thread_features),
	PROP(RAW_COHERENCY_MODE, raw_props.coherency_mode),
	PROP(RAW_THREAD_TLS_ALLOC, raw_props.thread_tls_alloc),
	PROP(RAW_GPU_FEATURES, raw_props.gpu_features),
	PROP(COHERENCY_NUM_GROUPS, coherency_info.num_groups),
	PROP(COHERENCY_NUM_CORE_GROUPS, coherency_info.num_core_groups),
	PROP(COHERENCY_COHERENCY, coherency_info.coherency),
	PROP(COHERENCY_GROUP_0, coherency_info.group[0].core_mask),
	PROP(COHERENCY_GROUP_1, coherency_info.group[1].core_mask),
	PROP(COHERENCY_GROUP_2, coherency_info.group[2].core_mask),
	PROP(COHERENCY_GROUP_3, coherency_info.group[3].core_mask),
	PROP(COHERENCY_GROUP_4, coherency_info.group[4].core_mask),
	PROP(COHERENCY_GROUP_5, coherency_info.group[5].core_mask),
	PROP(COHERENCY_GROUP_6, coherency_info.group[6].core_mask),
	PROP(COHERENCY_GROUP_7, coherency_info.group[7].core_mask),
	PROP(COHERENCY_GROUP_8, coherency_info.group[8].core_mask),
	PROP(COHERENCY_GROUP_9, coherency_info.group[9].core_mask),
	PROP(COHERENCY_GROUP_10, coherency_info.group[10].core_mask),
	PROP(COHERENCY_GROUP_11, coherency_info.group[11].core_mask),
	PROP(COHERENCY_GROUP_12, coherency_info.group[12].core_mask),
	PROP(COHERENCY_GROUP_13, coherency_info.group[13].core_mask),
	PROP(COHERENCY_GROUP_14, coherency_info.group[14].core_mask),
	PROP(COHERENCY_GROUP_15, coherency_info.group[15].core_mask),

#undef PROP
};

/**
 * kbase_populate_user_data - Populate user data properties from kbase props and
 *                             raw register values
 * @kbdev:  The kbase device pointer
 * @data:   The user properties data struct pointer
 */
static void kbase_populate_user_data(struct kbase_device *kbdev, struct gpu_props_user_data *data)
{
	struct kbase_gpu_props *kprops = &kbdev->gpu_props;
	struct kbasep_gpuprops_regdump *regdump = &PRIV_DATA_REGDUMP(kbdev);
	int i = 0;

	if (WARN_ON(!kbdev) || WARN_ON(!data) || WARN_ON(!regdump))
		return;

	/* Properties from kbase_gpu_props */
	data->core_props.version_status = kprops->gpu_id.version_status;
	data->core_props.minor_revision = kprops->gpu_id.version_minor;
	data->core_props.major_revision = kprops->gpu_id.version_major;
	data->core_props.gpu_freq_khz_max = kprops->gpu_freq_khz_max;
	data->core_props.log2_program_counter_size = kprops->log2_program_counter_size;
	data->l2_props.log2_line_size = kprops->log2_line_size;
	data->l2_props.num_l2_slices = kprops->num_l2_slices;
	data->raw_props.shader_present = kprops->shader_present;
	data->raw_props.l2_present = kprops->l2_present;
	data->raw_props.tiler_present = kprops->tiler_present;
	data->raw_props.stack_present = kprops->stack_present;

	/* On Bifrost+ GPUs, there is only 1 coherent group */
	data->coherency_info.num_groups = 1;
	data->coherency_info.num_core_groups = kprops->num_core_groups;
	data->coherency_info.group[0].core_mask = kprops->coherency_info.group.core_mask;
	data->coherency_info.group[0].num_cores = kprops->coherency_info.group.num_cores;

	data->thread_props.max_threads = kprops->max_threads;
	data->thread_props.impl_tech = kprops->impl_tech;
	data->raw_props.coherency_mode = kprops->coherency_mode;

	/* Properties (mostly) from raw register values */
	data->raw_props.gpu_id = regdump->gpu_id;

	{
		data->core_props.product_id = KBASE_UBFX64(regdump->gpu_id, 16U, 16);
	}

	for (i = 0; i < BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS; i++) {
		data->core_props.texture_features[i] = regdump->texture_features[i];
		data->raw_props.texture_features[i] = regdump->texture_features[i];
	}

	data->core_props.gpu_available_memory_size = kbase_totalram_pages() << PAGE_SHIFT;

	/*
	 * The CORE_FEATURES register has different meanings depending on GPU.
	 * On tGOx, bits[3:0] encode num_exec_engines.
	 * On CSF GPUs, bits[7:0] is an enumeration that needs to be parsed,
	 * instead.
	 * GPUs like tTIx have additional fields like LSC_SIZE that are
	 * otherwise reserved/RAZ on older GPUs.
	 */
#if !MALI_USE_CSF
	data->core_props.num_exec_engines = KBASE_UBFX64(regdump->core_features, 0, 4);
#endif

	data->l2_props.log2_cache_size = KBASE_UBFX64(regdump->l2_features, 16U, 8);
	data->coherency_info.coherency = regdump->mem_features;

	data->tiler_props.bin_size_bytes = 1U << KBASE_UBFX64(regdump->tiler_features, 0U, 6);
	data->tiler_props.max_active_levels = KBASE_UBFX32(regdump->tiler_features, 8U, 4);

	if (regdump->thread_max_workgroup_size == 0)
		data->thread_props.max_workgroup_size = THREAD_MWS_DEFAULT;
	else
		data->thread_props.max_workgroup_size = regdump->thread_max_workgroup_size;

	if (regdump->thread_max_barrier_size == 0)
		data->thread_props.max_barrier_size = THREAD_MBS_DEFAULT;
	else
		data->thread_props.max_barrier_size = regdump->thread_max_barrier_size;

	if (regdump->thread_tls_alloc == 0)
		data->thread_props.tls_alloc = kprops->max_threads;
	else
		data->thread_props.tls_alloc = regdump->thread_tls_alloc;

#if MALI_USE_CSF
	data->thread_props.max_registers = KBASE_UBFX32(regdump->thread_features, 0U, 22);
	data->thread_props.max_task_queue = KBASE_UBFX32(regdump->thread_features, 24U, 8);
	data->thread_props.max_thread_group_split = 0;
#else
	data->thread_props.max_registers = KBASE_UBFX32(regdump->thread_features, 0U, 16);
	data->thread_props.max_task_queue = KBASE_UBFX32(regdump->thread_features, 16U, 8);
	data->thread_props.max_thread_group_split = KBASE_UBFX32(regdump->thread_features, 24U, 6);
#endif

	if (data->thread_props.max_registers == 0) {
		data->thread_props.max_registers = THREAD_MR_DEFAULT;
		data->thread_props.max_task_queue = THREAD_MTQ_DEFAULT;
		data->thread_props.max_thread_group_split = THREAD_MTGS_DEFAULT;
	}

	if (!kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_THREAD_GROUP_SPLIT))
		data->thread_props.max_thread_group_split = 0;

	/* Raw Register Values */
	data->raw_props.l2_features = regdump->l2_features;
	data->raw_props.core_features = regdump->core_features;
	data->raw_props.mem_features = regdump->mem_features;
	data->raw_props.mmu_features = regdump->mmu_features;
	data->raw_props.as_present = regdump->as_present;
	data->raw_props.js_present = regdump->js_present;

	for (i = 0; i < GPU_MAX_JOB_SLOTS; i++)
		data->raw_props.js_features[i] = regdump->js_features[i];

	data->raw_props.tiler_features = regdump->tiler_features;

	data->raw_props.thread_max_threads = regdump->thread_max_threads;
	data->raw_props.thread_max_workgroup_size = regdump->thread_max_workgroup_size;
	data->raw_props.thread_max_barrier_size = regdump->thread_max_barrier_size;
	data->raw_props.thread_features = regdump->thread_features;
	data->raw_props.thread_tls_alloc = regdump->thread_tls_alloc;
	data->raw_props.gpu_features = regdump->gpu_features;

}

int kbase_gpuprops_populate_user_buffer(struct kbase_device *kbdev)
{
	struct kbase_gpu_props *kprops = &kbdev->gpu_props;
	struct gpu_props_user_data props;
	u32 count = ARRAY_SIZE(gpu_property_mapping);
	u32 i;
	u32 size = 0;
	u8 *p;

	memset(&props, 0, sizeof(props));

	/* Populate user data structure from kbase props and raw register values */
	kbase_populate_user_data(kbdev, &props);

	/* Free private data after used to populate user data structure */
	kfree(kprops->priv_data);
	kprops->priv_data = NULL;

	for (i = 0; i < count; i++) {
		/* 4 bytes for the ID, and the size of the property */
		size += (u32)(4 + gpu_property_mapping[i].size);
	}

	kprops->prop_buffer_size = size;
	kprops->prop_buffer = kzalloc(size, GFP_KERNEL);

	if (!kprops->prop_buffer) {
		kprops->prop_buffer_size = 0;
		return -ENOMEM;
	}

	p = kprops->prop_buffer;

#define WRITE_U8(v) (*p++ = (v)&0xFF)
#define WRITE_U16(v)                \
	do {                        \
		WRITE_U8(v);        \
		WRITE_U8((v) >> 8); \
	} while (0)
#define WRITE_U32(v)                  \
	do {                          \
		WRITE_U16(v);         \
		WRITE_U16((v) >> 16); \
	} while (0)
#define WRITE_U64(v)                  \
	do {                          \
		WRITE_U32(v);         \
		WRITE_U32((v) >> 32); \
	} while (0)

	for (i = 0; i < count; i++) {
		u32 type = gpu_property_mapping[i].type;
		u8 type_size;
		const size_t offset = gpu_property_mapping[i].offset;
		const u64 dummy_backwards_compat_value = (u64)0;
		const void *field;

		if (likely(offset < sizeof(struct gpu_props_user_data)))
			field = ((const u8 *)&props) + offset;
		else
			field = &dummy_backwards_compat_value;

		switch (gpu_property_mapping[i].size) {
		case 1:
			type_size = KBASE_GPUPROP_VALUE_SIZE_U8;
			break;
		case 2:
			type_size = KBASE_GPUPROP_VALUE_SIZE_U16;
			break;
		case 4:
			type_size = KBASE_GPUPROP_VALUE_SIZE_U32;
			break;
		case 8:
			type_size = KBASE_GPUPROP_VALUE_SIZE_U64;
			break;
		default:
			dev_err(kbdev->dev, "Invalid gpu_property_mapping type=%d size=%d", type,
				gpu_property_mapping[i].size);
			kbase_gpuprops_free_user_buffer(kbdev);
			return -EINVAL;
		}

		WRITE_U32((type << 2) | type_size);

		switch (type_size) {
		case KBASE_GPUPROP_VALUE_SIZE_U8:
			WRITE_U8(*((const u8 *)field));
			break;
		case KBASE_GPUPROP_VALUE_SIZE_U16:
			WRITE_U16(*((const u16 *)field));
			break;
		case KBASE_GPUPROP_VALUE_SIZE_U32:
			WRITE_U32(*((const u32 *)field));
			break;
		case KBASE_GPUPROP_VALUE_SIZE_U64:
			WRITE_U64(*((const u64 *)field));
			break;
		default: /* Cannot be reached */
			WARN_ON(1);
			return -EINVAL;
		}
	}

	return 0;
}

void kbase_gpuprops_free_user_buffer(struct kbase_device *kbdev)
{
	kfree(kbdev->gpu_props.prop_buffer);
}

int kbase_device_populate_max_freq(struct kbase_device *kbdev)
{
	/* obtain max configured gpu frequency, if devfreq is enabled then
	 * this will be overridden by the highest operating point found
	 */
	kbdev->gpu_props.gpu_freq_khz_max = DEFAULT_GPU_FREQ_KHZ_MAX;
	return 0;
}
