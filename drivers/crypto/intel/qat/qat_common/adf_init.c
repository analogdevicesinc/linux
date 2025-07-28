// SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0-only)
/* Copyright(c) 2014 - 2020 Intel Corporation */
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include "adf_accel_devices.h"
#include "adf_cfg.h"
#include "adf_common_drv.h"
#include "adf_dbgfs.h"
#include "adf_heartbeat.h"
#include "adf_rl.h"
#include "adf_sysfs_ras_counters.h"
#include "adf_telemetry.h"

static LIST_HEAD(service_table);
static DEFINE_MUTEX(service_lock);

static void adf_service_add(struct service_hndl *service)
{
	mutex_lock(&service_lock);
	list_add(&service->list, &service_table);
	mutex_unlock(&service_lock);
}

int adf_service_register(struct service_hndl *service)
{
	memset(service->init_status, 0, sizeof(service->init_status));
	memset(service->start_status, 0, sizeof(service->start_status));
	adf_service_add(service);
	return 0;
}

static void adf_service_remove(struct service_hndl *service)
{
	mutex_lock(&service_lock);
	list_del(&service->list);
	mutex_unlock(&service_lock);
}

int adf_service_unregister(struct service_hndl *service)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(service->init_status); i++) {
		if (service->init_status[i] || service->start_status[i]) {
			pr_err("QAT: Could not remove active service\n");
			return -EFAULT;
		}
	}
	adf_service_remove(service);
	return 0;
}

/**
 * adf_dev_init() - Init data structures and services for the given accel device
 * @accel_dev: Pointer to acceleration device.
 *
 * Initialize the ring data structures and the admin comms and arbitration
 * services.
 *
 * Return: 0 on success, error code otherwise.
 */
static int adf_dev_init(struct adf_accel_dev *accel_dev)
{
	struct service_hndl *service;
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	int ret;

	if (!hw_data) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to init device - hw_data not set\n");
		return -EFAULT;
	}

	if (!test_bit(ADF_STATUS_CONFIGURED, &accel_dev->status) &&
	    !accel_dev->is_vf) {
		dev_err(&GET_DEV(accel_dev), "Device not configured\n");
		return -EFAULT;
	}

	if (adf_init_etr_data(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed initialize etr\n");
		return -EFAULT;
	}

	if (hw_data->init_device && hw_data->init_device(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed to initialize device\n");
		return -EFAULT;
	}

	if (hw_data->init_admin_comms && hw_data->init_admin_comms(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed initialize admin comms\n");
		return -EFAULT;
	}

	if (hw_data->init_arb && hw_data->init_arb(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed initialize hw arbiter\n");
		return -EFAULT;
	}

	if (hw_data->get_ring_to_svc_map)
		hw_data->ring_to_svc_map = hw_data->get_ring_to_svc_map(accel_dev);

	if (adf_ae_init(accel_dev)) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to initialise Acceleration Engine\n");
		return -EFAULT;
	}
	set_bit(ADF_STATUS_AE_INITIALISED, &accel_dev->status);

	if (adf_ae_fw_load(accel_dev)) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to load acceleration FW\n");
		return -EFAULT;
	}
	set_bit(ADF_STATUS_AE_UCODE_LOADED, &accel_dev->status);

	if (hw_data->alloc_irq(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed to allocate interrupts\n");
		return -EFAULT;
	}
	set_bit(ADF_STATUS_IRQ_ALLOCATED, &accel_dev->status);

	if (hw_data->ras_ops.enable_ras_errors)
		hw_data->ras_ops.enable_ras_errors(accel_dev);

	hw_data->enable_ints(accel_dev);
	hw_data->enable_error_correction(accel_dev);

	ret = hw_data->pfvf_ops.enable_comms(accel_dev);
	if (ret)
		return ret;

	if (!test_bit(ADF_STATUS_CONFIGURED, &accel_dev->status) &&
	    accel_dev->is_vf) {
		if (qat_crypto_vf_dev_config(accel_dev))
			return -EFAULT;
	}

	adf_heartbeat_init(accel_dev);
	ret = adf_rl_init(accel_dev);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	ret = adf_tl_init(accel_dev);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	/*
	 * Subservice initialisation is divided into two stages: init and start.
	 * This is to facilitate any ordering dependencies between services
	 * prior to starting any of the accelerators.
	 */
	list_for_each_entry(service, &service_table, list) {
		if (service->event_hld(accel_dev, ADF_EVENT_INIT)) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to initialise service %s\n",
				service->name);
			return -EFAULT;
		}
		set_bit(accel_dev->accel_id, service->init_status);
	}

	return 0;
}

/**
 * adf_dev_start() - Start acceleration service for the given accel device
 * @accel_dev:    Pointer to acceleration device.
 *
 * Function notifies all the registered services that the acceleration device
 * is ready to be used.
 * To be used by QAT device specific drivers.
 *
 * Return: 0 on success, error code otherwise.
 */
static int adf_dev_start(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct service_hndl *service;
	int ret;

	set_bit(ADF_STATUS_STARTING, &accel_dev->status);

	if (adf_ae_start(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "AE Start Failed\n");
		return -EFAULT;
	}
	set_bit(ADF_STATUS_AE_STARTED, &accel_dev->status);

	if (hw_data->send_admin_init(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed to send init message\n");
		return -EFAULT;
	}

	if (hw_data->measure_clock) {
		ret = hw_data->measure_clock(accel_dev);
		if (ret) {
			dev_err(&GET_DEV(accel_dev), "Failed measure device clock\n");
			return ret;
		}
	}

	/* Set ssm watch dog timer */
	if (hw_data->set_ssm_wdtimer)
		hw_data->set_ssm_wdtimer(accel_dev);

	/* Enable Power Management */
	if (hw_data->enable_pm && hw_data->enable_pm(accel_dev)) {
		dev_err(&GET_DEV(accel_dev), "Failed to configure Power Management\n");
		return -EFAULT;
	}

	if (hw_data->start_timer) {
		ret = hw_data->start_timer(accel_dev);
		if (ret) {
			dev_err(&GET_DEV(accel_dev), "Failed to start internal sync timer\n");
			return ret;
		}
	}

	adf_heartbeat_start(accel_dev);
	ret = adf_rl_start(accel_dev);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	ret = adf_tl_start(accel_dev);
	if (ret && ret != -EOPNOTSUPP)
		return ret;

	list_for_each_entry(service, &service_table, list) {
		if (service->event_hld(accel_dev, ADF_EVENT_START)) {
			dev_err(&GET_DEV(accel_dev),
				"Failed to start service %s\n",
				service->name);
			return -EFAULT;
		}
		set_bit(accel_dev->accel_id, service->start_status);
	}

	clear_bit(ADF_STATUS_STARTING, &accel_dev->status);
	set_bit(ADF_STATUS_STARTED, &accel_dev->status);

	if (!list_empty(&accel_dev->crypto_list) &&
	    (qat_algs_register() || qat_asym_algs_register())) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to register crypto algs\n");
		set_bit(ADF_STATUS_STARTING, &accel_dev->status);
		clear_bit(ADF_STATUS_STARTED, &accel_dev->status);
		return -EFAULT;
	}
	set_bit(ADF_STATUS_CRYPTO_ALGS_REGISTERED, &accel_dev->status);

	if (!list_empty(&accel_dev->compression_list) && qat_comp_algs_register()) {
		dev_err(&GET_DEV(accel_dev),
			"Failed to register compression algs\n");
		set_bit(ADF_STATUS_STARTING, &accel_dev->status);
		clear_bit(ADF_STATUS_STARTED, &accel_dev->status);
		return -EFAULT;
	}
	set_bit(ADF_STATUS_COMP_ALGS_REGISTERED, &accel_dev->status);

	adf_dbgfs_add(accel_dev);
	adf_sysfs_start_ras(accel_dev);

	return 0;
}

/**
 * adf_dev_stop() - Stop acceleration service for the given accel device
 * @accel_dev:    Pointer to acceleration device.
 *
 * Function notifies all the registered services that the acceleration device
 * is shuting down.
 * To be used by QAT device specific drivers.
 *
 * Return: void
 */
static void adf_dev_stop(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct service_hndl *service;
	bool wait = false;
	int ret;

	if (!adf_dev_started(accel_dev) &&
	    !test_bit(ADF_STATUS_STARTING, &accel_dev->status))
		return;

	adf_tl_stop(accel_dev);
	adf_rl_stop(accel_dev);
	adf_dbgfs_rm(accel_dev);
	adf_sysfs_stop_ras(accel_dev);

	clear_bit(ADF_STATUS_STARTING, &accel_dev->status);
	clear_bit(ADF_STATUS_STARTED, &accel_dev->status);

	if (!list_empty(&accel_dev->crypto_list) &&
	    test_bit(ADF_STATUS_CRYPTO_ALGS_REGISTERED, &accel_dev->status)) {
		qat_algs_unregister();
		qat_asym_algs_unregister();
	}
	clear_bit(ADF_STATUS_CRYPTO_ALGS_REGISTERED, &accel_dev->status);

	if (!list_empty(&accel_dev->compression_list) &&
	    test_bit(ADF_STATUS_COMP_ALGS_REGISTERED, &accel_dev->status))
		qat_comp_algs_unregister();
	clear_bit(ADF_STATUS_COMP_ALGS_REGISTERED, &accel_dev->status);

	list_for_each_entry(service, &service_table, list) {
		if (!test_bit(accel_dev->accel_id, service->start_status))
			continue;
		ret = service->event_hld(accel_dev, ADF_EVENT_STOP);
		if (!ret) {
			clear_bit(accel_dev->accel_id, service->start_status);
		} else if (ret == -EAGAIN) {
			wait = true;
			clear_bit(accel_dev->accel_id, service->start_status);
		}
	}

	if (hw_data->stop_timer)
		hw_data->stop_timer(accel_dev);

	hw_data->disable_iov(accel_dev);

	if (wait)
		msleep(100);

	if (test_bit(ADF_STATUS_AE_STARTED, &accel_dev->status)) {
		if (adf_ae_stop(accel_dev))
			dev_err(&GET_DEV(accel_dev), "failed to stop AE\n");
		else
			clear_bit(ADF_STATUS_AE_STARTED, &accel_dev->status);
	}
}

/**
 * adf_dev_shutdown() - shutdown acceleration services and data strucutures
 * @accel_dev: Pointer to acceleration device
 *
 * Cleanup the ring data structures and the admin comms and arbitration
 * services.
 */
static void adf_dev_shutdown(struct adf_accel_dev *accel_dev)
{
	struct adf_hw_device_data *hw_data = accel_dev->hw_device;
	struct service_hndl *service;

	if (!hw_data) {
		dev_err(&GET_DEV(accel_dev),
			"QAT: Failed to shutdown device - hw_data not set\n");
		return;
	}

	if (test_bit(ADF_STATUS_AE_UCODE_LOADED, &accel_dev->status)) {
		adf_ae_fw_release(accel_dev);
		clear_bit(ADF_STATUS_AE_UCODE_LOADED, &accel_dev->status);
	}

	if (test_bit(ADF_STATUS_AE_INITIALISED, &accel_dev->status)) {
		if (adf_ae_shutdown(accel_dev))
			dev_err(&GET_DEV(accel_dev),
				"Failed to shutdown Accel Engine\n");
		else
			clear_bit(ADF_STATUS_AE_INITIALISED,
				  &accel_dev->status);
	}

	list_for_each_entry(service, &service_table, list) {
		if (!test_bit(accel_dev->accel_id, service->init_status))
			continue;
		if (service->event_hld(accel_dev, ADF_EVENT_SHUTDOWN))
			dev_err(&GET_DEV(accel_dev),
				"Failed to shutdown service %s\n",
				service->name);
		else
			clear_bit(accel_dev->accel_id, service->init_status);
	}

	adf_rl_exit(accel_dev);

	if (hw_data->ras_ops.disable_ras_errors)
		hw_data->ras_ops.disable_ras_errors(accel_dev);

	adf_heartbeat_shutdown(accel_dev);

	adf_tl_shutdown(accel_dev);

	if (test_bit(ADF_STATUS_IRQ_ALLOCATED, &accel_dev->status)) {
		hw_data->free_irq(accel_dev);
		clear_bit(ADF_STATUS_IRQ_ALLOCATED, &accel_dev->status);
	}

	/* If not restarting, delete all cfg sections except for GENERAL */
	if (!test_bit(ADF_STATUS_RESTARTING, &accel_dev->status))
		adf_cfg_del_all_except(accel_dev, ADF_GENERAL_SEC);

	if (hw_data->exit_arb)
		hw_data->exit_arb(accel_dev);

	if (hw_data->exit_admin_comms)
		hw_data->exit_admin_comms(accel_dev);

	adf_cleanup_etr_data(accel_dev);
	adf_dev_restore(accel_dev);
}

int adf_dev_restarting_notify(struct adf_accel_dev *accel_dev)
{
	struct service_hndl *service;

	list_for_each_entry(service, &service_table, list) {
		if (service->event_hld(accel_dev, ADF_EVENT_RESTARTING))
			dev_err(&GET_DEV(accel_dev),
				"Failed to restart service %s.\n",
				service->name);
	}
	return 0;
}

int adf_dev_restarted_notify(struct adf_accel_dev *accel_dev)
{
	struct service_hndl *service;

	list_for_each_entry(service, &service_table, list) {
		if (service->event_hld(accel_dev, ADF_EVENT_RESTARTED))
			dev_err(&GET_DEV(accel_dev),
				"Failed to restart service %s.\n",
				service->name);
	}
	return 0;
}

void adf_error_notifier(struct adf_accel_dev *accel_dev)
{
	struct service_hndl *service;

	list_for_each_entry(service, &service_table, list) {
		if (service->event_hld(accel_dev, ADF_EVENT_FATAL_ERROR))
			dev_err(&GET_DEV(accel_dev),
				"Failed to send error event to %s.\n",
				service->name);
	}
}

int adf_dev_down(struct adf_accel_dev *accel_dev)
{
	int ret = 0;

	if (!accel_dev)
		return -EINVAL;

	mutex_lock(&accel_dev->state_lock);

	adf_dev_stop(accel_dev);
	adf_dev_shutdown(accel_dev);

	mutex_unlock(&accel_dev->state_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_dev_down);

int adf_dev_up(struct adf_accel_dev *accel_dev, bool config)
{
	int ret = 0;

	if (!accel_dev)
		return -EINVAL;

	mutex_lock(&accel_dev->state_lock);

	if (adf_dev_started(accel_dev)) {
		dev_info(&GET_DEV(accel_dev), "Device qat_dev%d already up\n",
			 accel_dev->accel_id);
		ret = -EALREADY;
		goto out;
	}

	if (config && GET_HW_DATA(accel_dev)->dev_config) {
		ret = GET_HW_DATA(accel_dev)->dev_config(accel_dev);
		if (unlikely(ret))
			goto out;
	}

	ret = adf_dev_init(accel_dev);
	if (unlikely(ret))
		goto out;

	ret = adf_dev_start(accel_dev);

out:
	mutex_unlock(&accel_dev->state_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adf_dev_up);

int adf_dev_restart(struct adf_accel_dev *accel_dev)
{
	int ret = 0;

	if (!accel_dev)
		return -EFAULT;

	adf_dev_down(accel_dev);

	ret = adf_dev_up(accel_dev, false);
	/* if device is already up return success*/
	if (ret == -EALREADY)
		return 0;

	return ret;
}
EXPORT_SYMBOL_GPL(adf_dev_restart);
