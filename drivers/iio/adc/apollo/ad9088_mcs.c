// SPDX-License-Identifier: GPL-2.0
/*
 * AD9088 MCS (Multi-Chip Synchronization) calibration support
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include "ad9088.h"

int ad9088_mcs_init_cal_setup(struct ad9088_phy *phy)
{
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	if (!device->dev_info.is_dual_clk) {
		ret = adi_apollo_mcs_cal_parameter_set(device, MCS_OFFSET_C_FEMTOSECONDS_INT64, 0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_parameter_set");
		if (ret)
			return ret;
	} else {
		ret = adi_apollo_mcs_cal_parameter_set(device, MCS_OFFSET_A_FEMTOSECONDS_INT64, 0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_parameter_set");
		if (ret)
			return ret;
		ret = adi_apollo_mcs_cal_parameter_set(device, MCS_OFFSET_B_FEMTOSECONDS_INT64, 0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_parameter_set");
		if (ret)
			return ret;
	}

	return 0;
}

int ad9088_mcs_init_cal_status_print(struct ad9088_phy *phy, char *buf,
				     adi_apollo_mcs_cal_init_status_t *cal_status)
{
	int len = 0;

	len += sprintf(buf + len,  "errorCode: %d.\n", cal_status->hdr.errorCode);
	len += sprintf(buf + len,  "percentComplete: %d.\n", cal_status->hdr.percentComplete);
	len += sprintf(buf + len,  "performanceMetric: %d.\n", cal_status->hdr.performanceMetric);
	len += sprintf(buf + len,  "iterCount: %d.\n", cal_status->hdr.iterCount);
	len += sprintf(buf + len,  "updateCount: %d.\n\n", cal_status->hdr.updateCount);

	len += sprintf(buf + len,  "mcsErr: %d.\n\n", cal_status->mcsErr);

	len += sprintf(buf + len,  "is_C_Locked: %d.\n", cal_status->data.is_C_Locked);
	len += sprintf(buf + len,  "is_A_Locked: %d.\n", cal_status->data.is_A_Locked);
	len += sprintf(buf + len,  "is_B_Locked: %d.\n\n", cal_status->data.is_B_Locked);

	len += sprintf(buf + len,  "diff_C_Before_femtoseconds: %lld.\n", cal_status->data.diff_C_Before_femtoseconds);
	len += sprintf(buf + len,  "diff_A_Before_femtoseconds: %lld.\n", cal_status->data.diff_A_Before_femtoseconds);
	len += sprintf(buf + len,  "diff_B_Before_femtoseconds: %lld.\n", cal_status->data.diff_B_Before_femtoseconds);
	len += sprintf(buf + len,  "internal_period_C_femtoseconds: %lld.\n", cal_status->data.internal_period_C_femtoseconds);
	len += sprintf(buf + len,  "internal_period_A_femtoseconds: %lld.\n", cal_status->data.internal_period_A_femtoseconds);
	len += sprintf(buf + len,  "internal_period_B_femtoseconds: %lld.\n", cal_status->data.internal_period_B_femtoseconds);
	len += sprintf(buf + len,  "diff_C_After_femtoseconds: %lld.\n", cal_status->data.diff_C_After_femtoseconds);
	len += sprintf(buf + len,  "diff_A_After_femtoseconds: %lld.\n", cal_status->data.diff_A_After_femtoseconds);
	len += sprintf(buf + len,  "diff_B_After_femtoseconds: %lld.\n", cal_status->data.diff_B_After_femtoseconds);
	len += sprintf(buf + len,  "recommended_offset_C_femtoseconds: %lld.\n",
		       cal_status->data.recommended_offset_C_femtoseconds);
	len += sprintf(buf + len,  "recommended_offset_A_femtoseconds: %lld.\n",
		       cal_status->data.recommended_offset_A_femtoseconds);
	len += sprintf(buf + len,  "recommended_offset_B_femtoseconds: %lld.\n",
		       cal_status->data.recommended_offset_B_femtoseconds);

	return len;
}

int ad9088_mcs_track_cal_status_print(struct ad9088_phy *phy, char *buf,
				      adi_apollo_mcs_cal_status_t *cal_status,
				      u8 print_full_state)
{
	int len = 0;

	if (print_full_state == 1) {
		len += sprintf(buf + len,  "errorCode: %d.\n", cal_status->hdr.errorCode);
		len += sprintf(buf + len,  "percentComplete: %d.\n", cal_status->hdr.percentComplete);
		len += sprintf(buf + len,  "performanceMetric: %d.\n", cal_status->hdr.performanceMetric);
		len += sprintf(buf + len,  "iterCount: %d.\n", cal_status->hdr.iterCount);
		len += sprintf(buf + len,  "updateCount: %d.\n\n", cal_status->hdr.updateCount);

		len += sprintf(buf + len,  "foreground_done: %d.\n", cal_status->mcs_tracking_cal_status.foreground_done);
		len += sprintf(buf + len,  "track_state[0]: %d.\n", cal_status->mcs_tracking_cal_status.track_state[0]);
		len += sprintf(buf + len,  "track_state[1]: %d.\n", cal_status->mcs_tracking_cal_status.track_state[1]);
		len += sprintf(buf + len,  "track_lock[0]: %d.\n", cal_status->mcs_tracking_cal_status.track_lock[0]);
		len += sprintf(buf + len,  "track_lock[1]: %d.\n", cal_status->mcs_tracking_cal_status.track_lock[1]);
		len += sprintf(buf + len,  "halt_active: %d.\n\n", cal_status->mcs_tracking_cal_status.halt_active);
		len += sprintf(buf + len,  "force_background_done[0]: %d.\n",
			       cal_status->mcs_tracking_cal_status.force_background_done[0]);
		len += sprintf(buf + len,  "force_background_done[1]: %d.\n",
			       cal_status->mcs_tracking_cal_status.force_background_done[1]);
		len += sprintf(buf + len,  "abort_done: %d.\n\n", cal_status->mcs_tracking_cal_status.abort_done);

		len += sprintf(buf + len,  "adf4382_specific_status[0].bleed_pol: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].bleed_pol);
		len += sprintf(buf + len,  "adf4382_specific_status[0].current_coarse_value: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].current_coarse_value);
		len += sprintf(buf + len,  "adf4382_specific_status[0].current_fine_value: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].current_fine_value);
		len += sprintf(buf + len,  "adf4382_specific_status[0].EOR_POS: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].EOR_POS);
		len += sprintf(buf + len,  "adf4382_specific_status[0].EOR_NEG: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].EOR_NEG);
		len += sprintf(buf + len,  "adf4382_specific_status[0].EOR_Coarse: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].EOR_Coarse);

		len += sprintf(buf + len,  "adf4382_specific_status[1].bleed_pol: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].bleed_pol);
		len += sprintf(buf + len,  "adf4382_specific_status[1].current_coarse_value: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].current_coarse_value);
		len += sprintf(buf + len,  "adf4382_specific_status[1].current_fine_value: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].current_fine_value);
		len += sprintf(buf + len,  "adf4382_specific_status[1].EOR_POS: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].EOR_POS);
		len += sprintf(buf + len,  "adf4382_specific_status[1].EOR_NEG: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].EOR_NEG);
		len += sprintf(buf + len,  "adf4382_specific_status[1].EOR_Coarse: %d.\n",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[1].EOR_Coarse);

		len += sprintf(buf + len,  "current_measure[0]: %lld.\n", cal_status->mcs_tracking_cal_status.current_measure[0]);
		len += sprintf(buf + len,  "current_measure[1]: %lld.\n", cal_status->mcs_tracking_cal_status.current_measure[1]);
	} else {
		len += sprintf(buf + len,  "Tracking Cal[0]: --> \t");
		len += sprintf(buf + len,  "bleed_pol: %d. \t",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].bleed_pol);
		len += sprintf(buf + len,  "current_coarse: %d. \t",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].current_coarse_value);
		len += sprintf(buf + len,  "current_fine: %d. \t",
			       cal_status->mcs_tracking_cal_status.adf4382_specific_status[0].current_fine_value);
		len += sprintf(buf + len,  "current_measure: %lld.\n", cal_status->mcs_tracking_cal_status.current_measure[0]);
	}

	return len;
}

int ad9088_delta_t_measurement_set(struct ad9088_phy *phy, u32 mode)
{
	adi_apollo_mailbox_cmd_mcs_bsync_set_config_t bsync_set_config_cmd = {0};
	adi_apollo_mailbox_resp_mcs_bsync_set_config_t bsync_set_config_resp = {0};
	adi_apollo_mailbox_resp_mcs_bsync_go_t bsync_go_resp = {0};
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	u32 bsync_divider = (phy->profile.clk_cfg.clocking_mode == ADI_APOLLO_CLOCKING_MODE_SDR_DIV_8 ? 8 : 4) *
			    phy->profile.mcs_cfg.internal_sysref_prd_digclk_cycles_center;

	switch (mode) {
	case 0:
		bsync_set_config_cmd.func_mode = APOLLO_MCS_BSYNC_ALIGN;
		break;
	case 1:
		bsync_set_config_cmd.func_mode = APOLLO_MCS_BSYNC_OUTPUT_EN;
		break;
	case 2:
		bsync_set_config_cmd.func_mode = APOLLO_MCS_BSYNC_OUTPUT_DIS;
		break;
	default:
		return -EINVAL;
	}

	bsync_set_config_cmd.bsync_div = bsync_divider;

	ret = adi_apollo_mailbox_mcs_bsync_set_config(device, &bsync_set_config_cmd, &bsync_set_config_resp);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mailbox_mcs_bsync_set_config");
	if (ret)
		return ret;

	if (bsync_set_config_resp.status)
		dev_warn(&phy->spi->dev, "bsync_set_config_resp.status: %d.\n", bsync_set_config_resp.status);

	ret = adi_apollo_mailbox_mcs_bsync_go(device, &bsync_go_resp);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mailbox_mcs_bsync_go");
	if (ret)
		return ret;

	if (bsync_go_resp.status)
		dev_warn(&phy->spi->dev, "bsync_go_resp.status: %d.\n", bsync_go_resp.status);

	return 0;
}

int ad9088_delta_t_measurement_get(struct ad9088_phy *phy, u32 mode, s64 *apollo_delta_t)
{
	adi_apollo_mailbox_resp_mcs_bsync_get_config_t bsync_get_config_resp = {0};
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	ret = adi_apollo_mailbox_mcs_bsync_get_config(device, &bsync_get_config_resp);
	if (ret)
		return ret;

	switch (mode) {
	case 0:
		*apollo_delta_t = bsync_get_config_resp.delta_t0;
		break;
	case 1:
		*apollo_delta_t = bsync_get_config_resp.delta_t1;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(&phy->spi->dev, "bsync_get_config_resp:\n");
	dev_dbg(&phy->spi->dev, "\t status: %d.\n", bsync_get_config_resp.status);
	dev_dbg(&phy->spi->dev, "\t func_mode: %d.\n", bsync_get_config_resp.func_mode);
	dev_dbg(&phy->spi->dev, "\t bsync_div: %d.\n", bsync_get_config_resp.bsync_div);
	dev_dbg(&phy->spi->dev, "\t done_flag: %d.\n", bsync_get_config_resp.done_flag);
	dev_dbg(&phy->spi->dev, "\t delta_t0: %lld.\n", bsync_get_config_resp.delta_t0);
	dev_dbg(&phy->spi->dev, "\t delta_t1: %lld.\n\n", bsync_get_config_resp.delta_t1);

	return 0;
}

int ad9088_mcs_init_cal_validate(struct ad9088_phy *phy,
				 adi_apollo_mcs_cal_init_status_t *cal_status)
{
	int ret = 0;

	adi_apollo_device_t *device = &phy->ad9088;
	u64 dev_clk_hz = (u64)phy->profile.clk_cfg.dev_clk_freq_kHz * 1000;

	/* External and Internal Time Difference must be within +/- 0.4 clock cycles */
	u32 post_cal_init_sysref_diff_cycles;
	u64 int_sysref_align_diff = abs(cal_status->data.diff_C_After_femtoseconds -
					cal_status->data.recommended_offset_C_femtoseconds);
	post_cal_init_sysref_diff_cycles = div64_u64(int_sysref_align_diff * dev_clk_hz, 1000000000000ULL);

	if (post_cal_init_sysref_diff_cycles > 400) {
		adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR,
					 "Time difference between internal and External SysRefs is too large: %u.%02u\n",
					 post_cal_init_sysref_diff_cycles / 1000, post_cal_init_sysref_diff_cycles % 1000);
		ret = -EFAULT;
		goto end;
	}

	/* Check if sysref is locked */
	if (cal_status->data.is_C_Locked != 1) {
		adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, "MCS Init Cal did not lock SysRefs.\n");
		ret = -EFAULT;
		goto end;
	}

	/* Check for Cal errors */
	if (cal_status->hdr.errorCode != 0) {
		adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, "MCS Init Cal Apollo CPU errorCode: 0x%X.\n",
					 cal_status->hdr.errorCode);
		ret = -EFAULT;
		goto end;
	}

	if (cal_status->mcsErr != 0) {
		adi_apollo_hal_log_write(device, ADI_CMS_LOG_ERR, "MCS Init Cal MCS errorCode: 0x%X.\n", cal_status->mcsErr);
		ret = -EFAULT;
		goto end;
	}

end:
	adi_apollo_hal_log_write(device, (ret == 0) ? ADI_CMS_LOG_API : ADI_CMS_LOG_ERR,
				 "MCS Init Cal Validation: %s.\n", (ret == 0) ? "Passed" : "Failed!");

	return ret;
}

int ad9088_mcs_tracking_cal_setup(struct ad9088_phy *phy, u16 mcs_track_decimation,
				  u16 initialize_track_cal)
{
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	/* Set MCS tracking cal decimation for more precise TDC measurements. */
	ret = adi_apollo_mcs_cal_tracking_decimation_set(device, mcs_track_decimation);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_tracking_decimation_set");
	if (ret)
		return ret;

	/* Optional: Calibration values for setting tracking offset between internal
	 * and external SYSREF may be customized to align with hardware setup
	 */
	ret = adi_apollo_mcs_cal_parameter_set(device, MCS_ADF4382_TRACK_TARGET_0_INT32, 0);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_parameter_set");
	if (ret)
		return ret;

	if (device->dev_info.is_dual_clk) {
		ret = adi_apollo_mcs_cal_parameter_set(device, MCS_ADF4382_TRACK_TARGET_1_INT32, 0);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_parameter_set");
		if (ret)
			return ret;
	}

	/* Enable MCS Tracking Cal. Tracking decimation needs to be updated before this API call. */
	ret = adi_apollo_mcs_cal_tracking_enable(device, 1);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_tracking_enable");
	if (ret)
		return ret;

	/* Initialize MCS Tracking Cal if not done by device profile or mcs_cal_config struct
	 * (i.e. cal_config.track_initialize = 1).
	 */
	if (initialize_track_cal) {
		ret = adi_apollo_mcs_cal_tracking_initialize_set(device);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_tracking_initialize_set");
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * ad9088_mcs_tracking_cal_validate - Validate MCS tracking calibration synchronicity
 * @phy: AD9088 PHY structure
 * @buf: Buffer to write validation results
 * @buf_size: Size of the buffer
 *
 * Compares the ADF4382 bleed current values reported by Apollo's MCS tracking
 * calibration with the actual hardware values read from ADF4382. This verifies
 * that the tracking calibration is properly synchronized.
 *
 * Return: Number of bytes written to buf, or negative error code
 */
int ad9088_mcs_tracking_cal_validate(struct ad9088_phy *phy, char *buf, size_t buf_size)
{
	adi_apollo_mcs_cal_status_t tracking_cal_status = {{0}};
	adi_apollo_device_t *device = &phy->ad9088;
	struct device *dev = &phy->spi->dev;
	long long hw_bleed_pol, hw_coarse_current, hw_fine_current;
	u8 fw_bleed_pol, fw_coarse_current;
	s16 fw_fine_current;
	bool need_unfreeze = false;
	int len = 0;
	int ret;

	if (!phy->iio_adf4382) {
		len = scnprintf(buf, buf_size, "ADF4382 IIO channel not available\n");
		return len;
	}

	if (!phy->mcs_cal_bg_tracking_run) {
		len = scnprintf(buf, buf_size, "BG tracking not running\n");
		return len;
	}

	/* Freeze tracking calibration to get consistent readings */
	if (!phy->mcs_cal_bg_tracking_freeze) {
		ret = adi_apollo_mcs_cal_bg_tracking_freeze(device);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_bg_tracking_freeze");
		if (ret)
			return ret;
		need_unfreeze = true;
	}

	/* Get Apollo's tracking calibration status */
	ret = adi_apollo_mcs_cal_tracking_status_get(device, &tracking_cal_status);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_tracking_status_get");
	if (ret)
		goto out_unfreeze;

	/* Read ADF4382 hardware values via IIO ext_info */
	ret = ad9088_iio_read_channel_ext_info(phy, phy->iio_adf4382, "bleed_pol", &hw_bleed_pol);
	if (ret)
		goto out_unfreeze;

	ret = ad9088_iio_read_channel_ext_info(phy, phy->iio_adf4382, "coarse_current", &hw_coarse_current);
	if (ret)
		goto out_unfreeze;

	ret = ad9088_iio_read_channel_ext_info(phy, phy->iio_adf4382, "fine_current", &hw_fine_current);
	if (ret)
		goto out_unfreeze;

	/* Get firmware-reported values */
	fw_bleed_pol = tracking_cal_status.mcs_tracking_cal_status.adf4382_specific_status[0].bleed_pol;
	fw_coarse_current = tracking_cal_status.mcs_tracking_cal_status.adf4382_specific_status[0].current_coarse_value;
	fw_fine_current = tracking_cal_status.mcs_tracking_cal_status.adf4382_specific_status[0].current_fine_value;

	/* Compare and report results */
	len = scnprintf(buf, buf_size,
			"MCS Tracking Cal Validation:\n"
			"  ADF4382 HW:  bleed_pol=%lld coarse=%lld fine=%lld\n"
			"  Apollo FW:   bleed_pol=%u coarse=%u fine=%d\n"
			"  Status:      %s\n",
			hw_bleed_pol, hw_coarse_current, hw_fine_current,
			fw_bleed_pol, fw_coarse_current, fw_fine_current,
			(hw_bleed_pol == fw_bleed_pol &&
			 hw_coarse_current == fw_coarse_current &&
			 hw_fine_current == fw_fine_current) ? "SYNCHRONIZED" : "MISMATCH");

	if (hw_bleed_pol != fw_bleed_pol)
		dev_warn(dev, "MCS Tracking: bleed_pol mismatch HW=%lld FW=%u\n",
			 hw_bleed_pol, fw_bleed_pol);

	if (hw_coarse_current != fw_coarse_current)
		dev_warn(dev, "MCS Tracking: coarse_current mismatch HW=%lld FW=%u\n",
			 hw_coarse_current, fw_coarse_current);

	if (hw_fine_current != fw_fine_current)
		dev_warn(dev, "MCS Tracking: fine_current mismatch HW=%lld FW=%d\n",
			 hw_fine_current, fw_fine_current);

out_unfreeze:
	if (need_unfreeze) {
		int ret2 = adi_apollo_mcs_cal_bg_tracking_unfreeze(device);
		ad9088_check_apollo_error(dev, ret2, "adi_apollo_mcs_cal_bg_tracking_unfreeze");
	}

	return ret < 0 ? ret : len;
}
