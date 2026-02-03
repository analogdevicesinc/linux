// SPDX-License-Identifier: GPL-2.0
/*
 * AD9088 Debugfs Interface
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include "ad9088.h"

enum ad9088_debugfs_cmd {
	DBGFS_NONE,
	DBGFS_BIST_PRBS_JRX,
	DBGFS_BIST_PRBS_JRX_ERR,
	DBGFS_BIST_PRBS_JTX,
	DBGFS_BIST_JRX_2D_EYE,
	DBGFS_DEV_API_INFO,
	DBGFS_DEV_UUID_INFO,
	DBGFS_DEV_DIE_INFO,
	DBGFS_DEV_CHIP_INFO,
	DBGFS_DEV_TEMP_INFO,
	DBGFS_HSCI_ENABLE,
	DBGFS_CLK_PWR_STAT,
	DBGFS_GENERIC,
	DBGFS_JRX_PHASE_ADJUST_CALC,
	DBGFS_JTX_LANE_DRIVE_SWING,
	DBGFS_JTX_LANE_PRE_EMPHASIS,
	DBGFS_JTX_LANE_POST_EMPHASIS,
	/* MCS calibration commands */
	DBGFS_MCS_INIT,
	DBGFS_MCS_DT0_MEASUREMENT,
	DBGFS_MCS_DT1_MEASUREMENT,
	DBGFS_MCS_DT_RESTORE,
	DBGFS_MCS_CAL_RUN,
	DBGFS_MCS_TRACK_CAL_SETUP,
	DBGFS_MCS_FG_TRACK_CAL_RUN,
	DBGFS_MCS_BG_TRACK_CAL_RUN,
	DBGFS_MCS_BG_TRACK_CAL_FREEZE,
	DBGFS_MCS_TRACK_STATUS,
	DBGFS_MCS_INIT_CAL_STATUS,
	DBGFS_MCS_TRACK_CAL_VALIDATE,
};

static const u8 lanes_all[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
	12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23
};

static u16 ad9088_jrx_serdes_en_mask(struct ad9088_phy *phy)
{
	u16 serdes = ADI_APOLLO_TXRX_SERDES_12PACK_NONE;

	if (phy->profile.jrx[0].common_link_cfg.lane_enables)
		serdes |= ADI_APOLLO_TXRX_SERDES_12PACK_A;
	if (phy->profile.jrx[1].common_link_cfg.lane_enables)
		serdes |= ADI_APOLLO_TXRX_SERDES_12PACK_B;

	return serdes;
}

static int ad9088_dbg(struct ad9088_phy *phy, int val, int val2, int val3, int val4)
{
#ifdef DEBUG
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	dev_info(&phy->spi->dev, " MCS - %d 0x%X\n", val, val2);
	switch (val) {
	case 1:
		adi_apollo_jrx_phase_adjust_set(device, val3 ? val3 : ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, val2);
		break;
	case 2:
		adi_apollo_jtx_phase_adjust_set(device, val3 ? val3 : ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, val2);
		break;
	case 3:
		adi_apollo_jtx_link_enable_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, val2);
		adi_apollo_jrx_link_enable_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, val2);
		break;
	case 4:
		/* Datapath reset */
		ret = adi_apollo_serdes_jrx_cal(&phy->ad9088);
		ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_cal");
		break;
	case 5:
		adi_apollo_txmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 1);
		adi_apollo_txmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 0);
		break;
	case 6:
		adi_apollo_clk_mcs_dyn_sync_sequence_run(device);
		break;
	case 7:
		adi_apollo_clk_mcs_oneshot_sync(device);
		break;
	case 8:
		adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(device);
		break;
	case 9:
		ad9088_print_sysref_phase(phy);
		break;
	case 10:
		adi_apollo_adc_bgcal_unfreeze(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R);
		break;
	case 11:
		adi_apollo_adc_bgcal_freeze(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R);
		break;
	default:
		return -EINVAL;
	}

	return 0;
#else
	return -ENOTSUPP;
#endif
}

static ssize_t ad9088_debugfs_read(struct file *file, char __user *userbuf,
				   size_t count, loff_t *ppos)
{
	struct ad9088_debugfs_entry *entry = file->private_data;
	struct iio_dev *indio_dev = entry->indio_dev;
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	adi_apollo_device_tmu_data_t tmu_data;
	adi_apollo_serdes_prbs_checker_status_t prbs_stat[24];
	adi_apollo_hal_protocol_e protocol;
	adi_apollo_clk_input_power_status_e pwr_stat_a, pwr_stat_b;
	adi_apollo_serdes_jrx_horiz_eye_resp_t horz_resp;
	adi_apollo_serdes_jrx_vert_eye_resp_t vert_resp;
	u64 val = 0;
	ssize_t len = 0;
	int ret, i, lane, prbs, duration;
	u8 uuid[ADI_APOLLO_UUID_NUM_BYTES];
	u16 api_rev[3];
	u8 die_id;

	if (*ppos)
		return 0;

	guard(mutex)(&phy->lock);

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			val = *(u8 *)entry->out_value;
			break;
		case 2:
			val = *(u16 *)entry->out_value;
			break;
		case 4:
			val = *(u32 *)entry->out_value;
			break;
		case 5:
			val = *(bool *)entry->out_value;
			break;
		case 8:
			val = *(u64 *)entry->out_value;
			break;
		default:
			ret = -EINVAL;
		}

	} else if (entry->cmd) {
		switch (entry->cmd) {
		case DBGFS_BIST_PRBS_JRX_ERR:
			ret = adi_apollo_serdes_jrx_prbs_checker_status(&phy->ad9088, phy->jrx_lanes, prbs_stat, phy->jrx_lanes_used);
			if (ret) {
				dev_err(&phy->spi->dev, "adi_apollo_serdes_jrx_prbs_checker_status() failed (%d)", ret);
				break;
			}

			for (i = 0; i < phy->jrx_lanes_used; i++)
				len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len, "%c: lane-%u %u/%u\n",
						phy->jrx_lanes[i] < 12 ? 'A' : 'B',
						phy->jrx_lanes[i] > 11 ? phy->jrx_lanes[i] - 12 : phy->jrx_lanes[i],
						prbs_stat[phy->jrx_lanes[i]].err_count, prbs_stat[phy->jrx_lanes[i]].err_sticky);
			break;
		case DBGFS_BIST_JRX_2D_EYE:
			if (!entry->val)
				return -EINVAL;

			lane = (entry->val & 0xFF) - 1;
			prbs = (entry->val >> 8) & 0xFF;
			duration = (entry->val >> 16) & 0xFFFF;

			entry->val = 0;

			if (phy->profile.jrx[0].common_link_cfg.lane_rate_kHz > 16000000) {
				ret = adi_apollo_serdes_jrx_bgcal_freeze(&phy->ad9088, ad9088_jrx_serdes_en_mask(phy));
				ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_bgcal_freeze");
				if (ret)
					return ret;
			}

			ret = adi_apollo_serdes_jrx_horiz_eye_sweep(&phy->ad9088, lane, prbs);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_horiz_eye_sweep");
			if (ret)
				return ret;

			msleep(duration); /* FIXME: Don't think this does anything */

			ret = adi_apollo_serdes_jrx_horiz_eye_sweep_resp_get(&phy->ad9088, lane, &horz_resp);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_horiz_eye_sweep_resp_get");
			if (ret)
				return ret;

			ret = adi_apollo_serdes_jrx_vert_eye_sweep(&phy->ad9088, lane);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_vert_eye_sweep");
			if (ret)
				return ret;

			msleep(duration); /* FIXME: Don't think this does anything */

			ret = adi_apollo_serdes_jrx_vert_eye_sweep_resp_get(&phy->ad9088, lane, &vert_resp);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_vert_eye_sweep_resp_get");
			if (ret)
				return ret;

			if (phy->profile.jrx[0].common_link_cfg.lane_rate_kHz > 16000000) {
				ret = adi_apollo_serdes_jrx_bgcal_unfreeze(&phy->ad9088, ad9088_jrx_serdes_en_mask(phy));
				ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_serdes_jrx_bgcal_unfreeze");
				if (ret)
					return ret;
			}

			len = snprintf(phy->dbuf, sizeof(phy->dbuf),
				       "# lane %u spo_steps %u rate %u spo_left %u spo_right %u version %u\n",
				       lane, ADI_APOLLO_SERDES_JRX_VERT_EYE_TEST_RESP_BUF_SIZE / 2,
				       phy->profile.jrx[lane >=
							     ADI_APOLLO_JESD_MAX_LANES_PER_SIDE].common_link_cfg.lane_rate_kHz,
				       horz_resp.spo_left, horz_resp.spo_right, horz_resp.ver);

			for (i = 0; i < ADI_APOLLO_SERDES_JRX_VERT_EYE_TEST_RESP_BUF_SIZE; i += 2)
				if (!(vert_resp.eye_heights_at_spo[i] == 127 && vert_resp.eye_heights_at_spo[i + 1] == -127))
					len += snprintf(phy->dbuf + len,
							sizeof(phy->dbuf) - len,
							"%d,%d,%d\n", (i / 2) - 16,
							vert_resp.eye_heights_at_spo[i] * 4,
							vert_resp.eye_heights_at_spo[i + 1] * 4);

			break;
		case DBGFS_DEV_TEMP_INFO:
			if (!phy->is_initialized)
				return -EBUSY;
			ret = adi_apollo_device_tmu_get(&phy->ad9088, &tmu_data);
			if (ret)
				break;

			len = snprintf(phy->dbuf, sizeof(phy->dbuf),
				       "TMU (deg C): serdes_pll=%d mpu_a=%d mpu_b=%d adc_a=%d clk_a=%d adc_b=%d clk_b=%d clk_c=%d (avg: %d, avg mask: 0x%04x)\n",
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_SERDES_PLL],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_MPU_A],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_MPU_B],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_ADC_A],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_CLK_A],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_ADC_B],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_CLK_B],
				       tmu_data.temp_degrees_celsius[ADI_APOLLO_DEVICE_TMU_CLK_C],
				       tmu_data.temp_degrees_celsius_avg,
				       tmu_data.avg_mask);
			break;
		case DBGFS_DEV_API_INFO:
			adi_apollo_device_api_revision_get(&phy->ad9088,
							   &api_rev[0], &api_rev[1], &api_rev[2]);

			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%u.%u.%u\n",
				       api_rev[0], api_rev[1], api_rev[2]);
			break;
		case DBGFS_DEV_UUID_INFO:
			ret = adi_apollo_device_uuid_get(&phy->ad9088, uuid, ADI_APOLLO_UUID_NUM_BYTES);
			if (ret)
				break;

			len = 0;
			for (i = 0; i < ADI_APOLLO_UUID_NUM_BYTES; i++)
				len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len, "%02x", uuid[i]);

			len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len, "\n");
			break;
		case DBGFS_DEV_DIE_INFO:
			ret = adi_apollo_device_die_id_get(&phy->ad9088, &die_id);
			if (ret < 0) {
				dev_err(&phy->spi->dev, "die_id failed (%d)\n", ret);
				break;
			}

			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "DieID %u\n",
				       die_id);
			break;
		case DBGFS_DEV_CHIP_INFO:
			ret = adi_apollo_device_chip_id_get(&phy->ad9088, &phy->chip_id);
			if (ret < 0) {
				dev_err(&phy->spi->dev, "chip_id failed (%d)\n", ret);
				break;
			}

			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "AD%X Rev. %u Grade %u\n",
				       phy->chip_id.prod_id, phy->chip_id.dev_revision, phy->chip_id.prod_grade);
			break;
		case DBGFS_CLK_PWR_STAT:
			ret = adi_apollo_clk_mcs_input_power_status_get(&phy->ad9088, &pwr_stat_a, &pwr_stat_b);
			if (ret)
				break;
			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "Clock input power detection A: %s\n",
				       !pwr_stat_a ? "GOOD" : (pwr_stat_a == ADI_APOLLO_CLK_PWR_UNDERDRIVEN ? "UNDERDRIVEN" :
							       (pwr_stat_a == ADI_APOLLO_CLK_PWR_OVERDRIVEN ? "OVERDRIVEN" : "UNUSED")));
			len += snprintf(phy->dbuf + len, sizeof(phy->dbuf) - len, "Clock input power detection B: %s\n",
					!pwr_stat_b ? "GOOD" : (pwr_stat_b == ADI_APOLLO_CLK_PWR_UNDERDRIVEN ? "UNDERDRIVEN" :
								(pwr_stat_b == ADI_APOLLO_CLK_PWR_OVERDRIVEN ? "OVERDRIVEN" : "UNUSED")));
			break;
		case DBGFS_HSCI_ENABLE:
			if (phy->hsci) {
				adi_apollo_hal_active_protocol_get(&phy->ad9088, &protocol);
				val = (protocol == ADI_APOLLO_HAL_PROTOCOL_HSCI);
			} else {
				val = 0;
			}
			break;
		case DBGFS_MCS_DT0_MEASUREMENT:
			ret = ad9088_delta_t_measurement_get(phy, 0, &entry->delta_t);
			if (!ret)
				len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%lld\n", entry->delta_t);
			break;
		case DBGFS_MCS_DT1_MEASUREMENT:
			ret = ad9088_delta_t_measurement_get(phy, 1, &entry->delta_t);
			if (!ret)
				len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%lld\n", entry->delta_t);
			break;
		case DBGFS_MCS_BG_TRACK_CAL_RUN:
			val = phy->mcs_cal_bg_tracking_run;
			break;
		case DBGFS_MCS_BG_TRACK_CAL_FREEZE:
			val = phy->mcs_cal_bg_tracking_freeze;
			break;
		case DBGFS_MCS_CAL_RUN: {
			adi_apollo_mcs_cal_init_status_t init_cal_status = {{0}};

			ret = adi_apollo_mcs_cal_init_status_get(&phy->ad9088, &init_cal_status);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_init_status_get");
			if (ret)
				break;
			ret = ad9088_mcs_init_cal_validate(phy, &init_cal_status);
			len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%s\n", ret ? "Failed" : "Passed");
			ret = 0; /* Don't propagate validation failure as read error */
			break;
		}
		case DBGFS_MCS_INIT_CAL_STATUS: {
			adi_apollo_mcs_cal_init_status_t init_cal_status = {{0}};

			ret = adi_apollo_mcs_cal_init_status_get(&phy->ad9088, &init_cal_status);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_init_status_get");
			if (ret)
				break;
			len = ad9088_mcs_init_cal_status_print(phy, phy->dbuf, &init_cal_status);
			break;
		}
		case DBGFS_MCS_TRACK_STATUS: {
			adi_apollo_mcs_cal_status_t tracking_cal_status = {{0}};

			if (!phy->mcs_cal_bg_tracking_run) {
				len = snprintf(phy->dbuf, sizeof(phy->dbuf), "BG tracking not running\n");
				break;
			}

			if (!phy->mcs_cal_bg_tracking_freeze) {
				ret = adi_apollo_mcs_cal_bg_tracking_freeze(&phy->ad9088);
				ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_freeze");
				if (ret)
					break;
			}

			ret = adi_apollo_mcs_cal_tracking_status_get(&phy->ad9088, &tracking_cal_status);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_tracking_status_get");
			if (ret)
				break;

			len = ad9088_mcs_track_cal_status_print(phy, phy->dbuf, &tracking_cal_status, 1);

			if (!phy->mcs_cal_bg_tracking_freeze) {
				ret = adi_apollo_mcs_cal_bg_tracking_unfreeze(&phy->ad9088);
				ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_unfreeze");
			}
			break;
		}
		case DBGFS_MCS_TRACK_CAL_VALIDATE:
			ret = ad9088_mcs_tracking_cal_validate(phy, phy->dbuf, sizeof(phy->dbuf));
			if (ret > 0)
				len = ret;
			break;
		case DBGFS_MCS_INIT:
		case DBGFS_MCS_DT_RESTORE:
		case DBGFS_MCS_TRACK_CAL_SETUP:
		case DBGFS_MCS_FG_TRACK_CAL_RUN:
			/* Write-only attributes, return 0 on read */
			val = 0;
			break;
		default:
			val = entry->val;
		}
	} else {
		return -EFAULT;
	}
	if (!len)
		len = snprintf(phy->dbuf, sizeof(phy->dbuf), "%llu\n", val);

	return simple_read_from_buffer(userbuf, count, ppos, phy->dbuf, len);
}

static int ad9088_val_to_jtx_prbs(int val)
{
	switch (val) {
	case 0:
		return 0;
	case 7:
		return ADI_APOLLO_SERDES_JTX_PRBS7;
	case 9:
		return ADI_APOLLO_SERDES_JTX_PRBS9;
	case 15:
		return ADI_APOLLO_SERDES_JTX_PRBS15;
	case 31:
		return ADI_APOLLO_SERDES_JTX_PRBS31;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static int ad9088_val_to_jrx_prbs(int val)
{
	switch (val) {
	case 0:
		return 0;
	case 7:
		return ADI_APOLLO_SERDES_JRX_PRBS7;
	case 9:
		return ADI_APOLLO_SERDES_JRX_PRBS9;
	case 15:
		return ADI_APOLLO_SERDES_JRX_PRBS15;
	case 31:
		return ADI_APOLLO_SERDES_JRX_PRBS31;
	default:
		return -EINVAL;
	}
	return -EINVAL;
}

static ssize_t ad9088_debugfs_write(struct file *file,
				    const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct ad9088_debugfs_entry *entry = file->private_data;
	struct iio_dev *indio_dev = entry->indio_dev;
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	adi_apollo_serdes_prbs_generator_enable_t prbs_gen;
	adi_apollo_serdes_prbs_checker_enable_t prbs_chk;
	int val2, val3, val4, ret, side, lane;
	s64 val;
	u16 phase;
	char buf[80];

	guard(mutex)(&phy->lock);

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%lli %i %i %i", &val, &val2, &val3, &val4);
	if (ret < 1)
		return -EINVAL;

	switch (entry->cmd) {
	case DBGFS_BIST_PRBS_JRX:
		if (ret < 1)
			return -EINVAL;

		prbs_chk.enable = !!val;
		prbs_chk.auto_mode = 0;
		prbs_chk.auto_mode_thres = 4; /* 2^n */
		prbs_chk.prbs_mode = ad9088_val_to_jrx_prbs(val);

		ret = adi_apollo_jrx_link_enable_set(&phy->ad9088, ADI_APOLLO_LINK_A1 | ADI_APOLLO_LINK_B1, ADI_APOLLO_DISABLE);
		if (ret) {
			dev_err(&phy->spi->dev, "Error enabling JRx links %d\n", ret);
			return ret;
		}

		adi_apollo_serdes_jrx_prbs_checker_enable(&phy->ad9088, phy->jrx_lanes, phy->jrx_lanes_used, &prbs_chk);

		/* Enable Apollo JRx links */
		ret = adi_apollo_jrx_link_enable_set(&phy->ad9088, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, ADI_APOLLO_ENABLE);
		if (ret) {
			dev_err(&phy->spi->dev, "Error enabling JRx links %d\n", ret);
			return ret;
		}

		if (prbs_chk.enable)
			adi_apollo_serdes_jrx_prbs_clear_error(&phy->ad9088, phy->jrx_lanes, phy->jrx_lanes_used);

		entry->val = val;

		return count;
	case DBGFS_BIST_PRBS_JTX:
		if (ret < 1)
			return -EINVAL;

		prbs_gen.enable = !!val;
		prbs_gen.mode = ad9088_val_to_jtx_prbs(val);

		adi_apollo_serdes_prbs_generator_enable(&phy->ad9088, (u8 *)lanes_all, ARRAY_SIZE(lanes_all), &prbs_gen);
		entry->val = val;

		return count;
	case DBGFS_HSCI_ENABLE:
		if (ret < 1)
			return -EINVAL;

		if (phy->hsci) {
			ret = adi_apollo_hal_active_protocol_set(&phy->ad9088,
								 val == 1 ? ADI_APOLLO_HAL_PROTOCOL_HSCI :
								 ADI_APOLLO_HAL_PROTOCOL_SPI0);
		} else {
			return -ENODEV;
		}
		break;
	case DBGFS_GENERIC:
		if (ret < 1)
			return -EINVAL;

		if (__is_defined(DEBUG))
			ret = ad9088_dbg(phy, val, val2, val3, val4);
		break;
	case DBGFS_BIST_JRX_2D_EYE:
		if (ret < 1)
			return -EINVAL;

		if (ret < 2)
			val2 = 7; /* PRBS7 */

		if (ret < 3)
			val3 = 10; /* 10 ms */

		if (val > 23)
			return -EINVAL;

		lane = val;

		if (lane >= ADI_APOLLO_JESD_MAX_LANES_PER_SIDE) {
			side = 1;
			lane -= ADI_APOLLO_JESD_MAX_LANES_PER_SIDE;
		} else {
			side = 0;
		}

		if (!(phy->profile.jrx[side].common_link_cfg.lane_enables & (1 << lane)))
			return -EINVAL;

		val = val + 1;

		ret = ad9088_val_to_jrx_prbs(val2);
		if (ret < 0)
			return ret;

		val2 = ret;
		/*           Time          PRBS                 Lane */
		entry->val = val3 << 16 | (val2 & 0xFF) << 8 | (val & 0xFF);

		return count;
	case DBGFS_JRX_PHASE_ADJUST_CALC:
		ret  = adi_apollo_jrx_phase_adjust_calc(&phy->ad9088, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0,
							ADI_APOLLO_JRX_PHASE_ADJ_MARGIN_DEFAULT, &phase);
		if (ret != API_CMS_ERROR_OK) {
			dev_err(&phy->spi->dev, "Error from adi_apollo_jrx_phase_adjust_calc() %d\n", ret);
			return ret;
		}
		entry->val = phase;
		return count;
	case DBGFS_JTX_LANE_DRIVE_SWING:
		if (ret < 3) {
			dev_err(&phy->spi->dev, "Attribute requires 3 arguments <link_side_mask> <lane> <swing>\n");
			return -EINVAL;
		}

		ret = adi_apollo_jtx_lane_drive_swing_set(&phy->ad9088, val, val2, val3);
		if (ret < 0)
			return ret;

		dev_info(&phy->spi->dev, "JTX Lane Drive Swing Set sides: 0x%X lane: %d swing: %d (%d)\n", (u32)val, val2, val3, ret);
		break;
	case DBGFS_JTX_LANE_PRE_EMPHASIS:
		if (ret < 3) {
			dev_err(&phy->spi->dev, "Attribute requires 3 arguments <link_side_mask> <lane> <emphasis>\n");
			return -EINVAL;
		}

		ret = adi_apollo_jtx_lane_pre_emphasis_set(&phy->ad9088, val, val2, val3);
		if (ret < 0)
			return ret;

		dev_info(&phy->spi->dev, "JTX Lane Pre Emphasis Set sides: 0x%X lane: %d emphasis: %d (%d)\n", (u32)val, val2, val3,
			 ret);
		break;
	case DBGFS_JTX_LANE_POST_EMPHASIS:
		if (ret < 3) {
			dev_err(&phy->spi->dev, "Attribute requires 3 arguments <link_side_mask> <lane> <emphasis>\n");
			return -EINVAL;
		}

		ret = adi_apollo_jtx_lane_post_emphasis_set(&phy->ad9088, val, val2, val3);
		if (ret < 0)
			return ret;

		dev_info(&phy->spi->dev, "JTX Lane Post Emphasis Set sides: 0x%X lane: %d emphasis: %d (%d)\n", (u32)val, val2, val3,
			 ret);
		break;
	/* MCS calibration commands */
	case DBGFS_MCS_INIT:
		if (val)
			ret = ad9088_mcs_init_cal_setup(phy);
		break;
	case DBGFS_MCS_DT0_MEASUREMENT:
		if (val)
			ret = ad9088_delta_t_measurement_set(phy, 0);
		break;
	case DBGFS_MCS_DT1_MEASUREMENT:
		if (val)
			ret = ad9088_delta_t_measurement_set(phy, 1);
		break;
	case DBGFS_MCS_DT_RESTORE:
		if (val)
			ret = ad9088_delta_t_measurement_set(phy, 2);
		break;
	case DBGFS_MCS_CAL_RUN:
		if (val) {
			ret = adi_apollo_mcs_cal_init_run(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_init_run");
		}
		break;
	case DBGFS_MCS_TRACK_CAL_SETUP:
		if (val)
			ret = ad9088_mcs_tracking_cal_setup(phy, phy->mcs_track_decimation, 1);
		break;
	case DBGFS_MCS_FG_TRACK_CAL_RUN:
		if (val) {
			ret = adi_apollo_mcs_cal_fg_tracking_run(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_fg_tracking_run");
		}
		break;
	case DBGFS_MCS_BG_TRACK_CAL_RUN:
		if (val) {
			ret = adi_apollo_mcs_cal_bg_tracking_run(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_run");
		} else {
			ret = adi_apollo_mcs_cal_bg_tracking_abort(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_abort");
		}
		if (!ret)
			phy->mcs_cal_bg_tracking_run = !!val;
		break;
	case DBGFS_MCS_BG_TRACK_CAL_FREEZE:
		if (!phy->mcs_cal_bg_tracking_run) {
			dev_err(&phy->spi->dev, "MCS BG Tracking Cal not running.\n");
			return -EFAULT;
		}
		if (val) {
			ret = adi_apollo_mcs_cal_bg_tracking_freeze(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_freeze");
		} else {
			ret = adi_apollo_mcs_cal_bg_tracking_unfreeze(&phy->ad9088);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_mcs_cal_bg_tracking_unfreeze");
		}
		if (!ret)
			phy->mcs_cal_bg_tracking_freeze = !!val;
		break;
	case DBGFS_MCS_TRACK_STATUS:
	case DBGFS_MCS_INIT_CAL_STATUS:
		/* Read-only attributes */
		return -EINVAL;
	default:
		break;
	}

	if (entry->out_value) {
		switch (entry->size) {
		case 1:
			*(u8 *)entry->out_value = val;
			break;
		case 2:
			*(u16 *)entry->out_value = val;
			break;
		case 4:
			*(u32 *)entry->out_value = val;
			break;
		case 5:
			*(bool *)entry->out_value = val;
			break;
		case 8:
			*(u64 *)entry->out_value = val;
			break;
		default:
			ret = -EINVAL;
		}
	}

	return count;
}

static const struct file_operations ad9088_debugfs_reg_fops = {
	.open = simple_open,
	.read = ad9088_debugfs_read,
	.write = ad9088_debugfs_write,
};

static void ad9088_add_debugfs_entry(struct ad9088_phy *phy,
				     struct iio_dev *indio_dev,
				     const char *propname, unsigned int cmd)
{
	unsigned int i = phy->ad9088_debugfs_entry_index;

	if (WARN_ON(i >= ARRAY_SIZE(phy->debugfs_entry)))
		return;

	phy->debugfs_entry[i].indio_dev = indio_dev;
	phy->debugfs_entry[i].propname = propname;
	phy->debugfs_entry[i].cmd = cmd;

	phy->ad9088_debugfs_entry_index++;
}

int ad9088_debugfs_register(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	int i;

	if (!iio_get_debugfs_dentry(indio_dev))
		return 0;

	debugfs_create_devm_seqfile(&conv->spi->dev, "status",
				    iio_get_debugfs_dentry(indio_dev),
				    ad9088_status_show);

	ad9088_add_debugfs_entry(phy, indio_dev,
				 "bist_prbs_select_jrx", DBGFS_BIST_PRBS_JRX);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "bist_prbs_select_jtx", DBGFS_BIST_PRBS_JTX);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "bist_prbs_error_counters_jrx", DBGFS_BIST_PRBS_JRX_ERR);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "bist_2d_eyescan_jrx", DBGFS_BIST_JRX_2D_EYE);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "api_version", DBGFS_DEV_API_INFO);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "uuid", DBGFS_DEV_UUID_INFO);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "die_id", DBGFS_DEV_DIE_INFO);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "chip_version", DBGFS_DEV_CHIP_INFO);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "clk_pwr_stat", DBGFS_CLK_PWR_STAT);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "temperature_status", DBGFS_DEV_TEMP_INFO);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "hsci_enable", DBGFS_HSCI_ENABLE);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "misc", DBGFS_GENERIC);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "jrx_phase_adjust_calc", DBGFS_JRX_PHASE_ADJUST_CALC);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "jtx_lane_drive_swing", DBGFS_JTX_LANE_DRIVE_SWING);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "jtx_lane_pre_emphasis", DBGFS_JTX_LANE_PRE_EMPHASIS);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "jtx_lane_post_emphasis", DBGFS_JTX_LANE_POST_EMPHASIS);

	/* MCS calibration entries */
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_init", DBGFS_MCS_INIT);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_dt0_measurement", DBGFS_MCS_DT0_MEASUREMENT);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_dt1_measurement", DBGFS_MCS_DT1_MEASUREMENT);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_dt_restore", DBGFS_MCS_DT_RESTORE);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_cal_run", DBGFS_MCS_CAL_RUN);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_track_cal_setup", DBGFS_MCS_TRACK_CAL_SETUP);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_fg_track_cal_run", DBGFS_MCS_FG_TRACK_CAL_RUN);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_bg_track_cal_run", DBGFS_MCS_BG_TRACK_CAL_RUN);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_bg_track_cal_freeze", DBGFS_MCS_BG_TRACK_CAL_FREEZE);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_track_status", DBGFS_MCS_TRACK_STATUS);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_init_cal_status", DBGFS_MCS_INIT_CAL_STATUS);
	ad9088_add_debugfs_entry(phy, indio_dev,
				 "mcs_track_cal_validate", DBGFS_MCS_TRACK_CAL_VALIDATE);

	for (i = 0; i < phy->ad9088_debugfs_entry_index; i++)
		debugfs_create_file(phy->debugfs_entry[i].propname, 0644,
				    iio_get_debugfs_dentry(indio_dev),
				    &phy->debugfs_entry[i],
				    &ad9088_debugfs_reg_fops);

	return 0;
}
