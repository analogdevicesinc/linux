// SPDX-License-Identifier: GPL-2.0
/*
 * AD9088 JESD204 FSM support
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include "ad9088.h"

static int ad9088_jesd204_link_init(struct jesd204_dev *jdev,
				    enum jesd204_state_op_reason reason,
				    struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_jesd_tx_cfg_t *jtx;
	adi_apollo_jesd_rx_cfg_t *jrx;
	u8 sideIdx, linkIdx;
	unsigned long lane_rate_kbps;
	int ret;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	switch (lnk->link_id) {
	case DEFRAMER_LINK_A0_TX:
	case DEFRAMER_LINK_A1_TX:
	case DEFRAMER_LINK_B0_TX:
	case DEFRAMER_LINK_B1_TX:
		sideIdx = (lnk->link_id - DEFRAMER_LINK_A0_TX) / 2;
		linkIdx = (lnk->link_id - DEFRAMER_LINK_A0_TX) % 2;

		jrx = &phy->profile.jrx[sideIdx];

		lnk->is_transmit = 1;
		lnk->num_lanes = jrx->rx_link_cfg[linkIdx].l_minus1 + 1;
		lnk->num_converters = jrx->rx_link_cfg[linkIdx].m_minus1 + 1;
		lnk->octets_per_frame = jrx->rx_link_cfg[linkIdx].f_minus1 + 1;
		lnk->frames_per_multiframe = jrx->rx_link_cfg[linkIdx].k_minus1 + 1;
		lnk->num_of_multiblocks_in_emb = jrx->rx_link_cfg[linkIdx].e_minus1 + 1;
		lnk->bits_per_sample = jrx->rx_link_cfg[linkIdx].np_minus1 + 1;
		lnk->converter_resolution = jrx->rx_link_cfg[linkIdx].n_minus1 + 1;
		lnk->jesd_version = jrx->common_link_cfg.ver == ADI_APOLLO_JESD_204C ? JESD204_VERSION_C : JESD204_VERSION_B;
		lnk->subclass = jrx->common_link_cfg.subclass;
		lnk->scrambling = jrx->rx_link_cfg[linkIdx].scr;
		lnk->high_density = jrx->rx_link_cfg[linkIdx].high_dens;
		lnk->ctrl_words_per_frame_clk = 0;
		lnk->ctrl_bits_per_sample = jrx->rx_link_cfg[linkIdx].cs;
		lnk->samples_per_conv_frame = jrx->rx_link_cfg[linkIdx].s_minus1 + 1;

		lnk->sample_rate = phy->profile.dac_config[sideIdx].dac_sampling_rate_Hz;
		lnk->sample_rate_div = jrx->rx_link_cfg[linkIdx].link_total_ratio;
		priv->serdes_jrx_cal_run = false;
		break;
	case FRAMER_LINK_A0_RX:
	case FRAMER_LINK_A1_RX:
	case FRAMER_LINK_B0_RX:
	case FRAMER_LINK_B1_RX:

		sideIdx = (lnk->link_id - FRAMER_LINK_A0_RX) / 2;
		linkIdx = (lnk->link_id - FRAMER_LINK_A0_RX) % 2;

		jtx = &phy->profile.jtx[sideIdx];

		lnk->is_transmit = 0;
		lnk->num_lanes = jtx->tx_link_cfg[linkIdx].l_minus1 + 1;
		lnk->num_converters = jtx->tx_link_cfg[linkIdx].m_minus1 + 1;
		lnk->octets_per_frame = jtx->tx_link_cfg[linkIdx].f_minus1 + 1;
		lnk->frames_per_multiframe = jtx->tx_link_cfg[linkIdx].k_minus1 + 1;
		lnk->num_of_multiblocks_in_emb = jtx->tx_link_cfg[linkIdx].e_minus1 + 1;
		lnk->bits_per_sample = jtx->tx_link_cfg[linkIdx].np_minus1 + 1;
		lnk->converter_resolution = jtx->tx_link_cfg[linkIdx].n_minus1 + 1;
		lnk->jesd_version = jtx->common_link_cfg.ver == ADI_APOLLO_JESD_204C ? JESD204_VERSION_C : JESD204_VERSION_B;
		lnk->subclass = jtx->common_link_cfg.subclass;
		lnk->scrambling = jtx->tx_link_cfg[linkIdx].scr;
		lnk->high_density = jtx->tx_link_cfg[linkIdx].high_dens;
		lnk->ctrl_words_per_frame_clk = 0;
		lnk->ctrl_bits_per_sample = jtx->tx_link_cfg[linkIdx].cs;
		lnk->samples_per_conv_frame = jtx->tx_link_cfg[linkIdx].s_minus1 + 1;

		lnk->sample_rate = phy->profile.adc_config[sideIdx].adc_sampling_rate_Hz;
		lnk->sample_rate_div = jtx->tx_link_cfg[linkIdx].link_total_ratio;
		break;
	default:
		return -EINVAL;
	}

	if (lnk->jesd_version == JESD204_VERSION_C)
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;
	else
		lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	ret = jesd204_link_get_rate_khz(lnk, &lane_rate_kbps);
	if (ret)
		return ret;

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_link_setup(struct jesd204_dev *jdev,
				     enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_device_t *device = &phy->ad9088;
	u32 subclass;
	int ret;

	adi_apollo_rxen_pwrup_ctrl_t rxen_config = {
		.sm_clk_rate = ADI_APOLLO_PUC_CLK_RATE_FS_DIV_32,
		.sm_en = 0,
		.spi_rxen = 1,
		.spi_rxen_en = 1
	};

	adi_apollo_txen_pwrup_ctrl_t txen_config = {
		.sm_clk_rate = ADI_APOLLO_PUC_CLK_RATE_FS_DIV_32,
		.sm_en = 0,
		.spi_txen = 1,
		.spi_txen_en = 1
	};

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	/* Enable Apollo JTx links */
	ret = adi_apollo_jtx_link_enable_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, ADI_APOLLO_ENABLE);
	if (ret) {
		dev_err(dev, "Error enabling JTx links %d\n", ret);
		return ret;
	}
	ret = adi_apollo_jtx_link_enable_set(device, ADI_APOLLO_LINK_A1 | ADI_APOLLO_LINK_B1, ADI_APOLLO_DISABLE);
	if (ret) {
		dev_err(dev, "Error enabling JTx links %d\n", ret);
		return ret;
	}

	/* Enable Apollo JRx links */
	ret = adi_apollo_jrx_link_enable_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, ADI_APOLLO_ENABLE);
	if (ret) {
		dev_err(dev, "Error enabling JRx links %d\n", ret);
		return ret;
	}
	ret = adi_apollo_jrx_link_enable_set(device, ADI_APOLLO_LINK_A1 | ADI_APOLLO_LINK_B1, ADI_APOLLO_DISABLE);
	if (ret) {
		dev_err(dev, "Error enabling JRx links %d\n", ret);
		return ret;
	}

	/* Enable Rx blocks - enable/disable via spi */
	ret = adi_apollo_rxen_pwrup_ctrl_set(device, ADI_APOLLO_RXEN_ADC_ALL, &rxen_config);
	if (ret) {
		dev_err(dev, "Error activating Rx blocks (%d)\n", ret);
		return ret;
	}

	/* Enable Tx blocks - enable/disable via spi */
	ret = adi_apollo_txen_pwrup_ctrl_set(device, ADI_APOLLO_TXEN_DAC_ALL, &txen_config);
	if (ret) {
		dev_err(dev, "Error activating Tx blocks(%d)\n", ret);
		return ret;
	}

	/* Datapath reset */
	adi_apollo_rxmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 1);
	adi_apollo_txmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 1);
	adi_apollo_rxmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 0);
	adi_apollo_txmisc_dp_reset(device, ADI_APOLLO_SIDE_ALL, 0);

	subclass = phy->profile.jtx->common_link_cfg.subclass || phy->profile.jrx->common_link_cfg.subclass;

	ret = adi_apollo_clk_mcs_subclass_set(&phy->ad9088, subclass);
	if (ret) {
		dev_err(dev, "Error setting subclass %d\n", ret);
		return ret;
	}
	ret = adi_apollo_jrx_subclass_set(&phy->ad9088, ADI_APOLLO_LINK_ALL, phy->profile.jrx->common_link_cfg.subclass);
	if (ret) {
		dev_err(dev, "Error setting subclass %d\n", ret);
		return ret;
	}

	ret = adi_apollo_jtx_subclass_set(&phy->ad9088, ADI_APOLLO_LINK_ALL, phy->profile.jtx->common_link_cfg.subclass);
	if (ret) {
		dev_err(dev, "Error setting subclass %d\n", ret);
		return ret;
	}

	ret = adi_apollo_clk_mcs_internal_sysref_per_set(&phy->ad9088,
							 phy->profile.mcs_cfg.internal_sysref_prd_digclk_cycles_center);
	if (ret) {
		dev_err(dev, "Error setting internal sysref period %d\n", ret);
		return ret;
	}

	/* Enable the MCS SYSREF receiver if subclass 1 */
	ret = adi_apollo_clk_mcs_sysref_en_set(&phy->ad9088, (subclass == 1) ? ADI_APOLLO_ENABLE : ADI_APOLLO_DISABLE);
	if (ret) {
		dev_err(dev, "Error setting MCS SYSREF receiver %d\n", ret);
		return ret;
	}

	ret = adi_apollo_adc_bgcal_freeze(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_adc_bgcal_freeze %d\n", ret);
		return ret;
	}

	ret = adi_apollo_clk_mcs_dyn_sync_sequence_run(device);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_clk_mcs_dyn_sync_sequence_run %d\n", ret);
		return ret;
	}
	ret = adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(device);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run %d\n", ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_setup_stage1(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_device_t *device = &phy->ad9088;
	adi_apollo_sysclock_cond_cfg_e cc_cal_cfg;
	int ret;
	u16 jrx_phase_adjust;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	phy->rx_en_mask = ADI_APOLLO_RXEN_ADC_ALL;
	phy->tx_en_mask = ADI_APOLLO_TXEN_DAC_ALL;

	ret = adi_apollo_adc_nyquist_zone_set(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R,
					      phy->rx_nyquist_zone);
	if (ret) {
		dev_err(dev, "Error setting ADC Nyquist zone %d\n", ret);
		return ret;
	}

	if (phy->cal_data_loaded_from_fw) {
		cc_cal_cfg = SYSCLKCONDITIONING_ENABLED_WARMBOOT_FROM_USER;
		dev_info(dev, "Run clock conditioning cal WARMBOOT from USER ...\n");
	} else {
		cc_cal_cfg = SYSCLKCONDITIONING_ENABLED;
		dev_info(dev, "Run clock conditioning cal (can take up to %d secs)...\n", ADI_APOLLO_SYSCLK_COND_CENTER_MAX_TO);
	}

	ret = adi_apollo_cfg_clk_cond_cal_cfg_set(device, cc_cal_cfg);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_cfg_clk_cond_cal_cfg_set %d\n", ret);
		return ret;
	}

	ret = adi_apollo_sysclk_cond_cal(device);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_sysclk_cond_cal");
	if (ret)
			return ret;

	/* Inspect the Apollo JRx and JTx link config */
	ret = ad9088_inspect_jrx_link_all(phy);
	if (ret) {
		dev_err(dev, "Error in ad9088_inspect_jrx_link_all %d\n", ret);
		return ret;
	}
	ret = ad9088_inspect_jtx_link_all(phy);
	if (ret) {
		dev_err(dev, "Error in ad9088_inspect_jtx_link_all %d\n", ret);
		return ret;
	}

	ret = adi_apollo_jrx_phase_adjust_calc(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0,
					       ADI_APOLLO_JRX_PHASE_ADJ_MARGIN_DEFAULT, &jrx_phase_adjust);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_jrx_phase_adjust_calc %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "JRX Phase Adjust: %d\n", jrx_phase_adjust);

	/* Set the jrx phase adjust */
	ret = adi_apollo_jrx_phase_adjust_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, jrx_phase_adjust);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_jrx_phase_adjust_set %d\n", ret);
		return ret;
	}

	/* Set the jtx phase adjust */
	ret = adi_apollo_jtx_phase_adjust_set(device, ADI_APOLLO_LINK_A0 | ADI_APOLLO_LINK_B0, 0);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_jtx_phase_adjust_set %d\n", ret);
		return ret;
	}

	/* ADC calibration */
	if (1) {
		u8 is_adc_nvm_fused;
		adi_apollo_init_cal_cfg_e init_cal_cfg;
		u32 adc_cal_chans = device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R;

		if (!phy->cal_data_loaded_from_fw) {
			ret = adi_apollo_hal_bf_get(device, BF_ADC_NVM_CALDATA_FUSED_INFO, &is_adc_nvm_fused, 1);
			if (ret != API_CMS_ERROR_OK) {
				dev_err(dev, "Error reading ADC NVM fused info %d\n", ret);
				return ret;
			}
			init_cal_cfg = is_adc_nvm_fused ? ADI_APOLLO_INIT_CAL_ENABLED_WARMBOOT_FROM_NVM : ADI_APOLLO_INIT_CAL_ENABLED;
			dev_info(dev, "Run ADC cal from %s (can take up to 100 secs)...\n", is_adc_nvm_fused ? "NVM" : "scratch");
		} else {
			dev_info(dev, "Run ADC CAL WARMBOOT from USER\n");
			init_cal_cfg = ADI_APOLLO_INIT_CAL_DISABLED_WARMBOOT_FROM_USER;
		}

		ret = adi_apollo_adc_init_cal_start(device, adc_cal_chans, init_cal_cfg);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_adc_init_cal_start");
		if (ret)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_setup_stage2(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	if (reason == JESD204_STATE_OP_REASON_INIT) {
		ret = adi_apollo_adc_init_cal_complete(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_adc_init_cal_complete");
		if (ret)
			return ret;

		ret = adi_apollo_adc_nyquist_zone_set(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R,
						phy->rx_nyquist_zone);
		if (ret) {
			dev_err(dev, "Error setting ADC Nyquist zone %d\n", ret);
			return ret;
		}

		ret = adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run(device);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_dyn_sync_rxtxlinks_sequence_run %d\n", ret);
			return ret;
		}

		if (phy->cddc_sample_delay_en) {
			ret = adi_apollo_bmem_cddc_delay_start(device, ADI_APOLLO_BMEM_ALL);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_bmem_cddc_delay_start");
			if (ret)
				return ret;
		}

		if (phy->fddc_sample_delay_en) {
			ret = adi_apollo_bmem_fddc_delay_start(device, ADI_APOLLO_BMEM_ALL);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret, "adi_apollo_bmem_fddc_delay_start");
			if (ret)
				return ret;
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_clks_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	int ret;

	u16 serdes;

	switch (lnk->link_id) {
	case DEFRAMER_LINK_A0_TX:
	case DEFRAMER_LINK_A1_TX:
		serdes = ADI_APOLLO_TXRX_SERDES_12PACK_A;
		break;
	case DEFRAMER_LINK_B0_TX:
	case DEFRAMER_LINK_B1_TX:
		serdes = ADI_APOLLO_TXRX_SERDES_12PACK_B;
		break;
	default:
		serdes = ADI_APOLLO_TXRX_SERDES_12PACK_NONE | ADI_APOLLO_TXRX_SERDES_12PACK_NONE;
		break;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->is_transmit && (reason == JESD204_STATE_OP_REASON_UNINIT) &&
	    phy->profile.jrx[0].common_link_cfg.lane_rate_kHz > 16000000) {
		ret = adi_apollo_serdes_jrx_bgcal_freeze(&phy->ad9088, serdes);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_serdes_jrx_bgcal_freeze");
		if (ret)
			return ret;
		dev_dbg(dev, "%s: SERDES JRx bg cal freeze\n", ad9088_fsm_links_to_str[lnk->link_id]);
	}

	if (lnk->is_transmit && (reason == JESD204_STATE_OP_REASON_INIT) &&
	    phy->profile.jrx[0].common_link_cfg.lane_rate_kHz > 8000000) {
		dev_info(dev, "%s: SERDES JRx cal Rate %u kBps via %s ...\n", ad9088_fsm_links_to_str[lnk->link_id],
			 phy->profile.jrx[0].common_link_cfg.lane_rate_kHz,
			 phy->cal_data_loaded_from_fw ? "WARMBOOT_FROM_USER" : "INIT_CAL");
		ret = adi_apollo_serdes_jrx_init_cal(&phy->ad9088, serdes,
			phy->cal_data_loaded_from_fw ? ADI_APOLLO_INIT_CAL_DISABLED_WARMBOOT_FROM_USER : ADI_APOLLO_INIT_CAL_ENABLED);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_serdes_jrx_init_cal");
		if (ret) {
			return ret;
		}

		if (phy->profile.jrx[0].common_link_cfg.lane_rate_kHz > 16000000) {
			ret = adi_apollo_serdes_jrx_bgcal_unfreeze(&phy->ad9088, serdes);
			ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_serdes_jrx_bgcal_unfreeze");
			if (ret)
				return ret;
			dev_dbg(dev, "%s: SERDES JRx bg cal unfreeze\n", ad9088_fsm_links_to_str[lnk->link_id]);
		}
	}

	if (!lnk->is_transmit) {
		ret = adi_apollo_jtx_link_enable_set(&phy->ad9088, ad9088_to_link(lnk->link_id),
						     reason == JESD204_STATE_OP_REASON_INIT);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_jtx_link_enable_set");
		if (ret)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_link_enable(struct jesd204_dev *jdev,
				      enum jesd204_state_op_reason reason,
				      struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->is_transmit) {
		/* txfe TX (JRX) link */
		ret = adi_apollo_jrx_link_enable_set(&phy->ad9088,
						     ad9088_to_link(lnk->link_id),
						     reason == JESD204_STATE_OP_REASON_INIT);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_jrx_link_enable_set");
		if (ret)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_link_running(struct jesd204_dev *jdev,
				       enum jesd204_state_op_reason reason,
				       struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = false;

		return JESD204_STATE_CHANGE_DONE;
	}

	if (lnk->is_transmit) {
		ad9088_print_link_phase(phy, lnk);
		ret = ad9088_jesd_rx_link_status_print(phy, lnk, 3);
		if (ret < 0)
			return JESD204_STATE_CHANGE_ERROR;
	} else {
		ret = ad9088_jesd_tx_link_status_print(phy, lnk, 3);
		if (ret < 0)
			return JESD204_STATE_CHANGE_ERROR;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_post_setup_stage1(struct jesd204_dev *jdev,
					    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_mcs_cal_init_status_t init_cal_status = {{0}};
	adi_apollo_device_t *device = &phy->ad9088;
	s64 apollo_delta_t0 = 0;
	s64 apollo_delta_t1 = 0;
	s64 adf4030_delta_t0 = 0;
	s64 adf4030_delta_t1 = 0;
	s64 calc_delay = 0;
	u64 bsync_out_period_fs;
	u64 path_delay, round_trip_delay = 0;
	s64 adf4030_phase;
	int val, val2;
	int ret, ret2;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	if (!phy->iio_adf4030 || !phy->iio_adf4382) {
		dev_info(dev, "Skipping MCS calibration\n");
		return JESD204_STATE_CHANGE_DONE;
	}

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		adi_apollo_mcs_cal_bg_tracking_abort(device);
		adi_apollo_mcs_cal_tracking_enable(device, 0);
		return JESD204_STATE_CHANGE_DONE;
	}

	ret = iio_read_channel_attribute(phy->iio_adf4030, &val, &val2, IIO_CHAN_INFO_FREQUENCY);
	if (ret < 0) {
		dev_err(dev, "Failed to read adf4030 frequency\n");
		return ret;
	}

	bsync_out_period_fs = div_u64(1000000000000000ULL, val);

	dev_dbg(dev, "bsync_out_period_fs %lld\n", bsync_out_period_fs);

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 1);
	if (ret < 0) {
		dev_err(dev, "Failed to enable adf4030 output\n");
		return ret;
	}
	ret = iio_write_channel_attribute(phy->iio_adf4030, 0, 0, IIO_CHAN_INFO_PHASE);
	if (ret < 0) {
		dev_err(dev, "Failed to set adf4030 phase\n");
		return ret;
	}

	ret = ad9088_mcs_init_cal_setup(phy);
	if (ret) {
		dev_err(dev, "Failed to setup MCS init cal\n");
		return ret;
	}

	ret = ad9088_delta_t_measurement_set(phy, 0);
	if (ret) {
		dev_err(dev, "Failed to set delta_t measurement 0\n");
		return ret;
	}
	ret = ad9088_delta_t_measurement_get(phy, 0, &apollo_delta_t0);
	if (ret) {
		dev_err(dev, "Failed to get delta_t measurement\n");
		return ret;
	}
	dev_dbg(dev, "apollo_delta_t0 %lld fs\n", apollo_delta_t0);
	ret = iio_read_channel_attribute(phy->iio_adf4030, &val, &val2, IIO_CHAN_INFO_PHASE);
	if (ret < 0) {
		dev_err(dev, "Failed to read adf4030 phase\n");
		return ret;
	}
	adf4030_delta_t0 = (s64)((((u64)val2) << 32) | (u32)val);
	dev_dbg(dev, "adf4030_delta_t0 %lld fs\n", adf4030_delta_t0);

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 0);
	if (ret < 0) {
		dev_err(dev, "Failed to disable adf4030 output\n");
		return ret;
	}
	ret = ad9088_delta_t_measurement_set(phy, 1);
	if (ret) {
		dev_err(dev, "Failed to set delta_t measurement 1\n");

		ret2 = ad9088_delta_t_measurement_set(phy, 2);
		if (ret2) {
			dev_err(dev, "Failed to set delta_t measurement 2\n");
			return ret2;
		}

		ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 1);

		return ret;
	}
	ret = ad9088_delta_t_measurement_get(phy, 1, &apollo_delta_t1);
	if (ret) {
		dev_err(dev, "Failed to get delta_t measurement\n");

		ret2 = ad9088_delta_t_measurement_set(phy, 2);
		if (ret2) {
			dev_err(dev, "Failed to set delta_t measurement 2\n");
			return ret2;
		}

		ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 1);

		return ret;
	}
	dev_dbg(dev, "apollo_delta_t1 %lld fs\n", apollo_delta_t1);

	ret = iio_read_channel_attribute(phy->iio_adf4030, &val, &val2, IIO_CHAN_INFO_PHASE);
	if (ret < 0) {
		dev_err(dev, "Failed to read adf4030 phase\n");

		ret2 = ad9088_delta_t_measurement_set(phy, 2);
		if (ret2) {
			dev_err(dev, "Failed to set delta_t measurement 2\n");
			return ret2;
		}

		ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 1);

		return ret;
	}
	adf4030_delta_t1 = (s64)((((u64)val2) << 32) | (u32)val);
	dev_dbg(dev, "adf4030_delta_t1 %lld fs\n", adf4030_delta_t1);

	ret = ad9088_delta_t_measurement_set(phy, 2);
	if (ret) {
		dev_err(dev, "Failed to set delta_t measurement 2\n");
		return ret;
	}

	calc_delay = (adf4030_delta_t0 - adf4030_delta_t1) - (apollo_delta_t1 - apollo_delta_t0);
	dev_dbg(dev, "calc_delay %lld fs\n", calc_delay);
	div64_u64_rem(calc_delay + bsync_out_period_fs, bsync_out_period_fs, &round_trip_delay);
	dev_dbg(dev, "round_trip_delay %lld fs\n", round_trip_delay);
	path_delay = round_trip_delay >> 1;

	dev_info(dev, "Total BSYNC path delay %lld fs\n", path_delay);

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "output_enable", 1);
	if (ret < 0) {
		dev_err(dev, "Failed to enable adf4030 output\n");
		return ret;
	}

	val = lower_32_bits(-1 * path_delay);
	val2 = upper_32_bits(-1 * path_delay);

	ret = iio_write_channel_attribute(phy->iio_adf4030, val, val2, IIO_CHAN_INFO_PHASE);
	if (ret < 0) {
		dev_err(dev, "Failed to set adf4030 phase\n");
		return ret;
	}

	if (__is_defined(DEBUG)) {
		ret = iio_read_channel_attribute(phy->iio_adf4030, &val, &val2, IIO_CHAN_INFO_PHASE);
		if (ret < 0) {
			dev_err(dev, "Failed to read adf4030 phase\n");
			return ret;
		}
		adf4030_phase = (s64)((((u64)val2) << 32) | (u32)val);
		dev_info(dev, "adf4030_phase %lld fs\n", adf4030_phase);
	}

	ret = adi_apollo_mcs_cal_init_run(device);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_init_run");
	if (ret)
		return ret;

	ret = adi_apollo_mcs_cal_init_status_get(device, &init_cal_status);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_init_status_get");
	if (ret)
		return ret;

	ret = ad9088_mcs_init_cal_validate(phy, &init_cal_status);
	if (ret) {
		dev_err(dev, "MCS Initcal Status: Failed\n");
		return ret;
	}

	dev_info(dev, "MCS Initcal Status: Passed\n");

	ret = ad9088_mcs_tracking_cal_setup(phy, 1023, 1);
	if (ret) {
		dev_err(dev, "Failed to setup MCS tracking cal\n");
		return ret;
	}

	ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4382, "en_auto_align", 1);
	if (ret < 0) {
		dev_err(dev, "Failed to enable adf4382 auto align\n");
		return ret;
	}

	ret = adi_apollo_mcs_cal_fg_tracking_run(device);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_fg_tracking_run");
	if (ret)
		return ret;

	ret = adi_apollo_mcs_cal_bg_tracking_run(device);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_bg_tracking_run");
	if (ret)
		return ret;

	phy->mcs_cal_bg_tracking_run = true;

	if (__is_defined(DEBUG)) {
		ret = adi_apollo_mcs_cal_init_status_get(device, &init_cal_status);
		ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_mcs_cal_init_status_get");
		if (ret)
			return ret;
		ret = ad9088_mcs_init_cal_status_print(phy, phy->dbuf, &init_cal_status);
		if (ret <= 0) {
			dev_err(dev, "Failed to print MCS init cal status\n");
			return ret;
		}

		dev_info(dev, "MCS Initcal Status: %s\n", phy->dbuf);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_post_setup_stage2(struct jesd204_dev *jdev,
					    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		adi_apollo_clk_mcs_trig_sync_enable(device, 0);
		adi_apollo_clk_mcs_trig_reset_disable(device);

		if (!IS_ERR_OR_NULL(phy->iio_adf4030) && jesd204_dev_is_top(jdev) &&
		    phy->aion_background_serial_alignment_en) {
			ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030,
								"background_serial_alignment_en", 0);
			if (ret < 0)
				dev_err(dev, "Failed to disable adf4030 background_serial_alignment_en\n");
		}

		return JESD204_STATE_CHANGE_DONE;
	}

	if (!IS_ERR_OR_NULL(phy->iio_adf4030) && jesd204_dev_is_top(jdev) && phy->aion_background_serial_alignment_en) {
		ret = ad9088_iio_write_channel_ext_info(phy, phy->iio_adf4030, "background_serial_alignment_en", 1);
		if (ret < 0) {
			dev_err(dev, "Failed to enable adf4030 background_serial_alignment_en\n");
			return ret;
		}
	}

	if (phy->trig_sync_en) {
		/* Use Trigger pin A0 to sync Rx and Tx */
		ret = adi_apollo_clk_mcs_sync_trig_map(device, ADI_APOLLO_RX_TX_ALL, ADI_APOLLO_TRIG_PIN_A0);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_sync_trig_map %d\n", ret);
			return ret;
		}

		/*
		 * Resync the Rx and Tx dig only during trig sync
		 */
		ret = adi_apollo_clk_mcs_trig_sync_enable(device, 0);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_sync_enable %d\n", ret);
			return ret;
		}
		ret = adi_apollo_clk_mcs_trig_reset_disable(device);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_reset_disable %d\n", ret);
			return ret;
		}

		/*
		 * Set trig_syn to 1. Apollo will wait for a trigger from the FPGA. When
		 * received, the FSRC will be reset.
		 *
		 * trig_sync is not self-clearing
		 */
		ret = adi_apollo_clk_mcs_trig_sync_enable(device, 0);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_sync_enable %d\n", ret);
			return ret;
		}
		ret = adi_apollo_clk_mcs_trig_reset_dsp_enable(device);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_reset_dsp_enable %d\n", ret);
			return ret;
		}
		ret = adi_apollo_clk_mcs_trig_sync_enable(device, 1);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_sync_enable %d\n", ret);
			return ret;
		}
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_post_setup_stage3(struct jesd204_dev *jdev,
					    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	if (phy->triq_req_gpio && phy->trig_sync_en) {
		gpiod_set_value(phy->triq_req_gpio, 1);
		udelay(1);
		gpiod_set_value(phy->triq_req_gpio, 0);
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_post_setup_stage4(struct jesd204_dev *jdev,
					    enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9088_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9088_phy *phy = priv->phy;
	adi_apollo_device_t *device = &phy->ad9088;
	int ret;
	u32 period_fs;

	if (reason != JESD204_STATE_OP_REASON_INIT) {
		phy->is_initialized = false;
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	if (phy->trig_sync_en) {
		u16 phase, phase1;
		/*
		 * Wait for the trigger sync to finish.
		 */
		ret = adi_apollo_hal_bf_wait_to_set(device, BF_TRIGGER_SYNC_DONE_A0_INFO(MCS_SYNC_MCSTOP0), 1000000,
						    100);   // Will set when Apollo recvs a trigger pulse
		if (ret) {
			dev_err(dev, "Error in adi_apollo_hal_bf_wait_to_set %d\n", ret);
			return ret;
		}
		ret = adi_apollo_clk_mcs_trig_phase_get(&phy->ad9088, ADI_APOLLO_TRIG_PIN_A0, &phase, &phase1);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_phase_get %d\n", ret);
			return ret;
		}

		period_fs = div64_u64(1000000000000000ULL * (phy->profile.clk_cfg.clocking_mode ==
							     ADI_APOLLO_CLOCKING_MODE_SDR_DIV_8 ? 8 : 4),
				       phy->profile.clk_cfg.dev_clk_freq_kHz * 1000ULL);

		dev_info(dev, "Trigger Phase %d (ideal %u) period %u fs\n", phase,
			 phy->profile.mcs_cfg.internal_sysref_prd_digclk_cycles_center / 2, period_fs);

		ret = adi_apollo_clk_mcs_trig_sync_enable(device, 0);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_sync_enable %d\n", ret);
			return ret;
		}
		ret = adi_apollo_clk_mcs_trig_reset_disable(device);
		if (ret) {
			dev_err(dev, "Error in adi_apollo_clk_mcs_trig_reset_disable %d\n", ret);
			return ret;
		}

		if (__is_defined(DEBUG)) {
			/*
			 * Read the trigger sync count. It will increment by two each time the FPGA sysref sequencer is executed
			 */

			u8 sync_input_count[4];

			adi_apollo_hal_bf_get(device, BF_SYNC_INPUT_COUNT_INFO(TXRX_PREFSRC_RECONF_TX_SLICE_0_TX_DIGITAL0),
					      &sync_input_count[0], 1);
			adi_apollo_hal_bf_get(device, BF_SYNC_INPUT_COUNT_INFO(TXRX_PREFSRC_RECONF_TX_SLICE_1_TX_DIGITAL0),
					      &sync_input_count[1], 1);
			adi_apollo_hal_bf_get(device, BF_SYNC_INPUT_COUNT_INFO(TXRX_PREFSRC_RECONF_TX_SLICE_0_TX_DIGITAL1),
					      &sync_input_count[2], 1);
			adi_apollo_hal_bf_get(device, BF_SYNC_INPUT_COUNT_INFO(TXRX_PREFSRC_RECONF_TX_SLICE_1_TX_DIGITAL1),
					      &sync_input_count[3], 1);
			dev_info(&phy->spi->dev, "sync_input_count=%d %d %d %d PHASE %d\n", sync_input_count[0], sync_input_count[1],
				 sync_input_count[2], sync_input_count[3], phase);
		}
	}

	ad9088_print_sysref_phase(phy);

	ret = adi_apollo_adc_bgcal_unfreeze(device, device->dev_info.is_8t8r ? ADI_APOLLO_ADC_ALL : ADI_APOLLO_ADC_ALL_4T4R);
	if (ret) {
		dev_err(dev, "Error in adi_apollo_adc_bgcal_unfreeze %d\n", ret);
		return ret;
	}

	phy->is_initialized = true;

	if (phy->hsci_disable_after_initial_configuration)
		adi_apollo_hal_active_protocol_set(&phy->ad9088, ADI_APOLLO_HAL_PROTOCOL_SPI0);

	return JESD204_STATE_CHANGE_DONE;
}

static int ad9088_jesd204_uninit(struct jesd204_dev *jdev,
			  enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	if (reason != JESD204_STATE_OP_REASON_UNINIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d reason %s\n", __func__, __LINE__, jesd204_state_op_reason_str(reason));

	return JESD204_STATE_CHANGE_DONE;
}

const struct jesd204_dev_data jesd204_ad9088_init = {
	.state_ops = {
		[JESD204_OP_DEVICE_INIT] = {
			.per_device = ad9088_jesd204_uninit,
		},
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9088_jesd204_link_init,
		},
		[JESD204_OP_LINK_SETUP] = {
			.per_device = ad9088_jesd204_link_setup,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_SETUP_STAGE1] = {
			.per_device = ad9088_jesd204_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_SETUP_STAGE2] = {
			.per_device = ad9088_jesd204_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9088_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = ad9088_jesd204_link_enable,
			.post_state_sysref = true,
		},
		[JESD204_OP_LINK_RUNNING] = {
			.per_link = ad9088_jesd204_link_running,
		},
		[JESD204_OP_OPT_POST_SETUP_STAGE1] = {
			.per_device = ad9088_jesd204_post_setup_stage1,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_POST_SETUP_STAGE2] = {
			.per_device = ad9088_jesd204_post_setup_stage2,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_POST_SETUP_STAGE3] = {
			.per_device = ad9088_jesd204_post_setup_stage3,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = ad9088_jesd204_post_setup_stage4,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},

	.max_num_links = 4,
	.num_retries = 0,
	.sizeof_priv = sizeof(struct ad9088_jesd204_priv),
};
