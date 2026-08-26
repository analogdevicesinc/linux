// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9088 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/types.h>
#include <linux/kstrtox.h>
#include <linux/property.h>
#include "ad9088.h"

static bool ad9088_ffh_mode_is_gpio(u8 mode)
{
	return mode == ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO ||
	       mode == ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO;
}

/* fddc/cddc numbers are per side, the NCO masks and the shadow state are not */
static u8 ad9088_ffh_fnco_num(const struct ad9088_chan_map *map)
{
	return map->fddc_num + map->side * ADI_APOLLO_FNCO_PER_SIDE_NUM;
}

static u8 ad9088_ffh_cnco_num(const struct ad9088_chan_map *map)
{
	return map->cddc_num + map->side * ADI_APOLLO_CNCO_PER_SIDE_NUM;
}

/* Retuns true if any FNCO/CNCO is in GPIO mode. */
static bool ad9088_ffh_gpio_active(struct ad9088_phy *phy, bool fnco, u8 terminal,
				   u8 idx, u8 new_mode)
{
	const struct _ad9088_ffh *ffh;
	u8 t, i;

	for (t = 0; t < ARRAY_SIZE(phy->ffh.dir); t++) {
		ffh = &phy->ffh.dir[t];

		for (i = 0; i < ADI_APOLLO_FNCO_NUM; i++)
			if (ad9088_ffh_mode_is_gpio(fnco && t == terminal && i == idx ?
						    new_mode : ffh->fnco.mode[i]))
				return true;

		for (i = 0; i < ADI_APOLLO_CNCO_NUM; i++)
			if (ad9088_ffh_mode_is_gpio(!fnco && t == terminal && i == idx ?
						    new_mode : ffh->cnco.mode[i]))
				return true;
	}

	return false;
}

static int ad9088_ffh_gpio_hop_enter(struct ad9088_phy *phy, bool fnco)
{
	int ret;

	ret = adi_apollo_gpio_hop_slice_select_set(&phy->ad9088,
						   fnco ? ADI_APOLLO_GPIO_BLOCK_FNCO :
							  ADI_APOLLO_GPIO_BLOCK_CNCO,
						   ADI_APOLLO_GPIO_HOP_SELECT_GPIO);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
				       "adi_apollo_gpio_hop_slice_select_set");
	if (ret)
		return ret;

	ret = adi_apollo_gpio_hop_block_select_set(&phy->ad9088,
						   ADI_APOLLO_GPIO_HOP_SELECT_GPIO);
	return ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_gpio_hop_block_select_set");
}

static int ad9088_ffh_gpio_hop_exit(struct ad9088_phy *phy, bool fnco, u8 terminal,
				    u8 idx, u8 new_mode)
{
	int ret;

	/*
	 * The block select is global, only revert to SPI if no reamining FNCO/CNCO
	 * is in GPIO mode.
	 */
	if (ad9088_ffh_gpio_active(phy, fnco, terminal, idx, new_mode))
		return 0;

	ret = adi_apollo_gpio_hop_slice_select_set(&phy->ad9088,
						   ADI_APOLLO_GPIO_BLOCK_FNCO,
						   ADI_APOLLO_GPIO_HOP_SELECT_SPI);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
				       "adi_apollo_gpio_hop_slice_select_set");
	if (ret)
		return ret;

	ret = adi_apollo_gpio_hop_slice_select_set(&phy->ad9088,
						   ADI_APOLLO_GPIO_BLOCK_CNCO,
						   ADI_APOLLO_GPIO_HOP_SELECT_SPI);
	ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
				       "adi_apollo_gpio_hop_slice_select_set");
	if (ret)
		return ret;

	ret = adi_apollo_gpio_hop_block_select_set(&phy->ad9088,
						   ADI_APOLLO_GPIO_HOP_SELECT_SPI);
	return ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_gpio_hop_block_select_set");
}

/**
 * ad9088_read_gpio_hop_array - Read GPIO hop array from device tree
 * @dev: Device pointer
 * @propname: Device tree property name
 * @array: Output array to fill
 * @count: Maximum number of elements to read
 *
 * Reads GPIO indices from device tree and fills the array. Missing values
 * are filled with ADI_APOLLO_GPIO_HOP_IDX_NONE (-1).
 *
 * Return: Number of GPIOs read, or 0 if property not found
 */
static int ad9088_read_gpio_hop_array(struct device *dev, const char *propname,
				      s8 *array, size_t count)
{
	u32 tmp[ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER]; /* max number */
	int ret, len, i;

	/* Initialize all to NONE (-1) */
	for (i = 0; i < count; i++)
		array[i] = ADI_APOLLO_GPIO_HOP_IDX_NONE;

	len = device_property_count_u32(dev, propname);
	if (len <= 0)
		return 0;

	/* Read up to count values */
	len = min(len, count);
	ret = device_property_read_u32_array(dev, propname, tmp, len);
	if (ret < 0)
		return 0;

	/* Copy to output array */
	for (i = 0; i < len; i++)
		array[i] = tmp[i];

	return len;
}

static void ad9088_ffh_gpio_hop_log(struct device *dev, bool reapply,
				    const char *what, int n_gpios)
{
	if (reapply)
		dev_dbg(dev, "Re-applied %d GPIO hop %s bits after MCS\n", n_gpios, what);
	else
		dev_info(dev, "Configured %d GPIO hop %s bits\n", n_gpios, what);
}

/**
 * ad9088_ffh_gpio_hop_pins_configure - Route the FFH hop word onto GPIO pins
 * @phy: Device handle
 * @reapply: Set when re-running after the initial probe
 *
 * Applies the adi,gpio-hop-{profile,block,side,slice,terminal} devicetree
 *
 * Has precedence over the MCS/BSYNC calibration (will undo MCS ADF4382 DELADJ/DELSTR
 * routes).
 *
 * Return: 0 on success, negative error code otherwise
 */
int ad9088_ffh_gpio_hop_pins_configure(struct ad9088_phy *phy, bool reapply)
{
	struct device *dev = &phy->spi->dev;
	int ret, n_gpios;

	/* Read GPIO hop profile configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-profile",
					     (s8 *)phy->gpio_hop_profile.index,
					     ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_profile_configure(&phy->ad9088,
							    &phy->gpio_hop_profile);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_hop_profile_configure");
		if (ret)
			return ret;

		ad9088_ffh_gpio_hop_log(dev, reapply, "profile", n_gpios);
	}

	/* Read GPIO hop block configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-block",
					     (s8 *)phy->gpio_hop_block.index,
					     ADI_APOLLO_GPIO_HOP_BLOCK_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_block_configure(&phy->ad9088,
							  &phy->gpio_hop_block);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_hop_block_configure");
		if (ret)
			return ret;

		ad9088_ffh_gpio_hop_log(dev, reapply, "block", n_gpios);
	}

	/* Read GPIO hop side configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-side",
					     (s8 *)phy->gpio_hop_side.index,
					     ADI_APOLLO_GPIO_HOP_SIDE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_side_configure(&phy->ad9088,
							 &phy->gpio_hop_side);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_hop_side_configure");
		if (ret)
			return ret;

		ad9088_ffh_gpio_hop_log(dev, reapply, "side", n_gpios);
	}

	/* Read GPIO hop slice configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-slice",
					     (s8 *)phy->gpio_hop_slice.index,
					     ADI_APOLLO_GPIO_HOP_SLICE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_slice_configure(&phy->ad9088,
							  &phy->gpio_hop_slice);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_hop_slice_configure");
		if (ret)
			return ret;

		ad9088_ffh_gpio_hop_log(dev, reapply, "slice", n_gpios);
	}

	/* Read GPIO hop terminal configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-terminal",
					     (s8 *)phy->gpio_hop_terminal.index,
					     ADI_APOLLO_GPIO_HOP_TERMINAL_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_terminal_configure(&phy->ad9088,
							     &phy->gpio_hop_terminal);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_hop_terminal_configure");
		if (ret)
			return ret;

		ad9088_ffh_gpio_hop_log(dev, reapply, "terminal", n_gpios);
	}

	return 0;
}

int ad9088_ffh_probe(struct ad9088_phy *phy)
{
	adi_apollo_fine_nco_hop_t fnco_hop_config = { };
	adi_apollo_coarse_nco_hop_t cnco_hop_config = { };
	struct device *dev = &phy->spi->dev;
	u32 quick_cfg;
	int ret;

	/*
	 * Program the full hop parameter set once for every controller, so that a
	 * runtime mode change only has to touch the profile select mode itself.
	 */
	fnco_hop_config.nco_trig_hop_sel = ADI_APOLLO_FNCO_TRIG_HOP_FREQ;
	fnco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;
	fnco_hop_config.phase_inc_auto_mode = ADI_APOLLO_NCO_AUTO_HOP_INCR;
	fnco_hop_config.phase_offset_auto_mode = ADI_APOLLO_NCO_AUTO_HOP_INCR;
	fnco_hop_config.phase_inc_high_limit = ADI_APOLLO_FNCO_PROFILE_NUM - 1;
	fnco_hop_config.phase_offset_high_limit = ADI_APOLLO_FNCO_PROFILE_NUM - 1;

	cnco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;
	cnco_hop_config.auto_mode = ADI_APOLLO_NCO_AUTO_HOP_INCR;
	cnco_hop_config.high_limit = ADI_APOLLO_CNCO_PROFILE_NUM - 1;
	cnco_hop_config.hop_ctrl_init = 1;

	ret = adi_apollo_fnco_hop_pgm(&phy->ad9088, ADI_APOLLO_RX,
				      ADI_APOLLO_FNCO_ALL, &fnco_hop_config);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_fnco_hop_pgm");
	if (ret)
		return ret;

	ret = adi_apollo_fnco_hop_pgm(&phy->ad9088, ADI_APOLLO_TX,
				      ADI_APOLLO_FNCO_ALL, &fnco_hop_config);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_fnco_hop_pgm");
	if (ret)
		return ret;

	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_RX,
					 ADI_APOLLO_CNCO_ALL, &cnco_hop_config);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_cnco_hop_enable");
	if (ret)
		return ret;

	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_TX,
					 ADI_APOLLO_CNCO_ALL, &cnco_hop_config);
	ret = ad9088_check_apollo_error(dev, ret, "adi_apollo_cnco_hop_enable");
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "adi,gpio-quick-config", &quick_cfg);
	if (!ret) {
		if (quick_cfg > ADI_APOLLO_QUICK_CFG_PROFILE_8)
			return dev_err_probe(dev, -EINVAL,
					     "Invalid GPIO quick config profile %u\n", quick_cfg);

		ret = adi_apollo_gpio_quick_config_mode_set(&phy->ad9088, quick_cfg);
		ret = ad9088_check_apollo_error(dev, ret,
					       "adi_apollo_gpio_quick_config_mode_set");
		if (ret)
			return ret;

		dev_dbg(dev, "Applied GPIO quick config profile %u\n", quick_cfg);
	}

	ret = ad9088_ffh_gpio_hop_pins_configure(phy, false);
	if (ret)
		return ret;

	/* Cache defaults */
	memset(&phy->ffh, 0, sizeof(union ad9088_ffh));
	for (u8 j = 0; j < ARRAY_SIZE(phy->ffh.dir); j++) {
		for (u8 i = 0; i < ADI_APOLLO_FNCO_NUM; i++)
			phy->ffh.dir[j].fnco.mode[i] = fnco_hop_config.profile_sel_mode;

		for (u8 i = 0; i < ADI_APOLLO_CNCO_NUM; i++)
			phy->ffh.dir[j].cnco.mode[i] = cnco_hop_config.profile_sel_mode;
	}

	return ret;
}

ssize_t ad9088_ext_info_read_ffh(struct iio_dev *indio_dev, uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 dir = chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX;
	struct ad9088_phy *phy = conv->phy;
	const struct ad9088_chan_map *map = ad9088_get_chan_map(phy, chan);
	u8 fnco_num, cnco_num;
	u8 index;

	if (!map)
		return -EINVAL;

	fnco_num = ad9088_ffh_fnco_num(map);
	cnco_num = ad9088_ffh_cnco_num(map);
	if (fnco_num >= ADI_APOLLO_FNCO_NUM || cnco_num >= ADI_APOLLO_CNCO_NUM)
		return -EINVAL;

	guard(mutex)(&phy->lock);

	switch (private) {
	case FFH_FNCO_INDEX:
		return sysfs_emit(buf, "%u\n", phy->ffh.dir[dir].fnco.index[fnco_num]);
	case FFH_FNCO_FREQUENCY:
		index = phy->ffh.dir[dir].fnco.index[fnco_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		return sysfs_emit(buf, "%llu\n",
				  phy->ffh.dir[dir].fnco.frequency[fnco_num][index]);
	case FFH_FNCO_SELECT:
		/* Stored incremented by 1 so that 0 flags "not selected" */
		return sysfs_emit(buf, "%d\n",
				  (int)phy->ffh.dir[dir].fnco.select[fnco_num] - 1);
	case FFH_FNCO_MODE:
		return sysfs_emit(buf, "%u\n", phy->ffh.dir[dir].fnco.mode[fnco_num]);
	case FFH_CNCO_INDEX:
		return sysfs_emit(buf, "%u\n", phy->ffh.dir[dir].cnco.index[cnco_num]);
	case FFH_CNCO_FREQUENCY:
		index = phy->ffh.dir[dir].cnco.index[cnco_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		return sysfs_emit(buf, "%llu\n",
				  phy->ffh.dir[dir].cnco.frequency[cnco_num][index]);
	case FFH_CNCO_SELECT:
		return sysfs_emit(buf, "%u\n", phy->ffh.dir[dir].cnco.select[cnco_num]);
	case FFH_CNCO_MODE:
		return sysfs_emit(buf, "%u\n", phy->ffh.dir[dir].cnco.mode[cnco_num]);
	default:
		return -EINVAL;
	}
}

ssize_t ad9088_ext_info_write_ffh(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 dir = chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX;
	struct ad9088_phy *phy = conv->phy;
	const struct ad9088_chan_map *map = ad9088_get_chan_map(phy, chan);
	adi_apollo_fine_nco_chan_pgm_t fnco_chan_config = { };
	adi_apollo_cduc_ratio_e tx_ratio;
	adi_apollo_cddc_ratio_e rx_ratio;
	u8 fnco_num, cnco_num;
	u8 index;
	u32 ftw_u32;
	u32 cddc_dcm;
	u16 fnco_en, cnco_en;
	bool hop_enable, gpio_mode, gpio_mode_old;
	u64 val;
	u64 ftw_u64, f, tmp;
	int ret;
	s64 sel;
	u8 mode;

	if (!map)
		return -EINVAL;

	fnco_num = ad9088_ffh_fnco_num(map);
	cnco_num = ad9088_ffh_cnco_num(map);
	if (fnco_num >= ADI_APOLLO_FNCO_NUM || cnco_num >= ADI_APOLLO_CNCO_NUM)
		return -EINVAL;

	fnco_en = BIT(fnco_num);
	cnco_en = BIT(cnco_num);

	guard(mutex)(&phy->lock);

	switch (private) {
	case FFH_FNCO_INDEX:
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;

		phy->ffh.dir[dir].fnco.index[fnco_num] = val;
		return len;
	case FFH_FNCO_FREQUENCY:
		ret = kstrtou64(buf, 10, &val);
		if (ret)
			return -EINVAL;

		index = phy->ffh.dir[dir].fnco.index[fnco_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;

		/* Needs to be enabled to apply */
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en, true);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_hop_enable");
		if (ret)
			return ret;

		if (chan->output) {
			tx_ratio = phy->profile.tx_path[map->side].tx_cduc[map->cddc_pi].drc_ratio;
			ret = adi_apollo_cduc_interp_bf_to_val(&phy->ad9088, tx_ratio, &cddc_dcm);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						       "adi_apollo_cduc_interp_bf_to_val");
			if (ret)
				return ret;

			f = phy->profile.dac_cfg[map->side].dac_sampling_rate_Hz;
		} else {
			rx_ratio = phy->profile.rx_path[map->side].rx_cddc[map->cddc_pi].drc_ratio;
			ret = adi_apollo_cddc_dcm_bf_to_val(&phy->ad9088, rx_ratio, &cddc_dcm);
			ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
						       "adi_apollo_cddc_dcm_bf_to_val");
			if (ret)
				return ret;

			f = phy->profile.adc_cfg[map->side].adc_sampling_rate_Hz;
		}

		adi_ad9088_calc_nco_ftw(phy, f, val, cddc_dcm, 32, &ftw_u64, &tmp, &tmp);
		ftw_u32 = ftw_u64;
		fnco_chan_config.drc_phase_inc = ftw_u32;

		ret = adi_apollo_fnco_chan_pgm(&phy->ad9088, dir, fnco_en,
					       index, &fnco_chan_config);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_chan_pgm");
		if (ret)
			return ret;

		phy->ffh.dir[dir].fnco.frequency[fnco_num][index] = val;

		/* Restore state */
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
						 phy->ffh.dir[dir].fnco.en[fnco_num]);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_hop_enable");
		if (ret)
			return ret;
		return len;
	case FFH_FNCO_SELECT:
		ret = kstrtos64(buf, 10, &sel);
		if (ret || sel < -1 || sel >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;

		hop_enable = !(sel == -1);
		val = sel;

		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
						 hop_enable);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_hop_enable");
		if (ret)
			return ret;

		phy->ffh.dir[dir].fnco.en[fnco_num] = hop_enable;
		if (!hop_enable)
			return len;

		mode = phy->ffh.dir[dir].fnco.mode[fnco_num];
		if (mode != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    mode != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;

		ret = adi_apollo_fnco_active_profile_set(&phy->ad9088, dir, fnco_en, val);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_active_profile_set");
		if (ret)
			return ret;

		/* Increment by 1 to use 0 to flag disabled */
		phy->ffh.dir[dir].fnco.select[fnco_num] = val + 1;
		return len;
	case FFH_FNCO_MODE:
		ret = kstrtou64(buf, 10, &val);
		if (ret || val > ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP)
			return -EINVAL;

		mode = phy->ffh.dir[dir].fnco.mode[fnco_num];
		gpio_mode = ad9088_ffh_mode_is_gpio(val);
		gpio_mode_old = ad9088_ffh_mode_is_gpio(mode);

		/*
		 * adi_apollo_fnco_hop_pgm() pulses HOP_CTRL_INIT,
		 * discarting hop profiles, use adi_apollo_fnco_profile_sel_mode_set()
		 * to preserve between mode changes.
		 */
		ret = adi_apollo_fnco_profile_sel_mode_set(&phy->ad9088, dir, fnco_en, val);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_profile_sel_mode_set");
		if (ret)
			return ret;

		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en, true);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_fnco_hop_enable");
		if (ret)
			return ret;

		phy->ffh.dir[dir].fnco.en[fnco_num] = true;

		if (gpio_mode)
			ret = ad9088_ffh_gpio_hop_enter(phy, true);
		else if (gpio_mode_old)
			ret = ad9088_ffh_gpio_hop_exit(phy, true, dir, fnco_num, val);
		if (ret)
			return ret;

		phy->ffh.dir[dir].fnco.mode[fnco_num] = val;
		return len;
	case FFH_CNCO_INDEX:
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;

		phy->ffh.dir[dir].cnco.index[cnco_num] = val;
		return len;
	case FFH_CNCO_FREQUENCY:
		ret = kstrtou64(buf, 10, &val);
		if (ret)
			return -EINVAL;

		index = phy->ffh.dir[dir].cnco.index[cnco_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;

		f = chan->output ? phy->profile.dac_cfg[map->side].dac_sampling_rate_Hz :
				   phy->profile.adc_cfg[map->side].adc_sampling_rate_Hz;
		adi_ad9088_calc_nco_ftw(phy, f, val, 1, 32, &ftw_u64, &tmp, &tmp);
		ftw_u32 = ftw_u64;

		ret = adi_apollo_cnco_profile_load(&phy->ad9088, dir, cnco_en,
						   ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT,
						   index, &ftw_u32, 1);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_cnco_profile_load");
		if (ret)
			return ret;

		phy->ffh.dir[dir].cnco.frequency[cnco_num][index] = val;
		return len;
	case FFH_CNCO_SELECT:
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;

		mode = phy->ffh.dir[dir].cnco.mode[cnco_num];
		if (mode != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    mode != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;

		ret = adi_apollo_cnco_active_profile_set(&phy->ad9088, dir, cnco_en, val);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_cnco_active_profile_set");
		if (ret)
			return ret;

		phy->ffh.dir[dir].cnco.select[cnco_num] = val;
		return len;
	case FFH_CNCO_MODE:
		ret = kstrtou64(buf, 10, &val);
		if (ret || val > ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP)
			return -EINVAL;

		mode = phy->ffh.dir[dir].cnco.mode[cnco_num];
		gpio_mode = ad9088_ffh_mode_is_gpio(val);
		gpio_mode_old = ad9088_ffh_mode_is_gpio(mode);

		/* adi_apollo_cnco_hop_enable() sets HOP_CTRL_INIT */
		ret = adi_apollo_cnco_profile_sel_mode_set(&phy->ad9088, dir, cnco_en, val);
		ret = ad9088_check_apollo_error(&phy->spi->dev, ret,
					       "adi_apollo_cnco_profile_sel_mode_set");
		if (ret)
			return ret;

		if (gpio_mode)
			ret = ad9088_ffh_gpio_hop_enter(phy, false);
		else if (gpio_mode_old)
			ret = ad9088_ffh_gpio_hop_exit(phy, false, dir, cnco_num, val);
		if (ret)
			return ret;

		phy->ffh.dir[dir].cnco.mode[cnco_num] = val;
		return len;
	default:
		return -EINVAL;
	}
}
