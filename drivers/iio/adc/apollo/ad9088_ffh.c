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

#define UINT32_MAX (0xffffffffU)

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
static int ad9088_read_gpio_hop_array(struct device *dev, const char *propname, int8_t *array, size_t count);

static u32 apollo_cnco_init_profile_load_func(adi_apollo_device_t *device, u32 nco_phase_inc_words[], u32 nco_phase_offset_words[], u32 length);
static u32 apollo_fnco_init_profile_load_func(adi_apollo_device_t *device, u32 nco_phase_inc_words[], u32 nco_phase_offset_words[], u32 length);
static u32 cnco_trig_by_tmaster(struct ad9088_phy *phy, bool enable_flag, u64 trig_per);
static u32 fnco_trig_by_tmaster(struct ad9088_phy *phy, bool enable_flag, u64 trig_per);

int ad9088_ffh_probe(struct ad9088_phy *phy)
{
	adi_apollo_fine_nco_hop_t fnco_hop_config;
	adi_apollo_coarse_nco_hop_t cnco_hop_config;
	struct device *dev = &phy->spi->dev;
	int ret, n_gpios;

	fnco_hop_config.nco_trig_hop_sel = ADI_APOLLO_FNCO_TRIG_HOP_FREQ_PHASE;
	fnco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;
	cnco_hop_config.auto_mode = ADI_APOLLO_NCO_AUTO_HOP_DECR;
	cnco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;

	ret = adi_apollo_fnco_hop_pgm(&phy->ad9088, ADI_APOLLO_RX,
				      ADI_APOLLO_CNCO_ALL, &fnco_hop_config);
	if (ret)
		return ret;
	ret = adi_apollo_fnco_hop_pgm(&phy->ad9088, ADI_APOLLO_TX,
				      ADI_APOLLO_FNCO_ALL, &fnco_hop_config);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_RX,
					 ADI_APOLLO_CNCO_ALL, &cnco_hop_config);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_TX,
					 ADI_APOLLO_CNCO_ALL, &cnco_hop_config);
	if (ret)
		return ret;

	/* Read GPIO hop profile configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-profile",
					     (int8_t *)phy->gpio_hop_profile.index,
					     ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_profile_configure(&phy->ad9088,
							    &phy->gpio_hop_profile);
		if (ret) {
			dev_err(dev, "Failed to configure GPIO hop profile: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Configured %d GPIO hop profile bits\n", n_gpios);
	}

	/* Read GPIO hop block configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-block",
					     (int8_t *)phy->gpio_hop_block.index,
					     ADI_APOLLO_GPIO_HOP_BLOCK_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_block_configure(&phy->ad9088,
							  &phy->gpio_hop_block);
		if (ret) {
			dev_err(dev, "Failed to configure GPIO hop block: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Configured %d GPIO hop block bits\n", n_gpios);
	}

	/* Read GPIO hop side configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-side",
					     (int8_t *)phy->gpio_hop_side.index,
					     ADI_APOLLO_GPIO_HOP_SIDE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_side_configure(&phy->ad9088,
							 &phy->gpio_hop_side);
		if (ret) {
			dev_err(dev, "Failed to configure GPIO hop side: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Configured %d GPIO hop side bits\n", n_gpios);
	}

	/* Read GPIO hop slice configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-slice",
					     (int8_t *)phy->gpio_hop_slice.index,
					     ADI_APOLLO_GPIO_HOP_SLICE_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_slice_configure(&phy->ad9088,
							  &phy->gpio_hop_slice);
		if (ret) {
			dev_err(dev, "Failed to configure GPIO hop slice: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Configured %d GPIO hop slice bits\n", n_gpios);
	}

	/* Read GPIO hop terminal configuration directly into phy structure */
	n_gpios = ad9088_read_gpio_hop_array(dev, "adi,gpio-hop-terminal",
					     (int8_t *)phy->gpio_hop_terminal.index,
					     ADI_APOLLO_GPIO_HOP_TERMINAL_BIT_NUMBER);
	if (n_gpios > 0) {
		ret = adi_apollo_gpio_hop_terminal_configure(&phy->ad9088,
							     &phy->gpio_hop_terminal);
		if (ret) {
			dev_err(dev, "Failed to configure GPIO hop terminal: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Configured %d GPIO hop terminal bits\n", n_gpios);
	}

	/* Cache defaults */
	memset(&phy->ffh, 0, sizeof(union ad9088_ffh));
	for (u8 i = 0; i < ADI_APOLLO_FNCO_PROFILE_NUM; i++) {
		for (u8 j = 0; j < 2; j++)
			phy->ffh.dir[j].fnco.mode[i] = cnco_hop_config.profile_sel_mode;
	}
	for (u8 i = 0; i < ADI_APOLLO_CNCO_PROFILE_NUM; i++) {
		for (u8 j = 0; j < 2; j++)
			phy->ffh.dir[j].cnco.mode[i] = fnco_hop_config.profile_sel_mode;
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
	u64 val;
	u8 index;

	if (!map)
		return -EINVAL;

	guard(mutex)(&phy->lock);

	switch (private) {
	case FFH_FNCO_INDEX:
		val = phy->ffh.dir[dir].fnco.index[map->fddc_num];
		break;
	case FFH_FNCO_FREQUENCY:
		index = phy->ffh.dir[dir].fnco.index[map->fddc_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[dir].fnco.frequency[index];
		break;
	case FFH_FNCO_SELECT:
		val = phy->ffh.dir[dir].fnco.select[map->fddc_num]-1;
		break;
	case FFH_FNCO_MODE:
		val = phy->ffh.dir[dir].fnco.mode[map->fddc_num];
		break;
	case FFH_CNCO_INDEX:
		val = phy->ffh.dir[dir].cnco.index[map->fddc_num];
		break;
	case FFH_CNCO_FREQUENCY:
		index = phy->ffh.dir[dir].cnco.index[map->cddc_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[dir].cnco.frequency[index];
		break;
	case FFH_CNCO_SELECT:
		val = phy->ffh.dir[dir].cnco.select[map->cddc_num];
		break;
	case FFH_CNCO_MODE:
		val = phy->ffh.dir[dir].cnco.mode[map->cddc_num];
				break;
	case FFH_TRIG_PERIOD:
		val = phy->ffh.dir[dir].cnco.trig_period;
		break;
	default:
		return -EINVAL;
	}
	return sprintf(buf, "%lld\n", val);
}

ssize_t ad9088_ext_info_write_ffh(struct iio_dev *indio_dev, uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 dir = chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX;
	struct ad9088_phy *phy = conv->phy;
	const struct ad9088_chan_map *map = ad9088_get_chan_map(phy, chan);
	u8 fnco_num, cnco_num;
	u8 index;
	u32 ftw_u32;
	u32 cddc_dcm;
	u16 fnco_en, cnco_en;
	bool hop_enable;
	u64 val, ret;
	u64 ftw_u64, f, tmp;
	u64 gpio, val2;
	u64 trig_per;
	u32 *cnco_phase_inc_words;
	u32 cnco_length;
	u32 cnco_phase_offset_words[ADI_APOLLO_CNCO_PROFILE_NUM];

	if (!map)
		return -EINVAL;

	guard(mutex)(&phy->lock);

	switch (private) {
	case FFH_FNCO_INDEX:
		if (map->fddc_num > ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_FNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		phy->ffh.dir[dir].fnco.index[map->fddc_num] = val;
		break;
	case FFH_FNCO_FREQUENCY:
		fnco_num = map->fddc_num + map->side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		ret = kstrtou64(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[dir].fnco.index[map->fddc_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		/* Needs to be enabled to apply */
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en, true);
		if (ret)
			return -EFAULT;
		if (chan->output) {
			adi_apollo_cduc_interp_bf_to_val(&phy->ad9088, phy->profile.tx_path[map->side].tx_cduc[map->cddc_num].drc_ratio, &cddc_dcm);
			f = phy->profile.dac_config[map->side].dac_sampling_rate_Hz;
		} else {
			adi_apollo_cddc_dcm_bf_to_val(&phy->ad9088, phy->profile.rx_path[map->side].rx_cddc[map->cddc_num].drc_ratio, &cddc_dcm);
			f = phy->profile.adc_config[map->side].adc_sampling_rate_Hz;
		}
		adi_ad9088_calc_nco_ftw(phy, f, val, cddc_dcm, 32, &ftw_u64, &tmp, &tmp);
		ftw_u32 = ftw_u64;
		ret = adi_apollo_fnco_profile_load(&phy->ad9088, dir, fnco_en,
						   ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT,
						   index, &ftw_u32, 1);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].fnco.frequency[index] = val;
		/* Restore state */
		adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
					   phy->ffh.dir[dir].fnco.en[index]);
		if (ret)
			return -EFAULT;
		break;
	case FFH_FNCO_SELECT:
		fnco_num = map->fddc_num + map->side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		ret = kstrtou64(buf, 10, &val);
		hop_enable = !(val == -1);
		if (ret || val >= ADI_APOLLO_FNCO_PROFILE_NUM || val < -1)
			return -EINVAL;
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
						 hop_enable);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].fnco.en[map->fddc_num] = hop_enable;
		if (!hop_enable)
			break;
		if (phy->ffh.dir[dir].fnco.mode[map->fddc_num] == ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO ||
			phy->ffh.dir[dir].fnco.mode[map->fddc_num] == ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO) {
			ret = adi_apollo_gpio_hop_profile_calc(&phy->ad9088,
								   &phy->gpio_hop_profile,
								   val, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev,
				 "Profile GPIO: mask: %llx value: %llx\n",
				 gpio, val2);
			ret = adi_apollo_gpio_hop_block_calc(&phy->ad9088,
								 &phy->gpio_hop_block,
								 0, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev,
				 "Block GPIO: mask: %llx value: %llx\n",
				 gpio, val2);
		}
		if (phy->ffh.dir[dir].fnco.mode[map->fddc_num] != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    phy->ffh.dir[dir].fnco.mode[map->fddc_num] != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;

		ret = adi_apollo_fnco_active_profile_set(&phy->ad9088, dir, fnco_en, val);
		if (ret)
			return -EFAULT;
		/* Increment by 1 to use 0 to flag disabled */
		phy->ffh.dir[dir].fnco.select[map->fddc_num] = val+1;
		phy->ffh.dir[dir].fnco.en[map->fddc_num] = hop_enable;
		break;
	case FFH_FNCO_MODE:
		fnco_num = map->fddc_num + map->side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		if (map->fddc_num > ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_NCO_CHAN_SEL_LEN || val < 0)
			return -EINVAL;
		ret = adi_apollo_fnco_profile_sel_mode_set(&phy->ad9088, dir, fnco_en, val);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].fnco.mode[map->fddc_num] = val;
		break;
	case FFH_CNCO_INDEX:
		if (map->cddc_num > ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		phy->ffh.dir[dir].cnco.index[map->cddc_num] = val;
		break;
	case FFH_CNCO_FREQUENCY:
		cnco_num = map->cddc_num + map->side*4;
		cnco_en = BIT(cnco_num);
		ret = kstrtou64(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[dir].cnco.index[map->cddc_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		if (chan->output)
			adi_ad9088_calc_nco_ftw(phy, phy->profile.dac_config[map->side].dac_sampling_rate_Hz, val, 1, 32, &ftw_u64, &tmp, &tmp);
		else
			adi_ad9088_calc_nco_ftw(phy, phy->profile.adc_config[map->side].adc_sampling_rate_Hz, val, 1, 32, &ftw_u64, &tmp, &tmp);
		ftw_u32 = ftw_u64;
		phy->ffh.dir[dir].cnco.nco_phase_inc_words[index] = ftw_u32;
		cnco_phase_inc_words = phy->ffh.dir[dir].cnco.nco_phase_inc_words;
		cnco_length = ADI_APOLLO_CNCO_PROFILE_NUM;
		memset(cnco_phase_offset_words, 0, sizeof(cnco_phase_offset_words));

		ret = apollo_cnco_init_profile_load_func(&phy->ad9088, cnco_phase_inc_words, cnco_phase_offset_words, cnco_length);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].cnco.frequency[index] = val;
		break;
	case FFH_CNCO_SELECT:
		cnco_num = map->cddc_num + map->side*4;
		cnco_en = BIT(cnco_num);
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		if (phy->ffh.dir[dir].cnco.mode[map->cddc_num] == ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO ||
			phy->ffh.dir[dir].cnco.mode[map->cddc_num] == ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO) {
			ret = adi_apollo_gpio_hop_profile_calc(&phy->ad9088,
								   &phy->gpio_hop_profile,
								   val, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev,
				 "Profile GPIO: mask: %llx value: %llx\n",
				 gpio, val2);
			ret = adi_apollo_gpio_hop_block_calc(&phy->ad9088,
								 &phy->gpio_hop_block,
								 0, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev,
				 "Block GPIO: mask: %llx value: %llx\n",
				 gpio, val2);
		}
		if (phy->ffh.dir[dir].cnco.mode[map->cddc_num] != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    phy->ffh.dir[dir].cnco.mode[map->cddc_num] != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;
		ret = adi_apollo_cnco_active_profile_set(&phy->ad9088, dir, cnco_en, val);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].cnco.select[map->cddc_num] = val;
		break;
	case FFH_CNCO_MODE:
		cnco_num = map->cddc_num + map->side*4;
		cnco_en = BIT(cnco_num);
		if (map->cddc_num > ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtou64(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_NCO_CHAN_SEL_LEN || val < 0)
			return -EINVAL;

		/* First set the profile selection mode */
		ret = adi_apollo_cnco_profile_sel_mode_set(&phy->ad9088, dir, cnco_en, val);
		if (ret)
			return -EFAULT;

		/* Then configure trigger master if needed */
		if (val == ADI_APOLLO_NCO_CHAN_SEL_TRIG_AUTO) {
			ret = cnco_trig_by_tmaster(phy, true, 0);
		} else {
			ret = cnco_trig_by_tmaster(phy, false, 0);
		}
		if (ret)
			return -EFAULT;

		phy->ffh.dir[dir].cnco.mode[map->cddc_num] = val;
		break;
	case FFH_TRIG_PERIOD:
		ret = kstrtou64(buf, 10, &trig_per);
		if (ret)
			return -EINVAL;

		dev_info(&indio_dev->dev, "Setting trigger period to %llu Fs cycles\n", trig_per);
		ret = cnco_trig_by_tmaster(phy, true, trig_per);
		val = phy->ffh.dir[dir].cnco.trig_period = trig_per;
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return len;
}


static int ad9088_read_gpio_hop_array(struct device *dev, const char *propname,
				      int8_t *array, size_t count)
{
	u32 tmp[ADI_APOLLO_GPIO_HOP_PROFILE_BIT_NUMBER]; /* max number */
	int ret, i;

	/* Initialize all to NONE (-1) */
	for (i = 0; i < count; i++)
		array[i] = ADI_APOLLO_GPIO_HOP_IDX_NONE;

	ret = device_property_count_u32(dev, propname);
	if (ret <= 0)
		return 0;

	/* Read up to count values */
	ret = device_property_read_u32_array(dev, propname, tmp,
					     min(ret, (int)count));
	if (ret < 0)
		return 0;

	/* Copy to output array */
	for (i = 0; i < ret; i++)
		array[i] = (int8_t)tmp[i];

	return ret;
}

static u32 apollo_cnco_init_profile_load_func(adi_apollo_device_t *device, u32 nco_phase_inc_words[], u32 nco_phase_offset_words[], u32 length)
{
    u32 err = API_CMS_ERROR_OK;

    err = adi_apollo_cnco_profile_load(device, ADI_APOLLO_TX, ADI_APOLLO_CNCO_ALL_4T4R,
	    ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT, 0, nco_phase_inc_words, length);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

static u32 apollo_fnco_init_profile_load_func(adi_apollo_device_t *device, u32 nco_phase_inc_words[], u32 nco_phase_offset_words[], u32 length)
{
    u32 err = API_CMS_ERROR_OK;

    err = adi_apollo_fnco_profile_load(device, ADI_APOLLO_TX, ADI_APOLLO_CNCO_ALL_4T4R,
	    ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT, 0, nco_phase_inc_words, length);
    ADI_CMS_ERROR_RETURN(err);

    return err;
}

static u32 cnco_trig_by_tmaster(struct ad9088_phy *phy,
				    bool enable_flag, u64 trig_per)
{
    u32 ret;
    adi_apollo_trig_mst_config_t trig_mst_config = {0};
	adi_apollo_coarse_nco_hop_t nco_hop_config;

    if (!phy)
	return -EINVAL;

    /* Default if userspace writes 0: use 20000000 sample clock cycles */
    if (!trig_per)
	trig_per = 2000;

    /* Route CNCO trigger source to Trigger Master */
    ret = adi_apollo_trigts_cdrc_trig_sel_mux_set(&phy->ad9088, ADI_APOLLO_TX,
						  ADI_APOLLO_CNCO_ALL,
						  ADI_APOLLO_TRIG_MASTER);

    /* Timestamp reset mode: by SPI */
    ret = adi_apollo_trigts_ts_reset_mode_set(&phy->ad9088, ADI_APOLLO_TX,
					      ADI_APOLLO_SIDE_ALL,
					      ADI_APOLLO_TRIG_TS_RESET_MODE_SPI);

    /* Configure Trigger Master */
    trig_mst_config.trig_period = trig_per; /* Units: Fs cycles */
    trig_mst_config.trig_offset = 32;       /* per user guide min for 4T4R */
    trig_mst_config.trig_enable = enable_flag ? ADI_APOLLO_TRIG_ENABLE : ADI_APOLLO_TRIG_DISABLE;

	dev_info(&phy->spi->dev, "trig_period=%llu, trig_offset=%llu, trig_enable=%u\n",
		 trig_mst_config.trig_period,
		 trig_mst_config.trig_offset,
		 trig_mst_config.trig_enable);

    ret = adi_apollo_trigts_cnco_trig_mst_config(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_CNCO_ALL, &trig_mst_config);
    /*
     * Reset timestamp before auto trigger, otherwise the reset could cause a hop
     * from the initial profile.
     */
	ret = adi_apollo_trigts_ts_reset(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_SIDE_ALL, 0);

    /*
    *  Profile selection method (REGMAP or GPIO or AUTO). Will hop to selected profile when triggered.
    */
    nco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_TRIG_AUTO;
    nco_hop_config.auto_mode = ADI_APOLLO_NCO_AUTO_HOP_INCR;    // No actual auto increment.
    nco_hop_config.hop_ctrl_init = 1;
    nco_hop_config.next_hop_number_wr_en = 1;
    adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_CNCO_ALL, &nco_hop_config);

    if (ret)
		return ret;
	return 0;
}

static u32 fnco_trig_by_tmaster(struct ad9088_phy *phy,
				    bool enable_flag, u64 trig_per)
{
    u32 ret;
    adi_apollo_trig_mst_config_t trig_mst_config = {0};

    if (!phy)
	return -EINVAL;

    /* Default if userspace writes 0: use 20000000 sample clock cycles */
    if (!trig_per)
	trig_per = 2000;

    /* Route CNCO trigger source to Trigger Master */
    ret = adi_apollo_trigts_cdrc_trig_sel_mux_set(&phy->ad9088, ADI_APOLLO_TX,
						  ADI_APOLLO_FNCO_ALL,
						  ADI_APOLLO_TRIG_MASTER);

    /* Timestamp reset mode: by SPI */
    ret = adi_apollo_trigts_ts_reset_mode_set(&phy->ad9088, ADI_APOLLO_TX,
					      ADI_APOLLO_SIDE_ALL,
					      ADI_APOLLO_TRIG_TS_RESET_MODE_SPI);

    /* Configure Trigger Master */
    trig_mst_config.trig_period = trig_per; /* Units: Fs cycles */
    trig_mst_config.trig_offset = 32;       /* per user guide min for 4T4R */
    trig_mst_config.trig_enable = enable_flag ? ADI_APOLLO_TRIG_ENABLE : ADI_APOLLO_TRIG_DISABLE;

	dev_info(&phy->spi->dev, "trig_period=%llu, trig_offset=%llu, trig_enable=%u\n",
		 trig_mst_config.trig_period,
		 trig_mst_config.trig_offset,
		 trig_mst_config.trig_enable);

    ret = adi_apollo_trigts_fnco_trig_mst_config(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FNCO_ALL, &trig_mst_config);
    /*
     * Reset timestamp before auto trigger, otherwise the reset could cause a hop
     * from the initial profile.
     */
	ret = adi_apollo_trigts_ts_reset(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_SIDE_ALL, 0);

	adi_apollo_fnco_hop_enable(&phy->ad9088, ADI_APOLLO_TX, ADI_APOLLO_FNCO_ALL, 1);

    if (ret)
		return ret;
	return 0;
}
