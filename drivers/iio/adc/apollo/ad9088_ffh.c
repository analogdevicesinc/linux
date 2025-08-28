#include <linux/types.h>
#include <linux/kstrtox.h>

#include "ad9088.h"

adi_apollo_gpio_hop_profile_t hop_config = {{19, 20, 21, 22, 23}};
adi_apollo_gpio_hop_block_t block_config = {{15, 16, 17, 18}};

int ad9088_ffh_probe(struct ad9088_phy *phy)
{
	adi_apollo_fine_nco_hop_t fnco_hop_config;
	adi_apollo_coarse_nco_hop_t cnco_hop_config;
	int ret;

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

	ret = adi_apollo_gpio_hop_profile_configure(&phy->ad9088, &hop_config);
	if (ret)
		return -EFAULT;
	ret = adi_apollo_gpio_hop_block_configure(&phy->ad9088, &block_config);
	if (ret)
		return -EFAULT;

	/* Cache defaults */
	memset(&phy->ffh, 0, sizeof(union ad9088_ffh));
	for (u8 i = 0; i < ADI_APOLLO_FNCO_PROFILE_NUM; i++) {
		for (u8 j = 0; j < 2; j++) {
			phy->ffh.dir[j].fnco.mode[i] = cnco_hop_config.profile_sel_mode;
		}
	}
	for (u8 i = 0; i < ADI_APOLLO_CNCO_PROFILE_NUM; i++) {
		for (u8 j = 0; j < 2; j++) {
			phy->ffh.dir[j].cnco.mode[i] = fnco_hop_config.profile_sel_mode;
		}
	}
	return ret;
}

ssize_t ad9088_ext_info_read_ffh(struct iio_dev *indio_dev, uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	u8 dir = chan->output ? ADI_APOLLO_TX : ADI_APOLLO_RX;
	struct ad9088_phy *phy = conv->phy;
	u8 cddc_num, fddc_num, side;
	u32 cddc_mask, fddc_mask;
	long long val;

	guard(mutex)(&phy->lock);
	ad9088_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				    &fddc_mask, &cddc_num, &cddc_mask, &side);

	switch (private) {
	case FFH_FNCO_INDEX:
		val = phy->ffh.dir[dir].fnco.index[fddc_num];
		break;
	case FFH_FNCO_FREQUENCY:
		val = phy->ffh.dir[dir].fnco.frequency[fddc_num];
		if (val >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[dir].fnco.frequency[val];
		break;
	case FFH_FNCO_SELECT:
		val = phy->ffh.dir[dir].fnco.select[fddc_num]-1;
		break;
	case FFH_FNCO_MODE:
		val = phy->ffh.dir[dir].fnco.mode[fddc_num];
		break;
	case FFH_CNCO_INDEX:
		val = phy->ffh.dir[dir].cnco.index[fddc_num];
		break;
	case FFH_CNCO_FREQUENCY:
		val = phy->ffh.dir[dir].cnco.frequency[cddc_num];
		if (val >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[dir].cnco.frequency[val];
		break;
	case FFH_CNCO_SELECT:
		val = phy->ffh.dir[dir].cnco.select[cddc_num];
		break;
	case FFH_CNCO_MODE:
		val = phy->ffh.dir[dir].cnco.mode[cddc_num];
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
	u8 cddc_num, fddc_num, side, fnco_num, cnco_num, index;
	struct ad9088_phy *phy = conv->phy;
	u32 cddc_mask, fddc_mask, ftw_u32;
	u32 cddc_dcm;
	u16 fnco_en, cnco_en;
	bool hop_enable;
	int val, ret;
	u64 ftw_u64, f, tmp;
	u64 gpio, val2;

	guard(mutex)(&phy->lock);
	ad9088_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				    &fddc_mask, &cddc_num, &cddc_mask, &side);

	switch (private) {
	case FFH_FNCO_INDEX:
		if (fddc_num > ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_FNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		phy->ffh.dir[dir].fnco.index[fddc_num] = val;
		break;
	case FFH_FNCO_FREQUENCY:
		fnco_num = fddc_num + side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		ret = kstrtoint(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[dir].fnco.index[fddc_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		/* Needs to be enabled to apply */
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en, true);
		if (ret)
			return -EFAULT;
		if (chan->output) {
			adi_apollo_cduc_interp_bf_to_val(&phy->ad9088, phy->profile.tx_path[side].tx_cduc[cddc_num].drc_ratio, &cddc_dcm);
			f = phy->profile.dac_config[side].dac_sampling_rate_Hz;
		} else {
			adi_apollo_cddc_dcm_bf_to_val(&phy->ad9088, phy->profile.rx_path[side].rx_cddc[cddc_num].drc_ratio, &cddc_dcm);
			f = phy->profile.adc_config[side].adc_sampling_rate_Hz;
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
		fnco_num = fddc_num + side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		ret = kstrtoint(buf, 10, &val);
		hop_enable = !(val == -1);
		if (ret || val >= ADI_APOLLO_FNCO_PROFILE_NUM || val < -1)
			return -EINVAL;
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
						 hop_enable);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].fnco.en[fddc_num] = hop_enable;
		if (!hop_enable)
			break;
		if (phy->ffh.dir[dir].fnco.mode[fddc_num] == ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO ||
		    phy->ffh.dir[dir].fnco.mode[fddc_num] == ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO) {
			ret = adi_apollo_gpio_hop_profile_calc(&phy->ad9088, &hop_config, val, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev, "Profile GPIO: mask: %llx value: %llx\n", gpio, val2);
			ret = adi_apollo_gpio_hop_block_calc(&phy->ad9088, &block_config, 0, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev, "Block GPIO: mask: %llx value: %llx\n", gpio, val2);
		}
		if (phy->ffh.dir[dir].fnco.mode[fddc_num] != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    phy->ffh.dir[dir].fnco.mode[fddc_num] != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;

		ret = adi_apollo_fnco_active_profile_set(&phy->ad9088, dir, fnco_en, val);
		if (ret)
			return -EFAULT;
		/* Increment by 1 to use 0 to flag disabled */
		phy->ffh.dir[dir].fnco.select[fddc_num] = val+1;
		phy->ffh.dir[dir].fnco.en[fddc_num] = hop_enable;
		break;
	case FFH_FNCO_MODE:
		fnco_num = fddc_num + side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		if (fddc_num > ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_NCO_CHAN_SEL_LEN || val < 0)
			return -EINVAL;
		ret = adi_apollo_fnco_profile_sel_mode_set(&phy->ad9088, dir, fnco_en, val);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].fnco.mode[fddc_num] = val;
		break;
	case FFH_CNCO_INDEX:
		if (cddc_num > ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		phy->ffh.dir[dir].cnco.index[cddc_num] = val;
		break;
	case FFH_CNCO_FREQUENCY:
		cnco_num = cddc_num + side*4;
		cnco_en = BIT(cnco_num);
		ret = kstrtoint(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[dir].cnco.index[cddc_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		if (chan->output)
			adi_ad9088_calc_nco_ftw(phy, phy->profile.dac_config[side].dac_sampling_rate_Hz, val, 1, 32, &ftw_u64, &tmp, &tmp);
		else
			adi_ad9088_calc_nco_ftw(phy, phy->profile.adc_config[side].adc_sampling_rate_Hz, val, 1, 32, &ftw_u64, &tmp, &tmp);
		ftw_u32 = ftw_u64;
		ret = adi_apollo_cnco_profile_load(&phy->ad9088, dir, cnco_en,
						   ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT,
						   index, &ftw_u32, 1);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].cnco.frequency[index] = val;
		break;
	case FFH_CNCO_SELECT:
		cnco_num = cddc_num + side*4;
		cnco_en = BIT(cnco_num);
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		if (phy->ffh.dir[dir].cnco.mode[cddc_num] == ADI_APOLLO_NCO_CHAN_SEL_TRIG_GPIO ||
		    phy->ffh.dir[dir].cnco.mode[cddc_num] == ADI_APOLLO_NCO_CHAN_SEL_DIRECT_GPIO) {
			ret = adi_apollo_gpio_hop_profile_calc(&phy->ad9088, &hop_config, val, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev, "Profile GPIO: mask: %llx value: %llx\n", gpio, val2);
			ret = adi_apollo_gpio_hop_block_calc(&phy->ad9088, &block_config, 0, &gpio, &val2);
			if (ret)
				return ret;
			dev_info(&conv->spi->dev, "Block GPIO: mask: %llx value: %llx\n", gpio, val2);
		}
		if (phy->ffh.dir[dir].cnco.mode[cddc_num] != ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP &&
		    phy->ffh.dir[dir].cnco.mode[cddc_num] != ADI_APOLLO_NCO_CHAN_SEL_TRIG_REGMAP)
			return -EINVAL;
		ret = adi_apollo_cnco_active_profile_set(&phy->ad9088, dir, cnco_en, val);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].cnco.select[cddc_num] = val;
		break;
	case FFH_CNCO_MODE:
		cnco_num = cddc_num + side*4;
		cnco_en = BIT(cnco_num);
		if (cddc_num > ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_NCO_CHAN_SEL_LEN || val < 0)
			return -EINVAL;
		ret = adi_apollo_cnco_profile_sel_mode_set(&phy->ad9088, dir, cnco_en, val);
		if (ret)
			return -EFAULT;
		phy->ffh.dir[dir].cnco.mode[cddc_num] = val;
		break;
	default:
		return -EINVAL;
	}

	return len;
}


