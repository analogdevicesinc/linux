#include <linux/types.h>
#include <linux/kstrtox.h>

#include "ad9088.h"

int ad9088_ffh_probe(struct ad9088_phy *phy)
{
	const adi_apollo_nco_profile_sel_mode_e profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_DIRECT_REGMAP;
	adi_apollo_coarse_nco_hop_t nco_hop_config;
	int ret;

	nco_hop_config.profile_sel_mode = ADI_APOLLO_NCO_CHAN_SEL_TRIG_AUTO;
	nco_hop_config.auto_mode = ADI_APOLLO_NCO_AUTO_HOP_DECR;

	ret = adi_apollo_fnco_profile_sel_mode_set(&phy->ad9088, ADI_APOLLO_RX,
						   ADI_APOLLO_FNCO_ALL, profile_sel_mode);
	if (ret)
		return ret;
	ret = adi_apollo_fnco_profile_sel_mode_set(&phy->ad9088, ADI_APOLLO_TX,
						   ADI_APOLLO_FNCO_ALL, profile_sel_mode);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_RX,
					 ADI_APOLLO_CNCO_ALL, &nco_hop_config);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_hop_enable(&phy->ad9088, ADI_APOLLO_TX,
					 ADI_APOLLO_CNCO_ALL, &nco_hop_config);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_profile_sel_mode_set(&phy->ad9088, ADI_APOLLO_RX,
						   ADI_APOLLO_CNCO_ALL, profile_sel_mode);
	if (ret)
		return ret;
	ret = adi_apollo_cnco_profile_sel_mode_set(&phy->ad9088, ADI_APOLLO_TX,
						   ADI_APOLLO_CNCO_ALL, profile_sel_mode);
	if (ret)
		return ret;

	return ret;
}

ssize_t ad9088_ext_info_read_ffh(struct iio_dev *indio_dev, uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9088_phy *phy = conv->phy;
	u8 cddc_num, fddc_num, side;
	u32 cddc_mask, fddc_mask;
	long long val;

	guard(mutex)(&phy->lock);
	ad9088_iiochan_to_fddc_cddc(phy, chan, &fddc_num,
				    &fddc_mask, &cddc_num, &cddc_mask, &side);

	switch (private) {
	case FFH_FNCO_INDEX:
		val = phy->ffh.dir[chan->output].fnco.index[fddc_num];
		break;
	case FFH_FNCO_FREQUENCY:
		val = phy->ffh.dir[chan->output].fnco.frequency[fddc_num];
		if (val >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[chan->output].fnco.frequency[val];
		break;
	case FFH_FNCO_SELECT:
		val = phy->ffh.dir[chan->output].fnco.select[fddc_num]-1;
		break;
	case FFH_CNCO_INDEX:
		val = phy->ffh.dir[chan->output].cnco.index[fddc_num];
		break;
	case FFH_CNCO_FREQUENCY:
		val = phy->ffh.dir[chan->output].cnco.frequency[fddc_num];
		if (val >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		val = phy->ffh.dir[chan->output].cnco.frequency[val];
		break;
	case FFH_CNCO_SELECT:
		val = phy->ffh.dir[chan->output].cnco.select[fddc_num]-1;
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
	u32 cddc_mask, fddc_mask;
	u16 fnco_en, cnco_en;
	bool hop_enable;
	int val, ret;

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
		phy->ffh.dir[chan->output].fnco.index[fddc_num] = val;
		break;
	case FFH_FNCO_FREQUENCY:
		fnco_num = fddc_num + side*8;
		fnco_en = GENMASK(fnco_num+1, fnco_num);
		ret = kstrtoint(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[chan->output].fnco.index[fddc_num];
		if (index >= ADI_APOLLO_FNCO_PROFILE_NUM)
			return -EINVAL;
		/* Needs to be enabled to apply */
		ret = adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en, true);
		if (ret)
			return ret;
		ret = adi_apollo_fnco_profile_load(&phy->ad9088, dir, fnco_en,
						   ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT,
						   index, &val, 1);
		if (ret)
			return ret;
		phy->ffh.dir[chan->output].fnco.frequency[index] = val;
		/* Restore state */
		adi_apollo_fnco_hop_enable(&phy->ad9088, dir, fnco_en,
					   phy->ffh.dir[chan->output].fnco.en[index]);
		if (ret)
			return ret;
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
			return ret;
		phy->ffh.dir[chan->output].fnco.en[fddc_num] = hop_enable;
		if (!hop_enable)
			break;
		ret = adi_apollo_fnco_active_profile_set(&phy->ad9088, dir, fnco_en, val);
		if (ret)
			return ret;
		/* Increment by 1 to use 0 to flag disabled */
		phy->ffh.dir[chan->output].fnco.select[fddc_num] = val+1;
		phy->ffh.dir[chan->output].fnco.en[fddc_num] = hop_enable;
		break;
	case FFH_CNCO_INDEX:
		if (cddc_num > ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = kstrtoint(buf, 10, &val);
		if (ret || val >= ADI_APOLLO_CNCO_PROFILE_NUM || val < 0)
			return -EINVAL;
		phy->ffh.dir[chan->output].cnco.index[cddc_num] = val;
		break;
	case FFH_CNCO_FREQUENCY:
		cnco_num = cddc_num + side*4;
		cnco_en = BIT(cnco_num);
		ret = kstrtoint(buf, 10, &val);
		if (ret || val < 0)
			return -EINVAL;
		index = phy->ffh.dir[chan->output].cnco.index[cddc_num];
		if (index >= ADI_APOLLO_CNCO_PROFILE_NUM)
			return -EINVAL;
		ret = adi_apollo_cnco_profile_load(&phy->ad9088, dir, cnco_en,
						   ADI_APOLLO_NCO_PROFILE_PHASE_INCREMENT,
						   index, &val, 1);
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
		ret = adi_apollo_cnco_active_profile_set(&phy->ad9088, dir, cnco_en, val);
		if (ret)
			return ret;

		phy->ffh.dir[chan->output].cnco.select[cddc_num] = val;
		break;
	default:
		return -EINVAL;
	}

	return len;
}


