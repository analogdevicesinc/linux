/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Analog Devices Generic AXI ADC IP core driver/library
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_dac_ip
 *
 * Copyright 2012-2021 Analog Devices Inc.
 */
#ifndef __ADI_AXI_DAC_H__
#define __ADI_AXI_DAC_H__

struct adi_axi_dac_conv;
struct device;
struct iio_chan_spec;

/**
 * struct adi_axi_dac_chip_info - Chip specific information
 * @name:		Chip name
 * @id:			Chip ID (usually product ID)
 * @channels:		Channel specifications of type @struct iio_chan_spec
 * @num_channels:	Number of @channels
 */
struct adi_axi_dac_chip_info {
	const char			*name;
	unsigned int			id;

	const struct iio_chan_spec	*channels;
	unsigned int			num_channels;
};

/**
 * struct adi_axi_dac_conv - data of the ADC attached to the AXI ADC
 * @chip_info:		chip info details for the client ADC
 * @preenable_setup:	op to run in the client before enabling the AXI ADC
 * @get_rate:		hook for client DAC to provide it's current rate
 * @reg_access:		IIO debugfs_reg_access hook for the client ADC
 * @read_raw:		IIO read_raw hook for the client ADC
 * @write_raw:		IIO write_raw hook for the client ADC
 */
struct adi_axi_dac_conv {
	const struct adi_axi_dac_chip_info		*chip_info;

	int (*preenable_setup)(struct adi_axi_dac_conv *conv);
	long long (*get_rate)(struct adi_axi_dac_conv *conv);
	int (*reg_access)(struct adi_axi_dac_conv *conv, unsigned int reg,
			  unsigned int writeval, unsigned int *readval);
	int (*read_raw)(struct adi_axi_dac_conv *conv,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask);
	int (*write_raw)(struct adi_axi_dac_conv *conv,
			 struct iio_chan_spec const *chan,
			 int val, int val2, long mask);
};

struct adi_axi_dac_conv *devm_adi_axi_dac_conv_register(struct device *dev,
							int sizeof_priv);

void *adi_axi_dac_conv_priv(struct adi_axi_dac_conv *conv);

#endif
