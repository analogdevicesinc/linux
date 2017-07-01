/*
 * AD9508 SPI Low Jitter Clock Generator
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_FREQUENCY_AD9508_H_
#define IIO_FREQUENCY_AD9508_H_

/**
 * struct ad9528_channel_spec - Output channel configuration
 *
 * @channel_num: Output channel number.
 * @sync_ignore_en: Ignore chip-level SYNC signal.
 * @output_dis: Disables, powers down the entire channel.
 * @driver_mode: Output driver mode (logic level family).
 * @divider_phase: Divider initial phase after a SYNC.
 * @channel_divider: 10-bit channel divider.
 * @extended_name: Optional descriptive channel name.
 */

struct ad9508_channel_spec {
	unsigned		channel_num;
	bool			sync_ignore_en;
				 /* CH0..CH5 */
	bool			output_dis;
	unsigned		driver_mode;
	unsigned		divider_phase;
	unsigned		channel_divider;
	char			extended_name[16];
};

/**
 * struct ad9528_platform_data - platform specific information
 *
 * @spi3wire: SPI 3-Wire mode enable;

 * @num_channels: Array size of struct ad9528_channel_spec.
 * @channels: Pointer to channel array.
 * @name: Optional alternative iio device name.
 */

struct ad9508_platform_data {
	bool				spi3wire;

	/* Output Channel Configuration */
	int				num_channels;
	struct ad9508_channel_spec	*channels;
	char				name[SPI_NAME_SIZE];

};

#endif /* IIO_FREQUENCY_AD9508_H_ */
