// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 SPI DDS (Direct Digital Synthesizer) driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#ifndef IIO_FREQUENCY_AD9910_H_
#define IIO_FREQUENCY_AD9910_H_

#include <linux/errno.h>
#include <linux/iio/backend.h>

#define AD9910_NUM_PROFILES	8
#define AD9910_RAM_FW_MAGIC	0x00AD9910

struct ad9910_backend;

/**
 * enum ad9910_channel - AD9910 channel identifiers
 *
 * @AD9910_CHANNEL_PHY: Physical output channel
 * @AD9910_CHANNEL_PROFILE_0: Profile 0 output channel
 * @AD9910_CHANNEL_PROFILE_1: Profile 1 output channel
 * @AD9910_CHANNEL_PROFILE_2: Profile 2 output channel
 * @AD9910_CHANNEL_PROFILE_3: Profile 3 output channel
 * @AD9910_CHANNEL_PROFILE_4: Profile 4 output channel
 * @AD9910_CHANNEL_PROFILE_5: Profile 5 output channel
 * @AD9910_CHANNEL_PROFILE_6: Profile 6 output channel
 * @AD9910_CHANNEL_PROFILE_7: Profile 7 output channel
 * @AD9910_CHANNEL_PARALLEL_PORT: Parallel port output channel
 * @AD9910_CHANNEL_DRG: Digital Ramp Generator output channel
 * @AD9910_CHANNEL_DRG_RAMP_UP: DRG ramp up channel
 * @AD9910_CHANNEL_DRG_RAMP_DOWN: DRG ramp down channel
 * @AD9910_CHANNEL_RAM: RAM control output channel
 * @AD9910_CHANNEL_OSK: Output Shift Keying output channel
 */
enum ad9910_channel {
	AD9910_CHANNEL_PHY = 100,
	AD9910_CHANNEL_PROFILE_0 = 101,
	AD9910_CHANNEL_PROFILE_1 = 102,
	AD9910_CHANNEL_PROFILE_2 = 103,
	AD9910_CHANNEL_PROFILE_3 = 104,
	AD9910_CHANNEL_PROFILE_4 = 105,
	AD9910_CHANNEL_PROFILE_5 = 106,
	AD9910_CHANNEL_PROFILE_6 = 107,
	AD9910_CHANNEL_PROFILE_7 = 108,
	AD9910_CHANNEL_PARALLEL_PORT = 110,
	AD9910_CHANNEL_DRG = 120,
	AD9910_CHANNEL_DRG_RAMP_UP = 121,
	AD9910_CHANNEL_DRG_RAMP_DOWN = 122,
	AD9910_CHANNEL_RAM = 130,
	AD9910_CHANNEL_OSK = 140,
};

/**
 * enum ad9910_destination - AD9910 DDS core parameter destination
 *
 * @AD9910_DEST_FREQUENCY: Frequency destination
 * @AD9910_DEST_PHASE: Phase destination
 * @AD9910_DEST_AMPLITUDE: Amplitude destination
 * @AD9910_DEST_POLAR: Polar destination
 */
enum ad9910_destination {
	AD9910_DEST_FREQUENCY,
	AD9910_DEST_PHASE,
	AD9910_DEST_AMPLITUDE,
	AD9910_DEST_POLAR,
};

/**
 * enum ad9910_reset_ctrl_id - AD9910 Reset Controller ID
 *
 * @AD9910_RESET_CTRL_DEVICE: AD9910 Device reset control
 * @AD9910_RESET_CTRL_IO: AD9910 IO reset control
 * @AD9910_RESET_CTRL_MAX: Maximum number of reset controls
 */
enum ad9910_reset_ctrl_id {
	AD9910_RESET_CTRL_DEVICE,
	AD9910_RESET_CTRL_IO,
	AD9910_RESET_CTRL_MAX,
};

/**
 * enum ad9910_pp_scan_type - Parallel Port scan types
 *
 * @AD9910_PP_SCAN_TYPE_FULL: Scan type with both format and data bits
 * @AD9910_PP_SCAN_TYPE_DATA_ONLY: Scan type with only data bits
 */
enum ad9910_pp_scan_type {
	AD9910_PP_SCAN_TYPE_FULL,
	AD9910_PP_SCAN_TYPE_DATA_ONLY,
};

/**
 * enum ad9910_drg_oper_mode - Digital Ramp Generator Operating Mode
 *
 * @AD9910_DRG_OPER_MODE_BIDIR: Normal Ramp Generation
 * @AD9910_DRG_OPER_MODE_RAMP_DOWN: No-dwell Low only operation
 * @AD9910_DRG_OPER_MODE_RAMP_UP: No-dwell High only operation
 * @AD9910_DRG_OPER_MODE_BIDIR_CONT: Both No-dwell High/Low operation
 */
enum ad9910_drg_oper_mode {
	AD9910_DRG_OPER_MODE_BIDIR,
	AD9910_DRG_OPER_MODE_RAMP_DOWN,
	AD9910_DRG_OPER_MODE_RAMP_UP,
	AD9910_DRG_OPER_MODE_BIDIR_CONT,
};

/**
 * enum ad9910_ram_oper_mode - AD9910 RAM Playback Operating Mode
 *
 * @AD9910_RAM_MODE_DIRECT_SWITCH: Direct profile switching between profiles
 * @AD9910_RAM_MODE_RAMP_UP: Ramp up for current profile
 * @AD9910_RAM_MODE_BIDIR: Ramp up/down for profile 0
 * @AD9910_RAM_MODE_BIDIR_CONT: Continuous ramp up/down for current profile
 * @AD9910_RAM_MODE_RAMP_UP_CONT: Continuous ramp up for current profile
 * @AD9910_RAM_MODE_SEQ: Sequenced playback of RAM profiles up to target profile
 * @AD9910_RAM_MODE_SEQ_CONT: Continuous sequenced playback of RAM profiles
 */
enum ad9910_ram_oper_mode {
	AD9910_RAM_MODE_DIRECT_SWITCH,
	AD9910_RAM_MODE_RAMP_UP,
	AD9910_RAM_MODE_BIDIR,
	AD9910_RAM_MODE_BIDIR_CONT,
	AD9910_RAM_MODE_RAMP_UP_CONT,
	AD9910_RAM_MODE_SEQ,
	AD9910_RAM_MODE_SEQ_CONT,
};

/**
 * struct ad9910_ram_fw - AD9910 RAM firmware format
 * @magic:	Magic number for RAM firmware validation
 * @cfr1:	Value of CFR1 register to be configured (not all fields are
 *		used, but this is included here for convenience)
 * @profiles:	Array of RAM profile configurations
 * @reserved:	Reserved field for future use, should be set to 0
 * @wcount:	Number of RAM words to be written
 * @words:	Array of RAM words to be written. Data pattern should be set in
 *		reverse order and wcount specifies the number of words in this
 *		array
 */
struct ad9910_ram_fw {
	__be32 magic;
	__be32 cfr1;
	__be64 profiles[AD9910_NUM_PROFILES];
	__be32 reserved;
	__be32 wcount;
	__be32 words[] __counted_by_be(wcount);
} __packed;

/**
 * struct ad9910_backend_ops - AD9910 Backend operations
 * @profile_set:	Set the active profile acting on the PROFILE[2:0] pins
 * @io_update:		Trigger an IO Update
 * @drg_oper_mode_set:	Set the DRG Operating Mode. Backend may implement custom
 *			DRCTL control based on the DRG operating mode.
 * @powerdown_set:	Set the Powerdown state
 */
struct ad9910_backend_ops {
	int (*profile_set)(struct ad9910_backend *back, unsigned int profile);
	int (*io_update)(struct ad9910_backend *back);
	int (*drg_oper_mode_set)(struct ad9910_backend *back,
				 enum ad9910_drg_oper_mode mode);
	int (*powerdown_set)(struct ad9910_backend *back, unsigned int enable);
};

/**
 * struct ad9910_backend_info - AD9910 Backend information
 * @base: IIO backend base information
 * @ops: AD9910 backend operations
 */
struct ad9910_backend_info {
	struct iio_backend_info base;
	const struct ad9910_backend_ops *ops;
};

/**
 * struct ad9910_backend - AD9910 Backend structure
 * @iio_back: IIO backend instance
 * @ops: AD9910 backend operations
 */
struct ad9910_backend {
	struct iio_backend *iio_back;
	const struct ad9910_backend_ops *ops;
};

#define ad9910_backend_check_op(back, op) ({ \
	struct ad9910_backend *____back = back;			\
	int ____ret = 0;					\
								\
	if (!____back->ops->op)					\
		____ret = -EOPNOTSUPP;				\
								\
	____ret;						\
})

#define ad9910_backend_op_call(back, op, args...) ({		\
	struct ad9910_backend *__back = back;			\
	int __ret;						\
								\
	__ret = ad9910_backend_check_op(__back, op);		\
	if (!__ret)						\
		__ret = __back->ops->op(__back, ##args);	\
								\
	__ret;							\
})

int devm_ad9910_backend_register(struct device *dev,
				 const struct ad9910_backend_info *info,
				 void *priv);

#endif /* IIO_FREQUENCY_AD9910_H_ */