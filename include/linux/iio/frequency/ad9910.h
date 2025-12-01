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

struct ad9910_backend;

/**
 * enum ad9910_channel - AD9910 channel identifiers in priority order
 */
enum ad9910_channel {
	AD9910_CHANNEL_SINGLE_TONE,
	AD9910_CHANNEL_PARALLEL_PORT,
	AD9910_CHANNEL_DRG,
	AD9910_CHANNEL_RAM,
	AD9910_CHANNEL_OSK,
};

/**
 * enum ad9910_destination - AD9910 DDS core parameter destination 
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
 * struct ad9910_backend_ops - AD9910 Backend operations
 * @profile_set: Set the active profile acting on the PROFILE[2:0] pins
 * @io_update: Trigger an IO Update
 * @drg_oper_mode_set: Set the DRG Operating Mode
 * @powerdown_set: Set the Powerdown state
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