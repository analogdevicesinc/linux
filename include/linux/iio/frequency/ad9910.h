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

/* channels in priority order */
#define AD9910_CHANNEL_SINGLE_TONE	0
#define AD9910_CHANNEL_PARALLEL_PORT	1
#define AD9910_CHANNEL_DRG		2
#define AD9910_CHANNEL_RAM		3
#define AD9910_CHANNEL_OSK		4

struct ad9910_backend;

/**
 * enum ad9910_drg_oper_mode - DRG Operating Mode
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
 * enum ad9910_drg_control_mode - DRG Control Mode
 * @AD9910_DRG_CTL_MODE_ON: output high for the DRCTL pin
 * @AD9910_DRG_CTL_MODE_OFF: output low for the DRCTL pin
 * @AD9910_DRG_CTL_MODE_OVERFLOW_TOGGLE: DRCTL pin toggles on DROVER pin events
 */
enum ad9910_drg_control_mode {
	AD9910_DRG_CTL_MODE_ON,
	AD9910_DRG_CTL_MODE_OFF,
	AD9910_DRG_CTL_MODE_OVERFLOW_TOGGLE,
};

/**
 * struct ad9910_backend_ops - AD9910 Backend operations
 * @profile_set: Set the active profile acting on the PROFILE[2:0] pins
 * @io_update: Trigger an IO Update
 * @io_reset: Trigger an IO Reset
 * @drg_control_mode_set: Set the DRG Control Mode
 * @drg_control_mode_get: Get the DRG Control Mode
 * @drg_oper_mode_set: Set the DRG Operating Mode
 * @powerdown_set: Set the Powerdown state
 * @reset: Set the Reset state
 */
struct ad9910_backend_ops {
	int (*profile_set)(struct ad9910_backend *back, unsigned int profile);
	int (*io_update)(struct ad9910_backend *back);
	int (*io_reset)(struct ad9910_backend *back);
	int (*drg_control_mode_set)(struct ad9910_backend *back,
				    enum ad9910_drg_control_mode mode);
	int (*drg_control_mode_get)(struct ad9910_backend *back,
				    enum ad9910_drg_control_mode *mode);
	int (*drg_oper_mode_set)(struct ad9910_backend *back,
				 enum ad9910_drg_oper_mode mode);
	int (*powerdown_set)(struct ad9910_backend *back, unsigned int enable);
	int (*reset)(struct ad9910_backend *back, unsigned int enable);
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