/* SPDX-License-Identifier: GPL-2.0-only */

#define LTC2497_ENABLE 0xA0
#define LTC2497_CONFIG_DEFAULT LTC2497_ENABLE
#define LTC2497_CONVERSION_TIME_MS 150ULL

enum chip_type {
	TYPE_LTC2496,
	TYPE_LTC2497,
	TYPE_LTC2499
};
struct chip_info {
	enum chip_type type;
	u32 resolution;
};

struct ltc2497core_driverdata {
	struct regulator *ref;
	const struct chip_info *chip_info;
	ktime_t time_prev;
	u8 addr_prev;

	int (*result_and_measure)(struct ltc2497core_driverdata *ddata,
				  u8 address, int *val);
};

int ltc2497core_probe(struct device *dev, struct iio_dev *indio_dev);
void ltc2497core_remove(struct iio_dev *indio_dev);

MODULE_IMPORT_NS(LTC2497);
