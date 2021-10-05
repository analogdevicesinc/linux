/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ADPD188 Integrated Optical Module for Smoke Detection
 *
 * Copyright 2021 Analog Devices Inc.
 */

#ifndef _ADPD188_H_
#define _ADPD188_H_

#define ADPD188_REG_GPIO_DRV			0x02
#define ADPD188_REG_DEVID			0x08
#define ADPD188_REG_GPIO_CTRL			0x0B

/** ADPD188_REG_GPIO_DRV */
#define ADPD188_GPIO1_DRV			BIT(9)
#define ADPD188_GPIO1_POL			BIT(8)
#define ADPD188_GPIO0_ENA			BIT(2)
#define ADPD188_GPIO0_DRV			BIT(1)
#define ADPD188_GPIO0_POL			BIT(0)

/** ADPD188_REG_DEVID */
#define ADPD188_DEVID_MASK				GENMASK(7, 0)

/** ADPD188_REG_GPIO_CTRL */
#define ADPD188_GPIO1_ALT_CONFIG		GENMASK(12, 8)
#define ADPD188_GPIO0_ALT_CONFIG		GENMASK(4, 0)

struct adpd188_ops {
	int (*reg_write)(void *bus, int reg_addr, int val);
	int (*reg_read)(void *bus, int reg_addr, int *val);
	int (*i2c_sub_write)(void *bus, int sub_addr, int reg_addr, int val);
	int (*i2c_sub_read)(void *bus, int sub_addr, int reg_addr, int *val);
};

enum adpd188_phy_opt {
	ADPD188_I2C,
	ADPD188_SPI
};

#define SAMPLE_WORDS	18
struct adpd188 {
	struct adpd188_ops *ops;
	enum adpd188_phy_opt phy_opt;
	void *bus;
	int no_devices;
	union {
		u32 word[SAMPLE_WORDS];
		u16 reg_data[SAMPLE_WORDS * 2];
	} data ____cacheline_aligned;
	struct completion value_ok;
};

enum supported_parts {
	ADPD188
};

int adpd188_core_probe(void *bus, struct adpd188_ops *phy,
		       enum adpd188_phy_opt opt,
		       int dev_no,
		       const char *name, int irq);

#endif /* _ADPD188_H_ */

