/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2022, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __DRIVERS_PINCTRL_ADI_H
#define __DRIVERS_PINCTRL_ADI_H

#include <stdbool.h>

#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>

#define ADI_PINCTRL_PIN(pin) PINCTRL_PIN(pin, #pin)

/*
 * Bit Mask Info for ADI's Pinctrl Word
 */
#define ADI_CONFIG_DRIVE_STRENGTH_MASK                        (0x0000000FU)
#define ADI_CONFIG_DRIVE_STRENGTH_MASK_BIT_POSITION           (0U)
#define ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK                   (0x00000010U)
#define ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK_BIT_POSITION      (4U)
#define ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK               (0x00000020U)
#define ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK_BIT_POSITION  (5U)
#define ADI_CONFIG_PULLUP_ENABLE_MASK                         (0x00000040U)
#define ADI_CONFIG_PULLUP_ENABLE_MASK_BIT_POSITION            (6)

struct platform_device;
extern const struct pinmux_ops adi_pmx_ops;
extern const struct dev_pm_ops adi_pinctrl_pm_ops;

/**
 * struct adi_pin_mio - PIO pin configurations
 * @input_pin: Pin Number
 * @mux_sel: source mux select value
 * @config: Configuration data for pin
 */
struct adi_pin_mio {
	unsigned int input_pin;
	unsigned int mux_sel;
	unsigned long config;
};

/**
 * struct adi_pin - describes a single pintctrl pin
 * @pin: the pin_id of this pin
 * @conf: config info for this pin
 */
struct adi_pin {
	unsigned int pin;
	union {
		struct adi_pin_mio mio;
	} conf;
};

/**
 * struct adi_pin_reg - describe a pin configuration
 * @pin_num: Pin Number
 * @mux_reg: mux register
 * @conf_reg: config register
 */
struct adi_pin_reg {
	uint32_t pin_num;
	uint32_t mux_reg;
	uint32_t conf_reg;
};

/**
 * structure containing the ADI pinctrl context
 *
 * @dev: a pointer back to containing device
 * @pctl: pin control class device
 * @info: soc specific information
 * @pin_regs: pin configuration information
 * @group_index: group
 * @mutex: mutex context
 */
struct adi_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	const struct adi_pinctrl_soc_info *info;
	struct adi_pin_reg *pin_regs;
	unsigned int group_index;
	struct mutex mutex;
	phys_addr_t phys_addr;
};

/**
 * SOC pinctrl data structure
 *
 * @pins struct containing pin information
 * @npins Number of pins
 * @adi_pinconf_get  hook for registered handler, get pin configuration
 * @adi_pinconf_set  hook for registered handler, set pin configuration
 * @adi_pinctrl_parse_pin  hook for registered handler, parse pin info
 *
 */
struct adi_pinctrl_soc_info {
	const struct pinctrl_pin_desc *pins;
	unsigned int npins;

	int (*adi_pinconf_get)(struct pinctrl_dev *pctldev, unsigned int pin_id, unsigned long *config);
	int (*adi_pinconf_set)(struct pinctrl_dev *pctldev, unsigned int pin_id, unsigned long *configs, unsigned int num_configs);
	void (*adi_pinctrl_parse_pin)(struct adi_pinctrl *ipctl, unsigned int *pin_id, struct adi_pin *pin, const __be32 **list_p);
};

int adi_pinctrl_probe(struct platform_device *pdev, const struct adi_pinctrl_soc_info *info);
int adi_pinconf_get_smc(struct pinctrl_dev *pctldev, unsigned int pin_id, unsigned long *config);
int adi_pinconf_set_smc(struct pinctrl_dev *pctldev, unsigned int pin_id, unsigned long *configs, unsigned int num_configs);

#endif /* __DRIVERS_PINCTRL_ADI_H */
