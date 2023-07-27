// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 BayLibre SAS
// Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
//
// Core MFD driver for MAXIM 77650/77651 charger/power-supply.
// Programming manual: https://pdfserv.maximintegrated.com/en/an/AN6428.pdf

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77650.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#define MAX77650_INT_GPI_F_MSK		BIT(0)
#define MAX77650_INT_GPI_R_MSK		BIT(1)
#define MAX77650_INT_GPI_MSK \
			(MAX77650_INT_GPI_F_MSK | MAX77650_INT_GPI_R_MSK)
#define MAX77650_INT_nEN_F_MSK		BIT(2)
#define MAX77650_INT_nEN_R_MSK		BIT(3)
#define MAX77650_INT_TJAL1_R_MSK	BIT(4)
#define MAX77650_INT_TJAL2_R_MSK	BIT(5)
#define MAX77650_INT_DOD_R_MSK		BIT(6)

#define MAX77650_INT_THM_MSK		BIT(0)
#define MAX77650_INT_CHG_MSK		BIT(1)
#define MAX77650_INT_CHGIN_MSK		BIT(2)
#define MAX77650_INT_TJ_REG_MSK		BIT(3)
#define MAX77650_INT_CHGIN_CTRL_MSK	BIT(4)
#define MAX77650_INT_SYS_CTRL_MSK	BIT(5)
#define MAX77650_INT_SYS_CNFG_MSK	BIT(6)

#define MAX77650_INT_GLBL_OFFSET	0
#define MAX77650_INT_CHG_OFFSET		1

#define MAX77650_SBIA_LPM_MASK		BIT(5)
#define MAX77650_SBIA_LPM_DISABLED	0x00

enum {
	MAX77643_INT_GPI0_F,
	MAX77643_INT_GPI0_R,
	MAX77643_INT_nEN_F,
	MAX77643_INT_nEN_R,
	MAX77643_INT_TJAL1_R,
	MAX77643_INT_TJAL2_R,
	MAX77643_INT_DOD_R,
};

enum {
	MAX77643_INT_GPI1_F,
	MAX77643_INT_GPI1_R,
	MAX77643_INT_SBB0_F,
	MAX77643_INT_SBB1_F,
	MAX77643_INT_SBB2_F,
	MAX77643_INT_LDO_F,
};

enum {
	MAX77650_INT_GPI,
	MAX77650_INT_nEN_F,
	MAX77650_INT_nEN_R,
	MAX77650_INT_TJAL1_R,
	MAX77650_INT_TJAL2_R,
	MAX77650_INT_DOD_R,
};

enum {
	MAX77650_INT_THM,
	MAX77650_INT_CHG,
	MAX77650_INT_CHGIN,
	MAX77650_INT_TJ_REG,
	MAX77650_INT_CHGIN_CTRL,
	MAX77650_INT_SYS_CTRL,
	MAX77650_INT_SYS_CNFG,
};

enum {
	MAX77654_INT_GPI0_F,
	MAX77654_INT_GPI0_R,
	MAX77654_INT_nEN_F,
	MAX77654_INT_nEN_R,
	MAX77654_INT_TJAL1_R,
	MAX77654_INT_TJAL2_R,
	MAX77654_INT_DOD1_R,
	MAX77654_INT_DOD0_R,
};

enum {
	MAX77654_INT_GPI1_F,
	MAX77654_INT_GPI1_R,
	MAX77654_INT_GPI2_F,
	MAX77654_INT_GPI2_R,
	MAX77654_INT_SBBTO,
	MAX77654_INT_LDO0_F,
	MAX77654_INT_LDO1_F,
};

enum {
	MAX77658_INT_GPI,
	MAX77658_INT_nEN_F,
	MAX77658_INT_nEN_R,
	MAX77658_INT_TJAL1_R,
	MAX77658_INT_TJAL2_R,
	MAX77658_INT_DOD1_R,
	MAX77658_INT_DOD0_R,
};

enum {
	MAX77658_INT_GPI1_F,
	MAX77658_INT_GPI1_R,
	MAX77658_INT_SBB0_F,
	MAX77658_INT_SBB1_F,
	MAX77658_INT_SBB2_F,
	MAX77658_INT_LDO0_F,
	MAX77658_INT_LDO1_F,
};

enum {
	MAX77659_INT_GPI0_F,
	MAX77659_INT_GPI0_R,
	MAX77659_INT_nEN_F,
	MAX77659_INT_nEN_R,
	MAX77659_INT_TJAL1_R,
	MAX77659_INT_TJAL2_R,
	MAX77659_INT_DOD_R,
};

enum {
	MAX77659_INT_GPI1_F,
	MAX77659_INT_GPI1_R,
	MAX77659_INT_SBBTO,
	MAX77659_INT_LDO_F,
};

enum {
	MAX77659_INT_THM,
	MAX77659_INT_CHG,
	MAX77659_INT_CHGIN,
	MAX77659_INT_TJ_REG,
	MAX77659_INT_SYS_CTRL,
};

#define MAX77650_REGULATOR	0
#define MAX77650_CHARGER	1
#define MAX77650_LED		2
#define MAX77650_GPIO		3
#define	MAX77650_ONKEY		4
#define	MAX77658_BATTERY	2

static const struct resource max77650_charger_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77650_INT_CHG, "CHG"),
	DEFINE_RES_IRQ_NAMED(MAX77650_INT_CHGIN, "CHGIN"),
};

static const struct resource max77650_gpio_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77650_INT_GPI, "GPI"),
};

static const struct resource max77650_onkey_resources[] = {
	DEFINE_RES_IRQ_NAMED(MAX77650_INT_nEN_F, "nEN_F"),
	DEFINE_RES_IRQ_NAMED(MAX77650_INT_nEN_R, "nEN_R"),
};

static const struct mfd_cell max77643_cells[] = {
	[MAX77650_REGULATOR] = {
		.name		= "max77650-regulator",
		.of_compatible	= "adi,max77643-regulator",
	},
};

static const struct mfd_cell max77650_cells[] = {
	[MAX77650_REGULATOR] = {
		.name		= "max77650-regulator",
		.of_compatible	= "maxim,max77650-regulator",
	},
	[MAX77650_CHARGER] = {
		.name		= "max77650-charger",
		.of_compatible	= "maxim,max77650-charger",
		.resources	= max77650_charger_resources,
		.num_resources	= ARRAY_SIZE(max77650_charger_resources),
	},
	[MAX77650_LED] = {
		.name		= "max77650-led",
		.of_compatible	= "maxim,max77650-led",
	},
	[MAX77650_GPIO] = {
		.name		= "max77650-gpio",
		.of_compatible	= "maxim,max77650-gpio",
		.resources	= max77650_gpio_resources,
		.num_resources	= ARRAY_SIZE(max77650_gpio_resources),
	},
	[MAX77650_ONKEY] = {
		.name		= "max77650-onkey",
		.of_compatible	= "maxim,max77650-onkey",
		.resources	= max77650_onkey_resources,
		.num_resources	= ARRAY_SIZE(max77650_onkey_resources),
	},
};

static const struct mfd_cell max77654_cells[] = {
	[MAX77650_REGULATOR] = {
		.name		= "max77650-regulator",
		.of_compatible	= "adi,max77654-regulator",
	},
	[MAX77650_CHARGER] = {
		.name = "max77654-charger",
		.of_compatible	= "adi,max77654-charger",
		.resources	= max77650_charger_resources,
		.num_resources	= ARRAY_SIZE(max77650_charger_resources),
	},
};

static const struct mfd_cell max77658_cells[] = {
	[MAX77650_REGULATOR] = {
		.name		= "max77650-regulator",
		.of_compatible	= "adi,max77658-regulator",
	},
	[MAX77650_CHARGER] = {
		.name = "max77658-charger",
		.of_compatible	= "adi,max77658-charger",
		.resources	= max77650_charger_resources,
		.num_resources	= ARRAY_SIZE(max77650_charger_resources),
	},
	[MAX77658_BATTERY] = {
		.name = "max77658-battery",
		.of_compatible	= "adi,max77658-battery",
	},
};

static const struct mfd_cell max77659_cells[] = {
	[MAX77650_REGULATOR] = {
		.name		= "max7750-regulator",
		.of_compatible	= "adi,max77659-regulator",
	},
	[MAX77650_CHARGER] = {
		.name = "max77659-charger",
		.of_compatible	= "adi,max77659-charger",
		.resources	= max77650_charger_resources,
		.num_resources	= ARRAY_SIZE(max77650_charger_resources),
	},
};

static const struct regmap_irq max77643_glbl0_irqs[] = {
	REGMAP_IRQ_REG(MAX77643_INT_GPI0_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77643_INT_GPI0_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77643_INT_nEN_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77643_INT_nEN_R, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77643_INT_TJAL1_R, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77643_INT_TJAL2_R, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77643_INT_DOD_R, 0, BIT(6)),
};

static const struct regmap_irq_chip max77643_glbl0_irq_chip = {
	.name			= "max77643-glbl0-irq",
	.status_base		= MAX77658_REG_INT_GLBL0,
	.mask_base		= MAX77643_REG_INTM_GLBL0,
	.num_regs		= 1,
	.irqs			= max77643_glbl0_irqs,
	.num_irqs		= ARRAY_SIZE(max77643_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77650_glbl_irqs[] = {
	[MAX77650_INT_GPI] = {
		.reg_offset = MAX77650_INT_GLBL_OFFSET,
		.mask = MAX77650_INT_GPI_MSK,
		.type = {
			.type_falling_val = MAX77650_INT_GPI_F_MSK,
			.type_rising_val = MAX77650_INT_GPI_R_MSK,
			.types_supported = IRQ_TYPE_EDGE_BOTH,
		},
	},
	REGMAP_IRQ_REG(MAX77650_INT_nEN_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77650_INT_nEN_R, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77650_INT_TJAL1_R, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77650_INT_TJAL2_R, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77650_INT_DOD_R, 0, BIT(6)),
};

static const struct regmap_irq_chip max77650_glbl_irq_chip = {
	.name			= "max77650-glbl-irq",
	.status_base		= MAX77650_REG_INT_GLBL,
	.mask_base		= MAX77650_REG_INTM_GLBL,
	.num_regs		= 1,
	.irqs			= max77650_glbl_irqs,
	.num_irqs		= ARRAY_SIZE(max77650_glbl_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77654_glbl0_irqs[] = {
	REGMAP_IRQ_REG(MAX77654_INT_GPI0_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77654_INT_GPI0_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77654_INT_nEN_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77654_INT_nEN_R, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77654_INT_TJAL1_R, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77654_INT_TJAL2_R, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77654_INT_DOD1_R, 0, BIT(6)),
	REGMAP_IRQ_REG(MAX77654_INT_DOD0_R, 0, BIT(7)),
};

static const struct regmap_irq_chip max77654_glbl0_irq_chip = {
	.name			= "max77654-glbl0-irq",
	.status_base		= MAX77658_REG_INT_GLBL0,
	.mask_base		= MAX77654_REG_INTM_GLBL0,
	.num_regs		= 1,
	.irqs			= max77654_glbl0_irqs,
	.num_irqs		= ARRAY_SIZE(max77654_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq_chip max77658_glbl0_irq_chip = {
	.name			= "max77658-glbl0-irq",
	.status_base		= MAX77658_REG_INT_GLBL0,
	.mask_base		= MAX77658_REG_INTM_GLBL0,
	.num_regs		= 1,
	.irqs			= max77654_glbl0_irqs,
	.num_irqs		= ARRAY_SIZE(max77654_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_glbl0_irqs[] = {
	REGMAP_IRQ_REG(MAX77659_INT_GPI0_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77659_INT_GPI0_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77659_INT_nEN_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77659_INT_nEN_R, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77659_INT_TJAL1_R, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77659_INT_TJAL2_R, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77659_INT_DOD_R, 0, BIT(7)),
};

static const struct regmap_irq_chip max77659_glbl0_irq_chip = {
	.name			= "max77659-glbl0-irq",
	.status_base		= MAX77658_REG_INT_GLBL0,
	.mask_base		= MAX77654_REG_INTM_GLBL0,
	.num_regs		= 1,
	.irqs			= max77659_glbl0_irqs,
	.num_irqs		= ARRAY_SIZE(max77659_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77643_glbl1_irqs[] = {
	REGMAP_IRQ_REG(MAX77643_INT_GPI1_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77643_INT_GPI1_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77643_INT_SBB0_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77643_INT_SBB1_F, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77643_INT_SBB2_F, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77643_INT_LDO_F, 0, BIT(5)),
};

static const struct regmap_irq_chip max77643_glbl1_irq_chip = {
	.name			= "max77643-glbl1-irq",
	.status_base		= MAX77643_REG_INT_GLBL1,
	.mask_base		= MAX77643_REG_INTM_GLBL1,
	.num_regs		= 1,
	.irqs			= max77643_glbl1_irqs,
	.num_irqs		= ARRAY_SIZE(max77643_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77654_glbl1_irqs[] = {
	REGMAP_IRQ_REG(MAX77654_INT_GPI1_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77654_INT_GPI1_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77654_INT_GPI2_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77654_INT_GPI2_R, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77654_INT_SBBTO, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77654_INT_LDO0_F, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77654_INT_LDO1_F, 0, BIT(6)),
};

static const struct regmap_irq_chip max77654_glbl1_irq_chip = {
	.name			= "max77654-glbl1-irq",
	.status_base		= MAX77658_REG_INT_GLBL1,
	.mask_base		= MAX77654_REG_INTM_GLBL1,
	.num_regs		= 1,
	.irqs			= max77654_glbl1_irqs,
	.num_irqs		= ARRAY_SIZE(max77654_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77658_glbl1_irqs[] = {
	REGMAP_IRQ_REG(MAX77658_INT_GPI1_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77658_INT_GPI1_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77658_INT_SBB0_F, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77658_INT_SBB1_F, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77658_INT_SBB2_F, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77658_INT_LDO0_F, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77658_INT_LDO1_F, 0, BIT(6)),
};

static const struct regmap_irq_chip max77658_glbl1_irq_chip = {
	.name			= "max77658-glbl1-irq",
	.status_base		= MAX77658_REG_INT_GLBL1,
	.mask_base		= MAX77658_REG_INTM_GLBL1,
	.num_regs		= 1,
	.irqs			= max77658_glbl1_irqs,
	.num_irqs		= ARRAY_SIZE(max77658_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_glbl1_irqs[] = {
	REGMAP_IRQ_REG(MAX77659_INT_GPI1_F, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77659_INT_GPI1_R, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77659_INT_SBBTO, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77659_INT_LDO_F, 0, BIT(5)),
};

static const struct regmap_irq_chip max77659_glbl1_irq_chip = {
	.name			= "max77659-glbl1-irq",
	.status_base		= MAX77658_REG_INT_GLBL1,
	.mask_base		= MAX77654_REG_INTM_GLBL1,
	.num_regs		= 1,
	.irqs			= max77659_glbl1_irqs,
	.num_irqs		= ARRAY_SIZE(max77659_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77650_chg_irqs[] = {
	REGMAP_IRQ_REG(MAX77650_INT_THM, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77650_INT_CHG, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77650_INT_CHGIN, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77650_INT_TJ_REG, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77650_INT_CHGIN_CTRL, 0, BIT(4)),
	REGMAP_IRQ_REG(MAX77650_INT_SYS_CTRL, 0, BIT(5)),
	REGMAP_IRQ_REG(MAX77650_INT_SYS_CNFG, 0, BIT(6)),
};

static const struct regmap_irq_chip max77650_chg_irq_chip = {
	.name			= "max77650-chg-irq",
	.status_base		= MAX77650_REG_INT_CHG,
	.mask_base		= MAX77650_REG_INTM_CHG,
	.num_regs		= 1,
	.irqs			= max77650_chg_irqs,
	.num_irqs		= ARRAY_SIZE(max77650_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq_chip max77654_chg_irq_chip = {
	.name			= "max77654-chg-irq",
	.status_base		= MAX77658_REG_INT_CHG,
	.mask_base		= MAX77658_REG_INTM_CHG,
	.num_regs		= 1,
	.irqs			= max77650_chg_irqs,
	.num_irqs		= ARRAY_SIZE(max77650_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq_chip max77658_chg_irq_chip = {
	.name			= "max77658-chg-irq",
	.status_base		= MAX77658_REG_INT_CHG,
	.mask_base		= MAX77658_REG_INTM_CHG,
	.num_regs		= 1,
	.irqs			= max77650_chg_irqs,
	.num_irqs		= ARRAY_SIZE(max77650_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_chg_irqs[] = {
	REGMAP_IRQ_REG(MAX77659_INT_THM, 0, BIT(0)),
	REGMAP_IRQ_REG(MAX77659_INT_CHG, 0, BIT(1)),
	REGMAP_IRQ_REG(MAX77659_INT_CHGIN, 0, BIT(2)),
	REGMAP_IRQ_REG(MAX77659_INT_TJ_REG, 0, BIT(3)),
	REGMAP_IRQ_REG(MAX77659_INT_SYS_CTRL, 0, BIT(4)),
};

static const struct regmap_irq_chip max77659_chg_irq_chip = {
	.name			= "max77659-chg-irq",
	.status_base		= MAX77658_REG_INT_CHG,
	.mask_base		= MAX77658_REG_INTM_CHG,
	.num_regs		= 1,
	.irqs			= max77659_chg_irqs,
	.num_irqs		= ARRAY_SIZE(max77659_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static struct irq_domain *max77650_get_irq_domain(struct device *dev,
						  int device_type)
{
	struct max77650_dev *max77650 = dev_get_drvdata(dev);
	struct irq_domain *domain;

	switch (device_type) {
	case MAX77650_CHARGER:
		domain = regmap_irq_get_domain(max77650->irqc_chg);
		break;
	case MAX77650_GPIO:
	case MAX77650_ONKEY:
		domain = regmap_irq_get_domain(max77650->irqc_glbl0);
		break;
	default:
		domain = NULL;
		break;
	}

	return domain;
}

static int max77650_pmic_irq_init(struct device *dev)
{
	const struct regmap_irq_chip *glbl0_chip, *glbl1_chip, *chg_chip;
	struct max77650_dev *max77650 = dev_get_drvdata(dev);
	int rv;

	switch (max77650->id) {
	case ID_MAX77643:
		glbl0_chip = &max77643_glbl0_irq_chip;
		glbl1_chip = &max77643_glbl1_irq_chip;
		break;
	case ID_MAX77650:
		glbl0_chip = &max77650_glbl_irq_chip;
		chg_chip = &max77650_chg_irq_chip;
		break;
	case ID_MAX77654:
		glbl0_chip = &max77654_glbl0_irq_chip;
		glbl1_chip = &max77654_glbl1_irq_chip;
		chg_chip = &max77654_chg_irq_chip;
		break;
	case ID_MAX77658:
		glbl0_chip = &max77658_glbl0_irq_chip;
		glbl1_chip = &max77658_glbl1_irq_chip;
		chg_chip = &max77658_chg_irq_chip;
		break;
	case ID_MAX77659:
		glbl0_chip = &max77659_glbl0_irq_chip;
		glbl1_chip = &max77659_glbl1_irq_chip;
		chg_chip = &max77659_chg_irq_chip;
		break;
	default:
		return -EINVAL;
	}

	if (max77650->id != ID_MAX77643) {
		rv = devm_regmap_add_irq_chip(dev, max77650->regmap,
					      max77650->irq,
					      IRQF_ONESHOT | IRQF_SHARED, 0,
					      chg_chip, &max77650->irqc_chg);
		if (rv)
			return dev_err_probe(dev, rv,
					     "Unable to add charger IRQ chip\n");
	}

	if (max77650->id != ID_MAX77650) {
		rv = devm_regmap_add_irq_chip(dev, max77650->regmap,
					      max77650->irq,
					      IRQF_ONESHOT | IRQF_SHARED, 0,
					      glbl1_chip,
					      &max77650->irqc_glbl1);
		if (rv)
			return dev_err_probe(dev, rv,
					     "Unable to add glbl1 IRQ chip\n");
	}

	return devm_regmap_add_irq_chip(dev, max77650->regmap, max77650->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       glbl0_chip, &max77650->irqc_glbl0);
}

static const struct regmap_config max77650_regmap_config = {
	.name		= "max77650",
	.reg_bits	= 8,
	.val_bits	= 8,
};

static int max77650_i2c_probe(struct i2c_client *i2c,
			      const struct i2c_device_id *id)
{
	int rv, n_devs, max77650_chip_id, i;
	struct device *dev = &i2c->dev;
	struct max77650_dev *max77650;
	const struct mfd_cell *cells;
	struct irq_domain *domain;
	unsigned int val;

	max77650 = devm_kzalloc(dev, sizeof(*max77650), GFP_KERNEL);
	if (!max77650)
		return -ENOMEM;

	i2c_set_clientdata(i2c, max77650);
	max77650->irq = i2c->irq;

	max77650->id  = (enum max77650_ids)device_get_match_data(dev);
	if (!max77650->id)
		max77650->id  = (enum max77650_ids)id->driver_data;

	if (!max77650->id)
		return -EINVAL;

	max77650->regmap = devm_regmap_init_i2c(i2c,
						&max77650_regmap_config);
	if (IS_ERR(max77650->regmap))
		return dev_err_probe(dev, PTR_ERR(max77650->regmap),
				     "Unable to initialize regmap\n");

	/*
	 * This IC has a low-power mode which reduces the quiescent current
	 * consumption to ~5.6uA but is only suitable for systems consuming
	 * less than ~2mA. Since this is not likely the case even on
	 * linux-based wearables - keep the chip in normal power mode.
	 */
	rv = regmap_update_bits(max77650->regmap,
				MAX77650_REG_CNFG_GLBL,
				MAX77650_SBIA_LPM_MASK,
				MAX77650_SBIA_LPM_DISABLED);
	if (rv) {
		dev_err(dev, "Unable to change the power mode\n");
		return rv;
	}

	rv = max77650_pmic_irq_init(dev);
	if (rv < 0)
		return dev_err_probe(dev, rv, "Unable to initialize IRQ\n");

	switch (max77650->id) {
	case ID_MAX77643:
		cells = max77643_cells;
		n_devs = ARRAY_SIZE(max77643_cells);
		break;
	case ID_MAX77650:
		rv = regmap_read(max77650->regmap, MAX77650_REG_CID, &val);
		if (rv) {
			dev_err(dev, "Unable to read Chip ID\n");
			return rv;
		}

		max77650_chip_id = MAX77650_CID_BITS(val);
		switch (max77650_chip_id) {
		case MAX77650_CID_77650A:
		case MAX77650_CID_77650C:
		case MAX77650_CID_77651A:
		case MAX77650_CID_77651B:
			break;
		default:
			dev_err(dev, "Chip not supported - ID: 0x%02x\n",
				max77650_chip_id);
			return -ENODEV;
		}

		rv = regmap_update_bits(max77650->regmap,
					MAX77650_REG_CNFG_GLBL,
					MAX77650_SBIA_LPM_MASK,
					MAX77650_SBIA_LPM_DISABLED);
		if (rv) {
			dev_err(dev, "Unable to change the power mode\n");
			return rv;
		}

		cells = max77650_cells;
		n_devs = ARRAY_SIZE(max77650_cells);
		break;
	case ID_MAX77654:
		cells = max77654_cells;
		n_devs = ARRAY_SIZE(max77654_cells);
		break;
	case ID_MAX77658:
		cells = max77658_cells;
		n_devs = ARRAY_SIZE(max77658_cells);
		break;
	case ID_MAX77659:
		cells = max77659_cells;
		n_devs = ARRAY_SIZE(max77659_cells);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < n_devs; i++) {
		domain = max77650_get_irq_domain(dev, i);

		rv = devm_mfd_add_devices(dev, -1, cells + i, 1,
					  NULL, 0, domain);
		if (rv)
			return dev_err_probe(dev, rv,
					     "Unable to add sub-device\n");
	}

	return 0;
}

static const struct of_device_id max77650_of_match[] = {
	{ .compatible = "adi,max77643", .data = (void *)ID_MAX77643 },
	{ .compatible = "maxim,max77650", .data = (void *)ID_MAX77650 },
	{ .compatible = "adi,max77654", .data = (void *)ID_MAX77654 },
	{ .compatible = "adi,max77658", .data = (void *)ID_MAX77658 },
	{ .compatible = "adi,max77659", .data = (void *)ID_MAX77659 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77650_of_match);

static const struct i2c_device_id max77650_i2c_id[] = {
	{ "max77643", ID_MAX77643 },
	{ "max77650", ID_MAX77650 },
	{ "max77654", ID_MAX77654 },
	{ "max77658", ID_MAX77658 },
	{ "max77659", ID_MAX77659 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max77650_i2c_id);

static struct i2c_driver max77650_i2c_driver = {
	.driver = {
		.name = "max77650",
		.of_match_table = max77650_of_match,
	},
	.probe = max77650_i2c_probe,
	.id_table = max77650_i2c_id,
};
module_i2c_driver(max77650_i2c_driver);

MODULE_DESCRIPTION("MAXIM 77650/77651 multi-function core driver");
MODULE_AUTHOR("Bartosz Golaszewski <bgolaszewski@baylibre.com>");
MODULE_LICENSE("GPL v2");
