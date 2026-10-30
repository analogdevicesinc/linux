/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Calling ADI ADSP boot ROM API functions from Linux.
 *
 * The interface is architecture neutral; each architecture (or SoC
 * family) that can call its boot ROM provides an implementation and sets
 * CONFIG_ADI_ADSP_ROM. For the ADSP-SC58x see
 * drivers/soc/adi/mach-sc5xx/rom-sc58x.c and
 * Documentation/arch/arm/adi/sc5xx-rom.rst.
 */
#ifndef __LINUX_SOC_ADI_ADSP_ROM_H
#define __LINUX_SOC_ADI_ADSP_ROM_H

#include <linux/err.h>
#include <linux/types.h>

struct device;
struct adsp_rom;

#if IS_ENABLED(CONFIG_ADI_ADSP_ROM)

struct adsp_rom *devm_adsp_rom_get(struct device *dev);
unsigned long adsp_rom_call(struct adsp_rom *rom, unsigned long fn,
			    unsigned long arg0, unsigned long arg1,
			    unsigned long arg2, unsigned long arg3);

#else

static inline struct adsp_rom *devm_adsp_rom_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

static inline unsigned long adsp_rom_call(struct adsp_rom *rom,
					  unsigned long fn,
					  unsigned long arg0,
					  unsigned long arg1,
					  unsigned long arg2,
					  unsigned long arg3)
{
	return 0;
}

#endif

#endif /* __LINUX_SOC_ADI_ADSP_ROM_H */
