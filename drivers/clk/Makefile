# SPDX-License-Identifier: GPL-2.0
# common clock types
obj-$(CONFIG_HAVE_CLK)		+= clk-devres.o clk-bulk.o clkdev.o
obj-$(CONFIG_COMMON_CLK)	+= clk.o
obj-$(CONFIG_CLK_KUNIT_TEST)	+= clk-test.o
clk-test-y			:= clk_test.o \
				   kunit_clk_parent_data_test.dtbo.o
obj-$(CONFIG_COMMON_CLK)	+= clk-divider.o
obj-$(CONFIG_COMMON_CLK)	+= clk-fixed-factor.o
obj-$(CONFIG_COMMON_CLK)	+= clk-fixed-rate.o
obj-$(CONFIG_CLK_FIXED_RATE_KUNIT_TEST)	+= clk-fixed-rate-test.o
clk-fixed-rate-test-y		:= clk-fixed-rate_test.o kunit_clk_fixed_rate_test.dtbo.o
obj-$(CONFIG_COMMON_CLK)	+= clk-adjustable-rate.o
obj-$(CONFIG_COMMON_CLK)	+= clk-gate.o
obj-$(CONFIG_CLK_GATE_KUNIT_TEST) += clk-gate_test.o
obj-$(CONFIG_COMMON_CLK)	+= clk-multiplier.o
obj-$(CONFIG_COMMON_CLK)	+= clk-mux.o
obj-$(CONFIG_COMMON_CLK)	+= clk-composite.o
obj-$(CONFIG_COMMON_CLK)	+= clk-fractional-divider.o
obj-$(CONFIG_CLK_FD_KUNIT_TEST) += clk-fractional-divider_test.o
obj-$(CONFIG_COMMON_CLK)	+= clk-gpio.o
ifeq ($(CONFIG_OF), y)
obj-$(CONFIG_COMMON_CLK)	+= clk-conf.o
endif

# KUnit specific helpers
ifeq ($(CONFIG_COMMON_CLK), y)
obj-$(CONFIG_KUNIT)		+= clk_kunit_helpers.o
endif

# hardware specific clock types
# please keep this section sorted lexicographically by file path name
obj-$(CONFIG_COMMON_CLK_APPLE_NCO)  	+= clk-apple-nco.o
obj-$(CONFIG_COMMON_CLK_ALTERA_A10_FPLL) += clk-altera-a10-fpll.o
obj-$(CONFIG_COMMON_CLK_ALTERA_C5_FPLL) += clk-altera-c5-fpll.o
obj-$(CONFIG_MACH_ASM9260)		+= clk-asm9260.o
obj-$(CONFIG_COMMON_CLK_AXI_CLKGEN)	+= clk-axi-clkgen.o
obj-$(CONFIG_ARCH_AXXIA)		+= clk-axm5516.o
obj-$(CONFIG_COMMON_CLK_BD718XX)	+= clk-bd718x7.o
obj-$(CONFIG_COMMON_CLK_BM1880)		+= clk-bm1880.o
obj-$(CONFIG_COMMON_CLK_CDCE706)	+= clk-cdce706.o
obj-$(CONFIG_COMMON_CLK_CDCE925)	+= clk-cdce925.o
obj-$(CONFIG_ARCH_CLPS711X)		+= clk-clps711x.o
obj-$(CONFIG_COMMON_CLK_CS2000_CP)	+= clk-cs2000-cp.o
obj-$(CONFIG_COMMON_CLK_EP93XX)		+= clk-ep93xx.o
obj-$(CONFIG_ARCH_SPARX5)		+= clk-sparx5.o
obj-$(CONFIG_COMMON_CLK_EN7523)		+= clk-en7523.o
obj-$(CONFIG_COMMON_CLK_FIXED_MMIO)	+= clk-fixed-mmio.o
obj-$(CONFIG_COMMON_CLK_FSL_FLEXSPI)	+= clk-fsl-flexspi.o
obj-$(CONFIG_COMMON_CLK_FSL_SAI)	+= clk-fsl-sai.o
obj-$(CONFIG_COMMON_CLK_GEMINI)		+= clk-gemini.o
obj-$(CONFIG_COMMON_CLK_ASPEED)		+= clk-aspeed.o
obj-$(CONFIG_MACH_ASPEED_G6)		+= clk-ast2600.o
obj-$(CONFIG_ARCH_HIGHBANK)		+= clk-highbank.o
obj-$(CONFIG_CLK_HSDK)			+= clk-hsdk-pll.o
obj-$(CONFIG_COMMON_CLK_K210)		+= clk-k210.o
obj-$(CONFIG_LMK04832)			+= clk-lmk04832.o
obj-$(CONFIG_COMMON_CLK_LAN966X)	+= clk-lan966x.o
obj-$(CONFIG_COMMON_CLK_LOCHNAGAR)	+= clk-lochnagar.o
obj-$(CONFIG_MACH_LOONGSON32)		+= clk-loongson1.o
obj-$(CONFIG_COMMON_CLK_LOONGSON2)	+= clk-loongson2.o
obj-$(CONFIG_COMMON_CLK_MAX77686)	+= clk-max77686.o
obj-$(CONFIG_COMMON_CLK_MAX9485)	+= clk-max9485.o
obj-$(CONFIG_ARCH_MILBEAUT_M10V)	+= clk-milbeaut.o
obj-$(CONFIG_ARCH_MOXART)		+= clk-moxart.o
obj-$(CONFIG_ARCH_NOMADIK)		+= clk-nomadik.o
obj-$(CONFIG_ARCH_NPCM7XX)	    	+= clk-npcm7xx.o
obj-$(CONFIG_ARCH_NSPIRE)		+= clk-nspire.o
obj-$(CONFIG_COMMON_CLK_PALMAS)		+= clk-palmas.o
obj-$(CONFIG_CLK_LS1028A_PLLDIG)	+= clk-plldig.o
obj-$(CONFIG_COMMON_CLK_PROXO)		+= clk-proxo.o
obj-$(CONFIG_COMMON_CLK_PWM)		+= clk-pwm.o
obj-$(CONFIG_CLK_QORIQ)			+= clk-qoriq.o
obj-$(CONFIG_COMMON_CLK_RK808)		+= clk-rk808.o
obj-$(CONFIG_COMMON_CLK_HI655X)		+= clk-hi655x.o
obj-$(CONFIG_COMMON_CLK_S2MPS11)	+= clk-s2mps11.o
obj-$(CONFIG_COMMON_CLK_SCMI)           += clk-scmi.o
obj-$(CONFIG_COMMON_CLK_SCPI)           += clk-scpi.o
obj-$(CONFIG_COMMON_CLK_SI5341)		+= clk-si5341.o
obj-$(CONFIG_COMMON_CLK_SI5351)		+= clk-si5351.o
obj-$(CONFIG_COMMON_CLK_SI514)		+= clk-si514.o
obj-$(CONFIG_COMMON_CLK_SI544)		+= clk-si544.o
obj-$(CONFIG_COMMON_CLK_SI570)		+= clk-si570.o
obj-$(CONFIG_COMMON_CLK_SP7021)		+= clk-sp7021.o
obj-$(CONFIG_COMMON_CLK_STM32F)		+= clk-stm32f4.o
obj-$(CONFIG_COMMON_CLK_STM32H7)	+= clk-stm32h7.o
obj-$(CONFIG_COMMON_CLK_TPS68470)      += clk-tps68470.o
clk-si5324drv-y				:= si5324drv.o clk-si5324.o
obj-$(CONFIG_COMMON_CLK_SI5324)		+= clk-si5324drv.o
obj-$(CONFIG_CLK_TWL6040)		+= clk-twl6040.o
obj-$(CONFIG_CLK_TWL)			+= clk-twl.o
obj-$(CONFIG_ARCH_VT8500)		+= clk-vt8500.o
obj-$(CONFIG_COMMON_CLK_RS9_PCIE)	+= clk-renesas-pcie.o
obj-$(CONFIG_COMMON_CLK_SI521XX)	+= clk-si521xx.o
obj-$(CONFIG_COMMON_CLK_VC3)		+= clk-versaclock3.o
obj-$(CONFIG_COMMON_CLK_VC5)		+= clk-versaclock5.o
obj-$(CONFIG_COMMON_CLK_VC7)		+= clk-versaclock7.o
obj-$(CONFIG_COMMON_CLK_WM831X)		+= clk-wm831x.o
obj-$(CONFIG_COMMON_CLK_XGENE)		+= clk-xgene.o

# please keep this section sorted lexicographically by directory path name
obj-y					+= actions/
obj-$(CONFIG_COMMON_CLK_ADI)		+= adi/
obj-y					+= analogbits/
obj-$(CONFIG_COMMON_CLK_AT91)		+= at91/
obj-$(CONFIG_ARCH_ARTPEC)		+= axis/
obj-$(CONFIG_ARC_PLAT_AXS10X)		+= axs10x/
obj-$(CONFIG_CLK_BAIKAL_T1)		+= baikal-t1/
obj-y					+= bcm/
obj-$(CONFIG_ARCH_BERLIN)		+= berlin/
obj-$(CONFIG_ARCH_DAVINCI)		+= davinci/
obj-$(CONFIG_ARCH_HISI)			+= hisilicon/
obj-$(CONFIG_COMMON_CLK_IDT8T49N24X)	+= idt/
obj-y					+= imgtec/
obj-y					+= imx/
obj-y					+= ingenic/
obj-$(CONFIG_ARCH_K3)			+= keystone/
obj-$(CONFIG_ARCH_KEYSTONE)		+= keystone/
obj-y					+= mediatek/
obj-$(CONFIG_ARCH_MESON)		+= meson/
obj-y					+= microchip/
ifeq ($(CONFIG_COMMON_CLK), y)
obj-$(CONFIG_ARCH_MMP)			+= mmp/
endif
obj-y					+= mstar/
obj-y					+= mvebu/
obj-$(CONFIG_ARCH_MXS)			+= mxs/
obj-$(CONFIG_ARCH_MA35)			+= nuvoton/
obj-$(CONFIG_COMMON_CLK_NXP)		+= nxp/
obj-$(CONFIG_COMMON_CLK_PISTACHIO)	+= pistachio/
obj-$(CONFIG_COMMON_CLK_PXA)		+= pxa/
obj-$(CONFIG_COMMON_CLK_QCOM)		+= qcom/
obj-y					+= ralink/
obj-y					+= renesas/
obj-$(CONFIG_ARCH_ROCKCHIP)		+= rockchip/
obj-$(CONFIG_COMMON_CLK_SAMSUNG)	+= samsung/
obj-$(CONFIG_CLK_SIFIVE)		+= sifive/
obj-y					+= socfpga/
obj-y					+= sophgo/
obj-$(CONFIG_PLAT_SPEAR)		+= spear/
obj-y					+= sprd/
obj-$(CONFIG_ARCH_STI)			+= st/
obj-$(CONFIG_ARCH_STM32)		+= stm32/
obj-y					+= starfive/
obj-$(CONFIG_ARCH_SUNXI)		+= sunxi/
obj-y					+= sunxi-ng/
obj-$(CONFIG_ARCH_TEGRA)		+= tegra/
obj-$(CONFIG_ARCH_THEAD)		+= thead/
obj-y					+= ti/
obj-$(CONFIG_CLK_UNIPHIER)		+= uniphier/
obj-$(CONFIG_ARCH_U8500)		+= ux500/
obj-y					+= versatile/
obj-$(CONFIG_COMMON_CLK_VISCONTI)	+= visconti/
ifeq ($(CONFIG_COMMON_CLK), y)
obj-$(CONFIG_X86)			+= x86/
endif
obj-y					+= xilinx/
obj-$(CONFIG_ARCH_ZYNQ)			+= zynq/
obj-$(CONFIG_COMMON_CLK_ZYNQMP)         += zynqmp/
