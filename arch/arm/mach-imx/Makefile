# SPDX-License-Identifier: GPL-2.0
obj-y := cpu.o system.o irq-common.o common.o

obj-$(CONFIG_SOC_IMX25) += cpu-imx25.o mach-imx25.o pm-imx25.o

obj-$(CONFIG_SOC_IMX27) += cpu-imx27.o pm-imx27.o mach-imx27.o

obj-$(CONFIG_SOC_IMX31) += mm-imx3.o cpu-imx31.o mach-imx31.o
obj-$(CONFIG_SOC_IMX35) += mm-imx3.o cpu-imx35.o mach-imx35.o

imx5-pm-$(CONFIG_PM) += pm-imx5.o
obj-$(CONFIG_SOC_IMX5) += cpu-imx5.o $(imx5-pm-y)

obj-$(CONFIG_MXC_TZIC) += tzic.o
obj-$(CONFIG_MXC_AVIC) += avic.o

ifeq ($(CONFIG_CPU_IDLE),y)
obj-$(CONFIG_SOC_IMX5) += cpuidle-imx5.o
obj-$(CONFIG_SOC_IMX6Q) += cpuidle-imx6q.o
AFLAGS_imx6sl_low_power_idle.o :=-Wa,-march=armv7-a
obj-$(CONFIG_SOC_IMX6SL) += cpuidle-imx6sl.o imx6sl_low_power_idle.o
AFLAGS_imx6sll_low_power_idle.o :=-Wa,-march=armv7-a
obj-$(CONFIG_SOC_IMX6SLL) += cpuidle-imx6sll.o imx6sll_low_power_idle.o
obj-$(CONFIG_SOC_IMX6SX) += cpuidle-imx6sx.o imx6sx_low_power_idle.o
AFLAGS_imx6sx_low_power_idle.o :=-Wa,-march=armv7-a
AFLAGS_imx6ul_low_power_idle.o :=-Wa,-march=armv7-a
AFLAGS_imx6ull_low_power_idle.o :=-Wa,-march=armv7-a
obj-$(CONFIG_SOC_IMX6UL) += cpuidle-imx6ul.o imx6ul_low_power_idle.o imx6ull_low_power_idle.o
obj-$(CONFIG_SOC_IMX7ULP) += cpuidle-imx7ulp.o pm-rpmsg.o
AFLAGS_imx7d_low_power_idle.o :=-Wa,-march=armv7-a
obj-$(CONFIG_SOC_IMX7D_CA7) += cpuidle-imx7d.o imx7d_low_power_idle.o
endif

ifdef CONFIG_SND_SOC_IMX_PCM_FIQ
obj-y += ssi-fiq.o
obj-y += ssi-fiq-ksym.o
endif

obj-$(CONFIG_HAVE_IMX_ANATOP) += anatop.o
obj-$(CONFIG_HAVE_IMX_GPC) += gpc.o
obj-$(CONFIG_HAVE_IMX_GPCV2) += gpcv2.o
obj-$(CONFIG_HAVE_IMX_MMDC) += mmdc.o
obj-$(CONFIG_HAVE_IMX_SRC) += src.o
obj-$(CONFIG_HAVE_IMX_DDRC) += ddrc.o
obj-$(CONFIG_HAVE_IMX_MU) += mu.o
ifneq ($(CONFIG_SOC_IMX6)$(CONFIG_SOC_LS1021A),)
obj-$(CONFIG_SMP) += headsmp.o platsmp.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
endif
obj-$(CONFIG_SOC_IMX6Q) += mach-imx6q.o ddr3_freq_imx6.o smp_wfe_imx6.o \
				lpddr2_freq_imx6q.o
obj-$(CONFIG_SOC_IMX6SL) += mach-imx6sl.o lpddr2_freq_imx6.o
obj-$(CONFIG_SOC_IMX6SLL) += mach-imx6sl.o lpddr2_freq_imx6sll.o
obj-$(CONFIG_SOC_IMX6SX) += mach-imx6sx.o ddr3_freq_imx6sx.o smp_wfe_imx6.o lpddr2_freq_imx6sx.o
obj-$(CONFIG_SOC_IMX6UL) += mach-imx6ul.o
obj-$(CONFIG_SOC_IMX7D_CA7) += mach-imx7d.o pm-imx7.o ddr3_freq_imx7d.o smp_wfe.o \
			   lpddr3_freq_imx.o suspend-imx7.o
obj-$(CONFIG_SOC_IMX7D_CM4) += mach-imx7d-cm4.o
obj-$(CONFIG_SOC_IMX7ULP) += mach-imx7ulp.o pm-imx7ulp.o

obj-y += busfreq-imx.o busfreq_ddr3.o busfreq_lpddr2.o
AFLAGS_smp_wfe.o :=-Wa,-march=armv7-a
AFLAGS_smp_wfe_imx6.o :=-Wa,-march=armv7-a
AFLAGS_ddr3_freq_imx7d.o :=-Wa,-march=armv7-a
AFLAGS_lpddr3_freq_imx.o :=-Wa,-march=armv7-a
AFLAGS_ddr3_freq_imx6.o :=-Wa,-march=armv7-a
AFLAGS_lpddr2_freq_imx6.o :=-Wa,-march=armv7-a
AFLAGS_lpddr2_freq_imx6q.o :=-Wa,-march=armv7-a
AFLAGS_lpddr2_freq_imx6sx.o :=-Wa,-march=armv7-a
AFLAGS_lpddr2_freq_imx6sll.o :=-Wa,-march=armv7-a
AFLAGS_ddr3_freq_imx6sx.o :=-Wa,-march=armv7-a

ifeq ($(CONFIG_SUSPEND),y)
obj-$(CONFIG_SOC_IMX6) += suspend-imx6.o
obj-$(CONFIG_SOC_IMX53) += suspend-imx53.o
obj-$(CONFIG_SOC_IMX7ULP) += suspend-imx7ulp.o
endif
ifeq ($(CONFIG_ARM_CPU_SUSPEND),y)
obj-$(CONFIG_SOC_IMX6) += resume-imx6.o
endif
obj-$(CONFIG_SOC_IMX6) += pm-imx6.o

obj-$(CONFIG_SOC_IMX1) += mach-imx1.o
obj-$(CONFIG_SOC_IMX50) += mach-imx50.o
obj-$(CONFIG_SOC_IMX51) += mach-imx51.o
obj-$(CONFIG_SOC_IMX53) += mach-imx53.o

obj-$(CONFIG_SOC_IMXRT) += mach-imxrt.o

obj-$(CONFIG_SOC_VF610) += mach-vf610.o

obj-$(CONFIG_SOC_LS1021A) += mach-ls1021a.o

ifneq ($(CONFIG_SOC_IMX6)$(CONFIG_SOC_IMX7),)
# Bus frequency by OPTEE OS
obj-$(CONFIG_OPTEE) += busfreq_optee.o
endif
