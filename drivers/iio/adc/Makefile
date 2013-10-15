#
# Makefile for IIO ADC drivers
#

obj-$(CONFIG_AD_SIGMA_DELTA) += ad_sigma_delta.o
obj-$(CONFIG_AD7266) += ad7266.o
obj-$(CONFIG_AD7298) += ad7298.o
obj-$(CONFIG_AD7923) += ad7923.o
obj-$(CONFIG_AD7476) += ad7476.o
obj-$(CONFIG_AD7791) += ad7791.o
obj-$(CONFIG_AD7793) += ad7793.o
obj-$(CONFIG_AD7887) += ad7887.o
obj-$(CONFIG_AT91_ADC) += at91_adc.o

cf_axi_adc-y := cf_axi_adc_core.o cf_axi_adc_ring.o
obj-$(CONFIG_CF_AXI_ADC) += cf_axi_adc.o ad9467.o ad9361.o ad_mc_adc.o ad_mc_speed.o ad_mc_controller.o
obj-$(CONFIG_CF_AXI_FFT) += cf_axi_fft_core.o

obj-$(CONFIG_AXI_JESD204B) += cf_axi_jesd204b.o

obj-$(CONFIG_AT91_ADC) += at91_adc.o
obj-$(CONFIG_EXYNOS_ADC) += exynos_adc.o
obj-$(CONFIG_LP8788_ADC) += lp8788_adc.o
obj-$(CONFIG_MAX1363) += max1363.o
obj-$(CONFIG_TI_ADC081C) += ti-adc081c.o
obj-$(CONFIG_TI_AM335X_ADC) += ti_am335x_adc.o
obj-$(CONFIG_VIPERBOARD_ADC) += viperboard_adc.o
obj-$(CONFIG_XILINX_XADC) += xilinx-xadc.o xilinx-xadc-events.o
