#
# Makefile for IIO ADC drivers
#

# When adding new entries keep the list in alphabetical order
obj-$(CONFIG_AD_SIGMA_DELTA) += ad_sigma_delta.o
obj-$(CONFIG_AD7266) += ad7266.o
obj-$(CONFIG_AD7298) += ad7298.o
obj-$(CONFIG_AD7923) += ad7923.o
obj-$(CONFIG_AD7476) += ad7476.o
obj-$(CONFIG_AD7791) += ad7791.o
obj-$(CONFIG_AD7793) += ad7793.o
obj-$(CONFIG_AD7887) += ad7887.o
obj-$(CONFIG_AT91_ADC) += at91_adc.o

cf_axi_adc-y := cf_axi_adc_core.o cf_axi_adc_ring.o cf_axi_adc_ring_stream.o
obj-$(CONFIG_CF_AXI_ADC) += cf_axi_adc.o ad9467.o ad9361.o admc_adc.o admc_speed.o admc_ctrl.o ad_adc.o
obj-$(CONFIG_CF_AXI_FFT) += cf_axi_fft_core.o

obj-$(CONFIG_AXI_JESD204B) += cf_axi_jesd204b.o cf_axi_jesd204b_v51.o cf_axi_jesd204b_gt.o


obj-$(CONFIG_AT91_ADC) += at91_adc.o
obj-$(CONFIG_EXYNOS_ADC) += exynos_adc.o
obj-$(CONFIG_LP8788_ADC) += lp8788_adc.o
obj-$(CONFIG_MAX1363) += max1363.o
obj-$(CONFIG_MCP320X) += mcp320x.o
obj-$(CONFIG_MCP3422) += mcp3422.o
obj-$(CONFIG_MEN_Z188_ADC) += men_z188_adc.o
obj-$(CONFIG_NAU7802) += nau7802.o
obj-$(CONFIG_TI_ADC081C) += ti-adc081c.o
obj-$(CONFIG_TI_AM335X_ADC) += ti_am335x_adc.o
obj-$(CONFIG_TWL4030_MADC) += twl4030-madc.o
obj-$(CONFIG_TWL6030_GPADC) += twl6030-gpadc.o
obj-$(CONFIG_VF610_ADC) += vf610_adc.o
obj-$(CONFIG_VIPERBOARD_ADC) += viperboard_adc.o
xilinx-xadc-y := xilinx-xadc-core.o xilinx-xadc-events.o
obj-$(CONFIG_XILINX_XADC) += xilinx-xadc.o
