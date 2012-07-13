#
# Makefile for IIO ADC drivers
#

obj-$(CONFIG_AD7266) += ad7266.o
obj-$(CONFIG_AT91_ADC) += at91_adc.o

cf_axi_adc-y := cf_axi_adc_core.o cf_axi_adc_ring.o
obj-$(CONFIG_CF_AXI_ADC) += cf_axi_adc.o ad9467.o
obj-$(CONFIG_CF_AXI_FFT) += cf_axi_fft_core.o
