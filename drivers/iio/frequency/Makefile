# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile iio/frequency
#

# When adding new entries keep the list in alphabetical order
obj-$(CONFIG_AD9508) += ad9508.o
obj-$(CONFIG_AD9517) += ad9517.o
obj-$(CONFIG_AD9523) += ad9523.o
obj-$(CONFIG_AD9528) += ad9528.o
obj-$(CONFIG_AD9548) += ad9548.o
obj-$(CONFIG_ADF4030) += adf4030.o
obj-$(CONFIG_ADF4159) += adf4159.o
obj-$(CONFIG_ADF4350) += adf4350.o
obj-$(CONFIG_ADF4360) += adf4360.o
obj-$(CONFIG_ADF4371) += adf4371.o
obj-$(CONFIG_ADF4377) += adf4377.o
obj-$(CONFIG_ADMFM2000) += admfm2000.o
obj-$(CONFIG_ADMV1013) += admv1013.o
obj-$(CONFIG_ADMV1014) += admv1014.o
obj-$(CONFIG_ADMV4420) += admv4420.o
obj-$(CONFIG_ADRF6780) += adrf6780.o
obj-$(CONFIG_ADF5355) += adf5355.o
obj-$(CONFIG_ADL5960) += adl5960.o

cf_axi_dds_drv-y := cf_axi_dds.o cf_axi_dds_buffer_stream.o
obj-$(CONFIG_CF_AXI_DDS) += cf_axi_dds_drv.o
obj-$(CONFIG_CF_AXI_DDS_AD9122) += ad9122.o
obj-$(CONFIG_CF_AXI_DDS_AD9144) += ad9144.o
obj-$(CONFIG_CF_AXI_DDS_AD9783) += ad9783.o
obj-$(CONFIG_HMC7044) += hmc7044.o
obj-$(CONFIG_LTC6952) += ltc6952.o

ad916x_drv-y := ad9162.o ad916x/ad916x_api.o  ad916x/ad916x_irq_api.o  ad916x/ad916x_jesd_api.o  ad916x/ad916x_jesd_test_api.o  ad916x/ad916x_nco_api.o  ad916x/ad916x_reg.o  ad916x/api_errors.o  ad916x/utils.o
obj-$(CONFIG_CF_AXI_DDS_AD9162) += ad916x_drv.o

ad917x_drv-y := ad9172.o ad917x/ad917x_api.o  ad917x/ad917x_jesd_api.o  ad917x/ad917x_nco_api.o  ad917x/ad917x_reg.o
obj-$(CONFIG_CF_AXI_DDS_AD9172) += ad917x_drv.o
obj-$(CONFIG_M2K_DAC) += m2k-dac.o

