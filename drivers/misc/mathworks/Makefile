#
# Makefile for the AXI4 drivers.
#

obj-$(CONFIG_MATHWORKS_IP_CORE) += mathworks_ip_common.o
obj-$(CONFIG_MATHWORKS_GENERIC_OF) += mathworks_generic_of.o
obj-$(CONFIG_MATHWORKS_GENERIC_PCI) += mathworks_generic_pci.o

obj-$(CONFIG_MWIPCORE) += mwipcore.o
obj-$(CONFIG_MWIPCORE_DMA_STREAMING) += mwipcore_dma_streaming.o
obj-$(CONFIG_MWIPCORE_IIO_STREAMING) += mwipcore_iio_streaming.o
obj-$(CONFIG_MWIPCORE_IIO_MM) += mwipcore_iio_mm.o
obj-$(CONFIG_MWIPCORE_IIO_SHAREDMEM) += mwipcore_iio_sharedmem.o

mwipcore_dma_streaming-objs := mw_stream_channel.o
mwipcore_iio_streaming-objs := mw_stream_iio_channel.o
mwipcore_iio_mm-objs := mw_mm_iio_channel.o
mwipcore_iio_sharedmem-objs := mw_sharedmem_iio_channel.o
mwipcore-objs := mathworks_ipcore.o

