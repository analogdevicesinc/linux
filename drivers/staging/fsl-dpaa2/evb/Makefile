
obj-$(CONFIG_FSL_DPAA2_EVB) += dpaa2-evb.o

dpaa2-evb-objs := evb.o dpdmux.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
