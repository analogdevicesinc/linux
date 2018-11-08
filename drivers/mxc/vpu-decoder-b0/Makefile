##
## Makefile for the VPU and M0 driver
##
DEFINES += -D REBOOT=1 \
		   -D BOOT_ARCH=1 \
		   -D DYNAMIC_MEM

EXTRA_CFLAGS += $(DEFINES)

obj-y = vpu-decoder.o
vpu-decoder-objs = vpu_b0.o \
	vpu_rpc.o \
	insert_startcode.o

clean:
	rm -rf $(vpu-decoder-objs)
