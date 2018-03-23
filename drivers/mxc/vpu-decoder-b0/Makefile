##
## Makefile for the VPU and M0 driver
##
DEFINES += -D HANDLE_EOS \
		   -D REBOOT=1 \
		   -D BOOT_ARCH=0

EXTRA_CFLAGS += $(DEFINES)

obj-y = vpu-decoder.o
vpu-decoder-objs = vpu_b0.o \
	vpu_rpc.o

clean:
	rm -rf $(vpu-decoder-objs)
