##
## Makefile for the VPU and M0 driver
##
DEFINES += -D REBOOT=1 \
		   -D BOOT_ARCH=1

EXTRA_CFLAGS += $(DEFINES)

obj-y = vpu-decoder.o
vpu-decoder-objs = vpu_b0.o \
	vpu_rpc.o \
	insert_startcode.o \
	vpu_debug_log.o

clean:
	rm -rf $(vpu-decoder-objs)
