##
## Makefile for the VPU and M0 driver
##

EXTRA_CFLAGS += $(DEFINES)

obj-y = vpu-encoder.o
vpu-encoder-objs = vpu_encoder_b0.o \
	vpu_encoder_ctrl.o \
	vpu_event_msg.o \
	vpu_encoder_rpc.o

clean:
	rm -rf $(vpu-encoder-objs)
