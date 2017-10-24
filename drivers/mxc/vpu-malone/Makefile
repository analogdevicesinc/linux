#
# Makefile for VPU driver.
#

MALONE_ROOT = $(srctree)/drivers/mxc/vpu-malone/Malone_Firmware

#---------------------------------------------------------------------------
# Header Include Paths

MALONE_KERN_HEADER = \
	      -I$(MALONE_ROOT)/DecKLib/KernelIF \
	      -I$(MALONE_ROOT)/DecKLib/Control \
	      -I$(MALONE_ROOT)/DecKLib/Incl

SYS_HEADER = -I$(MALONE_ROOT)/Incl \
	     -I$(MALONE_ROOT)/PAL/Incl \
	     -I$(srctree)/drivers/mxc/vpu-malone

MALONE_KERN_HEADER += $(SYS_HEADER)


#---------------------------------------------------------------------------
# Actual Source and Library control

MAL_KERN_OBJECTS = \
		Malone_Firmware/DecKLib/KernelIF/DecKernelLib.o \
		Malone_Firmware/DecKLib/KernelIF/DecKernelLibPrivate.o \
		Malone_Firmware/DecKLib/Control/DecKernelLibHWControl.o \
		Malone_Firmware/DecKLib/Control/DecKernelLibHWIsr.o 

MAL_KERN_OBJECTS += Malone_Firmware/PAL/pal.o

#---------------------------------------------------------------------------
# Build flags

DEFINES = -DDTV_GATHER_PERF_METRICS \
		  -DMVD_DTV_USERDATA \
		  -DMVD_WAIT_BOB_INACTIVE \
		  -DDECLIB_FORCE_HW_STOP \
		  -DMVD_NO_BSDMA_SAFETY_MARGIN \
		  -DMVD_CQ_ENABLE_REFILL \
		  -DMVD_SPP_HW_GOULOMB \
		  -DSVC_SFA_ADD_ERROR_CHECKING \
		  -DMVC_SFA_ADD_ERROR_CHECKING \
		  -DSVC_SPP_SAVE_CTX_PER_VCL_NAL \
		  -DMVD_CQ_CQSR \
		  -DAVC_SUPPORT_THRU_MVC \
		  -DMVC_ERROR_CONTROL_INSERT_SKIP_START_CONTROLS \
		  -DDECLIB_CTX_FLUSH_AFTER_SAVE \
		  -DDECLIB_SERVICE_EOS \
		  -DMVD_PERF_MEASURE \
		  -DVC1_ENABLED \
		  -DHEVC_ENABLED \
		  -DHEVC_CM_WORKAROUND \
		  -DHEVC_NEW_OUTPUT_TRIGGER \
		  -DHEVC_ALL_PICS_REF \
		  -DHEVC_SCAL_LIST_USE_YCRCB_XREF \
		  -DHEVC_SFA_ADD_ERROR_CHECKING \
		  -DMVD_DFE_DBG \
		  -DHEVC_JVT_MODEL=100 \
		  -DPAL_CLOCK_API \
		  -DSVC_ADDITIONAL_DEBUG \
		  -DDIAG_SUPPORT_ENABLED \
		  -DENABLE_PERF_TIMER \
		  -DFW_API_VERSION=19 \
		  -DGLOBAL_USE_RUN_TIME_CFG \
		  -DENABLE_TRACE_IN_RELEASE=0 \
		  -DYES=1 \
		  -DNO=0 \
		  -DNONE=0 \
		  -DNUP=1 \
		  -DUCOS=2 \
		  -DUCOS3=3 \
		  -DRTOS=0 \
		  -DUSE_DECODER \
		  -DARM=0 \
		  -DMIPS=1 \
		  -DX86=2 \
		  -DOR1K=3 \
		  -DCPU=0 \
		  -DNO_AL=0 \
		  -DCNXT_KAL=1 \
		  -DNXP_OSAL=2 \
		  -DOSAL=0 \
		  -DARM926=0 \
		  -DARMR5=1 \
		  -DARMA53=2 \
		  -DARM_CPU_TYPE=2 \
		  -DADS=0 \
		  -DRVDS=1 \
		  -DGNU_MIPS=2 \
		  -DGNU_MIPS_LNX=3 \
		  -DGNU_ARM=4 \
		  -DGNU_ARM_SOURCERY=5 \
		  -DGNU_X86=6 \
		  -DWIN_X86=7 \
		  -DDS5=8 \
		  -DGNU_OR32=9 \
		  -DGNU_ARM_LINARO=10 \
		  -DGNU_OR1K=11 \
		  -DTOOLSET=10 \
		  -DNO_DEBUG=0 \
		  -DBUILD_DEBUG=1 \
		  -DARRAY_DEBUG=2 \
		  -DFULL_DEBUG=3 \
		  -DDEBUG_CAPS=0 \
		  -DGENTB_PLATFORM=0 \
		  -DWIN_LIB=1 \
		  -DGEN_TB_ENC=2 \
		  -DTARGET_PLATFORM=0 \
		  -DVIDEO_TRANS=0 \
		  -DGTB_TRANS=1 \
		  -DGTB_DEC=2 \
		  -DWINDSOR_LIB=3 \
		  -DGTB_ENC=4 \
		  -DMEDIA_DEC=5 \
		  -DMEDIA_LIB=6 \
                  -DVPU_TEST_APP=7 \
		  -DTARGET_APP=7 \
		  -DPAL_CLOCK_API \
		  -DSVC_ADDITIONAL_DEBUG \
		  -DDIAG_SUPPORT_ENABLED \
		  -DENABLE_PERF_TIMER \
		  -DFW_API_VERSION=19 \
		  -DGLOBAL_USE_RUN_TIME_CFG \
		  -DDISABLE_TRACE \
		  -DENABLE_TRACE_IN_RELEASE=0 \
		  -DYES=1 \
		  -DNO=0 \
		  -DNONE=0 \
		  -DNUP=1 \
		  -DUCOS=2 \
		  -DUCOS3=3 \
		  -DRTOS=0 \
		  -DUSE_DECODER \
		  -DCHIP=0 \
		  -DEMULATION=1 \
		  -DHAPS=2 \
		  -DSIMULATION=3 \
		  -DCMODEL=4 \
		  -DTARGET_LEVEL=0 \
		  -DSVC_DISABLED=0 \
		  -DSVC_ENABLED=1 \
		  -DSVC_SUPPORT=0 \
		  -DMVC_DISABLED=0 \
		  -DMVC_ENABLED=1 \
		  -DMVC_SUPPORT=1 \
		  -DSFA_DISABLED=0 \
		  -DSFA_ENABLED=1 \
		  -DSFA_SUPPORT=1 \
		  -DCNXT_HW=0 \
		  -DNXP_HW=1 \
		  -DHWLIB=1 \
		  -DDTV=0 \
		  -DSTB=1 \
		  -DPLAYMODE=0 \
		  -DSTANDARD=0 \
		  -DREBOOT=1 \
		  -DBOOT_ARCH=0 \
		  -DTBPLAYER_FLOW_CHANGE_ON_REF_FRMS \
		  -DPULSAR_MERGE \
		  -DFSLCACHE_ENABLED \
		  -DDECLIB_ENABLE_DFE -DDECLIB_ENABLE_DBE \
		  -DDECLIB_ENABLE_DCP -DMVD_DCP_DYNAMIC_CONFIG \
		  -DDECLIB_4K_SUPPORTED -DHEVC_LEVEL_5PT0_SUPPORT \
		  -DPLAYER_LOCAL_THREAD \
		  -DDECLIB_ISR_IN_THREAD_CTX \
		  -DJPG_ENABLED \
		  -DJPGD_AUTO_DOWN_SCALE \
		  -DSPARK_ENABLED \
		  -DRV_ENABLED \
		  -DVP6_ENABLED \
		  -DVP8_ENABLED \
		  -DJPG_DPV_ENABLED \
		  -DMALONE_64BIT_ADDR \
		  -DDISABLE_TRACE

MALONE_KERN_DEFINEFLAGS = $(DEFINES)
MALONE_KERN_DEFINEFLAGS += -DVPU_KERNEL_BUILD

EXTRA_CFLAGS += $(MALONE_KERN_DEFINEFLAGS)
EXTRA_CFLAGS += $(MALONE_KERN_HEADER)

ifeq ($(CONFIG_MXC_VPU_MALONE_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

obj-$(CONFIG_MXC_VPU_MALONE) = decoder.o

decoder-objs = mxc_vpu-malone.o \
	       $(MAL_KERN_OBJECTS)

cmd_files := $(foreach f,$(decoder-objs),$(dir $(f)).$(notdir $(f)).cmd)
clean:
	rm -rf $(decoder-objs) $(cmd_files) *.o .*.cmd modules.builtin modules.order


