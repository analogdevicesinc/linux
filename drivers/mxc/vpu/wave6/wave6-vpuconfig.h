/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - product config definitions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef _VPU_CONFIG_H_
#define _VPU_CONFIG_H_

#define WAVE617_CODE                    0x6170
#define WAVE627_CODE                    0x6270
#define WAVE633_CODE                    0x6330
#define WAVE637_CODE                    0x6370
#define WAVE663_CODE                    0x6630
#define WAVE677_CODE                    0x6670

#define PRODUCT_CODE_W_SERIES(x) ({					\
		int c = x;						\
		((c) == WAVE617_CODE ||	(c) == WAVE627_CODE ||		\
		 (c) == WAVE633_CODE || (c) == WAVE637_CODE ||		\
		 (c) == WAVE663_CODE || (c) == WAVE677_CODE);		\
})

#define WAVE627ENC_WORKBUF_SIZE         (512 * 1024)
#define WAVE637DEC_WORKBUF_SIZE         (2 * 512 * 1024)
#define WAVE637DEC_WORKBUF_SIZE_FOR_CQ  (3 * 512 * 1024)

#define MAX_NUM_INSTANCE                32

#define W6_MAX_PIC_STRIDE               (4096U*4)
#define W6_DEF_DEC_PIC_WIDTH            720U
#define W6_DEF_DEC_PIC_HEIGHT           480U
#define W6_MIN_DEC_PIC_WIDTH            64U
#define W6_MIN_DEC_PIC_HEIGHT           64U
#define W6_MAX_DEC_PIC_WIDTH            4096U
#define W6_MAX_DEC_PIC_HEIGHT           2304U
#define W6_DEC_PIC_SIZE_STEP            1

#define W6_DEF_ENC_PIC_WIDTH            416U
#define W6_DEF_ENC_PIC_HEIGHT           240U
#define W6_MIN_ENC_PIC_WIDTH            256U
#define W6_MIN_ENC_PIC_HEIGHT           128U
#define W6_MAX_ENC_PIC_WIDTH            4096U
#define W6_MAX_ENC_PIC_HEIGHT           2304U
#define W6_ENC_PIC_SIZE_STEP            8

//  application specific configuration
#define W6_VPU_TIMEOUT                  6000
#define VPU_ENC_TIMEOUT                 6000
#define VPU_DEC_TIMEOUT                 6000

#define HOST_ENDIAN                     VDI_128BIT_LITTLE_ENDIAN
#define VPU_FRAME_ENDIAN                HOST_ENDIAN
#define VPU_STREAM_ENDIAN               HOST_ENDIAN
#define VPU_USER_DATA_ENDIAN            HOST_ENDIAN
#define VPU_SOURCE_ENDIAN               HOST_ENDIAN

// for WAVE encoder
#define USE_SRC_PRP_AXI         0
#define USE_SRC_PRI_AXI         1
#define DEFAULT_SRC_AXI         USE_SRC_PRP_AXI

/************************************************************************/
/* VPU COMMON MEMORY                                                    */
/************************************************************************/
#define COMMAND_QUEUE_DEPTH             (1)

#define W6_REMAP_INDEX0                 0
#define W6_REMAP_INDEX1                 1
#define W6_REMAP_MAX_SIZE               (1024 * 1024)

#define WAVE6_ARBUF_SIZE                (1024)
#define WAVE6_MAX_CODE_BUF_SIZE         (4 * 1024 * 1024)
#define WAVE6_EXTRA_CODE_BUF_SIZE       (256 * 1024)
#define WAVE6_TEMPBUF_SIZE              (3 * 1024 * 1024)
#define WAVE6_VUI_BUF_SIZE              (1024)

#define SIZE_COMMON                     (W6_REMAP_MAX_SIZE)

//=====4. VPU REPORT MEMORY  ======================//
#define WAVE6_UPPER_PROC_AXI_ID     0x0

#endif  /* _VPU_CONFIG_H_ */
