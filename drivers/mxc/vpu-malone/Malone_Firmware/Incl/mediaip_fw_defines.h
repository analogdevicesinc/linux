/***************************************************
  Copyright (c) 2015 Amphion Semiconductor Ltd
                All rights reserved.
 ***************************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************

  Filename:        mediaip_fw_defines.h
  Description:     Contains general definitions for
                   module based architecture
  Author:          Media IP FW team - Belfast

  TODO-KMC - Should remove this file!!
 **************************************************/

#ifndef _MEDIAIP_FW_DEFINES_H_
#define _MEDIAIP_FW_DEFINES_H_

/* In use MACROS for use by types shared across modules                                        */
/* When adding new modules, endeavour to maintain the same 'position' as that suggested by the */
/* module's position in the MEDIAIP_FW_HANDLE_TYPE enumeration - see Modules/Public/handle.h   */

#define IN_USE_DECODER   ( 1 << 0x0 )
#define IN_USE_ENCODER   ( 1 << 0x1 )
#define IN_USE_IMG_PORT  ( 1 << 0x2 )
#define IN_USE_DISPLAY   ( 1 << 0x3 )
#define IN_USE_VAMUX     ( 1 << 0x4 )
#define IN_USE_VIPP      ( 1 << 0x5 )
#define IN_USE_SYS_API   ( 1 << 0x10 )

/////////////////////////////////////////////////
// Frame Store Display Data make-up

// Read
#define FRAME_STORE_DISP_DATA_FSID(r)                  ((r>>0)&0x3f)
#define FRAME_STORE_DISP_DATA_RPT_FIRST_FLD(r)         ((r>>15)&0x1)	  // IDJ: Pass RFF (borrow a bit from unused base addr field)
#define FRAME_STORE_DISP_DATA_SPS_IDC_BITS(r)          ((r>>16)&0x7)
#define FRAME_STORE_DISP_DATA_DANG_FIELD_BIT(r)        ((r>>19)&0x1)
#define FRAME_STORE_DISP_DATA_FIELD_MODE_BIT(r)        ((r>>20)&0x1)
#define FRAME_STORE_DISP_DATA_BOT_FIRST_BIT(r)         ((r>>21)&0x1)
#define FRAME_STORE_DISP_DATA_SKIP_PIC_BITS(r)         ((r>>22)&0x3)
#define FRAME_STORE_DISP_DATA_SYS_DATA_BITS(r)         ((r>>24)&0xFF)
#define FRAME_STORE_DISP_DATA_FRAME_BADDR(r)           ((r>>0)&0x7FFF)
#define FRAME_STORE_DATAFIELDS(a)				               (a&0xffff8000)	// IDJ - bit 15 used above

// Write
#define FRAME_STORE_DISP_DATA_PUT_FSID(a)                  ((a&0x3f) << 0)
#define FRAME_STORE_DISP_DATA_PUT_FRAME_BADDR(a)			     ((a&0x7fff) << 0)
#define FRAME_STORE_DISP_DATA_PUT_DUPLC_FRAME_SENT(a)      ((a&0x1) << 14)	 // RayC/JKD: but used to control when duplicate frames are sent
#define FRAME_STORE_DISP_DATA_PUT_DUPLC_FRAME_SENT_GET(a)  ((a>>14)&0x1)
#define FRAME_STORE_DISP_DATA_PUT_RPT_FIRST_FIELD(a)       ((a&0x1) << 15)	 // IDJ Borrow a bit from BADDR field (not used)
#define FRAME_STORE_DISP_DATA_PUT_DISP_PARAM_ID(a)         ((a&0x7) << 16)
#define FRAME_STORE_DISP_DATA_PUT_DANGLING_FIELD(a)        ((a&0x1) << 19)
#define FRAME_STORE_DISP_DATA_PUT_FIELD_MODE(a)            ((a&0x1) << 20)
#define FRAME_STORE_DISP_DATA_PUT_BOT_FIELD_FIRST(a)       ((a&0x1) << 21)
#define FRAME_STORE_DISP_DATA_PUT_TOP_SKIPPED(a)           ((a&0x1) << 22)
#define FRAME_STORE_DISP_DATA_PUT_BOT_SKIPPED(a)           ((a&0x1) << 23)
#define FRAME_STORE_DISP_DATA_PUT_SYS_DATA(a)              ((a&0xff) << 24)

#define NUM_DISP_PUSHES_MASK   0xFF00
#define NUM_DISP_PUSHES_POS     8
#define NUM_DISP_PUSHES_SET(x) (x<<NUM_DISP_PUSHES_POS)
#define NUM_DISP_PUSHES_GET(x)  ( (x&NUM_DISP_PUSHES_MASK) >> NUM_DISP_PUSHES_POS)

/////////////////////////////////////////////////
// Display Params

#define DISP_INFO_USER_DATA_ATTACHED_MASK 0x1
#define DISP_INFO_USER_DATA_ATTACHED_POS 0
#define DISP_INFO_GET_USER_DATA_ATTACHED(x) ((x&DISP_INFO_USER_DATA_ATTACHED_MASK)>>DISP_INFO_USER_DATA_ATTACHED_POS)
#define DISP_INFO_USER_DATA_MASK 0xFFFF
#define DISP_INFO_USER_DATA_POS   16
#define DISP_INFO_GET_USER_DATA(x) ((x>>DISP_INFO_USER_DATA_POS)&DISP_INFO_USER_DATA_MASK)

// uScanFormat defines

#define DISP_SCAN_FORMAT_INTERLACED      0x0
#define DISP_SCAN_FORMAT_PROGRESSIVE     0x1
#define DISP_SCAN_FORMAT_VALID_GET(r)    (r&0x1)
#define DISP_SCAN_FORMAT_GET(r)          ((r&0x2)>>1)
#define DISP_SCAN_FORMAT_VALID_SET(r)    ((r&0x1)<<0)
#define DISP_SCAN_FORMAT_SET(r)          ((r&0x1)<<1)

/////////////////////////////////////////////////
// General defines - base on Stream descriptor for Pecos

#define STOP_IMMEDIATE               0x0
#define STOP_COMPLETE_DISPLAY        0x1

#define FORMAT_VC1	                 0x2
#define FORMAT_MPEG2                 0x1
#define FORMAT_MPEG4                 0x0
#define FORMAT_MPEG2_DBEN            0x1
#define FORMAT_MPEG2_DBDRNGEN        0x2

#define STREAM_MODE_ES               0x1
#define STREAM_MODE_PES              0x0

#define DELIVERY_MODE_TSP_DIRECT     0x0
#define DELIVERY_MODE_BSP            0x1
#define DELIVERY_MODE_BSDMA          0x2

// Buffer Indices
#define SD_BUFIND_ESBUF_MASK         0xFF
#define SD_BUFIND_ESBUF_SHIFT        0
#define SD_BUFIND_STC_MASK           0x30000
#define SD_BUFIND_STC_SHIFT          16
#define SD_BUFIND_ESBUF_GET(x)       ((x&SD_BUFIND_ESBUF_MASK)>>SD_BUFIND_ESBUF_SHIFT)
#define SD_BUFIND_STC_GET(x)         ((x&SD_BUFIND_STC_MASK)>>SD_BUFIND_STC_SHIFT)

#define UD_ORDER_DECODE              0x1
#define UD_ORDER_DISPLAY             0x2
#define UD_ORDER_DECODEANDDISPLAY    0x3

#define PRINT_UART4(a,b,c,d)
#define PRINT_UART5(a,b,c,d,e)

#define DISPLAY_ASPECT_RATIO_4_3     0x2
#define DISPLAY_ASPECT_RATIO_16_9    0x3


/////////////////////////////////////////////////
// PTS capture descriptor flags
#define PTS_DESCRIPTOR_PTS_LO                          0x00000000
#define PTS_DESCRIPTOR_DTS_LO                          0x00000004
#define PTS_DESCRIPTOR_FLAGS                           0x00000008
#define PTS_DESCRIPTOR_WRAP_COUNT                      0x00000009
#define PTS_DESCRIPTOR_TIMEBASE_ID                     0x0000000A
#define PTS_DESCRIPTOR_MATURITY_ADDRESS                0x0000000C

#define PTS_DESCRIPTOR_FLAG_PTS_HI_MASK                0x00000001
#define PTS_DESCRIPTOR_FLAG_PTS_HI_BIT                 0
#define PTS_DESCRIPTOR_FLAG_DTS_HI_MASK                0x00000002
#define PTS_DESCRIPTOR_FLAG_DTS_HI_BIT                 1
#define PTS_DESCRIPTOR_FLAG_STC_PARITY_MASK            0x00000004
#define PTS_DESCRIPTOR_FLAG_STC_PARITY_BIT             2
#define PTS_DESCRIPTOR_FLAG_PENDING_MASK               0x00000008
#define PTS_DESCRIPTOR_FLAG_PENDING_BIT                3
#define PTS_DESCRIPTOR_FLAG_PESERROR_MASK              0x00000010
#define PTS_DESCRIPTOR_FLAG_PESERROR_BIT               4

#define PTS_DESCRIPTOR_FLAG_ADDRESS_WRAP_COUNT_BIT     8
#define PTS_DESCRIPTOR_FLAG_ADDRESS_WRAP_COUNT_MASK    0x0000FF00
#define PTS_DESCRIPTOR_FLAG_TIMEBASE_ID_BIT            16
#define PTS_DESCRIPTOR_FLAG_TIMEBASE_ID_MASK           0x000F0000
#define PTS_DESCRIPTOR_FLAG_AVC_TAG_BIT                24
#define PTS_DESCRIPTOR_FLAG_AVC_TAG_MASK               0xFF000000

// PTS debug descriptor extensions
#define PTS_DESCRIPTOR_DEBUG                           0x00000010
#define PTS_DESCRIPTOR_DEBUG_PTS_DTS                   0x00000010
#define PTS_DESCRIPTOR_DEBUG_STC_SNAPSHOT              0x00000014
#define PTS_DESCRIPTOR_DEBUG_DIFFERENCE                0x00000018
#define PTS_DESCRIPTOR_DEBUG_FLAGS                     0x0000001C

#define PTS_DESCRIPTOR_DEBUG_FLAG_NO_PTS               0x00000001
#define PTS_DESCRIPTOR_DEBUG_FLAG_NO_STC               0x00000002
#define PTS_DESCRIPTOR_DEBUG_FLAG_IS_DTS               0x00000004
#define PTS_DESCRIPTOR_DEBUG_FLAG_RUNNING              0x00000008
#define PTS_DESCRIPTOR_DEBUG_FLAG_INSANE               0x00000010
#define PTS_DESCRIPTOR_DEBUG_FLAG_STC_B                0x00000020
#define PTS_DESCRIPTOR_DEBUG_FLAG_STC_CURRENT          0x00000040
#define PTS_DESCRIPTOR_DEBUG_FLAG_TIMEBASE_MISMATCH    0x00000080
#define PTS_DESCRIPTOR_DEBUG_FLAG_LATE                 0x00000100
#define PTS_DESCRIPTOR_DEBUG_FLAG_EARLY                0x00000200
#define PTS_DESCRIPTOR_DEBUG_FLAG_VERY_LATE            0x00000100

/////////////////////////////////////////////////
// Tag descriptor defines
// Flag definitions
#define TAGLIST_SEI_PIC_STRUCT_FLAG_MASK       0x1
#define TAGLIST_SEI_PIC_STRUCT_FLAG_SHIFT      0
#define TAGLIST_SEI_FRAME_FREEZE_FLAG_MASK     0x2
#define TAGLIST_SEI_FRAME_FREEZE_FLAG_SHIFT    1
#define TAGLIST_SEI_FRAME_RELEASE_FLAG_MASK    0x4
#define TAGLIST_SEI_FRAME_RELEASE_FLAG_SHIFT   2
#define TAGLIST_SEI_SKIP_FLAG_MASK             0x8
#define TAGLIST_SEI_SKIP_FLAG_SHIFT            3

//////////////////////////////////////////////////////////////
// Frame status ~ TEMP
#define  FRAME_INUSE_BYIPP           0x1
#define  FRAME_FREE                  0x0

//////////////////////////////////////////////////////////////
// Scan format ~ TEMP
#define SCAN_FORMAT_INTERLACED      0x0
#define SCAN_FORMAT_PROGRESSIVE     0x1
#define SCAN_FORMAT_VALID_GET(r)    (r&0x1)
#define SCAN_FORMAT_GET(r)          ((r&0x2)>>1)
#define SCAN_FORMAT_VALID_SET(r)    ((r&0x1)<<0)
#define SCAN_FORMAT_SET(r)          ((r&0x1)<<1)

//////////////////////////////////////////////////////////////
// Internal System Control Defines

#define NO_INTERNAL_CONTROL          0xFF
#define SFD_INTERNAL_CONTROL         0x1
#define AUTORECOVER_INTERNAL_CONTROL 0x2

//////////////////////////////////////////////////////////////
// SVC specific defines
#define MEDIAIP_MAX_SVC_DID          0x3
#define MEDIAIP_MAX_SVC_STR_BUFFERS  0x3

//////////////////////////////////////////////////////////////
// Malone specific defines
#define MEDIAIP_MAX_NUM_MALONES         0x2
#define MEDIAIP_MAX_NUM_MALONE_IRQ_PINS 0x2

#define MEDIAIP_MAX_NUM_FSLCACHES     0x4

//////////////////////////////////////////////////////////////
// Windsor specific defines
#define MEDIAIP_MAX_NUM_WINDSORS         0x1
#define MEDIAIP_MAX_NUM_WINDSOR_IRQ_PINS 0x2

//////////////////////////////////////////////////////////////
// Subsystem specific defines
#define MEDIAIP_MAX_NUM_IRQ_PINS        0x10
#define MEDIAIP_MAX_NUM_CMD_IRQ_PINS    0x2
#define MEDIAIP_MAX_NUM_MSG_IRQ_PINS    0x1
#define MEDIAIP_MAX_NUM_TIMER_IRQ_PINS  0x4
#define MEDIAIP_MAX_NUM_TIMER_IRQ_SLOTS 0x4

#define SUBSYSTEM_CFG_MAGIC_COOKIE      0xB0B1B2B3

//////////////////////////////////////////////////////////////
// Max supported picture resolution, except H.264 and JPEG

#define MEDIAIP_MAX_PIC_WIDTH  2048
#define MEDIAIP_MAX_PIC_HEIGHT 2048

//////////////////////////////////////////////////////////////
// Encoder user data programming vals

#define MEDIAIP_ENC_USER_DATA_WORDS        16
#define MEDIAIP_ENC_USER_DATA_BYTES        ( MEDIAIP_ENC_USER_DATA_WORDS << 2 )



#endif /* _MEDIAIP_FW_DEFINES_H_ */

/* End of File */
