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

  Filename:        mvd_sif_control.h
  Description:     Malone system interface
  Notes:

 ****************************************************/

#ifndef _MVD_SIF_CONTROL_H_
#define _MVD_SIF_CONTROL_H_

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Global Macros
/////////////////////////////////////////////////////////////////////////////////

// SIF data range definitions
#define MSD_SIF_FS_KBYTES_BITS(r)                         (((r)&0xFFFF0000)>>16)
#define MSD_SIF_FRAME_SIZE_MB_BITS(r)                     ((r)&0xFFFF)
#define MSD_PUT_SIF_FRAME_SIZE_MB_BITS(w)                 ((w)&0xFFFF)
#define MSD_PUT_SIF_FS_KBYTES_BITS(w)                     (((w)&0xFFFF)<<16)

#define MSD_PUT_FRAME_WIDTH_MB_BITS(w)                    ((w)&0xFF)
#define MSD_PUT_FRAME_HEIGHT_MB_BITS(w)                   (((w)&0xFF)<<8)
#define MSD_PUT_FRAME_SIZE_MB_BITS(w)                     (((w)&0xFFFF)<<16)

#define MSD_DPB_IN_KBYTES_BITS(r)                         ((r)&0xFFFF)

#define MPD_SIF_DISP_Q_CNT_BITS(r)                        ((r)&0xFF)
#define MPD_SIF_DISP_Q_FULL_BITS(r)                       ((r>>8)&0x1)
#define MPD_SIF_DISP_Q_REQ_BIT(r)                         ((r>>16)&0x1)

#define MPD_SIF_DISP_Q_PUSH_FS_IDC_BITS(w)                ((w)&0x1F)
#define MPD_SIF_DISP_Q_PUSH_SPS_IDC_BITS(w)               (((w)&0x7)<<16)
#define MPD_SIF_DISP_Q_PUSH_DANG_FIELD_BIT(w)             (((w)&0x1)<<19)
#define MPD_SIF_DISP_Q_PUSH_FIELD_MODE_BIT(w)             (((w)&0x2)<<19)
#define MPD_SIF_DISP_Q_PUSH_BOT_FIRST_BIT(w)              (((w)&0x1)<<21)
#define MPD_SIF_DISP_Q_PUSH_SKIP_PIC_BITS(w)              (((w)&0x3)<<22)
#define MPD_SIF_DISP_Q_SYS_DATA_BITS(w)                   (((w)&0xFF)<<24)

#define MPD_SIF_DISP_Q_PUSH_CRC_MODE_BIT(w)               (((w)&0x1)<<20)

#define MPD_SIF_FS_PACK_UNIT_BITS(r)                      ((r)&0x7)
#define MPD_SIF_FS_BASE_UNIT_BITS(r)                      ((r>>12)&0x7)
#define MPD_SIF_SYS_DATA_EDGE_BIT(r)                      ((r>>16)&0x1)
#define MPD_SIF_BS_SYNC_USED_BIT(r)                       ((r>>17)&0x1)

#define MPD_SIF_SYS_DATA_BITS(r)                          ((r)&0xFF)

#define MPD_SIF_FRAME_DEC_IRQ_POS                         1
#define MPD_SIF_SLICE_DEC_IRQ_POS                         2
#define MPD_SIF_START_CODE_IRQ_POS                        3
#define MPD_SIF_SEMAPHORE_IRQ_BIT(r)                      ((r)&0x1)
#define MPD_SIF_FRAME_DEC_IRQ_BIT(r)                      (((r)&0x2)>>1)
#define MPD_SIF_SLICE_DEC_IRQ_BIT(r)                      (((r)&0x4)>>2)
#define MPD_SIF_START_CODE_IRQ_BIT(r)                     (((r)&0x8)>>3)


#define MSD_SIF_QPULL_IRQ_BIT(r)                          (((r)&0x10)>>4)
#define MSD_SIF_QPULL_SHIFT                               4
#define MSD_SIF_QPULL_MASK                                0x10
#define MPD_SIF_BSDMA_IRQ_BIT(r)                          (((r)&0x80)>>7)
#define PUT_MSD_STREAM_ID(w)                              ((w)&0xF)
#define PUT_MSD_FORMAT(w)                                 (((w)&0x3)<<4)

// Sempahore reg bits
#define FRAME_DISPLAYED_BITS(r)                           ((r)&0x1)

// MSD_SIF_DPB_FS_SIZE_ADDR
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_POS        0
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_SIZE       8
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_MASK       0xff
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_POS       8
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_SIZE      8
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_MASK      0xff
#define MSD_SIF_DPB_FS_SIZE_KBYTES_POS        16
#define MSD_SIF_DPB_FS_SIZE_KBYTES_SIZE       14
#define MSD_SIF_DPB_FS_SIZE_KBYTES_MASK       0x3fff
#define MSD_SIF_DPB_FS_SIZE_WIDTH_INMBS_GET(val) ((val>>MSD_SIF_DPB_FS_WIDTH_IN_MB_POS)&MSD_SIF_DPB_FS_WIDTH_IN_MB_MASK)
#define MSD_SIF_DPB_FS_SIZE_HEIGHT_INMBS_GET(val) ((val>>MSD_SIF_DPB_FS_HEIGHT_IN_MB_POS)&MSD_SIF_DPB_FS_HEIGHT_IN_MB_MASK)
#define MSD_SIF_DPB_FS_SIZE_KBYTES(val) ((val>>MSD_SIF_DPB_FS_SIZE_KBYTES_POS)&MSD_SIF_DPB_FS_SIZE_KBYTES_MASK)

// MSD_SIF_DPB_FS_SIZE_EXT_ADDR
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_EXT_POS            0
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_EXT_SIZE           2
#define MSD_SIF_DPB_FS_WIDTH_IN_MB_EXT_MASK           0x3
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_EXT_POS           2
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_EXT_SIZE          2
#define MSD_SIF_DPB_FS_HEIGHT_IN_MB_EXT_MASK          0x3
#define MSD_SIF_DPB_FS_SIZE_KBYTES_EXT_POS            4
#define MSD_SIF_DPB_FS_SIZE_KBYTES_EXT_SIZE           3
#define MSD_SIF_DPB_FS_SIZE_KBYTES_EXT_MASK           0x7
#define MSD_SIF_DPB_FS_SIZE_WIDTH_INMBS_EXT_GET(val)  ((val>>MSD_SIF_DPB_FS_WIDTH_IN_MB_POS)&MSD_SIF_DPB_FS_WIDTH_IN_MB_EXT_MASK)
#define MSD_SIF_DPB_FS_SIZE_HEIGHT_INMBS_EXT_GET(val) ((val>>MSD_SIF_DPB_FS_HEIGHT_IN_MB_POS)&MSD_SIF_DPB_FS_HEIGHT_IN_MB_EXT_MASK)
#define MSD_SIF_DPB_FS_SIZE_KBYTES_EXT(val)           ((val>>MSD_SIF_DPB_FS_SIZE_KBYTES_POS)&MSD_SIF_DPB_FS_SIZE_KBYTES_MASK)

//MSD_SIF_DPB_FRM_SIZE_ADDR
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_POS        0
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_SIZE       8
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_MASK       0xff
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_POS       8
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_SIZE      8
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_MASK      0xff
#define MSD_SIF_DPB_FRM_SIZE_MBS_POS          16
#define MSD_SIF_DPB_FRM_SIZE_MBS_SIZE         16
#define MSD_SIF_DPB_FRM_SIZE_MBS_MASK          0xffff
#define MSD_SIF_DPB_FRM_SIZE_INMBS(val)      ((val>>MSD_SIF_DPB_FRM_SIZE_MBS_POS)&MSD_SIF_DPB_FRM_SIZE_MBS_MASK)
#define MSD_SIF_DPB_FRM_WIDTH_INMBS_GET(val) ((val>>MSD_SIF_DPB_FRM_WIDTH_IN_MB_POS)&MSD_SIF_DPB_FRM_WIDTH_IN_MB_MASK)
#define MSD_SIF_DPB_FRM_HEIGHT_INMBS_GET(val) ((val>>MSD_SIF_DPB_FRM_HEIGHT_IN_MB_POS)&MSD_SIF_DPB_FRM_HEIGHT_IN_MB_MASK)

//MSD_SIF_DPB_FRM_SIZE_EXT_ADDR
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_POS        0
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_SIZE       1
#define MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_MASK       0x1
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_POS       1
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_SIZE      1
#define MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_MASK      0x1
#define MSD_SIF_DPB_FRM_SIZE_MBS_EXT_POS           2
#define MSD_SIF_DPB_FRM_SIZE_MBS_EXT_SIZE          3
#define MSD_SIF_DPB_FRM_SIZE_MBS_EXT_MASK          0x7
#define MSD_SIF_DPB_FRM_SIZE_INMBS_EXT(val)       ((val>>MSD_SIF_DPB_FRM_SIZE_MBS_POS)&MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_MASK)
#define MSD_SIF_DPB_FRM_WIDTH_INMBS_EXT_GET(val)  ((val>>MSD_SIF_DPB_FRM_WIDTH_IN_MB_POS)&MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_MASK)
#define MSD_SIF_DPB_FRM_HEIGHT_INMBS_EXT_GET(val) ((val>>MSD_SIF_DPB_FRM_HEIGHT_IN_MB_POS)&MSD_SIF_DPB_FRM_SIZE_MBS_EXT_MASK)
#define MSD_PUT_FRAME_WIDTH_MB_EXT_BITS(val)      (((val>>8)&MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_MASK)<<MSD_SIF_DPB_FRM_WIDTH_IN_MB_EXT_POS)
#define MSD_PUT_FRAME_HEIGHT_MB_EXT_BITS(val)     (((val>>8)&MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_MASK)<<MSD_SIF_DPB_FRM_HEIGHT_IN_MB_EXT_POS)
#define MSD_PUT_FRAME_SIZE_MB_EXT_BITS(val)       (((val>>16)&MSD_SIF_DPB_FRM_SIZE_MBS_EXT_MASK)<<MSD_SIF_DPB_FRM_SIZE_MBS_EXT_POS)


//MSD_SIF_DPB_FS_SETTING_ADDR
#define MSD_SIF_FS_PACK_UNIT_POS              0
#define MSD_SIF_FS_PACK_UNIT_SIZE             3
#define MSD_SIF_FS_PACK_UNIT(val)             ((val>>MSD_SIF_FS_PACK_UNIT_POS)&0x7)
#define MSD_SIF_FS_PACK_WIDTH_POS             4
#define MSD_SIF_FS_PACK_WIDTH_SIZE            3
#define MSD_SIF_FS_PACK_ID                    (1<<8)
#define MSD_SIF_FRM_SYNC_EDGE                 (1<<16)
#define MSD_SIF_BS_SYNC_USED                  (1<<17)
#define MSD_SIF_FS_BASE_UNIT_POS              12
#define MSD_SIF_FS_BASE_UNIT(val)             ((val>>MSD_SIF_FS_BASE_UNIT_POS)&0x7)

// Standard pack unit sizes
#define SIF_SPB_BASE_UNITS_1KBYTES            0x4
#define SIF_SPB_BASE_UNITS_2KBYTES            0x5
#define SIF_SPB_BASE_UNITS_4KBYTES            0x6
#define SIF_SPB_BASE_UNITS_8KBYTES            0x7

//MSD_SIF_DPB_OFFSET_ADDR
//MSD_SIF_FRM_SYNC_DATA_ADDR
//MSD_SIF_FRM_DANGLING_ADDR

//MSD_SIF_DPB_CONFIG_ADDR
#define MSD_SIF_DPB_LUT_ENAB               (1<<0)
#define MSD_SIF_DPB_LUT_ADDR_MSB           (1<<1)
#define MSD_SIF_DPB_USE_FS_IDC             (1<<2)
#define MSD_SIF_DPB_RSHIFT_CA_POS              3
#define MSD_SIF_DPB_RSHIFT_CA_SIZE             3

//MSD_SIF_DPB_LUT_LOAD_ADDR
#define MSD_SIF_DPB_LUT_VALUE_POS                0
#define MSD_SIF_DPB_LUT_VALUE_SIZE              20
#define MSD_SIF_DPB_LUT_ADDR_POS                24
#define MSD_SIF_DPB_LUT_ADDR_SIZE                6

//MSD_SIF_LOAD_DPB_NUMB_ADDR
#define MSD_SIF_LOAD_DPB_NUMB_NUMB_POS         0
#define MSD_SIF_LOAD_DPB_NUMB_NUMB_SIZE        5
#define MSD_SIF_LOAD_DPB_NUMB_NUMB(val)        ((val>>MSD_SIF_LOAD_DPB_NUMB_NUMB_POS)&0x1f)
#define MSD_SIF_LOAD_DPB_NUMB_SID_POS          12
#define MSD_SIF_LOAD_DPB_NUMB_SID_SIZE         4

//MSD_SIF_DEC_STATUS_ADDR
#define MSD_SIF_DEC_STATUS_SLC_PROG_POS             0
#define MSD_SIF_DEC_STATUS_SLC_PROG_SIZE            1
#define MSD_SIF_DEC_STATUS_SLC_PROG(val)            ((val>>MSD_SIF_DEC_STATUS_SLC_PROG_POS)&0x1)

#define MSD_SIF_DEC_STATUS_IMG_PROG_POS             1
#define MSD_SIF_DEC_STATUS_IMG_PROG_SIZE            1
#define MSD_SIF_DEC_STATUS_IMG_PROG(val)            ((val>>MSD_SIF_DEC_STATUS_IMG_PROG_POS)&0x1)

#define MSD_SIF_DEC_STATUS_FRM_PROG_POS             2
#define MSD_SIF_DEC_STATUS_FRM_PROG_SIZE            1
#define MSD_SIF_DEC_STATUS_FRM_PROG(val)            ((val>>MSD_SIF_DEC_STATUS_FRM_PROG_POS)&0x1)

#define MSD_SIF_DEC_STATUS_VPMD_BUSY_POS            3
#define MSD_SIF_DEC_STATUS_VPMD_BUSY_SIZE           1
#define MSD_SIF_DEC_STATUS_VPMD_BUSY(val)           ((val>>MSD_SIF_DEC_STATUS_VPMD_BUSY_POS)&0x1)

#define MSD_SIF_DEC_STATUS_VDBF_BUSY_POS            4
#define MSD_SIF_DEC_STATUS_VDBF_BUSY_SIZE           1
#define MSD_SIF_DEC_STATUS_VDBF_BUSY(val)           ((val>>MSD_SIF_DEC_STATUS_VDBF_BUSY_POS)&0x1)

#define MSD_SIF_DEC_STATUS_XOB_IDLE_POS             5
#define MSD_SIF_DEC_STATUS_XOB_IDLE_SIZE            1
#define MSD_SIF_DEC_STATUS_XOB_IDLE(val)            ((val>>MSD_SIF_DEC_STATUS_XOB_IDLE_POS)&0x1)

#define MSD_SIF_DEC_STATUS_DPBMC_IDLE_POS           6
#define MSD_SIF_DEC_STATUS_DPBMC_IDLE_SIZE          1
#define MSD_SIF_DEC_STATUS_DPBMC_IDLE(val)          ((val>>MSD_SIF_DEC_STATUS_DPBMC_IDLE_POS)&0x1)

#define MSD_SIF_DEC_STATUS_BS_EMPTY_POS             7
#define MSD_SIF_DEC_STATUS_BS_EMPTY_SIZE            1
#define MSD_SIF_DEC_STATUS_BS_EMPTY(val)            ((val>>MSD_SIF_DEC_STATUS_BS_EMPTY_POS)&0x1)

/* From Kronos Rev.B */
#define MSD_SIF_DEC_STATUS_MBQ_STAT_POS             8
#define MSD_SIF_DEC_STATUS_MBQ_STAT_SIZE            2
#define MSD_SIF_DEC_STATUS_MBQ_STAT(val)            ((val>>MSD_SIF_DEC_STATUS_MBQ_STAT_POS)&0x3)

#define MSD_SIF_DEC_STATUS_MPR_FIFO_POS             10
#define MSD_SIF_DEC_STATUS_MPR_FIFO_SIZE            6

#define MSD_SIF_DEC_STATUS_MPR_FIFOS_NEMPTY_POS     16
#define MSD_SIF_DEC_STATUS_MPR_FIFOS_NEMPTY_SIZE    1

#define MSD_SIF_DEC_STATUS_VMIF_CLSET_IDLE_POS      17
#define MSD_SIF_DEC_STATUS_VMIF_CLSET_IDLE_SIZE     2

#define MSD_SIF_DEC_STATUS_PIXIF_IDLE_POS           19
#define MSD_SIF_DEC_STATUS_PIXIF_IDLE_SIZE          1

#define MSD_SIF_DEC_STATUS_MBIWR_TAGFIFO_EMPTY_POS  22
#define MSD_SIF_DEC_STATUS_MBIWR_TAGFIFO_EMPTY_SIZE 1
#define MSD_SIF_DEC_STATUS_MBIWR_TAGFIFO_EMPTY(val) ((val>>MSD_SIF_DEC_STATUS_MBIWR_TAGFIFO_EMPTY_POS)&0x1)


//MSD_SIF_BS2RBSP_STATUS_ADDR   6'h32   // R, bs2rbsp status  -- [bs_sync frm_sync_data] got_start_code, nal_unit_done, buf_level
// BSD data range definitions
#define MSD_SIF_BS2RBSP_GOT_START_CODE_BITS(r) ((r)&0x1)
#define MSD_SIF_BS2RBSP_NAL_UNIT_DONE_BIT(r)   (((r)&0x2)>>1)
#define MSD_SIF_BS2RBSP_CBUF_LEVEL_BITS(r)     (((r)&0x1FC)>>2)
#define MSD_SIF_BS2RBSP_GOT_START_CODE_MASK    0x1
#define MSD_SIF_BS2RBSP_GOT_SCODE              (1<<0)
#define MSD_SIF_BS2RBSP_NALU_FINISH            (1<<1)
#define MSD_SIF_BS2RBSP_EMPTY                  (1<<2)
#define MSD_SIF_BS2RBSP_BUF_LEVEL_POS          3
#define MSD_SIF_BS2RBSP_BUF_LEVEL_SIZE         4
#define MSD_SIF_BS2RBSP_BUF_LEVEL(val)         ((val>>MSD_SIF_BS2RBSP_BUF_LEVEL_POS)&0xf)
#define MSD_SIF_BS2RBSP_GOT_PESSCODE           1<<8
#define MSD_SIF_BS2RBSP_SCD_LEVEL_POS          10
#define MSD_SIF_BS2RBSP_SCD_LEVEL_MASK         0x3
#define MSD_SIF_BS2RBSP_3RD_BTYE_POS           12
#define MSD_SIF_BS2RBSP_3RD_BTYE_MASK          0xFF
#define MSD_SIF_BS2RBSP_SCD_DIS_POS            20
#define MSD_SIF_BS2RBSP_SCD_DIS_MASK           0x1
#define MSD_SIF_BS2RBSP_LONG_SCODE_POS         21
#define MSD_SIF_BS2RBSP_LONG_SCODE_MASK        0x1
#define MSD_SIF_BS2RBSP_SHORT_SYNC_POS         22
#define MSD_SIF_BS2RBSP_SHORT_SYNC_MASK        0x1

#define MSD_SIF_BS2RBSP_SCD_LEVEL(val)         ((val >> MSD_SIF_BS2RBSP_SCD_LEVEL_POS ) & MSD_SIF_BS2RBSP_SCD_LEVEL_MASK )
#define MSD_SIF_BS2RBSP_SCD_3RD_BTYE(val)      ((val >> MSD_SIF_BS2RBSP_3RD_BTYE_POS ) & MSD_SIF_BS2RBSP_3RD_BTYE_MASK )
#define MSD_SIF_BS2RBSP_SCD_DIS(val)           ((val >> MSD_SIF_BS2RBSP_SCD_DIS_POS ) & MSD_SIF_BS2RBSP_SCD_DIS_MASK )
#define MSD_SIF_BS2RBSP_LONG_SCODE(val)        ((val >> MSD_SIF_BS2RBSP_LONG_SCODE_POS ) & MSD_SIF_BS2RBSP_LONG_SCODE_MASK )
#define MSD_SIF_BS2RBSP_SHORT_SYNC(val)        ((val >> MSD_SIF_BS2RBSP_SHORT_SYNC_POS ) & MSD_SIF_BS2RBSP_SHORT_SYNC_MASK )

//MSD_SIF_BS2RBSP_FEED_ADDR
#define MSD_SIF_BS2RBSP_FEED_CTRL              (1<<0)  // W: RBSP feed start & stop control,
#define MSD_SIF_BS2RBSP_NAL_EI_FLAG            (1<<1)  // R only, on nalu bases, cleared by HW at start of nalu,
#define MSD_SIF_BS2RBSP_FEED_STOP              ((~MSD_SIF_BS2RBSP_FEED_CTRL)&1)

//MSD_SIF_BS2RBSP_SCODE_ADDR
#define MSD_SIF_BS2RBSP_START_CODE_POS        0
#define MSD_SIF_BS2RBSP_START_CODE_SIZE       8
#define MSD_SIF_BS2RBSP_START_CODE(val)       ((val>>MSD_SIF_BS2RBSP_START_CODE_POS)&0xff)
#define MSD_SIF_RS2RBSP_SHORT_SC_POS          8
#define MSD_SIF_RS2RBSP_SHORT_SC_SIZE         1
#define MSD_SIF_RS2RBSP_SHORT_SC_FLAG(val)    ((val>>MSD_SIF_RS2RBSP_SHORT_SC_POS)&0x1)
#define MSD_SIF_BS2RBSP_SYNC_DATA_POS         16
#define MSD_SIF_BS2RBSP_SYNC_DATA(val)        ((val & 0xff0000) >> MSD_SIF_BS2RBSP_SYNC_DATA_POS)
#define MSD_SIF_BS2RBSP_SYNC_DATA_SET(val)    ((val<<MSD_SIF_BS2RBSP_SYNC_DATA_POS) & 0xff0000)

#define MSD_SIF_BS2RBSP_SYNC_DATA_SIZE        8
#define MSD_SIF_BS2RBSP_SYNC_DATA_PTSDTSFLAGS_POS 0
#define MSD_SIF_BS2RBSP_SYNC_DATA_DTS_POS 2
#define MSD_SIF_BS2RBSP_SYNC_DATA_PTS_POS 3
#define MSD_SIF_BS2RBSP_SYNC_DATA_ERR_POS 7
#define MSD_SIF_BS2RBSP_SYNC_DATA_ERR (1<<MSD_SIF_BS2RBSP_SYNC_DATA_ERR_POS)
#define MSD_SIF_BS2RBSP_SYNC_FLAG             (1<<24)     // 0 if bs_sync_used is set to '0'

//MSD_SIF_BS2RBSP_SCDCTRL_ADDR
#define MSD_SIF_BS2RBSP_SCDCTRL_EXPLICIT_CTRL_POS               0
#define MSD_SIF_BS2RBSP_SCDCTRL_ERR_SLCMRG_POS                  3
#define MSD_SIF_BS2RBSP_SCDCTRL_DETECT_JPEG_MARKERS_POS         5
#define MSD_SIF_BS2RBSP_SCDCTRL_STOP_SLICE_ON_SYNC_POS          6
#define MSD_SIF_BS2RBSP_SCDCTRL_USE_OLD_EMUL_PREVENT_POS        7
#define MSD_SIF_BS2RBSP_SCDCTRL_SCODE_DETECT_DISABLE_PERIOD_POS 8
#define MSD_SIF_BS2RBSP_SCDCTRL_EXPLICIT_CTRL       (1<<MSD_SIF_BS2RBSP_SCDCTRL_EXPLICIT_CTRL_POS)
#define MSD_SIF_BS2RBSP_SCDCTRL_ERR_SLCMRG          (1<<MSD_SIF_BS2RBSP_SCDCTRL_ERR_SLCMRG_POS)
#define MSD_SIF_BS2RBSP_SCDCTRL_DETECT_JPEG_MARKERS (1<<MSD_SIF_BS2RBSP_SCDCTRL_DETECT_JPEG_MARKERS_POS)
#define MSD_SIF_BS2RBSP_SCDCTRL_STOP_SLICE_ON_SYNC  (1<<MSD_SIF_BS2RBSP_SCDCTRL_STOP_SLICE_ON_SYNC_POS)

//MSD_SIF_CTRL_STATUS_ADDR
#define MSD_SIF_CTRL_SEMAPHORE_INTR_BIT        (1<<0)  // Read Only, '1'
#define MSD_SIF_CTRL_IMAGE_INTR_ENAB_BIT       (1<<1)
#define MSD_SIF_CTRL_SLICE_INTR_ENAB_BIT       (1<<2)
#define MSD_SIF_CTRL_SCODE_INTR_ENAB_BIT       (1<<3)
#define MSD_SIF_CTRL_QPULL_INTR_ENAB_BIT       (1<<4)
#define MSD_SIF_CTRL_DTLERR_INTR_ENAB_BIT      (1<<5)  // Took over FORCE_ENTRY bit as not needed in status
#define MSD_SIF_CTRL_PESSC_INTR_ENAB_BIT       (1<<6)  // Took over FORCE_ENTRY bit as not needed in status
#define MSD_SIF_CTRL_BSDMA_INTR_ENAB_BIT       (1<<7)
#define MSD_SIF_CTRL_MP2D_SLC_MERGE_BIT        (1<<8)
#define MSD_SIF_CTRL_BS2RBSP_ENAB_BIT          (1<<9)
#define MSD_SIF_CTRL_SCODE_IN_FEED_BIT         (1<<10)
#define MSD_SIF_CTRL_SLC_RESYNC_DISAB_BIT      (1<<11)
#define MSD_SIF_CTRL_ALT_IRQ_CLR_BIT           (1<<12)
#define MSD_SIF_CTRL_ASPD_SSC_ENAB_BIT         (1<<13) // SPD short start code enable
#define MSD_SIF_CTRL_BBB_RSCMD_DISAB_BIT       (1<<14) // BBB read-sensitive command disable - if set, BBB get_bits and exp-golumb commands require write first
#define MSD_SIF_CTRL_RSBXFR_INTR_ENAB_BIT      (1<<15)
#define MSD_SIF_CTRL_IRQ_SELECT_BITS_SHIFT     (16)

//MSD_SIF_INTR_STATUS_ADDR
#define MSD_SIF_INTR_SEMAPHORE_BIT             (1<<0)   // Controlled by semaphore interrupt mask, cannot be disabled or forced
#define MSD_SIF_INTR_IMAGE_DONE_BIT            (1<<1)
#define MSD_SIF_INTR_SLICE_DONE_BIT            (1<<2)
#define MSD_SIF_INTR_SCODE_FOUND_BIT           (1<<3)
#define MSD_SIF_INTR_DISPQ_PULL_BIT            (1<<4)
#define MSD_SIF_INTR_FORCE_ENTRY_BIT           (1<<5)   // Always enabled
#define MSD_SIF_INTR_PES_BIT                   (1<<6)
#define MSD_SIF_INTR_BSDMA_BIT                 (1<<7)
#define MSD_SIF_INTR_FORCE_EXIT_BIT            (1<<8)   // Always enabled
#define MSD_SIF_INTR_DTL_ERR_BIT               (1<<9)
#define MSD_SIF_INTR2_EXTENSION_BITS           (3<<10)  // Always Enabled
#define MSD_SIF_INTR2_EXTENSION_BIT_1          (1<<10)  // Always Enabled
#define MSD_SIF_INTR2_EXTENSION_BIT_2          (1<<11)  // Always Enabled
#define MSD_SIF_INTR3_EXTENSION_BITS           (3<<12)  // Always Enabled
#define MSD_SIF_INTR3_EXTENSION_BIT_1          (1<<12)  // Always Enabled
#define MSD_SIF_INTR3_EXTENSION_BIT_2          (1<<13)  // Always Enabled
#define MSD_SIF_INTR_RSBXFR_DONE_BIT           (1<<15)

//MSD_SIF_CTRL2_STATUS_ADDR
#define MSD_SIF_CTRL2_IRQ_MASK                 0x80000000
#define MSD_SIF_CTRL2_RPR_DONE_INTR_ENAB_BIT   ((1<<4)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_BSD_DONE_BIT             ((1<<5)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_SPP_SCODE_INTR_ENAB_BIT  ((1<<6)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_SPP_PESSC_INTR_ENAB_BIT  ((1<<7)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_SPP_BSDMA_INTR_ENAB_BIT  ((1<<8)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_CSC_DONE_ENAB_BIT        ((1<<9) |MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_CQ_ENAB_BIT              ((1<<10)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_DFE_DONE_ENAB_BIT        ((1<<13)|MSD_SIF_CTRL2_IRQ_MASK)
#define MSD_SIF_CTRL2_DFE_SLC_DONE_ENAB_BIT    ((1<<14)|MSD_SIF_CTRL2_IRQ_MASK)

//MSD_SIF_INTR2_STATUS_ADDR
#define MSD_SIF_INTR2_RPR_BIT                  (1<<4)
#define MSD_SIF_INTR2_SPP_SCODE_FOUND_BIT      (1<<6)
#define MSD_SIF_INTR2_SPP_PESSC_FOUND_BIT      (1<<7)
#define MSD_SIF_INTR2_SPP_BSDMA_BIT            (1<<8)
#define MSD_SIF_INTR2_CSC_BIT                  (1<<9)
#define MSD_SIF_INTR2_CQ_BIT                   (1<<10)
#define MSD_SIF_INTR2_DFE_DONE_BIT             (1<<13)
#define MSD_SIF_INTR2_DFE_SLC_DONE_BIT         (1<<14)

//MSD_SIF_CTRL3_STATUS_ADDR
#define MSD_SIF_CTRL3_IRQ_MASK                 0x40000000
#define MSD_SIF_CTRL3_DBE0_CQ_ENAB_BIT         ((1<<0)|MSD_SIF_CTRL3_IRQ_MASK)
#define MSD_SIF_CTRL3_DBE1_CQ_ENAB_BIT         ((1<<1)|MSD_SIF_CTRL3_IRQ_MASK)
#define MSD_SIF_CTRL3_DBE0_DONE_ENAB_BIT       ((1<<4)|MSD_SIF_CTRL3_IRQ_MASK)
#define MSD_SIF_CTRL3_DBE1_DONE_ENAB_BIT       ((1<<5)|MSD_SIF_CTRL3_IRQ_MASK)
#define MSD_SIF_CTRL3_DBE0_SLC_DONE_ENAB_BIT   ((1<<8)|MSD_SIF_CTRL3_IRQ_MASK)
#define MSD_SIF_CTRL3_DBE1_SLC_DONE_ENAB_BIT   ((1<<9)|MSD_SIF_CTRL3_IRQ_MASK)

//MSD_SIF_INTR3_STATUS_ADDR
#define MSD_SIF_INTR3_DBE0_CQ_BIT              (1<<0)
#define MSD_SIF_INTR3_DBE1_CQ_BIT              (1<<1)
#define MSD_SIF_INTR3_DBE0_DONE_BIT            (1<<4)
#define MSD_SIF_INTR3_DBE1_DONE_BIT            (1<<5)
#define MSD_SIF_INTR3_DBE0_SLC_DONE_BIT        (1<<8)
#define MSD_SIF_INTR3_DBE1_SLC_DONE_BIT        (1<<9)

//MSD_SIF_RESET_COMMAND_ADDR
#define MSD_SIF_RESET_DEC_ENGINE_BIT           (1<<0)
#define MSD_SIF_RESET_BS_INBUF_BIT             (1<<1)
#define MSD_SIF_RESET_DISP_QUEUE_BIT           (1<<2)
#define MSD_SIF_RESET_DTL_CACHE_BIT            (1<<3)
#define MSD_SIF_RESET_SLICE_BIT                (1<<4)
#define MSD_SIF_RESET_SPP_BIT                  (1<<5)
#define MSD_SIF_RESET_DTL_4K_CACHE_BIT         (1<<6)    /* TODO - useme */
#define MSD_SIF_RESET_DBE_BIT                  (1<<7)
#define MSD_SIF_RESET_DFE_BIT                  (1<<8)

//MSD_CTX_NEXT_SF_ADDR
#define MSD_CTX_NEXT_BS_IDC_POS       0
#define MSD_CTX_NEXT_BS_IDC_MASK      0xF
#define MSD_CTX_NEXT_FORMAT_POS       4
#define MSD_CTX_NEXT_FORMAT_MASK      0x1F

// PES control
//volatile u_int32  pes_setup;
#define MSD_SIF_PES_SETUP_STRMID_POS              0
#define MSD_SIF_PES_SETUP_SET_STRMID(id)          ((id&0xff)<<MSD_SIF_PES_SETUP_STRMID_POS)
#define MSD_SIF_PES_SETUP_STRMIDMASK_POS          8
#define MSD_SIF_PES_SETUP_SET_STRMIDMASK(mask)    ((mask&0xff)<<MSD_SIF_PES_SETUP_STRMIDMASK_POS)
#define MSD_SIF_PES_SETUP_PESIRQEN_POS            16
#define MSD_SIF_PES_SETUP_PESIRQEN                (1<<MSD_SIF_PES_SETUP_PESIRQEN_POS)
#define MSD_SIF_PES_SETUP_WAITPESHDR_POS          17
#define MSD_SIF_PES_SETUP_WAITPESHDR              (1<<MSD_SIF_PES_SETUP_WAITPESHDR_POS)
#define MSD_SIF_PES_SETUP_ERRORMODE_POS           18
#define MSD_SIF_PES_SETUP_SET_ERRORMODE(type)     ((type&0x3)<<MSD_SIF_PES_SETUP_ERRORMODE_POS)
#define MSD_SIF_PES_SETUP_ERRORMODE_NONE          0x0
#define MSD_SIF_PES_SETUP_ERRORMODE_EIFLAG        0x2
#define MSD_SIF_PES_SETUP_ERRORMODE_DISCONMARK    0x1
#define MSD_SIF_PES_SETUP_PESEN_POS               20
#define MSD_SIF_PES_SETUP_PESEN                   (1<<MSD_SIF_PES_SETUP_PESEN_POS)
#define MSD_SIF_PES_SETUP_CHKPKTLEN_POS           21
#define MSD_SIF_PES_SETUP_CHKPKTLEN               (1<<MSD_SIF_PES_SETUP_CHKPKTLEN_POS)
#define MSD_SIF_PES_SETUP_SYNC_ALWAYS_POS         22
#define MSD_SIF_PES_SETUP_SYNC_ALWAYS             (1<<MSD_SIF_PES_SETUP_SYNC_ALWAYS_POS)
#define MSD_SIF_PES_SETUP_HOLD_DODGY_PTS_POS      23
#define MSD_SIF_PES_SETUP_HOLD_DODGY_PTS          (1<<MSD_SIF_PES_SETUP_HOLD_DODGY_PTS_POS)
// Optionally disallow match inside PKT, to support artificial streams that do not have emulation prevention bytes
#define MSD_SIF_PES_SETUP_ENFORCE_PKTLEN_POS      24
#define MSD_SIF_PES_SETUP_ENFORCE_PKTLEN          (1<<MSD_SIF_PES_SETUP_ENFORCE_PKTLEN_POS)

//volatile u_int32  pes_status;
#define MSD_SIF_PES_STATUS_STATE_POS              0
#define MSD_SIF_PES_STATUS_GET_STATE(status)      ((status>>MSD_SIF_PES_STATUS_STATE_POS)&0x3f)
#define MSD_SIF_PES_STATUS_EVENT_POS              6
#define MSD_SIF_PES_STATUS_GET_EVENT(status)      ((status>>MSD_SIF_PES_STATUS_EVENT_POS)&0x3)
#define MSD_SIF_PES_STATUS_PTS_PRESENT 0x2
#define MSD_SIF_PES_STATUS_PTSDTS_PRESENT 0x3
#define MSD_SIF_PES_STATUS_ERROR_PRESENT 0x1
#define MSD_SIF_PES_STATUS_TOPDTS_POS             8
#define MSD_SIF_PES_STATUS_GET_TOPDTS(status)      ((status>>MSD_SIF_PES_STATUS_TOPDTS_POS)&0x1)
#define MSD_SIF_PES_STATUS_SET_TOPDTS(status)      ((status & 1) << MSD_SIF_PES_STATUS_TOPDTS_POS)
#define MSD_SIF_PES_STATUS_TOPPTS_POS             9
#define MSD_SIF_PES_STATUS_GET_TOPPTS(status)      ((status>>MSD_SIF_PES_STATUS_TOPPTS_POS)&0x1)
#define MSD_SIF_PES_STATUS_SET_TOPPTS(status)      ((status & 1) << MSD_SIF_PES_STATUS_TOPPTS_POS)
#define MSD_SIF_PES_STATUS_STALLED_POS            10
#define MSD_SIF_PES_STATUS_GET_STALLED(status)    ((status>>MSD_SIF_PES_STATUS_STALLED_POS)&0x1)
#define MSD_SIF_PES_STATUS_STREAM_ID_MASK          0xFF
#define MSD_SIF_PES_STATUS_STREAM_ID_POS           12
#define MSD_SIF_PES_STATUS_GET_STREAMID(status)    ((status>>MSD_SIF_PES_STATUS_STREAM_ID_POS)&MSD_SIF_PES_STATUS_STREAM_ID_MASK)
#define MSD_SIF_PES_STATUS_TIMEBASE_ID_MASK        0x7
#define MSD_SIF_PES_STATUS_TIMEBASE_ID_POS         12
#define MSD_SIF_PES_STATUS_GET_TIMEBASE(status)    ((status>>MSD_SIF_PES_STATUS_TIMEBASE_ID_POS)&MSD_SIF_PES_STATUS_TIMEBASE_ID_MASK)
#define MSD_SIF_PES_STATUS_PARITY_MASK             0x1
#define MSD_SIF_PES_STATUS_PARITY_POS              15
#define MSD_SIF_PES_STATUS_GET_PARITY(status)      ((status>>MSD_SIF_PES_STATUS_PARITY_POS)&MSD_SIF_PES_STATUS_PARITY_MASK)

typedef enum
{
  pes_Seek        =  4, // 6'b0001_00, // Wait for initial start pes start code
  pes_DumpPrefix  =  8, // 6'b0010_00, // Dumping the 0x00_00_01. Move on when get the 0x01
  pes_DumpStrmID  = 12, // 6'b0011_00, // Dumping the 0x00_00_01. Move on when get the 0x01
  pes_WaitClear   = 16, // 6'b0100_00, // Stalled waiting for firmware to clear the previous PTS
  pes_PackLenH    = 20, // 6'b0101_00, //
  pes_PackLenL    = 24, // 6'b0110_00, //
  pes_Discon_0    = 28, // 6'b0111_00, // Output 0x00 from internally generates discontinuity
  pes_Discon_1    = 32, // 6'b1000_00, // Output 0x00
  pes_Discon_2    = 36, // 6'b1001_00, // Output 0x01
  pes_Discon_3    = 40, // 6'b1010_00, // Output 0xB4
  pes_ParseFW     = 60, // 6'b1111_00, // Just let firmware read the data one at a time
  pes_Flags1      =  1, // 6'b0000_01,
  pes_Flags2      =  5, // 6'b0001_01,
  pes_HdrLen      =  9, // 6'b0010_01,
  pes_PTS1        =  3, // 6'b0000_11,
  pes_PTS2        =  7, // 6'b0001_11,
  pes_PTS3        = 11, // 6'b0010_11,
  pes_PTS4        = 15, // 6'b0011_11,
  pes_PTS5        = 19, // 6'b0100_11,
  pes_DTS1        = 23, // 6'b0101_11,
  pes_DTS2        = 27, // 6'b0110_11,
  pes_DTS3        = 31, // 6'b0111_11,
  pes_DTS4        = 35, // 6'b1000_11,
  pes_DTS5        = 39, // 6'b1001_11,
  pes_DumpHdr     = 43, // 6'b1010_11, // Skipping over remainder of headre
  pes_PayloadSync = 45, // 6'b1011_01, // Outputting first payload byte to SCD
  pes_Payload     = 49  // 6'b1100_01  // Outputting payload to SCD

} PES_STATE;

// BSDMA Control
//volatile u_int32 bsdma_command;
#define MSD_SIF_BSDMA_CMND_START              0x1
#define MSD_SIF_BSDMA_CMND_FETCHSTRDESC       0x2
#define MSD_SIF_BSDMA_CMND_STOP               0x4
#define MSD_SIF_BSDMA_CMND_CLEAR              0x8
#define MSD_SIF_BSDMA_CMND_FORCE_DESC_UPDATE  0x10
#define MSD_SIF_BSDMA_CMND_FORCE_CALC_LEVEL   0x20

//volatile u_int32 bsdma_options;
#define MSD_SIF_BSDMA_OPT_ENABLE  0x1
#define MSD_SIF_BSDMA_OPT_PERIEN  0x2
#define MSD_SIF_BSDMA_OPT_OFLIRQEN  0x4
#define MSD_SIF_BSDMA_OPT_DISCONEN  0x8
#define MSD_SIF_BSDMA_OPT_DESCUPPER_POS  4
#define MSD_SIF_BSDMA_OPT_LOCBUFSIZE_POS  8
#define MSD_SIF_BSDMA_OPT_LOCBUFOFFSET_POS  16
#define MSD_SIF_BSDMA_OPT_DEBUG_FEED (1<<24)
#define MSD_SIF_BSDMA_OPT_BURST      (1<<25)

#define MSD_SIF_BSDMA_OPT_SAFEREADMARGIN_POS 26
#define MSD_SIF_BSDMA_OPT_TRACEBACK_POS  28
#define MSD_BSDMA_STRIDE_END_IRQ_P0S         30     // Interrupt when all data read
#define MSD_BSDMA_STRIDE_END_IRQ (1<<MSD_BSDMA_STRIDE_END_IRQ_P0S)
#define MSD_BSDMA_STRIDE_DONE_IRQ_POS        31   // Interrupt when all data used
#define MSD_BSDMA_STRIDE_DONE_IRQ ((u_int32)(1<<MSD_BSDMA_STRIDE_DONE_IRQ_POS))


#define MSD_SIF_BSDMA_OPT_0B_SAFEMARGIN (0<<MSD_SIF_BSDMA_OPT_SAFEREADMARGIN_POS)
#define MSD_SIF_BSDMA_OPT_128B_SAFEMARGIN (1<<MSD_SIF_BSDMA_OPT_SAFEREADMARGIN_POS)
#define MSD_SIF_BSDMA_OPT_2KB_SAFEMARGIN (2<<MSD_SIF_BSDMA_OPT_SAFEREADMARGIN_POS)
#define MSD_SIF_BSDMA_OPT_32KB_SAFEMARGIN (3<<MSD_SIF_BSDMA_OPT_SAFEREADMARGIN_POS)

#define MSD_SIF_BSDMA_OPT_TRACEBACK_0     (0<<MSD_SIF_BSDMA_OPT_TRACEBACK_POS)
#define MSD_SIF_BSDMA_OPT_TRACEBACK_16K   (1<<MSD_SIF_BSDMA_OPT_TRACEBACK_POS)
#define MSD_SIF_BSDMA_OPT_TRACEBACK_256K  (2<<MSD_SIF_BSDMA_OPT_TRACEBACK_POS)
#define MSD_SIF_BSDMA_OPT_TRACEBACK_4M    (3<<MSD_SIF_BSDMA_OPT_TRACEBACK_POS)

/* Constant defines on the 160/128 64-bit words of IBB memory    */
/* Split between the BSDMA stream buffer and the H.264 IBB usage */
#define IBBBUF_SIZE        128
#define IBBBUF_SIZE_LARGE  160
#define BSDMA_IBBBUF_USAGE  16

//volatile u_int32 bsdma_status;
#define MSD_SIF_BSDMA_STATUS_CB_LEVEL    (0xFF)
#define MSD_SIF_BSDMA_STATUS_NMEMWORDS   (0x1f<<8)
#define MSD_SIF_BSDMA_STATUS_BELOW_LWM   (0x1<<13)   //  Currently below Low Water Mark
#define MSD_SIF_BSDMA_STATUS_BELOW_LWMH  (0x1<<14)   //  Below Low Water Mark since last cleared
#define MSD_SIF_BSDMA_STATUS_BUFOFLOW    (0x1<<15)
#define MSD_SIF_BSDMA_STATUS_DMA_STATE   (0x1F<<16)
#define MSD_SIF_BSDMA_STATUS_MCX_STATE   (0x3F<<21)
#define MSD_SIF_BSDMA_STATUS_RQ_STATE    (0x1<<27)
#define MSD_SIF_BSDMA_STATUS_ABOVE_HWM   (0x1<<28)   //  Currently above High Water Mark
#define MSD_SIF_BSDMA_STATUS_ABOVE_HWMH  (0x1<<29)   //  Above High Water Mark since last cleared
#define MSD_SIF_BSDMA_STATUS_STRIDE_END  (0x1<<30)   //  all data fed
#define MSD_SIF_BSDMA_STATUS_STRIDE_DONE ((u_int32)(0x1<<31))   //  all data used

#define MSD_SIF_BSDMA_EXTOPT_LOOPBIT_POS    0x0
#define MSD_SIF_BSDMA_EXTOPT_CB_MIN_POS     3
#define MSD_SIF_BSDMA_EXTOPT_REQ_8          (1<<10)
#define MSD_SIF_BSDMA_EXTOPT_SINGLE_STRIDE  (1<<11)
#define MSD_SIF_BSDMA_EXTOPT_INIT_IRQ       (1<<12)
#define MSD_SIF_BSDMA_EXTOPT_MIN_UPDATES    (1<<13)
#define MSD_SIF_BSDMA_EXTOPT_CHECK_WPTR     (1<<14)

//volatile u_int32 bsdma_ext_options;
#define MSD_SIF_BSDMA_EXTOPT_LOOPBIT_SET(w) ((w&0x7)<<MSD_SIF_BSDMA_EXTOPT_LOOPBIT_POS)
#define MSD_SIF_BSDMA_EXTOPT_LOOPBIT_GET(r) ((r>>MSD_SIF_BSDMA_EXTOPT_LOOPBIT_POS)&0x7)
#define MSD_SIF_BSDMA_EXTOPT_SINGLESHOTEXITMODE MSD_SIF_BSDMA_EXTOPT_SINGLE_STRIDE

// This define is used only when FW controls how close the RP can get to the WP
// For pecos we are assuming at least two full burst of distance
//#define BSDMA_WP_AHEAD_OF_RP_MARGIN (2*16*8)
#define BSDMA_WP_AHEAD_OF_RP_MARGIN (8*16*8)
#define BSDMA_DESC_REAL_ADDR(x) (x&0xfffffff)
#define BSDMA_DESC_WRAP_ADDR(x) (x&0xf0000000)
#define BSDMA_DESC_NOPARITY_MASK 0xfffffffe


#define MSD_FORMAT_H264          (0x1  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_VC1           (0x2  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_MP2           (0x3  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_AVS           (0x4  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_ASP           (0x5  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_JPG           (0xD  <<MSD_CTX_NEXT_FORMAT_POS) // Uses aspd HW and ext. format flag
#define MSD_FORMAT_RV8           (0x6  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_RV9           (0xE  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_VP6           (0x7  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_VP8           (0xF  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_VP3           (0x7  <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_HEVC          (0x10 <<MSD_CTX_NEXT_FORMAT_POS)
#define MSD_FORMAT_NULL          (0x0  <<MSD_CTX_NEXT_FORMAT_POS)

//MSD_SIF_DPBMC_SETUP_ADDR
#define MSD_SIF_DPBMC_SET_CRC_TYPE(datasrc,datatype) (((0x3&datatype)<<2)|(datasrc&0x3))

#define MSD_SIF_DPBMC_CLEAR_CRC2_TYPE_MASK            0xFF0FFFFF
#define MSD_SIF_DPBMC_SET_CRC2_TYPE(datasrc,datatype) (((0x3&datatype)<<22)|((datasrc&0x3)<<20))
// Alternate method of setup

#define MSD_SIF_DPBMC_CLEAR_CRC2_MAJOR_MINOR_TYPE_MASK 0x87CFFFFF
#define MSD_SIF_DPBMC_SET_CRC2_TYPE_MAJOR(datatype)    (( 0x3 & datatype ) << 20 )
#define MSD_SIF_DPBMC_SET_CRC2_TYPE_MINOR(datasrc)     (( 0xf & datasrc ) << 27 )
#define MSD_SIF_DPBMC_GET_CRC2_TYPE_MAJOR(datatype)    (( datatype >> 20 ) & 0x3 )
#define MSD_SIF_DPBMC_GET_CRC2_TYPE_MINOR(datasrc)     (( datasrc  >> 27)  & 0xf )
#define MSD_SIF_DPBMC_SET_CRC_ALT_TYPE_MAJOR(datatype) (( 0x3 & datatype ) << 2 )
#define MSD_SIF_DPBMC_SET_CRC_ALT_TYPE_MINOR(datasrc)  (( 0xf & datasrc ) << 23 )
#define MSD_SIF_DPBMC_SET_CRC_USE_ALT_TYPE(use)        (( use & 0x1 ) << 22 )

#define MSD_SIF_DPBMC_CRC_RDDATA_TYPE   0x0
#define MSD_SIF_DPBMC_CRC_WRDATA_TYPE   0x1
#define MSD_SIF_DPBMC_CRC_RDADDR_TYPE   0x2
#define MSD_SIF_DPBMC_CRC_WRADDR_TYPE   0x3
#define MSD_SIF_DPBMC_CRC_BPP_SRCTYPE   0x3
#define MSD_SIF_DPBMC_CRC_MBI_SRCTYPE   0x2
#define MSD_SIF_DPBMC_CRC_PIXEL_SRCTYPE 0x0

#define MSD_SIF_DPBMC_CRC_ALT_DBE1_SRCTYPE      0xf
#define MSD_SIF_DPBMC_CRC_ALT_TES_SRCTYPE       0xe
#define MSD_SIF_DPBMC_CRC_ALT_DFE_SRCTYPE       0xd
#define MSD_SIF_DPBMC_CRC_ALT_MPSDBF_UV_SRCTYPE 0xc
#define MSD_SIF_DPBMC_CRC_ALT_SVC_RSMP_SRCTYPE  0xb
#define MSD_SIF_DPBMC_CRC_ALT_SVC_META_SRCTYPE  0xa
#define MSD_SIF_DPBMC_CRC_ALT_MBQ_SRCTYPE       0x9
#define MSD_SIF_DPBMC_CRC_ALT_CSC_SRCTYPE       0x8
#define MSD_SIF_DPBMC_CRC_ALT_RPR_SRCTYPE       0x7
#define MSD_SIF_DPBMC_CRC_ALT_MPSDBF_SRCTYPE    0x6
#define MSD_SIF_DPBMC_CRC_ALT_MBI_SRCTYPE       0x5
#define MSD_SIF_DPBMC_CRC_ALT_MX12_SRCTYPE      0x4
#define MSD_SIF_DPBMC_CRC_ALT_MX34_SRCTYPE      0x3
#define MSD_SIF_DPBMC_CRC_ALT_RSB_SRCTYPE       0x2
#define MSD_SIF_DPBMC_CRC_ALT_MCX_SRCTYPE       0x1
#define MSD_SIF_DPBMC_CRC_ALT_BSD_SRCTYPE       0x0

#define MSD_SIF_DPBMC_OUTSTAND_REQ_POS 4
#define MSD_SIF_DPBMC_OUTSTAND_REQ_MASK 0x7
#define MSD_SIF_DPBMC_SET_OUTSTAND_REQ(num_req) ((num_req&MSD_SIF_DPBMC_OUTSTAND_REQ_MASK)<<MSD_SIF_DPBMC_OUTSTAND_REQ_POS)
#define MSD_SIF_DPBMC_OUTSTAND_WR_REQ_POS 7
#define MSD_SIF_DPBMC_OUTSTAND_WR_REQ_MASK 0x3
#define MSD_SIF_DPBMC_SET_OUTSTAND_WR_REQ(num_req) ((num_req&MSD_SIF_DPBMC_OUTSTAND_WR_REQ_MASK)<<MSD_SIF_DPBMC_OUTSTAND_WR_REQ_POS)
#define MSD_SIF_DPBMC_DBFAL_MODE_POS 9
#define MSD_SIF_DPBMC_DBFAL_MODE_MASK 0x3
#define MSD_SIF_DPBMC_SET_DBFAL_MODE(num_req) ((num_req&MSD_SIF_DPBMC_DBFAL_MODE_MASK)<<MSD_SIF_DPBMC_DBFAL_MODE_POS)
#define MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_POS  0x0
#define MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_MASK 0x1
#define MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH(a) ((a&MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_MASK)<<MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_POS)
#define MSD_SIF_DPBMC_SET_CLR_CHROMA_DPATH (~(MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_MASK << MSD_SIF_DPBMC_SET_USE_CHROMA_DPATH_POS))


// VMIF MBI CACHE
#define MSD_VMIF_MBI_CACHE_RDCLR_POS       0
#define MSD_VMIF_MBI_CACHE_RDCLR_MASK      0x7
#define MSD_VMIF_MBI_CACHE_RDDIS_POS       3
#define MSD_VMIF_MBI_CACHE_RDDIS_MASK      0x7
#define MSD_VMIF_MBI_CACHE_RDSCLR_DIS_POS  6
#define MSD_VMIF_MBI_CACHE_RDSCLR_DIS_MASK 0x7
#define MSD_VMIF_MBI_CACHE_WRCLR_POS       9
#define MSD_VMIF_MBI_CACHE_WRCLR_MASK      0x7
#define MSD_VMIF_MBI_CACHE_WRDIS_POS       12
#define MSD_VMIF_MBI_CACHE_WRDIS_MASK      0x7
#define MSD_VMIF_MBI_CACHE_WRFLSH_POS      15
#define MSD_VMIF_MBI_CACHE_WRFLSH_MASK     0x7

#define MSD_VMIF_MBI_CACHE_SET_RDCLR(w)      ((w&MSD_VMIF_MBI_CACHE_RDCLR_MASK)<<MSD_VMIF_MBI_CACHE_RDCLR_POS)
#define MSD_VMIF_MBI_CACHE_SET_RDDIS(w)      ((w&MSD_VMIF_MBI_CACHE_RDDIS_MASK)<<MSD_VMIF_MBI_CACHE_RDDIS_POS)
#define MSD_VMIF_MBI_CACHE_SET_RDSCLR_DIS(w) ((w&MSD_VMIF_MBI_CACHE_RDSCLR_DIS_MASK)<<MSD_VMIF_MBI_CACHE_RDSCLR_DIS_POS)
#define MSD_VMIF_MBI_CACHE_SET_WRCLR(w)      ((w&MSD_VMIF_MBI_CACHE_WRCLR_MASK)<<MSD_VMIF_MBI_CACHE_WRCLR_POS)
#define MSD_VMIF_MBI_CACHE_SET_WRDIS(w)      ((w&MSD_VMIF_MBI_CACHE_WRDIS_MASK)<<MSD_VMIF_MBI_CACHE_WRDIS_POS)
#define MSD_VMIF_MBI_CACHE_SET_WRFLSH(w)     ((w&MSD_SIF_DPBMC_OUTSTAND_WR_REQ_MASK)<<MSD_VMIF_MBI_CACHE_WRFLSH_POS)


// DPBMC SETUP 2
#define MSD_SIF_DPBMC_SET_OUTSTAND_MBIRD_MASK 0x3
#define MSD_SIF_DPBMC_SET_OUTSTAND_MBIRD_POS  14
#define MSD_VMIF_MPSAL_MODE_DISOB_BIT        13
#define MSD_VMIF_MPSAL_MODE_REQ16_BIT        10
#define MSD_VMIF_MPSAL_MODE_16PIX_POS         9
//      MSD_SIF_COUNT_STRB_SEL_BITS          8:1
#define MSD_SIF_COUNT_RD_CMD_STRB_SEL        7
#define MSD_SIF_COUNT_WR_CMD_STRB_SEL        6
#define MSD_SIF_COUNT_WR_STRB_SEL            5
#define MSD_SIF_COUNT_RD_STRB_SEL            4
#define MSD_SIF_COUNT_UV_RD_STRB_SEL         3
#define MSD_SIF_COUNT_RSMP_RD_STRB_SEL       2
#define MSD_SIF_COUNT_RSMP_MBMETA_STRB_SEL   1
#define MSD_SIF_COUNT_MBI_RD_STRB_SEL        0

#define MSD_SIF_COUNT_STRB_SEL_MASK          0x1
#define MSD_SIF_COUNT_STRB_SET_USE_RD_CMD_STRB(a)      (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_RD_CMD_STRB_SEL       )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_WR_CMD_STRB(a)      (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_WR_CMD_STRB_SEL       )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_WR_STRB(a)          (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_WR_STRB_SEL           )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_RD_STRB(a)          (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_RD_STRB_SEL           )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_UV_RD_STRB(a)       (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_UV_RD_STRB_SEL        )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_RSMP_RD_STRB(a)     (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_RSMP_RD_STRB_SEL      )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_RSMP_MBMETA_STRB(a) (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_RSMP_MBMETA_STRB_SEL  )<<1)
#define MSD_SIF_COUNT_STRB_SET_USE_MBI_RD_STRB(a)      (((a&MSD_SIF_COUNT_STRB_SEL_MASK)         <<MSD_SIF_COUNT_MBI_RD_STRB_SEL       )<<1)
#define MSD_SIF_DPBMC_SET_OUTSTAND_MBIRD(a)            ((a&MSD_SIF_DPBMC_SET_OUTSTAND_MBIRD_MASK)<<MSD_SIF_DPBMC_SET_OUTSTAND_MBIRD_POS)


#define MSD_DPBMC_DPBCOH_POS          9
#define MSD_DPBMC_DPBCOH_MASK         0x1
#define MSD_DPBMC_DPBCOH_SET(a) ((a&MSD_DPBMC_DPBCOH_MASK)<<MSD_DPBMC_DPBCOH_POS)
#define MSD_DPBMC_MBICOH_POS          10
#define MSD_DPBMC_MBICOH_MASK         0x1
#define MSD_DPBMC_MBICOH_SET(a) ((a&MSD_DPBMC_MBICOH_MASK)<<MSD_DPBMC_MBICOH_POS)
#define MSD_DPBMC_RSBCOH_POS          11
#define MSD_DPBMC_RSBCOH_MASK         0x1
#define MSD_DPBMC_RSBCOH_SET(a) ((a&MSD_DPBMC_RSBCOH_MASK)<<MSD_DPBMC_RSBCOH_POS)
#define MSD_SIF_DPBMC_BASE_UNIT_POS   12
#define MSD_SIF_DPBMC_BASE_UNIT_MASK 0x7
#define MSD_SIF_DPBMC_SET_BASE_UNIT(base) ((base&MSD_SIF_DPBMC_BASE_UNIT_MASK)<<MSD_SIF_DPBMC_BASE_UNIT_POS)
#define MSD_DPBMC_INT_WAIT_IDLE_POS   15
#define MSD_DPBMC_INT_WAIT_IDLE_MASK  0x1
#define MSD_DPBMC_INT_WAIT_IDLE_SET(a) ((a&MSD_DPBMC_INT_WAIT_IDLE_MASK)<<MSD_DPBMC_INT_WAIT_IDLE_POS)
#define MSD_DPBMC_SC_INT_WAIT_IDLE_SET_POS      31
#define MSD_DPBMC_SC_INT_WAIT_IDLE_SET_MASK     0x1
#define MSD_DPBMC_SC_INT_WAIT_IDLE_SET(a) ((a&MSD_DPBMC_SC_INT_WAIT_IDLE_SET_MASK)<<MSD_DPBMC_SC_INT_WAIT_IDLE_SET_POS)
#define MSD_DPBMC_ARB_MCX_POS         16
#define MSD_DPBMC_ARB_MCX_MASK        0x1
#define MSD_DPBMC_ARB_MCX_SET(a) ((a&MSD_DPBMC_ARB_MCX_MASK)<<MSD_DPBMC_ARB_MCX_POS)
#define MSD_DPBMC_ARB_MCX_CLR(a) (~((a&MSD_DPBMC_ARB_MCX_MASK)<<MSD_DPBMC_ARB_MCX_POS))
#define MSD_DPBMC_ARB_BSDMA_POS       17
#define MSD_DPBMC_ARB_BSDMA_MASK      0x1
#define MSD_DPBMC_ARB_BSDMA_SET(a) ((a&MSD_DPBMC_ARB_BSDMA_MASK)<<MSD_DPBMC_ARB_BSDMA_POS)
#define MSD_DPBMC_ARB_BSDMA_CLR(a) (~((a&MSD_DPBMC_ARB_BSDMA_MASK)<<MSD_DPBMC_ARB_BSDMA_POS))
#define MSD_DPBMC_DIS_CRC_AUTO_RESET_POS   19
#define MSD_DPBMC_DIS_CRC_AUTO_RESET_MASK  0x1
#define MSD_DPBMC_DIS_CRC_AUTO_RESET_SET(a) ((a&MSD_DPBMC_DIS_CRC_AUTO_RESET_MASK)<<MSD_DPBMC_DIS_CRC_AUTO_RESET_POS)
#define MSD_DPBMC_DIS_CRC_AUTO_RESET_CLR(a) (~((a&MSD_DPBMC_DIS_CRC_AUTO_RESET_MASK)<<MSD_DPBMC_DIS_CRC_AUTO_RESET_POS))

// Local memory configuration (power down)
#define MSD_SIF_LMEM_CONFIG_PD_MASK    0x1
#define MSD_SIF_LMEM_CONFIG_PD_CCHE0_POS 0
#define MSD_SIF_LMEM_CONFIG_PD_RSB_POS   1
#define MSD_SIF_LMEM_CONFIG_PD_AVSD_POS  2
#define MSD_SIF_LMEM_CONFIG_PD_MPGD_POS  3
#define MSD_SIF_LMEM_CONFIG_PD_VC1D_POS  4
#define MSD_SIF_LMEM_CONFIG_PD_AVCD_POS  5
#define MSD_SIF_LMEM_CONFIG_PD_COMM_POS  6
#define MSD_SIF_LMEM_CONFIG_PD_RPR_POS   7
#define MSD_SIF_LMEM_CONFIG_PD_CCHE1_POS 14
#define MSD_SIF_LMEM_CONFIG_PD_HEVD_POS  15
#define MSD_SIF_LMEM_CONFIG_PD_VMIF_POS  17

#define MSD_SIF_LMEM_CONFIG_SET_PD_CCHE0(a)   ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_CCHE0_POS )
#define MSD_SIF_LMEM_CONFIG_SET_PD_RSB(a)     ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_RSB_POS   )
#define MSD_SIF_LMEM_CONFIG_SET_PD_AVSD(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_AVSD_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_MPGD(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_MPGD_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_VC1D(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_VC1D_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_AVCD(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_AVCD_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_COMM(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_COMM_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_RPR(a)     ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_RPR_POS   )
#define MSD_SIF_LMEM_CONFIG_SET_PD_CCHE1(a)   ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_CCHE1_POS )
#define MSD_SIF_LMEM_CONFIG_SET_PD_HEVD(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_HEVD_POS  )
#define MSD_SIF_LMEM_CONFIG_SET_PD_VMIF(a)    ((a&MSD_SIF_LMEM_CONFIG_PD_MASK) <<MSD_SIF_LMEM_CONFIG_PD_VMIF_POS  )

#define MSD_SIF_LMEM_CONFIG_RESET_PD_CCHE0(a) (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_CCHE0_POS )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_RSB(a)   (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_RSB_POS   )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_AVSD(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_AVSD_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_MPGD(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_MPGD_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_VC1D(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_VC1D_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_AVCD(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_AVCD_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_COMM(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_COMM_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_RPR(a)   (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_RPR_POS   )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_CCHE1(a) (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_CCHE1_POS )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_HEVD(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_HEVD_POS  )))
#define MSD_SIF_LMEM_CONFIG_RESET_PD_VMIF(a)  (a & (~(MSD_SIF_LMEM_CONFIG_PD_MASK << MSD_SIF_LMEM_CONFIG_PD_VMIF_POS  )))



// Row-store buffer control
//--------------------------
// MSD_SIF_RSB_CTRL_STAT_ADDR (0x6e) //0x1b8
#define MSD_SIF_RSB_SWITCH_REGION_ACCESS_POS    0x0
#define MSD_SIF_RSB_SWITCH_REGION_ACCESS_MASK   0x1
#define MSD_SIF_RSB_SWITCH_REGION_ACCESS_GET(r) (( r >> MSD_SIF_RSB_SWITCH_REGION_ACCESS_POS ) & MSD_SIF_RSB_SWITCH_REGION_ACCESS_MASK )
#define MSD_SIF_RSB_SWITCH_REGION_ACCESS_SET(w) (( w & MSD_SIF_RSB_SWITCH_REGION_ACCESS_MASK ) << MSD_SIF_RSB_SWITCH_REGION_ACCESS_POS )
#define MSD_SIF_RSB_INT_BASE_ADDR_SET(a) ((0x1fff & a)<<0) // 12:0
#define MSD_SIF_RSB_XFR_XNUM_MASK        ((0xff )<<13)
#define MSD_SIF_RSB_XFR_XNUM_SET(a)      ((0xff & a )<<13) //20:13
#define MSD_SIF_RSB_XFR_YNUM_MASK        ((0x7 )<<21)   // 23:21
#define MSD_SIF_RSB_XFR_YNUM_SET(a)      ((0x7 & a)<<21)   // 23:21
#define MSD_SIF_RSB_SPLIT_FACTOR_MASK    ((0x3 )<<24)  //25:24
#define MSD_SIF_RSB_SPLIT_FACTOR_SET(a)  ((0x3 & a)<<24)  //25:24
#define MSD_SIF_RSB_XFR_MODE_MASK        ((0x3 )<<26)  //27:26
#define MSD_SIF_RSB_XFR_MODE_SET(a)      ((0x3 & a)<<26)  //27:26
#define MSD_SIF_RSB_XFR_START_MASK       ((0x1 )<<28) //28
#define MSD_SIF_RSB_XFR_START_SET(a)     ((0x1 & a)<<28) //28
#define MSD_SIF_RSB_APB_ACC_EN_MASK      ((0x1 )<<31) //31
#define MSD_SIF_RSB_APB_ACC_EN_SET(a)    ((0x1 & a)<<31) //31

#define MSD_SIF_RSB_XFR_PROG_GET(a)      (( a >>29) & 0x1) //29
#define MSD_SIF_RSB_XFR_DONE_GET(a)      (( a >>30) & 0x1) //30
#define MSD_SIF_RSB_XFR_MODE_GET(a)      (( a >>26) & 0x3) //27:26


// Ancillary RSB transfer defines
//-------------------------------
#define MSD_SIF_RSB_INT_BASE_ADDR_MASK   0x1FFFF

//MSD_SIF_EXT_RSB_BASE_ADDR (0X6f) //0x1bc
#define MSD_SIF_EXT_RSB_BASE_ADDR_SET(a)  (a & 0xffffff) // 31:3
//MSD_SIF_EXT_XFR_PARAMS_ADDR (0x70) 0x1c0
#define MSD_SIF_EXT_XFR_PARAM_TF_TYPE_MASK   ((0x3) <<30 ) //31:30  0:2DLU; 1:2DCR 2:MINF 3:1DTF ;
#define MSD_SIF_EXT_XFR_PARAM_TF_TYPE(a)     ((a & 0x3) <<30 ) //31:30  0:2DLU; 1:2DCR 2:MINF 3:1DTF ;
#define MSD_SIF_EXT_XFR_PARAM_TYPE_GET(a)    (( a >>30) & 0x3) //31:30
// 2D transferred
#define MSD_SIF_EXT_XFR_PARAM_FIELD_FRAME_MASK ((0x01)<<29) //29 : field 1 ; 0 frame
#define MSD_SIF_EXT_XFR_PARAM_TOP_ROW_POS_MASK ((0x7ff)<<18) // row position in frame of top row of data in the transfer
#define MSD_SIF_EXT_XFR_PARAM_COL_LFT_POS_MASK ((0xff )<<10) // collumn position in frame of left-most word in the transfer
#define MSD_SIF_EXT_XFR_PARAM_NUM_ROW_MASK     ((0x1f )<<5)  // number of rows in a tile to be transferred
#define MSD_SIF_EXT_XFR_PARAM_NUM_PER_ROW_MASK ((0x3) <<0)  // number of words per row in a tile to be transferred

#define MSD_SIF_EXT_XFR_PARAM_FIELD_FRAME(a) ((a & 0x01)<<29) //29 : field 1 ; 0 frame
#define MSD_SIF_EXT_XFR_PARAM_TOP_ROW_POS(a) ((a & 0x7ff)<<18) // row position in frame of top row of data in the transfer
#define MSD_SIF_EXT_XFR_PARAM_COL_LFT_POS(a) ((a & 0xff )<<10) // collumn position in frame of left-most word in the transfer
#define MSD_SIF_EXT_XFR_PARAM_NUM_ROW(a)     ((a & 0x1f )<<5)  // number of rows in a tile to be transferred
#define MSD_SIF_EXT_XFR_PARAM_NUM_PER_ROW(a) ((a & 0x3) <<0)  // number of words per row in a tile to be transferred
// 1D transferred
#define MSD_SIF_EXT_XFR_PARAM_NUM_WORDS_MASK ((0x1fff)<<0) // number of words to be transferred under 1D
#define MSD_SIF_EXT_XFR_PARAM_NUM_WORDS(a)   ((a & 0x1fff)<<0) // number of words to be transferred under 1D
#define MSD_SIF_EXT_XFR_BASE_ADDR_SET(a)  ( a & 0xffffff) // 31:3

#define MSD_SIF_PWT_BASE_ADDR_POS           0x0
#define MSD_SIF_PWT_BASE_ADDR_MASK          0x3FFF
#define MSD_SIF_PXD_BASE_ADDR_POS           0x10
#define MSD_SIF_PXD_BASE_ADDR_MASK          0x3FFF

#define MSD_SIF_PUT_PWT_BASE(w)        (( w & MSD_SIF_PWT_BASE_ADDR_MASK ) << MSD_SIF_PWT_BASE_ADDR_POS )
#define MSD_SIF_PUT_PXD_BASE(w)        (( w & MSD_SIF_PXD_BASE_ADDR_MASK ) << MSD_SIF_PXD_BASE_ADDR_POS )

#define MSD_SIF_BSD_BASE_ADDR_POS           0x0
#define MSD_SIF_BSD_BASE_ADDR_MASK          0x3FFF
#define MSD_SIF_DBFC_BASE_ADDR_POS          0x10
#define MSD_SIF_DBFC_BASE_ADDR_MASK         0x3FFF
#define MSD_SIF_RSB_USE_APB_OFF_POS         0x1F
#define MSD_SIF_RSB_USE_APB_OFF_MASK        0x1

#define MSD_SIF_PUT_BSD_BASE(w)        (( w & MSD_SIF_BSD_BASE_ADDR_MASK ) << MSD_SIF_BSD_BASE_ADDR_POS )
#define MSD_SIF_PUT_DBFC_BASE(w)       (( w & MSD_SIF_DBFC_BASE_ADDR_MASK ) << MSD_SIF_DBFC_BASE_ADDR_POS )
#define MSD_SIF_PUT_RSB_USE_APB(w)     (( w & MSD_SIF_RSB_USE_APB_OFF_MASK ) << MSD_SIF_RSB_USE_APB_OFF_POS )

#define MSD_SIF_MPRR_BASE_ADDR_POS          0x0
#define MSD_SIF_MPRR_BASE_ADDR_MASK         0x3FFF
#define MSD_SIF_MPRC_BASE_ADDR_POS          0x10
#define MSD_SIF_MPRC_BASE_ADDR_MASK         0x3FFF

#define MSD_SIF_PUT_MPRR_BASE(w)       (( w & MSD_SIF_MPRR_BASE_ADDR_MASK ) << MSD_SIF_MPRR_BASE_ADDR_POS )
#define MSD_SIF_PUT_MPRC_BASE(w)       (( w & MSD_SIF_MPRC_BASE_ADDR_MASK ) << MSD_SIF_MPRC_BASE_ADDR_POS )


/* MVD VMIF Options */
#define MSD_VMIF_TAG_ACK_ON_FLUSH_POS         0
#define MSD_VMIF_MAX_RD_REQ_SIZE_POS          1
#define MSD_VMIF_CACHE_ENAB_POS               3
#define MSD_VMIF_CACHE_RST_STAT_POS           4
#define MSD_VMIF_SPLIT_READS_POS              5
#define MSD_VMIF_SPLIT_WRITES_POS             6
#define MSD_VMIF_FS_OFFSET_8PIX_POS           7
#define MSD_VMIF_FS_OFFSET_CHROMA_8PIX_POS    8
#define MSD_VMIF_CACHE_16WAY_MODE_POS         9
#define MSD_VMIF_SCALE_MODE_POS               10
#define MSD_VMIF_RSB_EXT_CACHE_ENAB           11
#define MSD_VMIF_MAX_RD_SIZE_CONFIG_POS       12
#define MSD_VMIF_MBI_RD_BUS_ENAB_POS          13
#define MSD_VMIF_CACHE_DIS_AUTO_RST_POS       15
#define MSD_VMIF_MBI_WR_BUS_ENAB_POS          16

#define MSD_VMIF_DISABLE_BIT_POS              18
#define MSD_VMIF_USE_RPR_STRIDES_POS          19
#define MSD_VMIF_DBG_SEL_POS                  20
#define MSD_VMIF_USE_4K_CACHE_CTRL_POS        23
#define MSD_VMIF_4K_CACHE_ENAB_POS            24
#define MSD_VMIF_4K_CACHE_RST_STAT_POS        25
#define MSD_VMIF_4K_CACHE_DIS_AUTO_RST_POS    26
#define MSD_VMIF_PACK_FORMAT_POS              27
#define MSD_VMIF_CACHE_MODE_POS               28
#define MSD_VMIF_MPSAL_POS                    30
#define MSD_VMIF_CLR_FSBA_POS                 31

/* Extension : MVD VMIF Options */
/* With the introduction of Pixif some of thee bits have been re-used */
#define MSD_VMIF_USE_FBC_RPR_MASK             0x08000000             /* Bit 27      */
#define MSD_VMIF_USE_FBC_RPR_POS              27

#define MSD_VMIF_USE_MBI_RD_RPR_POS           13
#define MSD_VMIF_USE_MBI_RD_CSC_POS           14
#define MSD_VMIF_USE_MBI_RD_MASK              0x0006000              /* Bits 14: 13 */

#define MSD_VMIF_USE_MBI_WR_RPR_POS           23
#define MSD_VMIF_USE_MBI_WR_CSC_POS           24
#define MSD_VMIF_USE_MBI_WR_JPG_DBF_POS       25
#define MSD_VMIF_USE_MBI_WR_MP2_VC1_POS       26
#define MSD_VMIF_USE_MBI_WR_MASK              0x7800000              /* Bits 26: 23 */

#define MSD_VMIF_MAX_RD_128                0
#define MSD_VMIF_MAX_RD_256                1
#define MSD_VMIF_MAX_RD_512                2

/* MVD VMIF RSB ERT ARBITER CONTROL */

#define MSD_VMIF_RSB_CLIENT_DBF            0x0
#define MSD_VMIF_RSB_CLIENT_MPR            0x1
#define MSD_VMIF_RSB_CLIENT_PXD            0x2
#define MSD_VMIF_RSB_CLIENT_BSD            0x3
#define MSD_VMIF_RSB_CLIENT_PWT            0x4
#define MSD_VMIF_RSB_CLIENT_HUF            0x5
#define MSD_VMIF_RSB_CLIENT_MTL            0x6
#define MSD_VMIF_RSB_CLIENT_APB            0x7
#define MSD_VMIF_RSB_CLIENT_XFR            0x8
#define MSD_VMIF_RSB_CLIENT_BBD            0x9
#define MSD_VMIF_RSB_CLIENT_CQ             0xa
#define MSD_VMIF_RSB_CLIENT_BOB            0xb
#define MSD_VMIF_RSB_CLIENT_DFE            0xb   /* Yes this is deliberately the same - they replace the other in certain core configs */
#define MSD_VMIF_RSB_CLIENT_CQBE           0xc

#define MVD_RSB_ARB_CLIENT_MASK            0xFFFF
#define MVD_RSB_ARB_READ_CLIENT_SHIFT      0x10

/* RSB Defines */

#define MSD_RSB_XFR_TYPE_LUMA    0x0
#define MSD_RSB_XFR_TYPE_CHROMA  0x1
#define MSD_RSB_XFR_TYPE_MBINFO  0x2
#define MSD_RSB_XFR_TYPE_DATA    0x3

#define MSD_RSB_MODE_ONCHIP_TOEXT    0x0 // direct is chip to ext ; is chip transaction with ext ;
#define MSD_RSB_MODE_OFFCHIP_TOEXT   0x1 // direct is OFF chip to ext ; is off RSB transaction with ext ; off RSB is usable ; on chip not used
#define MSD_RSB_MODE_ONCHIP_FREXT    0x2 // direct is ext to chip ; is chip transaction with ext ;
#define MSD_RSB_MODE_OFFCHIP_FREXT   0x3 // direct is ext to off RSB ; is off RSB transaction with ext ;off RSB is usable ; on chip not used
#define MSD_RSB_APB_ACC_EN_BIT       0x80000000


/* MSD RPR Defines */
#define MSD_RPR_SIZE_WIDTH_POS         0
#define MSD_RPR_SIZE_HEIGHT_POS       16
#define MSD_RPR_SIZE_WIDTH_MASK   0x3fff //14 bits

#define MSD_RPR_FILL_VALUE_Y_POS       0
#define MSD_RPR_FILL_VALUE_U_POS       8
#define MSD_RPR_FILL_VALUE_V_POS      16
#define MSD_RPR_FILL_VALUE_MASK     0xff  // 8bits

#define MSD_RPR_CTRL_START_POS         0
#define MSD_RPR_CTL_FILL_MODE_POS      1
#define MSD_RPR_CTL_ROUNDING_POS       2
#define MSD_RPR_CTL_H_PRIME_M_POS      8
#define MSD_RPR_CTL_V_PRIME_N_POS     12
#define MSD_RPR_CTL_PRIME_MASK       0xf
#define MSD_RPR_CTL_IN_FS_IDC_POS     16
#define MSD_RPR_CTL_OUT_FS_IDC_POS    24
#define MSD_RPR_CTL_FS_IDC_MASK     0x1f

#define MSD_RPR_CTRL_START_BIT         1

#define MSD_RPR_STATUS_IN_PROG_POS    29


/* Decoupled data packing */
/* Pack Base Set */
#define MSD_SIF_PLQ_PACK_ADDR_MASK        0xFFFFFFFF
#define MSD_SIF_PLQ_PACK_ADDR_SHIFT       0x0

#define MSD_SIF_SET_PLQ_PACK_ADDR( w )    (( w & MSD_SIF_PLQ_WRPACK_ADDR_MASK ) << MSD_SIF_PLQ_WRPACK_ADDR_SHIFT )

/* PRB Bin configuration */
#define MSD_SIF_PLQ_BIN_SIZE_MASK        0x3FFFFF
#define MSD_SIF_PLQ_BIN_SIZE_SHIFT       0x0
#define MSD_SIF_PLQ_NUM_BINS_MASK        0x3FF
#define MSD_SIF_PLQ_NUM_BINS_SHIFT       0x16

#define MSD_SIF_SET_PLQ_BIN_SIZE( w )    (( w & MSD_SIF_PLQ_BIN_SIZE_MASK ) << MSD_SIF_PLQ_BIN_SIZE_SHIFT )
#define MSD_SIF_SET_PLQ_NUM_BINS( w )    (( w & MSD_SIF_PLQ_NUM_BINS_MASK ) << MSD_SIF_PLQ_NUM_BINS_SHIFT )

/* DCP Bin configuration */
#define MSD_SIF_DCP_START_BINDEX_MASK    0x3FF
#define MSD_SIF_DCP_START_BINDEX_SHIFT   0x0
#define MSD_SIF_DCP_NUM_BINS_MASK        0x3FF
#define MSD_SIF_DCP_NUM_BINS_SHIFT       0xA

#define MSD_SIF_SET_DCP_START_BINDEX( w )(( w & MSD_SIF_DCP_START_BINDEX_MASK ) << MSD_SIF_DCP_START_BINDEX_SHIFT )
#define MSD_SIF_SET_DCP_NUM_BINS( w )    (( w & MSD_SIF_DCP_NUM_BINS_MASK     ) << MSD_SIF_DCP_NUM_BINS_SHIFT     )


/////////////////////////////////////////////////////////////////////////////////
//  Function Prototypes
/////////////////////////////////////////////////////////////////////////////////

#endif /* _MVD_SIF_CONTROL_H_ */

/* End of file */
