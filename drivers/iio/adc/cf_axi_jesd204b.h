/*
 * ADI AXI-JESD204B Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

/*
 * 0x00   0x00   [31: 0]  version[31:0]         32'h00010061  (1.0a)
 * ---------------------------------------------------------------------------
 * 0x01   0x04   [ 4: 4]  sysref_sel            Select sw(0x1) or hw(0x0) sysref generation
 *               [ 3: 3]  lanesync_enb          Enable (0x1) lane synchronization
 *               [ 2: 2]  scr_enb               Enable (0x1) scrambling
 *               [ 1: 1]  sysref_enb            Enable (0x1) re-alignment at every sysref pulses
 *               [ 0: 0]  err_disb              Disable (0x1) error reporting via sync
 * ---------------------------------------------------------------------------
 * 0x02   0x08   [ 0: 0]  sysref_int            Software controlled sysref (0->1 transition)
 * ---------------------------------------------------------------------------
 * 0x03   0x0c   [12: 8]  frmcnt                Number of frames per multi-frame (n-1)
 *               [ 7: 0]  bytecnt               Number of bytes (octets) per frame (n-1)
 * ---------------------------------------------------------------------------
 * 0x04   0x10   [12: 0]  buffdelay             Buffer delay from multi-frame
 * ---------------------------------------------------------------------------
 * 0x05   0x14   [ 3: 2]  test_mode             Test mode (?)
 *               [ 1: 0]  lane_sel              Select lanes (0, 1, 2 or 3) for reporting
 * ---------------------------------------------------------------------------
 * The following registers are based on lane_sel
 * ---------------------------------------------------------------------------
 * 0x06   0x18   [31: 0]  bufcnt                Buffer count (latency)
 * 0x07   0x1c   [31: 0]  init_data0            ILS Data 0
 * 0x08   0x20   [31: 0]  init_data1            ILS Data 1
 * 0x09   0x24   [31: 0]  init_data2            ILS Data 2
 * 0x0a   0x28   [31: 0]  init_data3            ILS Data 3
 * 0x0b   0x2c   [31: 0]  test_mfcnt            Test MF count (?)
 * 0x0c   0x30   [31: 0]  test_ilacnt           Test ILA count (?)
 * 0x0d   0x34   [31: 0]  test_errcnt           Test ERROR count (?)
 * ---------------------------------------------------------------------------
 * 0x10   0x40   [ 1: 1]  es_mode               Eye scan data mode (see below)
 *               [ 0: 0]  es_start              Start eye scan capture (0->1 transition)
 *
 * The eye scan data mode supports two fromats-
 *   If mode is set (0x1), data consists of both sample and error counts.
 *      data[31:16] = sample_count; and
 *      data[15: 0] = error_count;
 *   If mode is set (0x0), data consists only the error count and is filled as an RGB value
 *      data[31:24] = 0x00;
 *      data[23: 8] = error_count; and
 *      data[ 7: 0] = 0x00;
 *   This allows direct color coding, zero errors will be black.
 * ---------------------------------------------------------------------------
 * 0x11   0x44   [20:16]  prescale_step         Prescale step
 *               [12: 8]  prescale_max          Prescale maximum
 *               [ 4: 0]  prescale_min          Prescale minimum
 * ---------------------------------------------------------------------------
 * 0x12   0x48   [23:16]  voffset_step          Vertical offset (voltage) step
 *               [15: 8]  voffset_max           Vertical offset (voltage) maximum
 *               [ 7: 0]  voffset_min           Vertical offset (voltage) minimum
 * ---------------------------------------------------------------------------
 * 0x13   0x4c   [27:16]  hoffset_max           Horizontal offset (time) maximum
 *               [11: 0]  hoffset_min           Horizontal offset (time) minimum
 * ---------------------------------------------------------------------------
 * 0x14   0x50   [11: 0]  hoffset_step          Horizontal offset (time) step
 * ---------------------------------------------------------------------------
 * 0x15   0x54   [31: 0]  start_address         Buffer start address
 * ---------------------------------------------------------------------------
 * 0x16   0x58   [15: 0]  hsize                 Buffer horizontal size
 * ---------------------------------------------------------------------------
 * 0x17   0x5c   [31:16]  hmax                  Buffer horizontal maximum
 *               [15: 0]  hmin                  Buffer horizontal minimum
 * ---------------------------------------------------------------------------
 * 0x18   0x60   [ 5: 5]  ovf                   Buffer overflow (RW1C)
 *               [ 4: 4]  unf                   Buffer underflow (RW1C)
 *               [ 3: 3]  error                 Target Error (RW1C)
 *               [ 2: 2]  ready                 Target Ready (RW1C)
 *               [ 1: 1]  mb_state              MB busy (0x1) or idle (0x0)
 *               [ 0: 0]  es_state              ES busy (0x1) or idle (0x0)
 */

#ifndef ADI_JESD204B_H_
#define ADI_JESD204B_H_

/* PCORE CoreFPGA register map */

#define AXI_JESD204B_REG_VERSION		0x00
#define AXI_JESD204B_REG_CTRL		0x04
#define AXI_JESD204B_REG_SYSREF_TRIG	0x08
#define AXI_JESD204B_REG_FRMCTRL		0x0C
#define AXI_JESD204B_REG_BUFDELAY		0x10
#define AXI_JESD204B_REG_TEST_MODE	0x14
#define AXI_JESD204B_REG_BUFCNT		0x18
#define AXI_JESD204B_REG_INIT_DATA0	0x1C
#define AXI_JESD204B_REG_INIT_DATA1	0x20
#define AXI_JESD204B_REG_INIT_DATA2	0x24
#define AXI_JESD204B_REG_INIT_DATA3	0x28
#define AXI_JESD204B_REG_TEST_MFCNT	0x2C
#define AXI_JESD204B_REG_TEST_ILACNT	0x30
#define AXI_JESD204B_REG_TEST_ERRCNT	0x34
#define AXI_JESD204B_REG_ES_CTRL		0x40
#define AXI_JESD204B_REG_ES_PRESCALE	0x44
#define AXI_JESD204B_REG_ES_VOFFSET	0x48
#define AXI_JESD204B_REG_ES_HOFFSET	0x4C
#define AXI_JESD204B_REG_ES_HOFFSET_STP	0x50
#define AXI_JESD204B_REG_ES_START_ADDR	0x54
#define AXI_JESD204B_REG_ES_HSIZE		0x58
#define AXI_JESD204B_REG_ES_HMAXMIN	0x5C
#define AXI_JESD204B_REG_ES_STATUS	0x60

/* AXI_JESD204B_REG_CTRL */
#define AXI_JESD204B_CTRL_ERR_DIS			(1 << 0)
#define AXI_JESD204B_CTRL_SYSREF_EN		(1 << 1)
#define AXI_JESD204B_CTRL_SCR_EN			(1 << 2)
#define AXI_JESD204B_CTRL_LANESYNC_EN		(1 << 3)
#define AXI_JESD204B_CTRL_SYSREF_SW_EN		(1 << 4)
#define AXI_JESD204B_CTRL_SYSREF_HW_EN		(0 << 4)

/* AXI_JESD204B_REG_SYSREF_TRIG */
#define AXI_JESD204B_SYSREF_TRIG			(1 << 0)

/* AXI_JESD204B_REG_FRMCTRL */
#define AXI_JESD204B_FRMCTRL_FRMCNT(x)		((x) << 8)
#define AXI_JESD204B_FRMCTRL_BYTECNT(x)		(((x) & 0xFF) << 0)

/* AXI_JESD204B_REG_BUFDELAY */
#define AXI_JESD204B_BUFDELAY(x)			((x) << 0)

/* AXI_JESD204B_REG_TEST_MODE */
#define AXI_JESD204B_REG_TEST_MODE_LS(x)		((x) << 0)
#define AXI_JESD204B_REG_TEST_MODE_EN(x)		((x) << 2)
#define AXI_JESD204B_REG_TEST_MODE_JESD_RESET	(1 << 4)
#define AXI_JESD204B_REG_TEST_MODE_GTX_RESET	(1 << 5)

/* AXI_JESD204B_REG_INIT_DATA0 */
#define AXI_JESD204B_INIT0_DID(x)	(((x) >> 0) & 0xFF) /* DID Device ID */
#define AXI_JESD204B_INIT0_BID(x)	(((x) >> 8) & 0xFF) /* BID Bank ID */
#define AXI_JESD204B_INIT0_LID(x)	(((x) >> 13) & 0x1F) /* LID Lane ID */
#define AXI_JESD204B_INIT0_L(x)		(((x) >> 18) & 0x1F) /* Number of Lanes per Device*/
#define AXI_JESD204B_INIT0_SCR(x)	(((x) >> 23) & 0x1) /* SCR Scrambling Enabled */
#define AXI_JESD204B_INIT0_F(x)		(((x) >> 24) & 0xFF) /* Octets per Frame */

/* AXI_JESD204B_REG_INIT_DATA1 */

#define AXI_JESD204B_INIT1_K(x)		(((x) >> 0) & 0x1F) /* Frames per Multiframe */
#define AXI_JESD204B_INIT1_M(x)		(((x) >> 5) & 0xFF) /* Converters per Device */
#define AXI_JESD204B_INIT1_N(x)		(((x) >> 13) & 0x1F) /* Converter Resolution */
#define AXI_JESD204B_INIT1_CS(x)		(((x) >> 18) & 0x3) /* Control Bits per Sample */
#define AXI_JESD204B_INIT1_S(x)		(((x) >> 20) & 0x1F) /* Samples per Converter per Frame Cycle */
#define AXI_JESD204B_INIT1_ND(x)		(((x) >> 25) & 0x1F) /* Total Bits per Sample */
#define AXI_JESD204B_INIT1_HD(x)		(((x) >> 30) & 0x1) /* High Density Format */

/* AXI_JESD204B_REG_INIT_DATA2 */

#define AXI_JESD204B_INIT2_FCHK(x)	(((x) >> 16) & 0xFF) /* Checksum */
#define AXI_JESD204B_INIT2_CF(x)		(((x) >> 24) & 0x1F) /* Control Words per Frame Cycle per Link */

/* AXI_JESD204B_REG_INIT_DATA3 */

#define AXI_JESD204B_INIT3_ADJCNT(x)		(((x) >> 0) & 0xF) /* ADJCNT Adjustment step count */
#define AXI_JESD204B_INIT3_PHYADJ(x)		(((x) >> 4) & 0x1) /* PHYADJ Adjustment request */
#define AXI_JESD204B_INIT3_ADJDIR(x)		(((x) >> 5) & 0x1) /* ADJDIR Adjustment direction */
#define AXI_JESD204B_INIT3_JESDV(x)		(((x) >> 6) & 0x7) /* JESD204 Version */
#define AXI_JESD204B_INIT3_SUBCLASSV(x)		(((x) >> 9) & 0x7) /* JESD204 subclass version */

/* AXI_JESD204B_REG_ES_CTRL */
#define AXI_JESD204B_REG_ES_CTRL_START		(1 << 0)
#define AXI_JESD204B_REG_ES_CTRL_NORM		(1 << 1)
#define AXI_JESD204B_REG_ES_CTRL_RGB		(0 << 1)

/* AXI_JESD204B_REG_ES_PRESCALE */
#define AXI_JESD204B_REG_ES_PRESCALE_STEP(x)	(((x) & 0x1F) << 16)
#define AXI_JESD204B_REG_ES_PRESCALE_MAX(x)	(((x) & 0x1F) << 8)
#define AXI_JESD204B_REG_ES_PRESCALE_MIN(x)	((x) & 0x1F)

/* AXI_JESD204B_REG_ES_VOFFSET */
#define AXI_JESD204B_REG_ES_VOFFSET_STEP(x)	(((x) & 0xFF) << 16)
#define AXI_JESD204B_REG_ES_VOFFSET_MAX(x)	(((x) & 0xFF) << 8)
#define AXI_JESD204B_REG_ES_VOFFSET_MIN(x)	(((unsigned)(x)) & 0xFF)

/* AXI_JESD204B_REG_ES_HOFFSET */
#define AXI_JESD204B_REG_ES_HOFFSET_MAX(x)	(((x) & 0xFFF) << 16)
#define AXI_JESD204B_REG_ES_HOFFSET_MIN(x)	(((unsigned)(x)) & 0xFFF)

/* AXI_JESD204B_REG_ES_HMAXMIN */
#define AXI_JESD204B_REG_ES_HMAXMIN_MAX(x)	((x) << 16)
#define AXI_JESD204B_REG_ES_HMAXMIN_MIN(x)	((x) & 0xFFFF)

/* AXI_JESD204B_REG_ES_STATUS */
#define AXI_JESD204B_REG_ES_STATUS_ES_BUSY	(1 << 0)
#define AXI_JESD204B_REG_ES_STATUS_MB_BUSY	(1 << 1)
#define AXI_JESD204B_REG_ES_STATUS_READY		(1 << 2)
#define AXI_JESD204B_REG_ES_STATUS_ERROR		(1 << 3)
#define AXI_JESD204B_REG_ES_STATUS_BUF_UNF	(1 << 4)
#define AXI_JESD204B_REG_ES_STATUS_BUF_OVF	(1 << 5)

#endif /* ADI_JESD204B_H_ */
