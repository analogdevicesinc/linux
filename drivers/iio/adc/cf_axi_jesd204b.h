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
 */

#ifndef ADI_JESD204B_H_
#define ADI_JESD204B_H_

/* PCORE CoreFPGA register map */

#define AXI_JESD204B_REG_VERSION	0x00
#define AXI_JESD204B_REG_CTRL		0x04
#define AXI_JESD204B_REG_SYSREF_TRIG	0x08
#define AXI_JESD204B_REG_FRMCTRL	0x0C
#define AXI_JESD204B_REG_BUFDELAY	0x10
#define AXI_JESD204B_REG_TEST_MODE	0x14
#define AXI_JESD204B_REG_BUFCNT		0x18  
#define AXI_JESD204B_REG_INIT_DATA0	0x1C
#define AXI_JESD204B_REG_INIT_DATA1	0x20
#define AXI_JESD204B_REG_INIT_DATA2	0x24
#define AXI_JESD204B_REG_INIT_DATA3	0x28
#define AXI_JESD204B_REG_TEST_MFCNT	0x2C
#define AXI_JESD204B_REG_TEST_ILACNT	0x30
#define AXI_JESD204B_REG_TEST_ERRCNT	0x34

/* AXI_JESD204B_REG_CTRL */
#define AXI_JESD204B_CTRL_ERR_DIS	(1 << 0)
#define AXI_JESD204B_CTRL_SYSREF_EN	(1 << 1)
#define AXI_JESD204B_CTRL_SCR_EN	(1 << 2)
#define AXI_JESD204B_CTRL_LANESYNC_EN	(1 << 3)
#define AXI_JESD204B_CTRL_SYSREF_SW_EN	(1 << 4)
#define AXI_JESD204B_CTRL_SYSREF_HW_EN	(0 << 4)

/* AXI_JESD204B_REG_SYSREF_TRIG */
#define AXI_JESD204B_SYSREF_TRIG	(1 << 0)

/* AXI_JESD204B_REG_FRMCTRL */
#define AXI_JESD204B_FRMCTRL_FRMCNT(x)	((x) << 8)
#define AXI_JESD204B_FRMCTRL_BYTECNT(x)	(((x) & 0xFF) << 0)

/* AXI_JESD204B_REG_BUFDELAY */
#define AXI_JESD204B_BUFDELAY(x)	((x) << 0)

#endif /* ADI_JESD204B_H_ */
