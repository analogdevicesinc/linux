// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DRM Driver for DSI output on Raspberry Pi RP1
 *
 * Copyright (c) 2023 Raspberry Pi Limited.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/rp1_platform.h>
#include "drm/drm_print.h"

#include "rp1_dsi.h"

/* ------------------------------- Synopsis DSI ------------------------ */
#define     DSI_VERSION_CFG                       0x000
#define     DSI_PWR_UP                            0x004
#define     DSI_CLKMGR_CFG                        0x008
#define     DSI_DPI_VCID                          0x00C
#define     DSI_DPI_COLOR_CODING                  0x010
#define     DSI_DPI_CFG_POL                       0x014
#define     DSI_DPI_LP_CMD_TIM                    0x018
#define     DSI_DBI_VCID                          0x01C
#define     DSI_DBI_CFG                           0x020
#define     DSI_DBI_PARTITIONING_EN               0x024
#define     DSI_DBI_CMDSIZE                       0x028
#define     DSI_PCKHDL_CFG                        0x02C
#define     DSI_GEN_VCID                          0x030
#define     DSI_MODE_CFG                          0x034
#define     DSI_VID_MODE_CFG                      0x038
#define     DSI_VID_PKT_SIZE                      0x03C
#define     DSI_VID_NUM_CHUNKS                    0x040
#define     DSI_VID_NULL_SIZE                     0x044
#define     DSI_VID_HSA_TIME                      0x048
#define     DSI_VID_HBP_TIME                      0x04C
#define     DSI_VID_HLINE_TIME                    0x050
#define     DSI_VID_VSA_LINES                     0x054
#define     DSI_VID_VBP_LINES                     0x058
#define     DSI_VID_VFP_LINES                     0x05C
#define     DSI_VID_VACTIVE_LINES                 0x060
#define     DSI_EDPI_CMD_SIZE                     0x064
#define     DSI_CMD_MODE_CFG                      0x068
#define     DSI_GEN_HDR                           0x06C
#define     DSI_GEN_PLD_DATA                      0x070
#define     DSI_CMD_PKT_STATUS                    0x074
#define     DSI_TO_CNT_CFG                        0x078
#define     DSI_HS_RD_TO_CNT                      0x07C
#define     DSI_LP_RD_TO_CNT                      0x080
#define     DSI_HS_WR_TO_CNT                      0x084
#define     DSI_LP_WR_TO_CNT                      0x088
#define     DSI_BTA_TO_CNT                        0x08C
#define     DSI_SDF_3D                            0x090
#define     DSI_LPCLK_CTRL                        0x094
#define     DSI_PHY_TMR_LPCLK_CFG                 0x098
#define     DSI_PHY_TMR_HS2LP_LSB       16
#define     DSI_PHY_TMR_LP2HS_LSB       0
#define     DSI_PHY_TMR_CFG                       0x09C
#define     DSI_PHY_TMR_RD_CFG                    0x0F4
#define     DSI_PHYRSTZ                           0x0A0
#define     DSI_PHY_IF_CFG                        0x0A4
#define     DSI_PHY_ULPS_CTRL                     0x0A8
#define     DSI_PHY_TX_TRIGGERS                   0x0AC
#define     DSI_PHY_STATUS                        0x0B0

#define     DSI_PHY_TST_CTRL0                     0x0B4
#define     DSI_PHY_TST_CTRL1                     0x0B8
#define     DSI_INT_ST0                           0x0BC
#define     DSI_INT_ST1                           0x0C0
#define     DSI_INT_MASK0_CFG                     0x0C4
#define     DSI_INT_MASK1_CFG                     0x0C8
#define     DSI_PHY_CAL                           0x0CC
#define     DSI_HEXP_NPKT_CLR                     0x104
#define     DSI_HEXP_NPKT_SIZE                    0x108
#define     DSI_VID_SHADOW_CTRL                   0x100

#define     DSI_DPI_VCID_ACT                      0x10C
#define     DSI_DPI_COLOR_CODING_ACT              0x110
#define     DSI_DPI_LP_CMD_TIM_ACT                0x118
#define     DSI_VID_MODE_CFG_ACT                  0x138
#define     DSI_VID_PKT_SIZE_ACT                  0x13C
#define     DSI_VID_NUM_CHUNKS_ACT                0x140
#define     DSI_VID_NULL_SIZE_ACT                 0x144
#define     DSI_VID_HSA_TIME_ACT                  0x148
#define     DSI_VID_HBP_TIME_ACT                  0x14C
#define     DSI_VID_HLINE_TIME_ACT                0x150
#define     DSI_VID_VSA_LINES_ACT                 0x154
#define     DSI_VID_VBP_LINES_ACT                 0x158
#define     DSI_VID_VFP_LINES_ACT                 0x15C
#define     DSI_VID_VACTIVE_LINES_ACT             0x160
#define     DSI_SDF_3D_CFG_ACT                    0x190

#define     DSI_INT_FORCE0                        0x0D8
#define     DSI_INT_FORCE1                        0x0DC

#define     DSI_AUTO_ULPS_MODE                    0x0E0
#define     DSI_AUTO_ULPS_ENTRY_DELAY             0x0E4
#define     DSI_AUTO_ULPS_WAKEUP_TIME             0x0E8
#define     DSI_EDPI_ADV_FEATURES                 0x0EC

#define     DSI_DSC_PARAMETER                     0x0F0

/* And some bitfield definitions */

#define DSI_PCKHDL_EOTP_TX_EN  BIT(0)
#define DSI_PCKHDL_BTA_EN      BIT(2)

#define DSI_VID_MODE_LP_CMD_EN        BIT(15)
#define DSI_VID_MODE_FRAME_BTA_ACK_EN BIT(14)
#define DSI_VID_MODE_LP_HFP_EN        BIT(13)
#define DSI_VID_MODE_LP_HBP_EN        BIT(12)
#define DSI_VID_MODE_LP_VACT_EN       BIT(11)
#define DSI_VID_MODE_LP_VFP_EN        BIT(10)
#define DSI_VID_MODE_LP_VBP_EN        BIT(9)
#define DSI_VID_MODE_LP_VSA_EN        BIT(8)
#define DSI_VID_MODE_SYNC_PULSES      0
#define DSI_VID_MODE_SYNC_EVENTS      1
#define DSI_VID_MODE_BURST            2

#define DSI_CMD_MODE_ALL_LP           0x10f7f00
#define DSI_CMD_MODE_ACK_RQST_EN      BIT(1)

#define DPHY_PWR_UP_SHUTDOWNZ_LSB 0
#define DPHY_PWR_UP_SHUTDOWNZ_BITS BIT(DPHY_PWR_UP_SHUTDOWNZ_LSB)

#define DPHY_CTRL0_PHY_TESTCLK_LSB 1
#define DPHY_CTRL0_PHY_TESTCLK_BITS BIT(DPHY_CTRL0_PHY_TESTCLK_LSB)
#define DPHY_CTRL0_PHY_TESTCLR_LSB 0
#define DPHY_CTRL0_PHY_TESTCLR_BITS BIT(DPHY_CTRL0_PHY_TESTCLR_LSB)

#define DPHY_CTRL1_PHY_TESTDIN_LSB  0
#define DPHY_CTRL1_PHY_TESTDIN_BITS  (0xff << DPHY_CTRL1_PHY_TESTDIN_LSB)
#define DPHY_CTRL1_PHY_TESTDOUT_LSB 8
#define DPHY_CTRL1_PHY_TESTDOUT_BITS (0xff << DPHY_CTRL1_PHY_TESTDOUT_LSB)
#define DPHY_CTRL1_PHY_TESTEN_LSB 16
#define DPHY_CTRL1_PHY_TESTEN_BITS BIT(DPHY_CTRL1_PHY_TESTEN_LSB)

#define DSI_PHYRSTZ_SHUTDOWNZ_LSB  0
#define DSI_PHYRSTZ_SHUTDOWNZ_BITS BIT(DSI_PHYRSTZ_SHUTDOWNZ_LSB)
#define DSI_PHYRSTZ_RSTZ_LSB  1
#define DSI_PHYRSTZ_RSTZ_BITS BIT(DSI_PHYRSTZ_RSTZ_LSB)
#define DSI_PHYRSTZ_ENABLECLK_LSB 2
#define DSI_PHYRSTZ_ENABLECLK_BITS BIT(DSI_PHYRSTZ_ENABLECLK_LSB)
#define DSI_PHYRSTZ_FORCEPLL_LSB 3
#define DSI_PHYRSTZ_FORCEPLL_BITS  BIT(DSI_PHYRSTZ_FORCEPLL_LSB)

#define DPHY_HS_RX_CTRL_LANE0_OFFSET  0x44
#define DPHY_PLL_INPUT_DIV_OFFSET 0x17
#define DPHY_PLL_LOOP_DIV_OFFSET 0x18
#define DPHY_PLL_DIV_CTRL_OFFSET 0x19

#define DPHY_PLL_BIAS_OFFSET 0x10
#define DPHY_PLL_BIAS_VCO_RANGE_LSB 3
#define DPHY_PLL_BIAS_USE_PROGRAMMED_VCO_RANGE BIT(7)

#define DPHY_PLL_CHARGE_PUMP_OFFSET 0x11
#define DPHY_PLL_LPF_OFFSET 0x12

#define DSI_WRITE(reg, val)  writel((val),  dsi->hw_base[RP1DSI_HW_BLOCK_DSI] + (reg))
#define DSI_READ(reg)        readl(dsi->hw_base[RP1DSI_HW_BLOCK_DSI] + (reg))

// ================================================================================
// Register block : RPI_MIPICFG
// Version        : 1
// Bus type       : apb
// Description    : Register block to control mipi DPHY
// ================================================================================
#define RPI_MIPICFG_REGS_RWTYPE_MSB 13
#define RPI_MIPICFG_REGS_RWTYPE_LSB 12
// ================================================================================
// Register    : RPI_MIPICFG_CLK2FC
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_CLK2FC_OFFSET 0x00000000
#define RPI_MIPICFG_CLK2FC_BITS   0x00000007
#define RPI_MIPICFG_CLK2FC_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_CLK2FC_SEL
// Description : select a clock to be sent to the frequency counter
//               7 = none
//               6 = none
//               5 = none
//               4 = rxbyteclkhs (187.5MHz)
//               3 = rxclkesc0 (20MHz)
//               2 = txbyteclkhs (187.5MHz)
//               1 = txclkesc (125MHz)
//               0 = none
#define RPI_MIPICFG_CLK2FC_SEL_RESET  0x0
#define RPI_MIPICFG_CLK2FC_SEL_BITS   0x00000007
#define RPI_MIPICFG_CLK2FC_SEL_MSB    2
#define RPI_MIPICFG_CLK2FC_SEL_LSB    0
#define RPI_MIPICFG_CLK2FC_SEL_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_CFG
// JTAG access : asynchronous
// Description : Top level configuration
#define RPI_MIPICFG_CFG_OFFSET 0x00000004
#define RPI_MIPICFG_CFG_BITS   0x00000111
#define RPI_MIPICFG_CFG_RESET  0x00000001
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_CFG_DPIUPDATE
// Description : Indicate the DSI block that the next frame will have a new video configuration
#define RPI_MIPICFG_CFG_DPIUPDATE_RESET  0x0
#define RPI_MIPICFG_CFG_DPIUPDATE_BITS   0x00000100
#define RPI_MIPICFG_CFG_DPIUPDATE_MSB    8
#define RPI_MIPICFG_CFG_DPIUPDATE_LSB    8
#define RPI_MIPICFG_CFG_DPIUPDATE_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_CFG_SEL_TE_EXT
// Description : Select the TE source: 1 - ext, 0 - int
#define RPI_MIPICFG_CFG_SEL_TE_EXT_RESET  0x0
#define RPI_MIPICFG_CFG_SEL_TE_EXT_BITS   0x00000010
#define RPI_MIPICFG_CFG_SEL_TE_EXT_MSB    4
#define RPI_MIPICFG_CFG_SEL_TE_EXT_LSB    4
#define RPI_MIPICFG_CFG_SEL_TE_EXT_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_CFG_SEL_CSI_DSI_N
// Description : Select PHY direction: input to CSI, output from DSI. CSI 1 DSI 0
#define RPI_MIPICFG_CFG_SEL_CSI_DSI_N_RESET  0x1
#define RPI_MIPICFG_CFG_SEL_CSI_DSI_N_BITS   0x00000001
#define RPI_MIPICFG_CFG_SEL_CSI_DSI_N_MSB    0
#define RPI_MIPICFG_CFG_SEL_CSI_DSI_N_LSB    0
#define RPI_MIPICFG_CFG_SEL_CSI_DSI_N_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_TE
// JTAG access : synchronous
// Description : Tearing effect processing
#define RPI_MIPICFG_TE_OFFSET 0x00000008
#define RPI_MIPICFG_TE_BITS   0x10ffffff
#define RPI_MIPICFG_TE_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_TE_ARM
// Description : Tearing effect arm
#define RPI_MIPICFG_TE_ARM_RESET  0x0
#define RPI_MIPICFG_TE_ARM_BITS   0x10000000
#define RPI_MIPICFG_TE_ARM_MSB    28
#define RPI_MIPICFG_TE_ARM_LSB    28
#define RPI_MIPICFG_TE_ARM_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_TE_HALT_CYC
// Description : When arm pulse has been seen, wait for te; then halt the dpi block
//		 for this many clk_dpi cycles
#define RPI_MIPICFG_TE_HALT_CYC_RESET  0x000000
#define RPI_MIPICFG_TE_HALT_CYC_BITS   0x00ffffff
#define RPI_MIPICFG_TE_HALT_CYC_MSB    23
#define RPI_MIPICFG_TE_HALT_CYC_LSB    0
#define RPI_MIPICFG_TE_HALT_CYC_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_MONITOR
// JTAG access : asynchronous
// Description : DPHY status monitors for analog DFT
#define RPI_MIPICFG_DPHY_MONITOR_OFFSET 0x00000010
#define RPI_MIPICFG_DPHY_MONITOR_BITS   0x00111fff
#define RPI_MIPICFG_DPHY_MONITOR_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_MONITOR_LOCK
// Description : None
#define RPI_MIPICFG_DPHY_MONITOR_LOCK_RESET  0x0
#define RPI_MIPICFG_DPHY_MONITOR_LOCK_BITS   0x00100000
#define RPI_MIPICFG_DPHY_MONITOR_LOCK_MSB    20
#define RPI_MIPICFG_DPHY_MONITOR_LOCK_LSB    20
#define RPI_MIPICFG_DPHY_MONITOR_LOCK_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_MONITOR_BISTOK
// Description : None
#define RPI_MIPICFG_DPHY_MONITOR_BISTOK_RESET  0x0
#define RPI_MIPICFG_DPHY_MONITOR_BISTOK_BITS   0x00010000
#define RPI_MIPICFG_DPHY_MONITOR_BISTOK_MSB    16
#define RPI_MIPICFG_DPHY_MONITOR_BISTOK_LSB    16
#define RPI_MIPICFG_DPHY_MONITOR_BISTOK_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK
// Description : None
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK_RESET  0x0
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK_BITS   0x00001000
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK_MSB    12
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK_LSB    12
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATECLK_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA
// Description : None
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA_RESET  0x0
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA_BITS   0x00000f00
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA_MSB    11
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA_LSB    8
#define RPI_MIPICFG_DPHY_MONITOR_STOPSTATEDATA_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_MONITOR_TESTDOUT
// Description : None
#define RPI_MIPICFG_DPHY_MONITOR_TESTDOUT_RESET  0x00
#define RPI_MIPICFG_DPHY_MONITOR_TESTDOUT_BITS   0x000000ff
#define RPI_MIPICFG_DPHY_MONITOR_TESTDOUT_MSB    7
#define RPI_MIPICFG_DPHY_MONITOR_TESTDOUT_LSB    0
#define RPI_MIPICFG_DPHY_MONITOR_TESTDOUT_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_CTRL_0
// JTAG access : asynchronous
// Description : DPHY control for analog DFT
#define RPI_MIPICFG_DPHY_CTRL_0_OFFSET 0x00000014
#define RPI_MIPICFG_DPHY_CTRL_0_BITS   0x0000003f
#define RPI_MIPICFG_DPHY_CTRL_0_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE
// Description : When set in lpmode, TXCLKESC is driven from clk_vec(driven from clocks block)
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE_BITS   0x00000020
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE_MSB    5
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE_LSB    5
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_LPMODE_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA
// Description : When set, drive the DPHY from the test registers
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA_BITS   0x00000010
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA_MSB    4
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA_LSB    4
#define RPI_MIPICFG_DPHY_CTRL_0_TEST_ENA_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS
// Description : When test_ena is set, disable cfg_clk
#define RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS_BITS   0x00000008
#define RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS_MSB    3
#define RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS_LSB    3
#define RPI_MIPICFG_DPHY_CTRL_0_CFG_CLK_DIS_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS
// Description : When test_ena is set, disable refclk
#define RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS_BITS   0x00000004
#define RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS_MSB    2
#define RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS_LSB    2
#define RPI_MIPICFG_DPHY_CTRL_0_REFCLK_DIS_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS
// Description : When test_ena is set, disable txclkesc
#define RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS_BITS   0x00000002
#define RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS_MSB    1
#define RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS_LSB    1
#define RPI_MIPICFG_DPHY_CTRL_0_TXCLKESC_DIS_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS
// Description : When test_ena is set, disable txbyteclkhs
#define RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS_BITS   0x00000001
#define RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS_MSB    0
#define RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS_LSB    0
#define RPI_MIPICFG_DPHY_CTRL_0_TXBYTECLKHS_DIS_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_CTRL_1
// JTAG access : asynchronous
// Description : DPHY control for analog DFT
#define RPI_MIPICFG_DPHY_CTRL_1_OFFSET 0x00000018
#define RPI_MIPICFG_DPHY_CTRL_1_BITS   0x7fffffff
#define RPI_MIPICFG_DPHY_CTRL_1_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL_BITS   0x40000000
#define RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL_MSB    30
#define RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL_LSB    30
#define RPI_MIPICFG_DPHY_CTRL_1_FORCEPLL_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ_BITS   0x20000000
#define RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ_MSB    29
#define RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ_LSB    29
#define RPI_MIPICFG_DPHY_CTRL_1_SHUTDOWNZ_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_RSTZ
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_RSTZ_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_RSTZ_BITS   0x10000000
#define RPI_MIPICFG_DPHY_CTRL_1_RSTZ_MSB    28
#define RPI_MIPICFG_DPHY_CTRL_1_RSTZ_LSB    28
#define RPI_MIPICFG_DPHY_CTRL_1_RSTZ_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ_BITS   0x08000000
#define RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ_MSB    27
#define RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ_LSB    27
#define RPI_MIPICFG_DPHY_CTRL_1_MASTERSLAVEZ_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_BISTON
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_BISTON_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_BISTON_BITS   0x04000000
#define RPI_MIPICFG_DPHY_CTRL_1_BISTON_MSB    26
#define RPI_MIPICFG_DPHY_CTRL_1_BISTON_LSB    26
#define RPI_MIPICFG_DPHY_CTRL_1_BISTON_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK_BITS   0x02000000
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK_MSB    25
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK_LSB    25
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTHSCLK_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK_BITS   0x01000000
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK_MSB    24
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK_LSB    24
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLECLK_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3_BITS   0x00800000
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3_MSB    23
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3_LSB    23
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2_BITS   0x00400000
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2_MSB    22
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2_LSB    22
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1_BITS   0x00200000
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1_MSB    21
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1_LSB    21
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0_BITS   0x00100000
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0_MSB    20
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0_LSB    20
#define RPI_MIPICFG_DPHY_CTRL_1_ENABLE_0_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3_BITS   0x00080000
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3_MSB    19
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3_LSB    19
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2_BITS   0x00040000
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2_MSB    18
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2_LSB    18
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1_BITS   0x00020000
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1_MSB    17
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1_LSB    17
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0_BITS   0x00010000
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0_MSB    16
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0_LSB    16
#define RPI_MIPICFG_DPHY_CTRL_1_BASEDIR_0_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3_BITS   0x00008000
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3_MSB    15
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3_LSB    15
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2_BITS   0x00004000
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2_MSB    14
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2_LSB    14
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1_BITS   0x00002000
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1_MSB    13
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1_LSB    13
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0_BITS   0x00001000
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0_MSB    12
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0_LSB    12
#define RPI_MIPICFG_DPHY_CTRL_1_TXLPDTESC_0_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3_BITS   0x00000800
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3_MSB    11
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3_LSB    11
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2_BITS   0x00000400
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2_MSB    10
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2_LSB    10
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1_BITS   0x00000200
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1_MSB    9
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1_LSB    9
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0_BITS   0x00000100
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0_MSB    8
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0_LSB    8
#define RPI_MIPICFG_DPHY_CTRL_1_TXVALIDESC_0_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3_BITS   0x00000080
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3_MSB    7
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3_LSB    7
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2_BITS   0x00000040
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2_MSB    6
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2_LSB    6
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1_BITS   0x00000020
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1_MSB    5
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1_LSB    5
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0_BITS   0x00000010
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0_MSB    4
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0_LSB    4
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTESC_0_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3_BITS   0x00000008
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3_MSB    3
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3_LSB    3
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2_BITS   0x00000004
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2_MSB    2
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2_LSB    2
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1_BITS   0x00000002
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1_MSB    1
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1_LSB    1
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0_BITS   0x00000001
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0_MSB    0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0_LSB    0
#define RPI_MIPICFG_DPHY_CTRL_1_TXREQUESTDATAHS_0_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_CTRL_2
// JTAG access : asynchronous
// Description : DPHY control for analog DFT
#define RPI_MIPICFG_DPHY_CTRL_2_OFFSET 0x0000001c
#define RPI_MIPICFG_DPHY_CTRL_2_BITS   0x000007ff
#define RPI_MIPICFG_DPHY_CTRL_2_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_2_TESTCLK
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLK_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLK_BITS   0x00000400
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLK_MSB    10
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLK_LSB    10
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLK_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_2_TESTEN
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_2_TESTEN_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_2_TESTEN_BITS   0x00000200
#define RPI_MIPICFG_DPHY_CTRL_2_TESTEN_MSB    9
#define RPI_MIPICFG_DPHY_CTRL_2_TESTEN_LSB    9
#define RPI_MIPICFG_DPHY_CTRL_2_TESTEN_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_2_TESTCLR
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLR_RESET  0x0
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLR_BITS   0x00000100
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLR_MSB    8
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLR_LSB    8
#define RPI_MIPICFG_DPHY_CTRL_2_TESTCLR_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_2_TESTDIN
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_2_TESTDIN_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_2_TESTDIN_BITS   0x000000ff
#define RPI_MIPICFG_DPHY_CTRL_2_TESTDIN_MSB    7
#define RPI_MIPICFG_DPHY_CTRL_2_TESTDIN_LSB    0
#define RPI_MIPICFG_DPHY_CTRL_2_TESTDIN_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_CTRL_3
// JTAG access : asynchronous
// Description : DPHY control for analog DFT
#define RPI_MIPICFG_DPHY_CTRL_3_OFFSET 0x00000020
#define RPI_MIPICFG_DPHY_CTRL_3_BITS   0xffffffff
#define RPI_MIPICFG_DPHY_CTRL_3_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3_BITS   0xff000000
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3_MSB    31
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3_LSB    24
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2_BITS   0x00ff0000
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2_MSB    23
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2_LSB    16
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1_BITS   0x0000ff00
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1_MSB    15
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1_LSB    8
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0_BITS   0x000000ff
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0_MSB    7
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0_LSB    0
#define RPI_MIPICFG_DPHY_CTRL_3_TXDATAESC_0_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_DPHY_CTRL_4
// JTAG access : asynchronous
// Description : DPHY control for analog DFT
#define RPI_MIPICFG_DPHY_CTRL_4_OFFSET 0x00000024
#define RPI_MIPICFG_DPHY_CTRL_4_BITS   0xffffffff
#define RPI_MIPICFG_DPHY_CTRL_4_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3_BITS   0xff000000
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3_MSB    31
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3_LSB    24
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_3_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2_BITS   0x00ff0000
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2_MSB    23
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2_LSB    16
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_2_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1_BITS   0x0000ff00
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1_MSB    15
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1_LSB    8
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_1_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0
// Description : None
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0_RESET  0x00
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0_BITS   0x000000ff
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0_MSB    7
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0_LSB    0
#define RPI_MIPICFG_DPHY_CTRL_4_TXDATAHS_0_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_INTR
// JTAG access : synchronous
// Description : Raw Interrupts
#define RPI_MIPICFG_INTR_OFFSET 0x00000028
#define RPI_MIPICFG_INTR_BITS   0x0000000f
#define RPI_MIPICFG_INTR_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTR_DSI_HOST
// Description : None
#define RPI_MIPICFG_INTR_DSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTR_DSI_HOST_BITS   0x00000008
#define RPI_MIPICFG_INTR_DSI_HOST_MSB    3
#define RPI_MIPICFG_INTR_DSI_HOST_LSB    3
#define RPI_MIPICFG_INTR_DSI_HOST_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTR_CSI_HOST
// Description : None
#define RPI_MIPICFG_INTR_CSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTR_CSI_HOST_BITS   0x00000004
#define RPI_MIPICFG_INTR_CSI_HOST_MSB    2
#define RPI_MIPICFG_INTR_CSI_HOST_LSB    2
#define RPI_MIPICFG_INTR_CSI_HOST_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTR_DSI_DMA
// Description : None
#define RPI_MIPICFG_INTR_DSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTR_DSI_DMA_BITS   0x00000002
#define RPI_MIPICFG_INTR_DSI_DMA_MSB    1
#define RPI_MIPICFG_INTR_DSI_DMA_LSB    1
#define RPI_MIPICFG_INTR_DSI_DMA_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTR_CSI_DMA
// Description : None
#define RPI_MIPICFG_INTR_CSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTR_CSI_DMA_BITS   0x00000001
#define RPI_MIPICFG_INTR_CSI_DMA_MSB    0
#define RPI_MIPICFG_INTR_CSI_DMA_LSB    0
#define RPI_MIPICFG_INTR_CSI_DMA_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_INTE
// JTAG access : synchronous
// Description : Interrupt Enable
#define RPI_MIPICFG_INTE_OFFSET 0x0000002c
#define RPI_MIPICFG_INTE_BITS   0x0000000f
#define RPI_MIPICFG_INTE_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTE_DSI_HOST
// Description : None
#define RPI_MIPICFG_INTE_DSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTE_DSI_HOST_BITS   0x00000008
#define RPI_MIPICFG_INTE_DSI_HOST_MSB    3
#define RPI_MIPICFG_INTE_DSI_HOST_LSB    3
#define RPI_MIPICFG_INTE_DSI_HOST_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTE_CSI_HOST
// Description : None
#define RPI_MIPICFG_INTE_CSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTE_CSI_HOST_BITS   0x00000004
#define RPI_MIPICFG_INTE_CSI_HOST_MSB    2
#define RPI_MIPICFG_INTE_CSI_HOST_LSB    2
#define RPI_MIPICFG_INTE_CSI_HOST_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTE_DSI_DMA
// Description : None
#define RPI_MIPICFG_INTE_DSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTE_DSI_DMA_BITS   0x00000002
#define RPI_MIPICFG_INTE_DSI_DMA_MSB    1
#define RPI_MIPICFG_INTE_DSI_DMA_LSB    1
#define RPI_MIPICFG_INTE_DSI_DMA_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTE_CSI_DMA
// Description : None
#define RPI_MIPICFG_INTE_CSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTE_CSI_DMA_BITS   0x00000001
#define RPI_MIPICFG_INTE_CSI_DMA_MSB    0
#define RPI_MIPICFG_INTE_CSI_DMA_LSB    0
#define RPI_MIPICFG_INTE_CSI_DMA_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_INTF
// JTAG access : synchronous
// Description : Interrupt Force
#define RPI_MIPICFG_INTF_OFFSET 0x00000030
#define RPI_MIPICFG_INTF_BITS   0x0000000f
#define RPI_MIPICFG_INTF_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTF_DSI_HOST
// Description : None
#define RPI_MIPICFG_INTF_DSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTF_DSI_HOST_BITS   0x00000008
#define RPI_MIPICFG_INTF_DSI_HOST_MSB    3
#define RPI_MIPICFG_INTF_DSI_HOST_LSB    3
#define RPI_MIPICFG_INTF_DSI_HOST_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTF_CSI_HOST
// Description : None
#define RPI_MIPICFG_INTF_CSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTF_CSI_HOST_BITS   0x00000004
#define RPI_MIPICFG_INTF_CSI_HOST_MSB    2
#define RPI_MIPICFG_INTF_CSI_HOST_LSB    2
#define RPI_MIPICFG_INTF_CSI_HOST_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTF_DSI_DMA
// Description : None
#define RPI_MIPICFG_INTF_DSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTF_DSI_DMA_BITS   0x00000002
#define RPI_MIPICFG_INTF_DSI_DMA_MSB    1
#define RPI_MIPICFG_INTF_DSI_DMA_LSB    1
#define RPI_MIPICFG_INTF_DSI_DMA_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTF_CSI_DMA
// Description : None
#define RPI_MIPICFG_INTF_CSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTF_CSI_DMA_BITS   0x00000001
#define RPI_MIPICFG_INTF_CSI_DMA_MSB    0
#define RPI_MIPICFG_INTF_CSI_DMA_LSB    0
#define RPI_MIPICFG_INTF_CSI_DMA_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_INTS
// JTAG access : synchronous
// Description : Interrupt status after masking & forcing
#define RPI_MIPICFG_INTS_OFFSET 0x00000034
#define RPI_MIPICFG_INTS_BITS   0x0000000f
#define RPI_MIPICFG_INTS_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTS_DSI_HOST
// Description : None
#define RPI_MIPICFG_INTS_DSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTS_DSI_HOST_BITS   0x00000008
#define RPI_MIPICFG_INTS_DSI_HOST_MSB    3
#define RPI_MIPICFG_INTS_DSI_HOST_LSB    3
#define RPI_MIPICFG_INTS_DSI_HOST_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTS_CSI_HOST
// Description : None
#define RPI_MIPICFG_INTS_CSI_HOST_RESET  0x0
#define RPI_MIPICFG_INTS_CSI_HOST_BITS   0x00000004
#define RPI_MIPICFG_INTS_CSI_HOST_MSB    2
#define RPI_MIPICFG_INTS_CSI_HOST_LSB    2
#define RPI_MIPICFG_INTS_CSI_HOST_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTS_DSI_DMA
// Description : None
#define RPI_MIPICFG_INTS_DSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTS_DSI_DMA_BITS   0x00000002
#define RPI_MIPICFG_INTS_DSI_DMA_MSB    1
#define RPI_MIPICFG_INTS_DSI_DMA_LSB    1
#define RPI_MIPICFG_INTS_DSI_DMA_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_INTS_CSI_DMA
// Description : None
#define RPI_MIPICFG_INTS_CSI_DMA_RESET  0x0
#define RPI_MIPICFG_INTS_CSI_DMA_BITS   0x00000001
#define RPI_MIPICFG_INTS_CSI_DMA_MSB    0
#define RPI_MIPICFG_INTS_CSI_DMA_LSB    0
#define RPI_MIPICFG_INTS_CSI_DMA_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_BLOCK_ID
// JTAG access : asynchronous
// Description : Block Identifier
#define RPI_MIPICFG_BLOCK_ID_OFFSET 0x00000038
#define RPI_MIPICFG_BLOCK_ID_BITS   0xffffffff
#define RPI_MIPICFG_BLOCK_ID_RESET  0x4d495049
#define RPI_MIPICFG_BLOCK_ID_MSB    31
#define RPI_MIPICFG_BLOCK_ID_LSB    0
#define RPI_MIPICFG_BLOCK_ID_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_INSTANCE_ID
// JTAG access : asynchronous
// Description : Block Instance Identifier
#define RPI_MIPICFG_INSTANCE_ID_OFFSET 0x0000003c
#define RPI_MIPICFG_INSTANCE_ID_BITS   0x0000000f
#define RPI_MIPICFG_INSTANCE_ID_RESET  0x00000000
#define RPI_MIPICFG_INSTANCE_ID_MSB    3
#define RPI_MIPICFG_INSTANCE_ID_LSB    0
#define RPI_MIPICFG_INSTANCE_ID_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_RSTSEQ_AUTO
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_RSTSEQ_AUTO_OFFSET 0x00000040
#define RPI_MIPICFG_RSTSEQ_AUTO_BITS   0x00000007
#define RPI_MIPICFG_RSTSEQ_AUTO_RESET  0x00000007
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_AUTO_CSI
// Description : 1 = reset is controlled by the sequencer
//               0 = reset is controlled by rstseq_ctrl
#define RPI_MIPICFG_RSTSEQ_AUTO_CSI_RESET  0x1
#define RPI_MIPICFG_RSTSEQ_AUTO_CSI_BITS   0x00000004
#define RPI_MIPICFG_RSTSEQ_AUTO_CSI_MSB    2
#define RPI_MIPICFG_RSTSEQ_AUTO_CSI_LSB    2
#define RPI_MIPICFG_RSTSEQ_AUTO_CSI_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_AUTO_DPI
// Description : 1 = reset is controlled by the sequencer
//               0 = reset is controlled by rstseq_ctrl
#define RPI_MIPICFG_RSTSEQ_AUTO_DPI_RESET  0x1
#define RPI_MIPICFG_RSTSEQ_AUTO_DPI_BITS   0x00000002
#define RPI_MIPICFG_RSTSEQ_AUTO_DPI_MSB    1
#define RPI_MIPICFG_RSTSEQ_AUTO_DPI_LSB    1
#define RPI_MIPICFG_RSTSEQ_AUTO_DPI_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER
// Description : 1 = reset is controlled by the sequencer
//               0 = reset is controlled by rstseq_ctrl
#define RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER_RESET  0x1
#define RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER_BITS   0x00000001
#define RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER_MSB    0
#define RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER_LSB    0
#define RPI_MIPICFG_RSTSEQ_AUTO_BUSADAPTER_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_RSTSEQ_PARALLEL
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_RSTSEQ_PARALLEL_OFFSET 0x00000044
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BITS   0x00000007
#define RPI_MIPICFG_RSTSEQ_PARALLEL_RESET  0x00000006
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_PARALLEL_CSI
// Description : Is this reset parallel (i.e. not part of the sequence)
#define RPI_MIPICFG_RSTSEQ_PARALLEL_CSI_RESET  0x1
#define RPI_MIPICFG_RSTSEQ_PARALLEL_CSI_BITS   0x00000004
#define RPI_MIPICFG_RSTSEQ_PARALLEL_CSI_MSB    2
#define RPI_MIPICFG_RSTSEQ_PARALLEL_CSI_LSB    2
#define RPI_MIPICFG_RSTSEQ_PARALLEL_CSI_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_PARALLEL_DPI
// Description : Is this reset parallel (i.e. not part of the sequence)
#define RPI_MIPICFG_RSTSEQ_PARALLEL_DPI_RESET  0x1
#define RPI_MIPICFG_RSTSEQ_PARALLEL_DPI_BITS   0x00000002
#define RPI_MIPICFG_RSTSEQ_PARALLEL_DPI_MSB    1
#define RPI_MIPICFG_RSTSEQ_PARALLEL_DPI_LSB    1
#define RPI_MIPICFG_RSTSEQ_PARALLEL_DPI_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER
// Description : Is this reset parallel (i.e. not part of the sequence)
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER_BITS   0x00000001
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER_MSB    0
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER_LSB    0
#define RPI_MIPICFG_RSTSEQ_PARALLEL_BUSADAPTER_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_RSTSEQ_CTRL
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_RSTSEQ_CTRL_OFFSET 0x00000048
#define RPI_MIPICFG_RSTSEQ_CTRL_BITS   0x00000007
#define RPI_MIPICFG_RSTSEQ_CTRL_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_CTRL_CSI
// Description : 1 = keep the reset asserted
//               0 = keep the reset deasserted
//               This is ignored if rstseq_auto=1
#define RPI_MIPICFG_RSTSEQ_CTRL_CSI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_CTRL_CSI_BITS   0x00000004
#define RPI_MIPICFG_RSTSEQ_CTRL_CSI_MSB    2
#define RPI_MIPICFG_RSTSEQ_CTRL_CSI_LSB    2
#define RPI_MIPICFG_RSTSEQ_CTRL_CSI_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_CTRL_DPI
// Description : 1 = keep the reset asserted
//               0 = keep the reset deasserted
//               This is ignored if rstseq_auto=1
#define RPI_MIPICFG_RSTSEQ_CTRL_DPI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_CTRL_DPI_BITS   0x00000002
#define RPI_MIPICFG_RSTSEQ_CTRL_DPI_MSB    1
#define RPI_MIPICFG_RSTSEQ_CTRL_DPI_LSB    1
#define RPI_MIPICFG_RSTSEQ_CTRL_DPI_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER
// Description : 1 = keep the reset asserted
//               0 = keep the reset deasserted
//               This is ignored if rstseq_auto=1
#define RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER_BITS   0x00000001
#define RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER_MSB    0
#define RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER_LSB    0
#define RPI_MIPICFG_RSTSEQ_CTRL_BUSADAPTER_ACCESS "RW"
// ================================================================================
// Register    : RPI_MIPICFG_RSTSEQ_TRIG
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_RSTSEQ_TRIG_OFFSET 0x0000004c
#define RPI_MIPICFG_RSTSEQ_TRIG_BITS   0x00000007
#define RPI_MIPICFG_RSTSEQ_TRIG_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_TRIG_CSI
// Description : Pulses the reset output
#define RPI_MIPICFG_RSTSEQ_TRIG_CSI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_TRIG_CSI_BITS   0x00000004
#define RPI_MIPICFG_RSTSEQ_TRIG_CSI_MSB    2
#define RPI_MIPICFG_RSTSEQ_TRIG_CSI_LSB    2
#define RPI_MIPICFG_RSTSEQ_TRIG_CSI_ACCESS "SC"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_TRIG_DPI
// Description : Pulses the reset output
#define RPI_MIPICFG_RSTSEQ_TRIG_DPI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_TRIG_DPI_BITS   0x00000002
#define RPI_MIPICFG_RSTSEQ_TRIG_DPI_MSB    1
#define RPI_MIPICFG_RSTSEQ_TRIG_DPI_LSB    1
#define RPI_MIPICFG_RSTSEQ_TRIG_DPI_ACCESS "SC"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER
// Description : Pulses the reset output
#define RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER_BITS   0x00000001
#define RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER_MSB    0
#define RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER_LSB    0
#define RPI_MIPICFG_RSTSEQ_TRIG_BUSADAPTER_ACCESS "SC"
// ================================================================================
// Register    : RPI_MIPICFG_RSTSEQ_DONE
// JTAG access : synchronous
// Description : None
#define RPI_MIPICFG_RSTSEQ_DONE_OFFSET 0x00000050
#define RPI_MIPICFG_RSTSEQ_DONE_BITS   0x00000007
#define RPI_MIPICFG_RSTSEQ_DONE_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_DONE_CSI
// Description : Indicates the current state of the reset
#define RPI_MIPICFG_RSTSEQ_DONE_CSI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_DONE_CSI_BITS   0x00000004
#define RPI_MIPICFG_RSTSEQ_DONE_CSI_MSB    2
#define RPI_MIPICFG_RSTSEQ_DONE_CSI_LSB    2
#define RPI_MIPICFG_RSTSEQ_DONE_CSI_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_DONE_DPI
// Description : Indicates the current state of the reset
#define RPI_MIPICFG_RSTSEQ_DONE_DPI_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_DONE_DPI_BITS   0x00000002
#define RPI_MIPICFG_RSTSEQ_DONE_DPI_MSB    1
#define RPI_MIPICFG_RSTSEQ_DONE_DPI_LSB    1
#define RPI_MIPICFG_RSTSEQ_DONE_DPI_ACCESS "RO"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER
// Description : Indicates the current state of the reset
#define RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER_RESET  0x0
#define RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER_BITS   0x00000001
#define RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER_MSB    0
#define RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER_LSB    0
#define RPI_MIPICFG_RSTSEQ_DONE_BUSADAPTER_ACCESS "RO"
// ================================================================================
// Register    : RPI_MIPICFG_DFTSS
// JTAG access : asynchronous
// Description : None
#define RPI_MIPICFG_DFTSS_OFFSET 0x00000054
#define RPI_MIPICFG_DFTSS_BITS   0x0000001f
#define RPI_MIPICFG_DFTSS_RESET  0x00000000
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DFTSS_JTAG_COPY
// Description : None
#define RPI_MIPICFG_DFTSS_JTAG_COPY_RESET  0x0
#define RPI_MIPICFG_DFTSS_JTAG_COPY_BITS   0x00000010
#define RPI_MIPICFG_DFTSS_JTAG_COPY_MSB    4
#define RPI_MIPICFG_DFTSS_JTAG_COPY_LSB    4
#define RPI_MIPICFG_DFTSS_JTAG_COPY_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY
// Description : None
#define RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY_RESET  0x0
#define RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY_BITS   0x00000008
#define RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY_MSB    3
#define RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY_LSB    3
#define RPI_MIPICFG_DFTSS_JTAG_ACCESS_ONLY_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS
// Description : None
#define RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS_RESET  0x0
#define RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS_BITS   0x00000004
#define RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS_MSB    2
#define RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS_LSB    2
#define RPI_MIPICFG_DFTSS_BYPASS_OUTSYNCS_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DFTSS_BYPASS_INSYNCS
// Description : None
#define RPI_MIPICFG_DFTSS_BYPASS_INSYNCS_RESET  0x0
#define RPI_MIPICFG_DFTSS_BYPASS_INSYNCS_BITS   0x00000002
#define RPI_MIPICFG_DFTSS_BYPASS_INSYNCS_MSB    1
#define RPI_MIPICFG_DFTSS_BYPASS_INSYNCS_LSB    1
#define RPI_MIPICFG_DFTSS_BYPASS_INSYNCS_ACCESS "RW"
// --------------------------------------------------------------------------------
// Field       : RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS
// Description : None
#define RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS_RESET  0x0
#define RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS_BITS   0x00000001
#define RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS_MSB    0
#define RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS_LSB    0
#define RPI_MIPICFG_DFTSS_BYPASS_RESETSYNCS_ACCESS "RW"

#define CFG_WRITE(reg, val)  writel((val),  dsi->hw_base[RP1DSI_HW_BLOCK_CFG] + (reg ## _OFFSET))
#define CFG_READ(reg)        readl(dsi->hw_base[RP1DSI_HW_BLOCK_CFG] + (reg ## _OFFSET))

/* ------------------------------- DPHY setup stuff ------------------------ */

static void dphy_transaction(struct rp1_dsi *dsi, uint8_t test_code, uint8_t test_data)
{
	/*
	 * See pg 101 of mipi dphy bidir databook
	 * Assume we start with testclk high.
	 * Each APB write takes at least 10ns and we ignore TESTDOUT
	 * so there is no need for extra delays between the transitions.
	 */

	DSI_WRITE(DSI_PHY_TST_CTRL1, test_code | DPHY_CTRL1_PHY_TESTEN_BITS);
	DSI_WRITE(DSI_PHY_TST_CTRL0, 0);
	DSI_READ(DSI_PHY_TST_CTRL1); /* XXX possibly not needed */
	DSI_WRITE(DSI_PHY_TST_CTRL1, test_data);
	DSI_WRITE(DSI_PHY_TST_CTRL0, DPHY_CTRL0_PHY_TESTCLK_BITS);
}

static u64 dphy_get_div(u32 refclk, u64 vco_freq, u32 *ptr_m, u32 *ptr_n)
{
	/*
	 * See pg 77-78 of dphy databook
	 * fvco = m/n * refclk
	 * with the limit
	 * 40MHz >= fREFCLK / N >= 5MHz
	 * M (multiplier) must be an even number between 2 and 300
	 * N (input divider) must be an integer between 1 and 100
	 *
	 * In practice, given a 50MHz reference clock, it can produce any
	 * multiple of 10MHz, 11.1111MHz, 12.5MHz, 14.286MHz or 16.667MHz
	 * with < 1% error for all frequencies above 495MHz.
	 *
	 * vco_freq should be set to the lane bit rate (not the MIPI clock
	 * which is half of this). These frequencies are now measured in Hz.
	 * They should fit within u32, but u64 is needed for calculations.
	 */

	static const u32 REF_DIVN_MAX = 40000000;
	static const u32 REF_DIVN_MIN =  5000000;
	u32 n, best_n, best_m;
	u64 best_err = vco_freq;

	for (n = 1 + refclk / REF_DIVN_MAX; n * REF_DIVN_MIN <= refclk && n < 100; ++n) {
		u32 half_m = DIV_U64_ROUND_CLOSEST(n * vco_freq, 2 * refclk);

		if (half_m < 150) {
			u64 f = div_u64(mul_u32_u32(2 * half_m, refclk), n);
			u64 err = (f > vco_freq) ? f - vco_freq : vco_freq - f;

			if (err < best_err) {
				best_n = n;
				best_m = 2 * half_m;
				best_err = err;
				if (err == 0)
					break;
			}
		}
	}

	if (64 * best_err >= vco_freq)
		return 0;

	*ptr_n = best_n;
	*ptr_m = best_m;
	return div_u64(mul_u32_u32(best_m, refclk), best_n);
}

struct hsfreq_range {
	u16 mhz_max;
	u8  hsfreqrange;
	u8  clk_lp2hs;
	u8  clk_hs2lp;
	u8  data_lp2hs; /* excluding clk lane entry */
	u8  data_hs2lp;
};

/* See Table A-3 on page 258 of dphy databook */
static const struct hsfreq_range hsfreq_table[] = {
	{   89, 0b000000, 32, 20, 26, 13 },
	{   99, 0b010000, 35, 23, 28, 14 },
	{  109, 0b100000, 32, 22, 26, 13 },
	{  129, 0b000001, 31, 20, 27, 13 },
	{  139, 0b010001, 33, 22, 26, 14 },
	{  149, 0b100001, 33, 21, 26, 14 },
	{  169, 0b000010, 32, 20, 27, 13 },
	{  179, 0b010010, 36, 23, 30, 15 },
	{  199, 0b100010, 40, 22, 33, 15 },
	{  219, 0b000011, 40, 22, 33, 15 },
	{  239, 0b010011, 44, 24, 36, 16 },
	{  249, 0b100011, 48, 24, 38, 17 },
	{  269, 0b000100, 48, 24, 38, 17 },
	{  299, 0b010100, 50, 27, 41, 18 },
	{  329, 0b000101, 56, 28, 45, 18 },
	{  359, 0b010101, 59, 28, 48, 19 },
	{  399, 0b100101, 61, 30, 50, 20 },
	{  449, 0b000110, 67, 31, 55, 21 },
	{  499, 0b010110, 73, 31, 59, 22 },
	{  549, 0b000111, 79, 36, 63, 24 },
	{  599, 0b010111, 83, 37, 68, 25 },
	{  649, 0b001000, 90, 38, 73, 27 },
	{  699, 0b011000, 95, 40, 77, 28 },
	{  749, 0b001001, 102, 40, 84, 28 },
	{  799, 0b011001, 106, 42, 87, 30 },
	{  849, 0b101001, 113, 44, 93, 31 },
	{  899, 0b111001, 118, 47, 98, 32 },
	{  949, 0b001010, 124, 47, 102, 34 },
	{  999, 0b011010, 130, 49, 107, 35 },
	{ 1049, 0b101010, 135, 51, 111, 37 },
	{ 1099, 0b111010, 139, 51, 114, 38 },
	{ 1149, 0b001011, 146, 54, 120, 40 },
	{ 1199, 0b011011, 153, 57, 125, 41 },
	{ 1249, 0b101011, 158, 58, 130, 42 },
	{ 1299, 0b111011, 163, 58, 135, 44 },
	{ 1349, 0b001100, 168, 60, 140, 45 },
	{ 1399, 0b011100, 172, 64, 144, 47 },
	{ 1449, 0b101100, 176, 65, 148, 48 },
	{ 1500, 0b111100, 181, 66, 153, 50 },
};

static void dphy_set_hsfreqrange(struct rp1_dsi *dsi, u32 freq_mhz)
{
	unsigned int i;

	if (freq_mhz < 80 || freq_mhz > 1500)
		drm_err(dsi->drm, "DPHY: Frequency %u MHz out of range\n",
			freq_mhz);

	for (i = 0; i < ARRAY_SIZE(hsfreq_table) - 1; i++) {
		if (freq_mhz <= hsfreq_table[i].mhz_max)
			break;
	}

	dsi->hsfreq_index = i;
	dphy_transaction(dsi, DPHY_HS_RX_CTRL_LANE0_OFFSET,
			 hsfreq_table[i].hsfreqrange << 1);
}

static u32 dphy_configure_pll(struct rp1_dsi *dsi, u32 refclk, u32 vco_freq)
{
	u32 m = 0;
	u32 n = 0;
	u32 actual_vco_freq = dphy_get_div(refclk, vco_freq, &m, &n);

	if (actual_vco_freq) {
		dphy_set_hsfreqrange(dsi, actual_vco_freq / 1000000);
		/* Program m,n from registers */
		dphy_transaction(dsi, DPHY_PLL_DIV_CTRL_OFFSET, 0x30);
		/* N (program N-1) */
		dphy_transaction(dsi, DPHY_PLL_INPUT_DIV_OFFSET, n - 1);
		/* M[8:5] ?? */
		dphy_transaction(dsi, DPHY_PLL_LOOP_DIV_OFFSET, 0x80 | ((m - 1) >> 5));
		/* M[4:0] (program M-1) */
		dphy_transaction(dsi, DPHY_PLL_LOOP_DIV_OFFSET, ((m - 1) & 0x1F));
		drm_dbg_driver(dsi->drm,
			       "DPHY: vco freq want %uHz got %uHz = %d * (%uHz / %d), hsfreqrange = 0x%02x\n",
			       vco_freq, actual_vco_freq, m, refclk, n,
			       hsfreq_table[dsi->hsfreq_index].hsfreqrange);
	} else {
		drm_err(dsi->drm,
			"rp1dsi: Error configuring DPHY PLL %uHz\n", vco_freq);
	}

	return actual_vco_freq;
}

static u32 dphy_init(struct rp1_dsi *dsi, u32 ref_freq, u32 vco_freq)
{
	u32 actual_vco_freq;

	/* Reset the PHY */
	DSI_WRITE(DSI_PHYRSTZ, 0);
	DSI_WRITE(DSI_PHY_TST_CTRL0, DPHY_CTRL0_PHY_TESTCLK_BITS);
	DSI_WRITE(DSI_PHY_TST_CTRL1, 0);
	DSI_WRITE(DSI_PHY_TST_CTRL0, (DPHY_CTRL0_PHY_TESTCLK_BITS | DPHY_CTRL0_PHY_TESTCLR_BITS));
	udelay(1);
	DSI_WRITE(DSI_PHY_TST_CTRL0, DPHY_CTRL0_PHY_TESTCLK_BITS);
	udelay(1);
	/* Since we are in DSI (not CSI2) mode here, start the PLL */
	actual_vco_freq = dphy_configure_pll(dsi, ref_freq, vco_freq);
	udelay(1);
	/* Unreset */
	DSI_WRITE(DSI_PHYRSTZ, DSI_PHYRSTZ_SHUTDOWNZ_BITS);
	udelay(1);
	DSI_WRITE(DSI_PHYRSTZ, (DSI_PHYRSTZ_SHUTDOWNZ_BITS | DSI_PHYRSTZ_RSTZ_BITS));
	udelay(1); /* so we can see PLL coming up? */

	return actual_vco_freq;
}

void rp1dsi_mipicfg_setup(struct rp1_dsi *dsi)
{
	/* Select DSI rather than CSI-2 */
	CFG_WRITE(RPI_MIPICFG_CFG, 0);
	/* Enable DSIDMA interrupt only */
	CFG_WRITE(RPI_MIPICFG_INTE, RPI_MIPICFG_INTE_DSI_DMA_BITS);
}

static unsigned long rp1dsi_refclk_freq(struct rp1_dsi *dsi)
{
	unsigned long u;

	u = (dsi->clocks[RP1DSI_CLOCK_REF]) ? clk_get_rate(dsi->clocks[RP1DSI_CLOCK_REF]) : 0;
	if (u < 1 || u >= (1ul << 30))
		u = 50000000ul; /* default XOSC frequency */
	return u;
}

static void rp1dsi_dpiclk_start(struct rp1_dsi *dsi, u32 byte_clock,
				unsigned int bpp, unsigned int lanes)
{
	/* Dummy clk_set_rate() to declare the actual DSI byte-clock rate */
	clk_set_rate(dsi->clocks[RP1DSI_CLOCK_BYTE], byte_clock);

	/*
	 * Prefer the DSI byte-clock source where possible, so that DSI and DPI
	 * clocks will be in an exact ratio and downstream devices can recover
	 * perfect timings. But when DPI clock is faster, fall back on PLL_SYS.
	 * To defeat rounding errors, specify explicitly which source to use.
	 */
	if (bpp >= 8 * lanes)
		clk_set_parent(dsi->clocks[RP1DSI_CLOCK_DPI], dsi->clocks[RP1DSI_CLOCK_BYTE]);
	else if (dsi->clocks[RP1DSI_CLOCK_PLLSYS])
		clk_set_parent(dsi->clocks[RP1DSI_CLOCK_DPI], dsi->clocks[RP1DSI_CLOCK_PLLSYS]);

	clk_set_rate(dsi->clocks[RP1DSI_CLOCK_DPI], (4 * lanes * byte_clock) / (bpp >> 1));
	clk_prepare_enable(dsi->clocks[RP1DSI_CLOCK_DPI]);
	drm_info(dsi->drm,
		 "rp1dsi: Nominal Byte clock %u DPI clock %lu (parent rate %lu)\n",
		 byte_clock,
		 clk_get_rate(dsi->clocks[RP1DSI_CLOCK_DPI]),
		 clk_get_rate(clk_get_parent(dsi->clocks[RP1DSI_CLOCK_DPI])));
}

static void rp1dsi_dpiclk_stop(struct rp1_dsi *dsi)
{
	if (dsi->clocks[RP1DSI_CLOCK_DPI])
		clk_disable_unprepare(dsi->clocks[RP1DSI_CLOCK_DPI]);
}

/* Choose the internal on-the-bus DPI format, and DSI packing flag. */
static u32 get_colorcode(enum mipi_dsi_pixel_format fmt)
{
	switch (fmt) {
	case MIPI_DSI_FMT_RGB666:
		return 0x104;
	case MIPI_DSI_FMT_RGB666_PACKED:
		return 0x003;
	case MIPI_DSI_FMT_RGB565:
		return 0x000;
	case MIPI_DSI_FMT_RGB888:
		return 0x005;
	}

	/* This should be impossible as the format is validated in
	 * rp1dsi_host_attach
	 */
	WARN_ONCE(1, "Invalid colour format configured for DSI");
	return 0x005;
}

/* Frequency limits for DPI, HS and LP clocks, and some magic numbers */
#define RP1DSI_DPI_MAX_KHZ     200000
#define RP1DSI_BYTE_CLK_MIN  10000000
#define RP1DSI_BYTE_CLK_MAX 187500000
#define RP1DSI_ESC_CLK_MAX   20000000
#define RP1DSI_TO_CLK_DIV        0x50
#define RP1DSI_LPRX_TO_VAL       0x40
#define RP1DSI_BTA_TO_VAL       0xd00

void rp1dsi_dsi_setup(struct rp1_dsi *dsi, struct drm_display_mode const *mode)
{
	int cmdtim;
	u32 timeout, mask, clkdiv;
	unsigned int bpp = mipi_dsi_pixel_format_to_bpp(dsi->display_format);
	u32 byte_clock = clamp((bpp * 125 * min(mode->clock, RP1DSI_DPI_MAX_KHZ)) / dsi->lanes,
			       RP1DSI_BYTE_CLK_MIN, RP1DSI_BYTE_CLK_MAX);

	DSI_WRITE(DSI_PHY_IF_CFG, dsi->lanes - 1);
	DSI_WRITE(DSI_DPI_CFG_POL, 0);
	DSI_WRITE(DSI_GEN_VCID, dsi->vc);
	DSI_WRITE(DSI_DPI_COLOR_CODING, get_colorcode(dsi->display_format));

	/*
	 * Flags to configure use of LP, EoTp, Burst Mode, Sync Events/Pulses.
	 * Note that Burst Mode implies Sync Events; the two flags need not be
	 * set concurrently, and in this RP1 variant *should not* both be set:
	 * doing so would (counter-intuitively) enable Sync Pulses and may fail
	 * if there is not sufficient time to return to LP11 state during HBP.
	 */
	mask =  DSI_VID_MODE_LP_HFP_EN  | DSI_VID_MODE_LP_HBP_EN |
		DSI_VID_MODE_LP_VACT_EN | DSI_VID_MODE_LP_VFP_EN |
		DSI_VID_MODE_LP_VBP_EN  | DSI_VID_MODE_LP_VSA_EN;
	if (dsi->display_flags & MIPI_DSI_MODE_LPM)
		mask |= DSI_VID_MODE_LP_CMD_EN;
	if (dsi->display_flags & MIPI_DSI_MODE_VIDEO_BURST)
		mask |= DSI_VID_MODE_BURST;
	else if (!(dsi->display_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE))
		mask |= DSI_VID_MODE_SYNC_EVENTS;
	else if (8 * dsi->lanes > bpp)
		mask &= ~DSI_VID_MODE_LP_HBP_EN; /* PULSE && inexact DPICLK => fix HBP time */
	DSI_WRITE(DSI_VID_MODE_CFG, mask);
	DSI_WRITE(DSI_CMD_MODE_CFG,
		  (dsi->display_flags & MIPI_DSI_MODE_LPM) ? DSI_CMD_MODE_ALL_LP : 0);
	DSI_WRITE(DSI_PCKHDL_CFG,
		  DSI_PCKHDL_BTA_EN |
		  ((dsi->display_flags & MIPI_DSI_MODE_NO_EOT_PACKET) ? 0 : DSI_PCKHDL_EOTP_TX_EN));

	/* Select Command Mode */
	DSI_WRITE(DSI_MODE_CFG, 1);

	/* Set timeouts and clock dividers */
	timeout = (bpp * mode->htotal * mode->vdisplay) / (7 * RP1DSI_TO_CLK_DIV * dsi->lanes);
	if (timeout > 0xFFFFu)
		timeout = 0;
	DSI_WRITE(DSI_TO_CNT_CFG, (timeout << 16) | RP1DSI_LPRX_TO_VAL);
	DSI_WRITE(DSI_BTA_TO_CNT, RP1DSI_BTA_TO_VAL);
	clkdiv = max(2u, 1u + byte_clock / RP1DSI_ESC_CLK_MAX); /* byte clocks per escape clock */
	DSI_WRITE(DSI_CLKMGR_CFG,
		  (RP1DSI_TO_CLK_DIV << 8) | clkdiv);

	/* Configure video timings */
	DSI_WRITE(DSI_VID_PKT_SIZE, mode->hdisplay);
	DSI_WRITE(DSI_VID_NUM_CHUNKS, 0);
	DSI_WRITE(DSI_VID_NULL_SIZE, 0);
	DSI_WRITE(DSI_VID_HSA_TIME,
		  (bpp * (mode->hsync_end - mode->hsync_start)) / (8 * dsi->lanes));
	DSI_WRITE(DSI_VID_HBP_TIME,
		  (bpp * (mode->htotal - mode->hsync_end)) / (8 * dsi->lanes));
	DSI_WRITE(DSI_VID_HLINE_TIME, (bpp * mode->htotal) / (8 * dsi->lanes));
	DSI_WRITE(DSI_VID_VSA_LINES, (mode->vsync_end - mode->vsync_start));
	DSI_WRITE(DSI_VID_VBP_LINES, (mode->vtotal - mode->vsync_end));
	DSI_WRITE(DSI_VID_VFP_LINES, (mode->vsync_start - mode->vdisplay));
	DSI_WRITE(DSI_VID_VACTIVE_LINES, mode->vdisplay);

	/* Init PHY */
	byte_clock = dphy_init(dsi, rp1dsi_refclk_freq(dsi), 8 * byte_clock) >> 3;

	DSI_WRITE(DSI_PHY_TMR_LPCLK_CFG,
		  (hsfreq_table[dsi->hsfreq_index].clk_lp2hs << DSI_PHY_TMR_LP2HS_LSB) |
		  (hsfreq_table[dsi->hsfreq_index].clk_hs2lp << DSI_PHY_TMR_HS2LP_LSB));
	DSI_WRITE(DSI_PHY_TMR_CFG,
		  (hsfreq_table[dsi->hsfreq_index].data_lp2hs << DSI_PHY_TMR_LP2HS_LSB) |
		  (hsfreq_table[dsi->hsfreq_index].data_hs2lp << DSI_PHY_TMR_HS2LP_LSB));

	/* Estimate how many LP bytes can be sent during vertical blanking (Databook 3.6.2.1) */
	cmdtim = mode->htotal;
	if (dsi->display_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		cmdtim -= mode->hsync_end - mode->hsync_start;
	cmdtim = (bpp * cmdtim - 64) / (8 * dsi->lanes);      /* byte clocks after HSS and EoTp */
	cmdtim -= hsfreq_table[dsi->hsfreq_index].data_hs2lp;
	cmdtim -= hsfreq_table[dsi->hsfreq_index].data_lp2hs;
	cmdtim = (cmdtim / clkdiv) - 24;                      /* escape clocks for commands */
	cmdtim = max(0, cmdtim >> 4);                         /* bytes (at 2 clocks per bit) */
	drm_info(dsi->drm, "rp1dsi: Command time (outvact): %d\n", cmdtim);
	DSI_WRITE(DSI_DPI_LP_CMD_TIM, cmdtim << 16);

	/* Wait for PLL lock */
	for (timeout = (1 << 14); timeout != 0; --timeout) {
		usleep_range(10, 50);
		if (DSI_READ(DSI_PHY_STATUS) & (1 << 0))
			break;
	}
	if (timeout == 0)
		drm_err(dsi->drm, "RP1DSI: Time out waiting for PLL\n");

	DSI_WRITE(DSI_LPCLK_CTRL,
		  (dsi->display_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS) ? 0x3 : 0x1);
	DSI_WRITE(DSI_PHY_TST_CTRL0, 0x2);
	DSI_WRITE(DSI_PWR_UP, 0x1);		/* power up */

	/* Now it should be safe to start the external DPI clock divider */
	rp1dsi_dpiclk_start(dsi, byte_clock, bpp, dsi->lanes);

	/* Wait for all lane(s) to be in Stopstate */
	mask = (1 << 4);
	if (dsi->lanes >= 2)
		mask |= (1 << 7);
	if (dsi->lanes >= 3)
		mask |= (1 << 9);
	if (dsi->lanes >= 4)
		mask |= (1 << 11);
	for (timeout = (1 << 10); timeout != 0; --timeout) {
		usleep_range(10, 50);
		if ((DSI_READ(DSI_PHY_STATUS) & mask) == mask)
			break;
	}
	if (timeout == 0)
		drm_err(dsi->drm, "RP1DSI: Time out waiting for lanes (%x %x)\n",
			mask, DSI_READ(DSI_PHY_STATUS));
}

void rp1dsi_dsi_send(struct rp1_dsi *dsi, u32 hdr, int len, const u8 *buf,
		     bool use_lpm, bool req_ack)
{
	u32 val;

	/* Wait for both FIFOs empty */
	for (val = 256; val > 0; --val) {
		if ((DSI_READ(DSI_CMD_PKT_STATUS) & 0xF) == 0x5)
			break;
		usleep_range(100, 150);
	}

	/*
	 * Update global configuration flags for LP/HS and ACK options.
	 * XXX It's not clear if having empty FIFOs (checked above and below) guarantees that
	 * the last command has completed and been ACKed, or how closely these control registers
	 * align with command/payload FIFO writes (as each is an independent clock-crossing)?
	 */
	val = DSI_READ(DSI_VID_MODE_CFG);
	if (use_lpm)
		val |= DSI_VID_MODE_LP_CMD_EN;
	else
		val &= ~DSI_VID_MODE_LP_CMD_EN;
	DSI_WRITE(DSI_VID_MODE_CFG, val);
	val = (use_lpm) ? DSI_CMD_MODE_ALL_LP : 0;
	if (req_ack)
		val |= DSI_CMD_MODE_ACK_RQST_EN;
	DSI_WRITE(DSI_CMD_MODE_CFG, val);
	(void)DSI_READ(DSI_CMD_MODE_CFG);

	/* Write payload (in 32-bit words) and header */
	for (; len > 0; len -= 4) {
		val = *buf++;
		if (len > 1)
			val |= (*buf++) << 8;
		if (len > 2)
			val |= (*buf++) << 16;
		if (len > 3)
			val |= (*buf++) << 24;
		DSI_WRITE(DSI_GEN_PLD_DATA, val);
	}
	DSI_WRITE(DSI_GEN_HDR, hdr);

	/* Wait for both FIFOs empty */
	for (val = 256; val > 0; --val) {
		if ((DSI_READ(DSI_CMD_PKT_STATUS) & 0xF) == 0x5)
			break;
		usleep_range(100, 150);
	}
}

int rp1dsi_dsi_recv(struct rp1_dsi *dsi, int len, u8 *buf)
{
	int i, j;
	u32 val;

	/* Wait until not busy and FIFO not empty */
	for (i = 1024; i > 0; --i) {
		val = DSI_READ(DSI_CMD_PKT_STATUS);
		if ((val & ((1 << 6) | (1 << 4))) == 0)
			break;
		usleep_range(100, 150);
	}
	if (!i) {
		drm_warn(dsi->drm, "Receive failed\n");
		return -EIO;
	}

	for (i = 0; i < len; i += 4) {
		/* Read fifo must not be empty before all bytes are read */
		if (DSI_READ(DSI_CMD_PKT_STATUS) & (1 << 4))
			break;

		val = DSI_READ(DSI_GEN_PLD_DATA);
		for (j = 0; j < 4 && j + i < len; j++)
			*buf++ = val >> (8 * j);
	}

	return (i >= len) ? len : (i > 0) ? i : -EIO;
}

void rp1dsi_dsi_stop(struct rp1_dsi *dsi)
{
	DSI_WRITE(DSI_MODE_CFG, 1);	/* Return to Command Mode */
	DSI_WRITE(DSI_LPCLK_CTRL, 2);	/* Stop the HS clock */
	DSI_WRITE(DSI_PWR_UP, 0x0);     /* Power down host controller */
	DSI_WRITE(DSI_PHYRSTZ, 0);      /* PHY into reset. */
	rp1dsi_dpiclk_stop(dsi);
}

void rp1dsi_dsi_set_cmdmode(struct rp1_dsi *dsi, int mode)
{
	DSI_WRITE(DSI_MODE_CFG, mode);
}
