// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_PHY_H__
#define __ADRV906X_PHY_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/bitfield.h>

#define ADRV906X_PHY_ID                         0x00000000
#define ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN        BIT(0)
#define ADRV906X_PHY_FLAGS_LOOPBACK_TEST        BIT(1)

/* ADI PCS registers */

/* The following registers and bitfields are defined in 802.3 clause 45.
 * We should replace them with those from mdio.h in future.
 */
#define ADRV906X_PCS_STATUS_3_REG                        9
#define ADRV906X_PCS_BRMGBT_STAT2                        33
#define   ADRV906X_PCS_BRMGBT_STAT2_LBLKLK               BIT(15)
#define ADRV906X_PCS_SEED_A0_REG                         34
#define ADRV906X_PCS_SEED_A1_REG                         35
#define ADRV906X_PCS_SEED_A2_REG                         36
#define ADRV906X_PCS_SEED_A3_REG                         37
#define ADRV906X_PCS_SEED_B0_REG                         38
#define ADRV906X_PCS_SEED_B1_REG                         39
#define ADRV906X_PCS_SEED_B2_REG                         40
#define ADRV906X_PCS_SEED_B3_REG                         41
#define ADRV906X_PCS_TEST_CTRL_REG                       42
#define ADRV906X_PCS_TEST_ERROR_CNT_REG                  43
#define ADRV906X_PCS_BER_HIGH_REG                        44
#define ADRV906X_PCS_ERROR_BLOCKS_REG                    45

/* Configuration values of PCS specific registers */
#define ADRV906X_PCS_CTRL2_TYPE_SEL_MSK                  GENMASK(3, 0)  /* PCS type selection */
#define ADRV906X_PCS_CTRL2_10GBR                         0x0000
#define ADRV906X_PCS_CTRL2_25GBR                         0x0007         /* 25GBASE-R type */
#define ADRV906X_PCS_CTRL1_SPEED10G (MDIO_CTRL1_SPEEDSELEXT | 0x00)     /* 10 Gb/s */
#define ADRV906X_PCS_CTRL1_SPEED25G (MDIO_CTRL1_SPEEDSELEXT | 0x14)     /* 25 Gb/s */

#define ADRV906X_PCS_STAT2_25GBR                         0x0080         /* 25GBASE-R */
#define ADRV906X_PCS_STAT2_10GBR                         0x0001         /* 10GBASE-R */

/* 802.3 clause 45 states PCS vendor registers to be numbered from 32768.
 * But ADRV906x PCS vendor registers are numbered from 46.
 */
#define ADRV906X_PCS_GENERAL_TX_REG                      46
#define ADRV906X_PCS_GENERAL_RX_REG                      47
#define   ADRV906X_PCS_GENERAL_SCRAMBLER_BYPASS_EN       BIT(15)
#define   ADRV906X_PCS_GENERAL_CPRI_EN                   BIT(14)
#define   ADRV906X_PCS_GENERAL_SERDES_BUS_WIDTH_MSK      GENMASK(13, 7)
#define   ADRV906X_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH  0x2000
#define   ADRV906X_PCS_GENERAL_SERDES_32_BITS_BUS_WIDTH  0x1000
#define   ADRV906X_PCS_GENERAL_HALF_DUTY_CYCLE_EN        BIT(4)
#define   ADRV906X_PCS_GENERAL_XGMII_BUS_WIDTH_MSK       BIT(3)
#define   ADRV906X_PCS_GENERAL_64_BITS_XGMII             0x0008
#define   ADRV906X_PCS_GENERAL_32_BITS_XGMII             0x0000
#define   ADRV906X_PCS_GENERAL_PRBS23_TESTPATTERN_EN     BIT(2)
#define   ADRV906X_PCS_GENERAL_PRBS7_TESTPATTERN_EN      BIT(1)
#define   ADRV906X_PCS_GENERAL_PATH_RESET                BIT(0)

#define ADRV906X_PCS_CFG_TX_REG                          48
#define   ADRV906X_PCS_CFG_TX_BIT_DELAY_MSK              GENMASK(15, 9)
#define   ADRV906X_PCS_CFG_TX_BUF_INIT_MSK               GENMASK(8, 1)
#define   ADRV906X_PCS_CFG_TX_BUF_INIT                   0x000A
#define   ADRV906X_PCS_CFG_TX_BUF_BYPASS_EN              BIT(0)

#define ADRV906X_PCS_CFG_RX_REG                          49
#define   ADRV906X_PCS_CFG_RX_GEARBOX_BYPASS_EN          BIT(12)
#define   ADRV906X_PCS_CFG_RX_SERDES_LOOPBACK_EN         BIT(11)
#define   ADRV906X_PCS_CFG_RX_CORE_IF_LOOPBACK_EN        BIT(10)
#define   ADRV906X_PCS_CFG_RX_COMMA_SEARCH_DIS           BIT(9)
#define   ADRV906X_PCS_CFG_RX_BUF_INIT_MSK               GENMASK(8, 1)
#define   ADRV906X_PCS_CFG_RX_BUF_INIT                   0x000A
#define   ADRV906X_PCS_CFG_RX_BUF_BYPASS_EN              BIT(0)

#define ADRV906X_PCS_BUF_STAT_TX_REG                     50
#define   ADRV906X_PCS_BUF_STAT_TX_FINE_DELAY            GENMASK(7, 0)
#define ADRV906X_PCS_BUF_STAT_RX_REG                     51
#define   ADRV906X_PCS_BUF_STAT_RX_FINE_DELAY            GENMASK(7, 0)
#define ADRV906X_PCS_DELAY_RX_REG                        52
#define   ADRV906X_PCS_DELAY_RX_BIT_SLIP                 GENMASK(14, 8)
#define ADRV906X_PCS_DISP_ERR_REG                        53
#define ADRV906X_PCS_CODE_ERR_REG                        54
#define ADRV906X_PCS_CPCS_SHCV_REG                       55

#define ADRV906X_PCS_RS_FEC_CTRL_REG                     200
#define   ADRV906X_PCS_RS_FEC_CTRL_EN                    BIT(2)
#define ADRV906X_PCS_RS_FEC_STAT_REG                     201
#define   ADRV906X_PCS_RS_FEC_STAT_ALIGN                 BIT(14)

int adrv906x_phy_register(void);
void adrv906x_phy_unregister(void);

#endif /* __ADRV906X_PHY_H__ */
