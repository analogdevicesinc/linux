/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*!
 * Header file used to configure SoC pin list.
 */

#ifndef _SC_PINS_H
#define _SC_PINS_H

/* Includes */

/* Defines */

#define SC_P_ALL            UINT16_MAX      /* !< All pins */

/*!
 * @name Pin Definitions
 */
/*@{*/
#define SC_P_SIM0_CLK                            0
#define SC_P_SIM0_RST                            1
#define SC_P_SIM0_IO                             2
#define SC_P_SIM0_PD                             3
#define SC_P_SIM0_POWER_EN                       4
#define SC_P_SIM0_GPIO0_00                       5
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SIM           6
#define SC_P_M40_I2C0_SCL                        7
#define SC_P_M40_I2C0_SDA                        8
#define SC_P_M40_GPIO0_00                        9
#define SC_P_M40_GPIO0_01                        10
#define SC_P_M41_I2C0_SCL                        11
#define SC_P_M41_I2C0_SDA                        12
#define SC_P_M41_GPIO0_00                        13
#define SC_P_M41_GPIO0_01                        14
#define SC_P_GPT0_CLK                            15
#define SC_P_GPT0_CAPTURE                        16
#define SC_P_GPT0_COMPARE                        17
#define SC_P_GPT1_CLK                            18
#define SC_P_GPT1_CAPTURE                        19
#define SC_P_GPT1_COMPARE                        20
#define SC_P_UART0_RX                            21
#define SC_P_UART0_TX                            22
#define SC_P_UART0_RTS_B                         23
#define SC_P_UART0_CTS_B                         24
#define SC_P_UART1_TX                            25
#define SC_P_UART1_RX                            26
#define SC_P_UART1_RTS_B                         27
#define SC_P_UART1_CTS_B                         28
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLH        29
#define SC_P_SCU_PMIC_MEMC_ON                    30
#define SC_P_SCU_WDOG_OUT                        31
#define SC_P_PMIC_I2C_SDA                        32
#define SC_P_PMIC_I2C_SCL                        33
#define SC_P_PMIC_EARLY_WARNING                  34
#define SC_P_PMIC_INT_B                          35
#define SC_P_SCU_GPIO0_00                        36
#define SC_P_SCU_GPIO0_01                        37
#define SC_P_SCU_GPIO0_02                        38
#define SC_P_SCU_GPIO0_03                        39
#define SC_P_SCU_GPIO0_04                        40
#define SC_P_SCU_GPIO0_05                        41
#define SC_P_SCU_GPIO0_06                        42
#define SC_P_SCU_GPIO0_07                        43
#define SC_P_SCU_BOOT_MODE0                      44
#define SC_P_SCU_BOOT_MODE1                      45
#define SC_P_SCU_BOOT_MODE2                      46
#define SC_P_SCU_BOOT_MODE3                      47
#define SC_P_SCU_BOOT_MODE4                      48
#define SC_P_SCU_BOOT_MODE5                      49
#define SC_P_LVDS0_GPIO00                        50
#define SC_P_LVDS0_GPIO01                        51
#define SC_P_LVDS0_I2C0_SCL                      52
#define SC_P_LVDS0_I2C0_SDA                      53
#define SC_P_LVDS0_I2C1_SCL                      54
#define SC_P_LVDS0_I2C1_SDA                      55
#define SC_P_LVDS1_GPIO00                        56
#define SC_P_LVDS1_GPIO01                        57
#define SC_P_LVDS1_I2C0_SCL                      58
#define SC_P_LVDS1_I2C0_SDA                      59
#define SC_P_LVDS1_I2C1_SCL                      60
#define SC_P_LVDS1_I2C1_SDA                      61
#define SC_P_COMP_CTL_GPIO_1V8_3V3_LVDSGPIO      62
#define SC_P_MIPI_DSI0_I2C0_SCL                  63
#define SC_P_MIPI_DSI0_I2C0_SDA                  64
#define SC_P_MIPI_DSI0_GPIO0_00                  65
#define SC_P_MIPI_DSI0_GPIO0_01                  66
#define SC_P_MIPI_DSI1_I2C0_SCL                  67
#define SC_P_MIPI_DSI1_I2C0_SDA                  68
#define SC_P_MIPI_DSI1_GPIO0_00                  69
#define SC_P_MIPI_DSI1_GPIO0_01                  70
#define SC_P_COMP_CTL_GPIO_1V8_3V3_MIPIDSIGPIO   71
#define SC_P_MIPI_CSI0_MCLK_OUT                  72
#define SC_P_MIPI_CSI0_I2C0_SCL                  73
#define SC_P_MIPI_CSI0_I2C0_SDA                  74
#define SC_P_MIPI_CSI0_GPIO0_00                  75
#define SC_P_MIPI_CSI0_GPIO0_01                  76
#define SC_P_MIPI_CSI1_MCLK_OUT                  77
#define SC_P_MIPI_CSI1_GPIO0_00                  78
#define SC_P_MIPI_CSI1_GPIO0_01                  79
#define SC_P_MIPI_CSI1_I2C0_SCL                  80
#define SC_P_MIPI_CSI1_I2C0_SDA                  81
#define SC_P_HDMI_TX0_TS_SCL                     82
#define SC_P_HDMI_TX0_TS_SDA                     83
#define SC_P_COMP_CTL_GPIO_3V3_HDMIGPIO          84
#define SC_P_ESAI1_FSR                           85
#define SC_P_ESAI1_FST                           86
#define SC_P_ESAI1_SCKR                          87
#define SC_P_ESAI1_SCKT                          88
#define SC_P_ESAI1_TX0                           89
#define SC_P_ESAI1_TX1                           90
#define SC_P_ESAI1_TX2_RX3                       91
#define SC_P_ESAI1_TX3_RX2                       92
#define SC_P_ESAI1_TX4_RX1                       93
#define SC_P_ESAI1_TX5_RX0                       94
#define SC_P_SPDIF0_RX                           95
#define SC_P_SPDIF0_TX                           96
#define SC_P_SPDIF0_EXT_CLK                      97
#define SC_P_SPI3_SCK                            98
#define SC_P_SPI3_SDO                            99
#define SC_P_SPI3_SDI                            100
#define SC_P_SPI3_CS0                            101
#define SC_P_SPI3_CS1                            102
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB       103
#define SC_P_ESAI0_FSR                           104
#define SC_P_ESAI0_FST                           105
#define SC_P_ESAI0_SCKR                          106
#define SC_P_ESAI0_SCKT                          107
#define SC_P_ESAI0_TX0                           108
#define SC_P_ESAI0_TX1                           109
#define SC_P_ESAI0_TX2_RX3                       110
#define SC_P_ESAI0_TX3_RX2                       111
#define SC_P_ESAI0_TX4_RX1                       112
#define SC_P_ESAI0_TX5_RX0                       113
#define SC_P_MCLK_IN0                            114
#define SC_P_MCLK_OUT0                           115
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHC       116
#define SC_P_SPI0_SCK                            117
#define SC_P_SPI0_SDO                            118
#define SC_P_SPI0_SDI                            119
#define SC_P_SPI0_CS0                            120
#define SC_P_SPI0_CS1                            121
#define SC_P_SPI2_SCK                            122
#define SC_P_SPI2_SDO                            123
#define SC_P_SPI2_SDI                            124
#define SC_P_SPI2_CS0                            125
#define SC_P_SPI2_CS1                            126
#define SC_P_SAI1_RXC                            127
#define SC_P_SAI1_RXD                            128
#define SC_P_SAI1_RXFS                           129
#define SC_P_SAI1_TXC                            130
#define SC_P_SAI1_TXD                            131
#define SC_P_SAI1_TXFS                           132
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHT       133
#define SC_P_ADC_IN7                             134
#define SC_P_ADC_IN6                             135
#define SC_P_ADC_IN5                             136
#define SC_P_ADC_IN4                             137
#define SC_P_ADC_IN3                             138
#define SC_P_ADC_IN2                             139
#define SC_P_ADC_IN1                             140
#define SC_P_ADC_IN0                             141
#define SC_P_MLB_SIG                             142
#define SC_P_MLB_CLK                             143
#define SC_P_MLB_DATA                            144
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOLHT       145
#define SC_P_FLEXCAN0_RX                         146
#define SC_P_FLEXCAN0_TX                         147
#define SC_P_FLEXCAN1_RX                         148
#define SC_P_FLEXCAN1_TX                         149
#define SC_P_FLEXCAN2_RX                         150
#define SC_P_FLEXCAN2_TX                         151
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOTHR       152
#define SC_P_USB_SS3_TC0                         153
#define SC_P_USB_SS3_TC1                         154
#define SC_P_USB_SS3_TC2                         155
#define SC_P_USB_SS3_TC3                         156
#define SC_P_COMP_CTL_GPIO_3V3_USB3IO            157
#define SC_P_USDHC1_RESET_B                      158
#define SC_P_USDHC1_VSELECT                      159
#define SC_P_USDHC2_RESET_B                      160
#define SC_P_USDHC2_VSELECT                      161
#define SC_P_USDHC2_WP                           162
#define SC_P_USDHC2_CD_B                         163
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSELSEP       164
#define SC_P_ENET0_MDIO                          165
#define SC_P_ENET0_MDC                           166
#define SC_P_ENET0_REFCLK_125M_25M               167
#define SC_P_ENET1_REFCLK_125M_25M               168
#define SC_P_ENET1_MDIO                          169
#define SC_P_ENET1_MDC                           170
#define SC_P_COMP_CTL_GPIO_1V8_3V3_GPIOCT        171
#define SC_P_QSPI1A_SS0_B                        172
#define SC_P_QSPI1A_SS1_B                        173
#define SC_P_QSPI1A_SCLK                         174
#define SC_P_QSPI1A_DQS                          175
#define SC_P_QSPI1A_DATA3                        176
#define SC_P_QSPI1A_DATA2                        177
#define SC_P_QSPI1A_DATA1                        178
#define SC_P_QSPI1A_DATA0                        179
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI1         180
#define SC_P_QSPI0A_DATA0                        181
#define SC_P_QSPI0A_DATA1                        182
#define SC_P_QSPI0A_DATA2                        183
#define SC_P_QSPI0A_DATA3                        184
#define SC_P_QSPI0A_DQS                          185
#define SC_P_QSPI0A_SS0_B                        186
#define SC_P_QSPI0A_SS1_B                        187
#define SC_P_QSPI0A_SCLK                         188
#define SC_P_QSPI0B_SCLK                         189
#define SC_P_QSPI0B_DATA0                        190
#define SC_P_QSPI0B_DATA1                        191
#define SC_P_QSPI0B_DATA2                        192
#define SC_P_QSPI0B_DATA3                        193
#define SC_P_QSPI0B_DQS                          194
#define SC_P_QSPI0B_SS0_B                        195
#define SC_P_QSPI0B_SS1_B                        196
#define SC_P_COMP_CTL_GPIO_1V8_3V3_QSPI0         197
#define SC_P_PCIE_CTRL0_CLKREQ_B                 198
#define SC_P_PCIE_CTRL0_WAKE_B                   199
#define SC_P_PCIE_CTRL0_PERST_B                  200
#define SC_P_PCIE_CTRL1_CLKREQ_B                 201
#define SC_P_PCIE_CTRL1_WAKE_B                   202
#define SC_P_PCIE_CTRL1_PERST_B                  203
#define SC_P_COMP_CTL_GPIO_1V8_3V3_PCIESEP       204
#define SC_P_USB_HSIC0_DATA                      205
#define SC_P_USB_HSIC0_STROBE                    206
#define SC_P_CALIBRATION_0_HSIC                  207
#define SC_P_CALIBRATION_1_HSIC                  208
#define SC_P_EMMC0_CLK                           209
#define SC_P_EMMC0_CMD                           210
#define SC_P_EMMC0_DATA0                         211
#define SC_P_EMMC0_DATA1                         212
#define SC_P_EMMC0_DATA2                         213
#define SC_P_EMMC0_DATA3                         214
#define SC_P_EMMC0_DATA4                         215
#define SC_P_EMMC0_DATA5                         216
#define SC_P_EMMC0_DATA6                         217
#define SC_P_EMMC0_DATA7                         218
#define SC_P_EMMC0_STROBE                        219
#define SC_P_EMMC0_RESET_B                       220
#define SC_P_COMP_CTL_GPIO_1V8_3V3_SD1FIX        221
#define SC_P_USDHC1_CLK                          222
#define SC_P_USDHC1_CMD                          223
#define SC_P_USDHC1_DATA0                        224
#define SC_P_USDHC1_DATA1                        225
#define SC_P_CTL_NAND_RE_P_N                     226
#define SC_P_USDHC1_DATA2                        227
#define SC_P_USDHC1_DATA3                        228
#define SC_P_CTL_NAND_DQS_P_N                    229
#define SC_P_USDHC1_DATA4                        230
#define SC_P_USDHC1_DATA5                        231
#define SC_P_USDHC1_DATA6                        232
#define SC_P_USDHC1_DATA7                        233
#define SC_P_USDHC1_STROBE                       234
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL2         235
#define SC_P_USDHC2_CLK                          236
#define SC_P_USDHC2_CMD                          237
#define SC_P_USDHC2_DATA0                        238
#define SC_P_USDHC2_DATA1                        239
#define SC_P_USDHC2_DATA2                        240
#define SC_P_USDHC2_DATA3                        241
#define SC_P_COMP_CTL_GPIO_1V8_3V3_VSEL3         242
#define SC_P_ENET0_RGMII_TXC                     243
#define SC_P_ENET0_RGMII_TX_CTL                  244
#define SC_P_ENET0_RGMII_TXD0                    245
#define SC_P_ENET0_RGMII_TXD1                    246
#define SC_P_ENET0_RGMII_TXD2                    247
#define SC_P_ENET0_RGMII_TXD3                    248
#define SC_P_ENET0_RGMII_RXC                     249
#define SC_P_ENET0_RGMII_RX_CTL                  250
#define SC_P_ENET0_RGMII_RXD0                    251
#define SC_P_ENET0_RGMII_RXD1                    252
#define SC_P_ENET0_RGMII_RXD2                    253
#define SC_P_ENET0_RGMII_RXD3                    254
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETB    255
#define SC_P_ENET1_RGMII_TXC                     256
#define SC_P_ENET1_RGMII_TX_CTL                  257
#define SC_P_ENET1_RGMII_TXD0                    258
#define SC_P_ENET1_RGMII_TXD1                    259
#define SC_P_ENET1_RGMII_TXD2                    260
#define SC_P_ENET1_RGMII_TXD3                    261
#define SC_P_ENET1_RGMII_RXC                     262
#define SC_P_ENET1_RGMII_RX_CTL                  263
#define SC_P_ENET1_RGMII_RXD0                    264
#define SC_P_ENET1_RGMII_RXD1                    265
#define SC_P_ENET1_RGMII_RXD2                    266
#define SC_P_ENET1_RGMII_RXD3                    267
#define SC_P_COMP_CTL_GPIO_1V8_3V3_ENET_ENETA    268
#define SC_P_ANA_TEST_OUT_P                      269
#define SC_P_ANA_TEST_OUT_N                      270
#define SC_P_XTALI                               271
#define SC_P_XTALO                               272
#define SC_P_RTC_XTALI                           273
#define SC_P_RTC_XTALO                           274
#define SC_P_PMIC_ON_REQ                         275
#define SC_P_ON_OFF_BUTTON                       276
#define SC_P_SNVS_TAMPER_OUT0                    277
#define SC_P_SNVS_TAMPER_OUT1                    278
#define SC_P_SNVS_TAMPER_IN0                     279
#define SC_P_SNVS_TAMPER_IN1                     280
/*@}*/

#endif /* _SC_PINS_H */

