/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*!
 * Header file containing types used across multiple service APIs.
 */

#ifndef _SC_TYPES_H
#define _SC_TYPES_H

/* Includes */

#include <soc/imx8/sc/scfw.h>

/* Defines */

/*!
 * @name Defines for common frequencies
 */
/*@{*/
#define SC_32KHZ            32768	/* 32KHz */
#define SC_10MHZ         10000000	/* 10MHz */
#define SC_20MHZ         20000000	/* 20MHz */
#define SC_25MHZ         25000000	/* 25MHz */
#define SC_27MHZ         27000000	/* 27MHz */
#define SC_40MHZ         40000000	/* 40MHz */
#define SC_45MHZ         45000000	/* 45MHz */
#define SC_50MHZ         50000000	/* 50MHz */
#define SC_60MHZ         60000000	/* 60MHz */
#define SC_66MHZ         66666666	/* 66MHz */
#define SC_74MHZ         74250000	/* 74.25MHz */
#define SC_80MHZ         80000000	/* 80MHz */
#define SC_83MHZ         83333333	/* 83MHz */
#define SC_84MHZ         84375000	/* 84.37MHz */
#define SC_100MHZ       100000000	/* 100MHz */
#define SC_125MHZ       125000000	/* 125MHz */
#define SC_133MHZ       133333333	/* 133MHz */
#define SC_135MHZ       135000000	/* 135MHz */
#define SC_150MHZ       150000000	/* 150MHz */
#define SC_160MHZ       160000000	/* 160MHz */
#define SC_166MHZ       166666666	/* 160MHz */
#define SC_175MHZ       175000000	/* 175MHz */
#define SC_180MHZ       180000000	/* 180MHz */
#define SC_200MHZ       200000000	/* 200MHz */
#define SC_250MHZ       250000000	/* 250MHz */
#define SC_266MHZ       266666666	/* 266MHz */
#define SC_300MHZ       300000000	/* 300MHz */
#define SC_320MHZ       320000000	/* 320MHz */
#define SC_325MHZ       325000000	/* 325MHz */
#define SC_333MHZ       333333333	/* 333MHz */
#define SC_350MHZ       350000000	/* 350MHz */
#define SC_372MHZ       372000000	/* 372MHz */
#define SC_375MHZ       375000000	/* 375MHz */
#define SC_400MHZ       400000000	/* 400MHz */
#define SC_500MHZ       500000000	/* 500MHz */
#define SC_594MHZ       594000000	/* 594MHz */
#define SC_650MHZ       650000000	/* 650MHz */
#define SC_667MHZ       666666667	/* 667MHz */
#define SC_675MHZ       675000000	/* 675MHz */
#define SC_700MHZ       700000000	/* 700MHz */
#define SC_720MHZ       720000000	/* 720MHz */
#define SC_750MHZ       750000000	/* 750MHz */
#define SC_800MHZ       800000000	/* 800MHz */
#define SC_850MHZ       850000000	/* 850MHz */
#define SC_900MHZ       900000000	/* 900MHz */
#define SC_1000MHZ     1000000000	/* 1GHz */
#define SC_1056MHZ     1056000000	/* 1.056GHz */
#define SC_1188MHZ     1188000000	/* 1.188GHz */
#define SC_1260MHZ     1260000000	/* 1.26GHz */
#define SC_1300MHZ     1300000000	/* 1.3GHz */
#define SC_1400MHZ     1400000000	/* 1.4GHz */
#define SC_1500MHZ     1500000000	/* 1.5GHz */
#define SC_1600MHZ     1600000000	/* 1.6GHz */
#define SC_1800MHZ     1800000000	/* 1.8GHz */
#define SC_2000MHZ     2000000000	/* 2.0GHz */
#define SC_2112MHZ     2112000000	/* 2.12GHz */

/*@}*/

/*!
 * @name Defines for 24M related frequencies
 */
/*@{*/
#define SC_8MHZ           8000000	/* 8MHz */
#define SC_12MHZ         12000000	/* 12MHz */
#define SC_19MHZ         19800000	/* 19.8MHz */
#define SC_24MHZ         24000000	/* 24MHz */
#define SC_48MHZ         48000000	/* 48MHz */
#define SC_120MHZ       120000000	/* 120MHz */
#define SC_132MHZ       132000000	/* 132MHz */
#define SC_144MHZ       144000000	/* 144MHz */
#define SC_192MHZ       192000000	/* 192MHz */
#define SC_211MHZ       211200000	/* 211.2MHz */
#define SC_240MHZ       240000000	/* 240MHz */
#define SC_264MHZ       264000000	/* 264MHz */
#define SC_352MHZ       352000000	/* 352MHz */
#define SC_360MHZ       360000000	/* 360MHz */
#define SC_384MHZ       384000000	/* 384MHz */
#define SC_396MHZ       396000000	/* 396MHz */
#define SC_432MHZ       432000000	/* 432MHz */
#define SC_480MHZ       480000000	/* 480MHz */
#define SC_600MHZ       600000000	/* 600MHz */
#define SC_744MHZ       744000000	/* 744MHz */
#define SC_792MHZ       792000000	/* 792MHz */
#define SC_864MHZ       864000000	/* 864MHz */
#define SC_960MHZ       960000000	/* 960MHz */
#define SC_1056MHZ     1056000000	/* 1056MHz */
#define SC_1200MHZ     1200000000	/* 1.2GHz */
#define SC_1464MHZ     1464000000	/* 1.464GHz */
#define SC_2400MHZ     2400000000	/* 2.4GHz */
/*@}*/

/*!
 * @name Defines for A/V related frequencies
 */
/*@{*/
#define SC_62MHZ         62937500	/* 62.9375MHz */
#define SC_755MHZ       755250000	/* 755.25MHz */
/*@}*/

/*!
 * @name Defines for type widths
 */
/*@{*/
#define SC_FADDR_W      36	/* Width of sc_faddr_t */
#define SC_BOOL_W       1	/* Width of bool */
#define SC_ERR_W        4	/* Width of sc_err_t */
#define SC_RSRC_W       10	/* Width of sc_rsrc_t */
#define SC_CTRL_W       6	/* Width of sc_ctrl_t */
/*@}*/

#define SC_R_ALL        UINT16_MAX	/* All resources */
#define SC_P_ALL        UINT16_MAX	/* All pads */

/*!
 * This type is used to store a system (full-size) address.
 */
typedef uint64_t sc_faddr_t;

/*!
 * This type is used to indicate error response for most functions.
 */
typedef enum sc_err_e {
	SC_ERR_NONE = 0,	/* Success */
	SC_ERR_VERSION = 1,	/* Incompatible API version */
	SC_ERR_CONFIG = 2,	/* Configuration error */
	SC_ERR_PARM = 3,	/* Bad parameter */
	SC_ERR_NOACCESS = 4,	/* Permission error (no access) */
	SC_ERR_LOCKED = 5,	/* Permission error (locked) */
	SC_ERR_UNAVAILABLE = 6,	/* Unavailable (out of resources) */
	SC_ERR_NOTFOUND = 7,	/* Not found */
	SC_ERR_NOPOWER = 8,	/* No power */
	SC_ERR_IPC = 9,		/* Generic IPC error */
	SC_ERR_BUSY = 10,	/* Resource is currently busy/active */
	SC_ERR_FAIL = 11,	/* General I/O failure */
	SC_ERR_LAST
} sc_err_t;

/*!
 * This type is used to indicate a resource. Resources include peripherals
 * and bus masters (but not memory regions). Note items from list should
 * never be changed or removed (only added to at the end of the list).
 */
typedef enum sc_rsrc_e {
	SC_R_A53 = 0,
	SC_R_A53_0 = 1,
	SC_R_A53_1 = 2,
	SC_R_A53_2 = 3,
	SC_R_A53_3 = 4,
	SC_R_A72 = 5,
	SC_R_A72_0 = 6,
	SC_R_A72_1 = 7,
	SC_R_A72_2 = 8,
	SC_R_A72_3 = 9,
	SC_R_CCI = 10,
	SC_R_DB = 11,
	SC_R_DRC_0 = 12,
	SC_R_DRC_1 = 13,
	SC_R_GIC_SMMU = 14,
	SC_R_IRQSTR_M4_0 = 15,
	SC_R_IRQSTR_M4_1 = 16,
	SC_R_SMMU = 17,
	SC_R_GIC = 18,
	SC_R_DC_0_BLIT0 = 19,
	SC_R_DC_0_BLIT1 = 20,
	SC_R_DC_0_BLIT2 = 21,
	SC_R_DC_0_BLIT_OUT = 22,
	SC_R_DC_0_CAPTURE0 = 23,
	SC_R_DC_0_CAPTURE1 = 24,
	SC_R_DC_0_WARP = 25,
	SC_R_DC_0_INTEGRAL0 = 26,
	SC_R_DC_0_INTEGRAL1 = 27,
	SC_R_DC_0_VIDEO0 = 28,
	SC_R_DC_0_VIDEO1 = 29,
	SC_R_DC_0_FRAC0 = 30,
	SC_R_DC_0_FRAC1 = 31,
	SC_R_DC_0 = 32,
	SC_R_GPU_2_PID0 = 33,
	SC_R_DC_0_PLL_0 = 34,
	SC_R_DC_0_PLL_1 = 35,
	SC_R_DC_1_BLIT0 = 36,
	SC_R_DC_1_BLIT1 = 37,
	SC_R_DC_1_BLIT2 = 38,
	SC_R_DC_1_BLIT_OUT = 39,
	SC_R_DC_1_CAPTURE0 = 40,
	SC_R_DC_1_CAPTURE1 = 41,
	SC_R_DC_1_WARP = 42,
	SC_R_DC_1_INTEGRAL0 = 43,
	SC_R_DC_1_INTEGRAL1 = 44,
	SC_R_DC_1_VIDEO0 = 45,
	SC_R_DC_1_VIDEO1 = 46,
	SC_R_DC_1_FRAC0 = 47,
	SC_R_DC_1_FRAC1 = 48,
	SC_R_DC_1 = 49,
	SC_R_GPU_3_PID0 = 50,
	SC_R_DC_1_PLL_0 = 51,
	SC_R_DC_1_PLL_1 = 52,
	SC_R_SPI_0 = 53,
	SC_R_SPI_1 = 54,
	SC_R_SPI_2 = 55,
	SC_R_SPI_3 = 56,
	SC_R_UART_0 = 57,
	SC_R_UART_1 = 58,
	SC_R_UART_2 = 59,
	SC_R_UART_3 = 60,
	SC_R_UART_4 = 61,
	SC_R_EMVSIM_0 = 62,
	SC_R_EMVSIM_1 = 63,
	SC_R_DMA_0_CH0 = 64,
	SC_R_DMA_0_CH1 = 65,
	SC_R_DMA_0_CH2 = 66,
	SC_R_DMA_0_CH3 = 67,
	SC_R_DMA_0_CH4 = 68,
	SC_R_DMA_0_CH5 = 69,
	SC_R_DMA_0_CH6 = 70,
	SC_R_DMA_0_CH7 = 71,
	SC_R_DMA_0_CH8 = 72,
	SC_R_DMA_0_CH9 = 73,
	SC_R_DMA_0_CH10 = 74,
	SC_R_DMA_0_CH11 = 75,
	SC_R_DMA_0_CH12 = 76,
	SC_R_DMA_0_CH13 = 77,
	SC_R_DMA_0_CH14 = 78,
	SC_R_DMA_0_CH15 = 79,
	SC_R_DMA_0_CH16 = 80,
	SC_R_DMA_0_CH17 = 81,
	SC_R_DMA_0_CH18 = 82,
	SC_R_DMA_0_CH19 = 83,
	SC_R_DMA_0_CH20 = 84,
	SC_R_DMA_0_CH21 = 85,
	SC_R_DMA_0_CH22 = 86,
	SC_R_DMA_0_CH23 = 87,
	SC_R_DMA_0_CH24 = 88,
	SC_R_DMA_0_CH25 = 89,
	SC_R_DMA_0_CH26 = 90,
	SC_R_DMA_0_CH27 = 91,
	SC_R_DMA_0_CH28 = 92,
	SC_R_DMA_0_CH29 = 93,
	SC_R_DMA_0_CH30 = 94,
	SC_R_DMA_0_CH31 = 95,
	SC_R_I2C_0 = 96,
	SC_R_I2C_1 = 97,
	SC_R_I2C_2 = 98,
	SC_R_I2C_3 = 99,
	SC_R_I2C_4 = 100,
	SC_R_ADC_0 = 101,
	SC_R_ADC_1 = 102,
	SC_R_FTM_0 = 103,
	SC_R_FTM_1 = 104,
	SC_R_CAN_0 = 105,
	SC_R_CAN_1 = 106,
	SC_R_CAN_2 = 107,
	SC_R_DMA_1_CH0 = 108,
	SC_R_DMA_1_CH1 = 109,
	SC_R_DMA_1_CH2 = 110,
	SC_R_DMA_1_CH3 = 111,
	SC_R_DMA_1_CH4 = 112,
	SC_R_DMA_1_CH5 = 113,
	SC_R_DMA_1_CH6 = 114,
	SC_R_DMA_1_CH7 = 115,
	SC_R_DMA_1_CH8 = 116,
	SC_R_DMA_1_CH9 = 117,
	SC_R_DMA_1_CH10 = 118,
	SC_R_DMA_1_CH11 = 119,
	SC_R_DMA_1_CH12 = 120,
	SC_R_DMA_1_CH13 = 121,
	SC_R_DMA_1_CH14 = 122,
	SC_R_DMA_1_CH15 = 123,
	SC_R_DMA_1_CH16 = 124,
	SC_R_DMA_1_CH17 = 125,
	SC_R_DMA_1_CH18 = 126,
	SC_R_DMA_1_CH19 = 127,
	SC_R_DMA_1_CH20 = 128,
	SC_R_DMA_1_CH21 = 129,
	SC_R_DMA_1_CH22 = 130,
	SC_R_DMA_1_CH23 = 131,
	SC_R_DMA_1_CH24 = 132,
	SC_R_DMA_1_CH25 = 133,
	SC_R_DMA_1_CH26 = 134,
	SC_R_DMA_1_CH27 = 135,
	SC_R_DMA_1_CH28 = 136,
	SC_R_DMA_1_CH29 = 137,
	SC_R_DMA_1_CH30 = 138,
	SC_R_DMA_1_CH31 = 139,
	SC_R_UNUSED1 = 140,
	SC_R_UNUSED2 = 141,
	SC_R_UNUSED3 = 142,
	SC_R_UNUSED4 = 143,
	SC_R_GPU_0_PID0 = 144,
	SC_R_GPU_0_PID1 = 145,
	SC_R_GPU_0_PID2 = 146,
	SC_R_GPU_0_PID3 = 147,
	SC_R_GPU_1_PID0 = 148,
	SC_R_GPU_1_PID1 = 149,
	SC_R_GPU_1_PID2 = 150,
	SC_R_GPU_1_PID3 = 151,
	SC_R_PCIE_A = 152,
	SC_R_SERDES_0 = 153,
	SC_R_MATCH_0 = 154,
	SC_R_MATCH_1 = 155,
	SC_R_MATCH_2 = 156,
	SC_R_MATCH_3 = 157,
	SC_R_MATCH_4 = 158,
	SC_R_MATCH_5 = 159,
	SC_R_MATCH_6 = 160,
	SC_R_MATCH_7 = 161,
	SC_R_MATCH_8 = 162,
	SC_R_MATCH_9 = 163,
	SC_R_MATCH_10 = 164,
	SC_R_MATCH_11 = 165,
	SC_R_MATCH_12 = 166,
	SC_R_MATCH_13 = 167,
	SC_R_MATCH_14 = 168,
	SC_R_PCIE_B = 169,
	SC_R_SATA_0 = 170,
	SC_R_SERDES_1 = 171,
	SC_R_HSIO_GPIO = 172,
	SC_R_MATCH_15 = 173,
	SC_R_MATCH_16 = 174,
	SC_R_MATCH_17 = 175,
	SC_R_MATCH_18 = 176,
	SC_R_MATCH_19 = 177,
	SC_R_MATCH_20 = 178,
	SC_R_MATCH_21 = 179,
	SC_R_MATCH_22 = 180,
	SC_R_MATCH_23 = 181,
	SC_R_MATCH_24 = 182,
	SC_R_MATCH_25 = 183,
	SC_R_MATCH_26 = 184,
	SC_R_MATCH_27 = 185,
	SC_R_MATCH_28 = 186,
	SC_R_LCD_0 = 187,
	SC_R_LCD_0_PWM_0 = 188,
	SC_R_LCD_0_I2C_0 = 189,
	SC_R_LCD_0_I2C_1 = 190,
	SC_R_PWM_0 = 191,
	SC_R_PWM_1 = 192,
	SC_R_PWM_2 = 193,
	SC_R_PWM_3 = 194,
	SC_R_PWM_4 = 195,
	SC_R_PWM_5 = 196,
	SC_R_PWM_6 = 197,
	SC_R_PWM_7 = 198,
	SC_R_GPIO_0 = 199,
	SC_R_GPIO_1 = 200,
	SC_R_GPIO_2 = 201,
	SC_R_GPIO_3 = 202,
	SC_R_GPIO_4 = 203,
	SC_R_GPIO_5 = 204,
	SC_R_GPIO_6 = 205,
	SC_R_GPIO_7 = 206,
	SC_R_GPT_0 = 207,
	SC_R_GPT_1 = 208,
	SC_R_GPT_2 = 209,
	SC_R_GPT_3 = 210,
	SC_R_GPT_4 = 211,
	SC_R_KPP = 212,
	SC_R_MU_0A = 213,
	SC_R_MU_1A = 214,
	SC_R_MU_2A = 215,
	SC_R_MU_3A = 216,
	SC_R_MU_4A = 217,
	SC_R_MU_5A = 218,
	SC_R_MU_6A = 219,
	SC_R_MU_7A = 220,
	SC_R_MU_8A = 221,
	SC_R_MU_9A = 222,
	SC_R_MU_10A = 223,
	SC_R_MU_11A = 224,
	SC_R_MU_12A = 225,
	SC_R_MU_13A = 226,
	SC_R_MU_5B = 227,
	SC_R_MU_6B = 228,
	SC_R_MU_7B = 229,
	SC_R_MU_8B = 230,
	SC_R_MU_9B = 231,
	SC_R_MU_10B = 232,
	SC_R_MU_11B = 233,
	SC_R_MU_12B = 234,
	SC_R_MU_13B = 235,
	SC_R_ROM_0 = 236,
	SC_R_FSPI_0 = 237,
	SC_R_FSPI_1 = 238,
	SC_R_IEE = 239,
	SC_R_IEE_R0 = 240,
	SC_R_IEE_R1 = 241,
	SC_R_IEE_R2 = 242,
	SC_R_IEE_R3 = 243,
	SC_R_IEE_R4 = 244,
	SC_R_IEE_R5 = 245,
	SC_R_IEE_R6 = 246,
	SC_R_IEE_R7 = 247,
	SC_R_SDHC_0 = 248,
	SC_R_SDHC_1 = 249,
	SC_R_SDHC_2 = 250,
	SC_R_ENET_0 = 251,
	SC_R_ENET_1 = 252,
	SC_R_MLB_0 = 253,
	SC_R_DMA_2_CH0 = 254,
	SC_R_DMA_2_CH1 = 255,
	SC_R_DMA_2_CH2 = 256,
	SC_R_DMA_2_CH3 = 257,
	SC_R_DMA_2_CH4 = 258,
	SC_R_USB_0 = 259,
	SC_R_USB_1 = 260,
	SC_R_USB_0_PHY = 261,
	SC_R_USB_2 = 262,
	SC_R_USB_2_PHY = 263,
	SC_R_DTCP = 264,
	SC_R_NAND = 265,
	SC_R_LVDS_0 = 266,
	SC_R_LVDS_0_PWM_0 = 267,
	SC_R_LVDS_0_I2C_0 = 268,
	SC_R_LVDS_0_I2C_1 = 269,
	SC_R_LVDS_1 = 270,
	SC_R_LVDS_1_PWM_0 = 271,
	SC_R_LVDS_1_I2C_0 = 272,
	SC_R_LVDS_1_I2C_1 = 273,
	SC_R_LVDS_2 = 274,
	SC_R_LVDS_2_PWM_0 = 275,
	SC_R_LVDS_2_I2C_0 = 276,
	SC_R_LVDS_2_I2C_1 = 277,
	SC_R_M4_0_PID0 = 278,
	SC_R_M4_0_PID1 = 279,
	SC_R_M4_0_PID2 = 280,
	SC_R_M4_0_PID3 = 281,
	SC_R_M4_0_PID4 = 282,
	SC_R_M4_0_RGPIO = 283,
	SC_R_M4_0_SEMA42 = 284,
	SC_R_M4_0_TPM = 285,
	SC_R_M4_0_PIT = 286,
	SC_R_M4_0_UART = 287,
	SC_R_M4_0_I2C = 288,
	SC_R_M4_0_INTMUX = 289,
	SC_R_M4_0_SIM = 290,
	SC_R_M4_0_WDOG = 291,
	SC_R_M4_0_MU_0B = 292,
	SC_R_M4_0_MU_0A0 = 293,
	SC_R_M4_0_MU_0A1 = 294,
	SC_R_M4_0_MU_0A2 = 295,
	SC_R_M4_0_MU_0A3 = 296,
	SC_R_M4_0_MU_1A = 297,
	SC_R_M4_1_PID0 = 298,
	SC_R_M4_1_PID1 = 299,
	SC_R_M4_1_PID2 = 300,
	SC_R_M4_1_PID3 = 301,
	SC_R_M4_1_PID4 = 302,
	SC_R_M4_1_RGPIO = 303,
	SC_R_M4_1_SEMA42 = 304,
	SC_R_M4_1_TPM = 305,
	SC_R_M4_1_PIT = 306,
	SC_R_M4_1_UART = 307,
	SC_R_M4_1_I2C = 308,
	SC_R_M4_1_INTMUX = 309,
	SC_R_M4_1_SIM = 310,
	SC_R_M4_1_WDOG = 311,
	SC_R_M4_1_MU_0B = 312,
	SC_R_M4_1_MU_0A0 = 313,
	SC_R_M4_1_MU_0A1 = 314,
	SC_R_M4_1_MU_0A2 = 315,
	SC_R_M4_1_MU_0A3 = 316,
	SC_R_M4_1_MU_1A = 317,
	SC_R_SAI_0 = 318,
	SC_R_SAI_1 = 319,
	SC_R_SAI_2 = 320,
	SC_R_IRQSTR_SCU2 = 321,
	SC_R_IRQSTR_DSP = 322,
	SC_R_ELCDIF_PLL = 323,
	SC_R_UNUSED6 = 324,
	SC_R_AUDIO_PLL_0 = 325,
	SC_R_PI_0 = 326,
	SC_R_PI_0_PWM_0 = 327,
	SC_R_PI_0_PWM_1 = 328,
	SC_R_PI_0_I2C_0 = 329,
	SC_R_PI_0_PLL = 330,
	SC_R_PI_1 = 331,
	SC_R_PI_1_PWM_0 = 332,
	SC_R_PI_1_PWM_1 = 333,
	SC_R_PI_1_I2C_0 = 334,
	SC_R_PI_1_PLL = 335,
	SC_R_SC_PID0 = 336,
	SC_R_SC_PID1 = 337,
	SC_R_SC_PID2 = 338,
	SC_R_SC_PID3 = 339,
	SC_R_SC_PID4 = 340,
	SC_R_SC_SEMA42 = 341,
	SC_R_SC_TPM = 342,
	SC_R_SC_PIT = 343,
	SC_R_SC_UART = 344,
	SC_R_SC_I2C = 345,
	SC_R_SC_MU_0B = 346,
	SC_R_SC_MU_0A0 = 347,
	SC_R_SC_MU_0A1 = 348,
	SC_R_SC_MU_0A2 = 349,
	SC_R_SC_MU_0A3 = 350,
	SC_R_SC_MU_1A = 351,
	SC_R_SYSCNT_RD = 352,
	SC_R_SYSCNT_CMP = 353,
	SC_R_DEBUG = 354,
	SC_R_SYSTEM = 355,
	SC_R_SNVS = 356,
	SC_R_OTP = 357,
	SC_R_VPU_PID0 = 358,
	SC_R_VPU_PID1 = 359,
	SC_R_VPU_PID2 = 360,
	SC_R_VPU_PID3 = 361,
	SC_R_VPU_PID4 = 362,
	SC_R_VPU_PID5 = 363,
	SC_R_VPU_PID6 = 364,
	SC_R_VPU_PID7 = 365,
	SC_R_VPU_UART = 366,
	SC_R_VPUCORE = 367,
	SC_R_VPUCORE_0 = 368,
	SC_R_VPUCORE_1 = 369,
	SC_R_VPUCORE_2 = 370,
	SC_R_VPUCORE_3 = 371,
	SC_R_DMA_4_CH0 = 372,
	SC_R_DMA_4_CH1 = 373,
	SC_R_DMA_4_CH2 = 374,
	SC_R_DMA_4_CH3 = 375,
	SC_R_DMA_4_CH4 = 376,
	SC_R_ISI_CH0 = 377,
	SC_R_ISI_CH1 = 378,
	SC_R_ISI_CH2 = 379,
	SC_R_ISI_CH3 = 380,
	SC_R_ISI_CH4 = 381,
	SC_R_ISI_CH5 = 382,
	SC_R_ISI_CH6 = 383,
	SC_R_ISI_CH7 = 384,
	SC_R_MJPEG_DEC_S0 = 385,
	SC_R_MJPEG_DEC_S1 = 386,
	SC_R_MJPEG_DEC_S2 = 387,
	SC_R_MJPEG_DEC_S3 = 388,
	SC_R_MJPEG_ENC_S0 = 389,
	SC_R_MJPEG_ENC_S1 = 390,
	SC_R_MJPEG_ENC_S2 = 391,
	SC_R_MJPEG_ENC_S3 = 392,
	SC_R_MIPI_0 = 393,
	SC_R_MIPI_0_PWM_0 = 394,
	SC_R_MIPI_0_I2C_0 = 395,
	SC_R_MIPI_0_I2C_1 = 396,
	SC_R_MIPI_1 = 397,
	SC_R_MIPI_1_PWM_0 = 398,
	SC_R_MIPI_1_I2C_0 = 399,
	SC_R_MIPI_1_I2C_1 = 400,
	SC_R_CSI_0 = 401,
	SC_R_CSI_0_PWM_0 = 402,
	SC_R_CSI_0_I2C_0 = 403,
	SC_R_CSI_1 = 404,
	SC_R_CSI_1_PWM_0 = 405,
	SC_R_CSI_1_I2C_0 = 406,
	SC_R_HDMI = 407,
	SC_R_HDMI_I2S = 408,
	SC_R_HDMI_I2C_0 = 409,
	SC_R_HDMI_PLL_0 = 410,
	SC_R_HDMI_RX = 411,
	SC_R_HDMI_RX_BYPASS = 412,
	SC_R_HDMI_RX_I2C_0 = 413,
	SC_R_ASRC_0 = 414,
	SC_R_ESAI_0 = 415,
	SC_R_SPDIF_0 = 416,
	SC_R_SPDIF_1 = 417,
	SC_R_SAI_3 = 418,
	SC_R_SAI_4 = 419,
	SC_R_SAI_5 = 420,
	SC_R_GPT_5 = 421,
	SC_R_GPT_6 = 422,
	SC_R_GPT_7 = 423,
	SC_R_GPT_8 = 424,
	SC_R_GPT_9 = 425,
	SC_R_GPT_10 = 426,
	SC_R_DMA_2_CH5 = 427,
	SC_R_DMA_2_CH6 = 428,
	SC_R_DMA_2_CH7 = 429,
	SC_R_DMA_2_CH8 = 430,
	SC_R_DMA_2_CH9 = 431,
	SC_R_DMA_2_CH10 = 432,
	SC_R_DMA_2_CH11 = 433,
	SC_R_DMA_2_CH12 = 434,
	SC_R_DMA_2_CH13 = 435,
	SC_R_DMA_2_CH14 = 436,
	SC_R_DMA_2_CH15 = 437,
	SC_R_DMA_2_CH16 = 438,
	SC_R_DMA_2_CH17 = 439,
	SC_R_DMA_2_CH18 = 440,
	SC_R_DMA_2_CH19 = 441,
	SC_R_DMA_2_CH20 = 442,
	SC_R_DMA_2_CH21 = 443,
	SC_R_DMA_2_CH22 = 444,
	SC_R_DMA_2_CH23 = 445,
	SC_R_DMA_2_CH24 = 446,
	SC_R_DMA_2_CH25 = 447,
	SC_R_DMA_2_CH26 = 448,
	SC_R_DMA_2_CH27 = 449,
	SC_R_DMA_2_CH28 = 450,
	SC_R_DMA_2_CH29 = 451,
	SC_R_DMA_2_CH30 = 452,
	SC_R_DMA_2_CH31 = 453,
	SC_R_ASRC_1 = 454,
	SC_R_ESAI_1 = 455,
	SC_R_SAI_6 = 456,
	SC_R_SAI_7 = 457,
	SC_R_AMIX = 458,
	SC_R_MQS_0 = 459,
	SC_R_DMA_3_CH0 = 460,
	SC_R_DMA_3_CH1 = 461,
	SC_R_DMA_3_CH2 = 462,
	SC_R_DMA_3_CH3 = 463,
	SC_R_DMA_3_CH4 = 464,
	SC_R_DMA_3_CH5 = 465,
	SC_R_DMA_3_CH6 = 466,
	SC_R_DMA_3_CH7 = 467,
	SC_R_DMA_3_CH8 = 468,
	SC_R_DMA_3_CH9 = 469,
	SC_R_DMA_3_CH10 = 470,
	SC_R_DMA_3_CH11 = 471,
	SC_R_DMA_3_CH12 = 472,
	SC_R_DMA_3_CH13 = 473,
	SC_R_DMA_3_CH14 = 474,
	SC_R_DMA_3_CH15 = 475,
	SC_R_DMA_3_CH16 = 476,
	SC_R_DMA_3_CH17 = 477,
	SC_R_DMA_3_CH18 = 478,
	SC_R_DMA_3_CH19 = 479,
	SC_R_DMA_3_CH20 = 480,
	SC_R_DMA_3_CH21 = 481,
	SC_R_DMA_3_CH22 = 482,
	SC_R_DMA_3_CH23 = 483,
	SC_R_DMA_3_CH24 = 484,
	SC_R_DMA_3_CH25 = 485,
	SC_R_DMA_3_CH26 = 486,
	SC_R_DMA_3_CH27 = 487,
	SC_R_DMA_3_CH28 = 488,
	SC_R_DMA_3_CH29 = 489,
	SC_R_DMA_3_CH30 = 490,
	SC_R_DMA_3_CH31 = 491,
	SC_R_AUDIO_PLL_1 = 492,
	SC_R_AUDIO_CLK_0 = 493,
	SC_R_AUDIO_CLK_1 = 494,
	SC_R_MCLK_OUT_0 = 495,
	SC_R_MCLK_OUT_1 = 496,
	SC_R_PMIC_0 = 497,
	SC_R_PMIC_1 = 498,
	SC_R_SECO = 499,
	SC_R_CAAM_JR1 = 500,
	SC_R_CAAM_JR2 = 501,
	SC_R_CAAM_JR3 = 502,
	SC_R_SECO_MU_2 = 503,
	SC_R_SECO_MU_3 = 504,
	SC_R_SECO_MU_4 = 505,
	SC_R_HDMI_RX_PWM_0 = 506,
	SC_R_A35 = 507,
	SC_R_A35_0 = 508,
	SC_R_A35_1 = 509,
	SC_R_A35_2 = 510,
	SC_R_A35_3 = 511,
	SC_R_DSP = 512,
	SC_R_DSP_RAM = 513,
	SC_R_CAAM_JR1_OUT = 514,
	SC_R_CAAM_JR2_OUT = 515,
	SC_R_CAAM_JR3_OUT = 516,
	SC_R_VPU_DEC_0 = 517,
	SC_R_VPU_ENC_0 = 518,
	SC_R_CAAM_JR0 = 519,
	SC_R_CAAM_JR0_OUT = 520,
	SC_R_PMIC_2 = 521,
	SC_R_DBLOGIC = 522,
	SC_R_HDMI_PLL_1 = 523,
	SC_R_BOARD_R0 = 524,
	SC_R_BOARD_R1 = 525,
	SC_R_BOARD_R2 = 526,
	SC_R_BOARD_R3 = 527,
	SC_R_BOARD_R4 = 528,
	SC_R_BOARD_R5 = 529,
	SC_R_BOARD_R6 = 530,
	SC_R_BOARD_R7 = 531,
	SC_R_MJPEG_DEC_MP = 532,
	SC_R_MJPEG_ENC_MP = 533,
	SC_R_VPU_TS_0 = 534,
	SC_R_VPU_MU_0 = 535,
	SC_R_VPU_MU_1 = 536,
	SC_R_VPU_MU_2 = 537,
	SC_R_VPU_MU_3 = 538,
	SC_R_VPU_ENC_1 = 539,
	SC_R_VPU = 540,
	SC_R_LAST
} sc_rsrc_t;

/* NOTE - please add by replacing some of the UNUSED from above! */

/*!
 * This type is used to indicate a control.
 */
typedef enum sc_ctrl_e {

	SC_C_TEMP = 0,
	SC_C_TEMP_HI = 1,
	SC_C_TEMP_LOW = 2,
	SC_C_PXL_LINK_MST1_ADDR = 3,
	SC_C_PXL_LINK_MST2_ADDR = 4,
	SC_C_PXL_LINK_MST_ENB = 5,
	SC_C_PXL_LINK_MST1_ENB = 6,
	SC_C_PXL_LINK_MST2_ENB = 7,
	SC_C_PXL_LINK_SLV1_ADDR = 8,
	SC_C_PXL_LINK_SLV2_ADDR = 9,
	SC_C_PXL_LINK_MST_VLD = 10,
	SC_C_PXL_LINK_MST1_VLD = 11,
	SC_C_PXL_LINK_MST2_VLD = 12,
	SC_C_SINGLE_MODE = 13,
	SC_C_ID = 14,
	SC_C_PXL_CLK_POLARITY = 15,
	SC_C_LINESTATE = 16,
	SC_C_PCIE_G_RST = 17,
	SC_C_PCIE_BUTTON_RST = 18,
	SC_C_PCIE_PERST = 19,
	SC_C_PHY_RESET = 20,
	SC_C_PXL_LINK_RATE_CORRECTION = 21,
	SC_C_PANIC = 22,
	SC_C_PRIORITY_GROUP = 23,
	SC_C_TXCLK = 24,
	SC_C_CLKDIV = 25,
	SC_C_DISABLE_50 = 26,
	SC_C_DISABLE_125 = 27,
	SC_C_SEL_125 = 28,
	SC_C_MODE = 29,
	SC_C_SYNC_CTRL0 = 30,
	SC_C_KACHUNK_CNT = 31,
	SC_C_KACHUNK_SEL = 32,
	SC_C_SYNC_CTRL1 = 33,
	SC_C_DPI_RESET = 34,
	SC_C_MIPI_RESET = 35,
	SC_C_DUAL_MODE = 36,
	SC_C_VOLTAGE = 37,
	SC_C_PXL_LINK_SEL = 38,
	SC_C_OFS_SEL = 39,
	SC_C_OFS_AUDIO = 40,
	SC_C_OFS_PERIPH = 41,
	SC_C_OFS_IRQ = 42,
	SC_C_RST0 = 43,
	SC_C_RST1 = 44,
	SC_C_SEL0 = 45,
	SC_C_CALIB0 = 46,
	SC_C_CALIB1 = 47,
	SC_C_CALIB2 = 48,
	SC_C_IPG_DEBUG = 49,
	SC_C_IPG_DOZE = 50,
	SC_C_IPG_WAIT = 51,
	SC_C_IPG_STOP = 52,
	SC_C_IPG_STOP_MODE = 53,
	SC_C_IPG_STOP_ACK = 54,
	SC_C_SYNC_CTRL = 55,
	SC_C_LAST
} sc_ctrl_t;

/*!
 * This type is used to indicate a pad. Valid values are SoC specific.
 *
 * Refer to the SoC [Pad List](@ref PADS) for valid pad values.
 */
typedef uint16_t sc_pad_t;

/* Extra documentation of standard types */

#ifdef DOXYGEN
    /*!
     * Type used to declare a true/false boolean.
     */
typedef enum { false = 0, true = 1 } bool;

    /*!
     * Type used to declare an 8-bit integer.
     */
typedef __INT8_TYPE__ int8_t;

    /*!
     * Type used to declare a 16-bit integer.
     */
typedef __INT16_TYPE__ int16_t;

    /*!
     * Type used to declare a 32-bit integer.
     */
typedef __INT32_TYPE__ int32_t;

    /*!
     * Type used to declare a 64-bit integer.
     */
typedef __INT64_TYPE__ int64_t;

    /*!
     * Type used to declare an 8-bit unsigned integer.
     */
typedef __UINT8_TYPE__ uint8_t;

    /*!
     * Type used to declare a 16-bit unsigned integer.
     */
typedef __UINT16_TYPE__ uint16_t;

    /*!
     * Type used to declare a 32-bit unsigned integer.
     */
typedef __UINT32_TYPE__ uint32_t;

    /*!
     * Type used to declare a 64-bit unsigned integer.
     */
typedef __UINT64_TYPE__ uint64_t;
#endif

#endif				/* _SC_TYPES_H */
