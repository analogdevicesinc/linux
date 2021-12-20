// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/gpio/consumer.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#include <linux/module.h>
#include <linux/moduleparam.h>

//address of ADE90XX registers
#define	ADDR_AIGAIN					0x000
#define	ADDR_AIGAIN0				0x001
#define	ADDR_AIGAIN1				0x002
#define	ADDR_AIGAIN2				0x003
#define	ADDR_AIGAIN3				0x004
#define	ADDR_AIGAIN4				0x005
#define	ADDR_APHCAL0				0x006
#define	ADDR_APHCAL1				0x007
#define	ADDR_APHCAL2				0x008
#define	ADDR_APHCAL3				0x009
#define	ADDR_APHCAL4				0x00A
#define	ADDR_AVGAIN					0x00B
#define	ADDR_AIRMSOS				0x00C
#define	ADDR_AVRMSOS				0x00D
#define	ADDR_APGAIN					0x00E
#define	ADDR_AWATTOS				0x00F
#define	ADDR_AVAROS					0x010
#define	ADDR_AFWATTOS				0x011
#define	ADDR_AFVAROS				0x012
#define	ADDR_AIFRMSOS				0x013
#define	ADDR_AVFRMSOS				0x014
#define	ADDR_AVRMSONEOS				0x015
#define	ADDR_AIRMSONEOS				0x016
#define	ADDR_AVRMS1012OS			0x017
#define	ADDR_AIRMS1012OS			0x018
#define	ADDR_BIGAIN					0x020
#define	ADDR_BIGAIN0				0x021
#define	ADDR_BIGAIN1				0x022
#define	ADDR_BIGAIN2				0x023
#define	ADDR_BIGAIN3				0x024
#define	ADDR_BIGAIN4				0x025
#define	ADDR_BPHCAL0				0x026
#define	ADDR_BPHCAL1				0x027
#define	ADDR_BPHCAL2				0x028
#define	ADDR_BPHCAL3				0x029
#define	ADDR_BPHCAL4				0x02A
#define	ADDR_BVGAIN					0x02B
#define	ADDR_BIRMSOS				0x02C
#define	ADDR_BVRMSOS				0x02D
#define	ADDR_BPGAIN					0x02E
#define	ADDR_BWATTOS				0x02F
#define	ADDR_BVAROS					0x030
#define	ADDR_BFWATTOS				0x031
#define	ADDR_BFVAROS				0x032
#define	ADDR_BIFRMSOS				0x033
#define	ADDR_BVFRMSOS				0x034
#define	ADDR_BVRMSONEOS				0x035
#define	ADDR_BIRMSONEOS				0x036
#define	ADDR_BVRMS1012OS			0x037
#define	ADDR_BIRMS1012OS			0x038
#define	ADDR_CIGAIN					0x040
#define	ADDR_CIGAIN0				0x041
#define	ADDR_CIGAIN1				0x042
#define	ADDR_CIGAIN2				0x043
#define	ADDR_CIGAIN3				0x044
#define	ADDR_CIGAIN4				0x045
#define	ADDR_CPHCAL0				0x046
#define	ADDR_CPHCAL1				0x047
#define	ADDR_CPHCAL2				0x048
#define	ADDR_CPHCAL3				0x049
#define	ADDR_CPHCAL4				0x04A
#define	ADDR_CVGAIN					0x04B
#define	ADDR_CIRMSOS				0x04C
#define	ADDR_CVRMSOS				0x04D
#define	ADDR_CPGAIN					0x04E
#define	ADDR_CWATTOS				0x04F
#define	ADDR_CVAROS					0x050
#define	ADDR_CFWATTOS				0x051
#define	ADDR_CFVAROS				0x052
#define	ADDR_CIFRMSOS				0x053
#define	ADDR_CVFRMSOS				0x054
#define	ADDR_CVRMSONEOS				0x055
#define	ADDR_CIRMSONEOS				0x056
#define	ADDR_CVRMS1012OS			0x057
#define	ADDR_CIRMS1012OS			0x058
#define	ADDR_CONFIG0				0x060
#define	ADDR_MTTHR_L0				0x061
#define	ADDR_MTTHR_L1				0x062
#define	ADDR_MTTHR_L2				0x063
#define	ADDR_MTTHR_L3				0x064
#define	ADDR_MTTHR_L4				0x065
#define	ADDR_MTTHR_H0				0x066
#define	ADDR_MTTHR_H1				0x067
#define	ADDR_MTTHR_H2				0x068
#define	ADDR_MTTHR_H3				0x069
#define	ADDR_MTTHR_H4				0x06A
#define	ADDR_NIRMSOS				0x06B
#define	ADDR_ISUMRMSOS				0x06C
#define	ADDR_NIGAIN					0x06D
#define	ADDR_NPHCAL					0x06E
#define	ADDR_NIRMSONEOS				0x06F
#define	ADDR_NIRMS1012OS			0x070
#define	ADDR_VNOM					0x071
#define	ADDR_DICOEFF				0x072
#define	ADDR_ISUMLVL				0x073
#define	ADDR_AI_PCF					0x20A
#define	ADDR_AV_PCF					0x20B
#define	ADDR_AIRMS					0x20C
#define	ADDR_AVRMS					0x20D
#define	ADDR_AIFRMS					0x20E
#define	ADDR_AVFRMS					0x20F
#define	ADDR_AWATT					0x210
#define	ADDR_AVAR					0x211
#define	ADDR_AVA					0x212
#define	ADDR_AFWATT					0x213
#define	ADDR_AFVAR					0x214
#define	ADDR_AFVA					0x215
#define	ADDR_APF					0x216
#define	ADDR_AVTHD					0x217
#define	ADDR_AITHD					0x218
#define	ADDR_AIRMSONE				0x219
#define	ADDR_AVRMSONE				0x21A
#define	ADDR_AIRMS1012				0x21B
#define	ADDR_AVRMS1012				0x21C
#define	ADDR_AMTREGION				0x21D
#define	ADDR_BI_PCF					0x22A
#define	ADDR_BV_PCF					0x22B
#define	ADDR_BIRMS					0x22C
#define	ADDR_BVRMS					0x22D
#define	ADDR_BIFRMS					0x22E
#define	ADDR_BVFRMS					0x22F
#define	ADDR_BWATT					0x230
#define	ADDR_BVAR					0x231
#define	ADDR_BVA					0x232
#define	ADDR_BFWATT					0x233
#define	ADDR_BFVAR					0x234
#define	ADDR_BFVA					0x235
#define	ADDR_BPF					0x236
#define	ADDR_BVTHD					0x237
#define	ADDR_BITHD					0x238
#define	ADDR_BIRMSONE				0x239
#define	ADDR_BVRMSONE				0x23A
#define	ADDR_BIRMS1012				0x23B
#define	ADDR_BVRMS1012				0x23C
#define	ADDR_BMTREGION				0x23D
#define	ADDR_CI_PCF					0x24A
#define	ADDR_CV_PCF					0x24B
#define	ADDR_CIRMS					0x24C
#define	ADDR_CVRMS					0x24D
#define	ADDR_CIFRMS					0x24E
#define	ADDR_CVFRMS					0x24F
#define	ADDR_CWATT					0x250
#define	ADDR_CVAR					0x251
#define	ADDR_CVA					0x252
#define	ADDR_CFWATT					0x253
#define	ADDR_CFVAR					0x254
#define	ADDR_CFVA					0x255
#define	ADDR_CPF					0x256
#define	ADDR_CVTHD					0x257
#define	ADDR_CITHD					0x258
#define	ADDR_CIRMSONE				0x259
#define	ADDR_CVRMSONE				0x25A
#define	ADDR_CIRMS1012				0x25B
#define	ADDR_CVRMS1012				0x25C
#define	ADDR_CMTREGION				0x25D
#define	ADDR_NI_PCF					0x265
#define	ADDR_NIRMS					0x266
#define	ADDR_NIRMSONE				0x267
#define	ADDR_NIRMS1012				0x268
#define	ADDR_ISUMRMS				0x269
#define	ADDR_VERSION2				0x26A
#define	ADDR_AWATT_ACC				0x2E5
#define	ADDR_AWATTHR_LO				0x2E6
#define	ADDR_AWATTHR_HI				0x2E7
#define	ADDR_AVAR_ACC				0x2EF
#define	ADDR_AVARHR_LO				0x2F0
#define	ADDR_AVARHR_HI				0x2F1
#define	ADDR_AVA_ACC				0x2F9
#define	ADDR_AVAHR_LO				0x2FA
#define	ADDR_AVAHR_HI				0x2FB
#define	ADDR_AFWATT_ACC				0x303
#define	ADDR_AFWATTHR_LO			0x304
#define	ADDR_AFWATTHR_HI			0x305
#define	ADDR_AFVAR_ACC				0x30D
#define	ADDR_AFVARHR_LO				0x30E
#define	ADDR_AFVARHR_HI				0x30F
#define	ADDR_AFVA_ACC				0x317
#define	ADDR_AFVAHR_LO				0x318
#define	ADDR_AFVAHR_HI				0x319
#define	ADDR_BWATT_ACC				0x321
#define	ADDR_BWATTHR_LO				0x322
#define	ADDR_BWATTHR_HI				0x323
#define	ADDR_BVAR_ACC				0x32B
#define	ADDR_BVARHR_LO				0x32C
#define	ADDR_BVARHR_HI				0x32D
#define	ADDR_BVA_ACC				0x335
#define	ADDR_BVAHR_LO				0x336
#define	ADDR_BVAHR_HI				0x337
#define	ADDR_BFWATT_ACC				0x33F
#define	ADDR_BFWATTHR_LO			0x340
#define	ADDR_BFWATTHR_HI			0x341
#define	ADDR_BFVAR_ACC				0x349
#define	ADDR_BFVARHR_LO				0x34A
#define	ADDR_BFVARHR_HI				0x34B
#define	ADDR_BFVA_ACC				0x353
#define	ADDR_BFVAHR_LO				0x354
#define	ADDR_BFVAHR_HI				0x355
#define	ADDR_CWATT_ACC				0x35D
#define	ADDR_CWATTHR_LO				0x35E
#define	ADDR_CWATTHR_HI				0x35F
#define	ADDR_CVAR_ACC				0x367
#define	ADDR_CVARHR_LO				0x368
#define	ADDR_CVARHR_HI				0x369
#define	ADDR_CVA_ACC				0x371
#define	ADDR_CVAHR_LO				0x372
#define	ADDR_CVAHR_HI				0x373
#define	ADDR_CFWATT_ACC				0x37B
#define	ADDR_CFWATTHR_LO			0x37C
#define	ADDR_CFWATTHR_HI			0x37D
#define	ADDR_CFVAR_ACC				0x385
#define	ADDR_CFVARHR_LO				0x386
#define	ADDR_CFVARHR_HI				0x387
#define	ADDR_CFVA_ACC				0x38F
#define	ADDR_CFVAHR_LO				0x390
#define	ADDR_CFVAHR_HI				0x391
#define	ADDR_PWATT_ACC				0x397
#define	ADDR_NWATT_ACC				0x39B
#define	ADDR_PVAR_ACC				0x39F
#define	ADDR_NVAR_ACC				0x3A3
#define	ADDR_IPEAK					0x400
#define	ADDR_VPEAK					0x401
#define	ADDR_STATUS0				0x402
#define	ADDR_STATUS1				0x403
#define	ADDR_EVENT_STATUS			0x404
#define	ADDR_MASK0					0x405
#define	ADDR_MASK1					0x406
#define	ADDR_EVENT_MASK				0x407
#define	ADDR_OILVL					0x409
#define	ADDR_OIA					0x40A
#define	ADDR_OIB					0x40B
#define	ADDR_OIC					0x40C
#define	ADDR_OIN					0x40D
#define	ADDR_USER_PERIOD			0x40E
#define	ADDR_VLEVEL					0x40F
#define	ADDR_DIP_LVL				0x410
#define	ADDR_DIPA					0x411
#define	ADDR_DIPB					0x412
#define	ADDR_DIPC					0x413
#define	ADDR_SWELL_LVL				0x414
#define	ADDR_SWELLA					0x415
#define	ADDR_SWELLB					0x416
#define	ADDR_SWELLC					0x417
#define	ADDR_APERIOD				0x418
#define	ADDR_BPERIOD				0x419
#define	ADDR_CPERIOD				0x41A
#define	ADDR_COM_PERIOD				0x41B
#define	ADDR_ACT_NL_LVL				0x41C
#define	ADDR_REACT_NL_LVL			0x41D
#define	ADDR_APP_NL_LVL				0x41E
#define	ADDR_PHNOLOAD				0x41F
#define	ADDR_WTHR					0x420
#define	ADDR_VARTHR					0x421
#define	ADDR_VATHR					0x422
#define	ADDR_LAST_DATA_32			0x423
#define	ADDR_ADC_REDIRECT			0x424
#define	ADDR_CF_LCFG				0x425
#define	ADDR_TEMP_TRIM				0x474
#define	ADDR_RUN					0x480
#define	ADDR_CONFIG1				0x481
#define	ADDR_ANGL_VA_VB				0x482
#define	ADDR_ANGL_VB_VC				0x483
#define	ADDR_ANGL_VA_VC				0x484
#define	ADDR_ANGL_VA_IA				0x485
#define	ADDR_ANGL_VB_IB				0x486
#define	ADDR_ANGL_VC_IC				0x487
#define	ADDR_ANGL_IA_IB				0x488
#define	ADDR_ANGL_IB_IC				0x489
#define	ADDR_ANGL_IA_IC				0x48A
#define	ADDR_DIP_CYC				0x48B
#define	ADDR_SWELL_CYC				0x48C
#define	ADDR_OISTATUS				0x48F
#define	ADDR_CFMODE					0x490
#define	ADDR_COMPMODE				0x491
#define	ADDR_ACCMODE				0x492
#define	ADDR_CONFIG3				0x493
#define	ADDR_CF1DEN					0x494
#define	ADDR_CF2DEN					0x495
#define	ADDR_CF3DEN					0x496
#define	ADDR_CF4DEN					0x497
#define	ADDR_ZXTOUT					0x498
#define	ADDR_ZXTHRSH				0x499
#define	ADDR_ZX_LP_SEL				0x49A
#define	ADDR_SEQ_CYC				0x49C
#define	ADDR_PHSIGN					0x49D
#define	ADDR_WFB_CFG				0x4A0
#define	ADDR_WFB_PG_IRQEN			0x4A1
#define	ADDR_WFB_TRG_CFG			0x4A2
#define	ADDR_WFB_TRG_STAT			0x4A3
#define	ADDR_CONFIG5				0x4A4
#define	ADDR_CRC_RSLT				0x4A8
#define	ADDR_CRC_SPI				0x4A9
#define	ADDR_LAST_DATA_16			0x4AC
#define	ADDR_LAST_CMD				0x4AE
#define	ADDR_CONFIG2				0x4AF
#define	ADDR_EP_CFG					0x4B0
#define	ADDR_PWR_TIME				0x4B1
#define	ADDR_EGY_TIME				0x4B2
#define	ADDR_CRC_FORCE				0x4B4
#define	ADDR_CRC_OPTEN				0x4B5
#define	ADDR_TEMP_CFG				0x4B6
#define	ADDR_TEMP_RSLT				0x4B7
#define	ADDR_PSM2_CFG				0x4B8
#define	ADDR_PGA_GAIN				0x4B9
#define	ADDR_CHNL_DIS				0x4BA
#define	ADDR_WR_LOCK				0x4BF
#define	ADDR_VAR_DIS				0x4E0
#define	ADDR_RESERVED1				0x4F0
#define	ADDR_VERSION				0x4FE
#define	ADDR_AI_SINC_DAT			0x500
#define	ADDR_AV_SINC_DAT			0x501
#define	ADDR_BI_SINC_DAT			0x502
#define	ADDR_BV_SINC_DAT			0x503
#define	ADDR_CI_SINC_DAT			0x504
#define	ADDR_CV_SINC_DAT			0x505
#define	ADDR_NI_SINC_DAT			0x506
#define	ADDR_AI_LPF_DAT				0x510
#define	ADDR_AV_LPF_DAT				0x511
#define	ADDR_BI_LPF_DAT				0x512
#define	ADDR_BV_LPF_DAT				0x513
#define	ADDR_CI_LPF_DAT				0x514
#define	ADDR_CV_LPF_DAT				0x515
#define	ADDR_NI_LPF_DAT				0x516
#define	ADDR_AV_PCF_1				0x600
#define	ADDR_BV_PCF_1				0x601
#define	ADDR_CV_PCF_1				0x602
#define	ADDR_NI_PCF_1				0x603
#define	ADDR_AI_PCF_1				0x604
#define	ADDR_BI_PCF_1				0x605
#define	ADDR_CI_PCF_1				0x606
#define	ADDR_AIRMS_1				0x607
#define	ADDR_BIRMS_1				0x608
#define	ADDR_CIRMS_1				0x609
#define	ADDR_AVRMS_1				0x60A
#define	ADDR_BVRMS_1				0x60B
#define	ADDR_CVRMS_1				0x60C
#define	ADDR_NIRMS_1				0x60D
#define	ADDR_AWATT_1				0x60E
#define	ADDR_BWATT_1				0x60F
#define	ADDR_CWATT_1				0x610
#define	ADDR_AVA_1					0x611
#define	ADDR_BVA_1					0x612
#define	ADDR_CVA_1					0x613
#define	ADDR_AVAR_1					0x614
#define	ADDR_BVAR_1					0x615
#define	ADDR_CVAR_1					0x616
#define	ADDR_AFVAR_1				0x617
#define	ADDR_BFVAR_1				0x618
#define	ADDR_CFVAR_1				0x619
#define	ADDR_APF_1					0x61A
#define	ADDR_BPF_1					0x61B
#define	ADDR_CPF_1					0x61C
#define	ADDR_AVTHD_1				0x61D
#define	ADDR_BVTHD_1				0x61E
#define	ADDR_CVTHD_1				0x61F
#define	ADDR_AITHD_1				0x620
#define	ADDR_BITHD_1				0x621
#define	ADDR_CITHD_1				0x622
#define	ADDR_AFWATT_1				0x623
#define	ADDR_BFWATT_1				0x624
#define	ADDR_CFWATT_1				0x625
#define	ADDR_AFVA_1					0x626
#define	ADDR_BFVA_1					0x627
#define	ADDR_CFVA_1					0x628
#define	ADDR_AFIRMS_1				0x629
#define	ADDR_BFIRMS_1				0x62A
#define	ADDR_CFIRMS_1				0x62B
#define	ADDR_AFVRMS_1				0x62C
#define	ADDR_BFVRMS_1				0x62D
#define	ADDR_CFVRMS_1				0x62E
#define	ADDR_AIRMSONE_1				0x62F
#define	ADDR_BIRMSONE_1				0x630
#define	ADDR_CIRMSONE_1				0x631
#define	ADDR_AVRMSONE_1				0x632
#define	ADDR_BVRMSONE_1				0x633
#define	ADDR_CVRMSONE_1				0x634
#define	ADDR_NIRMSONE_1				0x635
#define	ADDR_AIRMS1012_1			0x636
#define	ADDR_BIRMS1012_1			0x637
#define	ADDR_CIRMS1012_1			0x638
#define	ADDR_AVRMS1012_1			0x639
#define	ADDR_BVRMS1012_1			0x63A
#define	ADDR_CVRMS1012_1			0x63B
#define	ADDR_NIRMS1012_1			0x63C
#define	ADDR_AV_PCF_2				0x680
#define	ADDR_AI_PCF_2				0x681
#define	ADDR_AIRMS_2				0x682
#define	ADDR_AVRMS_2				0x683
#define	ADDR_AWATT_2				0x684
#define	ADDR_AVA_2					0x685
#define	ADDR_AVAR_2					0x686
#define	ADDR_AFVAR_2				0x687
#define	ADDR_APF_2					0x688
#define	ADDR_AVTHD_2				0x689
#define	ADDR_AITHD_2				0x68A
#define	ADDR_AFWATT_2				0x68B
#define	ADDR_AFVA_2					0x68C
#define	ADDR_AFIRMS_2				0x68D
#define	ADDR_AFVRMS_2				0x68E
#define	ADDR_AIRMSONE_2				0x68F
#define	ADDR_AVRMSONE_2				0x690
#define	ADDR_AIRMS1012_2			0x691
#define	ADDR_AVRMS1012_2			0x692
#define	ADDR_BV_PCF_2				0x693
#define	ADDR_BI_PCF_2				0x694
#define	ADDR_BIRMS_2				0x695
#define	ADDR_BVRMS_2				0x696
#define	ADDR_BWATT_2				0x697
#define	ADDR_BVA_2					0x698
#define	ADDR_BVAR_2					0x699
#define	ADDR_BFVAR_2				0x69A
#define	ADDR_BPF_2					0x69B
#define	ADDR_BVTHD_2				0x69C
#define	ADDR_BITHD_2				0x69D
#define	ADDR_BFWATT_2				0x69E
#define	ADDR_BFVA_2					0x69F
#define	ADDR_BFIRMS_2				0x6A0
#define	ADDR_BFVRMS_2				0x6A1
#define	ADDR_BIRMSONE_2				0x6A2
#define	ADDR_BVRMSONE_2				0x6A3
#define	ADDR_BIRMS1012_2			0x6A4
#define	ADDR_BVRMS1012_2			0x6A5
#define	ADDR_CV_PCF_2				0x6A6
#define	ADDR_CI_PCF_2				0x6A7
#define	ADDR_CIRMS_2				0x6A8
#define	ADDR_CVRMS_2				0x6A9
#define	ADDR_CWATT_2				0x6AA
#define	ADDR_CVA_2					0x6AB
#define	ADDR_CVAR_2					0x6AC
#define	ADDR_CFVAR_2				0x6AD
#define	ADDR_CPF_2					0x6AE
#define	ADDR_CVTHD_2				0x6AF
#define	ADDR_CITHD_2				0x6B0
#define	ADDR_CFWATT_2				0x6B1
#define	ADDR_CFVA_2					0x6B2
#define	ADDR_CFIRMS_2				0x6B3
#define	ADDR_CFVRMS_2				0x6B4
#define	ADDR_CIRMSONE_2				0x6B5
#define	ADDR_CVRMSONE_2				0x6B6
#define	ADDR_CIRMS1012_2			0x6B7
#define	ADDR_CVRMS1012_2			0x6B8
#define	ADDR_NI_PCF_2				0x6B9
#define	ADDR_NIRMS_2				0x6BA
#define	ADDR_NIRMSONE_2				0x6BB
#define	ADDR_NIRMS1012_2			0x6BC
#define ADDR_WF_BUFF				0x800

#define ADE9078_WRITE_REG(x)		(x << 4)
#define ADE9078_READ_REG(x)			(((x) << 4) | 0x08)

/*Configuran registers*/
/*PGA@0x0000. Gain of all channels=1*/
#define ADE9078_PGA_GAIN 			0x0000

/*Default configuration*/
#define ADE9078_CONFIG0 			0x00000000

/*CF3/ZX pin outputs Zero crossing */
#define ADE9078_CONFIG1				0x0002

/*Default High pass corner frequency of 1.25Hz*/
#define ADE9078_CONFIG2				0x0A00

/*Peak and overcurrent detection disabled*/
#define ADE9078_CONFIG3				0x0000

/*50Hz operation, 3P4W Wye configuration, signed accumulation*/
/*Clear bit 8 i.e. ACCMODE=0x00xx for 50Hz operation*/
/*ACCMODE=0x0x9x for 3Wire delta when phase B is used as reference*/
#define ADE9078_ACCMODE 			0x0000

/*Line period and zero crossing obtained from VA*/
#define ADE9078_ZX_LP_SEL 			0x0000

/*Disable all interrupts*/
#define ADE9078_MASK0 				0x00000000

/*Disable all interrupts*/
#define ADE9078_MASK1 				0x00000000

/*Events disabled */
#define ADE9078_EVENT_MASK 			0x00000000

/*Assuming Vnom=1/2 of full scale.*/
/*Refer Technical reference manual for detailed calculations.*/
#define ADE9078_VLEVEL				0x0022EA28

/* Set DICOEFF= 0xFFFFE000 when integrator is enabled*/
#define ADE9078_DICOEFF 			0x00000000

/*DSP ON*/
#define ADE9078_RUN_ON 				0xFFFFFFFF

/*Energy Accumulation Settings*/
/*Enable energy accumulation, accumulate samples at 8ksps*/
/*latch energy accumulation after EGYRDY*/
/*If accumulation is changed to half line cycle mode, change EGY_TIME*/
#define ADE9078_EP_CFG 				0x0011

/*Accumulate 4000 samples*/
#define ADE9078_EGY_TIME 			0x0FA0

/*Constant Definitions***/
/*ADE9000 FDSP: 8000sps, ADE9078 FDSP: 4000sps*/
#define ADE9078_FDSP 				4000
#define ADE9078_WFB_CFG 			0x0329
#define ADE9078_WFB_PAGE_SIZE 		128
#define ADE9078_WFB_BYTES_IN_PAGE 	4
#define ADE9078_WFB_PAGE_ARRAY_SIZE \
	ADE9078_WFB_PAGE_SIZE * ADE9078_WFB_BYTES_IN_PAGE
#define ADE9078_WFB_FULL_BUFF_SIZE 	\
	ADE9078_WFB_PAGE_ARRAY_SIZE * 16
#define ADE9078_WFB_FULL_BUFF_NR_SAMPLES \
	ADE9078_WFB_PAGE_SIZE * 16

/*Status and Mask register bits*/
#define ADE9078_ST0_EGYRDY 			BIT(0)
#define ADE9078_ST0_REVAPA 			BIT(1)
#define ADE9078_ST0_REVAPB 			BIT(2)
#define ADE9078_ST0_REVAPC 			BIT(3)
#define ADE9078_ST0_REVRPA 			BIT(4)
#define ADE9078_ST0_REVRPB 			BIT(5)
#define ADE9078_ST0_REVRPC 			BIT(6)
#define ADE9078_ST0_REVPSUM1 		BIT(7)
#define ADE9078_ST0_REVPSUM2 		BIT(8)
#define ADE9078_ST0_REVPSUM3 		BIT(9)
#define ADE9078_ST0_REVPSUM4 		BIT(10)
#define ADE9078_ST0_CF1 			BIT(11)
#define ADE9078_ST0_CF2 			BIT(12)
#define ADE9078_ST0_CF3 			BIT(13)
#define ADE9078_ST0_CF4 			BIT(14)
#define ADE9078_ST0_DREADY 			BIT(15)
#define ADE9078_ST0_WFB_TRIG_IRQ 	BIT(16)
#define ADE9078_ST0_PAGE_FULL 		BIT(17)
#define ADE9078_ST0_PWRRDY 			BIT(18)
#define ADE9078_ST0_PF_RDY 			BIT(21)
#define ADE9078_ST0_WFB_TRIG 		BIT(22)
#define ADE9078_ST0_COH_WFB_FULL 	BIT(23)
#define ADE9078_ST0_MISMTCH 		BIT(24)

#define ADE9078_ST1_ANLOAD			BIT(0)
#define ADE9078_ST1_RNLOAD			BIT(1)
#define ADE9078_ST1_VANLOAD			BIT(2)
#define ADE9078_ST1_RFNOLOAD		BIT(4)
#define ADE9078_ST1_ZXTOVA			BIT(6)
#define ADE9078_ST1_ZXTOVB			BIT(7)
#define ADE9078_ST1_ZXTOVC			BIT(8)
#define ADE9078_ST1_ZXVA			BIT(9)
#define ADE9078_ST1_ZXVB			BIT(10)
#define ADE9078_ST1_ZXVC			BIT(11)
#define ADE9078_ST1_ZXCOMB			BIT(12)
#define ADE9078_ST1_ZXIA			BIT(13)
#define ADE9078_ST1_ZXIB			BIT(14)
#define ADE9078_ST1_ZXIC			BIT(15)
#define ADE9078_ST1_RSTDONE			BIT(16)
#define ADE9078_ST1_SEQERR			BIT(18)
#define ADE9078_ST1_CRC_CHG			BIT(26)
#define ADE9078_ST1_CRC_DONE		BIT(27)
#define ADE9078_ST1_ERROR0			BIT(28)
#define ADE9078_ST1_ERROR1			BIT(29)
#define ADE9078_ST1_ERROR2			BIT(30)
#define ADE9078_ST1_ERROR3			BIT(31)
#define ADE9078_ST_ERROR \
	ADE9078_ST1_ERROR0 | \
	ADE9078_ST1_ERROR1 | \
	ADE9078_ST1_ERROR2 | \
	ADE9078_ST1_ERROR3


/*Full scale Codes referred from Datasheet.Respective digital codes are
 * produced when ADC inputs are at full scale. Do not Change. */
#define ADE9078_RMS_FULL_SCALE_CODES  	52866837
#define ADE9000_WATT_FULL_SCALE_CODES 	20694066
#define ADE9078_PCF_FULL_SCALE_CODES  	74770000

/*Phase and channel definitions*/
#define ADE9078_PHASE_A_NR				0
#define ADE9078_PHASE_B_NR				2
#define ADE9078_PHASE_C_NR				4
#define ADE9078_PHASE_A_NAME			"A"
#define ADE9078_PHASE_B_NAME			"B"
#define ADE9078_PHASE_C_NAME			"C"

#define ADE9078_SCAN_POS_IA				BIT(0)
#define ADE9078_SCAN_POS_VA				BIT(1)
#define ADE9078_SCAN_POS_IB				BIT(2)
#define ADE9078_SCAN_POS_VB				BIT(3)
#define ADE9078_SCAN_POS_IC				BIT(4)
#define ADE9078_SCAN_POS_VC				BIT(5)

#define ADE9078_MAX_PHASE_NR			3

#define PHASE_ADDR_ADJUST(addr,chan)	((chan << 4) | (addr))

#define ADE9078_CHANNEL(num, name)						\
	{													\
		.type = IIO_CURRENT,							\
		.channel = num,									\
		.extend_name = name,							\
		.address = PHASE_ADDR_ADJUST(ADDR_AI_PCF,num),	\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.event_spec = ade9078_events,					\
		.num_event_specs = ARRAY_SIZE(ade9078_events),	\
		.scan_index = num,								\
		.scan_type = {									\
			.sign = 's',								\
			.realbits = 32,								\
			.storagebits = 32,							\
			.shift = 0,									\
			.endianness = IIO_BE,						\
		},												\
	},													\
	{													\
		.type = IIO_VOLTAGE,							\
		.channel = num,									\
		.extend_name = name,							\
		.address = PHASE_ADDR_ADJUST(ADDR_AV_PCF,num),	\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.event_spec = ade9078_events,					\
		.num_event_specs = ARRAY_SIZE(ade9078_events),	\
		.scan_index = num + 1,							\
		.scan_type = {									\
			.sign = 's',								\
			.realbits = 32,								\
			.storagebits = 32,							\
			.shift = 0,									\
			.endianness = IIO_BE,						\
		},												\
	},													\
	{													\
		.type = IIO_CURRENT,							\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_AIRMS,num),	\
		.extend_name = name "_rms",						\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	},													\
	{													\
		.type = IIO_VOLTAGE,							\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_AVRMS,num),	\
		.extend_name = name "_rms",						\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	},													\
	{													\
		.type = IIO_POWER,								\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_AVAR,num),	\
		.extend_name = name "_reactiv",					\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	},													\
	{													\
		.type = IIO_POWER,								\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_AVA,num),		\
		.extend_name = name "_apparent",				\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	},													\
	{													\
		.type = IIO_POWER,								\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_AWATT,num),	\
		.extend_name = name "_fund_reactiv",			\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	},													\
	{													\
		.type = IIO_POWER,								\
		.channel = num,									\
		.address = PHASE_ADDR_ADJUST(ADDR_APF,num),		\
		.extend_name = name "_factor",					\
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |  \
							BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = -1								\
	}


/*
 * struct ade9078_device - ade9078 specific data
 * @lock		mutex for the device
 * @slock		spinlock used for irq handling
 * @gpio_reset	reset gpio pointer, retrieved from DT
 * @irq0_bits	IRQ0 mask and status bits, are set by the driver and are passed
 * 				to the IC after being set
 * @irq1_bits	IRQ1 mask and status bits, are set by the driver and are passed
 * 				to the IC after being set
 * @irq1_status status variable updated in irq1 and used in IIO event readout
 * @rst_done	flag for when reset sequence irq has been received
 * @wf_mode		wave form buffer mode, read datasheet for more details,
 * 				retrieved from DT
 * @wfb_trg_cfg	wave form buffer triger configuration, read datasheet for more
 * 				details, retrieved from DT
 * @triggered	flag used in irq0 ISR to mark that the frame buffer has been
 * 				triggered for output
 * @spi 		spi device associated to the ade9078
 * @tx			transmit buffer for the spi
 * @rx			receive buffer for the spi
 * @tx_buff		transmit buffer for the iio buffer trough spi, used in iio
 * 				buffer configuration
 * @rx_buff		receive buffer for the iio buffer trough spi, will contain the
 * 				samples from the IC wave form buffer
 * @xfer		transfer setup used in iio buffer configuration
 * @spi_msg		message transfer trough spi, used in iio buffer configuration
 * @regmap		register map pointer
 * @indio_dev:	the IIO device
 * @trig		iio trigger pointer, is connected to IRQ0 and IRQ1
 */
struct ade9078_device {
	struct mutex lock;
	spinlock_t slock;
	struct gpio_desc *gpio_reset;
	u32 irq0_bits;
	u32 irq1_bits;
	u32 irq1_status;
	bool rst_done;
	u8 wf_mode;
	u16 wfb_trg_cfg;
	volatile bool triggered;
	struct spi_device *spi;
	u8 *tx;
	u8 *rx;
	u8 tx_buff[2];
	union
	{
		u8 byte[ADE9078_WFB_FULL_BUFF_SIZE];
		__be32 word[ADE9078_WFB_FULL_BUFF_NR_SAMPLES];
	}rx_buff ____cacheline_aligned;
	struct spi_transfer	xfer[2];
	struct spi_message spi_msg;
	struct regmap *regmap;
	struct iio_dev *indio_dev;
	struct iio_trigger *trig;
};

static const struct iio_event_spec ade9078_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE) |
						BIT(IIO_EV_INFO_VALUE),
	},
};

//IIO channels of the ade9078 for each phase individually
static const struct iio_chan_spec ade9078_a_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_A_NR, ADE9078_PHASE_A_NAME),
};
static const struct iio_chan_spec ade9078_b_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_B_NR, ADE9078_PHASE_B_NAME),
};
static const struct iio_chan_spec ade9078_c_channels[] = {
		ADE9078_CHANNEL(ADE9078_PHASE_C_NR, ADE9078_PHASE_C_NAME),
};

/*
 * ade9078_spi_write_reg() - ade9078 write register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be written to the ade9078
 */
static int ade9078_spi_write_reg(void *context, unsigned int reg,
		unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_WRITE_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[5] = (u8)  val & 0xFF;
	ade9078_dev->tx[4] = (u8) (val >> 8) & 0xFF;
	ade9078_dev->tx[3] = (u8) (val >> 16) & 0xFF;
	ade9078_dev->tx[2] = (u8) (val >> 24) & 0xFF;
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
	{
		ade9078_dev->tx[3] = (u8)  val & 0xFF;
		ade9078_dev->tx[2] = (u8) (val >> 8) & 0xFF;
		xfer[0].len = 4;
	}

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret)
	{
		dev_err(&ade9078_dev->spi->dev, "problem when writing register 0x%x",
				reg);
	}

	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_spi_write_reg() - ade9078 read register over SPI
 * the data format for communicating with the ade9078 over SPI
 * is very specific
 * @context:	void pointer to the SPI device
 * @reg:		address of the of desired register
 * @val:  		value to be read to the ade9078
 */
static int ade9078_spi_read_reg(void *context, unsigned int reg,
		unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);

	u16 addr;
	int ret = 0;
	struct spi_transfer xfer[] = {
		{
			.tx_buf = ade9078_dev->tx,
			.bits_per_word = 8,
			.len = 2,
		},
		{
			.rx_buf = ade9078_dev->rx,
			.bits_per_word = 8,
			.len = 6,
		},
	};

	addr = ADE9078_READ_REG(reg);

	mutex_lock(&ade9078_dev->lock);
	ade9078_dev->tx[1] = (u8) addr;
	ade9078_dev->tx[0] = (u8) (addr >> 8);

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		xfer[1].len = 4;

	ret = spi_sync_transfer(ade9078_dev->spi, xfer, ARRAY_SIZE(xfer));
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "problem when reading register 0x%x",
				reg);
		goto err_ret;
	}

	//registers which are 16 bits
	if(reg > 0x480 && reg < 0x4FE)
		*val = (ade9078_dev->rx[0] << 8) | ade9078_dev->rx[1];
	else
		*val = (ade9078_dev->rx[0] << 24) | (ade9078_dev->rx[1] << 16) |
		(ade9078_dev->rx[2] << 8) | ade9078_dev->rx[3];

err_ret:
	mutex_unlock(&ade9078_dev->lock);
	return ret;
}

/*
 * ade9078_en_wfb() - enables or disables the WFBuffer in the ADE9078
 * @ade9078_dev:		ade9078 device data
 * @state:				true for enabled; false for disabled
 */
static int ade9078_en_wfb(struct ade9078_device *ade9078_dev, bool state)
{
	if(state)
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG,
				BIT_MASK(4), BIT_MASK(4));
	else
		return regmap_update_bits(ade9078_dev->regmap, ADDR_WFB_CFG,
				BIT_MASK(4), 0);
}

/*
 * ade9078_update_mask0() - updates interrupt mask0 and resets all of the status
 * 							register 0
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_update_mask0(struct ade9078_device *ade9078_dev)
{
	unsigned int ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS0,
			0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK0, ade9078_dev->irq0_bits);
	if (ret)
		return ret;

	return 0;
}

/*
 * ade9078_update_mask0() - updates interrupt mask1 and resets all of the status
 * 							register 1
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_update_mask1(struct ade9078_device *ade9078_dev)
{
	unsigned int ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS1,
			0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK1, ade9078_dev->irq1_bits);
	if (ret)
		return ret;

	return 0;
}

/*
 * ade9078_test_bits() - tests the bits of a given register within the IC
 * @map:				regmap device
 * @reg:				register to be tested
 * @bits:				bits to be checked
 *
 * Returns 0 if at least one of the tested bits is not set, 1 if all tested
 * bits are set and a negative error number if the underlying regmap_read()
 * fails.
 */
static int ade9078_test_bits(struct regmap *map, unsigned int reg,
		unsigned int bits)
{
	unsigned int val, ret;

	ret = regmap_read(map, reg, &val);
	if (ret)
		return ret;

	return (val & bits) == bits;
}

/*
 * ade9078_irq0_thread() - Thread for IRQ0. It reads Status register 0 and
 * checks for the IRQ activation. This is configured to acquire samples in to
 * the IC buffer and dump it in to the iio_buffer according to Stop When Buffer
 * Is Full Mode, Stop Filling on Trigger and Capture Around Trigger from the
 * ADE9078 Datasheet
 */
static irqreturn_t ade9078_irq0_thread(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;
	u32 status;
	u32 handled_irq = 0;

	regmap_read(ade9078_dev->regmap, ADDR_STATUS0, &status);
	dev_dbg(&ade9078_dev->spi->dev, "IRQ0 status 0x%x", status);
	ade9078_dev->triggered = false;

	if(((status & ADE9078_ST0_PAGE_FULL) == ADE9078_ST0_PAGE_FULL) &&
	((ade9078_dev->irq0_bits & ADE9078_ST0_PAGE_FULL) ==
			ADE9078_ST0_PAGE_FULL))
	{
		//Stop Filling on Trigger and Center Capture Around Trigger
		if(ade9078_dev->wf_mode){
			regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG,
					ade9078_dev->wfb_trg_cfg);
			ade9078_dev->irq0_bits |= ADE9078_ST0_WFB_TRIG_IRQ;
		}
		else{
			//Stop When Buffer Is Full Mode
			ade9078_en_wfb(ade9078_dev, false);
//			iio_trigger_poll(ade9078_dev->trig);
			ade9078_dev->triggered = true;
		}

		//disable Page full interrupt
		ade9078_dev->irq0_bits &= ~ADE9078_ST0_PAGE_FULL;
		regmap_write(ade9078_dev->regmap, ADDR_MASK0,
				ade9078_dev->irq0_bits);

		dev_dbg(&ade9078_dev->spi->dev, "IRQ0 ADE9078_ST0_PAGE_FULL");
		handled_irq |= ADE9078_ST0_PAGE_FULL;
	}

	if(((status & ADE9078_ST0_WFB_TRIG_IRQ) == ADE9078_ST0_WFB_TRIG_IRQ) &&
	((ade9078_dev->irq0_bits & ADE9078_ST0_WFB_TRIG_IRQ) ==
			ADE9078_ST0_WFB_TRIG_IRQ))
	{
		//Stop Filling on Trigger and Center Capture Around Trigger
		ade9078_en_wfb(ade9078_dev, false);
//		iio_trigger_poll(ade9078_dev->trig);
		ade9078_dev->triggered = true;

		handled_irq |= ADE9078_ST0_WFB_TRIG_IRQ;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ0 ADE9078_ST0_WFB_TRIG_IRQ");
	}

	regmap_write(ade9078_dev->regmap, ADDR_STATUS0, handled_irq);

	dev_dbg(&ade9078_dev->spi->dev, "IRQ0 thread done");

	return IRQ_HANDLED;
}

/*
 * ade9078_irq1_thread() - Thread for IRQ1. It reads Status register 1 and
 * checks for the IRQ activation. This thread handles the reset condition and
 * the zero-crossing conditions for all 3 phases on Voltage and Current
 */
static irqreturn_t ade9078_irq1_thread(int irq, void *data)
{
	struct ade9078_device *ade9078_dev = data;
	struct iio_dev *indio_dev = ade9078_dev->indio_dev;
	int result;
	u32 status;
	s64 timestamp = iio_get_time_ns(indio_dev);

	//reset
	if(ade9078_dev->rst_done == false){
		result = ade9078_test_bits(ade9078_dev->regmap, ADDR_STATUS1,
				ADE9078_ST1_RSTDONE);
		if(result < 0)
			dev_err(&ade9078_dev->spi->dev, "Error testing reset done");
		else if(result == 1)
			ade9078_dev->rst_done = true;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 Reset");
		goto irq_done;
	}

	regmap_read(ade9078_dev->regmap, ADDR_STATUS1, &status);

	//crossings
	if(((status & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXVA;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVA");
	}
	if(((status & ADE9078_ST1_ZXTOVA) == ADE9078_ST1_ZXTOVA) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVA) == ADE9078_ST1_ZXTOVA))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXTOVA;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXTOVA");
	}

	if(((status & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXVB;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVB");

	}
	if(((status & ADE9078_ST1_ZXTOVB) == ADE9078_ST1_ZXTOVB) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVB) == ADE9078_ST1_ZXTOVB))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXTOVB;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXTOVB");
	}

	if(((status & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXVC;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXVC");
	}
	if(((status & ADE9078_ST1_ZXTOVC) == ADE9078_ST1_ZXTOVC) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVC) == ADE9078_ST1_ZXTOVC))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXTOVC;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXTOVC");
	}
	if(((status & ADE9078_ST1_ZXIA) == ADE9078_ST1_ZXIA) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXIA) == ADE9078_ST1_ZXIA))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							ADE9078_PHASE_A_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXIA;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXIA");
	}
	if(((status & ADE9078_ST1_ZXIB) == ADE9078_ST1_ZXIB) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXIB) == ADE9078_ST1_ZXIB))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							ADE9078_PHASE_B_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXIB;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXIB");
	}
	if(((status & ADE9078_ST1_ZXIC) == ADE9078_ST1_ZXIC) &&
	   ((ade9078_dev->irq1_bits & ADE9078_ST1_ZXIC) == ADE9078_ST1_ZXIC))
	{
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_CURRENT,
							ADE9078_PHASE_C_NR,
							IIO_EV_TYPE_THRESH,
							IIO_EV_DIR_EITHER),
				   timestamp);
		ade9078_dev->irq1_status |= ADE9078_ST1_ZXIC;
		dev_dbg(&ade9078_dev->spi->dev, "IRQ1 ADE9078_ST1_ZXIC");
	}

irq_done:
	dev_dbg(&ade9078_dev->spi->dev, "IRQ1 thread done");
	return IRQ_HANDLED;
}

/*
 * ade9078_pop_wfb() - parses the SPI receive buffer, rearranges
 * the bits and pushes the data to the IIO buffer
 */
static void ade9078_pop_wfb(struct iio_poll_func *pf)
{
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 i;

	for(i=0; i <= ADE9078_WFB_FULL_BUFF_NR_SAMPLES; i++)
	{
		iio_push_to_buffers(ade9078_dev->indio_dev,
				&ade9078_dev->rx_buff.word[i]);
	}

	dev_dbg(&ade9078_dev->spi->dev, "Pushed to buffer");
}

/*
 * ade9078_trigger_handler() - the bottom half of the pollfunc
 * for the iio trigger buffer. It acquires data trough the SPI
 * and rearranges the data to match BE
 */
static irqreturn_t ade9078_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	dev_dbg(&ade9078_dev->spi->dev, "Triggered");
	if(bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
	{
		dev_err(&ade9078_dev->spi->dev, "Bitmap empty in trigger handler");
		goto err_out;
	}

	mutex_lock(&ade9078_dev->lock);
	ret = spi_sync(ade9078_dev->spi, &ade9078_dev->spi_msg);
	if(ret)
	{
		mutex_unlock(&ade9078_dev->lock);
		dev_err(&ade9078_dev->spi->dev, "SPI fail in trigger handler");
		goto err_out;
	}

	ade9078_pop_wfb(pf);

	mutex_unlock(&ade9078_dev->lock);

err_out:

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

/*
 * ade9078_configure_scan() - sets up the transfer parameters
 * as well as the tx and rx buffers
 * @indio_dev:		the IIO device
 */
static int ade9078_configure_scan(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret = 0;
	u16 addr;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	addr = ADE9078_READ_REG(ADDR_WF_BUFF);

	ade9078_dev->tx_buff[1] = (u8) addr;
	ade9078_dev->tx_buff[0] = (u8) (addr >> 8);

	ade9078_dev->xfer[0].tx_buf = &ade9078_dev->tx_buff[0];
	ade9078_dev->xfer[0].bits_per_word = 8;
	ade9078_dev->xfer[0].len = 2;

	ade9078_dev->xfer[1].rx_buf = &ade9078_dev->rx_buff.byte[0];
	ade9078_dev->xfer[1].bits_per_word = 8;
	ade9078_dev->xfer[1].len = ADE9078_WFB_FULL_BUFF_SIZE;

	spi_message_init(&ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[0], &ade9078_dev->spi_msg);
	spi_message_add_tail(&ade9078_dev->xfer[1], &ade9078_dev->spi_msg);

	return ret;
}

/*
 * ade9078_read_raw() - IIO read function
 * @indio_dev:		the IIO device
 * @chan:			channel specs of the ade9078
 * @val:			first half of the read value
 * @val2:			second half of the read value
 * @mask:			info mask of the channel
 */
static int ade9078_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	int ret;
	int32_t *measured;
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = regmap_read(ade9078_dev->regmap, chan->address, measured);

		iio_device_release_direct_mode(indio_dev);
		*val = *measured;

		return IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:
		switch(chan->type) {
		case IIO_CURRENT:
			if(chan->address >= ADDR_AI_PCF && chan->address <= ADDR_CI_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if(chan->address >= ADDR_AIRMS && chan->address <= ADDR_CIRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_VOLTAGE:
			if(chan->address >= ADDR_AV_PCF && chan->address <= ADDR_CV_PCF){
				*val = 1;
				*val2 = ADE9078_PCF_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			if(chan->address >= ADDR_AVRMS && chan->address <= ADDR_CVRMS){
				*val = 1;
				*val2 = ADE9078_RMS_FULL_SCALE_CODES;
				return IIO_VAL_FRACTIONAL;
			}
			break;
		case IIO_POWER:
			*val = 1;
			*val2 = ADE9000_WATT_FULL_SCALE_CODES;
			return IIO_VAL_FRACTIONAL;
			break;
		default: break;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

/*
 * ade9078_reg_acess() - IIO debug register access
 * @indio_dev:		the IIO device
 * @reg:			register to be accessed
 * @tx_val:			value to be transmitted
 * @rx_val:			value to be received
 */
static int ade9078_reg_acess(struct iio_dev *indio_dev,
		unsigned int reg,
		unsigned int tx_val,
		unsigned int *rx_val)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(ade9078_dev->regmap, reg, rx_val);
	else
		return regmap_write(ade9078_dev->regmap, reg, tx_val);
}

/*
 * ade9078_write_event_config() - IIO event configure to enable zero-crossing
 * and zero-crossing timeout on voltage and current for each phases. These
 * events will also influence the trigger conditions for the buffer capture.
 */
static int ade9078_write_event_config(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				int state)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 number;
	u32 status1 = 0;
	int ret;

	dev_dbg(&ade9078_dev->spi->dev, "Enter event");

	number = chan->channel;
	dev_dbg(&ade9078_dev->spi->dev, "Event channel %d", number);
	switch(number){
	case ADE9078_PHASE_A_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVA | ADE9078_ST1_ZXTOVA;
				ade9078_dev->wfb_trg_cfg |= BIT(6);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVA set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVA &
										  ~ADE9078_ST1_ZXTOVA;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVA &
											~ADE9078_ST1_ZXTOVA;
				ade9078_dev->wfb_trg_cfg &= ~BIT(6);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVA cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIA;
				ade9078_dev->wfb_trg_cfg |= BIT(3);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIA set");
			}
			else {
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIA;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIA;
				ade9078_dev->wfb_trg_cfg &= ~BIT(3);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIA cleared");
			}
		}
		break;
	case ADE9078_PHASE_B_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVB | ADE9078_ST1_ZXTOVB;
				ade9078_dev->wfb_trg_cfg |= BIT(7);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVB set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVB &
										  ~ADE9078_ST1_ZXTOVB;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVB &
											~ADE9078_ST1_ZXTOVB;
				ade9078_dev->wfb_trg_cfg &= ~BIT(7);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVB cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIB;
				ade9078_dev->wfb_trg_cfg |= BIT(4);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIB set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIB;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIB;
				ade9078_dev->wfb_trg_cfg &= ~BIT(4);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIB cleared");
			}
		}
		break;
	case ADE9078_PHASE_C_NR:
		if(chan->type == IIO_VOLTAGE){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXVC | ADE9078_ST1_ZXTOVC;
				ade9078_dev->wfb_trg_cfg |= BIT(8);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVC set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXVC &
										  ~ADE9078_ST1_ZXTOVC;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVC &
											~ADE9078_ST1_ZXTOVC;
				ade9078_dev->wfb_trg_cfg &= ~BIT(8);
				dev_dbg(&ade9078_dev->spi->dev, "ZXVC cleared");
			}
		}
		else if(chan->type == IIO_CURRENT){
			if(state){
				ade9078_dev->irq1_bits |= ADE9078_ST1_ZXIC;
				ade9078_dev->wfb_trg_cfg |= BIT(5);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIC set");
			}
			else{
				ade9078_dev->irq1_bits &= ~ADE9078_ST1_ZXIC;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIC;
				ade9078_dev->wfb_trg_cfg &= ~BIT(5);
				dev_dbg(&ade9078_dev->spi->dev, "ZXIC cleared");
			}
		}
		break;
	default:
		break;
	}

	ret = ade9078_update_mask1(ade9078_dev);
	if(ret)
		return ret;

	//clear relevant status
	status1 |= ADE9078_ST1_ZXVA | ADE9078_ST1_ZXTOVA |
			   ADE9078_ST1_ZXVB | ADE9078_ST1_ZXTOVB |
			   ADE9078_ST1_ZXVC | ADE9078_ST1_ZXTOVC |
			   ADE9078_ST1_ZXIA | ADE9078_ST1_ZXIB | ADE9078_ST1_ZXIC;
	regmap_write(ade9078_dev->regmap, ADDR_STATUS1, status1);

	return 0;
}

/*
 * ade9078_read_event_vlaue() - Outputs the result of the zero-crossing for
 * voltage and current for each phase.
 * Result:
 * 0 - if crossing event not set
 * 1 - if crossing event occurred
 * -1 - if crossing timeout (only for Voltages)
 */
static int ade9078_read_event_vlaue(struct iio_dev *indio_dev,
	      const struct iio_chan_spec *chan,
	      enum iio_event_type type,
	      enum iio_event_direction dir,
	      enum iio_event_info info,
	      int *val, int *val2)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 number;
	u32 status1;
	u32 handled_irq1 = 0;

	*val = 0;
	status1 = ade9078_dev->irq1_status;

	number = chan->channel;
	switch(number){
	case ADE9078_PHASE_A_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVA) == ADE9078_ST1_ZXVA){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVA;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVA;
			}
			else if((status1 & ADE9078_ST1_ZXTOVA) == ADE9078_ST1_ZXTOVA){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVA;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXTOVA;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVA) !=
					ADE9078_ST1_ZXTOVA)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIA) == ADE9078_ST1_ZXIA){
				handled_irq1 |= ADE9078_ST1_ZXIA;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIA;
				*val = 1;
			}

		}
		break;
	case ADE9078_PHASE_B_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVB) == ADE9078_ST1_ZXVB){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVB;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVB;
			}
			else if((status1 & ADE9078_ST1_ZXTOVB) == ADE9078_ST1_ZXTOVB){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVB;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXTOVB;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVB) !=
					ADE9078_ST1_ZXTOVB)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIB) == ADE9078_ST1_ZXIB){
				handled_irq1 |= ADE9078_ST1_ZXIB;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIB;
				*val = 1;
			}

		}
		break;
	case ADE9078_PHASE_C_NR:
		if(chan->type == IIO_VOLTAGE){
			if((status1 & ADE9078_ST1_ZXVC) == ADE9078_ST1_ZXVC){
				*val = 1;
				handled_irq1 |= ADE9078_ST1_ZXVC;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXVC;
			}
			else if((status1 & ADE9078_ST1_ZXTOVC) == ADE9078_ST1_ZXTOVC){
				*val = -1;
				handled_irq1 |= ADE9078_ST1_ZXTOVC;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXTOVC;
			}
			if((ade9078_dev->irq1_bits & ADE9078_ST1_ZXTOVC) !=
					ADE9078_ST1_ZXTOVC)
				*val = 0;
		}
		else if(chan->type == IIO_CURRENT){
			if((status1 & ADE9078_ST1_ZXIC) == ADE9078_ST1_ZXIC){
				handled_irq1 |= ADE9078_ST1_ZXIC;
				ade9078_dev->irq1_status &= ~ADE9078_ST1_ZXIC;
				*val = 1;
			}

		}
		break;
	default:
		break;
	}

	regmap_write(ade9078_dev->regmap, ADDR_STATUS1, handled_irq1);
	dev_dbg(&ade9078_dev->spi->dev, "Read event handled_irq1 0x%x", handled_irq1);
	return IIO_VAL_INT;
}

/*
 * ade9078_config_wfb() - reads the ade9078 node and configures the wave form
 * buffer based on the options set. Additionally is reads the active scan mask
 * in order to set the input data of the buffer. There are only a few available
 * input configurations permitted by the IC, any unpermitted configuration will
 * result in all channels being active.
 * @indio_dev:		the IIO device
 */
static int ade9078_config_wfb(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u32 wfg_cfg_val = 0;
	u32 tmp;
	int ret;

	bitmap_to_arr32(&wfg_cfg_val, indio_dev->active_scan_mask,
			indio_dev->masklength);

	switch(wfg_cfg_val)
	{
	case ADE9078_SCAN_POS_IA | ADE9078_SCAN_POS_VA :
		wfg_cfg_val = 0x1;
		break;
	case ADE9078_SCAN_POS_IB | ADE9078_SCAN_POS_VB :
		wfg_cfg_val = 0x2;
		break;
	case ADE9078_SCAN_POS_IC | ADE9078_SCAN_POS_VC :
		wfg_cfg_val = 0x3;
		break;
	case ADE9078_SCAN_POS_IA :
		wfg_cfg_val = 0x8;
		break;
	case ADE9078_SCAN_POS_VA :
		wfg_cfg_val = 0x9;
		break;
	case ADE9078_SCAN_POS_IB :
		wfg_cfg_val = 0xA;
		break;
	case ADE9078_SCAN_POS_VB :
		wfg_cfg_val = 0xB;
		break;
	case ADE9078_SCAN_POS_IC :
		wfg_cfg_val = 0xC;
		break;
	case ADE9078_SCAN_POS_VC :
		wfg_cfg_val = 0xD;
		break;
	default:
		wfg_cfg_val = 0x0;
		break;
	}

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-cap-sel", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-cap-sel: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 5;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-mode", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-mode: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 6;
	ade9078_dev->wf_mode = tmp;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-src", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-src: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 8;

	ret = of_property_read_u32((&ade9078_dev->spi->dev)->of_node,
			"adi,wf-in-en", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wf-in-en: %d\n", ret);
		return ret;
	}
	wfg_cfg_val |= tmp << 12;

	return regmap_write(ade9078_dev->regmap, ADDR_WFB_CFG, wfg_cfg_val);
}

/*
 * ade9078_wfb_interrupt_setup() - Configures the wave form buffer interrupt
 * according to modes
 * @ade9078_dev:		ade9078 device data
 * @mode:				modes according to datasheet; values [0-2]
 *
 * This sets the interrupt register and other registers related to the
 * interrupts according to mode [0-2] from the datasheet
 */
static int ade9078_wfb_interrupt_setup(struct ade9078_device *ade9078_dev,
		u8 mode)
{
	int ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG, 0x0);
	if(ret)
		return ret;

	if(mode == 1 || mode == 0)
	{
		ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_PG_IRQEN, 0x8000);
		if(ret)
			return ret;
	}
	else if(mode == 2)
	{
		ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_PG_IRQEN, 0x80);
		if(ret)
			return ret;

	}

	ade9078_dev->irq0_bits |= ADE9078_ST0_PAGE_FULL;

	return ret;
}

/*
 * ade9078_buffer_preenable() - configures the wave form buffer, sets the
 * interrupts and enables the buffer
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	ret = ade9078_config_wfb(indio_dev);
	if (ret)
		return ret;

	switch (ade9078_dev->wf_mode)
	{
	case 0:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 0);
		break;
	case 1:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 1);
		break;
	case 2:
		ret = ade9078_wfb_interrupt_setup(ade9078_dev, 2);
		break;
	default:
		break;
	}

	ret = ade9078_update_mask0(ade9078_dev);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-enable update mask0 fail");
		return ret;
	}
	ret = ade9078_en_wfb(ade9078_dev, true);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-enable wfb enable fail");
		return ret;
	}

	return ret;
}

/*
 * ade9078_buffer_postenable() - after the IIO is enabled we wait for the
 * wave form buffer flag to be enabled and push the output to the IIO buffer
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	u8 count = 0;
	unsigned long flags;

	while(ade9078_dev->triggered == false)
	{
		msleep_interruptible(1);
		count++;
		if(count > 9)
		{
			dev_err(&ade9078_dev->spi->dev, "Post-enable trigger timeout");
			return -ESRCH;
		}
	}

	spin_lock_irqsave(&ade9078_dev->slock, flags);
	iio_trigger_poll(ade9078_dev->trig);
	spin_unlock_irqrestore(&ade9078_dev->slock, flags);
	return 0;
}

/*
 * ade9078_buffer_postdisable() - after the iio is disable
 * this will disable the ade9078 internal buffer for acquisition
 * @indio_dev:		the IIO device
 */
static int ade9078_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ade9078_device *ade9078_dev = iio_priv(indio_dev);
	int ret;

	ret = ade9078_en_wfb(ade9078_dev, false);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-disable wfb disable fail");
		return ret;
	}

	regmap_write(ade9078_dev->regmap, ADDR_WFB_TRG_CFG,
			0x0);

	ade9078_dev->irq0_bits &= ~ADE9078_ST0_WFB_TRIG_IRQ &
							  ~ADE9078_ST0_PAGE_FULL;
	ret = ade9078_update_mask0(ade9078_dev);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Post-disable update maks0 fail");
		return ret;
	}

	ade9078_dev->triggered = true;

	return ret;
}

/*
 * ade9078_phase_gain_offset_setup() - reads the gain and offset for
 * I, V and P from the device-tree for each phase and sets them in the
 * respective registers
 * @ade9078_dev:		ade9078 device data
 * @phase_node: 		phase node in the device-tree
 * @phase_nr:			the number attributed to each phase, this also
 * 						represents the phase register offset
 */
static int ade9078_phase_gain_offset_setup(struct ade9078_device *ade9078_dev,
		struct device_node *phase_node, u32 phase_nr)
{
	int ret;
	u32 tmp;

	ret = of_property_read_u32(phase_node, "adi,igain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get igain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AIGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,vgain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get vgain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,irmsos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get irmsos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AIRMSOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,vrmsos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get vrmsos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVRMSOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,pgain", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get pgain: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_APGAIN,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,wattos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get wattos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AWATTOS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,varos", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get varos: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AVAROS,phase_nr), tmp);
	if(ret)
		return ret;

	ret = of_property_read_u32(phase_node, "adi,fvaros", &tmp);
	if (ret) {
		dev_err(&ade9078_dev->spi->dev, "Failed to get fvaros: %d\n", ret);
		tmp = 0;
	}
	ret = regmap_write(ade9078_dev->regmap,
			PHASE_ADDR_ADJUST(ADDR_AFVAROS,phase_nr), tmp);
	if(ret)
		return ret;

	return 0;
}

/*
 * ade9078_setup_iio_channels() - parses the phase nodes of the device-tree and
 * creates the iio channels based on the active phases in the DT. Each phase
 * has its own I, V and P channels with individual gain and offset parameters.
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_setup_iio_channels(struct ade9078_device *ade9078_dev)
{
	struct iio_chan_spec *chan;
	struct device_node *phase_node = NULL;
	u32 phase_nr;
	u32 chan_size = 0;
	int ret = 0;

	chan = devm_kcalloc(&ade9078_dev->spi->dev,
			ADE9078_MAX_PHASE_NR*ARRAY_SIZE(ade9078_a_channels),
			sizeof(*ade9078_a_channels), GFP_KERNEL);
	if(chan == NULL) {
		dev_err(&ade9078_dev->spi->dev,"Unable to allocate ADE9078 channels");
		return -ENOMEM;
	}
	ade9078_dev->indio_dev->num_channels = 0;
	ade9078_dev->indio_dev->channels = chan;

	for_each_available_child_of_node((&ade9078_dev->spi->dev)->of_node, phase_node){
		if (!of_node_name_eq(phase_node, "phase"))
			continue;

		ret = of_property_read_u32(phase_node, "reg", &phase_nr);
		if (ret) {
			dev_err(&ade9078_dev->spi->dev, "Could not read channel reg : %d\n",
					ret);
			goto put_phase_node;
		}

		switch(phase_nr)
		{
		case ADE9078_PHASE_A_NR:
			memcpy(chan, ade9078_a_channels, sizeof(ade9078_a_channels));
			chan_size = ARRAY_SIZE(ade9078_a_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_A_NR);
			break;
		case ADE9078_PHASE_B_NR:
			memcpy(chan, ade9078_b_channels, sizeof(ade9078_b_channels));
			chan_size = ARRAY_SIZE(ade9078_b_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_B_NR);
			break;
		case ADE9078_PHASE_C_NR:
			memcpy(chan, ade9078_c_channels, sizeof(ade9078_c_channels));
			chan_size = ARRAY_SIZE(ade9078_c_channels);
			ade9078_phase_gain_offset_setup(ade9078_dev, phase_node,
					ADE9078_PHASE_C_NR);
			break;
		default:
			break;
		}

		chan += chan_size;
		ade9078_dev->indio_dev->num_channels += chan_size;
	}

put_phase_node:
	of_node_put(phase_node);
	return ret;
}

/*
 * ade9078_reset() - Reset sequence for the ADE9078
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_reset(struct ade9078_device *ade9078_dev)
{
	ade9078_dev->rst_done = false;

	gpiod_set_value_cansleep(ade9078_dev->gpio_reset, 1);
	usleep_range(1, 100);
	gpiod_set_value_cansleep(ade9078_dev->gpio_reset, 0);
	msleep_interruptible(50);

	if(ade9078_dev->rst_done == false)
		return -EPERM;
	else
		return 0;
}

/*
 * ade9078_setup() - initial register setup of the ade9078
 * @ade9078_dev:		ade9078 device data
 */
static int ade9078_setup(struct ade9078_device *ade9078_dev)
{
	int ret = 0;

	dev_dbg(&ade9078_dev->spi->dev, "Setup started");
	ret = regmap_write(ade9078_dev->regmap, ADDR_PGA_GAIN, ADE9078_PGA_GAIN);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG0, ADE9078_CONFIG0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG1, ADE9078_CONFIG1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG2, ADE9078_CONFIG2);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_CONFIG3, ADE9078_CONFIG3);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ACCMODE, ADE9078_ACCMODE);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_ZX_LP_SEL, ADE9078_ZX_LP_SEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK0, ADE9078_MASK0);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_MASK1, ADE9078_MASK1);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EVENT_MASK, ADE9078_EVENT_MASK);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_WFB_CFG, ADE9078_WFB_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_VLEVEL, ADE9078_VLEVEL);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_DICOEFF, ADE9078_DICOEFF);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EGY_TIME, ADE9078_EGY_TIME);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_EP_CFG, ADE9078_EP_CFG);
	if(ret)
		return ret;
	ret = regmap_write(ade9078_dev->regmap, ADDR_RUN, ADE9078_RUN_ON);
	if(ret)
		return ret;

	msleep_interruptible(2);

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS0,
				0xFFFFFFFF);
	if (ret)
		return ret;

	ret = regmap_write(ade9078_dev->regmap, ADDR_STATUS1,
				0xFFFFFFFF);
	if (ret)
		return ret;

	dev_dbg(&ade9078_dev->spi->dev, "Setup finished");

	return ret;
}


static const struct iio_trigger_ops ade9078_trigger_ops = {
		.validate_device = iio_trigger_validate_own_device,
};

static const struct iio_buffer_setup_ops ade9078_buffer_ops = {
	.preenable = &ade9078_buffer_preenable,
	.postenable = &ade9078_buffer_postenable,
	.postdisable = &ade9078_buffer_postdisable,
};

static const struct iio_info ade9078_info = {
	.read_raw = &ade9078_read_raw,
	.debugfs_reg_access = &ade9078_reg_acess,
	.write_event_config = &ade9078_write_event_config,
	.read_event_value = &ade9078_read_event_vlaue,
};

/*
 * Regmap configuration
 * The register access of the ade9078 requires a 16 bit address
 * with the read flag on bit 3. This is not supported by default
 * regmap functionality, thus reg_read and reg_write have been
 * replaced with custom functions
 */
static const struct regmap_config ade9078_regmap_config = {
	.reg_bits = 16,
	.val_bits = 32,
	//reg_read and write require the use of devm_regmap_init
	//instead of devm_regmap_init_spi
	.reg_read = ade9078_spi_read_reg,
	.reg_write = ade9078_spi_write_reg,
	.zero_flag_mask = true,
};

static int ade9078_probe(struct spi_device *spi)
{
	struct ade9078_device *ade9078_dev;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct gpio_desc *gpio_reset;
	int irq;

	unsigned long irqflags = 0;
	int ret = 0;

	dev_info(&spi->dev,"Enter ade9078_probe\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ade9078_dev));
	if(indio_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 IIO");
		return -ENOMEM;
	}
	ade9078_dev = iio_priv(indio_dev);
	if(ade9078_dev == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 device structure");
		return -ENOMEM;
	}

	ade9078_dev->rx = devm_kcalloc(&spi->dev, 6, sizeof(*ade9078_dev->rx),
			GFP_KERNEL);
	if(ade9078_dev->rx == NULL)	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 RX Buffer");
		return-ENOMEM;
	}
	ade9078_dev->tx = devm_kcalloc(&spi->dev, 10, sizeof(*ade9078_dev->tx),
			GFP_KERNEL);
	if(ade9078_dev->tx == NULL) {
		dev_err(&spi->dev,"Unable to allocate ADE9078 TX Buffer");
		return -ENOMEM;
	}
	regmap = devm_regmap_init(&spi->dev, NULL, spi, &ade9078_regmap_config);
	if (IS_ERR(regmap))	{
		dev_err(&spi->dev,"Unable to allocate ADE9078 regmap");
		return PTR_ERR(regmap);
	}
	spi_set_drvdata(spi, ade9078_dev);

	ade9078_dev->rst_done = false;

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq0");
	if (irq < 0){
		dev_err(&spi->dev,"Unable to find irq0");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, NULL, ade9078_irq0_thread,
			irqflags | IRQF_ONESHOT, KBUILD_MODNAME, ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	irq = of_irq_get_byname((&spi->dev)->of_node, "irq1");
	if (irq < 0){
		dev_err(&spi->dev,"Unable to find irq1");
		return -EINVAL;
	}
	irqflags = irq_get_trigger_type(irq);
	ret = devm_request_threaded_irq(&spi->dev, irq, NULL, ade9078_irq1_thread,
			irqflags | IRQF_ONESHOT, KBUILD_MODNAME, ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}
	ade9078_dev->irq1_status = 0;

	trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d", KBUILD_MODNAME,
			indio_dev->id);
	if (!trig){
		dev_err(&spi->dev,"Unable to allocate ADE9078 trigger");
		return -ENOMEM;
	}
	iio_trigger_set_drvdata(trig, ade9078_dev);

	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (!gpio_reset){
		dev_err(&spi->dev,"Unable to allocate ADE9078 reset");
		return -ENOMEM;
	}

	ade9078_dev->triggered = false;
	ade9078_dev->spi = spi;
	ade9078_dev->spi->mode = SPI_MODE_0;
	spi_setup(ade9078_dev->spi);

	indio_dev->name = KBUILD_MODNAME;
	indio_dev->dev.parent = &ade9078_dev->spi->dev;
	indio_dev->info = &ade9078_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ade9078_dev->regmap = regmap;
	ade9078_dev->indio_dev = indio_dev;
	ade9078_setup_iio_channels(ade9078_dev);

	ade9078_dev->trig = trig;
	ade9078_dev->trig->dev.parent = &ade9078_dev->spi->dev;
	ade9078_dev->trig->ops = &ade9078_trigger_ops;

	ade9078_dev->gpio_reset = gpio_reset;
	ade9078_dev->wfb_trg_cfg = 0;

	mutex_init(&ade9078_dev->lock);

	ade9078_configure_scan(indio_dev);

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
			NULL, &ade9078_trigger_handler,	&ade9078_buffer_ops);
	if (ret) {
		dev_err(&spi->dev, "Failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	ret = iio_trigger_register(trig);
	if (ret)
	{
		dev_err(&spi->dev,"Unable to register ADE9078 trigger");
		return ret;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to register IIO device");
		return ret;
	}

	ret = ade9078_reset(ade9078_dev);
	if(ret){
		dev_err(&spi->dev, "ADE9078 reset failed");
		return ret;
	}
	dev_dbg(&spi->dev, "Reset done");

	ret = ade9078_setup(ade9078_dev);
	if (ret) {
		dev_err(&spi->dev, "Unable to setup ADE9078");
		return ret;
	}

	return ret;
};

static int ade9078_remove(struct spi_device *spi)
{
	struct ade9078_device *ade9078_dev = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = ade9078_dev->indio_dev;

	dev_info(&spi->dev,"Exit ade9078_probe\n");
	ade9078_dev->trig->dev.parent = NULL;
	iio_trigger_unregister(ade9078_dev->trig);
	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ade9078_id[] = {
		{"ade9078", 0},
		{}
};
static struct spi_driver ade9078_driver = {
		.driver = {
				.name = "ade9078",
		},
		.probe = ade9078_probe,
		.remove = ade9078_remove,
		.id_table = ade9078_id,
};

module_spi_driver(ade9078_driver);

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADE9078 High Performance, Polyphase Energy Metering IC Driver");
MODULE_LICENSE("GPL v2");
