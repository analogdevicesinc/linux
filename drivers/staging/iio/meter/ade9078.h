// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADE9078 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

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

/*HPF disabled*/
#define ADE9078_CONFIG0 			0x00000008

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
