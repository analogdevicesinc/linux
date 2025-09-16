/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Analog Devices ADSP family SRU control driver device tree bindings
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

//DAI0 Destinations
#define SPT0_ACLK_I                 0     //GROUP_A
#define SPT0_BCLK_I                 1     //GROUP_A
#define SPT1_ACLK_I                 2     //GROUP_A
#define SPT1_BCLK_I                 3     //GROUP_A
#define SPT2_ACLK_I                 4     //GROUP_A
#define SPT2_BCLK_I                 5     //GROUP_A
#define SRC0_CLK_IP_I               6     //GROUP_A
#define SRC0_CLK_OP_I               7     //GROUP_A
#define SRC1_CLK_IP_I               8     //GROUP_A
#define SRC1_CLK_OP_I               9     //GROUP_A
#define SRC2_CLK_IP_I              10     //GROUP_A
#define SRC2_CLK_OP_I              11     //GROUP_A
#define SRC3_CLK_IP_I              12     //GROUP_A
#define SRC3_CLK_OP_I              13     //GROUP_A
#define SPDIF0_TX_CLK_I            14     //GROUP_A
#define PDM0_CLK0_I                15     //GROUP_A
#define PDM0_BCLK_I                16     //GROUP_A
#define SPDIF0_TX_HFCLK_I          17     //GROUP_A
#define PCG0_EXTCLKA_I             18     //GROUP_A
#define PCG0_EXTCLKB_I             19     //GROUP_A
#define SPDIF0_TX_EXT_SYNC_I       20     //GROUP_A
#define PCG0_SYNC_CLKA_I           21     //GROUP_A
#define PCG0_SYNC_CLKB_I           22     //GROUP_A
#define SPT3_ACLK_I                23     //GROUP_A
#define SPT3_BCLK_I                24     //GROUP_A
#define PCG0_SYNC_CLKE_I           25     //GROUP_A
#define PCG0_SYNC_CLKF_I           26     //GROUP_A
#define PCG0_EXTCLKE_I             27     //GROUP_A
#define PCG0_EXTCLKF_I             28     //GROUP_A
#define SPT0_AD0_I                 29     //GROUP_B
#define SPT0_AD1_I                 30     //GROUP_B
#define SPT0_BD0_I                 31     //GROUP_B
#define SPT0_BD1_I                 32     //GROUP_B
#define SPT1_AD0_I                 33     //GROUP_B
#define SPT1_AD1_I                 34     //GROUP_B
#define SPT1_BD0_I                 35     //GROUP_B
#define SPT1_BD1_I                 36     //GROUP_B
#define SPT2_AD0_I                 37     //GROUP_B
#define SPT2_AD1_I                 38     //GROUP_B
#define SPT2_BD0_I                 39     //GROUP_B
#define SPT2_BD1_I                 40     //GROUP_B
#define SRC0_DAT_IP_I              41     //GROUP_B
#define SRC1_DAT_IP_I              42     //GROUP_B
#define SRC2_DAT_IP_I              43     //GROUP_B
#define SRC3_DAT_IP_I              44     //GROUP_B
#define SRC0_TDM_OP_I              45     //GROUP_B
#define SRC1_TDM_OP_I              46     //GROUP_B
#define SRC2_TDM_OP_I              47     //GROUP_B
#define SRC3_TDM_OP_I              48     //GROUP_B
#define SPDIF0_TX_DAT_I            49     //GROUP_B
#define PDM0_DAT0_I                50     //GROUP_B
#define PDM0_DAT1_I                51     //GROUP_B
#define SPDIF0_RX_I                52     //GROUP_B
#define SPT3_AD0_I                 53     //GROUP_B
#define SPT3_AD1_I                 54     //GROUP_B
#define SPT3_BD0_I                 55     //GROUP_B
#define SPT3_BD1_I                 56     //GROUP_B
#define SPT0_AFS_I                 57     //GROUP_C
#define SPT0_BFS_I                 58     //GROUP_C
#define SPT1_AFS_I                 59     //GROUP_C
#define SPT1_BFS_I                 60     //GROUP_C
#define SPT2_AFS_I                 61     //GROUP_C
#define SPT2_BFS_I                 62     //GROUP_C
#define SRC0_FS_IP_I               63     //GROUP_C
#define SRC0_FS_OP_I               64     //GROUP_C
#define SRC1_FS_IP_I               65     //GROUP_C
#define SRC1_FS_OP_I               66     //GROUP_C
#define SRC2_FS_IP_I               67     //GROUP_C
#define SRC2_FS_OP_I               68     //GROUP_C
#define SRC3_FS_IP_I               69     //GROUP_C
#define SRC3_FS_OP_I               70     //GROUP_C
#define SPDIF0_TX_FS_I             71     //GROUP_C
#define SPT3_AFS_I                 72     //GROUP_C
#define SPT3_BFS_I                 73     //GROUP_C
#define TM0_ACI14_I                74     //GROUP_C
#define PDM0_LRCLK_I               75     //GROUP_C
#define DAI0_PB01_I                76     //GROUP_D
#define DAI0_PB02_I                77     //GROUP_D
#define DAI0_PB03_I                78     //GROUP_D
#define DAI0_PB04_I                79     //GROUP_D
#define DAI0_PB05_I                80     //GROUP_D
#define DAI0_PB06_I                81     //GROUP_D
#define DAI0_PB07_I                82     //GROUP_D
#define DAI0_PB08_I                83     //GROUP_D
#define DAI0_PB09_I                84     //GROUP_D
#define DAI0_PB10_I                85     //GROUP_D
#define DAI0_PB11_I                86     //GROUP_D
#define DAI0_PB12_I                87     //GROUP_D
#define DAI0_PB13_I                88     //GROUP_D
#define DAI0_PB14_I                89     //GROUP_D
#define DAI0_PB15_I                90     //GROUP_D
#define DAI0_PB16_I                91     //GROUP_D
#define DAI0_PB17_I                92     //GROUP_D
#define DAI0_PB18_I                93     //GROUP_D
#define DAI0_PB19_I                94     //GROUP_D
#define DAI0_PB20_I                95     //GROUP_D
#define INV_DAI0_PB19_I            96     //GROUP_D
#define INV_DAI0_PB20_I            97     //GROUP_D
#define DAI0_MISCA0_I              98     //GROUP_E
#define DAI0_INT_6_I               99     //GROUP_E
#define DAI0_MISCA1_I             100     //GROUP_E
#define DAI0_INT_7_I              101     //GROUP_E
#define DAI0_MISCA2_I             102     //GROUP_E
#define DAI0_INT_8_I              103     //GROUP_E
#define DAI0_MISCA3_I             104     //GROUP_E
#define DAI0_INT_9_I              105     //GROUP_E
#define DAI0_MISCA4_I             106     //GROUP_E
#define DAI0_MISCA5_I             107     //GROUP_E
#define DAI0_INV_MISCA4_I         108     //GROUP_E
#define DAI0_INV_MISCA5_I         109     //GROUP_E
#define DAI0_INT_0_I              110     //GROUP_E
#define DAI0_INT_1_I              111     //GROUP_E
#define DAI0_INT_2_I              112     //GROUP_E
#define DAI0_INT_3_I              113     //GROUP_E
#define DAI0_INT_4_I              114     //GROUP_E
#define DAI0_INT_5_I              115     //GROUP_E
#define PCG_HWA_TRIG_I            116     //GROUP_E
#define PCG_HWB_TRIG_I            117     //GROUP_E
#define PCG_HWE_TRIG_I            118     //GROUP_E
#define PCG_HWF_TRIG_I            119     //GROUP_E
#define DAI0_PBEN01_I             120     //GROUP_F
#define DAI0_PBEN02_I             121     //GROUP_F
#define DAI0_PBEN03_I             122     //GROUP_F
#define DAI0_PBEN04_I             123     //GROUP_F
#define DAI0_PBEN05_I             124     //GROUP_F
#define DAI0_PBEN06_I             125     //GROUP_F
#define DAI0_PBEN07_I             126     //GROUP_F
#define DAI0_PBEN08_I             127     //GROUP_F
#define DAI0_PBEN09_I             128     //GROUP_F
#define DAI0_PBEN10_I             129     //GROUP_F
#define DAI0_PBEN11_I             130     //GROUP_F
#define DAI0_PBEN12_I             131     //GROUP_F
#define DAI0_PBEN13_I             132     //GROUP_F
#define DAI0_PBEN14_I             133     //GROUP_F
#define DAI0_PBEN15_I             134     //GROUP_F
#define DAI0_PBEN16_I             135     //GROUP_F
#define DAI0_PBEN17_I             136     //GROUP_F
#define DAI0_PBEN18_I             137     //GROUP_F
#define DAI0_PBEN19_I             138     //GROUP_F
#define DAI0_PBEN20_I             139     //GROUP_F

//DAI1 Destinations
#define SPT4_ACLK_I               140     //GROUP_A
#define SPT4_BCLK_I               141     //GROUP_A
#define SPT5_ACLK_I               142     //GROUP_A
#define SPT5_BCLK_I               143     //GROUP_A
#define SPT6_ACLK_I               144     //GROUP_A
#define SPT6_BCLK_I               145     //GROUP_A
#define SRC4_CLK_IP_I             146     //GROUP_A
#define SRC4_CLK_OP_I             147     //GROUP_A
#define SRC5_CLK_IP_I             148     //GROUP_A
#define SRC5_CLK_OP_I             149     //GROUP_A
#define SRC6_CLK_IP_I             150     //GROUP_A
#define SRC6_CLK_OP_I             151     //GROUP_A
#define SRC7_CLK_IP_I             152     //GROUP_A
#define SRC7_CLK_OP_I             153     //GROUP_A
#define SPDIF1_TX_CLK_I           154     //GROUP_A
#define PDM1_CLK0_I               155     //GROUP_A
#define PDM1_BCLK_I               156     //GROUP_A
#define SPDIF1_TX_HFCLK_I         157     //GROUP_A
#define PCG0_EXTCLKC_I            158     //GROUP_A
#define PCG0_EXTCLKD_I            159     //GROUP_A
#define SPDIF1_TX_EXT_SYNC_I      160     //GROUP_A
#define PCG0_SYNC_CLKC_I          161     //GROUP_A
#define PCG0_SYNC_CLKD_I          162     //GROUP_A
#define SPT7_ACLK_I               163     //GROUP_A
#define SPT7_BCLK_I               164     //GROUP_A
#define PCG0_SYNC_CLKG_I          165     //GROUP_A
#define PCG0_SYNC_CLKH_I          166     //GROUP_A
#define PCG0_EXTCLKG_I            167     //GROUP_A
#define PCG0_EXTCLKH_I            168     //GROUP_A
#define SPT4_AD0_I                169     //GROUP_B
#define SPT4_AD1_I                170     //GROUP_B
#define SPT4_BD0_I                171     //GROUP_B
#define SPT4_BD1_I                172     //GROUP_B
#define SPT5_AD0_I                173     //GROUP_B
#define SPT5_AD1_I                174     //GROUP_B
#define SPT5_BD0_I                175     //GROUP_B
#define SPT5_BD1_I                176     //GROUP_B
#define SPT6_AD0_I                177     //GROUP_B
#define SPT6_AD1_I                178     //GROUP_B
#define SPT6_BD0_I                179     //GROUP_B
#define SPT6_BD1_I                180     //GROUP_B
#define SRC4_DAT_IP_I             181     //GROUP_B
#define SRC5_DAT_IP_I             182     //GROUP_B
#define SRC6_DAT_IP_I             183     //GROUP_B
#define SRC7_DAT_IP_I             184     //GROUP_B
#define SRC4_TDM_OP_I             185     //GROUP_B
#define SRC5_TDM_OP_I             186     //GROUP_B
#define SRC6_TDM_OP_I             187     //GROUP_B
#define SRC7_TDM_OP_I             188     //GROUP_B
#define SPDIF1_TX_DAT_I           189     //GROUP_B
#define PDM1_DAT0_I               190     //GROUP_B
#define PDM1_DAT1_I               191     //GROUP_B
#define SPDIF1_RX_I               192     //GROUP_B
#define SPT7_AD0_I                193     //GROUP_B
#define SPT7_AD1_I                194     //GROUP_B
#define SPT7_BD0_I                195     //GROUP_B
#define SPT7_BD1_I                196     //GROUP_B
#define SPT4_AFS_I                197     //GROUP_C
#define SPT4_BFS_I                198     //GROUP_C
#define SPT5_AFS_I                199     //GROUP_C
#define SPT5_BFS_I                200     //GROUP_C
#define SPT6_AFS_I                201     //GROUP_C
#define SPT6_BFS_I                202     //GROUP_C
#define SRC4_FS_IP_I              203     //GROUP_C
#define SRC4_FS_OP_I              204     //GROUP_C
#define SRC5_FS_IP_I              205     //GROUP_C
#define SRC5_FS_OP_I              206     //GROUP_C
#define SRC6_FS_IP_I              207     //GROUP_C
#define SRC6_FS_OP_I              208     //GROUP_C
#define SRC7_FS_IP_I              209     //GROUP_C
#define SRC7_FS_OP_I              210     //GROUP_C
#define SPDIF1_TX_FS_I            211     //GROUP_C
#define SPT7_AFS_I                212     //GROUP_C
#define SPT7_BFS_I                213     //GROUP_C
#define TM0_ACI15_I               214     //GROUP_C
#define PDM1_LRCLK_I              215     //GROUP_C
#define DAI1_PB01_I               216     //GROUP_D
#define DAI1_PB02_I               217     //GROUP_D
#define DAI1_PB03_I               218     //GROUP_D
#define DAI1_PB04_I               219     //GROUP_D
#define DAI1_PB05_I               220     //GROUP_D
#define DAI1_PB06_I               221     //GROUP_D
#define DAI1_PB07_I               222     //GROUP_D
#define DAI1_PB08_I               223     //GROUP_D
#define DAI1_PB09_I               224     //GROUP_D
#define DAI1_PB10_I               225     //GROUP_D
#define DAI1_PB11_I               226     //GROUP_D
#define DAI1_PB12_I               227     //GROUP_D
#define DAI1_PB13_I               228     //GROUP_D
#define DAI1_PB14_I               229     //GROUP_D
#define DAI1_PB15_I               230     //GROUP_D
#define DAI1_PB16_I               231     //GROUP_D
#define DAI1_PB17_I               232     //GROUP_D
#define DAI1_PB18_I               233     //GROUP_D
#define DAI1_PB19_I               234     //GROUP_D
#define DAI1_PB20_I               235     //GROUP_D
#define INV_DAI1_PB19_I           236     //GROUP_D
#define INV_DAI1_PB20_I           237     //GROUP_D
#define DAI1_MISCA0_I             238     //GROUP_E
#define DAI1_INT_6_I              239     //GROUP_E
#define DAI1_MISCA1_I             240     //GROUP_E
#define DAI1_INT_7_I              241     //GROUP_E
#define DAI1_MISCA2_I             242     //GROUP_E
#define DAI1_INT_8_I              243     //GROUP_E
#define DAI1_MISCA3_I             244     //GROUP_E
#define DAI1_INT_9_I              245     //GROUP_E
#define DAI1_MISCA4_I             246     //GROUP_E
#define DAI1_MISCA5_I             247     //GROUP_E
#define DAI1_INV_MISCA4_I         248     //GROUP_E
#define DAI1_INV_MISCA5_I         249     //GROUP_E
#define DAI1_INT_0_I              250     //GROUP_E
#define DAI1_INT_1_I              251     //GROUP_E
#define DAI1_INT_2_I              252     //GROUP_E
#define DAI1_INT_3_I              253     //GROUP_E
#define DAI1_INT_4_I              254     //GROUP_E
#define DAI1_INT_5_I              255     //GROUP_E
#define PCG_HWC_TRIG_I            256     //GROUP_E
#define PCG_HWD_TRIG_I            257     //GROUP_E
#define PCG_HWG_TRIG_I            258     //GROUP_E
#define PCG_HWH_TRIG_I            259     //GROUP_E
#define DAI1_PBEN01_I             260     //GROUP_F
#define DAI1_PBEN02_I             261     //GROUP_F
#define DAI1_PBEN03_I             262     //GROUP_F
#define DAI1_PBEN04_I             263     //GROUP_F
#define DAI1_PBEN05_I             264     //GROUP_F
#define DAI1_PBEN06_I             265     //GROUP_F
#define DAI1_PBEN07_I             266     //GROUP_F
#define DAI1_PBEN08_I             267     //GROUP_F
#define DAI1_PBEN09_I             268     //GROUP_F
#define DAI1_PBEN10_I             269     //GROUP_F
#define DAI1_PBEN11_I             270     //GROUP_F
#define DAI1_PBEN12_I             271     //GROUP_F
#define DAI1_PBEN13_I             272     //GROUP_F
#define DAI1_PBEN14_I             273     //GROUP_F
#define DAI1_PBEN15_I             274     //GROUP_F
#define DAI1_PBEN16_I             275     //GROUP_F
#define DAI1_PBEN17_I             276     //GROUP_F
#define DAI1_PBEN18_I             277     //GROUP_F
#define DAI1_PBEN19_I             278     //GROUP_F
#define DAI1_PBEN20_I             279     //GROUP_F

//DAI0 Sources
#define DAI0_PB01_O_ABCDE         280     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB02_O_ABCDE         281     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB03_O_ABCDE         282     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB04_O_ABCDE         283     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB05_O_ABCDE         284     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB06_O_ABCDE         285     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB07_O_ABCDE         286     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB08_O_ABCDE         287     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB09_O_ABCDE         288     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB10_O_ABCDE         289     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB11_O_ABCDE         290     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB12_O_ABCDE         291     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB13_O_ABCDE         292     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB14_O_ABCDE         293     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB15_O_ABCDE         294     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB16_O_ABCDE         295     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB17_O_ABCDE         296     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB18_O_ABCDE         297     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB19_O_ABCDE         298     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_PB20_O_ABCDE         299     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI0_LOW_ACE              300     //GROUP_A GROUP_C GROUP_E
#define DAI0_HIGH_ACE             301     //GROUP_A GROUP_C GROUP_E
#define SPT0_AD0_O_BD             302     //GROUP_B GROUP_D
#define SPT0_AD1_O_BD             303     //GROUP_B GROUP_D
#define SPT0_BD0_O_BD             304     //GROUP_B GROUP_D
#define SPT0_BD1_O_BD             305     //GROUP_B GROUP_D
#define SPT1_AD0_O_BD             306     //GROUP_B GROUP_D
#define SPT1_AD1_O_BD             307     //GROUP_B GROUP_D
#define SPT1_BD0_O_BD             308     //GROUP_B GROUP_D
#define SPT1_BD1_O_BD             309     //GROUP_B GROUP_D
#define SPT2_AD0_O_BD             310     //GROUP_B GROUP_D
#define SPT2_AD1_O_BD             311     //GROUP_B GROUP_D
#define SPT2_BD0_O_BD             312     //GROUP_B GROUP_D
#define SPT2_BD1_O_BD             313     //GROUP_B GROUP_D
#define DAI0_CRS_PB03_O_A         314     //GROUP_A
#define SPT0_ACLK_O_A             315     //GROUP_A
#define SPT0_BCLK_O_A             316     //GROUP_A
#define SPT1_ACLK_O_A             317     //GROUP_A
#define SPT1_BCLK_O_A             318     //GROUP_A
#define SPT2_ACLK_O_A             319     //GROUP_A
#define SPT2_BCLK_O_A             320     //GROUP_A
#define SPDIF0_RX_CLK_O_A         321     //GROUP_A
#define SPDIF0_RX_TDMCLK_O_A      322     //GROUP_A
#define PCG0_CLKA_O_A             323     //GROUP_A
#define PCG0_CLKB_O_A             324     //GROUP_A
#define SRC0_DAT_OP_O_B           325     //GROUP_B
#define SRC1_DAT_OP_O_B           326     //GROUP_B
#define SRC2_DAT_OP_O_B           327     //GROUP_B
#define SRC3_DAT_OP_O_B           328     //GROUP_B
#define SRC0_TDM_IP_O_B           329     //GROUP_B
#define SRC1_TDM_IP_O_B           330     //GROUP_B
#define SRC2_TDM_IP_O_B           331     //GROUP_B
#define SRC3_TDM_IP_O_B           332     //GROUP_B
#define SPDIF0_RX_DAT_O_B         333     //GROUP_B
#define SPT3_AD0_O_B              334     //GROUP_B
#define SPT3_AD1_O_B              335     //GROUP_B
#define SPT3_BD0_O_B              336     //GROUP_B
#define SPT3_BD1_O_B              337     //GROUP_B
#define SPDIF0_TX_O_B             338     //GROUP_B
#define SRC7_CRS_DAT_OP_O_B       339     //GROUP_B
#define SRC7_CRS_TDM_IP_O_B       340     //GROUP_B
#define PDM0_SDATA_O_B            341     //GROUP_B
#define DAI0_LOW_B                342     //GROUP_B
#define DAI0_HIGH_B               343     //GROUP_B
#define DAI0_CRS_PB04_O_C         344     //GROUP_C
#define SPT0_AFS_O_C              345     //GROUP_C
#define SPT0_BFS_O_C              346     //GROUP_C
#define SPT1_AFS_O_C              347     //GROUP_C
#define SPT1_BFS_O_C              348     //GROUP_C
#define SPT2_AFS_O_C              349     //GROUP_C
#define SPT2_BFS_O_C              350     //GROUP_C
#define SPDIF0_FS_O_C             351     //GROUP_C
#define PCG0_FSA_O_C              352     //GROUP_C
#define PCG0_FSB_O_C              353     //GROUP_C
#define SPT0_ACLK_O_D             354     //GROUP_D
#define SPT0_BCLK_O_D             355     //GROUP_D
#define SPT1_ACLK_O_D             356     //GROUP_D
#define SPT1_BCLK_O_D             357     //GROUP_D
#define SPT2_ACLK_O_D             358     //GROUP_D
#define SPT2_BCLK_O_D             359     //GROUP_D
#define SPT0_AFS_O_D              360     //GROUP_D
#define SPT0_BFS_O_D              361     //GROUP_D
#define SPT1_AFS_O_D              362     //GROUP_D
#define SPT1_BFS_O_D              363     //GROUP_D
#define SPT2_AFS_O_D              364     //GROUP_D
#define SPT2_BFS_O_D              365     //GROUP_D
#define SPT3_AD0_O_D              366     //GROUP_D
#define SPT3_AD1_O_D              367     //GROUP_D
#define SPT3_BD0_O_D              368     //GROUP_D
#define SPT3_BD1_O_D              369     //GROUP_D
#define MLB0_CLKOUT_O_D           370     //GROUP_D
#define SPDIF0_TX_BLKSTART_O_D    371     //GROUP_D
#define SPT3_ACLK_O_D             372     //GROUP_D
#define SPT3_BCLK_O_D             373     //GROUP_D
#define SPT3_AFS_O_D              374     //GROUP_D
#define SPT3_BFS_O_D              375     //GROUP_D
#define PCG0_CLKA_O_D             376     //GROUP_D
#define PCG0_CLKB_O_D             377     //GROUP_D
#define PCG0_FSA_O_D              378     //GROUP_D
#define PCG0_FSB_O_D              379     //GROUP_D
#define SRC0_DAT_OP_O_D           380     //GROUP_D
#define SRC1_DAT_OP_O_D           381     //GROUP_D
#define SRC2_DAT_OP_O_D           382     //GROUP_D
#define SRC3_DAT_OP_O_D           383     //GROUP_D
#define SPDIF0_RX_DAT_O_D         384     //GROUP_D
#define SPDIF0_RX_FS_O_D          385     //GROUP_D
#define SPDIF0_RX_CLK_O_D         386     //GROUP_D
#define SPDIF0_RX_TDMCLK_O_D      387     //GROUP_D
#define SPDIF0_TX_O_D             388     //GROUP_D
#define SPT0_ATDV_O_D             389     //GROUP_D
#define SPT0_BTDV_O_D             390     //GROUP_D
#define SPT1_ATDV_O_D             391     //GROUP_D
#define SPT1_BTDV_O_D             392     //GROUP_D
#define SPT2_ATDV_O_D             393     //GROUP_D
#define SPT2_BTDV_O_D             394     //GROUP_D
#define SPT3_ATDV_O_D             395     //GROUP_D
#define SPT3_BTDV_O_D             396     //GROUP_D
#define SPDIF0_RX_LRCLK_REF_O_D   397     //GROUP_D
#define SPDIF0_RX_LRCLK_FB_O_D    398     //GROUP_D
#define PCG0_CRS_CLKC_O_D         399     //GROUP_D
#define PCG0_CRS_CLKD_O_D         400     //GROUP_D
#define PCG0_CRS_FSC_O_D          401     //GROUP_D
#define PCG0_CRS_FSD_O_D          402     //GROUP_D
#define DAI0_CRS_PB03_O_D         403     //GROUP_D
#define DAI0_CRS_PB04_O_D         404     //GROUP_D
#define PCG_CLKE_O_D              405     //GROUP_D
#define PCG_CLKF_O_D              406     //GROUP_D
#define PCG_FSE_O_D               407     //GROUP_D
#define PCG_FSF_O_D               408     //GROUP_D
#define PCG_CLKA_INV_O_D          409     //GROUP_D
#define PCG_CLKB_INV_O_D          410     //GROUP_D
#define PCG_CLKE_INV_O_D          411     //GROUP_D
#define PCG_CLKF_INV_O_D          412     //GROUP_D
#define PCG_FSA_INV_O_D           413     //GROUP_D
#define PCG_FSB_INV_O_D           414     //GROUP_D
#define PCG_FSE_INV_O_D           415     //GROUP_D
#define PCG_FSF_INV_O_D           416     //GROUP_D
#define PDM0_CLK0_O_D             417     //GROUP_D
#define PDM0_SDATA_O_D            418     //GROUP_D
#define DAI0_LOW_D                419     //GROUP_D
#define DAI0_HIGH_D               420     //GROUP_D
#define SPT0A_FS_O_E              421     //GROUP_E
#define SPT0B_FS_O_E              422     //GROUP_E
#define SPT1A_FS_O_E              423     //GROUP_E
#define SPT1B_FS_O_E              424     //GROUP_E
#define SPT2A_FS_O_E              425     //GROUP_E
#define SPT2B_FS_O_E              426     //GROUP_E
#define SPDIF0_TX_BLKSTART_O_E    427     //GROUP_E
#define PCG0_FSA_O_E              428     //GROUP_E
#define PCG0_CLKB_O_E             429     //GROUP_E
#define PCG0_FSB_O_E              430     //GROUP_E
#define DAI0_LOW_F                431     //GROUP_F
#define DAI0_HIGH_F               432     //GROUP_F
#define DAI0_MISCA0_O_F           433     //GROUP_F
#define DAI0_MISCA1_O_F           434     //GROUP_F
#define DAI0_MISCA2_O_F           435     //GROUP_F
#define DAI0_MISCA3_O_F           436     //GROUP_F
#define DAI0_MISCA4_O_F           437     //GROUP_F
#define DAI0_MISCA5_O_F           438     //GROUP_F
#define SPT0_ACLK_PBEN_O_F        439     //GROUP_F
#define SPT0_AFS_PBEN_O_F         440     //GROUP_F
#define SPT0_AD0_PBEN_O_F         441     //GROUP_F
#define SPT0_AD1_PBEN_O_F         442     //GROUP_F
#define SPT0_BCLK_PBEN_O_F        443     //GROUP_F
#define SPT0_BFS_PBEN_O_F         444     //GROUP_F
#define SPT0_BD0_PBEN_O_F         445     //GROUP_F
#define SPT0_BD1_PBEN_O_F         446     //GROUP_F
#define SPT1_ACLK_PBEN_O_F        447     //GROUP_F
#define SPT1_AFS_PBEN_O_F         448     //GROUP_F
#define SPT1_AD0_PBEN_O_F         449     //GROUP_F
#define SPT1_AD1_PBEN_O_F         450     //GROUP_F
#define SPT1_BCLK_PBEN_O_F        451     //GROUP_F
#define SPT1_BFS_PBEN_O_F         452     //GROUP_F
#define SPT1_BD0_PBEN_O_F         453     //GROUP_F
#define SPT1_BD1_PBEN_O_F         454     //GROUP_F
#define SPT2_ACLK_PBEN_O_F        455     //GROUP_F
#define SPT2_AFS_PBEN_O_F         456     //GROUP_F
#define SPT2_AD0_PBEN_O_F         457     //GROUP_F
#define SPT2_AD1_PBEN_O_F         458     //GROUP_F
#define SPT2_BCLK_PBEN_O_F        459     //GROUP_F
#define SPT2_BFS_PBEN_O_F         460     //GROUP_F
#define SPT2_BD0_PBEN_O_F         461     //GROUP_F
#define SPT2_BD1_PBEN_O_F         462     //GROUP_F
#define SPT3_ACLK_PBEN_O_F        463     //GROUP_F
#define SPT3_AFS_PBEN_O_F         464     //GROUP_F
#define SPT3_AD0_PBEN_O_F         465     //GROUP_F
#define SPT3_AD1_PBEN_O_F         466     //GROUP_F
#define SPT3_BCLK_PBEN_O_F        467     //GROUP_F
#define SPT3_BFS_PBEN_O_F         468     //GROUP_F
#define SPT3_BD0_PBEN_O_F         469     //GROUP_F
#define SPT3_BD1_PBEN_O_F         470     //GROUP_F
#define SPT0_ATDV_PBEN_O_F        471     //GROUP_F
#define SPT0_BTDV_PBEN_O_F        472     //GROUP_F
#define SPT1_ATDV_PBEN_O_F        473     //GROUP_F
#define SPT1_BTDV_PBEN_O_F        474     //GROUP_F
#define SPT2_ATDV_PBEN_O_F        475     //GROUP_F
#define SPT2_BTDV_PBEN_O_F        476     //GROUP_F
#define SPT3_ATDV_PBEN_O_F        477     //GROUP_F
#define SPT3_BTDV_PBEN_O_F        478     //GROUP_F
#define PDM0_CLK0_OE_O_F          479     //GROUP_F
#define PDM0_SDATA_OE_O_F         480     //GROUP_F

//DAI1 Sources
#define DAI1_PB01_O_ABCDE         481     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB02_O_ABCDE         482     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB03_O_ABCDE         483     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB04_O_ABCDE         484     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB05_O_ABCDE         485     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB06_O_ABCDE         486     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB07_O_ABCDE         487     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB08_O_ABCDE         488     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB09_O_ABCDE         489     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB10_O_ABCDE         490     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB11_O_ABCDE         491     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB12_O_ABCDE         492     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB13_O_ABCDE         493     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB14_O_ABCDE         494     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB15_O_ABCDE         495     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB16_O_ABCDE         496     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB17_O_ABCDE         497     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB18_O_ABCDE         498     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB19_O_ABCDE         499     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_PB20_O_ABCDE         500     //GROUP_A GROUP_B GROUP_C GROUP_D GROUP_E
#define DAI1_LOW_ACE              501     //GROUP_A GROUP_C GROUP_E
#define DAI1_HIGH_ACE             502     //GROUP_A GROUP_C GROUP_E
#define SPT4_AD0_O_BD             503     //GROUP_B GROUP_D
#define SPT4_AD1_O_BD             504     //GROUP_B GROUP_D
#define SPT4_BD0_O_BD             505     //GROUP_B GROUP_D
#define SPT4_BD1_O_BD             506     //GROUP_B GROUP_D
#define SPT5_AD0_O_BD             507     //GROUP_B GROUP_D
#define SPT5_AD1_O_BD             508     //GROUP_B GROUP_D
#define SPT5_BD0_O_BD             509     //GROUP_B GROUP_D
#define SPT5_BD1_O_BD             510     //GROUP_B GROUP_D
#define SPT6_AD0_O_BD             511     //GROUP_B GROUP_D
#define SPT6_AD1_O_BD             512     //GROUP_B GROUP_D
#define SPT6_BD0_O_BD             513     //GROUP_B GROUP_D
#define SPT6_BD1_O_BD             514     //GROUP_B GROUP_D
#define DAI1_CRS_PB03_O_A         515     //GROUP_A
#define SPT4_ACLK_O_A             516     //GROUP_A
#define SPT4_BCLK_O_A             517     //GROUP_A
#define SPT5_ACLK_O_A             518     //GROUP_A
#define SPT5_BCLK_O_A             519     //GROUP_A
#define SPT6_ACLK_O_A             520     //GROUP_A
#define SPT6_BCLK_O_A             521     //GROUP_A
#define SPDIF1_RX_CLK_O_A         522     //GROUP_A
#define SPDIF1_RX_TDMCLK_O_A      523     //GROUP_A
#define PCG0_CLKC_O_A             524     //GROUP_A
#define PCG0_CLKD_O_A             525     //GROUP_A
#define SRC4_DAT_OP_O_B           526     //GROUP_B
#define SRC5_DAT_OP_O_B           527     //GROUP_B
#define SRC6_DAT_OP_O_B           528     //GROUP_B
#define SRC7_DAT_OP_O_B           529     //GROUP_B
#define SRC4_TDM_IP_O_B           530     //GROUP_B
#define SRC5_TDM_IP_O_B           531     //GROUP_B
#define SRC6_TDM_IP_O_B           532     //GROUP_B
#define SRC7_TDM_IP_O_B           533     //GROUP_B
#define SPDIF1_RX_DAT_O_B         534     //GROUP_B
#define SPT7_AD0_O_B              535     //GROUP_B
#define SPT7_AD1_O_B              536     //GROUP_B
#define SPT7_BD0_O_B              537     //GROUP_B
#define SPT7_BD1_O_B              538     //GROUP_B
#define SPDIF1_TX_O_B             539     //GROUP_B
#define SRC3_CRS_DAT_OP_O_B       540     //GROUP_B
#define SRC3_CRS_TDM_IP_O_B       541     //GROUP_B
#define PDM1_SDATA_O_B            542     //GROUP_B
#define DAI1_LOW_B                543     //GROUP_B
#define DAI1_HIGH_B               544     //GROUP_B
#define DAI1_CRS_PB04_O_C         545     //GROUP_C
#define SPT4_AFS_O_C              546     //GROUP_C
#define SPT4_BFS_O_C              547     //GROUP_C
#define SPT5_AFS_O_C              548     //GROUP_C
#define SPT5_BFS_O_C              549     //GROUP_C
#define SPT6_AFS_O_C              550     //GROUP_C
#define SPT6_BFS_O_C              551     //GROUP_C
#define SPDIF1_FS_O_C             552     //GROUP_C
#define PCG0_FSC_O_C              553     //GROUP_C
#define PCG0_FSD_O_C              554     //GROUP_C
#define SPT4_ACLK_O_D             555     //GROUP_D
#define SPT4_BCLK_O_D             556     //GROUP_D
#define SPT5_ACLK_O_D             557     //GROUP_D
#define SPT5_BCLK_O_D             558     //GROUP_D
#define SPT6_ACLK_O_D             559     //GROUP_D
#define SPT6_BCLK_O_D             560     //GROUP_D
#define SPT4_AFS_O_D              561     //GROUP_D
#define SPT4_BFS_O_D              562     //GROUP_D
#define SPT5_AFS_O_D              563     //GROUP_D
#define SPT5_BFS_O_D              564     //GROUP_D
#define SPT6_AFS_O_D              565     //GROUP_D
#define SPT6_BFS_O_D              566     //GROUP_D
#define SPT7_AD0_O_D              567     //GROUP_D
#define SPT7_AD1_O_D              568     //GROUP_D
#define SPT7_BD0_O_D              569     //GROUP_D
#define SPT7_BD1_O_D              570     //GROUP_D
#define SPDIF1_TX_BLKSTART_O_D    572     //GROUP_D
#define SPT7_ACLK_O_D             573     //GROUP_D
#define SPT7_BCLK_O_D             574     //GROUP_D
#define SPT7_AFS_O_D              575     //GROUP_D
#define SPT7_BFS_O_D              576     //GROUP_D
#define PCG0_CLKC_O_D             577     //GROUP_D
#define PCG0_CLKD_O_D             578     //GROUP_D
#define PCG0_FSC_O_D              579     //GROUP_D
#define PCG0_FSD_O_D              580     //GROUP_D
#define SRC4_DAT_OP_O_D           581     //GROUP_D
#define SRC5_DAT_OP_O_D           582     //GROUP_D
#define SRC6_DAT_OP_O_D           583     //GROUP_D
#define SRC7_DAT_OP_O_D           584     //GROUP_D
#define SPDIF1_RX_DAT_O_D         585     //GROUP_D
#define SPDIF1_RX_FS_O_D          586     //GROUP_D
#define SPDIF1_RX_CLK_O_D         587     //GROUP_D
#define SPDIF1_RX_TDMCLK_O_D      588     //GROUP_D
#define SPDIF1_TX_O_D             589     //GROUP_D
#define SPT4_ATDV_O_D             590     //GROUP_D
#define SPT4_BTDV_O_D             591     //GROUP_D
#define SPT5_ATDV_O_D             592     //GROUP_D
#define SPT5_BTDV_O_D             593     //GROUP_D
#define SPT6_ATDV_O_D             594     //GROUP_D
#define SPT6_BTDV_O_D             595     //GROUP_D
#define SPT7_ATDV_O_D             596     //GROUP_D
#define SPT7_BTDV_O_D             597     //GROUP_D
#define SPDIF1_RX_LRCLK_REF_O_D   598     //GROUP_D
#define SPDIF1_RX_LRCLK_FB_O_D    599     //GROUP_D
#define PCG0_CRS_CLKA_O_D         600     //GROUP_D
#define PCG0_CRS_CLKB_O_D         601     //GROUP_D
#define PCG0_CRS_FSA_O_D          602     //GROUP_D
#define PCG0_CRS_FSB_O_D          603     //GROUP_D
#define DAI1_CRS_PB03_O_D         604     //GROUP_D
#define DAI1_CRS_PB04_O_D         605     //GROUP_D
#define PCG_CLKG_O_D              606     //GROUP_D
#define PCG_CLKH_O_D              607     //GROUP_D
#define PCG_FSG_O_D               608     //GROUP_D
#define PCG_FSH_O_D               609     //GROUP_D
#define PCG_CLKC_INV_O_D          610     //GROUP_D
#define PCG_CLKG_INV_O_D          611     //GROUP_D
#define PCG_CLKD_INV_O_D          612     //GROUP_D
#define PCG_CLKH_INV_O_D          613     //GROUP_D
#define PCG_FSC_INV_O_D           614     //GROUP_D
#define PCG_FSD_INV_O_D           615     //GROUP_D
#define PCG_FSG_INV_O_D           616     //GROUP_D
#define PCG_FSH_INV_O_D           617     //GROUP_D
#define PDM1_CLK0_O_D             618     //GROUP_D
#define PDM1_SDATA_O_D            619     //GROUP_D
#define DAI1_LOW_D                620     //GROUP_D
#define DAI1_HIGH_D               621     //GROUP_D
#define SPT4A_FS_O_E              622     //GROUP_E
#define SPT4B_FS_O_E              623     //GROUP_E
#define SPT5A_FS_O_E              624     //GROUP_E
#define SPT5B_FS_O_E              625     //GROUP_E
#define SPT6A_FS_O_E              626     //GROUP_E
#define SPT6B_FS_O_E              627     //GROUP_E
#define SPDIF1_TX_BLKSTART_O_E    628     //GROUP_E
#define PCG0_FSC_O_E              629     //GROUP_E
#define PCG0_CLKD_O_E             630     //GROUP_E
#define PCG0_FSD_O_E              631     //GROUP_E
#define DAI1_LOW_F                632     //GROUP_F
#define DAI1_HIGH_F               633     //GROUP_F
#define DAI1_MISCA0_O_F           634     //GROUP_F
#define DAI1_MISCA1_O_F           635     //GROUP_F
#define DAI1_MISCA2_O_F           636     //GROUP_F
#define DAI1_MISCA3_O_F           637     //GROUP_F
#define DAI1_MISCA4_O_F           638     //GROUP_F
#define DAI1_MISCA5_O_F           639     //GROUP_F
#define SPT4_ACLK_PBEN_O_F        640     //GROUP_F
#define SPT4_AFS_PBEN_O_F         641     //GROUP_F
#define SPT4_AD0_PBEN_O_F         642     //GROUP_F
#define SPT4_AD1_PBEN_O_F         643     //GROUP_F
#define SPT4_BCLK_PBEN_O_F        644     //GROUP_F
#define SPT4_BFS_PBEN_O_F         645     //GROUP_F
#define SPT4_BD0_PBEN_O_F         646     //GROUP_F
#define SPT4_BD1_PBEN_O_F         647     //GROUP_F
#define SPT5_ACLK_PBEN_O_F        648     //GROUP_F
#define SPT5_AFS_PBEN_O_F         649     //GROUP_F
#define SPT5_AD0_PBEN_O_F         650     //GROUP_F
#define SPT5_AD1_PBEN_O_F         651     //GROUP_F
#define SPT5_BCLK_PBEN_O_F        652     //GROUP_F
#define SPT5_BFS_PBEN_O_F         653     //GROUP_F
#define SPT5_BD0_PBEN_O_F         654     //GROUP_F
#define SPT5_BD1_PBEN_O_F         655     //GROUP_F
#define SPT6_ACLK_PBEN_O_F        656     //GROUP_F
#define SPT6_AFS_PBEN_O_F         657     //GROUP_F
#define SPT6_AD0_PBEN_O_F         658     //GROUP_F
#define SPT6_AD1_PBEN_O_F         659     //GROUP_F
#define SPT6_BCLK_PBEN_O_F        660     //GROUP_F
#define SPT6_BFS_PBEN_O_F         661     //GROUP_F
#define SPT6_BD0_PBEN_O_F         662     //GROUP_F
#define SPT6_BD1_PBEN_O_F         663     //GROUP_F
#define SPT7_ACLK_PBEN_O_F        664     //GROUP_F
#define SPT7_AFS_PBEN_O_F         665     //GROUP_F
#define SPT7_AD0_PBEN_O_F         666     //GROUP_F
#define SPT7_AD1_PBEN_O_F         667     //GROUP_F
#define SPT7_BCLK_PBEN_O_F        668     //GROUP_F
#define SPT7_BFS_PBEN_O_F         669     //GROUP_F
#define SPT7_BD0_PBEN_O_F         670     //GROUP_F
#define SPT7_BD1_PBEN_O_F         671     //GROUP_F
#define SPT4_ATDV_PBEN_O_F        672     //GROUP_F
#define SPT4_BTDV_PBEN_O_F        673     //GROUP_F
#define SPT5_ATDV_PBEN_O_F        674     //GROUP_F
#define SPT5_BTDV_PBEN_O_F        675     //GROUP_F
#define SPT6_ATDV_PBEN_O_F        676     //GROUP_F
#define SPT6_BTDV_PBEN_O_F        677     //GROUP_F
#define SPT7_ATDV_PBEN_O_F        678     //GROUP_F
#define SPT7_BTDV_PBEN_O_F        679     //GROUP_F
#define PDM1_CLK0_OE_O_F          680     //GROUP_F
#define PDM1_SDATA_OE_O_F         681     //GROUP_F
