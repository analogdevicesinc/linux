/**
 * dev-regs-macro.h - Cadence USB3 Device register definition
 *
 * Copyright (C) 2016 Cadence Design Systems - http://www.cadence.com
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __REG_USBSS_DEV_ADDR_MAP_MACRO_H__
#define __REG_USBSS_DEV_ADDR_MAP_MACRO_H__


/* macros for BlueprintGlobalNameSpace::USB_CONF */
#ifndef __USB_CONF_MACRO__
#define __USB_CONF_MACRO__

/* macros for field CFGRST */
#define USB_CONF__CFGRST__MASK                                      0x00000001U
#define USB_CONF__CFGSET__MASK                                      0x00000002U
#define USB_CONF__USB3DIS__MASK                                     0x00000008U
#define USB_CONF__DEVEN__MASK                                       0x00004000U
#define USB_CONF__DEVDS__MASK                                       0x00008000U
#define USB_CONF__L1EN__MASK                                        0x00010000U
#define USB_CONF__L1DS__MASK                                        0x00020000U
#define USB_CONF__CLK2OFFDS__MASK                                   0x00080000U
#define USB_CONF__U1EN__MASK                                        0x01000000U
#define USB_CONF__U1DS__MASK                                        0x02000000U
#define USB_CONF__U2EN__MASK                                        0x04000000U
#define USB_CONF__U2DS__MASK                                        0x08000000U
#endif /* __USB_CONF_MACRO__ */


/* macros for usbss_dev_register_block.usb_conf */
#ifndef __USB_STS_MACRO__
#define __USB_STS_MACRO__

/* macros for field CFGSTS */
#define USB_STS__CFGSTS__MASK                                       0x00000001U
#define USB_STS__USBSPEED__READ(src)     (((uint32_t)(src) & 0x00000070U) >> 4)

/* macros for field ENDIAN_MIRROR */
#define USB_STS__LPMST__READ(src)       (((uint32_t)(src) & 0x000c0000U) >> 18)

/* macros for field USB2CONS */
#define USB_STS__U1ENS__MASK                                        0x01000000U
#define USB_STS__U2ENS__MASK                                        0x02000000U
#define USB_STS__LST__READ(src)         (((uint32_t)(src) & 0x3c000000U) >> 26)

/* macros for field DMAOFF */
#endif /* __USB_STS_MACRO__ */


/* macros for usbss_dev_register_block.usb_sts */
#ifndef __USB_CMD_MACRO__
#define __USB_CMD_MACRO__

/* macros for field SET_ADDR */
#define USB_CMD__SET_ADDR__MASK                                     0x00000001U
#define USB_CMD__FADDR__WRITE(src)       (((uint32_t)(src) << 1) & 0x000000feU)
#endif /* __USB_CMD_MACRO__ */


/* macros for usbss_dev_register_block.usb_cmd */
#ifndef __USB_ITPN_MACRO__
#define __USB_ITPN_MACRO__

/* macros for field ITPN */
#endif /* __USB_ITPN_MACRO__ */


/* macros for usbss_dev_register_block.usb_iptn */
#ifndef __USB_LPM_MACRO__
#define __USB_LPM_MACRO__

/* macros for field HIRD */
#endif /* __USB_LPM_MACRO__ */


/* macros for usbss_dev_register_block.usb_lpm */
#ifndef __USB_IEN_MACRO__
#define __USB_IEN_MACRO__

/* macros for field CONIEN */
#define USB_IEN__CONIEN__MASK                                       0x00000001U
#define USB_IEN__DISIEN__MASK                                       0x00000002U
#define USB_IEN__UWRESIEN__MASK                                     0x00000004U
#define USB_IEN__UHRESIEN__MASK                                     0x00000008U
#define USB_IEN__U3EXTIEN__MASK                                     0x00000020U
#define USB_IEN__CON2IEN__MASK                                      0x00010000U
#define USB_IEN__U2RESIEN__MASK                                     0x00040000U
#define USB_IEN__L2ENTIEN__MASK                                     0x00100000U
#define USB_IEN__L2EXTIEN__MASK                                     0x00200000U
#endif /* __USB_IEN_MACRO__ */


/* macros for usbss_dev_register_block.usb_ien */
#ifndef __USB_ISTS_MACRO__
#define __USB_ISTS_MACRO__

/* macros for field CONI */
#define USB_ISTS__CONI__SHIFT                                                 0
#define USB_ISTS__DISI__SHIFT                                                 1
#define USB_ISTS__UWRESI__SHIFT                                               2
#define USB_ISTS__UHRESI__SHIFT                                               3
#define USB_ISTS__U3EXTI__SHIFT                                               5
#define USB_ISTS__CON2I__SHIFT                                               16
#define USB_ISTS__DIS2I__SHIFT                                               17
#define USB_ISTS__DIS2I__MASK                                       0x00020000U
#define USB_ISTS__U2RESI__SHIFT                                              18
#define USB_ISTS__L2ENTI__SHIFT                                              20
#define USB_ISTS__L2EXTI__SHIFT                                              21
#endif /* __USB_ISTS_MACRO__ */


/* macros for usbss_dev_register_block.usb_ists */
#ifndef __EP_SEL_MACRO__
#define __EP_SEL_MACRO__

/* macros for field EPNO */
#endif /* __EP_SEL_MACRO__ */


/* macros for usbss_dev_register_block.ep_sel */
#ifndef __EP_TRADDR_MACRO__
#define __EP_TRADDR_MACRO__

/* macros for field TRADDR */
#define EP_TRADDR__TRADDR__WRITE(src)           ((uint32_t)(src) & 0xffffffffU)
#endif /* __EP_TRADDR_MACRO__ */


/* macros for usbss_dev_register_block.ep_traddr */
#ifndef __EP_CFG_MACRO__
#define __EP_CFG_MACRO__

/* macros for field ENABLE */
#define EP_CFG__ENABLE__MASK                                        0x00000001U
#define EP_CFG__EPTYPE__WRITE(src)       (((uint32_t)(src) << 1) & 0x00000006U)
#define EP_CFG__MAXBURST__WRITE(src)     (((uint32_t)(src) << 8) & 0x00000f00U)
#define EP_CFG__MAXPKTSIZE__WRITE(src)  (((uint32_t)(src) << 16) & 0x07ff0000U)
#define EP_CFG__BUFFERING__WRITE(src)   (((uint32_t)(src) << 27) & 0xf8000000U)
#endif /* __EP_CFG_MACRO__ */


/* macros for usbss_dev_register_block.ep_cfg */
#ifndef __EP_CMD_MACRO__
#define __EP_CMD_MACRO__

/* macros for field EPRST */
#define EP_CMD__EPRST__MASK                                         0x00000001U
#define EP_CMD__SSTALL__MASK                                        0x00000002U
#define EP_CMD__CSTALL__MASK                                        0x00000004U
#define EP_CMD__ERDY__MASK                                          0x00000008U
#define EP_CMD__REQ_CMPL__MASK                                      0x00000020U
#define EP_CMD__DRDY__MASK                                          0x00000040U
#define EP_CMD__DFLUSH__MASK                                        0x00000080U
#endif /* __EP_CMD_MACRO__ */


/* macros for usbss_dev_register_block.ep_cmd */
#ifndef __EP_STS_MACRO__
#define __EP_STS_MACRO__

/* macros for field SETUP */
#define EP_STS__SETUP__MASK                                         0x00000001U
#define EP_STS__STALL__MASK                                         0x00000002U
#define EP_STS__IOC__MASK                                           0x00000004U
#define EP_STS__ISP__MASK                                           0x00000008U
#define EP_STS__DESCMIS__MASK                                       0x00000010U
#define EP_STS__TRBERR__MASK                                        0x00000080U
#define EP_STS__NRDY__MASK                                          0x00000100U
#define EP_STS__DBUSY__MASK                                         0x00000200U
#define EP_STS__OUTSMM__MASK                                        0x00004000U
#define EP_STS__ISOERR__MASK                                        0x00008000U
#endif /* __EP_STS_MACRO__ */


/* macros for usbss_dev_register_block.ep_sts */
#ifndef __EP_STS_SID_MACRO__
#define __EP_STS_SID_MACRO__

/* macros for field SID */
#endif /* __EP_STS_SID_MACRO__ */


/* macros for usbss_dev_register_block.ep_sts_sid */
#ifndef __EP_STS_EN_MACRO__
#define __EP_STS_EN_MACRO__

/* macros for field SETUPEN */
#define EP_STS_EN__SETUPEN__MASK                                    0x00000001U
#define EP_STS_EN__DESCMISEN__MASK                                  0x00000010U
#define EP_STS_EN__TRBERREN__MASK                                   0x00000080U
#endif /* __EP_STS_EN_MACRO__ */


/* macros for usbss_dev_register_block.ep_sts_en */
#ifndef __DRBL_MACRO__
#define __DRBL_MACRO__

/* macros for field DRBL0O */
#endif /* __DRBL_MACRO__ */


/* macros for usbss_dev_register_block.drbl */
#ifndef __EP_IEN_MACRO__
#define __EP_IEN_MACRO__

/* macros for field EOUTEN0 */
#define EP_IEN__EOUTEN0__MASK                                       0x00000001U
#define EP_IEN__EINEN0__MASK                                        0x00010000U
#endif /* __EP_IEN_MACRO__ */


/* macros for usbss_dev_register_block.ep_ien */
#ifndef __EP_ISTS_MACRO__
#define __EP_ISTS_MACRO__

/* macros for field EOUT0 */
#define EP_ISTS__EOUT0__MASK                                        0x00000001U
#define EP_ISTS__EIN0__MASK                                         0x00010000U
#endif /* __EP_ISTS_MACRO__ */


/* macros for usbss_dev_register_block.ep_ists */
#ifndef __USB_PWR_MACRO__
#define __USB_PWR_MACRO__

/* macros for field PSO_EN */
#endif /* __USB_PWR_MACRO__ */


/* macros for usbss_dev_register_block.usb_pwr */
#ifndef __USB_CONF2_MACRO__
#define __USB_CONF2_MACRO__

/* macros for field AHB_RETRY_EN */
#endif /* __USB_CONF2_MACRO__ */


/* macros for usbss_dev_register_block.usb_conf2 */
#ifndef __USB_CAP1_MACRO__
#define __USB_CAP1_MACRO__

/* macros for field SFR_TYPE */
#endif /* __USB_CAP1_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap1 */
#ifndef __USB_CAP2_MACRO__
#define __USB_CAP2_MACRO__

/* macros for field ACTUAL_MEM_SIZE */
#endif /* __USB_CAP2_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap2 */
#ifndef __USB_CAP3_MACRO__
#define __USB_CAP3_MACRO__

/* macros for field EPOUT_N */
#endif /* __USB_CAP3_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap3 */
#ifndef __USB_CAP4_MACRO__
#define __USB_CAP4_MACRO__

/* macros for field EPOUTI_N */
#endif /* __USB_CAP4_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap4 */
#ifndef __USB_CAP5_MACRO__
#define __USB_CAP5_MACRO__

/* macros for field EPOUTI_N */
#endif /* __USB_CAP5_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap5 */
#ifndef __USB_CAP6_MACRO__
#define __USB_CAP6_MACRO__

/* macros for field VERSION */
#endif /* __USB_CAP6_MACRO__ */


/* macros for usbss_dev_register_block.usb_cap6 */
#ifndef __USB_CPKT1_MACRO__
#define __USB_CPKT1_MACRO__

/* macros for field CPKT1 */
#endif /* __USB_CPKT1_MACRO__ */


/* macros for usbss_dev_register_block.usb_cpkt1 */
#ifndef __USB_CPKT2_MACRO__
#define __USB_CPKT2_MACRO__

/* macros for field CPKT2 */
#endif /* __USB_CPKT2_MACRO__ */


/* macros for usbss_dev_register_block.usb_cpkt2 */
#ifndef __USB_CPKT3_MACRO__
#define __USB_CPKT3_MACRO__

/* macros for field CPKT3 */
#endif /* __USB_CPKT3_MACRO__ */


/* macros for usbss_dev_register_block.usb_cpkt3 */
#ifndef __CFG_REG1_MACRO__
#define __CFG_REG1_MACRO__

/* macros for field DEBOUNCER_CNT */
#endif /* __CFG_REG1_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg1 */
#ifndef __DBG_LINK1_MACRO__
#define __DBG_LINK1_MACRO__

/* macros for field LFPS_MIN_DET_U1_EXIT */
#define DBG_LINK1__LFPS_MIN_GEN_U1_EXIT__WRITE(src) \
			(((uint32_t)(src)\
			<< 8) & 0x0000ff00U)
#define DBG_LINK1__LFPS_MIN_GEN_U1_EXIT_SET__MASK                   0x02000000U
#endif /* __DBG_LINK1_MACRO__ */


/* macros for usbss_dev_register_block.dbg_link1 */
#ifndef __DBG_LINK2_MACRO__
#define __DBG_LINK2_MACRO__

/* macros for field RXEQTR_AVAL */
#endif /* __DBG_LINK2_MACRO__ */


/* macros for usbss_dev_register_block.dbg_link2 */
#ifndef __CFG_REG4_MACRO__
#define __CFG_REG4_MACRO__

/* macros for field RXDETECT_QUIET_TIMEOUT */
#endif /* __CFG_REG4_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg4 */
#ifndef __CFG_REG5_MACRO__
#define __CFG_REG5_MACRO__

/* macros for field U3_HDSK_FAIL_TIMEOUT */
#endif /* __CFG_REG5_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg5 */
#ifndef __CFG_REG6_MACRO__
#define __CFG_REG6_MACRO__

/* macros for field SSINACTIVE_QUIET_TIMEOUT */
#endif /* __CFG_REG6_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg6 */
#ifndef __CFG_REG7_MACRO__
#define __CFG_REG7_MACRO__

/* macros for field POLLING_LFPS_TIMEOUT */
#endif /* __CFG_REG7_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg7 */
#ifndef __CFG_REG8_MACRO__
#define __CFG_REG8_MACRO__

/* macros for field POLLING_ACTIVE_TIMEOUT */
#endif /* __CFG_REG8_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg8 */
#ifndef __CFG_REG9_MACRO__
#define __CFG_REG9_MACRO__

/* macros for field POLLING_IDLE_TIMEOUT */
#endif /* __CFG_REG9_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg9 */
#ifndef __CFG_REG10_MACRO__
#define __CFG_REG10_MACRO__

/* macros for field POLLING_CONF_TIMEOUT */
#endif /* __CFG_REG10_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg10 */
#ifndef __CFG_REG11_MACRO__
#define __CFG_REG11_MACRO__

/* macros for field RECOVERY_ACTIVE_TIMEOUT */
#endif /* __CFG_REG11_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg11 */
#ifndef __CFG_REG12_MACRO__
#define __CFG_REG12_MACRO__

/* macros for field RECOVERY_CONF_TIMEOUT */
#endif /* __CFG_REG12_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg12 */
#ifndef __CFG_REG13_MACRO__
#define __CFG_REG13_MACRO__

/* macros for field RECOVERY_IDLE_TIMEOUT */
#endif /* __CFG_REG13_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg13 */
#ifndef __CFG_REG14_MACRO__
#define __CFG_REG14_MACRO__

/* macros for field HOTRESET_ACTIVE_TIMEOUT */
#endif /* __CFG_REG14_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg14 */
#ifndef __CFG_REG15_MACRO__
#define __CFG_REG15_MACRO__

/* macros for field HOTRESET_EXIT_TIMEOUT */
#endif /* __CFG_REG15_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg15 */
#ifndef __CFG_REG16_MACRO__
#define __CFG_REG16_MACRO__

/* macros for field LFPS_PING_REPEAT */
#endif /* __CFG_REG16_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg16 */
#ifndef __CFG_REG17_MACRO__
#define __CFG_REG17_MACRO__

/* macros for field PENDING_HP_TIMEOUT */
#endif /* __CFG_REG17_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg17 */
#ifndef __CFG_REG18_MACRO__
#define __CFG_REG18_MACRO__

/* macros for field CREDIT_HP_TIMEOUT */
#endif /* __CFG_REG18_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg18 */
#ifndef __CFG_REG19_MACRO__
#define __CFG_REG19_MACRO__

/* macros for field LUP_TIMEOUT */
#endif /* __CFG_REG19_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg19 */
#ifndef __CFG_REG20_MACRO__
#define __CFG_REG20_MACRO__

/* macros for field LDN_TIMEOUT */
#endif /* __CFG_REG20_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg20 */
#ifndef __CFG_REG21_MACRO__
#define __CFG_REG21_MACRO__

/* macros for field PM_LC_TIMEOUT */
#endif /* __CFG_REG21_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg21 */
#ifndef __CFG_REG22_MACRO__
#define __CFG_REG22_MACRO__

/* macros for field PM_ENTRY_TIMEOUT */
#endif /* __CFG_REG22_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg22 */
#ifndef __CFG_REG23_MACRO__
#define __CFG_REG23_MACRO__

/* macros for field UX_EXIT_TIMEOUT */
#endif /* __CFG_REG23_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg23 */
#ifndef __CFG_REG24_MACRO__
#define __CFG_REG24_MACRO__

/* macros for field LFPS_DET_RESET_MIN */
#endif /* __CFG_REG24_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg24 */
#ifndef __CFG_REG25_MACRO__
#define __CFG_REG25_MACRO__

/* macros for field LFPS_DET_RESET_MAX */
#endif /* __CFG_REG25_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg25 */
#ifndef __CFG_REG26_MACRO__
#define __CFG_REG26_MACRO__

/* macros for field LFPS_DET_POLLING_MIN */
#endif /* __CFG_REG26_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg26 */
#ifndef __CFG_REG27_MACRO__
#define __CFG_REG27_MACRO__

/* macros for field LFPS_DET_POLLING_MAX */
#endif /* __CFG_REG27_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg27 */
#ifndef __CFG_REG28_MACRO__
#define __CFG_REG28_MACRO__

/* macros for field LFPS_DET_PING_MIN */
#endif /* __CFG_REG28_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg28 */
#ifndef __CFG_REG29_MACRO__
#define __CFG_REG29_MACRO__

/* macros for field LFPS_DET_PING_MAX */
#endif /* __CFG_REG29_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg29 */
#ifndef __CFG_REG30_MACRO__
#define __CFG_REG30_MACRO__

/* macros for field LFPS_DET_U1EXIT_MIN */
#endif /* __CFG_REG30_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg30 */
#ifndef __CFG_REG31_MACRO__
#define __CFG_REG31_MACRO__

/* macros for field LFPS_DET_U1EXIT_MAX */
#endif /* __CFG_REG31_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg31 */
#ifndef __CFG_REG32_MACRO__
#define __CFG_REG32_MACRO__

/* macros for field LFPS_DET_U2EXIT_MIN */
#endif /* __CFG_REG32_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg32 */
#ifndef __CFG_REG33_MACRO__
#define __CFG_REG33_MACRO__

/* macros for field LFPS_DET_U2EXIT_MAX */
#endif /* __CFG_REG33_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg33 */
#ifndef __CFG_REG34_MACRO__
#define __CFG_REG34_MACRO__

/* macros for field LFPS_DET_U3EXIT_MIN */
#endif /* __CFG_REG34_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg34 */
#ifndef __CFG_REG35_MACRO__
#define __CFG_REG35_MACRO__

/* macros for field LFPS_DET_U3EXIT_MAX */
#endif /* __CFG_REG35_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg35 */
#ifndef __CFG_REG36_MACRO__
#define __CFG_REG36_MACRO__

/* macros for field LFPS_GEN_PING */
#endif /* __CFG_REG36_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg36 */
#ifndef __CFG_REG37_MACRO__
#define __CFG_REG37_MACRO__

/* macros for field LFPS_GEN_POLLING */
#endif /* __CFG_REG37_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg37 */
#ifndef __CFG_REG38_MACRO__
#define __CFG_REG38_MACRO__

/* macros for field LFPS_GEN_U1EXIT */
#endif /* __CFG_REG38_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg38 */
#ifndef __CFG_REG39_MACRO__
#define __CFG_REG39_MACRO__

/* macros for field LFPS_GEN_U3EXIT */
#endif /* __CFG_REG39_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg39 */
#ifndef __CFG_REG40_MACRO__
#define __CFG_REG40_MACRO__

/* macros for field LFPS_MIN_GEN_U1EXIT */
#endif /* __CFG_REG40_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg40 */
#ifndef __CFG_REG41_MACRO__
#define __CFG_REG41_MACRO__

/* macros for field LFPS_MIN_GEN_U2EXIT */
#endif /* __CFG_REG41_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg41 */
#ifndef __CFG_REG42_MACRO__
#define __CFG_REG42_MACRO__

/* macros for field LFPS_POLLING_REPEAT */
#endif /* __CFG_REG42_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg42 */
#ifndef __CFG_REG43_MACRO__
#define __CFG_REG43_MACRO__

/* macros for field LFPS_POLLING_MAX_TREPEAT */
#endif /* __CFG_REG43_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg43 */
#ifndef __CFG_REG44_MACRO__
#define __CFG_REG44_MACRO__

/* macros for field LFPS_POLLING_MIN_TREPEAT */
#endif /* __CFG_REG44_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg44 */
#ifndef __CFG_REG45_MACRO__
#define __CFG_REG45_MACRO__

/* macros for field ITP_WAKEUP_TIMEOUT */
#endif /* __CFG_REG45_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg45 */
#ifndef __CFG_REG46_MACRO__
#define __CFG_REG46_MACRO__

/* macros for field TSEQ_QUANTITY */
#endif /* __CFG_REG46_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg46 */
#ifndef __CFG_REG47_MACRO__
#define __CFG_REG47_MACRO__

/* macros for field ERDY_TIMEOUT_CNT */
#endif /* __CFG_REG47_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg47 */
#ifndef __CFG_REG48_MACRO__
#define __CFG_REG48_MACRO__

/* macros for field TWTRSTFS_J_CNT */
#endif /* __CFG_REG48_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg48 */
#ifndef __CFG_REG49_MACRO__
#define __CFG_REG49_MACRO__

/* macros for field TUCH_CNT */
#endif /* __CFG_REG49_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg49 */
#ifndef __CFG_REG50_MACRO__
#define __CFG_REG50_MACRO__

/* macros for field TWAITCHK_CNT */
#endif /* __CFG_REG50_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg50 */
#ifndef __CFG_REG51_MACRO__
#define __CFG_REG51_MACRO__

/* macros for field TWTFS_CNT */
#endif /* __CFG_REG51_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg51 */
#ifndef __CFG_REG52_MACRO__
#define __CFG_REG52_MACRO__

/* macros for field TWTREV_CNT */
#endif /* __CFG_REG52_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg52 */
#ifndef __CFG_REG53_MACRO__
#define __CFG_REG53_MACRO__

/* macros for field TWTRSTHS_CNT */
#endif /* __CFG_REG53_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg53 */
#ifndef __CFG_REG54_MACRO__
#define __CFG_REG54_MACRO__

/* macros for field TWTRSM_CNT */
#endif /* __CFG_REG54_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg54 */
#ifndef __CFG_REG55_MACRO__
#define __CFG_REG55_MACRO__

/* macros for field TDRSMUP_CNT */
#endif /* __CFG_REG55_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg55 */
#ifndef __CFG_REG56_MACRO__
#define __CFG_REG56_MACRO__

/* macros for field TOUTHS_CNT */
#endif /* __CFG_REG56_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg56 */
#ifndef __CFG_REG57_MACRO__
#define __CFG_REG57_MACRO__

/* macros for field LFPS_DEB_WIDTH */
#endif /* __CFG_REG57_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg57 */
#ifndef __CFG_REG58_MACRO__
#define __CFG_REG58_MACRO__

/* macros for field LFPS_GEN_U2EXIT */
#endif /* __CFG_REG58_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg58 */
#ifndef __CFG_REG59_MACRO__
#define __CFG_REG59_MACRO__

/* macros for field LFPS_MIN_GEN_U3EXIT */
#endif /* __CFG_REG59_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg59 */
#ifndef __CFG_REG60_MACRO__
#define __CFG_REG60_MACRO__

/* macros for field PORT_CONFIG_TIMEOUT */
#endif /* __CFG_REG60_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg60 */
#ifndef __CFG_REG61_MACRO__
#define __CFG_REG61_MACRO__

/* macros for field LFPS_POL_LFPS_TO_RXEQ */
#endif /* __CFG_REG61_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg61 */
#ifndef __CFG_REG62_MACRO__
#define __CFG_REG62_MACRO__

/* macros for field PHY_TX_LATENCY */
#endif /* __CFG_REG62_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg62 */
#ifndef __CFG_REG63_MACRO__
#define __CFG_REG63_MACRO__

/* macros for field U2_INACTIVITY_TMOUT */
#endif /* __CFG_REG63_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg63 */
#ifndef __CFG_REG64_MACRO__
#define __CFG_REG64_MACRO__

/* macros for field TFILTSE0 */
#endif /* __CFG_REG64_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg64 */
#ifndef __CFG_REG65_MACRO__
#define __CFG_REG65_MACRO__

/* macros for field TFILT */
#endif /* __CFG_REG65_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg65 */
#ifndef __CFG_REG66_MACRO__
#define __CFG_REG66_MACRO__

/* macros for field TWTRSTFS_SE0 */
#endif /* __CFG_REG66_MACRO__ */


/* macros for usbss_dev_register_block.cfg_reg66 */
#ifndef __DMA_AXI_CTRL_MACRO__
#define __DMA_AXI_CTRL_MACRO__

/* macros for field MAWPROT */
#endif /* __DMA_AXI_CTRL_MACRO__ */


/* macros for usbss_dev_register_block.dma_axi_ctrl */
#ifndef __DMA_AXI_ID_MACRO__
#define __DMA_AXI_ID_MACRO__

/* macros for field MAW_ID */
#endif /* __DMA_AXI_ID_MACRO__ */


/* macros for usbss_dev_register_block.dma_axi_id */
#ifndef __DMA_AXI_CAP_MACRO__
#define __DMA_AXI_CAP_MACRO__

/* macros for field RESERVED0 */
#endif /* __DMA_AXI_CAP_MACRO__ */


/* macros for usbss_dev_register_block.dma_axi_cap */
#ifndef __DMA_AXI_CTRL0_MACRO__
#define __DMA_AXI_CTRL0_MACRO__

/* macros for field B_MAX */
#endif /* __DMA_AXI_CTRL0_MACRO__ */


/* macros for usbss_dev_register_block.dma_axi_ctrl0 */
#ifndef __DMA_AXI_CTRL1_MACRO__
#define __DMA_AXI_CTRL1_MACRO__

/* macros for field ROT */
#endif /* __DMA_AXI_CTRL1_MACRO__ */


/* macros for usbss_dev_register_block.dma_axi_ctrl1 */
#endif /* __REG_USBSS_DEV_ADDR_MAP_MACRO_H__ */
