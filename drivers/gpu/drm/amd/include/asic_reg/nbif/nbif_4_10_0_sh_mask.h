/*
 * Copyright 2026 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _nbif_4_10_0_SH_MASK_HEADER
#define _nbif_4_10_0_SH_MASK_HEADER

//BIF_BX_PF0_RSMU_INDEX
#define BIF_BX_PF0_RSMU_INDEX__RSMU_INDEX__SHIFT                                                              0x0
#define BIF_BX_PF0_RSMU_INDEX__RSMU_INDEX_MASK                                                                0xFFFFFFFFL
//BIF_BX_PF0_RSMU_DATA
#define BIF_BX_PF0_RSMU_DATA__RSMU_DATA__SHIFT                                                                0x0
#define BIF_BX_PF0_RSMU_DATA__RSMU_DATA_MASK                                                                  0xFFFFFFFFL
//BIF_BX0_PCIE_INDEX
#define BIF_BX0_PCIE_INDEX__PCIE_INDEX__SHIFT                                                                 0x0
#define BIF_BX0_PCIE_INDEX__PCIE_INDEX_MASK                                                                   0xFFFFFFFFL
//BIF_BX0_PCIE_DATA
#define BIF_BX0_PCIE_DATA__PCIE_DATA__SHIFT                                                                   0x0
#define BIF_BX0_PCIE_DATA__PCIE_DATA_MASK                                                                     0xFFFFFFFFL
//RCC_STRAP0_RCC_BIF_STRAP2
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PCIESWUS_INDEX_APER_RANGE__SHIFT                                     0x0
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWUS_SPT__SHIFT                                                      0x1
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWDS_SPT__SHIFT                                                      0x2
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ENDP_LINKDOWN_DROP_DMA__SHIFT                                        0x5
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWITCH_LINKDOWN_DROP_DMA__SHIFT                                      0x6
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWUS_SEC_LVL_OVRD_EN__SHIFT                                          0x7
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_GMI_DNS_SDP_CLKREQ_TOGGLE_DIS__SHIFT                                 0x8
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ACS_MSKSEV_EP_HIDE_DIS__SHIFT                                        0x9
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_CFG_PG_FW_INTERLOCK_EXIT_EN__SHIFT                                   0xa
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_USB_PD_FUNC_DIS__SHIFT                                               0xc
#define RCC_STRAP0_RCC_BIF_STRAP2__RESERVED_BIF_STRAP2__SHIFT                                                 0xd
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_LTR_IN_ASPML1_DIS__SHIFT                                             0xe
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_GFXAZ_POWERSTATE_INTERLOCK_EN__SHIFT                                 0xf
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PWRBRK_DEGLITCH_CYCLE__SHIFT                                         0x10
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PWRBRK_DEGLITCH_BYPASS__SHIFT                                        0x18
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ARI_EN_UP__SHIFT                                                     0x19
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_VLINK_PMETO_LDN_EXIT_BY_LNKRST_DIS__SHIFT                            0x1f
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PCIESWUS_INDEX_APER_RANGE_MASK                                       0x00000001L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWUS_SPT_MASK                                                        0x00000002L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWDS_SPT_MASK                                                        0x00000004L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ENDP_LINKDOWN_DROP_DMA_MASK                                          0x00000020L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWITCH_LINKDOWN_DROP_DMA_MASK                                        0x00000040L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_SWUS_SEC_LVL_OVRD_EN_MASK                                            0x00000080L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_GMI_DNS_SDP_CLKREQ_TOGGLE_DIS_MASK                                   0x00000100L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ACS_MSKSEV_EP_HIDE_DIS_MASK                                          0x00000200L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_CFG_PG_FW_INTERLOCK_EXIT_EN_MASK                                     0x00000C00L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_USB_PD_FUNC_DIS_MASK                                                 0x00001000L
#define RCC_STRAP0_RCC_BIF_STRAP2__RESERVED_BIF_STRAP2_MASK                                                   0x00002000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_LTR_IN_ASPML1_DIS_MASK                                               0x00004000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_GFXAZ_POWERSTATE_INTERLOCK_EN_MASK                                   0x00008000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PWRBRK_DEGLITCH_CYCLE_MASK                                           0x00FF0000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_PWRBRK_DEGLITCH_BYPASS_MASK                                          0x01000000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_ARI_EN_UP_MASK                                                       0x02000000L
#define RCC_STRAP0_RCC_BIF_STRAP2__STRAP_VLINK_PMETO_LDN_EXIT_BY_LNKRST_DIS_MASK                              0x80000000L
//RCC_STRAP0_RCC_BIF_STRAP3
#define RCC_STRAP0_RCC_BIF_STRAP3__STRAP_VLINK_ASPM_IDLE_TIMER__SHIFT                                         0x0
#define RCC_STRAP0_RCC_BIF_STRAP3__STRAP_VLINK_PM_L1_ENTRY_TIMER__SHIFT                                       0x10
#define RCC_STRAP0_RCC_BIF_STRAP3__STRAP_VLINK_ASPM_IDLE_TIMER_MASK                                           0x0000FFFFL
#define RCC_STRAP0_RCC_BIF_STRAP3__STRAP_VLINK_PM_L1_ENTRY_TIMER_MASK                                         0xFFFF0000L
//RCC_STRAP0_RCC_BIF_STRAP5
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ENTRY_TIMER__SHIFT                                         0x0
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ON_SWUS_LDN_EN__SHIFT                                      0x10
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ON_SWUS_SECRST_EN__SHIFT                                   0x11
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_ENTER_COMPLIANCE_DIS__SHIFT                                    0x12
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_IGNORE_PSN_ON_VDM1_DIS__SHIFT                                        0x13
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_SMN_ERR_STATUS_MASK_EN_UPS__SHIFT                                    0x14
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_REG_PROTECTION_DIS__SHIFT                                            0x15
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_SMN_ERRRSP_DATA_FORCE__SHIFT                                         0x16
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_INTERMEDIATERSP_DATA_ALLF_DATA_FORCE__SHIFT                          0x18
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_EMER_POWER_REDUCTION_SUPPORTED__SHIFT                                0x19
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_EMER_POWER_REDUCTION_INIT_REQ__SHIFT                                 0x1b
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_PWRBRK_STATUS_TIMER__SHIFT                                           0x1c
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_BIF_GFX_REG_APER_REMAPPING_EN__SHIFT                                 0x1f
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ENTRY_TIMER_MASK                                           0x0000FFFFL
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ON_SWUS_LDN_EN_MASK                                        0x00010000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_LDN_ON_SWUS_SECRST_EN_MASK                                     0x00020000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_VLINK_ENTER_COMPLIANCE_DIS_MASK                                      0x00040000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_IGNORE_PSN_ON_VDM1_DIS_MASK                                          0x00080000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_SMN_ERR_STATUS_MASK_EN_UPS_MASK                                      0x00100000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_REG_PROTECTION_DIS_MASK                                              0x00200000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_SMN_ERRRSP_DATA_FORCE_MASK                                           0x00C00000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_INTERMEDIATERSP_DATA_ALLF_DATA_FORCE_MASK                            0x01000000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_EMER_POWER_REDUCTION_SUPPORTED_MASK                                  0x06000000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_EMER_POWER_REDUCTION_INIT_REQ_MASK                                   0x08000000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_PWRBRK_STATUS_TIMER_MASK                                             0x70000000L
#define RCC_STRAP0_RCC_BIF_STRAP5__STRAP_BIF_GFX_REG_APER_REMAPPING_EN_MASK                                   0x80000000L
//RCC_STRAP0_RCC_DEV0_EPF0_STRAP0
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_DEVICE_ID_DEV0_F0__SHIFT                                       0x0
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_MAJOR_REV_ID_DEV0_F0__SHIFT                                    0x10
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_MINOR_REV_ID_DEV0_F0__SHIFT                                    0x14
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_ATI_REV_ID_DEV0_F0__SHIFT                                      0x18
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_FUNC_EN_DEV0_F0__SHIFT                                         0x1c
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_LEGACY_DEVICE_TYPE_EN_DEV0_F0__SHIFT                           0x1d
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_D1_SUPPORT_DEV0_F0__SHIFT                                      0x1e
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_D2_SUPPORT_DEV0_F0__SHIFT                                      0x1f
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_DEVICE_ID_DEV0_F0_MASK                                         0x0000FFFFL
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_MAJOR_REV_ID_DEV0_F0_MASK                                      0x000F0000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_MINOR_REV_ID_DEV0_F0_MASK                                      0x00F00000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_ATI_REV_ID_DEV0_F0_MASK                                        0x0F000000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_FUNC_EN_DEV0_F0_MASK                                           0x10000000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_LEGACY_DEVICE_TYPE_EN_DEV0_F0_MASK                             0x20000000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_D1_SUPPORT_DEV0_F0_MASK                                        0x40000000L
#define RCC_STRAP0_RCC_DEV0_EPF0_STRAP0__STRAP_D2_SUPPORT_DEV0_F0_MASK                                        0x80000000L
//RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_SHORT_VALUE__SHIFT                                      0x0
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_LONG_VALUE__SHIFT                                       0x3
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_REQUIREMENT__SHIFT                                      0x6
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_SHORT_VALUE__SHIFT                                     0x7
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_LONG_VALUE__SHIFT                                      0xa
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_REQUIREMENT__SHIFT                                     0xd
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_MSG_DIS_IN_PM_NON_D0__SHIFT                               0xe
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_RST_LTR_IN_DL_DOWN__SHIFT                                 0xf
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__TX_CHK_FC_FOR_L1__SHIFT                                            0x10
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_DSTATE_USING_WDATA_EN__SHIFT                                   0x11
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_SHORT_VALUE_MASK                                        0x00000007L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_LONG_VALUE_MASK                                         0x00000038L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_S_REQUIREMENT_MASK                                        0x00000040L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_SHORT_VALUE_MASK                                       0x00000380L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_LONG_VALUE_MASK                                        0x00001C00L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_NS_REQUIREMENT_MASK                                       0x00002000L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_MSG_DIS_IN_PM_NON_D0_MASK                                 0x00004000L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_PRIV_RST_LTR_IN_DL_DOWN_MASK                                   0x00008000L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__TX_CHK_FC_FOR_L1_MASK                                              0x00010000L
#define RCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL__LTR_DSTATE_USING_WDATA_EN_MASK                                     0x00020000L
//RCC_DEV0_EPF0_RCC_DOORBELL_APER_EN
#define RCC_DEV0_EPF0_RCC_DOORBELL_APER_EN__BIF_DOORBELL_APER_EN__SHIFT                                       0x0
#define RCC_DEV0_EPF0_RCC_DOORBELL_APER_EN__BIF_DOORBELL_APER_EN_MASK                                         0x00000001L
//RCC_DEV0_EPF0_0_RCC_CONFIG_MEMSIZE
#define RCC_DEV0_EPF0_0_RCC_CONFIG_MEMSIZE__CONFIG_MEMSIZE__SHIFT                                             0x0
#define RCC_DEV0_EPF0_0_RCC_CONFIG_MEMSIZE__CONFIG_MEMSIZE_MASK                                               0xFFFFFFFFL
//BIF_BX0_INTERRUPT_CNTL
#define BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE__SHIFT                                                   0x0
#define BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_EN__SHIFT                                                         0x1
#define BIF_BX0_INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN__SHIFT                                                     0x3
#define BIF_BX0_INTERRUPT_CNTL__IH_INTR_DLY_CNTR__SHIFT                                                       0x4
#define BIF_BX0_INTERRUPT_CNTL__GEN_IH_INT_EN__SHIFT                                                          0x8
#define BIF_BX0_INTERRUPT_CNTL__BIF_RB_REQ_NONSNOOP_EN__SHIFT                                                 0xf
#define BIF_BX0_INTERRUPT_CNTL__DUMMYRD_BYPASS_IN_MSI_EN__SHIFT                                               0x10
#define BIF_BX0_INTERRUPT_CNTL__ALWAYS_SEND_INTPKT_AFTER_DUMMYRD_DIS__SHIFT                                   0x11
#define BIF_BX0_INTERRUPT_CNTL__BIF_RB_REQ_RELAX_ORDER_EN__SHIFT                                              0x12
#define BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK                                                     0x00000001L
#define BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_EN_MASK                                                           0x00000002L
#define BIF_BX0_INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN_MASK                                                       0x00000008L
#define BIF_BX0_INTERRUPT_CNTL__IH_INTR_DLY_CNTR_MASK                                                         0x000000F0L
#define BIF_BX0_INTERRUPT_CNTL__GEN_IH_INT_EN_MASK                                                            0x00000100L
#define BIF_BX0_INTERRUPT_CNTL__BIF_RB_REQ_NONSNOOP_EN_MASK                                                   0x00008000L
#define BIF_BX0_INTERRUPT_CNTL__DUMMYRD_BYPASS_IN_MSI_EN_MASK                                                 0x00010000L
#define BIF_BX0_INTERRUPT_CNTL__ALWAYS_SEND_INTPKT_AFTER_DUMMYRD_DIS_MASK                                     0x00020000L
#define BIF_BX0_INTERRUPT_CNTL__BIF_RB_REQ_RELAX_ORDER_EN_MASK                                                0x00040000L
//BIF_BX0_INTERRUPT_CNTL2
#define BIF_BX0_INTERRUPT_CNTL2__IH_DUMMY_RD_ADDR__SHIFT                                                      0x0
#define BIF_BX0_INTERRUPT_CNTL2__IH_DUMMY_RD_ADDR_MASK                                                        0xFFFFFFFFL
//BIF_BX0_BIF_DOORBELL_INT_CNTL
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_STATUS__SHIFT                                       0x0
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_STATUS__SHIFT                                      0x1
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_STATUS__SHIFT                            0x2
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_CLEAR__SHIFT                                        0x10
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_CLEAR__SHIFT                                       0x11
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_CLEAR__SHIFT                             0x12
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_ERR_EVENT_INTERRUPT_ENABLE__SHIFT                            0x17
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_DISABLE__SHIFT                                      0x18
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_DISABLE__SHIFT                                     0x19
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_DISABLE__SHIFT                           0x1a
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_DB_INTR_STATUS_WHEN_RB_ENABLE__SHIFT                               0x1c
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_IOH_RAS_INTR_STATUS_WHEN_RB_ENABLE__SHIFT                          0x1d
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_ATH_RAS_INTR_STATUS_WHEN_RB_ENABLE__SHIFT                          0x1e
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__TIMEOUT_ERR_EVENT_INTERRUPT_ENABLE__SHIFT                              0x1f
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_STATUS_MASK                                         0x00000001L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_STATUS_MASK                                        0x00000002L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_STATUS_MASK                              0x00000004L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_CLEAR_MASK                                          0x00010000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_CLEAR_MASK                                         0x00020000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_CLEAR_MASK                               0x00040000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_ERR_EVENT_INTERRUPT_ENABLE_MASK                              0x00800000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__DOORBELL_INTERRUPT_DISABLE_MASK                                        0x01000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_CNTLR_INTERRUPT_DISABLE_MASK                                       0x02000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__RAS_ATHUB_ERR_EVENT_INTERRUPT_DISABLE_MASK                             0x04000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_DB_INTR_STATUS_WHEN_RB_ENABLE_MASK                                 0x10000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_IOH_RAS_INTR_STATUS_WHEN_RB_ENABLE_MASK                            0x20000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__SET_ATH_RAS_INTR_STATUS_WHEN_RB_ENABLE_MASK                            0x40000000L
#define BIF_BX0_BIF_DOORBELL_INT_CNTL__TIMEOUT_ERR_EVENT_INTERRUPT_ENABLE_MASK                                0x80000000L
//BIF_BX0_BIF_FB_EN
#define BIF_BX0_BIF_FB_EN__FB_READ_EN__SHIFT                                                                  0x0
#define BIF_BX0_BIF_FB_EN__FB_WRITE_EN__SHIFT                                                                 0x1
#define BIF_BX0_BIF_FB_EN__FB_READ_EN_MASK                                                                    0x00000001L
#define BIF_BX0_BIF_FB_EN__FB_WRITE_EN_MASK                                                                   0x00000002L
//BIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL
#define BIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL__ADDRESS__SHIFT                                                      0x2
#define BIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL__ADDRESS_MASK                                                        0x0007FFFCL
//BIF_BX0_REMAP_HDP_REG_FLUSH_CNTL
#define BIF_BX0_REMAP_HDP_REG_FLUSH_CNTL__ADDRESS__SHIFT                                                      0x2
#define BIF_BX0_REMAP_HDP_REG_FLUSH_CNTL__ADDRESS_MASK                                                        0x0007FFFCL
//BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH__DOORBELL_SELFRING_GPA_APER_BASE_HIGH__SHIFT          0x0
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH__DOORBELL_SELFRING_GPA_APER_BASE_HIGH_MASK            0xFFFFFFFFL
//BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW__DOORBELL_SELFRING_GPA_APER_BASE_LOW__SHIFT            0x0
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW__DOORBELL_SELFRING_GPA_APER_BASE_LOW_MASK              0xFFFFFFFFL
//BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_EN__SHIFT                      0x0
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_MODE__SHIFT                    0x1
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_SIZE__SHIFT                    0x8
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_EN_MASK                        0x00000001L
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_MODE_MASK                      0x00000002L
#define BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL__DOORBELL_SELFRING_GPA_APER_SIZE_MASK                      0x000FFF00L
//BIF_BX_PF0_HDP_MEM_COHERENCY_FLUSH_CNTL
#define BIF_BX_PF0_HDP_MEM_COHERENCY_FLUSH_CNTL__HDP_MEM_FLUSH_ADDR__SHIFT                                    0x0
#define BIF_BX_PF0_HDP_MEM_COHERENCY_FLUSH_CNTL__HDP_MEM_FLUSH_ADDR_MASK                                      0x00000001L
//BIF_BX_PF0_GPU_HDP_FLUSH_REQ
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP0__SHIFT                                                              0x0
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP1__SHIFT                                                              0x1
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP2__SHIFT                                                              0x2
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP3__SHIFT                                                              0x3
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP4__SHIFT                                                              0x4
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP5__SHIFT                                                              0x5
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP6__SHIFT                                                              0x6
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP7__SHIFT                                                              0x7
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP8__SHIFT                                                              0x8
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP9__SHIFT                                                              0x9
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__SDMA0__SHIFT                                                            0xa
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__SDMA1__SHIFT                                                            0xb
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG0__SHIFT                                                        0xc
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG1__SHIFT                                                        0xd
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG2__SHIFT                                                        0xe
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG3__SHIFT                                                        0xf
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG4__SHIFT                                                        0x10
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG5__SHIFT                                                        0x11
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG6__SHIFT                                                        0x12
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG7__SHIFT                                                        0x13
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG8__SHIFT                                                        0x14
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG9__SHIFT                                                        0x15
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG10__SHIFT                                                       0x16
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG11__SHIFT                                                       0x17
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG12__SHIFT                                                       0x18
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG13__SHIFT                                                       0x19
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG14__SHIFT                                                       0x1a
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG15__SHIFT                                                       0x1b
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG16__SHIFT                                                       0x1c
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG17__SHIFT                                                       0x1d
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG18__SHIFT                                                       0x1e
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG19__SHIFT                                                       0x1f
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP0_MASK                                                                0x00000001L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP1_MASK                                                                0x00000002L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP2_MASK                                                                0x00000004L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP3_MASK                                                                0x00000008L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP4_MASK                                                                0x00000010L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP5_MASK                                                                0x00000020L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP6_MASK                                                                0x00000040L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP7_MASK                                                                0x00000080L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP8_MASK                                                                0x00000100L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__CP9_MASK                                                                0x00000200L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__SDMA0_MASK                                                              0x00000400L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__SDMA1_MASK                                                              0x00000800L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG0_MASK                                                          0x00001000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG1_MASK                                                          0x00002000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG2_MASK                                                          0x00004000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG3_MASK                                                          0x00008000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG4_MASK                                                          0x00010000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG5_MASK                                                          0x00020000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG6_MASK                                                          0x00040000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG7_MASK                                                          0x00080000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG8_MASK                                                          0x00100000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG9_MASK                                                          0x00200000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG10_MASK                                                         0x00400000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG11_MASK                                                         0x00800000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG12_MASK                                                         0x01000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG13_MASK                                                         0x02000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG14_MASK                                                         0x04000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG15_MASK                                                         0x08000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG16_MASK                                                         0x10000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG17_MASK                                                         0x20000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG18_MASK                                                         0x40000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_REQ__RSVD_ENG19_MASK                                                         0x80000000L
//BIF_BX_PF0_GPU_HDP_FLUSH_DONE
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP0__SHIFT                                                             0x0
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP1__SHIFT                                                             0x1
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP2__SHIFT                                                             0x2
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP3__SHIFT                                                             0x3
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP4__SHIFT                                                             0x4
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP5__SHIFT                                                             0x5
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP6__SHIFT                                                             0x6
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP7__SHIFT                                                             0x7
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP8__SHIFT                                                             0x8
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP9__SHIFT                                                             0x9
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA0__SHIFT                                                           0xa
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA1__SHIFT                                                           0xb
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG0__SHIFT                                                       0xc
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG1__SHIFT                                                       0xd
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG2__SHIFT                                                       0xe
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG3__SHIFT                                                       0xf
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG4__SHIFT                                                       0x10
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG5__SHIFT                                                       0x11
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG6__SHIFT                                                       0x12
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG7__SHIFT                                                       0x13
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG8__SHIFT                                                       0x14
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG9__SHIFT                                                       0x15
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG10__SHIFT                                                      0x16
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG11__SHIFT                                                      0x17
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG12__SHIFT                                                      0x18
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG13__SHIFT                                                      0x19
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG14__SHIFT                                                      0x1a
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG15__SHIFT                                                      0x1b
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG16__SHIFT                                                      0x1c
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG17__SHIFT                                                      0x1d
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG18__SHIFT                                                      0x1e
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG19__SHIFT                                                      0x1f
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP0_MASK                                                               0x00000001L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP1_MASK                                                               0x00000002L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP2_MASK                                                               0x00000004L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP3_MASK                                                               0x00000008L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP4_MASK                                                               0x00000010L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP5_MASK                                                               0x00000020L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP6_MASK                                                               0x00000040L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP7_MASK                                                               0x00000080L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP8_MASK                                                               0x00000100L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP9_MASK                                                               0x00000200L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA0_MASK                                                             0x00000400L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA1_MASK                                                             0x00000800L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG0_MASK                                                         0x00001000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG1_MASK                                                         0x00002000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG2_MASK                                                         0x00004000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG3_MASK                                                         0x00008000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG4_MASK                                                         0x00010000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG5_MASK                                                         0x00020000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG6_MASK                                                         0x00040000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG7_MASK                                                         0x00080000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG8_MASK                                                         0x00100000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG9_MASK                                                         0x00200000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG10_MASK                                                        0x00400000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG11_MASK                                                        0x00800000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG12_MASK                                                        0x01000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG13_MASK                                                        0x02000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG14_MASK                                                        0x04000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG15_MASK                                                        0x08000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG16_MASK                                                        0x10000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG17_MASK                                                        0x20000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG18_MASK                                                        0x40000000L
#define BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG19_MASK                                                        0x80000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL__S2A_DOORBELL_PORT0_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1__S2A_DOORBELL_PORT0_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL__S2A_DOORBELL_PORT1_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1__S2A_DOORBELL_PORT1_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL__S2A_DOORBELL_PORT2_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1__S2A_DOORBELL_PORT2_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL__S2A_DOORBELL_PORT3_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1__S2A_DOORBELL_PORT3_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL__S2A_DOORBELL_PORT4_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1__S2A_DOORBELL_PORT4_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_ENABLE__SHIFT                                  0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_AWID__SHIFT                                    0x1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_FENCE_ENABLE__SHIFT                            0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_RANGE_OFFSET__SHIFT                            0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_RANGE_SIZE__SHIFT                              0x11
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_64BIT_SUPPORT_DIS__SHIFT                       0x19
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_NEED_DEDUCT_RANGE_OFFSET__SHIFT                0x1a
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_DROP_EN__SHIFT                                 0x1b
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_AWADDR_31_28_VALUE__SHIFT                      0x1c
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_ENABLE_MASK                                    0x00000001L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_AWID_MASK                                      0x0000003EL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_FENCE_ENABLE_MASK                              0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_RANGE_OFFSET_MASK                              0x0001FF80L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_RANGE_SIZE_MASK                                0x01FE0000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_64BIT_SUPPORT_DIS_MASK                         0x02000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_NEED_DEDUCT_RANGE_OFFSET_MASK                  0x04000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_DROP_EN_MASK                                   0x08000000L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL__S2A_DOORBELL_PORT5_AWADDR_31_28_VALUE_MASK                        0xF0000000L
//GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_PORT_TYPE__SHIFT                       0x0
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_DIEID__SHIFT                           0x2
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_PORT_ID__SHIFT                         0x4
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_OVERRIDE_POISON_DATA__SHIFT                   0x6
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_4K_PAGE_DECODE_DISABLE__SHIFT                 0x7
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_PORT_TYPE_MASK                         0x00000003L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_DIEID_MASK                             0x0000000CL
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_TARGET_PORT_ID_MASK                           0x00000030L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_OVERRIDE_POISON_DATA_MASK                     0x00000040L
#define GDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1__S2A_DOORBELL_PORT5_4K_PAGE_DECODE_DISABLE_MASK                   0x00000080L
//RCC_DEV0_EPF2_STRAP2
#define RCC_DEV0_EPF2_STRAP2__STRAP_TEE_EXCLUSIVE_ATTR_SUPPORTED_DEV0_F2__SHIFT                               0x4
#define RCC_DEV0_EPF2_STRAP2__STRAP_NO_SOFT_RESET_DEV0_F2__SHIFT                                              0x7
#define RCC_DEV0_EPF2_STRAP2__STRAP_RESIZE_BAR_EN_DEV0_F2__SHIFT                                              0x8
#define RCC_DEV0_EPF2_STRAP2__STRAP_MAX_PASID_WIDTH_DEV0_F2__SHIFT                                            0x9
#define RCC_DEV0_EPF2_STRAP2__STRAP_MSI_PERVECTOR_MASK_CAP_DEV0_F2__SHIFT                                     0xe
#define RCC_DEV0_EPF2_STRAP2__STRAP_AER_EN_DEV0_F2__SHIFT                                                     0x10
#define RCC_DEV0_EPF2_STRAP2__STRAP_ACS_EN_DEV0_F2__SHIFT                                                     0x11
#define RCC_DEV0_EPF2_STRAP2__STRAP_CPL_ABORT_ERR_EN_DEV0_F2__SHIFT                                           0x14
#define RCC_DEV0_EPF2_STRAP2__STRAP_DPA_EN_DEV0_F2__SHIFT                                                     0x15
#define RCC_DEV0_EPF2_STRAP2__STRAP_MSI_MULTI_CAP_DEV0_F2__SHIFT                                              0x18
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_EN_DEV0_F2__SHIFT                                                   0x1c
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_EXE_PERMISSION_SUPPORTED_DEV0_F2__SHIFT                             0x1d
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_GLOBAL_INVALIDATE_SUPPORTED_DEV0_F2__SHIFT                          0x1e
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_PRIV_MODE_SUPPORTED_DEV0_F2__SHIFT                                  0x1f
#define RCC_DEV0_EPF2_STRAP2__STRAP_TEE_EXCLUSIVE_ATTR_SUPPORTED_DEV0_F2_MASK                                 0x00000010L
#define RCC_DEV0_EPF2_STRAP2__STRAP_NO_SOFT_RESET_DEV0_F2_MASK                                                0x00000080L
#define RCC_DEV0_EPF2_STRAP2__STRAP_RESIZE_BAR_EN_DEV0_F2_MASK                                                0x00000100L
#define RCC_DEV0_EPF2_STRAP2__STRAP_MAX_PASID_WIDTH_DEV0_F2_MASK                                              0x00003E00L
#define RCC_DEV0_EPF2_STRAP2__STRAP_MSI_PERVECTOR_MASK_CAP_DEV0_F2_MASK                                       0x00004000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_AER_EN_DEV0_F2_MASK                                                       0x00010000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_ACS_EN_DEV0_F2_MASK                                                       0x00020000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_CPL_ABORT_ERR_EN_DEV0_F2_MASK                                             0x00100000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_DPA_EN_DEV0_F2_MASK                                                       0x00200000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_MSI_MULTI_CAP_DEV0_F2_MASK                                                0x07000000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_EN_DEV0_F2_MASK                                                     0x10000000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_EXE_PERMISSION_SUPPORTED_DEV0_F2_MASK                               0x20000000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_GLOBAL_INVALIDATE_SUPPORTED_DEV0_F2_MASK                            0x40000000L
#define RCC_DEV0_EPF2_STRAP2__STRAP_PASID_PRIV_MODE_SUPPORTED_DEV0_F2_MASK                                    0x80000000L
//REGS_ROM_OFFSET_CTRL
#define REGS_ROM_OFFSET_CTRL__ROM_OFFSET__SHIFT                                                               0x0
#define REGS_ROM_OFFSET_CTRL__ROM_OFFSET_MASK                                                                 0x7FL
//RCC_DEV0_EPF0_1_RCC_CONFIG_MEMSIZE
#define RCC_DEV0_EPF0_1_RCC_CONFIG_MEMSIZE__CONFIG_MEMSIZE__SHIFT                                             0x0
#define RCC_DEV0_EPF0_1_RCC_CONFIG_MEMSIZE__CONFIG_MEMSIZE_MASK                                               0xFFFFFFFFL

#endif
