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
#ifndef _nbif_4_10_0_OFFSET_HEADER
#define _nbif_4_10_0_OFFSET_HEADER

#define regBIF_BX_PF0_RSMU_INDEX                                                                        0x0000
#define regBIF_BX_PF0_RSMU_INDEX_BASE_IDX                                                               1
#define regBIF_BX_PF0_RSMU_DATA                                                                         0x0001
#define regBIF_BX_PF0_RSMU_DATA_BASE_IDX                                                                1
#define regBIF_BX0_PCIE_INDEX                                                                           0x000c
#define regBIF_BX0_PCIE_INDEX_BASE_IDX                                                                  0
#define regBIF_BX0_PCIE_DATA                                                                            0x000d
#define regBIF_BX0_PCIE_DATA_BASE_IDX                                                                   0
#define regRCC_STRAP0_RCC_BIF_STRAP2                                                                    0x0006
#define regRCC_STRAP0_RCC_BIF_STRAP2_BASE_IDX                                                           2
#define regRCC_STRAP0_RCC_BIF_STRAP3                                                                    0x0007
#define regRCC_STRAP0_RCC_BIF_STRAP3_BASE_IDX                                                           2
#define regRCC_STRAP0_RCC_BIF_STRAP5                                                                    0x0009
#define regRCC_STRAP0_RCC_BIF_STRAP5_BASE_IDX                                                           2
#define regRCC_STRAP0_RCC_DEV0_EPF0_STRAP0                                                              0x0021
#define regRCC_STRAP0_RCC_DEV0_EPF0_STRAP0_BASE_IDX                                                     2
#define regRCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL                                                            0x0054
#define regRCC_EP_DEV0_0_EP_PCIE_TX_LTR_CNTL_BASE_IDX                                                   2
#define regRCC_DEV0_EPF0_RCC_DOORBELL_APER_EN                                                           0x00c0
#define regRCC_DEV0_EPF0_RCC_DOORBELL_APER_EN_BASE_IDX                                                  2
#define regRCC_DEV0_EPF0_0_RCC_CONFIG_MEMSIZE                                                           0x00c3
#define regRCC_DEV0_EPF0_0_RCC_CONFIG_MEMSIZE_BASE_IDX                                                  2
#define regBIF_BX0_INTERRUPT_CNTL                                                                       0x00f1
#define regBIF_BX0_INTERRUPT_CNTL_BASE_IDX                                                              2
#define regBIF_BX0_INTERRUPT_CNTL2                                                                      0x00f2
#define regBIF_BX0_INTERRUPT_CNTL2_BASE_IDX                                                             2
#define regBIF_BX0_BIF_DOORBELL_INT_CNTL                                                                0x00fe
#define regBIF_BX0_BIF_DOORBELL_INT_CNTL_BASE_IDX                                                       2
#define regBIF_BX0_BIF_FB_EN                                                                            0x0100
#define regBIF_BX0_BIF_FB_EN_BASE_IDX                                                                   2
#define regBIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL                                                             0x012d
#define regBIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL_BASE_IDX                                                    2
#define regBIF_BX0_REMAP_HDP_REG_FLUSH_CNTL                                                             0x012e
#define regBIF_BX0_REMAP_HDP_REG_FLUSH_CNTL_BASE_IDX                                                    2
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH                                              0x00f3
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH_BASE_IDX                                     2
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW                                               0x00f4
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW_BASE_IDX                                      2
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL                                                   0x00f5
#define regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL_BASE_IDX                                          2
#define regBIF_BX_PF0_HDP_MEM_COHERENCY_FLUSH_CNTL                                                      0x00f7
#define regBIF_BX_PF0_HDP_MEM_COHERENCY_FLUSH_CNTL_BASE_IDX                                             2
#define regBIF_BX_PF0_GPU_HDP_FLUSH_REQ                                                                 0x0106
#define regBIF_BX_PF0_GPU_HDP_FLUSH_REQ_BASE_IDX                                                        2
#define regBIF_BX_PF0_GPU_HDP_FLUSH_DONE                                                                0x0107
#define regBIF_BX_PF0_GPU_HDP_FLUSH_DONE_BASE_IDX                                                       2
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL                                                           0x4f0aeb
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1                                                          0x4f0aec
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1_BASE_IDX                                                 3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL                                                           0x4f0aed
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1                                                          0x4f0aee
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_1_CTRL1_BASE_IDX                                                 3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL                                                           0x4f0aef
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1                                                          0x4f0af0
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_2_CTRL1_BASE_IDX                                                 3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL                                                           0x4f0af1
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1                                                          0x4f0af2
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_3_CTRL1_BASE_IDX                                                 3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL                                                           0x4f0af3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1                                                          0x4f0af4
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_4_CTRL1_BASE_IDX                                                 3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL                                                           0x4f0af5
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL_BASE_IDX                                                  3
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1                                                          0x4f0af6
#define regGDC_S2A0_S2A_DOORBELL_ENTRY_5_CTRL1_BASE_IDX                                                 3
#define regRCC_DEV0_EPF2_STRAP2                                                                         0xd102
#define regRCC_DEV0_EPF2_STRAP2_BASE_IDX                                                                5
#define regREGS_ROM_OFFSET_CTRL                                                                         0xcc23
#define regREGS_ROM_OFFSET_CTRL_BASE_IDX                                                                5
#define regRCC_DEV0_EPF0_1_RCC_CONFIG_MEMSIZE                                                           0x2ffc0de3
#define regRCC_DEV0_EPF0_1_RCC_CONFIG_MEMSIZE_BASE_IDX                                                  5

#endif
