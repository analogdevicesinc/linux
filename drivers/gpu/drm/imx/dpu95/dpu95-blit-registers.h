/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2023 NXP
 */

#ifndef __DPU95_BLIT_REGISTERS_H__
#define __DPU95_BLIT_REGISTERS_H__

/* Registers defination */
#define COMCTRL_IPIDENTIFIER                       ((uint32_t)(0))

/* Register for interrupts in Common Control domain */
#define COMCTRL_INTERRUPTENABLE0                    ((uint32_t)(0x1008))
#define COMCTRL_INTERRUPTPRESET0                    ((uint32_t)(0x1014))
#define COMCTRL_INTERRUPTPRESET1                    ((uint32_t)(0x1018))
#define COMCTRL_INTERRUPTCLEAR0                     ((uint32_t)(0x1020))
#define COMCTRL_INTERRUPTSTATUS0                    ((uint32_t)(0x102c))

/* Register for interrupts in CMDSEQ domain */
#define CMDSEQ_INTERRUPTCLEAR0                     ((uint32_t)(0x11020))

/* Register for pixel engine path configuration */
#define PIXENGCFG_FETCHDECODE9_DYNAMIC              ((uint32_t)(0x91008))
#define PIXENGCFG_FETCHDECODE9_DYNAMIC_RESET_VALUE  0U

#define PIXENGCFG_FETCHROT9_DYNAMIC                ((uint32_t)(0x86008))
#define PIXENGCFG_FETCHROT9_DYNAMIC_RESET_VALUE    0U

#define PIXENGCFG_ROP9_DYNAMIC                      ((uint32_t)(0x41008))
#define PIXENGCFG_ROP9_DYNAMIC_RESET_VALUE          0x1000000U

#define PIXENGCFG_MATRIX9_DYNAMIC                   ((uint32_t)(0x61008))
#define PIXENGCFG_MATRIX9_DYNAMIC_RESET_VALUE       0x1000000U

#define PIXENGCFG_HSCALER9_DYNAMIC                  ((uint32_t)(0xb1008))
#define PIXENGCFG_HSCALER9_DYNAMIC_RESET_VALUE      0x1000000U

#define PIXENGCFG_VSCALER9_DYNAMIC                  ((uint32_t)(0xc1008))
#define PIXENGCFG_VSCALER9_DYNAMIC_RESET_VALUE      0x1000000U

#define PIXENGCFG_BLITBLEND9_DYNAMIC                ((uint32_t)(0x71008))
#define PIXENGCFG_BLITBLEND9_DYNAMIC_RESET_VALUE    0x1000000U

#define PIXENGCFG_STORE9_STATIC                     ((uint32_t)(0xe1008))
#define PIXENGCFG_STORE9_STATIC_RESET_VALUE         0x800010U
#define PIXENGCFG_STORE9_STATIC_RESET_MASK          0xFFFFFFFFU
#define PIXENGCFG_STORE9_STATIC_STORE9_SHDEN_MASK   0x1U
#define PIXENGCFG_STORE9_STATIC_STORE9_SHDEN_SHIFT  0U
#define PIXENGCFG_STORE9_STATIC_STORE9_POWERDOWN_MASK 0x10U
#define PIXENGCFG_STORE9_STATIC_STORE9_POWERDOWN_SHIFT 4U
#define PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE_MASK 0x100U
#define PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE_SHIFT 8U
#define PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE__SINGLE 0U
#define PIXENGCFG_STORE9_STATIC_STORE9_SYNC_MODE__AUTO 0x1U
#define PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET_MASK 0x800U
#define PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET_SHIFT 11U
/* Field Value: STORE9_SW_RESET__OPERATION, Normal Operation  */
#define PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET__OPERATION 0U
/* Field Value: STORE9_SW_RESET__SWRESET, Software Reset  */
#define PIXENGCFG_STORE9_STATIC_STORE9_SW_RESET__SWRESET 0x1U
#define PIXENGCFG_STORE9_STATIC_STORE9_DIV_MASK     0xFF0000U
#define PIXENGCFG_STORE9_STATIC_STORE9_DIV_SHIFT    16U
#define PIXENGCFG_STORE9_DYNAMIC                    ((uint32_t)(0xe100c))
#define PIXENGCFG_STORE9_TRIGGER                    ((uint32_t)(0xe1014))

/* pixengcfg */
#define PIXENGCFG_CLKEN_MASK 0x3000000U
#define PIXENGCFG_CLKEN_SHIFT 24U
/* Field Value: _CLKEN__DISABLE, Clock for block is disabled  */
#define PIXENGCFG_CLKEN__DISABLE 0U
#define PIXENGCFG_CLKEN__AUTOMATIC 0x1U
/* Field Value: _CLKEN__FULL, Clock for block is without gating  */
#define PIXENGCFG_CLKEN__FULL 0x3U

#define PIXENGCFG_DIVIDER_RESET 0x80

/* Register for different units */
#define FETCHDECODE9_STATICCONTROL                  ((uint32_t)(0x90008))
#define FETCHDECODE9_STATICCONTROL_OFFSET           ((uint32_t)(0x8))
#define FETCHDECODE9_STATICCONTROL_RESET_VALUE      0U
#define FETCHDECODE9_STATICCONTROL_SHDEN_MASK       0x1U
#define FETCHDECODE9_STATICCONTROL_SHDEN_SHIFT      0U

#define FETCHDECODE9_BURSTBUFFERMANAGEMENT          ((uint32_t)(0x9000c))
#define FETCHDECODE9_BASEADDRESS0                   ((uint32_t)(0x90028))
#define FETCHDECODE9_BASEADDRESSMSB0                ((uint32_t)(0x9002c))
#define FETCHDECODE9_SOURCEBUFFERATTRIBUTES0        ((uint32_t)(0x90038))
#define FETCHDECODE9_SOURCEBUFFERDIMENSION0         ((uint32_t)(0x9003c))
#define FETCHDECODE9_COLORCOMPONENTBITS0            ((uint32_t)(0x90040))
#define FETCHDECODE9_COLORCOMPONENTSHIFT0           ((uint32_t)(0x90044))
#define FETCHDECODE9_LAYEROFFSET0                   ((uint32_t)(0x90048))
#define FETCHDECODE9_CLIPWINDOWOFFSET0              ((uint32_t)(0x9004c))
#define FETCHDECODE9_CLIPWINDOWDIMENSIONS0          ((uint32_t)(0x90050))
#define FETCHDECODE9_CONSTANTCOLOR0                 ((uint32_t)(0x90054))
#define FETCHDECODE9_LAYERPROPERTY0                 ((uint32_t)(0x90058))
#define FETCHDECODE9_FRAMEDIMENSIONS                ((uint32_t)(0x90060))
#define FETCHDECODE9_FRAMERESAMPLING                ((uint32_t)(0x90064))
#define FETCHDECODE9_CONTROL                        ((uint32_t)(0x90070))

#define FETCHROT9_STATICCONTROL                    ((uint32_t)(0x80008))
#define FETCHROT9_STATICCONTROL_OFFSET             ((uint32_t)(0x8))
#define FETCHROT9_STATICCONTROL_RESET_VALUE        0U
#define FETCHROT9_STATICCONTROL_RESET_MASK         0xFFFFFFFFU
#define FETCHROT9_STATICCONTROL_SHDEN_MASK         0x1U
#define FETCHROT9_STATICCONTROL_SHDEN_SHIFT        0U
#define FETCHROT9_STATICCONTROL_BASEADDRESSSELECT_MASK 0x10U
#define FETCHROT9_STATICCONTROL_BASEADDRESSSELECT_SHIFT 4U

#define FETCHROT9_BURSTBUFFERMANAGEMENT            ((uint32_t)(0x8000c))
#define FETCHROT9_BASEADDRESS0                     ((uint32_t)(0x80020))
#define FETCHROT9_BASEADDRESSMSB0                  ((uint32_t)(0x80024))
#define FETCHROT9_SOURCEBUFFERATTRIBUTES0          ((uint32_t)(0x80030))
#define FETCHROT9_SOURCEBUFFERDIMENSION0           ((uint32_t)(0x80034))
#define FETCHROT9_COLORCOMPONENTBITS0              ((uint32_t)(0x80038))
#define FETCHROT9_COLORCOMPONENTSHIFT0             ((uint32_t)(0x8003c))
#define FETCHROT9_LAYEROFFSET0                     ((uint32_t)(0x80040))
#define FETCHROT9_CLIPWINDOWOFFSET0                ((uint32_t)(0x80044))
#define FETCHROT9_CLIPWINDOWDIMENSIONS0            ((uint32_t)(0x80048))
#define FETCHROT9_CONSTANTCOLOR0                   ((uint32_t)(0x8004c))
#define FETCHROT9_LAYERPROPERTY0                   ((uint32_t)(0x80050))
#define FETCHROT9_FRAMEDIMENSIONS                  ((uint32_t)(0x80058))
#define FETCHROT9_FRAMERESAMPLING                  ((uint32_t)(0x8005c))
#define FETCHROT9_CONTROL                          ((uint32_t)(0x80090))


#define FETCHECO9_STATICCONTROL                     ((uint32_t)(0xa0008))
#define FETCHECO9_STATICCONTROL_OFFSET              ((uint32_t)(0x8))
#define FETCHECO9_STATICCONTROL_RESET_VALUE         0U
#define FETCHECO9_STATICCONTROL_RESET_MASK          0xFFFFFFFFU
#define FETCHECO9_STATICCONTROL_SHDEN_MASK          0x1U
#define FETCHECO9_STATICCONTROL_SHDEN_SHIFT         0U

#define FETCHECO9_BURSTBUFFERMANAGEMENT             ((uint32_t)(0xa000c))
#define FETCHECO9_BASEADDRESS0                      ((uint32_t)(0xa0010))
#define FETCHECO9_BASEADDRESSMSB0                      ((uint32_t)(0xa0014))
#define FETCHECO9_SOURCEBUFFERATTRIBUTES0           ((uint32_t)(0xa0020))
#define FETCHECO9_SOURCEBUFFERDIMENSION0            ((uint32_t)(0xa0024))
#define FETCHECO9_COLORCOMPONENTBITS0               ((uint32_t)(0xa0028))
#define FETCHECO9_COLORCOMPONENTSHIFT0              ((uint32_t)(0xa002c))
#define FETCHECO9_LAYEROFFSET0                      ((uint32_t)(0xa0030))
#define FETCHECO9_CLIPWINDOWOFFSET0                 ((uint32_t)(0xa0034))
#define FETCHECO9_CLIPWINDOWDIMENSIONS0             ((uint32_t)(0xa0038))
#define FETCHECO9_CONSTANTCOLOR0                    ((uint32_t)(0xa003c))
#define FETCHECO9_LAYERPROPERTY0                    ((uint32_t)(0xa0040))
#define FETCHECO9_FRAMEDIMENSIONS                   ((uint32_t)(0xa0048))
#define FETCHECO9_FRAMERESAMPLING                   ((uint32_t)(0xa004c))
#define FETCHECO9_CONTROL                           ((uint32_t)(0xa0050))


#define ROP9_STATICCONTROL                          ((uint32_t)(0x40008))
#define ROP9_STATICCONTROL_OFFSET                   ((uint32_t)(0x8))
#define ROP9_STATICCONTROL_RESET_VALUE              0U
#define ROP9_STATICCONTROL_RESET_MASK               0xFFFFFFFFU
#define ROP9_STATICCONTROL_SHDEN_MASK               0x1U
#define ROP9_STATICCONTROL_SHDEN_SHIFT              0U
#define ROP9_CONTROL                                ((uint32_t)(0x4000c))


#define MATRIX9_STATICCONTROL                       ((uint32_t)(0x60008))
#define MATRIX9_STATICCONTROL_OFFSET                ((uint32_t)(0x8))
#define MATRIX9_STATICCONTROL_RESET_VALUE           0U
#define MATRIX9_STATICCONTROL_RESET_MASK            0xFFFFFFFFU
#define MATRIX9_STATICCONTROL_SHDEN_MASK            0x1U
#define MATRIX9_STATICCONTROL_SHDEN_SHIFT           0U

#define MATRIX9_CONTROL                             ((uint32_t)(0x6000c))


#define VSCALER9_STATICCONTROL                      ((uint32_t)(0xc0008))
#define VSCALER9_STATICCONTROL_OFFSET               ((uint32_t)(0x8))
#define VSCALER9_STATICCONTROL_RESET_VALUE          0U
#define VSCALER9_STATICCONTROL_RESET_MASK           0xFFFFFFFFU
#define VSCALER9_STATICCONTROL_SHDEN_MASK           0x1U
#define VSCALER9_STATICCONTROL_SHDEN_SHIFT          0U

#define VSCALER9_SETUP1                             ((uint32_t)(0xc000c))
#define VSCALER9_SETUP2                             ((uint32_t)(0xc0010))
#define VSCALER9_CONTROL                            ((uint32_t)(0xc0014))


#define HSCALER9_STATICCONTROL                      ((uint32_t)(0xb0008))
#define HSCALER9_STATICCONTROL_OFFSET               ((uint32_t)(0x8))
#define HSCALER9_STATICCONTROL_RESET_VALUE          0U
#define HSCALER9_STATICCONTROL_RESET_MASK           0xFFFFFFFFU
#define HSCALER9_STATICCONTROL_SHDEN_MASK           0x1U
#define HSCALER9_STATICCONTROL_SHDEN_SHIFT          0U

#define HSCALER9_SETUP1                             ((uint32_t)(0xb000c))
#define HSCALER9_SETUP2                             ((uint32_t)(0xb0010))
#define HSCALER9_CONTROL                            ((uint32_t)(0xb0014))


#define BLITBLEND9_STATICCONTROL                    ((uint32_t)(0x70008))
#define BLITBLEND9_STATICCONTROL_OFFSET             ((uint32_t)(0x8))
#define BLITBLEND9_STATICCONTROL_RESET_VALUE        0U
#define BLITBLEND9_STATICCONTROL_RESET_MASK         0xFFFFFFFFU
#define BLITBLEND9_STATICCONTROL_SHDEN_MASK         0x1U
#define BLITBLEND9_STATICCONTROL_SHDEN_SHIFT        0U

#define BLITBLEND9_CONTROL                          ((uint32_t)(0x7000c))
#define BLITBLEND9_CONSTANTCOLOR                    ((uint32_t)(0x70014))
#define BLITBLEND9_COLORREDBLENDFUNCTION            ((uint32_t)(0x70018))
#define BLITBLEND9_COLORGREENBLENDFUNCTION          ((uint32_t)(0x7001c))
#define BLITBLEND9_COLORBLUEBLENDFUNCTION           ((uint32_t)(0x70020))
#define BLITBLEND9_ALPHABLENDFUNCTION               ((uint32_t)(0x70024))
#define BLITBLEND9_BLENDMODE1                       ((uint32_t)(0x70028))
#define BLITBLEND9_BLENDMODE2                       ((uint32_t)(0x7002c))


#define STORE9_STATICCONTROL                        ((uint32_t)(0xe0008))
#define STORE9_STATICCONTROL_OFFSET                 ((uint32_t)(0x8))
#define STORE9_STATICCONTROL_RESET_VALUE            0U
#define STORE9_STATICCONTROL_RESET_MASK             0xFFFFFFFFU
#define STORE9_STATICCONTROL_SHDEN_MASK             0x1U
#define STORE9_STATICCONTROL_SHDEN_SHIFT            0U
#define STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_MASK 0x100U
#define STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT 8U

#define STORE9_BURSTBUFFERMANAGEMENT                ((uint32_t)(0xe000c))
#define STORE9_BASEADDRESS0                         ((uint32_t)(0xe0020))
#define STORE9_BASEADDRESSMSB0                      ((uint32_t)(0xe0024))
#define STORE9_DESTINATIONBUFFERATTRIBUTES0         ((uint32_t)(0xe0030))
#define STORE9_BASEADDRESS1                         ((uint32_t)(0xe0038))
#define STORE9_BASEADDRESSMSB1                      ((uint32_t)(0xe003c))
#define STORE9_DESTINATIONBUFFERATTRIBUTES1         ((uint32_t)(0xe0040))
#define STORE9_BASEADDRESS2                         ((uint32_t)(0xe0048))
#define STORE9_BASEADDRESSMSB2                      ((uint32_t)(0xe004c))
#define STORE9_DESTINATIONBUFFERATTRIBUTES2         ((uint32_t)(0xe0050))
#define STORE9_DESTINATIONBUFFERDIMENSION           ((uint32_t)(0xe0054))
#define STORE9_FRAMEOFFSET                          ((uint32_t)(0xe0058))
#define STORE9_COLORCOMPONENTBITS                   ((uint32_t)(0xe005c))
#define STORE9_COLORCOMPONENTSHIFT                  ((uint32_t)(0xe0060))
#define STORE9_CONTROL                              ((uint32_t)(0xe0064))
#define STORE9_START                                ((uint32_t)(0xe0068))


/* Register for command sequencer */
#define CMDSEQ_HIF                                  ((uint32_t)(0x10000))

#define CMDSEQ_LOCKUNLOCKHIF                        ((uint32_t)(0x10100))
#define CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__LOCK_KEY 0x5651F763U
#define CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__UNLOCK_KEY 0x691DB936U

#define CMDSEQ_LOCKUNLOCK                           ((uint32_t)(0x10180))
#define CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__LOCK_KEY      0x5651F763U
#define CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__UNLOCK_KEY    0x691DB936U

#define CMDSEQ_BUFFERADDRESS                        ((uint32_t)(0x10188))
#define CMDSEQ_BUFFERSIZE                           ((uint32_t)(0x10190))

#define CMDSEQ_CONTROL                              ((uint32_t)(0x10198))
#define CMDSEQ_CONTROL_OFFSET                       ((uint32_t)(0x198))
#define CMDSEQ_CONTROL_RESET_VALUE                  0U
#define CMDSEQ_CONTROL_RESET_MASK                   0xFFFFFFFFU
#define CMDSEQ_CONTROL_CLRAXIW_MASK                 0x1U
#define CMDSEQ_CONTROL_CLRAXIW_SHIFT                0U
#define CMDSEQ_CONTROL_CLRRBUF_MASK                 0x4U
#define CMDSEQ_CONTROL_CLRRBUF_SHIFT                2U
#define CMDSEQ_CONTROL_CLRCMDBUF_MASK               0x8U
#define CMDSEQ_CONTROL_CLRCMDBUF_SHIFT              3U
#define CMDSEQ_CONTROL_CLEAR_MASK                   0x10U
#define CMDSEQ_CONTROL_CLEAR_SHIFT                  4U

#define CMDSEQ_STATUS                               ((uint32_t)(0x1019c))
#define CMDSEQ_STATUS_OFFSET                        ((uint32_t)(0x19c))
#define CMDSEQ_STATUS_RESET_VALUE                   0x41000080U
#define CMDSEQ_STATUS_RESET_MASK                    0xFFFFFFFFU
#define CMDSEQ_STATUS_FIFOSPACE_MASK                0x1FFFFU
#define CMDSEQ_STATUS_IDLE_MASK                     0x40000000U

#endif
