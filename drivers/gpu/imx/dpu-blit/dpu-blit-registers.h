/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __DPU_BLIT_REGISTERS_H__
#define __DPU_BLIT_REGISTERS_H__

/* Registers Defination */
#define COMCTRL_IPIDENTIFIER                       ((uint32_t)(0))

#define PIXENGCFG_STORE9_TRIGGER                    ((uint32_t)(0x954))

#define COMCTRL_USERINTERRUPTMASK0                  ((uint32_t)(0x48))
#define COMCTRL_USERINTERRUPTMASK0_USERINTERRUPTMASK0_MASK 0xFFFFFFFFU
#define COMCTRL_USERINTERRUPTENABLE0                ((uint32_t)(0x80))

#define COMCTRL_INTERRUPTENABLE0                    ((uint32_t)(0x50))

#define COMCTRL_INTERRUPTSTATUS0                    ((uint32_t)(0x68))
#define COMCTRL_USERINTERRUPTSTATUS0                ((uint32_t)(0x98))

#define COMCTRL_USERINTERRUPTCLEAR0                 ((uint32_t)(0x90))
#define COMCTRL_USERINTERRUPTCLEAR0_USERINTERRUPTCLEAR0_MASK 0xFFFFFFFFU

#define COMCTRL_INTERRUPTCLEAR0                     ((uint32_t)(0x60))
#define COMCTRL_INTERRUPTCLEAR0_INTERRUPTCLEAR0_MASK 0xFFFFFFFFU


#define PIXENGCFG_FETCHDECODE9_DYNAMIC              ((uint32_t)(0x828))
#define PIXENGCFG_FETCHDECODE9_DYNAMIC_RESET_VALUE  0U

#define PIXENGCFG_FETCHWARP9_DYNAMIC                ((uint32_t)(0x848))
#define PIXENGCFG_FETCHWARP9_DYNAMIC_RESET_VALUE    0U

#define PIXENGCFG_ROP9_DYNAMIC                      ((uint32_t)(0x868))
#define PIXENGCFG_ROP9_DYNAMIC_RESET_VALUE          0x1000000U

#define PIXENGCFG_MATRIX9_DYNAMIC                   ((uint32_t)(0x8A8))
#define PIXENGCFG_MATRIX9_DYNAMIC_RESET_VALUE       0x1000000U

#define PIXENGCFG_HSCALER9_DYNAMIC                  ((uint32_t)(0x8C8))
#define PIXENGCFG_HSCALER9_DYNAMIC_RESET_VALUE      0x1000000U

#define PIXENGCFG_VSCALER9_DYNAMIC                  ((uint32_t)(0x8E8))
#define PIXENGCFG_VSCALER9_DYNAMIC_RESET_VALUE      0x1000000U

#define PIXENGCFG_BLITBLEND9_DYNAMIC                ((uint32_t)(0x928))
#define PIXENGCFG_BLITBLEND9_DYNAMIC_RESET_VALUE    0x1000000U

#define PIXENGCFG_STORE9_STATIC                     ((uint32_t)(0x948))
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

#define PIXENGCFG_STORE9_DYNAMIC                    ((uint32_t)(0x94C))

#define FETCHDECODE9_STATICCONTROL                  ((uint32_t)(0x1008))
#define FETCHDECODE9_STATICCONTROL_OFFSET           ((uint32_t)(0x8))
#define FETCHDECODE9_STATICCONTROL_RESET_VALUE      0U
#define FETCHDECODE9_STATICCONTROL_SHDEN_MASK       0x1U
#define FETCHDECODE9_STATICCONTROL_SHDEN_SHIFT      0U
#define FETCHDECODE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_MASK 0xFF0000U
#define FETCHDECODE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT 16U

#define FETCHDECODE9_BURSTBUFFERMANAGEMENT          ((uint32_t)(0x100C))
#define FETCHDECODE9_BASEADDRESS0                   ((uint32_t)(0x101C))
#define FETCHDECODE9_SOURCEBUFFERATTRIBUTES0        ((uint32_t)(0x1020))
#define FETCHDECODE9_SOURCEBUFFERDIMENSION0         ((uint32_t)(0x1024))
#define FETCHDECODE9_COLORCOMPONENTBITS0            ((uint32_t)(0x1028))
#define FETCHDECODE9_COLORCOMPONENTSHIFT0           ((uint32_t)(0x102C))
#define FETCHDECODE9_LAYEROFFSET0                   ((uint32_t)(0x1030))
#define FETCHDECODE9_CLIPWINDOWOFFSET0              ((uint32_t)(0x1034))
#define FETCHDECODE9_CLIPWINDOWDIMENSIONS0          ((uint32_t)(0x1038))
#define FETCHDECODE9_CONSTANTCOLOR0                 ((uint32_t)(0x103C))
#define FETCHDECODE9_LAYERPROPERTY0                 ((uint32_t)(0x1040))
#define FETCHDECODE9_FRAMEDIMENSIONS                ((uint32_t)(0x1044))
#define FETCHDECODE9_FRAMERESAMPLING                ((uint32_t)(0x1048))
#define FETCHDECODE9_CONTROL			    ((uint32_t)(0x1054))

#define FETCHWARP9_STATICCONTROL                    ((uint32_t)(0x1808))
#define FETCHWARP9_STATICCONTROL_OFFSET             ((uint32_t)(0x8))
#define FETCHWARP9_STATICCONTROL_RESET_VALUE        0xFF000000U
#define FETCHWARP9_STATICCONTROL_RESET_MASK         0xFFFFFFFFU
#define FETCHWARP9_STATICCONTROL_SHDEN_MASK         0x1U
#define FETCHWARP9_STATICCONTROL_SHDEN_SHIFT        0U
#define FETCHWARP9_STATICCONTROL_BASEADDRESSAUTOUPDATE_MASK 0xFF0000U
#define FETCHWARP9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT 16U
#define FETCHWARP9_STATICCONTROL_SHDLDREQSTICKY_MASK 0xFF000000U
#define FETCHWARP9_STATICCONTROL_SHDLDREQSTICKY_SHIFT 24U

#define FETCHWARP9_BURSTBUFFERMANAGEMENT            ((uint32_t)(0x180C))
#define FETCHWARP9_BASEADDRESS0                     ((uint32_t)(0x1810))
#define FETCHWARP9_SOURCEBUFFERATTRIBUTES0          ((uint32_t)(0x1814))
#define FETCHWARP9_SOURCEBUFFERDIMENSION0           ((uint32_t)(0x1818))
#define FETCHWARP9_COLORCOMPONENTBITS0              ((uint32_t)(0x181C))
#define FETCHWARP9_COLORCOMPONENTSHIFT0             ((uint32_t)(0x1820))
#define FETCHWARP9_LAYEROFFSET0                     ((uint32_t)(0x1824))
#define FETCHWARP9_CLIPWINDOWOFFSET0                ((uint32_t)(0x1828))
#define FETCHWARP9_CLIPWINDOWDIMENSIONS0            ((uint32_t)(0x182C))
#define FETCHWARP9_CONSTANTCOLOR0                   ((uint32_t)(0x1830))
#define FETCHWARP9_LAYERPROPERTY0                   ((uint32_t)(0x1834))
#define FETCHWARP9_FRAMEDIMENSIONS                  ((uint32_t)(0x1950))
#define FETCHWARP9_FRAMERESAMPLING                  ((uint32_t)(0x1954))
#define FETCHWARP9_CONTROL                          ((uint32_t)(0x1970))


#define FETCHECO9_STATICCONTROL                     ((uint32_t)(0x1C08))
#define FETCHECO9_STATICCONTROL_OFFSET              ((uint32_t)(0x8))
#define FETCHECO9_STATICCONTROL_RESET_VALUE         0U
#define FETCHECO9_STATICCONTROL_RESET_MASK          0xFFFFFFFFU
#define FETCHECO9_STATICCONTROL_SHDEN_MASK          0x1U
#define FETCHECO9_STATICCONTROL_SHDEN_SHIFT         0U
#define FETCHECO9_STATICCONTROL_BASEADDRESSAUTOUPDATE_MASK 0xFF0000U
#define FETCHECO9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT 16U

#define FETCHECO9_BURSTBUFFERMANAGEMENT             ((uint32_t)(0x1C0C))
#define FETCHECO9_BASEADDRESS0                      ((uint32_t)(0x1C10))
#define FETCHECO9_SOURCEBUFFERATTRIBUTES0           ((uint32_t)(0x1C14))
#define FETCHECO9_SOURCEBUFFERDIMENSION0            ((uint32_t)(0x1C18))
#define FETCHECO9_COLORCOMPONENTBITS0               ((uint32_t)(0x1C1C))
#define FETCHECO9_COLORCOMPONENTSHIFT0              ((uint32_t)(0x1C20))
#define FETCHECO9_LAYEROFFSET0                      ((uint32_t)(0x1C24))
#define FETCHECO9_CLIPWINDOWOFFSET0                 ((uint32_t)(0x1C28))
#define FETCHECO9_CLIPWINDOWDIMENSIONS0             ((uint32_t)(0x1C2C))
#define FETCHECO9_CONSTANTCOLOR0                    ((uint32_t)(0x1C30))
#define FETCHECO9_LAYERPROPERTY0                    ((uint32_t)(0x1C34))
#define FETCHECO9_FRAMEDIMENSIONS                   ((uint32_t)(0x1C38))
#define FETCHECO9_FRAMERESAMPLING                   ((uint32_t)(0x1C3C))
#define FETCHECO9_CONTROL                           ((uint32_t)(0x1C40))


#define ROP9_STATICCONTROL                          ((uint32_t)(0x2008))
#define ROP9_STATICCONTROL_OFFSET                   ((uint32_t)(0x8))
#define ROP9_STATICCONTROL_RESET_VALUE              0U
#define ROP9_STATICCONTROL_RESET_MASK               0xFFFFFFFFU
#define ROP9_STATICCONTROL_SHDEN_MASK               0x1U
#define ROP9_STATICCONTROL_SHDEN_SHIFT              0U

#define ROP9_CONTROL                                ((uint32_t)(0x200C))

#define MATRIX9_STATICCONTROL                       ((uint32_t)(0x2C08))
#define MATRIX9_STATICCONTROL_OFFSET                ((uint32_t)(0x8))
#define MATRIX9_STATICCONTROL_RESET_VALUE           0U
#define MATRIX9_STATICCONTROL_RESET_MASK            0xFFFFFFFFU
#define MATRIX9_STATICCONTROL_SHDEN_MASK            0x1U
#define MATRIX9_STATICCONTROL_SHDEN_SHIFT           0U

#define MATRIX9_CONTROL                             ((uint32_t)(0x2C0C))

#define HSCALER9_SETUP1                             ((uint32_t)(0x300C))
#define HSCALER9_SETUP2                             ((uint32_t)(0x3010))
#define HSCALER9_CONTROL                            ((uint32_t)(0x3014))

#define VSCALER9_STATICCONTROL                      ((uint32_t)(0x3408))
#define VSCALER9_STATICCONTROL_OFFSET               ((uint32_t)(0x8))
#define VSCALER9_STATICCONTROL_RESET_VALUE          0U
#define VSCALER9_STATICCONTROL_RESET_MASK           0xFFFFFFFFU
#define VSCALER9_STATICCONTROL_SHDEN_MASK           0x1U
#define VSCALER9_STATICCONTROL_SHDEN_SHIFT          0U

#define VSCALER9_SETUP1                             ((uint32_t)(0x340C))
#define VSCALER9_SETUP2                             ((uint32_t)(0x3410))
#define VSCALER9_SETUP3                             ((uint32_t)(0x3414))
#define VSCALER9_SETUP4                             ((uint32_t)(0x3418))
#define VSCALER9_SETUP5                             ((uint32_t)(0x341C))
#define VSCALER9_CONTROL                            ((uint32_t)(0x3420))

#define HSCALER9_STATICCONTROL                      ((uint32_t)(0x3008))
#define HSCALER9_STATICCONTROL_OFFSET               ((uint32_t)(0x8))
#define HSCALER9_STATICCONTROL_RESET_VALUE          0U
#define HSCALER9_STATICCONTROL_RESET_MASK           0xFFFFFFFFU
#define HSCALER9_STATICCONTROL_SHDEN_MASK           0x1U
#define HSCALER9_STATICCONTROL_SHDEN_SHIFT          0U

#define BLITBLEND9_STATICCONTROL                    ((uint32_t)(0x3C08))
#define BLITBLEND9_STATICCONTROL_OFFSET             ((uint32_t)(0x8))
#define BLITBLEND9_STATICCONTROL_RESET_VALUE        0U
#define BLITBLEND9_STATICCONTROL_RESET_MASK         0xFFFFFFFFU
#define BLITBLEND9_STATICCONTROL_SHDEN_MASK         0x1U
#define BLITBLEND9_STATICCONTROL_SHDEN_SHIFT        0U

#define BLITBLEND9_CONTROL                          ((uint32_t)(0x3C0C))
#define BLITBLEND9_CONSTANTCOLOR                    ((uint32_t)(0x3C14))
#define BLITBLEND9_COLORREDBLENDFUNCTION            ((uint32_t)(0x3C18))
#define BLITBLEND9_COLORGREENBLENDFUNCTION          ((uint32_t)(0x3C1C))
#define BLITBLEND9_COLORBLUEBLENDFUNCTION           ((uint32_t)(0x3C20))
#define BLITBLEND9_ALPHABLENDFUNCTION               ((uint32_t)(0x3C24))
#define BLITBLEND9_BLENDMODE1                       ((uint32_t)(0x3C28))
#define BLITBLEND9_BLENDMODE2                       ((uint32_t)(0x3C2C))


#define STORE9_STATICCONTROL                        ((uint32_t)(0x4008))
#define STORE9_STATICCONTROL_OFFSET                 ((uint32_t)(0x8))
#define STORE9_STATICCONTROL_RESET_VALUE            0U
#define STORE9_STATICCONTROL_RESET_MASK             0xFFFFFFFFU
#define STORE9_STATICCONTROL_SHDEN_MASK             0x1U
#define STORE9_STATICCONTROL_SHDEN_SHIFT            0U
#define STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_MASK 0x100U
#define STORE9_STATICCONTROL_BASEADDRESSAUTOUPDATE_SHIFT 8U

#define STORE9_BURSTBUFFERMANAGEMENT                ((uint32_t)(0x400C))
#define STORE9_BASEADDRESS                          ((uint32_t)(0x4018))
#define STORE9_DESTINATIONBUFFERATTRIBUTES          ((uint32_t)(0x401C))
#define STORE9_DESTINATIONBUFFERDIMENSION           ((uint32_t)(0x4020))
#define STORE9_FRAMEOFFSET                          ((uint32_t)(0x4024))
#define STORE9_COLORCOMPONENTBITS                   ((uint32_t)(0x4028))
#define STORE9_COLORCOMPONENTSHIFT                  ((uint32_t)(0x402C))
#define STORE9_CONTROL                              ((uint32_t)(0x4030))

#define STORE9_START                                ((uint32_t)(0x403C))

/* pixengcfg */
#define PIXENGCFG_CLKEN_MASK 0x3000000U
#define PIXENGCFG_CLKEN_SHIFT 24U
/* Field Value: _CLKEN__DISABLE, Clock for block is disabled  */
#define PIXENGCFG_CLKEN__DISABLE 0U
#define PIXENGCFG_CLKEN__AUTOMATIC 0x1U
/* Field Value: _CLKEN__FULL, Clock for block is without gating  */
#define PIXENGCFG_CLKEN__FULL 0x3U

#define PIXENGCFG_DIVIDER_RESET 0x80


/* command sequencer */
#define CMDSEQ_HIF                                  ((uint32_t)(0x400))

#define CMDSEQ_LOCKUNLOCKHIF                        ((uint32_t)(0x500))
#define CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__LOCK_KEY 0x5651F763U
#define CMDSEQ_LOCKUNLOCKHIF_LOCKUNLOCKHIF__UNLOCK_KEY 0x691DB936U

#define CMDSEQ_LOCKUNLOCK                           ((uint32_t)(0x580))
#define CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__LOCK_KEY      0x5651F763U
#define CMDSEQ_LOCKUNLOCK_LOCKUNLOCK__UNLOCK_KEY    0x691DB936U

#define CMDSEQ_BUFFERADDRESS                        ((uint32_t)(0x588))
#define CMDSEQ_BUFFERSIZE                           ((uint32_t)(0x58C))

#define CMDSEQ_CONTROL                              ((uint32_t)(0x594))
#define CMDSEQ_CONTROL_OFFSET                       ((uint32_t)(0x194))
#define CMDSEQ_CONTROL_RESET_VALUE                  0U
#define CMDSEQ_CONTROL_RESET_MASK                   0xFFFFFFFFU
#define CMDSEQ_CONTROL_CLRAXIW_MASK                 0x1U
#define CMDSEQ_CONTROL_CLRAXIW_SHIFT                0U
#define CMDSEQ_CONTROL_CLRRBUF_MASK                 0x4U
#define CMDSEQ_CONTROL_CLRRBUF_SHIFT                2U
#define CMDSEQ_CONTROL_CLRCMDBUF_MASK               0x8U
#define CMDSEQ_CONTROL_CLRCMDBUF_SHIFT              3U
#define CMDSEQ_CONTROL_CLEAR_MASK                   0x80000000U
#define CMDSEQ_CONTROL_CLEAR_SHIFT                  31U

#define CMDSEQ_STATUS                               ((uint32_t)(0x598))
#define CMDSEQ_STATUS_OFFSET                        ((uint32_t)(0x198))
#define CMDSEQ_STATUS_RESET_VALUE                   0x41000080U
#define CMDSEQ_STATUS_RESET_MASK                    0xFFFFFFFFU
#define CMDSEQ_STATUS_FIFOSPACE_MASK                0x1FFFFU
#define CMDSEQ_STATUS_IDLE_MASK                     0x40000000U

#endif
