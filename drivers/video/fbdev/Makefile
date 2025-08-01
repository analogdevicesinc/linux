# SPDX-License-Identifier: GPL-2.0
# Makefile for the Linux video drivers.
# 5 Aug 1999, James Simmons, <mailto:jsimmons@users.sf.net>
# Rewritten to use lists instead of if-statements.

# Each configuration option enables a list of files.

obj-y				+= core/

obj-$(CONFIG_FB_MACMODES)      += macmodes.o
obj-$(CONFIG_FB_SBUS)          += sbuslib.o
obj-$(CONFIG_FB_WMT_GE_ROPS)   += wmt_ge_rops.o

# Hardware specific drivers go first
obj-$(CONFIG_FB_ALTERA_VIP)	  += altvipfb.o
obj-$(CONFIG_FB_AMIGA)            += amifb.o c2p_planar.o
obj-$(CONFIG_FB_ARC)              += arcfb.o
obj-$(CONFIG_FB_CLPS711X)	  += clps711x-fb.o
obj-$(CONFIG_FB_CYBER2000)        += cyber2000fb.o
obj-$(CONFIG_FB_GRVGA)            += grvga.o
obj-$(CONFIG_FB_PM2)              += pm2fb.o
obj-$(CONFIG_FB_PM3)		  += pm3fb.o

obj-$(CONFIG_FB_I740)		  += i740fb.o
obj-$(CONFIG_FB_MATROX)		  += matrox/
obj-$(CONFIG_FB_RIVA)		  += riva/
obj-$(CONFIG_FB_NVIDIA)		  += nvidia/
obj-$(CONFIG_FB_ATY)		  += aty/ macmodes.o
obj-$(CONFIG_FB_ATY128)		  += aty/ macmodes.o
obj-$(CONFIG_FB_RADEON)		  += aty/
obj-$(CONFIG_FB_SIS)		  += sis/
obj-$(CONFIG_FB_VIA)		  += via/
obj-$(CONFIG_FB_KYRO)             += kyro/
obj-$(CONFIG_FB_SAVAGE)		  += savage/
obj-$(CONFIG_FB_GEODE)		  += geode/
obj-$(CONFIG_FB_NEOMAGIC)         += neofb.o
obj-$(CONFIG_FB_3DFX)             += tdfxfb.o
obj-$(CONFIG_FB_CONTROL)          += controlfb.o
obj-$(CONFIG_FB_PLATINUM)         += platinumfb.o
obj-$(CONFIG_FB_VALKYRIE)         += valkyriefb.o
obj-$(CONFIG_FB_CT65550)          += chipsfb.o
obj-$(CONFIG_FB_IMSTT)            += imsttfb.o
obj-$(CONFIG_FB_FM2)              += fm2fb.o
obj-$(CONFIG_FB_VT8623)           += vt8623fb.o
obj-$(CONFIG_FB_TRIDENT)          += tridentfb.o
obj-$(CONFIG_FB_S3)               += s3fb.o
obj-$(CONFIG_FB_ARK)              += arkfb.o
obj-$(CONFIG_FB_STI)              += stifb.o
obj-$(CONFIG_FB_FFB)              += ffb.o
obj-$(CONFIG_FB_CG6)              += cg6.o
obj-$(CONFIG_FB_CG3)              += cg3.o
obj-$(CONFIG_FB_BW2)              += bw2.o
obj-$(CONFIG_FB_CG14)             += cg14.o
obj-$(CONFIG_FB_P9100)            += p9100.o
obj-$(CONFIG_FB_TCX)              += tcx.o
obj-$(CONFIG_FB_LEO)              += leo.o
obj-$(CONFIG_FB_ACORN)            += acornfb.o
obj-$(CONFIG_FB_ATARI)            += atafb.o c2p_iplan2.o atafb_mfb.o \
                                     atafb_iplan2p2.o atafb_iplan2p4.o atafb_iplan2p8.o
obj-$(CONFIG_FB_MAC)              += macfb.o
obj-$(CONFIG_FB_HECUBA)           += hecubafb.o
obj-$(CONFIG_FB_N411)             += n411.o
obj-$(CONFIG_FB_HGA)              += hgafb.o
obj-$(CONFIG_FB_XVR500)           += sunxvr500.o
obj-$(CONFIG_FB_XVR2500)          += sunxvr2500.o
obj-$(CONFIG_FB_XVR1000)          += sunxvr1000.o
obj-$(CONFIG_FB_APOLLO)           += dnfb.o
obj-$(CONFIG_FB_Q40)              += q40fb.o
obj-$(CONFIG_FB_TGA)              += tgafb.o
obj-$(CONFIG_FB_HP300)            += hpfb.o
obj-$(CONFIG_FB_G364)             += g364fb.o
obj-$(CONFIG_FB_EP93XX)		  += ep93xx-fb.o
obj-$(CONFIG_FB_SA1100)           += sa1100fb.o
obj-$(CONFIG_FB_HIT)              += hitfb.o
obj-$(CONFIG_FB_ATMEL)		  += atmel_lcdfb.o
obj-$(CONFIG_FB_PVR2)             += pvr2fb.o
obj-$(CONFIG_FB_VOODOO1)          += sstfb.o
obj-$(CONFIG_FB_GOLDFISH)         += goldfishfb.o
obj-$(CONFIG_FB_68328)            += 68328fb.o
obj-$(CONFIG_FB_GBE)              += gbefb.o
obj-$(CONFIG_FB_CIRRUS)		  += cirrusfb.o
obj-$(CONFIG_FB_ASILIANT)	  += asiliantfb.o
obj-$(CONFIG_FB_PXA)		  += pxafb.o
obj-$(CONFIG_FB_PXA168)		  += pxa168fb.o
obj-$(CONFIG_PXA3XX_GCU)	  += pxa3xx-gcu.o
obj-$(CONFIG_MMP_DISP)           += mmp/
obj-$(CONFIG_FB_AU1100)		  += au1100fb.o
obj-$(CONFIG_FB_AU1200)		  += au1200fb.o
obj-$(CONFIG_FB_VT8500)		  += vt8500lcdfb.o
obj-$(CONFIG_FB_WM8505)		  += wm8505fb.o
obj-$(CONFIG_FB_PMAG_AA)	  += pmag-aa-fb.o
obj-$(CONFIG_FB_PMAG_BA)	  += pmag-ba-fb.o
obj-$(CONFIG_FB_PMAGB_B)	  += pmagb-b-fb.o
obj-$(CONFIG_FB_MAXINE)		  += maxinefb.o
obj-$(CONFIG_FB_METRONOME)        += metronomefb.o
obj-$(CONFIG_FB_BROADSHEET)       += broadsheetfb.o
obj-$(CONFIG_FB_S1D13XXX)	  += s1d13xxxfb.o
obj-$(CONFIG_FB_SH7760)		  += sh7760fb.o
obj-$(CONFIG_FB_IMX)              += imxfb.o
obj-$(CONFIG_FB_S3C)		  += s3c-fb.o
obj-$(CONFIG_FB_FSL_DIU)	  += fsl-diu-fb.o
obj-$(CONFIG_FB_COBALT)           += cobalt_lcdfb.o
obj-$(CONFIG_FB_IBM_GXT4500)	  += gxt4500.o
obj-$(CONFIG_FB_PS3)		  += ps3fb.o
obj-$(CONFIG_FB_SM501)            += sm501fb.o
obj-$(CONFIG_FB_UDL)		  += udlfb.o
obj-$(CONFIG_FB_SMSCUFX)	  += smscufx.o
obj-$(CONFIG_FB_XILINX)           += xilinxfb.o
obj-$(CONFIG_FB_SH_MOBILE_LCDC)	  += sh_mobile_lcdcfb.o
obj-y				  += omap/
obj-y                             += omap2/
obj-$(CONFIG_XEN_FBDEV_FRONTEND)  += xen-fbfront.o
obj-$(CONFIG_FB_CARMINE)          += carminefb.o
obj-$(CONFIG_FB_MB862XX)	  += mb862xx/
obj-$(CONFIG_FB_HYPERV)		  += hyperv_fb.o
obj-$(CONFIG_FB_OPENCORES)	  += ocfb.o
obj-$(CONFIG_FB_SM712)		  += sm712fb.o

# Platform or fallback drivers go here
obj-$(CONFIG_FB_UVESA)            += uvesafb.o
obj-$(CONFIG_FB_VESA)             += vesafb.o
obj-$(CONFIG_FB_EFI)              += efifb.o
obj-$(CONFIG_FB_VGA16)            += vga16fb.o
obj-$(CONFIG_FB_OF)               += offb.o
obj-$(CONFIG_FB_SSD1307)	  += ssd1307fb.o
obj-$(CONFIG_FB_SIMPLE)           += simplefb.o

# the test framebuffer is last
obj-$(CONFIG_FB_VIRTUAL)          += vfb.o
