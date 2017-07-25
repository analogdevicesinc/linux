/**
 * dev-regs-map.h - Cadence USB3 Device register map definition
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


#ifndef __REG_USBSS_DEV_ADDR_MAP_H__
#define __REG_USBSS_DEV_ADDR_MAP_H__

#include "dev-regs-macro.h"

struct usbss_dev_register_block_type {
	uint32_t usb_conf;                     /*        0x0 - 0x4        */
	uint32_t usb_sts;                      /*        0x4 - 0x8        */
	uint32_t usb_cmd;                      /*        0x8 - 0xc        */
	uint32_t usb_iptn;                     /*        0xc - 0x10       */
	uint32_t usb_lpm;                      /*       0x10 - 0x14       */
	uint32_t usb_ien;                      /*       0x14 - 0x18       */
	uint32_t usb_ists;                     /*       0x18 - 0x1c       */
	uint32_t ep_sel;                       /*       0x1c - 0x20       */
	uint32_t ep_traddr;                    /*       0x20 - 0x24       */
	uint32_t ep_cfg;                       /*       0x24 - 0x28       */
	uint32_t ep_cmd;                       /*       0x28 - 0x2c       */
	uint32_t ep_sts;                       /*       0x2c - 0x30       */
	uint32_t ep_sts_sid;                   /*       0x30 - 0x34       */
	uint32_t ep_sts_en;                    /*       0x34 - 0x38       */
	uint32_t drbl;                         /*       0x38 - 0x3c       */
	uint32_t ep_ien;                       /*       0x3c - 0x40       */
	uint32_t ep_ists;                      /*       0x40 - 0x44       */
	uint32_t usb_pwr;                      /*       0x44 - 0x48       */
	uint32_t usb_conf2;                    /*       0x48 - 0x4c       */
	uint32_t usb_cap1;                     /*       0x4c - 0x50       */
	uint32_t usb_cap2;                     /*       0x50 - 0x54       */
	uint32_t usb_cap3;                     /*       0x54 - 0x58       */
	uint32_t usb_cap4;                     /*       0x58 - 0x5c       */
	uint32_t usb_cap5;                     /*       0x5c - 0x60       */
	uint32_t PAD2_73;                     /*       0x60 - 0x64       */
	uint32_t usb_cpkt1;                    /*       0x64 - 0x68       */
	uint32_t usb_cpkt2;                    /*       0x68 - 0x6c       */
	uint32_t usb_cpkt3;                    /*       0x6c - 0x70       */
	char pad__0[0x90];                     /*       0x70 - 0x100      */
	uint32_t PAD2_78;                     /*      0x100 - 0x104      */
	uint32_t dbg_link1;                    /*      0x104 - 0x108      */
	uint32_t PAD2_80;                    /*      0x108 - 0x10c      */
	uint32_t PAD2_81;                     /*      0x10c - 0x110      */
	uint32_t PAD2_82;                     /*      0x110 - 0x114      */
	uint32_t PAD2_83;                     /*      0x114 - 0x118      */
	uint32_t PAD2_84;                     /*      0x118 - 0x11c      */
	uint32_t PAD2_85;                     /*      0x11c - 0x120      */
	uint32_t PAD2_86;                     /*      0x120 - 0x124      */
	uint32_t PAD2_87;                    /*      0x124 - 0x128      */
	uint32_t PAD2_88;                    /*      0x128 - 0x12c      */
	uint32_t PAD2_89;                    /*      0x12c - 0x130      */
	uint32_t PAD2_90;                    /*      0x130 - 0x134      */
	uint32_t PAD2_91;                    /*      0x134 - 0x138      */
	uint32_t PAD2_92;                    /*      0x138 - 0x13c      */
	uint32_t PAD2_93;                    /*      0x13c - 0x140      */
	uint32_t PAD2_94;                    /*      0x140 - 0x144      */
	uint32_t PAD2_95;                    /*      0x144 - 0x148      */
	uint32_t PAD2_96;                    /*      0x148 - 0x14c      */
	uint32_t PAD2_97;                    /*      0x14c - 0x150      */
	uint32_t PAD2_98;                    /*      0x150 - 0x154      */
	uint32_t PAD2_99;                    /*      0x154 - 0x158      */
	uint32_t PAD2_100;                    /*      0x158 - 0x15c      */
	uint32_t PAD2_101;                    /*      0x15c - 0x160      */
	uint32_t PAD2_102;                    /*      0x160 - 0x164      */
	uint32_t PAD2_103;                    /*      0x164 - 0x168      */
	uint32_t PAD2_104;                    /*      0x168 - 0x16c      */
	uint32_t PAD2_105;                    /*      0x16c - 0x170      */
	uint32_t PAD2_106;                    /*      0x170 - 0x174      */
	uint32_t PAD2_107;                    /*      0x174 - 0x178      */
	uint32_t PAD2_108;                    /*      0x178 - 0x17c      */
	uint32_t PAD2_109;                    /*      0x17c - 0x180      */
	uint32_t PAD2_110;                    /*      0x180 - 0x184      */
	uint32_t PAD2_111;                    /*      0x184 - 0x188      */
	uint32_t PAD2_112;                    /*      0x188 - 0x18c      */
	char pad__1[0x20];                     /*      0x18c - 0x1ac      */
	uint32_t PAD2_114;                    /*      0x1ac - 0x1b0      */
	uint32_t PAD2_115;                    /*      0x1b0 - 0x1b4      */
	uint32_t PAD2_116;                    /*      0x1b4 - 0x1b8      */
	uint32_t PAD2_117;                    /*      0x1b8 - 0x1bc      */
	uint32_t PAD2_118;                    /*      0x1bc - 0x1c0      */
	uint32_t PAD2_119;                    /*      0x1c0 - 0x1c4      */
	uint32_t PAD2_120;                    /*      0x1c4 - 0x1c8      */
	uint32_t PAD2_121;                    /*      0x1c8 - 0x1cc      */
	uint32_t PAD2_122;                    /*      0x1cc - 0x1d0      */
	uint32_t PAD2_123;                    /*      0x1d0 - 0x1d4      */
	uint32_t PAD2_124;                    /*      0x1d4 - 0x1d8      */
	uint32_t PAD2_125;                    /*      0x1d8 - 0x1dc      */
	uint32_t PAD2_126;                    /*      0x1dc - 0x1e0      */
	uint32_t PAD2_127;                    /*      0x1e0 - 0x1e4      */
	uint32_t PAD2_128;                    /*      0x1e4 - 0x1e8      */
	uint32_t PAD2_129;                    /*      0x1e8 - 0x1ec      */
	uint32_t PAD2_130;                    /*      0x1ec - 0x1f0      */
	uint32_t PAD2_131;                    /*      0x1f0 - 0x1f4      */
	uint32_t PAD2_132;                    /*      0x1f4 - 0x1f8      */
	uint32_t PAD2_133;                    /*      0x1f8 - 0x1fc      */
	uint32_t PAD2_134;                    /*      0x1fc - 0x200      */
	uint32_t PAD2_135;                    /*      0x200 - 0x204      */
	uint32_t PAD2_136;                    /*      0x204 - 0x208      */
	uint32_t PAD2_137;                    /*      0x208 - 0x20c      */
	uint32_t PAD2_138;                    /*      0x20c - 0x210      */
	uint32_t PAD2_139;                    /*      0x210 - 0x214      */
	uint32_t PAD2_140;                    /*      0x214 - 0x218      */
	uint32_t PAD2_141;                    /*      0x218 - 0x21c      */
	uint32_t PAD2_142;                    /*      0x21c - 0x220      */
	uint32_t PAD2_143;                    /*      0x220 - 0x224      */
	uint32_t PAD2_144;                    /*      0x224 - 0x228      */
	char pad__2[0xd8];                     /*      0x228 - 0x300      */
	uint32_t dma_axi_ctrl;                 /*      0x300 - 0x304      */
	uint32_t PAD2_147;                   /*      0x304 - 0x308      */
	uint32_t PAD2_148;                  /*      0x308 - 0x30c      */
	uint32_t PAD2_149;                /*      0x30c - 0x310      */
	uint32_t PAD2_150;                /*      0x310 - 0x314      */
};

#endif /* __REG_USBSS_DEV_ADDR_MAP_H__ */
