/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Packet engine driver for Analog Devices Incorporated
 *
 * Currently tested on SC598 processor
 *
 * Copyright (c) 2023 - Timesys Corporation
 *   Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#ifndef ADI_PKTE_H
#define ADI_PKTE_H

//#define PKTE_USE_SRAM
#ifdef PKTE_USE_SRAM
	#define PKTE_SRAM_ADDRESS        0x20001000
#endif

#define PKTE_MAX_KEY_SIZE      (AES_KEYSIZE_256 * 8)
#define PKTE_BUFLEN            (8 * 1024)
#define PKTE_RING_BUFFERS      4

#define PKTE_OP_UPDATE         1
#define PKTE_OP_FINAL          2

#define PKTE_FLAGS_HMAC_KEY_PREPARED BIT(8)
#define PKTE_FLAGS_HMAC              BIT(9)
#define PKTE_TCM_MODE                BIT(10)
#define PKTE_AUTONOMOUS_MODE         BIT(11)
#define PKTE_HOST_MODE               BIT(12)
#define PKTE_FLAGS_STARTED           BIT(13)
#define PKTE_FLAGS_COMPLETE          BIT(14)
#define PKTE_FLAGS_FINAL             BIT(15)
#define PKTE_FLAGS_FINUP             BIT(16)

#define INNER_OUTER_KEY_SIZE   64

#define CTL_STAT_OFFSET              0x00000000 /* Control Register */
#define SRC_ADDR_OFFSET              0x00000004 /* Source Address */
#define DEST_ADDR_OFFSET             0x00000008 /* Destination Address */
#define SA_ADDR_OFFSET               0x0000000C /* SA Address */
#define STATE_ADDR_OFFSET            0x00000010 /* State Record Address */
#define ARC4STATE_ADDR_OFFSET        0x00000014 /* ARC4 State Record Address */
#define USERID_OFFSET                0x00000018 /* User ID */
#define LEN_OFFSET                   0x0000001C /* Length Register */
#define CDRBASE_ADDR_OFFSET          0x00000080 /* Command Descriptor Ring Base Address */
#define RDRBASE_ADDR_OFFSET          0x00000084 /* Result Descriptor Ring Base Address */
#define RING_CFG_OFFSET              0x00000088 /* Ring Configuration */
#define RING_THRESH_OFFSET           0x0000008C /* Ring Threshold Registers */
#define CDSC_CNT_OFFSET              0x00000090 /* Command Descriptor Count Register */
#define CDSC_INCR_OFFSET             0x00000090 /* Command Descriptor Count Increment Register */
#define RDSC_CNT_OFFSET              0x00000094 /* Result Descriptor Count Registers */
#define RDSC_DECR_OFFSET             0x00000094 /* Result Descriptor Count Decrement Registers */
#define RING_PTR_OFFSET              0x00000098 /* Ring Pointer Status */
#define RING_STAT_OFFSET             0x0000009C /* Ring Status */
#define CFG_OFFSET                   0x00000100 /* Configuration Register */
#define STAT_OFFSET                  0x00000104 /* Status Register */
#define BUF_THRESH_OFFSET            0x0000010C /* Buffer Threshold Register */
#define INBUF_CNT_OFFSET             0x00000110 /* Input Buffer Count Register */
#define INBUF_INCR_OFFSET            0x00000110 /* Input Buffer Count Increment Register */
#define OUTBUF_CNT_OFFSET            0x00000114 /* Output Buffer Count Register */
#define OUTBUF_DECR_OFFSET           0x00000114 /* Output Buffer Count Decrement Register */
#define BUF_PTR_OFFSET               0x00000118 /* Buffer Pointer Register */
#define DMA_CFG_OFFSET               0x00000120 /* DMA Configuration Register */
#define ENDIAN_CFG_OFFSET            0x000001D0 /* Endian Configuration Register */
#define HLT_CTL_OFFSET               0x000001E0 /* Halt Control Register */
#define HLT_STAT_OFFSET              0x000001E0 /* Halt Status Register */
#define CONT_OFFSET                  0x000001E4 /* PKTE Continue Register */
#define CLK_CTL_OFFSET               0x000001E8 /* PE Clock Control Register */
#define IUMSK_STAT_OFFSET            0x00000200 /* Interrupt Unmasked Status Register */
#define IMSK_STAT_OFFSET             0x00000204 /* Interrupt Masked Status Register */
#define INT_CLR_OFFSET               0x00000204 /* Interrupt Clear Register */
#define INT_EN_OFFSET                0x00000208 /* Interrupt Enable Register */
#define INT_CFG_OFFSET               0x0000020C /* Interrupt Configuration Register */
#define IMSK_EN_OFFSET               0x00000210 /* Interrupt Mask Enable Register */
#define IMSK_DIS_OFFSET              0x00000214 /* Interrupt Mask Disable Register */
#define SA_CMD_OFFSET(x)             (0x00000400 + x*4) /* SA Command 0 */
#define SA_KEY_OFFSET(x)             (0x00000408 + x*4) /* SA Key Registers */
#define SA_IDIGEST_OFFSET(x)         (0x00000428 + x*4) /* SA Inner Hash Digest Registers */
#define SA_ODIGEST_OFFSET(x)         (0x00000448 + x*4) /* SA Outer Hash Digest Registers */
#define SA_SPI_OFFSET                0x00000468 /* SA SPI Register */
#define SA_SEQNUM_OFFSET(x)          (0x0000046C + x*4) /* SA Sequence Number Register */
#define SA_SEQNUM_MSK_OFFSET(x)      (0x00000474 + x*4) /* SA Sequence Number Mask Registers */
#define SA_ARC4IJPTR_OFFSET          0x0000047C  /* ARC4 i and j Pointer Register */
#define SA_NONCE_OFFSET              0x0000047C  /* SA Initialization Vector Register */
#define SA_RDY_OFFSET                0x0000047C  /* SA Ready Indicator */
#define STATE_IV_OFFSET(x)           (0x00000500 + x*4) /* State Init Vector Registers */
#define STATE_BYTE_CNT_OFFSET(x)     (0x00000510 + x*4) /* State Hash Byte Count Registers */
#define STATE_IDIGEST_OFFSET(x)      (0x00000518 + x*4) /* State Inner Digest Registers */
#define ARC4STATE_BUF_OFFSET         0x00000700  /* Start Entry of 256-byte ARC4 State Buffer */
#define DATAIO_BUF_OFFSET            0x00000800  /* Start Entry of 256-byte Data In/Out Buffer */

#define BITP_PKTE_CTL_STAT_PADCTLSTAT        24 /* Pad Control/Pad Status */
#define BITP_PKTE_CTL_STAT_EXTERRCD          20 /* Extended Error Code */
#define BITP_PKTE_CTL_STAT_EXTERR            19 /* Extended Error */
#define BITP_PKTE_CTL_STAT_SQNMERR           18 /* Sequence Number Error */
#define BITP_PKTE_CTL_STAT_PADERR            17 /* Pad Error */
#define BITP_PKTE_CTL_STAT_AUTHERR           16 /* Authentication Error */
#define BITP_PKTE_CTL_STAT_PADVAL             8 /* Pad Value */
#define BITP_PKTE_CTL_STAT_PRNGMD             6 /* PRNG Mode */
#define BITP_PKTE_CTL_STAT_HASHFINAL          4 /* Hash Final */
#define BITP_PKTE_CTL_STAT_INITARC4           3 /* Init ARC4 */
#define BITP_PKTE_CTL_STAT_PERDY              1 /* Packet Engine Ready */
#define BITP_PKTE_CTL_STAT_HOSTRDY            0 /* Host Ready */
#define BITM_PKTE_CTL_STAT_PADCTLSTAT        0xFF000000 /* Pad Control/Pad Status */
#define BITM_PKTE_CTL_STAT_EXTERRCD          0x00F00000 /* Extended Error Code */
#define BITM_PKTE_CTL_STAT_EXTERR            0x00080000 /* Extended Error */
#define BITM_PKTE_CTL_STAT_SQNMERR           0x00040000 /* Sequence Number Error */
#define BITM_PKTE_CTL_STAT_PADERR            0x00020000 /* Pad Error */
#define BITM_PKTE_CTL_STAT_AUTHERR           0x00010000 /* Authentication Error */
#define BITM_PKTE_CTL_STAT_PADVAL            0x0000FF00 /* Pad Value */
#define BITM_PKTE_CTL_STAT_PRNGMD            0x000000C0 /* PRNG Mode */
#define BITM_PKTE_CTL_STAT_HASHFINAL         0x00000010 /* Hash Final */
#define BITM_PKTE_CTL_STAT_INITARC4          0x00000008 /* Init ARC4 */
#define BITM_PKTE_CTL_STAT_PERDY             0x00000002 /* Packet Engine Ready */
#define BITM_PKTE_CTL_STAT_HOSTRDY           0x00000001 /* Host Ready */

#define BITP_PKTE_SRC_ADDR_VALUE             0 /* Packet Source Address */
#define BITM_PKTE_SRC_ADDR_VALUE             0xFFFFFFFF /* Packet Source Address */

#define BITP_PKTE_DEST_ADDR_VALUE            0 /* Packet Destination Address */
#define BITM_PKTE_DEST_ADDR_VALUE            0xFFFFFFFF /* Packet Destination Address */

#define BITP_PKTE_SA_ADDR_VALUE              0 /* SA Record Address */
#define BITM_PKTE_SA_ADDR_VALUE              0xFFFFFFFF /* SA Record Address */

#define BITP_PKTE_STATE_ADDR_VALUE           0 /* State Record Address */
#define BITM_PKTE_STATE_ADDR_VALUE           0xFFFFFFFF /* State Record Address */

#define BITP_PKTE_ARC4STATE_ADDR_VALUE       0 /* Arc4 State Record Address */
#define BITM_PKTE_ARC4STATE_ADDR_VALUE       0xFFFFFFFF /* Arc4 State Record Address */

#define BITP_PKTE_USERID_VALUE               0 /* Command Descriptor User ID */
#define BITM_PKTE_USERID_VALUE               0xFFFFFFFF /* Command Descriptor User ID */

#define BITP_PKTE_LEN_BYPASS                 24 /* Bypass */
#define BITP_PKTE_LEN_PEDONE                 23 /* PE Done */
#define BITP_PKTE_LEN_HSTRDY                 22 /* Host Ready */
#define BITP_PKTE_LEN_TOTLEN                 0  /* Total length */
#define BITM_PKTE_LEN_BYPASS                 0xFF000000 /* Bypass */
#define BITM_PKTE_LEN_PEDONE                 0x00800000 /* PE Done */
#define BITM_PKTE_LEN_HSTRDY                 0x00400000 /* Host Ready */
#define BITM_PKTE_LEN_TOTLEN                 0x000FFFFF /* Total length */

#define BITP_PKTE_CDRBASE_ADDR_VALUE         0 /* Command Descriptor Ring Base Address */
#define BITM_PKTE_CDRBASE_ADDR_VALUE         0xFFFFFFFF /* Command Descriptor Ring Base Address */

#define BITP_PKTE_RDRBASE_ADDR_VALUE         0 /* Result Descriptor Ring Base Address */
#define BITM_PKTE_RDRBASE_ADDR_VALUE         0xFFFFFFFF /* Result Descriptor Ring Base Address */

#define BITP_PKTE_RING_CFG_ENEXTTRIG         31 /* Enable  External Trigger */
#define BITP_PKTE_RING_CFG_RINGSZ            0 /* Ring Size */
#define BITM_PKTE_RING_CFG_ENEXTTRIG         0x80000000 /* Enable  External Trigger */
#define BITM_PKTE_RING_CFG_RINGSZ            0x000003FF /* Ring Size */

#define BITP_PKTE_RING_THRESH_TOEN           31 /* Timeout Enable */
#define BITP_PKTE_RING_THRESH_RDTO           26 /* Read Descriptor Timeout */
#define BITP_PKTE_RING_THRESH_RDRTHRSH       16 /* Result Descriptor Ring Threshold */
#define BITP_PKTE_RING_THRESH_CDRTHRSH        0 /* Command Descriptor Ring Threshold */
#define BITM_PKTE_RING_THRESH_TOEN           0x80000000 /* Timeout Enable */
#define BITM_PKTE_RING_THRESH_RDTO           0x3C000000 /* Read Descriptor Timeout */
#define BITM_PKTE_RING_THRESH_RDRTHRSH       0x03FF0000 /* Result Descriptor Ring Threshold */
#define BITM_PKTE_RING_THRESH_CDRTHRSH       0x000003FF /* Command Descriptor Ring Threshold */

#define BITP_PKTE_CDSC_CNT_VALUE             0 /* Command Descriptor Count */
#define BITM_PKTE_CDSC_CNT_VALUE             0x000007FF /* Command Descriptor Count */

#define BITP_PKTE_CDSC_INCR_VALUE            0 /* Command Descriptor Count Increment */
#define BITM_PKTE_CDSC_INCR_VALUE            0x000000FF /* Command Descriptor Count Increment */

#define BITP_PKTE_RDSC_CNT_VALUE             0 /* Result Descriptor Count */
#define BITM_PKTE_RDSC_CNT_VALUE             0x000007FF /* Result Descriptor Count */

#define BITP_PKTE_RDSC_DECR_VALUE            0 /* Read Count Decrement */
#define BITM_PKTE_RDSC_DECR_VALUE            0x000000FF /* Read Count Decrement */

#define BITP_PKTE_RING_PTR_RDRPTR            16 /* Result Descriptor Ring Write Pointer */
#define BITP_PKTE_RING_PTR_CDRPTR            0 /* Command Descriptor Ring Read Pointer */
#define BITM_PKTE_RING_PTR_RDRPTR            0x03FF0000 /* Result Descriptor Ring Write Pointer */
#define BITM_PKTE_RING_PTR_CDRPTR            0x000003FF /* Command Descriptor Ring Read Pointer */

#define BITP_PKTE_RING_STAT_RDRUNFL          1 /* Result Descriptor Ring Underflow */
#define BITP_PKTE_RING_STAT_CDROVFL          0 /* Command Descriptor Ring Overflow */
#define BITM_PKTE_RING_STAT_RDRUNFL          0x00000002 /* Result Descriptor Ring Underflow */
#define BITM_PKTE_RING_STAT_CDROVFL          0x00000001 /* Command Descriptor Ring Overflow */

#define BITP_PKTE_CFG_SWPDAT                 18 /* Swap Data */
#define BITP_PKTE_CFG_SWPSA                  17 /* Swap SA */
#define BITP_PKTE_CFG_SWPCDRD                16 /* Swap CD RD */
#define BITP_PKTE_CFG_ENCDRUPDT              10 /* Enable CDR Update */
#define BITP_PKTE_CFG_MODE                   8 /* Packet Engine Mode */
#define BITP_PKTE_CFG_RSTRING                1 /* Reset Ring */
#define BITP_PKTE_CFG_RSTPE                  0 /* Reset Packet Engine */
#define BITM_PKTE_CFG_SWPDAT                 0x00040000 /* Swap Data */
#define BITM_PKTE_CFG_SWPSA                  0x00020000 /* Swap SA */
#define BITM_PKTE_CFG_SWPCDRD                0x00010000 /* Swap CD RD */
#define BITM_PKTE_CFG_ENCDRUPDT              0x00000400 /* Enable CDR Update */
#define BITM_PKTE_CFG_MODE                   0x00000300 /* Packet Engine Mode */
#define BITM_PKTE_CFG_RSTRING                0x00000002 /* Reset Ring */
#define BITM_PKTE_CFG_RSTPE                  0x00000001 /* Reset Packet Engine */
#define ENUM_PKTE_CFG_HOST                   0x00000000 /* MODE: Direct Host Mode. */
#define ENUM_PKTE_CFG_TCM0                   0x00000100 /* MODE: TCM with RDR Disabled. */
#define ENUM_PKTE_CFG_TCM1                   0x00000200 /* MODE: TCM with RDR Enabled. */
#define ENUM_PKTE_CFG_AUTO                   0x00000300 /* MODE: Autonomous Ring Mode */

#define BITP_PKTE_STAT_OBUFFULLCNT           22 /* Output Buffer Full Count */
#define BITP_PKTE_STAT_IBUFEMPTYCNT          12 /* Input Buffer Empty Count */
#define BITP_PKTE_STAT_OBUFREQ               11 /* Output Buffer Request Active */
#define BITP_PKTE_STAT_IBUFREQ               10 /* Input Buffer Request Active */
#define BITP_PKTE_STAT_OPDN                   9 /* Operation Done */
#define BITP_PKTE_STAT_EXTERR                 8 /* Extended Error */
#define BITP_PKTE_STAT_SNUMERR                7 /* Sequence Number Error */
#define BITP_PKTE_STAT_PADERR                 6 /* Pad Error */
#define BITP_PKTE_STAT_AUTHERR                5 /* Authentication Error */
#define BITP_PKTE_STAT_OUTHSHDN               4 /* Outer Hash Done */
#define BITP_PKTE_STAT_INHSHDN                3 /* Inner Hash Done */
#define BITP_PKTE_STAT_ENCRYPTDN              2 /* Encrypt Done */
#define BITP_PKTE_STAT_OUTPTDN                1 /* PE Output Done */
#define BITP_PKTE_STAT_INPTDN                 0 /* Packet Engine Input Done */
#define BITM_PKTE_STAT_OBUFFULLCNT           0xFFC00000 /* Output Buffer Full Count */
#define BITM_PKTE_STAT_IBUFEMPTYCNT          0x003FF000 /* Input Buffer Empty Count */
#define BITM_PKTE_STAT_OBUFREQ               0x00000800 /* Output Buffer Request Active */
#define BITM_PKTE_STAT_IBUFREQ               0x00000400 /* Input Buffer Request Active */
#define BITM_PKTE_STAT_OPDN                  0x00000200 /* Operation Done */
#define BITM_PKTE_STAT_EXTERR                0x00000100 /* Extended Error */
#define BITM_PKTE_STAT_SNUMERR               0x00000080 /* Sequence Number Error */
#define BITM_PKTE_STAT_PADERR                0x00000040 /* Pad Error */
#define BITM_PKTE_STAT_AUTHERR               0x00000020 /* Authentication Error */
#define BITM_PKTE_STAT_OUTHSHDN              0x00000010 /* Outer Hash Done */
#define BITM_PKTE_STAT_INHSHDN               0x00000008 /* Inner Hash Done */
#define BITM_PKTE_STAT_ENCRYPTDN             0x00000004 /* Encrypt Done */
#define BITM_PKTE_STAT_OUTPTDN               0x00000002 /* PE Output Done */
#define BITM_PKTE_STAT_INPTDN                0x00000001 /* Packet Engine Input Done */

#define BITP_PKTE_BUF_THRESH_OUTBUF          16 /* Output Buffer Threshold */
#define BITP_PKTE_BUF_THRESH_INBUF            0 /* Input Buffer Threshold */
#define BITM_PKTE_BUF_THRESH_OUTBUF          0x00FF0000 /* Output Buffer Threshold */
#define BITM_PKTE_BUF_THRESH_INBUF           0x000000FF /* Input Buffer Threshold */

#define BITP_PKTE_INBUF_CNT_VALUE            0 /* Input Buffer Count */
#define BITM_PKTE_INBUF_CNT_VALUE            0x000001FF /* Input Buffer Count */

#define BITP_PKTE_INBUF_INCR_VALUE           0 /* Input Buffer Increment */
#define BITM_PKTE_INBUF_INCR_VALUE           0x000001FF /* Input Buffer Increment */

#define BITP_PKTE_OUTBUF_CNT_VALUE           0 /* Output Buffer Count */
#define BITM_PKTE_OUTBUF_CNT_VALUE           0x000001FF /* Output Buffer Count */

#define BITP_PKTE_OUTBUF_DECR_VALUE          0 /* Output Buffer Count Decrement */
#define BITM_PKTE_OUTBUF_DECR_VALUE          0x000001FF /* Output Buffer Count Decrement */

#define BITP_PKTE_BUF_PTR_OUTBUF             16 /* Output Buffer Pointer */
#define BITP_PKTE_BUF_PTR_INBUF               0 /* Input Buffer Pointer */
#define BITM_PKTE_BUF_PTR_OUTBUF             0x00FF0000 /* Output Buffer Pointer */
#define BITM_PKTE_BUF_PTR_INBUF              0x000000FF /* Input Buffer Pointer */

#define BITP_PKTE_DMA_CFG_IDLE               20 /* Idle Enable */
#define BITP_PKTE_DMA_CFG_INCR               19 /* Increment Enable */
#define BITP_PKTE_DMA_CFG_MSTRBIGEND         16 /* Master Big Endian */
#define BITP_PKTE_DMA_CFG_MXBRSTSZ            0 /* Max Burst Size */
#define BITM_PKTE_DMA_CFG_IDLE               0x00100000 /* Idle Enable */
#define BITM_PKTE_DMA_CFG_INCR               0x00080000 /* Increment Enable */
#define BITM_PKTE_DMA_CFG_MSTRBIGEND         0x00010000 /* Master Big Endian */
#define BITM_PKTE_DMA_CFG_MXBRSTSZ           0x0000000F /* Max Burst Size */

#define BITP_PKTE_ENDIAN_CFG_TGTBSWP         16 /* Target Byte Swap */
#define BITP_PKTE_ENDIAN_CFG_MSTRBSWP         0 /* Master Byte Swap */
#define BITM_PKTE_ENDIAN_CFG_TGTBSWP         0x00FF0000 /* Target Byte Swap */
#define BITM_PKTE_ENDIAN_CFG_MSTRBSWP        0x000000FF /* Master Byte Swap */

#define BITP_PKTE_HLT_CTL_WRRD                5 /* Halt On Write Result Descriptor */
#define BITP_PKTE_HLT_CTL_WRSA                4 /* Halt On Write SA */
#define BITP_PKTE_HLT_CTL_HWRDAT              3 /* Halt On Write Data */
#define BITP_PKTE_HLT_CTL_RDSA                2 /* Halt On Read SA */
#define BITP_PKTE_HLT_CTL_RDCD                1 /* Halt On Read Command Descriptor */
#define BITP_PKTE_HLT_CTL_EN                  0 /* Enable Halt Mode */
#define BITM_PKTE_HLT_CTL_WRRD               0x00000020 /* Halt On Write Result Descriptor */
#define BITM_PKTE_HLT_CTL_WRSA               0x00000010 /* Halt On Write SA */
#define BITM_PKTE_HLT_CTL_HWRDAT             0x00000008 /* Halt On Write Data */
#define BITM_PKTE_HLT_CTL_RDSA               0x00000004 /* Halt On Read SA */
#define BITM_PKTE_HLT_CTL_RDCD               0x00000002 /* Halt On Read Command Descriptor */
#define BITM_PKTE_HLT_CTL_EN                 0x00000001 /* Enable Halt Mode */

#define BITP_PKTE_HLT_STAT_DATSTATE          24 /* Data State */
#define BITP_PKTE_HLT_STAT_RDSASTATE         20 /* Read SA State */
#define BITP_PKTE_HLT_STAT_MNSTATE           16 /* Main State */
#define BITP_PKTE_HLT_STAT_WRRD               5 /* Halt On Write Result Descriptor */
#define BITP_PKTE_HLT_STAT_WRSA               4 /* Halt On Write SA */
#define BITP_PKTE_HLT_STAT_WRDAT              3 /* Halt On Write Data */
#define BITP_PKTE_HLT_STAT_RDSA               2 /* Halt On Read SA */
#define BITP_PKTE_HLT_STAT_RDCD               1 /* Halt On Read Command Descriptor */
#define BITP_PKTE_HLT_STAT_EN                 0 /* Halt Mode Enabled Status */
#define BITM_PKTE_HLT_STAT_DATSTATE          0x07000000 /* Data State */
#define BITM_PKTE_HLT_STAT_RDSASTATE         0x00F00000 /* Read SA State */
#define BITM_PKTE_HLT_STAT_MNSTATE           0x000F0000 /* Main State */
#define BITM_PKTE_HLT_STAT_WRRD              0x00000020 /* Halt On Write Result Descriptor */
#define BITM_PKTE_HLT_STAT_WRSA              0x00000010 /* Halt On Write SA */
#define BITM_PKTE_HLT_STAT_WRDAT             0x00000008 /* Halt On Write Data */
#define BITM_PKTE_HLT_STAT_RDSA              0x00000004 /* Halt On Read SA */
#define BITM_PKTE_HLT_STAT_RDCD              0x00000002 /* Halt On Read Command Descriptor */
#define BITM_PKTE_HLT_STAT_EN                0x00000001 /* Halt Mode Enabled Status */

#define BITP_PKTE_CONT_VALUE                  0 /* Continue Operating */
#define BITM_PKTE_CONT_VALUE                 0xFFFFFFFF /* Continue Operating */

#define BITP_PKTE_CLK_CTL_ENHSHCLK            4 /* Enable Hash Clock */
#define BITP_PKTE_CLK_CTL_ENARC4CLK           3 /* Enable ARC4 Clock */
#define BITP_PKTE_CLK_CTL_ENAESCLK            2 /* Enable AES Clock */
#define BITP_PKTE_CLK_CTL_ENDESCLK            1 /* Enable DES Clock */
#define BITP_PKTE_CLK_CTL_ENPECLK             0 /* Enable Packet Engine Clock */
#define BITM_PKTE_CLK_CTL_ENHSHCLK           0x00000010 /* Enable Hash Clock */
#define BITM_PKTE_CLK_CTL_ENARC4CLK          0x00000008 /* Enable ARC4 Clock */
#define BITM_PKTE_CLK_CTL_ENAESCLK           0x00000004 /* Enable AES Clock */
#define BITM_PKTE_CLK_CTL_ENDESCLK           0x00000002 /* Enable DES Clock */
#define BITM_PKTE_CLK_CTL_ENPECLK            0x00000001 /* Enable Packet Engine Clock */

#define BITP_PKTE_IUMSK_STAT_IFERR           18 /* Interface Error */
#define BITP_PKTE_IUMSK_STAT_PROCERR         17 /* PKTE Processing Error */
#define BITP_PKTE_IUMSK_STAT_RINGERR         16 /* PKTE Ring Error */
#define BITP_PKTE_IUMSK_STAT_HLT             15 /* Halt */
#define BITP_PKTE_IUMSK_STAT_OBUFTHRSH       11 /* Output Buffer Threshold */
#define BITP_PKTE_IUMSK_STAT_IBUFTHRSH       10 /* Input Buffer Threshold */
#define BITP_PKTE_IUMSK_STAT_OPDN             9 /* Operation Done */
#define BITP_PKTE_IUMSK_STAT_RDRTHRSH         1 /* RDR Threshold */
#define BITP_PKTE_IUMSK_STAT_CDRTHRSH         0 /* CDR Threshold */
#define BITM_PKTE_IUMSK_STAT_IFERR           0x00040000 /* Interface Error */
#define BITM_PKTE_IUMSK_STAT_PROCERR         0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_IUMSK_STAT_RINGERR         0x00010000 /* PKTE Ring Error */
#define BITM_PKTE_IUMSK_STAT_HLT             0x00008000 /* Halt */
#define BITM_PKTE_IUMSK_STAT_OBUFTHRSH       0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_IUMSK_STAT_IBUFTHRSH       0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_IUMSK_STAT_OPDN            0x00000200 /* Operation Done */
#define BITM_PKTE_IUMSK_STAT_RDRTHRSH        0x00000002 /* RDR Threshold */
#define BITM_PKTE_IUMSK_STAT_CDRTHRSH        0x00000001 /* CDR Threshold */

#define BITP_PKTE_IMSK_STAT_IFERR            18 /* Interface Error */
#define BITP_PKTE_IMSK_STAT_PROCERR          17 /* PKTE Processing Error */
#define BITP_PKTE_IMSK_STAT_RINGERR          16 /* PE Ring Error */
#define BITP_PKTE_IMSK_STAT_HLT              15 /* Halt */
#define BITP_PKTE_IMSK_STAT_OBUFTHRSH        11 /* Output Buffer Threshold */
#define BITP_PKTE_IMSK_STAT_IBUFTHRSH        10 /* Input Buffer Threshold */
#define BITP_PKTE_IMSK_STAT_OPDN              9 /* Operation Done */
#define BITP_PKTE_IMSK_STAT_RDRTHRSH          1 /* RDR Threshold */
#define BITP_PKTE_IMSK_STAT_CDRTHRSH          0 /* CDR Threshold */
#define BITM_PKTE_IMSK_STAT_IFERR            0x00040000 /* Interface Error */
#define BITM_PKTE_IMSK_STAT_PROCERR          0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_IMSK_STAT_RINGERR          0x00010000 /* PE Ring Error */
#define BITM_PKTE_IMSK_STAT_HLT              0x00008000 /* Halt */
#define BITM_PKTE_IMSK_STAT_OBUFTHRSH        0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_IMSK_STAT_IBUFTHRSH        0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_IMSK_STAT_OPDN             0x00000200 /* Operation Done */
#define BITM_PKTE_IMSK_STAT_RDRTHRSH         0x00000002 /* RDR Threshold */
#define BITM_PKTE_IMSK_STAT_CDRTHRSH         0x00000001 /* CDR Threshold */

#define BITP_PKTE_INT_CLR_IFERR              18 /* Interface Error */
#define BITP_PKTE_INT_CLR_PROCERR            17 /* PKTE Processing Error */
#define BITP_PKTE_INT_CLR_RINGERR            16 /* PKTE Ring Error */
#define BITP_PKTE_INT_CLR_HLT                15 /* Halt */
#define BITP_PKTE_INT_CLR_OBUFTHRSH          11 /* Output Buffer Threshold */
#define BITP_PKTE_INT_CLR_IBUFTHRSH          10 /* Input Buffer Threshold */
#define BITP_PKTE_INT_CLR_OPDN                9 /* Operation Done */
#define BITP_PKTE_INT_CLR_RDRTHRSH            1 /* RDR Threshold */
#define BITP_PKTE_INT_CLR_CDRTHRSH            0 /* CDR Threshold */
#define BITM_PKTE_INT_CLR_IFERR              0x00040000 /* Interface Error */
#define BITM_PKTE_INT_CLR_PROCERR            0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_INT_CLR_RINGERR            0x00010000 /* PKTE Ring Error */
#define BITM_PKTE_INT_CLR_HLT                0x00008000 /* Halt */
#define BITM_PKTE_INT_CLR_OBUFTHRSH          0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_INT_CLR_IBUFTHRSH          0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_INT_CLR_OPDN               0x00000200 /* Operation Done */
#define BITM_PKTE_INT_CLR_RDRTHRSH           0x00000002 /* RDR Threshold */
#define BITM_PKTE_INT_CLR_CDRTHRSH           0x00000001 /* CDR Threshold */

#define BITP_PKTE_INT_EN_IFERR               18 /* Interface Error */
#define BITP_PKTE_INT_EN_PROCERR             17 /* PKTE Processing Error */
#define BITP_PKTE_INT_EN_RINGERR             16 /* PKTE Ring Error */
#define BITP_PKTE_INT_EN_HLT                 15 /* Halt */
#define BITP_PKTE_INT_EN_OBUFTHRSH           11 /* Output Buffer Threshold */
#define BITP_PKTE_INT_EN_IBUFTHRSH           10 /* Input Buffer Threshold */
#define BITP_PKTE_INT_EN_OPDN                 9 /* Operation Done */
#define BITP_PKTE_INT_EN_RDRTHRSH             1 /* RDR Threshold */
#define BITP_PKTE_INT_EN_CDRTHRSH             0 /* CDR Threshold */
#define BITM_PKTE_INT_EN_IFERR               0x00040000 /* Interface Error */
#define BITM_PKTE_INT_EN_PROCERR             0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_INT_EN_RINGERR             0x00010000 /* PKTE Ring Error */
#define BITM_PKTE_INT_EN_HLT                 0x00008000 /* Halt */
#define BITM_PKTE_INT_EN_OBUFTHRSH           0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_INT_EN_IBUFTHRSH           0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_INT_EN_OPDN                0x00000200 /* Operation Done */
#define BITM_PKTE_INT_EN_RDRTHRSH            0x00000002 /* RDR Threshold */
#define BITM_PKTE_INT_EN_CDRTHRSH            0x00000001 /* CDR Threshold */

#define BITP_PKTE_INT_CFG_PULSECLR            1 /* Clear After Pulse Interrupt */
#define BITP_PKTE_INT_CFG_TYPE                0 /* Interrupt Type */
#define BITM_PKTE_INT_CFG_PULSECLR            0x00000002 /* Clear After Pulse Interrupt */
#define BITM_PKTE_INT_CFG_TYPE                0x00000001 /* Interrupt Type */

#define BITP_PKTE_IMSK_EN_IFERR              18 /* Interface Error */
#define BITP_PKTE_IMSK_EN_PROCERR            17 /* PKTE Processing Error */
#define BITP_PKTE_IMSK_EN_RINGERR            16 /* PKTE Ring Error */
#define BITP_PKTE_IMSK_EN_HLT                15 /* Halt */
#define BITP_PKTE_IMSK_EN_OBUFTHRSH          11 /* Output Buffer Threshold */
#define BITP_PKTE_IMSK_EN_IBUFTHRSH          10 /* Input Buffer Threshold */
#define BITP_PKTE_IMSK_EN_OPDN                9 /* Operation Done */
#define BITP_PKTE_IMSK_EN_RDRTHRSH            1 /* RDR Threshold */
#define BITP_PKTE_IMSK_EN_CDRTHRSH            0 /* CDR Threshold */
#define BITM_PKTE_IMSK_EN_IFERR              0x00040000 /* Interface Error */
#define BITM_PKTE_IMSK_EN_PROCERR            0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_IMSK_EN_RINGERR            0x00010000 /* PKTE Ring Error */
#define BITM_PKTE_IMSK_EN_HLT                0x00008000 /* Halt */
#define BITM_PKTE_IMSK_EN_OBUFTHRSH          0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_IMSK_EN_IBUFTHRSH          0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_IMSK_EN_OPDN               0x00000200 /* Operation Done */
#define BITM_PKTE_IMSK_EN_RDRTHRSH           0x00000002 /* RDR Threshold */
#define BITM_PKTE_IMSK_EN_CDRTHRSH           0x00000001 /* CDR Threshold */

#define BITP_PKTE_IMSK_DIS_IFERR             18 /* Interface Error */
#define BITP_PKTE_IMSK_DIS_PROCERR           17 /* PKTE Processing Error */
#define BITP_PKTE_IMSK_DIS_RINGERR           16 /* PKTE Ring Error */
#define BITP_PKTE_IMSK_DIS_HLT               15 /* Halt */
#define BITP_PKTE_IMSK_DIS_OBUFTHRSH         11 /* Output Buffer Threshold */
#define BITP_PKTE_IMSK_DIS_IBUFTHRSH         10 /* Input Buffer Threshold */
#define BITP_PKTE_IMSK_DIS_OPDN               9 /* Operation Done */
#define BITP_PKTE_IMSK_DIS_RDRTHRSH           1 /* RDR Threshold */
#define BITP_PKTE_IMSK_DIS_CDRTHRSH           0 /* CDR Threshold */
#define BITM_PKTE_IMSK_DIS_IFERR             0x00040000 /* Interface Error */
#define BITM_PKTE_IMSK_DIS_PROCERR           0x00020000 /* PKTE Processing Error */
#define BITM_PKTE_IMSK_DIS_RINGERR           0x00010000 /* PKTE Ring Error */
#define BITM_PKTE_IMSK_DIS_HLT               0x00008000 /* Halt */
#define BITM_PKTE_IMSK_DIS_OBUFTHRSH         0x00000800 /* Output Buffer Threshold */
#define BITM_PKTE_IMSK_DIS_IBUFTHRSH         0x00000400 /* Input Buffer Threshold */
#define BITM_PKTE_IMSK_DIS_OPDN              0x00000200 /* Operation Done */
#define BITM_PKTE_IMSK_DIS_RDRTHRSH          0x00000002 /* RDR Threshold */
#define BITM_PKTE_IMSK_DIS_CDRTHRSH          0x00000001 /* CDR Threshold */

#define BITP_PKTE_SA_CMD0_SVHASH             29 /* Save Hash */
#define BITP_PKTE_SA_CMD0_SVIV               28 /* Save IV */
#define BITP_PKTE_SA_CMD0_HASHSRC            26 /* Hash Source */
#define BITP_PKTE_SA_CMD0_IVSRC              24 /* IV Source */
#define BITP_PKTE_SA_CMD0_DIGESTLEN          20 /* Digest Length */
#define BITP_PKTE_SA_CMD0_HDRPROC            19 /* Header Processing */
#define BITP_PKTE_SA_CMD0_EXTPAD             18 /* Extended Pad */
#define BITP_PKTE_SA_CMD0_SCPAD              17 /* Stream Cipher Padding */
#define BITP_PKTE_SA_CMD0_HASH               12 /* Hash Algorithm Select */
#define BITP_PKTE_SA_CMD0_CIPHER              8 /* Cipher Algorithm Select */
#define BITP_PKTE_SA_CMD0_PADTYPE             6 /* Pad Type */
#define BITP_PKTE_SA_CMD0_OPGRP               4 /* Operation Group */
#define BITP_PKTE_SA_CMD0_DIR                 3 /* Direction */
#define BITP_PKTE_SA_CMD0_OPCD                0 /* Operation Code */
#define BITM_PKTE_SA_CMD0_SVHASH             0x20000000 /* Save Hash */
#define BITM_PKTE_SA_CMD0_SVIV               0x10000000 /* Save IV */
#define BITM_PKTE_SA_CMD0_HASHSRC            0x0C000000 /* Hash Source */
#define BITM_PKTE_SA_CMD0_IVSRC              0x03000000 /* IV Source */
#define BITM_PKTE_SA_CMD0_DIGESTLEN          0x00F00000 /* Digest Length */
#define BITM_PKTE_SA_CMD0_HDRPROC            0x00080000 /* Header Processing */
#define BITM_PKTE_SA_CMD0_EXTPAD             0x00040000 /* Extended Pad */
#define BITM_PKTE_SA_CMD0_SCPAD              0x00020000 /* Stream Cipher Padding */
#define BITM_PKTE_SA_CMD0_HASH               0x0000F000 /* Hash Algorithm Select */
#define BITM_PKTE_SA_CMD0_CIPHER             0x00000F00 /* Cipher Algorithm Select */
#define BITM_PKTE_SA_CMD0_PADTYPE            0x000000C0 /* Pad Type */
#define BITM_PKTE_SA_CMD0_OPGRP              0x00000030 /* Operation Group */
#define BITM_PKTE_SA_CMD0_DIR                0x00000008 /* Direction */
#define BITM_PKTE_SA_CMD0_OPCD               0x00000007 /* Operation Code */

#define BITP_PKTE_SA_CMD1_ENSQNCHK           29 /* Sequence Number Check Enable */
#define BITP_PKTE_SA_CMD1_AESDECKEY          28 /* AES Dec Key */
#define BITP_PKTE_SA_CMD1_AESKEYLEN          24 /* AES Key Length */
#define BITP_PKTE_SA_CMD1_ARC4KEYLEN         24 /* ARC4 Key Length */
#define BITP_PKTE_SA_CMD1_HSHCOFFST          16 /* Hash Crypt Offset */
#define BITP_PKTE_SA_CMD1_BYTEOFFST          13 /* Byte Offset */
#define BITP_PKTE_SA_CMD1_HMAC               12 /* Keyed-Hash SSL Message Authentication Code */
#define BITP_PKTE_SA_CMD1_SSLMAC             11 /* Ssl Mac */
#define BITP_PKTE_SA_CMD1_CIPHERMD            8 /* Cipher Mode */
#define BITP_PKTE_SA_CMD1_CPYPAD              3 /* Copy Pad */
#define BITP_PKTE_SA_CMD1_CPYPAYLD            2 /* Copy Payload */
#define BITP_PKTE_SA_CMD1_CPYHDR              1 /* Copy Header */
#define BITP_PKTE_SA_CMD1_CPYDGST             0 /* Copy Digest */
#define BITM_PKTE_SA_CMD1_ENSQNCHK           0x20000000 /* Sequence Number Check Enable */
#define BITM_PKTE_SA_CMD1_AESDECKEY          0x10000000 /* AES Dec Key */
#define BITM_PKTE_SA_CMD1_AESKEYLEN          0x07000000 /* AES Key Length */
#define BITM_PKTE_SA_CMD1_ARC4KEYLEN         0x1F000000 /* ARC4 Key Length */
#define BITM_PKTE_SA_CMD1_HSHCOFFST          0x00FF0000 /* Hash Crypt Offset */
#define BITM_PKTE_SA_CMD1_BYTEOFFST          0x00002000 /* Byte Offset */
#define BITM_PKTE_SA_CMD1_HMAC               0x00001000 /* Keyed-Hash SSL MAC */
#define BITM_PKTE_SA_CMD1_SSLMAC             0x00000800 /* Ssl Mac */
#define BITM_PKTE_SA_CMD1_CIPHERMD           0x00000300 /* Cipher Mode */
#define BITM_PKTE_SA_CMD1_CPYPAD             0x00000008 /* Copy Pad */
#define BITM_PKTE_SA_CMD1_CPYPAYLD           0x00000004 /* Copy Payload */
#define BITM_PKTE_SA_CMD1_CPYHDR             0x00000002 /* Copy Header */
#define BITM_PKTE_SA_CMD1_CPYDGST            0x00000001 /* Copy Digest */

#define BITP_PKTE_SA_SPI_VALUE                0 /* SA SPI */
#define BITM_PKTE_SA_SPI_VALUE               0xFFFFFFFF /* SA SPI */

#define BITP_PKTE_SA_ARC4IJPTR_JPTR           8 /* J Pointer */
#define BITP_PKTE_SA_ARC4IJPTR_IPTR           0 /* I Pointer */
#define BITM_PKTE_SA_ARC4IJPTR_JPTR          0x0000FF00 /* J Pointer */
#define BITM_PKTE_SA_ARC4IJPTR_IPTR          0x000000FF /* I Pointer */

#define BITP_PKTE_SA_NONCE_VALUE              0 /* SA Nonce */
#define BITM_PKTE_SA_NONCE_VALUE             0xFFFFFFFF /* SA Nonce */

#define BITP_PKTE_SA_RDY_VALUE                0 /* SA Ready */
#define BITM_PKTE_SA_RDY_VALUE               0xFFFFFFFF /* SA Ready */

#define BITP_PKTE_ARC4STATE_BUF_VALUE         0 /* Buffer value */
#define BITM_PKTE_ARC4STATE_BUF_VALUE        0xFFFFFFFF /* Buffer value */

#define BITP_PKTE_DATAIO_BUF_VALUE            0 /* Buffer Value */
#define BITM_PKTE_DATAIO_BUF_VALUE           0xFFFFFFFF /* Buffer Value */

#define BITP_PKTE_SA_IDIGEST_VALUE            0 /* Inner Hash Digest */
#define BITM_PKTE_SA_IDIGEST_VALUE           0xFFFFFFFF /* Inner Hash Digest */

#define BITP_PKTE_SA_KEY_VALUE                0 /* Cipher Key */
#define BITM_PKTE_SA_KEY_VALUE               0xFFFFFFFF /* Cipher Key */

#define BITP_PKTE_SA_ODIGEST_VALUE            0 /* Outer Hash Digest */
#define BITM_PKTE_SA_ODIGEST_VALUE           0xFFFFFFFF /* Outer Hash Digest */

#define BITP_PKTE_SA_SEQNUM_VALUE             0 /* SA Sequence Number */
#define BITM_PKTE_SA_SEQNUM_VALUE            0xFFFFFFFF /* SA Sequence Number */

#define BITP_PKTE_SA_SEQNUM_MSK_VALUE         0 /* SA Sequence Number */
#define BITM_PKTE_SA_SEQNUM_MSK_VALUE        0xFFFFFFFF /* SA Sequence Number */

#define BITP_PKTE_STATE_BYTE_CNT_VALUE        0 /* STATE Byte Count */
#define BITM_PKTE_STATE_BYTE_CNT_VALUE       0xFFFFFFFF /* STATE Byte Count */

#define BITP_PKTE_STATE_IDIGEST_VALUE         0 /* Saved Inner Hash Digest */
#define BITM_PKTE_STATE_IDIGEST_VALUE        0xFFFFFFFF /* Saved Inner Hash Digest */

#define BITP_PKTE_STATE_IV_VALUE              0 /* State Initialization Vector */
#define BITM_PKTE_STATE_IV_VALUE             0xFFFFFFFF /* State Initialization Vector */

/*!
 *Configures the mode of operation as encryption
 */
#define opcode_encrypt       0x0u
/*!
 *Configures the mode of operation as decryption
 */
#define opcode_decrypt       0x0u
/*!
 *Configures the mode of operation as encryption and hash
 */
#define opcode_encrypt_hash  0x1u
/*!
 *Configures the mode of operation as hash and decryption
 */
#define opcode_hash_decrypt  0x1u
/*!
 *Configures the mode of operation as hash
 */
#define opcode_hash          0x3u
/*!
 *Configures the direction as outbound. Used for encryption and encryption-hash operation.
 */
#define dir_outbound         0x0u
/*!
 *Configures the direction as inbound. Used for decryption and hash-decryption
 */
#define dir_inbound          0x1u
/*!
 *Configures the cipher mode as DES
 */
#define cipher_des           0x0u
/*!
 *Configures the cipher mode as TDES
 */
#define cipher_tdes          0x1u
/*!
 *Configures the cipher mode as ARC4
 */
#define cipher_arc4          0x2u
/*!
 *Configures the cipher mode as AES
 */
#define cipher_aes           0x3u
/*!
 *Configures the Cipher mode as NULL. This is used when Ciphering is not required.
 */
#define cipher_null          0xFu
/*!
 *Configures the Hash mode as MD5
 */
#define hash_md5             0x0u
/*!
 *Configures the Hash mode as SHA-1
 */
#define hash_sha1            0x1u
/*!
 *Configures the SHA mode as SHA-224
 */
#define hash_sha224          0x2u
/*!
 *Configures the SHA mode as SHA-256
 */
#define hash_sha256          0x3u
/*!
 *Configures the SHA mode as NULL. This is used when Hashing is not required.
 */
#define hash_null            0xFu
/*!
 *Configures the Digest length as 3 words i.e 96 bit output
 */
#define digest_length0       0x0u
/*!
 *Configures the Digest length as 1 word
 */
#define digest_length1       0x1u
/*!
 *Configures the Digest length as 2 words
 */
#define digest_length2       0x2u
/*!
 *Configures the Digest length as 3 words
 */
#define digest_length3       0x3u
/*!
 *Configures the Digest length as 4 words
 */
#define digest_length4       0x4u
/*!
 *Configures the Digest length as 5 words
 */
#define digest_length5       0x5u
/*!
 *Configures the Digest length as 6 words
 */
#define digest_length6       0x6u
/*!
 *Configures the Digest length as 7 words
 */
#define digest_length7       0x7u
/*!
 *Configures the Digest length as 8 words
 */
#define digest_length8       0x8u
/*!
 *Configures the Ciphering in ECB mode
 */
#define cipher_mode_ecb      0x0u
/*!
 *Configures the Ciphering in CBC mode
 */
#define cipher_mode_cbc      0x1u
/*!
 *Configures the Ciphering as ARC4 in stateless mode
 */
#define cipher_mode_arc4_stateless      0x0u
/*!
 *Configures the Ciphering as ARC4 as stateful mode
 */
#define cipher_mode_arc4_stateful      0x1u
/*!
 *Configures the Hash in standard mode
 */
#define hash_mode_standard  0x0u
/*!
 *Configures the Hash in HMAC mode
 */
#define hash_mode_hmac       0x1u
/*!
 *Configures the Key length as 0
 */
#define aes_key_length_other 0x0u
/*!
 *Configures the Key length as 128 bit
 */
#define aes_key_length128    0x2u
/*!
 *Configures the Key length as 192 bit
 */
#define aes_key_length192    0x3u
/*!
 *Configures the Key length as 256 bit
 */
#define aes_key_length256    0x4u
/*!
 *Configures the Key as Encryption key
 */
#define aes_key              0x0u
/*!
 *Configures the Key as Decryption key
 */
#define des_key              0x1u
/*!
 *Configures the Source for hash from SA
 */
#define hash_source_sa       0x0u
/*!
 *Configures the Source of hashing from state
 */
#define hash_source_state    0x2u
/*!
 *Configures the source for hash operation as default
 */
#define hash_source_no_load  0x3u
/*!
 *Configures the source for hash operation as Final
 */
#define final_hash           0x1u
/*!
 *Configures the source for hash as not final. Used for intermediate blocks.
 */
#define not_final_hash       0x0u

#define BITM_PKTE_HOST_MODE 0x00
#define BITM_PKTE_TCM_MODE  0x02
#define BITM_PKTE_AUTONOMOUS_MODE 0x03

struct PE_CDR {
	u32 PE_CTRL_STAT;
	u32 PE_SOURCE_ADDR;
	u32 PE_DEST_ADDR;
	u32 PE_SA_ADDR;
	u32 PE_STATE_ADDR;
	u32 PE_ARC4_STATE_ADDR;
	u32 PE_USER_ID;
	u32 PE_LENGTH;
};

struct PE_RDR {
	u32 PE_CTRL_STAT;
	u32 PE_SOURCE_ADDR;
	u32 PE_DEST_ADDR;
	u32 PE_SA_ADDR;
	u32 PE_STATE_ADDR;
	u32 PE_ARC4_STATE_ADDR;
	u32 PE_USER_ID;
	u32 PE_LENGTH;
};

struct SA_PARA {
	u32 SA_CMD0;
	u32 SA_CMD1;
};

struct SA_KEY {
	u32 SA_KEY0;
	u32 SA_KEY1;
	u32 SA_KEY2;
	u32 SA_KEY3;
	u32 SA_KEY4;
	u32 SA_KEY5;
	u32 SA_KEY6;
	u32 SA_KEY7;
};

struct SA_IDIGEST {
	u32 SA_IDIGEST0;
	u32 SA_IDIGEST1;
	u32 SA_IDIGEST2;
	u32 SA_IDIGEST3;
	u32 SA_IDIGEST4;
	u32 SA_IDIGEST5;
	u32 SA_IDIGEST6;
	u32 SA_IDIGEST7;
};

struct SA_ODIGEST {
	u32 SA_ODIGEST0;
	u32 SA_ODIGEST1;
	u32 SA_ODIGEST2;
	u32 SA_ODIGEST3;
	u32 SA_ODIGEST4;
	u32 SA_ODIGEST5;
	u32 SA_ODIGEST6;
	u32 SA_ODIGEST7;
};

struct SA_SEQ {
	u32 SA_SEQNUM0;
	u32 SA_SEQNUM1;
	u32 SA_SEQNUMMASK0;
	u32 SA_SEQNUMMASK1;
};

struct SA {
	struct SA_PARA SA_Para;
	struct SA_KEY SA_Key;
	struct SA_IDIGEST SA_Idigest;
	struct SA_ODIGEST SA_Odigest;
	u32 SA_SPI;
	struct SA_SEQ SA_Seq;
	u32 SA_READY;
};

struct PE_CDRing {
	struct PE_CDR PE_cdr0;
	struct PE_RDR PE_rdr0;
};

struct STATE {
	u32 STATE_IV0;
	u32 STATE_IV1;
	u32 STATE_IV2;
	u32 STATE_IV3;
	u32 STATE_BYTE_CNT0;
	u32 STATE_BYTE_CNT1;
	u32 STATE_IDIGEST0;
	u32 STATE_IDIGEST1;
	u32 STATE_IDIGEST2;
	u32 STATE_IDIGEST3;
	u32 STATE_IDIGEST4;
	u32 STATE_IDIGEST5;
	u32 STATE_IDIGEST6;
	u32 STATE_IDIGEST7;
};

struct ADI_PKTE_DESCRIPTOR {
	struct PE_CDR  CmdDescriptor[PKTE_RING_BUFFERS];
	struct PE_RDR  ResultDescriptor[PKTE_RING_BUFFERS];
	struct SA      SARecord[PKTE_RING_BUFFERS];
	struct STATE   State;
};

struct ADI_PKTE_COMMAND {
	/*! Configures the mode of operation using PKTE*/
	u32 opcode;
	/*! Configures the direction as inbound or outbound*/
	u32 direction;
	/*! Configures various cipher modes - AES/DES/TDES/ARC4*/
	u32 cipher;
	/*! Configures various hash mode - SHA1/SHA224/SHA256/MD5*/
	u32 hash;
	/*! Configures the Message Digest length for Hash operation*/
	u32 digest_length;
	/*! Configures the mode as ECB/CBC for encryption operation*/
	u32 cipher_mode;
	/*! Configure the hash mode as standard mode*/
	u32 hash_mode;
	/*! Configures the Encryption key length for encryption mode*/
	u32 aes_key_length;
	/*! Configures the key as encryption OR decryption key*/
	u32 aes_des_key;
	/*! Configures the hash source*/
	u32 hash_source;
	/*! Configures the final hash condition*/
	u32 final_hash_condition;
	/*! Configures the mode of operation- Autonomous/TCM/Host mode*/
	u32 pkte_mode;
};

struct ADI_PKTE_LIST {
	/*! Pointer to the source buffer for encryption/hash operation*/
	u32 *pSource;
	/*! Pointer to the destination buffer for encryption/hash operation*/
	u32 *pDestination;
	/*! Pointer to the Initialization vector for encryption operation*/
	u32 *pIV;
	/*! Pointer to the Key for the encryption/hash operation*/
	u32 *pKey;
	/*! Pointer to the initial digest for hash operation*/
	u32 *pIdigest;
	/*! Pointer to the output digest for hash operation*/
	u32 *pOdigest;
	/*! Size of the source buffer for encryption/hash operation*/
	u32  nSrcSize;
	/*! Any user ID can be passed*/
	u32 nUserID;
	/*! Above command structure*/
	struct ADI_PKTE_COMMAND pCommand;
};

struct ADI_PKTE_DEVICE {
	struct ADI_PKTE_DESCRIPTOR   pPkteDescriptor;
	struct ADI_PKTE_LIST         pPkteList;
	u32                   source[PKTE_RING_BUFFERS][PKTE_BUFLEN/4];
	u32                   destination[PKTE_BUFLEN];
};

struct adi_ctx {
	struct crypto_engine_op enginectx;
	struct adi_dev          *pkte_dev;
	int                     keylen;
	__be32                  key[AES_KEYSIZE_256 / sizeof(u32)];
	unsigned long           flags_skcipher;
};

struct adi_request_ctx {
	struct adi_dev	*pkte_dev;
	unsigned long		op;

	u8 digest[SHA256_DIGEST_SIZE] __aligned(sizeof(u32));
	size_t			digcnt;
	size_t			bufcnt;
	size_t			buflen;

	struct scatterlist	*sg;
	unsigned int		offset;
	unsigned int		total;

	int			nents;

	u32			*hw_context;
};

struct adi_algs_info {
	struct ahash_engine_alg	*algs_list;
	size_t			size;
};

struct adi_pdata {
	struct adi_algs_info	*algs_info;
	size_t			algs_info_size;
};

struct adi_dev {
	struct list_head	list;
	struct device		*dev;
	struct clk		*clk;
	struct reset_control	*rst;
	void __iomem		*io_base;
	phys_addr_t		phys_base;

	struct ahash_request	*req;
	struct crypto_engine	*engine;

	int			err;
	unsigned long		flags;

	const struct adi_pdata	*pdata;

	struct ADI_PKTE_DEVICE *pkte_device;
	bool src_count_set;
	u32 src_bytes_available;
	u32 ring_pos_produce;
	u32 ring_pos_consume;
	dma_addr_t dma_handle;

	u8			secret_key[PKTE_MAX_KEY_SIZE];
	int			secret_keylen;
};

struct adi_drv {
	struct list_head	dev_list;
	spinlock_t		lock; /* List protection access */
};

void adi_reset_state(struct adi_dev *hdev);
void adi_start_engine(struct adi_dev *pkte_dev);
void adi_init_spe(struct adi_dev *pkte_dev);
void adi_init_ring(struct adi_dev *pkte_dev);
void adi_configure_cdr(struct adi_dev *pkte_dev);
void adi_source_data(struct adi_dev *pkte_dev, u32 size);
void adi_config_sa_para(struct adi_dev *pkte_dev);
void adi_config_sa_key(struct adi_dev *pkte_dev, u32 Key[]);
void adi_config_state(struct adi_dev *pkte_dev, u32 IV[]);
void adi_write(struct adi_dev *pkte_dev, u32 offset, u32 value);

u32 adi_read(struct adi_dev *pkte_dev, u32 offset);
u32 adi_physical_address(struct adi_dev *pkte_dev, void *variableAddress);

struct adi_dev *adi_find_dev(struct adi_ctx *ctx);

int adi_hw_init(struct adi_dev *pkte_dev);

extern wait_queue_head_t wq_processing;
extern int processing;
extern wait_queue_head_t wq_ready;
extern bool ready;
extern struct adi_drv adi;

extern u32 Key[8];
extern u32 IV[4];
extern u32 IDigest[8];
extern u32 ODigest[8];

#endif
