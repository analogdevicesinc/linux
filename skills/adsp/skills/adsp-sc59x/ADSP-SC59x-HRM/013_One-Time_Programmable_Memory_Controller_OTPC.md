# One-Time Programmable Memory Controller (OTPC)

<!-- source: 013_One-Time_Programmable_Memory_Controller_OTPC.pdf | original pages 504–511 -->

## 11   One-Time Programmable Memory Controller (OTPC)

This chapter describes the operation of the OTP controller. The OTP module is a complete system integrating an OTP memory core with a programming controller, charge pump, and voltage regulator. A built-in Hamming Code Error Correction (ECC), and a fully implemented double-redundant program or read scheme protect the OTP data.

CAUTION: OTP memory does not support burst transfers, which are required to support cache line fills. As such, OTP memory should not be made cacheable. If it is, the OTP controller returns an error when a read access is attempted.

## OTPC Features

The OTP memory and controller have the following features:

- Built-in redundant read mode
- Built-in integrated power supply
- Built-in Hamming Code Error Correction (ECC)
- Full word serial (single bit at a time) programming with internal VPP

NOTE: The OTP module is protected by the System Memory Protection Unit (SMPU) and is always secure by default. Non secure access to the OTP by any module will fail unless allowed by the SMPU.

## OTPC Functional Description

## ADSP-SC59x OTPC Register List

The One-Time-Programmable Memory controller (OTPC) supports programming the OTP memory. A set of registers governs OTPC operations. For more information on OTPC functionality, see the OTPC register descriptions.

Table 11-1: ADSP-SC59x OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## ADSP-SC59x OTPC Interrupt List

Table 11-2: ADSP-SC59x OTPC Interrupt List

|   Interrupt ID | Name      | Description          | Sensitivity   | DMA Channel   |
|----------------|-----------|----------------------|---------------|---------------|
|              6 | OTPC0_ERR | OTPC0 Dual-bit Error | Level         |               |

## OTPC Memory Layout

This section details the memory layout of the OTPC memory.

Table 11-3: OTPC Memory Layout

| Name             | 32 bit aligned Byte Address   |   Size (bits) | Description                   |
|------------------|-------------------------------|---------------|-------------------------------|
| huk              | 0x5c                          |           256 | Hardware Unique Key           |
| rkek             | 0x7c                          |           128 | Root Key Encryption Key       |
| dek              | 0x8c                          |           128 | Local Encryption Key          |
| oem_public_key   | 0x9c                          |           512 | OEMPublic Key                 |
| pvt_128key0      | 0xdc                          |           128 | Customer Privatekey 0 128bits |
| pvt_128key1      | 0xec                          |           128 | Customer Privatekey 1 128bits |
| pvt_128key2      | 0xfc                          |           128 | Customer Privatekey 2 128bits |
| pvt_128key3      | 0x10c                         |           128 | Customer Privatekey 3 128bits |
| ek               | 0x11c                         |           256 | Endorsement Key               |
| secure_emu_key0  | 0x13c                         |           128 | Secure Emulation Key0         |
| secure_emu_key1  | 0x14c                         |           128 | Secure Emulation Key1         |
| reserved         | 0x160                         |           928 | reserved                      |
| emu_key_disable0 | 0x1d4                         |            16 | Secure Emulation Key0 Disable |
| emu_key_disable1 | 0x1d4                         |            16 | Secure Emulation Key1 Disable |
| public_key0      | 0x1d8                         |           512 | Customer Public Key0          |
| public_key1      | 0x218                         |           512 | Customer Public Key1          |
| reserved         | 0x258                         |           128 | reserved                      |
| cgu_ctl_WEN      | 0x268                         |             1 | CGU Config ctl_WEN            |

Table 11-3: OTPC Memory Layout (Continued)

| Name                     | 32 bit aligned Byte Address   |   Size (bits) | Description                     |
|--------------------------|-------------------------------|---------------|---------------------------------|
| cgu_div_WEN              | 0x268                         |             1 | CGU Config div_WEN              |
| cgu_reserved0            | 0x268                         |             1 | CGU Config reserved0            |
| cgu_div_DSEL             | 0x268                         |             5 | CGU Config div_DSEL             |
| cgu_div_CSEL             | 0x268                         |             5 | CGU Config div_CSEL             |
| cgu_div_S0SEL            | 0x268                         |             3 | CGU Config div_S0SEL            |
| cgu_div_SYSSEL           | 0x268                         |             5 | CGU Config div_SYSSEL           |
| cgu_div_S1SEL            | 0x268                         |             3 | CGU Config div_S1SEL            |
| cgu_div_OSEL             | 0x268                         |             7 | CGU Config div_OSEL             |
| cgu_ctl_DF               | 0x268                         |             1 | CGU Config ctl_DF               |
| cgu_ctl_MSEL             | 0x26c                         |             7 | CGU Config ctl_MSEL             |
| cgu_ctl_S0SELEXEN        | 0x26c                         |             1 | CGU Config ctl_S0SELEXEN        |
| cgu_ctl_S1SELEXEN        | 0x26c                         |             1 | CGU Config ctl_S1SELEXEN        |
| cgu_auto_disable         | 0x26c                         |             1 | CGU Disable Auto Alignment      |
| cgu_clkoutsel_CLKOUTSEL  | 0x26c                         |             5 | CGU Config clkoutsel_CLKOUTSEL  |
| cgu_clkoutsel_WEN        | 0x26c                         |             1 | CGU Config clkoutsel_WEN        |
| cgu_divex_S0SELEX        | 0x26c                         |             8 | CGU Config divex_S0SELEX        |
| cgu_divex_S1SELEX        | 0x26c                         |             8 | CGU Config divex_S1SELEX        |
| cgu_oscwctl0_WEN         | 0x270                         |             1 | CGU Config oscwctl0_WEN         |
| cgu_oscwctl0_HODF        | 0x270                         |             6 | CGU Config oscwctl0_HODF        |
| cgu_oscwctl0_HODEN       | 0x270                         |             1 | CGU Config oscwctl0_HODEN       |
| cgu_oscwctl0_CNGEN       | 0x270                         |             1 | CGU Config oscwctl0_CNGEN       |
| cgu_oscwctl0_BOUF        | 0x270                         |             5 | CGU Config oscwctl0_BOUF        |
| cgu_oscwctl0_BOUEN       | 0x270                         |             1 | CGU Config oscwctl0_BOUEN       |
| cgu_oscwctl0_FAULTEN     | 0x270                         |             1 | CGU Config oscwctl0_FAULTEN     |
| cgu_oscwctl0_MONDIS      | 0x270                         |             1 | CGU Config oscwctl0_MONDIS      |
| cgu_oscwctl0_FAULTPINDIS | 0x270                         |             1 | CGU Config oscwctl0_FAULTPINDIS |
| cgu_reserved3            | 0x270                         |            14 | CGU Config reserved3            |
| cgu_oscwctl1_WEN         | 0x274                         |             1 | CGU Config oscwctl1_WEN         |
| cgu_oscwctl1_HODF        | 0x274                         |             6 | CGU Config oscwctl1_HODF        |
| cgu_oscwctl1_HODEN       | 0x274                         |             1 | CGU Config oscwctl1_HODEN       |
| cgu_oscwctl1_CNGEN       | 0x274                         |             1 | CGU Config oscwctl1_CNGEN       |

Table 11-3: OTPC Memory Layout (Continued)

| Name                     | 32 bit aligned Byte Address   |   Size (bits) | Description                          |
|--------------------------|-------------------------------|---------------|--------------------------------------|
| cgu_oscwctl1_BOUF        | 0x274                         |             5 | CGU Config oscwctl1_BOUF             |
| cgu_oscwctl1_BOUEN       | 0x274                         |             1 | CGU Config oscwctl1_BOUEN            |
| cgu_oscwctl1_FAULTEN     | 0x274                         |             1 | CGU Config oscwctl1_FAULTEN          |
| cgu_oscwctl1_MONDIS      | 0x274                         |             1 | CGU Config oscwctl1_MONDIS           |
| cgu_oscwctl1_FAULTPINDIS | 0x274                         |             1 | CGU Config oscwctl1_FAULTPINDIS      |
| cgu_reserved4            | 0x274                         |            14 | CGU Config reserved4                 |
| flashStartAddress        | 0x278                         |            32 | Flash Start Address Override         |
| spiFlashBootCmd          | 0x27c                         |            32 | SPI Master Boot Command Override     |
| spiHostBootCmd           | 0x280                         |            32 | SPI Slave Boot Command Override      |
| lpBootCmd                | 0x284                         |            32 | LinkPort Boot Command Override       |
| uartBootCmd              | 0x288                         |            32 | UART Boot Command Override           |
| ospiHostBootCmd          | 0x28c                         |            32 | OSPI Boot Command Override           |
| emmcHostBootCmd          | 0x290                         |            32 | EMSI Boot Command Override           |
| emmcStartAddress         | 0x294                         |            32 | EMSI Start Address Override          |
| lpddrBootCmd             | 0x298                         |            32 | LinkPort DDR Boot Command Over- ride |
| ospi_read_data_capture   | 0x29c                         |            16 | OSPI Read Data Capture Register      |
| ospi_rdcr_reserved0      | 0x29c                         |            16 | OSPI RDCR Reserved0                  |
| bcfg_lockMonitor         | 0x2a0                         |             1 | Boot Config lockMonitor              |
| bcfg_reserved0           | 0x2a0                         |            15 | Boot Config reserved0                |
| bcfg_dstCompare          | 0x2a0                         |             1 | Boot Config dstCompare               |
| bcfg_reserved1           | 0x2a0                         |            15 | Boot Config reserved1                |
| bcfg_pubkey0Inv          | 0x2a4                         |             1 | Boot Config pubkey0Inv               |
| bcfg_reserved2           | 0x2a4                         |            15 | Boot Config reserved2                |
| bcfg_pubkey1Inv          | 0x2a4                         |             1 | Boot Config pubkey1Inv               |
| bcfg_reserved3           | 0x2a4                         |            15 | Boot Config reserved3                |
| bcfg_privkey0Inv         | 0x2a8                         |             1 | Boot Config privkey0Inv              |
| bcfg_reserved4           | 0x2a8                         |            15 | Boot Config reserved4                |
| bcfg_privkey1Inv         | 0x2a8                         |             1 | Boot Config privkey1Inv              |
| bcfg_reserved5           | 0x2a8                         |            15 | Boot Config reserved5                |
| bcfg_privkey2Inv         | 0x2ac                         |             1 | Boot Config privkey2Inv              |
| bcfg_reserved6           | 0x2ac                         |            15 | Boot Config reserved6                |

Table 11-3: OTPC Memory Layout (Continued)

| Name             | 32 bit aligned Byte Address   |   Size (bits) | Description                   |
|------------------|-------------------------------|---------------|-------------------------------|
| bcfg_privkey3Inv | 0x2ac                         |             1 | Boot Config privkey3Inv       |
| bcfg_reserved7   | 0x2ac                         |            15 | Boot Config reserved7         |
| bcfg_dmcEn       | 0x2b0                         |             1 | Boot Config dmcEn             |
| bcfg_reserved8   | 0x2b0                         |            15 | Boot Config reserved8         |
| bcfg_dmcInv      | 0x2b0                         |             1 | Boot Config dmcInv            |
| bcfg_reserved9   | 0x2b0                         |            15 | Boot Config reserved9         |
| bcfg_dmcDecDis   | 0x2b4                         |             1 | Boot Config dmcDecDis         |
| bcfg_reserved10  | 0x2b4                         |            31 | Boot Config reserved10        |
| reserved         | 0x2b8                         |            64 | reserved                      |
| antiroll_nv_cntr | 0x2c0                         |           512 | AntiRollback NV Counter       |
| gp1              | 0x300                         |           512 | General Purpose 1             |
| reserved         | 0x340                         |            24 | reserved                      |
| bootModeDisable  | 0x340                         |             8 | Boot Mode Disable Bits        |
| preboot_ddr_cfg  | 0x344                         |           384 | User PrebootDDR configuration |
| stageID          | 0x374                         |            48 | StageID                       |
| reserved         | 0x378                         |            16 | reserved                      |
| fsn              | 0x380                         |           128 | Factory Serial Number         |
| lock             | 0x48c                         |             1 | lockbit                       |
| reserved         | 0x48c                         |            15 | reserved                      |
| FA_EN            | 0x48c                         |             1 | Field Analysis Enable         |
| reserved         | 0x48c                         |            15 | reserved                      |

## Error Correction

The OTP memory features a Hamming error correction implementation. Signal bit errors are automatically corrected, and dual-bit errors are detected. Refer to OTPC Interrupt Signals.

ECC is always enabled. ECC applies to each 16-bit segment. Because of this functionality, each 16-bit location can only be written to once. Writing to a 16-bit location a second time results in unexpected behavior.

## OTPC Event Control

The following sections provide information on OTP events and error management.

## OTPC Interrupt Signals

When making 32-bit accesses to OTP memory, a double-bit error in any 16-bit segment triggers the OTPC\_INT interrupt. The OTPC also has the OTPC dual bit error (OTPC0\_ERR) with the interrupt ID of 6. See the System Event Controller (SEC) chapter for more information.

## OTPC Status and Error Signals

The OTP controller does not produce error signals.

## OTPC Programming Model

OTP programming is supported through the SSLDD OTP library which is a part of CCES 2.11.0.

## ADSP-SC59x OTPC Register Descriptions

OTP Memory Controller (OTPC) contains the following registers.

Table 11-4: ADSP-SC59x OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## OTP Security State Register

The OTPC\_SECU\_STATE register indicates the secure state. The register is updated only after the part is rebooted and locked.

Figure 11-1: OTPC\_SECU\_STATE Register Diagram

<!-- image -->

Table 11-5: OTPC\_SECU\_STATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                         |
|--------------------|------------|-------------------------------------------------|
| 1:0                | PARTLOCK   | Part Locked. The field indicates a locked part. |
| (R/NW)             |            | OTPC_SECU_STATE.PARTLOCK                        |
|                    |            | 0 OPEN part                                     |
|                    |            | 1 Locked part                                   |
|                    |            | 2 Reserved                                      |

## OTP Status Register

The OTPC\_STAT register bits indicate errors and flag status and control the protection bits.

Figure 11-2: OTPC\_STAT Register Diagram

<!-- image -->

Table 11-6: OTPC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:13 (R/NW)       | ADDRERR    | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. |
|                    |            | 0                                                                                                                                                                    | No error - proper OTP address                                                                                                                                        |
|                    |            | 1                                                                                                                                                                    | OTP address out of range                                                                                                                                             |
|                    |            | 2                                                                                                                                                                    | 8-bit OTP address                                                                                                                                                    |
|                    |            | 3                                                                                                                                                                    | Protected OTP address                                                                                                                                                |