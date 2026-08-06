# One-Time Programmable Memory Controller (OTPC)

<!-- source: 013_One-Time_Programmable_Memory_Controller_OTPC.pdf | original pages 465–476 -->

## 11   One-Time Programmable Memory Controller (OTPC)

This chapter describes the operation of the OTP controller. The OTP module is a complete system integrating an OTP memory core with a programming controller, charge pump, and voltage regulator. A built-in Hamming Code Error Correction (ECC), and a fully implemented double-redundant program or read scheme protect the OTP data.

OTP memory access is through the OTP API Overview provided by the ROM.

CAUTION: OTP memory does not support burst transfers, which are required to support cache line fills. As such, OTP memory should not be made cacheable. If it is, the OTP controller returns an error when a read access is attempted.

## OTPC Features

The OTP memory and controller have the following features:

- Built-in redundant read mode
- Built-in integrated power supply
- Built-in Hamming Code Error Correction (ECC)
- Full word serial (single bit at a time) programming with internal VPP

NOTE: The OTP module is protected by the System Memory Protection Unit (SMPU) and is always secure by default. Non secure access to the OTP by any module will fail unless allowed by the SMPU.

## OTPC Functional Description

## ADSP-2159x\_SC592\_SC594 OTPC Register List

The One-Time-Programmable Memory controller (OTPC) supports programming the OTP memory. A set of registers governs OTPC operations. For more information on OTPC functionality, see the OTPC register descriptions.

Table 11-1: ADSP-2159x\_SC592\_SC594 OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## ADSP-2159x\_SC592\_SC594 OTPC Interrupt List

Table 11-2: ADSP-2159x\_SC592\_SC594 OTPC Interrupt List

|   Interrupt ID | Name      | Description          | Sensitivity   | DMA Channel   |
|----------------|-----------|----------------------|---------------|---------------|
|              6 | OTPC0_ERR | OTPC0 Dual-bit Error | Level         |               |

## Error Correction

The OTP memory features a Hamming error correction implementation. Signal bit errors are automatically corrected, and dual-bit errors are detected. Refer to OTPC Interrupt Signals.

ECC is always enabled. ECC applies to each 16-bit segment. Because of this functionality, each 16-bit location can only be written to once. Writing to a 16-bit location a second time results in unexpected behavior.

## OTP Layout

This section details the memory layout of the OTP memory.

Table 11-3: ADSP -2159x OTP Layout

| Name            |   Size (bits) | 32-bit Aligned Byte Address   | Description                    |
|-----------------|---------------|-------------------------------|--------------------------------|
| huk             |           256 | 0x5c                          | Hardware Unique Key            |
| rkek            |           128 | 0x7c                          | Root Key encryption key        |
| dek             |           128 | 0x8c                          | Local encryption key           |
| oem_public_key  |           512 | 0x9c                          | OEMPublic Key                  |
| pvt_128key0     |           128 | 0xdc                          | Customer Privatekey 0 128 bits |
| pvt_128key1     |           128 | 0xec                          | Customer Privatekey 1 128 bits |
| pvt_128key2     |           128 | 0xfc                          | Customer Privatekey 2 128 bits |
| pvt_128key3     |           128 | 0x10c                         | Customer Privatekey 3 128 bits |
| ek              |           256 | 0x11c                         | Endorsement Key                |
| secure_emu_key0 |           128 | 0x13c                         | Secure Emulation Key0          |
| secure_emu_key1 |           128 | 0x14c                         | Secure Emulation Key1          |

Table 11-3: ADSP -2159x OTP Layout (Continued)

| Name                      |   Size (bits) | 32-bit Aligned Byte Address   | Description                    |
|---------------------------|---------------|-------------------------------|--------------------------------|
| reserved                  |           832 | 0x160                         | Reserved                       |
| emu_key0_disable          |            16 | 0x1d4                         | Secure emulation key0 disable  |
| emu_key1_disable          |            16 | 0x1d4                         | Secure emulation key1 disable  |
| public_key0               |           512 | 0x1d8                         | Customer Public Key0           |
| public_key1               |           512 | 0x218                         | Customer Public Key1           |
| reserved                  |            64 | 0x258                         | Reserved                       |
| reserved                  |           192 | 0x260                         | Reserved                       |
| cgu_ctl_WEN               |             1 | 0x278                         | CGU Config ctl_WEN             |
| cgu_div_WEN               |             1 | 0x278                         | CGU Config div_WEN             |
| cgu_reserved0             |             1 | 0x278                         | CGU Config reserved0           |
| cgu_div_DSEL              |             5 | 0x278                         | CGU Config div_DSEL            |
| cgu_div_CSEL              |             5 | 0x278                         | CGU Config div_CSEL            |
| cgu_div_S0SEL             |             3 | 0x278                         | CGU Config div_S0SEL           |
| cgu_div_SYSSEL            |             5 | 0x278                         | CGU Config div_SYSSEL          |
| cgu_div_S1SEL             |             3 | 0x278                         | CGU Config div_S1SEL           |
| cgu_div_OSEL              |             7 | 0x278                         | CGU Config div_OSEL            |
| cgu_ctl_DF                |             1 | 0x278                         | CGU Config ctl_DF              |
| cgu_ctl_MSEL              |             7 | 0x27c                         | CGU Config ctl_MSEL            |
| cgu_auto_disable          |             1 | 0x27c                         | CGU Disable Auto Alignment     |
| cgu_reserved1             |             6 | 0x27c                         | CGU Config reserved1           |
| cgu_clkout- sel_CLKOUTSEL |             5 | 0x27c                         | CGU Config clkoutsel_CLKOUTSEL |
| cgu_clkoutsel_WEN         |             1 | 0x27c                         | CGU Config clkoutsel_WEN       |
| cgu_reserved2             |            12 | 0x27c                         | CGU Config reserved2           |
| cgu_oscwctl0_WEN          |             1 | 0x280                         | CGU Config oscwctl0_WEN        |
| cgu_oscwctl0_HOD F        |             6 | 0x280                         | CGU Config oscwctl0_HODF       |
| cgu_oscwctl0_HO- DEN      |             1 | 0x280                         | CGU Config oscwctl0_HODEN      |
| cgu_oscwctl0_CNG EN       |             1 | 0x280                         | CGU Config oscwctl0_CNGEN      |
| cgu_oscwctl0_BOUF         |             5 | 0x280                         | CGU Config oscwctl0_BOUF       |

Table 11-3: ADSP -2159x OTP Layout (Continued)

| Name                      |   Size (bits) | 32-bit Aligned Byte Address   | Description                       |
|---------------------------|---------------|-------------------------------|-----------------------------------|
| cgu_oscwctl0_BOU EN       |             1 | 0x280                         | CGU Config oscwctl0_BOUEN         |
| cgu_oscwctl0_FAUL TEN     |             1 | 0x280                         | CGU Config oscwctl0_FAULTEN       |
| cgu_oscwctl0_MON DIS      |             1 | 0x280                         | CGU Config oscwctl0_MONDIS        |
| cgu_oscwctl0_FAUL TPINDIS |             1 | 0x280                         | CGU Config oscwctl0_FAULTPINDIS   |
| cgu_reserved3             |            14 | 0x280                         | CGU Config reserved3              |
| cgu_oscwctl1_WEN          |             1 | 0x284                         | CGU Config oscwctl1_WEN           |
| cgu_oscwctl1_HOD F        |             6 | 0x284                         | CGU Config oscwctl1_HODF          |
| cgu_oscwctl1_HO- DEN      |             1 | 0x284                         | CGU Config oscwctl1_HODEN         |
| cgu_oscwctl1_CNG EN       |             1 | 0x284                         | CGU Config oscwctl1_CNGEN         |
| cgu_oscwctl1_BOUF         |             5 | 0x284                         | CGU Config oscwctl1_BOUF          |
| cgu_oscwctl1_BOU EN       |             1 | 0x284                         | CGU Config oscwctl1_BOUEN         |
| cgu_oscwctl1_FAUL TEN     |             1 | 0x284                         | CGU Config oscwctl1_FAULTEN       |
| cgu_oscwctl1_MON DIS      |             1 | 0x284                         | CGU Config oscwctl1_MONDIS        |
| cgu_oscwctl1_FAUL TPINDIS |             1 | 0x284                         | CGU Config oscwctl1_FAULTPINDIS   |
| cgu_reserved4             |            14 | 0x284                         | CGU Config reserved3              |
| flashStartAddress         |            32 | 0x288                         | Flash Start Address Override      |
| spiMasterBootCmd          |            32 | 0x28c                         | SPI Master Boot Command Override  |
| spiSlaveBootCmd           |            32 | 0x290                         | SPI Slave Boot Command Override   |
| lpBootCmd                 |            32 | 0x294                         | LinkPort Boot Command Override    |
| uartBootCmd               |            32 | 0x298                         | UART Boot Command Override        |
| ospiMasterBootCmd         |            32 | 0x29c                         | OSPI Master Boot Command Override |
| ospi_read_data_cap- ture  |            16 | 0x2a0                         | OSPI Read Data Capture Register   |
| ospi_rdcr_reserved0       |            16 | 0x2a0                         | OSPI RDCR reserved0               |

Table 11-3: ADSP -2159x OTP Layout (Continued)

| Name             |   Size (bits) | 32-bit Aligned Byte Address   | Description                                |
|------------------|---------------|-------------------------------|--------------------------------------------|
| bcfg_lockMonitor |             1 | 0x2a4                         | Boot Config lockMonitor                    |
| bcfg_reserved0   |            31 | 0x2a4                         | Boot Config reserved2                      |
| bcfg_pubkey0Inv  |             1 | 0x2a8                         | Boot Config pubkey0Inv                     |
| bcfg_reserved1   |            15 | 0x2a8                         | Boot Config reserved4                      |
| bcfg_pubkey1Inv  |             1 | 0x2a8                         | Boot Config pubkey1Inv                     |
| bcfg_reserved2   |            15 | 0x2a8                         | Boot Config reserved5                      |
| bcfg_privkey0Inv |             1 | 0x2ac                         | Boot Config privkey0Inv                    |
| bcfg_reserved3   |            15 | 0x2ac                         | Boot Config reserved6                      |
| bcfg_privkey1Inv |             1 | 0x2ac                         | Boot Config privkey1Inv                    |
| bcfg_reserved4   |            15 | 0x2ac                         | Boot Config reserved7                      |
| bcfg_privkey2Inv |             1 | 0x2b0                         | Boot Config privkey2Inv                    |
| bcfg_reserved5   |            15 | 0x2b0                         | Boot Config reserved8                      |
| bcfg_privkey3Inv |             1 | 0x2b0                         | Boot Config privkey3Inv                    |
| bcfg_reserved6   |            15 | 0x2b0                         | Boot Config reserved9                      |
| bcfg_dmcEn       |             1 | 0x2b4                         | Boot Config dmcEn                          |
| bcfg_reserved7   |            15 | 0x2b4                         | Boot Config reserved10                     |
| bcfg_dmcInv      |             1 | 0x2b4                         | Boot Config dmcInv                         |
| bcfg_reserved8   |            15 | 0x2b4                         | Boot Config reserved11                     |
| reserved         |            32 | 0x2b8                         | Reserved                                   |
| otpTiming        |            16 | 0x2bc                         | otpTiming                                  |
| reserved         |            16 | 0x2bc                         | Reserved                                   |
| antiroll_nv_cntr |           512 | 0x2c0                         | AntiRollback NV Counter                    |
| gp1              |           512 | 0x300                         | General Purpose 1                          |
| reserved         |            24 | 0x340                         | Reserved                                   |
| bootModeDisable  |             8 | 0x340                         | Boot Mode Disable Bits                     |
| DDR_DLLCTLCF G   |            32 | 0x344                         | Content of DDR DLLCTL and DMC_CFG register |
| DDR_EMR2EMR3     |            32 | 0x348                         | Content of the DDR EMR2 and EMR3 Register  |
| DDR_CTL          |            32 | 0x34c                         | Content of the DDR Control                 |
| DDR_MREMR1       |            32 | 0x350                         | Content of the DDR MRand EMR1 Register     |
| DDR_TR0          |            32 | 0x354                         | Content of the DDR Timing0 Register        |

Table 11-3: ADSP -2159x OTP Layout (Continued)

| Name                    |   Size (bits) | 32-bit Aligned Byte Address   | Description                               |
|-------------------------|---------------|-------------------------------|-------------------------------------------|
| DDR_TR1                 |            32 | 0x358                         | Content of the DDR Timing1 Register       |
| DDR_TR2                 |            32 | 0x35c                         | Content of the DDR Timing2 Register       |
| DDR_ZQCTL0              |            32 | 0x360                         | Content of ZQCTL0 register                |
| DDR_ZQCTL1              |            32 | 0x364                         | Content of ZQCTL1 register                |
| DDR_ZQCTL2              |            32 | 0x368                         | Content of ZQCTL2 register                |
| DDRPHY_CACTL            |            32 | 0x36c                         | Content of DDRPHY_DDR_CA_CTL register     |
| BypassDe- lay_LANE0CTL1 |             6 | 0x370                         | Content of DDR_LANE0_CTL1[15:10] register |
| BypassDe- lay_LANE1CTL1 |             6 | 0x370                         | Content of DDR_LANE1_CTL1[15:10] register |
| BypassDe- lay_LANE0CTL0 |             6 | 0x370                         | Content of DDR_LANE0_CTL1[15:10] register |
| BypassDe- lay_LANE1CTL0 |             6 | 0x370                         | Content of DDR_LANE1_CTL1[15:10] register |
| reserved0               |             8 | 0x370                         | DDR config reserved0                      |
| stageID                 |            48 | 0x374                         | StageID                                   |
| reserved                |            16 | 0x378                         | Reserved                                  |
| fsn                     |           128 | 0x380                         | Factory Serial Number                     |

## OTPC Event Control

The following sections provide information on OTP events and error management.

## OTPC Interrupt Signals

When making 32-bit accesses to OTP memory, a double-bit error in any 16-bit segment triggers the OTPC\_INT interrupt. The OTPC also has the OTPC dual bit error (OTPC0\_ERR) with the interrupt ID of 6. See the System Event Controller (SEC) and Generic Interrupt Controller (GIC) chapter for more information.

## OTPC Status and Error Signals

The OTP controller does not produce error signals.

## OTP API Overview

The ROM/OTP SSLDD library provides a set of functions to facilitate OTP field access. The OTP memory is broken up into a set of specialized fields that are described in this section. The API removes the requirement of understanding the details of the layout or OTP access procedures.

All OTP accesses are made through the provided API.

## OTP Programming

The OTP programming API provides a simple access, abstracting particulars of the OTP controller.

Any fields that contain zero or null pointers are skipped.

All addresses are assumed to be byte addresses unless otherwise noted.

## OTP Program

Program OTP memory using a struct containing the following predefined data fields.

| Name               | OTP Program                           | -                                          |
|--------------------|---------------------------------------|--------------------------------------------|
| PP Define          | FUNC_ROM_OTPPGM                       |                                            |
| Prototype          | bool adi_rom_otp_pgm(otp_data* data); | -                                          |
| Argument           | data                                  | struct containing data to program OTP with |
| Return Value       | bool                                  | true for programming success               |
| Stack Requirements | valid stack                           | -                                          |

```
bool res = adi_rom_otp_pgm(data);
```

The following type of struct is available for programming. Refer to the ROM header file for the exact definition.

```
typedef struct { void     *reserved2; uint32_t ospi_read_data_capture:16; uint32_t (*huk)[ROM_OTP_SZ_huk]; uint32_t (*pvt_128key0)[ROM_OTP_SZ_pvt_128key0]; uint32_t (*pvt_128key1)[ROM_OTP_SZ_pvt_128key1]; uint32_t (*pvt_128key2)[ROM_OTP_SZ_pvt_128key2]; uint32_t (*pvt_128key3)[ROM_OTP_SZ_pvt_128key3]; uint32_t (*ek)[ROM_OTP_SZ_ek]; uint32_t (*secure_emu_key)[ROM_OTP_SZ_secure_emu_key]; uint32_t emu_key_disable:16; uint32_t (*public_key0)[ROM_OTP_SZ_public_key0]; uint32_t (*public_key1)[ROM_OTP_SZ_public_key1]; uint32_t (*boot_info)[ROM_OTP_SZ_boot_info]; uint8_t antiroll_nv_cntr; uint32_t (*gp1)[ROM_OTP_SZ_gp1]; uint32_t bootModeDisable:8;
```

```
uint32_t (*preboot_ddr_cfg)[ROM_OTP_SZ_preboot_ddr_cfg]; uint32_t (*stageID)[ROM_OTP_SZ_stageID]; } otp_data;
```

NOTE: Make OTP memory a non-cacheable region if the core needs access to it.

## OTP Reading

This API provides a unified source for retrieving OTP data fields.

All addresses are assumed to be byte addresses, unless otherwise noted.

A list of APIs follow:

| bool adi_rom_otp_get( OTPCMD cmd, uint32_t* data);   | OTP Get Field   |
|------------------------------------------------------|-----------------|

## OTP Get Field

Retrieves indicated data from OTP memory.

| Name               | OTP Get Field                                      |                                                         |
|--------------------|----------------------------------------------------|---------------------------------------------------------|
| Prototype          | bool adi_rom_otp_get( OTPCMD cmd, uint32_t* data); |                                                         |
| Argument           | cmd                                                | Indicates what data to fetch, based on the OTPCMD enum. |
| Argument           | data                                               | memory location to write the data to                    |
| Return Value       | bool                                               | true for a successful read                              |
| Stack Requirements | valid stack                                        |                                                         |

```
bool res = adi_rom_otp_get(otpcmd_info,data);
```

The data specified by the OTPCMD enum parameter is fetched from OTP memory and placed in the location specified by data. The OTPCMD enum contains entries for each field defined in OTP memory, for the most current list, refer to the OTP header file.

An example of the enum style follows:

```
typedef enum { otpcmd_huk = 2,           /*!< Hardware Unique Key */ otpcmd_rkek,              /*!< Root Key encryption key */ otpcmd_dek,               /*!< Local encryption key  */ otpcmd_oem_public_key,    /*!< OEM Public Key */ otpcmd_pvt_128key0,       /*!< Customer Privatekey 0 128bits */ otpcmd_pvt_128key1,       /*!< Customer Privatekey 1 128bits */ otpcmd_pvt_128key2,       /*!< Customer Privatekey 2 128bits */
```

```
otpcmd_pvt_128key3,       /*!< Customer Privatekey 3 128bits */ otpcmd_ek,                /*!< Endorsement Key */ otpcmd_secure_emu_key0,   /*!< Secure Emulation Key0 */ otpcmd_secure_emu_key1,   /*!< Secure Emulation Key1 */ otpcmd_emu_key0_disable,  /*!< Secure emulation key0 disable */ otpcmd_emu_key1_disable,  /*!< Secure emulation key1 disable */ otpcmd_public_key0,       /*!< Customer Public Key0 */ otpcmd_public_key1,       /*!< Customer Public Key1 */ otpcmd_boot_info,         /*!< Customer Programmable Boot Information */ otpcmd_otpTiming,         /*!< OTP Read timing override */ otpcmd_antiroll_nv_cntr,  /*!< AntiRollback NV Counter */ otpcmd_gp1,               /*!< General Purpose 1 */ otpcmd_bootModeDisable,   /*!< Boot Mode Disable Bits */ otpcmd_preboot_ddr_cfg,   /*!< User PrebootDDR configuration */ otpcmd_stageID            /*!< StageID */ } OTPCMD;
```

## OTP Counters

The OTPC module implements a counter API to allow easy reading or writing of the counter without dealing with the complexities of rewriting OTP memory sections that are ECC protected.

The OTPC module provides two functional APIs for counters. These APIs are not extra; the module uses the same get and pgm APIs. The APIs are functionally unique in the way that they set and retrieve data as counters in OTP memory.

The API uses a different method to count bits because each bit in OTP memory can only be set =1 once, and the ECC protects each 16-bit unit. This functionality essentially means that each 16-bit unit can only be written to once. Therefore, a counter that can count 0-31 requires 32 × 16 bits of memory.

The API receives and returns the value of the counter as a uint8\_t binary number. Writing a value less than the current value of the counter or greater than the maximum value results in an error.

To implement this functionality, the driver counts by shifting 1's from the left, treating each block as 1 bit. A threebit counter is encoded as follows.

|   bit 2 |   bit 1 |   bit 0 |   Value |
|---------|---------|---------|---------|
|    0000 |    0000 |    0000 |       0 |
|    0001 |    0000 |    0000 |       1 |
|    0001 |    0001 |    0000 |       2 |
|    0001 |    0001 |    0001 |       3 |

## Lock API

This API locks the device.

| Name               | Lock API             | -                |
|--------------------|----------------------|------------------|
| PP Define          | FUNC_ROM_LOCK        | -                |
| Prototype          | bool adi_rom_lock(); | -                |
| Return Value       | bool                 | true for success |
| Stack Requirements | valid stack          | -                |

bool res = adi\_rom\_lock();

Calling this function locks the device, making it a secure. Once locked, the OTPC\_SECU\_STATE register indicates that the part is locked, and access is limited. For more information, refer to the security documentation regarding a locked device.

NOTE: Locked Status. The OTPC\_SECU\_STATE register is updated only after the part is rebooted. After calling the lock function, the register still indicates that the part is open.

## ADSP-2159x\_SC592\_SC594 OTPC Register Descriptions

OTP Memory Controller (OTPC) contains the following registers.

Table 11-4: ADSP-2159x\_SC592\_SC594 OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## OTP Security State Register

The OTPC\_SECU\_STATE register indicates the secure state. The register is updated only after the part is rebooted and locked.

Figure 11-1: OTPC\_SECU\_STATE Register Diagram

<!-- image -->

Table 11-5: OTPC\_SECU\_STATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 1:0                | PARTLOCK   | Part Locked. The OTPC_SECU_STATE.PARTLOCK field indicates a locked part. |
| (R/NW)             |            | 0 OPEN part                                                              |
|                    |            | 1 Locked part                                                            |
|                    |            | 2 Reserved                                                               |
|                    |            | 3 Reserved                                                               |

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