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

## ADSP-2184x OTPC Register List

The One-Time-Programmable Memory controller (OTPC) supports programming the OTP memory. A set of registers governs OTPC operations. For more information on OTPC functionality, see the OTPC register descriptions.

Table 11-1: ADSP-2184x OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## ADSP-2184x OTPC Interrupt List

Table 11-2: ADSP-2184x OTPC Interrupt List

|   Interrupt ID | Name      | Description          | Sensitivity   | DMA Channel   |
|----------------|-----------|----------------------|---------------|---------------|
|            216 | OTPC0_ERR | OTPC0 Dual-bit Error | Level         |               |

## Error Correction

The OTP memory features a Hamming error correction implementation. Signal bit errors are automatically corrected, and dual-bit errors are detected. Refer to OTPC Interrupt Signals.

ECC is always enabled. ECC applies to each 16-bit segment. Because of this functionality, each 16-bit location can only be written to once. Writing to a 16-bit location a second time results in unexpected behavior.

## OTP Layout

This section details the memory layout of the OTP memory.

Table 11-3: ADSP-2184x/SC84x OTP Layout

| Name             | 32-bit Aligned Byte Address   |   Size (bits) | Description              |
|------------------|-------------------------------|---------------|--------------------------|
| xspi0_phyconfig0 | 0x3C                          |            32 | xSPI0 PHY Configu Word 0 |
| xspi0_phyconfig1 | 0x40                          |            32 | xSPI0 PHY Configu Word 1 |
| xspi0_phyconfig2 | 0x44                          |            32 | xSPI0 PHY Configu Word 2 |
| xspi0_phyconfig3 | 0x48                          |            32 | xSPI0 PHY Configu Word 3 |
| xspi1_phyconfig0 | 0x4C                          |            32 | xSPI1 PHY Configu Word 0 |
| xspi1_phyconfig1 | 0x50                          |            32 | xSPI1 PHY Configu Word 1 |
| xspi1_phyconfig2 | 0x54                          |            32 | xSPI1 PHY Configu Word 2 |
| xspi1_phyconfig3 | 0x58                          |            32 | xSPI1 PHY Configu Word 3 |
| huk              | 0x5C                          |           256 | Hardware Unique Key      |
| rkek             | 0x7C                          |           128 | Root Key encryption key  |
| dek              | 0x8C                          |           128 | Local encryption key     |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                     | 32-bit Aligned Byte Address   |   Size (bits) | Description                     |
|--------------------------|-------------------------------|---------------|---------------------------------|
| oem_public_key           | 0x9C                          |           512 | OEMPublic Key                   |
| pvt_128key0              | 0xDC                          |           128 | Customer Privatekey 0 128bits   |
| pvt_128key1              | 0xEC                          |           128 | Customer Privatekey 1 128bits   |
| pvt_128key2              | 0xFC                          |           128 | Customer Privatekey 2 128bits   |
| pvt_128key3              | 0x10C                         |           128 | Customer Privatekey 3 128bits   |
| ek                       | 0x11C                         |           256 | Endorsement Key                 |
| secure_emu_key0          | 0x13C                         |           256 | Secure Emulation Key0           |
| secure_emu_key1          | 0x15C                         |           256 | Secure Emulation Key1           |
| reserved                 | 0x180                         |           544 | reserved                        |
| xspi_sram_custom_header  | 0x1CC                         |            32 | xSPI SRAM Custom Header         |
| emu_key0_disable         | 0x1D0                         |            16 | Secure emulation key0 disable   |
| emu_key1_disable         | 0x1D0                         |            16 | Secure emulation key1 disable   |
| public_key0              | 0x1D4                         |           512 | Customer Public Key0            |
| public_key1              | 0x214                         |           512 | Customer Public Key1            |
| watchdog_count           | 0x254                         |            32 | Watchdog Timeout Count          |
| reserved                 | 0x258                         |            32 | reserved                        |
| cgu0_ctl_WEN             | 0x25C                         |             1 | CGU0 Config ctl_WEN             |
| cgu0_div_WEN             | 0x25C                         |             1 | CGU0 Config div_WEN             |
| cgu0_reserved0           | 0x25C                         |             1 | CGU0 Config reserved0           |
| cgu0_div_DSEL            | 0x25C                         |             5 | CGU0 Config div_DSEL            |
| cgu0_div_CSEL            | 0x25C                         |             5 | CGU0 Config div_CSEL            |
| cgu0_div_S0SEL           | 0x25C                         |             3 | CGU0 Config div_S0SEL           |
| cgu0_div_SYSSEL          | 0x25C                         |             5 | CGU0 Config div_SYSSEL          |
| cgu0_div_S1SEL           | 0x25C                         |             3 | CGU0 Config div_S1SEL           |
| cgu0_div_OSEL            | 0x25C                         |             7 | CGU0 Config div_OSEL            |
| cgu0_ctl_DF              | 0x25C                         |             1 | CGU0 Config ctl_DF              |
| cgu0_ctl_MSEL            | 0x260                         |             7 | CGU0 Config ctl_MSEL            |
| cgu0_ctl_S0SELEXEN       | 0x260                         |             1 | CGU0 Config ctl_S0SELEXEN       |
| cgu0_ctl_S1SELEXEN       | 0x260                         |             1 | CGU0 Config ctl_S1SELEXEN       |
| cgu0_auto_disable        | 0x260                         |             1 | CGU0 Disable Auto Alignment     |
| cgu0_clkoutsel_CLKOUTSEL | 0x260                         |             5 | CGU0 Config clkoutsel_CLKOUTSEL |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                     | 32-bit Aligned Byte Address   |   Size (bits) | Description                     |
|--------------------------|-------------------------------|---------------|---------------------------------|
| cgu0_clkoutsel_WEN       | 0x260                         |             1 | CGU0 Config clkoutsel_WEN       |
| cgu0_divex_S0SELEX       | 0x260                         |             8 | CGU0 Config divex_S0SELEX       |
| cgu0_divex_S1SELEX       | 0x260                         |             8 | CGU0 Config divex_S1SELEX       |
| cgu0_reserved1           | 0x264                         |            32 | CGU0 Config reserved1           |
| cgu1_ctl_WEN             | 0x268                         |             1 | CGU1 Config ctl_WEN             |
| cgu1_div_WEN             | 0x268                         |             1 | CGU1 Config div_WEN             |
| cgu1_reserved0           | 0x268                         |             1 | CGU1 Config reserved0           |
| cgu1_div_DSEL            | 0x268                         |             5 | CGU1 Config div_DSEL            |
| cgu1_div_CSEL            | 0x268                         |             5 | CGU1 Config div_CSEL            |
| cgu1_div_S0SEL           | 0x268                         |             3 | CGU1 Config div_S0SEL           |
| cgu1_div_SYSSEL          | 0x268                         |             5 | CGU1 Config div_SYSSEL          |
| cgu1_div_S1SEL           | 0x268                         |             3 | CGU1 Config div_S1SEL           |
| cgu1_div_OSEL            | 0x268                         |             7 | CGU1 Config div_OSEL            |
| cgu1_ctl_DF              | 0x268                         |             1 | CGU1 Config ctl_DF              |
| cgu1_ctl_MSEL            | 0x26C                         |             7 | CGU1 Config ctl_MSEL            |
| cgu1_ctl_S0SELEXEN       | 0x26C                         |             1 | CGU1 Config ctl_S0SELEXEN       |
| cgu1_ctl_S1SELEXEN       | 0x26C                         |             1 | CGU1 Config ctl_S1SELEXEN       |
| cgu1_auto_disable        | 0x26C                         |             1 | CGU1 Disable Auto Alignment     |
| cgu1_clkoutsel_CLKOUTSEL | 0x26C                         |             5 | CGU1 Config clkoutsel_CLKOUTSEL |
| cgu1_clkoutsel_WEN       | 0x26C                         |             1 | CGU1 Config clkoutsel_WEN       |
| cgu1_divex_S0SELEX       | 0x26C                         |             8 | CGU1 Config divex_S0SELEX       |
| cgu1_divex_S1SELEX       | 0x26C                         |             8 | CGU1 Config divex_S1SELEX       |
| cgu1_reserved1           | 0x270                         |            32 | CGU1 Config reserved1           |
| cdu_cfg_wen              | 0x274                         |             1 | CDU Config Enable               |
| cdu_cfg_clkin            | 0x274                         |             1 | CDU Clock Input                 |
| cdu_cfg0_clk_en          | 0x274                         |             1 | CDU CFG0 Clock Enable           |
| cdu_cfg0_clk_sel         | 0x274                         |             2 | CDU CFG0 Clock SEL              |
| cdu_cfg1_clk_en          | 0x274                         |             1 | CDU CFG1 Clock Enable           |
| cdu_cfg1_clk_sel         | 0x274                         |             2 | CDU CFG1 Clock SEL              |
| cdu_cfg2_clk_en          | 0x274                         |             1 | CDU CFG2 Clock Enable           |
| cdu_cfg2_clk_sel         | 0x274                         |             2 | CDU CFG2 Clock SEL              |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                   | 32-bit Aligned Byte Address   |   Size (bits) | Description                            |
|------------------------|-------------------------------|---------------|----------------------------------------|
| cdu_cfg3_clk_en        | 0x274                         |             1 | CDU CFG3 Clock Enable                  |
| cdu_cfg3_clk_sel       | 0x274                         |             2 | CDU CFG3 Clock SEL                     |
| cdu_cfg4_clk_en        | 0x274                         |             1 | CDU CFG4 Clock Enable                  |
| cdu_cfg4_clk_sel       | 0x274                         |             2 | CDU CFG4 Clock SEL                     |
| cdu_cfg5_clk_en        | 0x274                         |             1 | CDU CFG5 Clock Enable                  |
| cdu_cfg5_clk_sel       | 0x274                         |             2 | CDU CFG5 Clock SEL                     |
| cdu_cfg6_clk_en        | 0x274                         |             1 | CDU CFG6 Clock Enable                  |
| cdu_cfg6_clk_sel       | 0x274                         |             2 | CDU CFG6 Clock SEL                     |
| cdu_cfg7_clk_en        | 0x274                         |             1 | CDU CFG7 Clock Enable                  |
| cdu_cfg7_clk_sel       | 0x274                         |             2 | CDU CFG7 Clock SEL                     |
| cdu_cfg8_clk_en        | 0x274                         |             1 | CDU CFG8 Clock Enable                  |
| cdu_cfg8_clk_sel       | 0x274                         |             2 | CDU CFG8 Clock SEL                     |
| cdu_cfg9_clk_en        | 0x274                         |             1 | CDU CFG9 Clock Enable                  |
| cdu_cfg9_clk_sel       | 0x274                         |             2 | CDU CFG9 Clock SEL                     |
| cdu_cfg10_clk_en       | 0x278                         |             1 | CDU CFG10 Clock Enable                 |
| cdu_cfg10_clk_sel      | 0x278                         |             2 | CDU CFG10 Clock SEL                    |
| cdu_cfg11_clk_en       | 0x278                         |             1 | CDU CFG11 Clock Enable                 |
| cdu_cfg11_clk_sel      | 0x278                         |             2 | CDU CFG11 Clock SEL                    |
| cdu_cfg12_clk_en       | 0x278                         |             1 | CDU CFG12 Clock Enable                 |
| cdu_cfg12_clk_sel      | 0x278                         |             2 | CDU CFG12 Clock SEL                    |
| cdu_cfg13_clk_en       | 0x278                         |             1 | CDU CFG13 Clock Enable                 |
| cdu_cfg13_clk_sel      | 0x278                         |             2 | CDU CFG13 Clock SEL                    |
| cdu_cfg14_clk_en       | 0x278                         |             1 | CDU CFG14 Clock Enable                 |
| cdu_cfg14_clk_sel      | 0x278                         |             2 | CDU CFG14 Clock SEL                    |
| cdu_reserved0          | 0x278                         |            17 | CDU Reserved 0                         |
| flashStartAddress      | 0x27c                         |            32 | Flash Start Address Override           |
| altflashStartAddress   | 0x280                         |            32 | Alterate Flash Start Address Override  |
| spiControllerBootCmd   | 0x284                         |            32 | SPI Controller Boot Command Override   |
| xspiControllerBootCmd  | 0x288                         |            32 | XSPI Controller Boot Command Override  |
| xspiControllerBootCmd1 | 0x28c                         |            32 | XSPI Boot command1 Override            |
| xspi1ControllerBootCmd | 0x290                         |            32 | XSPI1 Controller Boot Command Override |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                    | 32-bit Aligned Byte Address   |   Size (bits) | Description                            |
|-------------------------|-------------------------------|---------------|----------------------------------------|
| xspi1ControllerBootCmd1 | 0x294                         |            32 | XSPI1 Boot command1 Override           |
| emmcControllerBootCmd0  | 0x298                         |            32 | eMMC Boot Command 0 Override           |
| emmcControllerBootCmd1  | 0x29c                         |            32 | eMMC Boot Command 1 Override           |
| emmcSDBootCmd0          | 0x2A0                         |            32 | eMMC SD Boot Command 0 Override        |
| emmcSDBootCmd1          | 0x2A4                         |            32 | eMMC SD Boot Command 1 Override        |
| spiTargetBootCmd        | 0x2A8                         |            32 | SPI Target Boot Command Override       |
| lpBootCmd               | 0x2AC                         |            32 | LinkPort Boot Command Override         |
| uartBootCmd             | 0x2B0                         |            32 | UART Boot Command Override             |
| altTargetBootCmd        | 0x2B4                         |            32 | Alternate Target Boot Command Override |
| bcfg_pubkey0Inv         | 0x2B8                         |             1 | Boot Config pubkey0Inv                 |
| bcfg_reserved2          | 0x2B8                         |            15 | Boot Config reserved2                  |
| bcfg_pubkey1Inv         | 0x2B8                         |             1 | Boot Config pubkey1Inv                 |
| bcfg_reserved3          | 0x2B8                         |            15 | Boot Config reserved3                  |
| bcfg_privkey0Inv        | 0x2BC                         |             1 | Boot Config privkey0Inv                |
| bcfg_reserved4          | 0x2BC                         |            15 | Boot Config reserved4                  |
| bcfg_privkey1Inv        | 0x2BC                         |             1 | Boot Config privkey1Inv                |
| bcfg_reserved5          | 0x2BC                         |            15 | Boot Config reserved5                  |
| bcfg_privkey2Inv        | 0x2C0                         |             1 | Boot Config privkey2Inv                |
| bcfg_reserved6          | 0x2C0                         |            15 | Boot Config reserved6                  |
| bcfg_privkey3Inv        | 0x2C0                         |             1 | Boot Config privkey3Inv                |
| bcfg_reserved7          | 0x2C0                         |            15 | Boot Config reserved7                  |
| bcfg_dmcEn              | 0x2C4                         |             1 | Boot Config dmcEn                      |
| bcfg_reserved8          | 0x2C4                         |            15 | Boot Config reserved8                  |
| bcfg_dmcInv             | 0x2C4                         |             1 | Boot Config dmcInv                     |
| bcfg_reserved9          | 0x2C4                         |            15 | Boot Config reserved9                  |
| bcfg_xspisramEn         | 0x2C8                         |             1 | Boot Config xspiramEn                  |
| bcfg_reserved8          | 0x2C8                         |            15 | Boot Config reserved8                  |
| bcfg_xspisramInv        | 0x2C8                         |             1 | Boot Config xspiramInv                 |
| bcfg_reserved9          | 0x2C8                         |            15 | Boot Config reserved9                  |
| bcfg_wdtDis             | 0x2CC                         |             1 | Boot Config wdtDis                     |
| bcfg_reserved10         | 0x2CC                         |            15 | Boot Config reserved10                 |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                   | 32-bit Aligned Byte Address   |   Size (bits) | Description                     |
|------------------------|-------------------------------|---------------|---------------------------------|
| bcfg_wdtResetEn        | 0x2CC                         |             1 | Boot Config wdtResetEn          |
| bcfg_reserved11        | 0x2CC                         |            15 | Boot Config reserved11          |
| bcfg_altMstrBootEn     | 0x2D0                         |             1 | Boot Config altMstrBootEn       |
| bcfg_reserved12        | 0x2D0                         |            15 | Boot Config reserved12          |
| bcfg_altMstrBootDis    | 0x2D0                         |             1 | Boot Config altMstrBootDis      |
| bcfg_reserved13        | 0x2D0                         |            15 | Boot Config reserved13          |
| bcfg_altTargetBootEn   | 0x2D4                         |             1 | Boot Config altTargetBootEn     |
| bcfg_reserved14        | 0x2D4                         |            15 | Boot Config reserved14          |
| bcfg_altTargetBootDis  | 0x2D4                         |             1 | Boot Config altTargetBootDis    |
| bcfg_reserved15        | 0x2D4                         |            15 | Boot Config reserved15          |
| bcfg_benchmark_En      | 0x2D8                         |             1 | Boot Config Benchmarking enable |
| bcfg_reserved16        | 0x2D8                         |            15 | Boot Config reserved16          |
| bcfg_fastcguDis        | 0x2D8                         |             1 | Boot Config fastcguDis          |
| bcfg_reserved17        | 0x2D8                         |            15 | Boot Config reserved17          |
| bcfg_dmcDecDis         | 0x2DC                         |             1 | Boot Config dmcDecDis           |
| bcfg_reserved18        | 0x2DC                         |            15 | Boot Config reserved17          |
| bcfg_secBootHSMDis     | 0x2DC                         |            16 | Boot Config secBootHSMDis       |
| antiroll_nv_cntr       | 0x2E4                         |           512 | AntiRollback NV Counter         |
| gp1                    | 0x324                         |           320 | General Purpose 1               |
| reserved               | 0x34c                         |            24 | reserved                        |
| bootModeDisable        | 0x34c                         |             8 | Boot Mode Disable Bits          |
| xspi_sram_phyconfig0   | 0x350                         |            32 | xSPI SRAM Phyconfig0            |
| xspi_sram_phyconfig1   | 0x354                         |            32 | xSPI SRAM Phyconfig1            |
| xspi_sram_phyconfig2   | 0x358                         |            32 | xSPI SRAM Phyconfig2            |
| xspi_sram_phyconfig3   | 0x35c                         |            32 | xSPI SRAM Phyconfig3            |
| xspi_sram_remapConfig  | 0x360                         |             2 | XSPI sram remapConfig           |
| xspi_sram_devicetype   | 0x360                         |             1 | XSPI sram devicetype            |
| xspi_sram_Addressbytes | 0x360                         |             2 | XSPI sram Addressbytes          |
| xspi_sram_bytesperaddr | 0x360                         |             1 | XSPI sram bytesperaddr          |
| xspi_sram_byteswap     | 0x360                         |             1 | XSPI sram byteswap              |
| xspi_sram_devnum       | 0x360                         |             1 | XSPI sram devnum                |

Table 11-3: ADSP-2184x/SC84x OTP Layout (Continued)

| Name                         | 32-bit Aligned Byte Address   |   Size (bits) | Description                  |
|------------------------------|-------------------------------|---------------|------------------------------|
| xspi_sram_csda_mindelay      | 0x360                         |             8 | XSPI sram csda_mindelay      |
| xspi_sram_clockDiv           | 0x360                         |             8 | XSPI sram clockDiv           |
| xspi_sram_readopcode1        | 0x360                         |             8 | XSPI sram readopcode1        |
| xspi_sram_readopcode2        | 0x364                         |             8 | XSPI sram readopcode2        |
| xspi_sram_writeopcode1       | 0x364                         |             8 | XSPI sram writeopcode1       |
| xspi_sram_writeopcode2       | 0x364                         |             8 | XSPI sram writeopcode2       |
| xspi_sram_readlatency        | 0x364                         |             8 | XSPI sram readlatency        |
| xspi_sram_writelatency       | 0x368                         |             8 | XSPI sram writelatency       |
| xspi_sram_busmode            | 0x368                         |             8 | XSPI sram busmode            |
| xspi_sram_targetsel          | 0x368                         |             2 | XSPI sram targetsel          |
| xspi_sram_wen_enable         | 0x368                         |             1 | XSPI sram wen_enable         |
| xspi_sram_wen_opcodeexenable | 0x368                         |             1 | XSPI sram wen_opcodeexenable |
| xspi_sram_wen_opcodeex       | 0x368                         |             8 | XSPI sram wen_opcodeex       |
| xspi_sram_reserved0          | 0x368                         |             4 | reserved0                    |
| xspi_sram_dev_active_max     | 0x36c                         |            32 | XSPI sram dev_active_max     |
| stageID                      | 0x370                         |            48 | StageID                      |
| reserved                     | 0x374                         |            16 | reserved                     |

## OTPC Event Control

The following sections provide information on OTP events and error management.

## OTPC Interrupt Signals

When making 32-bit accesses to OTP memory, a double-bit error in any 16-bit segment triggers the OTPC\_INT interrupt. The OTPC also has the OTPC dual bit error (OTPC0\_ERR) with the SEC ID of 5.

## OTPC Status and Error Signals

The OTP controller does not produce error signals.

## OTP API Overview

The ROM/OTP SSLDD library provides a set of functions to facilitate OTP field access. The OTP memory is broken up into a set of specialized fields that are described in this section. The API removes the requirement of understanding the details of the layout or OTP access procedures.

All OTP accesses are made through the provided API.

## OTP Programming

The OTP programming API provides a simple access, abstracting particulars of the OTP controller.

Any fields that contain zero or null pointers are skipped.

All addresses are assumed to be byte addresses unless otherwise noted.

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

## ADSP-2184x OTPC Register Descriptions

OTP Memory Controller (OTPC) contains the following registers.

Table 11-4: ADSP-2184x OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## OTP Security State Register

The OTPC\_SECU\_STATE register indicates the secure state. The register is updated only after the part is rebooted and locked.

Figure 11-1: OTPC\_SECU\_STATE Register Diagram

![Image](14_One-Time_Programmable_Memory_Controller_(OTPC)_artifacts/image_000000_f28db527100f32541b9eb3cc34223c00d82a08986e8c43a33510368c29224301.png)

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

![Image](14_One-Time_Programmable_Memory_Controller_(OTPC)_artifacts/image_000001_999f7aead21734ead8a7307accd301f63d81450b5fe53c8f0df41a8ea9d8e834.png)

Table 11-6: OTPC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:13 (R/NW)       | ADDRERR    | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. |
|                    |            | 0                                                                                                                                                                    | No error - proper OTP address                                                                                                                                        |
|                    |            | 1                                                                                                                                                                    | OTP address out of range                                                                                                                                             |
|                    |            | 2                                                                                                                                                                    | 8-bit OTP address                                                                                                                                                    |
|                    |            | 3                                                                                                                                                                    | Protected OTP address                                                                                                                                                |