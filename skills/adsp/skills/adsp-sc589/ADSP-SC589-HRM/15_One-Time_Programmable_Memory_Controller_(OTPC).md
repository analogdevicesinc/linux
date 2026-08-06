## 12   One-Time Programmable Memory Controller (OTPC)

This chapter describes the operation of the OTP controller. The OTP module is a complete system integrating an OTP memory core with a programming controller, charge pump, and voltage regulator. A built-in Hamming Code Error Correction (ECC), and a fully implemented double-redundant program or read scheme protect the OTP data.

OTP memory access is through the OTP API Overview provided by the ROM.

CAUTION: OTP memory does not support burst transfers, which are required to support cache line fills. As such, OTP memory should not be made cacheable. If it is, the OTP controller returns an error when a read access is attempted.

## OTPC Features

The OTP memory and controller have the following features:

- Built-in redundant read mode
- Built-in integrated power supply
- Built-in Hamming Code Error Correction (ECC)
- Full word serial (single bit at a time) programming with internal VPP

## Error Correction

The OTP memory features a Hamming error correction implementation. Signal bit errors are automatically corrected, and dual-bit errors are detected. Refer to OTPC Interrupt Signals.

ECC is always enabled. ECC applies to each 16-bit segment. Because of this functionality, each 16-bit location can only be written to once. Writing to a 16-bit location a second time results in unexpected behavior.

## OTP Layout

This section details the memory layout of the OTP memory.

## OTP Layout

Table 12-1: ADSP-SCxx OTP Layout

| Name                                 | Byte Address                         | Size (bits)                          | Description                            |
|--------------------------------------|--------------------------------------|--------------------------------------|----------------------------------------|
| Customer OTP Area - 896 bytes        | Customer OTP Area - 896 bytes        | Customer OTP Area - 896 bytes        | Customer OTP Area - 896 bytes          |
| huk                                  | 0x0 + 0 - 0x1c                       | 256                                  | Hardware unique key                    |
| pvt_128key0                          | 0x168 + 0 - 0x174                    | 128                                  | Customer private key 0 128bits         |
| pvt_128key1                          | 0x178 + 0 - 0x184                    | 128                                  | Customer private key 1 128bits         |
| pvt_128key2                          | 0x188 + 0 - 0x194                    | 128                                  | Customer private key 2 128bits         |
| pvt_128key3                          | 0x198 + 0 - 0x1a4                    | 128                                  | Customer private key 3 128bits         |
| ek                                   | 0x1a8 + 0 - 0x1c4                    | 256                                  | Endorsement key                        |
| secure_emu_key                       | 0x1c8 + 0 - 0x1d4                    | 128                                  | Secure emulation key                   |
| Reserved                             | 0x1d8 + 0 - 0x1fc                    | 320                                  | Reserved                               |
| public_key0                          | 0x200 + 0 - 0x23c                    | 512                                  | Customer public key 0                  |
| public_key1                          | 0x240 + 0 - 0x27c                    | 512                                  | Customer public key 1                  |
| boot_info                            | 0x280 + 0 - 0x2bc                    | 512                                  | Customer programmable boot information |
| antiroll_nv_cntr                     | 0x2c0 + 0 - 0x2fc                    | 512                                  | AntiRollback NV counter                |
| gp1                                  | 0x300 + 0 - 0x33c                    | 512                                  | General purpose 1                      |
| Reserved                             | 0x340 + 0 - 0x340                    | 24                                   | Reserved                               |
| bootModeDisable                      | 0x343 + 24 - 0x343                   | 8                                    | Boot mode disable bits                 |
| preboot_ddr_cfg                      | 0x344 + 0 - 0x370                    | 384                                  | User preboot DDR configuration         |
| stageID                              | 0x374 + 0 - 0x376                    | 48                                   | Stage ID                               |
| Reserved                             | 0x37a + 16 - 0x37a                   | 16                                   | Reserved                               |
| Reserved                             | 0x37c + 0 - 0x37c                    | 32                                   | Reserved                               |
| Overlaid Fields in Customer OTP Area | Overlaid Fields in Customer OTP Area | Overlaid Fields in Customer OTP Area | Overlaid Fields in Customer OTP Area   |
| otpTiming                            | 0x2be + 16 - 0x2be                   | 16                                   | OTP read timing override               |
| Overlaid Fields in ADI BOOT          | Overlaid Fields in ADI BOOT          | Overlaid Fields in ADI BOOT          | Overlaid Fields in ADI BOOT            |
| Customer BOOT - 116 Bytes            | Customer BOOT - 116 Bytes            | Customer BOOT - 116 Bytes            | Customer BOOT - 116 Bytes              |
| lock                                 | 0x48c + 0 - 0x48c                    | 1                                    | Lockbit                                |
| Reserved                             | 0x48c + 1 - 0x4fc                    | 927                                  | Reserved                               |

## OTPC Event Control

The following sections provide information on OTP events and error management.

## OTPC Interrupt Signals

When making 32-bit accesses to OTP memory, a double-bit error in any 16-bit segment triggers the OTPC\_INT interrupt. The OTPC also has the OTPC dual bit error (OTPC0\_ERR) with the SEC ID of 5 and the GIC ID of 37. See the System Event Controller (SEC) and Generic Interrupt Controller (GIC) chapter for more information.

## OTPC Status and Error Signals

The OTP controller does not produce error signals.

## OTP API Overview

The ROM provides a set of functions to facilitate OTP field access. The OTP memory is broken up into a set of specialized fields that are described in this section. The API removes the requirement of understanding the details of the layout or OTP access procedures.

All OTP accesses are made through the provided API.

## OTP Programming

The OTP programming API provides a simple access, abstracting particulars of the OTP controller.

Any fields that contain zero or null pointers are skipped.

All addresses are assumed to be byte addresses unless otherwise noted.

A list of APIs follows:

| bool adi_rom_otp_pgm(otp_data* data);   | OTP Program   |
|-----------------------------------------|---------------|
| bool adi_rom_lock();                    | Lock API      |

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

The following type of struct is available for programming. Refer to the ROM header file for the exact definition

```
typedef struct { uint32_t (*huk)[8]; uint32_t (*pvt_128key1)[4]; uint32_t (*pvt_128key2)[4]; uint32_t (*pvt_128key3)[4]; uint32_t (*pvt_128key4)[4]; uint32_t (*pvt_192key1)[6]; uint32_t (*pvt_192key2)[6]; uint32_t (*public_key0)[16]; uint32_t (*public_key1)[16]; uint32_t (*ek)[8]; uint32_t (*secure_emu_key)[4]; uint8_t  bootModeDisable; uint32_t (*boot_info)[16]; uint32_t (*gp0)[16]; uint32_t antiroll_nv_cntr; uint8_t  stageID; uint32_t (*preboot_ddr_cfg)[11]; } otp_data;
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

The data specified by the OTPCMD enum parameter is fetched from OTP memory and placed in the location specified by data. The OTPCMD enum contains entries for each field defined in OTP memory, for the most current list please refer to the OTP header file.

An example of the enum style follows:

| /* Field Name Description typedef enum { /* add msi bits */ otpcmd_reserved0 = 0, otpcmd_pvt_128key1, /* Private 128-bit Key 1 otpcmd_pvt_128key2, /* Private 128-bit Key 2 otpcmd_pvt_128key3, /* Private 128-bit Key 3 otpcmd_pvt_128key4, /* Private 128-bit Key 4 otpcmd_pvt_192key1, /* Private 192-bit Key 1 otpcmd_pvt_192key2, /* Private 192-bit Key 2 otpcmd_huk, /* Hardware Unique Key otpcmd_ek, /* Endorsement Key (EK) otpcmd_secure_emu_key, /* Secure Emulation Key otpcmd_public_key1, /* Customer Public Key 1 otpcmd_public_key2, /* Customer Public Key 2 otpcmd_antiroll_nv_cntr, /* Anti-Rollback NV Counter otpcmd_nonvolatile_cntr, /* NV Counter otpcmd_bootModeDisable, /* Boot Mode disable [8] */ otpcmd_stageID, /* Stage [8] */ otpcmd_gp0, /* General Purpose otpcmd_boot_info, /* Customer Programmable otpcmd_preboot_ddr_cfg, /* User Preboot DDR otpcmd_reserved1 /* invalid */ } OTPCMD;   | No of info configuration   |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------|

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

<!-- formula-not-decoded -->

Calling this function locks the device, making it a secure. Once locked, the OTPC\_SECU\_STATE register indicates that the part is locked, and access is limited. For more information, refer to the security documentation regarding a locked device.

NOTE: Locked Status. The OTPC\_SECU\_STATE register is updated only after the part is rebooted. After calling the lock function, the register still indicates that the part is open.

## ADSP-SC58x OTPC Interrupt List

## Table 12-2: ADSP-SC58x OTPC Interrupt List

|   Interrupt ID | Name      | Description          | Sensitivity   | DMA Channel   |
|----------------|-----------|----------------------|---------------|---------------|
|              5 | OTPC0_ERR | OTPC0 Dual-bit error | Level         |               |

## ADSP-SC58x OTPC Register Descriptions

OTP Memory Controller (OTPC) contains the following registers.

Table 12-3: ADSP-SC58x OTPC Register List

| Name            | Description                 |
|-----------------|-----------------------------|
| OTPC_SECU_STATE | OTP Security State Register |
| OTPC_STAT       | OTP Status Register         |

## OTP Security State Register

The OTPC\_SECU\_STATE register provides lock status information.

Figure 12-1: OTPC\_SECU\_STATE Register Diagram

![Image](15_One-Time_Programmable_Memory_Controller_(OTPC)_artifacts/image_000000_8235b0c4e7cbb92790c3a0a33dd4f14fd4db83ac407e21768aa597a50ae2e1ed.png)

Table 12-4: OTPC\_SECU\_STATE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                   |
|--------------------|------------|-----------------------------------------------------------|
| 1:0                | PARTLOCK   | Part Locked.                                              |
| (R/NW)             |            | The OTPC_SECU_STATE.PARTLOCK bit indicates a locked part. |
|                    |            | 0 OPEN part                                               |
|                    |            | 1 Locked part                                             |
|                    |            | 2 Unlocked part                                           |

## OTP Status Register

The OTPC\_STAT register bits indicate errors and flag status and control the protection bits.

Figure 12-2: OTPC\_STAT Register Diagram

![Image](15_One-Time_Programmable_Memory_Controller_(OTPC)_artifacts/image_000001_647c6d1b3ce7f3d5cecf5df6dbd7ed0a55faeedbbd6ab376385c8b621aa2e7ee.png)

Table 12-5: OTPC\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:13 (R/NW)       | ADDRERR    | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. | OTP Address Error. The OTPC_STAT.ADDRERR bit field indicates errors which occur when the OTP programming address is out of range or tries to access protected space. |
|                    |            | 0                                                                                                                                                                    | No error - proper OTP address                                                                                                                                        |
|                    |            | 1                                                                                                                                                                    | OTP address out of range                                                                                                                                             |
|                    |            | 2                                                                                                                                                                    | 8-bit OTP address                                                                                                                                                    |
|                    |            | 3                                                                                                                                                                    | Protected OTP address                                                                                                                                                |