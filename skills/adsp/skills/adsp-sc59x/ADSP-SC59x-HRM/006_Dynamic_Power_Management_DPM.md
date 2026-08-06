# Dynamic Power Management (DPM)

<!-- source: 006_Dynamic_Power_Management_DPM.pdf | original pages 198–210 -->

## 4   Dynamic Power Management (DPM)

The dynamic power management (DPM) unit of the processor controls transitions between different power-saving modes.

DPM provides facility for shutting down clocks for various peripherals. Shutting down peripheral's clocks is considered a power saving feature. To support peripheral clock gating functionality, DPM\_PER\_DIS0 and DPM\_PER\_DIS1 are available in the DPM module. The bits in these registers need to be programmed to shut off the clocks to peripherals that are not used in an application to save clock switching power.

## DPM Features

The DPM allows programs to control the power mode of the processor as follows.

- Permits operation of multiple, external wake-up sources

## DPM Functional Description

The DPM can be programmed to transition between power modes.

## ADSP-SC59x DPM Register List

A set of registers govern DPM operations. For more information on DPM functionality, see the DPM register descriptions.

Table 4-1: ADSP-SC59x DPM Register List

| Name         | Description                    |
|--------------|--------------------------------|
| DPM_CTL      | Control Register               |
| DPM_PER_DIS0 | Peripherals Disable Register 0 |
| DPM_PER_DIS1 | Peripherals Disable Register 1 |
| DPM_REVID    | Revision ID                    |
| DPM_STAT     | Status Register                |

## DPM Definitions

To make the best use of the DPM, it is useful to understand the following terms.

## CGU

Acronym for the clock generation unit (CGU), which is comprised of the PLL and PCU

## DPM

Acronym for the dynamic power management (DPM) controller.

## Full-on mode

The normal operating mode in which all clock domains are derived from the PLL.

## PCU

Acronym for the PLL control unit (PCU).

## PLL

Acronym for the phase-locked loop (PLL).

## RCU

Acronym for the reset control unit (RCU).

## DPM Operating Modes

The DPM includes several operating modes. The modes are:

- Reset
- Full-on

## Reset State

Reset is the initial state of the processor and is the result of a hardware or software triggered event. The DPM itself does not trigger entering reset. The external SYS\_HWRST pin or the RCU triggers entering reset. The DPM responds to reset by transitioning to its default state.

From Reset, the DPM always transitions to PLL Bypassed state.

## Full-on Mode

Full-on mode is the default state of the DPM after Reset.

In full-on mode, the processor can reach its maximum clock rate and power dissipation can be at its highest.

## DPM Event Control

The DPM event is triggered when an enabled wake-up is asserted. The DPM generates bus errors when a misaligned access to a register occurs. It also generates errors when an attempt is made to access unused DPM address space or a write-protected register.

## DPM Events

The DPM event interrupt is triggered when any bit in the DPM\_STAT register is set, indicating that an enabled wake-up was asserted. The DPM event interrupt stays active until the user clears any bits that are set in the DPM\_STAT register.

## DPM Errors

The DPM generates a bus error when a read or write transaction is attempted to an unused address within the DPM address range. It also generates a bus error when a misaligned access is made to a DPM register. In addition to the bus error, the DPM sets the DPM\_STAT.ADDRERR bit.

If a write to a write-protected DPM register is attempted, the DPM generates a bus error. In addition, the DPM sets the DPM\_STAT.LWERR bit.

## DPM Programming Model

The DPM\_PER\_DIS0 Register Mapping and DPM\_PER\_DIS1Register Mapping tables show the module clocks and the corresponding peripheral. The DPM\_PER\_DIS0 and DPM\_PER\_DIS1 registers are used to shut off the clock to each peripheral if it is not required by the application.

Table 4-2: DPM0\_PER\_DIS0 Register Mapping

| Peripheral Name   | Gated Module Clocks   | Type of sync on PER- DIS bit         | Effect latency (In cycles of module clocks)   |   DPM0_PER_DISn bit |
|-------------------|-----------------------|--------------------------------------|-----------------------------------------------|---------------------|
| FIR0/FFT          | cclk                  | Not Required                         | 1                                             |                   0 |
| IIR0              | cclk                  | Not Required                         | 1                                             |                   1 |
| DAI0              | sclk0 sclk1 sysclk    | Single Flop Single Flop Not Required | 2 2 1                                         |                   5 |
| DAI1              | sclk0 sclk1 sysclk    | Single Flop Single Flop Not Required | 2 2 1                                         |                   6 |

Table 4-2: DPM0\_PER\_DIS0 Register Mapping (Continued)

| Peripheral Name                      | Gated Module Clocks   | Type of sync on PER- DIS bit   |   Effect latency (In cycles of module clocks) |   DPM0_PER_DISn bit |
|--------------------------------------|-----------------------|--------------------------------|-----------------------------------------------|---------------------|
| MLB0                                 | sysclk                | Not Required                   |                                             1 |                   9 |
| EMDMA0                               | sysclk                | Not Required                   |                                             1 |                  13 |
| EMDMA1                               | sysclk                | Not Required                   |                                             1 |                  14 |
| CRYPTO ACCELERA- TOR-0 (EIP-150/PKP) | sysclk                | Not Required                   |                                             1 |                  16 |
| CRYPTO ACCELERA- TOR-1 (EIP-93/SPE)  | sysclk                | Not Required                   |                                             1 |                  17 |
| SMPU-SPIF                            | sysclk                | Not Required                   |                                             1 |                  19 |
| SMPU-L2CTL-CL2-0                     | sysclk                | Not Required                   |                                             1 |                  20 |
| SMPU-L2CTL-DL2-0                     | sysclk                | Not Required                   |                                             1 |                  21 |

Table 4-3: DPM0\_PER\_DIS1 Register Mapping

| Peripheral Name   | Gated Module Clocks   | Type of sync on PER- DIS bit   |   Effect latency (In cycles of module clocks) |   DPM0_PER_DISn bit |
|-------------------|-----------------------|--------------------------------|-----------------------------------------------|---------------------|
| TWI0              | sclk0                 | Single Flop                    |                                             2 |                   0 |
| TWI1              | sclk0                 | Single Flop                    |                                             2 |                   1 |
| TWI2              | sclk0                 | Single Flop                    |                                             2 |                   2 |
| TWI3              | sclk0                 | Single Flop                    |                                             2 |                   3 |
| TWI4              | sclk0                 | Single Flop                    |                                             2 |                   4 |
| TWI5              | sclk0                 | Single Flop                    |                                             2 |                   5 |
| OSPI              | sysclk                | Not Required                   |                                             1 |                  27 |

## ADSP-SC59x DPM Register Descriptions

Dynamic Power Management (DPM) contains the following registers.

Table 4-4: ADSP-SC59x DPM Register List

| Name         | Description                    |
|--------------|--------------------------------|
| DPM_CTL      | Control Register               |
| DPM_PER_DIS0 | Peripherals Disable Register 0 |
| DPM_PER_DIS1 | Peripherals Disable Register 1 |

Table 4-4: ADSP-SC59x DPM Register List (Continued)

| Name      | Description     |
|-----------|-----------------|
| DPM_REVID | Revision ID     |
| DPM_STAT  | Status Register |

## Control Register

The DPM\_CTL register controls sleep modes selections and PLL operations of the DPM. A write protect feature permits locking out changes to this register.

Figure 4-1: DPM\_CTL Register Diagram

<!-- image -->

Table 4-5: DPM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the DPM_CTL.LOCK bit is set, the DPM_CTL register is read only (locked). |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                |
| 31 (R/W)           | LOCK       | 1 Lock                                                                                                                                  |

## Peripherals Disable Register 0

The DPM\_PER\_DIS0 register is used to shut off the clocks to peripherals that are not used in an application in order to save clock switching power.

Figure 4-2: DPM\_PER\_DIS0 Register Diagram

<!-- image -->

Table 4-6: DPM\_PER\_DIS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit.                                                                                               |
| 30:0 (R/W)         | VALUE      | Peripheral Disable. The DPM_PER_DIS0.VALUE bits are used to shut of clocks in individual peripher- als. |
| 30:0 (R/W)         | VALUE      | 0 SH0_FIR0                                                                                              |
| 30:0 (R/W)         | VALUE      | 1 SH0_IIR0                                                                                              |
| 30:0 (R/W)         | VALUE      | 2 SH0_IIR1                                                                                              |
| 30:0 (R/W)         | VALUE      | 3 SH0_IIR2                                                                                              |
| 30:0 (R/W)         | VALUE      | 4 SH0_IIR3                                                                                              |
| 30:0 (R/W)         | VALUE      | 5 DAI0                                                                                                  |
| 30:0 (R/W)         | VALUE      | 6 DAI1                                                                                                  |
| 30:0 (R/W)         | VALUE      | 7 Reserved                                                                                              |
| 30:0 (R/W)         | VALUE      | 8 Reserved                                                                                              |
| 30:0 (R/W)         | VALUE      | 9 MLB0                                                                                                  |
| 30:0 (R/W)         | VALUE      | 10 EMAC0 (10/100/1000)                                                                                  |
| 30:0 (R/W)         | VALUE      | 11 EMAC1 (10/100)                                                                                       |
| 30:0 (R/W)         | VALUE      | 12 MSHC0                                                                                                |
| 30:0 (R/W)         | VALUE      | 13 EMDMA0/CH1                                                                                           |

Table 4-6: DPM\_PER\_DIS0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration            |
|--------------------|------------|---------------------------|------------------------------------|
|                    |            |                        14 | EMDMA0/CH0                         |
|                    |            |                        15 | Reserved                           |
|                    |            |                        16 | SMPU_SPIF (SPI2)                   |
|                    |            |                        17 | CRYPTO ACCELERATOR-1 (EIP-150/PKP) |
|                    |            |                        18 | CRYPTO ACCELERATOR-0 (EIP-93/SPE)  |
|                    |            |                        19 | SMPU-SMOY                          |
|                    |            |                        20 | SMPU-L2CTL-CL2-0                   |
|                    |            |                        21 | SMPU-L2CTL-DL2-0                   |
|                    |            |                        22 | SMPU-L2CTL-CL2-1                   |
|                    |            |                        23 | SMPU-L2CTL-DL2-1                   |
|                    |            |                        24 | SMPU-L2CTL-CL2-2                   |
|                    |            |                        25 | Reserved                           |
|                    |            |                        26 | Reserved                           |
|                    |            |                        27 | SMPU-DMC0                          |
|                    |            |                        28 | Reserved                           |
|                    |            |                        29 | CAN0                               |
|                    |            |                        30 | CAN1                               |

## Peripherals Disable Register 1

The DPM\_PER\_DIS1 register is used to shut off the clocks to peripherals that are not used in an application to save clock switching power.

Figure 4-3: DPM\_PER\_DIS1 Register Diagram

<!-- image -->

Table 4-7: DPM\_PER\_DIS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit.                                                                                               |
| 18:0 (R/W)         | VALUE      | Peripheral Disable. The DPM_PER_DIS1.VALUE bits are used to shut of clocks in individual peripher- als. |
| 18:0 (R/W)         | VALUE      | 0 TWI0                                                                                                  |
| 18:0 (R/W)         | VALUE      | 1 TWI1                                                                                                  |
| 18:0 (R/W)         | VALUE      | 2 TWI2                                                                                                  |
| 18:0 (R/W)         | VALUE      | 3 TWI3                                                                                                  |
| 18:0 (R/W)         | VALUE      | 4 TWI4                                                                                                  |
| 18:0 (R/W)         | VALUE      | 5 TWI5                                                                                                  |
| 18:0 (R/W)         | VALUE      | 6 xSPI_SYSCLK                                                                                           |
| 18:0 (R/W)         | VALUE      | 7 USBULPI                                                                                               |
| 18:0 (R/W)         | VALUE      | 8 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 9 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 10 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 11 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 12 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 13 Reserved                                                                                             |

Table 4-7: DPM\_PER\_DIS1 Register Fields (Continued)

| Bit No.   |   Bit Name | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            |                           |
|           |         14 | XSPI_CLK010               |
|           |         15 | PDM                       |
|           |         16 | Reserved                  |
|           |         17 | Reserved                  |
|           |         18 | Reserved                  |

## Revision ID

The DPM\_REVID register provides the revision of the DPM module.

Figure 4-4: DPM\_REVID Register Diagram

<!-- image -->

Table 4-8: DPM\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version ID.         |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## Status Register

The DPM\_STAT register contains bits that report the state of the module and various errors.

Figure 4-5: DPM\_STAT Register Diagram

<!-- image -->

Table 4-9: DPM\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | LWERR      | Lock Write Error. The DPM_STAT.LWERR bit indicates that a write transaction attempted an access to a write protected register. Triggers the DPMLV_PSLVERR interrupt.                                                                                                 |
| 16 (R/W1C)         | ADDRERR    | Address Error. The DPM_STAT.ADDRERR bit indicates that a read or write transaction attempted an access to an unimplemented address or a write transaction attempted an access to a read only register or accesses are non aligned. Triggers the DPMLV_PSLVERR inter- |
| 7:4 (R/NW)         | PRVMODE    | 1 Active Previous Mode. The DPM_STAT.PRVMODE bit field indicates the previous mode of the module.                                                                                                                                                                    |
|                    |            | 0 Reset                                                                                                                                                                                                                                                              |
|                    |            | Reserved                                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                    |
|                    |            | 3 Reserved                                                                                                                                                                                                                                                           |
|                    |            | 4 Reserved                                                                                                                                                                                                                                                           |
|                    |            | 5-15 Reserved                                                                                                                                                                                                                                                        |

Table 4-9: DPM\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 3:0 (R/NW)         | CURMODE    | Current Mode. The DPM_STAT.CURMODE bit field indicates the current mode of the module. |