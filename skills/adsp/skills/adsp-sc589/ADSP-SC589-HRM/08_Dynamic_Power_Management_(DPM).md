## 5   Dynamic Power Management (DPM)

The dynamic power management (DPM) unit of the processor controls transitions between different power-saving modes.

## DPM Features

The DPM allows programs to control the power mode of the processor as follows.

- Permits operation of multiple, external wake-up sources

## DPM Functional Description

The DPM can be programmed to transition between power modes.

## ADSP-SC58x DPM Register List

A set of registers govern DPM operations. For more information on DPM functionality, see the DPM register descriptions.

Table 5-1: ADSP-SC58x DPM Register List

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

The DPM\_PER\_DIS0 Register Mapping table shows the module clocks and the corresponding peripheral. The DPM\_PER\_DIS0 register is used to shut off the clock to each peripheral if it is not required by the application.

NOTE: In the table, SMPU-L2CTL-CL2\_x and SMPU-L2CTL-DL2\_x correspond to the SMPU modules associated with the core and the DMA ports of L2 respectively. The six instances of SMPU blocks are shown in the Figure 9-1 ADSP-SC58x Complete L2 System Block Diagram - two SMPUs for each L2CLTL, one for the port 0-core and another for the port 1-DMA.

Table 5-2: DPM\_PER\_DIS0 Register Mapping

| Peripheral Name   | Gated Module clocks   |   DPM_PER_DIS0 bit |
|-------------------|-----------------------|--------------------|
| FIR0              | SCLK0                 |                  0 |
| IIR0              | SCLK0                 |                  1 |
| FFT0              | SYSCLK                |                  2 |
| RTC0              | SCLK0                 |                  4 |
| DAI0              | SCLK0 CLK05 SYSCLK    |                  5 |

Table 5-2: DPM\_PER\_DIS0 Register Mapping (Continued)

| Peripheral Name                    | Gated Module clocks   |   DPM_PER_DIS0 bit |
|------------------------------------|-----------------------|--------------------|
| DAI1                               | SCLK0 CLK05 SYSCLK    |                  6 |
| HAE0                               | SCLK0                 |                  7 |
| SINC0                              | SCLK0                 |                  8 |
| MLB0                               | SYSCLK                |                  9 |
| EMAC0 (GigE)                       | CLKO7 SCLK0           |                 10 |
| EMAC1(10/100)                      | SCLK0                 |                 11 |
| MSI0                               | CLKO9 SCLK0           |                 12 |
| EMDMA0/CH0                         | SCLK0                 |                 13 |
| EMDMA0/CH1                         | SCLK0                 |                 14 |
| PCIe                               | CLKO6 SYSCLK          |                 15 |
| CRYPTO ACCELERATOR-0(EIP-150/ PKP) | SCLK0                 |                 16 |
| CRYPTO ACCELERATOR-1(EIP-93/SPE)   | SCLK0                 |                 17 |
| SMPU-SMC                           | SCLK0                 |                 18 |
| SMPU-SPI2                          | SCLK1                 |                 19 |
| SMPU-L2CTL-CL2-0                   | SYSCLK                |                 20 |
| SMPU-L2CTL-DL2-0                   | SYSCLK                |                 21 |
| SMPU-L2CTL-CL2-1                   | SYSCLK                |                 22 |
| SMPU-L2CTL-DL2-1                   | SYSCLK                |                 23 |
| SMPU-L2CTL-CL2-2                   | SYSCLK                |                 24 |
| SMPU-L2CTL-DL2-2                   | SYSCLK                |                 25 |
| SMPU-PCIE-Slave                    | SYSCLK                |                 26 |
| SMPU-DMC0                          | SYSCLK                |                 27 |
| SMPU-DMC1                          | SYSCLK                |                 28 |
| CAN0                               | CLKO4                 |                 29 |
| CAN1                               | CLKO4                 |                 30 |

Table 5-3: DPM\_PER\_DIS1 Register Mapping

| Peripheral Name   | Gated Module Clocks   |   DPM_PER_DIS1 bit |
|-------------------|-----------------------|--------------------|
| TWI0              | SCLK0                 |                  0 |
| TWI1              | SCLK0                 |                  1 |
| TWI2              | SCLK0                 |                  2 |
| USB0              | SCLK0                 |                  3 |
| USB1              | SCLK0                 |                  4 |

## ADSP-SC58x DPM Register Descriptions

Dynamic Power Management (DPM) contains the following registers.

Table 5-4: ADSP-SC58x DPM Register List

| Name         | Description                    |
|--------------|--------------------------------|
| DPM_CTL      | Control Register               |
| DPM_PER_DIS0 | Peripherals Disable Register 0 |
| DPM_PER_DIS1 | Peripherals Disable Register 1 |
| DPM_REVID    | Revision ID                    |
| DPM_STAT     | Status Register                |

## Control Register

The DPM\_CTL register controls sleep modes selections and PLL operations of the DPM. A write protect feature permits locking out changes to this register.

Figure 5-1: DPM\_CTL Register Diagram

![Image](08_Dynamic_Power_Management_(DPM)_artifacts/image_000000_92ca20295c2a8af5ad3073a767a41b0e5ebd711e9c2d4b7ed260b6e3c393c005.png)

Table 5-5: DPM\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock. If the global lock bit is set ( SPU_CTL.GLCK bit =1) and the DPM_CTL.LOCK bit is set, the DPM_CTL register is read only (locked). |
| 31 (R/W)           | LOCK       | 0 Unlock                                                                                                                                |
| 31 (R/W)           | LOCK       | 1 Lock                                                                                                                                  |

## Peripherals Disable Register 0

The DPM\_PER\_DIS0 register is used to shut off the clocks to peripherals that are not used in an application to save clock switching power.

Figure 5-2: DPM\_PER\_DIS0 Register Diagram

![Image](08_Dynamic_Power_Management_(DPM)_artifacts/image_000001_94bae5e184c14a95b6ddfa3690e3dfcf9e30d1a8673c505e6354ce415d43a745.png)

Table 5-6: DPM\_PER\_DIS0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit.                                                                                               |
| 30:0 (R/W)         | VALUE      | Peripheral Disable. The DPM_PER_DIS0.VALUE bits are used to shut of clocks in individual peripher- als. |
| 30:0 (R/W)         |            | 0 FIR0                                                                                                  |
| 30:0 (R/W)         |            | 1 IIR0                                                                                                  |
| 30:0 (R/W)         |            | 2 FFT0                                                                                                  |
| 30:0 (R/W)         |            | 3 Reserved                                                                                              |
| 30:0 (R/W)         |            | 4 RTC0                                                                                                  |
| 30:0 (R/W)         |            | 5 DAI0                                                                                                  |
| 30:0 (R/W)         |            | 6 DAI1                                                                                                  |
| 30:0 (R/W)         |            | 7 HAE0                                                                                                  |
| 30:0 (R/W)         |            | 8 SINC0                                                                                                 |
| 30:0 (R/W)         |            | 9 MLB0                                                                                                  |
| 30:0 (R/W)         |            | 10 EMAC0 (GigE)                                                                                         |
| 30:0 (R/W)         |            | 11 EMAC1(10/100)                                                                                        |
| 30:0 (R/W)         |            | 12 MSI0                                                                                                 |
| 30:0 (R/W)         |            | 13 EMDMA0/CH1                                                                                           |

Table 5-6: DPM\_PER\_DIS0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |   Description/Enumeration | Description/Enumeration            |
|--------------------|------------|---------------------------|------------------------------------|
|                    |            |                        14 | EMDMA0/CH0                         |
|                    |            |                        15 | PCIe-Slave                         |
|                    |            |                        16 | CRYPTO ACCELERATOR-0 (EIP-150/PKP) |
|                    |            |                        17 | CRYPTO ACCELERATOR-1 (EIP-150/PKP) |
|                    |            |                        18 | SMPU-SMC                           |
|                    |            |                        19 | Reserved                           |
|                    |            |                        20 | SMPU-L2CTL-CL2-0                   |
|                    |            |                        21 | SMPU-L2CTL-DL2-0                   |
|                    |            |                        22 | SMPU-L2CTL-CL2-1                   |
|                    |            |                        23 | SMPU-L2CTL-DL2-1                   |
|                    |            |                        24 | SMPU-L2CTL-CL2-2                   |
|                    |            |                        25 | SMPU-L2CTL-DL2-2                   |
|                    |            |                        26 | SMPU-PCIE-SLV                      |
|                    |            |                        27 | SMPU-DMC0                          |
|                    |            |                        28 | SMPU-DMC1                          |
|                    |            |                        29 | CAN0                               |
|                    |            |                        30 | CAN1                               |

## Peripherals Disable Register 1

The DPM\_PER\_DIS1 register is used to shut off the clocks to peripherals that are not used in an application to save clock switching power.

Figure 5-3: DPM\_PER\_DIS1 Register Diagram

![Image](08_Dynamic_Power_Management_(DPM)_artifacts/image_000002_33f87373864ffa1d11970822e18a9a2ee8c6027e13c32913fac7b4d01f0181b1.png)

Table 5-7: DPM\_PER\_DIS1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | LOCK       | Lock Bit.                                                                                               |
| 18:0 (R/W)         | VALUE      | Peripheral Disable. The DPM_PER_DIS1.VALUE bits are used to shut of clocks in individual peripher- als. |
| 18:0 (R/W)         | VALUE      | 0 TWI0                                                                                                  |
| 18:0 (R/W)         | VALUE      | 1 TWI1                                                                                                  |
| 18:0 (R/W)         | VALUE      | 2 TWI2                                                                                                  |
| 18:0 (R/W)         | VALUE      | 3 USB0                                                                                                  |
| 18:0 (R/W)         | VALUE      | 4 USB1                                                                                                  |
| 18:0 (R/W)         | VALUE      | 5 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 6 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 7 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 8 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 9 Reserved                                                                                              |
| 18:0 (R/W)         | VALUE      | 10 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 11 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 12 Reserved                                                                                             |
| 18:0 (R/W)         | VALUE      | 13 Reserved                                                                                             |

Table 5-7: DPM\_PER\_DIS1 Register Fields (Continued)

| Bit No.   |   Bit Name | Description/Enumeration   |
|-----------|------------|---------------------------|
| (Access)  |            |                           |
|           |         14 | Reserved                  |
|           |         15 | Reserved                  |
|           |         16 | Reserved                  |
|           |         17 | Reserved                  |
|           |         18 | Reserved                  |

## Revision ID

The DPM\_REVID register provides the revision of the DPM module.

Figure 5-4: DPM\_REVID Register Diagram

![Image](08_Dynamic_Power_Management_(DPM)_artifacts/image_000003_21ac7cce54c1adaef5016f3f1843f0972e71217134d1c8cee6b4cb7d20359cdc.png)

Table 5-8: DPM\_REVID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:4                | MAJOR      | Major Version ID.         |
| 3:0 (R/NW)         | REV        | Incremental Version ID.   |

## Status Register

The DPM\_STAT register contains bits that report the state of the module and various errors.

Figure 5-5: DPM\_STAT Register Diagram

![Image](08_Dynamic_Power_Management_(DPM)_artifacts/image_000004_1e5d98e8914d843c7afc1fb0d2708a58e7aebd31d5ad710825b8c5a4c45958e3.png)

Table 5-9: DPM\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W1C)         | HVBSYERR   | HV Busy Error. The DPM_STAT.HVBSYERR bit indicates that a Read access to the Wake Status or Restoren registers (during engineering transfers only) while the DPMLV is transferring registers from the DPMHV. Triggers the DPMLV_PSLVERR interrupt.                                    |
| 17 (R/W1C)         | LWERR      | Lock Write Error. The DPM_STAT.LWERR bit indicates that a write transaction attempted an access to a write protected register. Triggers the DPMLV_PSLVERR interrupt.                                                                                                                  |
| 16 (R/W1C)         | ADDRERR    | Address Error. The DPM_STAT.ADDRERR bit indicates that a read or write transaction attempted an access to an unimplemented address or a write transaction attempted an access to a read only register or accesses are non aligned. Triggers the DPMLV_PSLVERR inter- rupt. 0 Inactive |
| 7:4 (R/NW)         | PRVMODE    | Previous Mode. The DPM_STAT.PRVMODE bit field indicates the previous mode of the the module.                                                                                                                                                                                          |

Table 5-9: DPM\_STAT Register Fields (Continued)

| Bit No.    | Bit Name   | Description/Enumeration                                                                    |
|------------|------------|--------------------------------------------------------------------------------------------|
| (Access)   |            | 0 Reset                                                                                    |
|            |            | 1 Full-On                                                                                  |
|            |            | 2 Reserved                                                                                 |
|            |            | 3 Reserved                                                                                 |
|            |            | 4 Reserved                                                                                 |
|            |            | 5-15 Reserved                                                                              |
| 3:0 (R/NW) | CURMODE    | Current Mode. The DPM_STAT.CURMODE bit field indicates the current mode of the the module. |
|            |            | 0 Reserved                                                                                 |
|            |            | 1 Full-On                                                                                  |
|            |            | 2 Reserved                                                                                 |
|            |            | 3 Reserved                                                                                 |
|            |            | 4-15 Reserved                                                                              |