## 21   Extended Serial Peripheral Interface (xSPI)

The xSPI flash controller provides access to serial flash devices that support JESD216 and JESD251 standards. The controller accesses flash devices in modes that permit efficient communication with minimal software overhead. The supported HyperBus ™  protocol enables seamless communication with HyperFlash ™ /HyperRAM ™ devices. The standard SPI protocol is supported along with high performance dual, quad, and octal SPI modes, where data can be transferred on up to 8 data pins. In addition to the normal mode of operation in single transfer (STR) mode, the xSPI controller supports a dual transfer rate (DTR) mode. In DTR mode, data, address, and commands are transferred on both edges of the serial clock. With direct access mode support, any read/write to the xSPI memory-mapped space triggers a read/write to flash memory.

An integrated PHY handles the low-level timings of the data, address and control signals between the device and controller.

The xSPI controller also provides support for RWDS/DQS when sampling data. This feature improves read data capture, which, along with the PHY, enables high-speed data transfers.

## xSPI Features

The xSPI controller supports the following features:

- Memory-mapped direct mode operation for performing flash data transfers
- Auto Command Mode (ACMD) to perform various memory operations with minimum software overhead
- An ACMD DMA engine that triggers transactions using descriptors fetched from the main memory. It automatically moves data between the device memory and system host memory.
- STIG mode operation to issue flash commands
- Automatic flash status polling support for flash program
- Execute in Place (XIP) support
- Device Discovery (DD) to identify the memory device
- Support for single, dual, quad, or octal modes of operation
- Support for DDR mode and DTR protocol (including octal DDR protocol)

- Data integrity features (CRC or ECC) for flash memories
- Programmable baud rate, clock phase, and polarity
- Programmable interrupt generation
- Programmable chip select control timing
- Programmable dummy cycles for read and write operations
- Support for up to 80 MHz in non-DQS mode
- Support for up to 125 MHz in DQS mode

## Supported Memory Devices

The xSPI controller supports any device that is compatible with the JESD216 specification standard. In this standard, these devices are defined as profile 1 or profile 2 type devices. T raditional serial NOR flash devices tend to be compatible with the profile 1 type as defined in that standard. Devices supporting HyperBus (traditional HyperFlash and most HyperRAM or pSRAM devices) tend to be compatible with the profile 2 type. Sometimes, a pSRAM or HyperRAM device may support a profile 1 style interface; the device manufacturer selects the interface.

The supported devices include:

- Traditional serial SPI (single/dual/quad/octal) - Memory devices defined as profile 1 in the JEDEC JESD216 specification. The device may have been manufactured before the specification was released. Any device that specifies compliance with an xSPI profile 1, where profile 1 is described in the xSPI JEDEC JESD216 specification.
- Traditional HyperFlash - Memory devices defined as profile 2 in the JEDEC JESD216 specification. The device may have been manufactured before the specification was released. Any devices that specify specifies compliance with an xSPI profile 2, where profile 2 is described in the xSPI JEDEC JESD216 specification.
- Serial RAM and HyperRAM - Serial SRAM devices (for example, OctaRAM ™ /OctalRAM, HyperRAM, pSRAM) devices. These volatile RAM devices tend to be interfaced similar to profile 1 or profile 2. However, they must be occasionally self-refreshed to maintain state and may not require some elements of the sequences used to read or program traditional flash devices.
- Serial NAND - also referred to as SPI NAND

## xSPI Functional Description

This section provides information on the function of the xSPI module.

## ADSP-2184x XSPI Register List

The XSPI is an expanded SPI controller. It contains the following registers.

Table 21-1: ADSP-2184x XSPI Register List

| Name                        | Description                                |
|-----------------------------|--------------------------------------------|
| XSPI_BOOT_STAT              | Boot Status Register                       |
| XSPI_CMD0                   | Command Register 0                         |
| XSPI_CMD1                   | Command Register 1                         |
| XSPI_CMD2                   | Command Register 2                         |
| XSPI_CMD3                   | Command Register 3                         |
| XSPI_CMD4                   | Command Register 4                         |
| XSPI_CMD5                   | Command Register 5                         |
| XSPI_CMD_STAT               | Command Status Register                    |
| XSPI_CMD_STAT_PTR           | Command Status Pointer Register            |
| XSPI_DAC_CFG                | DAC Configuration Register                 |
| XSPI_DAC_REMAPADDR0         | Address Remapping Register 0               |
| XSPI_DAC_REMAPADDR1         | Address Remapping Register 1               |
| XSPI_DISCOVERY_GCTL         | Device Discovery Control Register          |
| XSPI_DMA_CTL                | DMAInterface Control Register              |
| XSPI_DMA_ERRADDR_HI         | DMAError Address High Register             |
| XSPI_DMA_ERRADDR_LO         | DMAError Address Low Register              |
| XSPI_ERS_SEQ_CFG0           | Erase Sequence Configuration Register 0    |
| XSPI_ERS_SEQ_CFG1           | Erase Sequence Configuration Register 1    |
| XSPI_ERS_SEQ_CFG2           | Erase Sequence Configuration Register 2    |
| XSPI_FEATURES               | Controller Features Register               |
| XSPI_GSTAT                  | General Controller Status Register         |
| XSPI_INT_EN                 | Interrupt Enable Register                  |
| XSPI_ISTAT                  | Interrupt Status Register                  |
| XSPI_LONGPOL_GCTL           | Long Polling Count Register                |
| XSPI_MINICTL_DEV_ACTIVE_MAX | Device Active Maximum Clock Cycle Register |
| XSPI_MINICTL_CLKMODE        | Clock Mode Control Register                |
| XSPI_MINICTL_DEV_DLY        | Device Delay Register                      |
| XSPI_MINICTL_HF_OFFSET      | HyperFlash Offset Register                 |
| XSPI_MINICTL_JEDEC_RST_TR   | JEDEC Reset Delay Register                 |
| XSPI_MINICTL_RST_PIN_CTL    | Hardware Reset Control Register            |
| XSPI_MINICTL_RST_RECOV      | Reset Recovery Delay Register              |

Table 21-1: ADSP-2184x XSPI Register List (Continued)

| Name                        | Description                               |
|-----------------------------|-------------------------------------------|
| XSPI_MINICTL_WP_CTL         | Write Protect Register                    |
| XSPI_PHY_DLLOB0             | PHY DLL Observable Points 0 Register      |
| XSPI_PHY_DLLOB1             | PHY DLL Observable Points 1 Register      |
| XSPI_PHY_DLL_CTL            | PHY DLL Control Register                  |
| XSPI_PHY_DLL_REQ_CTL        | Bus Requester DLL Control Register        |
| XSPI_PHY_DLL_COMP_CTL       | Bus Completer DLL Control Register        |
| XSPI_PHY_DLL_UPDT_CNT       | PHY DLL Resynchronization Register        |
| XSPI_PHY_DQS_TR             | PHY DQS Timing Register                   |
| XSPI_PHY_DQ_TR              | PHY DQTiming Register                     |
| XSPI_PHY_FEATURES           | PHY Features Register                     |
| XSPI_PHY_GATE_LPBK_CTL      | PHY Gate Loopback Control Register        |
| XSPI_PHY_GCTL               | PHY Global Control Register               |
| XSPI_PHY_GPIO_CTL0          | PHY GPIO Control Register 0               |
| XSPI_PHY_GPIO_CTL1          | PHY GPIO Control Register 1               |
| XSPI_PHY_GPIO_STAT0         | PHY GPIO Status Register 0                |
| XSPI_PHY_GPIO_STAT1         | PHY GPIO Status Register 1                |
| XSPI_PHY_GTSEL              | PHY Global Termination Control Register   |
| XSPI_PHY_IE_TR              | PHY DQS Input Enable Timing Register      |
| XSPI_PHY_OB0                | PHY Observable Points Register            |
| XSPI_PHY_REVID              | PHY Revision ID Register                  |
| XSPI_PHY_STATIC_TGL         | PHY Static Aging Register                 |
| XSPI_PHY_WR_DESKEW_PAD_CTL0 | PHY Deskew Write Register                 |
| XSPI_PROG_SEQ_CFG0          | Program Sequence Configuration Register 0 |
| XSPI_PROG_SEQ_CFG1          | Program Sequence Configuration Register 1 |
| XSPI_PROG_SEQ_CFG2          | Program Sequence Configuration Register 2 |
| XSPI_READ_SEQ_CFG0          | Read Sequence Configuration Register 0    |
| XSPI_READ_SEQ_CFG1          | Read Sequence Configuration Register 1    |
| XSPI_READ_SEQ_CFG2          | Read Sequence Configuration Register 2    |
| XSPI_REVID                  | Revision ID Register                      |
| XSPI_RST_SEQ_CFG0           | Reset Sequence Configuration Register 0   |
| XSPI_RST_SEQ_CFG1           | Reset Sequence Configuration Register 1   |

Table 21-1: ADSP-2184x XSPI Register List (Continued)

| Name                | Description                                          |
|---------------------|------------------------------------------------------|
| XSPI_SDMA_ADDR0     | Host DMABuffer Address Register 0                    |
| XSPI_SDMA_ADDR1     | Host DMABuffer Address Register 1                    |
| XSPI_SDMA_SIZ       | Host DMABlock Size Register                          |
| XSPI_SDMA_TRD_STAT  | Host DMAThread Status Register                       |
| XSPI_SEQ_GCTL0      | Sequence Configuration Register 0                    |
| XSPI_SEQ_GCTL1      | Sequence Configuration Register 1                    |
| XSPI_SHORTPOL_GCTL  | Short Polling Count Register                         |
| XSPI_STAT_SEQ_CFG0  | Status Checking Sequence Configuration Register 0    |
| XSPI_STAT_SEQ_CFG1  | Status Checking Sequence Configuration Register 1    |
| XSPI_STAT_SEQ_CFG10 | Status Checking Sequence Configuration Register 10   |
| XSPI_STAT_SEQ_CFG2  | Status Checking Sequence Configuration Register 2    |
| XSPI_STAT_SEQ_CFG3  | Status Checking Sequence Configuration Register 3    |
| XSPI_STAT_SEQ_CFG4  | Status Checking Sequence Configuration Register 4    |
| XSPI_STAT_SEQ_CFG5  | Status Checking Sequence Configuration Register 5    |
| XSPI_STAT_SEQ_CFG7  | Status Checking Sequence Configuration Register 7    |
| XSPI_STAT_SEQ_CFG8  | Status Checking Sequence Configuration Register 8    |
| XSPI_STAT_SEQ_CFG9  | Status Checking Sequence Configuration Register 9    |
| XSPI_TRD_COMP_ISTAT | Auto Command Engine Interrupt Status Thread Register |
| XSPI_TRD_ERR_INT_EN | Thread Error Interrupt Enable Register               |
| XSPI_TRD_ERR_ISTAT  | Thread Error Interrupt Status Register               |
| XSPI_TRD_STAT       | Auto Command Engine Thread Status Register           |
| XSPI_WE_SEQ_CFG0    | WEL Sequence Configuration Register                  |
| XSPI_WORKMODE_CTL   | Device Control Register                              |
| XSPI_XIP_GCTL       | XIP Configuration Register                           |

## ADSP-2184x xSPI Interrupt List

Table 21-2: ADSP-2184x SPI Interrupt List

|   Interrupt ID | Name      | Description             | Sensitivity   | DMA Channel   |
|----------------|-----------|-------------------------|---------------|---------------|
|            386 | XSPI0_IRQ | XSPI0 Interrupt Request | Level         |               |
|            387 | XSPI1_IRQ | XSPI1 Interrupt Request | Level         |               |

## xSPI Block Diagram

The xSPI module is comprised of:

- Main command sequencer
- PHY block
- Internal transmit/receive FIFOs
- DAC/STIG controller
- Register interface

The xSPI Controller Block Diagram shows the xSPI controller functional blocks.

Figure 21-1: xSPI Controller Block Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000000_f1f8b1a95dadfbe5e968b94189bc1ab1eaa33d367c2a28a70785a3b3b4642409.png)

## Architectural Concepts

There are three clock sources for xSPI controller : XSPI\_CLK, SCLK0\_0, SYSCLK\_0. SYSCLK\_0 is used to transfer data over the bus between the controller (core/DMA) and the xSPI controller. XSPI\_CLK is driven on the external XSPI\_CLK pin to transfer the data to/from external devices. And SCLK0\_0 is used to access the xSPI controller registers to perform basic configuration of the controller and interrupt handling.

NOTE: For XSPI0, XSPI\_CLK is sourced from CLKO10 (output clock of CDU) and for XSPI1, XSPI\_CLK is sourced from CLKO8 (output clock of CDU). A user can select one of the options for CLKO8 and CLKO10 to source xSPI controllers. Rrefer to the CDU chapter for the clocking diagram and more details.

The xSPI controller supports independent modes/controllers for different memory devices:

- Direct mode
- STIG mode
- Auto Command Mode (ACMD, CDMA or PIO)

The bus interface provides access to controller status and configuration registers. The AXI host interface is used in direct mode, STIG mode (when necessary), and optionally in ACMD mode. The AXI internal DMA interface is only used in ACMD mode, where the internal DMA module of the xSPI controller owns and manages system transfers, and handles DMA descriptor fetches and writes and associated data buffer transfers. The DMA module used in ACMD mode implements the bus requester capability. It can automatically transfer data from/to system memory and from/to the connected xSPI device (using internal SRAM).

The boot engine and device discovery blocks in the xSPI controller use the internal DMA interface and FIFOs.

The main command sequencer holds the main sequencing logic. It is responsible for building the command structures and meeting the required device timings. It also interfaces with the PHY module and regulates data flow between the physical layer and the system interface. PHY is responsible for handling the low-level timings of the data, address, and control signals between the device and controller.

## Operating Modes

There are three main usage modes for the xSPI controller: direct mode, STIG mode, and ACMD mode. These modes can perform standard read/write transactions to a device memory. However, only one mode can be active at a time.

One active work mode is selected at a time to communicate with an attached device. ACMD or direct modes are the primary operational modes. STIG mode is used as an accompanying mode to send commands or a sequences of commands to the device that may not be directly supported in ACMD or direct mode. STIG mode is a highly flexible mode that can also be used to write or read data to the device and perform other operations. However, STIG mode relies more on software to construct the necessary command structures.

ACMD mode requires that software either manage a descriptor table with high-level sequences defined within, or manage some command registers within the controller to issue these same high-level sequences. The controller can automatically translate high-level sequences (reset, erase, erase\_all, program, read) into subsequences that are tied to the device type.

Direct mode, as the name suggests, directly translates read and write operations detected on the AXI host bus completer interface to equivalent low-level read and write subsequences at the device. As such, direct mode can only support read or program operations. It relies on STIG mode to perform other operations such as erase. It can also be used to directly boot from the device, when required.

## Direct Mode

In direct mode, reads and writes that are issued by the processor (either by the core or MDMA controller) enter the controller using the AXI bus completer interface. The read/write operations are directly translated to read/write at the memory device. The shaded portion of the Direct Mode Block Diagram shows the flow. Direct mode allows the

processor to execute code directly, or have direct access from/to the attached memory device. This flow is sometimes referred to as eXecute-In-Place or XIP . The controller support devices operating in XIP mode (in all work modes, not just direct). However, there are steps the host must take to trigger entry to and exit from XIP mode.

See the topic Operating in Direct Mode for details on operating in direct mode, programming the mode, and usage restrictions.

Figure 21-2: Direct Mode Block Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000001_f2032b9f6e9d02b1bd5ea64cce79a9e42931fd522e7741c6efe74d6b548e05f0.png)

## STIG Mode

Software Triggered Instruction Generator (STIG) mode is configured and triggered using the programming interface. This mode is very flexible and configurable. In addition to performing standard reads and writes, it can also be used to access memory device registers and/or trigger functions in the memory device such as erase, read JEDEC ID, etc. STIG mode uses data paths that are similar to direct mode. However, the commands are configured and triggered through access to registers using the APB interface. Data can optionally be passed from the AXI bus completer interface, or through other registers in the APB register block. The shaded portion of the STIG Mode Block Diagram shows the operational flow.

See the topic Operating in STIG Mode for details on operating in STIG mode, programming the mode, and usage restrictions.

Figure 21-3: STIG Mode Block Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000002_af4c926a999e7b31e02692f7383f506e7bdf47d1c50fc8a81446930704fe021b.png)

## ACMD Mode

Auto Command Mode (ACMD) mode, the user configures an advanced DMA engine capable of operating multiple parallel threads. The configuration uses controller registers and/or descriptors held in the main memory. The shaded portion of the ACMD Mode Block Diagram shows the operational flow. The ACMD DMA engine triggers transactions using the descriptors fetch from main memory to automatically move data between the device memory and system memory. ACMD mode is configured to use the AXI internal DMA bus requester interface to transfer data and to collect and process DMA descriptors.

ACMD mode can also be configured to use the AXI host interface instead. In this case, the controller signals to the processor using an interrupt or status register when it requires the host to send write data on the bus completer AXI write ports, or when it needs the processor to issue an AXI read to fetch read data.

See the topic Operating in ACMD Mode for details on operating in ACMD mode, programming the mode, and usage restrictions.

Figure 21-4: ACMD Block Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000003_91413af24fa131e2ed35111a8a11a191998b460ca788de0296640d6a4ec28776.png)

## Integrated Soft PHY

The xSPI PHY controller is responsible for handling the low-level timings of the data, address, and control signals between the device and controller. It provides additional logic, which operates as a bridge between formulated xSPI transactions and the timing-aware off-chip xSPI transfer to the external device. PHY has its own programmable registers. Accessing these registers occurs through the standard controller APB address space. Before the processor can communicate with the memory device, the PHY must be initialized and locked.

NOTE: In ADSP-2156x processors, the internal DMA of the xSPI controller is not supported.

See the section xSPI PHY Controller for details on how to use PHY, programming the PHY, and usage restrictions.

## XIP Mode

The eXecute-In-Place (XIP) mode is also referred to as continuous mode. When the controller is in XIP mode, the processor executes code directly from the attached memory device or from where the processor has direct access to the attached memory device. The controller supports this function using the direct work mode. If executing code directly is needed, enable direct work mode.

The device uses XIP mode to improve read performance by removing the need for the controller to send the read opcode for any read operation. If the read opcode is typically sent over eight device clock cycles (SDR, serial transmission of the opcode), the read operation latency is reduced by eight cycles. This difference can significantly improve performance when executing code directly from the device. On the downside, when XIP mode is entered, the device can only perform read operations. If a different operation is required, XIP mode must be exited. Although

the controller supports XIP mode, there are steps the host must take to trigger entry to and exit from XIP mode. These steps depend on the selected work mode. See the section Entering and Exiting XIP Mode.

## Configuring the Controller

To use the controller in direct, ACMD, or STIG mode, there are global configuration registers that must be configured. These are defined in the common sequence registers.

Use the following steps to configure the controller.

1. Configure the XSPI\_SEQ\_GCTL0 register to select the correct xSPI profile as per the memory device, to enable or disable the data swap feature, to select single or dual bytes per address, device page program size, device page read size, and other desired features.
2. Configure the XSPI\_SEQ\_GCTL1 register. This register is used only for SPI NAND profile to configure plane count, pages per block, extended page size, etc.
3. Configure the erase/read/program sequence. Configure the read or program sequence that the controller must send to the memory device. This sequence depends on the attached memory device. Refer to the device data sheet for details.

Some devices require a write enable sub sequence to be issued prior to a program operation (for example, to configure the write enable latch or WEL in the device). By default, it is assumed to be memory device requirement. However, the sequence can be disabled using the XSPI\_WE\_SEQ\_CFG0.P1\_EN bit. By default, the command extension of the write enable sequence is disabled. It is sent in a serial SDR format with an opcode of 0x6. If any of the parameters need changes, configure the XSPI\_WE\_SEQ\_CFG0 register accordingly.

Some devices require the device to be polled for correctness and readiness after issuing an operation. Due to the differences between how device types interpret and respond to status requests, and different types of status requests, the controller requires several programmable registers that must be tuned to the device requirements. For direct mode, the controller can optionally check the device fail or busy status following a program or erase operation (or additionally for soft\_reset/read for SPI NAND devices). For SPI NAND devices, the controller can also check the ECC status following a read page command. All of these requirements are configured using the XSPI\_STAT\_SEQ\_CFGn registers ( XSPI\_STAT\_SEQ\_CFG0 to XSPI\_STAT\_SEQ\_CFG10 ).

NOTE: When using the device discovery engine, the registers are updated automatically after power on reset. Refer to section Device Discovery for details.

The Global Configurations Registers table shows the registers used to configure the controller for various operations and types of flash devices. These registers must be initialized and configured when operating in direct or ACMD mode.

Table 21-3: Global Configurations Registers

|                                                   | Profile 1 Devices                                                                                                                                               | Profile 2 - HyperFlash Devices                                                                      | Profile 2 - HyperRAM Devices   | SPI NAND Devices                                                                                                                                                |
|---------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------|--------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Program                                           | XSPI_PROG_SEQ_CF G0 XSPI_PROG_SEQ_CF G1                                                                                                                         | XSPI_PROG_SEQ_CF G2                                                                                 | XSPI_PROG_SEQ_CF G2            | XSPI_PROG_SEQ_CF G0 XSPI_PROG_SEQ_CF G1                                                                                                                         |
| Read                                              | XSPI_READ_SEQ_CF G0 XSPI_READ_SEQ_CF G1                                                                                                                         | XSPI_READ_SEQ_CF G2                                                                                 | XSPI_READ_SEQ_CF G2            | XSPI_READ_SEQ_CF G0 XSPI_READ_SEQ_CF G1                                                                                                                         |
| Write                                             | XSPI_WE_SEQ_CFG0                                                                                                                                                | N/A                                                                                                 | N/A                            | XSPI_WE_SEQ_CFG0                                                                                                                                                |
| Device Fail Status Check Device Busy Status Check | XSPI_STAT_SEQ_CF G0 XSPI_STAT_SEQ_CF G1 XSPI_STAT_SEQ_CF G2 XSPI_STAT_SEQ_CF G3 XSPI_STAT_SEQ_CF G5 XSPI_STAT_SEQ_CF G7 XSPI_STAT_SEQ_CF G8 XSPI_STAT_SEQ_CF G9 | XSPI_STAT_SEQ_CF G4 XSPI_STAT_SEQ_CF G5 XSPI_STAT_SEQ_CF G7 XSPI_STAT_SEQ_CF G8 XSPI_STAT_SEQ_CF G9 | N/A                            | XSPI_STAT_SEQ_CF G0 XSPI_STAT_SEQ_CF G1 XSPI_STAT_SEQ_CF G2 XSPI_STAT_SEQ_CF G3 XSPI_STAT_SEQ_CF G5 XSPI_STAT_SEQ_CF G7 XSPI_STAT_SEQ_CF G8 XSPI_STAT_SEQ_CF G9 |
| ECC Status Check                                  | N/A                                                                                                                                                             | N/A                                                                                                 | N/A                            | XSPI_STAT_SEQ_CF G10                                                                                                                                            |

## Operating in Direct Mode

In direct mode, reads and writes that are issued by the processor enter the controller through the AXI bus completer interface. The read/write operations are directly translated to reads and writes at the memory device. Along with standard code execution, this mode can also be used immediately after power on reset to boot data from the attached memory device.

NOTE: This mode is different from the dedicated boot feature. The boot engine is specifically used to transfer boot code from the memory device to an area of host memory following power on reset (from which the host can then boot). There is a dedicated xSPI boot mode for the processor to support this feature. See the xSPI Controller Boot Mode topic in the Booting chapter for details.

Direct work mode is enabled by default ( XSPI\_WORKMODE\_CTL.WORKMODE ). The XSPI\_WORKMODE\_CTL register can only be changed when the controller is in an idle state. This state can be determined by polling the CRTL\_BUSY pin of the xSPI flash controller or by polling the XSPI\_GSTAT.CTL\_BUSY bit. When direct mode is enabled, access to the XSPI\_CMDn registers (which are only required for other work modes) are ignored and should be avoided.

Read and program/write transactions to the memory device occur only as a direct consequence of the processor applying read or write AXI transactions to the AXI bus completer interface through memory-mapped xSPI addresses.

NOTE: In direct mode, the configured page size for write or read transfers must be configured to a value wider than the selected word size of the AXI transaction (sA*SIZE). Also, the xSPI page boundaries must be aligned to the AXI transaction word size (sA*SIZE).

In addition to configuring the global registers described in Configuring the Controller, configure the direct mode specific registers such as XSPI\_DAC\_CFG and XSPI\_DAC\_REMAPADDR0 / XSPI\_DAC\_REMAPADDR1 :

1. Write to the XSPI\_DAC\_CFG register to select the appropriate bank number in the attached memory device, to enable or disable AXI bus completer address remap, XIP mode, and other desired features.
2. Configure the remap address offset in the XSPI\_DAC\_REMAPADDR0 register.

## Write Limitations

By default, the xSPI controller does not support the following functions:

- Byte writes (for example, an AXI write with an AWSIZE value of 0) to a device that is 16-bit addressed, or to an octal DDR configured device
- Issuing a write burst to an odd address to a device that is 16-bit addressed, or to an octal DDR configured device
- Using AXI write strobes to mask bytes on the write bus

These write operations can optionally be supported by setting the bit XSPI\_DAC\_CFG.RWDS\_CAP\_EN bit if the connected device supports byte masking using the DQS signal. DQS is traditionally used by a memory device to transfer a strobe with read data to help read data sampling. However, some devices also use the same signal as a byte mask for writes (in which case the limitations can be avoided).

## AXI Wrapping Bursts

The following wrapping bursts restrictions are taken from the AXI standard:

- The start address must be aligned to the size of each transfer
- The length of the burst must be 2, 4, 8, or 16 transfers

Wrapping bursts are used for cache line accesses issued after a cache miss, and are, therefore, more relevant to the direct mode of operation than ACMD/STIG work modes. Internally, the xSPI controller handles AXI wrapping bursts by splitting them into separate incrementing bursts, automatically. Wrapping bursts on the AXI bus completer interface are supported even when the attached memory device does not support wrapping.

There is no restriction on using wrapping read bursts. However, there is one known scenario where wrapping write bursts received by the controller operating in direct mode through the AXI bus completer interface are not be supported. This scenario is described below:

1. The wrapping write burst restriction applies only to attached memory devices that require time to execute any program (write) operation. Flash devices, typically, operate in this manner.
2. While any program (write) operation is ongoing within the memory device, the controller has a function (which, by default, is enabled) to automatically check the status of the memory device. It does not allow any other read or write operation from the controller to proceed until the memory device has indicated that it is ready for a new operation. This function is programmable and can be disabled.
3. If the user has chosen to disable the function above (through the XSPI\_STAT\_SEQ\_CFG1 register), the splitting of the AXI wrapping write transaction into separate write incrementing transactions results in the second incrementing write being discarded by the memory device.

Therefore, wrapping write bursts cannot be supported when the attached memory device requires time to process write operations and the controller function to check ready/busy status of the device has been disabled (using the XSPI\_STAT\_SEQ\_CFG1 register). Otherwise, wrapping write bursts in direct mode are fully supported.

## Entering and Exiting XIP Mode

The XSPI\_DAC\_CFG.XIP\_EN\_MB\_VAL and XSPI\_DAC\_CFG.XIP\_DIS\_MB\_VAL bit fields control how and when XIP mode in the device is entered or exited when using direct mode. By default, XIP mode is disabled. The XIP Mode Register Configurations table shows the registers used to configure XIP mode operations.

Most memory device use either of the one or both of the following steps to enter the XIP mode.

- Configure a specific bit in the memory configuration/status register (this step is not mandatory for all flash devices, but it which can be achieved easily using STIG mode).
- Send a read command with the correct value applied in the mode byte; the info is located immediately after the address phase of the read transaction

Set the XSPI\_DAC\_CFG.XIP\_EN\_MB\_VAL bit to enter XIP mode while operating in direct mode. Follow up with a normal direct mode read transaction. As the read transaction is executed and the read sequence is sent to the device, the xSPI controller sends the value encoded in the XSPI\_XIP\_GCTL.XIP\_EN\_MB\_VALUE register bit, immediately following the address phase of the transaction. Once the read transaction completes, the XSPI\_XIP\_GCTL.XIP\_EN bit is set (=1) to indicate that both the controller and the memory device are now configured to work in XIP mode.

IMPORTANT: A direct read transfer to enable or disable XIP mode in the controller and memory device must not have more than 64B of data and must not cause crossing between pages.

To enable XIP mode during power on reset (assuming that the memory device is already configured to work in XIP mode in a non-volatile manner from reset), the XSPI\_XIP\_GCTL.XIP\_EN bit should be set (=1). Then, each read transaction is performed in XIP mode. For example, the controller issues the read operation without sending any opcode and additionally setting the enable XIP mode bit following the address phase (when

XSPI\_DAC\_CFG.XIP\_EN\_MB\_VAL = 1). The value of the XSPI\_DAC\_CFG.XIP\_EN\_MB\_VAL field is sent following the address phase of the transaction.

To exit XIP mode in both the xSPI controller and the memory device, a new direct read transaction must be issued to the device with the disable XIP mode bit following address phase. Set (=1) the XSPI\_DAC\_CFG.XIP\_DIS\_MB\_VAL bit to disable XIP mode. Issue a new direct read. As the read is executed and the read sequence is sent to the device, the controller sends the value encoded in the XSPI\_XIP\_GCTL.XIP\_DIS\_MB\_VALUE field immediately following the address phase of the transaction. Once completed, the XSPI\_XIP\_GCTL.XIP\_EN bit is cleared (=0) to indicate that the controller and memory device have exited XIP mode and are configured to work in normal mode.

Table 21-4: XIP Mode Register Configurations

|   XSPI_XIP_GCTL.XI P_EN |   XSPI_DAC_CFG.XIP _EN_MB_VAL |   XSPI_DAC_CFG.XIP _DIS_MB_VAL |   XSPI_READ_SEQ_CF G1.P1_MB_EN | Description                                                                                                                                         |
|-------------------------|-------------------------------|--------------------------------|--------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
|                       0 |                             0 |                              0 |                              0 | Normal read mode with- out mode byte. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT )                                               |
|                       0 |                             0 |                              1 |                              0 | Normal read mode with the additional disable mode byte following the address phase. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT ) |
|                       0 |                             0 |                              0 |                              1 | Normal read mode with the additional disable mode byte following the address phase (number of dummy cycles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT )    |

Table 21-4: XIP Mode Register Configurations (Continued)

|   XSPI_XIP_GCTL.XI P_EN |   XSPI_DAC_CFG.XIP _EN_MB_VAL | XSPI_DAC_CFG.XIP _DIS_MB_VAL   | XSPI_READ_SEQ_CF G1.P1_MB_EN   | Description                                                                                                                                                                                                                                          |
|-------------------------|-------------------------------|--------------------------------|--------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                       0 |                             1 | N/A                            | N/A                            | Enables XIP mode in the controller and memory device on the next read transfer - the enable mode byte is sent after the opcode and address phas- es. Then, XSPI_XIP_GCTL.XI P_EN =1. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT ) |
|                       1 |                             0 | 0                              | N/A                            | Controller reports DIR_CMD_ERR error. Mode byte must be sent when controller is con- figured to work in XIP mode.                                                                                                                                    |
|                       1 |                             0 | 1                              | N/A                            | Disables XIP mode in the controller and memory device on the next read transfer - the disable mode byte is sent after the address phase. Then, XSPI_XIP_GCTL.XI P_EN =0. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT )             |
|                       1 |                             1 | N/A                            | N/A                            | Controller works in XIP mode - the enable mode byte is sent after the ad- dress phase. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT )                                                                                               |

Table 21-4: XIP Mode Register Configurations (Continued)

|   XSPI_XIP_GCTL.XI P_EN |   XSPI_DAC_CFG.XIP _EN_MB_VAL |   XSPI_DAC_CFG.XIP _DIS_MB_VAL |   XSPI_READ_SEQ_CF G1.P1_MB_EN | Description                                                                                                                                         |
|-------------------------|-------------------------------|--------------------------------|--------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
|                       0 |                             0 |                              1 |                              1 | Normal read mode with the additional disable mode byte following the address phase. (Number of dummy cy- cles = XSPI_READ_SEQ_CF G1.P1_MB_DMY_CNT ) |

## Operating in ACMD Mode

The ACMD work mode relieves the host from issuing read and write commands directly. Instead, the host uses a dedicated engine to translate high-level commands into a series of low-level commands on the xSPI interface. It selects the optimal command sequence to send and to follow all transfer restrictions defined for the targeted memory device.

The auto command engine receives high-level commands either through a DMA descriptor format or directly through programmable registers. It is capable of operating multiple parallel threads that are configured using the controller registers and descriptors held in the main memory. It can operate in one of two modes - CDMA mode or PIO mode. Data transfers can be performed using either xSPI internal DMA mode or host DMA mode.

CDMA mode uses an embedded DMA to trigger various memory transactions using descriptors it fetches from main memory, automatically moving data between device memory and the host memory. In PIO work mode, dedicated controller registers are used in place of the descriptors to trigger the operation in the selected execution thread. The normal use case is to configure ACMD mode to use the internal DMA interface. The controller issues the relevant bus transactions directly to transfer data and to collect and process DMA descriptors. It is, however, possible to enable the ACMD engine to use the host DMA interface instead. In this case, the xSPI controller signals the host using an interrupt or status register when it requires the host to send write data on the host DMA write ports, or when it needs the host to issue a read command to fetch read data.

The auto command engine operates in the system (mACLK) clock domain. This mode uses system SRAM to buffer descriptor content locally; it is synchronous to mACLK.

NOTE: mACLK - This clock is SCLK0. It drives the front end of the controller. Both the AXI internal DMA and host DMA interfaces are synchronous to this clock.

The auto command engine can handle the following high-level sequences:

- Reset - sequence which allows the reset command to be sent to the selected device
- Erase - sequence to erase part of device or a whole device at once
- Program - sequence to transfer a specified amount of data from the host to the xSPI device
- Read - sequence to transfer a specified amount of data from the xSPI device to the Host

When operating in CDMA mode, the high-level sequence selection is made using a command descriptor. The DMA uses a standard structure where the head of the descriptor list is stored in APB register space. Each queued descriptor is chained together (one descriptor points to the next descriptor). For extra flexibility, up to eight descriptor chains can be managed simultaneously by the controller, and this is referred to as execution threads. Refer to section, Operating in ACMD Mode on for further details.

In PIO mode, the sequence is defined directly in specific command registers. Refer to section, Using the Auto Command in PIO Mode for further details.

NOTE: The high-level sequence the controller sends often must be broken up into multiple sub-sequences applied at the low-level xSPI interface. For example, it is common that prior to a program operation, a write enable sequence must be issued to instruct the memory device that a write operation is imminent, and, therefore, to configure the write enable latch of the device. This, obviously, depends on the attached memory type and controller configuration. The auto command engine selects the most appropriate low-level interface.

## Using the Auto Command in CDMA Mode

To enable the auto command engine in CDMA mode, configure XSPI\_WORKMODE\_CTL.WORKMODE = 3. This operation can be done when the controller is in an idle state (poll the XSPI\_GSTAT.CTL\_BUSY bit). If software tries to access the XSPI\_WORKMODE\_CTL register when the controller is in the busy state, the access is ignored. To trigger the auto command engine in CDMA work mode, the XSPI\_CMD0 , XSPI\_CMD2 and XSPI\_CMD3 registers must be configured to pass the descriptor chain start address and the thread number associated with the descriptor chain. The command format for these registers are shown in the following tables.

When the ACMD mode is enabled, each write to the XSPI\_CMD0 register starts execution on a new command. Therefore, the XSPI\_CMD0 register must be written last; a write access to this register triggers the execution of the programmed operation.

Table 21-5: XSPI\_CMD0 Format for ACMD mode

| XSPI_CMD0 (offset 0x0)   | XSPI_CMD0 (offset 0x0)   | XSPI_CMD0 (offset 0x0)   | XSPI_CMD0 (offset 0x0)   |
|--------------------------|--------------------------|--------------------------|--------------------------|
| 31:30                    | 29:27                    | 26:24                    | 23:0                     |
| CDMA/PIO =00             | Reserved                 | TRD_NUM (thread number)  | Reserved                 |

Table 21-6: XSPI\_CMD2 Format for ACMD mode

| XSPI_CMD2                                    |
|----------------------------------------------|
| (offset 0x2) [31:0]                          |
| Lower 32 bits of the descriptor head address |

Table 21-7: XSPI\_CMD3 Format for ACMD mode

| XSPI_CMD3 (offset 0x3) [31:0]                |
|----------------------------------------------|
| Upper 32 bits of the descriptor head address |

The host must write zero to the CDMA/PIO field of the XSPI\_CMD0 register and provide the address of the head of the linked descriptor list using the XSPI\_CMD3 and XSPI\_CMD2 registers.

The controller issues a bus transaction to fetch the complete descriptor from main memory. The Command Descriptor Structure and Command Descriptor Layout tables present the details of the descriptor format.

Table 21-8: Command Descriptor Structure

|   No | 63:48                 | 47:32                 | 31:16                 | 15:0                  |
|------|-----------------------|-----------------------|-----------------------|-----------------------|
|    0 | Next pointer          | Next pointer          | Next pointer          | Next pointer          |
|    1 | System memory pointer | System memory pointer | System memory pointer | System memory pointer |
|    2 | xSPI pointer          | xSPI pointer          | xSPI pointer          | xSPI pointer          |
|    3 | Reserved              | Reserved              | Reserved              | Reserved              |
|    4 | Command counter       | Command counter       | Command flags         | Command type          |
|    5 | Reserved              | Reserved              | Status                | Status                |

Table 21-9: Command Descriptor Layout

| Field                 | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Next pointer          | Next descriptor address. Address needs to follow system bus restric- tions.                                                                                                                                                                                                                                                                                                                                                                                                           |
| System memory pointer | System memory address required for data DMAcommands. For read operations, this field is the address where the start of the read data is placed. For write operations, this field is the location of the start of the write data.                                                                                                                                                                                                                                                      |
| xSPI pointer          | The field indicates the xSPI address for sequences that require ad- dressing of the xSPI address space.                                                                                                                                                                                                                                                                                                                                                                               |
| Command type          | This field identifies the operation that the controller must perform. The encoding is: • 0x1000 - Erase the number of sequential sectors specified in the command counter field • 0x1001 - Full chip erase • 0x1100 - Device reset • 0x1101 - Device JEDEC reset • 0x2100 - Command to program the number of data bytes specified in the command counter field • 0x2200 - Command to read the number of data bytes specified in the command counter field • Other values are reserved |
| Command flags         | This field contains different control flags for the operation of the command (See the Valid Command Flag table).                                                                                                                                                                                                                                                                                                                                                                      |

Table 21-9: Command Descriptor Layout (Continued)

| Field           | Description                                                                                                                                                                                                                                                                                 |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Command counter | For read/program operations, the field indicates the number of data bytes to transfer minus one. For example, to transfer 1024 bytes, the value encoded is 1023. For an erase operation, this field represents the number of sectors to erase. For other operations, this field is ignored. |
| Status          | The controller updates this field with the command status once the command operation is complete, and writes the status back to the descriptor. Refer to the Table 21-11 Descriptor Status Field table for bit descriptions.                                                                |

On receipt of a complete descriptor, the controller constructs a low-level command or a series of low-level commands.

NOTE: If XIP mode is enabled in the controller (that is, XSPI\_XIP\_GCTL.XIP\_EN = 1, which is supported by some devices to improve read performance), the connected device can only be read. Therefore, any other command will fail. As such, invoking the CDMA descriptor with a non-read command type results in the controller ignoring it and raising a DSC\_ERROR fail status.

The Valid Command Flag table describes the allowed values in the command flags field of the descriptor.

Table 21-10: Valid Command Flag

| Bits   | Name     | Description                                                                                                                                                                                                                                                                                                                                                 |
|--------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:11  | Reserved | Reserved                                                                                                                                                                                                                                                                                                                                                    |
| 10     | DMA_SEL  | This bit indicates which DMAinterface is used to transfer data for read or program op- erations. If the selected command does not require a data transfer, this bit is ignored. 0: Selects the host DMAinterface 1 : Selects the internal DMAinterface Refer to section,Selecting the DMAInter- face in ACMD mode for details on how to configure this bit. |
| 9      | CONT     | Set (=1) the bit to indicate that another valid descriptor after this has been queued and is ready for the controller to fetch and process. Clear (=0) the bit when this descriptor is last in the descriptor chain.                                                                                                                                        |
| 8      | INT      | Set (=1) the bit to instruct the con- troller to issue an interrupt after the completion of descriptor processing. The triggered interrupt is identified in the                                                                                                                                                                                             |

Table 21-10: Valid Command Flag (Continued)

|   Bits | Name       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|        |            | XSPI_TRD_COMP_ISTAT register. The bit of the interrupt status register that is set (=1) maps to the thread of the descriptor (as indicated originally in the descriptor by the TRD[n] field).                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|      7 | MB_XIP_DIS | When XIP mode is enabled in the controller and the attached memory device, setting this bit (=1) indicates to the controller that it must send mode bits to disable and exit XIP mode. When set, the controller sends mode bits as specified in the XSPI_XIP_GCTL.XIP_DIS_MB_VAL UE field. Additionally, the XSPI_XIP_GCTL.XIP_EN_MB_VALU E bit is cleared to disable XIP mode in the controller. This bit is valid only for read commands. If XSPI_READ_SEQ_CFG1.P1_MB_EN =1 and XSPI_XIP_GCTL.XIP_EN = 0 the controller ignores the value of this field and it is treated as set (=1) (for example, mode bits are sent regardless of the value of this field). |
|      6 | MB_XIP_EN  | This bit triggers entry to XIP mode. Setting this bit indicates to the controller that it must send mode bits to the device to enable XIP mode. The mode bits that are sent de- pend on the XSPI_XIP_GCTL.XIP_EN_MB_VALU E field. Additionally, the XSPI_XIP_GCTL.XIP_EN bit is set to enable XIP mode in the controller. This bit is valid only for read commands.                                                                                                                                                                                                                                                                                              |

Table 21-10: Valid Command Flag (Continued)

| Bits   | Name          | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5      | xSPI_PTR_CONT | If this bit is set, the xSPI pointer field of current descriptor is ignored, and command engine uses the existing xSPI pointer value. Essentially, setting this bit allows the host to continue a data transfer from the address on the flash interface where a previously com- pleted descriptor finished. This feature can only be triggered if a pre- vious descriptor in the chain had this bit cleared and the xSPI pointer was initialized. It must also only be used for a command of the same type (for example, a series of read descriptors or series of write descriptors). For SPI NAND devices, a transfer is always continued from the beginning of the next page even if the current page was not filled completely (column address = 0, row address incremented). |
| 4      | SYS_PTR_CONT  | If this bit is set (=1), the system memory pointer field of the current descriptor is ig- nored. The command engine uses the exist- ing memory pointer value. This bit allows the data transfer to continue on the host interface where the previously completed de- scriptor finished. This feature can only be triggered if a pre- vious descriptor in the chain had this bit cleared and the system memory pointer was initialized. It must also only be used for commands of the same type (for example, a series of read descriptors or series of write descriptors).                                                                                                                                                                                                        |
| 3      | Reserved      | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 2:0    | BANK          | The field indicates the device number (bank/CS) that is selected for sequence exe- cution.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

NOTE: If the INT bit is not set in the current descriptor and the following descriptor chain is dropped from execution due to an error, the controller does not set the TRD[n] interrupt flag in the XSPI\_TRD\_COMP\_ISTAT register. The error condition is, however, always reported by setting the TRD[n] bit in the XSPI\_TRD\_ERR\_ISTAT register. The bit can be used to detect that the descriptor chain execution was interrupted due to an error. The n in the field names represent the thread number for which the error or complete condition occurred.

Opon completion of a command, the controller writes back the status into the descriptor status field. This field value is ignored during descriptor read operation. The status word is described in the Descriptor Status Field table.

Table 21-11: Descriptor Status Field

| Bits   | Name           | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24  | Reserved       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 23:16  | ECC_STAT       | Value of the ECC status returned by the SPI NAND device. The value of this field applies to last read operation. It is updated only when fail checking is enabled.                                                                                                                                                                                                                                                                                                                                                                            |
| 15     | COMPLETE       | When set, the bit denotes that the controller has updated status information and the op- eration is complete. This bit is set even when the operation ended as a failure. This bit should be in a cleared state while descriptor is constructed.                                                                                                                                                                                                                                                                                              |
| 14     | FAIL           | When set, the bit denotes that the operation failed to complete successfully.                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 13:6   | Reserved       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 5      | ECC_CORR_ERROR | The bit indicates that an ECC correctable error was detected during page read. This bit is valid only for SPI NAND devices when XSPI_STAT_SEQ_CFG10.ECC_FAIL _EN =1.                                                                                                                                                                                                                                                                                                                                                                          |
| 4      | DEVICE_ERROR   | The bit indicates that a device error was detected during a read status operation for erase/program/read.                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 3      | DQS_ERROR      | This bit is set when an incorrect number of DQS pulses were detected during a direct mode read or status check. Essentially, this status is passed from the soft PHY and could mean: The PHY is configured badly such that it ex- pects DQS strobes but never received them, or, The value ( XSPI_PHY_GATE_LPBK_CTL.RD_DE L_SEL field ) is incorrect and has caused data corruption and pointer misalignment in the PHY. To resolve this issue, the PHY must be reset to clear the DQS_OVF and DQS_UR flags and reset the read data pointers. |

Table 21-11: Descriptor Status Field (Continued)

|   Bits | Name      | Description                                                                                         |
|--------|-----------|-----------------------------------------------------------------------------------------------------|
|      2 | CRC_ERROR | When set, the bit indicates that the control- ler detected a CRC error.                             |
|      1 | BUS_ERROR | When set, the bit indicates that the con- troller got an error response on the system DMAinterface. |
|      0 | DSC_ERROR | This bit indicates that an invalid descriptor sequence has been detected.                           |

The auto command engine accept new commands only when the thread selected is in am idle state. The host software should check the thread status before it queues any new commands. The threads status is available in the XSPI\_TRD\_STAT register. Commands sent to a busy thread are ignored and XSPI\_ISTAT.CMD\_IGNORED = 1.

## Using the Auto Command in PIO Mode

In PIO work mode, dedicated command registers are used to trigger operation in a selected execution thread of the auto command engine. In this mode it is only possible to trigger a single command in each of the execution threads. This mode is useful when host software wants to trigger a single operation. In this case it is much simpler and faster to program a few controller registers than prepare a descriptor list in system memory. Functionality of this work mode is similar to the CDMA work mode for cases where the descriptors chain contains only a single descriptor. The auto command engine executes the programmed command and returns a status word separately for each execution thread. Status words are mapped into controller status registers, where it remains valid until a new command is triggered on this thread.

Depending on the selected operation, the number of XSPI\_CMD* required to be programmed. The registers described as unused for given operation do not need to be set. XSPI\_CMD1 to XSPI\_CMD5 registers retain the contents from previous writes meaning that if the current register configuration matches what is needed for the next operation, it can be left unchanged. The only exception is the XSPI\_CMD0 register. This register must always be written on each operation. Write access to this register triggers operation execution. Host software needs to initialize all required command registers before it writes to command register 0.

- The reset and chip erase sequences only requires XSPI\_CMD0 to be written to. XSPI\_CMD1 through XSPI\_CMD5 registers are unused.
- The sector erase sequence requires XSPI\_CMD0 , XSPI\_CMD1 , XSPI\_CMD4 and XSPI\_CMD5 to be written, XSPI\_CMD2 and XSPI\_CMD3 are unused.
- The program and read sequences require all XSPI\_CMD* registers to be written.

The XSPI\_CMD0 -XSPI\_CMD5 tables below show the layout for the available high-level sequences that may be triggered in PIO mode. The operation is always triggered when XSPI\_CMD0 is written to, so configure XSPI\_CMD1 -XSPI\_CMD5 registers first.

Table 21-12: XSPI\_CMD0 Layout for PIO Sequences

| Bits   | Name       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:30  | CDMA/PIO   | Set (=1) for PIO work mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 29:27  | -          | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 26:24  | TRD_NUM    | This field selects the destination thread number for command. Software can select any available thread. Commands can be is- sued in parallel to all threads.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 23     | -          | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 22:20  | BANK/CS    | Selects device targeted by the sequence                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 19     | DMA_SEL    | This bit selects which DMAinterface should be used for transferring data. It is only rele- vant for commands that transfer data (for example, read or program operations). Clear (=0) to use the host DMAinterface. Set (=1) to use the internal DMAinterface.                                                                                                                                                                                                                                                                                                                                                                                                          |
| 18     | INT        | If this bit is set, an interrupt is issued after the operation has finished. The triggered interrupt is indicated in the XSPI_TRD_COMP_ISTAT register, where interrupt bit is selected by the thread num- ber in the TRD[n] field.                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 17     | MB_XIP_DIS | When XIP mode is enabled in the controller and the attached memory device, setting this bit indicates to the controller that it must send mode bits to disable and exit XIP mode. When set, the controller sends mode bits as specified in the XSPI_XIP_GCTL.XIP_DIS_MB_VAL UE field . Additionally, the XSPI_XIP_GCTL.XIP_EN_MB_VALU E bit is cleared to disable XIP mode in the controller. This bit is valid only for read operations. If XSPI_READ_SEQ_CFG1.P1_MB_EN = 1 and XSPI_XIP_GCTL.XIP_EN_MB_VALU E = 0 the controller ignores the value of this field. It is treated as set (= 1) (for example, mode bits are sent regardless of the value of this field). |

Table 21-12: XSPI\_CMD0 Layout for PIO Sequences (Continued)

| Bits   | Name      | Description                                                                                                                                                                                                                                                                                                                                                             |
|--------|-----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16     | MB_XIP_EN | This bit triggers entry to XIP mode. Setting this bit indicates to the controller that it must send mode bits to the device to enable XIP mode. The mode bits sent depends on the XSPI_XIP_GCTL.XIP_EN_MB_VALU E field. Additionally, the XSPI_XIP_GCTL.XIP_EN_MB_VALU E bit is set to enable XIP mode in the con- troller. This bit is valid only for read operations. |
| 15:0   | CMD_TYPE  | This field identifies the kind of operation that is executed by the con- troller. It is con- figured as follows: 0x1100 : Reset sequence (soft reset) 0x1101 : Reset sequence (JEDEC reset) 0x1000 : Sector erase sequence 0x1001 : Chip erase sequence 0x2100 : Program sequence 0x2200 : Read sequence                                                                |

The XSPI\_CMD1 register is required for all operations other than reset and chip erase. It holds the lower half of the memory device starting address where the operation takes place.

Table 21-13: XSPI\_CMD1 Layout for PIO Sequences

| Bits   | Name                | Description                                                                                     |
|--------|---------------------|-------------------------------------------------------------------------------------------------|
| 31:0   | xSPIAddress (Lower) | This field contains the lower half of the starting address of the operation in the xSPI device. |

The XSPI\_CMD2 is only required for read and program operations. It holds the lower half of the host side address where the operation fetches data for writes, or places data for reads.

Table 21-14: XSPI\_CMD2 Layout for PIO Sequences

| Bits   | Name           | Description                                                                     |
|--------|----------------|---------------------------------------------------------------------------------|
| 31:0   | SYS_ADDR_PTR_L | Host memory address required forDMA transfers. It is lower half of the address. |

The XSPI\_CMD3 is only required for read and program operations. It holds the upper half of the host side address where the operation should fetch data for writes, or place data for reads.

Table 21-15: XSPI\_CMD3 Layout for PIO Sequences

| Bits   | Name           | Description                                                                     |
|--------|----------------|---------------------------------------------------------------------------------|
| 31:0   | SYS_ADDR_PTR_H | Host memory address required forDMA transfers. It is upper half of the address. |

The XSPI\_CMD4 is required for sector erase operations as well as read and program operations. For sector erase, it holds the number of sectors that must be erased. For read/program operations, it holds the number of data bytes that should be transferred.

Table 21-16: XSPI\_CMD4 Layout for PIO SECTOR ERASE Sequences

| Bits   | Name     | Description                                                                                                                                                                                         |
|--------|----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0   | SECT_CNT | Sector counter. Determines how many sec- tors shouldbe erased. Number of sectors is decoded as a value of this field plus 1 (that is, to erase 1 sector this field should have value 0, and so on). |

Table 21-17: XSPI\_CMD4 Layout for PIO READ/PROGRAM Sequences

| Bits   | Name     | Description                                                                                                                                                                |
|--------|----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0   | DATA_CNT | Data counter. Determines how many data bytes should be transferred minus 1. For ex- ample, if 1024 bytes should be transferred, the value to be programmed should be 1023. |

The XSPI\_CMD5 is required for all operations other than reset and chip erase. It holds the upper half of the memory device starting address where the operation should take place.

Table 21-18: XSPI\_CMD5 Layout for PIO Sequences

| Bits   | Name                 | Description                                                                                                                                                                                                                                                    |
|--------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0   | xSPI Address (Upper) | This field contains additional address space for the xSPI address. It is utilized for SPI NAND devices to hold both the row and column address as a single xSPI address. For sector erase operations, bits that belong to column address must be cleared (=0). |

## Status Checking

The host can check the operating status of each active thread. After the thread ID is written to the XSPI\_CMD\_STAT\_PTR register, the status can be read from the XSPI\_CMD\_STAT register. The COMPLETE bit is cleared when a new operation is triggered, and all other fields within the XSPI\_CMD\_STAT register should at this point be ignored. When the selected operation is complete, the controller sets the COMPLETE bit and all other register fields reflects the status of that operation.

The Status Checking table describes the status fields.

Table 21-19: Status Checking

| Bits   | Name           | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24  | Reserved       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 23:16  | ECC_STAT       | Value of the ECC status returned by the SPI NAND device. Value of this field applies to last read operation and is updated only when fail checking is enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 15     | COMPLETE       | When set, denotes that the controller has updated status information and the opera- tion is complete. This bit shall be set even if the operation ended as a failure.                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 14     | FAIL           | When set, denotes that operation failed to complete successfully.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 13:6   | Reserved       | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 5      | ECC_CORR_ERROR | ECC correctable error was detected during page read. This field is valid only for SPI NAND devices when the XSPI_STAT_SEQ_CFG10.ECC_FAIL _EN =1.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4      | DEVICE_ERROR   | Device error was detected during read status operation for erase/program/read operation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 3      | DQS_ERROR      | Incorrect DQS pulses number was detected during the data read operation. This is infor- mation from the PHY that either the DQS strobe did not appear during read (for exam- ple, device is not connected to the control- ler) or RD_DEL_SEL signal value is incor- rect ( XSPI_PHY_GATE_LPBK_CTL.RD_DE L_SEL ) and data read from the flash device are corrupted and read FIFO pointers in the PHY are misaligned. The DLL_RST signals to clear DQS_OVF and DQS_UR flags in the PHY and clear all PHY read data pointers. One of these is required by the PHY before continuing to work after the DQS_ERROR signal asser- tion. |
| 2      | CRC_ERROR      | When set, it indicates that the controller de- tected a CRC error.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 21-19: Status Checking (Continued)

|   Bits | Name      | Description                                                                                    |
|--------|-----------|------------------------------------------------------------------------------------------------|
|      1 | BUS_ERROR | When set, it indicates that the controller got an error response on the system DMAinter- face. |
|      0 | CMD_ERROR | This bit indicates that an invalid command sequence has been detected.                         |

## Selecting the DMA Interface

ACMD mode can be configured to use an internal DMA interface to have the controller issue the relevant bus transactions to transfer data and to collect and process DMA descriptors. This mode requires the least amount of software overhead. It is, however, also possible to enable the ACMD engine to use the host DMA interface instead.

- To use the ACMD in CDMA mode with data transfers passing via the host DMA interface, follow the process as described in the section, Using the Auto Command in CDMA Mode, clearing the DMA\_SEL bit in the command flags.
- To use the ACMD in CDMA mode with data transfers passing via the internal DMA interface, follow the process as described in the section, Using the Auto Command in CDMA Mode, setting the DMA\_SEL bit in the command flags.
- To use the ACMD in PIO mode with data transfers passing via the host DMA interface, follow the process as described in the section, Using the Auto Command in PIO Mode, clearing the DMA\_SEL bit of the XSPI\_CMD0 register.
- To use the ACMD in PIO mode with data transfers passing via the internal DMA interface, follow the process as described in the section, Using the Auto Command in PIO Mode, setting the DMA\_SEL bit of the XSPI\_CMD0 register.

When transfers using the bus completer interface are enabled, the following sequence also needs to be followed by the external host to transfer the required data:

1. Poll the XSPI\_ISTAT.SDMA\_TRIGG bit. Or, if an interrupt is preferable, unmask the interrupt associated with SDMA\_TRIGG by setting the XSPI\_INT\_EN.SDMA\_TRIGG\_EN bit. When the bit is asserted, the host can proceed.
2. The external host reads the XSPI\_SDMA\_SIZ register, the XSPI\_SDMA\_TRD\_STAT register and the XSPI\_SDMA\_ADDR0 / XSPI\_SDMA\_ADDR1 registers. The XSPI\_SDMA\_SIZ register reports the size of the data transfer that must be transferred. The XSPI\_SDMA\_TRD\_STAT register reports the thread ID of the operation that triggered SDMA data transfer and also reports the data transfer direction. The XSPI\_SDMA\_ADDR0 / XSPI\_SDMA\_ADDR1 registers indicate the address (awaddr/araddr) the external host must use on the write or read transaction when it issues it.
3. Before starting data transmission, the external host must clear the XSPI\_ISTAT.SDMA\_TRIGG register by writing 1.

4. Host can now transfer data. For program operations, it is expected that the host will issue one or more write requests with a total size matching the data transfer indicated in step 2. For read operations, it is expected that the host will issue one or more read requests with a total size matching the data transfer indicated in step 2. The address issued by the host should match that being reported in the XSPI\_SDMA\_ADDR0 / XSPI\_SDMA\_ADDR1 registers.

If the host ignores any of the requirements and starts the data transfer when the host DMA is not ready, the XSPI\_ISTAT.SDMA\_ERR flag is set. If the XSPI\_INT\_EN.SDMA\_ERR\_EN bit is set, an interrupt is also triggered. When the XSPI\_DMA\_CTL.SDMA\_ERR\_RSP bit is set, an error response is returned; if it is cleared, the OK response is returned.

If the host sends an unsupported transaction to a bus completer interface, the host DMA ignores this access and the XSPI\_ISTAT.SDMA\_ERR flag is set. If an error response is detected on the system bus during host DMA transfer, the fail and bus error bits are set in the last operation status. The last operation status can check in:

- The XSPI\_CMD\_STAT register. After selecting thread in which command was executed (for ACMD PIO work mode). Thread is selected by value of the XSPI\_CMD\_STAT\_PTR register. This method is valid for the ACMD PIO and STIG work modes.
- The status field of the command descriptor. This method is valid for the CDMA work mode.

Performing transactions in wrong direction (that is, the host issuing a write transaction to the bus completer port in case of sending read operation command to an xSPI device) is forbidden and causes the XSPI\_ISTAT.SDMA\_ERR flag to be set. If the XSPI\_INT\_EN.SDMA\_ERR\_EN bit is set, an interrupt is also triggered.

## Rules and Limitations

The host must follow the following rules when using ACMD mode:

- For SPI NAND devices, the initial column address must be specified within the range of the page size.
- For profile 2 or profile 1, octal DDR devices, the number of bytes to be transferred must be even.
- For profile 2 or profile 1, octal DDR devices, the start address of the transfer in the xSPI device must be even.
- For profile 1 and octal DDR devices, the command extension must be enabled.
- For DATA\_PER\_ADDR (when the device is 16-bit addressed), the start address must be even and the number of bytes to transfer must be even.
- If XIP mode is enabled, only read operations are permitted.
- Only the supported command types as illustrated in this section should be attempted.

Violating any of the above rules results in the controller returning the DSC\_ERROR/CMD\_ERROR status.

- NOTE: DSC\_ERROR and CMD\_ERROR status bits are only intended to ease debug and integration and should never be triggered for correct controller operation. Using the controller outside of the descriptor and command rules may lead to undefined behavior.

In automatic work modes, the controller writes/reads data consecutively from the specified address. If a page boundary is crossed, the transaction continues from the beginning of the next page.

## Operating in STIG Mode

The Software-Triggered Instruction Generator (STIG) mode allows software to define the low-level commands and sequences sent to the memory device. Relative to direct or ACMD mode, it is more flexible because software can control the low-level characteristics of each operation carried out on the device. STIG mode is normally used with the direct or ACMD modes of operation.

To enable the STIG work mode, the XSPI\_WORKMODE\_CTL.WORKMODE field must be set (= 1) . This can be done only when the controller is in an idle state. The STIG Mode Registers table shows the registers used to configure STIG mode operations.

Table 21-20: STIG Mode Registers

| Bits   | Name      | Description                                                                                                        |
|--------|-----------|--------------------------------------------------------------------------------------------------------------------|
| 31:0   | XSPI_CMD0 | A write to this register triggers the STIG op- eration and has no other purpose. Reading this register is ignored. |
| 31:0   | XSPI_CMD1 | Holds bits [31:0] of the STIG instruction                                                                          |
| 31:0   | XSPI_CMD2 | Holds bits [63:32] of the STIG instruction                                                                         |
| 31:0   | XSPI_CMD3 | Holds bits [95:64] of the STIG instruction                                                                         |
| 31:0   | XSPI_CMD4 | Holds bits [127:96] of the STIG instruction                                                                        |

## Instruction Variants

The main instruction variants for STIG mode include the generic command instruction, legacy HyperFlash or profile 2 instructions, and legacy SPI xSPI profile 1 or SPI NAND instructions. The following tables show the structure of the STIG command instruction ( XSPI\_CMDn ) for each variant.

Table 21-21: STIG Command Instruction - Legacy SPI, xSPI Profile 1 or SPI NAND

| Name                                           | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                          |
|------------------------------------------------|---------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Reserved                                       | 127:126 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                  |
| t CMS /t CEM Enable ( XSPI_SEQ_GCTL0.TCMS_EN ) | 125     | Setting this bit enables the feature that automatically forces the controller to pause activity to the memory device to al- low it to perform a periodic refresh operation. Some volatile memory devices require this functionality (for example, Hy- perRAM). This functionality is referred to as t CMS timing. When set, the controller pauses activity as specified in the XSPI_MINICTL_DEV_ACTIVE_MAX register. |

Table 21-21: STIG Command Instruction - Legacy SPI, xSPI Profile 1 or SPI NAND (Continued)

| Name                                 | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------------------------|---------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Instruction Termination (INSTR_LINK) | 124     | This bit is only relevant when the instruction type field of the instruction = 0, 32, 36 or 96. See the section Instruction Type Variants for details on instruction type configuration. The field assigned to this bit indicates whether the instruction is to be terminated (CS is deactivated after the instruction finishes) or the instruction is glued with the next instruction (providing it is ready to be fetched from the command FIFO). 0 - Current sequence is terminated and not glued with another instruction of the same type 1 - Current sequence is glued with the next STIG instruction                             |
| Reserved                             | 123:119 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| CRC Variant (CRC_VARIANT)            | 118     | Determines when the CRC is calculated on an outbound com- mand. When cleared (=0), calculate for the address bytes only. When set (=1), calculate for all bytes in the sequence.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| CRC Enable (CRC_EN)                  | 117     | Set to enable dynamic CRC calculation and output the value on xSPI flash interface in the transfer modes as specified by ADDR_EDGE_MODE and ADDR_NO_OF_IOS. If the oc- tal DDR (8D) variant is configured, the calculated CRC byte is maintained for a full flash clock cycle.                                                                                                                                                                                                                                                                                                                                                          |
| XIP Enable (XIP_EN)                  | 116     | Setting this bit instructs the controller to block the sending of the opcode, which is needed when the device is operating in XIP mode 0 - Opcode transfer phase is sent 1 - Opcode transfer phase is not sent (XIP enabled)                                                                                                                                                                                                                                                                                                                                                                                                            |
| Address Shift (ADDR_SHIFT)           | 115     | When the ADDR_EDGE_MODE and ADDR_NO_OF_IOS configuration determines the octal DDR variant, two bytes are required to be sent in a single flash clock cycle. Therefore, the host must configure an even number of bytes in the quoted case. However, there are specific exceptions such as the Adesto octal DTR SFDP command which requires a {A2,A1,A0,do not care} flash interface address alignment rather than a common {A3,A2,A1,A0} alignment. The ADDR_SHIFT bit shifts the address byte chain by one byte towards the left and fills the least - significant byte with zeroes (when an octal DDR address structure is required). |
| Reserved                             | 114:111 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| Bank Number (BANK_NUM)               | 110:108 | This field informs the controller on which memory device the sequences should be executed. This information directly maps to the chip select (CS) pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 21-21: STIG Command Instruction - Legacy SPI, xSPI Profile 1 or SPI NAND (Continued)

| Name                                    | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|-----------------------------------------|---------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Opcode Edge Mode (OPCODE_EDGE_MODE)     | 107     | This bit selects SDR or DDR operation at the memory device 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Reserved                                | 106     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Opcode IO Mode (OPCODE_NO_OF_IOS)       | 105:104 | Determines number of active IOs for opcode phase of the instruction 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                                                                                                                                                                                                                                                 |
| Reserved                                | 103:101 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Opcode Extension Enable (OPCODE_EXT_EN) | 100     | Enables dual byte opcode for the instruction 0 - Only CMD[87:80] is transmitted in the opcode phase of the instruction. 1 - CMD[87:80] followed by CMD_EXT[79:72] is transmit- ted in the opcode phase of the instruction. If the OPCODE_EDGE_MODE and OP- CODE_NO_OF_IOS configuration determines the octal DDR variant, two bytes are required to be sent in a single flash clock cycle. The host must also set OPCODE_EXT_EN =1. Failure to do this results in an error being reported. |
| Address Edge Mode (ADDR_EDGE_MODE)      | 99      | Determines edge mode for the address phase of the instruction. 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                             |
| Reserved                                | 98      | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Address IO Mode (ADDR_NO_OF_IOS)        | 97:96   | Determines the number of active IOs for the address phase of the instruction 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                                                                                                                                                                                                                                        |
| Reserved                                | 95      | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 21-21: STIG Command Instruction - Legacy SPI, xSPI Profile 1 or SPI NAND (Continued)

| Name                                                                               | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|------------------------------------------------------------------------------------|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Address Number of bytes (ADDR_NO_OF_BYTES)                                         | 94:92  | Determines the number of address bytes to be sent in the in- struction. Up to six address bytes can be sent in one sequence. A value of 7 encoded here is not permitted. A value of 0 encoded implies that no address bytes are sent (for example, address phase is disabled). If ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration determines the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. The host must configure an even number of address. If the host configures ADDR_NO_OF_BYTES = 1 or 3 or 5, the controller generates an error.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Reserved                                                                           | 91:90  | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| Number of data bytes (DATA_NO_OF_BYTES) or Number of mode bytes (MODE_NO_OF_BYTES) | 89:88  | For cases where the number of data (or mode) bytes is ≤ 2, this field allows that data to be included within this instruction, directly avoiding the need to glue a separate data sequence instruction. If there are more than two data bytes required to be transferred, the instruction must be glued with a data sequence. Note that if up to two data (or mode) bytes are transferred directly from this instruction, the format in which they are pre- sented to the device depends on the ADDR_EDGE_MODE and ADDR_NO_OF_IOS configuration. If the host needs the data to be sent in a different format, this field must be cleared (=0). Instead, transfer the data using a glued data sequence instruction, or generic instruction where the format of those data bytes can be individually configured. 0 - zero data/mode bytes are sent to device (data phase disa- bled) 1 - One byte is transferred to the device 2 - Two bytes are transferred to the device 3 - N/A If ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration determines the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. The host must configure an even number of bytes. Failure to do this results in an error being reported. |
| CMD                                                                                | 87:80  | Opcode mnemonic                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| CMD_EXT                                                                            | 79:72  | Opcode extension mnemonic                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| ADDR5                                                                              | 71:64  | Byte 5 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ADDR4                                                                              | 63:56  | Byte 4 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 21-21: STIG Command Instruction - Legacy SPI, xSPI Profile 1 or SPI NAND (Continued)

| Name             | Bits   | Description                                                                                                                                                                     |
|------------------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ADDR3            | 55:48  | Byte 3 of the address phase                                                                                                                                                     |
| ADDR2            | 47:40  | Byte 2 of the address phase                                                                                                                                                     |
| ADDR1            | 39:32  | Byte 1 of the address phase                                                                                                                                                     |
| ADDR0            | 31:24  | Byte 0 of the address phase                                                                                                                                                     |
| DATA1/MODE1      | 23:16  | Byte 1 of the data/mode phase                                                                                                                                                   |
| DATA0/MODE0      | 15:8   | Byte 0 of the data/mode phase                                                                                                                                                   |
| Reserved         | 7      | N/A                                                                                                                                                                             |
| Instruction Type | 6:0    | This field defines the type (index) of the instruction. A descrip- tion of each value and its corresponding instruction can be found in the section, Instruction Type Variants. |

Table 21-22: STIG Instruction - Legacy HyperFlash or xSPI Profile 2

| Name                                 | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------------------------|---------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Reserved                             | 127:126 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| t CMS /t CEM Enable (TCMS_EN)        | 125     | Setting this bit enables the feature that automatically forces the controller to pause activity to the memory device to allow it to perform a periodic refresh operation. Some volatile memory devices require this functionality (for ex- ample, HyperRAM) and is referred to as t CMS timing. When set, the controller pauses activity as specified in the XSPI_MINICTL_DEV_ACTIVE_MAX register.                                                                                                                                                                                                                           |
| Instruction Termination (INSTR_LINK) | 124     | This bit is only relevant when the instruction_type field of the instruction = 0, 32, 36 or 96. Refer to the Instruction Type Variants section for details on instruction type configuration. The field assigned to this bit indicates whether the instruction is to be terminated (CS is to be deactivated after the instruction finishes) or the instruction is to be glued with the next instruc- tion providing it is ready to be fetched from the command FIFO. 0 - Current sequence is terminated and not glued with another instruction of the same type 1 - Current sequence is glued with the next STIG instruction |
| Reserved                             | 123:119 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| CRC Variant (CRC_VARIANT)            | 118     | When the CRC is calculated on the outbound command, this bit indicates when cleared (=0) to calculate for the address bytes only, or when set (=1) to calculate for all bytes in the sequence.                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 21-22: STIG Instruction - Legacy HyperFlash or xSPI Profile 2 (Continued)

| Name                                | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|-------------------------------------|---------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CRC Enable (CRC_EN)                 | 117     | Set to enable dynamic CRC calculation and output this val- ue on xSPI flash interface in the transfer modes specified by ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration. If an octal DDR (8D) variant is configured, the calcu- lated CRC byte is maintained for full flash clock cycle.                                                                                                                                                                                                                                                                                                                               |
| XIP Enable (XIP_EN)                 | 116     | Setting this bit instructs the controller to block the sending of the opcode, which is needed when the device is operating in XIP mode. 0 - Opcode transfer phase is sent 1 - Opcode transfer phase is not sent (XIP enabled)                                                                                                                                                                                                                                                                                                                                                                                             |
| Address Shift (ADDR_SHIFT)          | 115     | If ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration determine the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. The host must configure an even number of bytes in the quoted case. However, there are specific exceptions. For example, the Ades- to octal DTR SFDP command requires a {A2,A1,A0,do not care} flash interface address alignment rather than the common {A3,A2,A1,A0} alignment. The ADDR_SHIFT bit shifts the address byte chain by one byte towards the left and fills the least - significant byte with zeroes providing the octal DDR ad- dress is required. |
| Reserved                            | 114:111 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Bank Number (BANK_NUM)              | 110:108 | This field informs the controller on which memory device the sequence should be executed. This information directly maps to the chip select (CS) pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| Opcode Edge Mode (OPCODE_EDGE_MODE) | 107     | This bit selects SDR or DDR operation at the memory device 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| Reserved                            | 106     | NA                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Opcode IO Mode (OPCODE_NO_OF_IOS)   | 105:104 | Indicates the number of active IOs for opcode phase of the instruction 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| Reserved                            | 103:101 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 21-22: STIG Instruction - Legacy HyperFlash or xSPI Profile 2 (Continued)

| Name                                       | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------------------------------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Opcode Extension Enable (OPCODE_EXT_EN)    | 100    | Enables dual byte opcode for the instruction 0 - Only CMD[87:80] is transmitted in the opcode phase of the instruction 1 - CMD[87:80] followed by CMD_EXT[79:72] is transmit- ted in the opcode phase of the instruction If OPCODE_EDGE_MODE and OPCODE_NO_OF_IOS configuration determines the octal DDR variant, two bytes are required to be sent in a single flash clock cycle. It is therefore required that the host also sets OPCODE_EXT_EN. Failure to do this results in an error being reported.                                                                                                     |
| Address Edge Mode (ADDR_EDGE_MODE)         | 99     | Determines edge mode for address phase of the instruction 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| Reserved                                   | 98     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Address IO Mode (ADDR_NO_OF_IOS)           | 97:96  | Determines number of active IOs for the address phase of the instruction 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| Reserved                                   | 95     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Address Number of bytes (ADDR_NO_OF_BYTES) | 94:92  | Determines number of address bytes to be sent in the instruc- tion. Up to 6 address bytes can be sent in one sequence. A value of 7 encoded here is not permitted. A value of 0 encoded implies that no address bytes is sent (for example, the address phase is disabled). If ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration determines the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. It is, therefore, required for the host to configure an even number of address. If the host configures ADDR_NO_OF_BYTES = 1, 3 or 5, the controller generates an error. |
| Reserved                                   | 91:90  | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 21-22: STIG Instruction - Legacy HyperFlash or xSPI Profile 2 (Continued)

| Name                                                                               | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|------------------------------------------------------------------------------------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Number of data bytes (DATA_NO_OF_BYTES) or Number of mode bytes (MODE_NO_OF_BYTES) | 89:88  | For cases where the number of data (or mode) bytes is ≤ 2, this field allows that data be included within this instruction, directly avoiding the need to glue a separate data sequence instruction. If there are more than two data bytes required to be transferred, the instruction must be glued with a data sequence. Note that if up to two data (or mode) bytes are transferred directly from this instruction, the format in which they are pre- sented to the device depends on the ADDR_EDGE_MODE and ADDR_NO_OF_IOS configuration. If the host needs the data to be sent in a different format, the field must be cleared (=0) . Instead, transfer the data using a glued data sequence instruction, or generic instruction, where the format of the data bytes can be individually configured. 0 - Zero data/mode bytes are sent to device (data phase disa- bled) 1 - One byte is transferred to the device 2 - Two bytes are transferred to the device 3 - N/A If ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration determines the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. The host must configure an even number of bytes. Failure to do this results in an error being reported. |
| CMD                                                                                | 87:80  | Opcode mnemonic                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| CMD_EXT                                                                            | 79:72  | Opcode extension mnemonic                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ADDR5                                                                              | 71:64  | Byte 5 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ADDR4                                                                              | 63:56  | Byte 4 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ADDR3                                                                              | 55:48  | Byte 3 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ADDR2                                                                              | 47:40  | Byte 2 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ADDR1                                                                              | 39:32  | Byte 1 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| ADDR0                                                                              | 31:24  | Byte 0 of the address phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| DATA1/MODE1                                                                        | 23:16  | Byte 1 of the data/mode phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| DATA0/MODE0                                                                        | 15:8   | Byte 0 of the data/mode phase                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| Reserved                                                                           | 7      | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Instruction Type                                                                   | 6:0    | This field defines the type (index) of the instruction. A descrip- tion of each value and its corresponding instruction can be found in the section, Instruction Type Variants.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 21-23: Generic Command STIG Instruction

| Name                                 | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------------------------|---------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Reserved                             | 127:125 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Instruction Termination (INSTR_LINK) | 124     | This bit is only relevant when the instruction_type field of the instruction=0, 32, 36 or 96. Refer to section Instruction Type Variants for details on the instruction type setting. The field assigned to this bit indicates whether the instruction is to be terminated (CS must be deactivated after the instruction finishes) or glued with the next instruction (providing it is ready to be fetched from the command FIFO). 0 - Current sequence is terminated and not glued with another instruction of the same type 1 - Current sequence is glued with the next STIG instruction |
| Reserved                             | 123:118 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| CRC Enable (CRC_EN)                  | 117     | Set to enable dynamic CRC calculation and output this value on the xSPI flash interface. The transfer modes are specified by ADDR_EDGE_MODE and ADDR_NO_OF_IOS config- uration. If the octal DDR (8D) variant is configured, the calcu- lated CRC byte is maintained for a full flash clock cycle.                                                                                                                                                                                                                                                                                         |
| Reserved                             | 116:111 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Bank Number (BANK_NUM)               | 110:108 | This field informs the controller on which memory device the sequence should be executed. This information directly maps to the chip select (CS) pin.                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| Edge Mode (EDGE_MODE)                | 107     | This bit selects SDR or DDR operation at the memory device. 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| Reserved                             | 106     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| IO Mode (NO_OF_IOS)                  | 105:104 | Determines number of active IOs for the instruction 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Reserved                             | 103:92  | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 21-23: Generic Command STIG Instruction (Continued)

| Name                          | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|-------------------------------|--------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Number of bytes (NO_OF_BYTES) | 91:88  | Determines the number of bytes to be sent in the instruction At least one byte must be sent, but no more than 10 bytes in one sequence. If EDGE_MODE and NO_OF_IOS config- uration determines the octal DDR variant, two bytes must be sent in a single flash clock cycle. The host must config- ure an even number of bytes in the quoted case (allowable NO_OF_BYTES = 0 or 2 or 4 or 6 or 8 or 10). If the host configures NO_OF_BYTES = 1 or 3 or 5 or 7 or 9, the controller does not accept the transfer request and generates an error. |
| BYTE9                         | 87:80  | The 10th byte sent when NO_OF_BYTES = 10                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| BYTE8                         | 79:72  | The 9th byte sent when NO_OF_BYTES ≥ 9                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE7                         | 71:64  | The 8th byte sent when NO_OF_BYTES ≥ 8                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE6                         | 63:56  | The 7th byte sent when NO_OF_BYTES ≥ 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE5                         | 55:48  | The 6th byte sent when NO_OF_BYTES ≥ 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE4                         | 47:40  | The 5th byte sent when NO_OF_BYTES ≥ 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE3                         | 39:32  | The 4th byte sent when NO_OF_BYTES ≥ 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE2                         | 31:24  | The 3rd byte sent when NO_OF_BYTES ≥ 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE1                         | 23:16  | The 2nd byte sent when NO_OF_BYTES ≥ 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| BYTE0                         | 15:8   | The 1st byte sent                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| Reserved                      | 7      | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| Instruction Type              | 6:0    | This field defines the type (index) of the instruction. A descrip- tion of each value and its corresponding instruction can be found in the section Instruction Type Variants.                                                                                                                                                                                                                                                                                                                                                                 |

## Glued Data Instruction

The Data Instruction Structure table outlines the structure of the main data instruction variant that the host issues as the final glued instruction of a sequence of instructions. This variant cannot be used by itself and must be glued to one of the other variants. It is typically used to define how data is transferred in accordance with the details specified in the other three main command variants. As such, this variant must never be the first instruction issued by the host; it is always the last instruction.

Table 21-24: Data Instruction Structure

| Name     | Bits    | Description   |
|----------|---------|---------------|
| Reserved | 127:123 | N/A           |

Table 21-24: Data Instruction Structure (Continued)

| Name                                       | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------------------------------|---------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CRC Chunk Size (CRC_CHUNK_SIZE)            | 122:120 | This field presents the size of the CRC chunk as configured in the memory device. It is the number of bytes after which the CRC byte is placed onto the bus. 1 - N/A 2 - 8B chunk size 3 - 16B chunk size 4 - 32B chunk size 5 - 64B chunk size 6 - 128B chunk size 7 - 256B chunk size                                                                                                                                                                                                                                                                                                                                                                           |
| CRC Read Status Inversion Enable (CRC_INV) | 119     | 8 - 512B chunk size This bit controls whether the controller requests one more status slice to check its data integrity. The expectation is that the redundant data chunk will be inverted (as applicable for Macronix MX25 octal devices). If redundant, data from flash is not inverted. A CRC error is generated. This bit must only be set when CRC_EN = 1, DATA_EDGE_MODE = 1 (DDR), DATA_NO_OF_IOS = 3 (octal) and DATA_NO_OF_BYTES = 2 because this set of parameters matches the implementation of Macronix MX25 octal devices. If CRC_INV = 1, the CRC_CHUNK_SIZE configuration is ignored because these two sequence parameters are mutually exclusive. |
| CRC Output Enable (CRC_OE)                 | 118     | This bit specifies whether the CDC is an output on both clock edges, with the CRC being inverted on the second clock edge. 0 - Inverted CRC is not output 1 - Inverted CRC is output                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| CRC Enable (CRC_EN)                        | 117     | This bit is set to enable dynamic CRC calculation and output this value on the xSPI flash interface. The trans- fer modes are specified by ADDR_EDGE_MODE and ADDR_NO_OF_IOS configurations. If an octal DDR (8D) variant is configured, the calculated CRC byte is maintained for a full flash clock cycle.                                                                                                                                                                                                                                                                                                                                                      |

Table 21-24: Data Instruction Structure (Continued)

| Name                                          | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|-----------------------------------------------|---------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CRC Unaligned Chunk Enable (CRC_UAL_CHUNK_EN) | 116     | Setting this bit instructs the controller to take into consider- ation the command address in determining where the CRC bytes are expected to be returned from the memory device. This bit can be set (=1) only when the sequence (DATA_SEQ) is glued with the READ_PROFILE_1 sequence. It must be cleared (=0), otherwise. For example, if the original flash address is 0x4 and CRC_CHUNK_SIZE is 16B, the first CRC from the device is expected to occur after 12 bytes (CRC_CHUNK_SIZE - (device address)). Only even addresses are supported (when CRC_UAL_CHUNK_EN = 1) as required by Macronix MX25 octal devices. |
| CRC Unaligned Chunk Check (CRC_UAL_CHUNK_CHK) | 115     | Set this bit to have the controller check the correctness of a CRC unaligned chunk from a flash device. It can be set (=1) only when CRC_UAL_CHUNK_EN = 1. It must be cleared (=0), otherwise. Because Macronix MX25 octal devices do not guarantee the correctness of CRC cal- culated for unaligned chunk, the controller provides control of this check. When configured, DATA_NO_OF_BYTES accommodates many CRC chunks; the CRC correctness is checked for all chunk-aligned CRC slices regardless of the CRC_UAL_CHUNK_CHK value.                                                                                    |
| Reserved                                      | 114:113 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Data Swap                                     | 112     | When set (=1), this bit swaps the order of bytes transferred in a single octal DDR clock cycle. When DATA_EDGE_MODE and DATA_NO_OF_IOS do not reflect the octal DDR mode, this bit should be cleared (=0).                                                                                                                                                                                                                                                                                                                                                                                                                |
| Reserved                                      | 111     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Bank Number (BANK_NUM)                        | 110:108 | This field informs the controller on which memory device the sequence should be executed. This value directly maps to the chip select (CS) pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Data Edge Mode (DATA_EDGE_MODE)               | 107     | This bit selects SDR or DDR operation at the memory device 0 - SDR 1 - DDR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| Reserved                                      | 106     | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 21-24: Data Instruction Structure (Continued)

| Name                                                        | Bits    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                |
|-------------------------------------------------------------|---------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Data IO Mode (DATA_NO_OF_IOS)                               | 105:104 | Determines number of active IOs for all data bytes of the instruction Data bytes associated with the instruction are taken from an internal data FIFO (for write operations) and put into the internal data FIFO (for read operations) 0 - Single 1 - Dual 2 - Quad 3 - Octal                                                                                                                                                              |
| Reserved                                                    | 103:101 | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Transfer Direction (DIR)                                    | 100     | Determines the direction of the data stream 0 - Read from memory device 1 - Write to memory device                                                                                                                                                                                                                                                                                                                                         |
| Reserved                                                    | 99:93   | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Data Size per Flash Address Location (DATA_PER_ADDR)        | 92      | Indicates how many bytes of data are assigned to a single flash memory address. The controller uses this information to internally calculate consecutive addresses of all data bytes in a sequence. The address incrementation step is assumed to be 1. 0 - The memory device is byte addressable 1 - The memory device is two byte addressable                                                                                            |
| Reserved                                                    | 91:90   | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Number of dummy (latency) cycles (NO_OF_DUMMY)              | 89:84   | Indicates the number of dummy cycles before the data phase Note that the definition of latency cycles for legacy Hyper- Flash, and xSPI profile 2 devices overlaps with the address transfer phase by one clock cycle. The number of required latency cycles is one less than programmed. If an external board delay incurs a few more clock cycle long additional latency (for read commands), compensation is handled by the PHY module. |
| HyperFlash Read Crossing Boundary Enable (HF_READ_BOUND_EN) | 83      | The controller uses this bit to calculate a read transaction cross- ing a page boundary phase in which data is not valid (based on the number of dummy cycles (NO_OF_DUMMY) and the start address of the associated instruction). During the latency period, the device does not generate DQS.                                                                                                                                             |
| Reserved                                                    | 82:80   | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 21-24: Data Instruction Structure (Continued)

| Name                                    | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|-----------------------------------------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Number of data bytes (DATA_NO_OF_BYTES) | 79:48  | Indicates the number of data bytes to be sent in the instruction. All data bytes are taken from the data FIFO of the controller (for write operations) and put into the data FIFO (for read operations). When DATA_EDGE_MODE and DATA_NO_OF_IOS con- figuration indicates the octal DDR variant, two bytes are re- quired to be sent in a single flash clock cycle. Therefore, the host must configure an even number of data bytes. Failure to do this causes the controller to generate an error.                                                                 |
| Reserved                                | 47:25  | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Status Data Source (STATUS_SOURCE)      | 24     | The bit is valid only when DIR = 0 (read transaction) and there are ≤ 2 bytes to be read. When there are >2 bytes to be read, this bit should be zero. This bit determines how the host eventually receives the data. If there are ≤ 2 bytes to be read, the host can choose to have the two bytes reported in the STIG status register at offset 0x44. Otherwise, the read data must be fetched using the bus com- pleter AXI interface. 0 - Read data is sent to the AXI bus completer interface 1 - Read data is sent to the STIG status register at offset 0x44 |
| Reserved                                | 23:7   | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Instruction Type                        | 6:0    | This value is always fixed at 127 for the glued data instruction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Instruction Type Variants

The Typical Device Operations table shows typical operations and the associated value in the instruction type field that the host can carry out on the device using instruction variant 1. The table also outlines whether an instruction supports instruction gluing.

NOTE: If a data instruction is glued to the back of one of the instructions, the data instruction must have an instruction type field configured to 128.

Table 21-25: Typical Device Operations - Variant 1

| Operation   |   Instruction Type | Assumed Instruction Gluing                    |
|-------------|--------------------|-----------------------------------------------|
| Read        |                  1 | Yes - Glue with data instruction              |
| Read In XIP |                  1 | Yes - Glue with data instruction              |
| Write       |                  0 | Yes, if data length is greater than two bytes |
| Set WEL     |                  0 | No                                            |
| Clear WEL   |                  0 | No                                            |

Table 21-25: Typical Device Operations - Variant 1 (Continued)

| Operation                            |   Instruction Type | Assumed Instruction Gluing                    |
|--------------------------------------|--------------------|-----------------------------------------------|
| Erase Sector                         |                  0 | No                                            |
| Chip Erase                           |                  0 | No                                            |
| Program Suspend                      |                  0 | No                                            |
| Program Resume                       |                  0 | No                                            |
| Read Device Register                 |                  1 | No                                            |
| Write Device Register                |                  0 | No                                            |
| Setup Read Burst                     |                  0 | No                                            |
| Clear Flag Register                  |                  0 | No                                            |
| Enter Deep Powerdown                 |                  0 | No                                            |
| Exit Deep Powerdown                  |                  0 | No                                            |
| Reset Enable                         |                  0 | No                                            |
| Soft Reset                           |                  2 | No                                            |
| Read JEDEC ID                        |                  1 | No                                            |
| Read SFDP                            |                  1 | Yes - Glue with data instruction              |
| Enter Default Protocol Mode          |                  0 | No                                            |
| Enter SDR Octal                      |                  0 | No                                            |
| Global Protection                    |                  0 | No                                            |
| Global Unprotection                  |                  0 | No                                            |
| Protect Sector                       |                  0 | No                                            |
| Unprotect Sector                     |                  0 | No                                            |
| Enter 4-Byte Address Mode            |                  0 | No                                            |
| Exit 4-Byte Address Mode             |                  0 | No                                            |
| SPI_NAND Page Read                   |                  0 | Yes - Glue with data instruction              |
| SPI_NAND Read From Cache             |                  0 | Yes - Glue with data instruction              |
| SPI_NAND Page Read Page Cache Random |                  0 | No                                            |
| SPI_NAND Page Read Page Cache Last   |                  0 | No                                            |
| SPI_NAND Program Load                |                  0 | Yes, if data length is greater than two bytes |
| SPI_NAND Program Load Random         |                  0 | Yes, if data length is greater than two bytes |
| SPI_NAND Program Execute             |                  0 | Yes, if data length is greater than two bytes |
| SPI_NAND Set Feature                 |                  0 | Yes, if data length is greater than two bytes |
| SPI_NAND Get Feature                 |                  1 | Yes - Glue with data instruction              |

Table 21-25: Typical Device Operations - Variant 1 (Continued)

| Operation        |   Instruction Type | Assumed Instruction Gluing                  |
|------------------|--------------------|---------------------------------------------|
| SPI_NAND Read ID |                 32 | Yes - Glue with data or variant instruction |

The Typical Device Operations table shows typical operations and the associated value in the instruction type field that the host can carry out on the device using instruction variant 2. The table outlines some of the instruction fields that must be forced to a fixed value to carry out the operation, and whether an instruction supports instruction gluing.

The variant should be used when trying to formulate a generic or custom command sequence that is not a known protocol-specific instruction. For this instruction, select an instruction type field of 96. This instruction is permitted to be glued with another variant 3 instruction or with a data sequence instruction.

Table 21-26: Typical Device Operations - Variant 2

| Operation                                 |   Instruction Type | Assumed Instruction Gluing                                                               |
|-------------------------------------------|--------------------|------------------------------------------------------------------------------------------|
| Write/Read for HyperRAM Devices           |                 32 | Yes - Glue with data or variant instruction                                              |
| WREN 1                                    |                 33 | No                                                                                       |
| WREN 2                                    |                 34 | No                                                                                       |
| Profile 2 Read                            |                 35 | Yes - Glue with data instruction                                                         |
| Profile 2 Program                         |                 36 | Yes, if the data length is greater than two bytes. Can glue with data or another variant |
| Erase Sector                              |                 37 | No                                                                                       |
| Chip Erase                                |                 38 | No                                                                                       |
| Erase Suspend                             |                 39 | No                                                                                       |
| Erase Resume                              |                 40 | No                                                                                       |
| Program Suspend                           |                 41 | No                                                                                       |
| Program Resume                            |                 42 | No                                                                                       |
| Read Status                               |                 43 | Yes - Glue with data instruction                                                         |
| Read Volatile Memory Register             |                 44 | Yes - Glue with data instruction                                                         |
| Read Non-Volatile Memory Register         |                 45 | Yes - Glue with data instruction                                                         |
| Read Customer Memory Register             |                 47 | Yes - Glue with data instruction                                                         |
| Status Register Clear                     |                 48 | No                                                                                       |
| Write Volatile Memory Register            |                 49 | No                                                                                       |
| Write Non-Volatile Memory Register Enable |                 50 | No                                                                                       |
| Write Custom Memory Register Enable       |                 51 | Yes - Complete with another variant 2 in- struction with instruction type = 32           |
| Enter Deep Powerdown                      |                 52 | No                                                                                       |

Table 21-26: Typical Device Operations - Variant 2 (Continued)

| Operation                   |   Instruction Type | Assumed Instruction Gluing       |
|-----------------------------|--------------------|----------------------------------|
| Reset/ASO Exit              |                 53 | No                               |
| Enter Default Protocol Mode |                 54 | No                               |
| Read SFDP                   |                 55 | Yes - Glue with data instruction |

## Switching Work Modes

STIG work mode provides an interface to execute low-level transactions on the xSPI flash interface. It is expected to be used in conjunction with ACMD or direct work mode. Therefore, switching into and out of STIG mode can become a common requirement. The host software must enter and exit STIG mode when the target xSPI flash device is in the idle state.

Before the host switches to STIG mode, it must wait for all operations that address the same xSPI flash device to be finished. Before the host exits STIG mode, it must check whether the device is in an idle state. Waiting for the last STIG command to complete execution may not be enough for cases where the command caused the xSPI device go into a busy state. In such cases, the host can poll the status register of the device. It can issue STIG instructions to execute read status commands and continue to do this until the memory device has reached an idle state. Once the device is in the idle state, the host can exit STIG mode.

## Programming STIG Mode

Complete the following steps to program STIG mode.

1. Wait for xSPI flash controller to be in the idle state, then, poll the XSPI\_GSTAT.CTL\_BUSY bit.
2. Configure the XSPI\_WORKMODE\_CTL field to 2'b01 to enable STIG work mode.
3. Configure the command registers ( XSPI\_CMD1 to XSPI\_CMD4 ) .
4. Enable the STIG DONE interrupt when completion of the requested STIG transaction must raise external interrupt or sdma\_trigg for the AXI bus completer interface transactions. Set the XSPI\_INT\_EN.STIG\_DONE\_EN , XSPI\_INT\_EN.SDMA\_TRIGG\_EN , and XSPI\_INT\_EN.GINT\_EN bits to enable these interrupts.
5. Start the STIG operation by writing to the XSPI\_CMD0 register. Set XSPI\_GSTAT.GCMD\_ENG\_MC\_BUSY = 1.
6. Based on the instruction type field, the host must select one of the following:
- Gluing with a data sequence, proceed to the next step.
- For other instructions without the gluing condition, skip to step 9.
- Otherwise, return to configuring the command register step.
7. When the instruction that contains the data sequence command indicates a read direction, and the number of data to be read is one or two bytes and the STATUS\_SOURCE field in the instruction =1, then skip to step 9.

In all other cases, proceed to the next step.

8. The host transfers data through the AXI host interface of the controller to complete the STIG operation. In this step, XSPI\_GSTAT.GCMD\_ENG\_MC\_BUSY =1.
- a. Poll the XSPI\_ISTAT.SDMA\_TRIGG bit, or alternatively wait for the interrupt if the SDMA\_TRIGG interrupt was unmasked. When XSPI\_ISTAT.SDMA\_TRIGG =1, proceed.
- b. Read the XSPI\_SDMA\_SIZ and XSPI\_SDMA\_TRD\_STAT registers to determine the number of bytes to transfer, the thread ID and transfer direction.
- c. Clear the XSPI\_ISTAT.SDMA\_TRIGG flag.
- d. The external host can now transfer data. For program operations, it is expected that the external host will issue one or more AXI write requests with a total size matching the data transfer indicated in step b. For read operations, it is expected that the external host will issue one or more AXI read requests with a total size matching the data transfer indicated in step b. The external host should issue an address that matches the one reported in the XSPI\_SDMA\_ADDR0 and XSPI\_SDMA\_ADDR1 registers.
- e. If the host ignores any of the requirements described above and starts the data transfer when the host DMA is not ready, the XSPI\_ISTAT.SDMA\_ERR flag is set. If XSPI\_INT\_EN.SDMA\_ERR\_EN = 1, an interrupt is also triggered. When XSPI\_DMA\_CTL.SDMA\_ERR\_RSP =1, an error response is returned. When XSPI\_DMA\_CTL.SDMA\_ERR\_RSP =0, an OK response is returned.
- f. If the host sends an unsupported transaction to the bus completer interface, the bus completer DMA ignores the access and the SDMA\_ERR flag in the XSPI\_ISTAT register is set. If an AXI error response is detected on the system bus during host DMA transfer, the fail and bus error bits are set in the last operation status. The last operation status can be checked in the XSPI\_CMD\_STAT register.
- g. Performing transactions in wrong direction (that is, a write transaction to host DMA port in case of sending read operation command to xSPI device) is forbidden and causes the XSPI\_ISTAT.SDMA\_ERR flag to be set.When XSPI\_INT\_EN.SDMA\_ERR\_EN = 1, an interrupt is also triggered. When XSPI\_DMA\_CTL.SDMA\_ERR\_RSP =1, an error response is returned. When XSPI\_DMA\_CTL.SDMA\_ERR\_RSP =0, an OK response is returned.
9. Wait for the STIG transaction completion status. Poll the COMPLETE bit in the XSPI\_CMD\_STAT register or view the status through the external interrupt from the STIG\_DONE (when correctly enabled by the host). In this step, XSPI\_GSTAT.GCMD\_ENG\_BUSY =1.
10. Check the STIG status by reading the XSPI\_CMD\_STAT register. In this step, the generic command engine busy flags are programmed to a low value.
11. If another STIG operation must be executed, go to step 3. Otherwise, if the work mode must be changed, wait for the controller idle state by polling the XSPI\_GSTAT.CTL\_BUSY bit.

## STIG Mode Status

The host can check the latest STIG operation status by reading the XSPI\_CMD\_STAT register.

After every new operation is triggered, the COMPLETE bit is cleared and other bits in the status register are not valid. When the operation completes, the COMPLETE bit is set and other bits in the status register become valid.

Table 21-27: STIG Mode XSPI\_CMD\_STAT Bit Descriptions

| Name          | Bits   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|---------------|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| DATA_FROM_DEV | 31:16  | This field holds one or two bytes read from memory device. Fields of the DATA_SEQ instruction must be configured as follow: DATA_NO_OF_BYTES = 1 - one byte, or 2 - two bytes STATUS_SOURCE = 1. In other cases, data from the flash device is available on the bus completer data interface.                                                                                                                                                                                                                                                                                                                                                                  |
| COMPLETE      | 15     | When set, it denotes that the controller has updated status information and the operation is complete. This bit is set even when the operation ended as a failure.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| FAIL          | 14     | When set, it denotes that the operation failed to complete successfully.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| Reserved      | 13:4   | N/A                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| DQS_ERROR     | 3      | The bit indicates that an incorrect DQS pulses number was detected during the data read operation. This is information from the PHY that either the DQS strobe did not appear during read (for example, the device is not connected to the controller) or the RD_DEL_SEL signal value is incorrect (field in the XSPI_PHY_GATE_LPBK_CTL ) and data read from flash device are corrupted and read FIFO pointers in the PHY are misaligned. The DLL RESET or RESET signals clear the un- derrun/overflow flags in the PHY and clear all PHY read data pointers. One of these flags is required by the PHY before continuing to work after a DQS_ERROR assertion. |
| CRC_ERROR     | 2      | This bit indicates that a CRC error has been received                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| BUS_ERROR     | 1      | When set, it indicates that the controller received an error response on the system DMAinterface.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| CMD_ERROR     | 0      | This bit indicates that an invalid command sequence has been detected.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

## STIG Limitations

STIG mode has the following limitations:

- The HOLD# functionality (alternative function of DQ3 pin of some legacy SPI devices) is not supported in STIG mode. Since STIG mode is fully software controlled, the HOLD# function is not particularly useful. Transfer management can instead be handled by higher-level layers (system layers) throttling when STIG transfers are triggered.
- The xSPI flash controller does not support wrap data flash transfer commands (all data to/from device are considered linear).
- The maximum number of bytes that can be read when using a generic STIG instruction type is limited to 64 bytes. Similarly, a maximum of 64 bytes can be read for all legacy HyperFlash and xSPI profile 2 instructions other than the READ\_PROFILE\_2 and READ\_SFDP\_P2.
- If XIP mode is used, the read operation that is used to either enter or exit XIP mode should not attempt to transfer more than 64B.

## Sequencing STIG Operation

Internally, the STIG instructions are held in the main data FIFO. Based on the instruction type, glued instructions are sent and buffered in a second-level internal cache FIFO. This cache FIFO can accommodate up to 16 glued sequences (if more glued sequences are requested, an incorrect sequence error is generated). The instructions are processed together when the INSTR\_LINK field = 0 of the last instruction in the instruction chain.

Internally, the controller translates the instructions and breaks them down into the correct sequence required by the memory device based on the type of the command. The general structure of xSPI frame is shown in the STIG Frame Structure diagram.

Figure 21-5: STIG Frame Structure

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000004_baf94e555a36318881ceb4cd7400aedba8bd5fea1491c4af1517345f3bd92bfb.png)

The blocks correspond to xSPI transfer phases. Depending on the individual command layout , some phases may be dropped; however, the order is always maintained.(Refer to JESD216 or a particular xSPI device specification). The EDGE\_MODE and NO\_Of\_IOS can also differ in between phases. Per JESD216, only incremental increases over the number of IOs used or number of clock edges used is permitted. For example, the (opcode=1S, addr=8D, data=8D) transfer variant may be supported by current devices, but, a variant like (opcode=1S, addr=8D, data=4S)

is not. JESD216 does not allow for transfers with a number of active IO lines that differs from one another more than once. Also, note that if any of the phases are configured to be 8D and the associated NO\_OF\_BYTES corresponds to an odd value, the controller generates an error and does not accept the instruction. The 8D variant always requires an even number of bytes. The flexible command sequence structure of the controller allows any configuration of EDGE\_MODE and NO\_OF\_IOS that provides a future proven solution for new variants of future revisions of JESD216.

## Event Handling

The controller reports errors slightly differently depending on the selected work mode:

- For STIG or ACMD PIO work modes, the status is reported using the XSPI\_CMD\_STAT register at offset 0x44.
- For ACMD CDMA work mode, the status is reported using the status field of descriptor associated with a specified thread and the XSPI\_CMD\_STAT register.
- For direct work mode - the DIR\_* bits in the XSPI\_ISTAT register reflect the status.

The Error Descriptions table identifies the errors that can occur.

Table 21-28: Error Descriptions

| Error            | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| DQS Error        | This type of error is reported when the attached PHY detects a DQS underrun/overflow situation. When the DQS error is detected then data stream transferred from the xSPI flash memory can be corrupted. The host should assert DLL_RST_N or RST_N to clear DQS underrun/overflow flags in the PHY and clear all PHY read data pointers.                                                                                                                                                                                            |
| System Bus Error | For ACMD or STIG work modes, for the AXI bus completer interface, this error is reported when: 1. The host attempts to access the bus completer interface before it is permitted (refer to the required steps in STIG and ACMD sections of this document). 2. The host issues an unsupported burst type. The controller only supports incremental (non-wrapping) bursts. For the AXI bus requester interface (not applicable for STIG or direct mode) the error may occur when an AXI error response is returned on RRESP or BRESP. |

Table 21-28: Error Descriptions (Continued)

| Error        | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CMDError     | STIG mode: This error occurs when the specific STIG sequence as defined in this document is not followed properly. To rectify, review the sequence carefully and repeat. Direct mode: In this mode, the DIR_CMD_ERR interrupt is triggered when the host has issued an AXI transaction that cannot be executed by the controller for one of the following reasons. • The command extension is not enabled when the opcode phase reflects octal DDR mode. • The number of address bytes is not even when the address phase reflects octal DDR mode. • The address value is not even when the address phase reflects octal DDR mode. • The number of data bytes is not even when the data phase reflects octal DDR mode, unless RWDS byte masking function- ality is enabled through the direct access configuration register (only available for direct work modes). • The number of bytes for the status checking sequence is not even when data phase reflects octal DDR mode. • Device and controller are configured to work in XIP mode but, the write access was made from the host. • Device and controller are configured to work in XIP mode but, the sending of mode byte (enable/disable) is disabled. • The DATA_PER_ADDR parameter is set and transfer address or transfer size is not even, unless RWDS byte masking func- tionality is enabled through the direct access configuration reg- ister (only available for direct work modes). • Word size (sAWSIZE or sARSIZE) is greater than the selected page size in profile 1 or profile 2 - HF. ACMD mode: Conditions which cause the CMD_ERROR being triggered are described in the section, Rules and Limitations. |
| CRC Error    | When CRC checking is enabled, this error is reported when the controller has detected a CRC error.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| Device Error | A device error is reported when the attached memory device re- turned fail after checking the status of the last erase/program/read operation. It is applicable only for ACMD and direct work modes. The status of a read operation is only checked for SPI NAND devi-                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 21-28: Error Descriptions (Continued)

| Error     | Description                                                                                                                                                                                         |
|-----------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           | ces. Detecting an error in this case means that the device returned an uncorrectable ECC error.                                                                                                     |
| ECC Error | An ECC correctable error is reported when the attached memory device returns a correctable error during a read operation and when the XSPI_STAT_SEQ_CFG10.ECC_FAIL_EN field is set in the register. |

## Device Discovery

Device discovery (DD) is used to identify the memory device and pre-configure controller registers to the required state automatically following power on reset. Automated device discovery is implemented in the controller using the SFDP as defined in JEDEC standard JESD216. The SFDP standard provides a consistent method of describing the functional and feature capabilities of serial flash devices in a standard set of parameter tables. The SFDP must be implemented in the attached memory device for discovery to be successful.

Device discovery can be triggered in two ways - automatically, following power on reset (by default, this always occurs, unless the controller input discovery\_inhibit is set (=1), or manually using the discovery control XSPI\_DISCOVERY\_GCTL register.

## Discovery Modes

Device discovery can automatically identify the memory device at power on reset and initialize many of the controller registers. However, the user can alternatively choose to trigger device discovery manually. There are three modes of operation for device discovery: full automatic, preconfigured SFDP , and preconfigured non-SFDP .

Full Automatic Mode Detection - In this default mode, the controller must first determine how to communicate with an unknown device. Per JESD216, if the memory device supports device discovery, it must also support the read SFDP command. The controller, therefore, attempts to communicate by sending read SFDP commands in different transfer formats. Each time it waits for an indication that the device has interpreted the command successfully. When a valid SFDP header is recognized, the controller knows which transfer format has been successful. It proceeds automatically to fetch and decode the SFDP database residing within the device. As the database is fetched from the device, the controller updates the controller registers accordingly.

To trigger device discovery using this mode as part of power on reset initialization, ensure inputs XSPI\_DISCOVERY\_GCTL.INHIBIT = 0 and XSPI\_DISCOVERY\_GCTL.NUM\_LINES = 0.

To trigger device discovery using this mode from the registers, write the following values to the XSPI\_DISCOVERY\_GCTL register.

- XSPI\_DISCOVERY\_GCTL.REQ\_TYP = 0

```
· XSPI_DISCOVERY_GCTL.NUM_LINES = 0 · XSPI_DISCOVERY_GCTL.REQ = 1
```

When operating in full automatic mode detection, the initial querying of the memory device to identify the correct transfer mode is performed as part of the controller's initialization process. Once completed, the XSPI\_DISCOVERY\_GCTL register (at 0x260) is automatically updated.

Preconfigured SFDP mode - If the user already knows what memory device the controller is connected to and that SFDP is supported, the controller does not need to query the device with different transfer formats of the read SFDP command. Device discovery is triggered as in full automatic detection mode, but it bypasses the initial sequence to identify the required transfer mode. This can speed up the device identification process.

To trigger device discovery using this mode as part of power on reset initialization, ensure inputs XSPI\_DISCOVERY\_GCTL.INHIBIT = 0 and XSPI\_DISCOVERY\_GCTL.NUM\_LINES !=0. To trigger device discovery using this mode from registers, write the following values to the XSPI\_DISCOVERY\_GCTL register:

- XSPI\_DISCOVERY\_GCTL.REQ\_TYP = 0
- XSPI\_DISCOVERY\_GCTL.NUM\_LINES != 0 (choose the value suited to the attached device, refer to the XSPI\_DISCOVERY\_GCTL register definition for details)
- XSPI\_DISCOVERY\_GCTL.REQ = 1

Preconfigured non-SFDP mode - In this mode, the controller does not attempt to fetch the SFDP database from the memory device. This alternative configuration method allows the controller to operate with a pre-known memory device. It is optional, and the user can alternatively program each controller register manually.

To trigger device discovery using this mode as part of power on reset initialization, ensure inputs XSPI\_DISCOVERY\_GCTL.INHIBIT = 1 and XSPI\_DISCOVERY\_GCTL.NUM\_LINES !=0. To trigger device discovery using this mode from the registers, write the following values to the XSPI\_DISCOVERY\_GCTL register:

- XSPI\_DISCOVERY\_GCTL.REQ\_TYP = 1
- XSPI\_DISCOVERY\_GCTL.NUM\_LINES != 0 (choose the value suited to the attached device, refer to the XSPI\_DISCOVERY\_GCTL register definition for details)
- XSPI\_DISCOVERY\_GCTL.REQ = 1

## Discovery Flow

The Discovery Flow Diagram shows the operations and decisions that are part of the device discovery process.If device discovery is inhibited, or no valid SFDP header or parameter page header is recognized during the flow, the SFDP database/parameter page cannot be obtained from the device and all controller registers retain the reset values.

The host must not trigger any operation while device discovery is in progress. The discovery process completion is marked by the controller setting the XSPI\_DISCOVERY\_GCTL.PASS bit. When full automatic mode is in use, discovery is complete when either the INIT\_COMP pin is high, or XSPI\_GSTAT.INIT\_PASS =1.

Figure 21-6: Device Discovery Flow

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000005_58081bb0f9a310866ecec4eb44bf36143c1bfcbe6b1dcd68c845b6e0370f8db9.png)

## SFDP Command Structure

When full automatic mode detection is active, the controller must determine how to communicate with the device. It communicates by repeatedly sending read SFDP commands in different transfer formats. This mode is selected when XSPI\_DISCOVERY\_GCTL.NUM\_LINES signals is configured to 4'd0. The algorithm that the controller executes differs from the suggested algorithm presented in the JESD216 specification document. The algorithm starts with single mode, goes through dual, quad, and octal modes, and finishes with the read SFDP command (compliant with legacy HyperFlash and xSPI profile 2.0).

The read SFDP commands are sent in the following order. If the device does not successfully interpret any of these steps, the device discovery is deemed to have failed.

1. 1S-1S-1S - opcode, address and data phases used for reading SFDP database are sent on one line in SDR mode. Command uses three address bytes and eight dummy clock cycles.
2. 1D-1D-1D - opcode, address and data phases used for reading SFDP database are sent on one line in DDR mode. Command uses three address bytes and eight dummy clock cycles.
3. 2S-2S-2S - opcode, address and data phases used for reading SFDP database are sent on two lines in SDR mode. Command uses three address bytes and eight dummy clock cycles.
4. 2D-2D-2D - opcode, address and data phases used for reading SFDP database are sent on two lines in DDR mode. Command uses three address bytes and eight dummy clock cycles.
5. 4S-4S-4S - opcode, address and data phases used for reading SFDP database are sent on four lines in SDR mode. Command uses three address bytes and eight dummy clock cycles.
6. 4D-4D-4D (Quad DDR* - presented as 4S-4D-4D in JESD216 specification) - opcode, address, and data phases used for reading SFDP database are sent on four lines. The opcode phase is sent in DDR mode with enabled command extension value. Address and data phases are sent in DDR mode. Command uses 3 address bytes and eight dummy clock cycles.
7. 4D-4D-4D (Quad DDR) - opcode, address and data phases used for reading SFDP database are sent on four lines in DDR mode. Command uses three address bytes and eight dummy clock cycles.
8. 8S-8S-8S - opcode, address and data phases used for reading SFDP database are sent on eight lines in SDR mode. Command uses three address bytes and eight dummy clock cycles.
9. 8S-8S-8S - opcode, address and data phases used for reading SFDP database are sent on eight lines in SDR mode. Command uses four address bytes and 20 dummy clock cycles. The opcode is sent twice and the second one is logical negation of the first one.
10. 8D-8D-8D - opcode, address and data phases used for reading SFDP database are sent on eight lines in DDR mode. Command uses four address bytes and 8 dummy clock cycles. The opcode extension is repetition of the opcode.
11. 8D-8D-8D - opcode, address and data phases used for reading SFDP database are sent on eight lines in DDR mode. Command sends the address over two clock cycles (meaning four bytes are send as this is octal DDR), but only three address bytes are valid. The opcode extension is repetition of the opcode.
12. 8D-8D-8D - opcode, address and data phases used for reading SFDP database are sent on eight lines in DDR mode. Command uses four address bytes and 20 dummy clock cycles. The opcode extension is logical negation of the opcode.
13. 8D-8D-8D - read SFDP command for legacy HyperFlash and xSPI profile 2.0 memory devices with 15 latency clock cycles. When preconfigured mode is selected, the above step is not required.

To use preconfigured mode in device discovery, configure the parameters of the read SFDP (READ\_SFDP) instructions using XSPI\_DISCOVERY\_GCTL register fields shown in the following table.

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register

|   NUM_LINES | ABNUM   | DMY_CNT   | CMD_TYP[1]   |   CMD_TYP[0] | OE_EN   | OE_VAL   | Comment                                                                                     |
|-------------|---------|-----------|--------------|--------------|---------|----------|---------------------------------------------------------------------------------------------|
|           1 | N/A     | N/A       | N/A          |            0 | N/A     | N/A      | Read SFDP command in 1S-1S-1S mode with three ad- dress bytes and eight dummy clock cycles. |
|           1 | N/A     | N/A       | N/A          |            1 | N/A     | N/A      | Read SFDP command in 1D-1D-1D mode with three address bytes and eight dummy clock cycles.   |
|           2 | N/A     | N/A       | N/A          |            0 | N/A     | N/A      | Read SFDP command in 2S-2S-2S mode with three ad- dress bytes and eight dummy clock cycles. |
|           2 | N/A     | N/A       | N/A          |            1 | N/A     | N/A      | Read SFDP command in 2D-2D-2D mode with three address bytes and eight dummy clock cycles.   |
|           4 | N/A     | N/A       | 0            |            0 | N/A     | N/A      | Read SFDP command in 4S-4S-4S mode with three ad- dress bytes and eight dummy clock cycles. |
|           4 | N/A     | N/A       | 1            |            0 | N/A     | N/A      | Read SFDP command in 4D-4D-4D mode (quad DDR* - pre- sented as 4S-4D-4D in                  |

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register (Continued)

|   NUM_LINES | ABNUM   | DMY_CNT   | CMD_TYP[1]   |   CMD_TYP[0] | OE_EN   | OE_VAL   | Comment                                                                                                                                                                                                                    |
|-------------|---------|-----------|--------------|--------------|---------|----------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           4 | N/A     | N/A       | N/A          |            1 | N/A     | N/A      | Read SFDP command in 4D-4D-4D mode (quad DDR) with three address bytes and with eight dummy clock cycles.                                                                                                                  |
|           8 | 0/1     | 0/1       | N/A          |            0 | 0       | N/A      | Read SFDP command in 8S-8S-8S mode with three or four address bytes and with eight or 20 dummy clock cycles. The op- code is sent one time.                                                                                |
|           8 | 0/1     | 0/1       | N/A          |            0 | 1       | 1        | Read SFDP command in 8S-8S-8S mode with three or four address bytes and with eight or 20 dummy clock cycles. The op- code is sent two times and the second (com- mand exten- sion) is a logical negation of the first one. |
|           8 | 0/1     | 0/1       | N/A          |            0 | 1       | 0        | Read SFDP command in 8S-8S-8S mode                                                                                                                                                                                         |

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register (Continued)

|   NUM_LINES | ABNUM   | DMY_CNT   | CMD_TYP[1]   |   CMD_TYP[0] | OE_EN   |   OE_VAL | Comment                                                                                                                                                                                                                                                                                                                                                                       |
|-------------|---------|-----------|--------------|--------------|---------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|           8 | 0/1     | 0/1       | N/A          |            1 | N/A     |        0 | the first one. Read SFDP command in 8D-8D-8D mode with four address bytes and with eight or 20 dummy clock cycles. The opcode ex- tension is a rep- etition of the opcode. If three address bytes are selected, the controller sends the address over two clock cy- cles (meaning four bytes are sent due to oc- tal DDR). But, only the first three address bytes are valid. |
|           8 | 0/1     | 0/1       | N/A          |            1 | N/A     |        1 | Read SFDP command in 8D-8D-8D mode with four address bytes and with eight or 20 dummy clock cycles. The opcode                                                                                                                                                                                                                                                                |

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register (Continued)

| NUM_LINES   | ABNUM   | DMY_CNT   | CMD_TYP[1]   | CMD_TYP[0]   | OE_EN   | OE_VAL   | Comment                                                                                                                                                                                                                                         |
|-------------|---------|-----------|--------------|--------------|---------|----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|             |         |           |              |              |         |          | extension is the logical negation of the opcode. If three address bytes are select- ed controller sends the ad- dress over two clock cycles (meaning four bytes are sent due to octal DDR), but on- ly the first three address bytes are valid. |

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register (Continued)

|   NUM_LINES | ABNUM       | DMY_CNT        | CMD_TYP[1]     | CMD_TYP[0]     | OE_EN          | OE_VAL         | Comment                                                                                                                                                                                                                                                                                                                                                                                                                       |
|-------------|-------------|----------------|----------------|----------------|----------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|          12 | hf_bound_en | latency_cnt[4] | latency_cnt[3] | latency_cnt[2] | latency_cnt[1] | latency_cnt[0] | Read SFDP command in 8D-8D-8D mode for xSPI profile 2.0 with variable num- ber of latency clock cycles and enabled or disabled detec- tion of the in- ter pages cross- ing latency clock cycles. This value should be con- figured to N-1, where Nis the value expected by the memory device. If device ex- pects 16 laten- cy clock cycles then set 15 la- tency clock cy- cles as follow: DMY_CNT = 0 CMD_TYP = 3 OE_EN = 1 |
|          14 | N/A         | N/A            | 0/1            | 0/1            | N/A            | 0              | OE_VAL = 1 Read parameter page command in 1-1-1 mode for legacy SPI NAND device (at POR com- mand is sent to device first). CMD_TYP field/port is used to set                                                                                                                                                                                                                                                                 |

Table 21-29: Read SFDP Command Variations - XSPI\_DISCOVERY\_GCTL Register (Continued)

|   NUM_LINES | ABNUM   | DMY_CNT   | CMD_TYP[1]   | CMD_TYP[0]   | OE_EN   |   OE_VAL | Comment                                                                                                                                                                                        |
|-------------|---------|-----------|--------------|--------------|---------|----------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|             |         |           |              |              |         |          | XSPI_SEQ_G CTL1.PLANE _CNT parame- ter value.                                                                                                                                                  |
|          14 | N/A     | N/A       | 0/1          | 0/1          | N/A     |        1 | Read parameter page command in 1-1-1 mode for legacy SPI NAND device (at PoR com- mand is not send to device). CMD_TYP field/port is used to set XSPI_SEQ_G CTL1.PLANE _CNT parame- ter value. |

## SPI NAND Workflow

When connected to an SPI NAND memory device, the device discovery executes the following commands to read the parameter page database from the flash device:

1. Wait for MISCREG\_XSPI\_CTL1.RB\_VLD\_TM - this step is done only when device discovery is enabled at PoR. The RB\_VALID\_TIME is taken from the MISCREG\_XSPI\_CTL1 32-bit controller input.
2. Send 8'hFF reset command if MISCREG\_XSPI\_CTL0.EXTP\_VL bootstrap port is 0, otherwise the reset command is not sent - this step is done only when device discovery is enabled at PoR.
3. Wait for the device ready state by reading the OIP bit in the flash device (get feature command 8'h0F with one address byte 8'hC0) - this step is done only when device discovery is enabled at PoR and if MISCREG\_XSPI\_CTL0.EXTP\_VL - bootstrap port is 1'b0.
4. Read configuration register (get feature command 8'h0F with one address byte 8'hB0).
5. Configure bit 6th in the configuration register to enter parameter page read mode (configure feature command 8'h1F with one address byte 8'hB0).
6. Send read page 13h command with the block address of 24'h0001.
7. Wait for device ready state by reading the OIP bit in the flash device (get feature command 8'h0F with one address byte 8'hC0).
8. Send the read from cache 8'h0B command with the page address of 16'd0 and eight dummy clock cycles to fetch the parameter page signature.

9. Send the read from cache 8'h0B command with the page address of 16'd80 and eight dummy clock cycles to fetch the page size, page size extension and block size.
10. Restore the value in the configuration register read in step 3 (configure feature command 8'h1F with one address byte 8'hB0).
11. If a valid parameter page header was received, then configure the controller to SPI NAND mode.
12. Return status.

## Discovery Status

When device discovery is performed as part of controller initialization, the status can be interpreted by looking at the INIT\_FAIL controller output as follows.

NOTE: In the Discovery Status Parameters table, the controller's parameters related to CRC are updated to the values from the bootstrap ports (discovery\_crc\_*) except where invalid values of XSPI\_DISCOVERY\_GCTL.NUM\_LINES are configured, or where the controller failed to detect the device accurately.

Table 21-30: Discovery Status Parameters

|   XSPI_DISCOVERY_GCTL.IN HIBIT | XSPI_DISCOVERY_GCTL.NU M_LINES [3:0]   | Description (INIT_FAIL status is valid when INIT_PASS = 1)                                                                                                                                                                                                                                                                                         |
|--------------------------------|----------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                              1 | 0                                      | INIT_FAIL = 2'b00 - Device discovery is disabled Note that the PHY registers are still updated separately - see the section xSPI PHY Controller.                                                                                                                                                                                                   |
|                              1 | 1, 2, 4, 8 or 12                       | INIT_FAIL = 2'b00 - The SFDP database was not read. The controller was, however, configured to the selected mode.                                                                                                                                                                                                                                  |
|                              1 | 14                                     | INIT_FAIL = 2'b00 - The SFDP database was not read. The controller was, however, configured to SPI NAND mode.                                                                                                                                                                                                                                      |
|                              1 | 3, 5, 6, 7, 9, 10, 11, 13 or 15        | INIT_FAIL = 2'b00 - The SFDP data- base was not read. Invalid configuration of XSPI_DISCOVERY_GCTL.NUM_LINES . The controller registers remain unchanged.                                                                                                                                                                                          |
|                              0 | 0                                      | INIT_FAIL = 2'b00 - Device discovery enabled and detected an xSPI profile 1 device or profile 2 device during auto-detec- tion mode. INIT_FAIL = 2'b10 - Device discovery enabled and detected a legacy SPI device during auto-detection mode INIT_FAIL = 2'b01 - Device discovery enabled but did not detect a device during auto-detection mode. |

Table 21-30: Discovery Status Parameters (Continued)

|   XSPI_DISCOVERY_GCTL.IN HIBIT | XSPI_DISCOVERY_GCTL.NU M_LINES [3:0]   | Description (INIT_FAIL status is valid when INIT_PASS = 1)                                                                                                                                                                                                                                                                                                                      |
|--------------------------------|----------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                              0 | 1, 2, 4, 8 or 12                       | INIT_FAIL = 2'b00 - Device discovery enabled and detected an xSPI profile 1 device or profile 2 device during preconfig- ured detection mode. INIT_FAIL = 2'b10 - Device discovery enabled and detected a legacy SPI device during pre-configured detection mode. INIT_FAIL = 2'b01 - Device discovery enabled but did not detect a device during preconfigured-detection mode. |
|                              0 | 14                                     | INIT_FAIL = 2'b00 - Device discovery enabled and detected a legacy SPI NAND device during pre-configured detection mode. INIT_FAIL = 2'b01 - Device discovery enabled but did not detect a legacy SPI NAND device during preconfigureddetec- tion mode.                                                                                                                         |
|                              0 | 3, 5, 6, 7, 9, 10, 11, 13 or 15        | INIT_FAIL = 2'b01 - Invalid configuration on XSPI_DISCOVERY_GCTL.NUM_LINES . The controller registers remain unchanged.                                                                                                                                                                                                                                                         |

When device discovery is triggered from the registers, the same status can be interpreted using the XSPI\_DISCOVERY\_GCTL.FAIL field of the discovery control register.

|   XSPI_DISCOVERY_GCTL.IN HIBIT |   XSPI_DISCOVERY_GCTL.NU M_LINES [3:0] | Description ( XSPI_DISCOVERY_GCTL.FAIL status is valid when XSPI_DISCOVERY_GCTL.PASS = 1)                                                                                                                                                                                                                                                                                                              |
|--------------------------------|----------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                              0 |                                      0 | XSPI_DISCOVERY_GCTL.FAIL = 2'b00 - Device discov- ery enabled and detected an xSPI profile 1 device or profile 2 device during auto-detection mode. XSPI_DISCOVERY_GCTL.FAIL = 2'b10 - Device discov- ery enabled and detected a legacy SPI device during auto-detec- tion mode. XSPI_DISCOVERY_GCTL.FAIL = 2'b01 - Device discov- ery enabled but did not detect a device during auto-detection mode. |

|   XSPI_DISCOVERY_GCTL.IN HIBIT | XSPI_DISCOVERY_GCTL.NU M_LINES [3:0]   | Description ( XSPI_DISCOVERY_GCTL.FAIL status is valid when XSPI_DISCOVERY_GCTL.PASS = 1)                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------------------|----------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                              0 | 1, 2, 4, 8 or 12                       | XSPI_DISCOVERY_GCTL.FAIL = 2'b00 - Device discov- ery enabled and detected an xSPI profile 1 device or profile 2 device during preconfigured detection mode. XSPI_DISCOVERY_GCTL.FAIL = 2'b10 - Device discov- ery enabled and detected a legacy SPI device during pre-config- ured detection mode. XSPI_DISCOVERY_GCTL.FAIL = 2'b01 - Device discov- ery enabled but did not detect device during preconfigured detection mode. Presents how to configure device discovery module in precon- figured detection mode. |
|                              0 | 14                                     | XSPI_DISCOVERY_GCTL.FAIL = 2'b00 - Device discov- ery enabled and detected a legacy SPI NAND device during pre-configured detection mode. XSPI_DISCOVERY_GCTL.FAIL = 2'b01 - Device discov- ery enabled but did not detect a legacy SPI NAND device during preconfigured detection mode.                                                                                                                                                                                                                              |
|                              0 | 3, 5, 6, 7, 9, 10, 11, 13 or 15        | XSPI_DISCOVERY_GCTL.FAIL = 2'b01 - Invalid config- uration of XSPI_DISCOVERY_GCTL.NUM_LINES . The controller registers remain unchanged.                                                                                                                                                                                                                                                                                                                                                                              |
|                              1 | 1, 2, 4, 8 or 12                       | XSPI_DISCOVERY_GCTL.FAIL = 2'b00 - The SFDP database was not read. The controller is configured to the selected mode.                                                                                                                                                                                                                                                                                                                                                                                                 |
|                              1 | 14                                     | XSPI_DISCOVERY_GCTL.FAIL = 2'b00 - The SFDP database was not read. The controller is configured to SPI NAND mode.                                                                                                                                                                                                                                                                                                                                                                                                     |

## Discovery Restrictions

As defined in the JESD216 standard, flash devices can implement various parameter tables. But, only the JEDEC basic flash parameter table is mandatory. The following restrictions apply to xSPI profile 1 and profile 2 memory devices with a SFDP revision number is greater than or equal to 16'h0107:

- For xSPI profile 1 memory devices that are configured to any mode other than single, the SFDP database must implement the SCCR (status, control and configuration register) map for the xSPI memory devices parameter table. This implementation allows the controller to update its registers accurately. If the device does not implement the map fully, some of the sequence-related registers of the controller can get commonly used values instead.

- For xSPI profile 2 memory device, the SFDP database must implement the SCCR map for xSPI profile 2 memory devices parameter table. This implementation allows the controller to update its registers accurately. If the device does not implement the map fully, some of the sequence-related registers of the controller can get commonly used values instead.

## Discovery Limitations

Note the following device discovery limitations:

- The device discovery module must not be used to detect a device that is configured to operate in XIP mode.
- The device discovery module must not be used to detect the HyperRAM device. Only profile 1, profile 2 - HF and SPI NAND devices are supported for device discovery.

## Legacy Devices

When the number of dummy clock cycles for a read command (READ\_SEQ\_PL\_DUMMY\_CNT) parameter cannot be read from the device status/configuration register, the controller configures a default value for single, dual, quad modes or 16/20 for octal modes. The device discovery module ends with a 2'b10 status, but, with the wrong number of dummy clock cycles. In a similar way, if the number of dummy clock cycles for a status checking command (READ\_SEQ\_PL\_DEV\_RDY\_DUMMY\_CNT) cannot be read from the from the SFDP database, the device discovery module configures a default value. See the section Controller Reconfiguration.

## Device Specific Configuration

Macronix MX25 octal devices - To configure the controller to STR-OPI mode without trying to read SFDP database, device discovery must be run in not full discovery pre-configuration mode:

- XSPI\_DISCOVERY\_GCTL.INHIBIT = 1 or XSPI\_DISCOVERY\_GCTL.REQ\_TYP = 1

```
· XSPI_DISCOVERY_GCTL.NUM_LINES = 8 · XSPI_DISCOVERY_GCTL.ABNUM = 1 · XSPI_DISCOVERY_GCTL.DMY_CNT = 1 · XSPI_DISCOVERY_GCTL.CMD_TYP = 0 · XSPI_DISCOVERY_GCTL.OE_EN = 1 · XSPI_DISCOVERY_GCTL.OE_VAL = 1.
```

To configure the controller to DTR-OPI mode without trying to read SFDP database, device discovery must be run in not full discovery pre-configuration mode:

```
· XSPI_DISCOVERY_GCTL.INHIBIT = 1 or XSPI_DISCOVERY_GCTL.REQ_TYP · XSPI_DISCOVERY_GCTL.NUM_LINES = 8 · XSPI_DISCOVERY_GCTL.ABNUM = 1 · XSPI_DISCOVERY_GCTL.DMY_CNT = 1
```

- = 1

- XSPI\_DISCOVERY\_GCTL.CMD\_TYP = 1
- XSPI\_DISCOVERY\_GCTL.OE\_EN = 0 or 1
- XSPI\_DISCOVERY\_GCTL.OE\_VAL = 1

## Controller Reconfiguration

When device discovery completes successfully, the controller automatically updates various registers in the controller. The registers depend on the type of memory device attached. For profile 1 or SPI NAND devices, the registers used to define how the command (opcode), address, and data are transferred between the controller and device. The Device Discovery Parameter tables show the parameters in the device sequence registers that require updates.

Table 21-31: Device Discovery Parameters - Profile 1 or SPI NAND devices

| Detected/Selected Mode                      | Command Phase (*cmd_ios, *cmd_edge, *cmd_ext_en, *cmd_ext_val)   | Address Phase (*addr_ios, *addr_edge,*addr_cnt)   | Data Phase (*data_ios, *data_edge)   |
|---------------------------------------------|------------------------------------------------------------------|---------------------------------------------------|--------------------------------------|
| Single SDR                                  | 0,0, 0, I                                                        | 0*,0, 3(4)**                                      | 0*,0                                 |
| Single DDR                                  | 0,1, 0, I                                                        | 0*,1, 3(4)**                                      | 0*,1                                 |
| Dual SDR                                    | 1,0, 0, I                                                        | 1,0, 3(4)**                                       | 1,0                                  |
| Dual                                        | 1,1, 0, I                                                        | 1,1, 3(4)**                                       | 1,1                                  |
| Quad SDR                                    | 2,0, 0, I                                                        | 2,0, 3(4)**                                       | 2,0                                  |
| Quad DDR                                    | 2,1, 0, I                                                        | 2,1, 3(4)**                                       | 2,1                                  |
| Quad DDR                                    | 2,1, 1,N                                                         | 2,1, 3(4)**                                       | 2,1                                  |
| Octal SDR without command extension         | 3,0, 0, I                                                        | 3,0, 3(4)**                                       | 3,0                                  |
| Octal SDR with command ex- tension repeated | 3,0, 1, R                                                        | 3,0, 3(4)**                                       | 3,0                                  |
| Octal SDR with command ex- tension inverted | 3, 0, 1, I                                                       | 3, 0, 3(4)**                                      | 3, 0                                 |
| Octal DDR with command ex- tension repeated | 3, 1, 1, R                                                       | 3, 1, 4**                                         | 3, 1                                 |
| Octal DDR with command ex- tension inverted | 3, 1, 1, I                                                       | 3, 1, 4**                                         | 3, 1                                 |
| SPI NAND                                    | 0, 0, 0, R                                                       | 0, 0, 2                                           | 0, 0                                 |

NOTE: · * = single SDR/DDR mode, the number of IOs for address and data phases (for read and program commands) can, in many devices, be reprogrammed higher than what is shown in the table; but, the lower value is chosen as a result of device discovery.

- ** = the number of address bytes for read/program/erase sector commands is configured to four when:
- the device operates with four address bytes, or
- when SFDP indicates support for four address byte commands, or
- when octal DDR mode is detected (in which case only an even number of address bytes can be transferred during the address phase).
- In all other cases, the device discovery module configures the number of address bytes based on the number of address bytes that is used to detect the device.
- R = the command extension is repeated
- I = the command extension is inverted
- N = command value consists of the first half of the opcode (bits[7:4]) repeated twice, and command extension value consists of second half of the opcode (bits[3:0]) repeated twice. For example, if the attached memory requires 0x12 opcode for a program page operation, then PROG\_SEQ\_PL\_CMD\_VAL is configured to 8'h11 and PROG\_SEQ\_PL\_CMD\_EXT\_VAL is configured to 8'h22. This mode is presented as 4S-4D-4D in the JESD216 specification.

## Common Sequence Register Updates

For profile 1, profile 2 (HF) or SPI NAND devices, the Device Discovery Parameters table shows the common sequence parameters in the XSPI\_SEQ\_GCTL0 and XSPI\_SEQ\_GCTL1 registers. The registers are updated after discovery.

Table 21-32: Device Discovery Parameters - Profile 1, Profile 2(HF) or SPI NAND devices

| Parameter Name   | Full Discovery Process                                                                                                                                                      | Not Full Discovery Process                                                                                                                    |
|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| TYP              | • 0 when a legacy SPI or xSPI profile 1 device is detected • 1 when a legacy HyperFlash or xSPI profile 2 device is detected • 3 when a legacy SPI NAND device is detected. | • 0 for legacy SPI or xSPI profile 1 config- uration • 1 for legacy HyperFlash or xSPI profile 2 configuration • 3 for SPI NAND configuration |
| DATA_PER_ADDR    | • 0 when legacy SPI, xSPI profile 1 and SPI NAND devices are detected • 1 when a legacy HyperFlash or xSPI profile 2 device is detected                                     | • 0 for legacy SPI, xSPI profile 1, or SPI NAND configuration • 1 for legacy HyperFlash or xSPI profile 2 configuration                       |

Table 21-32: Device Discovery Parameters - Profile 1, Profile 2(HF) or SPI NAND devices (Continued)

| Parameter Name   | Full Discovery Process                                                                                                           | Not Full Discovery Process                                                                                                                                                                                                                |
|------------------|----------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PSIZ_PGM         | Value is read from the SFDP database or parameter page                                                                           | 4'd8 for legacy SPI or xSPI profile 1 config- uration 4'd9 for legacy HyperFlash or xSPI profile 2 configuration 4'd11 for SPI NAND configuration if XSPI_DISCOVERY_GCTL.ABNUM boot- strap port/register field is 1'b0. Otherwise, 4'd12. |
| PSIZ_RD          | For SPI NAND devices, the value is read from the parameter page. Otherwise, 4'd15 value is configured.                           | 4'd15 for profile 1 and profile 2 - HF 4'd11 for SPI NAND configuration if XSPI_DISCOVERY_GCTL.ABNUM boot- strap port/register field is 1'b0. Otherwise, 4'd12.                                                                           |
| DATA_SWAP        | 1 when the SFDP header is swapped in octal DDR mode. Otherwise, 0.                                                               | 1 for legacy SPI or xSPI profile 1 when octal DDR mode is configured, number of address bytes is 4, and number of dummy cycles is 20 1 for legacy HyperFlash or xSPI profile 2 configuration. Otherwise, 0.                               |
| CRC_EN           | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_EN after controller initialization process only                | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_EN after controller initialization process only                                                                                                                         |
| CRC_VAR          | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_VAR- IANT after controller initialization process only         | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_VAR- IANT after controller initialization process only                                                                                                                  |
| CRC_OE           | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_CHUNK after controller initialization process only             | Parameter is updated from bootstrap port signal DISCOVERY_SEQ_CRC_OE after controller initialization process only                                                                                                                         |
| CHUNK_SIZ        | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_CHUNK_SIZE after con- troller initialization process only    | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_CHUNK_SIZE after con- troller initialization process only                                                                                                             |
| UAL_CHUNK_EN     | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_UAL_CHUNK_EN after controller initialization process only    | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_UAL_CHUNK_EN after controller initialization process only                                                                                                             |
| UAL_CHUNK_CHK    | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_UAL_CHUNK_CHK af- ter controller initialization process only | Parameter is updated from bootstrap port signal DISCOV- ERY_SEQ_CRC_UAL_CHUNK_CHK af- ter controller initialization process only                                                                                                          |

Table 21-32: Device Discovery Parameters - Profile 1, Profile 2(HF) or SPI NAND devices (Continued)

| Parameter Name   | Full Discovery Process                                                                                  | Not Full Discovery Process                                                                    |
|------------------|---------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| PSIZ_EXT         | For SPI NAND devices, the value is read from the parameter page                                         | 64 value is configured if XSPI_DISCOVERY_GCTL.DMY_CNT = 1'b0. Otherwise, 128.                 |
| PCA_SIZ          | For SPI NAND devices when page size equals 4KB, 1'b1 value is set. Otherwise, 1'b0 value is configured. | Value is configured to the value of XSPI_DISCOVERY_GCTL.ABNUM field/ bootstrap port           |
| PLANE_CNT        | Value is configured to the value of XSPI_DISCOVERY_GCTL.CMD_TYP register field/bootstrap port           | Value is configured to the value of XSPI_DISCOVERY_GCTL.CMD_TYP register field/bootstrap port |
| PPER_BLOCK       | Value is read from the parameter page                                                                   | 6 (64 pages per block) value is configured                                                    |
| TCMS_EN          | 1'd0 value is configured                                                                                | 1'd0 value is configured                                                                      |

## Reset Sequence Parameters for Profile 1 Devices

For profile 1 devices, the reset sequence registers are used when a reset command is issued to the memory device. The parameters are identified in the XSPI\_RST\_SEQ\_CFG0 and XSPI\_RST\_SEQ\_CFG1 registers. The registers are updated as shown in the Reset Sequence Parameters - Profile 1 Devices table.

Table 21-33: Reset Sequence Parameters - Profile 1 Devices

| Parameter Name                       | Full Discovery Process                                                                        | Not Full Discovery Process                                                                         |
|--------------------------------------|-----------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| XSPI_RST_SEQ_CFG0.CMD0_EN            | 0 when the device supports the soft reset command - 0xF0                                      | Always 1                                                                                           |
| XSPI_RST_SEQ_CFG0.CMD0_EN            | Always 8'h66 command is configured                                                            | Always 8'h66 command is configured                                                                 |
| XSPI_RST_SEQ_CFG0.CMD1_VALUE         | When the device supports soft reset com- mand, 8'hF0. Otherwise, 8'h99 command is configured. | Always 8'h99 command is configured                                                                 |
| XSPI_RST_SEQ_CFG1.P1_DAT_VAL UE      | Always 8'hD0 value is configured                                                              | Always 8'hD0 value is configured                                                                   |
| XSPI_RST_SEQ_CFG0.DAT_EN             | Always 0                                                                                      | Always 0                                                                                           |
| XSPI_ERS_SEQ_CFG0.CMD_VALUE          | Read from the SFDP database                                                                   | 8'h20 command when XSPI_DISCOVERY_GCTL.ABNUM = 0, 8'h21 command when XSPI_DISCOVERY_GCTL.ABNUM = 1 |
| XSPI_ERS_SEQ_CFG1.P1_SECT_SI Z_VALUE | Read from the SFDP database                                                                   | Always 4'd12 (4kB)                                                                                 |
| XSPI_ERS_SEQ_CFG2.ERSA_P1_CM D_VALUE | Always 8'hC7 command is configured                                                            | Always 8'hC7 command is configured                                                                 |

## Erase Sequence Register Updates

For profile 1 devices, the erase sequence registers are used when an erase\_sector or erase\_all command is issued to the memory device. The parameters are identified in the XSPI\_ERS\_SEQ\_CFG0 , XSPI\_ERS\_SEQ\_CFG1 , and XSPI\_ERS\_SEQ\_CFG2 registers. The registers are updated as shown in the Erase\_Sector / Erase\_All Sequence Parameters table.

Table 21-34: Erase\_Sector / Erase\_All Sequence Parameters - Profile 1 Devices

| Parameter Name                       | Full Discovery Process                                                           | Not Full Discovery Process         |
|--------------------------------------|----------------------------------------------------------------------------------|------------------------------------|
| XSPI_ERS_SEQ_CFG0.CMD_VALUE          | 8'hD8 command is configured                                                      | 8'hD8 command is configured        |
| XSPI_ERS_SEQ_CFG1.P1_SECT_SI Z_VALUE | Calculated as follow: 2 N , where N=column address size + pages row address size | 5'd18 value is configured          |
| XSPI_ERS_SEQ_CFG2.ERSA_P1_CM D_VALUE | Always 8'hC7 command is configured                                               | Always 8'hC7 command is configured |

For SPI NAND devices, the erase sequence registers are used when an erase\_sector command is issued to the memory device. The parameters are identified in the XSPI\_ERS\_SEQ\_CFG0 and XSPI\_ERS\_SEQ\_CFG1 registers. The registers are updated as shown in the Erase\_Sector Sequence Parameters table.

Table 21-35: Erase\_Sector Sequence Parameters - SPI NAND Devices

| Parameter Name                       | Full Discovery Process                                                           | Not Full Discovery Process         |
|--------------------------------------|----------------------------------------------------------------------------------|------------------------------------|
| XSPI_ERS_SEQ_CFG0.CMD_VALUE          | 8'hD8 command is configured                                                      | 8'hD8 command is configured        |
| XSPI_ERS_SEQ_CFG1.P1_SECT_SI Z_VALUE | Calculated as follow: 2 N , where N=column address size + pages row address size | 5'd18 value is configured          |
| XSPI_ERS_SEQ_CFG2.ERSA_P1_CM D_VALUE | Always 8'hC7 command is configured                                               | Always 8'hC7 command is configured |

## Write Enable Sequence Register Updates

For profile 1 or SPI NAND devices, the write enable sequence parameters are identified in the XSPI\_WE\_SEQ\_CFG0 register. The registers are used when the controller needs to prepare the device for a program operation. The Write Enable Sequence Parameters table shows the parameters for full and not full discovery.

Table 21-36: Write Enable Sequence Parameters - Profile 1 and SPI NAND devices

| Parameter Name                 | Full Discovery Process             | Not Full Discovery Process         |
|--------------------------------|------------------------------------|------------------------------------|
| XSPI_WE_SEQ_CFG0.P1_CMD_VALU E | Always 8'h06 command is configured | Always 8'h06 command is configured |
| XSPI_WE_SEQ_CFG0.P1_EN         | Always1'b1 value is set            | Always1'b1 value is set            |

## Program Sequence Register Updates

For profile 1, profile 2 HyperFlash, or SPI NAND devices, the program sequence parameters are identified in the XSPI\_PROG\_SEQ\_CFG0 , XSPI\_PROG\_SEQ\_CFG1 , and XSPI\_PROG\_SEQ\_CFG2 registers. The registers

used when the controller needs to perform a program (write) operation. The parameters are shown in the Program Sequence Parameters tables.

Table 21-37: Program Sequence Parameters - Profile 1

| Detected/Select- ed Mode   | Parameter Name                   | Full Discovery Process                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Not Full Discovery Process                                                                                                    |
|----------------------------|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------|
| Single                     | XSPI_PROG_SEQ_CF G0.P1_CMD_VALUE | 8'h8E command is configured (1-8-8 with 4 address bytes) when the SFDP database indi- cates support for this command by device 8'h84 command is configured (1-1-8 with 4 ad-dress bytes) when SFDP database indi- cates support for this command by device 8'h3E command is configured (1-4-4 with 4 address bytes) when SFDP database indi- cates indicates support for command by de- vice 8'h34 command is configured (1-1-4 with 4 address bytes) when SFDP database indi- cates indicates support for this command by device 8'h12 command is configured (1-1-1 with 4 address bytes) when SFDP database indi- cates indicates support for this command by device Otherwise, 8'h02 command is configured (1-1-1 with 3 or 4 address bytes) | 8'h02 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 8'h12 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 |
| Single                     |                                  | 6'd0 value is configured                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 6'd0 value is configured                                                                                                      |
| Dual, Quad, Octal          | XSPI_PROG_SEQ_CF G0.P1_CMD_VALUE | 8'h12 command is configured when the SFDP database indicates support for this command by device Otherwise, 8'h02 command is configured                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | 8'h02 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 8'h12 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 |
| Dual, Quad, Octal          | XSPI_PROG_SEQ_CF G0.P1_DMY_CNT   | 6'd0 value is configured                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | 6'd0 value is configured                                                                                                      |

Table 21-38: Program Sequence Parameters - Profile 2 HF Devices

| Parameter Name                     | Full Discovery Process                | Not Full Discovery Process            |
|------------------------------------|---------------------------------------|---------------------------------------|
| XSPI_PROG_SEQ_CFG2.P2_MSK_CM D_MOD | Always 0 is configured                | Always 0 is configured                |
| XSPI_PROG_SEQ_CFG2.P2_BURST_ TYP   | Always 1 (linear burst) is set        | Always 1 (linear burst) is set        |
| XSPI_PROG_SEQ_CFG2.P2_TARGET       | Always 0 (memory space) is configured | Always 0 (memory space) is configured |

Table 21-38: Program Sequence Parameters - Profile 2 HF Devices (Continued)

| Parameter Name                     | Full Discovery Process                                                                                                                             | Not Full Discovery Process                                                                                                                         |
|------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| XSPI_PROG_SEQ_CFG2.P2_LATENC Y_CNT | Parameter is not updated after discovery pro- cedure since it is related with profile 2 - HR which is not supported by the device discovery module | Parameter is not updated after discovery pro- cedure since it is related with profile 2 - HR which is not supported by the device discovery module |

Table 21-39: Program Sequence Parameters - SPI NAND Devices

| Parameter Name                  | Parameter Value             |
|---------------------------------|-----------------------------|
| XSPI_PROG_SEQ_CFG0.P1_CMD_VALUE | 8'h02 command is configured |

## Read Sequence Register Updates

For profile 1, profile HyperFlash, or SPI NAND devices, read sequence registers are identified in XSPI\_READ\_SEQ\_CFG0 , XSPI\_READ\_SEQ\_CFG1 and XSPI\_READ\_SEQ\_CFG2 registers. The registers used when the controller needs to perform a read operation. The parameters are shown in the Read Sequence Parameters table.

Table 21-40: Read Sequence Parameters - Profile 1 Devices

| Detected / Selected Mode   | Parameter Name   | Full Discovery Process                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Not Full Discovery Process                                                                                                                                                                                                                                                                                                                                                                                  |
|----------------------------|------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Single                     | P1_CMD_VALUE     | When the SFDP database indicates supporting the command by device: 8'hCC command is configured (1-8-8 with 4 address bytes) 8'h7C command is configured (1-1-8 with 4 address bytes) 8'hEC command is configured (1-4-4 with 4 address bytes) 8'h6C command is configured (1-1-4 with 4 address bytes) 8'hBC command is configured (1-2-2 with 4 address bytes) 8'h3C command is configured (1-1-2 with 4 address bytes) 8'h0C command is configured (1-1-1 with 4 address bytes) Read command value for 1-8-8 mode with 3 or 4 address bytes is read from SFDP database and configured Read command value for 1-1-8 mode with 3 or 4 address bytes is read from SFDP database and configured Read command value for 1-4-4 mode with 3 or 4 address bytes is read from SFDP data- base and configured Read command value for 1-1-4 mode with 3 or 4 address bytes is read from SFDP database and configured Read command value for 1-2-2 mode with 3 or 4 address bytes is read from SFDP database and configured Read command value for 1-1-2 mode with 3 or 4 address bytes is read from SFDP database and configured | 8'h03 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 and XSPI_DISCOVERY_GCTL.DMY_CNT = 0 8'h13 command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 and XSPI_DISCOVERY_GCTL.DMY_CNT = 0 8'h0B command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 and XSPI_DISCOVERY_GCTL.DMY_CNT = 1 8'h0C command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 and XSPI_DISCOVERY_GCTL.DMY_CNT = 1 |
| Single                     | P1_DMY_CNT       | Number of dummy clock cycles is read from the SFDP database. For 8'0B and 8'h0C read commands 8 num- ber of dummy cycles are set.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | 0 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 0, 8 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 1.                                                                                                                                                                                                                                                         |

Table 21-40: Read Sequence Parameters - Profile 1 Devices (Continued)

| Detected / Selected Mode   | Parameter Name   | Full Discovery Process                                                                                                                                                           | Not Full Discovery Process                                                                                                                                                                                                                                                                                                                                                                                                               |
|----------------------------|------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Dual                       | P1_CMD_VALUE     | Read command is read from the SFDP data- base.                                                                                                                                   | 8'h0B command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 8'h0C command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1                                                                                                                                                                                                                                                                                                            |
| Dual                       | P1_DMY_CNT       | If possible, number of dummy clock cycles is read from the memory configuration/status register. Otherwise, the number of dummy clock cycles is read from the SFDP database.     | 8 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 0 10 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 1                                                                                                                                                                                                                                                                                       |
| Quad                       | P1_CMD_VALUE     | Read command is read from the SFDP data- base.                                                                                                                                   | 8'h0B command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0, 8'h0C command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1.                                                                                                                                                                                                                                                                                                          |
| Quad                       | P1_DMY_CNT       | If possible, the number of dummy clock cycles is read from the memory configuration/status register. Otherwise, the number of dummy clock cycles is read from the SFDP database. | 8 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 0                                                                                                                                                                                                                                                                                                                                                                 |
| Octal                      | P1_CMD_VALUE     | 8'h0C command is sconfigured when the SFDP database indicates supporting this com- mand by device. Otherwise, 8'h0B command is configured                                        | 8'hEC command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 XSPI_DISCOVERY_GCTL.DMY_CNT = 1, XSPI_DISCOVERY_GCTL.CMD_TYP [0] = 0 and XSPI_DISCOVERY_GCTL.OE_EN = 1 8'h EE command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT = 1 XSPI_DISCOVERY_GCTL.CMD_TYP [0] =1 8'h 0B command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 0 8'h0C command is configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 |

Table 21-40: Read Sequence Parameters - Profile 1 Devices (Continued)

| Detected / Selected Mode   | Parameter Name   | Full Discovery Process                                                                                                                                                                                                                                                                                                    | Not Full Discovery Process                                                                                                                         |
|----------------------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| Octal                      | P1_DMY_CNT       | If possible, number of dummy clock cycles is read from the memory configuration/status register. Otherwise, 16 dummy clock cycles are config- ured when 8 dummy clock cycles have been used for device detection, or, 20 dummy clock cycles are configured when 20 dummy clock cycles have been used for device detection | 16 dummy clock cycles are configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 0 20 dummy clock cycles is configured when XSPI_DISCOVERY_GCTL.DMY_CNT = 1 |

Table 21-41: Read Sequence Parameters - Profile 1 Devices (mode byte)

| Parameter Name                    | Full Discovery Process                                                                                                                                                                                                                                   | Not Full Discovery Process                                      |
|-----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------|
| XSPI_READ_SEQ_CFG1.P1_MB_DMY _CNT | If possible, (number of clock cycles to send the mode byte (mb_factor) is not greater than XSPI_READ_SEQ_CFG1.P1_MB_DMY _CNT ), the value equals the P1_MB_DMY_CNT - mb_factor. Otherwise, 0 is configured. mb_factor = 2*(3 -P1_ADDR_IOS- P1_ADDR_EDGE) | Value is computed in the same way as for full discovery process |
| XSPI_READ_SEQ_CFG1.P1_MB_EN       | If the basic flash parameter table indicates support for mode byte for the selected read command, then 1. Otherwise, 0.                                                                                                                                  | Always 1'b0                                                     |
| XSPI_XIP_GCTL.XIP_DIS_MB_VAL UE   | Always 8'hFF value                                                                                                                                                                                                                                       | Always 8'hFF value                                              |

## *When the address phase is in octal DDR mode, then mb\_factor = 1.

Table 21-42: Read Sequence Parameters - Profile 2 HF Devices

| Parameter Name                     | Full Discovery Process                                                                                                                                                                                                         | Not Full Discovery Process                                                                    |
|------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| XSPI_READ_SEQ_CFG2.P2_LATENC Y_CNT | If possible, the number of latency clock cy- cles is read from the device status/configura- tion register. Otherwise, the number of latency clock cy- cles is configured to the number that is used to read the SFDP database. | Number of latency clock cycles is configured to number that is used to read the SFDP database |

Table 21-42: Read Sequence Parameters - Profile 2 HF Devices (Continued)

| Parameter Name                     | Full Discovery Process                                                                                | Not Full Discovery Process                      |
|------------------------------------|-------------------------------------------------------------------------------------------------------|-------------------------------------------------|
| XSPI_READ_SEQ_CFG2.P2_HF_BOU ND_EN | Value is equal to the XSPI_DISCOVERY_GCTL.ABNUM for preconfigured mode, or, 1 for auto-detection mode | Value is equal to the XSPI_DISCOVERY_GCTL.ABNUM |
| XSPI_READ_SEQ_CFG2.P2_MSK_CM D_MOD | Always 0 is configured                                                                                | Always 0 is configured                          |
| XSPI_READ_SEQ_CFG2.P2_BURST_ TYP   | Always 1 (linear burst) is configured                                                                 | Always 1 (linear burst) is configured           |
| XSPI_READ_SEQ_CFG2.P2_TARGET       | Always 0 (memory space) is configured                                                                 | Always 0 (memory space) is configured           |

Table 21-43: Read Sequence Parameters - SPI NAND Devices

| Parameter Name                  | Parameter Value                     |
|---------------------------------|-------------------------------------|
| XSPI_READ_SEQ_CFG0.P1_CMD_VALUE | 8'h0B command is configured         |
| XSPI_READ_SEQ_CFG0.P1_DMY_CNT   | 8 dummy clock cycles are configured |

Table 21-44: Read Sequence Parameters for SPI-NAND Devices (mode byte)

| Parameter Name                             | Parameter Value                     |
|--------------------------------------------|-------------------------------------|
| XSPI_READ_SEQ_CFG1.P1_MB_EN                | 0                                   |
| XSPI_READ_SEQ_CFG1.P1_MB_DMY_CNT           | 0 dummy clock cycles are configured |
| XSPI_XIP_GCTL.XIP_DIS_MB_VALUE             | Always 8'hFF value                  |
| XSPI_READ_SEQ_CFG1.P1_CACHE_RANDOM_READ_EN | 0                                   |

## Status Sequence Register Updates

For profile 1, profile HyperFlash, or NAND devices, the status sequence registers are identified in the XSPI\_STAT\_SEQ \_CFG register. The registers are used when the controller needs to check the status of the memory device. The parameters are shown in the Status Sequence Parameters table.

Table 21-45: Status Sequence Parameters - Profile 1 Devices

| Parameter Name                          | Full Discovery Process                                                                                                                                                      | Not Full Discovery Process                                                                              |
|-----------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|
| XSPI_ERS_SEQ_CFG0.ADDR _CNT             | When possible, the number of address bytes is read from the SFDP database. Otherwise, 2 ad- dress bytes are configured for octal DDR mode and 1 address byte is configured. | Four address bytes are configured when XSPI_DISCOVERY_GCTL.ABNUM = 1 and XSPI_DISCOVERY_GCTL.DMY_CNT =1 |
| XSPI_STAT_SEQ_CFG1.P1_ ERS_FAIL_ADDR_EN | Address phase is enabled when the SFDP data- base indicates a requirement of an address phase                                                                               | Address phase is always disabled                                                                        |

Table 21-45: Status Sequence Parameters - Profile 1 Devices (Continued)

| Parameter Name                             | Full Discovery Process                                                                                                                                                         | Not Full Discovery Process                                                                                                                                          |
|--------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                                            | for reading the erase error bit. Otherwise, the address phase is disabled.                                                                                                     |                                                                                                                                                                     |
| XSPI_STAT_SEQ_CFG1.P1_ ERS_FAIL_DMY_CNT    | Number of dummy clock cycles is read from the SFDP database.                                                                                                                   | 8 dummy clock cycles are configured for octal modes. Otherwise, 1 dummy clock cycle is con- figured when the data phase is in DDR mode and 0 for other cases.       |
| XSPI_STAT_SEQ_CFG2.P1_ ERS_FAIL_CMD_VALUE  | Command is read from the SFDP database when it indicates support for an erase error bit. Other- wise, the 8'h05 command is configured.                                         | 8'h05 command is configured                                                                                                                                         |
| XSPI_STAT_SEQ_CFG1.P1_ PROG_FAIL_ADDR_EN   | Address phase is enabled when the SFDP da- tabase indicates the requirement of an address phase for reading the program error bit. Other- wise, the address phase is disabled. | Address phase is always disabled                                                                                                                                    |
| XSPI_STAT_SEQ_CFG1.P1_ PROG_FAIL_DMY_CNT   | Number of dummy clock cycles is read from the SFDP database                                                                                                                    | 8 dummy clock cycles are configured for octal modes. Otherwise, 1 dummy clock cycle is con- figured when the data phase is in DDR mode and 0 for other cases.       |
| XSPI_STAT_SEQ_CFG2.P1_ PROG_FAIL_CMD_VALUE | Command is read from the SFDP database when it indicates support of the program error bit. Otherwise, the 8'h05 command is configured.                                         | 8'h05 command is configured                                                                                                                                         |
| XSPI_STAT_SEQ_CFG1.P1_ DEV_RDY_ADDR_EN     | Address phase is enabled when the SFDP da- tabase indicates the requirement of an address phase for reading the RDY/ BUSY bit. Other- wise, the address phase is disabled.     | Address phase is enabled when XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT =1, XSPI_DISCOVERY_GCTL.CMD_TYP [0] = 0, and XSPI_DISCOVERY_GCTL.OE_EN = 1 |
| XSPI_STAT_SEQ_CFG1.P1_ ERS_FAIL_ADDR_EN    | Address phase is enabled when the SFDP da- tabase indicates the requirement of an address phase for reading the erase error bit. Otherwise, the address phase is disabled.     | Address phase is always disabled                                                                                                                                    |
| XSPI_STAT_SEQ_CFG1.P1_ ERS_FAIL_DMY_CNT    | Number of dummy clock cycles is read from the SFDP database                                                                                                                    | 8 dummy clock cycles are configured for octal modes. Otherwise, 1 dummy clock cycle is con- figured when the data phase is in DDR mode and 0 for other cases.       |
| XSPI_STAT_SEQ_CFG2.P1_ ERS_FAIL_CMD_VALUE  | Command is read from the SFDP database when it indicates support for the erase error bit. Other- wise, the 8'h05 command is configured.                                        | 8'h05 command is configured                                                                                                                                         |
| XSPI_STAT_SEQ_CFG1.P1_ PROG_FAIL_ADDR_EN   | Address phase is enabled when the SFDP da- tabase indicates the requirement of an address phase for reading the program error bit. Other- wise, the address phase is disabled. | Address phase is always disabled                                                                                                                                    |

Table 21-45: Status Sequence Parameters - Profile 1 Devices (Continued)

| Parameter Name                             | Full Discovery Process                                                                                                                                                     | Not Full Discovery Process                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| XSPI_STAT_SEQ_CFG1.P1_ PROG_FAIL_DMY_CNT   | Number of dummy clock cycles is read from the SFDP database                                                                                                                | 8 dummy clock cycles are configured for octal modes. Otherwise, 1 dummy clock cycle is con- figured when the data phase is in DDR mode and 0 for other cases.                                                                                                                                                                                                                                                                                           |
| XSPI_STAT_SEQ_CFG2.P1_ PROG_FAIL_CMD_VALUE | Command is read from the SFDP database when it indicates support for the program error bit. Otherwise, the 8'h05 command is configured.                                    | 8'h05 command is configured                                                                                                                                                                                                                                                                                                                                                                                                                             |
| XSPI_STAT_SEQ_CFG1.P1_ DEV_RDY_ADDR_EN     | Address phase is enabled when the SFDP da- tabase indicates the requirement of an address phase for reading the RDY/ BUSY bit. Other- wise, the address phase is disabled. | Address phase is enabled when XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT = 1, XSPI_DISCOVERY_GCTL.CMD_TYP = 0 and XSPI_DISCOVERY_GCTL.OE_EN = 1 or when XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT =1 and XSPI_DISCOVERY_GCTL.CMD_TYP [0] = 1 Otherwise, the address phase is disabled.                                                                                                                                 |
| XSPI_STAT_SEQ_CFG1.P1_ DEV_RDY_DMY_CNT     | Number of dummy clock cycles is read from the SFDP database.                                                                                                               | 4 dummy clock cycles are set if XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT = 1, XSPI_DISCOVERY_GCTL.CMD_TYP = 0 and XSPI_DISCOVERY_GCTL.OE_EN is 1 or if XSPI_DISCOVERY_GCTL.ABNUM = 1, XSPI_DISCOVERY_GCTL.DMY_CNT = 1 and XSPI_DISCOVERY_GCTL.CMD_TYP [0] = 1 else 8 dummy clock cycles are configured for other octal modes Otherwise, 1 dummy clock cycles is configured when data phase in DDR mode and 0 in other all other modes |
| XSPI_STAT_SEQ_CFG2.P1_ DEV_RDY_CMD_VALUE   | Command is read from the SFDP database when it supports the RDY/BUSY bit. Otherwise, the 8'h05 command is configured.                                                      | 8'h05 command is configured                                                                                                                                                                                                                                                                                                                                                                                                                             |

Table 21-46: Status Sequence Parameters - Profile 2 HF Devices

| Parameter Name                     | Parameter Value                                                                                                                                                                                                              | Not Full Discovery Process                                                                      |
|------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| XSPI_STAT_SEQ_CFG4.P2_LATENC Y_CNT | When possible, the number of latency clock cycles is read from the device status/configu- ration register. Otherwise, the number of la- tency clock cycles is configured to a number that is used to read the SFDP database. | Number of latency clock cycles is configured to a number that is used to read the SFDP database |
| XSPI_STAT_SEQ_CFG4.P2_MSK_CM D_MOD | Always 0                                                                                                                                                                                                                     | Always 0                                                                                        |

Table 21-47: Status Sequence Parameters

| Parameter Name                           | Parameter Value                                                                                                                          | Not Full Discovery Process                                   |
|------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|
| XSPI_STAT_SEQ_CFG5.PROG_FAIL _EN         | 1 when the SFDP database indicates support for the program error bit. Otherwise, 0.                                                      | Always 0                                                     |
| XSPI_STAT_SEQ_CFG5.PROG_FAIL _SIZ        | 1 when the data phase reflects octal DDR mode. Otherwise, 0.                                                                             | 1 when the data phase reflects octal DDR mode. Otherwise, 0. |
| XSPI_STAT_SEQ_CFG5.PROG_FAIL _VALUE      | Value is read from the SFDP database when it indicates support for the program error bit. Otherwise, 0.                                  | Always 0                                                     |
| XSPI_STAT_SEQ_CFG5.PROG_FAIL _IDX        | Value is read from the SFDP database when it indicates support for the program error bit. Otherwise, 0.                                  | Always 0                                                     |
| XSPI_STAT_SEQ_CFG8.PROG_FAIL _ADDR_VALUE | Value is read from the SFDP database when it indicates support for the program error bit. Otherwise, 0x00000000 address is con- figured. | Always 0x00000000 address is configured.                     |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_ EN          | 1 when the SFDP database indicates support for the program error bit. Otherwise, 0.                                                      | Always 0                                                     |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_ SIZ         | 1 when the data phase reflects octal DDR mode. Otherwise, 0.                                                                             | 1 when the data phase reflects octal DDR mode. Otherwise, 0. |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_ VALUE       | Value is read from the SFDP database when it indicates support for the erase error bit. Otherwise, 0.                                    | Always 0                                                     |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_ IDX         | Value is read from the SFDP database when it indicates support for the erase error bit. Otherwise, 0.                                    | Always 0                                                     |
| XSPI_STAT_SEQ_CFG9.ERS_FAIL_ ADDR_VALUE  | Value is read from the SFDP database if it indicatesupport for the erase error bit, Oth- erwise, 0x00000000 address is configured.       | Always 0x00000000 address is configured                      |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_E N           | Always 1                                                                                                                                 | Always 1                                                     |

Table 21-47: Status Sequence Parameters (Continued)

| Parameter Name                         | Parameter Value                                                                                                                                                                    | Not Full Discovery Process                                                     |
|----------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------|
| XSPI_STAT_SEQ_CFG5.DEV_RDY_S IZ        | 1 when the data phase reflects octal DDR mode. Otherwise, 0.                                                                                                                       | 1 when the data phase reflects octal DDR mode. Otherwise, 0.                   |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_V ALUE      | Value is read from the SFDP database if it indicates support for the RDY/BUSY bit, Otherwise, 0 for legacy SPI and xSPI profile 1 or 1 for legacy HyperFlash and xSPI pro- file 2. | 0 for legacy SPI and xSPI profile 1 1 for legacy HyperFlash and xSPI profile 2 |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_I DX        | Value is read from the SFDP database if it indicates support for the RDY/BUSY bit, Otherwise, 0 for legacy SPI and xSPI profile 1 or 7 for legacy HyperFlash and xSPI pro- file 2. | 0 for legacy SPI and xSPI profile 1 7 for legacy HyperFlash and xSPI profile 2 |
| XSPI_STAT_SEQ_CFG7.DEV_RDY_A DDR_VALUE | Value is read from the SFDP database if it indicates support for the RDY/BUSY bit, Otherwise, 0x00000000 address is config- ured.                                                  | Always 0x00000000 address is configured.                                       |

The following status sequence parameters are updated in the XSPI\_STAT\_SEQ\_CFGn registers for SPI NAND devices following device discovery.

Table 21-48: Status Sequence Parameters for SPI NAND Devices

| Parameter Name                            | Parameter Value                     |
|-------------------------------------------|-------------------------------------|
| XSPI_ERS_SEQ_CFG0.ADDR_CNT                | 0 (one address byte)                |
| XSPI_STAT_SEQ_CFG1.P1_ERS_FAIL_ADDR_EN    | 1                                   |
| XSPI_STAT_SEQ_CFG1.P1_ERS_FAIL_DMY_CNT    | 0 dummy clock cycles are configured |
| XSPI_STAT_SEQ_CFG2.P1_ERS_FAIL_CMD_VALUE  | 8'h0F command is configured         |
| XSPI_STAT_SEQ_CFG1.P1_PROG_FAIL_ADDR_EN   | 1                                   |
| XSPI_STAT_SEQ_CFG1.P1_PROG_FAIL_DMY_CNT T | 0 dummy clock cycles are configured |
| XSPI_STAT_SEQ_CFG2.P1_PROG_FAIL_CMD_VALUE | 8'h0F command is configured         |
| XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_ADDR_EN     | 1                                   |
| XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_DMY_CNT     | 0 dummy clock cycles are configured |
| XSPI_STAT_SEQ_CFG2.P1_DEV_RDY_CMD_VALUE   | 8'h0F command is configured         |
| XSPI_STAT_SEQ_CFG5.PROG_FAIL_SIZ          | 0                                   |
| XSPI_STAT_SEQ_CFG5.PROG_FAIL_VALUE        | 1                                   |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_IDX           | 3                                   |
| XSPI_STAT_SEQ_CFG8.PROG_FAIL_ADDR_VALUE   | 32'h00000C0 value is configured     |

Table 21-48: Status Sequence Parameters for SPI NAND Devices (Continued)

| Parameter Name                         | Parameter Value                 |
|----------------------------------------|---------------------------------|
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_EN         | 1                               |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_SIZ        | 0                               |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_VALUE      | 1                               |
| XSPI_STAT_SEQ_CFG5.ERS_FAIL_IDX        | 2.                              |
| XSPI_STAT_SEQ_CFG9.ERS_FAIL_ADDR_VALUE | 32'h00000C0 value is configured |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_EN          | 1                               |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_SIZ         | 0                               |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_VALUE       | 0                               |
| XSPI_STAT_SEQ_CFG5.DEV_RDY_IDX         | 0                               |
| XSPI_STAT_SEQ_CFG7.DEV_RDY_ADDR_VALUE  | 32'h00000C0 value is configured |
| XSPI_STAT_SEQ_CFG10.CRDY_VALUE         | 0                               |
| XSPI_STAT_SEQ_CFG10.CRDY_IDX           | 7                               |
| XSPI_STAT_SEQ_CFG10.ECC_FAIL_EN        | 0                               |
| XSPI_STAT_SEQ_CFG10.ECC_FAIL_VALUE     | 0                               |
| XSPI_STAT_SEQ_CFG10.ECC_FAIL_MSK       | 0                               |

## Boot Engine

xSPI controller provides an option to boot directly from the attached memory device using direct mode following controller reset. Alternatively, the controller can use a dedicated boot engine to transfer the boot code from the memory device to an area of host memory. The boot engine automatically triggers a page read command and probes the status of the transfer.

The boot engine assumes that the boot image is stored in the xSPI device connected to bank 0. The boot image is stored in the xSPI memory. It is composed of a configuration record and main data. The configuration record contains the following information:

- Main data size - the overall data size that need to be transferred during boot process
- Host address - the address on the host side where the boot image is transferred
- Image offset - the address inside the xSPI device from where the boot image is transferred
- SPI NAND page size - For NAND devices, this provides the SPI NAND device page size that is used to determine number of read iterations

After the boot process completes, the boot engine module writes the boot operation status. Only a single boot image copy is stored in the xSPI memory. This kind of memory is less prone to errors. So, no additional copies are required.

The figure shows the boot engine module functional diagram.

Figure 21-7: Boot Engine Flow

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000006_6f7d778097d1904d2f4ce67435c5c3af60765185e5abb61399ec5dc90fedc86d.png)

## Configuring the Boot Engine

The boot engine is enabled by writing 1 to MISCREG\_XSPI\_CTL0.BT\_EN field before doing a controller reset. Once reset is done, the controller fetches a 256-bit configuration record stored in the memory device. This record should have been previously stored in the memory device from address 0x0. The Boot Engine Configuration Record Layout figure provides the details of the required layout of the configuration record.

- Main data size - exists at memory device address 0x00. The lower 32 bits contain the size of the boot record information, the higher 32 bits are reserved.
- Image offset - exists at memory device address 0x08. The lower 48 bits contain the starting address of the boot image within the memory device. The higher 16 bits are reserved.
- Host address - exists at memory device address 0x10. The whole 64 bits contain the host address indicating where the boot image is copied.
- SPI NAND page size - exists at memory device address 0x18. It provides SPI NAND device page size which is used to determine the number of read iterations. Information is stored in the first 16 bits. The upper 48 bits are reserved.

Figure 21-8: Boot Engine Configuration Record Layout

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000007_05d886b14be5625cbefa9b03fc26797cbe09eb38b6007b30b1a373fe1994be60.png)

The register fields table shows the bits used by the boot engine to select valid commands and manage data transfer between the xSPI device and host memory. Register content is established by the device discovery process or register field reset vectors. The boot engine assumes that the information in these control registers are valid when boot process is triggered.

The following configuration register fields are used in the boot process:

Table 21-49: Registers for Boot

| Register           | Bit                                                                                                                                                                                                                                                                                                     |
|--------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| XSPI_SEQ_GCTL0     | - XSPI_SEQ_GCTL0.TYP - XSPI_SEQ_GCTL0.DAT_SWAP - XSPI_SEQ_GCTL0.UAL_CHUNK_CHK - XSPI_SEQ_GCTL0.UAL_CHUNK_EN - XSPI_SEQ_GCTL0.CHUNK_SIZ - XSPI_SEQ_GCTL0.CRC_OE - XSPI_SEQ_GCTL0.CRC_VAR - XSPI_SEQ_GCTL0.CRC_EN - XSPI_SEQ_GCTL0.DAT_PER_ADDR                                                           |
| XSPI_XIP_GCTL      | - XSPI_XIP_GCTL.XIP_DIS_MB_VALUE                                                                                                                                                                                                                                                                        |
| XSPI_READ_SEQ_CFG0 | - XSPI_READ_SEQ_CFG0.P1_DMY_CNT - XSPI_READ_SEQ_CFG0.P1_DAT_EDGE - XSPI_READ_SEQ_CFG0.P1_DAT_IOS - XSPI_READ_SEQ_CFG0.P1_ADDR_EDGE - XSPI_READ_SEQ_CFG0.P1_ADDR_IOS - XSPI_READ_SEQ_CFG0.P1_ADDR_CNT - XSPI_READ_SEQ_CFG0.P1_CMD_EDGE - XSPI_READ_SEQ_CFG0.P1_CMD_IOS - XSPI_READ_SEQ_CFG0.P1_CMD_VALUE |

Table 21-49: Registers for Boot (Continued)

| Register                                      | Bit                                                                                                                                                                                                                                                                      |
|-----------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| XSPI_READ_SEQ_CFG1                            | - XSPI_READ_SEQ_CFG1.P1_CMD_EXT_EN - XSPI_READ_SEQ_CFG1.P1_CMD_EXT_VALUE - XSPI_READ_SEQ_CFG1.P1_MB_EN - XSPI_READ_SEQ_CFG1.P1_MB_DMY_CNT                                                                                                                                |
| XSPI_READ_SEQ_CFG2                            | - XSPI_READ_SEQ_CFG2.P2_LATENCY_CNT - XSPI_READ_SEQ_CFG2.P2_HF_BOUND_EN - XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD - XSPI_READ_SEQ_CFG2.P2_BURST_TYP - XSPI_READ_SEQ_CFG2.P2_TARGET                                                                                             |
| XSPI_STAT_SEQ_CFG0 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG0.P1_DAT_EDGE - XSPI_STAT_SEQ_CFG0.P1_DAT_IOS - XSPI_STAT_SEQ_CFG0.P1_CMD_EXT_EN - XSPI_STAT_SEQ_CFG0.P1_CMD_EDGE - XSPI_STAT_SEQ_CFG0.P1_CMD_IOS - XSPI_STAT_SEQ_CFG0.P1_ADDR_EDGE - XSPI_STAT_SEQ_CFG0.P1_ADDR_IOS - XSPI_STAT_SEQ_CFG0.P1_ADDR_CNT |
| XSPI_STAT_SEQ_CFG1 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_ADDR_EN - XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_DMY_CNT                                                                                                                                                                                          |
| XSPI_STAT_SEQ_CFG2 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG2.P1_DEV_RDY_CMD_VALUE                                                                                                                                                                                                                                |
| XSPI_STAT_SEQ_CFG3 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG3.P1_DEV_RDY_CMD_EXT_VALUE                                                                                                                                                                                                                            |
| XSPI_STAT_SEQ_CFG5 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG5.DEV_RDY_SIZ - XSPI_STAT_SEQ_CFG5.DEV_RDY_VALUE - XSPI_STAT_SEQ_CFG5.DEV_RDY_IDX                                                                                                                                                                     |
| XSPI_STAT_SEQ_CFG7 (only for SPI NAND device) | - XSPI_STAT_SEQ_CFG7.DEV_RDY_ADDR_VALUE                                                                                                                                                                                                                                  |

## xSPI PHY Controller

The PHY handles the low-level timings of the data, address, and control signals between the device and controller. The PHY has its own programmable register group.

## PHY Initialization

Before the host can communicate with the memory device, software must initialize the PHY and lock it. The soft PHY contains three synthesizable delay lines (DLLs) that are built from 256 delay elements. The first delay line is the controller delay line and it measures how many delay elements are needed to match the XSPI\_CLK clock

period. The other two delay lines are targets, one for the transmission TX outbound path and the other for the reception RX inbound path.

The TX target delay line can use the controller delay line measurements to influence how much delay to add to transmitted data based on a percentage of the clock period. This improves the eye diagram (a common indicator of the quality of signals in high-speed digital transmissions) of the memory device. Similarly, the RX target delay line can shift the read sampling clock as a percentage of the clock period to find the ideal sampling point for the inbound read data. Before this can happen however, the controller delay line must be configured and this is done immediately after power on reset (PoR). This is PHY locking, a fully internal PHY function that attempts to lock the bus requestor delay line. Until the PHY is locked, there can be no access to or from the memory device.

When choosing to disable the controller delay line (that is, saturation mode), the target delay line cannot be programmed based on a percentage of the clock period and instead must be manually configured. This is suitable for when operating the PHY at slower data rates, when the controller delay line does not have enough delay elements to measure the complete clock period. Running at these slower rates does not require fine tuning of the target delay lines (when running at SDR) and manual configured is acceptable.

- NOTE: PHY locking should not be confused with PHY training. PHY training is a software application (not part of the controller deliverables) that can be implemented to help tune the PHY to find the pbest sampling point and allow the highest possible XSPI\_CLK frequency. The controller does not implement PHY training.

To lock the controller delay line and have the general PHY ready to transfer data, some of the PHY registers must be programmed with appropriate values that are dependent on the XSPI\_CLK frequency. The PHY runs at the same frequency as the attached memory device clock and uses system parameters such as board skew, pad delays, and so forth. Failure to program suitable values can result in the PHY inability to lock and unable to properly wake up.

## Initial PHY Registers to Configure

The PHY registers that must be initially configured before attempting to lock the PHY are described in the following sections:

- XSPI\_PHY\_DLL\_CTL Register
- XSPI\_PHY\_DQ\_TR Register
- XSPI\_PHY\_DQS\_TR Register
- XSPI\_PHY\_GATE\_LPBK\_CTL Register
- XSPI\_PHY\_DLL\_REQ\_CTL Register
- XSPI\_PHY\_DLL\_COMP\_CTL Register

## XSPI\_PHY\_DLL\_CTL Register

This register generally controls how the PHY target delay lines are routinely updated to compensate for drift caused by dynamic factors such as temperature or open-circuit voltage (OCV). It also contains register settings that are dependent on the attached memory device and operating mode.

The XSPI\_PHY\_DLL\_CTL.SDR\_EDGE\_ACT [21:21] bit controls on which clock edge the PHY controller transmits write data. It is only required for SDR operation and when the PHY controller defaults to propagating write data on the clock rising edge. Generally keep at the default state of zero, unless the memory device (in SDR operation) is unable to sample the data.

The XSPI\_PHY\_DLL\_CTL.DQS\_LST\_DAT\_DRP\_EN [20:20] bit must only be set to one, when all of the following conditions are met:

- The attached memory device issues read data on the negative clock edge.
- The memory device supports DQS (transfer two data words per clock cycle) and the PHY has been set up to sample read data using that DQS signal. The DQS that the memory device sends contains one extra edge than normal and the PHY must compensate for the added edge.

## XSPI\_PHY\_DQ\_TR Register

This register controls the timing of the main data (DQ) bus as it switches from the PHY to the memory device. Adjustments can be made to control when the attached DQ pads switch direction (that is, to add delay when the output enable switches from input to output and from output to input). There is also an option to define additional latency on the data bus itself, in units of one-half clock cycle.

NOTE: The XSPI\_PHY\_DQ\_TR.DAT\_OE\_END field (which defaults to 0x2 and controls when the output enable switches from output to input) must always be configured to a minimum value of 0x1. This register does not influence the locking procedure.

## XSPI\_PHY\_DQS\_TR Register

This register determines the read data sampling method used by the PHY. See PHY Read Sampling Method Selection for more details.

CAUTION: This register must be updated to reflect the read data sampling method to successfully read any data from the device.

## XSPI\_PHY\_GATE\_LPBK\_CTL Register

This register controls the gate for read data sampling to occur and enables the loopback BIST test modes within the PHY. There are fields that must be set up prior to locking. When the register fields are not mentioned in this section, the field can be left in its default state.

- XSPI\_PHY\_GATE\_LPBK\_CTL.GATE\_CFG - Use when the PHY is not using DQS from the device to sample read data. (Determined by the read data sampling method described in PHY Read Sampling Method Selection.) PHY uses information from the controller to determine which cycle the read data should be returning from the device, and when it should be sampling the DQ bus. When the round-trip delay of the entire read operation (based on WRITE CLK PAD delay + PCB trace to device + PCB trace from device + DQ PAD delay) exceeds one clock cycle, then the PHY must also delay the cycle in which it samples that data.

When DQS from the device is not used to sample read data, the

XSPI\_PHY\_GATE\_LPBK\_CTL.GATE\_CFG field = floor (round-trip delay in ns / clock period in ns)

- XSPI\_PHY\_GATE\_LPBK\_CTL.RD\_DEL\_SEL = ceiling (round-trip delay in ns / clock period in ns) + 2

## XSPI\_PHY\_DLL\_REQ\_CTL Register

CAUTION: With a small value, the initial lock time is longer. Values less than 0x04 may cause no lock by the delay line.

Use these register fields to configure and direct the controller delay line, prior to the PHY locking.

- XSPI\_PHY\_DLL\_REQ\_CTL.DLL\_BYPASS\_MODE - When set (=1) (default), it disables the controller delay line and the PHY always locks immediately. This forces saturation mode . When the user wants the PHY to operate in saturation mode, this field should be set and the PHY requires no further effort to lock.
- XSPI\_PHY\_DLL\_REQ\_CTL.PHASE\_DT\_SEL - This defaults to zero, but Analog Devices recommends it be configured to 0x2.
- XSPI\_PHY\_DLL\_REQ\_CTL.DLL\_START\_PT - The controller delay line can take some time to measure the clock period and then signal that the lock was achieved. It must start from a fixed number of delay elements and either add or remove delay elements until it finds the appropriate delay matching the clock period. Configuring this field can speed up the locking time. This field should be configured such that it is not greater than 7/8 ths of the clock period (use the worst-case element delay).

Example: When the clock frequency is 200 MHz (5 ns cycle time) and a worst-case element delay of 80 ps, this field equals [5 * (7/8) / .080 = 54] elements. When the locking time is not a concern, a small value such as 0x04 can be used instead to ensure that the delay line does not lock on a harmonic.

## XSPI\_PHY\_DLL\_COMP\_CTL Register

This register controls the TX and RX target delay lines. When the PHY is operating in saturation mode, the number of delay elements can be manually programmed for both delay lines. Otherwise, each delay line can be precisely delayed by steps (1/256 th of the clock period). The XSPI\_PHY\_DLL\_COMP\_CTL register does not need to be updated to initially lock the PHY, but it must be given appropriate values for the memory device to be able to sample the data sent by the PHY, and for the PHY to sample the data sent by the device.

On the transmit side, the DQ line can be delayed using the TX target delay line. On the receive side, the sampling clock (which depends on the chosen read data sampling method described in PHY Read Sampling Method Selection) is delayed by the RX target delay line. Choose initial values for these fields-setup times, skew between DQS and the data, board delays, and so forth-all affect where the eye diagram begins.

Initial values for the RX target delay line can be calculated using the steps found in the following section. The delivered script automatically calculates the required values, but these are for additional guidance.

IMPORTANT: Performing a full training sequence is the only way to calculate the most optimum delay value to find the center of the eye diagram.

## Initial Value Selection for XSPI\_PHY\_DLL\_COMP\_CTL Register

1. Express the quarter\_cycle\_period in picoseconds. For example, when the clock period is 10 ns, configure this field equal to 2500 ps.

2. For the delay\_of\_each\_element variable, calculate the expected delay through each implemented PHY delay element in ps. The delay elements within the PHY are made up of two NAND2 gates as selected in the hand instantiated cells. For example, when the delay through each NAND2 is 12 ps, the delay element = 24 ps.
3. Determine the max\_dqs\_dq\_skew (in ns) maximum skew between the DQS strobe and the DQ data obtained from the memory device datasheet tdqsq value.
4. The max\_dqs\_dq\_skew\_perclk skew as a percentage of the clock equals (max\_dqs\_dq\_skew) divided by the clock period.
5. The rx\_setup time in ns is the time the device takes to drive the read data following the edge of the clock. This time is specified in the memory device datasheet. Note there are specific times for SDR and for each edge to valid data in DDR. Pick the largest value (worst case).
6. Calculate the rx\_setup\_gt\_half value when rx\_setup is greater than one-half the clock cycle. Determine the difference between these two values in nanoseconds. Formula: rx\_setup\_gt\_half = rx\_setup -(clock\_period / 2).
7. Calculate the rx\_setup\_gt\_full value when rx\_setup+4 is greater than a clock cycle. Determine the difference between these two values in nanoseconds. Formula: rx\_setup\_gt\_full = rx\_setup + 4 -clock\_period.
8. Calculate the tx\_setup\_de value. Let tx\_setup be the setup time in ps, that the target memory device requires before it samples write data. This is often referred to as tDVCH in the device data sheet. Divide the tx\_setup time by the delay of each element to get the number of delay elements needed to cover the set up.
9. Calculate the tx\_skew\_margin\_de value for the TX skew margin. Let quarter\_cycle\_de be the quarter\_cycle\_period variable divided by the delay of each element to get the number of delay elements needed to cover the ¼ of the clock period. Divide again by five to get the number of delay elements needed to cover 1/20 th of the clock period. This value is chosen for the TX skew margin.

## Value Selection for RX Target Delay Lines

Use the RX Target Delay Line Parameters table to configure initial values for the RX target delay line.

Table 21-50: RX Target Delay Line Parameters

| Parameters                                                                               | Initial Value to Store in RX Delay Line Field                |
|------------------------------------------------------------------------------------------|--------------------------------------------------------------|
| Saturation Mode = 1 Using DQS from device                                                | ceiling ((max_dqs_dq_skew*1000) / delay_of_each_element) + 1 |
| Saturation Mode = 1 Using Internal or External Loopback DQS rx_setup ≤ half clock period | 1                                                            |

Table 21-50: RX Target Delay Line Parameters (Continued)

| Parameters                                                                                                           | Initial Value to Store in RX Delay Line Field                                                                                                                                                                                |
|----------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Saturation Mode = 1 Using Internal or External Loopback DQS rx_setup > half clock period rx_setup + 4 ≤ clock period | Setup time is quite large relative to the clock period. Need to delay initial sample point accordingly. ceiling ((rx_setup_gt_half*1000) / delay_of_each_element) + 1                                                        |
| Saturation Mode = 1 Using Internal or External Loopback DQS rx_setup + 4 > clock period                              | Setup time is very large relative to the clock period. For SDR modes, Analog Devices recommends setting SDR_EDGE_ACT to force the capture edge to be negative. ceiling ((rx_setup_gt_full*1000) / delay_of_each_element) + 1 |
| Saturation Mode = 0 Using DQS from device max_dqs_dq_skew_perclk > 1                                                 | 255                                                                                                                                                                                                                          |
| Saturation Mode = 0 Using DQS from device max_dqs_dq_skew_perclk ≤ 1                                                 | max_dqs_dq_skew_perclk * 256 + 0.01*256                                                                                                                                                                                      |
| Saturation Mode = 0 Using Internal or External Loopback DQS rx_setup ≤ half clock period                             | 0                                                                                                                                                                                                                            |
| Saturation Mode = 0 Using Internal or External Loopback DQS rx_setup > half clock period rx_setup + 4 ≤ clock period | ceiling (256*(rx_setup_gt_half/clk_period)) + 25; Adding 25 extra elements to guarantee minimal skew.                                                                                                                        |
| Saturation Mode = 0 Using Internal or External Loopback rx_setup + 4 > clock period                                  | ceiling (256*(rx_setup_gt_full/clk_period)) + 25; Adding 25 extra elements to guarantee minimal skew.                                                                                                                        |

CAUTION: The maximum value that can be programmed for the controller delay line is 255. When any calculation exceeds 255, configure the value to 255.

## Value Selection for TX Target Delay Lines

Use the TX Target Delay Line Parameters table to configure initial values for the TX target delay line.

Table 21-51: TX Target Delay Line Parameters

| Parameters                                                                                                                                       | Initial Value to Store in TX Delay Line Field       |
|--------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------|
| Saturation Mode = 1 flash setup is greater than quarter of cycle - skew margin ( tx_setup_de > (quarter_cycle_de - tx_skew_margin_de) )          | 0x5                                                 |
| Saturation Mode = 1 flash setup is less than or equal to quarter of cycle - skew margin ( tx_setup_de ≤ (quarter_cycle_de - tx_skew_margin_de) ) | Set to 25% That is, 25% ceiling(quarter_cy- cle_de) |
| Saturation Mode = 0 flash setup is greater than quarter of a cycle ( tx_setup_de > quarter_cycle_de )                                            | 0x15                                                |
| Saturation Mode = 0 flash setup is less than or equal to quarter of cycle ( tx_setup_de = quarter_cycle_de )                                     | 0x33                                                |

CAUTION: The maximum value that can be programmed for the controller delay line is 255. When any calculation exceeds 255, configure the value to 255.

## PHY Re-initialization

Once the host is operational, it can update the PHY registers and trigger a re-initialization or locking procedure using bits 24 and 25 of the XSPI\_PHY\_DLL\_CTL register. A re-initialization must be performed whenever the PHY registers are modified or when the user has changed the XSPI\_CLK frequency.

## Resynchronization of Target Delay Lines

While the PHY is running, the target delay lines can still occasionally require resynchronization as a result of temperature changes, OCV, and how much that change alters the delay through the delay elements. The frequency at which this resync needs to happen depends on how much change those dynamic factors can make on the delay through the delay line and whether that change is sufficient that the sampling point is no longer within the valid data window.

There is no need to resync when the DLL\_LOCK\_VALUE has not changed or has only changed by a small amount. Analog Devices recommends that software monitors the XSPI\_PHY\_DLLOB0.DLL\_LOCK\_VALUE field and triggers a resync when the XSPI\_PHY\_DLLOB0.DLL\_LOCK\_VALUE has changed by a significant amount since the last resync (for example, 10%). Alternatively, a user can choose to resync periodically, often enough to mitigate these changes in delay, but which does not impact overall performance.

During a resynchronization, the memory device cannot be accessed. For automatic recurring delay line resynchronizations, configure the XSPI\_PHY\_DLL\_UPDT\_CNT.RESYNC\_CNT field to a value other than 0x0. The value defines the time interval between each resync in xspi\_clk cycles .

CAUTION: The memory interface must be idle during resync operation.

When a resync happens while the controller is communicating with the memory device, the resync is automatically withheld until the memory operation completes. Similarly, when a resync is already in progress, the controller withholds any normal operation triggered by the host until that resync completes. The XSPI\_PHY\_DLL\_CTL.RESYNC\_IDLE\_CNT field defines the wait time after a resync operation completes when normal traffic can resume.

## PHY Read Sampling Method Selection

To use the PHY, decide which read sampling mechanism to use. Essentially, there are two options the user can select from when using the PHY:

- DQS Mode for PHY Read Sampling
- Non-DQS Mode (Loopback) for PHY Read Sampling

## DQS Mode for PHY Read Sampling

The highest device clock frequencies and, therefore, performance is achieved when the memory device drives DQS. DQS is an independent signal that is driven parallel to the read data and acts as a source synchronous clock from the device to the PHY. The PHY takes the DQS from the device, passes it through the receive delay line (for programmable incremental delay steps) and uses the resulting signal as a strobe to sample incoming read data. To support this option, the memory device being targeted must support DQS. Additionally, a physical hardware DQS pin connection must be provided. Ensure that bit 20 of the XSPI\_PHY\_DQS\_TR register is cleared (=0).

## Non-DQS Mode (Loopback) for PHY Read Sampling

The memory device sometimes does not supply DQS or lacks DQS hardware support on board. The alternative option is to mimic the data sampling strobe, referred to as the DQS loopback options of the PHY. Generally, this mode is referred as on-DQS (loopback) mode.

## Non-DQS Loopback Mode With a Pad

Internal loopback through the pad\_mem\_rebar\_t pad . This still requires the integrator to implement the pad\_mem\_rebar\_t pad , although it is not bonded out to a pin. This option can be chosen when dynamic factors like OCV are not important (perhaps because the operating frequency is slower or only SDR operation is needed). This loopback function is performed within the pad and includes pad delays. Depending on the chosen pads, pad delays can be significant, so including these in the loopback path can be important. To support this option, the user must implement the pad\_mem\_rebar\_t pad , but it is not connected to anything on the board. Bits 20 and 21 of the XSPI\_PHY\_DQS\_TR register must be set (=1). Bit 22 of the XSPI\_PHY\_DQS\_TR register must be cleared (=0).

Figure 21-9: Logic Diagram for Non-DQS (Loopback) Mode With Pad

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000008_2662bd92886e9ca43cfc2c997703a2136f329263d870752e738ad1d264094a6d.png)

## Memory Device Data Integrity

Some memory devices support advanced data integrity checking functions (for example, Macronix MX25). The controller is compatible with the following device capabilities.

## ECC Data Integrity

Some memory devices have built-in Error Checking and Correcting (ECC). During a page program operation, the device automatically creates the ECC. During a read operation, the device checks and when necessary, automatically corrects errors. These devices can output an ECC status signal (ECS#) to signal the ECC correction status. The status is optionally passed to the controller using the XSPI\_DFI\_GP\_OPEN\_DRAIN input, which can be visible with the interrupt status register.

## CRC Parity Check

Some memory devices have built-in cyclic redundancy check (CRC) capabilities. For write operations, the controller must send the CRC code to the device for checking. For read operations, the device must send the CRC code and the code must be validated using the controller.

The controller can generate the CRC on the address and calculate it with an exclusive-OR of the address bytes. This is sent to the device immediately after the address bytes. The controller can also generate a CRC on a chunk (programmable number of bytes ranging from 8 bytes through 512 bytes) of the write data. A CRC code is inserted into the outbound data stream immediately following that data chunk. Configuring the controller to use CRC depends on the active work mode. The CRC on the data bytes is calculated by exclusively-ORing the data bytes within a chunk.

## CRC Using Direct or ACMD Mode

CRC is enabled using the XSPI\_SEQ\_GCTL0 [8:8] bit. It can be configured to always assume that a CRC code is immediately present after the address bytes, or when it is to be inserted into the data stream after a chunk of bytes (independent of whether the stream is sending address bytes or data bytes). This operation is controlled using bit nine of the same register. Some devices require that the CRC be sent first with the CRC value and then repeated with an inverted value. Enable this option using the XSPI\_SEQ\_GCTL0 [10:10] bit. Control the chunk size (number of bytes protected by the CRC) with the XSPI\_SEQ\_GCTL0 [14:12] field. When the first CRC code

protects the address and is part of the data bytes, bit 16 should be set (=1). When checking that the first CRC code for read data is required, then bit 17 must also be set.

## CRC Using STIG Mode

The parameters defined in the XSPI\_SEQ\_GCTL0 register apply also for the direct and ACMD modes available directly in the glued data STIG instruction described in the Glued Data Instruction section. Enable the CRC using the XSPI\_SEQ\_GCTL0.CRC\_EN field. Control the chunk size (number of bytes protected by the CRC) using the XSPI\_SEQ\_GCTL0.CHUNK\_SIZ field. When the first CRC code protects the address and part of the data bytes, the XSPI\_SEQ\_GCTL0.UAL\_CHUNK\_EN field must be cleared (=0). When needing to check the first CRC code for read data, the XSPI\_SEQ\_GCTL0.UAL\_CHUNK\_CHK bit must be set (=1).

## Miscellaneous Low-level Configurations

The following configurations are supported using the ACMD, direct, and STIG modes.

## Write Protection

Some devices that operate in single or dual mode support the use of a write protect signal using the DQ2 pin. The controller can support this feature by using the XSPI\_MINICTL\_WP\_CTL register. See the XSPI\_MINICTL\_WP\_CTL register description for configuration details.

## Device Reset

When operating in single or dual mode, some devices have a dedicated #RESET pin or support the hardware reset using the DQ3 pin. The controller supports the driving of reset on any bank (chip select) with the XSPI\_MINICTL\_WP\_CTL register.

## SPI Clock CPOL and CPHA Configuration

When operating in Single Data Rate (SDR), the transfers to and from the SPI memory device can operate in SPI mode 0 (polarity CPOL=0, phase CPHA=0, neither checked) or SPI mode 3 (CPOL=1, CPHA=1, both checked). It is unlikely this default configuration needs changing, but the option is available using the XSPI\_MINICTL\_CLKMODE register.

## JEDEC Reset Delay Settings

When issuing a JEDEC reset instruction to the memory device, the Joint Electron Device Engineering Council Solid State Technology Association (JEDEC) standard requires the controller to adhere to the t CSL  and t CSH  timing parameters. When non-default settings are required, a user programs the XSPI\_MINICTL\_JEDEC\_RST\_TR register with specific values.

## Device Delay Settings

The XSPI\_MINICTL\_DEV\_DLY register can provide minimum delays between transactions at the memory device interface.

## Reset Recovery Delay Settings

Following a hardware reset, the user can force a minimum time delay using the XSPI\_MINICTL\_RST\_RECOV register, before the chip select moves to an active state.

## tCMS Timing Pause Refresh Settings

Some volatile memory devices can require a periodic refresh (for example, HyperRAM). This refresh action is referred to as t CMS timing. Use the XSPI\_MINICTL\_DEV\_ACTIVE\_MAX register and the XSPI\_SEQ\_GCTL0.TCMS\_EN bit for the controller to configure a maximum time that the chip select remains in an active state to perform a periodic refresh operation. The controller automatically breaks up larger transactions. This action is transparant to the host interface.

## Legacy HyperFlash or Profile 2 Address Settings

By default, the host address must be rearranged to form the Command/Address (CA) structure for many legacy HyperFlash and profile 2 devices. Typically, the CA field has a reserved section; some bits of the host address must be shifted. The XSPI\_MINICTL\_HF\_OFFSET register provides the user a method to configure the mapping of these reserved bits.

## Multiple Device Connections to Controller

The xSPI controller can connect to a maximum of eight memory devices using the chip select (CS). Throughout this hardware reference, each device connected to an individual chip select is referred to as a bank. Mixed device types are permitted, but, cannot be used simultaneously. For example, when bank zero is connected to a quad SPI NOR flash device and bank one is connected to a HyperBus style device, then some reconfiguration of the xSPI register space is required when the host switches between accessing these devices.

## SPI NAND Device Operation Settings

For SPI NAND devices, the memory device address contains both row and column addresses. The SPI NAND Device Memory Address figure shows the field layout. The boundary between the row and column addresses can be configured using the XSPI\_SEQ\_GCTL1.PCA\_SIZ field. When the selected sequences require a number of bits aligned to 8 (for example, when a sequence requires 16 bits and contains 4 dummy bits before the 12 bits of the column address), the added dummy bits are internally cleared (=0).

Figure 21-10: SPI NAND Device Memory Address

| seq_page_ca_size + 24b   | seq_page_ca_size + 24b   | seq_page_ca_size (12 or 13 bits)   | seq_page_ca_size (12 or 13 bits)   |
|--------------------------|--------------------------|------------------------------------|------------------------------------|
| RESERVED (0)             | ROW ADDRESS              | ROW ADDRESS                        | COLUMN ADDRESS                     |

The SPI NAND devices require a specific page layout. The host software must program the controller to adhere to this field layout. It especially important not to overwrite the reserved areas as bad block data.

## ADSP-2184x XSPI Register Descriptions

Expanded Serial Peripheral Interface (XSPI) contains the following registers.

Table 21-52: ADSP-2184x XSPI Register List

| Name                        | Description                                |
|-----------------------------|--------------------------------------------|
| XSPI_BOOT_STAT              | Boot Status Register                       |
| XSPI_CMD0                   | Command Register 0                         |
| XSPI_CMD1                   | Command Register 1                         |
| XSPI_CMD2                   | Command Register 2                         |
| XSPI_CMD3                   | Command Register 3                         |
| XSPI_CMD4                   | Command Register 4                         |
| XSPI_CMD5                   | Command Register 5                         |
| XSPI_CMD_STAT               | Command Status Register                    |
| XSPI_CMD_STAT_PTR           | Command Status Pointer Register            |
| XSPI_DAC_CFG                | DAC Configuration Register                 |
| XSPI_DAC_REMAPADDR0         | Address Remapping Register 0               |
| XSPI_DAC_REMAPADDR1         | Address Remapping Register 1               |
| XSPI_DISCOVERY_GCTL         | Device Discovery Control Register          |
| XSPI_DMA_CTL                | DMAInterface Control Register              |
| XSPI_DMA_ERRADDR_HI         | DMAError Address High Register             |
| XSPI_DMA_ERRADDR_LO         | DMAError Address Low Register              |
| XSPI_ERS_SEQ_CFG0           | Erase Sequence Configuration Register 0    |
| XSPI_ERS_SEQ_CFG1           | Erase Sequence Configuration Register 1    |
| XSPI_ERS_SEQ_CFG2           | Erase Sequence Configuration Register 2    |
| XSPI_FEATURES               | Controller Features Register               |
| XSPI_GSTAT                  | General Controller Status Register         |
| XSPI_INT_EN                 | Interrupt Enable Register                  |
| XSPI_ISTAT                  | Interrupt Status Register                  |
| XSPI_LONGPOL_GCTL           | Long Polling Count Register                |
| XSPI_MINICTL_DEV_ACTIVE_MAX | Device Active Maximum Clock Cycle Register |
| XSPI_MINICTL_CLKMODE        | Clock Mode Control Register                |
| XSPI_MINICTL_DEV_DLY        | Device Delay Register                      |
| XSPI_MINICTL_HF_OFFSET      | HyperFlash Offset Register                 |
| XSPI_MINICTL_JEDEC_RST_TR   | JEDEC Reset Delay Register                 |

Table 21-52: ADSP-2184x XSPI Register List (Continued)

| Name                        | Description                               |
|-----------------------------|-------------------------------------------|
| XSPI_MINICTL_RST_PIN_CTL    | Hardware Reset Control Register           |
| XSPI_MINICTL_RST_RECOV      | Reset Recovery Delay Register             |
| XSPI_MINICTL_WP_CTL         | Write Protect Register                    |
| XSPI_PHY_DLLOB0             | PHY DLL Observable Points 0 Register      |
| XSPI_PHY_DLLOB1             | PHY DLL Observable Points 1 Register      |
| XSPI_PHY_DLL_CTL            | PHY DLL Control Register                  |
| XSPI_PHY_DLL_REQ_CTL        | Bus Requester DLL Control Register        |
| XSPI_PHY_DLL_COMP_CTL       | Bus Completer DLL Control Register        |
| XSPI_PHY_DLL_UPDT_CNT       | PHY DLL Resynchronization Register        |
| XSPI_PHY_DQS_TR             | PHY DQS Timing Register                   |
| XSPI_PHY_DQ_TR              | PHY DQTiming Register                     |
| XSPI_PHY_FEATURES           | PHY Features Register                     |
| XSPI_PHY_GATE_LPBK_CTL      | PHY Gate Loopback Control Register        |
| XSPI_PHY_GCTL               | PHY Global Control Register               |
| XSPI_PHY_GPIO_CTL0          | PHY GPIO Control Register 0               |
| XSPI_PHY_GPIO_CTL1          | PHY GPIO Control Register 1               |
| XSPI_PHY_GPIO_STAT0         | PHY GPIO Status Register 0                |
| XSPI_PHY_GPIO_STAT1         | PHY GPIO Status Register 1                |
| XSPI_PHY_GTSEL              | PHY Global Termination Control Register   |
| XSPI_PHY_IE_TR              | PHY DQS Input Enable Timing Register      |
| XSPI_PHY_OB0                | PHY Observable Points Register            |
| XSPI_PHY_REVID              | PHY Revision ID Register                  |
| XSPI_PHY_STATIC_TGL         | PHY Static Aging Register                 |
| XSPI_PHY_WR_DESKEW_PAD_CTL0 | PHY Deskew Write Register                 |
| XSPI_PROG_SEQ_CFG0          | Program Sequence Configuration Register 0 |
| XSPI_PROG_SEQ_CFG1          | Program Sequence Configuration Register 1 |
| XSPI_PROG_SEQ_CFG2          | Program Sequence Configuration Register 2 |
| XSPI_READ_SEQ_CFG0          | Read Sequence Configuration Register 0    |
| XSPI_READ_SEQ_CFG1          | Read Sequence Configuration Register 1    |
| XSPI_READ_SEQ_CFG2          | Read Sequence Configuration Register 2    |
| XSPI_REVID                  | Revision ID Register                      |

Table 21-52: ADSP-2184x XSPI Register List (Continued)

| Name                | Description                                          |
|---------------------|------------------------------------------------------|
| XSPI_RST_SEQ_CFG0   | Reset Sequence Configuration Register 0              |
| XSPI_RST_SEQ_CFG1   | Reset Sequence Configuration Register 1              |
| XSPI_SDMA_ADDR0     | Host DMABuffer Address Register 0                    |
| XSPI_SDMA_ADDR1     | Host DMABuffer Address Register 1                    |
| XSPI_SDMA_SIZ       | Host DMABlock Size Register                          |
| XSPI_SDMA_TRD_STAT  | Host DMAThread Status Register                       |
| XSPI_SEQ_GCTL0      | Sequence Configuration Register 0                    |
| XSPI_SEQ_GCTL1      | Sequence Configuration Register 1                    |
| XSPI_SHORTPOL_GCTL  | Short Polling Count Register                         |
| XSPI_STAT_SEQ_CFG0  | Status Checking Sequence Configuration Register 0    |
| XSPI_STAT_SEQ_CFG1  | Status Checking Sequence Configuration Register 1    |
| XSPI_STAT_SEQ_CFG10 | Status Checking Sequence Configuration Register 10   |
| XSPI_STAT_SEQ_CFG2  | Status Checking Sequence Configuration Register 2    |
| XSPI_STAT_SEQ_CFG3  | Status Checking Sequence Configuration Register 3    |
| XSPI_STAT_SEQ_CFG4  | Status Checking Sequence Configuration Register 4    |
| XSPI_STAT_SEQ_CFG5  | Status Checking Sequence Configuration Register 5    |
| XSPI_STAT_SEQ_CFG7  | Status Checking Sequence Configuration Register 7    |
| XSPI_STAT_SEQ_CFG8  | Status Checking Sequence Configuration Register 8    |
| XSPI_STAT_SEQ_CFG9  | Status Checking Sequence Configuration Register 9    |
| XSPI_TRD_COMP_ISTAT | Auto Command Engine Interrupt Status Thread Register |
| XSPI_TRD_ERR_INT_EN | Thread Error Interrupt Enable Register               |
| XSPI_TRD_ERR_ISTAT  | Thread Error Interrupt Status Register               |
| XSPI_TRD_STAT       | Auto Command Engine Thread Status Register           |
| XSPI_WE_SEQ_CFG0    | WEL Sequence Configuration Register                  |
| XSPI_WORKMODE_CTL   | Device Control Register                              |
| XSPI_XIP_GCTL       | XIP Configuration Register                           |

## Boot Status Register

The XSPI\_BOOT\_STAT register provides the status of the most recent boot operation, triggered by the automated boot controller.

Figure 21-11: XSPI\_BOOT\_STAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000009_b326e1f72a9a68c1fc5ae3396f0c5439d9b66efc876f6e1cc23066e998afb597.png)

Table 21-53: XSPI\_BOOT\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | BUS_ERR    | Bus Error Status. The XSPI_BOOT_STAT.BUS_ERR field describes bus status during the boot proc- ess. When set, the boot process failed due to the bus interface receiving an error response from the target. |
| 1 (R/NW)           | CRC_ERR    | CRC Error Status. The XSPI_BOOT_STAT.CRC_ERR field describes CRC status during the boot process. When set, the boot process failed due to the CRC error on the xSPI interface.                             |
| 0 (R/NW)           | DQS_ERR    | DQS Error Status. The XSPI_BOOT_STAT.DQS_ERR bit field describes the DQS status during the boot process. When set, the boot process failed due to the DQS error on the xSPI interface. 0 No Error Detected |

## Command Register 0

Writing data to the XSPI\_CMD0 register initiates a new transaction of the xSPI flash controller in CDMA/PIO and STIG work mode.

Encoding of the register depends on the selected work mode.

Figure 21-12: XSPI\_CMD0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000010_e39cddb29ac4e5d18c9ad280c67e0aaa80297ce4a535b5db90588ae0f7ab2dcb.png)

Table 21-54: XSPI\_CMD0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 0 Value.          |
| (R/W)              |            | Command 0 register field. |

## Command Register 1

The XSPI\_CMD1 register is only used in CDMA/PIO and STIG work mode. Field encoding of the register depends on the selected work mode.

Figure 21-13: XSPI\_CMD1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000011_0ab9263fd4326abf9a3716d7c9ae228a5eba717ecdce5dc584cebce26f6a5f7f.png)

Table 21-55: XSPI\_CMD1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 1 Value.          |
| (R/W)              |            | Command 1 register field. |

## Command Register 2

The XSPI\_CMD2 register is only used in CDMA/PIO and STIG work mode. Field encoding of the register depends on the selected work mode.

Figure 21-14: XSPI\_CMD2 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000012_c6583bdd711c3641b111d6782a927e5a56d8fcb9d64df8fa06222e2d0d5f7780.png)

Table 21-56: XSPI\_CMD2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 2 Value.          |
| (R/W)              |            | Command 2 register field. |

## Command Register 3

The XSPI\_CMD3 register is only used in CDMA/PIO and STIG work mode; the definition changes depending on the work mode.

Figure 21-15: XSPI\_CMD3 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000013_910afc2b2f64305d2c6c3125ed58592ba52d16442cbd7fcf4eddd50ebfc21dfa.png)

Table 21-57: XSPI\_CMD3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 3 Value.          |
| (R/W)              |            | Command 3 register field. |

## Command Register 4

The XSPI\_CMD4 register is only used in CDMA/PIO and STIG work mode; the definition changes depending on the work mode.

Figure 21-16: XSPI\_CMD4 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000014_e3b83183f7a035d26c4dae53a6c9853121916fdf2ffc603e2dd45b6201e1a81b.png)

Table 21-58: XSPI\_CMD4 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 4 Value.          |
| (R/W)              |            | Command 4 register field. |

## Command Register 5

The XSPI\_CMD5 register is only used in CDMA/PIO and STIG work mode; the definition changes depending on the work mode.

Figure 21-17: XSPI\_CMD5 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000015_4a728739297e8f9ba19d2233e2f3423f4ae81161bbfc7a2a67d47dd33c679cbb.png)

Table 21-59: XSPI\_CMD5 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | VALUE      | Command 5 Value.          |
| (R/W)              |            | Command 5 register field. |

## Command Status Register

The XSPI\_CMD\_STAT register reports the command status for the selected thread in ACMD and STIG work mode, when an xSPI flash transaction has completed.

Figure 21-18: XSPI\_CMD\_STAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000016_143e647eea5cb332ce93a065ff6195f62a18ac6c9b9ea2634e8c8b18fa6e6701.png)

Table 21-60: XSPI\_CMD\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | STAT       | Command Status. When operating in ACMD mode, the XSPI_CMD_STAT.STAT bit field reports the descriptor status associated with the selected thread (the thread to which this status belongs is selected via the XSPI_CMD_STAT_PTR register). When operating in STIG mode, this register reports the STIG completion status. Refer to section "STIG Mode Status" for a full bitwise definition. |

## Command Status Pointer Register

The XSPI\_CMD\_STAT\_PTR register selects the thread that is used for outputting the command status in the XSPI\_CMD\_STAT register.

This register is relevant for ACMD mode only.

Figure 21-19: XSPI\_CMD\_STAT\_PTR Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000017_98838ac61322cee83f16f68ca6190f328fd8b411e19aee2c3677b168ae7a344e.png)

Table 21-61: XSPI\_CMD\_STAT\_PTR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 2:0                | THRD_SEL   | Thread ID. The XSPI_CMD_STAT_PTR.THRD_SEL bit field indicates the thread ID status; the status is available in the XSPI_CMD_STAT register. |
| (R/W)              |            |                                                                                                                                            |

## DAC Configuration Register

The XSPI\_DAC\_CFG register holds specific configuration information required while operating in direct work mode.

Figure 21-20: XSPI\_DAC\_CFG Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000018_a5b0760b541d404039f62130ec6812d377748beebe2b10260f797081414a8bb8.png)

Table 21-62: XSPI\_DAC\_CFG Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                    |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28:16 (R/W)        | ADDR_MASK      | Profile 2 Address Mask. The XSPI_DAC_CFG.ADDR_MASK bit field enables masking bits [44:32] of the system address for read/write transfers for profile 2.                                                                                                                                                                                    |
| 12 (R/W)           | RMP_ADDR_EN    | Enable Host Data Interface Address Remapping. When the XSPI_DAC_CFG.RMP_ADDR_EN bit is set (=1), the incoming AXI host address is adapted and sent to the flash device as (address - N), where Nis the value stored in the remap address register (XSPI_DAC_REMAPADDRn).                                                                   |
| 9 (R/W)            | XIP_DIS_MB_VAL | XIP Mode Disable. The XSPI_DAC_CFG.XIP_DIS_MB_VAL bit is used to trigger exit from XIP mode within the device. When XSPI_DAC_CFG.XIP_DIS_MB_VAL is set (=1), the controller sends mode bits specified in the XSPI_XIP_GCTL. XIP_DIS_MB_VALUE on the next read transaction. This disables XIP work mode for both the device and controller. |
| 8 (R/W)            | XIP_EN_MB_VAL  | XIP Mode Enable. The XSPI_DAC_CFG.XIP_EN_MB_VAL bit is used to trigger entry to XIP mode within the device. When XSPI_DAC_CFG.XIP_EN_MB_VAL is set (=1), the con- troller sends mode bits as specified in XSPI_XIP_GCTL. XIP_EN_MB_VAL on the next read transaction. This switches both device and controller into XIP work mode.          |

Table 21-62: XSPI\_DAC\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | RWDS_CAP_EN | RWDS Byte Mask Enable. When enabled, the controller supports byte masking in the connected device by automatically translating the incoming AXI host interface write strobes as needed. If the connected device does not support RWDS byte masking, the XSPI_DAC_CFG.RWDS_CAP_EN bit must be cleared. Note that when the XSPI_DAC_CFG.RWDS_CAP_EN bit is cleared, the controller assumes all bytes on the AXI write channel have an equivalent AXI write strobe set to 1. Also note that when this bit is cleared, the user cannot send byte writes to an octal DDR device or to a device that is 16-bit addressed. |
| 2:0 (R/W)          | DAC_BNK_NUM | Bank Number. The XSPI_DAC_CFG.DAC_BNK_NUM bit field indicates the bank (the attached memory device) targeted by the controller while operating in direct mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

## Address Remapping Register 0

When XSPI\_DAC\_CFG.RMP\_ADDR\_EN =1, the incoming AXI host interface address is adapted and sent to the flash device as (address - N), where N[31:0] is the value stored in this register.

Figure 21-21: XSPI\_DAC\_REMAPADDR0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000019_0fe3c8aef581971a68cb70c4ce43808e309d81ac355a100b126f5c7d5b72ca46.png)

Table 21-63: XSPI\_DAC\_REMAPADDR0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ADDR_VALUE0 | Remapping Address Low. The XSPI_DAC_REMAPADDR0.ADDR_VALUE0 bit field indicates the remapping of the incoming address on the AXI host interface to a different address used by the flash device. The value of this register must be aligned to 8 bytes. |

## Address Remapping Register 1

When XSPI\_DAC\_CFG.RMP\_ADDR\_EN =1, the incoming host interface address is adapted and sent to the flash device as (address - N), where N[63:32] is the value stored in this register.

Figure 21-22: XSPI\_DAC\_REMAPADDR1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000020_d932bf2ce67da7b3c3f851a30227a910c39086b5655104e8ab6a2043a81a35fc.png)

Table 21-64: XSPI\_DAC\_REMAPADDR1 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                      |
|--------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ADDR_VALUE1 | Remapping Address High. The XSPI_DAC_REMAPADDR1.ADDR_VALUE1 bit field indicates the remapping of the incoming address on the host interface to a different address used by the flash device. |

## Device Discovery Control Register

The XSPI\_DISCOVERY\_GCTL register controls device discovery.

Figure 21-23: XSPI\_DISCOVERY\_GCTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000021_60044ad652aa39b38addb460d1e3cd6a14cdbaaff48869508bcaaad0cec21d18.png)

Table 21-65: XSPI\_DISCOVERY\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | BNK        | Discovery Bank Select. The XSPI_DISCOVERY_GCTL.BNK bit field indicates the discovery bank. Writing                                                                                                                                                       | Discovery Bank Select. The XSPI_DISCOVERY_GCTL.BNK bit field indicates the discovery bank. Writing                                                                                                                                                       |
| 15:12 (R/W)        | NUM_LINES  | Discovery IO Number of Lines. This is a 4-bit value used in discovery mode. Writing a value to the XSPI_DISCOVERY_GCTL.NUM_LINES bit field selects number of xSPI I/Os used by device discovery. This field is updated after the initialization process. | Discovery IO Number of Lines. This is a 4-bit value used in discovery mode. Writing a value to the XSPI_DISCOVERY_GCTL.NUM_LINES bit field selects number of xSPI I/Os used by device discovery. This field is updated after the initialization process. |
|                    |            | 0                                                                                                                                                                                                                                                        | Auto                                                                                                                                                                                                                                                     |
|                    |            | 1                                                                                                                                                                                                                                                        | 1 line                                                                                                                                                                                                                                                   |
|                    |            | 2                                                                                                                                                                                                                                                        | 2 lines                                                                                                                                                                                                                                                  |
|                    |            | 3                                                                                                                                                                                                                                                        | Reserved                                                                                                                                                                                                                                                 |
|                    |            | 4                                                                                                                                                                                                                                                        | 4 lines                                                                                                                                                                                                                                                  |

Table 21-65: XSPI\_DISCOVERY\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 5 Reserved                                                                                                                                                                          |
|                    |            | 6 Reserved                                                                                                                                                                          |
|                    |            | 7 Reserved                                                                                                                                                                          |
|                    |            | 8 8 lines                                                                                                                                                                           |
|                    |            | 9 Reserved                                                                                                                                                                          |
|                    |            | 10 Reserved                                                                                                                                                                         |
|                    |            | 11 Reserved                                                                                                                                                                         |
|                    |            | 12 8 lines for legacy HyperFlash and xSPI profile                                                                                                                                   |
|                    |            | 13 Reserved                                                                                                                                                                         |
|                    |            | 14 1 line for legacy SPI NAND                                                                                                                                                       |
|                    |            | 15 Reserved                                                                                                                                                                         |
| 11 (R/W)           | ABNUM      | Discovery Number Address. The XSPI_DISCOVERY_GCTL.ABNUM bit enables discovery 4-bit addressing. This field is updated after the initialization process.                             |
| 11 (R/W)           | ABNUM      | 0 3-bit addressing                                                                                                                                                                  |
| 11 (R/W)           | ABNUM      | 1 4-bit addressing                                                                                                                                                                  |
| 10 (R/W)           | DMY_CNT    | Discovery Dummy Clock Cycles Count. The XSPI_DISCOVERY_GCTL.DMY_CNT bit indicates the discovery number of dummy clock cycles. This bit is updated after the initialization process. |
| 10 (R/W)           | DMY_CNT    | 0 8 dummy clock cycles                                                                                                                                                              |
| 10 (R/W)           | DMY_CNT    | 1 20 dummy clock cycles                                                                                                                                                             |
| 9:8 (R/W)          | CMD_TYP    | Discovery Command Type Mode Enable. The XSPI_DISCOVERY_GCTL.CMD_TYP bit selects the discovery command type mode. This field is updated after the initialization process.            |
| 9:8 (R/W)          | CMD_TYP    | 0 SDR mode enabled                                                                                                                                                                  |
| 9:8 (R/W)          | CMD_TYP    | 1 DDR mode enabled                                                                                                                                                                  |
| 9:8 (R/W)          | CMD_TYP    | 2 DTR mode enabled (quad mode only)                                                                                                                                                 |
| 9:8 (R/W)          | CMD_TYP    | 3 Reserved                                                                                                                                                                          |

Table 21-65: XSPI\_DISCOVERY\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | OE_EN      | Discovery Extended Opcode Enable. The XSPI_DISCOVERY_GCTL.OE_EN bit enables the discovery extended opcode. This field is updated after the initialization process.                                                                                     |
| 7 (R/W)            | OE_EN      | 0 Disable                                                                                                                                                                                                                                              |
| 7 (R/W)            | OE_EN      | 1 Enable                                                                                                                                                                                                                                               |
| 6 (R/W)            | OE_VAL     | Discovery Extended Opcode Value. The XSPI_DISCOVERY_GCTL.OE_VAL bit field indicates the discovery extended opcode value. This field is updated after the initialization process.                                                                       |
| 6 (R/W)            | OE_VAL     | 0 Extended opcode is 8'hA5                                                                                                                                                                                                                             |
| 6 (R/W)            | OE_VAL     | 1 Extended opcode is 8'h5A                                                                                                                                                                                                                             |
| 5 (R/NW)           | INHIBIT    | Discovery Inhibit Status. The XSPI_DISCOVERY_GCTL.INHIBIT bit indicates whether device discovery is inhibited at power on.                                                                                                                             |
| 5 (R/NW)           | INHIBIT    | 0 Discovery allow                                                                                                                                                                                                                                      |
| 5 (R/NW)           | INHIBIT    | 1 Discovery inhibit                                                                                                                                                                                                                                    |
| 4:3 (R/NW)         | FAIL       | Result of Last Discovery Operation. The XSPI_DISCOVERY_GCTL.FAIL bit field indicates the result of the last dis- covery operation. Valid when PASS = 1.                                                                                                |
| 4:3 (R/NW)         | FAIL       | 0 xSPI or SPI NAND device detected                                                                                                                                                                                                                     |
| 4:3 (R/NW)         | FAIL       | 1 Fail                                                                                                                                                                                                                                                 |
| 4:3 (R/NW)         | FAIL       | 2 Legacy SPI device detected                                                                                                                                                                                                                           |
| 4:3 (R/NW)         | FAIL       | 3 N/A                                                                                                                                                                                                                                                  |
| 2 (R/NW)           | PASS       | Status of Last Discovery Operation. The XSPI_DISCOVERY_GCTL.PASS bit indicates the status of the last discovery operation. XSPI_DISCOVERY_GCTL.PASS =1 when the device discovery opera- tion has finished. The result can be read from the FAIL field. |
| 1 (R/W)            | REQ_TYP    | Discovery Request Type. The XSPI_DISCOVERY_GCTL.REQ_TYP bit indicates the discovery request type.                                                                                                                                                      |
| 1 (R/W)            | REQ_TYP    | 0 Full discovery. Perform full discovery process (try to detect device)                                                                                                                                                                                |
| 1 (R/W)            | REQ_TYP    | 1 Selected mode. Configure registers only to selected mode (not full discovery process)                                                                                                                                                                |

Table 21-65: XSPI\_DISCOVERY\_GCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | REQ        | Discovery Request. Writing 1 to the XSPI_DISCOVERY_GCTL.REQ bit triggers the device discovery operation. The XSPI_DISCOVERY_GCTL.REQ bit is cleared by hardware when the device discovery operation completes. |

## DMA Interface Control Register

The XSPI\_DMA\_CTL register is a common register for both the host and internal DMA interface.

Figure 21-24: XSPI\_DMA\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000022_d9566b3fbb7ca14d84a1528d642603ea70cbff2f2441e16ced9a3781bfa5ba84.png)

Table 21-66: XSPI\_DMA\_CTL Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:18 (R/W)        | WORD_SIZE    | DMAWord Size. The XSPI_DMA_CTL.WORD_SIZE bit field indicates the AXI transaction size used by the internal DMAinterface to transfer data.                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 17 (R/W)           | SDMA_ERR_RSP | SDMA Error Response. AXI error responses can only occur in ACMD or STIG work modes. When XSPI_DMA_CTL.SDMA_ERR_RSP is set (=1), an AXI error response is returned if one of the following is true. 1. The host attempts to access the target interface before it is permitted (refer to the required steps in STIG and ACMD sections of this document). 2. The host issues an unsupported burst type. The controller only supports incremen- tal bursts (non-wrapping) bursts currently. When XSPI_DMA_CTL.SDMA_ERR_RSP is cleared (=0), an OK response is re- turned. |
| 16 (R/W)           | OTE          | Outstanding Transaction Enable. The XSPI_DMA_CTL.OTE bit enables an outstanding transaction. This only applies to the internal DMAinterface. The AXI host interface ignores this bit and accepts all incoming transactions.                                                                                                                                                                                                                                                                                                                                            |

Table 21-66: XSPI\_DMA\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | BURST_SEL  | Burst Size. The XSPI_DMA_CTL.BURST_SEL bit field indicates the maximum AXI burst size used by the internal DMAinterface. The maximum burst size can be calculated as XSPI_DMA_CTL.BURST_SEL +1. Note that this field must be changed only when controller is in an idle state. |

## DMA Error Address High Register

Internal DMA interface error address [63:32]. The XSPI\_DMA\_ERRADDR\_HI register holds the upper 32 bits of the address of the AXI request on the system bus requester data interface that caused the CDMA\_TERR or DDMA\_TERR bits in the XSPI\_ISTAT register to be set. This register is overwritten when further error responses are detected.

Figure 21-25: XSPI\_DMA\_ERRADDR\_HI Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000023_4e8f83381d85ea0b308ea7c3ea4080e30c426976efe1eaa93cc796eb335e65b4.png)

Table 21-67: XSPI\_DMA\_ERRADDR\_HI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | High Address Value. The XSPI_DMA_ERRADDR_HI.VALUE bit field indicates the address of the first request on the internal DMAinterface that returned an error response. |

## DMA Error Address Low Register

Internal DMA interface error address [31:0]. The XSPI\_DMA\_ERRADDR\_LO register holds the lower 32 bits of the address of the AXI request on the system bus requester data interface that caused the CDMA\_TERR or DDMA\_TERR bits in the XSPI\_ISTAT register to be set. The XSPI\_DMA\_ERRADDR\_LO register is overwritten when further error responses are detected.

Figure 21-26: XSPI\_DMA\_ERRADDR\_LO Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000024_1cb10418df6dc9a3449adb2ed814226e0ad55feefb60e1f7aeae10d796a1f206.png)

Table 21-68: XSPI\_DMA\_ERRADDR\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Low Address Value. The XSPI_DMA_ERRADDR_LO.VALUE bit field indicates the address of the first request on the internal DMAinterface that returned an error response. |

## Erase Sequence Configuration Register 0

The XSPI\_ERS\_SEQ\_CFG0 register configures the erase\_sector sequence for profile 1 and SPI NAND in ACMD work mode.

Figure 21-27: XSPI\_ERS\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000025_805e9c3dcd1e13b218b257d249d65f3e122cd15e35c467a0589e97438b80d7bd.png)

Table 21-69: XSPI\_ERS\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                    |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | ADDR_EDGE     | Address Phase SDR/DDR Select. The XSPI_ERS_SEQ_CFG0.ADDR_EDGE bit selects between SDR/DDR mode for the address phase.                      |
| 25:24 (R/W)        | ADDR_IOS      | Address Phase IO Lines Select. The XSPI_ERS_SEQ_CFG0.ADDR_IOS bit field indicates the number of data lines used to send the address phase. |
| 25:24 (R/W)        |               | 0 One data line used (for example, serial)                                                                                                 |
| 25:24 (R/W)        |               | 1 Two data lines used                                                                                                                      |
| 25:24 (R/W)        |               | 2 Four data lines used                                                                                                                     |
| 25:24 (R/W)        |               | 3 Eight data lines used                                                                                                                    |
| 23:16 (R/W)        | CMD_EXT_VALUE | Command Extension Value. The XSPI_ERS_SEQ_CFG0.CMD_EXT_VALUE bit field indicates the command extension value, when enabled.                |
| 15 (R/W)           | CMD_EXT_EN    | Enable Command Extension. The XSPI_ERS_SEQ_CFG0.CMD_EXT_EN bit enables command extension.                                                  |

Table 21-69: XSPI\_ERS\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/W)        | ADDR_CNT   | Address Byte Count. The XSPI_ERS_SEQ_CFG0.ADDR_CNT bit field indicates the number of address bytes.                                       |
| 11 (R/W)           | CMD_EDGE   | Command Phase SDR/DDR Select. The XSPI_ERS_SEQ_CFG0.CMD_EDGE bit field selects between SDR/DDR mode for the command phase.                |
| 9:8 (R/W)          | CMD_IOS    | Command Phase IO Lines Select. The XSPI_ERS_SEQ_CFG0.CMD_IOS bit field indicates the number of data lines used to send the command phase. |
| 9:8 (R/W)          | CMD_IOS    | 0 One data line used (for example, serial)                                                                                                |
| 9:8 (R/W)          | CMD_IOS    | 1 Two data lines used                                                                                                                     |
| 9:8 (R/W)          | CMD_IOS    | 2 Four data lines used                                                                                                                    |
| 9:8 (R/W)          | CMD_IOS    | 3 Eight data lines used                                                                                                                   |
| 7:0 (R/W)          | CMD_VALUE  | Command Mnemonic Value. The XSPI_ERS_SEQ_CFG0.CMD_VALUE bit field indicates the command mne- monic value.                                 |

## Erase Sequence Configuration Register 1

The XSPI\_ERS\_SEQ\_CFG1 register configures the erase\_sector sequence for profile 1 and SPI NAND in ACMD work mode.

Figure 21-28: XSPI\_ERS\_SEQ\_CFG1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000026_6bba04e31e50cf91b663a6de59afdd159d9650be11a96eb5e9da86b1bad4812d.png)

Table 21-70: XSPI\_ERS\_SEQ\_CFG1 Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|-------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4:0 (R/W)          | P1_SECT_SIZ_VALUE | Sector Size. The XSPI_ERS_SEQ_CFG1.P1_SECT_SIZ_VALUE bit field indicates the sec- tor size. Value encoded as 2 erss_seq_p1_sect_size : 8'h00 - 1B 8'h01 - 2B 8'h02 - 4B ... 8'h0f - 32kB 8'h10 - 64kB ... 8'h1f - (2 31 )B |

## Erase Sequence Configuration Register 2

The XSPI\_ERS\_SEQ\_CFG2 register configures the erase\_all sequence for profile 1 in ACMD work mode.

Figure 21-29: XSPI\_ERS\_SEQ\_CFG2 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000027_b13f27e5394c6f46de14aa1073536238468c833e71ea00e681ab9c81a95cda81.png)

Table 21-71: XSPI\_ERS\_SEQ\_CFG2 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                           | Description/Enumeration                                                                                                                           |
|--------------------|---------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | ER- SA_P1_CMD_EXT_VAL- UE | Command Extension Value. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EXT_VALUE bit field indicates the command extension value.                             | Command Extension Value. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EXT_VALUE bit field indicates the command extension value.                             |
| 15 (R/W)           | ERSA_P1_CMD_EXT_EN        | Enable Command Extension. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EXT_EN bit field enables the com- mand extension.                                     | Enable Command Extension. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EXT_EN bit field enables the com- mand extension.                                     |
| 11 (R/W)           | ERSA_P1_CMD_EDGE          | Command Phase SDR/DDR Select. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EDGE bit selects between SDR/DDR mode for the command phase.                      | Command Phase SDR/DDR Select. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_EDGE bit selects between SDR/DDR mode for the command phase.                      |
| 9:8 (R/W)          | ERSA_P1_CMD_IOS           | Command Phase IO Lines Select. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_IOS bit field indicates the number of data lines used to send the command phase. | Command Phase IO Lines Select. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_IOS bit field indicates the number of data lines used to send the command phase. |
| 9:8 (R/W)          | ERSA_P1_CMD_IOS           | 0                                                                                                                                                 | One data line used (for example, serial)                                                                                                          |
| 9:8 (R/W)          | ERSA_P1_CMD_IOS           | 1                                                                                                                                                 | Two data lines used                                                                                                                               |
| 9:8 (R/W)          | ERSA_P1_CMD_IOS           | 2                                                                                                                                                 | Four data lines used                                                                                                                              |
| 9:8 (R/W)          | ERSA_P1_CMD_IOS           | 3                                                                                                                                                 | Eight data lines used                                                                                                                             |
| 7:0 (R/W)          | ERSA_P1_CMD_VALUE         | Command Mnemonic Value. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_VALUE bit field indicates the com-                                                      | Command Mnemonic Value. The XSPI_ERS_SEQ_CFG2.ERSA_P1_CMD_VALUE bit field indicates the com-                                                      |

## Controller Features Register

The XSPI\_FEATURES register shows the available hardware features of the controller.

Figure 21-30: XSPI\_FEATURES Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000028_894fb267a006f9dc595b1597ae73c969c0aa16654399ed9f5a7e29531ccd9ffd.png)

Table 21-72: XSPI\_FEATURES Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                  |
|--------------------|-------------|--------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/NW)       | NUMBNKS     | Max Banks Supported. The XSPI_FEATURES.NUMBNKS bit field indicates the maximum number of banks supported by hardware.    |
| 25:24 (R/NW)       | NUMBNKS     | 0 One bank                                                                                                               |
| 25:24 (R/NW)       | NUMBNKS     | 1 Two banks                                                                                                              |
| 25:24 (R/NW)       | NUMBNKS     | 2 Four banks                                                                                                             |
| 25:24 (R/NW)       | NUMBNKS     | 3 Eight banks                                                                                                            |
| 23:22 (R/NW)       | SFR_INTF    | SFR Interface Type. The XSPI_FEATURES.SFR_INTF bit field indicates the SFR interface type 1- APB, other values reserved. |
| 21 (R/NW)          | DMA_DAT_SIZ | Host DMAData Width. The XSPI_FEATURES.DMA_DAT_SIZ bit indicates the host DMAdata width. 0 32 bit                         |
| 21 (R/NW)          | DMA_DAT_SIZ | 1 64 bit                                                                                                                 |
| 21 (R/NW)          | DMA_DAT_SIZ |                                                                                                                          |

Table 21-72: XSPI\_FEATURES Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                         | Description/Enumeration                                                                                                         |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/NW)          | DMA_ADDR_SIZ | Host DMAAddress Width. The XSPI_FEATURES.DMA_ADDR_SIZ bit indicates the host DMAaddress width.                                  | Host DMAAddress Width. The XSPI_FEATURES.DMA_ADDR_SIZ bit indicates the host DMAaddress width.                                  |
| 20 (R/NW)          | DMA_ADDR_SIZ | 0                                                                                                                               | 32 bit                                                                                                                          |
| 20 (R/NW)          | DMA_ADDR_SIZ | 1                                                                                                                               | 64 bit                                                                                                                          |
| 19:18 (R/NW)       | DMA_INTF     | DMAInterface Type. The XSPI_FEATURES.DMA_INTF bit field indicates the DMAinterface type (0-AXI4 other values reserved).         | DMAInterface Type. The XSPI_FEATURES.DMA_INTF bit field indicates the DMAinterface type (0-AXI4 other values reserved).         |
| 16 (R/NW)          | BOOT         | Boot Feature Present. The XSPI_FEATURES.BOOT bit indicates whether the boot feature is present.                                 | Boot Feature Present. The XSPI_FEATURES.BOOT bit indicates whether the boot feature is present.                                 |
| 12 (R/NW)          | ASF          | ASF Feature Present. The XSPI_FEATURES.ASF bit indicates whether the ASF feature is present.                                    | ASF Feature Present. The XSPI_FEATURES.ASF bit indicates whether the ASF feature is present.                                    |
| 3:0 (R/NW)         | NUMTRDS      | Number of Threads Supported. The XSPI_FEATURES.NUMTRDS bit field indicates the number of threads availa- ble in the controller. | Number of Threads Supported. The XSPI_FEATURES.NUMTRDS bit field indicates the number of threads availa- ble in the controller. |
| 3:0 (R/NW)         | NUMTRDS      | 0                                                                                                                               | One thread                                                                                                                      |
| 3:0 (R/NW)         | NUMTRDS      | 1                                                                                                                               | Two threads                                                                                                                     |
| 3:0 (R/NW)         | NUMTRDS      | 2                                                                                                                               | Four threads                                                                                                                    |
| 3:0 (R/NW)         | NUMTRDS      | 3                                                                                                                               | Eight threads                                                                                                                   |
| 3:0 (R/NW)         | NUMTRDS      | 4-15                                                                                                                            | Reserved                                                                                                                        |

## General Controller Status Register

The XSPI\_GSTAT register indicates the status of the controller.

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000029_039fe8961409404f6679825be30b1bcd3174d24f4d94a731c8c9594ddc284487.png)

INIT\_PASS (R)

Initialization Complete

Figure 21-31: XSPI\_GSTAT Register Diagram

Table 21-73: XSPI\_GSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | INIT_PASS  | Initialization Complete. When XSPI_GSTAT.INIT_PASS is set (=1), the xSPI controller has completed its reset and initialization process.                                                     |
| 9:8 (R/NW)         | INIT_FAIL  | Initialization Status. The XSPI_GSTAT.INIT_FAIL bit field indicates the initialization process status.                                                                                      |
| 9:8 (R/NW)         | INIT_FAIL  | 0 xSPI device detected                                                                                                                                                                      |
| 9:8 (R/NW)         | INIT_FAIL  | 1 Initialization has failed                                                                                                                                                                 |
| 9:8 (R/NW)         | INIT_FAIL  | 2 Legacy SPI device detected                                                                                                                                                                |
| 9:8 (R/NW)         | INIT_FAIL  | 3 N/A                                                                                                                                                                                       |
| 7 (R/NW)           | CTL_BUSY   | Controller Busy. The XSPI_GSTAT.CTL_BUSY bit indicates whether controller is in the busy state. Note that this bit is also routed to the controller interface via the CTRL_BUSY pin. 0 Idle |
| 7 (R/NW)           | CTL_BUSY   | 1 Busy                                                                                                                                                                                      |
| 7 (R/NW)           | CTL_BUSY   |                                                                                                                                                                                             |

Table 21-73: XSPI\_GSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/NW)           | DISCOVERY_BUSY   | Discovery Busy. When XSPI_GSTAT.DISCOVERY_BUSY is set (=1), device discovery is in prog- ress. When device discovery is inhibited, this bit is also set during the PHY initializa- tion procedure.                                                                                                                                            |
| 4 (R/NW)           | GCMD_ENG_MC_BUSY | GCMDEngine MCBusy. The bit XSPI_GSTAT.GCMD_ENG_MC_BUSY is relevant for STIG mode only. This bit indicates when the controller is waiting for next/last instruction in a glued instruction chain or it is executing a requested sequence on the xSPI interface.                                                                                |
| 3 (R/NW)           | GCMD_ENG_BUSY    | GCMDEngine Busy. When operating in direct work mode, the XSPI_GSTAT.GCMD_ENG_BUSY is set (=1), when the direct CMDgenerator is busy. Note that no new requests on the host interface are accepted while XSPI_GSTAT.GCMD_ENG_BUSY is set. When operating in STIG work mode, the XSPI_GSTAT.GCMD_ENG_BUSY is set while the STIG engine is busy. |
| 2 (R/NW)           | ACMD_ENG_BUSY    | ACMD Engine Busy. When XSPI_GSTAT.ACMD_ENG_BUSY is set (=1), the auto command engine is busy (ACMD mode only).                                                                                                                                                                                                                                |
| 1 (R/NW)           | MDMA_BUSY        | MDMABusy. When XSPI_GSTAT.MDMA_BUSY is set (=1), the internal DMAinterface is busy.                                                                                                                                                                                                                                                           |
| 0 (R/NW)           | SDMA_BUSY        | SDMA Busy. When XSPI_GSTAT.SDMA_BUSY is set (=1), the host interface is busy.                                                                                                                                                                                                                                                                 |

## Interrupt Enable Register

If the selected bit of the XSPI\_INT\_EN register is set (=1), the rising edge of the corresponding bit in the XSPI\_ISTAT register triggers an interrupt.

Figure 21-32: XSPI\_INT\_EN Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000030_222fe559ae2f53ddb0adc97f049efbc1c3a69b8207b69cf1800891d66fad4516.png)

Table 21-74: XSPI\_INT\_EN Register Fields

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                                                                                                                                               |
|--------------------|----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | GINT_EN              | Global Interrupt Enable Flag. The XSPI_INT_EN.GINT_EN bit enables a global interrupt flag.                                                                                            |
| 28 (R/W)           | DIR_DEV_ERR_EN       | Interrupt Enable for Uncorrectable ECC Error. The XSPI_INT_EN.DIR_DEV_ERR_EN bit enables an interrupt when an uncor- rectable ECC or program fail error occurred in direct work mode. |
| 27 (R/W)           | DIR_ECC_CORR_ERR_E N | Interrupt Enable for Correctable ECC Error. The XSPI_INT_EN.DIR_ECC_CORR_ERR_EN bit enables an interrupt when a correctable ECC error occurred in direct work mode.                   |

Table 21-74: XSPI\_INT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                         |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | DIR_CMD_ERR_EN | Interrupt Enable for Command Error. The XSPI_INT_EN.DIR_CMD_ERR_EN bit enables an interrupt when an invalid command sequence has been detected in direct work mode.                             |
| 25 (R/W)           | DIR_DQS_ERR_EN | Interrupt Enable for DQS Error. The XSPI_INT_EN.DIR_DQS_ERR_EN bit enables an interrupt when the con- troller returns a DQS error after a read or status checking command in direct work mode.  |
| 24 (R/W)           | DIR_CRC_ERR_EN | Interrupt Enable for CRC Error. The XSPI_INT_EN.DIR_CRC_ERR_EN bit enables an interrupt when the con- troller returns a CRC error after a read or status checking command in direct work mode.  |
| 23 (R/W)           | STIG_DONE_EN   | Interrupt Enable for STIG Done. The XSPI_INT_EN.STIG_DONE_EN bit enables an interrupt when an instruction in a glued chain is complete.                                                         |
| 22 (R/W)           | SDMA_ERR_EN    | Interrupt Enable for SDMA Error. The XSPI_INT_EN.SDMA_ERR_EN bit enables an interrupt when an illegal access to the host DMAinterface is detected.                                              |
| 21 (R/W)           | SDMA_TRIGG_EN  | Interrupt Enable for SDMA Trigger. The XSPI_INT_EN.SDMA_TRIGG_EN bit enables an interrupt when the trigger condition for the host DMAis met.                                                    |
| 20 (R/W)           | CMD_IGNORED_EN | Interrupt Enable for Ignored Command. The XSPI_INT_EN.CMD_IGNORED_EN bit enables an interrupt for detection of an ignored command.                                                              |
| 18 (R/W)           | DDMA_TERR_EN   | Interrupt Enable for Data DMAError. The XSPI_INT_EN.DDMA_TERR_EN bit enables an interrupt for detecting a data internal DMAerror.                                                               |
| 17 (R/W)           | CDMA_TERR_EN   | Interrupt Enable for Command DMAError. The XSPI_INT_EN.CDMA_TERR_EN bit enables an interrupt for detecting an auto CMDengine target error.                                                      |
| 16 (R/W)           | CTL_IDLE_EN    | Interrupt Enable for Idle State. The XSPI_INT_EN.CTL_IDLE_EN bit enables an interrupt for detecting that the controller has returned to the idle state.                                         |
| 15 (R/W)           | GP_OD3_EN      | Interrupt Enable for GP OD3 Transition. The XSPI_INT_EN.GP_OD3_EN bit enables an interrupt for detecting the high- to-low or low-to-high transition on the XSPI_DFI_GP_OPEN_DRAIN[3] input pin. |

Table 21-74: XSPI\_INT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | GP_OD2_EN  | Interrupt Enable for GP OD2 Transition. The XSPI_INT_EN.GP_OD2_EN bit enables an interrupt for detecting the high- to-low or low-to-high transition on the XSPI_DFI_GP_OPEN_DRAIN[2] input pin. |
| 13 (R/W)           | GP_OD1_EN  | Interrupt Enable for GP OD1 Transition. The XSPI_INT_EN.GP_OD1_EN bit enables an interrupt for detecting the high- to-low or low-to-high transition on the XSPI_DFI_GP_OPEN_DRAIN[1] input pin. |
| 12 (R/W)           | GP_OD0_EN  | Interrupt Enable for GP OD0 Transition. The XSPI_INT_EN.GP_OD0_EN bit enables an interrupt for detecting the high- to-low or low-to-high transition on the XSPI_DFI_GP_OPEN_DRAIN[0] input pin. |

## Interrupt Status Register

The XSPI\_ISTAT register contains information about functional areas requiring servicing. Many of the bits serve as an indicator to further read and service various status registers. After servicing the interrupt source associated with a bit, the user must clear that interrupt source bit by writing a 1 to it.

Figure 21-33: XSPI\_ISTAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000031_0615ddc51dca2225304ee4d0628c9ec28efac2ed7ea069ff86f98673d2349e98.png)

Table 21-75: XSPI\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                         |
|--------------------|------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W1C)         | DIR_DEV_ERR      | Device Return or Uncorrectable ECC Error. The XSPI_ISTAT.DIR_DEV_ERR bit is set (=1) when a program operation in direct mode failed. That is, the device returned the program fail bit when the status was checked. This bit is also set when an uncorrectable ECC error occurs in direct mode. |
| 27 (R/W1C)         | DIR_ECC_CORR_ERR | Correctable ECC Error. The XSPI_ISTAT.DIR_ECC_CORR_ERR bit is set (=1) when a correctable ECC error occurs in direct mode.                                                                                                                                                                      |

Table 21-75: XSPI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W1C)         | DIR_CMD_ERR | Command Error. The XSPI_ISTAT.DIR_CMD_ERR bit is set (=1) when the host has attempted to do something that is unsupported or incorrect. For example, the host has triggered a write when the device is operating in XIP mode. The full list of error conditions is described in the Event Handling section.                                                                                                                                                                                                                                                                                 |
| 25 (R/W1C)         | DIR_DQS_ERR | DQS Error. The XSPI_ISTAT.DIR_DQS_ERR bit is set (=1) when an incorrect number of DQS pulses were detected during a direct mode read or status check. The status is passed from the soft PHY and could mean that the PHY is con- figured badly such that it expects DQS strobes but never received them or that the XSPI_PHY_GATE_LPBK_CTL .RD_DEL_SEL value is incorrect and has caused data corruption and pointer misalignment in the PHY. To resolve this issue, the PHY must be reset to clear the XSPI_PHY_OB0.DQS_UR and XSPI_PHY_OB0.DQS_OV flags and reset the read data pointers. |
| 24 (R/W1C)         | DIR_CRC_ERR | CRC Error. The XSPI_ISTAT.DIR_CRC_ERR bit is set (=1) when CRC checking is enabled and a CRC error after a read or status checking command in direct work mode has been detected.                                                                                                                                                                                                                                                                                                                                                                                                           |
| 23 (R/W1C)         | STIG_DONE   | STIG Done. The XSPI_ISTAT.STIG_DONE bit is set (=1) when the last instruction in a glued chain has completed.                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 22 (R/W1C)         | SDMA_ERR    | SDMA Error. The XSPI_ISTAT.SDMA_ERR bit is set (=1) when an illegal access to the target DMAinterface is detected.                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 21 (R/W1C)         | SDMA_TRIGG  | SDMA Trigger. The XSPI_ISTAT.SDMA_TRIGG bit indicates when the trigger condition for the target DMAis met. It is relevant to STIG and ACMD mode only and used to instruct the host when an access on the host interface is to be performed. For details, see the Operating in ACMD or STIG mode section.                                                                                                                                                                                                                                                                                    |
| 20 (R/W1C)         | CMD_IGNORED | Command Ignored. In ACMD work mode, the controller detected that a command that was sent by the host was to an already busy thread and ignored it. In STIG work mode, the controller detected that a command was sent when the STIG engine was already busy and ignored it.                                                                                                                                                                                                                                                                                                                 |
| 18 (R/W1C)         | DDMA_TERR   | Bus Completer Data DMAError. ACMD mode only. The XSPI_ISTAT.DDMA_TERR bit is set (=1) when a system bus error was detected on the internal DMAinterface during data reads or writes.                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 21-75: XSPI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/W1C)         | CDMA_TERR  | Command DMAError. Command DMAtarget error. ACMD mode only. The XSPI_ISTAT.CDMA_TERR bit is set (=1) when a system bus error was detected on the internal DMAinterface during descriptor reads or writes. |
| 16 (R/W1C)         | CTL_IDLE   | xSPI Controller Idle. This is general status. The XSPI_ISTAT.CTL_IDLE bit indicates that the xSPI controller has returned to an idle state.                                                              |
| 15 (R/W1C)         | GP_OD3     | GP OD3 Transition Status. The XSPI_ISTAT.GP_OD3 bit indicates a high-to-low or low-to-high transition was detected on the XSPI_DFI_GP_OPEN_DRAIN[3] input pin.                                           |
| 14 (R/W1C)         | GP_OD2     | GP OD2 Transition Status. The XSPI_ISTAT.GP_OD2 bit indicates a high-to-low or low-to-high transition detected on the XSPI_DFI_GP_OPEN_DRAIN[2] input pin.                                               |
| 13 (R/W1C)         | GP_OD1     | GP OD1 Transition Status. The XSPI_ISTAT.GP_OD1 bit indicates a high-to-low or low-to-high transition detected on the XSPI_DFI_GP_OPEN_DRAIN[1] input pin.                                               |
| 12 (R/W1C)         | GP_OD0     | GP OD0 Transition Status. The XSPI_ISTAT.GP_OD0 bit indicates a high-to-low or low-to-high transition detected on the XSPI_DFI_GP_OPEN_DRAIN[0] input pin.                                               |

## Long Polling Count Register

The XSPI\_LONGPOL\_GCTL register contains the wait count value for long polling.

Figure 21-34: XSPI\_LONGPOL\_GCTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000032_8f0704f9fad1a8a7fbced356693a9eb78e58de69a8c01653700918851ce163ed.png)

Table 21-76: XSPI\_LONGPOL\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Long Poll Value. The XSPI_LONGPOL_GCTL.VALUE bit field indicates the number of system clock cycles after an erase/write operation has been issued before the controller starts to check device status (ready/busy and fail/pass). First status checking polling happens after at least this many number of system clock cycles. The next status checking will happen every XSPI_SHORTPOLL_GCTL.VALUE cycles. The long polling value should be significantly larger the short polling value. |

## Device Active Maximum Clock Cycle Register

The XSPI\_MINICTL\_DEV\_ACTIVE\_MAX register is used to introduce a maximum number of xSPI clock cycles through which the CS number is kept active (low) on the memory interface. It is sometimes referred to as tCMS or tCEM timing. It is generally only relevant for RAM devices that must be periodically refreshed. The refresh is typically handled internal to the device. But, to fit this refresh operation along with normal controller requests, the controller my need to limit its access time. The user can program this register to force the controller to obey that requirement.

Figure 21-35: XSPI\_MINICTL\_DEV\_ACTIVE\_MAX Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000033_4564f36d449e31863edbbe2319770e9481be51174cd0ce6e5c661f1aaa0c95cc.png)

Table 21-77: XSPI\_MINICTL\_DEV\_ACTIVE\_MAX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Value. The XSPI_MINICTL_DEV_ACTIVE_MAX.VALUE bit field is only valid when XSPI_SEQ_GTL0.TCMS_EN is set (=1). When using the STIG work mode, this is also only valid when the TCMS_EN bit of the STIG instruction is set. The XSPI_MINICTL_DEV_ACTIVE_MAX.VALUE bit field must only be enabled while working with RAM devices that require timing constraint for chip select low pulse width (the most common name is tCMS or tCEM in device specification). |

## Clock Mode Control Register

The XSPI\_MINICTL\_CLKMODE register defines the CPOL/CPHA settings for legacy SPI mode (defaulting to mode 0).

Figure 21-36: XSPI\_MINICTL\_CLKMODE Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000034_e32240e78cd839d86b1e831e6e765020c953664879fa7fd7fc5fa27d7f8f4c24.png)

Table 21-78: XSPI\_MINICTL\_CLKMODE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | VALUE      | SPI Clock Mode Select. The XSPI_MINICTL_CLKMODE.VALUE bit indicates the SPI clock mode. For DDR transfers, this bit must be cleared (=0) to meet DDR flash timings. For SDR transfers, allowable values are as follows: 0 - SPI mode 0 (clock is low when SPI bus is in idle) 1 - SPI mode 3 (clock is high when SPI bus is in idle) |

## Device Delay Register

The XSPI\_MINICTL\_DEV\_DLY register is used to introduce relative device selection delays with respect to generated xSPI flash interface.

Figure 21-37: XSPI\_MINICTL\_DEV\_DLY Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000035_4d76e469aa955c08ba35f85674996ce3aa6a99536dd1280c2871be20f45b0938.png)

Table 21-79: XSPI\_MINICTL\_DEV\_DLY Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                       |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | CSDA_MIN_DLY | Minimum Chip Select Deassertion. The XSPI_MINICTL_DEV_DLY.CSDA_MIN_DLY bit field indicates the Mini- mum Chip Select (CSDA_MIN) deassertion timing.                                                           |
| 15:8 (R/W)         | CSEOT_DLY    | Chip Select End Of Transfer. The XSPI_MINICTL_DEV_DLY.CSEOT_DLY bit field indicates the Chip Select End Of Transfer (CSEOT). It improves the CS deassertion device timing to the last active clock edge.      |
| 7:0 (R/W)          | CSSOT_DLY    | Chip Select Start Of Transfer. The XSPI_MINICTL_DEV_DLY.CSSOT_DLY bit field indicates the Chip Select Start Of Transfer (CSSOT). It improves the CS deassertion device timing to the first active clock edge. |

## HyperFlash Offset Register

The XSPI\_MINICTL\_HF\_OFFSET register is only applicable for legacy HyperFlash or xSPI profile 2 devices and can be used to configure the reserved area of the Command/Address (CA) field.

Figure 21-38: XSPI\_MINICTL\_HF\_OFFSET Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000036_39d655758ff8d4f3a1ae22330dc3a5aaf25288aede82aafce9be9327273239e8.png)

Table 21-80: XSPI\_MINICTL\_HF\_OFFSET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------|
| 13:8 (R/W)         | SIZ        | Offset Size. The XSPI_MINICTL_HF_OFFSET.SIZ bit field indicates the offset size of the reserved area in the command format.    |
| 5:0 (R/W)          | IDX        | Start Index. The XSPI_MINICTL_HF_OFFSET.IDX bit field indicates the starting index of the reserved area in the command format. |

## JEDEC Reset Delay Register

The XSPI\_MINICTL\_JEDEC\_RST\_TR register is used to introduce relative device selection delays applicable for JEDEC reset instruction.

Figure 21-39: XSPI\_MINICTL\_JEDEC\_RST\_TR Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000037_ae171b5096c31d79efc610d295e1321b25a56d9fc9b5b89b336cc91b417ef0f8.png)

Table 21-81: XSPI\_MINICTL\_JEDEC\_RST\_TR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | TCSL_DLY   | TCSL Timing Configuration. The XSPI_MINICTL_JEDEC_RST_TR.TCSL_DLY bit field indicates how many xSPI clock cycles constitute tCSL timing of JEDEC reset instruction. |
| 7:0 (R/W)          | TCSH_DLY   | TCSH Timing Configuration. The XSPI_MINICTL_JEDEC_RST_TR.TCSH_DLY bit field indicates how many xSPI clock cycles constitute tCSH timing of JEDEC reset instruction. |

## Hardware Reset Control Register

The XSPI\_MINICTL\_RST\_PIN\_CTL register configures the software controlled hardware reset.

Figure 21-40: XSPI\_MINICTL\_RST\_PIN\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000038_9325f01700e2fcf587c93f2ad143c8fad8d386b66ca67632655c76d14b4f935e.png)

Table 21-82: XSPI\_MINICTL\_RST\_PIN\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | RST_BNK7   | Bank 7 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK7 bit activates software control- led hardware reset signal on bank 7 (such as a device connected to chip select bit 7). It is applicable only for devices that support the RESET pin. 0 Disable        |
| 14 (R/W)           | RST_BNK6   | Bank 6 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK6 bit activates software control- led hardware reset signal on bank 6 (such as a device connected to chip select bit 6). It is applicable only for devices that support the RESET pin. 0 Disable Enable |
| 14 (R/W)           | RST_BNK6   | 1                                                                                                                                                                                                                                                         |
| 14 (R/W)           | RST_BNK6   |                                                                                                                                                                                                                                                           |

Table 21-82: XSPI\_MINICTL\_RST\_PIN\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | RST_BNK5   | Bank 5 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK5 bit activates software control- led hardware reset signal on bank 5 (such as a device connected to chip select bit 5). It is applicable only for devices that support the RESET pin. |
| 13 (R/W)           | RST_BNK5   | 0 Disable                                                                                                                                                                                                                                |
| 13 (R/W)           | RST_BNK5   | 1 Enable                                                                                                                                                                                                                                 |
| 12 (R/W)           | RST_BNK4   | Bank 4 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK4 bit activates software control- led hardware reset signal on bank 0 (such as a device connected to chip select bit 4). It is applicable only for devices that support the RESET pin. |
| 12 (R/W)           | RST_BNK4   | 0 Disable                                                                                                                                                                                                                                |
| 12 (R/W)           | RST_BNK4   | 1 Enable                                                                                                                                                                                                                                 |
| 11 (R/W)           | RST_BNK3   | Bank 3 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK3 bit activates software control- led hardware reset signal on bank 3 (such as a device connected to chip select bit 3). It is applicable only for devices that support the RESET pin. |
| 11 (R/W)           | RST_BNK3   | 0 Disable                                                                                                                                                                                                                                |
| 11 (R/W)           | RST_BNK3   | 1 Enable                                                                                                                                                                                                                                 |
| 10 (R/W)           | RST_BNK2   | Bank 2 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK2 bit activates software control- led hardware reset signal on bank 2 (such as a device connected to chip select bit 2). It is applicable only for devices that support the RESET pin. |
| 10 (R/W)           | RST_BNK2   | 0 Disable                                                                                                                                                                                                                                |
| 10 (R/W)           | RST_BNK2   | 1 Enable                                                                                                                                                                                                                                 |
| 9 (R/W)            | RST_BNK1   | Bank 1 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK1 bit activates software control- led hardware reset signal on bank 1 (such as a device connected to chip select bit 1). It is applicable only for devices that support the RESET pin. |
| 9 (R/W)            | RST_BNK1   | 0 Disable                                                                                                                                                                                                                                |
| 9 (R/W)            | RST_BNK1   | 1 Enable                                                                                                                                                                                                                                 |

Table 21-82: XSPI\_MINICTL\_RST\_PIN\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | RST_BNK0       | Bank 0 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_BNK0 bit activates the software con- trolled hardware reset signal on bank 0 (such as a device connected to chip select bit 0). It is applicable only for devices that support the RESET pin.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 4 (R/W)            | RST_OPT        | Hardware Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_OPT bit indicates the hardware reset option.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            | RST_DQ3_ENABLE | Enable DQ3 Reset. The XSPI_MINICTL_RST_PIN_CTL.RST_DQ3_ENABLE bit enables passing RESET to the DQ3 port of the device (by switching direction of DQ3 pad).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W)            | HW_RST         | Software Controlled Hardware Reset. The value of this field is directly routed to the DQ3 or RESET# output signal (depending on XSPI_MINICTRL_RST_PIN_CTL. RST_OPT). The value can be changed only when xSPI flash interface is in an idle state. The value of the software controlled hardware reset signal is overwritten when DQ3 is a valid transaction pin. The controller does not check the device hardware reset setup/hold timings - this must be ensured by the host. The host is also responsible for triggering a suitable reset method by selecting the cor- responding bank number and reset method (as defined in this register). The controller does not drive RESET value during read data phase of any active transfer (as transfer direction switches in this transfer part). |

## Reset Recovery Delay Register

The XSPI\_MINICTL\_RST\_RECOV register is used to introduce relative reset recovery delay with respect to generated xSPI flash interface.

Figure 21-41: XSPI\_MINICTL\_RST\_RECOV Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000039_c3b679ef83a492894df82b8c8a69f5f7652739c0353de4da51e51886409b2b96.png)

Table 21-83: XSPI\_MINICTL\_RST\_RECOV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Additional Delay for CS Deassertion. The XSPI_MINICTL_RST_RECOV.VALUE bit field defines the additional delay for CS deassertion to accommodate device reset recovery timing. |

## Write Protect Register

The XSPI\_MINICTL\_WP\_CTL register controls write protection.

Figure 21-42: XSPI\_MINICTL\_WP\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000040_47727bb7c5b4479e43a63435b7f57aa56f6c3926d45652ec8651b5799ce9aa6e.png)

Table 21-84: XSPI\_MINICTL\_WP\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | EN         | Enable Write Protection. The XSPI_MINICTL_WP_CTL.EN bit enables passing the write protect signal to the device (by switching direction of DQ2 pad).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W)            | WP         | Write Protect. The XSPI_MINICTL_WP_CTL.WP bit indicates the write protect signal for all devices. Value of this register is directly routed to the DQ2 output signal. The value can be changed only when xSPI flash interface is in an idle state. Value of the write protection signal is overwritten in case DQ2 is a valid transaction pin. The controller does not check the write protect setup/hold timings - this must be ensured by the host. The controller does not drive the write protect value during the read data phase of any active transfer. Write protection on DQ2 functionality is only supported by flash devices and when the controller is configured in single and dual SPI modes. |

## PHY DLL Observable Points 0 Register

The XSPI\_PHY\_DLLOB0 register holds observable points in the PHY.

Figure 21-43: XSPI\_PHY\_DLLOB0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000041_db2ff9687bebe5bc9a7eea67af7b01300de786d3023949b1b9ad6f556e8dcbe3.png)

Table 21-85: XSPI\_PHY\_DLLOB0 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | LOCK_INC_DBG   | DLL Lock Increment State. The XSPI_PHY_DLLOB0.LOCK_INC_DBG bit field holds the state of the cumula- tive DLL_LOCK_INC register when the DLL_UNLOCK_CNT field (bits [7:3]) of this parameter was triggered to increment or was last saturated at a value of 0x1f.                                                                                                                                                                                                                                                                                                                                   |
| 23:16 (R/NW)       | LOCK_DEC_DBG   | DLL Lock Decrement State. The XSPI_PHY_DLLOB0.LOCK_DEC_DBG bit field holds the state of the cumula- tive DLL_LOCK_DEC register when the DLL_UNLOCK_CNT field (bits [7:3]) of this parameter was triggered to decrement or was last saturated at a value of 0x1f.                                                                                                                                                                                                                                                                                                                                   |
| 15:8 (R/NW)        | DLL_LOCK_VALUE | DLL Lock Delays. The XSPI_PHY_DLLOB0.DLL_LOCK_VALUE bit field indicates the number of delay elements that the DLL has determined for lock in either full clock or half clock mode. In full clock mode, this value equals the number of delay elements in one cycle. In half clock mode, this value equals the number of delay elements in one half clock cycle. In saturation mode, this value equals the maximum number of delay elements. The slaves use this value to configure their delays for the clock write and read DQS signals. This value is valid only when locking mechanism is done. |
| 7:3 (R/NW)         | DLL_UNLOCK_CNT | DLL Unlock Count. The XSPI_PHY_DLLOB0.DLL_UNLOCK_CNT bit field indicates the num- ber of times that the bus requester DLL consecutive increment or decrement value programmed in the DLL_LOCK_NUM field (bits [18:16]) of the XSPI_PHY_DLL_REQ_CTL register has been triggered. The DLL_UNLOCK_CNT saturates at a value of 0x1f. Asserting the XSPI_PHY_DLL_CTL.DLL_RST resets this counter to 0.                                                                                                                                                                                                  |

Table 21-85: XSPI\_PHY\_DLLOB0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2:1 (R/NW)         | DLL_LOCKED_MODE | DLL Lock Mode. The XSPI_PHY_DLLOB0.DLL_LOCKED_MODE bit field indicates the mode in which the DLL has achieved the lock.                                                                                                                                                                                                                                                                                                                 |
|                    |                 | 0 Full Clock Mode. The bus requester delay line was long enough to lock on one full clock cycle of delay. In this mode, the DLL_LOCK_VALUE field (bits [15:8]) of this parameter indicates the number of delays in full clock cycles.                                                                                                                                                                                                   |
|                    |                 | 1 Reserved                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |                 | 2 Half Clock Mode. The bus requester delay line was not long enough to lock one full cycle of delay but could lock on a half-cycle of delay. In this mode, the DLL_LOCK_VALUE field (bits [15:8]) of this param- eter indicates the number of delays in one half clock cycles.                                                                                                                                                          |
|                    |                 | 3 Saturation Mode. The bus requester delay line was not long enough to lock on a full or a half-clock cycle. In this mode, the encoder value is fixed at the maximum delay line setting and the bus requester DLL will be disabled. The bus completer delay lines continue to use the fractional delays based upon the fixed saturation value of the delay line.                                                                        |
| 0 (R/NW)           | DLL_LOCK        | DLL Lock Status. The XSPI_PHY_DLLOB0.DLL_LOCK bit indicates the status of the DLL locking when the DLL lock logic is found (not inc AND not dec) OR (an inc then dec) OR (a dec then inc). When XSPI_PHY_DLL_REQ_CTL.DLL_START_PT is configured to be smaller than half a clock period, the first found (a dec then inc) is not the DLL locking point , but DLL_LOCK is asserted. 0 - DLL has not locked 1 - DLL is locked 0 Not Locked |

## PHY DLL Observable Points 1 Register

The XSPI\_PHY\_DLLOB1 register holds observable points in the PHY.

Figure 21-44: XSPI\_PHY\_DLLOB1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000042_04b234fff04d9305ca36ab7c6fb9368b9f7a4cb1b008ab83e16e0852a4cef282.png)

Table 21-86: XSPI\_PHY\_DLLOB1 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                  |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/NW)       | DECODER_OUT_WR | Write Encoded Value. The XSPI_PHY_DLLOB1.DECODER_OUT_WR bit field holds the encoded value for the clock write delay line for this slice. |
| 7:0 (R/NW)         | DECODER_OUT_RD | Read Encoded Value. The XSPI_PHY_DLLOB1.DECODER_OUT_RD bit field holds the encoded value for the read delay line for this slice.         |

## PHY DLL Control Register

The XSPI\_PHY\_DLL\_CTL register configures the resynchronization of the bus completer DLL of the PHY. When the PHY is used with the xSPI controller, this register is automatically updated by device discovery during initialization.

Figure 21-45: XSPI\_PHY\_DLL\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000043_3d30bace9836a0e35871ed61d62b45cd2adc942c1aba70b371f50caa13247730.png)

Table 21-87: XSPI\_PHY\_DLL\_CTL Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/W)           | DFI_CTRLUPD_REQ | Resynchronize DLLs. The XSPI_PHY_DLL_CTL.DFI_CTRLUPD_REQ bit signals to resynchronize the DLLs and read and write FIFO pointers. To send the update request to the PHY, the host must first set this bit (=1), then, wait until the bit is cleared (=0). Do not use this signal when automatic resynchronization is enabled (XSPI_PHY_DLL_UPDT_CNT is not zero).                                                                                                                        |
| 24 (R/W)           | DLL_RST_N       | DLL Reset. The XSPI_PHY_DLL_CTL.DLL_RST_N bit resets the DLLs of the PHY and starts searching for lock again.                                                                                                                                                                                                                                                                                                                                                                           |
| 21 (R/W)           | SDR_EDGE_ACT    | SDR Mode Edge Active. The XSPI_PHY_DLL_CTL.SDR_EDGE_ACT bit indicates how the PHY sam- ples data on the edges of the sampling clock. In SDR mode, only one sample is needed. When XSPI_PHY_DLL_CTL.SDR_EDGE_ACT is cleared (=0), the con- troller propagates data from the positive edge of the PHY sampling clock. When XSPI_PHY_DLL_CTL.SDR_EDGE_ACT is set (=1), the controller propagates data from the negative edge of PHY sampling clock. In DDR mode, this bit must be cleared. |

Table 21-87: XSPI\_PHY\_DLL\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | DQS_LST_DAT_DRP_EN | Enable DQS Last Data Drop. XSPI_PHY_DLL_CTL.DQS_LST_DAT_DRP_EN is set (=1) when the flash de- vice issues data on the negative edge of the flash clock and returns them with DQS, and the PHY is configured to sample data in DQS mode. In this case, the number of DQS edges equals to number of requested data + 1. When XSPI_PHY_DLL_CTL.DQS_LST_DAT_DRP_EN is set, the controller internally requests this redundant data at the end of the transfer cleaning up the PHY FIFO. |
| 17 (R/W)           | EXTENDED_WR_MODE   | . This PHY register field is not applicable for xSPI flash Controller.                                                                                                                                                                                                                                                                                                                                                                                                             |
| 16 (R/W)           | EXTENDED_RD_MODE   | . This PHY register field is not applicable for xSPI flash controller.                                                                                                                                                                                                                                                                                                                                                                                                             |
| 11:8 (R/W)         | RESYNC_HI_WAIT_CNT | DLL Update Wait Count. The XSPI_PHY_DLL_CTL.RESYNC_HI_WAIT_CNT bit field defines the number of xSPI clock cycles for which the DLL update request (XSPI_PHY_DLL_CTL.DFI_CTRLUPD_REQ) must be asserted to resynchronize the DLLs and read and write FIFO pointers.                                                                                                                                                                                                                  |
| 7:0 (R/W)          | RESYNC_IDLE_CNT    | DLL Update Wait Time. The XSPI_PHY_DLL_CTL.RESYNC_IDLE_CNT bit field defines the wait time (in terms of xSPI clock cycles) between the deassertion of the DLL update request (XSPI_PHY_DLL_CTL.DFI_CTRLUPD_REQ) and resuming traffic to the PHY.                                                                                                                                                                                                                                   |

## Bus Requester DLL Control Register

The XSPI\_PHY\_DLL\_REQ\_CTL register controls the bus requester DLL logic.

Figure 21-46: XSPI\_PHY\_DLL\_REQ\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000044_57acdb370ae8980b26ee46898cc639aeda4cae06360accb14a86eee446c3b38d.png)

Table 21-88: XSPI\_PHY\_DLL\_REQ\_CTL Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23 (R/W)           | DLL_BYPASS_MODE | Bus Requester DLL Mode Enable. The XSPI_PHY_DLL_REQ_CTL.DLL_BYPASS_MODE bit enables DLL bypass mode control. It controls the bypass mode of the bus requester and bus completer DLLs. The XSPI_PHY_DLL_REQ_CTL.DLL_BYPASS_MODE bit is intended to be used only for debug.                                                                          |
| 22:20 (R/W)        | PHASE_DT_SEL    | Phase Detect Delay. The XSPI_PHY_DLL_REQ_CTL.PHASE_DT_SEL bit field indicates the number of delay elements to be inserted between the phase detect flip-flops. Defaults to 0x0, although the recommended value is 2 elements. But, if a lock condition is not detected, the user should increase the number of delay elements. 0 One Delay Element |
| 22:20 (R/W)        |                 | 1 Two Delay Elements                                                                                                                                                                                                                                                                                                                               |
| 22:20 (R/W)        |                 |                                                                                                                                                                                                                                                                                                                                                    |

Table 21-88: XSPI\_PHY\_DLL\_REQ\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |              | 2 Three Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |              | 3 Four Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |              | 4 Five Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|                    |              | 5 Six Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |              | 6 Seven Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |              | 7 Eight Delay Elements                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 18:16 (R/W)        | DLL_LOCK_NUM | DLL Lock Number. The XSPI_PHY_DLL_REQ_CTL.DLL_LOCK_NUM bit field holds the number of consecutive increment or decrement indications that trigger an unlock condition and increment the XSPI_PHY_DLLOB0.DLL_UN- LOCK_CNT field and either the XSPI_PHY_DLLOB0.LOCK_DEC_DBG or XSPI_PHY_DLLOB0.LOCK_DEC_DBG fields.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 7:0 (R/W)          | DLL_START_PT | Initial Delay Start. The XSPI_PHY_DLL_REQ_CTL.DLL_START_PT bit field indicates the initial delay value for the DLL. This value is also used as the increment value if the initial value is less than a half-clock cycle. This field must be set such that it is not greater than 7/8ths of a clock period given the worst case element delay. For example, if the frequency is 200MHz (5ns cycle time) with a worst case element 80ps delay, this field must be configured to = 5 * (7/8) / .080 = 54 elements. This calculation helps determine the start point which achieves the fastest lock. However, a small value such as 0x04 may be used instead to ensure that the DLL does not lock on a harmonic. Note that with a small value like this, the initial lock time is longer. Value smaller than 0x04 may cause no lock by DLL. |

## Bus Completer DLL Control Register

The XSPI\_PHY\_DLL\_COMP\_CTL register controls the bus completer DLL.

Figure 21-47: XSPI\_PHY\_DLL\_COMP\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000045_a41d945faa3135e240010f25dbffd5494d362708e04b09eab32794c7b58bb6a8.png)

Table 21-89: XSPI\_PHY\_DLL\_COMP\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | CLK_WR_DLY | Clock Write Delay. The XSPI_PHY_DLL_COMP_CTL.CLK_WR_DLY bit field controls the clock write delay line which adjusts the write DQbit timing in 1/256th steps of the clock period in normal DLL locked mode. In bypass mode, this field directly programs the number of delay elements. |
| 7:0 (R/W)          | RD_DQS_DLY | DQS Read Delay. The XSPI_PHY_DLL_COMP_CTL.RD_DQS_DLY bit field controls the read DQS delay which adjusts the timing in 1/256th of the clock period when in normal DLL locked mode. In bypass mode, this field directly programs the number of delay elements.                         |

## PHY DLL Resynchronization Register

The XSPI\_PHY\_DLL\_UPDT\_CNT register configures the resynchronization of the bus completer DLL of the PHY.

Figure 21-48: XSPI\_PHY\_DLL\_UPDT\_CNT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000046_260d9001d2912a3f72cac139800ba1e3944347c491734279d94880483d5b9b04.png)

Table 21-90: XSPI\_PHY\_DLL\_UPDT\_CNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | RESYNC_CNT | Resynchronization of Bus Completer DLL of PHY. The XSPI_PHY_DLL_UPDT_CNT.RESYNC_CNT bit field defines the time interval (in terms of xSPI clock) to send an update (XSPI_PHY_DLL_CTL.DFI_CTRLUPD_REQ=1) to the PHY to re-synchronize the bus completer DLL values with that of the bus requester DLL and to also re-synchronize the read and write FIFO pointers in the read path. If the value in this field is zero, the controller will not further DLL update requests to the PHY. DFI_CTRLUPD_REQ signal can be controlled directly by the host using XSPI_PHY_DLL_CTL.DFI_CTRLUPD_REQ. Note: While this feature is enabled, access to the PHY registers shall not be performed. |

## PHY DQS Timing Register

The XSPI\_PHY\_DQS\_TR register controls the DQS related timing.

Figure 21-49: XSPI\_PHY\_DQS\_TR Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000047_5f30e70d8844836361884d2144accd87ff7167a2652d5f3b5ebd7ff02bc49334.png)

Table 21-91: XSPI\_PHY\_DQS\_TR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | LPBK_DQS   | Internal Loop Back Configuration. The XSPI_PHY_DQS_TR.LPBK_DQS bit is used in conjunction with bits 22 and 20 to control how read data is sampled by the PHY. The XSPI_PHY_DQS_TR.LPBK_DQS bit is only valid when bit 20 is set (=1), meaning it is only relevant when read data is not being sampled by DQS from the memory device. If bit 20 is cleared (=0), the XSPI_PHY_DQS_TR.LPBK_DQS bit should also be cleared. When using the PHY with the xSPI controller, this bit must be set to the same value as bit 20.This bit selects which internal source is used by the PHY to sample the read data. This is internally generated and is passed out of the PHY via rebar_opad If bit 22 of this register is '0', then it will be internally looped back within the rebar pad. If bit 22 of this register is '1', the rebar_opad will be passed through the rebar pad and it will be the responsibility of the integrator to loop that back to the PHY into the lpbk_dqs pin. 0 - Use phony DQS for data capture 1 - Use lpbk_dqs for data capture. [/list]. |

Table 21-91: XSPI\_PHY\_DQS\_TR Register Fields (Continued)

| Bit No. (Access)   | Bit Name           | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|--------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | PHONY_DQS          | DQS Mode Configuration. The XSPI_PHY_DQS_TR.PHONY_DQS bit is used in conjunction with bits 22 and 21 to control how read data is sampled by the PHY. The XSPI_PHY_DQS_TR.PHONY_DQS bit selects whether the read data sent by the memory device is sampled by DQS supplied by the memory device, or by a signal locally generated within the PHY. |
| 16 (R/W)           | PHONY_DQS_SEL      | PHONY DQS Configuration. When XSPI_PHY_DQS_TR.PHONY_DQS_SEL is cleared (=0), the PHO- NY_DQS is synchronous with rising edge of the CLK_PHY before sending to the entry flops. When XSPI_PHY_DQS_TR.PHONY_DQS_SEL is set (=1), the PHO- NY_DQS signal is synchronous with the falling edge of CLK_PHY before sending to the entry flops.         |
| 15:12 (R/W)        | DQS_SEL_TSEL_START | DQPad Termination Select Enable Time. The XSPI_PHY_DQS_TR.DQS_SEL_TSEL_START bit field defines the DQpad dynamic termination select enable time. Larger values add greater delay to when tsel turns on. Each bit changes the output enable time by a 1/2 cycle resolution.                                                                       |
| 11:8 (R/W)         | DQS_SEL_TSEL_END   | DQPad Termination Select Disable Time. The XSPI_PHY_DQS_TR.DQS_SEL_TSEL_END bit field defines the DQpad dynamic termination select disable time. Larger values increase the delay to when tsel turns off. Each bit changes the output enable time by a 1/2 cycle resolution.                                                                     |

## PHY DQ Timing Register

The XSPI\_PHY\_DQ\_TR register controls the DQ related timing.

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000048_9afbcdcdbeef172bb7cbe0671bc0fe37d241376746707b5761c48a1794651bd7.png)

Write Data Path Additional Latency

Figure 21-50: XSPI\_PHY\_DQ\_TR Register Diagram

Table 21-92: XSPI\_PHY\_DQ\_TR Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                            |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | DAT_CLKPRD_DLY | Write Data Path Additional Latency. The XSPI_PHY_DQ_TR.DAT_CLKPRD_DLY bit field defines additional latency on the write data path. It also adds a clock cycle delay for the data OE path which is equivalent of adding 2 to DAT_OE_END and DAT_OE_START.                                                           |
| 15:12 (R/W)        | DAT_TSEL_START | DQPad Time Delay Enable. The XSPI_PHY_DQ_TR.DAT_TSEL_START bit field defines the DQpad dy- namic termination select enable time. Larger values add greater delay to when tsel turns on. Each bit changes the output enable time by a 1/2 cycle resolution.                                                         |
| 11:8 (R/W)         | DAT_TSEL_END   | DQPad Disable Time Delay Cycle. The XSPI_PHY_DQ_TR.DAT_TSEL_END bit field defines the DQpad dynamic termination select disable time. Larger values increase the delay to when tsel turns off. Each bit changes the output enable time by a 1/2 cycle resolution.                                                   |
| 6:4 (R/W)          | DAT_OE_START   | DQPad OE Window Start Point. The XSPI_PHY_DQ_TR.DAT_OE_START bit field adjusts the starting point of the DQpad output enable window. Lower numbers pull the rising edge earlier in time and larger numbers cause the rising edge to be delayed. Each bit changes the output enable time by a 1/2 cycle resolution. |
| 2:0 (R/W)          | DAT_OE_END     | DQPad OE Window End Point. The XSPI_PHY_DQ_TR.DAT_OE_END bit field adjusts the ending point of the DQpad output enable window. Lower numbers pull the falling edge earlier in time and larger numbers cause the falling edge to be delayed. Each bit changes the output enable time by a 1/2 cycle resolution.     |

## PHY Features Register

The XSPI\_PHY\_FEATURES register shows the available hardware features.

Figure 21-51: XSPI\_PHY\_FEATURES Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000049_f6fc190227c8732146deb2b4401be18e5e731f11132161c4896dfbfca92fa818.png)

Table 21-93: XSPI\_PHY\_FEATURES Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                            |
|--------------------|--------------|--------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | ASF          | Automotive Safety. The XSPI_PHY_FEATURES.ASF bit indicates support for an automotive safety feature.               |
| 14 (R/NW)          | PLL          | PLL. The XSPI_PHY_FEATURES.PLL bit indicates support for PLL.                                                      |
| 13 (R/NW)          | JTAG_SUP     | JTAG. The XSPI_PHY_FEATURES.JTAG_SUP bit indicates support for JTAG multi- plexing.                                |
| 12 (R/NW)          | EXT_LPBK_DQS | External Loopback DQS. The XSPI_PHY_FEATURES.EXT_LPBK_DQS bit indicates support for an exter- nal LPBK_DQS IO pad. |

Table 21-93: XSPI\_PHY\_FEATURES Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                        |
|--------------------|-----------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/NW)          | REG_INTF        | SFR Interface. The XSPI_PHY_FEATURES.REG_INTF bit indicates the SFR interface type. This is an encoded value.                                  |
| 11 (R/NW)          | REG_INTF        | 0 DFI                                                                                                                                          |
| 11 (R/NW)          | REG_INTF        | 1 APB                                                                                                                                          |
| 10 (R/NW)          | PER_BIT_DESKEW  | Per Bit Deskew. The XSPI_PHY_FEATURES.PER_BIT_DESKEW bit indicates support for per-bit deskew.                                                 |
| 9 (R/NW)           | DFI_CLOCK_RATIO | DFI Clock Ratio. The XSPI_PHY_FEATURES.DFI_CLOCK_RATIO bit indicates support for a clock ratio on the DFI interface. This is an encoded value. |
| 9 (R/NW)           | DFI_CLOCK_RATIO | 0 1:1                                                                                                                                          |
| 9 (R/NW)           | DFI_CLOCK_RATIO | 1 1:2                                                                                                                                          |
| 8 (R/NW)           | AGING           | Aging. The XSPI_PHY_FEATURES.AGING bit indicates support for aging in delay lines.                                                             |
| 7 (R/NW)           | DLL_TAP_NUM     | Number of Taps in Delay. The XSPI_PHY_FEATURES.DLL_TAP_NUM bit indicates the number of taps in the delay line.                                 |
| 7 (R/NW)           | DLL_TAP_NUM     | 0 128                                                                                                                                          |
| 7 (R/NW)           | DLL_TAP_NUM     | 1 256                                                                                                                                          |
| 6:5 (R/NW)         | BNK_NUM         | Max Banks. The XSPI_PHY_FEATURES.BNK_NUM bit field indicates the maximum number of banks supported by hardware. This is an encoded value.      |
| 6:5 (R/NW)         | BNK_NUM         | 0 One bank                                                                                                                                     |
| 6:5 (R/NW)         | BNK_NUM         | 1 Two banks                                                                                                                                    |
| 6:5 (R/NW)         | BNK_NUM         | 2 Four banks                                                                                                                                   |
| 6:5 (R/NW)         | BNK_NUM         | 3 Eight banks                                                                                                                                  |
| 4 (R/NW)           | SD_EMMC         | SD/EMMC. The XSPI_PHY_FEATURES.SD_EMMC bit indicates support for SD/eMMC.                                                                      |
| 3 (R/NW)           | XSPI            | XSPI. The XSPI_PHY_FEATURES.XSPI bit indicates support for xSPI.                                                                               |

Table 21-93: XSPI\_PHY\_FEATURES Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | SDR_16BIT  | 16-Bit SDR. The XSPI_PHY_FEATURES.SDR_16BIT bit indicates support for 16-bit in ON- FI SDR work mode. |
| 1 (R/NW)           | ONFI_41    | ONFI41. The XSPI_PHY_FEATURES.ONFI_41 bit indicates support for ONFI4.1 - NAND flash.                 |
| 0 (R/NW)           | ONFI_40    | ONFI40. The XSPI_PHY_FEATURES.ONFI_40 bit indicates support for ONFI4.0 - NAND flash.                 |

## PHY Gate Loopback Control Register

The XSPI\_PHY\_GATE\_LPBK\_CTL register controls the gate and loopback control related timing.

Figure 21-52: XSPI\_PHY\_GATE\_LPBK\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000050_f90edbfa9161c440c63a6a989eb58190672c632078f44b687247157573a96868.png)

Table 21-94: XSPI\_PHY\_GATE\_LPBK\_CTL Register Fields

| Bit No. (Access)   | Bit Name           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|--------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:19 (R/W)        | RD_DEL_SEL         | Read Data Delay. The XSPI_PHY_GATE_LPBK_CTL.RD_DEL_SEL bit defines the read data delay. It indicates the number of cycles to delay the dfi_rddata_en signal prior to enabling the read FIFO. After this delay, the read pointers begin incrementing the read FIFO. If sync_method is set (=1) the value of this field must take into account the synchroni- zation time of the pointers in the entry FIFO (adding three clock cycles should be sufficient). |
| 15:13 (R/W)        | LPBK_ERR_CHECK_TIM | Loopback Error Check. The XSPI_PHY_GATE_LPBK_CTL.LPBK_ERR_CHECK_TIM bit field indicates the cycle delay between the LFSR and loopback error check logic to ensure that the LFSR sourced data and data being looped back arrive at the same clock cycle for com- parison. This value is related to the XSPI_PHY_GATE_LPBK_CTL .RD_DEL_SEL field, and is equal to 7 - RD_DEL_SEL.                                                                             |
| 12 (R/W)           | LPBK_FAIL_MUXSEL   | Data Output. The XSPI_PHY_GATE_LPBK_CTL.LPBK_FAIL_MUXSEL bit selects the data output type for XSPI_PHY_OB0.LPBK_DQ_DAT. expected data                                                                                                                                                                                                                                                                                                                       |
| 12 (R/W)           | LPBK_FAIL_MUXSEL   | 0 Return the                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 12 (R/W)           | LPBK_FAIL_MUXSEL   | 1 Return the actual data                                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 21-94: XSPI\_PHY\_GATE\_LPBK\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | LPBK_CTL       | Loopback Control. The XSPI_PHY_GATE_LPBK_CTL.LPBK_CTL bit field indicates the loopback control.                                                                                                                                                                                                                                                                           |
| 11:10 (R/W)        | LPBK_CTL       | 0 Normal operation mode                                                                                                                                                                                                                                                                                                                                                   |
| 11:10 (R/W)        | LPBK_CTL       | 1 Loopback start. Enable loopback write mode.                                                                                                                                                                                                                                                                                                                             |
| 11:10 (R/W)        | LPBK_CTL       | 2 Loopback stop. Stop loopback to check error register.                                                                                                                                                                                                                                                                                                                   |
| 11:10 (R/W)        | LPBK_CTL       | 3 Clear. Clear loopback registers.                                                                                                                                                                                                                                                                                                                                        |
| 9 (R/W)            | LPBK_INT       | Loopback Multiplexer. The XSPI_PHY_GATE_LPBK_CTL.LPBK_INT bit controls the loopback read multiplexer.                                                                                                                                                                                                                                                                     |
| 9 (R/W)            | LPBK_INT       | 0 External loopback                                                                                                                                                                                                                                                                                                                                                       |
| 9 (R/W)            | LPBK_INT       | 1 Internal loopback                                                                                                                                                                                                                                                                                                                                                       |
| 8 (R/W)            | LPBK_EN        | Loopback Enable. The XSPI_PHY_GATE_LPBK_CTL.LPBK_EN bit controls the internal write mul- tiplexer.                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | LPBK_EN        | 0 Normal operation                                                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | LPBK_EN        | 1 Enable loopback                                                                                                                                                                                                                                                                                                                                                         |
| 5:4 (R/W)          | GATE_CFG_CLOSE | Gate Close. The XSPI_PHY_GATE_LPBK_CTL.GATE_CFG_CLOSE bit field allows extend- ing the closing of the DQS gate. Recommended value is zero. rebar_dfi - Read enable output signal. It is an internal signal between the xSPI PHY and xSPI controller. dfi_cebar - Peripheral select output (CS) signal. It is an internal signal between the xSPI PHY and xSPI controller. |
| 3:0 (R/W)          | GATE_CFG       | Gate Open Time. Coarse adjust of gate open time. The XSPI_PHY_GATE_LPBK_CTL.GATE_CFG bit field is the number of cycles to delay the dfi_rddata_en signal prior to opening the gate in full cycle increments. Decreasing this value pulls the gate earlier in time. This field must be programmed such that the gate signal lands in the valid DQS gate window.            |

## PHY Global Control Register

The XSPI\_PHY\_GCTL register handles the global control settings for the PHY.

Figure 21-53: XSPI\_PHY\_GCTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000051_5defef598ef7f87fbf05bbc05ab22ab13fd1a5abde1e6e8bcb1e8f72575f35cb.png)

Table 21-95: XSPI\_PHY\_GCTL Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                   |
|--------------------|----------------|---------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | CTL_CLKPRD_DLY | Control Latency. The XSPI_PHY_GCTL.CTL_CLKPRD_DLY bit indicates an additional latency on the control signals WE/RE/CE/WP. |

## PHY GPIO Control Register 0

The XSPI\_PHY\_GPIO\_CTL0 register is a general purpose register. The [31:0] vector is brought to the PHY I/Os. These pins can be used to control any static settings that may be required for the connected I/O pads.

Figure 21-54: XSPI\_PHY\_GPIO\_CTL0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000052_829ee13b4a6a12abd45f3ee9b666739faa9c255a900ac2db5944459d242c3d24.png)

Table 21-96: XSPI\_PHY\_GPIO\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | GPIO Control Value 0. The [31:0] vector is brought to the PHY I/Os. The XSPI_PHY_GPIO_CTL0.VALUE bit field can used to control any static settings that may be required for the connected IO pads. |

## PHY GPIO Control Register 1

The XSPI\_PHY\_GPIO\_CTL1 register is a general purpose register. The [31:0] vector is brought to the PHY I/Os. These pins can be used to control any static settings that may be required for the connected I/O pads.

Figure 21-55: XSPI\_PHY\_GPIO\_CTL1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000053_7beb990a9105da4d9c2c5a9e35d5615930c565527f30bbfe188ec26dc105a6a6.png)

Table 21-97: XSPI\_PHY\_GPIO\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | GPIO Control Value 1. The [31:0] vector is brought to the PHY I/Os. The XSPI_PHY_GPIO_CTL1.VALUE bit field can used to control any static settings that may be required for the connected IO pads. |

## PHY GPIO Status Register 0

The XSPI\_PHY\_GPIO\_STAT0 register is a general purpose register. A [31:0] vector is brought from the PHY I/Os to this register. This can be used as a status register.

Figure 21-56: XSPI\_PHY\_GPIO\_STAT0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000054_1bf83b82147b100cbb55bb718d1b21e6c1c125d2d5ed9e12393c52dd50661de5.png)

Table 21-98: XSPI\_PHY\_GPIO\_STAT0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | GPIO Status Value 0. A [31:0] vector is brought from the PHY IOs to this register. The XSPI_PHY_GPIO_STAT0.VALUE bit field can used as a status register. |

## PHY GPIO Status Register 1

The XSPI\_PHY\_GPIO\_STAT1 register is a general purpose register. A [31:0] vector is brought from the PHY I/Os to this register. This can be used as a status register.

Figure 21-57: XSPI\_PHY\_GPIO\_STAT1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000055_1e44e211c9842e7e1743d94f4261d8d4ddb42138114acbd7cad1924b3daa8147.png)

Table 21-99: XSPI\_PHY\_GPIO\_STAT1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | GPIO Status Value 1. A [31:0] vector is brought from the PHY I/Os to this register. The XSPI_PHY_GPIO_STAT1.VALUE bit field can be used as a status register. |

## PHY Global Termination Control Register

The XSPI\_PHY\_GTSEL register handles the global control settings for the termination selects for reads.

Figure 21-58: XSPI\_PHY\_GTSEL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000056_fcc91f711b186a2352e83e29a2254bacbb493998978eb16c0bef3b3e455ee4e4.png)

Table 21-100: XSPI\_PHY\_GTSEL Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                          |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:20 (R/W)        | OFF_VALUE_DAT | Termination Select Off Data. The XSPI_PHY_GTSEL.OFF_VALUE_DAT bit field indicates the termination se- lect off value for the data.               |
| 19:16 (R/W)        | RD_VALUE_DAT  | Termination Select Read Data. The XSPI_PHY_GTSEL.RD_VALUE_DAT bit field indicates the termination select read value for the data.                |
| 15:12 (R/W)        | OFF_VALUE_DQS | Termination Select Off Data Strobe. The XSPI_PHY_GTSEL.OFF_VALUE_DQS bit field indicates the termination se- lect off value for the data strobe. |
| 11:8 (R/W)         | RD_VALUE_DQS  | Termination Select Read Data Strobe. The XSPI_PHY_GTSEL.RD_VALUE_DQS bit field indicates the termination select read value for the data strobe.  |

## PHY DQS Input Enable Timing Register

The XSPI\_PHY\_IE\_TR register controls the DQS related timing.

Figure 21-59: XSPI\_PHY\_IE\_TR Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000057_8e64282d40b219f12f79be8c129d614f9c4ad01123159f2d44bdc271a3a6afdb.png)

Table 21-101: XSPI\_PHY\_IE\_TR Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/W)           | IE_ALWAYS_ON    | Input Enable Always On. The XSPI_PHY_IE_TR.IE_ALWAYS_ON bit forces the input enable(s) to be on always.                                                                                                                                                                                                                                                                                       |
| 18:16 (R/W)        | DQ_IE_START     | DQInput Enable Start. The XSPI_PHY_IE_TR.DQ_IE_START bit field indicates the start position for the DQinput enable.                                                                                                                                                                                                                                                                           |
| 14:12 (R/W)        | DQ_IE_STOP      | DQInput Enable Stop. The XSPI_PHY_IE_TR.DQ_IE_STOP bit field indicates the stop position for the DQinput enable.                                                                                                                                                                                                                                                                              |
| 10:8 (R/W)         | DQS_IE_START    | DQS Input Enable Start. The XSPI_PHY_IE_TR.DQS_IE_START bit field indicates the start position for the DQS input enable.                                                                                                                                                                                                                                                                      |
| 6:4 (R/W)          | DQS_IE_STOP     | DQS Input Enable Stop. The XSPI_PHY_IE_TR.DQS_IE_STOP bit field indicates the stop position for the DQS input enable.                                                                                                                                                                                                                                                                         |
| 3:0 (R/W)          | RDDAT_EN_IE_DLY | Internal Read Data Enable delay. The XSPI_PHY_IE_TR.RDDAT_EN_IE_DLY bit field indicates the number of clocks of delay for the DFI_RDDATA_EN signal to line it up with the true (normal) DFI read data position. The controller must deliver an early version of the read data enable to allow time for the input pads to turn on and this field allows the PHY to create the original timing. |

## PHY Observable Points Register

The XSPI\_PHY\_OB0 register holds observable points in the PHY.

Figure 21-60: XSPI\_PHY\_OB0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000058_17ba74111c2fece23885e17741a918567cf655f3405a2e65b9593957c1e7d4e7.png)

Table 21-102: XSPI\_PHY\_OB0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|-------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25 (R/NW)          | DQS_OVF     | DQS Overflow Error. The XSPI_PHY_OB0.DQS_OVF bit indicates that the logic gate was closed too late. For example, the number of DQS strobes exceed the capacity of the entry FIFO. It indicates that XSPI_PHY_GATE_LPBK_CTL.RD_DEL_SE signal value is too high and DFI_RDDATA are corrupted. It is possible that overflow status is asserted with underrun status - in such case the overflow takes the precedence. The XSPI_PHY_DLL_CTL.DLL_RST_N bit or RST_N signal clears this flag. |
| 24 (R/NW)          | DQS_UR      | DQS Underrun Error. The XSPI_PHY_OB0.DQS_UR bit indicates that the logic gate had to be forced closed. It indicates that either the DQS strobe did not appear during read or XSPI_PHY_GATE_LPBK_CTL.RD_DEL_SEL signal value is too low and DFI_RDDATA are corrupted. The XSPI_PHY_DLL_CTL.DLL_RST_N bit or RST_N signal clears this flag.                                                                                                                                               |
| 23:8 (R/NW)        | LPBK_DQ_DAT | Loopback Error. If errors are encountered in loopback test, the XSPI_PHY_OB0.LPBK_DQ_DAT bit field reports the actual data or the expected data, depending on the setting of the XSPI_PHY_GATE_LPBK_CTL.LPBK_FAIL_MUXSEL bit. This field is not clear by the clear state of the loopback. If there are no errors in loopback test, the value is zero (or value from previous state).                                                                                                    |

Table 21-102: XSPI\_PHY\_OB0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1:0 (R/NW)         | LPBK_STAT  | Loopback Status. The XSPI_PHY_OB0.LPBK_STAT bit field indicates the loopback status. Bit0 - loopback start; Defines the status of the loopback mode. 0 = Not in loopback mode; 1 = In loopback mode. Bit1 - loopback status; Defines the status of the loopback mode. 0 = Last Loopback test had no errors; 1 = Last loopback test contained data errors. |

## PHY Revision ID Register

The XSPI\_PHY\_REVID register contains the release identification number.

Figure 21-61: XSPI\_PHY\_REVID Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000059_026412f6e6435364e585e2a523e3d66215db80e400f677868367a8472ff38c5d.png)

Table 21-103: XSPI\_PHY\_REVID Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | PHY_MAGIC_NUM | Magic Number. The XSPI_PHY_REVID.PHY_MAGIC_NUM bit field indicates the magic number.                   |
| 15:8 (R/NW)        | PHY_FIX       | Fixed number. The XSPI_PHY_REVID.PHY_FIX bit field indicates the fixed number (minor revision number). |
| 7:0 (R/NW)         | PHY_REV       | PHY Revision. The XSPI_PHY_REVID.PHY_REV bit field indicates the PHY revision number.                  |

## PHY Static Aging Register

The XSPI\_PHY\_STATIC\_TGL register controls the static aging feature of the PHY.

Figure 21-62: XSPI\_PHY\_STATIC\_TGL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000060_c37f84cc000b659f737749d9de48455b166cafd1052361da6e708681f36b6fb2.png)

Table 21-104: XSPI\_PHY\_STATIC\_TGL Register Fields

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                                                                                                                               |
|--------------------|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22:20 (R/W)        | STATIC_TOGG_EN        | Static Toggle Enable. The XSPI_PHY_STATIC_TGL.STATIC_TOGG_EN bit field enables the toggle signal during static activity. When cleared (=0), the feature is disabled. Bit 0 - Bus requester delay line enable Bit 1 - Read path delay line enable Bit 2 - Write path delay line enable |
| 16 (R/W)           | STATIC_TOGG_GEN       | Global Static Toggle Enable. The XSPI_PHY_STATIC_TGL.STATIC_TOGG_GEN bit enables the toggle sig- nal during static activity.                                                                                                                                                          |
| 15:0 (R/W)         | STAT- IC_TOGG_CLK_DIV | Clock Divider. The XSPI_PHY_STATIC_TGL.STATIC_TOGG_CLK_DIV bit field indicates the clock divider used to create the toggle signal.                                                                                                                                                    |

## PHY Deskew Write Register

The XSPI\_PHY\_WR\_DESKEW\_PAD\_CTL0 register holds the values of the phase detect block for each DQ bit on the write path.

Figure 21-63: XSPI\_PHY\_WR\_DESKEW\_PAD\_CTL0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000061_875cb16a5ab37c2dce3e9ac079e429f167e005bf7c4ae0f87a0264725ca322ad.png)

Table 21-105: XSPI\_PHY\_WR\_DESKEW\_PAD\_CTL0 Register Fields

| Bit No. (Access)   | Bit Name                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|-------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | DQ_SW_DQ_PHASE_BY- PASS | DQPhase Detect. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_DQ_PHASE_BYPASS is set (=1), the clock write delay line configuration is used to determine the HALF_CYCLE_SHIFT. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_DQ_PHASE_BYPASS is cleared (=0), the phase detect circuit is used to determine the HALF_CYCLE_SHIFT.                                                                                                                                                                      | DQPhase Detect. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_DQ_PHASE_BYPASS is set (=1), the clock write delay line configuration is used to determine the HALF_CYCLE_SHIFT. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_DQ_PHASE_BYPASS is cleared (=0), the phase detect circuit is used to determine the HALF_CYCLE_SHIFT.                                                                                                                                                                      |
| 6 (R/W)            | DQ_SW_DQ_PHASE_BY- PASS | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Use phase detect circuit                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | DQ_SW_DQ_PHASE_BY- PASS | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Use the clock write delay line configuration. A de- lay line configuration of 0x00-0x7f means HALF_CY- CLE_SHIFT= 0 and a delay line configuration of 0x80-0xff means HALF_CYCLE_SHIFT = 1.                                                                                                                                                                                                                                                                                              |
| 5 (R/W)            | DQ_EN_SW_HALF_CY- CLE   | Half Cycle Shift Enable. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_EN_SW_HALF_CYCLE is set (=1), it enables the software half cycle shift. This determines whether the write data is transferred to the clock write domain on the positive or negative edge of the PHY clock. This field is valid when DQ_SW_DQ_PHASE_BYPASS =0. Note: If the user chooses to control the half cycle shift manually, it is important that DQ_SW_HALF_CYCLE_SHIFT = 0 if the delay is less than a 1/2 cycle and | Half Cycle Shift Enable. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_EN_SW_HALF_CYCLE is set (=1), it enables the software half cycle shift. This determines whether the write data is transferred to the clock write domain on the positive or negative edge of the PHY clock. This field is valid when DQ_SW_DQ_PHASE_BYPASS =0. Note: If the user chooses to control the half cycle shift manually, it is important that DQ_SW_HALF_CYCLE_SHIFT = 0 if the delay is less than a 1/2 cycle and |
| 5 (R/W)            | DQ_EN_SW_HALF_CY- CLE   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Hardware automatically controls shifting needed for the write level delay line.                                                                                                                                                                                                                                                                                                                                                                                                          |
| 5 (R/W)            | DQ_EN_SW_HALF_CY- CLE   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | DQ_SW_HALF_CYCLE_SHIFT field defines the shift                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 21-105: XSPI\_PHY\_WR\_DESKEW\_PAD\_CTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                      |
|--------------------|--------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DQ_SW_HALF_CY- CLE_SHIFT | Half Clock Delay Enable. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_HALF_CYCLE_SHIFT is set (=1), a half clock delay is added to the write data path.                                                                                                                                                                                            | Half Clock Delay Enable. When XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_SW_HALF_CYCLE_SHIFT is set (=1), a half clock delay is added to the write data path.                                                                                                                                                                                            |
| 4 (R/W)            | DQ_SW_HALF_CY- CLE_SHIFT | 0                                                                                                                                                                                                                                                                                                                                            | No effect                                                                                                                                                                                                                                                                                                                                    |
| 4 (R/W)            | DQ_SW_HALF_CY- CLE_SHIFT | 1                                                                                                                                                                                                                                                                                                                                            | Adds a half clock delay to the write data path                                                                                                                                                                                                                                                                                               |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | DLL Phase Detect Select. The XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_PHASE_DETECT_SEL bit field indicates the DLL phase detect selector for DQgeneration to handle the clock domain crossing between the clock and clock write signal. It indicates the number of delay elements to be inserted between the phase detect flip-flops. Defaults to 0x0. | DLL Phase Detect Select. The XSPI_PHY_WR_DESKEW_PAD_CTL0.DQ_PHASE_DETECT_SEL bit field indicates the DLL phase detect selector for DQgeneration to handle the clock domain crossing between the clock and clock write signal. It indicates the number of delay elements to be inserted between the phase detect flip-flops. Defaults to 0x0. |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 0                                                                                                                                                                                                                                                                                                                                            | One delay element                                                                                                                                                                                                                                                                                                                            |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 1                                                                                                                                                                                                                                                                                                                                            | Two delay elements                                                                                                                                                                                                                                                                                                                           |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 2                                                                                                                                                                                                                                                                                                                                            | Three delay elements                                                                                                                                                                                                                                                                                                                         |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 3                                                                                                                                                                                                                                                                                                                                            | Four delay elements                                                                                                                                                                                                                                                                                                                          |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 4                                                                                                                                                                                                                                                                                                                                            | Five delay elements                                                                                                                                                                                                                                                                                                                          |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 5                                                                                                                                                                                                                                                                                                                                            | Six delay elements                                                                                                                                                                                                                                                                                                                           |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 6                                                                                                                                                                                                                                                                                                                                            | Seven delay elements                                                                                                                                                                                                                                                                                                                         |
| 2:0 (R/W)          | DQ_PHASE_DE- TECT_SEL    | 7                                                                                                                                                                                                                                                                                                                                            | Eight delay elements                                                                                                                                                                                                                                                                                                                         |

## Program Sequence Configuration Register 0

The XSPI\_PROG\_SEQ\_CFG0 register configures the program sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-64: XSPI\_PROG\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000062_0ff86f98beb87396f3c7a545680eac9435d95eb7e4832e2ba6c15fdcbb2598eb.png)

Table 21-106: XSPI\_PROG\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                | Description/Enumeration                                                                                                                                |
|--------------------|--------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | P1_DMY_CNT   | Number of Dummy Cycle. The XSPI_PROG_SEQ_CFG0.P1_DMY_CNT bit field indicates the number of dummy cycles in profile 1. If 0, dummy cycles are disabled. | Number of Dummy Cycle. The XSPI_PROG_SEQ_CFG0.P1_DMY_CNT bit field indicates the number of dummy cycles in profile 1. If 0, dummy cycles are disabled. |
| 23 (R/W)           | P1_DAT_EDGE  | Data Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for the data phase.                                     | Data Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for the data phase.                                     |
| 21:20 (R/W)        | P1_DAT_IOS   | Data Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase.                 | Data Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase.                 |
|                    |              | 0                                                                                                                                                      | One data line used (for example, serial)                                                                                                               |
|                    |              | 1                                                                                                                                                      | Two data lines used                                                                                                                                    |
|                    |              | 2                                                                                                                                                      | Four data lines used                                                                                                                                   |
|                    |              | 3                                                                                                                                                      | Eight data lines used                                                                                                                                  |
| 19 (R/W)           | P1_ADDR_EDGE | Address Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR                                                          | Address Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR                                                          |

Table 21-106: XSPI\_PROG\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                       | Description/Enumeration                                                                                                                       |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| 17:16 (R/W)        | P1_ADDR_IOS  | Address Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_ADDR_IOS bit field indicates the number of data lines used to send the address phase. | Address Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_ADDR_IOS bit field indicates the number of data lines used to send the address phase. |
| 17:16 (R/W)        | P1_ADDR_IOS  | 0                                                                                                                                             | One data line used (for example, serial)                                                                                                      |
| 17:16 (R/W)        | P1_ADDR_IOS  | 1                                                                                                                                             | Two data lines used                                                                                                                           |
| 17:16 (R/W)        | P1_ADDR_IOS  | 2                                                                                                                                             | Four data lines used                                                                                                                          |
| 17:16 (R/W)        | P1_ADDR_IOS  | 3                                                                                                                                             | Eight data lines used                                                                                                                         |
| 14:12 (R/W)        | P1_ADDR_CNT  | Address Byte Count. The XSPI_PROG_SEQ_CFG0.P1_ADDR_CNT bit field indicates the number of address bytes.                                       | Address Byte Count. The XSPI_PROG_SEQ_CFG0.P1_ADDR_CNT bit field indicates the number of address bytes.                                       |
| 11 (R/W)           | P1_CMD_EDGE  | Command Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for the command phase.                      | Command Phase SDR/DDR Select. The XSPI_PROG_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for the command phase.                      |
| 9:8 (R/W)          | P1_CMD_IOS   | Command Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase.  | Command Phase IO Line Select. The XSPI_PROG_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase.  |
| 9:8 (R/W)          | P1_CMD_IOS   | 0                                                                                                                                             | One data line used (for example, serial)                                                                                                      |
| 9:8 (R/W)          | P1_CMD_IOS   | 1                                                                                                                                             | Two data lines used                                                                                                                           |
| 9:8 (R/W)          | P1_CMD_IOS   | 2                                                                                                                                             | Four data lines used                                                                                                                          |
| 9:8 (R/W)          | P1_CMD_IOS   | 3                                                                                                                                             | Eight data lines used                                                                                                                         |
| 7:0 (R/W)          | P1_CMD_VALUE | Command Mnemonic Value. The XSPI_PROG_SEQ_CFG0.P1_CMD_VALUE bit field indicates the command                                                   | Command Mnemonic Value. The XSPI_PROG_SEQ_CFG0.P1_CMD_VALUE bit field indicates the command                                                   |

## Program Sequence Configuration Register 1

The XSPI\_PROG\_SEQ\_CFG1 register configures the program sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-65: XSPI\_PROG\_SEQ\_CFG1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000063_b235fab26c15b55a1903ee16053bbbf658d8a57c0d1df24d24984470bdb42dfc.png)

Table 21-107: XSPI\_PROG\_SEQ\_CFG1 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                             |
|--------------------|------------------|---------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | P1_CMD_EXT_VALUE | Command Extension Value. The XSPI_PROG_SEQ_CFG1.P1_CMD_EXT_VALUE bit field indicates the com- mand extension value. |
| 0 (R/W)            | P1_CMD_EXT_EN    | Command Extension Enable. The XSPI_PROG_SEQ_CFG1.P1_CMD_EXT_EN bit enables the command ex- tension.                 |

## Program Sequence Configuration Register 2

The XSPI\_PROG\_SEQ\_CFG2 register configures the program sequence for profile 2 in ACMD and direct work modes.

Figure 21-66: XSPI\_PROG\_SEQ\_CFG2 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000064_63838ef81755d4f85b9a6430d6e705a80dbd52c887ae1db73624d18f456738ed.png)

Table 21-108: XSPI\_PROG\_SEQ\_CFG2 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:8 (R/W)         | P2_LATENCY_CNT | Number of Latency Cycles. The XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT bit field indicates the number of latency cycles for profile 2 - HR only. When XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT is cleared (=0), latency cycles are disabled. Configure the XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT bit field to N-1, where Nis the number of latency clock cycles expected by the memory device.                                                                                                                                                                                                  | Number of Latency Cycles. The XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT bit field indicates the number of latency cycles for profile 2 - HR only. When XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT is cleared (=0), latency cycles are disabled. Configure the XSPI_PROG_SEQ_CFG2.P2_LATENCY_CNT bit field to N-1, where Nis the number of latency clock cycles expected by the memory device.                                                                                                                                                                                                  |
| 2 (R/W)            | P2_MSK_CMD_MOD | Profile 2 Command Extension Mask/Mode. The XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD bit indicates the profile 2 command extension variant. This influences bits [44:40] of the CA. When XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is set (=1), CA[44:40] is config- ured to all ones. In direct work mode, when XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of the CA are configured to (sAWADDR[45:41] logically AND'ed with DAC_ADDR_MASk[12:8]). In ACMD work mode, when XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of the CA are cleared (= 0). | Profile 2 Command Extension Mask/Mode. The XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD bit indicates the profile 2 command extension variant. This influences bits [44:40] of the CA. When XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is set (=1), CA[44:40] is config- ured to all ones. In direct work mode, when XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of the CA are configured to (sAWADDR[45:41] logically AND'ed with DAC_ADDR_MASk[12:8]). In ACMD work mode, when XSPI_PROG_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of the CA are cleared (= 0). |
| 1 (R/W)            | P2_BURST_TYP   | Burst Type. The XSPI_PROG_SEQ_CFG2.P2_BURST_TYP bit indicates the burst type; it corresponds to the 45th Command/Address (CA) bit assignment. Although the user can set wrapped burst, the controller itself does not currently support wrapping bursts and therefore must always have this bit set (=1).                                                                                                                                                                                                                                                                     | Burst Type. The XSPI_PROG_SEQ_CFG2.P2_BURST_TYP bit indicates the burst type; it corresponds to the 45th Command/Address (CA) bit assignment. Although the user can set wrapped burst, the controller itself does not currently support wrapping bursts and therefore must always have this bit set (=1).                                                                                                                                                                                                                                                                     |
| 1 (R/W)            | P2_BURST_TYP   | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Wrapped burst                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/W)            | P2_BURST_TYP   | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Linear burst                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 21-108: XSPI\_PROG\_SEQ\_CFG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | P2_TARGET  | Target Space. The XSPI_PROG_SEQ_CFG2.P2_TARGET bit indicates the target space, it corre- sponds to the 46th Command/Address (CA) bit assignment. |
| 0 (R/W)            | P2_TARGET  | 0 Memory space                                                                                                                                   |
| 0 (R/W)            | P2_TARGET  | 1 Register space                                                                                                                                 |

## Read Sequence Configuration Register 0

The XSPI\_READ\_SEQ\_CFG0 register configures the read sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-67: XSPI\_READ\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000065_fbdcc26a67dc918dd6183d9ff9759edae48bcd401460e148c0548ca646b9bfe3.png)

Table 21-109: XSPI\_READ\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                               |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29:24 (R/W)        | P1_DMY_CNT  | Number of Dummy Cycles. The XSPI_READ_SEQ_CFG0.P1_DMY_CNT bit field indicates the number of dummy cycles. If 0 - dummy cycles disabled. This field is used when sending mode- bits is disabled. Otherwise, the read_seq_p1_mb_dummy_cnt must be used. | Number of Dummy Cycles. The XSPI_READ_SEQ_CFG0.P1_DMY_CNT bit field indicates the number of dummy cycles. If 0 - dummy cycles disabled. This field is used when sending mode- bits is disabled. Otherwise, the read_seq_p1_mb_dummy_cnt must be used. |
| 23 (R/W)           | P1_DAT_EDGE | Data Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for data phase.                                                                                                                                        | Data Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for data phase.                                                                                                                                        |
| 21:20 (R/W)        | P1_DAT_IOS  | Data Phase IO Line Select. The XSPI_READ_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase.                                                                                                                | Data Phase IO Line Select. The XSPI_READ_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase.                                                                                                                |
|                    |             | 0                                                                                                                                                                                                                                                     | One data line used (for example, serial)                                                                                                                                                                                                              |
|                    |             | 1                                                                                                                                                                                                                                                     | Two data lines used                                                                                                                                                                                                                                   |
|                    |             | 2                                                                                                                                                                                                                                                     | Four data lines used                                                                                                                                                                                                                                  |
|                    |             | 3                                                                                                                                                                                                                                                     | Eight data lines used                                                                                                                                                                                                                                 |

Table 21-109: XSPI\_READ\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                        | Description/Enumeration                                                                                                                        |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | P1_ADDR_EDGE | Address Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR mode for address phase.                          | Address Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR mode for address phase.                          |
| 17:16 (R/W)        | P1_ADDR_IOS  | Address Phase IO Lines Select. The XSPI_READ_SEQ_CFG0.P1_ADDR_IOS bit field indicates the number of data lines used to send the address phase. | Address Phase IO Lines Select. The XSPI_READ_SEQ_CFG0.P1_ADDR_IOS bit field indicates the number of data lines used to send the address phase. |
| 17:16 (R/W)        | P1_ADDR_IOS  | 0                                                                                                                                              | One data line used (for example, serial)                                                                                                       |
| 17:16 (R/W)        | P1_ADDR_IOS  | 1                                                                                                                                              | Two data lines used                                                                                                                            |
| 17:16 (R/W)        | P1_ADDR_IOS  | 2                                                                                                                                              | Four data lines used                                                                                                                           |
| 14:12 (R/W)        | P1_ADDR_CNT  | Number of Address Bytes. The XSPI_READ_SEQ_CFG0.P1_ADDR_CNT bit field indicates the number of address bytes.                                   | Number of Address Bytes. The XSPI_READ_SEQ_CFG0.P1_ADDR_CNT bit field indicates the number of address bytes.                                   |
| 11 (R/W)           | P1_CMD_EDGE  | Command Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for command phase.                           | Command Phase SDR/DDR Select. The XSPI_READ_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for command phase.                           |
| 9:8 (R/W)          | P1_CMD_IOS   | Command Phase IO Lines Select. The XSPI_READ_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase.  | Command Phase IO Lines Select. The XSPI_READ_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase.  |
| 9:8 (R/W)          | P1_CMD_IOS   | 0                                                                                                                                              | One data line used (for example, serial)                                                                                                       |
| 9:8 (R/W)          | P1_CMD_IOS   | 1                                                                                                                                              | Two data lines used                                                                                                                            |
| 9:8 (R/W)          | P1_CMD_IOS   | 2                                                                                                                                              | Four data lines used                                                                                                                           |
| 9:8 (R/W)          | P1_CMD_IOS   | 3                                                                                                                                              | Eight data lines used                                                                                                                          |
| 7:0 (R/W)          | P1_CMD_VALUE | Command Mnemonic Value. The XSPI_READ_SEQ_CFG0.P1_CMD_VALUE bit field indicates the command                                                    | Command Mnemonic Value. The XSPI_READ_SEQ_CFG0.P1_CMD_VALUE bit field indicates the command                                                    |

## Read Sequence Configuration Register 1

The XSPI\_READ\_SEQ\_CFG1 register configures the read sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-68: XSPI\_READ\_SEQ\_CFG1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000066_da33fefff9357afa3921458e5f5f75aa587aa327818e01d10b0495e60373a286.png)

Table 21-110: XSPI\_READ\_SEQ\_CFG1 Register Fields

| Bit No. (Access)   | Bit Name                  | Description/Enumeration                                                                                                                                                                                                                                                                      |
|--------------------|---------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | P1_MB_EN                  | Mode Bit Enable. When XSPI_READ_SEQ_CFG1.P1_MB_EN is set (=1), the mode bits as defined in the XSPI_XIP_GCTL.XIP_DIS_MB_VAL field, are sent following the address bytes.                                                                                                                     |
| 29:24 (R/W)        | P1_MB_DMY_CNT             | Number of dummy cycles. The XSPI_READ_SEQ_CFG1.P1_MB_DMY_CNT bit field indicates the number of dummy cycles. When XSPI_READ_SEQ_CFG1.P1_MB_DMY_CNT is cleared (=0), dummy cycles are disabled. This field is used when the sending mode bits function is enabled.                            |
| 15:8 (R/W)         | P1_CMD_EXT_VALUE          | Command Extension Value. The XSPI_READ_SEQ_CFG1.P1_CMD_EXT_VALUE bit enables the command extension value.                                                                                                                                                                                    |
| 4 (R/W)            | P1_CACHE_RAN- DOM_READ_EN | Random Cache Read Enable. The XSPI_READ_SEQ_CFG1.P1_CACHE_RANDOM_READ_EN bit changes be- havior of the read sequence to utilize the read page cache random/read page cache random last commands. This field is not used in direct mode. This bit is only applicable to the SPI NAND profile. |

Table 21-110: XSPI\_READ\_SEQ\_CFG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                          |
|--------------------|---------------|--------------------------------------------------------------------------------------------------|
| 0 (R/W)            | P1_CMD_EXT_EN | Enables Command Extension. The XSPI_READ_SEQ_CFG1.P1_CMD_EXT_EN bit enables command exten- sion. |

## Read Sequence Configuration Register 2

The XSPI\_READ\_SEQ\_CFG2 register configures the read sequence for profile 2 in ACMD and direct work modes.

Figure 21-69: XSPI\_READ\_SEQ\_CFG2 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000067_86fbe34bf67137618319326dd6788ac55c9c87be3c0f14b6d44bc8570e1486f8.png)

Table 21-111: XSPI\_READ\_SEQ\_CFG2 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                        |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:8 (R/W)         | P2_LATENCY_CNT | Number of Latency Cycles. The XSPI_READ_SEQ_CFG2.P2_LATENCY_CNT bit field indicates the num- ber of latency cycles. When XSPI_READ_SEQ_CFG2.P2_LATENCY_CNT is cleared (=0), latency clock cycles are disabled. Configure XSPI_READ_SEQ_CFG2.P2_LATENCY_CNT to N-1, where Nis the number of latency clock cycles expected by the memory device. |
| 3 (R/W)            | P2_HF_BOUND_EN | Enable Read Operation Page Boundary. The XSPI_READ_SEQ_CFG2.P2_HF_BOUND_EN bit field is used by the con- troller to calculate read transaction crossing page boundary. It is valid only when profile 2 - HF is selected. 0 Disable                                                                                                             |
| 3 (R/W)            | P2_HF_BOUND_EN | 1 Enable                                                                                                                                                                                                                                                                                                                                       |
| 3 (R/W)            | P2_HF_BOUND_EN |                                                                                                                                                                                                                                                                                                                                                |

Table 21-111: XSPI\_READ\_SEQ\_CFG2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | P2_MSK_CMD_MOD | Profile 2 Command Extension Mask/Mode. The XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD bit field indicates the profile 2 command extension variant.This bit influences bits [44:40] of Command/Address. When XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is set (=1), bits [44:40] of Command/Address is set to 1. In direct work mode, when XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of Command/Address is set to (sARADDR[45:41] logically AND'ed with dac_addr_mask[12:8]). In ACMD work mode, when XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of Command/Address is cleared. | Profile 2 Command Extension Mask/Mode. The XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD bit field indicates the profile 2 command extension variant.This bit influences bits [44:40] of Command/Address. When XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is set (=1), bits [44:40] of Command/Address is set to 1. In direct work mode, when XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of Command/Address is set to (sARADDR[45:41] logically AND'ed with dac_addr_mask[12:8]). In ACMD work mode, when XSPI_READ_SEQ_CFG2.P2_MSK_CMD_MOD is cleared (=0), bits [44:40] of Command/Address is cleared. |
| 1 (R/W)            | P2_BURST_TYP   | Burst Type. The XSPI_READ_SEQ_CFG2.P2_BURST_TYP bit indicates the burst type; it corresponds to 45th Command/Address (CA) bit assignment. Although the user can set wrapped burst, the controller itself does not currently support wrapping bursts; therefore, configure XSPI_READ_SEQ_CFG2.P2_BURST_TYP = 1.                                                                                                                                                                                                                                                                                           | Burst Type. The XSPI_READ_SEQ_CFG2.P2_BURST_TYP bit indicates the burst type; it corresponds to 45th Command/Address (CA) bit assignment. Although the user can set wrapped burst, the controller itself does not currently support wrapping bursts; therefore, configure XSPI_READ_SEQ_CFG2.P2_BURST_TYP = 1.                                                                                                                                                                                                                                                                                           |
|                    |                | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Wrapped burst                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |                | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Linear burst                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 0 (R/W)            | P2_TARGET      | Target Space. The XSPI_READ_SEQ_CFG2.P2_TARGET bit indicates the target space; it corre- sponds to 46th Command/Address (CA) bit assignment.                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Target Space. The XSPI_READ_SEQ_CFG2.P2_TARGET bit indicates the target space; it corre- sponds to 46th Command/Address (CA) bit assignment.                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |                | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Memory space                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |                | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Register space                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

## Revision ID Register

The XSPI\_REVID register contains the release identification number.

Figure 21-70: XSPI\_REVID Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000068_d5ddb3e9a3f715e707071f14e7b8d5626a3dce0b7be16d9b474a762f61ae6745.png)

Table 21-112: XSPI\_REVID Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                     |
|--------------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:16 (R/NW)       | CTL_MAGIC_NUM | Magic Number. The XSPI_REVID.CTL_MAGIC_NUM bit field indicates the controller's magic number. It is a unique number characteristic for the xSPI controller. |
| 15:8 (R/NW)        | CTL_FIX       | Fixed Number. The XSPI_REVID.CTL_FIX bit field indicates the fixed number (minor revision number).                                                          |
| 7:0 (R/NW)         | CTL_REV       | Controller Revision. The XSPI_REVID.CTL_REV bit field indicates the controller revision number.                                                             |

## Reset Sequence Configuration Register 0

The XSPI\_RST\_SEQ\_CFG0 register configures the reset sequence for profile 1 and SPI NAND in ACMD work mode.

Figure 21-71: XSPI\_RST\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000069_de5090f1d97d3bff52e670b671c92bc4a4ffb287711dbfd28b22888ffa5d978a.png)

Table 21-113: XSPI\_RST\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | CMD_EDGE   | Command Phase SDR/DDR Select. The XSPI_RST_SEQ_CFG0.CMD_EDGE bit selects between SDR/DDR mode for both command phases.                                         |
| 25:24 (R/W)        | CMD_IOS    | Command Phase IO Lines Select. The XSPI_RST_SEQ_CFG0.CMD_IOS bit field indicates the number of data lines used to send commands for both command phases.       |
| 25:24 (R/W)        |            | 0 One data line used (for example, serial)                                                                                                                     |
| 25:24 (R/W)        |            | 1 Two data lines used                                                                                                                                          |
| 25:24 (R/W)        |            | 2 Four data lines used                                                                                                                                         |
| 25:24 (R/W)        |            | 3 Eight data lines used                                                                                                                                        |
| 22 (R/W)           | DAT_EN     | Enable Data Phase. The XSPI_RST_SEQ_CFG0.DAT_EN bit enables sending the data phase (confir- mation byte in) following the CMD1 phase.                          |
| 21 (R/W)           | DAT_EDGE   | Data Phase SDR/DDR Select. The XSPI_RST_SEQ_CFG0.DAT_EDGE bit selects between SDR/DDR mode for the data phase (confirmation byte in) following the CMD1 phase. |

Table 21-113: XSPI\_RST\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                           |
|--------------------|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:18 (R/W)        | DAT_IOS               | Data Phase IO Lines Selection. The XSPI_RST_SEQ_CFG0.DAT_IOS bit field indicates the number of lines used to send the data phase (confirmation byte in) following the CMD1 phase. |
| 16 (R/W)           | CMD0_EN               | Enable Command 0 Phase. The XSPI_RST_SEQ_CFG0.CMD0_EN bit enables the CMD0 phase.                                                                                                 |
| 15:8 (R/W) 7:0     | CMD1_VALUE CMD0_VALUE | Command 1 Opcode. The XSPI_RST_SEQ_CFG0.CMD1_VALUE bit field indicates the command mne- monic value for the CMD1 phase. Command 0 Opcode.                                         |

## Reset Sequence Configuration Register 1

The XSPI\_RST\_SEQ\_CFG1 register configures the reset sequence for profile 1 and SPI NAND in ACMD work mode.

Figure 21-72: XSPI\_RST\_SEQ\_CFG1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000070_a02e2e716e7e8731810d73b3dc9f3d69a931aad50bb9360126381e7cfe063ae5.png)

Table 21-114: XSPI\_RST\_SEQ\_CFG1 Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                      |
|--------------------|-------------------|----------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | P1_DAT_VALUE      | Command 1 Confirmation. Value of confirmation byte in (if enabled) the following CMD1 phase. |
| 23:16 (R/W)        | P1_CMD1_EXT_VALUE | Command 1 Opcode. Command extension value of the CMD1 phase (if enabled).                    |
| 15:8 (R/W)         | P1_CMD0_EXT_VALUE | Command 0 Opcode. Command extension value of the CMD0 phase (if enabled).                    |
| 1 (R/W)            | P1_CMD1_EXT_EN    | Enable Command 1. Command extension enable for the CMD1 phase.                               |
| 0 (R/W)            | P1_CMD0_EXT_EN    | Enable Command 0. Command extension enable for the CMD0 phase.                               |

## Host DMA Buffer Address Register 0

The XSPI\_SDMA\_ADDR0 register stores the buffer address in the host memory that is used as a sink/source for the host DMA transfer. The host DMA address is based on the memory pointer field that was programed by the host as part of the CDMA/PIO command.

A single CDMA/PIO command can trigger multiple transfers on the host interface, so the host DMA address value is automatically incremented and updated before each DMA transfer.

Figure 21-73: XSPI\_SDMA\_ADDR0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000071_3cff48e70f8a949b83e1eea21011b02097bbc1b5a794aaad42942c8168510134.png)

Table 21-115: XSPI\_SDMA\_ADDR0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | LO_VALUE   | Low Address Value. The XSPI_SDMA_ADDR0.LO_VALUE bit field indicates the lower part of the SDMA destination/source address. |

## Host DMA Buffer Address Register 1

The XSPI\_SDMA\_ADDR1 register stores the buffer address in the host memory that is used as a sink/source for the host DMA transfer. The host DMA address is based on the memory pointer field that was programed by the host as part of the CDMA/PIO command.

A single CDMA/PIO command can trigger multiple transfers on the host interface, so the host DMA address value is automatically incremented and updated before each DMA transfer.

Figure 21-74: XSPI\_SDMA\_ADDR1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000072_dfaef3875207807860c3086ad50c90ee1a974146a529a4a4da7f1c9dace41c99.png)

Table 21-116: XSPI\_SDMA\_ADDR1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | HI_VALUE   | High Address Value. The XSPI_SDMA_ADDR1.HI_VALUE bit field indicates the higher part of the SDMA destination/source address. |

## Host DMA Block Size Register

The XSPI\_SDMA\_SIZ register contains the transferred data block size for the host DMA module.

Figure 21-75: XSPI\_SDMA\_SIZ Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000073_f33d3c42b5cd0e8563f232952cdc09b47241b8b3f03720c908755581a7bb4987.png)

Table 21-117: XSPI\_SDMA\_SIZ Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | SDMA Block Size. The XSPI_SDMA_SIZ.VALUE bit field indicates the transferred data block size in bytes for the host DMAmodule. Data size is rounded up to the data bus word size. |

## Host DMA Thread Status Register

The XSPI\_SDMA\_TRD\_STAT register contains information for the current host DMA transaction as related to the execution thread.

Figure 21-76: XSPI\_SDMA\_TRD\_STAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000074_55133a7a44887ef286dd4412f9ccc6af98369af2b2fb3bb015db2f948a5e76bb.png)

Table 21-118: XSPI\_SDMA\_TRD\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | DIR        | DMADirection. The XSPI_SDMA_TRD_STAT.DIR bit field indicates the transfer direction related to current host DMAtransfer (0-read; 1-write). |
| 2:0 (R/NW)         | TRD        | Thread ID. The XSPI_SDMA_TRD_STAT.TRD bit field indicates the thread number associated with transferred data block for the host DMAmodule. |

## Sequence Configuration Register 0

The XSPI\_SEQ\_GCTL0 register configures common values for sequences in CDMA, PIO and direct work modes.

Figure 21-77: XSPI\_SEQ\_GCTL0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000075_ffa18a88fd9230eae493c6e9ccb1222c0426a0df73097c191bd9e54c8a582326.png)

Table 21-119: XSPI\_SEQ\_GCTL0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                 | Description/Enumeration                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|
| 24:23 (R/W)        | TYP        | Sequence Type. The XSPI_SEQ_GCTL0.TYP bit field indicates the sequence type (common for all sequences). | Sequence Type. The XSPI_SEQ_GCTL0.TYP bit field indicates the sequence type (common for all sequences). |
|                    |            | 0                                                                                                       | Profile 1                                                                                               |
|                    |            | 1                                                                                                       | Profile 2 - HF (HyperFlash)                                                                             |
|                    |            | 2                                                                                                       | Profile 2 - HR (HyperRAM)                                                                               |
|                    |            | 3                                                                                                       | SPI NAND                                                                                                |

Table 21-119: XSPI\_SEQ\_GCTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21 (R/W)           | DAT_PER_ADDR  | Data Bytes per Address. The XSPI_SEQ_GCTL0.DAT_PER_ADDR bit indicates the data organization of the xSPI memory. Note that the xSPI address/pointer specified during sending CDMA/PIO and direct command is always byte-aligned (for example, when this field is set, the xSPI address must be even). |
| 20 (R/W)           | DAT_SWAP      | Enable Reversed Byte Order. When set (=1), the XSPI_SEQ_GCTL0.DAT_SWAP bit enables reverse byte order. The XSPI_SEQ_GCTL0.DAT_SWAP bit can be set only when data phase reflects octal DDR mode. In other modes the XSPI_SEQ_GCTL0.DAT_SWAP bit must be cleared (=0). 0 Disable                       |
| 18 (R/W)           | TCMS_EN       | 1 Enable Enable TCMS Timing in Profile 1. When set (=1), the XSPI_SEQ_GCTL0.TCMS_EN bit enables tCMS timing in pro- file 1. Refer to the XSPI_MINICTL_DEV_ACTIVE_MAX register at offset 0x1018 for further details.                                                                                  |
| 17                 | UAL_CHUNK_CHK | 0 Disable 1 Enable                                                                                                                                                                                                                                                                                   |
| (R/W)              |               | Enable CRC Checking of Unaligned Chunk. Setting the XSPI_SEQ_GCTL0.UAL_CHUNK_CHK bit enables the checking of a CRC unaligned chunk from the flash device. It can be set (=1) only if crc_ual_chunk_en = 1. Otherwise, the XSPI_SEQ_GCTL0.UAL_CHUNK_CHK bit must be cleared (=0) . 0 Disable 1 Enable |
| 16 (R/W)           | UAL_CHUNK_EN  | Enable CRC Chunk. Setting the XSPI_SEQ_GCTL0.UAL_CHUNK_EN bit enables taking into consider- ation the command address to determine after how many bytes CRC data slice is expected to be returned by the flash device.                                                                               |

Table 21-119: XSPI\_SEQ\_GCTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/W)        | CHUNK_SIZ  | Chunk Size. The XSPI_SEQ_GCTL0.CHUNK_SIZ field indicates the number of bytes after which CRC occurs.                                                                                                                                                                                                                                                          |
| 14:12 (R/W)        | CHUNK_SIZ  | 1 8B                                                                                                                                                                                                                                                                                                                                                          |
| 14:12 (R/W)        | CHUNK_SIZ  | 2 16B                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | CHUNK_SIZ  | 3 32B                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | CHUNK_SIZ  | 4 64B                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | CHUNK_SIZ  | 5 128B                                                                                                                                                                                                                                                                                                                                                        |
| 14:12 (R/W)        | CHUNK_SIZ  | 6 256B                                                                                                                                                                                                                                                                                                                                                        |
| 14:12 (R/W)        | CHUNK_SIZ  | 7 512B                                                                                                                                                                                                                                                                                                                                                        |
| 10 (R/W)           | CRC_OE     | CRC OE. The XSPI_SEQ_GCTL0.CRC_OE field determines whether the controller expects                                                                                                                                                                                                                                                                             |
| 10 (R/W)           | CRC_OE     | 0 Disable                                                                                                                                                                                                                                                                                                                                                     |
| 10 (R/W)           | CRC_OE     | 1 Enable                                                                                                                                                                                                                                                                                                                                                      |
| 9 (R/W)            | CRC_VAR    | Select CRC Variant.                                                                                                                                                                                                                                                                                                                                           |
| 9 (R/W)            | CRC_VAR    | 0 Address Transfer Phase Only. CRC is calculated for all bytes of address transfer phase only and put on the bus after the address transfer phase.                                                                                                                                                                                                            |
| 9 (R/W)            | CRC_VAR    | 1 Sequence. CRC is calculated for all bytes in sequence and put on the bus after all bytes in the sequence.                                                                                                                                                                                                                                                   |
| 8 (R/W)            | CRC_EN     | Enable CRC. When set (=1), the XSPI_SEQ_GCTL0.CRC_EN bit enables dynamic CRC calcula- tion that is based on all previous bytes in the current sequence. Software puts this value on the xSPI flash interface. Not required by Legacy Hyper flash and xSPI Profile 2.0 Devices but can be useful for external flash Monitor to control data integrity. Disable |
| 8 (R/W)            | CRC_EN     | 0                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            | CRC_EN     | 1 Enable                                                                                                                                                                                                                                                                                                                                                      |

Table 21-119: XSPI\_SEQ\_GCTL0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:4 (R/W)          | PSIZ_PGM   | Device Page Program Size. The XSPI_SEQ_GCTL0.PSIZ_PGM bit field indicates the page size of device being used for program operations. Number of bytes in page = 2 page_size . Allowed values are: 4'b0000 - 1B 4'b0001 - 2B ... 4'b1000 - 256B 4'b1001 - 512B 4'b1010 - 1024B 4'b1011 - 2048B 4'b1100 - 4096B 4'b1111 - N/A This field is not used in direct mode for profile 2 - HR or when SPI NAND device is selected. |
| 3:0 (R/W)          | PSIZ_RD    | Device Page Read Size. The XSPI_SEQ_GCTL0.PSIZ_RD bit field indicates the page size of the device used for read operations. Number of bytes in page = 2 page_size . Allowed values are: 4'b0000 - 1B 4'b0001 - 2B ... 4'b1000 - 256B 4'b1001 - 512B 4'b1010 - 1024B 4'b1011 - 2048B 4'b1100 - 4096B 4'b1101 - N/A ... 4'b1111 - unlimited (controller sends all data with single xSPI command)                           |

## Sequence Configuration Register 1

The XSPI\_SEQ\_GCTL1 register configures common values for sequences in CDMA, PIO and direct work modes.

Figure 21-78: XSPI\_SEQ\_GCTL1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000076_ebd07eec4f329c3d4be43731dd4885eaaabfabe6b8c79ca33a2d96c9a42ce4aa.png)

Table 21-120: XSPI\_SEQ\_GCTL1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 29:28 (R/W)        | PLANE_CNT  | Plane Count. The XSPI_SEQ_GCTL1.PLANE_CNT bit field indicates the number of planes in the SPI NAND device (encoded as 2 N ).             |
| 29:28 (R/W)        | PLANE_CNT  | 0 Single plane                                                                                                                           |
| 29:28 (R/W)        | PLANE_CNT  | 1 Two planes                                                                                                                             |
| 29:28 (R/W)        | PLANE_CNT  | 2 Four planes                                                                                                                            |
| 29:28 (R/W)        | PLANE_CNT  | 3 Reserved                                                                                                                               |
| 26:24 (R/W)        | PPER_BLOCK | Pages per Block. The XSPI_SEQ_GCTL1.PPER_BLOCK bit field indicates the number of pages per blocks for SPI NAND device (encoded as 2 N ): |
| 26:24 (R/W)        | PPER_BLOCK | 0 1 page per block                                                                                                                       |
| 26:24 (R/W)        | PPER_BLOCK | 1 2 pages per block                                                                                                                      |
| 26:24 (R/W)        | PPER_BLOCK | 2 4 pages per block                                                                                                                      |
| 26:24 (R/W)        | PPER_BLOCK | 3 8 pages per block                                                                                                                      |
| 26:24 (R/W)        | PPER_BLOCK | 4 16 pages per block                                                                                                                     |
| 26:24 (R/W)        | PPER_BLOCK | 5 32 pages per block                                                                                                                     |
| 26:24 (R/W)        | PPER_BLOCK | 6 64 pages per block                                                                                                                     |
| 26:24 (R/W)        | PPER_BLOCK | 7 128 pages per block                                                                                                                    |

Table 21-120: XSPI\_SEQ\_GCTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/W)           | PCA_SIZ    | Column Address Width. The XSPI_SEQ_GCTL1.PCA_SIZ bit indicates the width of the column address for SPI NAND devices. The value of this field is used to calculate the next page address when the data size specified in the sequence exceeds the current page capacity. 0 12 bit address width |
| 8:0 (R/W)          | PSIZ_EXT   | Extended Page Size. The XSPI_SEQ_GCTL1.PSIZ_EXT bit field indicates the extended page size area (spare area size) for SPI NAND devices. The number of data bytes transmitted to/from each page is extended by a value of this field. This field is not used in direct mode.                    |

## Short Polling Count Register

The XSPI\_SHORTPOL\_GCTL register contains the status monitor cycle count value.

Figure 21-79: XSPI\_SHORTPOL\_GCTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000077_381a130ed4043436ca25b65a1256b4bad56fb6dffc9318432971a5d1ac08f365.png)

Table 21-121: XSPI\_SHORTPOL\_GCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Short Poll Value. The XSPI_SHORTPOL_GCTL.VALUE bit field indicates the minimum number of system clocks after a long polling delay before the controller starts to poll for status if the controller was busy during the first status poll attempt. The long polling value should be significantly larger the short polling value. |

## Status Checking Sequence Configuration Register 0

The XSPI\_STAT\_SEQ\_CFG0 register configures the status checking sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-80: XSPI\_STAT\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000078_20173774376cb8bf7fda9875a9679a1a65e7df419131972185d841377eab092d.png)

Table 21-122: XSPI\_STAT\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                 | Description/Enumeration                                                                                                                 |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | P1_DAT_EDGE  | Data Phase SDR/DDR Select. The XSPI_STAT_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for data phase.                          | Data Phase SDR/DDR Select. The XSPI_STAT_SEQ_CFG0.P1_DAT_EDGE bit selects between SDR/DDR mode for data phase.                          |
| 21:20 (R/W)        | P1_DAT_IOS   | Data Phase IO Lines Select. The XSPI_STAT_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase. | Data Phase IO Lines Select. The XSPI_STAT_SEQ_CFG0.P1_DAT_IOS bit field indicates the number of data lines used to send the data phase. |
|                    |              | 0                                                                                                                                       | One data line used (for example, serial)                                                                                                |
|                    |              | 1                                                                                                                                       | Two data lines used                                                                                                                     |
|                    |              | 2                                                                                                                                       | Four data lines used                                                                                                                    |
|                    |              | 3                                                                                                                                       | Eight data lines used                                                                                                                   |
| 12 (R/W)           | P1_ADDR_EDGE | Address Phase SDR/DDR Select. The XSPI_STAT_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR mode for address phase.                   | Address Phase SDR/DDR Select. The XSPI_STAT_SEQ_CFG0.P1_ADDR_EDGE bit selects between SDR/DDR mode for address phase.                   |

Table 21-122: XSPI\_STAT\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                         |
|--------------------|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:10 (R/W)        | P1_ADDR_IOS   | Address Phase IO Lines Select. The XSPI_STAT_SEQ_CFG0.P1_ADDR_IOS bit field indicates the number of data lines used to send the address phase.  |
| 11:10 (R/W)        | P1_ADDR_IOS   | 0 One data line used (for example, serial)                                                                                                      |
| 11:10 (R/W)        | P1_ADDR_IOS   | 1 Two data lines used                                                                                                                           |
| 11:10 (R/W)        | P1_ADDR_IOS   | 2 Four data lines used                                                                                                                          |
| 11:10 (R/W)        | P1_ADDR_IOS   | 3 Eight data lines used                                                                                                                         |
| 9:8 (R/W)          | P1_ADDR_CNT   | Number of Address Bytes. The XSPI_STAT_SEQ_CFG0.P1_ADDR_CNT bit field indicates the number of address bytes for all status sequences.           |
| 9:8 (R/W)          | P1_ADDR_CNT   | 0 One address byte                                                                                                                              |
| 9:8 (R/W)          | P1_ADDR_CNT   | 1 Two address bytes                                                                                                                             |
| 9:8 (R/W)          | P1_ADDR_CNT   | 2 Three address bytes                                                                                                                           |
| 9:8 (R/W)          | P1_ADDR_CNT   | 3 Four address bytes                                                                                                                            |
| 5 (R/W)            | P1_CMD_EXT_EN | Command Extension Enable. The XSPI_STAT_SEQ_CFG0.P1_CMD_EXT_EN bit enables the command ex- tension.                                             |
| 4 (R/W)            | P1_CMD_EDGE   | Command Phase SDR/DDR Select. The XSPI_STAT_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for command phase.                            |
| 1:0 (R/W)          | P1_CMD_IOS    | Command Phase IO Lines Select. The XSPI_STAT_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command. serial) |
| 1:0 (R/W)          | P1_CMD_IOS    | 0 One data line used (for example,                                                                                                              |
| 1:0 (R/W)          | P1_CMD_IOS    | 1 Two data lines used                                                                                                                           |
| 1:0 (R/W)          | P1_CMD_IOS    | 2 Four data lines used                                                                                                                          |
| 1:0 (R/W)          | P1_CMD_IOS    | 3 Eight data lines used                                                                                                                         |

## Status Checking Sequence Configuration Register 1

The XSPI\_STAT\_SEQ\_CFG1 register configures the status checking sequence for profile 1 and SPI NAND ACMD and direct work modes.

Figure 21-81: XSPI\_STAT\_SEQ\_CFG1 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000079_b97ac00d35685c950851718ffdf46f0559bed83618ae4def710801d26bd9951a.png)

Table 21-123: XSPI\_STAT\_SEQ\_CFG1 Register Fields

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                                                                            |
|--------------------|-----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | P1_ERS_FAIL_ADDR_EN   | Enable Address Phase. The XSPI_STAT_SEQ_CFG1.P1_ERS_FAIL_ADDR_EN bit enables the address phase for checking the fail status after the erase operation. This field is not used in direct work mode.                                 |
| 29:24 (R/W)        | P1_ERS_FAIL_DMY_CN T  | Number of Dummy Clock Cycles. The XSPI_STAT_SEQ_CFG1.P1_ERS_FAIL_DMY_CNT bit field indicates the number of dummy clock cycles used to check the fail status after the erase operation. This field is not used in direct work mode. |
| 22 (R/W)           | P1_PROG_FAIL_ADDR_ EN | Enable Address Phase. The XSPI_STAT_SEQ_CFG1.P1_PROG_FAIL_ADDR_EN bit enables the ad- dress phase for checking the fail status after the program operation.                                                                        |
| 21:16 (R/W)        | P1_PROG_FAIL_DMY_C NT | Number of Dummy Clock Cycles. The XSPI_STAT_SEQ_CFG1.P1_PROG_FAIL_DMY_CNT bit field indicates the number of dummy clock cycles used to check the fail status after the program opera- tion.                                        |
| 6 (R/W)            | P1_DEV_RDY_ADDR_E N   | Enable Address Phase. The XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_ADDR_EN bit enables the address phase for checking the ready/busy status after the program/erase and soft reset/read (only for SPI NAND) operation.                        |

Table 21-123: XSPI\_STAT\_SEQ\_CFG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name            | Description/Enumeration                                                                                                                                                                                                                    |
|--------------------|---------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/W)          | P1_DEV_RDY_DMY_CN T | Number of Dummy Clock Cycles. The XSPI_STAT_SEQ_CFG1.P1_DEV_RDY_DMY_CNT bit field indicates the number of dummy clock cycles needed to check the ready/busy status after pro- gram/erase or soft reset/read (only for SPI NAND) operation. |

## Status Checking Sequence Configuration Register 10

The XSPI\_STAT\_SEQ\_CFG10 register configures the status checking sequence for SPI NAND devices in ACMD and direct work modes.

Figure 21-82: XSPI\_STAT\_SEQ\_CFG10 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000080_db5e264fcccdae5c6e84069f1c7f0fde09fff480f24171b5e83a15cd6b85f760.png)

Table 21-124: XSPI\_STAT\_SEQ\_CFG10 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | ECC_FAIL_EN    | Enable ECC Status Check. The XSPI_STAT_SEQ_CFG10.ECC_FAIL_EN bit enables checking ECC status after a read page operation.                                                                                                                                                                                                                                                                            |
| 27 (R/W)           | CRDY_VALUE     | Value to Compare with Status Word. The XSPI_STAT_SEQ_CFG10.CRDY_VALUE bit indicates the value to be com- pared with the selected status bit in order to detect if the device is in a ready state after the read page cache random operation (CRBSY bit). This field is not used in direct work mode.                                                                                                 |
| 26:24 (R/W)        | CRDY_IDX       | CRBSY Info in Status Word. The XSPI_STAT_SEQ_CFG10.CRDY_IDX bit field indicates which bit of the status word contains the Cache Read Busy (CRBSY) bit information for the SPI NAND read page cache random operation. This field is not used in direct work mode.                                                                                                                                     |
| 23:16 (R/W)        | ECC_CORR_VALUE | Value to Compare with Status Word. The XSPI_STAT_SEQ_CFG10.ECC_CORR_VALUE bit field indicates the value to be compared with the status word masked by the XSPI_STAT_SEQ_CFG10.ECC_FAIL_MSK field in order to detect if the device returned a correctable ECC error during SPI NAND page read operation. This can be used to detect a single range of correctable errors returned by the xSPI device. |

Table 21-124: XSPI\_STAT\_SEQ\_CFG10 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                    |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | ECC_FAIL_VALUE | Value to Compare with Status Word. The XSPI_STAT_SEQ_CFG10.ECC_FAIL_VALUE bit field indicates the value to be compared with the status word masked by the XSPI_STAT_SEQ_CFG10.ECC_FAIL_MSK field in order to detect if the device returned an uncorrectable ECC error during SPI NAND page read operation. |
| 7:0 (R/W)          | ECC_FAIL_MSK   | ECC Mask. The XSPI_STAT_SEQ_CFG10.ECC_FAIL_MSK bit field indicates the mask used to select which bits of status word carries the ECC status.                                                                                                                                                               |

## Status Checking Sequence Configuration Register 2

The XSPI\_STAT\_SEQ\_CFG2 register configures the status checking sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-83: XSPI\_STAT\_SEQ\_CFG2 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000081_643e7219174dc0f832672e3ecf00972aa5fce0b8585d263a627a72ccfa88d3cb.png)

Table 21-125: XSPI\_STAT\_SEQ\_CFG2 Register Fields

| Bit No. (Access)   | Bit Name                | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|-------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | P1_PROG_FAIL_CMD_V ALUE | Program Opcode. The XSPI_STAT_SEQ_CFG2.P1_PROG_FAIL_CMD_VALUE bit field contains the command mnemonic value that is used to check the fail status after program operation.                                                        |
| 15:8 (R/W)         | P1_ERS_FAIL_CMD_VAL UE  | Erase Opcode. The XSPI_STAT_SEQ_CFG2.P1_ERS_FAIL_CMD_VALUE bit field contains the command mnemonic value that is used to check the fail status after the erase operation. This field is not used in direct work mode.             |
| 7:0 (R/W)          | P1_DEV_RDY_CMD_VA LUE   | Ready Busy Opcode. The XSPI_STAT_SEQ_CFG2.P1_DEV_RDY_CMD_VALUE bit field contains the command mnemonic value that is used to check the ready/busy status after pro- gram/erase and soft erase/read (only for SPI NAND) operation. |

## Status Checking Sequence Configuration Register 3

The XSPI\_STAT\_SEQ\_CFG3 register configures the status checking sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-84: XSPI\_STAT\_SEQ\_CFG3 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000082_b17fca69b975aa11af7e8b9ed2606d8cb905a78ac352f15e2e2af435eb946b15.png)

Table 21-126: XSPI\_STAT\_SEQ\_CFG3 Register Fields

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|-----------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | P1_PROG_FAIL_CMD_E XT_VALUE | Program Opcode Extension. The XSPI_STAT_SEQ_CFG3.P1_PROG_FAIL_CMD_EXT_VALUE bit field contains the command mnemonic value that is used to check the fail status after the program operation.                                                        |
| 15:8 (R/W)         | P1_ERS_FAIL_CMD_EXT _VALUE  | Erase Opcode Extension. The XSPI_STAT_SEQ_CFG3.P1_ERS_FAIL_CMD_EXT_VALUE bit field con- tains the command mnemonic value that is used to check the fail status after the erase operation. This field is not used in direct work mode.               |
| 7:0 (R/W)          | P1_DEV_RDY_CMD_EX T_VALUE   | Ready Busy Opcode Extension. The XSPI_STAT_SEQ_CFG3.P1_DEV_RDY_CMD_EXT_VALUE bit field con- tains the command mnemonic value that is used to check the ready/busy status after the program/erase and soft reset/read (only for SPI NAND) operation. |

## Status Checking Sequence Configuration Register 4

The XSPI\_STAT\_SEQ\_CFG4 register configures the status checking sequence for profile 2 - HF in ACMD and direct work modes.

Figure 21-85: XSPI\_STAT\_SEQ\_CFG4 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000083_bf07e2d1380ee7b7ab92122b51ebc44ff32b426b378b8e7375938968757f4726.png)

Table 21-127: XSPI\_STAT\_SEQ\_CFG4 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:8 (R/W)         | P2_LATENCY_CNT | Latency Between CA and Status Reading. The XSPI_STAT_SEQ_CFG4.P2_LATENCY_CNT bit field indicates the number of latency cycles between the CA and status reading. Configure XSPI_STAT_SEQ_CFG4.P2_LATENCY_CNT to N-1, where Nis the number of latency clock cycles expected by the memory device.                                        |
| 2 (R/W)            | P2_MSK_CMD_MOD | Profile 2 Command Extension Mask/Mode. The XSPI_STAT_SEQ_CFG4.P2_MSK_CMD_MOD bit field indicates the profile 2 command extension variant. The value of this field influences bits[44:40] of the Command/Address (CA). If this bit is set (=1), bits [44:40] of the CA are set (=1). Otherwise, bits [44:40] of the CA are cleared (=0). |

## Status Checking Sequence Configuration Register 5

The XSPI\_STAT\_SEQ\_CFG5 register configures the status checking sequence for profile 1, SPI NAND and profile 2 - HF in ACMD and direct work modes.

Figure 21-86: XSPI\_STAT\_SEQ\_CFG5 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000084_15285a2f4d7653f2ae052622052fb15d38640b4f5cf7c624851718f02418af72.png)

Table 21-128: XSPI\_STAT\_SEQ\_CFG5 Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                |
|--------------------|-----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | PROG_FAIL_EN    | Enable Failure Check after Program. The XSPI_STAT_SEQ_CFG5.PROG_FAIL_EN bit enables checking the fail sta- tus after program operation.                                                                                |
| 29 (R/W)           | PROG_FAIL_SIZ   | Size of Status Word. The XSPI_STAT_SEQ_CFG5.PROG_FAIL_SIZ bit indicates the size of the sta- tus word (0-1B, 1-2B).                                                                                                    |
| 28 (R/W)           | PROG_FAIL_VALUE | Value to Compare with Status Word. The XSPI_STAT_SEQ_CFG5.PROG_FAIL_VALUE bit indicates the value to be compared with selected status bit in order to detect that the device is in fail state after program operation. |
| 27:24 (R/W)        | PROG_FAIL_IDX   | Failure Info in Status Word. The XSPI_STAT_SEQ_CFG5.PROG_FAIL_IDX bit field indicates which bit of the status word contains the failure information after the program command.                                         |

Table 21-128: XSPI\_STAT\_SEQ\_CFG5 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                  |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | ERS_FAIL_EN    | Enable Failure Check after Erase. The XSPI_STAT_SEQ_CFG5.ERS_FAIL_EN bit enables checking the fail status after an erase operation. This field is not used in direct work mode.                                                                                          |
| 13 (R/W)           | ERS_FAIL_SIZ   | Size of Status Word. The XSPI_STAT_SEQ_CFG5.ERS_FAIL_SIZ bit indicates the size of status word (0-1B, 1-2B). This field is not used in direct work mode.                                                                                                                 |
| 12 (R/W)           | ERS_FAIL_VALUE | Value to Compare with Status Word. The XSPI_STAT_SEQ_CFG5.ERS_FAIL_VALUE bit indicates the value to be compared with the selected status bit in order to detect that the device is in the ready state after erase operation. This field is not used in direct work mode. |
| 11:8 (R/W)         | ERS_FAIL_IDX   | Failure Info in Status Word. The XSPI_STAT_SEQ_CFG5.ERS_FAIL_IDX bit field indicates which bit of the status word contains the failure information after the erase command. This field is not used in direct work mode.                                                  |
| 6 (R/W)            | DEV_RDY_EN     | Enable RDY/BUSY Status. The XSPI_STAT_SEQ_CFG5.DEV_RDY_EN bit enables checking the ready/busy status after program/erase operations.                                                                                                                                     |
| 5 (R/W)            | DEV_RDY_SIZ    | Size of Status Word. The XSPI_STAT_SEQ_CFG5.DEV_RDY_SIZ bit indicates the size of status word (0-1B, 1-2B).                                                                                                                                                              |
| 4 (R/W)            | DEV_RDY_VALUE  | Value to Compare Status Word. The XSPI_STAT_SEQ_CFG5.DEV_RDY_VALUE bit indicates the value to be compared with selected status bit in order to detect that the device is ready after a program/erase or soft reset/read (only for SPI NAND) operation.                   |
| 3:0 (R/W)          | DEV_RDY_IDX    | Ready Busy in Status Word. The XSPI_STAT_SEQ_CFG5.DEV_RDY_IDX bit field indicates which bit of the status word contains the ready/busy information to be polled after a program/erase or soft erase/read (only for SPI NAND) operation.                                  |

## Status Checking Sequence Configuration Register 7

The XSPI\_STAT\_SEQ\_CFG7 register configures the status checking sequence for profile 1, SPI NAND and profile 2 - HF in ACMD and direct work modes.

Figure 21-87: XSPI\_STAT\_SEQ\_CFG7 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000085_af8809915e8231a9df4b54df307968f74f242b5c5b2c0745018de1187b20e4b1.png)

Table 21-129: XSPI\_STAT\_SEQ\_CFG7 Register Fields

| Bit No. (Access)   | Bit Name             | Description/Enumeration                                                                                                                                                                                                                   |
|--------------------|----------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DEV_RDY_ADDR_VAL- UE | Address to Check READY/BUSY Status. The XSPI_STAT_SEQ_CFG7.DEV_RDY_ADDR_VALUE bit field indicates the value of the address used to check the ready/busy status after the program/erase and soft reset/read (only for SPI NAND) operation. |

## Status Checking Sequence Configuration Register 8

The XSPI\_STAT\_SEQ\_CFG8 register configures the status checking sequence for profile 1, SPI NAND and profile 2 - HF in ACMD and direct work modes.

Figure 21-88: XSPI\_STAT\_SEQ\_CFG8 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000086_73fb0df4ee01803ee3e74fd04f26e3b3b9f7acbb6a9c7c43c1cb0cefbb5534e1.png)

Table 21-130: XSPI\_STAT\_SEQ\_CFG8 Register Fields

| Bit No. (Access)   | Bit Name               | Description/Enumeration                                                                                                                                                                            |
|--------------------|------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PROG_FAIL_ADDR_VAL- UE | Address to Check Failure Status after Program. The XSPI_STAT_SEQ_CFG8.PROG_FAIL_ADDR_VALUE bit field indicates the value of the address used to check the fail status after the program operation. |

## Status Checking Sequence Configuration Register 9

The XSPI\_STAT\_SEQ\_CFG9 register configures the status checking sequence for profile 1, SPI NAND and profile 2 - HF in ACMD work mode.

Figure 21-89: XSPI\_STAT\_SEQ\_CFG9 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000087_bd16909e74da0d48353677eecf797de5fd546b180c832b91b1f2cd89527f2ee1.png)

Table 21-131: XSPI\_STAT\_SEQ\_CFG9 Register Fields

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                               |
|--------------------|-----------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ERS_FAIL_ADDR_VAL- UE | Address to Check Failure Status after Erase. The XSPI_STAT_SEQ_CFG9.ERS_FAIL_ADDR_VALUE bit field indicates the value of the address used to check fail status after erase operation. |

## Auto Command Engine Interrupt Status Thread Register

Each bit of the field corresponds to an auto command engine thread and holds the descriptor status for that selected thread. It is set only when INT bit of descriptor is set.

Figure 21-90: XSPI\_TRD\_COMP\_ISTAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000088_daab4821931d2e1dc9edd2d4d6b03ed601b4f76d0ebe552c9b4936df303cc631.png)

Table 21-132: XSPI\_TRD\_COMP\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                        |
|--------------------|------------|----------------------------------------------------------------|
| 7 (R/W1C)          | TRD7       | Thread 7 Operation Complete. Thread 7 operation complete flag. |
| 6 (R/W1C)          | TRD6       | Thread 6 Operation Complete. Thread 6 operation complete flag. |
| 5 (R/W1C)          | TRD5       | Thread 5 Operation Complete. Thread 5 operation complete flag. |
| 4 (R/W1C)          | TRD4       | Thread 4 Operation Complete. Thread 4 operation complete flag. |
| 3 (R/W1C)          | TRD3       | Thread 3 Operation Complete. Thread 3 operation complete flag. |
| 2 (R/W1C)          | TRD2       | Thread 2 Operation Complete. Thread 2 operation complete flag. |
| 1 (R/W1C)          | TRD1       | Thread 1 Operation Complete. Thread 1 operation complete flag. |
| 0 (R/W1C)          | TRD0       | Thread 0 Operation Complete. Thread 0 operation complete flag. |

## Thread Error Interrupt Enable Register

If the selected bit of the XSPI\_TRD\_ERR\_INT\_EN register is set, the rising edge of the corresponding bit in the XSPI\_TRD\_COMP\_ISTAT register causes the setting of the external interrupt line.

Figure 21-91: XSPI\_TRD\_ERR\_INT\_EN Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000089_7f088a64efd4e659e119fd1ce25921055efdc6ef47d491f22cd6cf65a8a62605.png)

Table 21-133: XSPI\_TRD\_ERR\_INT\_EN Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                        |
|--------------------|----------------|------------------------------------------------|
| 7:0                | TRD_ERR_INT_EN | Interrupt Enable for Thread Error.             |
| (R/W)              |                | Interrupt enable for detecting a thread error. |

## Thread Error Interrupt Status Register

The XSPI\_TRD\_ERR\_ISTAT register can be read to determine whether a particular thread (in the auto command engine) has encountered an error condition.

To get more information on the error, software must read the status field of the descriptor (CDMA mode) or the appropriate status register (PIO mode).

Figure 21-92: XSPI\_TRD\_ERR\_ISTAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000090_8abbb9d0c59027b9cbc11dec5b1bdec13f9800a0fa189b4aef9018c352e0334e.png)

Table 21-134: XSPI\_TRD\_ERR\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration         |
|--------------------|------------|---------------------------------|
| 7 (R/W1C)          | TRD7       | Thread 7 Error. Thread 7 error. |
| 6 (R/W1C)          | TRD6       | Thread 6 Error. Thread 6 error. |
| 5 (R/W1C)          | TRD5       | Thread 5 Error. Thread 5 error. |
| 4 (R/W1C)          | TRD4       | Thread 4 Error. Thread 4 error. |
| 3 (R/W1C)          | TRD3       | Thread 3 Error. Thread 3 error. |
| 2 (R/W1C)          | TRD2       | Thread 2 Error. Thread 2 error. |
| 1 (R/W1C)          | TRD1       | Thread 1 Error. Thread 1 error. |

Table 21-134: XSPI\_TRD\_ERR\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 0                  | TRD0       | Thread 0 Error.           |
| (R/W1C)            |            | Thread 0 error.           |

## Auto Command Engine Thread Status Register

The XSPI\_TRD\_STAT register indicates the auto command engine thread status (ACMD mode only).

Figure 21-93: XSPI\_TRD\_STAT Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000091_2181c34de63f482f3039e7778f4d12be6ac21295ea0dc340e169e11b6c570843.png)

Table 21-135: XSPI\_TRD\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | TRD_BUSY   | Thread ID Busy. The XSPI_TRD_STAT.TRD_BUSY bit indicates the auto command engine thread busy status. When set, the corresponding thread is currently busy. |

## WEL Sequence Configuration Register

The XSPI\_WE\_SEQ\_CFG0 register configures the Write Enable Latch (WEL) sequence for profile 1 and SPI NAND in ACMD and direct work modes.

Figure 21-94: XSPI\_WE\_SEQ\_CFG0 Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000092_9aef1d13b9d560df39fbdc61c85f38ea1cc0d1aef0fa4e8667af4baeaf3af0ea.png)

Table 21-136: XSPI\_WE\_SEQ\_CFG0 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                     | Description/Enumeration                                                                                                                     |
|--------------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | P1_EN            | Enable WEL Command. The XSPI_WE_SEQ_CFG0.P1_EN bit enables the WEL command.                                                                 | Enable WEL Command. The XSPI_WE_SEQ_CFG0.P1_EN bit enables the WEL command.                                                                 |
| 23:16 (R/W)        | P1_CMD_EXT_VALUE | Command Extension Value. The XSPI_WE_SEQ_CFG0.P1_CMD_EXT_VALUE bit field indicates the com- mand extension value.                           | Command Extension Value. The XSPI_WE_SEQ_CFG0.P1_CMD_EXT_VALUE bit field indicates the com- mand extension value.                           |
| 15 (R/W)           | P1_CMD_EXT_EN    | Command Extension Enable. The XSPI_WE_SEQ_CFG0.P1_CMD_EXT_EN bit enables the command exten- sion.                                           | Command Extension Enable. The XSPI_WE_SEQ_CFG0.P1_CMD_EXT_EN bit enables the command exten- sion.                                           |
| 11 (R/W)           | P1_CMD_EDGE      | Command Phase SDR/DDR Select. The XSPI_WE_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for command phase.                          | Command Phase SDR/DDR Select. The XSPI_WE_SEQ_CFG0.P1_CMD_EDGE bit selects between SDR/DDR mode for command phase.                          |
| 9:8 (R/W)          | P1_CMD_IOS       | Command Phase IO Lines Select. The XSPI_WE_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase. | Command Phase IO Lines Select. The XSPI_WE_SEQ_CFG0.P1_CMD_IOS bit field indicates the number of data lines used to send the command phase. |
|                    |                  | 0                                                                                                                                           | One data line used (for example, serial)                                                                                                    |
|                    |                  | 1                                                                                                                                           | Two data lines used                                                                                                                         |
|                    |                  | 2                                                                                                                                           | Four data lines used                                                                                                                        |
|                    |                  | 3                                                                                                                                           | Eight data lines used                                                                                                                       |

Table 21-136: XSPI\_WE\_SEQ\_CFG0 Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                   |
|--------------------|--------------|-----------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | P1_CMD_VALUE | Command Mnemonic Value. The XSPI_WE_SEQ_CFG0.P1_CMD_VALUE bit field indicates the command mnemonic value. |

## Device Control Register

The XSPI\_WORKMODE\_CTL register controls the device working modes.

Figure 21-95: XSPI\_WORKMODE\_CTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000093_e98ea3d09365d844f09cf160d471c0cf42d9e70e6b04de773d48460e38e46173.png)

Table 21-137: XSPI\_WORKMODE\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:5 (R/W)          | WORKMODE   | Controller Work Mode. The XSPI_WORKMODE_CTL.WORKMODE bit field indicates the work mode of the controller. Allowed values are 00 - Direct mode 01 - STIG mode 11 - ACMD mode                                                                                                                                                                                                                                                                                                                |
| 3 (R/W)            | ON_ERR     | Controller On Error. This control bit is relevant only for ACMD work mode. If the XSPI_WORKMODE_CTL.ON_ERR bit is cleared (=0) and an error occurs, the con- troller drops execution of all further operations programmed in a single thread at the page (for read or program commands) or sector (for the sector erase command) boundary. The XSPI_WORKMODE_CTL.ON_ERR bit applies to both command DMAand PIO work modes. When XSPI_WORKMODE_CTL.ON_ERR is set (=1), execution continues. |

## XIP Configuration Register

The XSPI\_XIP\_GCTL register configures the controller in XIP work mode in CDMA, PIO and direct work modes.

Figure 21-96: XSPI\_XIP\_GCTL Register Diagram

![Image](24_Extended_Serial_Peripheral_Interface_(xSPI)_artifacts/image_000094_7abfe238f364dca6fb0aece16ac131797d570cae1ee4603bd7b2ca641f136fdc.png)

Table 21-138: XSPI\_XIP\_GCTL Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                     |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:16 (R/W)        | XIP_DIS_MB_VALUE | XIP Mode Bit Disable Value. The XSPI_XIP_GCTL.XIP_DIS_MB_VALUE bit field indicates the value of the mode bits required to disable XIP mode.                                                                                                                                 |
| 15:8 (R/W)         | XIP_EN_MB_VALUE  | XIP Mode Bit Enable Value. The XSPI_XIP_GCTL.XIP_EN_MB_VALUE bit field indicates the value of the mode bits required to enable XIP mode.                                                                                                                                    |
| 7:0 (R/W)          | XIP_EN           | XIP Mode Enable. The XSPI_XIP_GCTL.XIP_EN bit field indicates the XIP mode enabled for a selected memory bank. When XIP mode is enabled, only read sequences are valid. Invoking any other command sequence is ignored and CMD_ERROR/DSC_ER- ROR/XSPI_ISTAT.DIR_CMD_ERR =1. |