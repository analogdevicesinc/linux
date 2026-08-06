# Octal Serial Peripheral Interface (OSPI)

<!-- source: 025_Octal_Serial_Peripheral_Interface_OSPI.pdf | original pages 1465–1531 -->

## 23   Octal Serial Peripheral Interface (OSPI)

The Octal Serial Peripheral Interface (OSPI) controller provides access to serial flash devices. It is designed to access flash devices with its modes enabling efficient communication, with minimum overhead on software. Standard SPI is supported along with high performance dual, quad, and octal SPI modes where data can be transferred on up to 8 data pins. Along with normal mode of operation in single transfer (STR) mode, the OSPI controller supports dual transfer rate (DTR) mode, where data, address, and commands are transferred on both edges of the serial clock. With direct access mode support, any read/write to the OSPI memory space triggers a read/write to flash memory.

Direct read accesses using core and DMA are possible, with the latter minimizing software overhead. The OSPI controller also provides Software T riggered Instruction Generation (STIG) mode, which can be used to erase the data on flash devices and access volatile and nonvolatile configuration registers, legacy flash status register, and other status/protection registers.

The OSPI provides a glueless hardware interface with SPI flash devices. The OSPI peripheral includes programmable baud rates, clock phase, clock polarity, separate dummy cycles for read and write accesses, data sampling control, programmable page size/block size, and so on. With this independent control of how many lines to use for command, address, and data allows to virtually interface any flash device with the processor through OSPI controller.

The OSPI also provides support for DQS when sampling data to improve read data capture and PHY mode support to enable high-speed data transfers.

## OSPI Features

The OSPI controller supports the following features:

- Full-duplex, synchronous, serial interface
- Memory mapped direct mode operation for performing flash data transfers
- STIG mode operation to issue flash commands
- Automatic flash status polling support for flash program through direct access mode
- XIP (Execute in Place) support
- Support for single, dual, quad, or octal modes of operation
- Support for DDR mode and DTR protocol (including Octal DDR protocol)

- Up to 16-bit data transfer in each SPI clock cycle
- Programmable baud rate, clock phase, and polarity
- Programmable interrupt generation
- Programmable chip select control timing
- Programmable dummy cycles for read and write operations
- Tune data capture mechanism to improve high speed operation
- Programmable write protected regions to block system writes
- Support for DQS to sample the data to improve read data capturing
- PHY mode support to allow the high speed data transfers

## OSPI Functional Description

## ADSP-SC59x OSPI Register List

The OSPI is an Octal SPI controller. It contains the following registers.

Table 23-1: ADSP-SC59x OSPI Register List

| Name           | Description                               |
|----------------|-------------------------------------------|
| OSPI_CTL       | Octal SPI Control Register                |
| OSPI_DLY       | Device Delay Register                     |
| OSPI_DRICTL    | Device Read Instruction Control Register  |
| OSPI_DWICTL    | Device Write Instruction Control Register |
| OSPI_DSCTL     | Device Size Control Register              |
| OSPI_DLLOB_LWR | DLL Observable Register (Lower)           |
| OSPI_DLLOB_UP  | DLL Observable Register (Upper)           |
| OSPI_FCA       | Flash Command Address Register            |
| OSPI_FCCTL     | Flash Command Control Register            |
| OSPI_FCMCTL    | Flash Command Control Memory Register     |
| OSPI_FCRD_LWR  | Flash Command Read Data Register (Lower)  |
| OSPI_FCRD_UP   | Flash Command Read Data Register (Upper)  |
| OSPI_FCWD_LWR  | Flash Command Write Data Register (Lower) |
| OSPI_FCWD_UP   | Flash Command Write Data Register (Upper) |
| OSPI_IMSK      | Interrupt Mask Register                   |
| OSPI_ISTAT     | Interrupt Status Register                 |

Table 23-1: ADSP-SC59x OSPI Register List (Continued)

| Name            | Description                       |
|-----------------|-----------------------------------|
| OSPI_WRPROT_LWR | Lower Write Protection Register   |
| OSPI_MBCTL      | Mode Bit Control Register         |
| OSPI_MODID      | Module ID Register                |
| OSPI_POLLEXP    | Polling Expiration Register       |
| OSPI_OE_LWR     | Opcode Extension Register (Lower) |
| OSPI_OE_UP      | Opcode Extension Register (Upper) |
| OSPI_PHYCTL     | PHY Control Register              |
| OSPI_PHYMCTL    | PHY DLL Master Control Register   |
| OSPI_POLSTAT    | Polling Flash Status Register     |
| OSPI_RDC        | Read Data Capture Register        |
| OSPI_REMAPADDR  | Remap Address Register            |
| OSPI_WRPROT_UP  | Upper Write Protection Register   |
| OSPI_WRPROT_CTL | Write Protection Control Register |
| OSPI_WCCTL      | Write Completion Control Register |

## ADSP-SC59x OSPI Interrupt List

Table 23-2: ADSP-SC59x OSPI Interrupt List

|   Interrupt ID | Name      | Description   | Sensitivity   | DMA Channel   |
|----------------|-----------|---------------|---------------|---------------|
|            137 | OSPI0_INT | OSPI0 Error   | Level         |               |

## OSPI Block Diagram

The OSPI module is comprised of:

- Low level SPI protocol controller
- Internal transmit/receive FIFOs
- DAC
- STIG controller
- Register interface

The OSPI Controller Block Diagram shows the OSPI controller functional blocks.

Figure 23-1: OSPI Controller Block Diagram

<!-- image -->

## Architectural Concepts

There are two clock sources for the OSPI controller and one generated clock for clocking external flash device.

The SYSCLK is the main system clock used to transfer data over the AHB bus between the controller (core/DMA) and the OSPI controller. The SCLK0\_0 clock is used to access the OSPI controller registers to perform basic configuration of the controller and interrupt handling. The OSPI\_REFCLK, is used to serialize the data and drive the OSPI interface. The external clock driven on the OSPI\_CLK pin, which is synchronous to the reference clock, is derived from the OSPI\_REFCLK and divided using internal baud rate dividers. Using the OSPI\_REFCLK clock, allows the core to decouple the frequency of the SPI flash device from system clocks, thereby providing a flexible clocking solution.

The OSPI controller supports two independent controllers (to support different types of flash accesses):

- Direct Access Controller (DAC)
- Software Triggered Instruction Generator (STIG)

The high-level architecture of the OSPI controller is divided into two blocks. The first is the high-level OSPI controller that interfaces with the rest of the SoC. It also queues flash transactions based on data from AHB and APB bus interfaces. The second module is the low-level SPI Protocol Controller. It is used for serializing data from highlevel OSPI controller and translating them into the SPI transfer protocol where there are two data paths. The first path preserves backward compatibility with the previous SoCs. This path bypasses the PHY module. Use this path when there is no strict requirement for high performance (clock divider is greater or equal 4).

The second data path is through the PHY Module and is for dividers lower than 4. The configuration is set by software. The controller waits for a valid direct access from the AHB bus (which runs off SYSCLK) or for a software triggered read from the APB bus. When such an event occurs, the corresponding internal controller is selected

(DAC or STIG). The flash command generator block arbitrates between accesses and forwards control into the lowlevel SPI module. This block works on the reference clock. T ransmit data is synchronized internally to the TX FIFO and then serialized to the SPI interface. When the direction of transfer changes and the device returns data to the controller, data is read into the RX FIFO and then synchronized to the internal clocks. The SPI control logic selects the source of the data path (with or without PHY) based on the register configuration. The reference clock and the associated delayed variants are relevant for this transfer type when the PHY mode is enabled.

## Direct Access Controller (DAC)

Direct access refers to the operation where AHB bus accesses targeted to the OSPI memory mapped space of processor directly triggers a read/write on flash memory. It is memory mapped and used to access and directly execute the code from external flash memory. When DAC is enabled through the OSPI\_CTL.DACEN field, any incoming AHB access is serviced by the DAC. Direct accesses can be launched by accessing the OSPI memory address space using core or via one of the MDMA channels. In memory mapped mode, communication to a flash device is automated. The flash memory is accessible directly through reads/write of the processor address space. This access allows code to execute directly from flash devices (true XIP operations). The content can be cached for good performance.

When servicing AHB reads, the DAC sends an additional access downstream apart from the one issued by the AHB in single AHB burst. This is invisible from the system interface. It is a predicted read and ensures that the SPI core operates with maximum bandwidth. The AHB burst not defined by the AMBA specification. It is defined as follows:

- The first access of the AHB burst is defined by:

An AHB access that is non-sequential (based on address comparison).

An AHB access that is non-sequential or sequential when the downstream modules are idle.

- The last access of the AHB burst is defined by an AHB access that is sequential and precedes a new burst.
- Size of the AHB burst is the number of AHB accesses (first access to last access).
- The number of DAC requests per AHB burst is equal to the size of the AHB burst + 1.

When AHB clock is slower than the SPI data rate, the sequential read transfer may be interrupted on the SPI side and continues after the AHB clock accepts the last data. The limiting value gets narrower as the SPI clock divider and AHB access size decrease and the number of data IOs (single, dual, quad, or octal) increase. If the system cannot meet the SPI data rate, use higher SPI clock divider. This decreases the SPI data rate but prevents OSPI accesses to external flash to break in between and pass the Opcode-Address-Dummy sequence each time. This increases the overall performance by keeping the data transfer continuous.

The controller is designed to work at high data rates. This predicted read results in redundant data during mixed AHB accesses on slow system data rate. Mixed accesses implies that direct AHB sequence of single non-sequential access following few sequential accesses. The predicted read cannot be dropped because of SPI transfer interruption. It is sent as a separate SPI transaction which creates redundant data on the interface for one access. To avoid this issue, drop this data by system or introduce wait states on AHB before issuing a non-sequential direct AHB access.

For bus writes, the DAC triggers a sequence of write commands that mimic the way reads are processed, though the number of DAC write requests are equal to the number of bus write requests received. The bus controller ensures

that flash bursts do not break the flash page boundary. When a page boundary is detected, only the number of byte accesses up to that boundary are forwarded. A sequential direct write request that crosses a page boundary must be detected as non-sequential, causing the controller to force the flash device to enter a self-timed page program cycle. The controller supports splitting the writes crossing the page boundaries only for word-aligned addresses.

The external bus controller supplying data for writes must guarantee write data for a particular page that can be provided to the controller in a timely fashion to avoid downstream starvation. If there is a large delay in the system issuing a sequential write, flash write cycle may be prematurely initiated, reducing the device lifetime.

Flash erase operations, which may be required before a page write, are triggered by software using the STIG. Once a page program cycle has been started, the OSPI controller automatically polls for write cycle to complete before allowing any bus accesses to complete. This is achieved by holding the subsequent AHB direct accesses in wait state.

For memory mapped accesses using the DAC, the flash memory is accessible using the address space 0x60000000 0x7FFFFFFF, as indicated in the processor datasheet. Since both SPI2 and OSPI controllers can accesses this space in memory mapped mode, there are control bits in the SCB5\_SPI2\_OSPI\_REMAP register to control which controller is able to access this flash memory mapped space. Either OSPI or SPI2 can exclusively access whole of the flash memory space or there is an option to divide this space between OSPI and SPI2.

When the SCB5\_SPI2\_OSPI\_REMAP register = 0x0, the entire flash memory mapped space is only accessible by the SPI2 controller. Do not try to access this space with the OSPI controller.

When the SCB5\_SPI2\_OSPI\_REMAP register = 0x1, the entire flash memory mapped space is only accessible by the OSPI controller. Do not try to access this space with the SPI2 controller.

When the SCB5\_SPI2\_OSPI\_REMAP register = 0x2, the flash memory mapped space is divided between the OSPI and SPI2 controllers. The flash memory mapped space (0x60000000 - 0x602FFFFF) is only accessible by SPI2 and the flash memory mapped space (0x60300000 - 0x7FFFFFFF) is only accessible by OSPI. This option is only valid for the ADSP-SC59x/2159x HPC package.

## Software Triggered Instruction Generator (STIG)

DAC is primarily used to transfer data. To access the volatile and nonvolatile configuration registers, the legacy Flash SPI Status register, other status/protection registers and perform erase operations, a separate software controller is required.

The STIG is controlled using the OSPI\_FCCTL register by setting up the command to issue to the flash device. This is a generic controller used to perform instructions that the flash device supports from the extended SPI protocol. The STIG controller command is sent to the flash device and written to the software in the OSPI\_FCCTL.OPCODE field. This is different from the OSPI\_DRICTL.OPCODERD and OSPI\_DWICTL.OPCODEWR fields, as these fields do not impact the STIG operation.

Configuring instructions that are not compliant with the flash specifications can cause unpredicted behavior of the controller. Bit 0 is used to trigger the command, bit 1 used by software to poll the status of the command execution. For reads, when the command has been serviced (bit 1 toggles from 1 to 0), up to 8 bytes of read data is placed in the OSPI\_FCRD\_LWR and OSPI\_FCRD\_UP registers. For writes, data must be placed in the OSPI\_FCWD\_LWR and OSPI\_FCWD\_UP registers. The STIG completion request can be also checked by the corresponding interrupt.

The interrupt indicates that the controller is ready to accept a new STIG request. The STIG completion request is not equivalent to completion on the SPI side. For example, if STIG is configured to the data command only on transmit, data is taken from the corresponding STIG register fields and put into TX FIFO. As all write bytes are known, another STIG can be queued before completing serialization of the current one.

There are few commands that require more data to read than 8 bytes (for example, READ ID command). Additional STIG memory bank is implemented to accommodate this data, if required. The STIG memory bank is controlled by the OSPI\_FCMCTL register. If enabled, the number of bytes to read in the STIG is extended to 16 bytes, as defined in the OSPI\_FCMCTL.RDSZ field. There are few commands (excluding read arrays that are not intended to be handled in STIG mode, but in Direct Access Mode) that return more than 8 bytes to the controller.

If the number of read bytes in the STIG, as defined in the OSPI\_FCMCTL.RDSZ field, exceeds the memory bank depth, the remaining data overwrites the STIG memory bank locations starting from its first address. When memory bank is enabled, the OSPI\_FCRD\_LWR and OSPI\_FCRD\_UP registers keep the last 8 bytes read from the flash device by the STIG. For example, to get a single byte from the last eight bytes of a long continuous read SPI data chain, there is no need to access the STIG memory bank as data can be obtained from the OSPI\_FCRD\_LWR and OSPI\_FCRD\_UP registers. T o access more data, STIG memory bank data request must be triggered. It is controlled by the OSPI\_FCMCTL register and works analogously to trigger the STIG. Bit 0 ( OSPI\_FCMCTL.TRIGREQ ) is used to trigger the command. Bit 1 ( OSPI\_FCMCTL.BNKREQ ) used by software to poll the status of the command execution. When bit 1 toggles from 1 to 0, the data byte ( OSPI\_FCMCTL.BNKDATA ) from the corresponding address ( OSPI\_FCMCTL.BNKADDR ) is valid. The address must be set before triggering the STIG memory bank access. Each consecutive STIG access overwrites the previous one so that the data in the bank fits into byte index fetched by the last STIG access configured to use the memory bank (first incoming byte equals first address of the memory bank, second one equals the second address and so on).

## Servicing STIG Request

The OSPI controller determines the number of bytes in the OSPI\_FCCTL register to be sent to the flash device, when there is a STIG request. The OSPI\_FCCTL.OPCODE field indicates the instruction to be sent and is always pushed first. If there is an address to send, the address (size is also programmed in the same register) is sent next. The address is stored in the OSPI\_FCA register. If there are dummy cycles (size of which is also programmed in the OSPI\_FCCTL register), they are sent next.

For write data (size of which is also programmed in the OSPI\_FCCTL register), up to 8 bytes can be sent (as stored in the OSPI\_FCWD\_LWR and OSPI\_FCWD\_UP registers). When read data is collected from the flash device, the OSPI controller stores it in the OSPI\_FCRD\_LWR and OSPI\_FCRD\_UP registers.

## Arbitration Between DAC and STIG Access

When multiple controllers are simultaneously active, a simple, fixed priority arbitration scheme is used to arbitrate between each interface and to access the external flash.

The fixed priority is defined as follows, highest priority first:

1. Direct Access Write
2. STIG Access

## 3. Direct Access Read

When one controller is servicing the request, the other controller is back pressured while waiting to be serviced.

## Auto Polling for DAC Write Access

For DAC write accesses, which initiate flash program operation, the OSPI controller provides an option to automatically poll the flash status for program completion. When a flash write operation is triggered through DAC, the OSPI controller sends the write data up to page size defined in the OSPI\_DSCTL.PGSZ field. Once a page boundary is reached, the DAC stops sending data to the flash device as the flash device enters the program mode. Once the device is in program mode, the OSPI controller can automatically send the flash status read commands to check the state of the flash.

The OSPI\_WCCTL register defines the setting to poll the flash device correctly. By default, it is configured to poll bit 0 of the Device STATUS register (using opcode 0x05), which is common across all devices to indicate Write in Progress (WIP). However, in some devices (such as Micron devices, where size is &gt; 512 MB), it is required that the controller polls a different bit of a different device register. This means the controller must issue a different command during this polling phase. For example, in Micron N25Q devices, it is bit 7 of the FLAG STATUS register (opcode 0x70) instead of bit 0 of the STATUS register that must be polled. For this reason, the OSPI\_WCCTL register has been provided for software to determine the bit and the opcode to use to poll for write completion, and also the number of successive valid polls that should take place.

To ensure that the bandwidth of the device is not affected by continuous read status SPI transactions, prolong the delay between the successive flash status read command by programming the OSPI\_WCCTL.REPDLY field. This feature is implemented to free up the SPI bandwidth, if required. However, defining this delay is not always desired as the ready bit indication can come later what impacts the overall performance. This register must be setup while the controller is idle.

When the OSPI controller starts to service a STIG request, it sets the OSPI\_FCCTL.STAT bit to indicate a command execution is in progress. When the OSPI controller is in the auto polling state, servicing a STIG request is different. Several devices are inaccessible after a program operation until the device has completed that write. Few of them have a possibility to suspend the programming page. It can be controlled by the OSPI\_POLSTAT.STAT bit, which indicates active auto polling phase. After requesting a STIG, the OSPI controller issues an appropriate opcode to memory. While servicing a STIG (in auto polling phase), the OSPI\_FCCTL.STAT bit remains steady and other parts of transfer such as address or dummy bits are disabled (to issue program suspend command, only opcode is needed).

## SPI Command Translation

Requests issued by the DAC or STIG are translated into a sequence of byte transfers to send downstream (before serializing the flash device). These sequences depend on the requested transfer, except the following example (1-byte non-sequential read):

## INSTRUCTION OPCODE &gt; ADDRESS &gt; MODEBYTE &gt; DUMMY CYCLES &gt; 1-DATA BYTE

NOTE: During data phase mode, data programmed in the OSPI\_MBCTL register is driven on pins. If mode data is not enabled, the address phase is followed directly by dummy cycles. During the dummy cycle phase the

OSPI data pins (D0-D7) are three-stated (High-z). To ensure the correct signal level on the data pins during this phase, external pull-ups can be used on data pins.

For sequential accesses, an extra data byte per read is pushed to the flash device on the back of the above sequence, assuming that it can be done without any gap between each transferred byte.

The actual sequence sent to the flash device depends on the requested transfers (non-sequential or sequential), whether the device is configured in XIP mode or not, and the state of the OSPI\_DRICTL and OSPI\_DWICTL registers. For writes, the write enable latch (WEL) within the flash device must be high before a write sequence can be issued.

The OSPI controller automatically issues a WEL command before triggering a write command via the DAC (the program need not perform this operation). To increase flexibility and performance, turn off this feature by setting the OSPI\_DWICTL.WELDIS bit. The opcode for WEL is 0x06 and common between the devices. However, the WEL operation is programmed using the OSPI\_OE\_LWR.XRDBYT / OSPI\_OE\_UP.FSTBYTWEL bits and (optionally) the OSPI\_OE\_LWR.XWRBYT / OSPI\_OE\_UP.XBYTWEL bits (flash device requires 2-byte command code for WEL).

When write requests from the DAC are not received and all outstanding requests have been sent, the flash device automatically starts the page program write cycle. Any incoming request at this time is held in wait state until the cycle is completed. The OSPI controller automatically polls the flash device legacy SPI status register to identify when the write cycle has completed. This is achieved by sending the Read Status Register (RDSR) opcode to the flash device and waiting until the device indicates that the write cycle is completed (the Write in Progress bit is cleared to zero and WEL bit is cleared to zero or device ready bit is set to one). The WREN and RDSR device instructions are only sent by the controller. For other specific instruction that the user determines must be sent to the device (for example, if the device needs to be unprotected before a write command is issued), must be separately handled by issuing flash commands via the STIG.

## Hold and Reset Control

There is an option to trigger hold and reset features on the I/O of the flash device. The hold functionality is common across the devices and takes an alternative function of the DQ3 pin (applicable when device does not operate in quad SPI or DDR modes). The transfer can be held and resumed by the OSPI\_CTL.HLD bit. Devices with hold feature on DQ3 need another dedicated pin for hardware reset. Devices without hold feature have an alternative reset on the DQ3 pin, which makes the additional reset pin redundant. The controller supports both variants. The OSPI\_CTL.RSTCFG field allows the user to configure the hardware reset solution implemented in the device. After configuration, it is possible to trigger hold or reset features using I/O ( OSPI\_CTL.HLD or OSPI\_CTL.RST bits).

NOTE: The OSPI controller does not support dedicated reset pin to control the reset of flash devices. Flash devices with reset functionality on DQ3 can be controlled directly via OSPI controller.

After hold activation, the controller is in waiting state. Any other operations must not be requested before de-asserting the OSPI\_CTL.HLD bit. The hold feature is useful when any SPI transaction needs to be prolonged to adjust it into specific point in time. T o check if the hold request is suspended, the OSPI\_CTL.IDLE bit can be polled

for. If SPI is not in the idle state, the transfer is suspended. Software must reset the OSPI\_CTL.HLD bit before any new SPI transaction. In case hold request is set before starting any transfer, it will be in hold state after it starts.

Hardware reset must be activated when CS is high (no valid transaction is present on SPI bus). It can be checked by polling the OSPI\_CTL.IDLE bit. If the controller is in the idle state and no other transfer requests are queued, hardware reset can be triggered. Reset feature is useful when any write, program, or erase operation needs to be cancelled. No transfer request is permitted before driving the reset back to inactive. T riggering hold or reset on the DQ3 pin at the time the device is configured to work in Quad SPI or DDR mode, overwrites transfer data on the DQ3 with 0. This behavior is considered as a software error. Ensure that the flash device is introduced to suitable SPI Mode (by polling its Configuration Register) before triggering alternative DQ3 function.

## Flash Instruction Type Support

To send correct read and write opcodes, software must initialize the OSPI\_DRICTL and OSPI\_DWICTL registers. These registers include fields to setup the required instruction opcodes that is intended to be used to access the flash (default is basic read and basic page program) as well as the instruction type, edge mode (DDR or SDR) and whether the instruction uses single, dual, quad, or octal pins for address and data transfers.

To ensure that the controller can operate from a reset state, the registers are reset to an opcode compatible with single I/O standard SPI devices. Though it is applicable for both reads and writes, the OSPI\_DRICTL.INSTRTYP field appears only once (not included in the OSPI\_DWICTL register). If software sets this to any value other than 0, the OSPI\_DRICTL.ADDRTRNSFR / OSPI\_DWICTL.ADDRTRNSFR / OSPI\_DRICTL.DATATRNSFR / OSPI\_DWICTL.DATATRNSFR bits become don't care. It is made available to allow software to support less common flash instructions, where opcode, address, and data are sent on 2 or 4 data lines (opcode from several instructions are sent serially to the flash device, even for dual/quad instructions).

## Dual Data Rate (DDR) Operations

The OSPI controller supports DDR mode of operation, in which some or all transfer phases happen in DDR mode, where the I/O lines are driven/sampled on both rising and falling edges of the clock.

In DDR mode, the OSPI controller supports the following modes:

- Dual Transfer Rate (DTR) mode, where command is driven in SDR mode and address/dummy cycles/data are driven in DDR mode.
- DTR protocol mode, where everything including command is driven in DDR mode.

For DTR mode, there are specific commands which are recognized by the flash as DTR commands. These are DTR Read commands which can be handled in STR Protocol (but in DTR Mode). This is achieved by setting the OSPI\_CTL.DTREN bit to 0 and OSPI\_DRICTL.DDREN bit to 1.

For DTR protocol mode, all normal commands can be sent in DDR fashion, provided the flash is already configured in a mode to accept the commands in DDR mode. Support for DTR protocol provides significant enhancement. This is enabled in the OSPI controller by setting the OSPI\_DRICTL.DDREN bit to 1. It allows the device to work in DTR mode for each possible command (not only DTR read ones). It can also handle opcode phase in DTR mode, which improves the overall performance.

There are several devices (for example, Micron N25Q512A) that can handle read operations in DTR mode. They can issue and capture data on both rising and falling edges while working with a dedicated command type. This enables the controller to maintain same throughput twice the lower frequency of SPI\_CLK for STR mode.

In MT25Q family of Micron devices, DTR protocol is implemented. It enables device to handle all commands in DTR mode. DTR Read commands detect DTR mode based on dedicated opcode. Therefore, opcode must be sent as STR. When DTR protocol is enabled, the device does not need opcode to detect edge mode, as it can recognize based on the volatile or nonvolatile bit in the flash configuration register.

The Read Configuration Examples table illustrates how software must configure the controller for selected specific read and write instruction supported by the different devices.

Table 23-3: Read Configuration Examples

| Command                         | Opcode (Num- ber of Lanes/ Edge Mode)   | Address/ Dum- my/ Mode (Number of Lanes/ Edge Mode)   | Data (Number of Lanes/ Edge mode)   |   Instruction Type OSPI_ DRICTL. INSTRTYP |   Address Trans- fer Type OSPI_ DRICTL. ADDRTRNSFR |   Data Transfer Type OSPI_ DRICTL. DATATRNSFR |   DDR Bit Ena- ble OSPI_ DRICTL. DDREN |
|---------------------------------|-----------------------------------------|-------------------------------------------------------|-------------------------------------|-------------------------------------------|----------------------------------------------------|-----------------------------------------------|----------------------------------------|
| READ                            | 1/SDR                                   | 1/SDR                                                 | 1/SDR                               |                                         0 |                                                  0 |                                             0 |                                      0 |
| FAST_READ                       | 1/SDR                                   | 1/SDR                                                 | 1/SDR                               |                                         0 |                                                  0 |                                             0 |                                      0 |
| DTR FAST_READ                   | 1/SDR                                   | 1/DDR                                                 | 1/DDR                               |                                         0 |                                                  0 |                                             0 |                                      1 |
| DOR (Dual output Fast Read)     | 1/SDR                                   | 1/SDR                                                 | 2/SDR                               |                                         0 |                                                  0 |                                             1 |                                      0 |
| DTR DOR (Dual output Fast Read) | 1/SDR                                   | 1/DDR                                                 | 2/DDR                               |                                         0 |                                                  0 |                                             1 |                                      1 |
| DIOR (Dual I/O Fast Read)       | 1/SDR                                   | 2/SDR                                                 | 2/SDR                               |                                         0 |                                                  1 |                                             1 |                                      0 |
| DTR DIOR (Dual I/O Fast Read)   | 1/SDR                                   | 2/DDR                                                 | 2/DDR                               |                                         0 |                                                  1 |                                             1 |                                      1 |
| QOR (Quad output Fast Read)     | 1/SDR                                   | 1/SDR                                                 | 4/SDR                               |                                         0 |                                                  0 |                                             2 |                                      0 |
| DTR QOR (Quad output Fast Read) | 1/SDR                                   | 1/DDR                                                 | 4/DDR                               |                                         0 |                                                  0 |                                             2 |                                      1 |
| QIOR (Quad I/O Fast Read)       | 1/SDR                                   | 4/SDR                                                 | 4/SDR                               |                                         0 |                                                  2 |                                             2 |                                      0 |

Table 23-3: Read Configuration Examples (Continued)

| Command                              | Opcode (Num- ber of Lanes/ Edge Mode)   | Address/ Dum- my/ Mode (Number of Lanes/ Edge Mode)   | Data (Number of Lanes/ Edge mode)   |   Instruction Type OSPI_ DRICTL. INSTRTYP | Address Trans- fer Type OSPI_ DRICTL. ADDRTRNSFR   | Data Transfer Type OSPI_ DRICTL. DATATRNSFR   |   DDR Bit Ena- ble OSPI_ DRICTL. DDREN |
|--------------------------------------|-----------------------------------------|-------------------------------------------------------|-------------------------------------|-------------------------------------------|----------------------------------------------------|-----------------------------------------------|----------------------------------------|
| DTR QIOR (Quad I/O Fast Read)        | 1/SDR                                   | 4/DDR                                                 | 4/DDR                               |                                         0 | 2                                                  | 2                                             |                                      1 |
| OOR (Octal output Fast Read)         | 1/SDR                                   | 1/SDR                                                 | 8/SDR                               |                                         0 | 0                                                  | 3                                             |                                      0 |
| DTR OOR (Octal output Fast Read)     | 1/SDR                                   | 1/DDR                                                 | 8/DDR                               |                                         0 | 0                                                  | 3                                             |                                      1 |
| OIOR (Octal I/O Fast Read)           | 1/SDR                                   | 8/SDR                                                 | 8/SDR                               |                                         0 | 3                                                  | 3                                             |                                      0 |
| DTR OIOR (Octal I/O Fast Read)       | 1/SDR                                   | 8/DDR                                                 | 8/DDR                               |                                         0 | 3                                                  | 3                                             |                                      1 |
| DCFR (Dual Command Fast Read)        | 2/SDR                                   | 2/SDR                                                 | 2/SDR                               |                                         1 | don't care                                         | don't care                                    |                                      0 |
| DTR DCFR (Dual Com- mand Fast Read)  | 2/SDR                                   | 2/DDR                                                 | 2/DDR                               |                                         1 | don't care                                         | don't care                                    |                                      1 |
| QCFR (Quad Command Fast Read)        | 4/SDR                                   | 4/SDR                                                 | 4/SDR                               |                                         2 | don't care                                         | don't care                                    |                                      0 |
| DTR QCFR (Quad Com- mand Fast Read)  | 4/SDR                                   | 4/DDR                                                 | 4/DDR                               |                                         2 | don't care                                         | don't care                                    |                                      1 |
| OCFR (Octal Command Fast Read)       | 8/SDR                                   | 8/SDR                                                 | 8/SDR                               |                                         3 | don't care                                         | don't care                                    |                                      0 |
| DTR OCFR (Octal Com- mand Fast Read) | 8/SDR                                   | 8/DDR                                                 | 8/DDR                               |                                         3 | don't care                                         | don't care                                    |                                      1 |

NOTE: This data is applicable for 3-byte and 4-byte address variants of the commands.

In DTR protocol, all transfer phases (including opcode) take DDR edge mode independent of the command under execution. DTR protocol is enabled by the OSPI\_CTL.DTREN bit. It has higher priority than the OSPI\_DRICTL.DDREN bit. Therefore, if the OSPI\_CTL.DTREN bit is set, irrespective of value programmed in the OSPI\_DRICTL.DDREN bit, all command, address, dummy cycles, and data are transferred in DDR mode.

Table 23-4: Write Configuration Examples

| Command                                     |   Opcode (Number of Lanes) |   Address/ Dummy/Mode (Number of Lanes) |   Data (Number of Lanes) |   Instruction OSPI_DRICTL .INSTRTYP | Address Transfer Type OSPI_DWICTL .ADDRTRNSFR   | Data Transfer Type OSPI_DWICTL .DATATRNSFR   |
|---------------------------------------------|----------------------------|-----------------------------------------|--------------------------|-------------------------------------|-------------------------------------------------|----------------------------------------------|
| PP (Page Pro- gram)                         |                          1 |                                       1 |                        1 |                                   0 | 0                                               | 0                                            |
| DIFP (Dual In- put Fast Program)            |                          1 |                                       1 |                        2 |                                   0 | 0                                               | 1                                            |
| DIEFP (Dual In- put Extended Fast Program)  |                          1 |                                       2 |                        2 |                                   0 | 1                                               | 1                                            |
| QIFP (Quad In- put Fast Program)            |                          1 |                                       1 |                        4 |                                   0 | 0                                               | 2                                            |
| QIEFP (Quad In- put Extended Fast Program)  |                          1 |                                       4 |                        4 |                                   0 | 2                                               | 2                                            |
| OIFP (Octal In- put Fast Program)           |                          1 |                                       1 |                        8 |                                   0 | 0                                               | 3                                            |
| OIEFP (Octal In- put Extended Fast Program) |                          1 |                                       8 |                        8 |                                   0 | 3                                               | 3                                            |
| DCPP (Dual Command Page Program)            |                          2 |                                       2 |                        2 |                                   1 | don't care                                      | don't care                                   |
| QCPP (Quad Command Page Program)            |                          4 |                                       4 |                        4 |                                   2 | don't care                                      | don't care                                   |
| OCPP (Octal Command Page Program)           |                          8 |                                       8 |                        8 |                                   3 | don't care                                      | don't care                                   |

NOTE: This data is applicable for both 3-byte or 4-byte address variants of the commands.

In DTR protocol, all transfer phases (including opcode) take DDR edge mode independent of the command under execution.

## Data Sampling

The OSPI provides flexibility to configure the sampling point of read data with the SPI clock to meet setup and hold timings of the flash devices when operating at higher speed. The OSPI\_RDC register provides an option to control the sampling point. In OSPI controller, data received on the OSPI pins is sampled by internal reference clock (OSPI\_REFCLK). In STR mode, by default (when the OSPI\_RDC.DLYRD = 0), the incoming data is on the first falling edge of the OSPI\_REFCLK in second half of SPI clock. In DTR mode, it is on the first falling edge of the OSPI\_REFCLK in second quarter of SPI clock.

The OSPI\_RDC.SMPLEDG bit selects the edge of the OSPI\_REFCLK, in which data from flash memory are sampled. This increases the sampling resolution by two times. The OSPI\_RDC.DLYRD field controls the additional number of OSPI\_REFCLK cycles that must be applied to the internal read data capture circuit. Large clock-to-out delay of the flash memory, trace delays, and other device delays may impose an upper limit on the flash clock frequency which is less for the flash memory to operate. To compensate this, software must set this register to a value that guarantees robust data captures.

The OSPI\_RDC.SMPLEDG bit selects the edge of the SYSCLK, in which data from flash memory are sampled. This increases the sampling resolution by two times. The OSPI\_RDC.DLYRD field controls the additional number of SYSCLK cycles that must be applied to the internal read data capture circuit. Large clock-to-out delay of the flash memory, trace delays, and other device delays may impose an upper limit on the flash clock frequency which is less for the flash memory to operate. To compensate this, software must set this register to a value that guarantees robust data captures.

For example, when the baud divider ( OSPI\_CTL.BAUD field) is programmed to 7, it results in divider of 16. Therefore, SPI\_CLK: OSPI\_REFCLK = 16:1 (every 1 SPI\_CLK cycle has 16 OSPI\_REFCLK cycles). When the OSPI\_RDC.DLYRD bits are programmed to X,

In STR mode of operation, received data is captured on:

- (16/2) + X th  falling edge of OSPI\_REFCLK if OSPI\_RDC.SMPLEDG = 0 · (16/2) + X th  rising edge of OSPI\_REFCLK if OSPI\_RDC.SMPLEDG = 1 In DTR mode of operation, received data is captured on: · (16/4) + X th  falling edge of OSPI\_REFCLK if OSPI\_RDC.SMPLEDG = 0 · (16/4) + X th  rising edge of OSPI\_REFCLK if OSPI\_RDC.SMPLEDG = 1

To improve the hold timing during transfers in DTR mode, the OSPI\_RDC.DDRDLYRD delays the transmitted data by programmable number of OSPI\_REFCLK cycles.

NOTE: The OSPI interface for Octal DTR mode may not work at higher frequencies for flash devices that require DQS.

## PHY Module Architecture

The PHY Module is divided into the following four sub-blocks:

- Data Transmitter

The data transmitter serializes the transmit data to assign complaint signal values into the controller output interface. The input data size is two bytes to cover the octal DDR scenario where two bytes are sent within a single reference clock cycle. The configuration interface covers selected SPI protocols (single, dual, quad, or octal) or command types (DDR, SDR). These values are necessary to calculate the number of bits that fit into a clock cycle and consequently what set value must be set by the hardware.

- Data Receiver

The data receiver block samples data from flash device.

- Delay Line Module

The delay line module block provides an appropriate delay for the input clock to ensure correct data transmitting and sampling. This block contains two separate delay paths for the OSPI clock, one feeding the external FLASH device and the other feeding the internal PHY sampling clock used for capturing data from the device when the PHY mode is enabled. Delay line values are configurable in software.

- Clock Arbiter

The clock arbiter handles gating the clock logic for clocks forwarded to the DLL inputs.

The PHY Module Block Diagram shows the sub-blocks in the PHY module. The sections following the diagram describe the architecture of the sub-blocks in more detail.

Figure 23-2: PHY Module Block Diagram

<!-- image -->

## PHY Data Transmitter

The data transmitter latches queued data to transmit on both the negative and positive edges of the reference clock. Then data is chopped to fit into the output interface. With the PHY module, in the last stage of the TX data path data from both edges are connected, forwarded back to the low-level OSPI controller, and then put on the OSPI controller outputs if the PHY enable bit is asserted. Otherwise, the controller outputs generated by the low-level OSPI controller are selected.

## PHY Data Receiver

In the data receiver module, data is sampled on both edges of the delayed reference clock (sampling clock) ensuring a wide range of the sampling window. Read data is gathered in shift registers and then merged into a byte-sized input FIFO. Data reception is programmable to work either with the delayed reference clock or with the DQS input signal from the flash device.

## Delay Line Module

Due to the asynchronous nature of flash devices, it is important to address the timing requirements for capturing and receiving data between the controller and the flash. The PHY module contains logic to meet the timing requirements. The delay compensation circuit is designed with the following features:

- Programmable OSPI clock delay specified as a percentage of a clock cycle

- Programmable sampling clock delay specified as a percentage of a clock cycle
- Delay compensation circuit re-sync circuitry activated during refresh cycles to compensate for temperature and voltage drift

The delay compensation circuitry relies on a controller/target approach. The controller delay line is used to determine how many delay elements constitute a complete cycle. This count, along with the programmable fractional delay settings, determines the actual number of delay elements to program into the target delay lines. The controller and target delay lines are identical. This approach allows the memory controller to observe a clock and then delay other signals by a fixed percentage of that clock.

DLL Locking is controlled through the register interface. When the DLL is reset, the controller DLL performs a locking procedure starting with the DLL start point value in the OSPI\_PHYMCTL.INITDLY bit field and the current frequency of operation. The DLL start point value must not exceed a delay of 1 and 1/4 cycles of delay, when calculated by estimating the worst case timing through a delay element at the highest frequency the DLL will be operating at. For example, if the maximum operating frequency is 200 MHz (period of 5 ns) and the worst case delay element has a 85 ps delay, then program the DLL start point to 5 / 0.085 = 58.8 = 59 elements = 0x3B.

With this setting, the controller DLL is will correctly lock for all frequencies below 200 MHz and in any process corner. If the delay provided by the delay line is enough to cover a full clock cycle, the controller DLL is in full clock mode. In this case, the DLL lock value in the OSPI\_DLLOB\_LWR.LWRLOCK bit field reports the number of delay element in one full clock cycle. The target delay line fractional setting uses this number to determine the number of delay elements to add to the target delay lines. For example, if the DLL lock value = 50 = 0x32, then the target delay line percentage = 25% = 0x20, and the target delay line delay value = 50 × 0.25 = 12.5 elements. The DLL Module will round the calculated value into an integer number of delay elements.

If the frequency of operation is such that the delay line is not long enough to accommodate a full cycle of delay, the controller DLL will automatically detect this and switch to half clock mode. In this mode, the controller DLL attempts to lock when the delay in the delay line reaches a half-clock cycle. If lock is achieved in half clock mode, the target delay lines are automatically adjusted to program a fractional delay of a full cycle. There is no need to change the target delay settings based upon the lock mode of the controller DLL. For example, if the DLL lock value = 50 = 0x32, then the target delay line percentage = 25% = 0x20, and the target delay line delay value = (50 2) × 0.25 = 25 elements. Do not work in half clock mode when using a loop-back sampling method or when the SPI clock is configured to SPI Mode 3 (CPHA =1, CPOL =1).

If the frequency of operation is such that the delay line is not long enough to capture a half-clock cycle of delay, the controller DLL will indicate lock and set the number of delays to the maximum length of the delay line. This is called saturation mode. There is no need to change the target delay settings in this mode. The target delay settings will be fixed at the fractional delay based upon the maximum delay of the delay line times 2. For example, if the DLL lock value = MAX = 128 = 0x80, and the target delay line percentage = 25% = 0x20, then the target delay line delay value = (128 × 2) × 0.25 = 64 elements.

## PHY Clock Arbiter

The OSPI clock is generated only when the SPI control logic FSM indicates that an SPI transfer is ongoing. Conversely, the reference clock is continuously generated. This module generates a sampling clock that is ready to pass

through the RX delay line. It is generated when read data phase of SPI transfer is detected based on information from the SPI control module. The sampling clock is optionally sourced from DQS or loop-back input.

## PHY Pipeline Mode

The PHY pipeline mode is for a direct read mode of operation. The flash command generator pipelines and puts a few expected sequential accesses into the TX FIFO causing the CS to stay active since the low-level OSPI controller controls the internal TX FIFO fill level. To correctly trigger a direct read in pipeline mode, first poll the OSPI\_CTL.IDLE bit to determine that the TX FIFO is empty. The sequential data transfer is interrupted when non-sequential access occur or too many wait states are introduced by the SPB controller to keep the flash transaction continuous. Introducing wait states slows down the system data rate.

Enable this mode when following conditions are met:

- SYSCLK &gt; 1.1 × OSPI\_REFCLK

NOTE: In PHY mode the reference clock is taken from the CDU CLKO10

- Accesses are only 32-bits in size

## Programming Concepts

Software configures the OSPI controller before communicating with the flash device. The static configuration bits must be setup before the OSPI controller is enabled using the OSPI\_CTL.EN field. T o change the controller configuration, disable it before reconfiguring.

## Configuring OSPI after Reset

The OSPI controller comes up with a state that is suitable to perform basic reads and writes using the DAC. Basic read (opcode 0x03) and write (opcode 0x02) instructions are supported by all target devices. The controller also wakes up with a baud rate divider setting of 32. When the reference clock operates at 500 MHz after reset, the effective SPI clock is 15.625 MHz. This must be slow enough to meet all timing requirements of all target devices without any further device programming.

When the target device does not use three address bytes, the OSPI\_DSCTL register must be modified to the appropriate size. If software wants writes to the device and the number of bytes per device page is not equal to 256, the OSPI\_DSCTL register must also be modified. The software must enable the write protect feature prior to enabling the OSPI controller. This blocks any AHB writes from taking effect. The OSPI\_WRPROT\_LWR , OSPI\_WRPROT\_UP , OSPI\_WRPROT\_CTL , and OSPI\_DSCTL.PGSZ fields must be setup.

After reset, software can read from and write to the flash device. Enabling/disabling the controller and DAC is achieved by writing to the corresponding OSPI\_CTL register fields. Maintain the default values of the baud rate divisor and default state of the OSPI\_CTL.CPOL / OSPI\_CTL.CPHA bits. A write data value of 0x00780081 is recommended.

## Programming Dummy Cycles

When programming the dummy cycles for OSPI reads (DAC and STIG-initiated reads) from the flash device, mode data should be considered as it impacts the actual number of dummy cycles seen on the bus. When mode data is not enabled, then the number of dummy cycles is that same as those set in dummy cycles field, respectively, for STIG or DAC operation.

However, when mode data is enabled, then in addition to the programmed number of dummy cycles, a few extra cycles are needed to transmit out the mode data. The number of cycles added depends on the bus mode of the OSPI. For example, when the address phase of the command is configured to be sent on a single line in STR mode, then the mode bits consume eight cycles. When the address phase of the command is configured to be sent on four lines in DTR mode, then the mode bits consume one cycle. These extra cycles should be considered when configuring the dummy cycles for the controller, so the overall dummy clock cycles match the dummy cycles provided in flash data sheet.

## Configuring OSPI for Optimal Use

Software must accurately configure the controller to optimally access the flash.

To configure the controller:

1. Wait until pending STIG are completed or poll the OSPI\_CTL.IDLE field.
2. Disable the OSPI\_CTL.DACEN field. The OSPI controller can be completely disabled using the OSPI\_CTL.EN field.
3. Update the OSPI\_DLY register. This register allows the user to tweak how the chip select is driven after each flash access. This is required as each device may have different timing requirements. As the serial clock frequency is increased, these timing requirements become critical. The numbers programmed in this register are based on the period of OSPI\_REFCLK.

For example, an ATMEL device needs 50 ns minimum time before CS can be re-asserted after it has been deasserted. By default, the controller provides a a minimum of one SCLK period. When the device is operating at 50 MHz, the SCLK period is only 20 ns. Therefore, additional 30 ns are required. As the register defines the number of OSPI\_REFCLK cycles to add, if OSPI\_REFCLK is running at 500 MHz (2 ns period), user must program a value of at least 15 to the OSPI\_DLY.DSRT bits. This delay can be extended during auto-polling phase. Polling repetition delay can be defined in the OSPI\_WCCTL.REPDLY field.

4. Update the OSPI\_DSCTL register. The number of bytes per page is required to perform any write operation on flash. The number of bytes per device block is only required if the write protect feature is used.
5. Setup and enable the write protection registers ( OSPI\_WRPROT\_LWR , OSPI\_WRPROT\_UP , and OSPI\_WRPROT\_CTL ), if write protection features are required.
6. Enable the required interrupts using the OSPI\_IMSK register.
7. Update the OSPI\_REMAPADDR register to remap the DAC addresses to different address in flash.
8. Set the baud rate divisor in the OSPI\_CTL.BAUD to define the required clock frequency of the target device.

9. Select the appropriate chip select signal to use through the OSPI\_CTL.BAUD bits.
10. Update the OSPI\_RDC register. This register delays when read data is captured and helps when read data path from the device to the controller is long and the device clock frequency is high.
11. Enable the OSPI controller and DAC using the OSPI\_CTL register.

## Configuring OSPI for DAC Read Operation

1. Configure the OSPI\_DRICTL register:
- a. Program the desired opcode for Read command (0xBB for Dual SPI read).
- b. Configure th instruction type, address transfer type, and data transfer type as per the opcode.
- c. Configure the dummy cycles for the selected command opcode depending on the flash device.
- d. Set the Mode bit Enable if the mode data needs to be transferred during dummy cycles phase.
- e. Set the DDR enable bit if the command works in DTR mode.
2. If the operation requires dual opcode, set the OSPI\_CTL.OPCODEEN bit.
3. Update the second byte for read opcode in the OSPI\_OE\_LWR.XRDBYT field.
4. To operate in DTR protocol mode, set the OSPI\_CTL.DTREN bit.
10. If this bit is set, the OSPI\_DRICTL.DDREN bit has no effect and all phases of command including the opcode are done in DDR mode.
5. Update the OSPI\_DSCTL.ADDRSZ field.
6. Update the OSPI\_MBCTL.MODE field.
7. Enable the DAC mode using the OSPI\_CTL.DACEN bit, if not enabled.
8. Start the read transfer by accessing the OSPI memory mapped space through core or MDMA accesses. Any access to address 0x60000000 will access the flash address 0x0 if address remapping is 0.
9. If using MDMA for the transfer, wait for the DMA to complete.
16. NOTE: For accesses through core, software need not check FIFO levels. All the FIFOs are internally maintained by DAC.

## Configuring OSPI for DAC Write Operation

1. Configure the OSPI\_DWICTL register:
- a. Program the desired opcode for write command like 0x02 for single SPI page program.
- b. Configure the address transfer type and data transfer type as per the opcode.
- c. Configure the dummy cycles for selected command opcode depending on the flash device.

2. Update the OSPI\_DRICTL.INSTRTYP field as per the selected opcode.
3. Clear the OSPI\_DWICTL.WELDIS bit to manually send the WEL (Write Enable) instruction to flash. Else, the controller automatically issues WEL to flash before sending any write command to flash.
3. If WEL command for flash under use is not 0x06, update the OSPI\_OE\_UP.FSTBYTWEL field.
4. If the operation requires dual opcode, set the OSPI\_CTL.OPCODEEN bit.
5. Update the second byte for read opcode in the OSPI\_OE\_LWR.XWRBYT field.
6. Update the number of address bytes the command expects in the OSPI\_DSCTL.ADDRSZ field.
7. As the controller supports the automatic polling for write operation complete, update the OSPI\_WCCTL register accordingly to enable the automatic polling for flash program operation complete.
- a. Enable the automatic polling by clearing the OSPI\_WCCTL.DIS bit.
- b. Update Polling opcode and define polling bit index and polarity as per the flash device.
- c. Update polling count and polling repetition delay.
- d. Optionally, update the OSPI\_POLLEXP register if polling expiration is enabled.
8. Enable the DAC mode using the OSPI\_CTL.DACEN , if not enabled.
9. Start the read transfer by accessing the OSPI memory mapped space through core or MDMA accesses. Any access to address 0x60000000 will access the flash address 0x0 if address remapping is 0.
10. Wait for the DMA to complete, if using MDMA for the transfer.
15. NOTE: For accesses through core, software need not check FIFO levels. All FIFOs are internally maintained by DAC.

## Issuing STIG Command

This is a typical method that software uses to access the flash device registers and perform erase operations. It can also be used to access the flash array (though only 8 data bytes can be read or written at a time), as defined in the OSPI\_FCRD\_LWR , OSPI\_FCWD\_LWR , OSPI\_FCRD\_UP and, OSPI\_FCWD\_UP registers.

To issue a STIG command:

1. Set the OSPI\_IMSK.STIGREQ\_MSK bit to enable the STIG command completion interrupt.
2. Update the OSPI\_FCCTL.OPCODE and OSPI\_FCCTL.DMY bits, as per the flash command that need to be executed via STIG.
3. Clear the OSPI\_FCCTL.STIGBNKEN bit.
4. Set the OSPI\_FCCTL.ADDREN bit, iwhen the command requires the address to be driven.

When set:

- a. Update the OSPI\_FCCTL.ADDRSZ field according to the number of address bytes expected by the command.
- b. Update the OSPI\_FCA register with the desired flash address.
5. When the command expects the data to be sent to flash, set the OSPI\_FCCTL.WREN bit.

When set:

- a. Update the OSPI\_FCCTL.WRSZ field according to the number of data bytes to be sent to flash.
- b. Update the OSPI\_FCWD\_LWR and OSPI\_FCWD\_UP registers with data to be sent to flash.
6. When the command expects the data to be received from the flash, set the OSPI\_FCCTL.RDEN bit. When set, update the OSPI\_FCCTL.RDSZ field according to the number of data bytes to expected from flash.
7. Update the OSPI\_FCCTL.MODEEN bit, when the mode data are sent during dummy cycles. When set, update the OSPI\_MBCTL.MODE field with desired mode data.
8. Set the OSPI\_FCCTL.EXE bit to execute the STIG command.
9. Set the OSPI\_ISTAT.STIGREQ bit (or wait for interrupt if enabled) indicating that the STIG command execution is completed.
10. When the command expects the data to be received from the flash, read the data from the OSPI\_FCWD\_LWR and OSPI\_FCWD\_UP registers.

NOTE: Do not set the OSPI\_FCCTL.WREN and OSPI\_FCCTL.RDEN bits simultaneously. A given command can either read data or send data to the flash, but not both at the same time.

With normal STIG command, only 8 bytes can be read at a time, but with support of STIG memory bank up to 16 bytes can be read in to STIG memory bank. This is controlled by the OSPI\_FCCTL.STIGBNKEN bit and OSPI\_FCMCTL register. Commands issued using this interface have a higher priority than all other read accesses coming from AHB, and interrupts any read commands being requested by the DAC.

To issue a STIG memory bank read command:

1. Set the OSPI\_IMSK.STIGREQ\_MSK bit to enable STIG command completion interrupt.
2. Update the OSPI\_FCCTL.OPCODE and OSPI\_FCCTL.DMY bits as per the flash command that need to be executed via STIG.
3. Set the OSPI\_FCCTL.STIGBNKEN bit.
4. Set the OSPI\_FCCTL.ADDREN bit, if the command requires the address to be driven.

If set:

- a. Update the OSPI\_FCCTL.ADDRSZ field according to the number of address bytes expected by the command.

- b. Update the OSPI\_FCA register with the desired flash address.
5. Update the OSPI\_FCCTL.MODEEN bit, if the mode data are sent during dummy cycles.

If set, update the OSPI\_MBCTL.MODE field with the desired mode data.

6. Set the OSPI\_FCCTL.EXE bit to execute the STIG command.
7. Set the OSPI\_ISTAT.STIGREQ bit (or wait for interrupt if enabled) indicating that the STIG command execution is completed.

At this stage, 16 bytes of data read from the flash device are available in the STIG memory bank. To access this data, a STIG memory bank read must be issued. Each read can read one byte form STIG memory bank at a time. To read all data, 16 reads must be issued.

To read a data byte from STIG memory bank:

1. Set the OSPI\_FCMCTL.BNKADDR field. This can be any value from 0 to 16.
2. Set the OSPI\_FCMCTL.TRIGREQ bit to trigger the STIG memory bank read.
3. Poll the OSPI\_FCMCTL.BNKREQ bit to clear.
4. Read the requested data byte from the OSPI\_FCMCTL.BNKDATA .

## Entering XIP mode

The controller supports XIP operations to minimize the latency for back-to-back reads or code execution.

If the flash device comes in XIP on power up, software cannot discover the state of XIP from POR via the Flash Status Register reads. The only operation of a flash device when XIP mode is enabled, is an XIP read. In such cases, software must be aware that if flash enters XIP from POR, the OSPI\_MBCTL.MODE and OSPI\_CTL.XIPIMM fields must be set. This makes the controller to enter XIP mode immediately and start communicating with the flash device in XIP mode on next read issued. Therefore, it does not require the read opcode to be transferred. To exit XIP mode, this bit must be set to 0. This takes effect in the attached device only after the next read instruction is executed. Software must ensure that at least one read instruction is requested after resetting this bit before it can be sure XIP mode in the device is exited.

OSPI\_CTL.XIPRD : If it is not known that the flash device enters XIP from POR, and XIP from POR may be supported by the attached flash device, software can attempt to exit the XIP mode by issuing an XIP exit command using a STIG command. For this, software must be aware of the mode bit requirements of that device, as XIP entry and exit changes per device.

XIP mode is supported in several flash devices. However, flash manufacturers do not have a consistent standard approach. Most of them use signature bits that are sent to the device immediately following the address bytes. Few of them (such as Micron devices) use signature bits and require a flash device configuration register write to enable XIP .

The following section describes how software ensures entry into XIP mode for the flash devices compliant with the OSPI controller.

## Micron N25Q, MT25Q, and MT35X Devices

XIP mode must first be enabled by setting the corresponding field of VCR within the flash device. The VCR can be written to using the OSPI\_FCCTL register to issue a VCR write command.

1. Disable the DAC using the OSPI\_CTL.DACEN bit to ensure that no new AHB read accesses are sent to the flash device.
2. Use STIG mode to issue a VCR write to flash memory.
3. Set XIP mode bits in the OSPI\_MBCTL.MODE field to 8'b00000000.
4. Enable the OSPI controller XIP mode by setting the OSPI\_CTL.XIPRD bit.
5. Re-enable the DAC.

## Micron (Supporting Basic XIP Mode), Winbond, Spansion Devices

1. Disable the DAC using the OSPI\_CTL.DACEN bit to ensure that no new AHB read accesses are sent to the flash device.
2. Set XIP mode bits in the OSPI\_MBCTL.MODE to:
- 8'b10000000, for Micron devices supporting basic XIP mode
- 8'b00100000, for Winbond devices
- 8'b10100000, for Spansion devices
3. Enable XIP mode by setting the OSPI\_CTL.XIPRD bit.
4. Reenable the DAC.

## Using PHY Mode

The OSPI PHY mode extends the architecture of the OSPI flash controller to allow the interfacing of high-speed flash devices and ensure the reliable data transfers at high speed. For non-PHY operations, the controller assumes that OSPI transfer clock is at least 4 times lower than the reference clock to ensure correct data transactions in SDR mode and 8 times lower for DDR transactions. This provides a comfortable regulation of data synchronization, but it is not effective from a dynamic power efficiency standpoint.

The PHY mode allows transfers at speeds that match the increasing reference clocks of emerging higher performance flash devices, including frequencies close to 200 MHz. The PHY module works with the OSPI flash controller to enable DDR or SDR transfers at the device frequencies. For example, when a given flash device operates at 200 MHz, the reference clock is also 200 MHz, eliminating the need for high speed peripheral clocks.

The default reference clock (OSPI\_REFLK) for OSPI is SYSCLK, but to enable support for PHY mode the reference clock is selected from one of the options from CDU output clock 10 (Refer to the CDU chapter). By default, it is connected to SYSCLK which is suitable for non-PHY mode of operation. For PHY mode, the reference clock (OSPI\_REFLK) is either SCLK0\_0 or SCLK1\_1.

To configure PHY mode, first implement the procedure described in the Configuring OSPI for Optimal Use section and then proceed with the following steps based on the DLL mode configuration:

## DLL Bypass Mode

1. Enable the PHY mode by setting the OSPI\_CTL.PHYEN bit and enable the DDR protocol with the OSPI\_CTL.DTREN bit.

NOTE: Ensure that the device is configured to work in DDR Protocol (do not confuse with DDR commands).

2. Calibrate the software using the following sub steps:
- a. Calculate how many delay elements are necessary to shift the reference clock period by 25% and program this value in the OSPI\_PHYCTL.TXDLY bit field. This is the best case for DDR transfers from the setup/hold timing standpoint. The delay may vary slightly in a real design.
- b. Resynchronize the DLLs by setting the OSPI\_PHYCTL.RESYNC bit. If this has already been set by a previous resynchronization, toggle the bit to 0 and then back to 1 to trigger the re-synchronization DLL logic.
- c. Enable the PHY bypass mode by setting the OSPI\_PHYMCTL.BYPCTL bit.
- d. Trigger a read request from a location where the value is predictable. Depending on the device, this is the parameter page, ID, status, data from the OTP region, or data from the location of the flash array.
- e. Check for data correctness by incrementing the value of the RX clock delay using the OSPI\_PHYCTL.RXDLY bit field, resynchronizing the DLLs, triggering a valid read request, and then checking for the correct data and store information. Continue to check until the range boundary of the RX clock delay is achieved and then proceed to the next step.
3. Set the RX clock delay value with one from the middle of valid range based on the information in storage.
4. Resynchronize the DLLs
5. Set the device read instruction register for Octal Read DDR Configuration. Configure each transfer phase to work in octal mode and set the number of dummy cycles to a minimum of the number specified in the documentation of the device. More dummy cycles may be set to accommodate additional read path delays in actual systems data.
6. Enable pipeline mode by setting the OSPI\_CTL.PIPEPHYEN bit.
7. Perform a sequential read of data consistent with the conditions indicated in the PHY Pipeline Mode section.
8. Poll for IDLE with the OSPI\_CTL.IDLE bit and when it is asserted to high trigger the next transfer request.

## DLL Controller Mode

1. Enable the PHY mode by setting the OSPI\_CTL.PHYEN bit and enable the DDR protocol with the OSPI\_CTL.DTREN bit.

- NOTE: Ensure that the device is configured to work in DDR protocol (do not confuse with DDR commands).
2. Calibrate the software using the following sub steps:
- a. Drive the OSPI\_CTL.RST bit low to reset the DLL.
- b. Calculate initial delay value for the controller DLL according to the PHY DLL controller control register bits[6:0] and write this value into the OSPI\_PHYMCTL.INITDLY bit field.
- c. Set DLL reset bit ( OSPI\_CTL.RST ) back to high. The controller delay line automatically starts the locking procedure with the configuration given once the DLL module is in reset state.
- d. Poll the OSPI\_DLLOB\_LWR.LOCK bit. When it is set, the lock is done.
- e. Resynchronize the DLLs by setting the OSPI\_PHYCTL.RESYNC bit. If this has already been set by a previous resynchronization, toggle the bit to 0 and then back to 1 to trigger the re-synchronization DLL logic.
- f. Set the TX DLL delay bit field ( OSPI\_PHYCTL.TXDLY ) and the RX DLL delay bit field ( OSPI\_PHYCTL.RXDLY ) to the current percentage clock offsets. Wait 20 reference clock cycles for the new configuration to propagate before triggering the next SPI transfer.
- g. Trigger a read request from a location where the value is predictable. Depending on the device, this is the parameter page, ID, status, data from the OTP region, or data from the location of the flash array.
- h. Check for data correctness by incrementing the value of the RX clock delay using the OSPI\_PHYCTL.RXDLY bit field, resynchronizing the DLLs, triggering a valid read request, and then checking for the correct data and store information. Continue to check until the range boundary of the RX clock delay is achieved and then proceed to the next step.
3. Set the RX clock delay value with one from the middle of valid range based on the information in storage.
4. Resynchronize the DLLs
5. Set the device read instruction register for octal read DDR configuration. Configure each transfer phase to work in octal mode and set the number of dummy cycles to a minimum of the number specified in the documentation of the device. More dummy cycles may be set to accommodate additional read path delays in actual systems data.
6. Enable pipeline mode by setting the OSPI\_CTL.PIPEPHYEN bit.
7. Perform a sequential read of data consistent with the conditions indicated in the PHY pipeline Mode section.
8. Poll for IDLE with the OSPI\_CTL.IDLE bit and when it is asserted to high trigger the next transfer request.

## ADSP-SC59x OSPI Register Descriptions

Octal SPI (OSPI) contains the following registers.

Table 23-5: ADSP-SC59x OSPI Register List

| Name            | Description                               |
|-----------------|-------------------------------------------|
| OSPI_CTL        | Octal SPI Control Register                |
| OSPI_DLY        | Device Delay Register                     |
| OSPI_DRICTL     | Device Read Instruction Control Register  |
| OSPI_DWICTL     | Device Write Instruction Control Register |
| OSPI_DSCTL      | Device Size Control Register              |
| OSPI_DLLOB_LWR  | DLL Observable Register (Lower)           |
| OSPI_DLLOB_UP   | DLL Observable Register (Upper)           |
| OSPI_FCA        | Flash Command Address Register            |
| OSPI_FCCTL      | Flash Command Control Register            |
| OSPI_FCMCTL     | Flash Command Control Memory Register     |
| OSPI_FCRD_LWR   | Flash Command Read Data Register (Lower)  |
| OSPI_FCRD_UP    | Flash Command Read Data Register (Upper)  |
| OSPI_FCWD_LWR   | Flash Command Write Data Register (Lower) |
| OSPI_FCWD_UP    | Flash Command Write Data Register (Upper) |
| OSPI_IMSK       | Interrupt Mask Register                   |
| OSPI_ISTAT      | Interrupt Status Register                 |
| OSPI_WRPROT_LWR | Lower Write Protection Register           |
| OSPI_MBCTL      | Mode Bit Control Register                 |
| OSPI_MODID      | Module ID Register                        |
| OSPI_POLLEXP    | Polling Expiration Register               |
| OSPI_OE_LWR     | Opcode Extension Register (Lower)         |
| OSPI_OE_UP      | Opcode Extension Register (Upper)         |
| OSPI_PHYCTL     | PHY Control Register                      |
| OSPI_PHYMCTL    | PHY DLL Master Control Register           |
| OSPI_POLSTAT    | Polling Flash Status Register             |
| OSPI_RDC        | Read Data Capture Register                |
| OSPI_REMAPADDR  | Remap Address Register                    |
| OSPI_WRPROT_UP  | Upper Write Protection Register           |
| OSPI_WRPROT_CTL | Write Protection Control Register         |
| OSPI_WCCTL      | Write Completion Control Register         |

## Octal SPI Control Register

The OSPI\_CTL register contains basic configuration fields of the controller.

Figure 23-3: OSPI\_CTL Register Diagram

<!-- image -->

Table 23-6: OSPI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | IDLE       | Idle. The OSPI_CTL.IDLE bit indicates an idle status. The serial interface and low level SPI pipeline are idle. This is a status read-only bit. Note: This is a re-timed signal. There is an inherent delay in generating this signal. |

Table 23-6: OSPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | OPCODEEN   | Dual Byte Opcode Enable. The OSPI_CTL.OPCODEEN bit is set (=1) when the target Flash device supports dual byte opcode (Macronix MX25). It is applicable for Octal I/O mode or protocol only. It should be be cleared (=0) when the device is configured to work in another SPI mode. If enabled, the supplementing bytes are taken from the OSPI_OE_LWR and OSPI_OE_UP registers. |
| 25 (R/W)           | PIPEPHYEN  | Pipeline PHY Mode Enable. The OSPI_CTL.PIPEPHYEN bit is relevant only for configuration with PHY mod- ule. It is asserted to 1 between consecutive PHY pipeline reads transfers and deasserted to 0 otherwise.                                                                                                                                                                    |
| 24 (R/W)           | DTREN      | Enable DTR Protocol. The OSPI_CTL.DTREN bit enables the DTR protocol. It must be set if the device is configured to work in DTR protocol.                                                                                                                                                                                                                                         |
| 22:19 (R/W)        | BAUD       | Master Mode Baud Rate Divisor (2 to 32). The OSPI_CTL.BAUD bit field selects the baud rate divisor. SPI Baud Rate = (Master Reference Clock)/ Baud Rate Divisor where, Baud Rate Divisor is: 4'b0000 = /2 4'b0001 = /4 4'b0010 = /6 4'b0011 = /8 4'b0100 = /10 ... 4'b1111 = /32                                                                                                  |

Table 23-6: OSPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/W)           | XIPIMM     | Enter XIP Mode Immediately. When XIP is enabled, clearing the OSPI_CTL.XIPIMM bit (= 0) causes the control- ler to exit XIP mode on the next read instruction. When the OSPI_CTL.XIPIMM bit is set (= 1), operate the device in XIP mode im- mediately. Use this register when the external device wakes up in XIP mode (as per the contents of its non-volatile configuration register). The controller assumes the next read instruc- tion is passed to the device as an XIP instruction, and therefore does not require the read opcode to be transferred. Note: To exit XIP mode, the OSPI_CTL.XIPIMM bit should be cleared (=0). This takes effect in the attached device only after the next read instruction is executed. Soft- ware must ensure that at least one read instruction is requested after resetting the OSPI_CTL.XIPIMM bit (to ensure that XIP mode is exited). | Enter XIP Mode Immediately. When XIP is enabled, clearing the OSPI_CTL.XIPIMM bit (= 0) causes the control- ler to exit XIP mode on the next read instruction. When the OSPI_CTL.XIPIMM bit is set (= 1), operate the device in XIP mode im- mediately. Use this register when the external device wakes up in XIP mode (as per the contents of its non-volatile configuration register). The controller assumes the next read instruc- tion is passed to the device as an XIP instruction, and therefore does not require the read opcode to be transferred. Note: To exit XIP mode, the OSPI_CTL.XIPIMM bit should be cleared (=0). This takes effect in the attached device only after the next read instruction is executed. Soft- ware must ensure that at least one read instruction is requested after resetting the OSPI_CTL.XIPIMM bit (to ensure that XIP mode is exited). |
| 18 (R/W)           | XIPIMM     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | If XIP is enabled, it causes the controller to exit XIP mode on the next read instruction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 17 (R/W)           | XIPRD      | Enter XIP Mode on Next Read. When XIP is enabled, clearing the OSPI_CTL.XIPRD bit (= 0) causes the controller to exit XIP mode on the next read instruction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Enter XIP Mode on Next Read. When XIP is enabled, clearing the OSPI_CTL.XIPRD bit (= 0) causes the controller to exit XIP mode on the next read instruction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 17 (R/W)           | XIPRD      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | If XIP is enabled, it cause the controller to exit XIP mode on the next read instruction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 17 (R/W)           | XIPRD      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | If XIP is disabled, this setting informs the controller that the device is ready to enter XIP on the next read in- struction                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 16 (R/W)           | AHBADDREN  | Enable AHB Address Remapping. The OSPI_CTL.AHBADDREN bit enables the bus address remapping (direct access mode only).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Enable AHB Address Remapping. The OSPI_CTL.AHBADDREN bit enables the bus address remapping (direct access mode only).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 23-6: OSPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | WRPROT     | Write Protect Flash. The OSPI_CTL.WRPROT bit, when set (=1), drives the write protect pin of the Flash. This is resynchronized to the generated memory clock, as necessary.                                                                                                                                                                                                                                                                                                  | Write Protect Flash. The OSPI_CTL.WRPROT bit, when set (=1), drives the write protect pin of the Flash. This is resynchronized to the generated memory clock, as necessary.                                                                                                                                                                                                                                                                                                  |
| 13:10 (R/W)        | SSEL       | Peripheral Chip Select Lines. The OSPI_CTL.SSEL bit field indicates the peripheral chip select line. If OSPI_CTL.SSELDCODE = 0, ss[3:0] is output: ss[3:0] .......... SSEL[3:0] 4'bxxx0..........1110.......SSEL0 selected 4'bxx01..........1101.......SSEL1 selected 4'bx011..........1011.......SSEL2 selected 4'b0111..........0111.......SSEL3 selected 4'b1111..........1111.......No peripheral selected If OSPI_CTL.SSELDCODE = 1, ss[3:0] directly drives SSEL[3:0]. | Peripheral Chip Select Lines. The OSPI_CTL.SSEL bit field indicates the peripheral chip select line. If OSPI_CTL.SSELDCODE = 0, ss[3:0] is output: ss[3:0] .......... SSEL[3:0] 4'bxxx0..........1110.......SSEL0 selected 4'bxx01..........1101.......SSEL1 selected 4'bx011..........1011.......SSEL2 selected 4'b0111..........0111.......SSEL3 selected 4'b1111..........1111.......No peripheral selected If OSPI_CTL.SSELDCODE = 1, ss[3:0] directly drives SSEL[3:0]. |
| 9 (R/W)            | SSELDCODE  | Peripheral Select Decode. The OSPI_CTL.SSELDCODE bit indicates the peripheral select decode.                                                                                                                                                                                                                                                                                                                                                                                 | Peripheral Select Decode. The OSPI_CTL.SSELDCODE bit indicates the peripheral select decode.                                                                                                                                                                                                                                                                                                                                                                                 |
| 9 (R/W)            | SSELDCODE  | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Only one of the 4 selects n_ss_out [3:0] is active                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/W)            | DACEN      | Direct Access Controller Enable. The OSPI_CTL.DACEN bit enables the direct access controller. When the direct access controller and indirect access controller are disabled, all bus re- quests are completed with an error response.                                                                                                                                                                                                                                        | Direct Access Controller Enable. The OSPI_CTL.DACEN bit enables the direct access controller. When the direct access controller and indirect access controller are disabled, all bus re- quests are completed with an error response.                                                                                                                                                                                                                                        |
| 7 (R/W)            | DACEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Disable Direct Access Controller once current transfer of the data word is complete                                                                                                                                                                                                                                                                                                                                                                                          |
| 7 (R/W)            | DACEN      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Enable Direct Access Controller                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 6 (R/W)            | RSTCFG     | Reset Configuration. The OSPI_CTL.RSTCFG bit indicates the reset pin configuration.                                                                                                                                                                                                                                                                                                                                                                                          | Reset Configuration. The OSPI_CTL.RSTCFG bit indicates the reset pin configuration.                                                                                                                                                                                                                                                                                                                                                                                          |
| 6 (R/W)            | RSTCFG     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Reset Feature on DQ3 Pin of the Device                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 6 (R/W)            | RSTCFG     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | Reset Feature on Dedicated Pin of the Device (Control- ling the 5th bit Influences the Output)                                                                                                                                                                                                                                                                                                                                                                               |
| 5 (R/W)            | RST        | Reset Pin. The OSPI_CTL.RST bit, when set (=1), drives the reset pin of the Flash. When                                                                                                                                                                                                                                                                                                                                                                                      | Reset Pin. The OSPI_CTL.RST bit, when set (=1), drives the reset pin of the Flash. When                                                                                                                                                                                                                                                                                                                                                                                      |

Table 23-6: OSPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | HLD        | Hold Pin. The OSPI_CTL.HLD bit, when set (=1), drives the hold pin of the Flash. When cleared (=0), it deactivates the hold pin.                                                                             | Hold Pin. The OSPI_CTL.HLD bit, when set (=1), drives the hold pin of the Flash. When cleared (=0), it deactivates the hold pin.                                                                             |
| 3 (R/W)            | PHYEN      | PHY Mode Enable. When the OSPI_CTL.PHYEN bit is enabled, the controller is informed that PHY module is to be used for handling SPI transfers. This bit is relevant only for configura- tion with PHY module. | PHY Mode Enable. When the OSPI_CTL.PHYEN bit is enabled, the controller is informed that PHY module is to be used for handling SPI transfers. This bit is relevant only for configura- tion with PHY module. |
| 2 (R/W)            | CPHA       | Clock Phase. The OSPI_CTL.CPHA bit selects the clock phase.                                                                                                                                                  | Clock Phase. The OSPI_CTL.CPHA bit selects the clock phase.                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                            | SPI clock is active outside the word                                                                                                                                                                         |
| 1 (R/W)            | CPOL       | Clock Polarity. The OSPI_CTL.CPOL bit selects clock polarity outside SPI word.                                                                                                                               | Clock Polarity. The OSPI_CTL.CPOL bit selects clock polarity outside SPI word.                                                                                                                               |
|                    |            | 0                                                                                                                                                                                                            | OSPI clock is quiescent low                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                            | OSPI clock is quiescent high                                                                                                                                                                                 |
| 0 (R/W)            | EN         | Enable. The OSPI_CTL.EN bit enables OSPI operation.                                                                                                                                                          | Enable. The OSPI_CTL.EN bit enables OSPI operation.                                                                                                                                                          |
|                    |            | 0                                                                                                                                                                                                            | Disable OSPI Module                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                            | Enable OSPI Module                                                                                                                                                                                           |

## Device Delay Register

The OSPI\_DLY register introduces relative delays in the generation of the master output signals. All timings are defined in cycles of the SPI REFERENCE CLOCK/ext\_clk.

Figure 23-4: OSPI\_DLY Register Diagram

<!-- image -->

Table 23-7: OSPI\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | DSRT       | Clock Delay for Chip Select Deassert. The OSPI_DLY.DSRT field indicates the delay in the master reference clocks for the length that the master mode chip select outputs are deasserted between transactions. The minimum delay is always an SCLK period to ensure the chip select is not asserted again within an SCLK period. |
| 23:16 (R/W)        | DACT       | Clock Delay for Chip Select Deactivation. The OSPI_DLY.DACT field indicates the delay in the master reference clocks be- tween one chip select being deactivated and the activation of another. This is used to ensure a quiet period between the selection of two different slaves and requires the transmit FIFO to be empty. |
| 15:8 (R/W)         | LSTTRAN    | Clock Delay for Last Transaction. The OSPI_DLY.LSTTRAN field indicates the delay in the master reference clocks between last bit of the current transaction and deasserting the device chip select (n_ss_out). By default, the chip select is deasserted on the cycle following the comple- tion of the current transaction.    |
| 7:0 (R/W)          | INIT       | Clock Delay. The OSPI_DLY.INIT field indicates the delay in the master reference clocks be- tween setting n_ss_out low and the first bit transfer.                                                                                                                                                                              |

## Device Read Instruction Control Register

This register defines the configuration of Multiple-SPI READ instruction. This register should be setup while the controller is idle.

Figure 23-5: OSPI\_DRICTL Register Diagram

<!-- image -->

Table 23-8: OSPI\_DRICTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 28:24 (R/W)        | DMYRD      | Dummy Read Clock Cycles. The OSPI_DRICTL.DMYRD field indicates the number of dummy clock cycles re- quired by the device for a read instruction. | Dummy Read Clock Cycles. The OSPI_DRICTL.DMYRD field indicates the number of dummy clock cycles re- quired by the device for a read instruction. |
| 20 (R/W)           | MODEEN     | Mode Enable. The OSPI_DRICTL.MODEEN field is set (=1) to ensure that the OSPI_MBCTL.MODE bits are sent following the address bytes.              | Mode Enable. The OSPI_DRICTL.MODEEN field is set (=1) to ensure that the OSPI_MBCTL.MODE bits are sent following the address bytes.              |
| 17:16 (R/W)        | DATATRNSFR | Data Transfer Type. The OSPI_DRICTL.DATATRNSFR field indicates the data transfer type for stand- ard SPI modes.                                  | Data Transfer Type. The OSPI_DRICTL.DATATRNSFR field indicates the data transfer type for stand- ard SPI modes.                                  |
|                    |            | 0                                                                                                                                                | SIO Mode. Data is shifted to the device on DQ0 only and from the device on DQ1 only.                                                             |
|                    |            | 1                                                                                                                                                | Used for Dual Input/Output instructions. For data transfers, DQ0 and DQ1 are used as both inputs and outputs.                                    |
|                    |            | 2                                                                                                                                                | Used for Quad Input/Output instructions. For data transfers, DQ0, DQ1, DQ2, and DQ3 are used as both inputs and outputs.                         |
|                    |            | 3                                                                                                                                                | Used for Quad Input/Output instructions. For data transfers, DQ[7:0] are used as both inputs and outputs.                                        |

Table 23-8: OSPI\_DRICTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W)        | ADDRTRNSFR | Address Transfer Type. The OSPI_DRICTL.ADDRTRNSFR field indicates the address transfer type for standard SPI modes.                                                                                                                                                                                                                                                                                       | Address Transfer Type. The OSPI_DRICTL.ADDRTRNSFR field indicates the address transfer type for standard SPI modes.                                                                                                                                                                                                                                                                                       |
| 13:12 (R/W)        | ADDRTRNSFR | 0                                                                                                                                                                                                                                                                                                                                                                                                         | Addresses can be shifted to the device on DQ0 only.                                                                                                                                                                                                                                                                                                                                                       |
| 13:12 (R/W)        | ADDRTRNSFR | 1                                                                                                                                                                                                                                                                                                                                                                                                         | Addresses can be shifted to the device on DQ0 and DQ1 only.                                                                                                                                                                                                                                                                                                                                               |
| 13:12 (R/W)        | ADDRTRNSFR | 2                                                                                                                                                                                                                                                                                                                                                                                                         | Addresses can be shifted to the device on DQ0, DQ1, DQ2, and DQ3.                                                                                                                                                                                                                                                                                                                                         |
| 13:12 (R/W)        | ADDRTRNSFR | 3                                                                                                                                                                                                                                                                                                                                                                                                         | Addresses can be shifted to the device on DQ[7:0]                                                                                                                                                                                                                                                                                                                                                         |
| 10 (R/W)           | DDREN      | DDR Enable. The OSPI_DRICTL.DDREN field indicates whether the opcode from the OSPI_DRICTL.OPCODERD field is compliant with one of the DDR read com- mands. Set the OSPI_DRICTL.DDREN field to 1 when opcode from bits 7 to 0 is compliant with DDR command. Master output data is issued by the controller in DDR fashion starting of the address SPI transfer phase. It is not applicable for STIG Mode. | DDR Enable. The OSPI_DRICTL.DDREN field indicates whether the opcode from the OSPI_DRICTL.OPCODERD field is compliant with one of the DDR read com- mands. Set the OSPI_DRICTL.DDREN field to 1 when opcode from bits 7 to 0 is compliant with DDR command. Master output data is issued by the controller in DDR fashion starting of the address SPI transfer phase. It is not applicable for STIG Mode. |
| 9:8 (R/W)          | INSTRTYP   | Instruction Type. The OSPI_DRICTL.INSTRTYP field indicates the instruction type.                                                                                                                                                                                                                                                                                                                          | Instruction Type. The OSPI_DRICTL.INSTRTYP field indicates the instruction type.                                                                                                                                                                                                                                                                                                                          |
| 9:8 (R/W)          | INSTRTYP   | 0                                                                                                                                                                                                                                                                                                                                                                                                         | Use standard SPI mode. Data is shifted to the device on DQ0 only and from the device on DQ1 only.                                                                                                                                                                                                                                                                                                         |
| 9:8 (R/W)          | INSTRTYP   | 1                                                                                                                                                                                                                                                                                                                                                                                                         | Use DIO-SPI mode. Instruction sent on DQ0 and DQ1.                                                                                                                                                                                                                                                                                                                                                        |
| 9:8 (R/W)          | INSTRTYP   | 2                                                                                                                                                                                                                                                                                                                                                                                                         | Use QIO-SPI mode. Instruction sent on DQ0, DQ1, DQ2, and DQ3.                                                                                                                                                                                                                                                                                                                                             |
| 9:8 (R/W)          | INSTRTYP   | 3                                                                                                                                                                                                                                                                                                                                                                                                         | Use OIO-SPI mode. Instruction sent on DQ[7:0].                                                                                                                                                                                                                                                                                                                                                            |
| 7:0 (R/W)          | OPCODERD   | Read Opcode in Non-XIP Mode. The OSPI_DRICTL.OPCODERD field indicates the read opcode to use when not in                                                                                                                                                                                                                                                                                                  | Read Opcode in Non-XIP Mode. The OSPI_DRICTL.OPCODERD field indicates the read opcode to use when not in                                                                                                                                                                                                                                                                                                  |

## Device Write Instruction Control Register

This register defines the configuration of Multiple-SPI WRITE (Program Page) instruction. This register should be setup while the controller is idle.

Figure 23-6: OSPI\_DWICTL Register Diagram

<!-- image -->

Table 23-9: OSPI\_DWICTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                          | Description/Enumeration                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| 28:24 (R/W)        | DMYWR      | Dummy Write Clock Cycles. The OSPI_DWICTL.DMYWR field indicates the number of dummy clock cycles re- quired by the device for write instruction. | Dummy Write Clock Cycles. The OSPI_DWICTL.DMYWR field indicates the number of dummy clock cycles re- quired by the device for write instruction. |
| 17:16 (R/W)        | DATATRNSFR | Data Transfer Type. The OSPI_DWICTL.DATATRNSFR field indicates the data transfer type for stand- ard SPI modes.                                  | Data Transfer Type. The OSPI_DWICTL.DATATRNSFR field indicates the data transfer type for stand- ard SPI modes.                                  |
|                    |            |                                                                                                                                                  | 0 SIO Mode. Data is shifted to the device on DQ0 only and from the device on DQ1 only.                                                           |
|                    |            |                                                                                                                                                  | 1 Used for Dual Input/Output instructions. For data transfers, DQ0 and DQ1 are used as both inputs and outputs.                                  |
|                    |            |                                                                                                                                                  | 2 Used for Octal Input/Output Instructions. For data transfers, DQ0, DQ1, DQ2, and DQ3 are used as both inputs and outputs.                      |
|                    |            |                                                                                                                                                  | 3 Used for Octal Input/Output instructions. For data transfers, DQ[7:0] are used as both inputs and outputs.                                     |

Table 23-9: OSPI\_DWICTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W)        | ADDRTRNSFR | Address Transfer Type. The OSPI_DWICTL.ADDRTRNSFR field indicates the address transfer type for standard SPI modes. 0 Addresses can be shifted to the device on DQ0 only. 1 Addresses can be shifted to the device on DQ0 and DQ1. |
| 8 (R/W)            | WELDIS     | WEL Disable. The OSPI_DWICTL.WELDIS bit disables the automatic issuing of the WEL com- mand before a write operation for DAC or INDAC.                                                                                             |
| 7:0                | OPCODEWR   |                                                                                                                                                                                                                                    |
| (R/W)              |            | Write Opcode. Write Opcode                                                                                                                                                                                                         |

## Device Size Control Register

This register allows the user to define the memory organization of using Flash Devices. This register should be setup while the controller is idle.

Figure 23-7: OSPI\_DSCTL Register Diagram

<!-- image -->

Table 23-10: OSPI\_DSCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20:16 (R/W)        | BLKSZ      | Number of Bytes per Block. The controller uses the OSPI_DSCTL.BLKSZ field to perform write protection log- ic. The number of bytes per block must be a power of 2. 0: 1 byte 1: 2 bytes 3: 8 bytes ... 16: 65535 bytes and so on |
| 15:4 (R/W)         | PGSZ       | Number of Bytes per Device Page. The controller uses the OSPI_DSCTL.PGSZ field to perform flash writes up to and across page boundaries.                                                                                         |
| 3:0 (R/W)          | ADDRSZ     | Number of Address Bytes. The OSPI_DSCTL.ADDRSZ field indicates the address size for DAC mode. A value of 0 indicates 1 byte.                                                                                                     |

## DLL Observable Register (Lower)

The OSPI\_DLLOB\_LWR register contains the DLL status. This register is synchronized into the APB domain by the octal-SPI flash controller and updates dynamically.

Figure 23-8: OSPI\_DLLOB\_LWR Register Diagram

<!-- image -->

Table 23-11: OSPI\_DLLOB\_LWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | INCSTAT    | State of Cumulative DLL Lock Inc Register. The OSPI_DLLOB_LWR.INCSTAT field indicates the state of the cumulative dll_lock_inc register.                                             |
| 23:16 (R/NW)       | DECSTAT    | State of Cumulative DLL Lock Dec Register. The OSPI_DLLOB_LWR.DECSTAT field indicates the state of the cumulative dll_lock_dec register.                                             |
| 15 (R/NW)          | LOCK_LB    | Loop Back Lock. The OSPI_DLLOB_LWR.LOCK_LB bit indicates that lock of loop back is done.                                                                                             |
| 14:8 (R/NW)        | LWRLOCK    | DLL Observable Lower Lock Value. The OSPI_DLLOB_LWR.LWRLOCK field indicates the DLL encoder value from the master DLL to slave DLLs.                                                 |
| 7:3 (R/NW)         | UNLOCK     | DLL Observable Lower Unlock Counter. The OSPI_DLLOB_LWR.UNLOCK field indicates the number of increments or dec- rements required for the master DLL to complete the locking process. |
| 2:1 (R/NW)         | LOCK       | Mode in Which DLL is Locked. The OSPI_DLLOB_LWR.LOCK field indicates the mode in which the DLL has ach- ieved the lock.                                                              |

Table 23-11: OSPI\_DLLOB\_LWR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 0                  | STAT       | Indicates DLL Status.     |
| (R/NW)             |            |                           |

## DLL Observable Register (Upper)

The OSPI\_DLLOB\_UP register contains the DLL status. This register is synchronized into the APB domain by the octal-SPI flash controller and updates dynamically.

<!-- image -->

TXOP (R)

Output DLL Observable Upper Transmit Decoder

Figure 23-9: OSPI\_DLLOB\_UP Register Diagram

Table 23-12: OSPI\_DLLOB\_UP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 22:16 (R/NW)       | TXOP       | DLL Observable Upper Transmit Decoder Output. The OSPI_DLLOB_UP.TXOP field indicates the encoded value for the TX delay line for this slice. |
| 6:0 (R/NW)         | RXOP       | DLL Observable Upper Receive Decoder Output. The OSPI_DLLOB_UP.RXOP field indicates the encoded value for the RX delay line for this slice.  |

## Flash Command Address Register

This register allows the user to define the address of the command using by the STIG controller. This register should be setup while the controller is idle.

Figure 23-10: OSPI\_FCA Register Diagram

<!-- image -->

Table 23-13: OSPI\_FCA Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | ADDR       | Command Address. The OSPI_FCA.ADDR field indicates the address used by the command specified in the OSPI_FCCTL.OPCODE field. This must be setup before triggering the STIG command using the OSPI_FCCTL.EXE field. |

## Flash Command Control Register

This register controls SPI transactions generated by STIG. It allows the user to define corresponding SPI frame to particular command, triggering the transfer and polling for its completion.

Figure 23-11: OSPI\_FCCTL Register Diagram

<!-- image -->

Table 23-14: OSPI\_FCCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | OPCODE     | Command Opcode. The OSPI_FCCTL.OPCODE field must be configured before triggering the STIG command. For example, 0x20 maps to sub sector erase. Writing to the OSPI_FCCTL.EXE field launches the command. Note: Using this approach to issue commands to the device makes use of the instruc- tion type of the OSPI_DRICTL and OSPI_DWICTL registers. |
| 23 (R/W)           | RDEN       | Read Data Enable. When the OSPI_FCCTL.RDEN bit is set (=1), the command specified in the OSPI_FCCTL.OPCODE can read data bytes received from the device.                                                                                                                                                                                             |
| 22:20 (R/W)        | RDSZ       | Number of Read Data Bytes. The OSPI_FCCTL.RDSZ field indicates the number of read data bytes; up to 8 data bytes can be read using this command. Configure the OSPI_FCCTL.RDSZ field to 0 for 1 byte and 7 for 8 bytes.                                                                                                                              |

Table 23-14: OSPI\_FCCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19 (R/W)           | ADDREN     | Command Address Enable. When the OSPI_FCCTL.ADDREN bit is set (=1), the command specified in the OSPI_FCCTL.OPCODE field requires an address. This bit must be set before trigger- ing the command via writing a 1 to the execute field.                                                                       |
| 18 (R/W)           | MODEEN     | Mode Bit Enable. When the OSPI_FCCTL.MODEEN bit is set (=1), the OSPI_MBCTL.MODE field is sent following the address bytes.                                                                                                                                                                                    |
| 17:16 (R/W)        | ADDRSZ     | Number of Address Bytes. The OSPI_FCCTL.ADDRSZ field indicates the number of address bytes required (the address itself is programmed in the OSPI_FCA register. This field must be con- figured before triggering the command via bit 0 of this register. 0 1 address byte 1 2 address bytes 2 3 address bytes |
| 15 (R/W)           | WREN       | Write Data Enable. When set (=1), the command specified in the OSPI_FCCTL.OPCODE field enables write data bytes to be sent to the device.                                                                                                                                                                      |
| 14:12 (R/W)        | WRSZ       | Number of Write Data Bytes. The OSPI_FCCTL.WRSZ field indicates the number of write data bites; up to 8 data bytes may be written using this command. Configure to 0 for 1 byte, 7 for 8 bytes.                                                                                                                |
| 11:7 (R/W)         | DMY        | Number of Dummy Cycles. The OSPI_FCCTL.DMY field indicates the number of dummy cycles required. This field should be configured before triggering the command via the OSPI_FCCTL.EXE field.                                                                                                                    |
| 2 (R/W)            | STIGBNKEN  | STIG Memory Bank Enable. The OSPI_FCCTL.STIGBNKEN bit must be set (=1) before triggering the flash memory bank transfer. It must be cleared (=0) before triggering any other operation. The OSPI_FCCTL.STIGBNKEN bit is set before triggering the command via OSPI_FCCTL.EXE field.                            |
| 1 (R/NW)           | STAT       | Command Execution in Progress.                                                                                                                                                                                                                                                                                 |
| 0 (RX/W)           | EXE        | Execute the STIG Command.                                                                                                                                                                                                                                                                                      |

## Flash Command Control Memory Register

This register controls the Memory Bank accesses. It also defines the number of bytes intended to get by STIG access configured to use the STIG Memory Bank.

Figure 23-12: OSPI\_FCMCTL Register Diagram

<!-- image -->

Table 23-15: OSPI\_FCMCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------|
| 28:20 (R/W)        | BNKADDR    | Memory Bank Address. The OSPI_FCMCTL.BNKADDR field indicates the address of the memory bank from which data is read. |
| 18:16 (R/W)        | RDSZ       | Number of STIG Bytes. The OSPI_FCMCTL.RDSZ field indicates the number of read bytes for the extended STIG.           |
| 15:8 (R/NW)        | BNKDATA    | Last Requested Data from STIG Memory Bank.                                                                           |
| 1 (R/NW)           | BNKREQ     | Memory Bank Data Request in Progress.                                                                                |
| 0 (RX/W)           | TRIGREQ    | Trigger Memory Bank Data Request. Trigger the Memory Bank data request. This bit is internally synchronized.         |

## Flash Command Read Data Register (Lower)

This register keeps the last 4 bytes read by STIG SPI access.

Figure 23-13: OSPI\_FCRD\_LWR Register Diagram

<!-- image -->

Table 23-16: OSPI\_FCRD\_LWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | DATA       | Data. The OSPI_FCRD_LWR.DATA field contains data that is returned by the flash device for any status or configuration read operation carried out by triggering the STIG event in the OSPI_FCCTL register. The register is valid when the OSPI_FCCTL.STAT bit is low. |

## Flash Command Read Data Register (Upper)

This register keeps the last but 4 bytes read by STIG SPI access. This register in conjunction with the Flash Command Read Data Register (Lower) enables the controller to keep 8 last bytes read from the Flash Device using STIG.

Figure 23-14: OSPI\_FCRD\_UP Register Diagram

<!-- image -->

Table 23-17: OSPI\_FCRD\_UP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | DATA       | Data. The OSPI_FCRD_UP.DATA field contains data that is returned by the flash device for any status or configuration read operation carried out by triggering the STIG event in the OSPI_FCCTL register. The register is valid when the OSPI_FCCTL.STAT bit is low. |

## Flash Command Write Data Register (Lower)

The OSPI\_FCWD\_LWR register takes the first 4 bytes to be written by STIG.

Figure 23-15: OSPI\_FCWD\_LWR Register Diagram

<!-- image -->

Table 23-18: OSPI\_FCWD\_LWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DATA       | Command Write Data Lower Byte. The OSPI_FCWD_LWR.DATA field contains data written to the flash for any status or configuration write operation carried out by triggering the STIG event in the OSPI_FCCTL register. This field must be setup before triggering the command using the OSPI_FCCTL.EXE field. |

## Flash Command Write Data Register (Upper)

This register takes the bytes ranging from 5 to 8 to be written by STIG.

Figure 23-16: OSPI\_FCWD\_UP Register Diagram

<!-- image -->

Table 23-19: OSPI\_FCWD\_UP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DATA       | Command Write Data Upper Byte. The OSPI_FCWD_UP.DATA field contains data written to the flash for any status or configuration write operation carried out by triggering the event in the OSPI_FCCTL register. This field must be setup before triggering the command using the OSPI_FCCTL.EXE field. |

## Interrupt Mask Register

The OSPI\_IMSK register allows the user to mask/unmask particular interrupt sources. This register must be setup when the controller is idle.

Figure 23-17: OSPI\_IMSK Register Diagram

<!-- image -->

Table 23-20: OSPI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                 |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | STIGREQ_MSK  | STIG Request Completion Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled.      |
| 13 (R/W)           | POLEXP_MSK   | Polling Expiration Detected Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled.  |
| 5 (R/W)            | ILLACCES_MSK | Illegal Access Detected Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled.      |
| 4 (R/W)            | WRPROT_MSK   | Protected Area Write Attempt Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled. |
| 1 (R/W)            | UNDRFLW_MSK  | Underflow Detected Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled.           |
| 0 (R/W)            | MODEFAIL_MSK | Mode MFailure Mask. 0 = the interrupt for the corresponding interrupt status register bit is disabled. 1 = the interrupt for the corresponding interrupt status register bit is enabled.                |

## Interrupt Status Register

The status fields in the OSPI\_ISTAT register are set when the described event occurs, and the interrupt is enabled in the OSPI\_IMSK register. If any of these bit fields are set, the interrupt output is asserted high. The fields are each cleared by writing a 1 to the field.

Note: Bit fields [10:6] are valid only when legacy SPI mode is active.

Figure 23-18: OSPI\_ISTAT Register Diagram

<!-- image -->

Table 23-21: OSPI\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | STIGREQ    | STIG Request Completion Interrupt. The OSPI_ISTAT.STIGREQ bit indicates that the controller is ready to get anoth- er STIG request.                                                                                |
| 13 (R/W)           | POLEXP     | Poll Expiration Cycles. The OSPI_ISTAT.POLEXP bit indicates that the maximum number of program- med polls cycles has expired.                                                                                      |
| 5 (R/W)            | ILLACCES   | Illegal Bus Access Detected. The OSPI_ISTAT.ILLACCES bit indicates that an illegal bus access has been de- tected. Bus wrapping bursts and the use of SPLIT/RETRY accesses cause the error in- terrupt to trigger. |
| 4 (R/W)            | WRPROT     | Write to Protected Area is Attempted and Rejected. Write to protected area was attempted and rejected.                                                                                                             |

Table 23-21: OSPI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                  | Description/Enumeration                                                                                                                                                                             |
|--------------------|------------|----------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1                  | UNDRFLW    | Underflow Detection. The OSPI_ISTAT.UNDRFLW bit indicates that an underflow condition has been detected. | Underflow Detection. The OSPI_ISTAT.UNDRFLW bit indicates that an underflow condition has been detected.                                                                                            |
| (R/W)              |            | 0                                                                                                        | No underflow is detected.                                                                                                                                                                           |
|                    |            | 1                                                                                                        | underflow is detected and an attempt to transfer data is made when the small TX FIFO is empty. This may occur when AHB write data is slowly provided to keep up with the requested write operation. |

## Lower Write Protection Register

The OSPI\_WRPROT\_LWR register allows the user to define lower boundary of the write protection area. This register should be setup while the controller is idle.

Figure 23-19: OSPI\_WRPROT\_LWR Register Diagram

<!-- image -->

Table 23-22: OSPI\_WRPROT\_LWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | SUBSECT    | Subsector. The OSPI_WRPROT_LWR.SUBSECT field defines the lower end of the address range to be protected. The address to be programmed must be OSPI memory map ad- dress (starting from 0x60000000) shifted by the value in the OSPI_DSCTL.BLKSZ . For example, if lower address of flash to be protected is 0x100 and the OSPI_DSCTL.BLKSZ = 4, the value to be programmed in this register must be 0x60000100 >> 4, which is 0x06000010. |

## Mode Bit Control Register

This register allows the user to define the mode bits for corresponding Flash Device. It also provides configuration for CRC aware SPI transfers. This register should be setup while the controller is idle.

Figure 23-20: OSPI\_MBCTL Register Diagram

<!-- image -->

Table 23-23: OSPI\_MBCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|
| 7:0                | MODE       | Mode.                                                                                                                  |
| (R/W)              |            | The OSPI_MBCTL.MODE field is sent to the device following the address bytes when the mode bit transmission is enabled. |

## Module ID Register

This register provides the IP release number and the configuration data.

Figure 23-21: OSPI\_MODID Register Diagram

<!-- image -->

Table 23-24: OSPI\_MODID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/NW)       | PATCH      | Fix/Patch Number. The OSPI_MODID.PATCH field indicates the fix/patch number related to the OSPI_MODID.REVID field. |
| 23:8 (R/NW)        | REVID      | Module/Revision ID Number. The OSPI_MODID.REVID field holds the module revision.                                   |
| 1:0 (R/NW)         | CFGID      | Configuration ID. The OSPI_MODID.CFGID field holds the configuration ID number.                                    |

## Polling Expiration Register

This register defines maximum number of poll cycles. If the expected value of the bit being polled is not gotten after number defined in this register, the auto-polling is done on the next phase. The value stored in this register matters only if Write Completion Control Register bits[15] and Polling Flash Status Register bit[8] are set to "1". If autopolling is not disabled by Write Completion Control Register bits[14], at least two auto-polling phases are queued before execution and popped after polled bit have taken expected value. This register should be setup while the controller is idle.

Figure 23-22: OSPI\_POLLEXP Register Diagram

<!-- image -->

Table 23-25: OSPI\_POLLEXP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | PEC        | Poll Expiration Cycles. The OSPI_POLLEXP.PEC field defines the number of polls cycles before expira- tion. |

## Opcode Extension Register (Lower)

This register provides the supplementing opcodes for Dual Byte Opcode Mode activated by Octal-SPI Configuration Register bit[30].

Figure 23-23: OSPI\_OE\_LWR Register Diagram

<!-- image -->

Table 23-26: OSPI\_OE\_LWR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                |
|--------------------|------------|----------------------------------------|
| 31:24 (R/W)        | XRDBYT     | Supplement Byte of Any Read Opcode.    |
| 23:16 (R/W)        | XWRBYT     | Supplement Byte of Any Write Opcode.   |
| 15:8 (R/W)         | XPOLBYT    | Supplement Byte of Any Polling Opcode. |
| 7:0 (R/W)          | XSTIGBYT   | Supplement Byte of Any STIG Opcode.    |

## Opcode Extension Register (Upper)

This register provides the supplementing opcodes for Dual Byte Opcode Mode activated by Octal-SPI Configuration Register bit[30]. Additionally, it allows the user to define the Write Enable Latch (WEL) command first byte opcode used for automatic WEL insertion before any AHB write access.

Figure 23-24: OSPI\_OE\_UP Register Diagram

<!-- image -->

Table 23-27: OSPI\_OE\_UP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration            |
|--------------------|------------|------------------------------------|
| 31:24 (R/W)        | FSTBYTWEL  | First Byte of Any WEL Opcode.      |
| 23:16 (R/W)        | XBYTWEL    | Supplement Byte of Any WEL Opcode. |

## PHY Control Register

Figure 23-25: OSPI\_PHYCTL Register Diagram

<!-- image -->

Table 23-28: OSPI\_PHYCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (RX/W)          | RESYNC     | Resynchronization Delay. The OSPI_PHYCTL.RESYNC bit is used for resynchronization delay lines to update them with values from the OSPI_PHYCTL.RXDLY and OSPI_PHYCTL.TXDLY fields. |
| 30 (RX/W)          | RST        | DLL Reset. The OSPI_PHYCTL.RST bit is used for reset of Delay Lines by software.                                                                                                  |
| 29 (R/W)           | RXBYP      | Receive DLL Bypass. The OSPI_PHYCTL.RXBYP field determines if RX DLL is bypassed.                                                                                                 |
| 22:16 (R/W)        | TXDLY      | Transmit DLL Delay. The OSPI_PHYCTL.TXDLY field determines the number of delay elements to insert on data path between ref_clk and spi_clk.                                       |
| 6:0 (R/W)          | RXDLY      | Receive DLL Delay. The OSPI_PHYCTL.RXDLY field determines the number of delay elements to insert on data path between ref_clk and rx_dll_clk.                                     |

## PHY DLL Master Control Register

Figure 23-26: OSPI\_PHYMCTL Register Diagram

<!-- image -->

Table 23-29: OSPI\_PHYMCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/W)           | LCK        | PHY Master Lock Mode. The OSPI_PHYMCTL.LCK bit indicates whether the master delay line locks on a full cycle or half cycle of delay.                      |
| 23 (R/W)           | BYPCTL     | Bypass Mode Control. The OSPI_PHYMCTL.BYPCTL bit indicates the bypass mode of the master and slave DLLs.                                                  |
| 22:20 (R/W)        | PDSEL      | PHY Master Phase Detect Selector. The OSPI_PHYMCTL.PDSEL field selects the number of delay elements to be in- serted between the phase detect flip-flops. |
| 18:16 (R/W)        | IND        | Increment/Decrement Indication. The OSPI_PHYMCTL.IND field indicates the number of consecutive increment or decrement indications.                        |
| 6:0 (R/W)          | INITDLY    | Initial Delay. The OSPI_PHYMCTL.INITDLY field is the initial delay value for the DLL.                                                                     |

## Polling Flash Status Register

This register provides auto-polling data. It acts as the extension for the Write Completion Control Register where full status is not available and any action can be taken only relying on the indication of single bit being polled for.

Figure 23-27: OSPI\_POLSTAT Register Diagram

<!-- image -->

Table 23-30: OSPI\_POLSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | DMY        | Number of Dummy Cycles for Auto Polling. The OSPI_POLSTAT.DMY field indicates user-defined additional dummy cycles for read status during the auto-polling state. It may be necessary to add some cycles if the external delay of read data path shifts it outside the defined clock cycle. |
| 8 (R/NW)           | STAT       | Device Status Valid. The OSPI_POLSTAT.STAT bit is set (=1) when value in bits from 7 to 0 is valid.                                                                                                                                                                                         |
| 7:0 (R/NW)         | DEVSTAT    | Status Register of Device. The OSPI_POLSTAT.DEVSTAT field indicates the actual status register of the de- vice.                                                                                                                                                                             |

## Read Data Capture Register

This register is used to adjust SPI transfer conditions in order to fetch and capture data reliably. This register should be setup while the controller is idle.

<!-- image -->

DDRDLYRD (R/W)

DDR Read Delay

Figure 23-28: OSPI\_RDC Register Diagram

Table 23-31: OSPI\_RDC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | DDRDLYRD   | DDR Read Delay. The OSPI_RDC.DDRDLYRD field indicates the delay of the transmitted data by the number of ref_clk cycles. This field is relevant only when DDR read Command is exe- cuted. Otherwise, it can be ignored. | DDR Read Delay. The OSPI_RDC.DDRDLYRD field indicates the delay of the transmitted data by the number of ref_clk cycles. This field is relevant only when DDR read Command is exe- cuted. Otherwise, it can be ignored. |
| 8 (R/W)            | DQSEN      | DQS Enable. If the OSPI_RDC.DQSEN bit is set (=1), the signal from DQS input is driven into RX DLL and is used for data capturing in PHY mode instead of internally generated gated ref_clk.                            | DQS Enable. If the OSPI_RDC.DQSEN bit is set (=1), the signal from DQS input is driven into RX DLL and is used for data capturing in PHY mode instead of internally generated gated ref_clk.                            |
| 5 (R/W)            | SMPLEDG    | Sample Edge Selection. The OSPI_RDC.SMPLEDG bit selects the edge on which data outputs from flash memory is sampled.                                                                                                    | Sample Edge Selection. The OSPI_RDC.SMPLEDG bit selects the edge on which data outputs from flash memory is sampled.                                                                                                    |
|                    |            | 0                                                                                                                                                                                                                       | Data outputs from Flash are sampled on falling edge of the ref_clk.                                                                                                                                                     |
|                    |            | 1                                                                                                                                                                                                                       | Data outputs from Flash are sampled on rising edge of the ref_clk.                                                                                                                                                      |
| 4:1 (R/W)          | DLYRD      | Read Delay. The OSPI_RDC.DLYRD field indicates the delay of the read data capturing logic by                                                                                                                            | Read Delay. The OSPI_RDC.DLYRD field indicates the delay of the read data capturing logic by                                                                                                                            |

## Remap Address Register

This register allows the user to define the address offset for DAC accesses. This register should be setup while the controller is idle.

Figure 23-29: OSPI\_REMAPADDR Register Diagram

<!-- image -->

Table 23-32: OSPI\_REMAPADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | OFFSET     | Value. The OSPI_REMAPADDR.OFFSET field is used to remap an incoming bus address to a different address used by the Flash. |

## Upper Write Protection Register

This register allows the user to define upper boundary of the write protection area. This register should be setup while the controller is idle.

Figure 23-30: OSPI\_WRPROT\_UP Register Diagram

<!-- image -->

Table 23-33: OSPI\_WRPROT\_UP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | SUBSECT    | Subsector. The OSPI_WRPROT_UP.SUBSECT field defines the upper end of the address range to be protected. The address to be programmed must be an OSPI memory map ad- dress (starting from 0x60000000) shifted by the value in the OSPI_DSCTL.BLKSZ . For example, if the upper address of flash to be protected is 0x100 and the OSPI_DSCTL.BLKSZ = 4, the value to be programmed in this register must be 0x60000100 >> 4, which is 0x06000010. |

## Write Protection Control Register

This register allows the user to define the configuration of write protection settings. This register should be setup while the controller is idle.

Figure 23-31: OSPI\_WRPROT\_CTL Register Diagram

<!-- image -->

Table 23-34: OSPI\_WRPROT\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | EN         | Write Protection Enable Bit. When set (=1), any AHB write access with an address within the protection region de- fined in the lower and upper write protection registers is rejected. An AHB error re- sponse is generated and an interrupt source triggered. When cleared (=0), the protection region is disabled.                                                           |
| 0 (R/W)            | INV        | Write Protection Inversion Bit. When set (=1), the protection region defined in the lower and upper write protection registers is inverted meaning it is the region that the system is permitted to write to. When cleared (=0), the protection region defined in the lower and upper write protec- tion registers is the region that the system is not permitted to write to. |

## Write Completion Control Register

The OSPI\_WCCTL register defines how the controller polls the device following a write transfer.

Figure 23-32: OSPI\_WCCTL Register Diagram

<!-- image -->

Table 23-35: OSPI\_WCCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:24 (R/W)        | REPDLY     | Polling Repetition Delay. The OSPI_WCCTL.REPDLY field defines the additional delay required to ensure that the Chip Select is deasserted during auto polling phase.           |
| 23:16 (R/W)        | CNT        | Poll Count. The OSPI_WCCTL.CNT field defines the number of times the controller should ex- pect to see a true result from polling in successive reads of the device register. |
| 15 (R/W)           | EXPEN      | Enable Polling Expiration. The OSPI_WCCTL.EXPEN bit is set (=1) to enable auto polling expiration.                                                                            |
| 14 (R/W)           | DIS        | Disable Polling. The OSPI_WCCTL.DIS bit switches off automatic polling function.                                                                                              |
| 13 (R/W)           | POLRTY     | Polling Polarity. The OSPI_WCCTL.POLRTY bit defines the polling polarity.                                                                                                     |
| 13 (R/W)           | POLRTY     | 0 Write transfer to the device will be complete if the pol- led bit is 0.                                                                                                     |
| 13 (R/W)           | POLRTY     | 1 Write transfer to the device will be complete if the pol- led bit is 1.                                                                                                     |

Table 23-35: OSPI\_WCCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10:8 (R/W)         | INDEX      | Polling Index. The OSPI_WCCTL.INDEX field defines the bit index that must be polled. A value of 010 means that bit 2 of the returned data is polled for. A value of 111 means that bit 7 of the returned data is polled for.                                                                              |
| 7:0 (R/W)          | OPCODE     | Opcode. The OSPI_WCCTL.OPCODE field defines the opcode that must be issued by the controller when it is automatically polling for device program completion. This com- mand is issued followed all device write operations. By default, this polls the standard device status register using opcode 0x05. |