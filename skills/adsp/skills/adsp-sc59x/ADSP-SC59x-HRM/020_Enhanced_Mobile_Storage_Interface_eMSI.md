# Enhanced Mobile Storage Interface (eMSI)

<!-- source: 020_Enhanced_Mobile_Storage_Interface_eMSI.pdf | original pages 964–1140 -->

## 18   Enhanced Mobile Storage Interface (eMSI)

The eMSI is a high performance, highly configurable and programmable, eMMC (embedded Multi-Media Card) and SD (secure digital) host controller. The eMSI is required to communicate with SD cards and eMMC devices targeted for the mobile/portable markets. The device memory to which the eMSI sends or receives data is typically a device for NAND flash storage. The eMSI is optimized for portable electronic devices and high-speed storage applications (for example embedded applications).

## eMSI Features

The eMSI includes the following features.

## General Features

- Supports eMMC protocols including JEDEC eMMC 5.1 specification
- Supported eMMC modes
- Legacy (0-26 MHz SDR - 1-bit, 4-bit, 8-bit support)
- HS SDR (0-50 MHz SDR -1-bit, 4-bit, 8-bit support)
- HS DDR (0-50 MHz DDR - 4-bit, 8-bit support)
- Supports boot operation and alternative boot operation
- Supports SD memory (SDSC, SDHC, SDXC) and wake up interrupt for SD cards
- Supported SD modes
- Default Speed/DS (0-25 MHz SDR - 1-bit, 4-bit support)
- High Speed/HS (0-44.4 MHz SDR - 1-bit, 4-bit support)
- Supports an internal DMA with the following data transfer types for eMMC modes:
- SDMA (Single operation DMA)
- ADMA2 (Advance DMA 2)

- ADMA3 (Advance DMA 3)
- Maximum block size of 1024 bytes supported
- FIFO size is 2048 bytes (parity enabled)
- Supports Command Queuing Engine (CQE) and compliant with eMMC CQ HCI
- Programmable scheduler algorithm selection of task execution. Supports data prefetch for back-to-back write operations
- 10 MHz CQE timer clock

## Supported Standards Compliance

- JEDEC eMMC 5.1 Specification - JESD84-B51, February 2015
- SD Specifications Part A2 SD Host Controller Standard Specification Version 4.20, August 2015
- SD Specifications Part A2 Host Controller Version 4.20 Supplementary Notes Version 1.00 Draft 0.60 December 2015
- SD Specifications Part 1 Physical layer Specification Version 6.00 February 2017

## eMSI Functional Description

The following sections describe the function of the eMSI controller.

## ADSP-SC59x EMSI Register List

The enhanced mobile storage interface (eMSI) is a high speed JEDEC eMMC 5.1 complaint controller. The eMSI controller can connect to single SD card or eMMC device at a time. A set of registers governs eMSI controller operations. For more information on eMSI functionality, see the eMSI controller Register Descriptions section.

Table 18-1: ADSP-SC59x EMSI Register List

| Name                 | Description                                        |
|----------------------|----------------------------------------------------|
| EMSI_ADMA_ADDR_LO    | ADMA System Address Register - Low                 |
| EMSI_ADMA_DESADDR_LO | ADMA3 Integrated Descriptor Address Register - Low |
| EMSI_ADMA_ERR_STAT   | ADMA Error Status Register                         |
| EMSI_ARG             | Argument Register                                  |
| EMSI_AUTOCMD_STAT    | Auto CMDStatus Register                            |
| EMSI_BASEADDR0       | Base Address 0 Register                            |
| EMSI_BASEADDR1       | Base Address 1 Register                            |
| EMSI_BLKCNT          | 16-bit Block Count Register                        |
| EMSI_BLKSZ           | Block Size Register                                |

Table 18-1: ADSP-SC59x EMSI Register List (Continued)

| Name                 | Description                                                |
|----------------------|------------------------------------------------------------|
| EMSI_BOOT_CTL        | eMMC Boot Control Register                                 |
| EMSI_CAP1            | Capabilities 1 Register - 0 to 31                          |
| EMSI_CAP2            | Capabilities Register - 32 to 63                           |
| EMSI_CLK_CTL         | Clock Control Register                                     |
| EMSI_CMD             | Command Register                                           |
| EMSI_CQVER           | Command Queuing Version Register                           |
| EMSI_CQ_CAP          | Command Queuing Capabilities Register                      |
| EMSI_CQ_CFG          | Command Queuing Configuration Register                     |
| EMSI_CQ_CRARG        | CQ Command Response Argument Register                      |
| EMSI_CQ_CRDCT        | Command Response for Direct Command Register               |
| EMSI_CQ_CRI          | CQ Command Response Index                                  |
| EMSI_CQ_CTL          | Command Queuing Control Register                           |
| EMSI_CQ_DPT          | Device Pending Tasks Register                              |
| EMSI_CQ_DQSTAT       | Device Queue Status Register                               |
| EMSI_CQ_IC           | Command Queuing Interrupt Coalescing Register              |
| EMSI_CQ_ISTAT        | Command Queuing Interrupt Status Register                  |
| EMSI_CQ_ISTAT_EN     | Command Queuing Interrupt Status Enable Register           |
| EMSI_CQ_ISTAT_INTEN  | Command Queuing Interrupt Signal Enable Register           |
| EMSI_CQ_RMEM         | Command Response Mode Error Mask Register                  |
| EMSI_CQ_SSCFG1       | CQ Send Status Configuration 1 Register                    |
| EMSI_CQ_SSCFG2       | CQ Send Status Configuration 2 Register                    |
| EMSI_CQ_TCLR         | Command Queuing Task Clear Register                        |
| EMSI_CQ_TCN          | Command Queuing Task Completion Notification Register      |
| EMSI_CQ_TDBR         | Command Queuing DoorBell Register                          |
| EMSI_CQ_TDL_BADDR    | Command Queuing Task Descriptor List Base Address Register |
| EMSI_CQ_TERRINFO     | CQ Task Error Information Register                         |
| EMSI_CTL1            | EMSI Control 1 Register                                    |
| EMSI_CTL2            | EMSI Control 2 Register                                    |
| EMSI_HOST_CNTRL_VERS | SD Host Controller Specification Version                   |
| EMSI_EMMC_CTL        | eMMC Control Register                                      |
| EMSI_ERR_STAT        | Error Interrupt Status Register                            |

Table 18-1: ADSP-SC59x EMSI Register List (Continued)

| Name                 | Description                                            |
|----------------------|--------------------------------------------------------|
| EMSI_ERR_STAT_EN     | Error Interrupt Status Enable Register                 |
| EMSI_ERR_STAT_INTEN  | Error Interrupt Signal Enable Register                 |
| EMSI_FRC_AUTOCMDSTAT | Force Event Register for Auto CMDError Status Register |
| EMSI_FRC_ERRSTAT     | Force Event Register for Error Interrupt Status        |
| EMSI_ISTAT           | Interrupt Status Register                              |
| EMSI_ISTAT_EN        | Interrupt Status Enable Register                       |
| EMSI_ISTAT_INTEN     | Interrupt Signal Enable Register                       |
| EMSI_PRESET_DS       | Preset Value for Default Speed                         |
| EMSI_PRESET_HSDDR    | Preset Value for HSDDR                                 |
| EMSI_PRESET_HSSDR    | Preset Value for HSSDR                                 |
| EMSI_PRESET_INIT     | Preset Value for Initialization                        |
| EMSI_PRESET_LEGACY   | Preset Value for Legacy Mode                           |
| EMSI_PSTATE          | Present State Register                                 |
| EMSI_RESP0           | Response Register 0                                    |
| EMSI_RESP1           | Response Register 1                                    |
| EMSI_RESP2           | Response Register 2                                    |
| EMSI_RESP3           | Response Register 3                                    |
| EMSI_SDMA_ADDR       | SDMA System Address Register                           |
| EMSI_SWRST           | Software Reset Register                                |
| EMSI_TO_CTL          | Timeout Control Register                               |
| EMSI_TRNSFRMODE      | Transfer Mode Register                                 |
| EMSI_VER_ID          | EMSI Version                                           |
| EMSI_VER_TYPE        | EMSI Version Type                                      |
| EMSI_WU_CTL          | Wakeup Control Register                                |

## ADSP-SC59x EMSI Interrupt List

Table 18-2: ADSP-SC59x EMSI Interrupt List

|   Interrupt ID | Name         | Description            | Sensitivity   | DMA Channel   |
|----------------|--------------|------------------------|---------------|---------------|
|            237 | EMSI0_STAT   | EMSI0 Status Interrupt | Level         |               |
|            239 | EMSI0_Wakeup | EMSI0 Wakeup Interrupt | Level         |               |

## ADSP-SC59x EMSI Trigger List

Table 18-3: ADSP-SC59x EMSI Trigger List Generators

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         |        |               |               |

Table 18-4: ADSP-SC59x EMSI Trigger List Receivers

| Trigger ID   | Name   | Description   | Sensitivity   |
|--------------|--------|---------------|---------------|
| None         |        |               |               |

## eMSI Block Diagram

The eMSI Block Diagram shows the main functional blocks within the eMSI. These blocks are described in more detail in eMSI Architectural Concepts.

Figure 18-1: eMSI Block Diagram

<!-- image -->

NOTE: The card-detect and write-protect signals are from the SD card socket and not from the SD card.

## eMSI Architectural Concepts

The following sections describe the functions and features of the eMSI controller as well as the eMMC and SD protocols.

The eMSI controller communicates with the device using a message-based bus protocol in which the eMSI controller sends commands serially using the EMSI\_CMD signal. Some commands require response from the eMMC device. This response is also sent serially on the EMSI\_CMD signal.

Data transfers, both to and from the eMMC device, occur using the data signals. The number of data lines used for the data transfer can be configured to 1 ( EMSI\_D0 ) bit, 4 ( EMSI\_D3 -EMSI\_D0 ) bit, or 8 ( EMSI\_D7 -EMSI\_D0 ) bit. All EMSI\_CMD and EMSI\_D[n] transfers are synchronous with EMSI\_CLK . Cyclic redundancy codes (CRC) are used to protect commands, responses, and data transfers from transmission errors. A CRC7 code is generated for every command sent by the eMSI and every response returned by the eMMC device on the EMSI\_CMD signal.

## Bus Interface Unit (BIU)

The Bus Interface Unit (BIU) implements the logic to transfer data to SCB (system crossbars). The SCB transfers data to and from the system memory through the SCB, respectively. This unit has logic to primarily access the eMSI controller registers by using an SCB and MMRG fabric. This module supports only the little-endian scheme for register accesses.

## DMA Engine

The DMA engine unit handles data transfer between controller and system memory. The key features of this unit are as follows.

- Supports SDMA/ADMA2/ADMA3 modes based on the configuration.
- Fetching the descriptor and data.
- Write back the received data packets to the system memory.

## eMSI Controller Registers

The eMSI controller register unit is comprised of the standard SD host controller registers as specified in the SD Specifications Part A2 SD Host Controller Standard Specification Version 4.20. It also includes command queuing registers that are compliant to the JEDEC eMMC 5.1 specification.

The host ensures the clock supply to the controller. The clock supply is required for accessing the registers in eMSI controller.

## FIFO Controller

The FIFO controller interfaces the packet buffer (FIFO) of the eMSI controller and the SD/eMMC protocol controller unit (SD/eMMC unit). The buffer (FIFO) depth is 2048 bytes. The FIFO stores data along with parity bits generated by the MEPU. The parity error is handled by the MEC.

## SD/eMMC Protocol Controller Unit

The SD/eMMC protocol controller unit is responsible for handling SD and eMMC interface protocols. The key features of this unit are:

- Generate SD/eMMC command and data packets
- Generate CRC and check for command and data packets
- Handle packet timeouts
- Support 1-bit, 4-bit, and 8-bit bus width for eMMC data transfer
- Support 1-bit and 4-bit bus width for SD card data transfer

## Command Queuing Engine (CQE)

The Command Queueing Engine (CQE) denotes the hardware unit executing the Command Queueing (CQ) activities. The CQE manages the interface between the host software and the eMMC device as well as the data transfers.

## Generating SD/eMMC Commands

The eMSI controller generates SD/eMMC commands and sends a command to the eMSI bus (this includes the internally generated CRC7). When initiating a transaction using SDMA (Single Operation DMA) generated transfers, ADMA (Advanced DMA) generated transfers, and non-data transfers, the host driver programs registers sequentially from EMSI\_SDMA\_ADDR to EMSI\_CMD . The beginning register may be chosen based on the type of transaction. Table 18-5 Register Programming Sequence Before Issuing Commands shows the register programming sequence needed before issuing the commands. The last written register is always EMSI\_CMD because writing to the upper byte of this register triggers the issuance of an SD/eMMC command.

The host driver should not read the SDMA system address, block size and block count registers during a data transaction unless the transfer is stopped because the value is changing and not stable.

To prevent an overwrite of registers while issuing data transfer based commands, the 32-bit block count ( EMSI\_SDMA\_ADDR ), block size ( EMSI\_BLKSZ ), 16-bit block count ( EMSI\_BLKCNT ) and transfer mode ( EMSI\_TRNSFRMODE ) registers is write protected by the eMSI controller, while command inhibit ( EMSI\_PSTATE.CMD\_INHIBIT\_DAT ) is set to 1. (When host version 4 enable = 0, the SDMA system address is not protected by this signal.) The host driver must not write the argument and command registers while EMSI\_PSTATE.CMD\_INHIBIT\_DAT is set to 1.

Table 18-5: Register Programming Sequence Before Issuing Commands

| Command Type                     | Register to program before issuing the commands in sequence                       |
|----------------------------------|-----------------------------------------------------------------------------------|
| Non Data transfer based commands | EMSI_ARG , EMSI_CMD                                                               |
| Data transfer based commands     | EMSI_SDMA_ADDR , EMSI_BLKSZ , EMSI_BLKCNT , EMSI_ARG , EMSI_TRNSFRMODE , EMSI_CMD |

## Block Count

The block count parameter used by the eMSI controller to determine the total data length of the data transfer. It does this by multiplying the block length (block size). The data transfer length set to the SD/eMMC data transfer must be equivalent to data transfer length sets to ADMA2 and ADMA3 descriptors (in predefined data transfers).

## Selection of 16-Bit or 32-bit Block Count

Due to the increased capacity of eMMC devices, the data that is available to transfer to and from these devices has also increased. To allow for this increase the eMSI controller extends block counts from 16-bit to 32-bit for all operations in SD/eMMC mode, SDMA and ADMA.

Because SDMA may use the ADMA system address register ( EMSI\_ADMA\_ADDR\_LO ) to support 32-bit addressing, the SDMA system address register ( EMSI\_SDMA\_ADDR ) is re-assigned to the 32-bit block count register. Selection of the block count registers is either 16-bit ( EMSI\_BLKCNT ) or 32-bit block count ( EMSI\_SDMA\_ADDR ) and is defined as follows, which allows mixed use of 16-bit or 32-bit block count.

- If the Host Version 4 Enable bit ( EMSI\_CTL2.HOST\_VER4\_EN ) is cleared to 0 or the 16-bit EMSI\_BLKCNT register is configured to non-zero, the 16-bit EMSI\_BLKCNT register is selected.
- If Host Version 4 Enable is set to 1 and 16-bit EMSI\_BLKCNT register is configured to 0000h, the 32-bit block count register is selected.

Use of block count is enabled by setting the EMSI\_TRNSFRMODE.BLOCK\_COUNT\_EN bit during data transfers.

## Preset Value Registers

As the operating eMSI bus clock frequency depends on the system implementation, it is difficult to determine the frequency parameters in using the standard host driver. When the preset value enable is set using the EMSI\_CTL2.PRESET\_VAL\_EN bit, automatic eMSI bus clock frequency generation is performed without considering frequency dividers set by the host driver ( EMSI\_CLK\_CTL.FREQ\_SEL / EMSI\_CLK\_CTL.UPPER\_FREQ\_SEL ). The eMSI controller automatically selects the eMSI bus clock frequency as specified in the preset value (as per frequency dividers values set in preset value registers) depending on the selected bus speed mode ( EMSI\_CTL2.EMMC\_MODE\_SEL ). For example, if the program selects the eMMC legacy speed mode then the eMSI controller chooses dividers from the EMSI\_PRESET\_LEGACY.FREQ\_SEL bit field and the same dividers are set in the EMSI\_CLK\_CTL.FREQ\_SEL bit field. If the bit is cleared to 0, the eMSI bus clock frequency is selected by the host driver frequency dividers that are set by the host driver using the EMSI\_CLK\_CTL.FREQ\_SEL / EMSI\_CLK\_CTL.UPPER\_FREQ\_SEL bit field.

A preset value for initialization ( EMSI\_PRESET\_INIT ) is not selected by the bus speed mode. Before starting the initialization sequence, the host driver needs to set a clock preset value to EMSI\_CLK\_CTL.FREQ\_SEL / EMSI\_CLK\_CTL.UPPER\_FREQ\_SEL by reading the contents of the EMSI\_PRESET\_INIT register. Preset value enable ( EMSI\_CTL2.PRESET\_VAL\_EN ) can be set after initialization is completed.

## Automatic Command Generation (Auto CMD)

The eMSI controller internally generates Auto CMD12 (STOP\_TRANSMISSION) or Auto CMD23 (SET\_BLOCK\_COUNT) commands as per configuration in the EMSI\_TRNSFRMODE.AUTO\_CMD\_EN bit field. Automatic commands generation improve the performance as it eliminates the process of manually writing registers.

## Auto CMD23

Auto CMD23 feature is useful in predefined multiple block operations. A block count is configured in the argument of CMD23 to specify a transfer length of following CMD18 (READ\_MULTIPLE\_BLOCK) or CMD25 (WRITE\_MULTIPLE\_BLOCK) commands. Auto CMD23 is a feature that automatically issues a CMD23 before a CMD18 or CMD25 is sent to the SD cards/eMMC device (when EMSI\_TRNSFRMODE.AUTO\_CMD\_EN = 0x2). This avoids performance deterioration during memory accesses by removing the interrupt service of CMD23. The EMSI\_SDMA\_ADDR register is assigned to the 32-bit block count register for CMD23.

NOTE: CMD23 is supported in version 3.00 and above SD cards.

## Auto CMD12

Open ended multiple block transfers for SD cards/eMMC require CMD12 (STOP\_TRANSMISSION) command to stop the data transactions. The eMSI controller automatically issues CMD12 when the last block transfer is completed. The host driver can enable this command using the EMSI\_TRNSFRMODE.AUTO\_CMD\_EN bit = 0x1 when issuing multiple block transfer (CMD18 and CMD25) commands. Auto CMD12 timing synchronization with the last data block is accomplished by hardware in the eMSI controller. Auto CMD12 is useful when block count is finite(&gt;0, block count is enabled) and CMD23 is not used (for example in open ended transfers).

## Data Transfer Modes

The eMSI controller supports data transfer using Direct Memory Access (DMA). There are three types pf DMA operations available: SDMA, ADMA2 and ADMA3.

## Direct Memory Access (DMA)

The Direct Memory Access (DMA) transfers data from the host memory to the device port and conversely. The controller uses a descriptor (except in SDMA transfers) to move data efficiently from source to destination with minimal core intervention. Programs can configure the controller to interrupt the core in situations such as data transmit and receive transfer completion from the eMMC device, as well as other normal or error conditions. The DMA transfers the data received from the eMMC device to the data buffer in the host memory, and it transfers transmit data from the data buffer in the host memory to the eMSI FIFO. Descriptors that reside in the memory act as pointers to these buffers. DMA supports both single block and multi-block transfers.

## Non Descriptor (Single Operation) DMA

SDMA (Single Operation DMA) performs a single read/write SD/eMMC command operation at a time. SDMA is suitable for short data transfers because SDMA requires an address update at the page boundary of system memory. A DMA interrupt generated at every page boundary uses core resources to reprogram the new system address. For short data transfers, writing a register is preferred to creating descriptors.

SDMA supports various size page boundaries (from 4 kB to 512 kB, configured using the EMSI\_BLKSZ.SDMA\_BUF\_BDARY bit field). A page boundary is the size of a contiguous buffer in system memory. The SDMA transfer waits at every boundary specified by the EMSI\_BLKSZ.SDMA\_BUF\_BDARY bit field

and the eMSI controller generates the DMA interrupt to request the host driver to update the SDMA system address register.

## Descriptor (Advanced) DMA

A long data transfer should use ADMA (Advanced DMA) to avoid performance bottlenecks caused when an interrupt occurs at every page boundary. ADMA2 and ADMA3 adopt the scatter gather DMA algorithm so that higher data transfer speeds are available. The host driver can program a list of data transfers between system memory and an SD card eMMC using the descriptor table. ADMA2 performs one read/write command operation at a time. ADMA3 can program multiple read/write operations in a descriptor table. ADMA3 is used to perform large data transfers.

## Advanced DMA 2 (ADMA2)

Figure 18-2 ADAM2 Block Diagram, shows the ADMA2 data transfer between the eMSI controller and the system memory as well as the descriptor table that is created by the host driver (for more information, see ADMA2 Descriptor Format). A 32-bit address descriptor table is used for a system with 32-bit addressing. In the descriptor table, each line contains information about the address, its length (descriptor data transfer size) and an attributes that specifies the operation of the descriptor line. ADMA2 uses the 32-bit ADMA address register ( EMSI\_ADMA\_ADDR\_LO ) for the descriptor pointer. An ADMA2 transfer is triggered by writing to the command register and ADMA2 processes each line in the descriptor table. The descriptor table is created in the system memory by the host driver. ADMA2 fetches one descriptor line and executes it until the end of descriptor denoted by (End = 1 in attribute) is found.

Figure 18-2: ADAM2 Block Diagram

<!-- image -->

Figure 18-3 ADMA2 Data Transfer Example shows a typical ADMA2 descriptor program. The host driver describes the descriptor table with each slice is placed somewhere in contiguous system memory. The host driver describes the descriptor table that contain the address, length, and attributes. Each sliced data is transferred in turn as programmed in the descriptor.

Figure 18-3: ADMA2 Data Transfer Example

<!-- image -->

## Data Address and Data Length Requirements

There are three requirements to program the descriptor:

1. The minimum unit of address is 4 bytes (32-bits).
2. The maximum data length of each descriptor line is less than 64 KB (if data length set to 10-bit) or 64 MB (if data length set to 26-bit).
3. Total Length = Length 1 + Length 2 + Length 3 + ... + Length n = multiple of block size

Using 4 byte units of address simplifies byte enable control on the 32-bit data bus. If the total length of a descriptor were not a multiple of block size, ADMA2 transfers might not be terminated. In this case, a data timeout occurs and the transfer is stopped by an abort command. Therefore the total length should be a multiple of block size.

It is recommended that the total of the data lengths of all descriptors be equal to the total data transfer size where: total data transfer size = block count x block size.

## General Descriptor Table Format

Figure 18-4 General Descriptor Table Format shows the general format of a descriptor table. One descriptor line consumes 64 bits (8 bytes) using 32-bit addressing mode. The attribute is used to control the descriptor. Act0 = 0 is assigned to ADMA2 descriptor. Act0 = 1 is assigned to the ADMA3 descriptor and is used to extend descriptors for ADMA3. Refer to Advanced DMA 3 (ADMA3) for more information.

Figure 18-4: General Descriptor Table Format

<!-- image -->

## ADMA2 Descriptor Format

Figure 18-5 ADMA2 Descriptor Table Format shows the ADMA2 descriptor table. Act0 = 0 is assigned to the ADMA2 descriptor. Three action symbols are specified by the combination of Act2 and Act1. A no operation (NOP) skips the current descriptor line and fetches the next one. A T ran operation transfers data designated by address and length field. A Link operation is used to connect separated two descriptors. The address field of link points to next descriptor table. The combination of Act2 = 0 and Act1 = 1 is reserved and defined in the same operation as a NOP . All other combinations are the same as No Operation (NOP).

## 64-BIT ADMA2 LINE (32-BIT ADDRESSING MODE)

| 32-BIT ADDRESS   | 16-BIT LENGTH   | 10-BIT LENGTH   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   |
|------------------|-----------------|-----------------|-------------|-------------|-------------|-------------|-------------|-------------|
| 63 32            | 31 16           | 15 06           | 05          | 04          | 03          | 02          | 01          | 00          |
| XXXX_XXXXh       | XXXXh           | XXXXXXXXXXb     | ACT2        | ACT1        | ACT0        | INT         | END         | VALID       |

|   ACT2 |   ACT1 |   ACT0 | SYMBOL   | COMMENT         | OPERATION                                                  |
|--------|--------|--------|----------|-----------------|------------------------------------------------------------|
|      0 |      0 |      0 | NOP      | NO OPERATION    | DO NOT EXECUTE CURRENT LINE AND GO TO NEXT LINE            |
|      0 |      1 |      0 | RSV      | RESERVED        | SAME AS NOP. NOT EXECUTE CURRENT LINE AND GO TO NEXT LINE. |
|      1 |      0 |      0 | TRAN     | TRANSFER DATA   | TRANSFER DATA OF 1 DESCRIPTOR LINE                         |
|      1 |      1 |      0 | LINK     | LINK DESCRIPTOR | LINK TO ANOTHER DESCRIPTOR                                 |

Figure 18-5: ADMA2 Descriptor Table Format

<!-- image -->

The Int attribute may be set only in an ADMA2 descriptor. The 26-bit data length mode is added to reduce the number of descriptor lines for large continuous data. The following tables shows the definition of 16-bit data length and 26-bit data length. 16-bit data length is selected when the EMSI\_CTL2.ADMA2\_LEN\_MODE bit is cleared to 0. 26-bit data length is selected when the EMSI\_CTL2.ADMA2\_LEN\_MODE bit is set to 1.

Table 18-6: ADMA2 16-bit Length Mode

| 16-bit Length (D31-D16)   | Value of Length   |
|---------------------------|-------------------|
| 000h                      | 65536 bytes       |
| 0001h                     | 1 byte            |
| 0002h                     | 2 bytes           |
| ...                       | ...               |
| FFFFh                     | 65535 bytes       |

Table 18-7: ADMA2 26-bit Length Mode

| 26-bit Length (D15-D06, D31-D16)   | Value of Length   |
|------------------------------------|-------------------|
| 000_0000h                          | 64M bytes         |
| 000_0001h                          | 1 byte            |
| 000_0002h                          | 2 bytes           |
| 000_0003h                          | 3 bytes           |
| ...                                | ...               |
| 3FF_FFFFh                          | 64M-1 bytes       |

## ADMA2 Error Status Register

An error that occurs during an ADMA2 transfer may stop ADMA2 operation and generate an ADMA error interrupt. The EMSI\_ADMA\_ERR\_STAT.ERR\_STATES bit field holds state of ADMA2 stopped.

## Advanced DMA 3 (ADMA3)

As described above for ADMA2, SD/eMMC commands are issued when the host driver writes to the eMSI controller registers. For ADMA3, a host program performs multiple ADMA2 operations using command descriptors to issue SD/eMMC commands. A multi-block data transfer between system memory and an SD card/eMMC is programmed by using a pair of command descriptors and an ADMA2 descriptor. ADMA3 performs multiple multiblock data transfers by using an integrated descriptor. Figure 18-6 Example of ADMA3 Operation shows an example ADMA3 operation where three data blocks (data A, data B and data C) are written to different areas of the SD card/eMMC.

An integrated descriptor consists of pointers to command descriptors. Each command descriptor is followed by an ADMA2 descriptor. The first descriptor pair is programed to transfer data A, the second pair data B and the third pair data C. The location of the integrated descriptor is configured using the ADMA3 integrated descriptor address register ( EMSI\_ADMA\_DESADDR\_LO ). ADMA3 fetches pointers one by one in the integrated descriptor and executes descriptors designated by the pointer. ADMA3 sets the contents of a command descriptor in the eMSI controller registers ( EMSI\_SDMA\_ADDR to EMSI\_CMD sequentially) to issue an SD/eMMC command and then executes the ADMA2 descriptor. The first operation transfers data A from system memory to the SD card/eMMC. The second operation transfers data B and the third operation transfers data C. When execution of all descriptors pointed by the integrated descriptor is completed, ADMA3 generates a transfer complete interrupt to inform the host driver that the operation is complete.

Figure 18-6: Example of ADMA3 Operation

<!-- image -->

## Command Descriptor Format

As shown in Figure 18-7 Command Descriptor Format, 32-bit register data is configured in bits 63-32 of each descriptor line. Command descriptor types are distinguished by Attribute. If Attribute indicates the command descriptor for SD/eMMC mode (Act2-0 = 001b), the 32-bit register fields are written to the eMSI controller registers sequentially from the EMSI\_SDMA\_ADDR to EMSI\_CMD registers. When the EMSI\_CMD register is written, an SD/eMMC command is issued. The eMSI controller has a pointer to a descriptor line for the command descriptor and the ADMA2 descriptor. The pointer is incremented after reading every descriptor line. When the last line of the command descriptor is read, the pointer is assumed to point to the top of the ADMA2 descriptor, which is placed just after the command descriptor. The eMSI controller ignores the INT of Attribute in this descriptor.

## COMMAND DESCRIPTOR (FOR 32-BIT ADDRESSING MODE)

| 32-BIT REGISTER   | RESERVED   | RESERVED   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   | ATTRIBUTE   |
|-------------------|------------|------------|-------------|-------------|-------------|-------------|-------------|-------------|
| 63 32             | 31 16      | 15 06      | 05          | 04          | 03          | 02          | 01          | 00          |
| 0xXXXX_XXXX       | 0x0000     | 0000000b   | ACT2        | ACT1        | ACT0        | INT         | END         | VALID       |

32-BIT REGISTER FIELDS ARE WRITTEN TO THE eMSI CONTROLLER REGISTERS FROM 0x000 to 0x00F (EMSI\_SDMA\_ADDR to EMSI\_CMD) TO ISSUE AN SD/eMMC COMMAND

## COMMAND DESCRIPTOR FOR SD/eMMC MODE

Figure 18-7: Command Descriptor Format

| 32-BIT REGISTER                 | RESERVED   | ATTRIBUTE   |                                                                 |
|---------------------------------|------------|-------------|-----------------------------------------------------------------|
| 32-BIT Block Count              | ALL 0      | 001001b     | SET TO 0x003 - 0x000 (EMSI_SDMA_ADDR Register)                  |
| 16-BIT BLOCK COUNT + BLOCK SIZE | ALL 0      | 001001b     | SET TO 0x007 - 0x004 (EMSI_BLKCNT and EMSI_BLKSZ Register)      |
| ARGUMENT                        | ALL 0      | 001001b     | SET TO 0x00B - 0x008 (EMSI_ARG Register)                        |
| COMMAND + TRANSFER MODE         | ALL 0      | 001011b     | SET TO 0x00F - 0x00C (EMSI_CMD and EMSI_TRANSFERMODE Registers) |

The programing requirements of command descriptors for SD/eMMC mode are:

- Setting infinite data transfer ( EMSI\_TRNSFRMODE.BLOCK\_COUNT\_EN = 0) is not allowed, except for single block transfers ( EMSI\_TRNSFRMODE.MULTI\_BLK\_SEL = 0).
- As ADMA3 only supports 32-bit block count mode, the EMSI\_TRNSFRMODE.BLOCK\_COUNT\_EN bit must be set to 1 and the 16-bit block count must be set to 0x0000 ( EMSI\_BLKCNT ). Clearing stop count (32-bit block count = 0) EMSI\_SDMA\_ADDR is not allowed, except for single block transfers ( EMSI\_TRNSFRMODE.MULTI\_BLK\_SEL = 0).
- Configuring no data transfer (block size = 0) is not allowed, except when using the no data transfer command in a command descriptor (where CDEnd set to 1).
- When using the memory data transfer command, The Auto CMD Auto Select of Auto CMD Enable ( EMSI\_TRNSFRMODE.AUTO\_CMD\_EN ) must be used so that Auto CMD 23 is sent to the eMMC device if EMSI\_CTL2.CMD23\_EN = 1. For Auto CMD 12, Auto Command Disabled ( EMSI\_TRNSFRMODE.AUTO\_CMD\_EN = 0) is used.

## Integrated Descriptor Format

Figure 18-8 Integrated Descriptor Format shows how multiple of pointers to command descriptors are configured in the integrated descriptor. The pointer of a 32-bit address is configured to bit 63-32 in the 64-bit integrated descriptor. The eMSI controller ignores the INT of Attribute in this descriptor.

## INTEGRATED DESCRIPTOR (32-BIT ADDRESSING)

Figure 18-8: Integrated Descriptor Format

<!-- image -->

The ADMA3 integrated descriptor address register ( EMSI\_ADMA\_DESADDR\_LO ) is used to designate the location of the integrated descriptor. The start of an ADMA3 is triggered by writing to offset EMSI\_ADMA\_DESADDR\_LO in 32-bit addressing mode.

## Response Error Check During ADMA3 Operation

The eMSI controller should perform response checks to prevent performance loss due to the host driver doing this task. When creating the command descriptor, configure the following 3 bits in the transfer mode register to: EMSI\_TRNSFRMODE.RESP\_INT\_DIS = 1, EMSI\_TRNSFRMODE.RESP\_ERR\_CHK\_EN = 1. Clear EMSI\_TRNSFRMODE.RESP\_TYPE = 0 if the command is for an SD/eMMC enabled response check by the eMSI controller.

In SD/eMMC mode, if an error is detected in R1, a response error interrupt is generated in the EMSI\_ERR\_STAT register. ADMA3 is stopped and the host driver can read the error response from the EMSI\_RESP0 register.

## Command Queueing Engine

The Command Queueing Engine (CQE) denotes the hardware unit executing the Command Queueing (CQ) activities. The CQE manages the interface between the host software and the eMMC device, and the data transfers. The CQE receives tasks from the software though a Task Descriptor List (TDL) in the host memory and the doorbell register. The CQE issues CQ commands, CMD44 (QUEUED\_TASK\_PARAMS) and CMD45 (QUEUED\_TASK\_ADDRESS), to the eMMC device and stores the task information. The CQE also reads the queue status register of the device, decides the task to execute and issues the EXECUTE commands, CMD46 (EXE-

CUTE\_READ\_TASK) and CMD47 (EXECUTE\_WRITE\_TASK).

The Command Queuing Engine has the following features:

- Non-blocking issuance of data transfer tasks using a task list in host memory
- Optional notification upon completion via interrupt, with interrupt coalescing

- Direct-Command (DCMD) tasks through which software can send any eMMC command, by its index and argument, to the device, using CQE
- Queue-Barrier (QBR) where software can control the execution order of tasks, and task management and error recovery executed by software, by halting the CQE at known locations

To implement queuing of tasks in the eMMC device, the host asynchronously issues tasks using the TDL, which is in a memory location known to the CQE and contains up to 32 fixed-size slots. Each slot contains one task descriptor and one transfer descriptor. To issue a task, the software selects an available TDL slot and constructs the following two descriptors in it.

- Task Descriptor - encodes all information that defines the task. For instance, the start address, block count, and priority. This information is later passed to the device. The task descriptor can alternatively encode any arbitrary command that is sent directly to the device.
- Transfer Descriptor - points either to one continuous data buffer to/from which data is transferred (TRAN Descriptor) or to a scatter/gather list of any length (LINK descriptor). Figure 18-9 Command Queuing Host Controller Interface (HCI) Structure shows the structure of the TDL in the host memory and the slot numbers in the TDL. The slot numbers in the figure store the following information:
- 0 - Stores a data transfer task with a TRAN descriptor
- 1 - Stores a data transfer task with a LINK descriptor, pointing to a scatter/gather list
- 31 - Stores a DCMD descriptor

Figure 18-9: Command Queuing Host Controller Interface (HCI) Structure

<!-- image -->

## Task Processing

Once the two descriptors are written in slot i, software issues the task by writing 1 to bit i in the EMSI\_CQ\_TDBR register. This action signals to the CQE to process the task. Software must not write the EMSI\_CQ\_TDBR register

to issue the task before valid descriptors are ready in the appropriate slot. The CQE then reads the descriptors. Based on the information encoded in the task descriptor, the CQE generates QUEUED\_TASK\_PARAMS (CMD44) and QUEUED\_TASK\_ADDRESS (CMD45), and sends them to the device.

## Task Selection and Execution

The CQE is also in charge of reading the device's queue status register (QSR) to determine which tasks are ready for execution, selecting tasks for execution, and ordering their execution. These actions are described in this subsection.

Reading the QSR: When task(s) are queued in the device, the CQE reads the QSR to determine which tasks are ready for execution. The QSR is read using SEND\_ QUEUE\_STATUS (CMD13) commands. If a data transfer is ongoing, the CQE sends the SQS towards the end of the data transfer, as configured in the EMSI\_CQ\_SSCFG1.SQSCMD\_BLK\_CNT register field. If the bus is idle, the CQE periodically sends SQS commands, as configured in the EMSI\_CQ\_SSCFG1.SQSCMD\_IDLE\_TMR register field.

Selecting a Task: When one or more data transfer tasks are ready for execution and the bus is idle (or a previous data transfer is about to end), CQE is expected to select a task for execution. The selected task must be marked as ready for execution by the device.

Ordering Task Execution: When the bus is idle, the CQE sends EXECUTE\_READ\_TASK (CMD46) or EXE-CUTE\_WRITE\_TASK (CMD47), ordering the device to execute a task, identified by its task ID.

The CQE also feeds the task's transfer descriptor(s) to a DMA engine as a pointer to the data buffer in the host memory. A task's execution is considered completed once the data transfer phase is completed.

## Task Completion: Interrupts and Interrupt Coalescing

Upon the completion of a data transfer task, after the data has been transferred, an interrupt may be generated, if requested. If the INT bit in the task descriptor is set, a task complete (TCC) interrupt is generated. If the INT bit is not set, the transfer is counted towards coalescing. DCMD tasks are not counted towards coalescing. The interrupt coalescing mechanism allows the host to moderate its interrupt load. The mechanism is also timer-protected to avoid high latency in low-load cases.

## Direct Command (DCMD) Submission

When CQ is enabled, software still retains the ability to efficiently issue eMMC commands by their index and argument. To use the DCMD feature, it must be enabled where EMSI\_CQ\_CFG.DCMD\_EN = 1. To issue a direct command, the software writes a DCMD task descriptor to slot #31 of the TDL and rings the doorbell register ( EMSI\_CQ\_TDBR ). The command's index and argument are encoded directly in the DCMD task descriptor (in the fields otherwise used for address and block count).

The DCMD feature is targeted for the transmission of non-data commands which are allowed when CQ mode is enabled, such as CMD0, CMD12, or CMD13.

When processing the task descriptor, the CQE constructs the command by its index and argument, sends it to the device, and stores the response in a dedicated register.

The DCMD task descriptor is identified by the CQE by its placement in slot #31 of the TDL. Therefore, it may only be issued using slot #31, and may only be sent one at a time. The transfer descriptor following the DCMD descriptor is ignored by CQE.

## Queue-Barrier (QBR) Tasks

To enable a host's control on the ordering between tasks, a task can be marked as a Queue-Barrier (QBR) task by setting the QBR bit in the task descriptor. The following are guaranteed by the eMSI controller:

- A QBR task is only sent to the device after all the tasks issued before it have been executed (when the task queue in the device is empty).
- New tasks (tasks issued after the QBR task) are sent to the device after the QBR task execution is completed.

Specifically, the following rules apply:

- When the task marked with QBR is a data transfer task, the CQE may send the related QUEUED\_ TASK\_PARAMS (CMD44) and QUEUED\_ TASK\_ADDRESS (CMD45) commands to the device only after receiving the response for the EXECUTE\_READ\_TASK(CMD46)/ EXECUTE\_WRITE\_TASK(CMD47) for the last data transfer task in the device's queue (data transfer may still be on-going).
- When the task marked with QBR is a DCMD task, the CQE can send the related command only after all previously issued tasks have completed, including the associated data transfers.
- Commands related to tasks which are queued after the QBR task, may only be sent after the QBR task execution is completed.

After the CQE has halted, the device waits for the next command, so it is effectively halted as well. The CQE notifies software that it is halted by setting the EMSI\_CQ\_CTL.HALT bit and, optionally by an interrupt ( EMSI\_CQ\_ISTAT.HAC ). It is guaranteed that neither the device nor the host hardware (the CQE) will initiate any operation while the CQE is in the halt state. When software wants the CQE to resume operation, it clears the EMSI\_CQ\_CTL.HALT bit = 0. When 0 is written, the CQE continues its normal operation, based on the current state. In some error conditions, the halt bit may never actually set, because a response has not been received from the device. In these cases, software may assume the CQE is halted after waiting for a long enough time.

## eMMC Command Queuing Data Structures

This section describes the descriptor structures used in eMMC Command Queuing. As mentioned in the previous section, the host writes a pair of descriptors-task descriptor and transfer descriptor, which are organized in the host memory in the task descriptor list. This section discusses the following structures:

- Task Descriptor Structure
- Transfer Descriptor Structure
- Task Descriptor for Direct-Command (DCMD) Tasks

## Task Descriptor Structure

Table 18-8 Task Descriptor Structure; Lower 64 bits (Data Transfer Tasks) and Table 18-9 Task Descriptor Structure Upper 64 bits summarize the task descriptor structure for lower and upper 64 bits, and Table 18-10 Task Descriptor Fields describes task descriptor fields.

Table 18-8: Task Descriptor Structure; Lower 64 bits (Data Transfer Tasks)

| Block Address   | Block Count   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Attribute   | Attribute   | Attribute   | Attribute   |
|-----------------|---------------|-------------------------|-------------------------|-------------------------|-------------------------|-------------------------|-------------------------|-------------------------|-------------|-------------|-------------|-------------|
| 63:32           | 31:16         | 15                      | 14                      | 13                      | 12                      | 11                      | 10:7                    | 6                       | 5:3         | 2           | 1           | 0           |
| xxxx_xxx xh     | xxxxh         | Reliable Write          | QBR                     | Priority                | Data Di- rection        | Tag re- quest           | Context                 | Forced progra mming     | Act=101     | Int         | End= 1      | Valid =1    |

Table 18-9: Task Descriptor Structure Upper 64 bits

| Reserved   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Reserved   |
|------------|-------------------------|-------------------------|-------------------------|-------------------------|------------|
| 127:112    | 111                     | 110:106                 | 105:104                 | 103:96                  | 96:64      |
| 0x0000     | 0b                      | 00000b                  | Transaction Type        | 0x00                    | X          |

Table 18-10: Task Descriptor Fields

| Field Name                | Bit Location   | Description                                                                                                                                                                                                                                               | Encoded in CMD   |
|---------------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|
| Valid                     | 0              | 1 - The descriptor is effective and must be processed by hardware. 0 - The descriptor line must not be used.                                                                                                                                              |                  |
| End                       | 1              | Must always be set to 1. Every Task Descriptor is standalone.                                                                                                                                                                                             |                  |
| Int                       | 2              | Indicates the interrupt generation policy required for this task. 1 - Hardware generates an interrupt upon the task completion. 0 - Hardware counts task completion for interrupt coalescing.                                                             |                  |
| Act                       | 5:3            | Must be configured to b101 to indicate that this is a Task Descriptor.                                                                                                                                                                                    |                  |
| Forced Program- ming (FP) | 6              | When 1, FP is enabled. Data is forcefully programmed to a non-volatile storage instead of volatile cache while cache is turned ON.                                                                                                                        | CMD44            |
| Context ID                | 10:7           | A context is an active session, configured for a specific read/write pattern. A device may support one or more concurrent contexts, defined by a Context ID. Each context ID (besides #0) has a configuration field in EXT_CSD to control its be- havior. | CMD44            |
| Tag Request               | 11             | Indicates request to receive information (about specific data types) from the host.                                                                                                                                                                       | CMD44            |
| Data Direction            | 12             | Indicates the direction of data transfer. 1 - Device to Host (Read) 0 - Host to Device(Write)                                                                                                                                                             | CMD44            |

Table 18-10: Task Descriptor Fields (Continued)

| Field Name          | Bit Location   | Description                                                                                                                                                                                                                                       | Encoded in CMD   |
|---------------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------|
| Priority            | 13             | 1 - high 0 - simple                                                                                                                                                                                                                               | CMD44            |
| Queue Barrier (QBR) | 14             | Indicates the control of the host on the ordering between tasks.                                                                                                                                                                                  |                  |
| Reliable Write      | 15             | Indicates multiple block write with pre-defined block count and Reliable Write parameters.                                                                                                                                                        | CMD44            |
| Block Count         | 31:16          | Number of blocks to be read/written.                                                                                                                                                                                                              | CMD44            |
| Block Address       | 63:32          | Data block address                                                                                                                                                                                                                                | CMD45            |
| Reserved            | 95:64          |                                                                                                                                                                                                                                                   | N/A              |
| Reserved            | 103:96         |                                                                                                                                                                                                                                                   | N/A              |
| Transaction Type    | 105:104        | Transaction Type (TT) Field value description: 00 - Simple Data Task 01 - Data Task w/ Immediate Partition Access 10 - Device Management Sequence (DMS) 11 - reserved NOTE: When 64 b descriptors are used, the implied value of this field is 0b | CMD44            |
| Reserved            | 127:106        |                                                                                                                                                                                                                                                   | N/A              |

## Transfer Descriptor Structure

Table 18-11 Transfer Descriptor Structure (32-Bit Addressing) summarizes the transfer descriptor structure for 32bit addressing. Table 18-12 T ransfer Descriptor Fields describes transfer descriptor fields.

Table 18-11: Transfer Descriptor Structure (32-Bit Addressing)

| Address     | Length   | Reserved   | Attribute   | Attribute   | Attribute   | Attribute   |
|-------------|----------|------------|-------------|-------------|-------------|-------------|
| 63:32       | 31:16    | 15:6       | 5:3         | 2           | 1           | 0           |
| 0xXXXX_XXXX | xxxxh    | 0000000000 | Act         | Int         | End         | Valid       |

Table 18-12: Transfer Descriptor Fields

| Field Name   |   Bit Location | Description                                                                                                                                                                      |
|--------------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Valid        |             10 | 1 - The descriptor is effective and must be processed by hardware. 0 - The de- scriptor line must not be used.                                                                   |
| End          |              1 | 1 - The descriptor is the last descriptor in a descriptor list. 0 - Additional descriptors follow this descriptor. In the case of a TRAN descriptor, the value of this bit is 1. |

Table 18-12: Transfer Descriptor Fields (Continued)

| Field Name        | Bit Location   | Description                                                                                                                                                                       |
|-------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Int               | 2              | The value of this bit is cleared to 0 in command queuing.                                                                                                                         |
| Act               | 5:3            | 100 - TRAN - Address field of descriptor points to a data buffer. 110 - LINK - Address field of descriptor points to another descriptor. 000 - NOP no operation. Others: Reserved |
| Reserved          | 15:6           | Reserved                                                                                                                                                                          |
| Length            | 31:16          | Length of data buffer in bytes. A value of 0000 means 64 kB.                                                                                                                      |
| Address (32- bit) | 63:32          | Data buffer address in host memory, in 32-bit addressing mode Address must be configured for 32-bit boundary (lower 2 bits cleared to 0)                                          |

## Task Descriptor for Direct-Command (DCMD) Tasks

Table 18-13 Task Descriptor Structure summarizes the transfer descriptor structure for the DCMD tasks and Table 18-14 Transfer Descriptor Fields describes transfer descriptor fields.

Table 18-13: Task Descriptor Structure

| Com- mand Ar- gument   | Command Parameters   | Command Parameters   | Command Parameters   | Command Parameters   | Task Parameters Field   | Task Parameters Field   | Task Parameters Field   | Attribute   | Attribute   | Attribute   | Attribute   |
|------------------------|----------------------|----------------------|----------------------|----------------------|-------------------------|-------------------------|-------------------------|-------------|-------------|-------------|-------------|
| 63:32                  | 31:25                | 24:23                | 22                   | 21:16                | 15                      | 14                      | 13:6                    | 5:3         | 2           | 1           | 0           |
| xxxx_xxx xh            | Reserved (0000000 )  | Response Type        | CMD timing           | CMDIn- dex           | Rsvd (0)                | QBR                     | Reserved (0000000 )     | Act=101     | Int         | End=1       | Valid =1    |

Table 18-14: Transfer Descriptor Fields

| Field Name          | Bit Location   | Description                                                                                                                                                                                                                                       |
|---------------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Valid               | 0              | 1 - The descriptor is effective and must be processed by hardware. 0 - The de- scriptor line must not be used.                                                                                                                                    |
| End                 | 1              | Must always be set to 1. Every Task Descriptor is standalone.                                                                                                                                                                                     |
| Int                 | 2              | Indicates the interrupt generation policy required for this task. 1 - Hardware generates an interrupt upon the task completion. 0 - Hardware does not generate an interrupt upon the task completion. Interrupt coalescing is not used with DCMD. |
| Act                 | 5:3            | Is set to b101 to indicate that this is a Task Descriptor.                                                                                                                                                                                        |
| Reserved            | 13:6           |                                                                                                                                                                                                                                                   |
| Queue Barrier (QBR) | 14             | Indicates the control of the host on the ordering between tasks.                                                                                                                                                                                  |
| Reserved            | 15             |                                                                                                                                                                                                                                                   |

Table 18-14: Transfer Descriptor Fields (Continued)

| Field Name    | Bit Location   | Description                                                                                                                                                                                               |
|---------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| CMDIndex      | 21:16          | The index of the command to be sent to the device.                                                                                                                                                        |
| CMDTiming     | 22             | 1 - Command may be sent to device during data activity or busy time. 0 - Command may not be sent to device during data activity or busy time. NOTE: Software is 0 if response type is b11 (R1b).          |
| Response Type | 24:23          | This field indicates to the eMSI controller the response expected to be received from the device. 00 - No Response Expected 01 - Reserved 10 - R1, R4 11 - R1b NOTE: R2 and R3 are not supported in DCMD. |
| Reserved      | 31:25          |                                                                                                                                                                                                           |
| CMDArgument   | 63:32          | The argument of the command to be sent to the device.                                                                                                                                                     |

NOTE: For more information on CQE please refer to Annex B (Normative) Host Controller Interface for Command Queuing from the JEDEC eMMC 5.1 specification.

## eMMC Boot Operation

The eMSI controller supports both mandatory and alternate boot operations as per the JEDEC eMMC 5.1 specifications in 1-bit, 4-bit, and 8-bit SDR or legacy mode up to 50 MHz. In boot operation mode, the eMSI controller can read boot data from the eMMC by keeping the CMD line low (for mandatory boot) or sending CMD0 with the argument (0xFFFFFFFA) (alternate boot), before issuing CMD1. The data can be read from either the boot area or the user area depending on the extended CSD register configurations.

## Mandatory Boot Operation

If the CMD line is held low for 74 clock cycles and more after power-up or reset operation (either through CMD0 with the argument of 0xF0F0F0F0 or assertion of hardware reset for eMMC, if it is enabled in extended CSD register byte [162], bits [1:0]) before the first command is issued, the eMMC recognizes that boot mode is being initiated and starts preparing boot data internally.

The partition that from the eMSI controller reads the boot data can be selected in advance using EXT\_CSD byte [179], bits [5:3]. The eMSI controller can choose to use single data rate mode with backward-compatible interface timing or single data rate with high-speed interface timing by setting a proper value in EXT\_CSD register byte [177] bits [4:3]. EXT\_CSD register byte [228], bit 2 tells the eMSI controller if the high-speed timing during boot is supported by the device.

The configuration in eMMC device should be done before configuring eMSI controller for booting operation.

See eMMC Device Boot Programming Sequences for more information.

Refer to the JEDEC eMMC 5.1 specifications for more information. The mandatory boot timing is shown in the figure below.

Figure 18-10: Mandatory Boot Timing

<!-- image -->

## Alternate Boot Operation

Alternate boot is a command-based boot operation. After a power-up or reset operation (either assertion of CMD0 with the argument of 0xF0F0F0F0 or a hardware reset if it is enabled), the host issues CMD0 with the argument of 0xFFFFFFFA. After 74 clock cycles, before CMD1 is issued or the CMD line goes low, the eMMC recognizes that the boot mode is being initiated and starts preparing boot data internally.

All the mandatory boot operation EXT\_CSD register configurations are applicable for alternate boot operations.

The configuration in eMMC device should be done before configuring eMSI controller for booting operation.

See eMMC Device Boot Programming Sequences for more information. The alternate boot timing is shown in the figure below.

Figure 18-11: Alternate Boot Timing

<!-- image -->

## eMSI Event Control and Interrupts

## Interrupts

The eMSI controller generates interrupts based on various events. There are two interrupt outputs provided by the eMSI controller: EMSI\_STAT and EMSI\_WAKEUP . The interrupt signal must be used as the interrupt for different events. The interrupts are of level type, that is, the interrupt remains asserted (high) until it is cleared by the host or the software.

The interrupt status registers ( EMSI\_ERR\_STAT , EMSI\_ISTAT ) indicate the events that caused the interrupt. Each event can be prevented from asserting the interrupt on the interrupt status by setting the corresponding mask bits in the EMSI\_ERR\_STAT\_EN and EMSI\_ISTAT\_EN registers. A bit in interrupt status register is set only if the corresponding interrupt status enable is set and interrupt event is observed. The bit set in interrupt status register asserts an interrupt to the core only if the corresponding bit in the interrupt signal enable in the EMSI\_ERR\_STAT\_INTEN and EMSI\_ISTAT\_INTEN registers is set.

During standby mode (when the SD card is not being accessed) the EMSI\_WAKEUP signal can be used to identify any wakeup event, such as card removal or insertion.

- NOTE: The host driver is responsible for enabling wakeup signals and disabling interrupt signals when the host system enters standby mode and for disabling wakeup signals and enabling interrupt signals when the host system goes into active mode, when the SD card is in use. The host driver must not enable both at same time. Interrupt signals are enabled using the interrupt signal enable and wakeup signals are enabled using the wakeup event enable.

For information about status interrupts, refer to the EMSI\_ERR\_STAT , EMSI\_ISTAT , and EMSI\_WU\_CTL registers.

## Error Detection

The eMSI controller can detect different types of errors in SD and eMMC transactions. Error are detected in either the command or data portion of the transaction. When an error is detected, the error interrupt in the interrupt status register ( EMSI\_ISTAT.ERR\_INTERRUPT ) is set. The controller uses the error interrupt status register ( EMSI\_ERR\_STAT ) to report errors in SD and eMMC mode. The abort command is used to recover from errors that are detected during data transfers. Additionally, the auto command status register ( EMSI\_AUTOCMD\_STAT ) is used to report errors during Auto CMD operation and the ADMA error status register ( EMSI\_ADMA\_ERR\_STAT ) used to report errors in all ADMA based data transfer modes.

## Response Error Check

The eMSI controller supports a R1 response error check function to avoid the overhead of response error checking by the host driver. If an error is detected, the response error interrupt is generated in the error interrupt status register ( EMSI\_ERR\_STAT.RESP\_ERR ).

Table 18-15: Response Error Check

| Bit                        | Error                      |
|----------------------------|----------------------------|
| Error status checked in R1 | Error status checked in R1 |
| 31                         | OUT_OF_RANGE               |
| 30                         | ADDRESS_ERROR              |
| 29                         | BLOCK_LEN_ERROR            |
| 26                         | WP_VIOLATION               |
| 25                         | CARD_IS_LOCKED             |
| 23                         | COM_CRC_ERROR              |
| 21                         | CARD_ECC_FAILED            |
| 20                         | CC_ERROR                   |
| 19                         | ERROR                      |

NOTE: Please refer to Device Status section from JEDEC eMMC 5.1 specifications for more information.

## eMSI Programming Model

This section provides detailed procedures for programming the eMSI controller.

## Initial Programming Sequence

This section explains the programming steps required for the eMSI controller and the eMMC device before starting a data transfer with a connected eMMC device. Both the eMSI controller and the eMMC device need to be initialized.

1. Ensure clock supply to the eMSI controller. This clock is required for accessing most of the registers in eMSI controller.
2. Check if the card is already inserted by reading the present state register ( EMSI\_PSTATE ). This step is required for a removable card. If the card is already inserted, then go to Step 4.
3. Wait for the card insertion interrupt. This step is required for a removable card. The card detection sequence is described in Card Detection.
4. Configure the basic settings for the eMSI controller as described in eMSI Controller Setup Sequence. This includes the timeout counter value and clock generation parameters. The setup sequence for an SD/eMMC device is provided.
5. Enable all clocks signals as discussed in the programming sequence in eMSI Controller Clock Setup Sequence. At this stage, the eMSI controller is ready to communicate with the eMMC device.

6. If the eMSI controller is connected to an SD card, then the programming sequence described in SD Card Interface Detection must be executed and if the eMSI controller is connected to an eMMC device, then the programming sequence described in eMMC Card Interface Setup must be executed. This is a simplified sequence especially for an eMMC card as the card type is always known.
7. The sequence for Initializing and Identifying an eMMC Device must be executed for initializing and identifying an eMMC device. For SD cards, the SD Card Initialization and Identification sequence must be executed.
8. When the eMSI controller and eMMC device are initialized, the controller can start sending commands to the SD/eMMC device to perform the data transfer. Control and data commands are two types of commands that can be issued to the SD card/eMMC device. The control command is used to read or write any register in the SD card/eMMC device. The data command is used for writing or reading data to or from the device. Below are the control and data command sequences for SD card/eMMC devices.
- Control CMD - Issuing CMD Without Data Transfer
- Data CMD - Issuing CMD With Data Transfer

ADDITIONAL INFORMATION: Data transfer can be aborted using Abort Command Sequence.

## Card Detection

The following figure and procedure shows the sequence for detecting an SD card. The procedure applies to removable cards (SD and cards that use external sockets). This programming sequence is not required for an eMMC device as it is non- removable.

Figure 18-12: Card Detection

<!-- image -->

## 1. Enable interrupt for card detect:

```
a. Set EMSI_ISTAT_EN.CARD_INSERTION to 1. b. Set EMSI_ISTAT_INTEN.CARD_INSERTION to 1. c. Set EMSI_ISTAT_EN.CARD_REMOVAL to 1. d. Set EMSI_ISTAT_INTEN.CARD_REMOVAL to 1.
```

## 2. Clear card detect interrupt status:

- If card insertion interrupt is generated, set EMSI\_ISTAT.CARD\_INSERTION to 1.
- If card removal interrupt is generated, set EMSI\_ISTAT.CARD\_REMOVAL to 1.
3. Check if card is inserted: EMSI\_PSTATE.CARD\_INSERTED (0 = reset, debouncing, or no card).

## eMSI Controller Setup Sequence

This section discusses the eMSI controller setup sequence for an SD/eMMC device.

## eMMC Device Setup

The eMSI Controller Setup Sequence for eMMC Interface figure and procedure shows the eMSI controller setup sequence for an eMMC device. For more information on bit fields set in this sequence please refer to register descriptions.

Figure 18-13: eMSI Controller Setup Sequence for eMMC Interface

<!-- image -->

## 1. Configure common parameters for all versions

```
a. EMSI_TO_CTL.VALUE . b. EMSI_EMMC_CTL.CARD_IS_EMMC = 1.
```

2. Configure EMSI\_CLK\_CTL see eMSI Controller Clock Setup Sequence.

## 3. Configure version 4 parameters.

<!-- formula-not-decoded -->

## SD Interface Setup

The eMSI Controller Setup Sequence for SD Interface figure and procedure shows the eMSI controller setup sequence for an SD interface.

Figure 18-14: eMSI Controller Setup Sequence for SD Interface

<!-- image -->

NOTE: The eMSI controller supports version 4 mode.

1. Configure common parameters for all versions using the EMSI\_TO\_CTL.VALUE bit field.
2. Check if preset value is used?
- If no go to step 4.
- If yes go to step 3.
3. Set EMSI\_CTL2.PRESET\_VAL\_EN = 1.
4. Check host driver version?
- If version other than 4 go to end.
- If version equal to 4 go to step 5.

5. Configure version 4 parameters, EMSI\_CTL2.HOST\_VER4\_EN = 1.

## Clock Control

This section discusses the programming sequence for setting up controller internal clocks and the eMSI bus clock which is provided to the eMMC device. This section also describes programming sequence for changing eMSI bus clock frequency after initial clock setup is completed.

This section includes the following sequences:

- eMSI Controller Clock Setup Sequence
- eMSI Bus Clock Supply and Stop Sequence
- eMSI Bus Clock Frequency Change Sequence

## eMSI Controller Clock Setup Sequence

The eMSI Controller Clock Setup Sequence figure shows the sequence for setting up internal clocks for eMSI controller. The eMSI controller requires an eMSI input clock and eMSI TIMER CMQ clock from the system and an eMSI bus clock (clock supplied to eMMC device). The internal clocks should be initialized before accessing any eMSI controller registers.

Figure 18-15: eMSI Controller Clock Setup Sequence

<!-- image -->

1. Configure the EMSI\_CLK\_CTL.UPPER\_FREQ\_SEL and EMSI\_CLK\_CTL.FREQ\_SEL bits with the required divider value.
2. Configure the EMSI\_CLK\_CTL.INTERNAL\_CLK\_EN register.
3. Check the EMSI\_CLK\_CTL.INTERNAL\_CLK\_EN bit.
- If 0 recheck the EMSI\_CLK\_CTL.INTERNAL\_CLK\_STABLE bit.
- If 1 go to step 4.

ADDITIONAL INFORMATION: On timeout of 150 ms eMSI controller clock setup sequence failed.

4. Configure the EMSI\_CLK\_CTL.PLL\_EN bit.
5. Check the EMSI\_CLK\_CTL.INTERNAL\_CLK\_STABLE bit.
- If 0 recheck the EMSI\_CLK\_CTL.INTERNAL\_CLK\_STABLE bit.
- If 1 go to End

ADDITIONAL INFORMATION: On timeout of 150 ms eMSI controller clock setup sequence failed.

## eMSI Bus Clock Supply and Stop Sequence

The eMSI Bus Clock Supply and Stop Sequence figure shows the flow chart for stopping and supplying the eMSI bus clock. The procedure outlined in the figure applies to SD card/eMMC device.

NOTE: The clock supply and stop sequence is initiated, if required, only when there is no active data transfer between the eMSI controller and the eMMC device.

Figure 18-16: eMSI Bus Clock Supply and Stop Sequence

<!-- image -->

## eMSI Bus Clock Frequency Change Sequence

The eMSI Bus Clock Frequency Change figure shows the sequence for changing eMSI bus clock frequency.

Figure 18-17: eMSI Bus Clock Frequency Change

<!-- image -->

1. Execute the eMSI bus clock stop sequence.
2. Clear the EMSI\_CLK\_CTL.PLL\_EN bit = 0.
3. Check the EMSI\_CTL2.PRESET\_VAL\_EN bit.
- If 0 go to step 4.
- If 1 go to step 5.
4. Configure bus speed mode in the EMSI\_CTL2.EMMC\_MODE\_SEL bit field.
5. Configure EMSI\_CLK\_CTL.FREQ\_SEL and EMSI\_CLK\_CTL.UPPER\_FREQ\_SEL with required divider value.
6. Set EMSI\_CLK\_CTL.PLL\_EN = 1.
7. Check the EMSI\_CLK\_CTL.INTERNAL\_CLK\_STABLE bit.
- If 0 recheck the EMSI\_CLK\_CTL.INTERNAL\_CLK\_STABLE bit.
- If 1 go to End.

ADDITIONAL INFORMATION: On timeout of 150 ms eMSI bus clock frequency change sequence failed.

NOTE:

Software must issue EMSI\_SWRST.DAT and EMSI\_SWRST.CMD after an eMSI bus clock frequency change sequence to avoid the effect of any glitch on the sampling clock. The program must also disable the CDU clock output for the eMSI, before executing an eMSI bus clock frequency change sequence. After sequence execution, the program should reenable the CDU clock. Additionally, enable the eMSI bus clock using the eMSI bus clock supply sequence (to ensure a clock is supplied to the eMMC device for further operations).

## Card Interface Setup Sequence

This section discusses the programming sequence for setting up the card interface for an SD or eMMC card and contains the following topics:

- See SD Card Setup Sequence
- See eMMC Card Interface Setup

## SD Card Setup Sequence

The SD Card Setup Sequence figure shows the programming sequence to set up the SD card. This sequence configures the eMSI controller for detection of an SD card.

Figure 18-18: SD Card Setup Sequence

<!-- image -->

## eMMC Card Interface Setup

The eMMC Card Setup Sequence figure and procedure shows the programming sequence to set up an eMMC device. This sequence configures the eMSI controller for detection of an eMMC device.

Figure 18-19: eMMC Card Setup Sequence

<!-- image -->

1. Configure EMSI\_CTL2.EMMC\_MODE\_SEL = 0 bit for legacy mode.
2. Change the clock frequency (to 400 kHz or less). See eMSI Bus Clock Frequency Change Sequence.
3. Perform eMMC clock supply. See eMSI Bus Clock Supply and Stop Sequence.
4. Wait for voltage ramp up time and provide at least 74 clock cycles before issuing command.
5. eMMC device initialization and identification.

## Setting Timeout on the eMSI Bus

The Setting eMSI Bus Timeout figure and procedure shows the sequence for setting timeout on the eMSI bus. This value determines the interval by which DAT line timeouts are detected. For more information please refer to the corresponding register descriptions.

Figure 18-20: Setting eMSI Bus Timeout

1. Calculate a divisor for detecting timeout by reading the EMSI\_CAP1.TOUT\_CLK\_FREQ and EMSI\_CAP1.TOUT\_CLK\_UNIT registers.
2. Configure the EMSI\_TO\_CTL.VALUE register.

## Abort Transaction

An abort command to the SD/eMMC device is issued to abort an ongoing data transaction. To abort a transaction on an SD/eMMC device, CMD12 is issued.

Following are the instances when an abort command must be issued:

- To stop an infinite block transfer
- To stop a transfer due to a request from an application
- To stop a transfer due to an error detection

The sequence to issue an abort command is shown in Abort Command Sequence. An abort command can be issued asynchronously during data transfer. In an asynchronous abort the abort command is issued at any time irrespective of the state of data transfer as a subcommand. The method is explained in Asynchronous Abort.

## Abort Command Sequence

The Abort Command Sequence figure and procedure describe the abort command sequence.

<!-- image -->

Figure 18-21: Abort Command Sequence

<!-- image -->

1. Issue CMD12.
2. Issue CMD13.
3. Check R1.
- If in TRAN go to End.
- If not in TRAN go to step 5.
4. Issue CMD12.
5. Issue CMD13.
6. Check R1.
- If in TRAN go to End.
- If not in TRAN abort failed.

ADDITIONAL INFORMATION: Keep sending CMD13 until the card returns to the transfer state. In some cases it takes more than twice to send CMD13 for correctly detecting the transfer state. Applicable when CMD 12 is successfully sent.

## Asynchronous Abort

The Asynchronous Abort Sequence figure and procedure describe the asynchronous abort command sequence.

Figure 18-22: Asynchronous Abort Sequence

<!-- image -->

1. Issue abort command (see Abort Command Sequence).
2. Set the EMSI\_SWRST.DAT and EMSI\_SWRST.CMD bits = 1.
3. Check the EMSI\_SWRST.DAT (DR) and EMSI\_SWRST.CMD (CR) bits.
- DR = 1 or CR = 1 recheck the EMSI\_SWRST.DAT and EMSI\_SWRST.CMD bits.
- DR = 0 and CR = 0 go to End.

## eMMC Transaction Mode

This section discusses the programming sequences for setting the eMSI controller for an eMMC interface. This section also discusses how to program and initiate different boot speed modes.

This section discusses the following programming sequences:

- Initializing and Identifying an eMMC Device
- Issuing CMD With or Without Data Transfers for an eMMC Device
- Switching to Various Speed Modes in an eMMC Device
- Changing the Data Bus Width for an eMMC Device
- Command Queueing
- eMMC Device Boot Programming Sequences

## Initializing and Identifying an eMMC Device

The Card Initialization and Identification Programming Sequence figure shows the programming sequence to initialize and identify an eMMC device.

Figure 18-23: Card Initialization and Identification Programming Sequence

<!-- image -->

1. Send CMD1 with address mode required by host ( ≤ 2 GB or &gt;2GB). See Issuing CMD Without Data Transfer.
2. Check for OCR bit busy EMSI\_RESP0 == 1?
- a. No - go back to step 1.
- b. Yes - Go to step 3.
3. Check if EMSI\_RESP0 == 0x80FF8080 or 0xC0FF8080? If not, the device is not compliant.
4. Send CMD2 and obtain the devices CID See Issuing CMD Without Data Transfer.
5. Send CMD3 with chosen RCA (&gt;1) See Issuing CMD Without Data Transfer.
6. Send CMD9 and obtain devices CSD to determine the type of device See Issuing CMD Without Data Transfer.

## Switching to Various Speed Modes in an eMMC Device

The Programming Sequence to Switch to Various Speed Modes in an eMMC Device figure and procedure shows the programming sequence to switch to various speed modes in an eMMC device. In the figure, note that CMD6 is effective only during the transfer state.

Figure 18-24: Programming Sequence to Switch to Various Speed Modes in an eMMC Device

<!-- image -->

1. Execute data bus width change sequence if required.
2. Change to various bus speeds (if supported) for an eMMC card by sending CMD6 see Issuing CMD Without Data Transfer.
3. Execute clock frequency change sequence
4. Configure the same bus speed mode in the host controller using the EMSI\_CTL2.EMMC\_MODE\_SEL bit field. Configure the EMSI\_CTL1.HIGH\_SPEED\_EN bit field (high speed enable = 0 or 1).

## Changing the Data Bus Width for an eMMC Device

The Programming Sequence to Change Data Bus Width for an eMMC Device figure shows the programming sequence to change the data bus width for an eMMC device. In the figure, note that CMD6 is effective only during transfer state.

Figure 18-25: Programming Sequence to Change Data Bus Width for an eMMC Device

<!-- image -->

1. Change bit mode for eMMC card by sending CMD6 (See Issuing CMD Without Data Transfer).
2. Set the EMSI\_CTL1.EXT\_DAT\_XFER bit to 1 for 8-bit or set the EMSI\_CTL1.DAT\_XFER\_WIDTH bit to 1 for 4-bit or clear to 0 for 1-bit.

## Error Recovery in eMMC Mode

The Error Recovery Sequence in eMMC Mode figure and procedure shows the error recovery sequence for eMMC mode.

Figure 18-26: Error Recovery Sequence in eMMC Mode

<!-- image -->

Figure 18-27: Error Recovery Sequence in eMMC Mode (continued)

<!-- image -->

1. Check error interrupt status.
- D08, D03-00 is not set, CMD line error does not occur - go to step 8.
- D08, D03-00 is set, CMD line error occurs - go to step 3.
2. Check if only D8 is set.
- Yes (Auto CMD error) - go to step 6.
- No - go to step 3.
3. EMSI\_TRNSFRMODE.RESP\_INT\_DIS = = 1
- Yes - go to step 6.
- No - go to step 4.

4. Check if D00 is already set in the EMSI\_ISTAT register.
- Yes - go to step 6.
- No - go to step 5.
5. Wait for command complete interrupt.
6. Set software reset for CMD line using the EMSI\_SWRST.CMD bit.
7. Check the EMSI\_SWRST.CMD bit.
- If = 1 recheck.
- If = 0 - go to step 8.
8. Check error interrupt status.
- D09, D06-04 is not set, DAT line error does not occur - go to step 11.
- D09, D06-04 is set, DAT line error occurs - go to step 9.
9. Set software reset for DAT line using the EMSI\_SWRST.DAT bit.
10. Check the EMSI\_SWRST.DAT bit.
- If = 1 recheck.
- If = 0 - go to step 11.
11. Save previous error status.
12. Clear previous error status.
13. Issue Abort command sequence.
14. Check command inhibit (DAT) and (CMD)
- Command and Inhibit DAT = 1 or CMD = 1 - recheck.
- Command Inhibit DAT = 0 or CMD = 0 - go to step 15.
15. Check error interrupt status for Abort CMD
- One of D03-00 is set, CMD line error occurs - go to step 19.
- One of D03-00 is not set, CMD line error does not occur - go to step 16.
16. Check timeout of DAT line.
- DAT line timeout occurs - go to step 19.
- DAT line timeout does not occur - go to step 17.
17. Wait for more that 40 us.
18. Check DAT line

- One or more DAT lines is low - go to step 19.
- DAT line is high - go to step 20.
19. Return status (Non-recoverable error).
20. Return status (Recoverable error).
21. Enable error interrupt signal.

NOTE: The error recovery sequence returns either a Recoverable Error or a Non-Recoverable Error status. When a recoverable error is returned, the error is recovered fully. When the non-recoverable error status returns, both the controller and the eMMC device must be re-initialized after performing a power cycle for the eMMC device.

Set the EMSI\_CMD.TYPE bit field to 0x3 while issuing the abort CMD using CMD12. The software driver must issue a software reset for the DAT line while issuing a software reset for the CMD line. D00, D01, D02, D03, etc. corresponds to bit field 0 , 1, 2, 3 respectively from the EMSI\_ERR\_STAT register.

## Issuing CMD With or Without Data Transfers for an eMMC Device

Programs can issue commands with or without data transfers.

- The programming sequence for issuing a command without a data transfer for an eMMC device is like the sequence described in Issuing CMD Without Data Transfer.
- The programming sequence for issuing a command with a data transfer for an eMMC device is like the programming sequence described in the sections following Issuing CMD With Data Transfer. Data transfers can be performed using DMA. The supported DMA modes are SDMA, ADMA2, and ADMA3.

NOTE: The application must ensure that the EMSI\_BOOT\_CTL.BOOT\_ACK\_ENABLE bit is cleared (when set due to a previous boot CMD) before initiating any data transfer CMD.

## Issuing CMD Without Data Transfer

The eMMC Command Issue and Complete figure and procedure show the steps to issue a CMD without a data transfer.

Figure 18-28: eMMC Command Issue and Complete

<!-- image -->

1. Check the EMSI\_PSTATE.CMD\_INHIBIT bit. a. If 1 - CMD line used. Recheck until the EMSI\_PSTATE.CMD\_INHIBIT bit = 0. b. If 0 - go to step 2.
2. Check if host driver issues an eMMC command with/without using DAT line including busy signal.
- a. Without using DAT line - go to step 5.
- b. Using DAT line - go to step 3.

3. Check if host driver issues abort command.
4. Check EMSI\_PSTATE.CMD\_INHIBIT\_DAT .
- a. If 1 - DAT line used. Repeat until 0.

```
a. Yes - go to step 5. b. No - go to step 4.
```

b.

If 0 - go to step 5.

5. Configure ARGUMENT\_R to generate an eMMC command.
6. Configure CMD\_R.
7. Check EMSI\_TRNSFRMODE.RESP\_INT\_DIS
- a. If 1 - go to step 12.

b.

If 0 - go to step 8.

8. Wait for command complete interrupt ( EMSI\_ISTAT.CMD\_COMPLETE ).
9. Check if command complete interrupt has occurred.
- a. No - go back to step 8.
- b. Yes- go to step 10.
10. Set EMSI\_ISTAT.CMD\_COMPLETE = 1.
11. Get response data from response registers to get necessary information about issued command from the EMSI\_RESP0 / EMSI\_RESP1 / EMSI\_RESP2 / EMSI\_RESP3 registers.
12. Check if command uses transfer complete interrupt.
- a. No - go to step 15.
13. Wait for transfer complete interrupt and proceed to next step when transfer complete interrupt has occurred.
14. Clear transfer complete interrupt ( EMSI\_ISTAT.XFER\_COMPLETE = 1).
15. Check for errors in response status.
16. Return Status (no error).
17. Return status (response contents).

```
b. Yes - go to step 13.
```

```
a. Error - go to step 17. b. No error - go to step 16.
```

NOTE: Before issuing next command for an extra check, the program can poll on the EMSI\_PSTATE.DAT\_LINE\_ACTIVE bit until it is cleared for R1b response command types.

## Issuing CMD With Data Transfer

This section includes the sequence to issues a eMMC command with data transfer using SDMA, ADMA, and ADMA3 modes.

NOTE: While using ADMA2 or ADMA3, the host memory data buffer size and start address must not exceed 128 MB. When greater than 128 MB, the data buffer must be split using two descriptors such that a transfer attempting to cross the limit (of 128 MB) is not generated.

This section discusses the following programming sequences:

- Issuing CMD with Data Transfer (Using SDMA)
- Issuing CMD with Data Transfer (Using ADMA2)
- Issuing CMD with Data Transfer (Using ADMA3)

The ADMA3 ID descriptor, command descriptor, and ADMA2 descriptor are organized in a location in the host memory. Please refer to Advanced DMA 2 (ADMA2) and Advanced DMA 3 (ADMA3) for more information on ADMA2 and ADMA3 descriptors.

NOTE: The ADMA3 ID address, command descriptor address, and ADMA2 descriptor address should be 32 -bit aligned.

## Issuing CMD with Data Transfer (Using SDMA)

The Transaction Control with Data Transfer Using SDMA figure and procedure shows the transaction that uses SDMA for data transfer using the DAT line. The EMSI\_ERR\_STAT.ADMA\_ERR bit is set (=1) for all DMA (SDMA, ADMA2 and ADMA3) transfers for error response.

Figure 18-29: Transaction Control with Data Transfer Using SDMA

<!-- image -->

1. Configure the EMSI\_CTL1.DMA\_SEL bit field for SDMA.
2. Check the value of the EMSI\_CTL2.HOST\_VER4\_EN bit.

```
a. If 0 - go to step 3. b. If 1 - go to step 4.
```

3. Configure the SDMA system address register ( EMSI\_SDMA\_ADDR ).
4. Configure the ADMA system address register ( EMSI\_ADMA\_ADDR\_LO ).
5. Configure the value corresponding to executed data byte length of one block to the EMSI\_BLKSZ register.
6. Configure value corresponding to executed data block count to the EMSI\_BLKCNT register. Configure the size of contiguous buffer in system memory using the EMSI\_BLKSZ.SDMA\_BUF\_BDARY bit field.
7. Configure the argument value in the EMSI\_ARG register.
8. Configure the EMSI\_TRNSFRMODE register.

The host driver determines multiple/single block select, block count enable, data transfer direction, auto CMD12/23 enable and DMA enable. If EMSI\_TRNSFRMODE.RESP\_ERR\_CHK\_EN = 1, set EMSI\_TRNSFRMODE.RESP\_INT\_DIS = 1 and select the transfer mode type in the EMSI\_TRNSFRMODE.RESP\_TYPE register (= R1 response).

9. The eMMC command issued when the upper byte of the EMSI\_CMD register is written and SDMA is started.
10. Check if the response interrupt is disabled in the EMSI\_TRNSFRMODE.RESP\_INT\_DIS bit.
- a. If 1 - go to step 15.
11. Wait for the command complete interrupt ( EMSI\_ISTAT.CMD\_COMPLETE ) to complete.
12. Check if the EMSI\_ISTAT.CMD\_COMPLETE interrupt is complete.
- a. If no - go back to step 11.
- b. If yes - go to step 13.
13. Clear the command complete status by setting EMSI\_ISTAT.CMD\_COMPLETE = 1.
14. Get response data for response registers EMSI\_RESP0 / EMSI\_RESP1 / EMSI\_RESP2 / EMSI\_RESP3 to get the necessary information about the issued command.
15. Wait for the interrupt status of the EMSI\_ISTAT.DMA\_INTERRUPT and EMSI\_ISTAT.XFER\_COMPLETE bits.
16. Check the interrupt status.
- a. Transfer complete interrupt occurs - go to step 19.
- b. DMA interrupt occurs - go to step 17
17. Clear the DMA interrupt status by setting EMSI\_ISTAT.DMA\_INTERRUPT = 1.

```
b. If 0 - go to step 11.
```

18. Configure the next system address to the next data position to the system address register EMSI\_SDMA\_ADDR / EMSI\_ADMA\_ADDR\_LO .
19. Clear the transfer complete status and DMA interrupt status by setting the EMSI\_ISTAT.DMA\_INTERRUPT and EMSI\_ISTAT.XFER\_COMPLETE bits = 1.
3. NOTE: As per the SDMA Buffer Boundary parameter, the program should ensure that the buffer is properly aligned so that it is continuous in the memory up to the specified buffer boundary.

## Issuing CMD with Data Transfer (Using ADMA2)

The Transaction Control with Data Transfer Using DAT Line (Using ADMA2) figure and procedure describe transaction control using the DAT line for ADMA2 data transfers. For more information on ADMA2 operation and descriptor, see Advanced DMA 2 (ADMA2).

Figure 18-30: Transaction Control with Data Transfer Using DAT Line (Using ADMA2)

<!-- image -->

1. Configure DMA select in the EMSI\_CTL1.DMA\_SEL bit field.
2. Create a descriptor table for ADMA in the system memory.
3. Configure the descriptor address for ADMA in the EMSI\_ADMA\_ADDR\_LO registers
4. Configure the value corresponding to the executed byte length of one block in the EMSI\_BLKSZ register.

5. Configure the value corresponding to the executed data block count in the EMSI\_BLKCNT register.
6. Configure the argument value in the EMSI\_ARG register.
7. Configure the EMSI\_TRNSFRMODE register (= R1 response). The host driver determines multiple/single block select, block count enable, data transfer direction, auto

CMD12 enable and DMA enable. If EMSI\_TRNSFRMODE.RESP\_ERR\_CHK\_EN = 1, set EMSI\_TRNSFRMODE.RESP\_INT\_DIS = 1 and select EMSI\_TRNSFRMODE.RESP\_TYPE .

8. Configure the argument value in the EMSI\_CMD register. The eMMC command is issued when the upper byte of the EMSI\_CMD register is written.
9. Check if the response interrupt is disabled in the EMSI\_TRNSFRMODE.RESP\_INT\_DIS bit.
- a. If 1 - go to step 14.
10. Wait for the command complete interrupt ( EMSI\_ISTAT.CMD\_COMPLETE ) to complete.
11. Check if EMSI\_ISTAT.CMD\_COMPLETE is complete.
- a. If no - go back to step 10.
- b. If yes - go to step 12.
12. Clear command complete status by setting EMSI\_ISTAT.CMD\_COMPLETE = 1
13. Get response data for response registers EMSI\_RESP0 / EMSI\_RESP1 / EMSI\_RESP2 / EMSI\_RESP3 to get the necessary information about the issued command.
14. Wait for transfer complete interrupt in the EMSI\_ISTAT.XFER\_COMPLETE bit and the ADMA error interrupt in the EMSI\_ERR\_STAT.ADMA\_ERR bit.
15. Check the interrupt status.
- a. Transfer complete interrupt occurs - go to step 19.
- b. DMA interrupt occurs - go to step 17
16. Clear transfer complete interrupt status by setting EMSI\_ISTAT.XFER\_COMPLETE = 1
17. Clear ADMA error interrupt status.
18. Abort ADMA operation.

```
b. If 0 - go to step 10.
```

NOTE: During card read/write DMA operations, it is recommended that the very last transfer descriptor also has the End bit set for marginal performance improvement. The End bit should always be set on a descriptor of type TRAN (not for link descriptor or other type descriptor). Descriptor types are provided in the following table for quick reference. The descriptor type recommendation in Table 18-16 Descriptor Types is applicable only to ADMA2 and ADMA3 transfers.

Table 18-16: Descriptor Types

|   Act2 |   Act1 |   Act0 | Symbol   | Comment         | ADMA2 Descriptor Address                        |
|--------|--------|--------|----------|-----------------|-------------------------------------------------|
|      0 |      0 |      0 | NOP      | No operation    | Do not execute current line and go to next line |
|      0 |      1 |      0 | rsv      | Reserved        | Same as NOP                                     |
|      1 |      0 |      0 | Tran     | Transfer data   | Transfer data of one descriptor line            |
|      1 |      1 |      0 | Link     | Link descriptor | Link to another descriptor                      |

## Issuing CMD with Data Transfer (Using ADMA3)

The Transaction Control with Data Transfer Using DAT Line (Using ADMA3) figure and procedure describe transaction control using the DAT line for ADMA3 data transfers.

Figure 18-31: Transaction Control with Data Transfer Using DAT Line (Using ADMA3)

<!-- image -->

1. Configure DMA select.
2. Create command descriptors and ADMA2 descriptors.

3. Create integrated descriptors.
4. Configure the integrated DMA descriptor address in the EMSI\_ADMA\_ADDR\_LO register.
5. Wait for transfer complete interrupt or ADMA error interrupt.
6. Check interrupt status.
- a. ADMA error - go to step 8.
- b. No ADMA error - go to step 7.
7. Clear transfer complete interrupt status by setting EMSI\_ISTAT.XFER\_COMPLETE = 1.
8. Clear the ADMA error interrupt status
9. Clear the ADMA system address and ADMA error status.
10. Abort the ADMA operation.

The following list describes the descriptor requirements for an ADMA3 CMD:

- Configuring infinite data transfer ( EMSI\_TRNSFRMODE.BLOCK\_COUNT\_EN = 0) is not allowed except for single block transfer ( EMSI\_TRNSFRMODE.MULTI\_BLK\_SEL = 0)
- As ADMA3 only supports 32-bit block count mode, EMSI\_TRNSFRMODE.BLOCK\_COUNT\_EN is set to 1 and 16-bit block count ( EMSI\_BLKCNT.VALUE ) is configured to 0x0000. Configuring stop count (32-bit block count = 0) is not allowed except for single block transfers ( EMSI\_TRNSFRMODE.MULTI\_BLK\_SEL = 0).
- Configuring no data transfer (Block Size = 0) is not allowed, except for no data transfer command in the command descriptor ( CDEnd = 1).
- In cases where the memory data transfer command is used, the Auto CMD Auto Select option (0x3) in the EMSI\_TRNSFRMODE.AUTO\_CMD\_EN bit field is used so that the command descriptor is independent of eMMC device support CMD23. For other commands, auto command disabled (0x0) is used. If EMSI\_CTL2.CMD23\_EN = 1 then Auto CMD23 is issued before the transfer commands and if EMSI\_CTL2.CMD23\_EN = 0, then Auto CMD12 is issued after the transfer complete. For SD cards that do not support CMD23 (card versions 2.00 and below) configure the EMSI\_CTL2 register accordingly.

During card read/write DMA operations, it is recommended that the very last transfer descriptor also has the End bit set for marginal performance improvement. The End bit should always be set on a descriptor of type TRAN. The following table is applicable only to ADMA2 and ADMA3 transfers.

Table 18-17: Descriptor Types

|   Act2 |   Act1 |   Act0 | Symbol   | Comment      | ADMA2 Descriptor Address                            |
|--------|--------|--------|----------|--------------|-----------------------------------------------------|
|      0 |      0 |      0 | NOP      | No Operation | Do not execute current line and go to the next line |

Table 18-17: Descriptor Types (Continued)

|   Act2 |   Act1 |   Act0 | Symbol   | Comment         | ADMA2 Descriptor Address                                     |
|--------|--------|--------|----------|-----------------|--------------------------------------------------------------|
|      0 |      1 |      0 | RSV      | Reserved        | Same as Nop. Do not execure current line and go to next line |
|      1 |      0 |      0 | TRAN     | Transfer Data   | Transfer data of one descriptor line                         |
|      1 |      1 |      0 | LINK     | Link Descriptor | Link to another descriptor                                   |

## Command Queueing

The following sections describe the processes for initializing the command queuing engine and submitting and completing a task in the Command Queuing engine.

## Initializing the Command Queuing Engine

The Initializing the Command Queuing Engine figure shows the programming sequence to initialize the command queuing engine.

Figure 18-32: Initializing the Command Queuing Engine

<!-- image -->

1. Set EMSI\_SWRST.DAT = 1.
2. Configure the task descriptor list base address register, EMSI\_CQ\_TDL\_BADDR .
3. Configure a value corresponding to executed data byte length of one block in the EMSI\_BLKSZ register.
4. Configure the following registers and bits:
- a. EMSI\_TRNSFRMODE register.
- b. Configure EMSI\_CTL1.DMA\_SEL to ADMA2 only mode.
- c. Clear EMSI\_BLKCNT = 0.

- d. In the EMSI\_SDMA\_ADDR register (while using 32 bits) or configure the EMSI\_BLKCNT register (while using 16 bits).
5. Configure the command queuing interrupt coalescing register to enable interrupt coalescing and to configure the timeout ( EMSI\_CQ\_IC.TOUT\_VAL ) and threshold ( EMSI\_CQ\_IC.INTC\_TH ) parameters.
6. Configure the following interrupt registers to enable interrupts.
- a. EMSI\_CQ\_ISTAT , EMSI\_CQ\_ISTAT\_EN , EMSI\_CQ\_ISTAT\_INTEN b. EMSI\_ISTAT , EMSI\_ISTAT\_EN , EMSI\_ISTAT\_INTEN . c. Disable generation of command complete and transfer complete interrupts ( EMSI\_ISTAT.XFER\_COMPLETE , EMSI\_ISTAT.CMD\_COMPLETE ).
5. .
7. Configure the EMSI\_CQ\_SSCFG1 register for device queue status fetch polling time and block count. Configure the EMSI\_CQ\_SSCFG2 register for RCA.
8. Configure the EMSI\_CQ\_CFG.DCMD\_EN , EMSI\_CQ\_CFG.TASK\_DESC\_SIZE and EMSI\_CQ\_CFG.CQ\_EN bits.

Check the EXT\_CSD register byte 308 for command queuing support from card side. Byte 307 of the EXT\_CSD register denotes the command queuing depth that the device supports. The program must also enable command queuing from the eMMC device side before enabling from the eMSI controller side (CMDQ\_MODE\_EN [15] in the EXT\_CSD register).

## Submitting and Completing a Task in the Command Queuing Engine

The Submitting and Completing a Task figure and procedure shows the sequence to submit and complete a task in a command queuing engine.

NOTE: Task descriptors and transfer descriptors in the host memory are aligned to an 8-byte boundary.

Figure 18-33: Submitting and Completing a Task

<!-- image -->

1. Are there unoccupied slots?
2. Configure descriptors for post initialization, task, and transfer. Set the corresponding EMSI\_CQ\_TDBR bit of these registers.
3. Wait for the task complete interrupt ( EMSI\_CQ\_ISTAT.TCC ).
4. Read the task completion notification register ( EMSI\_CQ\_TCN ). If interrupt coalescing is inactive go to step 5. If interrupt coalescing is active go to step 6.
5. If interrupt coalescing was previously inactive, clear the EMSI\_CQ\_TCN register by writing 1 to the set of bits then clear the EMSI\_ISTAT.CQE\_EVENT bit. The slots mapped to the tasks are now free and can be reused.

```
a. If no - Stop. b. If yes - go to step 2.
```

6. If interrupt coalescing was previously active, clear the EMSI\_CQ\_IC.INTC\_RST bit then clear the EMSI\_CQ\_ISTAT.TCC bit and clear task notification. Then clear the EMSI\_ISTAT.CQE\_EVENT bit and empty the slot.
7. Setup interrupt coalescing if required.

## CQE Error Recovery

The CQE Recovery Mode figure and procedure shows the error recovery sequence for CQE mode.

Figure 18-34: CQE Recovery Mode

<!-- image -->

1. Set the EMSI\_CQ\_CTL.HALT bit.

```
2. Read the EMSI_CQ_CTL.HALT bit. 3. Check if EMSI_CQ_CTL.HALT bit set. a. No - go to step 2. b. Yes - go to step 4. 4. Is there a CMD 44/45 error? a. No, CMD46/47 or XFER error - go to step 5. b. Yes - go to step 6.
```

5. Other method of recovery (for example power cycle).
6. Task discard sequence for error task ID.
7. SW specific recovery procedure.

NOTE: Writes to the Command Queuing DoorBell register ( EMSI\_CQ\_TDBR ) after requesting a HALT (by writing 1 to the EMSI\_CQ\_CTL.HALT bit field may be ignored by the eMSI controller. An ignored EMSI\_CQ\_TDBR write generates an error.

## eMMC Device Boot Programming Sequences

Software must adhere to the following steps when working with boot operation. The partition that from the eMSI controller reads the boot data, can be selected in advance using EXT\_CSD byte [179], bits [5:3]. The eMSI controller can choose to use the

- Single data rate mode with backward-compatible interface timing,
- Single data rate with high-speed interface timing

The EXT\_CSD register byte [228], bit 2 tells the eMSI controller if the high-speed timing during boot is supported by the device.

Configure the eMMC device before configuring the eMSI controller for boot operation.

Please refer to JEDEC eMMC 5.1 specification for more information.

NOTE: The eMSI controller does not support boot operation in DDR speed mode.

## Preparing for a Boot

The Programming Sequence to Prepare for a Boot figure and procedure shows the programming sequence to prepare for a boot. This sequence is common for both the mandatory and alternate modes.

NOTE: The eMSI controller does not support boot in ADMA3 mode.

Figure 18-35: Programming Sequence to Prepare for a Boot

<!-- image -->

1. Configure the host controller with the proper voltage and clock required for booting (see eMSI Controller Clock Setup Sequence).
2. Configure the EMSI\_CTL2 register with the bus speed required for boot.
3. Configure the EMSI\_CTL1 register to define the bus width required for boot. Select the DMA type (if DMA is enabled for boot data transfer).
4. Clear all interrupt status (if any) in the EMSI\_ISTAT register. Configure the corresponding bits in the EMSI\_ISTAT\_EN , EMSI\_ISTAT\_INTEN , EMSI\_ERR\_STAT\_INTEN and EMSI\_ERR\_STAT\_EN registers for the interrupt required for boot.

## Initiating a Mandatory Boot in SDMA Mode

The Programming Sequence to Initiate a Mandatory Boot in SDMA Mode figure and procedure shows the programming sequence to initiate a mandatory boot in SDMA mode.

Figure 18-36: Programming Sequence to Initiate a Mandatory Boot in SDMA Mode

<!-- image -->

1. Check the value of the EMSI\_CTL2.HOST\_VER4\_EN bit field.
- a. 0 - go to step 2. b. 1 - go to step 3
2. Configure the SDMA system address register ( EMSI\_SDMA\_ADDR ).
3. Configure the ADMA system address register ( EMSI\_ADMA\_ADDR\_LO ).
4. Configure EMSI\_BLKSZ = 512 bytes.
5. Configure EMSI\_BLKCNT (as per boot size).
6. Configure the EMSI\_TRNSFRMODE register. The host determines multiple/single block select, block count enable, data transfer direction, and DMA enable.
7. Configure the EMSI\_BOOT\_CTL register. Set EMSI\_BOOT\_CTL.BOOT\_ACK\_ENABLE if boot ACK is expected.
8. Wait for interrupt status of the EMSI\_ISTAT.DMA\_INTERRUPT and EMSI\_ISTAT.XFER\_COMPLETE bits.
9. Check interrupt status.

```
a. Transfer complete interrupt occurs - go to step 12. b. DMA complete interrupt occurs - go to step 10. 10. Clear the DMA interrupt status ( EMSI_ISTAT.DMA_INTERRUPT = 1). 11. Configure the next system address to the next data position to EMSI_SDMA_ADDR . 12. Clear the transfer complete status and DMA interrupt status bits. Set EMSI_ISTAT.DMA_INTERRUPT = 1 and EMSI_ISTAT.XFER_COMPLETE = 1.
```

## Initiating a Mandatory Boot in ADMA2 Mode

The Programming Sequence to Initiate a Mandatory Boot in ADMA2 Mode figure shows the programming sequence for initiating a mandatory boot in ADMA2 mode.

Figure 18-37: Programming Sequence to Initiate a Mandatory Boot in ADMA2 Mode

<!-- image -->

1. Create a descriptor table for ADMA in the system memory.
2. Configure the descriptor address for ADMA in the system address registers ( EMSI\_ADMA\_ADDR\_LO ).
3. Configure the value corresponding to the executed data byte length of one block to the EMSI\_BLKSZ register.
4. Configure the value corresponding to the executed data block count to the EMSI\_BLKCNT register when block count is enabled.
5. Configure the EMSI\_TRNSFRMODE register. The host determines multiple/single block select, block count enable, data transfer direction, and DMA enable.

6. Configure the EMSI\_BOOT\_CTL register. Set the EMSI\_BOOT\_CTL.BOOT\_ACK\_ENABLE bit (if boot ACK is expected).
7. Wait for the transfer complete interrupt EMSI\_ISTAT.XFER\_COMPLETE or ADMA error interrupt EMSI\_ERR\_STAT.ADMA\_ERR .
8. Check interrupt status.
- a. Transfer complete interrupt occurs - go to step 9.
- b. ADMA error interrupt occurs - go to step 10.
9. Clear transfer complete status by setting EMSI\_ISTAT.XFER\_COMPLETE = 1.
10. Clear ADMA error interrupt status.

## Initiating Alternate Boot

Alternate Boot is a command-based boot operation and can be initiated by issuing CMD0 with the 0xFFFFFFFA argument. For the programming sequence for initiating an alternate boot, see Issuing CMD with Data Transfer (Using SDMA) and Issuing CMD with Data Transfer (Using ADMA2) with the following exceptions:

- If boot acknowledgment is expected, the EMSI\_BOOT\_CTL.BOOT\_TOUT\_CNT is programmed to configure the timeout counter for boot acknowledgment before setting the EMSI\_CMD register.
- Response check cannot be performed as CMD0 has no response.
- Issue CMD0 with 0xFFFFFFFA argument with EMSI\_CMD.DATA\_PRESENT\_SEL set to 1.

When the boot is initiated, the device is assumed to be in pre-boot state and 74 clock cycles has expired after power is stable before issuing CMD0.

## SD Transaction Mode

This section includes the sequences to generate and control different types of SD transactions that can be classified as transactions without data transfer using the DAT/CMD line and transactions with data transfer using the DAT/CMD line. The following procedures are included in this section.

- SD Card Initialization and Identification
- Changing Bus Width for an SD Card
- Changing Bus Speed Mode for an SD Card
- Issuing CMD Without Data Transfer
- Issuing CMD With Data Transfer

## SD Card Initialization and Identification

The SD Card Initialization and Identification figure shows the sequence for SD card initialization and identification.

Figure 18-38: SD Card Initialization and Identification

<!-- image -->

## Changing Bus Width for an SD Card

The SD Card Change Bus Width figure shows the sequence for changing the bit mode in a SD card.

Figure 18-40: Changing Bus Speed Mode

## Issuing CMD With or Without Data Transfers for an SD Card

Programs can issue commands with or without data transfers.

- The programming sequence for issuing a command without a data transfer for an SD card is like the sequence described in the Issuing CMD Without Data Transfer section.
- The programming sequence for issuing a command with a data transfer for an SD card is like the programming sequence described in the sections following Issuing CMD With Data Transfer.

Data transfers can be performed using DMA. The supported DMA modes are SDMA, ADMA2, and ADMA3.

Figure 18-39: SD Card Change Bus Width

<!-- image -->

## Changing Bus Speed Mode for an SD Card

The Changing Bus Speed Mode figure shows the process for changing bus speed mode in a SD card.

<!-- image -->

## ADSP-SC59x EMSI Register Descriptions

The Enhanced Mobile Storage Interface (EMSI) contains the following registers.

Table 18-18: ADSP-SC59x EMSI Register List

| Name                 | Description                                        |
|----------------------|----------------------------------------------------|
| EMSI_ADMA_ADDR_LO    | ADMA System Address Register - Low                 |
| EMSI_ADMA_DESADDR_LO | ADMA3 Integrated Descriptor Address Register - Low |
| EMSI_ADMA_ERR_STAT   | ADMA Error Status Register                         |
| EMSI_ARG             | Argument Register                                  |
| EMSI_AUTOCMD_STAT    | Auto CMDStatus Register                            |
| EMSI_BASEADDR0       | Base Address 0 Register                            |
| EMSI_BASEADDR1       | Base Address 1 Register                            |
| EMSI_BLKCNT          | 16-bit Block Count Register                        |
| EMSI_BLKSZ           | Block Size Register                                |
| EMSI_BOOT_CTL        | eMMC Boot Control Register                         |
| EMSI_CAP1            | Capabilities 1 Register - 0 to 31                  |
| EMSI_CAP2            | Capabilities Register - 32 to 63                   |
| EMSI_CLK_CTL         | Clock Control Register                             |
| EMSI_CMD             | Command Register                                   |
| EMSI_CQVER           | Command Queuing Version Register                   |
| EMSI_CQ_CAP          | Command Queuing Capabilities Register              |
| EMSI_CQ_CFG          | Command Queuing Configuration Register             |
| EMSI_CQ_CRARG        | CQ Command Response Argument Register              |
| EMSI_CQ_CRDCT        | Command Response for Direct Command Register       |
| EMSI_CQ_CRI          | CQ Command Response Index                          |
| EMSI_CQ_CTL          | Command Queuing Control Register                   |
| EMSI_CQ_DPT          | Device Pending Tasks Register                      |
| EMSI_CQ_DQSTAT       | Device Queue Status Register                       |
| EMSI_CQ_IC           | Command Queuing Interrupt Coalescing Register      |
| EMSI_CQ_ISTAT        | Command Queuing Interrupt Status Register          |
| EMSI_CQ_ISTAT_EN     | Command Queuing Interrupt Status Enable Register   |
| EMSI_CQ_ISTAT_INTEN  | Command Queuing Interrupt Signal Enable Register   |
| EMSI_CQ_RMEM         | Command Response Mode Error Mask Register          |
| EMSI_CQ_SSCFG1       | CQ Send Status Configuration 1 Register            |

Table 18-18: ADSP-SC59x EMSI Register List (Continued)

| Name                 | Description                                                |
|----------------------|------------------------------------------------------------|
| EMSI_CQ_SSCFG2       | CQ Send Status Configuration 2 Register                    |
| EMSI_CQ_TCLR         | Command Queuing Task Clear Register                        |
| EMSI_CQ_TCN          | Command Queuing Task Completion Notification Register      |
| EMSI_CQ_TDBR         | Command Queuing DoorBell Register                          |
| EMSI_CQ_TDL_BADDR    | Command Queuing Task Descriptor List Base Address Register |
| EMSI_CQ_TERRINFO     | CQ Task Error Information Register                         |
| EMSI_CTL1            | EMSI Control 1 Register                                    |
| EMSI_CTL2            | EMSI Control 2 Register                                    |
| EMSI_HOST_CNTRL_VERS | SD Host Controller Specification Version                   |
| EMSI_EMMC_CTL        | eMMC Control Register                                      |
| EMSI_ERR_STAT        | Error Interrupt Status Register                            |
| EMSI_ERR_STAT_EN     | Error Interrupt Status Enable Register                     |
| EMSI_ERR_STAT_INTEN  | Error Interrupt Signal Enable Register                     |
| EMSI_FRC_AUTOCMDSTAT | Force Event Register for Auto CMDError Status Register     |
| EMSI_FRC_ERRSTAT     | Force Event Register for Error Interrupt Status            |
| EMSI_ISTAT           | Interrupt Status Register                                  |
| EMSI_ISTAT_EN        | Interrupt Status Enable Register                           |
| EMSI_ISTAT_INTEN     | Interrupt Signal Enable Register                           |
| EMSI_PRESET_DS       | Preset Value for Default Speed                             |
| EMSI_PRESET_HSDDR    | Preset Value for HSDDR                                     |
| EMSI_PRESET_HSSDR    | Preset Value for HSSDR                                     |
| EMSI_PRESET_INIT     | Preset Value for Initialization                            |
| EMSI_PRESET_LEGACY   | Preset Value for Legacy Mode                               |
| EMSI_PSTATE          | Present State Register                                     |
| EMSI_RESP0           | Response Register 0                                        |
| EMSI_RESP1           | Response Register 1                                        |
| EMSI_RESP2           | Response Register 2                                        |
| EMSI_RESP3           | Response Register 3                                        |
| EMSI_SDMA_ADDR       | SDMA System Address Register                               |
| EMSI_SWRST           | Software Reset Register                                    |
| EMSI_TO_CTL          | Timeout Control Register                                   |

Table 18-18: ADSP-SC59x EMSI Register List (Continued)

| Name            | Description             |
|-----------------|-------------------------|
| EMSI_TRNSFRMODE | Transfer Mode Register  |
| EMSI_VER_ID     | EMSI Version            |
| EMSI_VER_TYPE   | EMSI Version Type       |
| EMSI_WU_CTL     | Wakeup Control Register |

## ADMA System Address Register - Low

The EMSI\_ADMA\_ADDR\_LO register holds the 32-bit system address for DMA transfer. This register is applicable for eMMC mode.

Figure 18-41: EMSI\_ADMA\_ADDR\_LO Register Diagram

<!-- image -->

Table 18-19: EMSI\_ADMA\_ADDR\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | ADMA System Address. The EMSI_ADMA_ADDR_LO.VALUE bits indicate the 32 bits of the ADMA system address. -SDMA: If Host Version 4 Enable is set (=1), this register stores the system address of the data location -ADMA2: This register stores the byte address of the executing command of the de- scriptor table -ADMA3: This register is set by ADMA3. ADMA2 increments the address of this reg- ister that points to the next line, every time a descriptor line is fetched. |

## ADMA3 Integrated Descriptor Address Register - Low

The EMSI\_ADMA\_DESADDR\_LO register holds the 32-bit Integrated Descriptor address.This register is applicable for eMMC mode.

Figure 18-42: EMSI\_ADMA\_DESADDR\_LO Register Diagram

<!-- image -->

Table 18-20: EMSI\_ADMA\_DESADDR\_LO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | ADMA Integrated Descriptor Address. The EMSI_ADMA_DESADDR_LO.VALUE bits indicate the 32-bit of the ADMA in- tegrated descriptor address. The start address of integrated descriptor is set to these reg- ister bits. The ADMA3 fetches one descriptor address and increments these bits to in- dicate the next descriptor address. |

## ADMA Error Status Register

The EMSI\_ADMA\_ERR\_STAT register stores the ADMA state during an ADMA error. This register is applicable for an eMMC mode.

Figure 18-43: EMSI\_ADMA\_ERR\_STAT Register Diagram

<!-- image -->

Table 18-21: EMSI\_ADMA\_ERR\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/NW)           | LEN_ERR    | ADMA Length Mismatch Error States. This error occurs in the following instances: - While the block count enable is being set, the total data length specified by the de- scriptor table is different from that specified by the block count and block length - When the total data length cannot be divided by the block length | ADMA Length Mismatch Error States. This error occurs in the following instances: - While the block count enable is being set, the total data length specified by the de- scriptor table is different from that specified by the block count and block length - When the total data length cannot be divided by the block length |
|                    |            | 0                                                                                                                                                                                                                                                                                                                               | No Error                                                                                                                                                                                                                                                                                                                        |
| 1:0 (R/NW)         | ERR_STATES | ADMA Error States. The EMSI_ADMA_ERR_STAT.ERR_STATES bits indicate the state of ADMA when an error occurs during ADMA data transfer.                                                                                                                                                                                            | ADMA Error States. The EMSI_ADMA_ERR_STAT.ERR_STATES bits indicate the state of ADMA when an error occurs during ADMA data transfer.                                                                                                                                                                                            |
|                    |            | 0                                                                                                                                                                                                                                                                                                                               | Stop DMA- SYS_ADR Register Points to a Location Next to the Error Descriptor                                                                                                                                                                                                                                                    |
|                    |            | 1                                                                                                                                                                                                                                                                                                                               | Fetch Descriptor - SYS_ADR Register Points to the Er- ror Descriptor                                                                                                                                                                                                                                                            |
|                    |            | 2                                                                                                                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                                                                                                                        |
|                    |            | 3                                                                                                                                                                                                                                                                                                                               | Transfer Data - SYS_ADR Register Points to a Location Next to the Error Descriptor                                                                                                                                                                                                                                              |

## Argument Register

The EMSI\_ARG register is used to configure the eMMC command argument.

Figure 18-44: EMSI\_ARG Register Diagram

<!-- image -->

Table 18-22: EMSI\_ARG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Command Argument. The EMSI_ARG.VALUE bits specify the eMMC command argument that is speci- fied in bits 39-8 of the Command format. |

## Auto CMD Status Register

The EMSI\_AUTOCMD\_STAT register is used to indicate the CMD12 response error of Auto CMD12, and the CMD23 response error of Auto CMD23. The host driver can determine the kind of Auto CMD12/CMD23 errors that can occur in this register. Auto CMD23 errors are indicated in bit 04-01.

This register is valid only when Auto CMD Error is set in the EMSI\_ERR\_STAT register. This register is applicable for eMMC mode.

Figure 18-45: EMSI\_AUTOCMD\_STAT Register Diagram

<!-- image -->

Table 18-23: EMSI\_AUTOCMD\_STAT Register Fields

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|-----------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/NW)           | CMD_NOT_IS- SUED_AUTO_CMD12 | Command Not Issued By Auto CMD12 Error. If the EMSI_AUTOCMD_STAT.CMD_NOT_ISSUED_AUTO_CMD12 bit is set (=1), CMD_wo_DAT is not executed due to an Auto CMD12 Error (D04-D01) in this register. The EMSI_AUTOCMD_STAT.CMD_NOT_ISSUED_AUTO_CMD12 bit is cleared (= 0) when Auto CMDError is generated by Auto CMD23. 0 No Error |
| 5 (R/NW)           | AUTO_CMD_RESP_ERR           | Auto CMDResponse Error. The EMSI_AUTOCMD_STAT.AUTO_CMD_RESP_ERR bit is set when Response Error Check Enable in the Transfer Mode register is set (=1) and an error is detected in R1 response of either Auto CMD12 or CMD13. This status is ignored if any bit be- tween D00 to D04 is set (=1). 0 No Error Error            |
| 5 (R/NW)           |                             | 1                                                                                                                                                                                                                                                                                                                            |
| 5 (R/NW)           |                             |                                                                                                                                                                                                                                                                                                                              |

Table 18-23: EMSI\_AUTOCMD\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|-----------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/NW)           | AUTO_CMD_IDX_ERR      | Auto CMDIndex Error. The EMSI_AUTOCMD_STAT.AUTO_CMD_IDX_ERR bit is set if the command in- dex error occurs in response to a command.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 3 (R/NW)           | AUTO_CMD_EBIT_ERR     | Auto CMDEnd Bit Error. The EMSI_AUTOCMD_STAT.AUTO_CMD_EBIT_ERR bit is set when detecting that the end bit of command response is 0.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 2 (R/NW)           | AUTO_CMD_CRC_ERR      | Auto CMDCRCError. The EMSI_AUTOCMD_STAT.AUTO_CMD_CRC_ERR bit is set when detecting a CRC error in the command response.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 1 (R/NW)           | AU- TO_CMD_TOUT_ERR   | 1 CRC Error Generated Auto CMDTimeout Error. The EMSI_AUTOCMD_STAT.AUTO_CMD_TOUT_ERR bit is set if no response is returned with 64 eMSI bus clock cycles from the end bit of the command. If the EMSI_AUTOCMD_STAT.AUTO_CMD_TOUT_ERR bit is set (=1), error status bits (D04-D01) are meaningless. 0 No Error                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/NW)           | AU- TO_CMD12_NOT_EXEC | 1 Time Out Auto CMD12 Not Executed. If multiple memory block data transfer is not started due to a command error, the EMSI_AUTOCMD_STAT.AUTO_CMD12_NOT_EXEC bit is not set because it is not necessary to issue an Auto CMD12. When the EMSI_AUTOCMD_STAT.AUTO_CMD12_NOT_EXEC bit is set (=1), the eMSI con- troller cannot issue Auto CMD12 to stop multiple memory block data transfer, due to some error. If the EMSI_AUTOCMD_STAT.AUTO_CMD12_NOT_EXEC bit is set (=1), error status bits (D04-D01) is meaningless. The EMSI_AUTOCMD_STAT.AUTO_CMD12_NOT_EXEC bit is cleared (=0) when Auto CMDError is generated by Auto CMD23. 0 Executed |

## Base Address 0 Register

The EMSI\_BASEADDR0 register used as a pointer for the base address 0 register.

Figure 18-46: EMSI\_BASEADDR0 Register Diagram

<!-- image -->

Table 18-24: EMSI\_BASEADDR0 Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration   |
|--------------------|-----------------|---------------------------|
| 11:0               | REG_OFFSET_ADDR | Base Offset Address.      |
| (R/NW)             |                 |                           |

## Base Address 1 Register

The EMSI\_BASEADDR1 register used as a pointer for the base address 1 register.

Figure 18-47: EMSI\_BASEADDR1 Register Diagram

<!-- image -->

Table 18-25: EMSI\_BASEADDR1 Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration   |
|--------------------|-----------------|---------------------------|
| 15:0               | REG_OFFSET_ADDR | Base Offset Address.      |
| (R/NW)             |                 |                           |

## 16-bit Block Count Register

The EMSI\_BLKCNT register is used to configure the number of data blocks. This register is applicable for eMMC mode.

Figure 18-48: EMSI\_BLKCNT Register Diagram

<!-- image -->

Table 18-26: EMSI\_BLKCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | 16-bit Block Count. The EMSI_BLKCNT.VALUE bit field has the following settings: If the Host Version 4 Enable bit is set 0 or the 16-bit Block Count register is set to non-zero, the 16-bit Block Count register is selected. If the Host Version 4 Enable bit is set 1 and the 16-bit Block Count register is set to zero, the 32-bit Block Count register is selected. Following are the values for this bit field: 0x0: Stop Count 0x1: 1 Block 0x2: 2 Blocks ... - ... 0xFFFF: 65535 Blocks Note: For Host Version 4 Enable = 0, this register must be set to 0000h before pro- gramming the 32-bit block count register when Auto CMD23 is enabled for non- DMAand ADMA modes. |

## Block Size Register

The EMSI\_BLKSZ register is used to configure an SDMA buffer boundary and the number of bytes in a data block. This register is applicable for eMMC mode.

Figure 18-49: EMSI\_BLKSZ Register Diagram

<!-- image -->

Table 18-27: EMSI\_BLKSZ Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                          |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/W)        | SDMA_BUF_BDARY | SDMA Buffer Boundary. The EMSI_BLKSZ.SDMA_BUF_BDARY bits specify the size of contiguous buffer in system memory. The SDMA transfer waits at every boundary specified by these fields and the eMSI controller generates the DMAinterrupt to request the host driver to up- date the SDMA system address register. | SDMA Buffer Boundary. The EMSI_BLKSZ.SDMA_BUF_BDARY bits specify the size of contiguous buffer in system memory. The SDMA transfer waits at every boundary specified by these fields and the eMSI controller generates the DMAinterrupt to request the host driver to up- date the SDMA system address register. |
|                    |                | 0                                                                                                                                                                                                                                                                                                                | 4K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                    |
|                    |                | 1                                                                                                                                                                                                                                                                                                                | 8K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                    |
|                    |                | 2                                                                                                                                                                                                                                                                                                                | 16K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                   |
|                    |                | 3                                                                                                                                                                                                                                                                                                                | 32K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                   |
|                    |                | 4                                                                                                                                                                                                                                                                                                                | 64K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                   |
|                    |                | 5                                                                                                                                                                                                                                                                                                                | 128K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                  |
|                    |                | 6                                                                                                                                                                                                                                                                                                                | 256K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                  |
|                    |                | 7                                                                                                                                                                                                                                                                                                                | 512K Bytes SDMA Buffer Boundary                                                                                                                                                                                                                                                                                  |

Table 18-27: EMSI\_BLKSZ Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:0 (R/W)         | XFER_BLKSZ | Transfer Block Size. The EMSI_BLKSZ.XFER_BLKSZ bits specify the block size of data transfers. When using an eMMC device, this bit field is set to 512 bytes. The EMSI_BLKSZ.XFER_BLKSZ bits can be accessed only if no transaction is execut- ing. Read operations to the bit field during transfers may return an invalid value, and write operations are ignored. The following are the value settings. - 0x1: 1 byte - 0x2: 2 bytes - 0x3: 3 bytes - ...... - 0x1FF: 511 byte - 0x200: 512 bytes - ...... - 0x800: 2048 bytes Note: This register must be programmed with a non-zero value for data transfer. |

## eMMC Boot Control Register

The EMSI\_BOOT\_CTL register is used to control the eMMC Boot operation.

Figure 18-50: EMSI\_BOOT\_CTL Register Diagram

<!-- image -->

Table 18-28: EMSI\_BOOT\_CTL Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                |
|--------------------|-----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:12 (R/W)        | BOOT_TOUT_CNT   | Boot Ack Timeout Counter Value. The EMSI_BOOT_CTL.BOOT_TOUT_CNT value determines the interval by which boot ack timeout (50 ms) is detected when boot ack is expected during boot operation. - 0xF: Reserved - 0xE: TMCLK x 2^27 -... - 0x1: TMCLK x 2^14 - 0x0: TMCLK x 2^13 NOTE: Please refer to EMSI_CAP1.TOUT_CLK_FREQ for TMCLK. | Boot Ack Timeout Counter Value. The EMSI_BOOT_CTL.BOOT_TOUT_CNT value determines the interval by which boot ack timeout (50 ms) is detected when boot ack is expected during boot operation. - 0xF: Reserved - 0xE: TMCLK x 2^27 -... - 0x1: TMCLK x 2^14 - 0x0: TMCLK x 2^13 NOTE: Please refer to EMSI_CAP1.TOUT_CLK_FREQ for TMCLK. |
| 8 (R/W)            | BOOT_ACK_ENABLE | Boot Acknowledge Enable. When the EMSI_BOOT_CTL.BOOT_ACK_ENABLE bit set, eMSI controller checks for boot acknowledge start pattern of 0-1-0 during boot operation. The EMSI_BOOT_CTL.BOOT_ACK_ENABLE bit is applicable for both mandatory and alternate boot mode.                                                                     | Boot Acknowledge Enable. When the EMSI_BOOT_CTL.BOOT_ACK_ENABLE bit set, eMSI controller checks for boot acknowledge start pattern of 0-1-0 during boot operation. The EMSI_BOOT_CTL.BOOT_ACK_ENABLE bit is applicable for both mandatory and alternate boot mode.                                                                     |
|                    |                 | 0                                                                                                                                                                                                                                                                                                                                      | Boot Ack Disable                                                                                                                                                                                                                                                                                                                       |
|                    |                 | 1                                                                                                                                                                                                                                                                                                                                      | Boot Ack Enable                                                                                                                                                                                                                                                                                                                        |
| 7 (RX/W)           | VALIDATE_BOOT   | Validate Mandatory Boot Enable bit. The EMSI_BOOT_CTL.VALIDATE_BOOT bit is used to validate the MAN_BOOT_EN bit.                                                                                                                                                                                                                       | Validate Mandatory Boot Enable bit. The EMSI_BOOT_CTL.VALIDATE_BOOT bit is used to validate the MAN_BOOT_EN bit.                                                                                                                                                                                                                       |
|                    |                 | 0                                                                                                                                                                                                                                                                                                                                      | Ignore Mandatory Boot Enable Bit                                                                                                                                                                                                                                                                                                       |
|                    |                 | 1                                                                                                                                                                                                                                                                                                                                      | Validate Mandatory Boot Enable Bit                                                                                                                                                                                                                                                                                                     |

Table 18-28: EMSI\_BOOT\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                  |
|--------------------|-------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | MAN_BOOT_EN | Mandatory Boot Enable. The EMSI_BOOT_CTL.MAN_BOOT_EN bit is used to initiate the mandatory boot operation. The application sets the EMSI_BOOT_CTL.MAN_BOOT_EN bit along with VALIDATE_BOOT bit. Writing 0 is ignored. The eMSI controller clears the EMSI_BOOT_CTL.MAN_BOOT_EN bit after the boot transfer is completed or ter- minated. |
| 0 (R/W)            | MAN_BOOT_EN | 0 Mandatory Boot Disable                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | MAN_BOOT_EN | 1 Mandatory Boot Enable                                                                                                                                                                                                                                                                                                                  |

## Capabilities 1 Register - 0 to 31

The EMSI\_CAP1 register provides the host driver with information specific to the eMSI controller implementation.

The eMSI controller may implement these values as fixed or during power on initialization. This register is segregated into two 32-bit registers: EMSI\_CAP1 -2. The EMSI\_CAP1 register is the lower part of capabilities register.

Figure 18-51: EMSI\_CAP1 Register Diagram

<!-- image -->

Table 18-29: EMSI\_CAP1 Register Fields

| Bit No. (Access)   | Bit Name          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|-------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:30 (R/NW)       | SLOT_TYPE_R       | Slot Type. The EMSI_CAP1.SLOT_TYPE_R bits indicate usage of a slot by a specific host sys- tem.                                                                                                                                                                                                                                                                                                                                | Slot Type. The EMSI_CAP1.SLOT_TYPE_R bits indicate usage of a slot by a specific host sys- tem.                                                                                                                                                                                                                                                                                                                                |
| 31:30 (R/NW)       | SLOT_TYPE_R       | 0                                                                                                                                                                                                                                                                                                                                                                                                                              | Removable Card Slot                                                                                                                                                                                                                                                                                                                                                                                                            |
| 31:30 (R/NW)       | SLOT_TYPE_R       | 1                                                                                                                                                                                                                                                                                                                                                                                                                              | Embedded Slot for One Device                                                                                                                                                                                                                                                                                                                                                                                                   |
| 31:30 (R/NW)       | SLOT_TYPE_R       | 2                                                                                                                                                                                                                                                                                                                                                                                                                              | Shared Bus Slot (SD Mode)                                                                                                                                                                                                                                                                                                                                                                                                      |
| 31:30 (R/NW)       | SLOT_TYPE_R       | 3                                                                                                                                                                                                                                                                                                                                                                                                                              | UHS-II Multiple Embedded Devices                                                                                                                                                                                                                                                                                                                                                                                               |
| 29                 | ASYNC_INT_SUPPORT | Asynchronous Interrupt Support (SD Mode only).                                                                                                                                                                                                                                                                                                                                                                                 | Asynchronous Interrupt Support (SD Mode only).                                                                                                                                                                                                                                                                                                                                                                                 |
| (R/NW)             | ASYNC_INT_SUPPORT | 0                                                                                                                                                                                                                                                                                                                                                                                                                              | Asynchronous Interrupt Not Supported                                                                                                                                                                                                                                                                                                                                                                                           |
| (R/NW)             | ASYNC_INT_SUPPORT | 1                                                                                                                                                                                                                                                                                                                                                                                                                              | Asynchronous Interrupt Supported                                                                                                                                                                                                                                                                                                                                                                                               |
| 28 (R/NW)          | SYS_ADDR_64_V3    | 64-bit System Address Support for V3. The EMSI_CAP1.SYS_ADDR_64_V3 bit sets the eMSI controller to support 64- bit system addressing of V3 mode. SDMA cannot be used in 64-bit addressing in Version 3 mode. If the EMSI_CAP1.SYS_ADDR_64_V3 bit is set (=1), to use 64-bit ADMA2 with a 96-bit descriptor is enabled by setting Host Version 4 Enable ( EMSI_CTL2.HOST_VER4_EN = 0) and DMAselect ( EMSI_CTL1.DMA_SEL = 11b). | 64-bit System Address Support for V3. The EMSI_CAP1.SYS_ADDR_64_V3 bit sets the eMSI controller to support 64- bit system addressing of V3 mode. SDMA cannot be used in 64-bit addressing in Version 3 mode. If the EMSI_CAP1.SYS_ADDR_64_V3 bit is set (=1), to use 64-bit ADMA2 with a 96-bit descriptor is enabled by setting Host Version 4 Enable ( EMSI_CTL2.HOST_VER4_EN = 0) and DMAselect ( EMSI_CTL1.DMA_SEL = 11b). |
| 28 (R/NW)          | SYS_ADDR_64_V3    | 0                                                                                                                                                                                                                                                                                                                                                                                                                              | 64-bit System Address for V3 is Not Supported                                                                                                                                                                                                                                                                                                                                                                                  |
| 28 (R/NW)          | SYS_ADDR_64_V3    | 1                                                                                                                                                                                                                                                                                                                                                                                                                              | 64-bit System Address for V3 is Supported                                                                                                                                                                                                                                                                                                                                                                                      |
| 27 (R/NW)          | SYS_ADDR_64_V4    | 64-bit System Address Support for V4. The EMSI_CAP1.SYS_ADDR_64_V4 bit sets the eMSI controller to support 64-                                                                                                                                                                                                                                                                                                                 | 64-bit System Address Support for V4. The EMSI_CAP1.SYS_ADDR_64_V4 bit sets the eMSI controller to support 64-                                                                                                                                                                                                                                                                                                                 |
|                    |                   | EMSI_CTL2 EMSI_CAP1.SYS_ADDR_64_V4 bit is set (=1), 64-bit DMAaddressing for                                                                                                                                                                                                                                                                                                                                                   | EMSI_CTL2 EMSI_CAP1.SYS_ADDR_64_V4 bit is set (=1), 64-bit DMAaddressing for                                                                                                                                                                                                                                                                                                                                                   |
|                    |                   | 0                                                                                                                                                                                                                                                                                                                                                                                                                              | 64-bit System Address for V4 is Not Supported                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |                   | 1                                                                                                                                                                                                                                                                                                                                                                                                                              | 64-bit System Address for V4 is Supported                                                                                                                                                                                                                                                                                                                                                                                      |

Table 18-29: EMSI\_CAP1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name                            | Description/Enumeration                                                                                                                                                                                                                             |
|--------------------|-------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/NW)          | VOLT_18                             | Voltage Support for 1.8V. 0 1.8V Not Supported                                                                                                                                                                                                      |
| 25 (R/NW)          | VOLT_30 Voltage Support for VOLT_33 | SD 3.0V or Embedded 1.2V. 0 SD 3.0V or Embedded 1.2V Not Supported 1 SD 3.0V or Embedded Supported Voltage Support for 3.3V. 0 3.3V Not Supported                                                                                                   |
| 24 (R/NW)          | SUS_RES_SUPPORT                     | 1 3.3V Supported Suspense/Resume Support. The EMSI_CAP1.SUS_RES_SUPPORT bit indicates whether the eMSI                                                                                                                                              |
| 23 (R/NW)          |                                     | controller supports suspend/resume functionality. If the EMSI_CAP1.SUS_RES_SUPPORT bit is 0, the host driver does not issue either suspend or resume commands because the suspend and resume mechanism is not supported. 0 Not Supported            |
| 22 (R/NW)          | SDMA_SUPPORT                        | 1 Supported                                                                                                                                                                                                                                         |
|                    |                                     | SDMA Support. The EMSI_CAP1.SDMA_SUPPORT bit indicates whether the eMSI controller is ca- pable of using SDMA to transfer data between the system memory and the host con- troller directly. 0 SDMA Not Supported 1 SDMA Supported                  |
| 21 (R/NW)          | HIGH_SPEED_SUPPORT                  | High Speed Support.                                                                                                                                                                                                                                 |
| 19 (R/NW)          | ADMA2_SUPPORT                       | The EMSI_CAP1.HIGH_SPEED_SUPPORT bit indicates whether the eMSI con- troller and the host system supports high speed mode and they can supply the eMSI bus clock frequency from 25 MHz to 50 MHz. 0 High Speed Not Supported 1 High Speed Supported |
|                    |                                     | ADMA2 Support. The EMSI_CAP1.ADMA2_SUPPORT bit indicates whether the eMSI controller is capable of using ADMA2.                                                                                                                                     |
|                    |                                     | 0 ADMA2 Not Supported                                                                                                                                                                                                                               |
|                    | 1                                   | ADMA2 Supported                                                                                                                                                                                                                                     |

Table 18-29: EMSI\_CAP1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                    |
|--------------------|----------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18 (R/NW)          | EMBEDDED_8_BIT | 8-bit Support for Embedded Device. The EMSI_CAP1.EMBEDDED_8_BIT bit indicates whether the eMSI controller is capable of using an 8-bit bus width mode. The EMSI_CAP1.EMBEDDED_8_BIT bit is not effective when the Slot Type is set to 10b. 0 8-bit Bus Width Not Supported |
| 17:16 (R/NW)       | MAX_BLK_LEN    | Maximum Block Length. The EMSI_CAP1.MAX_BLK_LEN bit indicates the maximum block size that the host driver can read and write to the buffer in the eMSI controller. The buffer transfers this block size without wait cycles. 0 512 Byte 1 1024 Byte 2 2048 Byte            |

Table 18-29: EMSI\_CAP1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/NW)        | BASE_CLK_FREQ | Base Clock Frequency for eMSI Bus Clock (50 MHz). The EMSI_CAP1.BASE_CLK_FREQ field indicate the base (maximum) clock fre- quency for the eMSI bus clock. The definition of these bits depend on the SD host controller version. - 6-Bit Base Clock Frequency: This mode is supported by the SD host controller ver- sion 1.00 and 2.00. The upper 2 bits are not effective and are always 0. The unit val- ues are 1 MHz. The supported clock range is 10 MHz to 63 MHz. - 0x00: Get information through another method - 0x01: 1 MHz - 0x02: 2 MHz ............. - 0x3F: 63 MHz - 0x40-0xFF: Not Supported - 8-Bit Base Clock Frequency: This mode is supported by the SD host controller ver- sion 3.00. The unit values are 1 MHz. The supported clock range is 10 MHz to 255 MHz. - 0x00: Get information through another method - 0x01: 1 MHz - 0x02: 2 MHz -- ............ - 0xFF: 255 MHz If the frequency is 16.5 MHz, the larger value is set to 0001001b (17 MHz) because the host driver uses this value to calculate the clock divider value and it does not ex- ceed the upper limit of the eMSI bus clock frequency. If the EMSI_CAP1.BASE_CLK_FREQ field are all 0, the host system has to get informa- tion using a different method. |
| 7 (R/NW)           | TOUT_CLK_UNIT | Timeout Clock Unit. The EMSI_CAP1.TOUT_CLK_UNIT bit shows the unit of base clock frequency used to detect data timeout error. 0 KHz                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 7 (R/NW)           | TOUT_CLK_UNIT | 1 MHz                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 18-29: EMSI\_CAP1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/NW)         | TOUT_CLK_FREQ | Timeout Clock Frequency. The EMSI_CAP1.TOUT_CLK_FREQ bit shows the base clock frequency used to detect Data Timeout Error. The timeout clock unit defines the unit of timeout clock frequency. It can be configured for kHz or MHz. - 0x00: Get information through another method - 0x01: 1KHz / 1MHz - 0x02: 2KHz / 2MHz - 0x03: 3KHz / 3MHz - ........... - 0x3F: 63KHz / 63MHz |

## Capabilities Register - 32 to 63

The EMSI\_CAP2 register provides the host driver with information specific to the eMSI controller implementation.

The eMSI controller may implement these values as fixed or loaded from the flash memory during power on initialization. This register is segregated into two 32-bit registers: EMSI\_CAP2 -EMSI\_CAP1 The EMSI\_CAP2 register is the upper part of capabilities register.

Figure 18-52: EMSI\_CAP2 Register Diagram

<!-- image -->

Table 18-30: EMSI\_CAP2 Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                              |
|--------------------|------------------|------------------------------------------------------------------------------------------------------|
| 28 (R/NW)          | VDD2_18V_SUPPORT | 1.8V VDD2 Support. The EMSI_CAP2.VDD2_18V_SUPPORT bit indicates support of VDD2 for the Host System. |
| 28 (R/NW)          | VDD2_18V_SUPPORT | 0 1.8V VDD2 is Not Supported                                                                         |
| 28 (R/NW)          | VDD2_18V_SUPPORT | 1 1.8V VDD2 is Supported                                                                             |

Table 18-30: EMSI\_CAP2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                       |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 27 (R/NW)          | ADMA3_SUPPORT   | ADMA3 Support. The EMSI_CAP2.ADMA3_SUPPORT bit indicates whether the eMSI controller is capable of using ADMA3.                                                                                                                                                                                                                               |
| 23:16 (R/NW)       | CLK_MUL         | Clock Multiplier. The EMSI_CAP2.CLK_MUL field indicate the clock multiplier of the programmable clock generator. Clearing (=0) the EMSI_CAP2.CLK_MUL field indicates that the eMSI controller does not support a programmable clock generator. - 0x0: Clock Multiplier is not Supported - 0x1: Clock MultiplierM=2 - 0x2: Clock MultiplierM=3 |
| 15:14 (R/NW)       | RE_TUNING_MODES | Re-Tuning Modes (UHS-I Only). The EMSI_CAP2.RE_TUNING_MODES bits select the re-tuning method and limit the maximum data length.                                                                                                                                                                                                               |
| 15:14 (R/NW)       | RE_TUNING_MODES | 0 Timer                                                                                                                                                                                                                                                                                                                                       |
| 15:14 (R/NW)       | RE_TUNING_MODES | 1 Timer and Re-Tuning Request (Not Supported)                                                                                                                                                                                                                                                                                                 |
| 15:14 (R/NW)       | RE_TUNING_MODES | 2 Auto Re-Tuning (for Transfer)                                                                                                                                                                                                                                                                                                               |
| 11:8 (R/NW)        | RETUNE_CNT      | 3 Reserved Timer Count for Re-Tuning (UHS-I Only). - 0x0: Re-Tuning Timer disabled - 0x1: 1 seconds - 0x2: 2 seconds - 0x3: 4 seconds - ........ - 0xB: 1024 seconds - 0xC: Reserved - 0xD: Reserved - 0xE: Reserved - 0xF: Get information from other source                                                                                 |

Table 18-30: EMSI\_CAP2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                      |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/NW)           | DRV_TYPED      | Driver Type DSupport (UHS-I only). The EMSI_CAP2.DRV_TYPED bit indicates support of driver type Dfor 1.8 signal- ing.                                                        |
| 6 (R/NW)           | DRV_TYPED      | 0 Driver Type Dis Not Supported                                                                                                                                              |
| 5 (R/NW)           | DRV_TYPEC      | Driver Type C Support (UHS-I only). The EMSI_CAP2.DRV_TYPEC bit indicates support of driver type C for 1.8 signal- ing.                                                      |
| 5 (R/NW)           | DRV_TYPEC      | 0 Driver Type C is Not Supported                                                                                                                                             |
| 5 (R/NW)           | DRV_TYPEC      | 1 Driver Type C is Supported                                                                                                                                                 |
| 4 (R/NW)           | DRV_TYPEA      | Driver Type A Support (UHS-I only). The EMSI_CAP2.DRV_TYPEA bit indicates support of driver type A for 1.8 signal- ing.                                                      |
| 4 (R/NW)           | DRV_TYPEA      | 0 Driver Type A is Not Supported                                                                                                                                             |
| 4 (R/NW)           | DRV_TYPEA      | 1 Driver Type A is Supported                                                                                                                                                 |
| 3 (R/NW)           | UHS2_SUPPORT   | eMMC Interface Enable.                                                                                                                                                       |
| 3 (R/NW)           | UHS2_SUPPORT   | 0 UHS-II is Not Supported                                                                                                                                                    |
| 2                  | DDR50_SUPPORT  | DDR50 Support (UHS-I only).                                                                                                                                                  |
| (R/NW)             | DDR50_SUPPORT  | 0 DDR50 is Not Supported                                                                                                                                                     |
| 2                  | DDR50_SUPPORT  | 1 DDR50 is Supported                                                                                                                                                         |
| 1 (R/NW)           | SDR104_SUPPORT | SDR104 Support (UHS-I only). The EMSI_CAP2.SDR104_SUPPORT bit reports that SDR104 requires tuning.                                                                           |
| 1 (R/NW)           | SDR104_SUPPORT | 0 SDR104 is Not Supported                                                                                                                                                    |
| 1 (R/NW)           | SDR104_SUPPORT | 1 SDR104 is Supported                                                                                                                                                        |
| 0 (R/NW)           | SDR50_SUPPORT  | SDR50 Support (UHS-I only). The EMSI_CAP2.SDR50_SUPPORT bit indicates that SDR50 is supported. The bit 13 (USE_TUNING_SDR50) indicates whether SDR50 requires tuning or not. |
| 0 (R/NW)           | SDR50_SUPPORT  | 0 SDR50 is Not Supported                                                                                                                                                     |
| 0 (R/NW)           | SDR50_SUPPORT  | 1 SDR50 is Supported                                                                                                                                                         |

## Clock Control Register

The EMSI\_CLK\_CTL register controls the eMSI bus clock to an eMMC device. This register is applicable for eMMC mode.

Figure 18-53: EMSI\_CLK\_CTL Register Diagram

<!-- image -->

Table 18-31: EMSI\_CLK\_CTL Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:8 (R/W)         | FREQ_SEL       | eMSI Bus Clock Frequency Select. The EMSI_CLK_CTL.FREQ_SEL bits are used to select the frequency of the eMSI bus clock signal. These bits depend on setting of ( EMSI_CTL2.PRESET_VAL_EN ). If preset value enable = 0, these bits are set by the host driver. If preset value enable = 1, these bits are automatically set to a value specified in one of the preset value register. The value is reflected on the lower 8-bit of the frequency divider signal. 10-bit Divided Clock Mode: - 0x3FF: 1/2046 divided clock - .......... - N: 1/2N divided clock - .......... - 0x002: 1/4 divided clock - 0x001: 1/2 divided clock - 0x000: base clock (0 to 50MHz) |
| 7:6 (R/W)          | UPPER_FREQ_SEL | Upper Frequency Select. The EMSI_CLK_CTL.UPPER_FREQ_SEL bits specify the upper 2 bits of 10-bit eMSI bus clock frequency select control. The value is reflected on the upper 2 bits of the frequency divider.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 18-31: EMSI\_CLK\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|-----------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | PLL_EN                | PLL Enable. The EMSI_CLK_CTL.PLL_EN bit is used to activate the PLL (applicable when Host Version 4 Enable = 1). When Host Version 4 Enable = 0, INTER- NAL_CLK_EN bit may be used to activate PLL. Note: If the EMSI_CLK_CTL.PLL_EN bit is not used to active the PLL when EMSI_CTL2.HOST_VER4_EN = 1, it is recommended to set the EMSI_CLK_CTL.PLL_EN bit to 1.                                                                      |
| 3 (R/W)            | PLL_EN                | 0 PLL is in Low Power Mode                                                                                                                                                                                                                                                                                                                                                                                                              |
| 2 (R/W)            | EMSI_BUS_CLK_EN       | eMSI Bus Clock Enable. The EMSI_CLK_CTL.EMSI_BUS_CLK_EN bit stops the SDCLK when cleared (=0). The SDCLK Frequency Select bit can be changed when the EMSI_CLK_CTL.EMSI_BUS_CLK_EN bit is cleared (=0).                                                                                                                                                                                                                                 |
| 2 (R/W)            | EMSI_BUS_CLK_EN       | 0 Disable Providing eMSI Bus Clock                                                                                                                                                                                                                                                                                                                                                                                                      |
| 2 (R/W)            | EMSI_BUS_CLK_EN       | 1 Enable Providing eMSI Bus Clock                                                                                                                                                                                                                                                                                                                                                                                                       |
| 1 (R/NW)           | INTERNAL_CLK_STA- BLE | Internal Clock Stable. The EMSI_CLK_CTL.INTERNAL_CLK_STABLE bit enables the host driver to check the clock stability twice after the Internal Clock Enable bit is set and after the PLL Enable bit is set.                                                                                                                                                                                                                              |
| 1 (R/NW)           | INTERNAL_CLK_STA- BLE | 0 Not Ready                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 1 (R/NW)           | INTERNAL_CLK_STA- BLE | 1 Ready                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | INTERNAL_CLK_EN       | Internal Clock Enable. The EMSI_CLK_CTL.INTERNAL_CLK_EN bit is cleared (=0) when the host driv- er is not using the eMSI controller. The eMSI controller must stop its internal clock to enter a very low power state. However, registers can still be read and written to. If the EMSI_CLK_CTL.INTERNAL_CLK_EN bit is not used to control the internal clock, it is recommended to set the EMSI_CLK_CTL.INTERNAL_CLK_EN bit to 1. Stop |
| 0 (R/W)            | INTERNAL_CLK_EN       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W)            | INTERNAL_CLK_EN       | 1 Oscillate                                                                                                                                                                                                                                                                                                                                                                                                                             |

## Command Register

The EMSI\_CMD register is used to provide the information related to a command and a response packet. This register is applicable for eMMC mode. Writing to the upper byte of this register triggers eMMC command generation.

Figure 18-54: EMSI\_CMD Register Diagram

<!-- image -->

Table 18-32: EMSI\_CMD Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:8 (R/W)         | INDEX            | Command Index. The EMSI_CMD.INDEX bits are set to the command number that is specified in bits 45-40 of the Command Format.                                                                                                                                                                                                                                     |
| 7:6 (R/W)          | TYPE             | Command Type. The EMSI_CMD.TYPE bits indicate the command type. Note: While issuing Abort CMD(CMD12) this field is set to 0x3. Normal                                                                                                                                                                                                                           |
| 7:6 (R/W)          | TYPE             | 0                                                                                                                                                                                                                                                                                                                                                               |
| 7:6 (R/W)          | TYPE             | 1 Reserved                                                                                                                                                                                                                                                                                                                                                      |
| 7:6 (R/W)          | TYPE             | 2 Reserved                                                                                                                                                                                                                                                                                                                                                      |
| 5 (R/W)            | DATA_PRESENT_SEL | Data Present Select. The EMSI_CMD.DATA_PRESENT_SEL bit is set (=1) to indicate that data is present and that the data is transferred using the DAT line. The EMSI_CMD.DATA_PRESENT_SEL bit is cleared (=0) in the following instan- ces: - Command using the CMDline - Command with no data transfer but using busy signal on the DAT[0] line 0 No Data Present |
| 5 (R/W)            | DATA_PRESENT_SEL | 1 Data Present                                                                                                                                                                                                                                                                                                                                                  |
| 5 (R/W)            | DATA_PRESENT_SEL |                                                                                                                                                                                                                                                                                                                                                                 |

Table 18-32: EMSI\_CMD Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | IDX_CHK_ENABLE   | Command Index Check Enable. The EMSI_CMD.IDX_CHK_ENABLE bit enables the eMSI controller to check the index field in the response to verify if it has the same value as the command index. If the value is not the same, it is reported as a Command Index error. Note: EMSI_CMD.IDX_CHK_ENABLE must be cleared (=0) for the command with no response, R2 response, R3 response and R4 response. 0 Disable |
| 3 (R/W)            | CRC_CHK_EN       | Command CRC Check Enable. The EMSI_CMD.CRC_CHK_EN bit enables the eMSI controller to check the CRC field in the response. If an error is detected, it is reported as a Command CRC error. Note: CRC Check enable must be cleared (=0) for the command with no response, R3 response, and R4 response.                                                                                                     |
| 2 (R/W)            | SUB_CMD_FLAG     | 1 Enable Sub Command Flag. The EMSI_CMD.SUB_CMD_FLAG bit distinguishes between a main command and a sub command. The Sub Command flag is set =1 while issuing an abort command (CMD12) while using asynchronous abort. 0 Main Command                                                                                                                                                                     |
| 1:0 (R/W)          | RESP_TYPE_SELECT | Response Type Select. The EMSI_CMD.RESP_TYPE_SELECT bit indicates the type of response expected from the card. 0 No Response 1 Response Length 136                                                                                                                                                                                                                                                        |

## Command Queuing Version Register

The EMSI\_CQVER register provides information about the version of the eMMC Command Queueing standard, which is implemented by the CQE in BCD format.

Figure 18-55: EMSI\_CQVER Register Diagram

<!-- image -->

Table 18-33: EMSI\_CQVER Register Fields

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                               |
|--------------------|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11:8 (R/NW)        | EMMC_VER_MAJOR  | Indicates the eMMC Major Version. The EMSI_CQVER.EMMC_VER_MAJOR bit indicates the eMMC major version (1st digit left of decimal point) in BCD format  |
| 7:4 (R/NW)         | EMMC_VER_MINOR  | Indicates the eMMC Minor Version. The EMSI_CQVER.EMMC_VER_MINOR bit indicates the eMMC minor version (1st digit right of decimal point) in BCD format |
| 3:0 (R/NW)         | EMMC_VER_SUFFIX | EEMC Version Suffix. The EMSI_CQVER.EMMC_VER_SUFFIX bit indicates the eMMC version suffix (2nd digit right of decimal point) in BCD format            |

## Command Queuing Capabilities Register

The EMSI\_CQ\_CAP register indicates the capabilities of the command queuing engine.

Figure 18-56: EMSI\_CQ\_CAP Register Diagram

<!-- image -->

Table 18-34: EMSI\_CQ\_CAP Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                               |
|--------------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/NW)          | CRYPTO_SUPPORT | Crypto Support. The EMSI_CQ_CAP.CRYPTO_SUPPORT bit indicates whether the Host Control- ler supports cryptographic operations.                                                                                                                                                         |
| 15:12 (R/NW)       | ITCFMUL        | Internal Timer Clock Frequency Multiplier. The EMSI_CQ_CAP.ITCFMUL bit field indicates the frequency of the clock used for interrupt coalescing timer and for determining the SQS polling period. See the EMSI_CQ_CAP.ITCFVAL definition for details. Values 0x5 to 0xF are reserved. |
| 9:0 (R/NW)         | ITCFVAL        | Internal Timer Clock Frequency Value. The EMSI_CQ_CAP.ITCFVAL bit field scales the frequency of the timer clock pro- vided by EMSI_CQ_CAP.ITCFMUL . The final clock frequency of actual timer clock is calculated as ITCFVAL x ITCFMUL. The default value is 10MHz.                   |

## Command Queuing Configuration Register

The EMSI\_CQ\_CFG register controls CQE behavior affecting the general operation of command queuing engine.

Figure 18-57: EMSI\_CQ\_CFG Register Diagram

<!-- image -->

Table 18-35: EMSI\_CQ\_CFG Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | DCMD_EN        | Slot 31 Task Descriptor. The EMSI_CQ_CFG.DCMD_EN bit indicates to the hardware whether the task de- scriptor in slot #31 of the TDL is a data transfer descriptor or a direct-command de- scriptor. The CQE uses the EMSI_CQ_CFG.DCMD_EN bit when a task is issued in slot #31, to determine how to decode the Task Descriptor. 0 Task Descriptor in Slot #31 is a Data Transfer Task De- |
| 8 (R/W)            | TASK_DESC_SIZE | Task Descriptor Size. The EMSI_CQ_CFG.TASK_DESC_SIZE bit indicates the size of task descriptor used in host memory. The EMSI_CQ_CFG.TASK_DESC_SIZE bit can only be configured when Command Queuing Enable bit is 0 (command queuing is disabled). 0 Task Descriptor Size is 64 Bits Bits                                                                                                  |
| 8 (R/W)            |                | 1 Task Descriptor Size is 128                                                                                                                                                                                                                                                                                                                                                             |
| 8 (R/W)            |                |                                                                                                                                                                                                                                                                                                                                                                                           |

Table 18-35: EMSI\_CQ\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | CQ_EN      | Enable command queuing engine (CQE). When the CQE is disable, the software controls the eMSI bus using the registers be- tween EMSI_SDMA_ADDR (offset 0x000) to EMSI_HOST_CNTRL_VERS (offset 0x0FE). Before the software writes to the EMSI_CQ_CFG.CQ_EN bit, the software verifies that the eMSI controller is in idle state and there are no ongoing commands or data transfers. When software wants to exit command queuing mode, it clears all pre- vious tasks (if any) before clearing (=0) the EMSI_CQ_CFG.CQ_EN bit. 0 Disable Command Queuing |
| 0 (R/W)            | CQ_EN      | 1 Enable Command Queuing                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | CQ_EN      |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

## CQ Command Response Argument Register

The EMSI\_CQ\_CRARG register stores the argument of the last received command response.

Figure 18-58: EMSI\_CQ\_CRARG Register Diagram

<!-- image -->

Table 18-36: EMSI\_CQ\_CRARG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Last Command Response argument. The EMSI_CQ_CRARG.VALUE bit field stores the argument of the last received command response. The controller updates the value every time a command response is received. |

## Command Response for Direct Command Register

The EMSI\_CQ\_CRDCT register stores the response of last executed DCMD.

Figure 18-59: EMSI\_CQ\_CRDCT Register Diagram

<!-- image -->

Table 18-37: EMSI\_CQ\_CRDCT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | DCMD_RESP  | Direct Command Response. The EMSI_CQ_CRDCT.DCMD_RESP bit field contains the response of the com- mand generated by the last direct command (DCMD) task that was sent. The contents of the EMSI_CQ_CRDCT.DCMD_RESP bit field are valid only after bit 31 of the register is cleared by the controller. |

## CQ Command Response Index

The EMSI\_CQ\_CRI register stores the index of the last received command response.

Figure 18-60: EMSI\_CQ\_CRI Register Diagram

<!-- image -->

Table 18-38: EMSI\_CQ\_CRI Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5:0 (R/NW)         | VALUE      | Last Command Response Index. The EMSI_CQ_CRI.VALUE bit field stores the index of the last received command response. The controller updates the value every time a command response is received. |

## Command Queuing Control Register

The EMSI\_CQ\_CTL register controls CQE behavior affecting the general operation of command queuing module or simultaneous operation of multiple tasks.

Figure 18-61: EMSI\_CQ\_CTL Register Diagram

<!-- image -->

Table 18-39: EMSI\_CQ\_CTL Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | CLR_ALL_TASKS | Clear All Tasks. The EMSI_CQ_CTL.CLR_ALL_TASKS bit can only be written when the control- ler is halted. The EMSI_CQ_CTL.CLR_ALL_TASKS bit does not clear tasks in the eMMC device. The software has to use the CMDQ_TASK_MGMT command (CMD48) separately to clear device's queue. | Clear All Tasks. The EMSI_CQ_CTL.CLR_ALL_TASKS bit can only be written when the control- ler is halted. The EMSI_CQ_CTL.CLR_ALL_TASKS bit does not clear tasks in the eMMC device. The software has to use the CMDQ_TASK_MGMT command (CMD48) separately to clear device's queue.                                                                                                                                                                                                                                                                                                                      |
| 8 (R/W)            | CLR_ALL_TASKS | 0                                                                                                                                                                                                                                                                                 | Programming 0 Has No Effect                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 8 (R/W)            | CLR_ALL_TASKS | 1                                                                                                                                                                                                                                                                                 | Clears All the Tasks in the Controller                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | HALT          | Halt Request and Resume. The EMSI_CQ_CTL.HALT bit provides a halt request and resume.                                                                                                                                                                                             | Halt Request and Resume. The EMSI_CQ_CTL.HALT bit provides a halt request and resume.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |               | 0                                                                                                                                                                                                                                                                                 | Software writes 0 to this bit to exit from the halt state and resume CQE activity.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |               | 1                                                                                                                                                                                                                                                                                 | Software writes 1 to the EMSI_CQ_CTL.HALT bit when it wants to acquire software control over the eMMC bus and to disable CQE from issuing command on the bus. For example, issuing a Discard Task com- mand (CMDQ_TASK_MGMT). When the software writes 1, CQE completes the ongoing task (if any in progress). After the task is completed and the CQE is in idle state, CQE does not issue new commands and indi- cates to the software by setting the EMSI_CQ_CTL.HALT bit to 1. The software can poll on the EMSI_CQ_CTL.HALT bit until it is set (=1) and only then send commands on the eMMC bus. |

## Device Pending Tasks Register

The EMSI\_CQ\_DPT register maintains the list of tasks that are queued into device and are awaiting execution completion.

Figure 18-62: EMSI\_CQ\_DPT Register Diagram

<!-- image -->

Table 18-40: EMSI\_CQ\_DPT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | DPT        | Device-Pending Tasks. Each of the 32 bits are bit mapped to the 32 tasks. - Bit-N(1): Task-N has been successfully queued into the device and is awaiting execu- tion - Bit-N(0): Task-N is not yet queued Bit n of this register is set if and only if QUEUED_TASK_PARAMS (CMD44) and QUEUED_TASK_ADDRESS (CMD45) were sent for this specific task and if this task has not been executed. The controller sets the EMSI_CQ_DPT.DPT bit after receiving a successful response for CMD45. CQE clears the EMSI_CQ_DPT.DPT bit after the task has completed execution. Software reads this register in the task-discard procedure to determine if the task is queued in the device. |

## Device Queue Status Register

The EMSI\_CQ\_DQSTAT register stores the most recent value of the device's queue status.

Figure 18-63: EMSI\_CQ\_DQSTAT Register Diagram

<!-- image -->

Table 18-41: EMSI\_CQ\_DQSTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Device Queue Status. Each of the EMSI_CQ_DQSTAT.VALUE bits are bit mapped to the 32 tasks as fol- lows: - Bit-N(1): Device has marked task Nas ready for execution - Bit-N(0): Task-N is not ready for execution. This task may be pending in the device or not submitted. The eMSI controller updates this register with response of the device queue status command. |

## Command Queuing Interrupt Coalescing Register

The EMSI\_CQ\_IC register controls and configures the interrupt coalescing feature.

Figure 18-64: EMSI\_CQ\_IC Register Diagram

<!-- image -->

Table 18-42: EMSI\_CQ\_IC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31                 | INTC_EN    | Interrupt Coalescing Enable.                                                                                                                                                                 | Interrupt Coalescing Enable.                                                                                                                                                                 |
| (R/W)              |            | 0                                                                                                                                                                                            | Interrupt Coalescing Mechanism is Disabled (Default)                                                                                                                                         |
|                    |            | 1                                                                                                                                                                                            | Interrupt Coalescing Mechanism is Active. Interrupts are Counted and Timed, and Coalesced Interrupts are Generated                                                                           |
| 20 (R/NW)          | INTC_STAT  | Interrupt Coalescing Status. The EMSI_CQ_IC.INTC_STAT bit indicates to the software whether any tasks (with INT=0) have completed and counted towards interrupt coalescing (that is, this is | Interrupt Coalescing Status. The EMSI_CQ_IC.INTC_STAT bit indicates to the software whether any tasks (with INT=0) have completed and counted towards interrupt coalescing (that is, this is |
|                    |            | 0                                                                                                                                                                                            | INT0 Task Completions Have Not Occurred Since Last Counter Reset (INTC Counter == 0)                                                                                                         |
|                    |            | 1                                                                                                                                                                                            | At Least One INT0 Task Completion Has Been Count- ed (INTC Counter > 0)                                                                                                                      |
| 16 (RX/W)          | INTC_RST   | Counter and Timer Reset. When the host driver writes 1 to the EMSI_CQ_IC.INTC_RST , the interrupt coa- lescing timer and counter are reset.                                                  | Counter and Timer Reset. When the host driver writes 1 to the EMSI_CQ_IC.INTC_RST , the interrupt coa- lescing timer and counter are reset.                                                  |
|                    |            | 0                                                                                                                                                                                            | No Effect                                                                                                                                                                                    |
|                    |            | 1                                                                                                                                                                                            | Interrupt Coalescing Timer and Counter are Reset                                                                                                                                             |

Table 18-42: EMSI\_CQ\_IC Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (RX/W)          | INTC_TH_WEN  | Interrupt Coalescing Counter Threshold Write Enable. When software writes 1 to the EMSI_CQ_IC.INTC_TH_WEN bit, the value EMSI_CQ_IC.INTC_TH is updated with the contents written on the same cycle.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 15 (RX/W)          | INTC_TH_WEN  | 0 Clears INTC_TH_WEN                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 12:8 (RX/W)        | INTC_TH      | Interrupt Coalescing Counter Threshold. Software uses the EMSI_CQ_IC.INTC_TH field to configure the number of task completions (only tasks with INT=0 in the task descriptor), which are required in or- der to generate an interrupt. Counter Operation: As data transfer tasks with INT=0 complete, they are counted by CQE. The counter is reset by software during the interrupt service routine. The coun- ter stops counting when it reaches the value configured in EMSI_CQ_IC.INTC_TH , and generates interrupt. - 0x0: Interrupt coalescing feature disabled - 0x1: Interrupt coalescing interrupt generated after 1 task when INT=0 completes - 0x2: Interrupt coalescing interrupt generated after 2 tasks when INT=0 completes - ........ - 0x1f: Interrupt coalescing interrupt generated after 31 tasks when INT=0 completes To write to this field, the EMSI_CQ_IC.INTC_TH_WEN bit must be set during the same write operation. |
| 7 (RX/W)           | TOUT_VAL_WEN | Threshold Value Write Enable. When software writes 1 to the EMSI_CQ_IC.TOUT_VAL_WEN bit, the value in the EMSI_CQ_IC.TOUT_VAL bit field is updated with the contents written on the same cycle.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 7 (RX/W)           | TOUT_VAL_WEN | 0 Clears TOUT_VAL_WEN                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 18-42: EMSI\_CQ\_IC Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:0 (R/W)          | TOUT_VAL   | Interrupt Coalescing Timeout Value. Software uses the EMSI_CQ_IC.TOUT_VAL field to configure the maximum time allowed between the completion of a task on the bus and the generation of an inter- rupt. Timer Operation: The timer is reset by software during the interrupt service routine. It starts running when the first data transfer task with INT=0 is completed, after the tim- er was reset. When the timer reaches the value configured in ICTOVAL field, it gener- ates an interrupt and stops. The timer's unit is equal to 1024 clock periods of the clock whose frequency is speci- fied in the EMSI_CQ_CAP.ITCFVAL bit field. - 0x0: Timer is disabled. Timeout-based interrupt is not generated - 0x1: Timeout on 01x1024 cycles of timer clock frequency - 0x2: Timeout on 02x1024 cycles of timer clock frequency - ........ - 0x7f: Timeout on 127x1024 cycles of timer clock frequency In order to write to this field, the EMSI_CQ_IC.TOUT_VAL_WEN bit must be set at the same write operation. |

## Command Queuing Interrupt Status Register

The EMSI\_CQ\_ISTAT register indicates pending interrupts that require service. Each bit in this register is asserted in response to a specific event, only if the respective bit is set in the EMSI\_CQ\_ISTAT\_EN register.

Figure 18-65: EMSI\_CQ\_ISTAT Register Diagram

<!-- image -->

Table 18-43: EMSI\_CQ\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W1C)          | TCL        | Task Cleared Interrupt. The EMSI_CQ_ISTAT.TCL bit is asserted (if EMSI_CQ_ISTAT_EN.TCL =1) when a task clear operation is completed by the CQE. The completed task clear opera- tion is either an individual task clear (by writing EMSI_CQ_TCLR ) or clearing of all tasks (by writing EMSI_CQ_CTL ). A value of 1 clears this status bit.                         | Task Cleared Interrupt. The EMSI_CQ_ISTAT.TCL bit is asserted (if EMSI_CQ_ISTAT_EN.TCL =1) when a task clear operation is completed by the CQE. The completed task clear opera- tion is either an individual task clear (by writing EMSI_CQ_TCLR ) or clearing of all tasks (by writing EMSI_CQ_CTL ). A value of 1 clears this status bit.                         |
| 2 (R/W1C)          | RED        | Response Error Detected Interrupt. The EMSI_CQ_ISTAT.RED status bit is asserted (if EMSI_CQ_ISTAT_EN.RED =1) when a response is received with an error bit set in the device status field. Config- ure the EMSI_CQ_RMEM register to identify the device status bit fields that may trig- ger an interrupt and that are masked. A value of 1 clears this status bit. | Response Error Detected Interrupt. The EMSI_CQ_ISTAT.RED status bit is asserted (if EMSI_CQ_ISTAT_EN.RED =1) when a response is received with an error bit set in the device status field. Config- ure the EMSI_CQ_RMEM register to identify the device status bit fields that may trig- ger an interrupt and that are masked. A value of 1 clears this status bit. |
| 2 (R/W1C)          | RED        | 0                                                                                                                                                                                                                                                                                                                                                                   | RED Interrupt is Not Set                                                                                                                                                                                                                                                                                                                                            |
| 2 (R/W1C)          | RED        | 1                                                                                                                                                                                                                                                                                                                                                                   | RED Interrupt is Set                                                                                                                                                                                                                                                                                                                                                |

Table 18-43: EMSI\_CQ\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | TCC        | Task Complete Interrupt. The EMSI_CQ_ISTAT.TCC status bit is asserted (if EMSI_CQ_ISTAT_EN.TCC =1) when at least one of the following conditions are met: - A task is completed and the INT bit is set in its Task Descriptor - Interrupt caused by Interrupt Coalescing logic due to timeout - Interrupt Coalescing logic reached the configured threshold A value of 1 clears this status bit. | Task Complete Interrupt. The EMSI_CQ_ISTAT.TCC status bit is asserted (if EMSI_CQ_ISTAT_EN.TCC =1) when at least one of the following conditions are met: - A task is completed and the INT bit is set in its Task Descriptor - Interrupt caused by Interrupt Coalescing logic due to timeout - Interrupt Coalescing logic reached the configured threshold A value of 1 clears this status bit. |
| 0 (R/W1C)          | HAC        | Halt Complete Interrupt. The EMSI_CQ_ISTAT.HAC status bit is asserted (only if EMSI_CQ_ISTAT_EN.HAC =1) when the EMSI_CQ_CTL.HALT bit transitions from 0 to 1 indicating that the eMSI controller has completed its current ongoing task and has entered the halt state. A value of 1 clears this status bit.                                                                                    | Halt Complete Interrupt. The EMSI_CQ_ISTAT.HAC status bit is asserted (only if EMSI_CQ_ISTAT_EN.HAC =1) when the EMSI_CQ_CTL.HALT bit transitions from 0 to 1 indicating that the eMSI controller has completed its current ongoing task and has entered the halt state. A value of 1 clears this status bit.                                                                                    |
| 0 (R/W1C)          | HAC        | 0                                                                                                                                                                                                                                                                                                                                                                                                | HAC Interrupt is Not Set                                                                                                                                                                                                                                                                                                                                                                         |
| 0 (R/W1C)          | HAC        | 1                                                                                                                                                                                                                                                                                                                                                                                                | HAC Interrupt is Set                                                                                                                                                                                                                                                                                                                                                                             |

## Command Queuing Interrupt Status Enable Register

The EMSI\_CQ\_ISTAT\_EN register enables and disables the reporting of the corresponding interrupt status to the host software in the EMSI\_CQ\_ISTAT register. When a bit is set (1) and the corresponding interrupt condition is active, then the bit in the EMSI\_CQ\_ISTAT register is asserted. Interrupt sources that are disabled (when 0) are not indicated in the EMSI\_CQ\_ISTAT register. This register is bit-index matched to the EMSI\_CQ\_ISTAT register.

Figure 18-66: EMSI\_CQ\_ISTAT\_EN Register Diagram

<!-- image -->

Table 18-44: EMSI\_CQ\_ISTAT\_EN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 3 (R/W)            | TCL        | Task Cleared Interrupt. Task cleared interrupt status enable.                       |
| 2 (R/W)            | RED        | Response Error Detected Interrupt. Response error detected interrupt status enable. |
| 1 (R/W)            | TCC        | tive Task Complete Interrupt. Task complete interrupt status enable.                |
| 1 (R/W)            | TCC        | 0 CQIS.TCC is Disabled                                                              |
| 1 (R/W)            | TCC        | 1 CQIS.TCC is Set When Its Interrupt Condition is Ac- tive                          |

Table 18-44: EMSI\_CQ\_ISTAT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                    |
|--------------------|------------|------------------------------------------------------------|
| 0                  | HAC        | Halt Complete Interrupt Status Enable.                     |
| (R/W)              |            | 0 CQIS.HAC is Disabled                                     |
|                    |            | 1 CQIS.HAC is Set When Its Interrupt Condition is Ac- tive |

## Command Queuing Interrupt Signal Enable Register

The EMSI\_CQ\_ISTAT\_INTEN register enables and disables the generation of interrupts to host software. When a bit is set and the corresponding bit in EMSI\_CQ\_ISTAT is set, then an interrupt is generated. This register is bitindex matched to the EMSI\_CQ\_ISTAT register.

Figure 18-67: EMSI\_CQ\_ISTAT\_INTEN Register Diagram

<!-- image -->

Table 18-45: EMSI\_CQ\_ISTAT\_INTEN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------|
| 3 (R/W)            | TCL        | Task Cleared Interrupt. Task cleared interrupt signal enable.                       | Task Cleared Interrupt. Task cleared interrupt signal enable.                       |
| 3 (R/W)            | TCL        | 0                                                                                   | CQIS.TCL Interrupt Signal Generation is Disabled                                    |
| 3 (R/W)            | TCL        | 1                                                                                   | CQIS.TCL Interrupt Signal Generation is Active                                      |
| 2 (R/W)            | RED        | Response Error Detected Interrupt. Response error detected interrupt signal enable. | Response Error Detected Interrupt. Response error detected interrupt signal enable. |
| 2 (R/W)            | RED        | 0                                                                                   | CQIS.RED Interrupt Signal Generation is Disabled                                    |
| 2 (R/W)            | RED        | 1                                                                                   | CQIS.RED Interrupt Signal Generation is Active                                      |
| 1                  | TCC        | Task Complete Interrupt.                                                            | Task Complete Interrupt.                                                            |
| (R/W)              |            | Task complete interrupt signal enable.                                              | Task complete interrupt signal enable.                                              |
|                    |            | 0                                                                                   | CQIS.TCC Interrupt Signal Generation is Disabled                                    |
|                    |            | 1                                                                                   | CQIS.TCC Interrupt Signal Generation is Active                                      |
| 0                  | HAC        | Halt Complete Interrupt. Halt complete interrupt signal enable.                     | Halt Complete Interrupt. Halt complete interrupt signal enable.                     |
| (R/W)              |            | 0                                                                                   | CQIS.HAC Interrupt Signal Generation is Disabled                                    |
|                    |            | 1                                                                                   | CQIS.HAC Interrupt Signal Generation is Active                                      |

## Command Response Mode Error Mask Register

The EMSI\_CQ\_RMEM register controls the generation of the response error detect (RED) interrupt. Only the bits enabled here can contribute to RED.

Figure 18-68: EMSI\_CQ\_RMEM Register Diagram

<!-- image -->

Table 18-46: EMSI\_CQ\_RMEM Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Interrupt Mask for Device Status R1/R1b Responses. The EMSI_CQ_RMEM.VALUE bits are bit-mapped to the device response. The EMSI_CQ_RMEM.VALUE bits are used as an interrupt mask on the device status field that is received in R1/R1b responses. - 1: When a R1/R1b response is received, with a bit i in the device status set, a RED interrupt is generated. - 0: When a R1/R1b response is received, bit i in the device status is ignored. The reset value of this register is set to trigger an interrupt on all "Error" type bits in the device status. Note: Responses to CMD13 (SQS) encode the QSR so that they are ignored by this logic. |

## CQ Send Status Configuration 1 Register

The EMSI\_CQ\_SSCFG1 register is used for removing an outstanding task in the CQE. The register controls when SEND\_QUEUE\_STATUS commands are sent.

Figure 18-69: EMSI\_CQ\_SSCFG1 Register Diagram

<!-- image -->

Table 18-47: EMSI\_CQ\_SSCFG1 Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 19:16 (R/W)        | SQSCMD_BLK_CNT | SQS Command Sent During Data Transfer. The EMSI_CQ_SSCFG1.SQSCMD_BLK_CNT field indicates when SQS CMDis sent while data transfer is in progress. A value of 'n' indicates that CQE sends status command on the CMDline, during the transfer of data block BLOCK_CNT-n, on the data lines, where BLOCK_CNT is the number of blocks in the current transaction. - 0x0: SEND_QUEUE_STATUS (CMD13) command is not sent during the transac- tion. Instead, it is sent only when the data lines are idle. - 0x1: SEND_QUEUE_STATUS command is to be sent during the last block of the transaction. - 0x2: SEND_QUEUE_STATUS command when last 2 blocks are pending. - 0x3: SEND_QUEUE_STATUS command when last 3 blocks are pending. - ........ - 0xf: SEND_QUEUE_STATUS command when last 15 blocks are pending. The EMSI_CQ_SSCFG1.SQSCMD_BLK_CNT bits should be programmed only when EMSI_CQ_CFG.CQ_EN is = 0. |

Table 18-47: EMSI\_CQ\_SSCFG1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | SQSCMD_IDLE_TMR | Polling Period Timer. The EMSI_CQ_SSCFG1.SQSCMD_IDLE_TMR bit field configures the polling pe- riod to be used when using periodic SEND_QUEUE_STATUS (CMD13) polling. Periodic polling is used when tasks are pending in the device, but no data transfer is in progress. When a SEND_QUEUE_STATUS response indicates that no task is ready for execution, the CQE counts the configured time until it issues the next SEND_QUEUE_STATUS. Timer units are clock periods of the clock whose frequency is specified in the EMSI_CQ_CAP.ITCFVAL bits. The minimum value is 0001h (1 clock period) and the maximum value is FFFFh (65535 clock periods). For example, a EMSI_CQ_CAP.ITCFVAL field value of 0 indicates a 19.2 MHz clock frequency (period = 52.08 ns). If the setting in EMSI_CQ_SSCFG1.SQSCMD_IDLE_TMR is 1000h, the calculated polling period is 4096*52.08 ns= 213.33 ns. The value in this field should be programmed only when EMSI_CQ_CFG.CQ_EN = 0. |

## CQ Send Status Configuration 2 Register

The EMSI\_CQ\_SSCFG2 register is used for configuring the RCA field in SEND\_QUEUE\_STATUS command argument.

Figure 18-70: EMSI\_CQ\_SSCFG2 Register Diagram

<!-- image -->

Table 18-48: EMSI\_CQ\_SSCFG2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | SQSCMD_RCA | CMD13 RCA Contents. The EMSI_CQ_SSCFG2.SQSCMD_RCA bit field provides CQE with the contents of the 16-bit RCA field in SEND_QUEUE_STATUS (CMD13) command argument. The CQE copies this field to bits 31:16 of the argument when transmitting the SEND_ QUEUE_STATUS (CMD13) command. |

## Command Queuing Task Clear Register

The EMSI\_CQ\_TCLR register is used for removing an outstanding task in the CQE. The register must be used only when the CQE is in a halt state.

Figure 18-71: EMSI\_CQ\_TCLR Register Diagram

<!-- image -->

Table 18-49: EMSI\_CQ\_TCLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TCLR       | Task Clear. Writing 1 to bit n of the EMSI_CQ_TCLR.TCLR bit field orders the CQE to clear a task that the software has previously issued. This bit can only be written when the CQE is in the halt state as indicated in the EMSI_CQ_CTL.HALT bit. When software writes 1 to a bit in this register, CQE up- dates the value to 1, and starts clearing the data structures related to the task. The CQE clears the bit fields (sets a value of 0) in the EMSI_CQ_TCLR register and in the EMSI_CQ_TDBR register once the clear operation is complete. Software must poll on this register until it is cleared to verify that a clear operation was done. |

## Command Queuing Task Completion Notification Register

The EMSI\_CQ\_TCN register is used by the CQE to notify software about completed tasks.

Figure 18-72: EMSI\_CQ\_TCN Register Diagram

<!-- image -->

Table 18-50: EMSI\_CQ\_TCN Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W1C)       | TCN        | Task Completion Notification. Each of the EMSI_CQ_TCN.TCN bits are bit mapped to the 32 tasks. - Bit-N(1): Task-N has completed execution (with success or errors) - Bit-N(0): Task-N has not completed, could be pending or not submitted. On task completion, software may read this bit field to know tasks that have complet- ed. After reading this register, software may clear the relevant bit fields by writing 1 to the corresponding bits. |

## Command Queuing DoorBell Register

The EMSI\_CQ\_TDBR register, causes software to trigger the CQE to process a new task.

Figure 18-73: EMSI\_CQ\_TDBR Register Diagram

<!-- image -->

Table 18-51: EMSI\_CQ\_TDBR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | DBR        | Command Queuing Door Bell. The software configures EMSI_CQ_TDL_BADDR and enables EMSI_CQ_TDBR EMSI_CQ_CFG CQE in CQCFG before using this register. Writing 1 to bit n of this register triggers the CQE to start processing the task encoded in slot n of the TDL. Writing 0 by the software does not have any impact on the hardware, and does not change the value of the register bit. The CQE always processes tasks according to the order submitted to the list by write transactions. The CQE processes data transfer tasks by reading the task descriptor and sending QUEUED_TASK_PARAMS (CMD44) and QUEUED_TASK_ADDRESS (CMD45) commands to the device. CQE processes DCMDtasks (in slot #31, when enabled) by reading the task descriptor, and generating the command encoded by its index and argument. The corresponding bit is cleared to 0 by CQE in one of the following events: - A task execution is completed (with success or error). - The task is cleared using EMSI_CQ_TCLR register. - All tasks are cleared using EMSI_CQ_CTL register. - CQE is disabled using EMSI_CQ_CFG register. Software may initiate multiple tasks at the same time (batch submission) by writing 1 to multiple bits of this register in the same transaction. In the case of batch submis- sion, CQE processes the tasks in order of the task index, starting with the lowest index. If one or more tasks in the batch are marked with QBR, the ordering of execution is based on said processing order. |

## Command Queuing Task Descriptor List Base Address Register

The EMSI\_CQ\_TDL\_BADDR register is used for configuring the lower 32 bits of the byte address of the head of the Task Descriptor List in the host memory.

Figure 18-74: EMSI\_CQ\_TDL\_BADDR Register Diagram

<!-- image -->

Table 18-52: EMSI\_CQ\_TDL\_BADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TDLBA      | Task Descriptor Byte Address. The EMSI_CQ_TDL_BADDR.TDLBA bit field stores the LSB bits (31:0) of the byte address of the head of the task descriptor List in system memory. The size of the task descriptor list is 32 x (task descriptor size + transfer descriptor size) as configured by the host driver. This address is set on 1 KB boundary. The lower 10 bits of this register are cleared (=0) by the software and are ignored by CQE. |

## CQ Task Error Information Register

The EMSI\_CQ\_TERRINFO register is updated by the CQE when an error occurs on data or a command related to a task activity. When such an error is detected by the CQE or indicated by the eMSI controller, the CQE stores task IDs and indices of commands that were executed on the command line and data lines when the error occurred in this register.

Software must use this information in the error recovery procedure.

Figure 18-75: EMSI\_CQ\_TERRINFO Register Diagram

<!-- image -->

Table 18-53: EMSI\_CQ\_TERRINFO Register Fields

| Bit No. (Access)   | Bit Name                | Description/Enumeration                                                                                                                                                  |
|--------------------|-------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/NW)          | TRANS_ERR_FIELDS_VA LID | Data Transfer Transaction Error. The EMSI_CQ_TERRINFO.TRANS_ERR_FIELDS_VALID bit is updated when an error is detected while a data transfer transaction was in progress. |
| 31 (R/NW)          | TRANS_ERR_FIELDS_VA LID | 0 Ignore Contents of TRANS_ERR_TASKID and TRANS_ERR_CMD_INDX                                                                                                             |
| 28:24 (R/NW)       | TRANS_ERR_TASKID        | Data Transfer Error Task ID. The EMSI_CQ_TERRINFO.TRANS_ERR_TASKID bit field captures the ID of the task that was executed and whose data transfer has errors.           |
| 21:16 (R/NW)       | TRANS_ERR_CMD_IND X     | Index of Command Data Errors. The EMSI_CQ_TERRINFO.TRANS_ERR_CMD_INDX bit field captures the index of the command that was executed and whose data transfer has errors.  |

Table 18-53: EMSI\_CQ\_TERRINFO Register Fields (Continued)

| Bit No. (Access)   | Bit Name                | Description/Enumeration                                                                                                                                                    |
|--------------------|-------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | RESP_ERR_FIELDS_VAL- ID | Error Detected While Command in Progress. The EMSI_CQ_TERRINFO.RESP_ERR_FIELDS_VALID bit is updated when an error is detected while a command transaction was in progress. |
| 15 (R/NW)          | RESP_ERR_FIELDS_VAL- ID | 0 Ignore Contents of RESP_ERR_TASKID and RESP_ERR_CMD_INDX                                                                                                                 |
| 12:8 (R/NW)        | RESP_ERR_TASKID         | Error Task ID. The EMSI_CQ_TERRINFO.RESP_ERR_TASKID bit field captures the ID of the task which was executed on the command line when the error occurred.                  |
| 5:0 (R/NW)         | RESP_ERR_CMD_INDX       | Command Line Command Index. The EMSI_CQ_TERRINFO.RESP_ERR_CMD_INDX bits capture the index of the command that was executed on the command line when the error occurred.    |

## EMSI Control 1 Register

The EMSI\_CTL1 register is used to control the operation of the eMSI controller. This register is applicable for eMMC mode.

Figure 18-76: EMSI\_CTL1 Register Diagram

<!-- image -->

Table 18-54: EMSI\_CTL1 Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                      |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/W)            | EXT_DAT_XFER | Extended Data Transfer Width. The EMSI_CTL1.EXT_DAT_XFER bit controls 8-bit bus width mode of an eMMC device.                                                                                                                                                                                                                | Extended Data Transfer Width. The EMSI_CTL1.EXT_DAT_XFER bit controls 8-bit bus width mode of an eMMC device.                                                                                                                                                                                                                |
| 5 (R/W)            | EXT_DAT_XFER | 0                                                                                                                                                                                                                                                                                                                            | Bus Width is Selected by the Data Transfer Width                                                                                                                                                                                                                                                                             |
| 5 (R/W)            | EXT_DAT_XFER | 1                                                                                                                                                                                                                                                                                                                            | 8-bit Bus Width                                                                                                                                                                                                                                                                                                              |
| 4:3 (R/W)          | DMA_SEL      | DMASelect. The EMSI_CTL1.DMA_SEL bit field is used to select the DMAtype. When EMSI_CTL2.HOST_VER4_EN =1: - 0x0: SDMA is selected - 0x1: Reserved - 0x2: ADMA2 is selected - 0x3: ADMA2 or ADMA3 is selected When EMSI_CTL2.HOST_VER4_EN =0: - 0x0: SDMA is selected - 0x1: Reserved - 0x2: 32-bit Address ADMA2 is selected | DMASelect. The EMSI_CTL1.DMA_SEL bit field is used to select the DMAtype. When EMSI_CTL2.HOST_VER4_EN =1: - 0x0: SDMA is selected - 0x1: Reserved - 0x2: ADMA2 is selected - 0x3: ADMA2 or ADMA3 is selected When EMSI_CTL2.HOST_VER4_EN =0: - 0x0: SDMA is selected - 0x1: Reserved - 0x2: 32-bit Address ADMA2 is selected |
| 4:3 (R/W)          | DMA_SEL      | 0                                                                                                                                                                                                                                                                                                                            | SDMA is Selected                                                                                                                                                                                                                                                                                                             |
| 4:3 (R/W)          | DMA_SEL      | 1                                                                                                                                                                                                                                                                                                                            | Reserved                                                                                                                                                                                                                                                                                                                     |
| 4:3 (R/W)          | DMA_SEL      | 2                                                                                                                                                                                                                                                                                                                            | ADMA2 is Selected                                                                                                                                                                                                                                                                                                            |
| 4:3 (R/W)          | DMA_SEL      | 3                                                                                                                                                                                                                                                                                                                            | ADMA2 or ADMA3 is Selected                                                                                                                                                                                                                                                                                                   |

Table 18-54: EMSI\_CTL1 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                            |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | HIGH_SPEED_EN  | High Speed Enable (eMMC Mode only). In eMMC mode, the EMSI_CTL1.HIGH_SPEED_EN bit is used to determine the selection of the preset value for high speed mode. This bit is set = 1 when the eMSI bus clock is 0-50MHz. This bit is cleared = 0 when the eMSI bus clock frequency is 0-25MHz.                                        |
| 2 (R/W)            | HIGH_SPEED_EN  | 0 Normal Speed Mode (eMSI bus clock frequency 0-25 MHz)                                                                                                                                                                                                                                                                            |
| 2 (R/W)            | HIGH_SPEED_EN  | 1 High Speed Mode (eMSI bus clock frequency 0-50 MHz)                                                                                                                                                                                                                                                                              |
| 1 (R/W)            | DAT_XFER_WIDTH | Data Transfer Width. For eMMC mode,the EMSI_CTL1.DAT_XFER_WIDTH bit selects the data transfer width of the eMSI controller. The host driver sets it to match the data width of the eMMC device.                                                                                                                                    |
| 1 (R/W)            | DAT_XFER_WIDTH | 0 1-bit Mode                                                                                                                                                                                                                                                                                                                       |
| 1 (R/W)            | DAT_XFER_WIDTH | 1 4-bit Mode                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W)            | LED_CTRL       | LED Control. The EMSI_CTL1.LED_CTRL bit is used to caution the user not to remove the card while the eMMC card is being accessed. The value is reflected on the EM- SI_LED_CONTROL signal. The EMSI_CTL1.LED_CTRL bit can be set to 1 by host driver at the start of data transfer and cleared when data transfer is complete. Off |
| 0 (R/W)            | LED_CTRL       | 0 LED                                                                                                                                                                                                                                                                                                                              |
| 0 (R/W)            | LED_CTRL       | 1 LED On                                                                                                                                                                                                                                                                                                                           |

## EMSI Control 2 Register

The EMSI\_CTL2 register is used to control how the eMSI controller operates. This register is applicable for eMMC mode.

Figure 18-77: EMSI\_CTL2 Register Diagram

<!-- image -->

Table 18-55: EMSI\_CTL2 Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|---------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | PRESET_VAL_EN | Preset Value Enable. The EMSI_CTL2.PRESET_VAL_EN bit enables automatic selection of the eMSI bus clock frequency using the preset value registers. When the EMSI_CTL2.PRESET_VAL_EN bit is set, the eMSI bus clock frequency generation (frequency select) is performed by the controller. These values are selected from the set of preset value registers based on the selected speed mode. | Preset Value Enable. The EMSI_CTL2.PRESET_VAL_EN bit enables automatic selection of the eMSI bus clock frequency using the preset value registers. When the EMSI_CTL2.PRESET_VAL_EN bit is set, the eMSI bus clock frequency generation (frequency select) is performed by the controller. These values are selected from the set of preset value registers based on the selected speed mode. |
| 15 (R/W)           | PRESET_VAL_EN | 0                                                                                                                                                                                                                                                                                                                                                                                             | eMSI bus clock controlled by Host Driver                                                                                                                                                                                                                                                                                                                                                      |
| 15 (R/W)           | PRESET_VAL_EN | 1                                                                                                                                                                                                                                                                                                                                                                                             | Automatic Selection by Preset Value are Enabled                                                                                                                                                                                                                                                                                                                                               |
| 12 (R/W)           | HOST_VER4_EN  | Host Version 4 Enable. The EMSI_CTL2.HOST_VER4_EN bit selects either Version 3.00 compatible mode or Version 4 mode. The functions of the following fields are modified for Host Version 4 mode. - SDMA Address: SDMA uses the EMSI_ADMA_ADDR_LO register instead of the                                                                                                                      | Host Version 4 Enable. The EMSI_CTL2.HOST_VER4_EN bit selects either Version 3.00 compatible mode or Version 4 mode. The functions of the following fields are modified for Host Version 4 mode. - SDMA Address: SDMA uses the EMSI_ADMA_ADDR_LO register instead of the                                                                                                                      |
| 12 (R/W)           | HOST_VER4_EN  | 0                                                                                                                                                                                                                                                                                                                                                                                             | Version 3.00 Compatible Mode                                                                                                                                                                                                                                                                                                                                                                  |
| 12 (R/W)           | HOST_VER4_EN  | 1                                                                                                                                                                                                                                                                                                                                                                                             | Version 4 Mode                                                                                                                                                                                                                                                                                                                                                                                |

Table 18-55: EMSI\_CTL2 Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                            | Description/Enumeration                                                                                                                                                            |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | CMD23_EN       | CMD23 Enable. If the device supports CMD23, the EMSI_CTL2.CMD23_EN bit is set (=1). The EMSI_CTL2.CMD23_EN bit is used to select Auto CMD23 or Auto CMD12 for ADMA3 data transfer. | CMD23 Enable. If the device supports CMD23, the EMSI_CTL2.CMD23_EN bit is set (=1). The EMSI_CTL2.CMD23_EN bit is used to select Auto CMD23 or Auto CMD12 for ADMA3 data transfer. |
| 11 (R/W)           | CMD23_EN       | 0                                                                                                                                                                                  | Auto CMD23 is Disabled                                                                                                                                                             |
| 11 (R/W)           | CMD23_EN       | 1                                                                                                                                                                                  | Auto CMD23 is Enabled                                                                                                                                                              |
| 10 (R/W)           | ADMA2_LEN_MODE | ADMA2 Length Mode. The EMSI_CTL2.ADMA2_LEN_MODE bit selects ADMA2 Length mode to be ei- ther 16-bit or 26-bit.                                                                     | ADMA2 Length Mode. The EMSI_CTL2.ADMA2_LEN_MODE bit selects ADMA2 Length mode to be ei- ther 16-bit or 26-bit.                                                                     |
| 10 (R/W)           | ADMA2_LEN_MODE | 0                                                                                                                                                                                  | 16-bit Data Length Mode                                                                                                                                                            |
| 10 (R/W)           | ADMA2_LEN_MODE | 1                                                                                                                                                                                  | 26-bit Data Length Mode                                                                                                                                                            |
| 2:0 (R/W)          | EMMC_MODE_SEL  | eMMC Speed Mode Select. In eMMC mode, the EMSI_CTL2.EMMC_MODE_SEL bits are used to select the speed mode.                                                                          | eMMC Speed Mode Select. In eMMC mode, the EMSI_CTL2.EMMC_MODE_SEL bits are used to select the speed mode.                                                                          |
|                    |                | 0                                                                                                                                                                                  | Legacy                                                                                                                                                                             |
|                    |                | 1                                                                                                                                                                                  | High Speed SDR                                                                                                                                                                     |
|                    |                | 2                                                                                                                                                                                  | Reserved                                                                                                                                                                           |
|                    |                | 3                                                                                                                                                                                  | Reserved                                                                                                                                                                           |
|                    |                | 4                                                                                                                                                                                  | High Speed DDR                                                                                                                                                                     |
|                    |                | 5                                                                                                                                                                                  | Reserved                                                                                                                                                                           |
|                    |                | 6                                                                                                                                                                                  | Reserved                                                                                                                                                                           |
|                    |                | 7                                                                                                                                                                                  | Reserved                                                                                                                                                                           |

## SD Host Controller Specification Version

The EMSI\_HOST\_CNTRL\_VERS register is used to indicate the supported SD host controller specification version number.

Figure 18-78: EMSI\_HOST\_CNTRL\_VERS Register Diagram

<!-- image -->

Table 18-56: EMSI\_HOST\_CNTRL\_VERS Register Fields

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                             |
|--------------------|------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/NW)         | SPEC_VERSION_NUM | Specification Version Number. The EMSI_HOST_CNTRL_VERS.SPEC_VERSION_NUM bits indicate the SD host controller specification version. The upper and lower 4-bits indicate the version. Values 0x06-0xFF are reserved. | Specification Version Number. The EMSI_HOST_CNTRL_VERS.SPEC_VERSION_NUM bits indicate the SD host controller specification version. The upper and lower 4-bits indicate the version. Values 0x06-0xFF are reserved. |
|                    |                  | 0                                                                                                                                                                                                                   | SD Host Controller Specification Version 1.00                                                                                                                                                                       |
|                    |                  | 1                                                                                                                                                                                                                   | SD Host Controller Specification Version 2.00                                                                                                                                                                       |
|                    |                  | 2                                                                                                                                                                                                                   | SD Host Controller Specification Version 3.00                                                                                                                                                                       |
|                    |                  | 3                                                                                                                                                                                                                   | SD Host Controller Specification Version 4.00                                                                                                                                                                       |
|                    |                  | 4                                                                                                                                                                                                                   | SD Host Controller Specification Version 4.10                                                                                                                                                                       |
|                    |                  | 5                                                                                                                                                                                                                   | SD Host Controller Specification Version 4.20                                                                                                                                                                       |

## eMMC Control Register

The EMSI\_EMMC\_CTL register is used to control the eMMC operation.

Figure 18-79: EMSI\_EMMC\_CTL Register Diagram

<!-- image -->

Table 18-57: EMSI\_EMMC\_CTL Register Fields

| Bit No. (Access)   | Bit Name               | Description/Enumeration                                                                                                                                              | Description/Enumeration                                                                                                                                              |
|--------------------|------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 10 (R/W)           | CQE_PREFETCH_DISA- BLE | Enable or Disable CQE's PREFETCH Feature. The EMSI_EMMC_CTL.CQE_PREFETCH_DISABLE bit field allows software to disable the CQE's data prefetch feature when set (=1). | Enable or Disable CQE's PREFETCH Feature. The EMSI_EMMC_CTL.CQE_PREFETCH_DISABLE bit field allows software to disable the CQE's data prefetch feature when set (=1). |
| 10 (R/W)           | CQE_PREFETCH_DISA- BLE | 0                                                                                                                                                                    | CQE Can Prefetch Data for Successive WRITE Trans- fers and Pipeline Successive READ Transfers                                                                        |
| 10 (R/W)           | CQE_PREFETCH_DISA- BLE | 1                                                                                                                                                                    | Prefetch for WRITE and Pipeline for READ are Disa- bled                                                                                                              |
| 9 (R/W)            | CQE_ALGO_SEL           | Scheduler Algorithm Selected for Execution. The EMSI_EMMC_CTL.CQE_ALGO_SEL bit selects the algorithm used for select- ing one of the many ready tasks for execution. | Scheduler Algorithm Selected for Execution. The EMSI_EMMC_CTL.CQE_ALGO_SEL bit selects the algorithm used for select- ing one of the many ready tasks for execution. |
| 9 (R/W)            | CQE_ALGO_SEL           | 0                                                                                                                                                                    | Priority Based Reordering with FCFS to Resolve Equal Priority Tasks                                                                                                  |
| 9 (R/W)            | CQE_ALGO_SEL           | 1                                                                                                                                                                    | First Come First Serve, in the Order of DBR Rings                                                                                                                    |
| 3 (R/W)            | EMMC_RST_N_OE          | Output Enable Control for EMMCDevice Reset Signal. The EMSI_EMMC_CTL.EMMC_RST_N_OE bit field controls the devise reset for EMMCdevices.                              | Output Enable Control for EMMCDevice Reset Signal. The EMSI_EMMC_CTL.EMMC_RST_N_OE bit field controls the devise reset for EMMCdevices.                              |
| 3 (R/W)            | EMMC_RST_N_OE          | 0                                                                                                                                                                    | eMMC_rst_n_oe is 0                                                                                                                                                   |
| 3 (R/W)            | EMMC_RST_N_OE          | 1                                                                                                                                                                    | eMMC_rst_n_oe is 1                                                                                                                                                   |

Table 18-57: EMSI\_EMMC\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | EMMC_RST_N             | EMMCDevice Reset Signal Control. The EMSI_EMMC_CTL.EMMC_RST_N register field controls the RST signal of the eMSI controller.                                                                                                                                                                                                                                                                 | EMMCDevice Reset Signal Control. The EMSI_EMMC_CTL.EMMC_RST_N register field controls the RST signal of the eMSI controller.                                                                                                                                                                                                                                                                 |
| 2 (R/W)            | EMMC_RST_N             | 0                                                                                                                                                                                                                                                                                                                                                                                            | Reset to eMMC device asserted (active Low)                                                                                                                                                                                                                                                                                                                                                   |
| 2 (R/W)            | EMMC_RST_N             | 1                                                                                                                                                                                                                                                                                                                                                                                            | Reset to eMMC device is deasserted                                                                                                                                                                                                                                                                                                                                                           |
| 1 (R/W)            | DISABLE_DA- TA_CRC_CHK | Disable Data CRC Check. The EMSI_EMMC_CTL.DISABLE_DATA_CRC_CHK bit controls masking of CRC16 error for card write in eMMC mode. This is useful in bus testing (CMD19) for an eMMC device. In bus testing, an eMMC card does not send CRC status for a block, which may generate CRC error. This CRC error can be masked using the EMSI_EMMC_CTL.DISABLE_DATA_CRC_CHK bit during bus testing. | Disable Data CRC Check. The EMSI_EMMC_CTL.DISABLE_DATA_CRC_CHK bit controls masking of CRC16 error for card write in eMMC mode. This is useful in bus testing (CMD19) for an eMMC device. In bus testing, an eMMC card does not send CRC status for a block, which may generate CRC error. This CRC error can be masked using the EMSI_EMMC_CTL.DISABLE_DATA_CRC_CHK bit during bus testing. |
| 1 (R/W)            | DISABLE_DA- TA_CRC_CHK | 0                                                                                                                                                                                                                                                                                                                                                                                            | DATA CRC Check is Enabled                                                                                                                                                                                                                                                                                                                                                                    |
| 1 (R/W)            | DISABLE_DA- TA_CRC_CHK | 1                                                                                                                                                                                                                                                                                                                                                                                            | DATA CRC Check is Disabled                                                                                                                                                                                                                                                                                                                                                                   |
| 0 (R/W)            | CARD_IS_EMMC           | eMMC Device Present. The EMSI_EMMC_CTL.CARD_IS_EMMC bit indicates the type of device connect- ed. An application programs the EMSI_EMMC_CTL.CARD_IS_EMMC bit based on the device connected to the eMSI controller.                                                                                                                                                                           | eMMC Device Present. The EMSI_EMMC_CTL.CARD_IS_EMMC bit indicates the type of device connect- ed. An application programs the EMSI_EMMC_CTL.CARD_IS_EMMC bit based on the device connected to the eMSI controller.                                                                                                                                                                           |
| 0 (R/W)            | CARD_IS_EMMC           | 0                                                                                                                                                                                                                                                                                                                                                                                            | Card Connected to the eMSI controller is a Non- eMMC Card                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W)            | CARD_IS_EMMC           | 1                                                                                                                                                                                                                                                                                                                                                                                            | Card Connected to the eMSI controller is an eMMC card                                                                                                                                                                                                                                                                                                                                        |

## Error Interrupt Status Register

The EMSI\_ERR\_STAT register indicates error occurred during eMMC transactions. Writing a 1 clears the bit and writing a 0 retains the bit unchanged. Signals defined in this register can be enabled by the Error Interrupt Status Enable register, but not by the EMSI\_ERR\_STAT\_INTEN register. More than one status can be cleared with a single register write. This register is applicable for eMMC mode.

Figure 18-80: EMSI\_ERR\_STAT Register Diagram

<!-- image -->

Table 18-58: EMSI\_ERR\_STAT Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                          |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W1C)         | BOOT_ACK_ERR | Boot Acknowledgment Error. The EMSI_ERR_STAT.BOOT_ACK_ERR bit is set when there is a timeout for boot acknowledgment or when detecting boot ack status having a value other than 010. This is applicable only when boot acknowledgment is expected in eMMC mode. | Boot Acknowledgment Error. The EMSI_ERR_STAT.BOOT_ACK_ERR bit is set when there is a timeout for boot acknowledgment or when detecting boot ack status having a value other than 010. This is applicable only when boot acknowledgment is expected in eMMC mode. |
| 12 (R/W1C)         | BOOT_ACK_ERR | 0                                                                                                                                                                                                                                                                | No Error                                                                                                                                                                                                                                                         |
| 12 (R/W1C)         | BOOT_ACK_ERR | 1                                                                                                                                                                                                                                                                | Error                                                                                                                                                                                                                                                            |
| 11                 | RESP_ERR     | Response Error. Host Version 4.00 supports a response error check function to avoid overhead of re-                                                                                                                                                              | Response Error. Host Version 4.00 supports a response error check function to avoid overhead of re-                                                                                                                                                              |
| 11                 | RESP_ERR     | 0                                                                                                                                                                                                                                                                | No Error                                                                                                                                                                                                                                                         |
| 11                 | RESP_ERR     | 1                                                                                                                                                                                                                                                                | Error                                                                                                                                                                                                                                                            |

Table 18-58: EMSI\_ERR\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W1C)          | ADMA_ERR         | ADMA Error. The EMSI_ERR_STAT.ADMA_ERR bit is set when the eMSI controller detects er- ror during ADMA-based data transfer. The error could be due to following reasons: - Error response received from System bus - ADMA3, ADMA2 Descriptors invalid - CQE Task or Transfer descriptors invalid When the error occurs, the state of the AD- MAis saved in the ADMA Error Status register. In eMMC CQE mode: The eMSI controller generates this Interrupt when it detects an invalid descriptor data (Valid=0) at the ST_FDS state. An ADMA error state in the ADMA error status indi- cates that an error has occurred in ST_FDS state. The host driver may find that Valid bit is not set at the error descriptor. |
| 9 (R/W1C)          | ADMA_ERR         | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 8 (R/W1C)          | AUTO_CMD_ERR     | Auto CMDError. This error status is used by Auto CMD12 and Auto CMD23 in eMMC mode. The EMSI_ERR_STAT.AUTO_CMD_ERR bit is set when detecting that any of the bits D00 to D05 in Auto CMDError Status register has changed from 0 to 1. D07 is ef- fective in case of Auto CMD12. The Auto CMDerror status register is valid while the EMSI_ERR_STAT.AUTO_CMD_ERR bit is set (=1) and may be cleared by clearing the EMSI_ERR_STAT.AUTO_CMD_ERR bit.                                                                                                                                                                                                                                                                  |
| 8 (R/W1C)          | AUTO_CMD_ERR     | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 8 (R/W1C)          | AUTO_CMD_ERR     | 1 Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 6 (R/W1C)          | DATA_END_BIT_ERR | Data End Bit Error. The EMSI_ERR_STAT.DATA_END_BIT_ERR bit is set in eMMC mode either when detecting a 0 at the end bit position of read data that uses the DAT line or at the end bit position of the CRC status.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 6 (R/W1C)          | DATA_END_BIT_ERR | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 6 (R/W1C)          | DATA_END_BIT_ERR | 1 Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 5 (R/W1C)          | DATA_CRC_ERR     | Data CRC Error. The EMSI_ERR_STAT.DATA_CRC_ERR bit is set in eMMC mode when detect- ing a CRC error when transferring read data which uses the DAT line, when detecting the write CRC status having a value of other than 010, or on a write CRC status time- out.                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5 (R/W1C)          | DATA_CRC_ERR     | 0 No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 5 (R/W1C)          | DATA_CRC_ERR     | 1 Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

Table 18-58: EMSI\_ERR\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|-----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W1C)          | DATA_TOUT_ERR   | Data Timeout Error. The EMSI_ERR_STAT.DATA_TOUT_ERR bit is set in eMMC mode when detect- ing one of the following timeout conditions: - Busy timeout for R1b type - Busy timeout after Write CRC status - Write CRC Status timeout - Read Data timeout                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W1C)          | CMD_IDX_ERR     | Command Index Error. The EMSI_ERR_STAT.CMD_IDX_ERR bit is set if a command index error occurs in the command response in eMMC mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2 (R/W1C)          | CMD_END_BIT_ERR | Command End Bit Error. The EMSI_ERR_STAT.CMD_END_BIT_ERR bit is set when detecting that the end bit of a command response is 0 in eMMC mode.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 1 (R/W1C)          | CMD_CRC_ERR     | Command CRC Error. Command CRC Error is generated in eMMC mode for following two cases. - The eMSI controller detects a CMDline conflict by monitoring the CMDline when a command is issued. If the eMSI controller drives the CMDline to 1 level, but de- tects 0 level on the CMDline at the next eMSI bus clock edge, then the eMSI control- ler aborts the command (stop driving CMDline) and set the EMSI_ERR_STAT.CMD_CRC_ERR bit to 1. The command timeout error is also set (=1) to distinguish a CMDline conflict. - If a response is returned and the command timeout error is clear (=0) (indicating no timeout), the EMSI_ERR_STAT.CMD_CRC_ERR bit is set (=1) when detecting a CRC error in the command response. 0 No Error |

Table 18-58: EMSI\_ERR\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                             |
|--------------------|--------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W1C)          | CMD_TOUT_ERR | Command Timeout Error. In eMMC mode,the EMSI_ERR_STAT.CMD_TOUT_ERR bit is set only if no re- sponse is returned within 64 eMSI bus clock cycles from the end bit of the command. If the eMSI controller detects a CMDline conflict, along with Command CRC Error bit, the EMSI_ERR_STAT.CMD_TOUT_ERR bit is set (=1), without waiting for 64 eMSI bus clock cycles. |
| 0 (R/W1C)          | CMD_TOUT_ERR | 0 No Error                                                                                                                                                                                                                                                                                                                                                          |
| 0 (R/W1C)          | CMD_TOUT_ERR | 1 Time Out                                                                                                                                                                                                                                                                                                                                                          |

## Error Interrupt Status Enable Register

The EMSI\_ERR\_STAT\_EN register sets the interrupt status for error interrupt status register ( EMSI\_ERR\_STAT ) when the corresponding bit in EMSI\_ERR\_STAT\_EN is set (=1). This register is applicable for eMMC mode.

Figure 18-81: EMSI\_ERR\_STAT\_EN Register Diagram

<!-- image -->

Table 18-59: EMSI\_ERR\_STAT\_EN Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                         |
|--------------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | BOOT_ACK_ERR | Boot Acknowledgment Error (eMMC Mode Only). When the EMSI_ERR_STAT_EN.BOOT_ACK_ERR bit is set (=1), it enables set- ting the corresponding error in the EMSI_ERR_STAT register.                 |
| 11 (R/W)           | RESP_ERR     | Response Error Status Enable (eMMC Mode Only). When the EMSI_ERR_STAT_EN.RESP_ERR bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register. 0 Masked 1 Enabled |

Table 18-59: EMSI\_ERR\_STAT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                  |
|--------------------|------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | ADMA_ERR         | ADMA Error Status Enable. When the EMSI_ERR_STAT_EN.ADMA_ERR bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register.                                  |
| 9 (R/W)            | ADMA_ERR         | 0 Masked                                                                                                                                                                                 |
| 9 (R/W)            | ADMA_ERR         | 1 Enabled                                                                                                                                                                                |
| 8 (R/W)            | AUTO_CMD_ERR     | Auto CMDError Status Enable (eMMC Mode only). When the EMSI_ERR_STAT_EN.AUTO_CMD_ERR bit is set (=1), it enables set- ting the corresponding error in the EMSI_ERR_STAT register.        |
| 8 (R/W)            | AUTO_CMD_ERR     | 0 Masked                                                                                                                                                                                 |
| 8 (R/W)            | AUTO_CMD_ERR     | 1 Enabled                                                                                                                                                                                |
| 6 (R/W)            | DATA_END_BIT_ERR | Data End Bit Error Status Enable (eMMC Mode only). When the EMSI_ERR_STAT_EN.DATA_END_BIT_ERR bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register. |
| 6 (R/W)            | DATA_END_BIT_ERR | 0 Masked                                                                                                                                                                                 |
| 6 (R/W)            | DATA_END_BIT_ERR | 1 Enabled                                                                                                                                                                                |
| 5 (R/W)            | DATA_CRC_ERR     | Data CRC Error Status Enable (eMMC Mode only). When the EMSI_ERR_STAT_EN.DATA_CRC_ERR bit is set (=1), it enables set- ting the corresponding error in the EMSI_ERR_STAT register.       |
| 5 (R/W)            | DATA_CRC_ERR     | 0 Masked                                                                                                                                                                                 |
| 5 (R/W)            | DATA_CRC_ERR     | 1 Enabled                                                                                                                                                                                |
| 4 (R/W)            | DATA_TOUT_ERR    | Data Timeout Error Status Enable (eMMC Mode only). When the EMSI_ERR_STAT_EN.DATA_TOUT_ERR bit is set (=1), it enables set- ting the corresponding error in the EMSI_ERR_STAT register.  |
| 4 (R/W)            | DATA_TOUT_ERR    | 0 Masked                                                                                                                                                                                 |
| 4 (R/W)            | DATA_TOUT_ERR    | 1 Enabled                                                                                                                                                                                |
| 3 (R/W)            | CMD_IDX_ERR      | Command Index Error Status Enable (eMMC Mode Only). When the EMSI_ERR_STAT_EN.CMD_IDX_ERR bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register.     |
| 3 (R/W)            | CMD_IDX_ERR      | 0 Masked                                                                                                                                                                                 |
| 3 (R/W)            | CMD_IDX_ERR      | 1 Enabled                                                                                                                                                                                |

Table 18-59: EMSI\_ERR\_STAT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name                 | Description/Enumeration                                                                                                                                                                            |
|--------------------|--------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | CMD_END_BIT_ERR_S TAT_EN | Command End Bit Error Status Enable (eMMC Mode Only). When the EMSI_ERR_STAT_EN.CMD_END_BIT_ERR_STAT_EN bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register. |
| 2 (R/W)            | CMD_END_BIT_ERR_S TAT_EN | 0 Masked                                                                                                                                                                                           |
| 2 (R/W)            | CMD_END_BIT_ERR_S TAT_EN | 1 Enabled                                                                                                                                                                                          |
| 1 (R/W)            | CMD_CRC_ERR              | Command CRC Error Status Enable (eMMC Mode Only). When the EMSI_ERR_STAT_EN.CMD_CRC_ERR bit is set (=1), it enables setting the corresponding error in the EMSI_ERR_STAT register.                 |
| 1 (R/W)            | CMD_CRC_ERR              | 0 Masked                                                                                                                                                                                           |
| 1 (R/W)            | CMD_CRC_ERR              | 1 Enabled                                                                                                                                                                                          |
| 0 (R/W)            | CMD_TOUT_ERR             | Command Timeout Error Status Enable (eMMC Mode Only). When the EMSI_ERR_STAT_EN.CMD_TOUT_ERR bit is set (=1), it enables set- ting the corresponding error in the EMSI_ERR_STAT register.          |
| 0 (R/W)            | CMD_TOUT_ERR             | 0 Masked                                                                                                                                                                                           |
| 0 (R/W)            | CMD_TOUT_ERR             | 1 Enabled                                                                                                                                                                                          |

## Error Interrupt Signal Enable Register

The EMSI\_ERR\_STAT\_INTEN register is used to select the interrupt status that is notified to the core as an interrupt. All status bits share the same 1-bit interrupt line. Setting any of these bits to 1 enables the interrupt. This register is applicable for eMMC mode.

Figure 18-82: EMSI\_ERR\_STAT\_INTEN Register Diagram

<!-- image -->

Table 18-60: EMSI\_ERR\_STAT\_INTEN Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                            |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (R/W)           | BOOT_ACK_ERR | Boot Acknowledgment Error (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.BOOT_ACK_ERR bit is set (=1), it enables generating an interrupt signal when the boot acknowledgment error in the EMSI_ERR_STAT register is set. 0 Masked |
| 11 (R/W)           | RESP_ERR     | Response Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.RESP_ERR bit is set (=1), it enables gener- ating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT reg- ister. 0 Masked         |
| 11 (R/W)           |              | 1 Enabled                                                                                                                                                                                                                          |
| 11 (R/W)           |              |                                                                                                                                                                                                                                    |

Table 18-60: EMSI\_ERR\_STAT\_INTEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                     |
|--------------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | ADMA_ERR         | ADMA Error Signal Enable. When the EMSI_ERR_STAT_INTEN.ADMA_ERR bit is set (=1), it enables gener- ating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT reg- ister.                                |
| 9 (R/W)            | ADMA_ERR         | 0 Masked                                                                                                                                                                                                                    |
| 9 (R/W)            | ADMA_ERR         | 1 Enabled                                                                                                                                                                                                                   |
| 8 (R/W)            | AUTO_CMD_ERR     | Auto CMDError Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.AUTO_CMD_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.            |
| 8 (R/W)            | AUTO_CMD_ERR     | 0 Masked                                                                                                                                                                                                                    |
| 8 (R/W)            | AUTO_CMD_ERR     | 1 Enabled                                                                                                                                                                                                                   |
| 6 (R/W)            | DATA_END_BIT_ERR | Data End Bit Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.DATA_END_BIT_ERR bit is set (=1), it en- ables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register. |
| 6 (R/W)            | DATA_END_BIT_ERR | 0 Masked                                                                                                                                                                                                                    |
| 6 (R/W)            | DATA_END_BIT_ERR | 1 Enabled                                                                                                                                                                                                                   |
| 5 (R/W)            | DATA_CRC_ERR     | Data CRC Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.DATA_CRC_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.           |
| 5 (R/W)            | DATA_CRC_ERR     | 0 Masked                                                                                                                                                                                                                    |
| 5 (R/W)            | DATA_CRC_ERR     | 1 Enabled                                                                                                                                                                                                                   |
| 4 (R/W)            | DATA_TOUT_ERR    | Data Timeout Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.DATA_TOUT_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.      |
| 4 (R/W)            | DATA_TOUT_ERR    | 0 Masked                                                                                                                                                                                                                    |
| 4 (R/W)            | DATA_TOUT_ERR    | 1 Enabled                                                                                                                                                                                                                   |

Table 18-60: EMSI\_ERR\_STAT\_INTEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | CMD_IDX_ERR     | Command Index Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.CMD_IDX_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.             |
| 2 (R/W)            | CMD_END_BIT_ERR | Command End Bit Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.CMD_END_BIT_ERR bit is set (=1), it ena- bles generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.     |
| 1 (R/W)            | CMD_CRC_ERR     | 1 Enabled Command CRC Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.CMD_CRC_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register.     |
| 0 (R/W)            | CMD_TOUT_ERR    | Command Timeout Error Signal Enable (eMMC Mode Only). When the EMSI_ERR_STAT_INTEN.CMD_TOUT_ERR bit is set (=1), it enables generating an interrupt signal error in the corresponding bit in the EMSI_ERR_STAT register. 0 Masked |

## Force Event Register for Auto CMD Error Status Register

The EMSI\_FRC\_AUTOCMDSTAT register is not a physically implemented but is an address at which the Auto CMD Error Status register ( EMSI\_AUTOCMD\_STAT ) can be written.This register is applicable for an eMMC mode. Setting a bit in this register sets the corresponding bit in the EMSI\_AUTOCMD\_STAT register.

Figure 18-83: EMSI\_FRC\_AUTOCMDSTAT Register Diagram

<!-- image -->

Table 18-61: EMSI\_FRC\_AUTOCMDSTAT Register Fields

| Bit No. (Access)   | Bit Name                    | Description/Enumeration                                                                                                                                                              |
|--------------------|-----------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (RX/W)           | CMD_NOT_IS- SUED_AUTO_CMD12 | Force Event for Command Not Issued by Auto CMD12 Error. Setting the EMSI_FRC_AUTOCMDSTAT.CMD_NOT_ISSUED_AUTO_CMD12 bit sets the corresponding bit in the EMSI_AUTOCMD_STAT register. |
| 5 (RX/W)           | AUTO_CMD_RESP_ERR           | Force Event for Auto CMDResponse Error. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD_RESP_ERR bit sets the cor- responding bit in the EMSI_AUTOCMD_STAT register.                       |
| 4 (RX/W)           | AUTO_CMD_IDX_ERR            | Force Event for Auto CMDIndex Error. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD_IDX_ERR bit sets the corre- sponding bit in the EMSI_AUTOCMD_STAT register. 0 Not Affected            |
| 4 (RX/W)           |                             | 1 Auto CMDIndex Error Status is Set                                                                                                                                                  |
| 4 (RX/W)           |                             |                                                                                                                                                                                      |

Table 18-61: EMSI\_FRC\_AUTOCMDSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name              | Description/Enumeration                                                                                                                                                        |
|--------------------|-----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (RX/W)           | AUTO_CMD_EBIT_ERR     | Force Event for Auto CMDEnd Bit Error. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD_EBIT_ERR bit sets the cor- responding bit in the EMSI_AUTOCMD_STAT register.                  |
| 2 (RX/W)           | AUTO_CMD_CRC_ERR      | Force Event for Auto CMDCRCError. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD_CRC_ERR bit sets the corre- sponding bit in the EMSI_AUTOCMD_STAT register.                        |
| 1 (RX/W)           | AU- TO_CMD_TOUT_ERR   | Force Event for Auto CMDTimeout Error. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD_TOUT_ERR bit sets the cor- responding bit in the EMSI_AUTOCMD_STAT register. 0 Not Affected   |
| 0 (RX/W)           | AU- TO_CMD12_NOT_EXEC | Force Event for Auto CMD12 Not Executed. Setting the EMSI_FRC_AUTOCMDSTAT.AUTO_CMD12_NOT_EXEC bit sets the corresponding bit in the EMSI_AUTOCMD_STAT register. 0 Not Affected |

## Force Event Register for Error Interrupt Status

The EMSI\_FRC\_ERRSTAT register is not physically implemented but is an address at which the error interrupt status register ( EMSI\_ERR\_STAT ) can be written. The effect of a write to this address is reflected in the ( EMSI\_ERR\_STAT ) register if the corresponding bit of the error interrupt status enable register ( EMSI\_ERR\_STAT\_EN ) is set. This register is applicable for an eMMC mode.

Figure 18-84: EMSI\_FRC\_ERRSTAT Register Diagram

<!-- image -->

Table 18-62: EMSI\_FRC\_ERRSTAT Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                        |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 12 (RX/W)          | BOOT_ACK_ERR | Force Event for Boot Ack Error. A write to the EMSI_FRC_ERRSTAT.BOOT_ACK_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.              | Force Event for Boot Ack Error. A write to the EMSI_FRC_ERRSTAT.BOOT_ACK_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.              |
| 12 (RX/W)          | BOOT_ACK_ERR | 0                                                                                                                                                                                                              | Not Affected                                                                                                                                                                                                   |
| 11 (RX/W)          | RESP_ERR     | Force Event for Response Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.RESP_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set. | Force Event for Response Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.RESP_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set. |
| 11 (RX/W)          | RESP_ERR     | 0                                                                                                                                                                                                              | Not Affected                                                                                                                                                                                                   |
| 11 (RX/W)          | RESP_ERR     | 1                                                                                                                                                                                                              | Response Error Status is Set                                                                                                                                                                                   |

Table 18-62: EMSI\_FRC\_ERRSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name         | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (RX/W)           | ADMA_ERR         | Force Event for ADMA Error. A write to the EMSI_FRC_ERRSTAT.ADMA_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.                                  |
| 9 (RX/W)           | ADMA_ERR         | 0 Not Affected                                                                                                                                                                                                             |
| 9 (RX/W)           | ADMA_ERR         | 1 ADMA Error Status is Set                                                                                                                                                                                                 |
| 8 (RX/W)           | AUTO_CMD_ERR     | Force Event for Auto CMDError (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.AUTO_CMD_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.          |
| 8 (RX/W)           | AUTO_CMD_ERR     | 0 Not Affected                                                                                                                                                                                                             |
| 8 (RX/W)           | AUTO_CMD_ERR     | 1 Auto CMDError Status is Set                                                                                                                                                                                              |
| 6 (RX/W)           | DATA_END_BIT_ERR | Force Event for Data End Bit Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.DATA_END_BIT_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set. |
| 6 (RX/W)           | DATA_END_BIT_ERR | 0 Not Affected                                                                                                                                                                                                             |
| 6 (RX/W)           | DATA_END_BIT_ERR | 1 Data End Bit Error Status is Set                                                                                                                                                                                         |
| 5 (RX/W)           | DATA_CRC_ERR     | Force Event for Data CRC Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.DATA_CRC_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.         |
| 5 (RX/W)           | DATA_CRC_ERR     | 0 Not Affected                                                                                                                                                                                                             |
| 5 (RX/W)           | DATA_CRC_ERR     | 1 Data CRC Error Status is Set                                                                                                                                                                                             |
| 4 (RX/W)           | DATA_TOUT_ERR    | Force Event for Data Timeout Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.DATA_TOUT_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.    |
| 4 (RX/W)           | DATA_TOUT_ERR    | 0 Not Affected                                                                                                                                                                                                             |
| 4 (RX/W)           | DATA_TOUT_ERR    | 1 Data Timeout Error Status is Set                                                                                                                                                                                         |

Table 18-62: EMSI\_FRC\_ERRSTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                      |
|--------------------|-----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (RX/W)           | CMD_IDX_ERR     | Force Event for Command Index Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.CMD_IDX_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.       |
| 3 (RX/W)           | CMD_IDX_ERR     | 0 Not Affected                                                                                                                                                                                                               |
| 2 (RX/W)           | CMD_END_BIT_ERR | Force Event for Command End Bit Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.CMD_END_BIT_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set. |
| 2 (RX/W)           | CMD_END_BIT_ERR | 0 Not Affected                                                                                                                                                                                                               |
| 2 (RX/W)           | CMD_END_BIT_ERR | 1 Command End Bit Error Status is Set                                                                                                                                                                                        |
| 1 (RX/W)           | CMD_CRC_ERR     | Force Event for Command CRC Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.CMD_CRC_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.         |
| 1 (RX/W)           | CMD_CRC_ERR     | 0 Not Affected                                                                                                                                                                                                               |
| 1 (RX/W)           | CMD_CRC_ERR     | 1 Command CRC Error Status is Set                                                                                                                                                                                            |
| 0 (RX/W)           | CMD_TOUT_ERR    | Force Event for Command Timeout Error (eMMC Mode Only). A write to the EMSI_FRC_ERRSTAT.CMD_TOUT_ERR address is reflected in the EMSI_ERR_STAT register if the corresponding bit of the EMSI_ERR_STAT_EN register is set.    |
| 0 (RX/W)           | CMD_TOUT_ERR    | 0 Not Affected                                                                                                                                                                                                               |
| 0 (RX/W)           | CMD_TOUT_ERR    | 1 Command Timeout Error Status is Set                                                                                                                                                                                        |

## Interrupt Status Register

The EMSI\_ISTAT register reflects the interrupt status. This register is applicable for eMMC mode.

Figure 18-85: EMSI\_ISTAT Register Diagram

<!-- image -->

Table 18-63: EMSI\_ISTAT Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                                                                            | Description/Enumeration                                                                                                                                                                                                            |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/NW)          | ERR_INTERRUPT | Error Interrupt. The EMSI_ISTAT.ERR_INTERRUPT bit is set if any bit in the EMSI_ERR_STAT register are set.                                                                                                                         | Error Interrupt. The EMSI_ISTAT.ERR_INTERRUPT bit is set if any bit in the EMSI_ERR_STAT register are set.                                                                                                                         |
| 15 (R/NW)          | ERR_INTERRUPT | 0                                                                                                                                                                                                                                  | No Error                                                                                                                                                                                                                           |
| 15 (R/NW)          | ERR_INTERRUPT | 1                                                                                                                                                                                                                                  | Error                                                                                                                                                                                                                              |
| 14 (R/W1C)         | CQE_EVENT     | Command Queuing Event. The EMSI_ISTAT.CQE_EVENT status bit is set if command queuing related event has occurred in eMMC mode. Read the EMSI_CQ_ISTAT / EMSI_CQ_TERRINFO registers for more details.                                | Command Queuing Event. The EMSI_ISTAT.CQE_EVENT status bit is set if command queuing related event has occurred in eMMC mode. Read the EMSI_CQ_ISTAT / EMSI_CQ_TERRINFO registers for more details.                                |
| 14 (R/W1C)         | CQE_EVENT     | 0                                                                                                                                                                                                                                  | No Event                                                                                                                                                                                                                           |
| 14 (R/W1C)         | CQE_EVENT     | 1                                                                                                                                                                                                                                  | Command Queuing Event is Detected                                                                                                                                                                                                  |
| 13 (R/NW)          | FX_EVENT      | FX Event. The EMSI_ISTAT.FX_EVENT status bit is set when R[14] of the response register is set (=1) and response type R1 is cleared (=0) in the EMSI_TRNSFRMODE register. This interrupt is used with the response check function. | FX Event. The EMSI_ISTAT.FX_EVENT status bit is set when R[14] of the response register is set (=1) and response type R1 is cleared (=0) in the EMSI_TRNSFRMODE register. This interrupt is used with the response check function. |
| 13 (R/NW)          | FX_EVENT      | 0                                                                                                                                                                                                                                  | No Event                                                                                                                                                                                                                           |
| 13 (R/NW)          | FX_EVENT      | 1                                                                                                                                                                                                                                  | FX Event is Detected                                                                                                                                                                                                               |

Table 18-63: EMSI\_ISTAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                    |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | CARD_REMOVAL   | Card Removal. The EMSI_ISTAT.CARD_REMOVAL bit is set if the card inserted in the present state register changes from 1 to 0.                                                                                                                                                                               |
| 7 (R/W1C)          | CARD_REMOVAL   | 0 Card State Stable or Debouncing                                                                                                                                                                                                                                                                          |
| 7 (R/W1C)          | CARD_REMOVAL   | 1 Card Removed                                                                                                                                                                                                                                                                                             |
| 6 (R/W1C)          | CARD_INSERTION | Card Insertion. The EMSI_ISTAT.CARD_INSERTION bit is set if the card inserted in the present state register changes from 0 to 1.                                                                                                                                                                           |
| 6 (R/W1C)          | CARD_INSERTION | 0 Card State Stable or Debouncing                                                                                                                                                                                                                                                                          |
| 6 (R/W1C)          | CARD_INSERTION | 1 Card Inserted                                                                                                                                                                                                                                                                                            |
| 3 (R/W1C)          | DMA_INTERRUPT  | DMAInterrupt. The EMSI_ISTAT.DMA_INTERRUPT bit is set if the eMSI controller detects the SDMA Buffer Boundary during transfer. In case of ADMA, by setting the Int field in the descriptor table, the eMSI controller generates this interrupt. This interrupt is not generated after a transfer complete. |
| 3 (R/W1C)          | DMA_INTERRUPT  | 0 No DMAInterrupt                                                                                                                                                                                                                                                                                          |
| 3 (R/W1C)          | DMA_INTERRUPT  | 1 DMAInterrupt is Generated                                                                                                                                                                                                                                                                                |
| 1 (R/W1C)          | XFER_COMPLETE  | Transfer Complete. The EMSI_ISTAT.XFER_COMPLETE bit is set when a read/write transfer and a command with status busy is completed.                                                                                                                                                                         |
| 1 (R/W1C)          | XFER_COMPLETE  | 0 Not Complete                                                                                                                                                                                                                                                                                             |
| 1 (R/W1C)          | XFER_COMPLETE  | 1 Command Execution is Completed                                                                                                                                                                                                                                                                           |
| 0 (R/W1C)          | CMD_COMPLETE   | Command Complete. In an eMMC Mode, the EMSI_ISTAT.CMD_COMPLETE bit is set when the end bit of a response except for Auto CMD12 and Auto CMD23. This interrupt is not generated when the EMSI_TRNSFRMODE.RESP_INT_DIS bit is set (=1). Complete                                                             |
| 0 (R/W1C)          | CMD_COMPLETE   | 0 No Command                                                                                                                                                                                                                                                                                               |
| 0 (R/W1C)          | CMD_COMPLETE   | 1 Command Complete                                                                                                                                                                                                                                                                                         |

## Interrupt Status Enable Register

The EMSI\_ISTAT\_EN register enables/masks the interrupt status for the corresponding bit in the interrupt status register ( EMSI\_ISTAT ) register. This register is applicable for eMMC mode.

Figure 18-86: EMSI\_ISTAT\_EN Register Diagram

<!-- image -->

Table 18-64: EMSI\_ISTAT\_EN Register Fields

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                    |
|--------------------|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | CQE_EVENT      | CQE Event Status Enable. The EMSI_ISTAT_EN.CQE_EVENT bit, when set (=1), enables the interrupt sta- tus for the corresponding bit in the EMSI_ISTAT register.                              |
| 13 (R/W)           | FX_EVENT       | FX Event Status Enable. The EMSI_ISTAT_EN.FX_EVENT bit is added from Version 4.10. 0 Masked                                                                                                |
| 7 (R/W)            | CARD_REMOVAL   | Card Removal Status Enable. The EMSI_ISTAT_EN.CARD_REMOVAL bit, when set (=1), enables the interrupt status for the corresponding bit in the EMSI_ISTAT register. 0 Masked                 |
| 6 (R/W)            | CARD_INSERTION | Card Insertion Status Enable. The EMSI_ISTAT_EN.CARD_INSERTION bit, when set (=1), enables the inter- rupt status for the corresponding bit in the EMSI_ISTAT register. 0 Masked 1 Enabled |

Table 18-64: EMSI\_ISTAT\_EN Register Fields (Continued)

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                                                                        |
|--------------------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | DMA_INTERRUPT | DMAInterrupt Status Enable. The EMSI_ISTAT_EN.DMA_INTERRUPT bit, when set (=1), enables the interrupt status for the corresponding bit in the EMSI_ISTAT register.             |
| 1 (R/W)            | XFER_COMPLETE | Transfer Complete Status Enable. The EMSI_ISTAT_EN.XFER_COMPLETE bit, when set (=1), enables the interrupt status for the corresponding bit in the EMSI_ISTAT register.        |
| 0 (R/W)            | CMD_COMPLETE  | Command Complete Status Enable. The EMSI_ISTAT_EN.CMD_COMPLETE bit, when set (=1), enables the interrupt status for the corresponding bit in the EMSI_ISTAT register. 0 Masked |
| 0 (R/W)            |               |                                                                                                                                                                                |

## Interrupt Signal Enable Register

The EMSI\_ISTAT\_INTEN register is used to select the interrupt status that is indicated to the core as the interrupt. All these status bits share the same 1-bit interrupt line. Setting any of these bits to 1 enables interrupt generation. This register is applicable for eMMC mode.

Figure 18-87: EMSI\_ISTAT\_INTEN Register Diagram

<!-- image -->

Table 18-65: EMSI\_ISTAT\_INTEN Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                          |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | CQE_EVENT    | Command Queuing Engine Event Signal Enable. When the EMSI_ISTAT_INTEN.CQE_EVENT bit is set (=1) it selects the inter- rupt status that is indicated to the core. |
| 13 (R/W)           | FX_EVENT     | FX Event Signal Enable. When the EMSI_ISTAT_INTEN.FX_EVENT bit is set (=1) it selects the interrupt status that is indicated to the core.                        |
| 7 (R/W)            | CARD_REMOVAL | Card Removal Signal Enable. When the EMSI_ISTAT_INTEN.CARD_REMOVAL bit is set (=1) it selects the in- terrupt status that is indicated to the core. 0 Masked     |
| 7 (R/W)            | 1            | Enabled                                                                                                                                                          |
| 7 (R/W)            |              |                                                                                                                                                                  |

Table 18-65: EMSI\_ISTAT\_INTEN Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                          |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | CARD_INSERTION | Card Insertion Signal Enable. When the EMSI_ISTAT_INTEN.CARD_INSERTION bit is set (=1) it selects the interrupt status that is indicated to the core.            |
| 3 (R/W)            | DMA_INTERRUPT  | DMAInterrupt Signal Enable. When the EMSI_ISTAT_INTEN.DMA_INTERRUPT bit is set (=1) it selects the interrupt status that is indicated to the core.               |
| 1 (R/W)            | XFER_COMPLETE  | Transfer Complete Signal Enable. When the EMSI_ISTAT_INTEN.XFER_COMPLETE bit is set (=1) it selects the interrupt status that is indicated to the core.          |
| 0 (R/W)            | CMD_COMPLETE   | Command Complete Signal Enable. When the EMSI_ISTAT_INTEN.CMD_COMPLETE bit is set (=1) it selects the in- terrupt status that is indicated to the core. 0 Masked |

## Preset Value for Default Speed

The EMSI\_PRESET\_DS register defines preset value for default speed mode in SD mode.

Figure 18-88: EMSI\_PRESET\_DS Register Diagram

<!-- image -->

Table 18-66: EMSI\_PRESET\_DS Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|
| 9:0                | FREQ_SEL   | SDCLK Frequency Select Value.                                                                                          |
| (R/NW)             |            | The EMSI_PRESET_DS.FREQ_SEL bits configure the 10-bit preset value that is set in the EMSI_CLK_CTL.FREQ_SEL bit field. |

## Preset Value for HSDDR

The EMSI\_PRESET\_HSDDR register defines the preset value for high speed DDR speed mode in eMMC mode.

Figure 18-89: EMSI\_PRESET\_HSDDR Register Diagram

<!-- image -->

Table 18-67: EMSI\_PRESET\_HSDDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                           |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | FREQ_SEL   | eMSI Bus Clock Frequency Select Value. The EMSI_PRESET_HSDDR.FREQ_SEL bits specify a 10-bit preset value that must be set in the EMSI_CLK_CTL.FREQ_SEL bit field, as described by an eMSI system. |

## Preset Value for HSSDR

The EMSI\_PRESET\_HSSDR register defines the preset value for high speed SDR speed mode in eMMC mode.

Figure 18-90: EMSI\_PRESET\_HSSDR Register Diagram

<!-- image -->

Table 18-68: EMSI\_PRESET\_HSSDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | FREQ_SEL   | eMSI Bus Clock Frequency Select Value. The EMSI_PRESET_HSSDR.FREQ_SEL bits specify a 10-bit preset value that must be set in the EMSI_CLK_CTL.FREQ_SEL bit field, as described by a eMSI system. |

## Preset Value for Initialization

The EMSI\_PRESET\_INIT register defines Preset Value for Initialization in eMMC mode.

Figure 18-91: EMSI\_PRESET\_INIT Register Diagram

<!-- image -->

Table 18-69: EMSI\_PRESET\_INIT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | FREQ_SEL   | eMSI Bus Clock Frequency Select Value. The EMSI_PRESET_INIT.FREQ_SEL bits configure the 10-bit preset value that is set in the EMSI_CLK_CTL.FREQ_SEL bit field. The value should be written manually in the EMSI_CLK_CTL.FREQ_SEL bit field for eMSI bus clock initiali- zation. |

## Preset Value for Legacy Mode

The EMSI\_PRESET\_LEGACY register defines the preset value for legacy speed mode in eMMC mode.

Figure 18-92: EMSI\_PRESET\_LEGACY Register Diagram

<!-- image -->

Table 18-70: EMSI\_PRESET\_LEGACY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:0 (R/NW)         | FREQ_SEL   | eMSI Bus Clock Frequency Select. The EMSI_PRESET_LEGACY.FREQ_SEL bits specify a 10-bit preset value that must be set in the EMSI_CLK_CTL.FREQ_SEL bit field, as described by a eMSI system. |

## Present State Register

The EMSI\_PSTATE register indicates the present status of the eMSI controller. This register is applicable for eMMC mode.

Figure 18-93: EMSI\_PSTATE Register Diagram

<!-- image -->

Table 18-71: EMSI\_PSTATE Register Fields

| Bit No. (Access)   | Bit Name      | Description/Enumeration                                                                                                      | Description/Enumeration                                                                                                      |
|--------------------|---------------|------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/NW)          | SUB_CMD_STAT  | Sub Command Status. The EMSI_PSTATE.SUB_CMD_STAT bit is used to distinguish between a main command and a sub command status. | Sub Command Status. The EMSI_PSTATE.SUB_CMD_STAT bit is used to distinguish between a main command and a sub command status. |
|                    |               | 0                                                                                                                            | Main Command Status                                                                                                          |
| 27 (R/NW)          | CMD_ISSUE_ERR | Command Not Issued by Error. The EMSI_PSTATE.CMD_ISSUE_ERR bit is set if a command cannot be issued                          | Command Not Issued by Error. The EMSI_PSTATE.CMD_ISSUE_ERR bit is set if a command cannot be issued                          |
|                    |               | 0                                                                                                                            | No Error for Issuing a Command                                                                                               |
|                    |               | 1                                                                                                                            | Command Cannot Be Issued                                                                                                     |

Table 18-71: EMSI\_PSTATE Register Fields (Continued)

| Bit No. (Access)   | Bit Name                | Description/Enumeration                                                                                                                                                                                                           |
|--------------------|-------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 24 (R/NW)          | CMD_LINE_LVL            | Command-Line Signal Level. The EMSI_PSTATE.CMD_LINE_LVL bit is used to check the CMDline level to recover from errors and for debugging. The EMSI_PSTATE.CMD_LINE_LVL field reflect the value of the CMDsignal.                   |
| 23:20 (R/NW)       | DAT_3_0                 | DAT[3:0] Line Signal Level. The EMSI_PSTATE.DAT_3_0 bit is used to check the DAT line level to recover from errors and for debugging.                                                                                             |
| 19 (R/NW)          | WR_PROTECT_SW_LVL       | Write Protect Switch Pin Level. The EMSI_PSTATE.WR_PROTECT_SW_LVL bit is supported only for memory and combo cards. The EMSI_PSTATE.WR_PROTECT_SW_LVL bit reflects the synchronized value of the WP signal.                       |
| 19 (R/NW)          | WR_PROTECT_SW_LVL       | 0 Write Protected                                                                                                                                                                                                                 |
| 18 (R/NW)          | CARD_DE- TECT_PIN_LEVEL | Card Detect Pin Level. The EMSI_PSTATE.CARD_DETECT_PIN_LEVEL bit reflects the inverse synchronized value of the CD signal.                                                                                                        |
| 18 (R/NW)          | CARD_DE- TECT_PIN_LEVEL | 0 No Card Present                                                                                                                                                                                                                 |
| 17 (R/NW)          | CARD_STABLE             | Card Stable. The EMSI_PSTATE.CARD_STABLE bit indicates the stability of the Card Detect Pin Level. A card is not detected if the EMSI_PSTATE.CARD_STABLE bit is set (=1) and the value of the EMSI_PSTATE.CARD_INSERTED bit is 0. |
| 17 (R/NW)          | CARD_STABLE             | 0 Reset or Debouncing                                                                                                                                                                                                             |
| 17 (R/NW)          | CARD_STABLE             | 1 No Card or Inserted                                                                                                                                                                                                             |
| 16 (R/NW)          | CARD_INSERTED           | Card Inserted. The EMSI_PSTATE.CARD_INSERTED bit indicates whether a card has been in- serted. The eMSI controller debounces this signal so that the host driver need not wait for it to stabilize.                               |
| 16 (R/NW)          | CARD_INSERTED           | 0 Reset, Debouncing, or No Card                                                                                                                                                                                                   |
| 16 (R/NW)          | CARD_INSERTED           | 1 Card Inserted                                                                                                                                                                                                                   |
| 9 (R/NW)           | RD_XFER_ACTIVE          | Read Transfer Active. The EMSI_PSTATE.RD_XFER_ACTIVE bit indicates whether a read transfer is active for eMMC mode.                                                                                                               |
| 9 (R/NW)           | RD_XFER_ACTIVE          | 0 No Valid Data                                                                                                                                                                                                                   |
| 9 (R/NW)           | RD_XFER_ACTIVE          | 1 Transferring Data                                                                                                                                                                                                               |

Table 18-71: EMSI\_PSTATE Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|-----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | WR_XFER_ACTIVE  | Write Transfer Active. The EMSI_PSTATE.WR_XFER_ACTIVE status indicates whether a write transfer is active for eMMC mode.                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 8 (R/NW)           | WR_XFER_ACTIVE  | 0 No Valid Data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 8 (R/NW)           | WR_XFER_ACTIVE  | 1 Transferring Data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 7:4 (R/NW)         | DAT_7_4         | DAT[7:4] Line Signal Level. The EMSI_PSTATE.DAT_7_4 bit is used to check the DAT line level to recover from errors and for debugging.                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 2 (R/NW)           | DAT_LINE_ACTIVE | DAT Line Active (eMMC Mode Only). The EMSI_PSTATE.DAT_LINE_ACTIVE bit indicates whether one of the DAT lines on the eMMC bus is in use. In the case of read transactions, the EMSI_PSTATE.DAT_LINE_ACTIVE bit indi- cates whether a read transfer is executing on the eMMC bus. In the case of write transactions, the EMSI_PSTATE.DAT_LINE_ACTIVE bit in- dicates whether a write transfer is executing on the eMMC bus. For a command with busy, this status indicates whether the command executing busy is executing on an eMMC bus. 0 DAT Line Inactive |
| 2 (R/NW)           | DAT_LINE_ACTIVE | 1 DAT Line Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 1 (R/NW)           | CMD_INHIBIT_DAT | Command Inhibit (DAT). The EMSI_PSTATE.CMD_INHIBIT_DAT bit is applicable for eMMC mode and is generated if either DAT line active or read transfer active is set (=1). If the EMSI_PSTATE.CMD_INHIBIT_DAT bit is cleared (=0) it indicates that the eMSI controller can issue subsequent eMMC commands.                                                                                                                                                                                                                                                      |
| 1 (R/NW)           | CMD_INHIBIT_DAT | 0 Can Issue Command Which Used DAT Line                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 1 (R/NW)           | CMD_INHIBIT_DAT | 1 Cannot Issue Command Which Used DAT Line                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 18-71: EMSI\_PSTATE Register Fields (Continued)

| Bit No. (Access)   | Bit Name    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|-------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/NW)           | CMD_INHIBIT | Command Inhibit (CMD). The EMSI_PSTATE.CMD_INHIBIT bit indicates the following : For eMMC mode, if the EMSI_PSTATE.CMD_INHIBIT bit is cleared (=0) it indi- cates that the CMDline is not in use and the eMSI controller can issue an eMMC command using the CMDline. The EMSI_PSTATE.CMD_INHIBIT bit is set when the command register is written. The EMSI_PSTATE.CMD_INHIBIT bit is cleared when the command response is received. The EMSI_PSTATE.CMD_INHIBIT bit is not cleared by the response of auto CMD12/23 but cleared by the response of read/write command. |
| 0 (R/NW)           | CMD_INHIBIT | 0 eMSI Controller is Ready to Issue a Command                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/NW)           | CMD_INHIBIT | 1 eMSI Controller is Not Ready to Issue a Command                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

## Response Register 0

The EMSI\_RESP0 register stores 39-08 bits of the Response Field for an eMMC mode. The response for an eMMC command can be a maximum of 128 bits. These 128 bits are segregated into four 32-bit registers, EMSI\_RESP0 -3.

Figure 18-94: EMSI\_RESP0 Register Diagram

<!-- image -->

Table 18-72: EMSI\_RESP0 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Command Response. The EMSI_RESP0.VALUE bits reflect 39-8 bits of eMMC Response Field. Note: For Auto CMD, the 32-bit response (bits 39-8 of the Response Field) is updated in the EMSI_RESP3 register. |

## Response Register 1

The EMSI\_RESP1 register stores bits 71-40 of the Response Field for eMMC mode. This register is used to store the response from the eMMC device. The response can be a maximum of 128 bits. These 128 bits are segregated into four 32-bit registers EMSI\_RESP0 -3.

Figure 18-95: EMSI\_RESP1 Register Diagram

<!-- image -->

Table 18-73: EMSI\_RESP1 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                  |
|--------------------|------------|--------------------------------------------------------------------------|
| 31:0               | VALUE      | Command Response.                                                        |
| (R/NW)             |            | The EMSI_RESP1.VALUE bits reflect bits 71-40 of the eMMC Response Field. |

## Response Register 2

The EMSI\_RESP2 register stores 103-72 bits of the Response Field for an eMMC mode. The response for eMMC command can be a maximum of 128 bits. These 128 bits are segregated into four 32-bit registers: EMSI\_RESP0 -3

Figure 18-96: EMSI\_RESP2 Register Diagram

<!-- image -->

Table 18-74: EMSI\_RESP2 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                  |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Response Bits 103-72. The EMSI_RESP2.VALUE bit field stores bits 103-72 of the Response Field for eMMC mode. The response for eMMC command can be a maximum of 128 bits. |

## Response Register 3

The EMSI\_RESP3 register stores bits 135-104 of the Response Field for an eMMC mode. The eMMC response can be a maximum of 128 bits. These 128 bits are segregated into four 32-bit registers: EMSI\_RESP0 -3.

Figure 18-97: EMSI\_RESP3 Register Diagram

<!-- image -->

Table 18-75: EMSI\_RESP3 Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Command Response. These bits reflect bits 135-104 of eMMC Response Field. Note: For Auto CMD, this register also reflects the 32-bit response (bits 39-8 of the Response Field). |

## SDMA System Address Register

The EMSI\_SDMA\_ADDR register is used to configure a 32-bit block count or a SDMA system address based on the Host Version 4 Enable bit in the EMSI\_CTL2 register. This register is applicable for eMMC mode.

Figure 18-98: EMSI\_SDMA\_ADDR Register Diagram

<!-- image -->

Table 18-76: EMSI\_SDMA\_ADDR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | 32-bit Block Count (SDMA System Address). The EMSI_SDMA_ADDR.VALUE bit field contains the system memory address for an SDMA transfer in the 32-bit addressing mode (Host Version 4 Enable = 0). When the eMSI controller stops a SDMA transfer, this bit field points to the system address of the next contiguous data position. It can be accessed only if no transaction is executing. Reading this register during data transfers may return an invalid value. 32-bit Block Count (Host Version 4 Enable = 1): From the SD Host Controller Ver- sion 4.10 specification, this register is redefined as 32-bit block count. The eMSI con- troller decrements the block count of this register for every block transfer and the data transfer stops when the count reaches zero. Following are the values for BLOCKCNT_SDMASA: - 0xFFFF_FFFF: 4G - 1 Block - ... - 0x0000_0002: 2 Blocks - 0x0000_0001: 1 Block - 0x0000_0000: Stop Count Note: - For Host Version 4 Enable = 0, the host driver does not program the system address in this register while operating in ADMA mode. The system address must be program- med in the ADMA System Address register. - For Host Version 4 Enable = 0, the host driver programs a non-zero 32-bit block count value in this register when Auto CMD23 is enabled for ADMA modes. Auto CMD23 cannot be used with SDMA. - This register must be programmed with a non-zero value for data transfer if the 32- bit block count register is used instead of the 16-bit block count register. |

## Software Reset Register

The EMSI\_SWRST register is used to generate a reset pulse by writing 1 to each bit of this register. After completing the reset, the eMSI controller clears each bit. As it takes some time to complete a software reset, the host driver confirms that these bits are 0. This register is applicable for eMMC mode.

Figure 18-99: EMSI\_SWRST Register Diagram

<!-- image -->

Table 18-77: EMSI\_SWRST Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | DAT        | Software Reset For DAT Line. The EMSI_SWRST.DAT bit is used in eMMC mode and it resets only a part of the data circuit and the DMAcircuit is also reset. The following registers and bits are cleared by the EMSI_SWRST.DAT bit: Present state register • Read transfer active • Write transfer active • DAT line active • Command Inhibit (DAT) Normal Interrupt status register • DMAinterrupt • Transfer complete. Work |
| 2 (R/W)            |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 2 (R/W)            |            | 1 Reset                                                                                                                                                                                                                                                                                                                                                                                                                    |

Table 18-77: EMSI\_SWRST Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1 (R/W)            | CMD        | Software Reset For CMDLine. The EMSI_SWRST.CMD bit resets only a part of the command circuit to be able to issue a command. This reset is effective only for a command issuing circuit (including response error statuses related to Command Inhibit (CMD) control) and does not af- fect the data transfer circuit. The eMSI controller can continue data transfer even after this reset is executed while handling subcommand-response errors. The following registers and bits are cleared by the EMSI_SWRST.CMD bit: - Present State register: Command Inhibit (CMD) bit - Interrupt Status register: Command Complete bit - Error Interrupt Status: Response error statuses related to Command Inhibit (CMD) bit | Software Reset For CMDLine. The EMSI_SWRST.CMD bit resets only a part of the command circuit to be able to issue a command. This reset is effective only for a command issuing circuit (including response error statuses related to Command Inhibit (CMD) control) and does not af- fect the data transfer circuit. The eMSI controller can continue data transfer even after this reset is executed while handling subcommand-response errors. The following registers and bits are cleared by the EMSI_SWRST.CMD bit: - Present State register: Command Inhibit (CMD) bit - Interrupt Status register: Command Complete bit - Error Interrupt Status: Response error statuses related to Command Inhibit (CMD) bit |
| 1 (R/W)            | CMD        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Work                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 1 (R/W)            | CMD        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Reset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 0 (R/W)            | ALL        | Software Reset for All. This reset affects the entire eMSI controller. During its initialization, the host driver sets (=1) the EMSI_SWRST.ALL bit to reset the eMSI controller. All registers are re- set except the capabilities register. If the EMSI_SWRST.ALL bit is set (=1), the host driver must issue reset command and reinitialize the card.                                                                                                                                                                                                                                                                                                                                                               | Software Reset for All. This reset affects the entire eMSI controller. During its initialization, the host driver sets (=1) the EMSI_SWRST.ALL bit to reset the eMSI controller. All registers are re- set except the capabilities register. If the EMSI_SWRST.ALL bit is set (=1), the host driver must issue reset command and reinitialize the card.                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | ALL        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Work                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | ALL        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Reset                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

## Timeout Control Register

The EMSI\_TO\_CTL register is used to set the Data Timeout Counter value for an eMMC mode according to the timer clock defined by the capabilities register, while initializing the eMSI controller.

Figure 18-100: EMSI\_TO\_CTL Register Diagram

<!-- image -->

Table 18-78: EMSI\_TO\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3:0 (R/W)          | VALUE      | Data Timeout Counter Value. The EMSI_TO_CTL.VALUE value determines the interval by which DAT line time- outs are detected. The timeout clock frequency is generated by dividing the base clock TMCLK value by this value. When setting this register, prevent inadvertent timeout events by clearing the EMSI_ERR_STAT_EN.DATA_TOUT_ERR bit. The values for these bits are: - 0xF: Reserved - 0xE: TMCLK x 2 27 - ......... - 0x1: TMCLK x 2 14 - 0x0: TMCLK x 2 13 Note: During a boot operating in an eMMC mode, an application must configure the boot data timeout value (approximately 1 sec) in the EMSI_TO_CTL.VALUE bit. |

## Transfer Mode Register

The EMSI\_TRNSFRMODE register is used to control the operation of data transfers for an eMMC mode. The host driver sets this register before issuing a command that transfers data.

Figure 18-101: EMSI\_TRNSFRMODE Register Diagram

<!-- image -->

Table 18-79: EMSI\_TRNSFRMODE Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W)            | RESP_INT_DIS | Response Interrupt Disable. The eMSI controller supports the response check function to avoid overhead of re- sponse error check by the host driver. Only a response type of R1 can be checked by the controller. If the host driver checks the response error, set the EMSI_TRNSFRMODE.RESP_INT_DIS bit to 0 and wait for command complete interrupt and then check the response register. If the eMSI controller checks the response error, set the EMSI_TRNSFRMODE.RESP_INT_DIS bit = 1 and set the EMSI_TRNSFRMODE.RESP_ERR_CHK_EN bit = 1. The command complete in- terrupt is disabled by the EMSI_TRNSFRMODE.RESP_INT_DIS bit regardless of the command complete signal enable. 0 Response Interrupt is Enabled |
| 8 (R/W)            | RESP_INT_DIS | 1 Response Interrupt is Disabled                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 8 (R/W)            | RESP_INT_DIS |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 18-79: EMSI\_TRNSFRMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name        | Description/Enumeration                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                     |
|--------------------|-----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | RESP_ERR_CHK_EN | Response Error Check Enable. The eMSI controller supports the response check function to avoid overhead of re- sponse error checking by host driver. Response types of only R1 can be checked by the                                                                                                                                        | Response Error Check Enable. The eMSI controller supports the response check function to avoid overhead of re- sponse error checking by host driver. Response types of only R1 can be checked by the                                                                                                                                        |
| 7 (R/W)            | RESP_ERR_CHK_EN | If the eMSI controller checks the response error, set the EMSI_TRNSFRMODE.RESP_ERR_CHK_EN bit = 1 and set the EMSI_TRNSFRMODE.RESP_INT_DIS bit = 1. If an error is detected, the Re- sponse Error interrupt is generated in the EMSI_ERR_STAT register. Note: Response error check must not be enabled for any response type other than R1. | If the eMSI controller checks the response error, set the EMSI_TRNSFRMODE.RESP_ERR_CHK_EN bit = 1 and set the EMSI_TRNSFRMODE.RESP_INT_DIS bit = 1. If an error is detected, the Re- sponse Error interrupt is generated in the EMSI_ERR_STAT register. Note: Response error check must not be enabled for any response type other than R1. |
| 7 (R/W)            | RESP_ERR_CHK_EN | 0                                                                                                                                                                                                                                                                                                                                           | Response Error Check is Disabled                                                                                                                                                                                                                                                                                                            |
| 6 (R/W)            | RESP_TYPE       | Response Type R1. The EMSI_TRNSFRMODE.RESP_TYPE bit selects R1 as a response type when the Response Error Check is selected. Error statuses checked in R1: - OUT_OF_RANGE - ADDRESS_ERROR - BLOCK_LEN_ERROR - WP_VIOLATION - CARD_IS_LOCKED - COM_CRC_ERROR - CARD_ECC_FAILED - CC_ERROR - ERROR                                            | Response Type R1. The EMSI_TRNSFRMODE.RESP_TYPE bit selects R1 as a response type when the Response Error Check is selected. Error statuses checked in R1: - OUT_OF_RANGE - ADDRESS_ERROR - BLOCK_LEN_ERROR - WP_VIOLATION - CARD_IS_LOCKED - COM_CRC_ERROR - CARD_ECC_FAILED - CC_ERROR - ERROR                                            |
| 6 (R/W)            | RESP_TYPE       | 0                                                                                                                                                                                                                                                                                                                                           | R1 (eMMC)                                                                                                                                                                                                                                                                                                                                   |
| 6 (R/W)            | RESP_TYPE       | 1                                                                                                                                                                                                                                                                                                                                           | Reserved                                                                                                                                                                                                                                                                                                                                    |
| 5 (R/W)            | MULTI_BLK_SEL   | Multi/Single Block Select. The EMSI_TRNSFRMODE.MULTI_BLK_SEL bit is set when issuing multiple- block transfer commands using the DAT line. If the EMSI_TRNSFRMODE.MULTI_BLK_SEL bit is cleared (=0) it is not necessary to set the block count register.                                                                                    | Multi/Single Block Select. The EMSI_TRNSFRMODE.MULTI_BLK_SEL bit is set when issuing multiple- block transfer commands using the DAT line. If the EMSI_TRNSFRMODE.MULTI_BLK_SEL bit is cleared (=0) it is not necessary to set the block count register.                                                                                    |
| 5 (R/W)            | MULTI_BLK_SEL   | 0                                                                                                                                                                                                                                                                                                                                           | Single Block                                                                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | MULTI_BLK_SEL   | 1                                                                                                                                                                                                                                                                                                                                           | Multiple Block                                                                                                                                                                                                                                                                                                                              |

Table 18-79: EMSI\_TRNSFRMODE Register Fields (Continued)

| Bit No. (Access)   | Bit Name       | Description/Enumeration                                                                                                                                                                                                                                                                                                |
|--------------------|----------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 4 (R/W)            | DATA_XFER_DIR  | Data Transfer Direction Select. The EMSI_TRNSFRMODE.DATA_XFER_DIR bit defines the direction of DAT line data transfers. The EMSI_TRNSFRMODE.DATA_XFER_DIR bit is set (=1) by the host driver to transfer data from the eMMC device to the eMSI controller and it is cleared (=0) for all other commands.               |
| 3:2 (R/W)          | AUTO_CMD_EN    | Auto Command Enable. The EMSI_TRNSFRMODE.AUTO_CMD_EN bit field determines the use of Auto Command functions.                                                                                                                                                                                                           |
| 1 (R/W)            | BLOCK_COUNT_EN | Block Count Enable. The EMSI_TRNSFRMODE.BLOCK_COUNT_EN bit is used to enable the block count register, which is relevant for multiple block transfers. If the EMSI_TRNSFRMODE.BLOCK_COUNT_EN bit is cleared (=0), the block count reg- ister is disabled, which is useful in executing an infinite transfer. 0 Disable |
| 0 (R/W)            | DMA_EN         | 1 Enable DMAEnable. The EMSI_TRNSFRMODE.DMA_EN bit enables the DMAfunctionality. If the EMSI_TRNSFRMODE.DMA_EN bit is set (=1), a DMAoperation begins when the host driver writes to the Command register (or EMSI_ADMA_DESADDR_LO for                                                                                 |
|                    |                | 0 No Data Transfer or Non-DMA Data Transfer                                                                                                                                                                                                                                                                            |

## EMSI Version

The EMSI\_VER\_ID register reflects the current release number of the eMSI controller.

Figure 18-102: EMSI\_VER\_ID Register Diagram

<!-- image -->

Table 18-80: EMSI\_VER\_ID Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Validate Mandatory Boot Enable Bit. The EMSI_VER_ID.VALUE bit field indicates the eMSI controller current release number that is read by an application. An application reading this register in conjunc- tion with the register, gathers details of the current release. |

## EMSI Version Type

The EMSI\_VER\_TYPE register reflects the current release type of the eMSI controller.

Figure 18-103: EMSI\_VER\_TYPE Register Diagram

<!-- image -->

Table 18-81: EMSI\_VER\_TYPE Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Current Release Type. This field indicates the eMSI controller current release type that is read by an applica- tion. An application reading this register in conjunction with the register, gathers de- tails of the current release. |

## Wakeup Control Register

The EMSI\_WU\_CTL register is mandatory for the Host Controller, but the wakeup functionality depends on the Host Controller system hardware and software. The Host Driver maintains voltage on the SD Bus by setting the SD Bus Power to 1 in the Power Control Register, while a wakeup event through the Card Interrupt is desired.

Figure 18-104: EMSI\_WU\_CTL Register Diagram

<!-- image -->

Table 18-82: EMSI\_WU\_CTL Register Fields

| Bit No. (Access)   | Bit Name     | Description/Enumeration                                                                                                                                                                                                                                                                                                          |
|--------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | CARD_REMOVAL | Wakeup Event Enable on SD Card Removal. The EMSI_WU_CTL.CARD_REMOVAL bit enables wakeup event through Card Re- moval assertion in the Normal Interrupt Status register. For the SDIO card, Wake Up Support (FN_WUS) in the Card Information Structure (CIS) register does not affect the EMSI_WU_CTL.CARD_REMOVAL bit. 0 Disable |
| 1 (R/W)            | CARD_INSERT  | Wakeup Event Enable on SD Card Insertion. The EMSI_WU_CTL.CARD_INSERT bit enables wakeup event through Card Inser- tion assertion in the Normal Interrupt Status register. FN_WUS (Wake Up Support) in CIS does not affect the EMSI_WU_CTL.CARD_INSERT bit. 0 Disable                                                            |
| 1 (R/W)            |              | 1 Enable                                                                                                                                                                                                                                                                                                                         |
| 1 (R/W)            |              |                                                                                                                                                                                                                                                                                                                                  |