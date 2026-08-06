# Direct Memory Access (DMA)

<!-- source: 310_Direct_Memory_Access_DMA.pdf | original pages 2989–3058 -->

## 36   Direct Memory Access (DMA)

The processor architecture distributes the DMA channels throughout the infrastructure. Often, the channels cluster together through system crossbars (SCB), sharing a single interface with the main system crossbar.

The DMA channels can perform transfers between memory and a peripheral or between one memory and another memory. Memory-to-memory DMA transfers (MDMA) require two DMA channels. One channel is the source channel, and the second, the destination channel.

All DMA channels can transport data to and from virtually all on-chip and off-chip memories.

DMA transfers on the processor use either a descriptor-based method or register-based method. Register-based DMA allows the processor directly to program DMA controller registers to initiate a DMA transfer. On completion, the controller registers can automatically update with their original setup values for continuous transfer, if needed. Descriptor-based DMA transfers require a set of parameters stored within memory to initiate a DMA sequence. Descriptor-based transfers allow the chaining together of multiple DMA sequences. In descriptor-based DMA operations, DMA channel programming can automatically set up and start another DMA transfer after the current sequence completes.

The DMA channel does not connect external memories and devices directly. Rather, data passes through an external-memory interface port. DMA operations can access any device the external memory interface supports. These interfaces typically include:

- Flash memory
- SRAM
- FIFOs
- Memory-mapped peripheral devices
- Dynamic Memory (if present)

## DMA Channel Features

The processor uses Direct Memory Access (DMA) to transfer data within memory spaces or between a memory space and a peripheral. The processor can specify data transfer operations and return to normal processing while the fully integrated DMA channel carries out the data transfers independent of processor activity. The DMA channels are dispersed throughout the infrastructure and interface with the system crossbar unit (SCB).

The following is a list of DMA interface features.

- Supports integer byte strides including byte strides of 0 and negative byte strides
- Register-based configuration
- Core writes DMA configuration
- Supports automatic reloading for continuous operation
- Flexible descriptor-based configuration
- DMA descriptors are fetched from memory
- Support for variable descriptor sizes
- Flexible flow control - T ransitions between the various descriptor-based modes and for DMA termination
- Orthogonal transfers
- Support for three transfer dimensions
- One and two dimensional (1D and 2D) transfers supported per descriptor set
- Three dimensional (3D) support provided by chained descriptor sets
- Configurable memory and peripheral-transfer word sizes
- Memory interface supports 8-bit, 16-bit, 32-bit, 64-bit, 128-bit, and 256-bit transfers
- Peripheral interface supports for 8-bit, 16-bit, and 32-bit transfers
- Interrupt notification
- Row or work unit completion
- Error conditions
- Incoming and outgoing trigger support
- Trigger generation for row or work unit completion
- Work unit can wait for incoming trigger
- MMR access bus - Provides access to memory-mapped registers for configuration, monitoring, and debug
- SCB crossbar interface connects the DMA channel to the system crossbar
- Peripheral DMA bus - Interfaces the DMA channel to a peripheral or another DMA channel
- Peripheral data-request interrupt support
- Bandwidth monitoring and limiting for MDMA channels

## DMA Channel Functional Description

This section provides a functional description of the DMA channel interface.

## ADSP-SC59x DMA Register List

The DMA channel controller (DMA) supports data transfers within memory spaces or between a memory space and a peripheral. The processor can specify data transfer operations and return to normal processing while the fully integrated DMA channel carries out the data transfers independent of processor activity. The DMA channels are dispersed throughout the infrastructure, as DMAn's. A set of registers governs DMA operations. For more information on DMA functionality, see the DMA register descriptions.

Table 36-1: ADSP-SC59x DMA Register List

| Name           | Description                                        |
|----------------|----------------------------------------------------|
| DMA_ADDRSTART  | Start Address of Current Buffer Register           |
| DMA_ADDR_CUR   | Current Address Register                           |
| DMA_BWLCNT     | Bandwidth Limit Count Register                     |
| DMA_BWLCNT_CUR | Bandwidth Limit Count Current Register             |
| DMA_BWMCNT     | Bandwidth Monitor Count Register                   |
| DMA_BWMCNT_CUR | Bandwidth Monitor Count Current Register           |
| DMA_CFG        | Configuration Register                             |
| DMA_DSCPTR_CUR | Current Descriptor Pointer Register                |
| DMA_DSCPTR_NXT | Pointer to Next Initial Descriptor Register        |
| DMA_DSCPTR_PRV | Previous Initial Descriptor Pointer Register       |
| DMA_STAT       | Status Register                                    |
| DMA_XCNT       | Inner Loop Count Start Value Register              |
| DMA_XCNT_CUR   | Current Count (1D) or Intra-row XCNT (2D) Register |
| DMA_XMOD       | Inner Loop Address Increment Register              |
| DMA_YCNT       | Outer Loop Count Start Value (2D only) Register    |
| DMA_YCNT_CUR   | Current Row Count (2D only) Register               |
| DMA_YMOD       | Outer Loop Address Increment (2D only) Register    |

## ADSP-SC59x DMA Channel List

Table 36-2: ADSP-SC59x DMA Channel List

| DMAID   | DMAChannel Name   | Description        |
|---------|-------------------|--------------------|
| DMA0    | SPORT0_A_DMA      | SPORT0 ChannelADMA |
| DMA1    | SPORT0_B_DMA      | SPORT0 ChannelBDMA |

Table 36-2: ADSP-SC59x DMA Channel List (Continued)

| DMAID   | DMAChannel Name   | Description                            |
|---------|-------------------|----------------------------------------|
| DMA2    | SPORT1_A_DMA      | SPORT1 ChannelADMA                     |
| DMA3    | SPORT1_B_DMA      | SPORT1 ChannelBDMA                     |
| DMA4    | SPORT2_A_DMA      | SPORT2 ChannelADMA                     |
| DMA5    | SPORT2_B_DMA      | SPORT2 ChannelBDMA                     |
| DMA6    | SPORT3_A_DMA      | SPORT3 ChannelADMA                     |
| DMA7    | SPORT3_B_DMA      | SPORT3 ChannelBDMA                     |
| DMA8    | MDMA0_SRC         | Memory DMAStream 0 Source Channel      |
| DMA9    | MDMA0_DST         | Memory DMAStream 0 Destination Channel |
| DMA10   | SPORT4_A_DMA      | SPORT4 ChannelADMA                     |
| DMA11   | SPORT4_B_DMA      | SPORT4 ChannelBDMA                     |
| DMA12   | SPORT5_A_DMA      | SPORT5 ChannelADMA                     |
| DMA13   | SPORT5_B_DMA      | SPORT5 ChannelBDMA                     |
| DMA14   | SPORT6_A_DMA      | SPORT6 ChannelADMA                     |
| DMA15   | SPORT6_B_DMA      | SPORT6 ChannelBDMA                     |
| DMA16   | SPORT7_A_DMA      | SPORT7 ChannelADMA                     |
| DMA17   | SPORT7_B_DMA      | SPORT7 ChannelBDMA                     |
| DMA18   | MDMA1_SRC         | Memory DMAStream 1 Source Channel      |
| DMA19   | MDMA1_DST         | Memory DMAStream 1 Destination Channel |
| DMA20   | UART0_TXDMA       | UART0 TransmitDMA                      |
| DMA21   | UART0_RXDMA       | UART0 ReceiveDMA                       |
| DMA22   | SPI0_TXDMA        | SPI0 TX DMAChannel                     |
| DMA23   | SPI0_RXDMA        | SPI0 RX DMAChannel                     |
| DMA24   | SPI1_TXDMA        | SPI1 TX DMAChannel                     |
| DMA25   | SPI1_RXDMA        | SPI1 RX DMAChannel                     |
| DMA26   | SPI2_TXDMA        | SPI2 TX DMAChannel                     |
| DMA27   | SPI2_RXDMA        | SPI2 RX DMAChannel                     |
| DMA28   | EPPI0_CH0_DMA     | EPPI0 Channel0DMA                      |
| DMA29   | EPPI0_CH1_DMA     | EPPI0 Channel1DMA                      |
| DMA30   | LP0_DMA           | LP0 DMAChannel                         |

Table 36-2: ADSP-SC59x DMA Channel List (Continued)

| DMAID   | DMAChannel Name   | Description                            |
|---------|-------------------|----------------------------------------|
| DMA34   | UART1_TXDMA       | UART1 TransmitDMA                      |
| DMA35   | UART1_RXDMA       | UART1 ReceiveDMA                       |
| DMA36   | LP1_DMA           | LP1 DMAChannel                         |
| DMA37   | UART2_TXDMA       | UART2 TransmitDMA                      |
| DMA38   | UART2_RXDMA       | UART2 ReceiveDMA                       |
| DMA39   | MDMA2_SRC         | Memory DMAStream 2 Source Channel      |
| DMA40   | MDMA2_DST         | Memory DMAStream 2 Destination Channel |
| DMA43   | MDMA3_SRC         | Memory DMAStream 3 Source Channel      |
| DMA44   | MDMA3_DST         | Memory DMAStream 3 Destination Channel |
| DMA45   | MDMA4_SRC         | Memory DMAStream 4 Source Channel      |
| DMA46   | MDMA4_DST         | Memory DMAStream 4 Destination Channel |
| DMA47   | MDMA5_SRC         | Memory DMAStream 5 Source Channel      |
| DMA48   | MDMA5_DST         | Memory DMAStream 5 Destination Channel |
| DMA49   | MDMA6_SRC         | Memory DMAStream 6 Source Channel      |
| DMA50   | MDMA6_DST         | Memory DMAStream 6 Destination Channel |
| DMA51   | MDMA7_SRC         | Memory DMAStream 7 Source Channel      |
| DMA52   | MDMA7_DST         | Memory DMAStream 7 Destination Channel |
| DMA53   | UART3_TXDMA       | UART3 TransmitDMA                      |
| DMA54   | UART3_RXDMA       | UART3 ReceiveDMA                       |
| DMA55   | SPI3_TXDMA        | SPI3 TX DMAChannel                     |
| DMA56   | SPI3_RXDMA        | SPI3 RX DMAChannel                     |

## DMA Definitions

To make the best use of the DMA controller, it is useful to understand the following terms.

## Descriptor

A single element of a descriptor set that maps to a specific register of a DMA channel.

## Descriptor Fetch

The action of retrieving descriptors from memory through memory read operations and loading then into the DMA channel registers upon their read return.

## Descriptor Set

An array of descriptors associated with a single work unit. The user can configure the size of the descriptor set. See Descriptor-Based Flow Modes .

## Disabled State

The channel is disabled because the enable bit = 0 or because of an error.

## DMAC

An acronym used for a DMA cluster.

## DMA Channel

A single DMA engine that has all the capabilities and registers as defined for a given processor. A DMA channel or engine is connected to a single peripheral.

## DMA Cluster

A grouping of multiple DMA channels with a shared SCB crossbar interface, controller, and arbiter. Also known as a DMAC.

## Initial Descriptor

The first descriptor in the descriptor set.

## MDMA

Memory-to-Memory DMA data transfer. Two DMA channels are paired to perform a memory read from one address location and a memory write of that data to another address location.

## Stop State

A time where the channel is enabled but not currently programmed to perform a data transfer. Programming the flow to STOP causes the channel to enter the stop state at the end of the work unit.

## User

Any person, debug, emulator, software routine, or action taken by the core that accesses the MMR registers of the DMA channel or peripherals, or sets up data and descriptors in memory.

## Wait State

If instructed to wait for a trigger, the channel enters this state once it has completed a work unit. The channel remains in this state until a trigger occurs. If a trigger came in before reaching the wait state, the channel skips over the wait state upon completion of the work unit.

## Work Unit

A single data transaction or series of data transactions performed based on the configuration of the DMA channel. For autobuffer mode, a new work unit is defined at the time all current count registers are initialized to start values. Once all the current count registers count down to zero, the work unit has completed.

## Work Unit Chain

A single work unit or a series of work units separated by a STOP or disabled state. The work units in the chain (except the last one) are programmed to the required descriptor flow. The last work unit in the chain is programmed to a flow of STOP or AUTO . STOP terminates the state at the end of that work unit. AUTO must be terminated by disabling the DMA channel. A work unit chain is also known as a descriptor chain.

## Block Diagram

The DMA Channel Block Diagram shows the functional blocks within the DMA interface.

Figure 36-1: DMA Channel Block Diagram

<!-- image -->

For more information on the interfaces, see:

- DMA Channel Peripheral DMA Bus
- Medium Band Width DMA Channel MMR Access Bus
- DMA Channel Event Control

## · DMA Channel SCB Interface

## Architectural Concepts

The DMA channel provides a method to transfer data between memory spaces or between memory and a peripheral using a number of system interfaces. The DMA channel provides an efficient method of distributing data throughout the system, freeing up the processor core for other operations. Each peripheral that supports DMA transfers has its own dedicated DMA channel or channels with its own register set. The register set configures and controls the operating modes of the DMA transfers.

## DMA Channel SCB Interface

The SCB interface connects the DMA channel to the SCB crossbar allowing for transfers to and from the processors internal memory and other suitable system resources.

The DMA channel connects to the system interconnect through the SCB interface. This connection lets the DMA channel perform work-unit data transfers with memories such as L1, L2 (internal), and L3 (external). In addition to work unit data transfers, the SCB interface also is used for fetching descriptor sets for all the descriptor-based transfer modes.

The DMA channel can support data bus widths of 16, 32, 64, or 128 bits. The data bus widths for a given DMA channel on a specific processor can vary and are not configurable. Read the DMA\_STAT.MBWID field to determine the assigned bus widths.

## SCB Interface Signals

The DMA channel operates at one of the SCLKn frequencies, as does the SCB interface. SYSCLK clocks all eight DMA channels. The SCB crossbar handles the internal arbitration of the transfer requests of all the controllers interfaced to the SCB crossbar instance as shown in the SCB Interface Signals table.

Table 36-3: SCB Interface Signals

| Signal            | Width (bits)       | Description                                                                                      |
|-------------------|--------------------|--------------------------------------------------------------------------------------------------|
| SCB_WRITE_DATA    | 16, 32, 64, or 128 | Data bus used for write operations. The width of the bus can be determined from DMA_STAT.MBWID . |
| SCB_WRITE_ADDRESS | 32                 | Write address bus. Provides the address of the first transfer in a burst transaction.            |
| SCB_READ_DATA     | 16, 32, 64, or 128 | Data bus used for read operations. The width of the bus can be determined from DMA_STAT.MBWID .  |
| SCB_READ_ADDRESS  | 32                 | Read address bus. Provides the address of the first transfer in a burst transaction.             |

## SCB Burst Transfers

The SCB interface supports burst transfers for memory read and write operations. The burst length is a function of the configurable memory size of the DMA channel for the work unit and the fixed bus width of the SCB data bus of the DMA channel.

- If the DMA channel configuration selects a memory transfer size less than or equal to the DMA channels bus width, the burst length is always 1.
- If the configured memory size is greater than the SCB interface bus width, the burst length is sufficient to transfer a transaction as specified by the configured memory size.

Table 36-4: DMA Channel SCB Burst Lengths

| Configured Memory Size   | Burst Length   | Burst Length   | Burst Length   | Burst Length   |
|--------------------------|----------------|----------------|----------------|----------------|
|                          | 16-bit Bus     | 32-bit Bus     | 64-Bit Bus     | 128-bit Bus    |
| 1 Byte                   | 1              | 1              | 1              | 1              |
| 2 Bytes                  | 1              | 1              | 1              | 1              |
| 4 Bytes                  | 2              | 1              | 1              | 1              |
| 8 Bytes                  | 4              | 2              | 1              | 1              |
| 16 Bytes                 | 8              | 4              | 2              | 1              |
| 32 Bytes                 | 16             | 8              | 4              | 2              |

## Data Address Alignment

To prevent addressing errors and to maximize bandwidth of the SCB interface to the DMA channel, data addresses align with a multiple of the programmable memory size of the DMA channels configuration. These configuration options appear in the Descriptor Set Address Alignment table.

There are situations in which entire work units may not transfer at the maximum configurable memory size. In this case, the entire work unit can transfer by reducing the configured memory size at the expense of bus bandwidth using descriptor sets as follows:

- The first descriptor set can be configured to transfer data until the larger memory size alignments are met.
- A second descriptor set with a larger memory size configuration then can be used to transfer the bulk of the data in the work unit.
- Finally, a third descriptor set can be used with a smaller memory size to complete any final data transfers that cannot meet the alignment requirements of the previous descriptor set configuration.

Table 36-5: DMA Channel Address Alignment Requirements

| Configured Memory Size   | Address Restriction   |
|--------------------------|-----------------------|
| 1 Byte                   | No restriction        |
| 2 Bytes                  | ADDR[0] == 0          |
| 4 Bytes                  | ADDR[1:0] == 0        |
| 8 Bytes                  | ADDR[2:0] == 0        |
| 16 Bytes                 | ADDR[3:0] == 0        |
| 32 Bytes                 | ADDR[4:0] == 0        |

## Descriptor Set Address Alignment

All descriptor set addresses and descriptors within a descriptor set must align to a 32-bit address. For descriptor set fetches, the DMA engine ignores the memory-size configuration of the DMA channel. This feature avoids the need to align descriptor sets based on the memory width configuration of the previous descriptor set.

For descriptor sets containing only a single descriptor, the transfer takes place as a single 32-bit transfer. For descriptor sets containing multiple descriptors, the DMA engine fetches each 32-bit descriptor individually and treats it as multiple 32-bit transfers.

## Peripheral Control Commands

The peripheral DMA bus of the DMA channel provides a means for peripherals on the processor to issue commands to the DMA channel. These commands provide greater control over the DMA channel operation. This control improves real-time performance and relieves control and interrupt demands on the core. Peripherals can send commands to the DMA controller over the 3-bit PERI\_CMD bus. The DMA control commands extend the set of operations available to the peripheral beyond the simple 'request data' command used by peripherals in general. Refer to the appropriate peripheral chapter for a description on how that peripheral uses DMA control commands.

These DMA control commands (see the PDMA\_CMD Peripheral DMA Control Commands table) are not visible to or controlled by the program. But, their use by a peripheral has implications for the structure of the DMA transfers that the peripheral can support. It is important to write application software such that it complies with certain restrictions, regarding work units and descriptor chains. Complying with this guideline makes the peripheral operate properly whenever it issues DMA control commands.

The PDMA\_CMD Peripheral DMA Control Commands table describes the commands the DMA controller issues. The following sections describe these commands in more detail.

Table 36-6: PDMA\_CMD Peripheral DMA Control Commands

| Command   | Name                | Description                                                      |
|-----------|---------------------|------------------------------------------------------------------|
| b#000     | NOP                 | No operation                                                     |
| b#001     | Restart             | Restarts the current work unit from the beginning                |
| b#010     | Finish              | Finishes the current work unit and starts the next               |
| b#011     | Interrupt           | Immediately sets the DMAcompletion interrupt in the DMAchan- nel |
| b#100     | Request Data        | Typical DMAdata request                                          |
| b#101     | Request Data Urgent | Urgent DMAdata request                                           |
| b#110     | Reserved            | Reserved                                                         |
| b#111     | Reserved            | Reserved                                                         |

## Idle Command

The DMA channel drives this command when the enabled peripheral has no data requests required.

## Request-Data Command

The request data command is a request for data transfers between the DMA channel and the peripheral. The request is held by the peripheral until granted or acknowledged by the DMA channel.

## Request-Data Urgent Command

The request-data urgent command behaves identically to the request data command, except that---during the commands assertion---the DMA channel performs its memory accesses with urgent priority. This priority includes both data and descriptor fetch memory accesses. For example, a DMA management capable peripheral can use this control command if an internal FIFO approaches a critical condition.

The request is held by the peripheral until granted or acknowledged by the DMA channel.

## Peripheral-Control Command Restrictions

The proper operation of the DMA channel FIFO leads to certain restrictions in the sequence of DMA peripheral control commands issued by a peripheral. The following sections describe these restrictions.

## DMA Channel Peripheral DMA Bus

The DMA channel connects to peripherals or other DMA channels through the peripheral DMA bus. This bus is a dedicated point-to-point interface supporting data bus widths of 8, 16, 32, or 64 bits. The data bus widths for a given DMA channel on a particular processor can vary and are not configurable. Reading the DMA\_STAT.PBWID field permits determining the assigned bus width.

The DMA channel operates at one of the SCLK frequencies, as does the peripheral DMA bus. The Peripheral DMA Bus Signals table provides descriptions of the peripheral DMA bus signals.

Table 36-7: Peripheral DMA Bus Signals

| Signal          | Width (bits)     | Description                                                                                                                                                                                                           |
|-----------------|------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| PDMA_WRITE_DATA | 8, 16, 32, or 64 | Data bus used for write operations. The width of the bus can be deter- mined from DMA_STAT.PBWID .                                                                                                                    |
| PDMA_READ_DATA  | 8, 16, 32, or 64 | Data bus used for read operations. The width of the bus can be deter- mined from DMA_STAT.PBWID .                                                                                                                     |
| PDMA_DMA_GRANT  |                  | Control signals to indicate that data is valid for DMAchannel read opera- tions (peripheral transmit). These signals indicate that the DMAchannel is ready to receive data for write operations (peripheral receive). |
| PDMA_CMD        | 3                | The peripheral uses the signal for issuing DMAchannel control com- mands.                                                                                                                                             |
| PDMA_CTRL       |                  | The peripheral uses the control signals to send various commands to the DMAchannel and control the direction of flow.                                                                                                 |

## Memory DMA and Triggering

A memory DMA (MDMA) channel provides a means of doing memory-to-memory DMA transfers among the various memory spaces that have DMA support.

The DMA controller implements memory DMA (MDMA) channels by interfacing two DMA channels through the peripheral DMA bus interface. One DMA channel serves for memory read operations, and the second channel servers for memory writes. Depending on the processor, a memory DMA channel can have an additional peripheral, such as a CRC peripheral. The additional peripheral is inserted into the peripheral DMA bus that optionally can be enabled.

MDMA channel configurations that do not involve an additional peripheral impose no restrictions on which of the DMA channels is used for the read operation or the write operation. But, the configuration of both channels cannot have the same transfer direction. For MDMA channel configurations that enable a peripheral between the read and write channels, be aware of possible restrictions imposed on which channel can be used for a given transfer direction.

Figure 36-2: MDMA Channel Dedicated Pair

<!-- image -->

Figure 36-3: MDMA Channel Pair with Peripheral

A memory-to-memory transfer always requires enabled source and destination channels. Because the channels interface through the peripheral DMA bus and can have an additional peripheral inserted into the peripheral DMA bus, programs must make sure to set the same values in the DMA\_CFG.PSIZE of both the source and destination channels.

The memory DMA channels support the full range of the DMA\_CFG.MSIZE options for the DMA transfers to and from the memories.

Because the MDMA channel consists of two DMA channels, the entire MDMA channel has two sets of FIFOs, one in the read channel and one in the write channel. This FIFO usage allows for more efficient bursting of both read and write transactions using the available bandwidth. While the DMA\_CFG.PSIZE configuration must be identical for both source and destination DMA channels, this restriction does not apply for the DMA\_CFG.MSIZE configuration.

Configure the DMA\_CFG.PSIZE bits to a value no larger than the supported bus width of the peripheral DMA bus. From a performance perspective, use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the PSIZE value used.

NOTE: This is applicable for all of the DMA channels, except the enhanced and high-speed MDMA channels. For the enhanced and high-speed MDMAs, the minimum PSIZE and MSIZE is 4 bytes (32 bits).

The independent source and destination DMA channels also have their own dedicated interrupt and trigger events. While it is normal practice to have only event generation performed at destination DMA completion, programs also can use other means of interrupt generation.

Configuration of an MDMA transfer is done in a similar manner to peripheral DMA transfers, except for writing two DMA channel registers instead of one.

To control the pace of data transfers, use triggers on either the memory read or the memory write channel pair used in an MDMA operation. Setting the DMA\_CFG.TWAIT bit in the memory read channel prevents both channels from transferring data before the system is ready. However, only configuring the memory write channel to wait for a trigger allows for data fetch from the memory in anticipation of the memory write operation.

The MDMA Streams table shows the details for each supported MDMA stream.

Table 36-8: MDMA Streams

|   MDMA Stream |   Source Channel |   Destination Channel | CRC Support   | Other Names   | Operating Clock      |   Bus Width (bits) |   Maximum Theoretical BW(Mbytes/ second) |
|---------------|------------------|-----------------------|---------------|---------------|----------------------|--------------------|------------------------------------------|
|             0 |                8 |                     9 | Yes           | CRC0          | SYSCLK (500 MHz max) |                 32 |                                     2000 |
|             1 |               18 |                    19 | Yes           | CRC1          | SYSCLK (500 MHz max) |                 32 |                                     2000 |
|             2 |               39 |                    40 | No            | MSMDMA        | SYSCLK (500 MHz max) |                 32 |                                     2000 |
|             3 |               43 |                    44 | No            | HSMDMA        | SYSCLK (500 MHz max) |                 64 |                                     4000 |
|             4 |               45 |                    46 | Yes           | CRC2          | SYSCLK (500 MHz max) |                 32 |                                     2000 |
|             5 |               47 |                    48 | Yes           | CRC3          | SYSCLK (500 MHz max) |                 32 |                                     2000 |
|             6 |               49 |                    50 | No            | MSMDMA1       | SYSCLK (500 MHz max) |                 32 |                                     2000 |

Table 36-8: MDMA Streams (Continued)

|   MDMA Stream |   Source Channel |   Destination Channel | CRC Support   | Other Names   | Operating Clock   |   Bus Width (bits) |   Maximum Theoretical BW(Mbytes/ second) |
|---------------|------------------|-----------------------|---------------|---------------|-------------------|--------------------|------------------------------------------|
|             7 |               51 |                    52 | No            | HSMDMA1       |                   |                 64 |                                     4000 |

## Medium Band Width DMA Channel MMR Access Bus

The MMR access bus provides access to all the DMA channels memory-mapped registers for DMA channel configuration, monitoring, and debug. The interface has a fixed 32-bit data bus for read and write accesses.

The MMR Access Bus Signals table provides descriptions of the MMR access bus signals.

Table 36-9: MMR Access Bus Signals

| Signal         |   Width (bits) | Description                                                  |
|----------------|----------------|--------------------------------------------------------------|
| MMR_WRITE_DATA |             32 | Data bus used for write operations to the MMRs from the core |
| MMR_READ_DATA  |             32 | Data bus used to return read data from the MMRs              |
| MMR_READ_ADDR  |              7 | Address used to select the MMRto access                      |

## DMA Channel Operation Flow

A detailed description of the flow of operation of the DMA channel appears in the following topics:

- Startup Flow
- Refresh Flow
- DMA Operating Modes
- Stop Mode
- DMA Channel Errors

## Startup Flow

Enabling a DMA operation on a given channel first requires directly writing some or all of the DMA parameter registers. The minimum set of register required to be initialized depends on the desired mode of operation as described in the following sections.

## Startup Minimum-Enable Requirements

To start a DMA operation on a given channel, some or all of the DMA parameter registers must first be initialized and configured to the desired DMA channels operating mode.

- For descriptor-array-based flow modes, at minimum, write the DMA\_DSCPTR\_CUR register prior to writing to the DMA\_CFG register, which is the special action required to start the DMA channel.

- For descriptor-list-based flow modes, at minimum, write the DMA\_DSCPTR\_NXT register prior to writing to the DMA\_CFG register, which is the special action required to start the DMA channel.
- For non-descriptor-based flow modes, write the DMA\_ADDRSTART , DMA\_XCNT , and DMA\_XMOD registers prior to writing the DMA\_CFG register.

Programs can write other registers that can remain static throughout the course of the DMA activity. The write to the DMA\_CFG register begins the DMA operation.

- ATTENTION: When software directly writes the DMA\_CFG register, the DMA controller recognizes this action as the special startup condition. This condition occurs when starting the DMA controller for the first time on this channel or occurs after the DMA channel stops. It is possible for the channel to flag a DMA error condition regardless of the DMA\_CFG.EN bit setting.

## Startup Operation

The startup operation is initiated by software directly writing the DMA\_CFG register when starting DMA for the first time on a channel or after the channel has entered to the stop state.

When the descriptor fetch is complete and the DMA channel is enabled, the DMA\_CFG descriptor element replaces the DMA\_CFG register content and assumes control. Before this point, the direct write to the DMA\_CFG register had control.

At startup, the selected flow mode and the descriptor size determine the course of the DMA initialization process. The DMA\_CFG.FLOW field determines whether to load more current registers from descriptor sets in memory. The DMA\_CFG.NDSIZE field details how many descriptor elements to fetch before starting the DMA operation. This process does not affect DMA registers that are not in the descriptor; no modifications are made to their prior values.

For descriptor-list flow modes, the channel copies the DMA\_DSCPTR\_NXT register value into the DMA\_DSCPTR\_CUR register. Then, the channel fetches new descriptor elements from memory. The DMA\_DSCPTR\_CUR register indexes each fetch, and the channel increments the index after each fetch. After completion of the descriptor fetch, the DMA\_DSCPTR\_CUR register points to the next 32-bit word in memory past the end of the descriptor.

If the descriptor fetch is for a descriptor-array mode transfer, the channel does not copy the DMA\_DSCPTR\_NXT register into the DMA\_DSCPTR\_CUR register. Instead , the descriptor fetch indexing begins with the value in the DMA\_DSCPTR\_CUR register.

If DMA\_CFG is not part of the fetched descriptor set, the previous value (originally as written on startup) controls the work unit operation. If the DMA\_CFG register is part of the fetched descriptor set, the value programmed by the MMR access controls only the loading of the first descriptor set fetched from memory. The configuration of the DMA\_CFG register controls the subsequent DMA work units of the fetched descriptor set.

After the descriptor fetch is complete or if the flow configuration was originally for one of the register-based flow modes, the DMA operation begins. The DMA channel immediately fills its FIFO. For a memory-write operation, the DMA channel begins accepting data from the peripheral. For a memory-read operation, the DMA channel begins memory reads when the SCB bus grants access to the DMA channel.

When the DMA channel performs its first data-memory access, its address and count computations take their input operands from the start registers. These registers can include DMA\_ADDRSTART , DMA\_XCNT , and DMA\_YCNT , if necessary. The channel writes results back to the current registers. These registers include DMA\_ADDR\_CUR , DMA\_XCNT\_CUR , and DMA\_YCNT\_CUR . Note that the current registers are not valid until the channel performs the first memory access, which can be some time after the write to the DMA\_CFG register starts the channel. Once started, the channel automatically loads the current registers from the appropriate descriptor elements, overwriting their previous contents. These automatic-load operations include:

- The channel copies the DMA\_ADDRSTART value to DMA\_ADDR\_CUR .
- The channel copies the DMA\_XCNT value to DMA\_XCNT\_CUR .
- The channel copies the DMA\_YCNT to DMA\_YCNT\_CUR .

## Refresh Flow

When the channel completes processing of a work unit, the DMA channel performs the following operations:

- Completes the transfer of all data between memory and the DMA channel.
- Performs a synchronized transition (if the DMA channel configuration is a memory read operation with the DMA\_CFG.SYNC bit enabled) and transfers all data to the peripheral before continuing.
- Forwards the signals from the DMA channel (if interrupts or triggers are enabled) and updates the DMA\_STAT register to indicate the interrupt request or trigger events.
- Clears the DMA\_STAT.RUN bit field to stop DMA operation (if the flow was set to stop mode) and transfers any remaining data in the FIFO of the DMA channel to the peripheral.
- Loads a new descriptor from memory into the DMA registers by way of the contents of the DMA\_DSCPTR\_CUR register (for descriptor-array mode) and increments the DMA\_DSCPTR\_CUR register
- The channel takes the descriptor size from the DMA\_CFG.NDSIZE value before the fetch.
- Copies the DMA\_DSCPTR\_NXT register into the DMA\_DSCPTR\_CUR register (for descriptor-list mode), fetches the descriptor from the new contents of the DMA\_DSCPTR\_CUR register, and places these contents into the DMA registers while incrementing the DMA\_DSCPTR\_CUR register.
- Checks for detection of an incoming trigger event (for descriptor-on-demand array mode):
- If the channel detects a trigger event, the DMA channel loads a new descriptor from memory into the DMA registers from the contents of the DMA\_DSCPTR\_CUR register, while incrementing the DMA\_DSCPTR\_CUR register. The channel takes the descriptor size from the DMA\_CFG.NDSIZE value before the fetch.
- If the channel detects no trigger event, the DMA channel begins the next work unit by reloading the current registers.
- Checks for detection of an incoming trigger event (for descriptor-on-demand list mode):

- If the channel detects a trigger event, the DMA channel copies the DMA\_DSCPTR\_NXT register value to the DMA\_DSCPTR\_CUR register, fetches the descriptor memory from the DMA\_DSCPTR\_CUR register, and places the contents into the DMA registers while incrementing the DMA\_DSCPTR\_CUR register.
- If the channel detects no trigger event, the DMA channel begins the next work unit by reloading the current registers as described in the next step.
- Begins the next work unit (if flow configuration is anything other than stop mode) by reloading the current registers ( DMA\_ADDR\_CUR , DMA\_XCNT\_CUR , and DMA\_YCNT\_CUR ) from their descriptor registers ( DMA\_ADDRSTART , DMA\_XCNT , and DMA\_YCNT )

## Work Unit Transition Flow

The DMA\_CFG.SYNC bit controls transitions from one work unit to the next work unit. In general, continuous transitions have lower latency at the cost of restrictions on changes of data format or addressed memory space in the two work units. These latency gains and data restrictions arise from the way the channel handles the DMA FIFO while fetching the next descriptor.

In continuous transitions, with disabled synchronization, the DMA FIFO pipeline continues to transfer data to and from the peripheral or destination memory. These transfers continue during the descriptor fetch and during the DMA channel pause between descriptor chains. By comparison, synchronized transitions provide better real-time synchronization of interrupts and triggers with a given peripheral state. Synchronized transitions also provide greater flexibility in the data formats and memory spaces of the two work units. This flexibility comes at the cost of higher latency in the transition. In synchronized transitions, the DMA FIFO pipeline drains to the destination or flushes (received data discarded) between work units.

NOTE: The DMA\_CFG.SYNC bit of the MDMA source channel controls work unit transitions for MDMA streams. Clear this reserved bit of the MDMA destination channel, placing it in the disabled state. In transmit (memory read) channels, the DMA\_CFG.SYNC bit of the last descriptor before the transition controls the transition behavior. In contrast, in receive channels, the DMA\_CFG.SYNC bit of the first descriptor of the next descriptor chain controls the transition.

## Work Unit Transmit and MDMA Source Transitions

In DMA transmit (memory read) and MDMA source channels, the DMA\_CFG.SYNC bit controls the interrupt timing at the end of the work unit. This bit also controls the handling of the DMA FIFO between the current and the next work unit.

If the DMA\_CFG.SYNC bit configuration disables synchronization, the DMA channel operates in continuous transition. In a continuous transition, just after reading the last data item from memory, the DMA channel starts all of the following operations parallel:

- Signals the interrupt request or trigger
- Updates the DMA\_STAT register to indicate DMA completion status
- Begins fetching the next descriptor
- Delivers the final data items from the DMA FIFO to the destination memory or peripheral

This process lets the DMA channel provide data from the FIFO to the peripheral continuously during the descriptor fetch latency period.

If the configuration disables synchronization, the final interrupt request or trigger (if enabled) occurs when the channel reads the last data from memory. This event occurs at the earliest time that the channel safely can modify the output memory buffer without affecting the previous data transmission. There can be a number of data items remaining in the FIFO and not yet at the peripheral. This number depends on the FIFO depth of the DMA channel. In this configuration, do not use the DMA interrupt request as the sole means of synchronizing the shutdown or reconfiguration of the peripheral following a transmission.

NOTE: If the configuration selects continuous transition on a transmit (memory read) descriptor, the next descriptor must have the same:

- Peripheral transfer size ( DMA\_CFG.PSIZE )
- Read or write direction
- Source memory (internal versus external) as the current descriptor

It is possible to disable synchronization by selecting continuous transition on a work unit with configuration for stop-flow mode and with enabled interrupts or triggers. This approach can result in the execution of the event service routine while draining of the final data is ongoing from the FIFO to the peripheral. If data transfers are in-progress, the FIFO is not yet empty. The DMA\_STAT.RUN bits of the DMA channels indicate this status. Do not start a new work unit with a different peripheral transfer size or direction while data transfers are in-progress.

CAUTION: Disabling the channel with the DMA\_CFG.EN bit while data transfers are in-progress causes the loss of the data in the FIFO.

A synchronized transition configuration directs the channel to drain the DMA FIFO to the destination memory or peripheral. This FIFO operation occurs before the channel signals any interrupt and before the channel fetches any subsequent descriptor or data. This operation incurs greater latency, but provides direct synchronization between the DMA interrupt and the state of the data at the peripheral.

If the configuration enables synchronization and enables interrupts, on the last descriptor in a work unit, the interrupt occurs when the channel transfers the final data to the peripheral. This event allows the service routine to switch properly to non-DMA transmit operation. When the event vectors to the interrupt service routine, the DMA channel FIFO is empty, and the DMA channel is no longer running (indicated by the DMA\_STAT.RUN bits).

A synchronized transition also allows greater flexibility in the format of the DMA descriptor chain. When enabled, the next descriptor can have any DMA\_CFG.PSIZE configuration or read/write direction supported by the peripheral and can come from either memory space (internal or external). This feature can be useful in managing MDMA work unit queues, since it is no longer necessary to interrupt the queue between dissimilar work units.

## Work Unit Receive and MDMA Destination Transitions

In DMA receive channels (memory write operations), the DMA\_CFG.SYNC bit controls the handling of the DMA FIFO between descriptor chains (not individual descriptor sets), during the DMA channel pause. The DMA channel pauses after the descriptor sets configured with stop flow mode are complete. Restart the channel (for example,

after an interrupt) by writing the DMA\_CFG register of the channel with a value that enables the DMA channel. If the configuration disables synchronization in the DMA\_CFG value of the new work unit, the configuration selects a continuous transition. In this mode, the DMA FIFO retains any data items received during the channel pause, and they are the first items written to memory in the new work unit. This mode of operation provides lower latency at work unit transitions and ensures no dropping of data items during a DMA pause. The channel provides this operation at the cost of certain restrictions on the DMA descriptors.

- NOTE: If the DMA\_CFG.SYNC bit disables synchronization on the first descriptor of a chain after a DMA pause, do not change the configuration of the DMA\_CFG.PSIZE field of the new chain from the previous descriptor chain (active before the pause). This restriction applies unless the DMA channel is reset between chains by disabling and then re-enabling the DMA channel.

If the DMA\_CFG.SYNC bit configuration enables synchronization, the channel uses a synchronized transition. In this mode, only the data that the DMA channel receives from the peripheral after the write to the DMA\_CFG register gets to memory. The channel discards any prior data items transferred from the peripheral to the DMA FIFO before this register write occurs. This operation provides direct synchronization between the data stream received from the peripheral and the timing of the channel restart, which occurs on the write to the DMA\_CFG register.

For receive DMA operations, the synchronization has no effect in transitions between work units in the same descriptor chain. When the flow mode of previous descriptor was not stopped, the DMA channel did not pause.

If a descriptor chain begins with synchronization enabled, there is no restriction on the DMA\_CFG.PSIZE of the new chain in comparison with the previous chain.

- NOTE: The peripheral transfer size ( DMA\_CFG.PSIZE ) must not change between one descriptor and the next in any DMA receive (memory write) channel within a single descriptor chain, regardless of the DMA\_CFG.SYNC bit setting. In other words, all memory write descriptor sets in a descriptor chain must have the same DMA\_CFG.PSIZE value. For any DMA receive channel (memory write operation), there is no restriction on changes of peripheral transfer size (internal versus external) between descriptors or descriptor chains.

## Transfer Termination and Shutdown Flow

This section describes channel transfer termination and shutdown in stop flow mode and in autobuffer flow mode.

## Stop Flow Mode

In stop flow mode, the DMA channel stops automatically after the work unit is complete. If using a list or array of descriptors to control DMA transfers and if every descriptor contains a DMA\_CFG descriptor element, configure the flow of the final DMA\_CFG descriptor element to stop mode, stopping the channel gracefully. After completion, the DMA channel remains in the stop state. Do not confuse this state with the disabled state, which either occurs due to a DMA error or occurs through disabling the DMA channel by configuring the DMA\_CFG.EN bit.

The intention of disabling the DMA channel through a write to the DMA\_CFG.EN bit is to shut down the DMA channel and to enter the disabled state. All memory and peripheral data transfers cease, and only peripheral interrupts pass through the DMA channels interrupt signals. However, the DMA channel maintains the

DMA\_STAT.RUN bits. For a write to memory, the outstanding memory-transaction counter tracks returning memory write acknowledgments and updates as required.

For memory reads, the outstanding memory-transaction count also tracks returning memory reads. The channel does not write the memory reads into the FIFO. The channel updates the counter to reflect the completion of the transaction, but the channel ignores the data. The DMA\_STAT.RUN bits remain in the waiting for write ACK or FIFO drain to peripheral state and do not change to stop or idle state until the return of all outstanding transactions.

When the DMA\_CFG.EN bit again enables the DMA channel, the channel performs a full reset and clears all counters. If an outstanding memory transaction returns an acknowledgment or read data after this event, a memory transaction error occurred, which generates an error event. Programs must ensure that all outstanding memory transactions complete before reconfiguring the DMA channel. For example, programs can poll the DMA\_STAT.RUN bits to return to the stop or idle state before proceeding.

## Autobuffer Flow Mode

In this mode, the flow does not use any descriptors in stored memory. Instead, the channel performs DMA in a continuous circular buffer fashion, based on user-programmed DMA register settings. On completion of the work unit, the channel reloads the parameter registers into the current registers, and the DMA controller resumes immediately with zero overhead. Consider this mode as a succession of automatically restarted work units.

For autobuffer-flow modes, the only way to cease operations is to disable the DMA channel through the DMA\_CFG.EN bit. One method of changing to a new work unit is:

- Disable the DMA channel
- Set up all the registers (and descriptors in memory, if used) except for DMA\_CFG
- Poll DMA\_STAT.RUN to wait for the status to reflect stop or idle state, and
- Write DMA\_CFG to the new configuration to begin the next work unit

In autobuffer-flow mode or for a list or array of descriptor sets without DMA\_CFG descriptors, use an MMR write to the DMA\_CFG register to terminate the DMA transfer process. Configure the value of the DMA\_CFG.EN bit in this register to disable the DMA channel.

- CAUTION: When the configuration disables a DMA channel, the DMA controller disables interrupt logic that is based on work unit transitions. Be aware of the system environment and current actions, so that additional interrupts are not required from the DMA channel.
- CAUTION: If disabled through DMA\_CFG.EN in the middle of a transaction, the DMA channel completes any transactions that have begun and avoids generating bus errors. However, the channel considers the action of re-enabling the DMA as a hard reset for all internal DMA channel components. Therefore, pay attention to that particular action to avoid unexpected results.

## DMA Channel Errors

When an error occurs, the DMA channel maintains all the state and register values that allow programs to diagnose error causes more thoroughly. The greatest benefit to the programmer is to know exactly what operational state the DMA channel was in at the exact moment the error occurred.

Take care to address the root cause of the error, whether or not the problem originated in the DMA channel. If not properly resolved, the error can result in an additional error shortly after operations resume. The problem can cause other errors elsewhere in the DMA channel or associated modules and circuitry. So, take care also to address those potential problems. Ensure that all outstanding memory reads and writes are complete or cleared before resuming DMA channel operation.

After addressing all issues and neutralizing all side effects of any errors, clear the DMA\_STAT.ERRC status field and restart the DMA channel by disabling then re-enabling the DMA channel through the DMA\_CFG.EN bit.

The following sections describe the error types.

## Status and Debug Errors

DMA channel error conditions can cause the DMA process to end abnormally. The DMA channel provides error detection as a tool for system development and debug, helping to identify DMA-related programming errors. When the DMA channel detects an error, the channel immediately stops and discards any returned memory-read transactions. The DMA\_STAT.RUN field of the DMA channel indicates the idle state after acknowledging all outstanding memory transactions. In addition, the channel asserts an error interrupt request and updates the DMA\_STAT.IRQERR field. Also, the channel updates the DMA\_STAT.ERRC field, indicating the error cause of the first detected error. Unless the error occurs at the exact moment that modification of register values occurs, the registers contain the error values.

All the DMA error interrupt requests are combined into a single shared interrupt request output. Combined error signals require reading the DMA\_STAT register of each DMA channel associated with a combined error interrupt request to determine the DMA channel responsible for the generation of the interrupt.

The DMA channel error interrupt handler performs the following actions:

- Read the DMA\_STAT register of each DMA channel, seeking a channel with the DMA\_STAT.IRQERR set to indicate an error.
- Read the DMA\_STAT.ERRC field of each DMA channel, determining the cause of the error.
- Clear the problem with the DMA channel. For example, fix the register values.
- Clear the error in the DMA channel through a write-1-to-clear operation to the DMA\_STAT.IRQERR bit.

If the channel flags any uncleared error other than a bandwidth monitor error, the channel reports no other error. If the channel reports an uncleared bandwidth monitor error, the channel reports any newly detected error through updating the DMA\_STAT.ERRC field.

## DMA Configuration Register Errors

The channel only flags these configuration errors when the DMA\_CFG.EN bit enables the DMA channel. Error flagging occurs when the configuration:

- Uses a reserved setting
- Enables DMA\_CFG.TWAIT in descriptor on-demand flow mode
- Uses an illegal DMA\_CFG.NDSIZE
- Uses an illegal DMA\_CFG.MSIZE
- Configures DMA\_XCNT = 0 or, when DMA\_YCNT = 0 in 2D DMA mode
- Uses non-zero value in DMA\_CFG.NDSIZE when DMA is configured in stop mode or auto mode
- Enables interrupt or outgoing triggers on DMA\_YCNT when DMA is configured in 1D mode
- Use a DMA\_CFG.MSIZE that exceeds the FIFO size of the DMA channel
- Uses an illegal DMA\_CFG.PSIZE
- Uses a DMA\_CFG.PSIZE that exceeds the FIFO size
- Uses a DMA\_CFG.PSIZE that exceeds the bus width
- Attempts to change from a transmit operation (memory read) to a receive operation without properly synching in the previous work unit or when it is the first work unit in a new chain
- Attempts to change DMA\_CFG.PSIZE of a transmit operation (memory read) without properly synching in previous work unit or when it is the first work unit in a new chain
- Attempts to change from receive operation (memory write) to a transmit operation during a descriptor chain. The channel only can change from receive to transmit if the new transmit is synchronized and is the first work unit.
- Attempts to change DMA\_CFG.PSIZE of a receive operation (memory write) when the operation was not the first work unit (with DMA\_CFG.SYNC enabled)

## Illegal Register Write During Run

The channel generates an error when a write occurs to writable registers of an enabled, running DMA channel. The channel blocks the write.

## Address Alignment Error

The channel generates an address alignment error when any of the following apply:

- Alignment of a descriptor address is not on a 32-bit boundary.
- The current DMA\_CFG.MSIZE configuration contains an unaligned transfer address. The DMA\_ADDRSTART register is not aligned according to the DMA\_CFG.MSIZE field.

## Memory Access Error

The channel generates a memory access error when the DMA process:

- attempts to access an unpopulated address,

- attempts to access a location that provokes a security violation

The error returned from the memory triggers the memory access error.

## Trigger Overrun Error

A trigger overrun error is generated when a new trigger input occurred while an outstanding trigger is waiting. This error is only generated if DMA\_CFG.TOVEN is enabled.

## Bandwidth-Monitor Error

The channel generates this error when the bandwidth-monitor count expires. This error is not fatal, and the DMA channel continues operation.

## Control Interface Error

The channel reports control-interface errors as bus errors to the bus controller. This error can result from:

- An address error
- A register write error (write to a read-only register)

## DMA Operating Modes

The DMA channel supports a number of different flow modes that control how the DMA channel progresses from one work unit to the next.

The flow mode of a DMA channel is not a global setting. A DMA descriptor set can include the descriptor responsible for configuring the flow of the work unit. There is no restriction, limiting the flow configuration to be the same for the entire descriptor chain. If the descriptor chain is not endless, the last descriptor set configures the flow to stop mode, which results in termination of the descriptor chain after the work unit completes. Another example for mixing flow modes is to create an endless descriptor-array. The configuration of the last descriptor set in the array selects the descriptor-list mode. The next descriptor pointer in this set of descriptors points to the first descriptor in the array.

## Register-Based Flow Modes

Register-based DMA operations require configuration by directly writing to the memory-mapped registers of the DMA channel.

Register-based DMA is the traditional method of DMA operation. Software writes all of the configuration of the DMA channel into the memory-mapped registers. This configuration includes information such as the source or destination address and length of the data in the transfer. The DMA controller then starts channel operation. The DMA channel supports the following register-based flow modes.

- Stop Mode
- Autobuffer Mode

The DMA channel supports variable descriptor set sizes within the configuration. The size of a descriptor set can contain as little as a single descriptor. The supported descriptor set sizes can differ between the various descriptorbased flow modes. In addition to the descriptor set size being configurable, descriptor-based DMA also allows altering the flow mode of the next descriptor set. This feature allows for the transition from descriptor-array mode to descriptor-list mode and permits configuring the flow to stop or autobuffer mode.

## Stop Mode

In stop mode, the DMA operation executes only once. If started, the DMA channel transfers the desired number of data words and stops itself again when finished. If the DMA channel is no longer used, software configures the enable bit to disable a paused channel. The channel also can generate interrupts and triggers for each row or work unit completion, depending on the desired operation.

## Autobuffer Mode

In autobuffer mode, the DMA operates repeatedly in a circular manner. If the transfer of all data words completes, the channel reloads the address pointer ( DMA\_ADDR\_CUR ) automatically with the DMA\_ADDRSTART value. The channel also can generate an interrupt.

The DMA\_CFG.FLOW field enables autobuffer mode. The configuration must load the DMA\_CFG.NDSIZE field value, such that the next descriptor size is zero.

## Descriptor-Based Flow Modes

Descriptor-based DMA operations fetch descriptor sets from memory allowing for autonomous loading of work units on other work units. Software does not need to set up the DMA sequences directly by writing into the DMA controller registers. Rather, software keeps DMA descriptor sets in memory.

Descriptor-based DMA operations have the following additional attributes.

- The DMA controller autonomously loads the descriptor set from memory to the affected DMA controller registers on demand.
- The channel can fetch descriptor sets from any memory space that supports DMA read operations.
- The descriptor set describes the next operation that the DMA controller performs.
- The descriptor set can include information such as the DMA configuration word as well as data source or destination address, transfer count, and address modify values.

A descriptor set describes a single work unit. The next work unit can reuse some values from the previous one descriptor set. But, this reusage is possible only if they are not overwritten in the subsequent descriptor set fetches and only if the work unit requires the use of this descriptor.

The DMA channel supports the following flow modes with descriptor-based operations.

- Descriptor-Array Mode
- Descriptor-List Mode
- Descriptor-On-Demand Modes

The DMA channel supports variable descriptor set sizes within the configuration. The size of a descriptor set can contain as little as a single descriptor and the supported descriptor set sizes can differ between the various descriptorbased flow modes. In addition to configurable descriptor set size, descriptor-based DMA also allows for altering of the flow mode of the next descriptor set. Programs can transition from one descriptor-based mode to another descriptor-based mode and can also transition to any of the register-based flow modes.

## Descriptor-Array Mode

When configured in this mode, the descriptor sets do not contain further descriptor pointers. Software writes the initial descriptor-pointer value, which points to an array of descriptors. This operation assumes that the individual descriptors reside next to each other and assumes that their addresses are known.

The Offsets for Descriptor-Array Mode Parameters and Descriptors table illustrates how to structure a descriptor set in memory. The descriptor sets must reside in a contiguous block or memory in the format shown in the table. Locate the first descriptor of the next descriptor set in the memory location immediately following the last descriptor of the current descriptor set. The values have the same order as the corresponding offset addresses of the memory-mapped register.

Table 36-10: Offsets for Descriptor -Array Mode Parameters and Descriptors

| Descriptor Offset   | Parameter Register   |
|---------------------|----------------------|
| 0x00                | DMA_ADDRSTART        |
| 0x04                | DMA_CFG              |
| 0x08                | DMA_XCNT             |
| 0x0C                | DMA_XMOD             |
| 0x10                | DMA_YCNT             |
| 0x14                | DMA_YMOD             |

All other DMA channel registers that were not loaded as a result of the descriptor set fetch, retain their previous values. The channel reloads all of the current registers between the descriptor set fetch and the start of the DMA operation for the work unit.

NOTE: At a minimum, write the DMA\_DSCPTR\_CUR register prior to writing to the DMA\_CFG register, which is the special action required to start the DMA channel.

## Descriptor-List Mode

In this flow mode, multiple descriptors form a chained list in which each descriptor set contains a pointer to the next descriptor set, allowing greater flexibility in memory layout options. When the channel fetches the descriptor set, the operation loads this pointer value into the next descriptor pointer register of the DMA channel.

## Descriptor Sets

The Offsets for Descriptor-List Mode Parameters and Descriptors table shows how to structure a descriptor set in memory. Disperse the placement of the descriptor sets throughout memory, having sets reside in different memory

blocks. But, each descriptor of the descriptor set must reside in a contiguous section of memory in the format shown in the table. The values have the same order as the corresponding offset addresses of the memory-mapped registers.

Table 36-11: Offsets for Descriptor-List Mode Parameters and Descriptors

| Descriptor Offset   | Parameter Register   |
|---------------------|----------------------|
| 0x00                | DMA_DSCPTR_NXT       |
| 0x04                | DMA_ADDRSTART        |
| 0x08                | DMA_CFG              |
| 0x0C                | DMA_XCNT             |
| 0x10                | DMA_XMOD             |
| 0x14                | DMA_YCNT             |
| 0x18                | DMA_YMOD             |

All other DMA channel registers that were not loaded as a result of the descriptor set fetch, retain their previous values. The channel reloads all of the current values of the registers between the descriptor set fetch and the start of the DMA operation for the work unit.

## Minimum Startup Requirements

At a minimum, write the DMA\_DSCPTR\_NXT register prior to write to the DMA\_CFG register, which is the special action required to start the DMA channel.

## Descriptor-On-Demand Modes

The Descriptor-Array Mode and Descriptor-List Mode each have an on-demand mode of operation.

In on-demand mode, at the end of the work unit, if the DMA channel has not detected an incoming trigger event, the channel repeats the current work unit. If the DMA channel receives an incoming trigger before completion of the work unit, the channel fetches a new descriptor set.

The Offsets for Descriptor-Array Mode Parameters and Descriptors and Offsets for Descriptor-List Mode Parameters and Descriptors tables illustrate how to structure each descriptor set in memory.

Table 36-12: Offsets for Descriptor-Array Mode Parameters and Descriptors

| Descriptor Offset   | Parameter Register   |
|---------------------|----------------------|
| 0x00                | DMA_ADDRSTART        |
| 0x04                | DMA_CFG              |
| 0x08                | DMA_XCNT             |
| 0x0C                | DMA_XMOD             |

Table 36-12: Offsets for Descriptor-Array Mode Parameters and Descriptors (Continued)

| Descriptor Offset   | Parameter Register   |
|---------------------|----------------------|
| 0x10                | DMA_YCNT             |
| 0x14                | DMA_YMOD             |

NOTE: For descriptor-array mode, at a minimum, write the DMA\_DSCPTR\_CUR register prior to writing to the DMA\_CFG register, which is the special action required to start the DMA channel.

Table 36-13: Offsets for Descriptor-List Mode Parameters and Descriptors

| Descriptor Offset   | Parameter Register   |
|---------------------|----------------------|
| 0x00                | DMA_DSCPTR_NXT       |
| 0x04                | DMA_ADDRSTART        |
| 0x08                | DMA_CFG              |
| 0x0C                | DMA_XCNT             |
| 0x10                | DMA_XMOD             |
| 0x14                | DMA_YCNT             |
| 0x18                | DMA_YMOD             |

NOTE: For descriptor-list mode, at a minimum, write the DMA\_DSCPTR\_NXT register prior to write to the DMA\_CFG register, which is the special action required to start the DMA channel.

## Data Transfer Modes

In addition to supporting basic one-dimensional DMA transfers, the DMA channel also supports two-dimensional functionality.

## Two-Dimensional DMA

Register-based flow modes and descriptor-based flow modes support two-dimensional data transfers.

In two-dimensional (2D) mode, the X-direction count ( DMA\_XCNT ), the X-direction modifier ( DMA\_XMOD ), the Y-direction count ( DMA\_YCNT ), and the Y-direction modifier ( DMA\_YMOD ) support arbitrary row and column sizes. Also, the modify values can be negative, allowing implementation of interleaved data streams. The DMA\_XCNT value specifies the row size, and the DMA\_YCNT value specifies the column size; where the DMA\_XCNT value must be 2 or greater.

The DMA start address ( DMA\_ADDRSTART ), the X-direction modifier ( DMA\_XMOD ), and the Y-direction modifier ( DMA\_YMOD ) specifications all are in bytes. The alignment must be a multiple of the DMA transfer word size; configured using the DMA\_CFG.MSIZE bit. Misalignment results in a DMA channel error.

The DMA\_XMOD register value is the byte-address increment that the channel applies after each transfer, decrementing the DMA\_XCNT register. The channel does not apply the DMA\_XCNT when the inner loop count ends with the DMA\_XCNT\_CUR register decrementing to 0 from 1. Except, the channel does apply the DMA\_XCNT on the final transfer, when the DMA\_YCNT register is 1 and the DMA\_XCNT register decrements from 1 to 0.

The DMA\_YMOD register value is the byte-address increment that the channel applies after each decrement of the value in DMA\_YCNT\_CUR . However, the channel does not apply the DMA\_YMOD value to the last item in the array on which the outer loop count ( DMA\_YCNT\_CUR ) also expires by decrementing from 1 to 0.

After the last transfer completes, DMA\_YCNT\_CUR is 1 and the DMA\_XCNT\_CUR register is 0. The DMA channels current address points to the last items address plus the DMA\_XMOD register value. If the DMA channel programming selects automatic refresh (such as in autobuffer mode), the channel reloads the DMA\_XCNT\_CUR , DMA\_YCNT\_CUR , and DMA\_ADDR\_CUR for the first data transfer of the next work unit.

Interrupt notification is configurable for end-of-row or end-of-work unit completion.

For example, two-dimensional DMA can be used to extract interleaved data (such as RGB values for a video frame ) by modifying both DMA\_XMOD and DMA\_YMOD values. The Capturing a Video Data Stream 2D DMA Example depicts the process of receiving a stream of the R, G, B values from an N*M frame. The inner loop of the 2D DMA configuration has three values ( DMA\_XCNT = 3) and a stride ( DMA\_XMOD ) of N*M, chosen such that successive elements in each row are 1-2-3, 4-5-6 and so forth. The outer loop of the 2D DMA configuration has N*M values ( DMA\_YCNT = N*M) and a negative stride ( DMA\_YMOD ) of 1-2*N*M chosen to instruct the DMA controller to jump from element 3 to 4, 6 to 7 and so forth at the end of each inner loop.

Figure 36-4: Capturing a Video Data Stream 2D DMA Example

<!-- image -->

## DMA Channel Event Control

The DMA channel supports a number of events that provide notification of work unit state, peripheral data request, peripheral interrupt request and completion events, and DMA channel error conditions. In addition to flexible interrupt configuration, the DMA channel also supports incoming and outgoing triggers which are useful in synchronizing the DMA channel with other system resources.

The DMA channel has two interrupt signals for support of a number of events such as work-unit state events, peripheral interrupt request (PIRQ) events, peripheral data request (PDR) events, and DMA channel errors. The channel reports DMA channel errors on a dedicated interrupt signal. All other interrupt sources share an interrupt signal.

In addition to flexible interrupt configuration, the DMA channel also supports incoming and outgoing triggers which are useful in synchronizing the DMA channel with other system resources.

The channel can signal the processor on DMA channel events using status information and optional interrupt requests. Programs can use these events to update the progress of data transfers and to request intervention from the processor core. Configure most DMA channel interrupts using bits in the DMA\_CFG register. Dedicated bits in the DMA\_STAT register report the occurrence of various events. Use write-one-to-clear (W1C) operations to clear interrupt requests from the status register.

NOTE: Hardware does not clear the interrupt status bits automatically, even when programs disable then reenable the DMA channel. In this situation, the channel deasserts the interrupt signal, after the program disables the DMA channel. But, the status bit remains set until software either re-enables the DMA channel or clears the status bit.

The DMA channel supports the following categories of events on the interrupt signals:

- Work-unit state events generate interrupts on row or on work unit DMA completion.
- A peripheral uses peripheral interrupt request (PIRQ) events to signal when it has completed the transfer of all data.
- A peripheral uses peripheral data request (PDR) events to request data from a disabled or idle DMA channel.
- Error events signal a failure in the work unit.

ATTENTION: While in an error state, the DMA channel does not generate an interrupt to the processor for a workunit state event or a PIRQ event, nor does the channel forward a PDR event.

## Event Signals

The Event Signals table provides descriptions of DMA channel events.

Table 36-14: Event Signals

| Signal        |   Width (bits) | Description                                                                                                                                                                                             |
|---------------|----------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| DMA_ERROR     |              1 | Used to signal an error condition in the DMAchannel. The source of the error can be determined by reading the DMA_STAT.ERRC bit.                                                                        |
| DONE_PIRQ_INT |              1 | Signal used to indicate DMAcompletions events, PIRQ events and also for forwarding PDR events based on configuration. Read the corresponding fields in DMA_STAT to de- termine the source of the event. |
| DMA_TRIG_OUT  |              1 | Trigger output that gets routed to the TRU and can be configured to provide notification on row or work unit completion.                                                                                |
| DMA_TRIG_IN   |              1 | Trigger input from the TRU that can be used to control the start of a work unit.                                                                                                                        |

## Work Unit State Events

Completing a row or a work unit generates a work-unit state event. For either of these events to generate an interrupt request, the configuration of the interrupt of the DMA channel must select one of the available work-unit completion modes.

- Current X count reaching 0 for row completion or 1D DMA work unit completion
- Current Y count reaching 0 for work unit completion of 2D DMA

NOTE: For 1D DMA, a DMA channel configuration error results if the configuration generates the interrupt request when the current Y counter reaches 0.

The DMA channel issues the last memory read or write transaction for the row or work unit, then pauses until the return of the read or write acknowledge. After successful acknowledge of the transfer, the DMA channel issues the interrupt request and continues to process the next row or work unit.

Waiting for acknowledgement of the memory access results in a delay. However, programs can read or modify data in the memory without adversely affecting or being affected by the DMA transfer.

- NOTE: While the DMA channel pauses waiting for acknowledgement of the memory transfer, the DMA channel is still capable of fetching the next descriptor set. This fetch gets the channel ready to process the next work unit as soon as the memory access completes.

The channel configuration of the synchronization feature also affects interrupt timing. For memory-read operations with synchronization enabled, the channel delays the interrupt request until the completion of the last transfer from the DMA channel FIFO to the peripheral. The synchronization feature does not affect interrupt timing for memory write operations.

## Peripheral Interrupt Request Events

For peripheral-transmit operations, a peripheral connected to the DMA channel can use peripheral interrupt request (PIRQ) events to indicate that data has left the channel FIFO and to indicate transfer completion.

In order to support PIRQ interrupts, correctly configure the interrupt of the DMA channel. This configuration disables the generation of interrupt requests based on the work unit state and, instead, results in generating an interrupt request when the DMA channel receives the command from the peripheral.

The channel only generates the interrupt request under the following conditions:

- The configuration enables the DMA channel
- The DMA channel is in the stop state
- The configuration of the DMA channel interrupt selects PIRQ operation

## Peripheral Data Request Events

Peripheral data request (PDR) events occur when an interfaced peripheral requests data from the DMA channel and the DMA channel (either disabled or enabled) is in the stop state.

When a peripheral sends a data request command to a disabled DMA channel, the DMA channel generates an interrupt to the System Event Controller (SEC). There is no status information reported about this event in the status register of the DMA channel. Instead, the channel identifies the PDR event from the fact that the DMA channel generated an interrupt while disabled. It is possible to further confirm event status by verifying the status of the peripheral interfaced to the DMA channel.

This operation forwards data requests as interrupts when the DMA channel is in the disabled state. The DMA channel can forward PDR events as an interrupt request when the DMA channel is in the stop state after the completion of a work unit. The forwarding of this interrupt when the DMA channel is in the stop state is optional and configured by the program during DMA channel configuration.

## DMA Channel Triggers

DMA channel triggers are useful for synchronizing the DMA channel with other events in the system. One usage is to combine channel triggers with each other to create ping-pong buffers. Another usage is to combine the triggers with interrupt requests to notify the processor on reaching a particular milestone that requires service. The channel also can use triggers to enforce a handshake DMA operation in which the trigger acts as a signal for a DMA request.

NOTE: Using the trigger to control the pace of data transfers, such as for handshake DMA, requires that all the data for the entire work unit is ready for transfer.

The DMA channel has a single incoming trigger that can control the pace of the data transfers performed by the DMA channel. The configuration can direct the DMA channel to wait for the incoming trigger before starting the work unit transfer or fetching a descriptor set from memory.

The DMA channel also has a single outgoing trigger signal. This configuration can direct this trigger to signal the end of row or an entire work unit. The DMA channel issues the last memory read or memory write transaction for the row or work unit, then pauses until return of the transfer acknowledge. After acknowledgement of the transfer, the DMA channel issues the trigger before processing the next row or work unit.

## Issuing Triggers

The DMA channel configuration can direct the channel to generate an outgoing trigger signal at the end of row or the end of a work unit. The DMA channel issues the last memory read or memory write transaction for the row or work unit, then pauses until the return of the transfer acknowledge. After acknowledgement of the transfer, the DMA channel issues the trigger before processing the next row or work unit.

- NOTE: While the DMA channel pauses waiting for acknowledgement of the memory transfer, the DMA channel is still capable of fetching the next descriptor set. This fetch gets the channel ready to process the next work unit as soon as the memory access completes.

## Waiting For Triggers

Programs can use triggering to control the pace of data transfers performed by the DMA channel. The DMA channel enters a wait state before beginning the next work unit if the configuration enables DMA\_CFG.TWAIT and either of the following apply:

- The channel receives a trigger since the last time the DMA channel left the wait state.
- The channel receives a trigger since its transition from disable to enable.

In the wait state, the DMA channel also does not perform a descriptor fetch. After receiving a trigger, the DMA channel leaves the wait state and begins the next work unit or fetches the next descriptor if configured for a descriptor-based mode of operation.

If a memory-mapped register write operation programs the channel with stop flow mode enabled ( DMA\_CFG.TWAIT bit) and the channel has not already received a trigger, the DMA channel enters a wait state before performing the data transfer. On receiving the trigger, the DMA channel begins the data transfer portion of the work unit. Once the data transfer is complete, the DMA channel enters the stop state.

If a memory-mapped register write operation programs the DMA channel with the flow mode configured to one of the descriptor-based modes, the DMA channel enters the wait state before performing the descriptor fetch. After completing the descriptor fetch, the DMA channel immediately proceeds to the data transfer, regardless of the value of the DMA\_CFG.TWAIT bit. If another (next) descriptor fetch follows the descriptor fetch, the DMA channel enters a wait state before fetching the next descriptor.

If the descriptor fetch returns a descriptor with stop flow mode, the DMA\_CFG.TWAIT value for that descriptor does not affect the DMA as the channel enters the stop state after completing the data transfer. The DMA channel only enters the wait state based on DMA\_CFG.TWAIT before the next work unit or descriptor fetch.

If the descriptor fetch returns a descriptor configured for autobuffer flow mode, the DMA\_CFG.TWAIT for that descriptor does not affect the DMA for the first work unit of the autobuffer transfer. After completing the first work unit and not receiving another trigger, the DMA channel enters the wait state before reinitializing its counters and address registers (if not configured for current addressing). The channel performs the next work unit after receiving the trigger.

The incoming trigger can occur when the DMA channel has not entered the wait state. The trigger can occur while the DMA channel is executing a work unit, is performing descriptor fetch, or is in the stop state. The trigger is held internally. After the work unit is complete, the DMA channel skips the wait state and proceeds directly to executing the following work unit. If the DMA\_CFG.TWAIT bit is not enabled, the DMA channel also skips the wait state. However, the trigger is held internally and is used the next time the configuration enables DMA\_CFG.TWAIT . This trigger retention allows programs to enable the DMA\_CFG.TWAIT functionality several work units apart without concern for losing a trigger. The DMA channels trigger-overrun enable functionality can be enabled in all work units to ensure that multiple triggers do not occur between the work units with the DMA\_CFG.TWAIT bit enabled.

## DMA Channel Programming Model

Several synchronization and control methods are available for use in development of software tasks which manage peripheral DMA and memory DMA. Software must accept requests for new DMA transfers from other software tasks, integrate these transfers into existing transfer queues, and reliably notify other tasks when the transfers are complete.

In the processor, it is possible to manage each peripheral DMA and memory DMA stream with a separate task or to manage them together with any other stream. Each DMA channel has independent, orthogonal control registers, resources, and interrupts. So, the selection of the control scheme for one channel does not affect the choice of control scheme on other channels. For example, one peripheral can use a linked-descriptor-list, interrupt-driven scheme while another peripheral can simultaneously use a demand-driven, buffer-at-a-time scheme synchronized by polling DMA events.

The topics that follow describe the steps required to configure the DMA channel for the various modes in addition to the programming concepts required for software synchronization.

## Mode Configuration

Use the step-by-step directions that follow to set up the DMA channel for operating modes.

## Register-Based Linear-Buffer Stop Flow Mode

This procedure configures the DMA channel of a peripheral to read data from internal memory and to send it to the peripheral for transmission. Upon DMA completion, the DMA channel enters the idle state until either disabled or reconfigured for a new transfer.

Assume that the peripheral is in a state where it is ready to transmit data received from the DMA channel.

The task involves writing to a number of DMA channel MMR registers to configure a DMA channel to:

- Read data from internal memory, and
- Send it to a peripheral connected to the peripheral DMA bus.
1. Write the DMA\_ADDRSTART register. ADDITIONAL INFORMATION: Software can use the address to calculate the most optimum possible DMA\_CFG.MSIZE .
2. Calculate the optimum DMA\_CFG.MSIZE based on the DMA\_ADDRSTART register and number of bytes in work unit.
- ADDITIONAL INFORMATION: The number of bytes in the work unit must be a multiple of the selected DMA\_CFG.MSIZE , and the calculation also must consider the start address alignment.
3. Write the DMA\_XCNT register based on the calculated DMA\_CFG.MSIZE . ADDITIONAL INFORMATION: The DMA\_XCNT value is the number of DMA\_CFG.MSIZE transfers to make up the entire work unit.
4. Write the DMA\_XMOD register.
- ADDITIONAL INFORMATION: For a linear buffer transfer, determine the value in DMA\_XMOD from the selected DMA\_CFG.MSIZE . Always specify this register as a number of bytes.
5. Write the DMA\_CFG register with DMA\_CFG.EN configured to enable the DMA channel.

ADDITIONAL INFORMATION: Set the DMA\_CFG.FLOW bit for STOP mode. Configure the DMA\_CFG.WNR bit for memory read operation. Configure the DMA\_CFG.PSIZE bits to a value no larger than the supported bus width of the peripheral DMA bus. From performance perspective, it is recommended to use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the DMA\_CFG.PSIZE value used.

- The DMA\_CFG.SYNC bit can be configured to control DMA completion notification timing.
- Interrupts and triggers also can be configured at this step depending on requirements.

Now, the DMA channel is enabled, and the buffer is transferred. The DMA channel enters the IDLE state upon completion of the work unit.

## Register-Based Autobuffer Flow Mode

This procedure configures the DMA channel of a peripheral to read data from internal memory and send it to the peripheral for transmission. The transmission of the buffer repeats endlessly. Upon DMA completion, the DMA channel restarts the DMA operation, creating an endless circular buffer transfer.

Assume the peripheral is in a state where it is ready to transmit data received from the DMA channel.

The task involves writing to a number of DMA channel MMR registers to configure a DMA channel to:

- Read data from internal memory, and
- Send it to a peripheral connected to the peripheral DMA bus.
1. Write the DMA\_ADDRSTART register.

ADDITIONAL INFORMATION: Use the address to calculate the optimum possible DMA\_CFG.MSIZE .

2. Calculate the optimum DMA\_CFG.MSIZE based on the DMA\_ADDRSTART register and number of bytes in work unit.

ADDITIONAL INFORMATION: The number of bytes in the work unit must be a multiple of the selected DMA\_CFG.MSIZE , and the calculation must consider the start address alignment.

3. Write the DMA\_XCNT register based on calculated DMA\_CFG.MSIZE . ADDITIONAL INFORMATION: The DMA\_XCNT register value is the number of DMA\_CFG.MSIZE transfers to make up the entire work unit.
4. Write the DMA\_XMOD register.
3. ADDITIONAL INFORMATION: For a linear buffer transfer, determine the value in DMA\_XMOD from the selected DMA\_CFG.MSIZE . Always specify this register as a number of bytes.
5. Write the DMA\_CFG register with the DMA\_CFG.EN bit configured to enable the DMA channel.

ADDITIONAL INFORMATION: Set the DMA\_CFG.FLOW bit for autobuffer mode. Configure the DMA\_CFG.WNR bit for memory read operation. Configure the DMA\_CFG.PSIZE bit to a value no larger than the supported bus width of the peripheral DMA bus. From performance perspective, it is recommended to use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the DMA\_CFG.PSIZE value used.

- The DMA\_CFG.SYNC bit can be configured to control DMA completion notification timing.
- Interrupts and triggers also can be configured at this step depending on requirements.

Now, the DMA channel is enabled, and the buffer transfers until the DMA channel is disabled.

## Descriptor-Array Flow Mode

This procedure configures the DMA channel of a peripheral to:

- Read data from memory as described by the descriptor sets in the array, and
- Send the data to the peripheral for transmission.

Descriptor sets are read from an array in memory to configure the individual work units.

Assume the peripheral is in a state where it is ready to transmit data received from the DMA channel. Assume that the array of descriptors is to be initialized with the last descriptor set configured for STOP flow mode.

The task involves writing to a number of DMA channel MMR registers to:

- Configure a DMA channel to read the array in memory, containing the first descriptor set that configured the DMA channel to retrieve, and
- Send the data to a peripheral connected to the peripheral DMA bus.

On DMA completion, the DMA channel enters the idle state until either disabled or reconfigured for a new transfer.

1. Write the DMA\_DSCPTR\_CUR register with the address of the array in which the descriptor sets are stored. ADDITIONAL INFORMATION: The array address must meet any processor alignments restrictions imposed by descriptor fetches.
2. Write the DMA\_CFG register with the DMA\_CFG.EN bit configured to enable the DMA channel. ADDITIONAL INFORMATION: Configure the DMA\_CFG.PSIZE bits to a value no larger than the supported bus width of the peripheral DMA bus. From performance perspective, it is recommended to use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the DMA\_CFG.PSIZE value used. Set the DMA\_CFG.FLOW bit for descriptor-array mode. Configure the

DMA\_CFG.NDSIZE bits to describe the number of descriptor elements contained within the first descriptor set. Configure the DMA\_CFG.WNR bit for memory read operation.

- The descriptor set that is fetched controls the DMA\_CFG.SYNC configuration and the interrupt or trigger configurations.

The first descriptor set is fetched from memory location provided by the DMA\_DSCPTR\_CUR register and loaded to the MMR registers of the DMA channel.

Now, the DMA channel is processing all the work units provided in the descriptor array. The DMA channel enters the IDLE state on completion of the final work unit that was configured for STOP flow mode.

## Descriptor-List Flow Mode

This procedure configures the DMA channel of a peripheral to:

- Read data from memory as described by the descriptor sets in the list, and
- Send it to the peripheral for transmission.

The DMA controller reads the descriptor sets from a list of descriptors. With the list, each descriptor set has a descriptor that points to the next descriptor set location in memory.

Assume the peripheral must be in a state where it is ready to transmit data received from the DMA channel. Assume that the list of descriptors must be initialized with the last descriptor set in the list configured for Stop flow mode.

The task involves writing to a number of DMA channel MMR registers to:

- Configure a DMA channel to read the list in memory, containing the first descriptor set that configured the DMA channel to retrieve, and
- Send the data to a peripheral connected to the peripheral DMA bus.

On DMA completion, the DMA channel enters the idle state until either disabled or reconfigured for a new transfer.

1. Write the DMA\_DSCPTR\_NXT register with the address of the first descriptor in the list to be processed. ADDITIONAL INFORMATION: The array address must meet any processor alignments restrictions imposed by descriptor fetches.
2. Write the DMA\_CFG register with the DMA\_CFG.EN configured to enable the DMA channel.
3. ADDITIONAL INFORMATION: Set the DMA\_CFG.FLOW for descriptor-list mode. Configure the DMA\_CFG.NDSIZE bit to describe the number of descriptor elements contained within the first descriptor set. Configure the DMA\_CFG.WNR bit for memory read operation. Configure the DMA\_CFG.PSIZE bit to a value no larger than the supported bus width of the peripheral DMA bus. From performance perspective, it is

recommended to use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the DMA\_CFG.PSIZE value used.

- The descriptor set that is fetched controls the DMA\_CFG.SYNC configuration and controls the interrupt or trigger configurations.

The first descriptor set is fetched from the memory location provided by DMA\_DSCPTR\_NXT and is loaded to the MMR registers of the DMA channel.

Now, the DMA channel is processing all the work units provided in the descriptor list. The DMA channel enters the idle state when the final work unit that was configured for stop-flow mode is complete.

## Register-Based Memory-to-Memory Transfer in Stop Flow Mode

This procedure configures a memory DMA channel pair in stop flow mode. One DMA channel is configured for memory read operations, while the other DMA channel is configured for memory write.

The task involves writing to a number of DMA channels on two DMA channels that create a memory DMA pair. On DMA completion, the DMA channel enters the idle state, until either the DMA channel is disabled or is reconfigured for a new transfer.

1. Write the DMA\_ADDRSTART register of the source DMA channel. ADDITIONAL INFORMATION: The address can be used to calculate the optimum DMA\_CFG.MSIZE possible.
2. Calculate the optimum DMA\_CFG.MSIZE based on the DMA\_ADDRSTART register and number of bytes in work unit. ADDITIONAL INFORMATION: The number of bytes in the work unit must be a multiple of the selected DMA\_CFG.MSIZE and the start address alignment must also be considered.
3. Write the DMA\_XCNT register of the source DMA channel based on calculated DMA\_CFG.MSIZE . ADDITIONAL INFORMATION: DMA\_XCNT is the number of DMA\_CFG.MSIZE transfers to make up the entire work unit.
4. Write the DMA\_XMOD register of the source DMA channel. ADDITIONAL INFORMATION: For a linear buffer transfer, determine the value in DMA\_XMOD from the selected DMA\_CFG.MSIZE . This register is always specified in the number of bytes.
5. Write the DMA\_ADDRSTART register of the destination DMA channel. ADDITIONAL INFORMATION: The address can be used to calculate the most optimum DMA\_CFG.MSIZE possible.
6. Calculate the optimum DMA\_CFG.MSIZE based on the DMA\_ADDRSTART register and number of bytes in work unit.

ADDITIONAL INFORMATION: The number of bytes in the work unit must be a multiple of the selected DMA\_CFG.MSIZE and the start address alignment must also be considered.

7. Write the DMA\_XCNT register of the destination DMA channel based on the calculated DMA\_CFG.MSIZE . ADDITIONAL INFORMATION: DMA\_XCNT is the number of DMA\_CFG.MSIZE transfers to make up the entire work unit.
8. Write the DMA\_XMOD register of the destination DMA channel. ADDITIONAL INFORMATION: For a linear buffer transfer, determine the value in DMA\_XMOD from the selected DMA\_CFG.MSIZE . This register is always specified in the number of bytes.
9. Write the DMA\_CFG register of the source DMA channel with DMA\_CFG.EN configured to enable the DMA channel.

ADDITIONAL INFORMATION: The DMA\_CFG.FLOW bit field must be configured for stop mode. The DMA\_CFG.WNR bit must be cleared for memory read operation. The DMA\_CFG.PSIZE bits must be configured to a value no larger than the supported bus width of the peripheral DMA bus. From performance perspective, it is recommended to use the largest possible DMA\_CFG.PSIZE value (for example, equal to the supported peripheral bus width ( DMA\_STAT.PBWID )). However, ensure that the number of bytes in the work unit is a multiple of the DMA\_CFG.PSIZE value used.

- The DMA\_CFG.SYNC bit can be configured to control DMA completion notification timing.
- Interrupts and triggers also can be configured at this step, depending on requirements. The interrupts and triggers are enabled within the destination DMA channel configuration.

The memory read DMA transfer begins.

10. Write the DMA\_CFG register of the destination DMA channel with DMA\_CFG.EN configured to enable the DMA channel.

ADDITIONAL INFORMATION: The DMA\_CFG.FLOW bit field must be configured for stop mode. The DMA\_CFG.WNR bit must be set for memory write operation. The DMA\_CFG.PSIZE bits must be configured to a value no larger than the supported bus width of the peripheral DMA bus. This value must also match the value written for the source DMA channel configuration.

- Interrupts and triggers also can be configured at this step depending on requirements.

The memory write DMA transfer begins.

Both memory DMA channels are now running and the data is transferred from the source address to the destination address. The DMA channel enters the IDLE state upon completion of the work unit.

## Programming Concepts

Using the features, operating modes, and event control for the DMA channel to their greatest potential requires an understanding of some DMA channel-related concepts.

## Synchronization of Software and DMA

A critical element of software DMA management is the synchronization of DMA work unit completion with software. This synchronization can be achieved using DMA channel interrupt request and trigger events and using a poll of the status bits of these events within the DMA channel registers, or combining these techniques. Processor polling of DMA address/count/status for completion is not a recommended programming practice. The requirements and limitations of processor polling place significant responsibility onto the code developer to be deeply aware of the underlying hardware. The interrupt requests and triggers are designed for efficient code development and reuse.

## Interrupt and Trigger Event-Based Synchronization

Interrupt and trigger based synchronization methods must avoid overrun. An overrun occurs when some events fail to invoke the event handler of a DMA channel for every event due to excessive latency in processing of events. The system design must ensure to either:

- Schedule only one event per channel (for example, at the end of a descriptor list), or
- Space the generated events sufficiently far apart in time that system processing budgets can guarantee service of every event.

The DMA channel issues status information through an interrupt request or trigger event or changes event status bits in the DMA\_STAT register. This status guarantees that the last memory operation of the work unit is complete. For memory read DMA transactions, this status means that the FIFO of the DMA channel safely receives the final memory read data. For DMA transactions writing to memory, this status indicates that the DMA channel received an acknowledge of completion of the last write transfer of the work unit.

## Register Polling Based Synchronization

Do not poll the DMA channel registers ( DMA\_ADDR\_CUR , DMA\_DSCPTR\_CUR , DMA\_XCNT\_CUR , or DMA\_YCNT\_CUR ) as a method of precisely synchronizing DMA with data processing. This approach is inaccurate due to the operation of the DMA channel FIFOs and DMA or memory pipelining. The current address, pointer, and count registers change several cycles in advance of the completion of the corresponding memory operation. This timing is measurable from the time at which the results of the operation are first visible to the core by memory read or write instructions.

For example, in a DMA channel memory write operation to external memory, assume DMA channel A initiates a DMA channel write operation. For memories with access latency, this operation requires many system-clock cycles. Meanwhile, DMA channel B (which does not in itself incur latency) initiates a transfer, which stalls behind the slow operation of channel A . Software monitoring channel B could not safely conclude whether the memory location pointed to by the DMA\_ADDR\_CUR of channel B . Also, the software cannot conclude whether the register has been written based solely on the contents of this register.

Polling of the current address, pointer, and count registers can permit loose synchronization of DMA with software. But, the software must allow for the lengths of the DMA or memory pipeline. Also, software must consider the length of the DMA FIFO for a particular peripheral. If the FIFOs are filled with incomplete work, the DMA channel does not advance current address, pointer, or count registers. The incomplete work includes reads that have been started but have not yet finished.

Additionally, software must consider the length of the pipelines to the destination memory. If the DMA FIFO length and channel memory-pipeline length are added, software can estimate the maximum number of incomplete memory operations in progress.

NOTE: The estimate would be a maximum, as the DMA or memory pipeline can include traffic from other DMA channels.

## Descriptor Queues

A system designer may want to write a DMA manager facility which accepts DMA requests from other software. The DMA manager software does not know in advance when new work requests are received or what these requests contain. The software could manage these transfers using a circular linked list of DMA descriptors. In such a list, each descriptor sets the DMA\_DSCPTR\_NXT descriptor, which points to the next descriptor set. And the last descriptor set in the list points to the first descriptor set.

The code that writes into this descriptor list could use the circular addressing modes of the processor. This approach does not need to use comparison and conditional instructions to manage the circular structure. In this case, the DMA\_DSCPTR\_NXT descriptor of each descriptor set can be written once at startup, and skipped over as new contents are written for each descriptor.

The recommended method for synchronization of a descriptor queue is to use an interrupt or trigger. The descriptor queue is structured, such that (at least) the final valid descriptor set is always programmed to generate an interrupt or trigger event upon completion. More detail is provided in the following sections.

- Queues Using Event Generation for Every Descriptor Set
- Queues Using Minimal Events

## Queues Using Event Generation for Every Descriptor Set

In this system, the DMA manager software synchronizes with the DMA channel by enabling an interrupt request or trigger on every descriptor set. Only use this method if the system design can guarantee that each work unit completion event is serviced separately (no interrupt or trigger overrun).

To maintain synchronization of the descriptor set queue, the non-interrupt software maintains a count of descriptor sets added to the queue. The event handler (either interrupt or trigger) maintains a count of completed descriptor sets removed from the queue. The counts are equal only when the DMA channel is paused after having processed all the descriptor sets.

When each new work unit event is received, the DMA manager software initializes a new descriptor set, taking care to set the flow to stop mode. Next, the software compares the descriptor set counts to determine whether the DMA channel is running. If the DMA channel is paused (counts equal), the software increments its count. Then, the software starts the DMA channel by writing the DMA\_CFG of the new descriptor set.

If the counts are unequal, the software instead modifies the DMA\_CFG of the next-to-last descriptor set, such that it now describes the newly queued descriptor set. This operation does not disrupt the DMA channel provided the rest of the descriptors of the set are initialized in advance. It is necessary to synchronize the software to the DMA to determine whether the DMA channel read the new or the old DMA\_CFG value.

The event handler performs the synchronization operation. When an event is detected, the handler reads the DMA\_STAT register of the DMA channel. If the DMA\_STAT.RUN bit indicates that the DMA channel is running, the channel has moved on to processing another descriptor. The event handler can increment its count and exit. If the DMA\_STAT.RUN bit indicates that the channel is not running, the channel is paused because either:

- There are no more descriptor sets to process, or
- The last descriptor set was queued too late

Where too late means that the modification of the DMA\_CFG of the next-to-last descriptor set occurred after that descriptor was read into the DMA channel. In this case, the event handler does the following:

- Writes the DMA\_CFG value appropriate for the last descriptor set to DMA\_CFG register of the DMA channel,
- Increments the completed descriptor count, and
- Exits

If the event latencies of the system are large enough to cause any of the events to be dropped, this system can fail. An event handler capable of safely synchronizing multiple descriptor set interrupt requests is complex, performing several MMR accesses to ensure robust operation. In such a system environment, a minimal event synchronization method is preferred.

## Queues Using Minimal Events

In this system, only one DMA interrupt request or trigger event is generated in the queue at any time. The DMA event handler for this system can also be extremely short. Here, the descriptor queue is organized into an active and a waiting portion, where events are enabled only on the last descriptor set in each portion.

When each new DMA request is processed, the software fills in the content of a new descriptor set and adds it to the waiting portion of the queue. The DMA\_CFG descriptor of the descriptor set must have the flow set to stop mode. If more than one request is received before the DMA queue completion event occurs, the non-interrupt code queues later descriptor sets. It forms a waiting portion of the queue separate from the active portion of the queue that the DMA channel is processing. In other words, all but the last active descriptor sets contain flow values for a descriptorbased mode and have no event enable set.

The last active descriptor set has the stop flow mode and an event generation enabled. Also, all but the last waiting descriptor sets are configured for descriptor-based flow modes with no event generation. Only the last waiting descriptor set is configured for stop flow mode and event generation enabled. This configuration ensures that the DMA channel can automatically process the whole active queue before then issuing one event. Also, this arrangement makes it easy to start the waiting queue within the event handler by a single DMA\_CFG register write.

After queuing a new waiting descriptor, the non-interrupt software leaves a message for its interrupt handler in a memory mailbox location. The location contains the desired DMA\_CFG value for starting the first waiting descriptor set in the waiting queue (or 0, indicating no waiting descriptors).

The software must not modify the contents of the active descriptor set queue directly once processing by the DMA channel has started, unless careful synchronization measures are taken. In the most straightforward implementation of a descriptor set queue, the DMA manager software never modifies descriptors on the active queue. Instead, the

DMA manager waits until the DMA queue completion event indicates that the processing of the entire active queue is complete.

When a DMA queue completion event is received, the event handler reads the mailbox from the non-interrupt software and writes the value to the DMA\_CFG register of the DMA channel. This write to a register restarts the queue, effectively transforming the waiting queue to an active queue. The event handler then passes a message back to the non-interrupt software indicating the location of the last descriptor set accepted into the active queue.

However, the event handler can read its mailbox and find a DMA\_CFG value of zero, indicating there is no more work to perform. It then passes an appropriate message back to the non-interrupt software indicating that the queue has stopped.

The non-interrupt software which accepts new DMA work unit requests must synchronize the activation of a new work unit with the interrupt handler. If the queue has stopped (the mailbox from the event handler is zero), the non-interrupt software must start the queue. (The queue starts by writing the first descriptor sets DMA\_CFG value to the DMA\_CFG register of the channel). If the queue is not stopped, the non-interrupt software must not write the DMA\_CFG register. (This write causes a DMA error). Instead, it must queue the descriptor onto the waiting queue and update its mailbox directed to the event handler.

## ADSP-SC59x DMA Register Descriptions

The Direct Memory Access module (DMA) contains the following registers.

Table 36-15: ADSP-SC59x DMA Register List

| Name           | Description                                        |
|----------------|----------------------------------------------------|
| DMA_ADDRSTART  | Start Address of Current Buffer Register           |
| DMA_ADDR_CUR   | Current Address Register                           |
| DMA_BWLCNT     | Bandwidth Limit Count Register                     |
| DMA_BWLCNT_CUR | Bandwidth Limit Count Current Register             |
| DMA_BWMCNT     | Bandwidth Monitor Count Register                   |
| DMA_BWMCNT_CUR | Bandwidth Monitor Count Current Register           |
| DMA_CFG        | Configuration Register                             |
| DMA_DSCPTR_CUR | Current Descriptor Pointer Register                |
| DMA_DSCPTR_NXT | Pointer to Next Initial Descriptor Register        |
| DMA_DSCPTR_PRV | Previous Initial Descriptor Pointer Register       |
| DMA_STAT       | Status Register                                    |
| DMA_XCNT       | Inner Loop Count Start Value Register              |
| DMA_XCNT_CUR   | Current Count (1D) or Intra-row XCNT (2D) Register |
| DMA_XMOD       | Inner Loop Address Increment Register              |

Table 36-15: ADSP-SC59x DMA Register List (Continued)

| Name         | Description                                     |
|--------------|-------------------------------------------------|
| DMA_YCNT     | Outer Loop Count Start Value (2D only) Register |
| DMA_YCNT_CUR | Current Row Count (2D only) Register            |
| DMA_YMOD     | Outer Loop Address Increment (2D only) Register |

## Start Address of Current Buffer Register

The DMA\_ADDRSTART register contains the start address of the work unit currently targeted for DMA. This register is read/write prior to enabling the channel, but is read-only after enabling channel.

Figure 36-5: DMA\_ADDRSTART Register Diagram

<!-- image -->

Table 36-16: DMA\_ADDRSTART Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Work Unit Address Start Value. The DMA_ADDRSTART.VALUE bit field contains the start address of the work unit currently targeted for DMA. |

## Current Address Register

The DMA\_ADDR\_CUR register contains the present memory transfer address for a given work unit. At the start of a work unit, the DMA\_ADDR\_CUR register is loaded from the DMA\_ADDRSTART register, and the DMA\_ADDR\_CUR register is incremented as each transfer occurs. The DMA\_ADDR\_CUR register is read/write prior to enabling the channel, but is read-only after enabling the channel.

Figure 36-6: DMA\_ADDR\_CUR Register Diagram

<!-- image -->

Table 36-17: DMA\_ADDR\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Work Unit Current Address Value. The DMA_ADDR_CUR.VALUE bit field contains the present memory transfer address for a given work unit. |

## Bandwidth Limit Count Register

The DMA\_BWLCNT register contains a count that determines how often the DMA issues memory transactions. The DMA loads the value from the DMA\_BWLCNT register into the DMA\_BWLCNT\_CUR register and decrements the current value each SYSCLK cycle. When DMA\_BWLCNT\_CUR reaches 0x0000, the next request is issued, and the DMA reloads DMA\_BWLCNT\_CUR . This bandwidth limit functionality is not applied to descriptor fetch requests. Programming 0x0000 allows the DMA to request as often as possible. 0xFFFF is a special case and causes requests to stop.

Figure 36-7: DMA\_BWLCNT Register Diagram

<!-- image -->

Table 36-18: DMA\_BWLCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | VALUE      | Bandwidth Limit Count. The DMA_BWLCNT.VALUE bit field contains a count that determines how often the DMAissues memory transactions. |

## Bandwidth Limit Count Current Register

The DMA\_BWLCNT\_CUR register contains the number of SYSCLK count cycles remaining before the next request is issued.

Figure 36-8: DMA\_BWLCNT\_CUR Register Diagram

<!-- image -->

Table 36-19: DMA\_BWLCNT\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------|
| 15:0               | VALUE      | Bandwidth Limit Count Current. The DMA_BWLCNT_CUR.VALUE bit field contains the number of SYSCLK count |
| (R/NW)             |            | cycles remaining before the next request is issued.                                                   |

## Bandwidth Monitor Count Register

The DMA\_BWMCNT register contains the maximum number of SYSCLK cycles allowed for a work unit to complete. Each time the DMA\_CFG register is written (MMR access only), a work unit ends or an autobuffer wraps. The DMA loads the value in this register into the DMA\_BWMCNT\_CUR register.

The DMA decrements DMA\_BWMCNT\_CUR every SYSCLK a work unit is active. If the DMA\_BWMCNT\_CUR register reaches 0x0000\_0000, the DMA\_STAT.IRQERR bit is set, and the DMA\_STAT.ERRC bit field is set to 0x6. The DMA\_BWMCNT\_CUR remains at 0x0000\_0000 until it is reloaded when the work unit completes.

Unlike other errors, a bandwidth monitor error does not stop work unit processing. Programming 0x0000\_0000 disables bandwidth monitor functionality.

Figure 36-9: DMA\_BWMCNT Register Diagram

<!-- image -->

Table 36-20: DMA\_BWMCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Bandwidth Monitor Count. The DMA_BWMCNT.VALUE bit field contains the maximum number of SYSCLK cy- cles allowed for a work unit to complete. |

## Bandwidth Monitor Count Current Register

The DMA\_BWMCNT\_CUR register contains the number of cycles remaining for the current descriptor to complete.

Figure 36-10: DMA\_BWMCNT\_CUR Register Diagram

<!-- image -->

Table 36-21: DMA\_BWMCNT\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0               | VALUE      | Bandwidth Monitor Count Current. The DMA_BWMCNT_CUR.VALUE bit field contains the number of cycles remaining for the current descriptor to complete. |
| (R/NW)             |            |                                                                                                                                                     |

## Configuration Register

The DMA\_CFG register sets up DMA parameters and operation modes. Writing to the DMA\_CFG register while a DMA process is already running causes a DMA error (except when clearing the DMA\_CFG.EN bit).

Figure 36-11: DMA\_CFG Register Diagram

<!-- image -->

Table 36-22: DMA\_CFG Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | PDRF       | Peripheral Data Request Forward. The DMA_CFG.PDRF bit defines how the DMAhandles data requests from the pe- ripheral while in idle state after a stop mode or memory read work unit. If set, the DMAforwards the peripheral data request as an interrupt. This bit applies only to the DMA_CFG.FLOW bits configured for stop and DMA_CFG.WNR bits configured for memory read. | Peripheral Data Request Forward. The DMA_CFG.PDRF bit defines how the DMAhandles data requests from the pe- ripheral while in idle state after a stop mode or memory read work unit. If set, the DMAforwards the peripheral data request as an interrupt. This bit applies only to the DMA_CFG.FLOW bits configured for stop and DMA_CFG.WNR bits configured for memory read. |
| 28 (R/W)           | PDRF       | 0                                                                                                                                                                                                                                                                                                                                                                             | Peripheral Data Request Not Forwarded                                                                                                                                                                                                                                                                                                                                         |
| 28 (R/W)           | PDRF       | 1                                                                                                                                                                                                                                                                                                                                                                             | Peripheral Data Request Forwarded                                                                                                                                                                                                                                                                                                                                             |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                        | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | TWOD       | Two Dimension Addressing Enable. The DMA_CFG.TWOD bit selects whether the DMAaddressing involves only DMA_XCNT and DMA_XMOD (one-dimensional DMA) or also involves DMA_YCNT and DMA_YMOD (two-dimensional DMA).                                                                                                                                                                                                                                | Two Dimension Addressing Enable. The DMA_CFG.TWOD bit selects whether the DMAaddressing involves only DMA_XCNT and DMA_XMOD (one-dimensional DMA) or also involves DMA_YCNT and DMA_YMOD (two-dimensional DMA).                                                                                                                                                                                                                                |
| 26 (R/W)           | TWOD       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                              | One-Dimensional Addressing                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 26 (R/W)           | TWOD       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                              | Two-Dimensional Addressing                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 25 (R/W)           | DESCIDCPY  | Descriptor ID Copy Control. The DMA_CFG.DESCIDCPY bit specifies when to copy the initial descriptor pointer to the DMA_DSCPTR_PRV register. A bus write to the DMA_CFG register to clear the DMA_CFG.EN bit causes theDMA to immediately use the new value of the DMA_CFG.DESCIDCPY bit. To preserve consistency (if required by application), match the new value of this bit to the previous value.                                          | Descriptor ID Copy Control. The DMA_CFG.DESCIDCPY bit specifies when to copy the initial descriptor pointer to the DMA_DSCPTR_PRV register. A bus write to the DMA_CFG register to clear the DMA_CFG.EN bit causes theDMA to immediately use the new value of the DMA_CFG.DESCIDCPY bit. To preserve consistency (if required by application), match the new value of this bit to the previous value.                                          |
| 25 (R/W)           | DESCIDCPY  | 0                                                                                                                                                                                                                                                                                                                                                                                                                                              | Never Copy                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 25 (R/W)           | DESCIDCPY  | 1                                                                                                                                                                                                                                                                                                                                                                                                                                              | Copy on Work Unit Complete                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 24 (R/W)           | TOVEN      | Trigger Overrun Error Enable. A trigger overrun occurs if more than one trigger was received before theDMA reached the trigger wait state. If DMA_CFG.TOVEN is set, a trigger overrun causes the DMAto flag an error. In cases where a trigger overrun is not a problem, clearing DMA_CFG.TOVEN prevents the overrun from causing an error and halting the DMA. The DMA_CFG.TOVEN operates independently of the DMA_CFG.TWAIT bit selec- tion. | Trigger Overrun Error Enable. A trigger overrun occurs if more than one trigger was received before theDMA reached the trigger wait state. If DMA_CFG.TOVEN is set, a trigger overrun causes the DMAto flag an error. In cases where a trigger overrun is not a problem, clearing DMA_CFG.TOVEN prevents the overrun from causing an error and halting the DMA. The DMA_CFG.TOVEN operates independently of the DMA_CFG.TWAIT bit selec- tion. |
| 24 (R/W)           | TOVEN      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                              | Ignore Trigger Overrun                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 24 (R/W)           | TOVEN      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                              | Error on Trigger Overrun                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 23:22 (R/W)        | TRIG       | Generate Outgoing Trigger. The DMA_CFG.TRIG selects whether the DMAissues an outgoing trigger, based on the work unit counter values. In one-dimensional mode, the only options are to trigger at the end of the whole work unit (trigger when DMA_XCNT_CUR reaches 0) or not to trigger at all. If in one-dimensional addressing mode, programming the DMA_CFG.TRIG bit field to trigger when DMA_YCNT_CUR reaches 0 (or to re- served) causes the DMAto flag a configuration error. In two-dimensional addressing mode, the trigger options are: at the end of each row of the inner loop (when DMA_XCNT_CUR reaches 0), only after completing the whole work unit (when DMA_YCNT_CUR reaches 0), or no trigger. If in two-dimensional mode and set to trigger when DMA_XCNT_CUR reaches 0, the DMAalso issues a trig- ger at the end of the work unit. If in two-dimensional addressing mode, programming DMA_CFG.TRIG to reserved causes the DMAto flag a configuration error. If DMA_CFG.TRIG is non-zero and the peripheral issues a finish command, the DMAissues a trigger after the finish procedure is complete. For more information about trigger generation timing, see the trigger section of the DMAfunctional description. | Generate Outgoing Trigger. The DMA_CFG.TRIG selects whether the DMAissues an outgoing trigger, based on the work unit counter values. In one-dimensional mode, the only options are to trigger at the end of the whole work unit (trigger when DMA_XCNT_CUR reaches 0) or not to trigger at all. If in one-dimensional addressing mode, programming the DMA_CFG.TRIG bit field to trigger when DMA_YCNT_CUR reaches 0 (or to re- served) causes the DMAto flag a configuration error. In two-dimensional addressing mode, the trigger options are: at the end of each row of the inner loop (when DMA_XCNT_CUR reaches 0), only after completing the whole work unit (when DMA_YCNT_CUR reaches 0), or no trigger. If in two-dimensional mode and set to trigger when DMA_XCNT_CUR reaches 0, the DMAalso issues a trig- ger at the end of the work unit. If in two-dimensional addressing mode, programming DMA_CFG.TRIG to reserved causes the DMAto flag a configuration error. If DMA_CFG.TRIG is non-zero and the peripheral issues a finish command, the DMAissues a trigger after the finish procedure is complete. For more information about trigger generation timing, see the trigger section of the DMAfunctional description. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Never Assert Trigger                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Trigger When XCNTCUR Reaches 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Trigger When YCNTCUR Reaches 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21:20 (R/W)        | INT        | Generate Interrupt Request. The DMA_CFG.INT bit field selects whether an interrupt request goes to the core based on work unit status or a peripheral interrupt request. For one-dimensional mode, setting the DMA_CFG.INT bits to generate an interrupt request when the DMA_YCNT_CUR register reaches 0 causes the DMAto flag a con- | Generate Interrupt Request. The DMA_CFG.INT bit field selects whether an interrupt request goes to the core based on work unit status or a peripheral interrupt request. For one-dimensional mode, setting the DMA_CFG.INT bits to generate an interrupt request when the DMA_YCNT_CUR register reaches 0 causes the DMAto flag a con- |
| 21:20 (R/W)        | INT        | 0 bit next descriptor set 0                                                                                                                                                                                                                                                                                                            | Never Assert Interrupt field specifies the number of descriptor descriptor fetch. The DMAloads set contains the next descriptor pointer does not contain the next descriptor                                                                                                                                                           |
| 21:20 (R/W)        | INT        | 1 2                                                                                                                                                                                                                                                                                                                                    | Interrupt When X Count Expires Interrupt When Y Count Expires                                                                                                                                                                                                                                                                          |
| 21:20 (R/W)        | INT        | 3                                                                                                                                                                                                                                                                                                                                      | Peripheral Interrupt request                                                                                                                                                                                                                                                                                                           |
| 18:16 (R/W)        | NDSIZE     | Next Descriptor Set Size.                                                                                                                                                                                                                                                                                                              | Next Descriptor Set Size.                                                                                                                                                                                                                                                                                                              |
| 18:16 (R/W)        | NDSIZE     |                                                                                                                                                                                                                                                                                                                                        | Fetch One Descriptor Element                                                                                                                                                                                                                                                                                                           |
| 18:16 (R/W)        | NDSIZE     | 1                                                                                                                                                                                                                                                                                                                                      | Fetch Two Descriptor Elements                                                                                                                                                                                                                                                                                                          |
| 18:16 (R/W)        | NDSIZE     | 2                                                                                                                                                                                                                                                                                                                                      | Fetch Three Descriptor Elements                                                                                                                                                                                                                                                                                                        |
| 18:16 (R/W)        | NDSIZE     | 3 4                                                                                                                                                                                                                                                                                                                                    | Fetch Four Descriptor Elements                                                                                                                                                                                                                                                                                                         |
| 18:16 (R/W)        | NDSIZE     |                                                                                                                                                                                                                                                                                                                                        | Fetch Five Descriptor Elements                                                                                                                                                                                                                                                                                                         |
| 18:16 (R/W)        | NDSIZE     | 5                                                                                                                                                                                                                                                                                                                                      | Fetch Six Descriptor Elements                                                                                                                                                                                                                                                                                                          |
| 18:16 (R/W)        | NDSIZE     | 6                                                                                                                                                                                                                                                                                                                                      | Fetch Seven Descriptor Elements                                                                                                                                                                                                                                                                                                        |
| 18:16 (R/W)        | NDSIZE     | 7                                                                                                                                                                                                                                                                                                                                      | Reserved                                                                                                                                                                                                                                                                                                                               |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | TWAIT      | Wait for Trigger. The DMA_CFG.TWAIT bit controls whether the DMAwaits for an incoming trigger from another channel or user. If the DMA_CFG.TWAIT bit is set, the DMAenters the wait state before starting the next work unit, including descriptor fetch when in de- scriptor mode. Do not use the wait for trigger control for descriptor-on-demand mode which causes an error. For more information, see the trigger section of the DMAfunc- tional description. |
| 14:12 (R/W)        | FLOW       | 1 Wait for Trigger (Halt before Work Unit) Next Operation. The DMA_CFG.FLOW bit field selects the descriptor handling options. 0 STOP. See the Stop Flow Mode section. 1 AUTO. See the Autobuffer Flow Mode section. 2 Reserved                                                                                                                                                                                                                                    |
| 10:8 (R/W)         | MSIZE      | Demand section. Memory Transfer Word Size. The DMA_CFG.MSIZE bits select memory transfer sizes of 8-, 16-, 32-, 64-, 128-, 256-bit words. The transfer start address ( DMA_ADDRSTART ) and transfer values ( DMA_XMOD , and, if needed, DMA_YMOD ) must be a multiple of the memory transfer unit size. 2 4 Bytes 3 8 Bytes 4 16 Bytes 5 32 Bytes                                                                                                                  |
|                    |            | or increment                                                                                                                                                                                                                                                                                                                                                                                                                                                       |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6:4 (R/W)          | PSIZE      | Peripheral Transfer Word Size. The DMA_CFG.PSIZE bits select peripheral transfer sizes as 8, 16, 32, or 64 bits wide. Each request and grant results in a single peripheral access. There are no bursts on the peripheral bus, so the DMA_CFG.PSIZE selection must be less than or equal to the width of the bus. If the selection is greater than the bus width, a configuration error occurs. The peripheral bus of the processor is dedicated to DMAand peripheral accesses.                                                          | Peripheral Transfer Word Size. The DMA_CFG.PSIZE bits select peripheral transfer sizes as 8, 16, 32, or 64 bits wide. Each request and grant results in a single peripheral access. There are no bursts on the peripheral bus, so the DMA_CFG.PSIZE selection must be less than or equal to the width of the bus. If the selection is greater than the bus width, a configuration error occurs. The peripheral bus of the processor is dedicated to DMAand peripheral accesses.                                                          |
| 3 (R/W)            | CADDR      | Use Current Address. When the DMA_CFG.CADDR bit is cleared, the DMAloads the DMA_ADDRSTART register on the first access of the work unit. When the DMA_CFG.CADDR bit is set, the DMAuses the DMA_ADDR_CUR register value for the starting address for the work unit and writes the same value to the DMA_ADDRSTART register. This operation permits continuation of a previous work unit. When DMAuses this mode, the fetched start address value (as part of the descriptor set at the end of a de- scriptor list or array) is ignored. | Use Current Address. When the DMA_CFG.CADDR bit is cleared, the DMAloads the DMA_ADDRSTART register on the first access of the work unit. When the DMA_CFG.CADDR bit is set, the DMAuses the DMA_ADDR_CUR register value for the starting address for the work unit and writes the same value to the DMA_ADDRSTART register. This operation permits continuation of a previous work unit. When DMAuses this mode, the fetched start address value (as part of the descriptor set at the end of a de- scriptor list or array) is ignored. |
| 3 (R/W)            | CADDR      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Load Starting Address                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 3 (R/W)            | CADDR      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Use Current Address                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | SYNC       | Synchronize Work Unit Transitions. Setting the DMA_CFG.SYNC bit clears the DMAFIFO and pointers before starting the first work unit of a work unit chain. When the transfer direction is memory read/transmit ( DMA_CFG.WNR =0), theDMA waits until all data transmits to a peripheral before moving on to the next work unit, clearing the FIFO and pointers. When the transfer direction is memory write/receive ( DMA_CFG.WNR =1), theDMA ignores the DMA_CFG.SYNC bit value after processing the first work unit of a work unit chain. As a channel can receive data when turned on but idle, data from the pe- ripheral can still be in the FIFO even though the channel was not programmed. When the DMA_CFG.SYNC bit field is set at the beginning of a work unit chain (during the first work unit), the DMAclears the FIFO, erasing the data put into the FIFO while the channel was idle. | Synchronize Work Unit Transitions. Setting the DMA_CFG.SYNC bit clears the DMAFIFO and pointers before starting the first work unit of a work unit chain. When the transfer direction is memory read/transmit ( DMA_CFG.WNR =0), theDMA waits until all data transmits to a peripheral before moving on to the next work unit, clearing the FIFO and pointers. When the transfer direction is memory write/receive ( DMA_CFG.WNR =1), theDMA ignores the DMA_CFG.SYNC bit value after processing the first work unit of a work unit chain. As a channel can receive data when turned on but idle, data from the pe- ripheral can still be in the FIFO even though the channel was not programmed. When the DMA_CFG.SYNC bit field is set at the beginning of a work unit chain (during the first work unit), the DMAclears the FIFO, erasing the data put into the FIFO while the channel was idle. |
| 2 (R/W)            | SYNC       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | No Synchronization                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 2 (R/W)            | SYNC       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Synchronize Channel                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/NW)           | WNR        | Write/Read Channel Direction. The DMA_CFG.WNR selects receive (write to memory) or transmit (read from memo- ry) channel direction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Write/Read Channel Direction. The DMA_CFG.WNR selects receive (write to memory) or transmit (read from memo- ry) channel direction.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 1 (R/NW)           | WNR        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Transmit (Read from memory)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1 (R/NW)           | WNR        | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Receive (Write to memory)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

Table 36-22: DMA\_CFG Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0 (R/W)            | EN         | DMAChannel Enable. The DMA_CFG.EN bit enables the selected DMAchannel. When a peripheral DMAchannel is enabled, data requests from the peripheral denote DMArequests. When a channel is disabled, the DMAunit ignores the peripheral data request and passes it directly to the system event controller. To avoid unexpected results, enable the DMAchannel before enabling the peripheral, and disable the peripheral before disabling the DMAchannel. A transition of the DMA_CFG.EN bit from 0 to 1 creates a hard reset of all internal counters and states, including the DMA_STAT register. (All other register values re- main unaffected.) A transition from 1 to 0 maintains all counters and registers for reading and analysis. Note that if a descriptor loads when this bit is cleared (see the DMA_CFG.FLOW field), the DMAtransitions to the off or idle state after the descriptor load is complete. 0 Disable |
| 0 (R/W)            |            | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 0 (R/W)            |            |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Current Descriptor Pointer Register

The DMA\_DSCPTR\_CUR register contains the memory address for the next descriptor to be loaded. The DMA\_DSCPTR\_CUR register is read/write prior to enabling the channel, but is read-only after enabling the channel. For DMA\_CFG.FLOW mode settings that involve descriptor fetches, this register is used to read descriptors into appropriate MMRs before a work unit begins. For descriptor list mode, the DMA\_DSCPTR\_CUR register is initialized from the DMA\_DSCPTR\_NXT register before fetching each descriptor set. Then, the address in the DMA\_DSCPTR\_CUR register increments as each descriptor is read.

When the entire descriptor set has been read, the DMA\_DSCPTR\_CUR register contains this value:

DMA\_DSCPTR\_CUR = Descriptor Start Address + Descriptor Size (# of elements)

For descriptor array mode, the DMA\_DSCPTR\_CUR register, and not the DMA\_DSCPTR\_NXT register, must be programmed by a MMR access before starting DMA operation.

Figure 36-12: DMA\_DSCPTR\_CUR Register Diagram

<!-- image -->

Table 36-23: DMA\_DSCPTR\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                      |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Pointer for Current Descriptor Element. The DMA_DSCPTR_CUR.VALUE bit field contains the memory address for the next descriptor to be loaded. |

## Pointer to Next Initial Descriptor Register

The DMA\_DSCPTR\_NXT register specifies the start location of the next descriptor set, which begins when the DMA activity specified by the current descriptor set completes. This register is read/write prior to enabling the channel, but is read-only after enabling channel.

The DMA\_DSCPTR\_NXT register is only used in descriptor list mode. At the start of a descriptor fetch in this mode, the DMA\_DSCPTR\_NXT register is copied into the DMA\_DSCPTR\_CUR register. During descriptor fetch, the DMA increments the DMA\_DSCPTR\_CUR register value after reading each element of the descriptor set.

In descriptor list mode, the DMA\_DSCPTR\_NXT register (not the DMA\_DSCPTR\_CUR register) must be programmed directly through MMR access, before the DMA operation is started. In descriptor array mode, the DMA disregards the DMA\_DSCPTR\_NXT register and uses the DMA\_DSCPTR\_CUR register to control descriptor fetch.

Figure 36-13: DMA\_DSCPTR\_NXT Register Diagram

<!-- image -->

Table 36-24: DMA\_DSCPTR\_NXT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                       |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Pointer to Next Descriptor Set. The DMA_DSCPTR_NXT.VALUE bit field specifies the start location of the next de- scriptor set. |

## Previous Initial Descriptor Pointer Register

The DMA\_DSCPTR\_PRV register contains the initial descriptor pointer for the previous work unit. If DMA\_CFG.DESCIDCPY is set, the DMA copies the initial descriptor pointer to DMA\_DSCPTR\_PRV after the work unit completes. Otherwise, the value is not updated.

To indicate an overrun, bit 0 of the DMA\_DSCPTR\_PRV register is used as a previous descriptor pointer overrun (PDPO) status bit. Due to aligned addressing combined with all descriptors being 32 bits in width, bits 0 and 1 of all descriptor pointers must be zero. Otherwise, an alignment error occurs when used for descriptor fetches. As a result, bit 1 and 0 of the DMA\_DSCPTR\_PRV register can be used for status. For more information, see the section on descriptor pointer capture in the DMA functional description.

Figure 36-14: DMA\_DSCPTR\_PRV Register Diagram

<!-- image -->

Table 36-25: DMA\_DSCPTR\_PRV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:2 (R/NW)        | DESCPPREV  | Descriptor Pointer for Previous Element. The DMA_DSCPTR_PRV.DESCPPREV bit field contains the initial descriptor pointer for the previous work unit.                                                                                                                                                                  |
| 0 (R/NW)           | PDPO       | Previous Descriptor Pointer Overrun. The DMA_DSCPTR_PRV.PDPO bit signifies an alignment error. Due to aligned ad- dressing combined with all descriptors being 32 bits in width, bits 0 and 1 of all de- scriptor pointers must be zero. Otherwise, an alignment error would occur when used for descriptor fetches. |
| 0 (R/NW)           | PDPO       | 0 No Error Occurred                                                                                                                                                                                                                                                                                                  |
| 0 (R/NW)           | PDPO       | 1 Error Occurred                                                                                                                                                                                                                                                                                                     |

## Status Register

The DMA\_STAT register indicates the status of DMA work units, the FIFO, errors, interrupts, and triggers.

Figure 36-15: DMA\_STAT Register Diagram

<!-- image -->

Table 36-26: DMA\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 20 (R/NW)          | TWAIT      | Trigger Wait Status. The DMA_STAT.TWAIT bit indicates whether the DMAhas or has not received a trigger. This bit is set until it reaches the next wait state. At that point, the bit is cleared, the DMAstops processing that work unit, and the following work unit proc- esses. The DMAdoes not distinguish between one or more triggers received. 0 No Trigger Received |
| 18:16 (R/NW)       | FIFOFILL   | FIFO Fill Status. The DMA_STAT.FIFOFILL bit field reports the quantity of data in the FIFO rela- tive to available space.                                                                                                                                                                                                                                                  |

Table 36-26: DMA\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 6                                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                                            |
|                    |            | 7                                                                                                                                                                                                                                                                                                                   | Full                                                                                                                                                                                                                                                                                                                |
| 15:14 (R/NW)       | MBWID      | Memory Bus Width. The DMA_STAT.MBWID bit field indicates the width of the memory bus connected to this DMA.                                                                                                                                                                                                         | Memory Bus Width. The DMA_STAT.MBWID bit field indicates the width of the memory bus connected to this DMA.                                                                                                                                                                                                         |
|                    |            | 0                                                                                                                                                                                                                                                                                                                   | 2 Bytes                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                   | 4 Bytes                                                                                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                                                   | 8 Bytes                                                                                                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                                                                                                   | 16 Bytes                                                                                                                                                                                                                                                                                                            |
| 13:12 (R/NW)       | PBWID      | Peripheral Bus Width. The DMA_STAT.PBWID bit field indicates the width of the peripheral bus connected to this DMA.                                                                                                                                                                                                 | Peripheral Bus Width. The DMA_STAT.PBWID bit field indicates the width of the peripheral bus connected to this DMA.                                                                                                                                                                                                 |
|                    |            | 0                                                                                                                                                                                                                                                                                                                   | 1 Byte                                                                                                                                                                                                                                                                                                              |
|                    |            | 1                                                                                                                                                                                                                                                                                                                   | 2 Bytes                                                                                                                                                                                                                                                                                                             |
|                    |            | 2                                                                                                                                                                                                                                                                                                                   | 4 Bytes                                                                                                                                                                                                                                                                                                             |
|                    |            | 3                                                                                                                                                                                                                                                                                                                   | 8 Bytes                                                                                                                                                                                                                                                                                                             |
| 10:8 (R/NW)        | RUN        | Run Status. The DMA_STAT.RUN bit field reports the DMA's current operational state. If the DMAis in idle or stop state, the DMA_CFG.EN bit is either 0 or 1. This bit field does not clear when a transition of the DMA_CFG.EN bit from 0 to 1 occurs. The DMA_STAT.RUN clears automatically when the DMAcompletes. | Run Status. The DMA_STAT.RUN bit field reports the DMA's current operational state. If the DMAis in idle or stop state, the DMA_CFG.EN bit is either 0 or 1. This bit field does not clear when a transition of the DMA_CFG.EN bit from 0 to 1 occurs. The DMA_STAT.RUN clears automatically when the DMAcompletes. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                   | Idle/Stop State                                                                                                                                                                                                                                                                                                     |
|                    |            | 1                                                                                                                                                                                                                                                                                                                   | Descriptor Fetch                                                                                                                                                                                                                                                                                                    |
|                    |            | 2                                                                                                                                                                                                                                                                                                                   | Data Transfer                                                                                                                                                                                                                                                                                                       |
|                    |            | 3                                                                                                                                                                                                                                                                                                                   | Waiting for Trigger                                                                                                                                                                                                                                                                                                 |
|                    |            | 4                                                                                                                                                                                                                                                                                                                   | Waiting for Write ACK/FIFO Drain to Peripheral                                                                                                                                                                                                                                                                      |
|                    |            | 5                                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                                            |
|                    |            | 6                                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                                            |
|                    |            | 7                                                                                                                                                                                                                                                                                                                   | Reserved                                                                                                                                                                                                                                                                                                            |
| 6:4 (R/NW)         | ERRC       | Error Cause. When an interrupt request error occurs ( DMA_STAT.IRQERR ), the DMAupdates                                                                                                                                                                                                                             | Error Cause. When an interrupt request error occurs ( DMA_STAT.IRQERR ), the DMAupdates                                                                                                                                                                                                                             |

Table 36-26: DMA\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Configuration Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Illegal Write Occurred While Channel Running                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Address Alignment Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Memory Access or Fabric Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Trigger Overrun                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Bandwidth Monitor Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 2 (R/W1C)          | PIRQ       | Peripheral Interrupt Request. The DMA_STAT.PIRQ bit indicates a peripheral interrupt request. Programs can use the DMA_STAT.PIRQ bit status to help determine which DMAasserted the inter- rupt request. This bit also helps to distinguish between an interrupt request caused by the state of the work unit and an interrupt request caused by the peripheral.                                                                                                                                                                                                                  | Peripheral Interrupt Request. The DMA_STAT.PIRQ bit indicates a peripheral interrupt request. Programs can use the DMA_STAT.PIRQ bit status to help determine which DMAasserted the inter- rupt request. This bit also helps to distinguish between an interrupt request caused by the state of the work unit and an interrupt request caused by the peripheral.                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | No Interrupt request                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Interrupt Request signaled by peripheral                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 1 (R/W1C)          | IRQERR     | Error Interrupt Request. The DMA_STAT.IRQERR bit indicates that the DMAhas detected a documented rule violation during DMAprogramming or operation. The DMAcannot, however, flag all possible programming or operation issues to indicate errors. Programmers can use the DMA_STAT.IRQERR bit to help determine which DMAissued the error in- terrupt request. The DMA_STAT.IRQERR does not clear a transition of the DMA_CFG.EN bit from 0 to 1. Clear the DMA_STAT.IRQERR bit with a write-1- to-clear operation prior to the DMA_CFG.EN transition for the fields to be reset. | Error Interrupt Request. The DMA_STAT.IRQERR bit indicates that the DMAhas detected a documented rule violation during DMAprogramming or operation. The DMAcannot, however, flag all possible programming or operation issues to indicate errors. Programmers can use the DMA_STAT.IRQERR bit to help determine which DMAissued the error in- terrupt request. The DMA_STAT.IRQERR does not clear a transition of the DMA_CFG.EN bit from 0 to 1. Clear the DMA_STAT.IRQERR bit with a write-1- to-clear operation prior to the DMA_CFG.EN transition for the fields to be reset. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | No Error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Error Occurred                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W1C)          | IRQDONE    | Work Unit/Row Done Interrupt. The DMA_STAT.IRQDONE bit indicates that the DMAhas detected the completion of a work unit or row (inner loop count) and has issued an interrupt request. Programs can use the DMA_STAT.IRQDONE status to help determine which DMAasserted the interrupt request. Programs can also use these bits to help distinguish between an in- terrupt request based on the state of the work unit and an interrupt request made by                                                                                                                           | Work Unit/Row Done Interrupt. The DMA_STAT.IRQDONE bit indicates that the DMAhas detected the completion of a work unit or row (inner loop count) and has issued an interrupt request. Programs can use the DMA_STAT.IRQDONE status to help determine which DMAasserted the interrupt request. Programs can also use these bits to help distinguish between an in- terrupt request based on the state of the work unit and an interrupt request made by                                                                                                                           |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Inactive                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Active                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

## Inner Loop Count Start Value Register

For 2D DMA, the DMA\_XCNT register contains the inner loop count. This value selects the number of DMA\_CFG.MSIZE size data transfers that make up the length of a row. For 1D DMA, the DMA\_XCNT register specifies the number of DMA\_CFG.MSIZE size data transfers for the entire work unit. The DMA\_XCNT register is read/write prior to enabling the channel, but is read-only after enabling channel. Note that the DMA generates a configuration error if this register is 0x0 when a work unit begins.

Figure 36-16: DMA\_XCNT Register Diagram

<!-- image -->

Table 36-27: DMA\_XCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Work Unit Inner Loop Counter Start Value. For 2D DMA, the DMA_XCNT.VALUE bit field contains the inner loop count. For 1D DMA, DMA_XCNT.VALUE specifies the number of DMA_CFG.MSIZE size data transfers for the entire work unit. |

## Current Count (1D) or Intra-row XCNT (2D) Register

For 1D DMA, the DMA loads the DMA\_XCNT\_CUR register from the DMA\_XCNT register at the beginning of each work unit. For 2D DMA, the DMA loads the DMA\_XCNT\_CUR register from the DMA\_XCNT register after the end of each row. The DMA decrements the value in DMA\_XCNT\_CUR register each time a DMA\_CFG.MSIZE size data transfer occurs. When the count in DMA\_XCNT\_CUR register expires, the work unit is complete. In 2D DMA, the DMA\_XCNT\_CUR value is 0 only when the entire transfer is complete.

Figure 36-17: DMA\_XCNT\_CUR Register Diagram

<!-- image -->

Table 36-28: DMA\_XCNT\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Work Unit Outer Loop Counter Start Value. For 1D DMA, the DMA_XCNT_CUR.VALUE bit field holds the DMA_XCNT value at the beginning of each work unit. For 2D DMA, the DMA_XCNT_CUR.VALUE bit field holds the DMA_XCNT value after the end of each row. |

## Inner Loop Address Increment Register

The DMA\_XMOD register contains a signed, two's-complement byte address increment. In 1D DMA, this increment is the stride that is applied after each DMA\_CFG.MSIZE size data transfer. The DMA\_XMOD register is read/write prior to enabling the channel, but is read-only after enabling the channel.

The DMA\_XMOD register value is specified in bytes, regardless of the work unit size. In 2D DMA, this increment is applied after each DMA\_CFG.MSIZE size data transfer in the inner loop, up to but not including the last DMA\_CFG.MSIZE size data transfer in each inner loop. After the last DMA\_CFG.MSIZE size data transfer in each inner loop, the DMA\_YMOD register is applied instead, including the last DMA\_CFG.MSIZE size data transfer of a work unit.

The DMA\_XMOD field can be set to 0. In this case, DMA is performed repeatedly to or from the same address. This approach can be useful for transferring data between a data register and an external memory-mapped peripheral.

Figure 36-18: DMA\_XMOD Register Diagram

<!-- image -->

Table 36-29: DMA\_XMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                         |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | VALUE      | Inner Loop Address Increment in Bytes. The DMA_XMOD.VALUE bit field contains a signed, two's-complement byte address increment. |

## Outer Loop Count Start Value (2D only) Register

For 2D DMA, the DMA\_YCNT register contains the outer loop count. This register is not used in 1D DMA mode. The DMA\_YCNT register specifies the number of rows in the outer loop of a 2D DMA sequence. The DMA\_YCNT register is read/write prior to enabling the channel, but is read-only after enabling channel. Note that the DMA generates a configuration error if this register is 0x0 when a work unit begins.

Figure 36-19: DMA\_YCNT Register Diagram

<!-- image -->

Table 36-30: DMA\_YCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Work Unit Inner Loop Counter Current Value.                             |
| (R/W)              |            | For 2D DMA, the DMA_YCNT.VALUE bit field contains the outer loop count. |

## Current Row Count (2D only) Register

For 2D DMA, the DMA loads the DMA\_YCNT\_CUR register from the DMA\_YCNT register at the beginning of each 2D DMA session. The DMA\_YCNT\_CUR register is not used for 1D DMA. The DMA decrements the DMA\_YCNT\_CUR register each time the DMA\_XCNT\_CUR register expires during 2D DMA operation, signifying the completion of an entire row transfer.

Figure 36-20: DMA\_YCNT\_CUR Register Diagram

<!-- image -->

Table 36-31: DMA\_YCNT\_CUR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/NW)        | VALUE      | Work Unit Outer Loop Counter Current Value. For 2D DMA, the DMA_YCNT_CUR.VALUE bit field holds the value from the DMA_YCNT register at the beginning of each 2D DMAsession. |

## Outer Loop Address Increment (2D only) Register

The DMA\_YMOD register contains a signed, two's-complement value. This byte address increment is applied after each decrement of the DMA\_YCNT\_CUR register. The value is the offset between the last word of one row and the first word of the next row. Note that DMA\_YMOD is specified in bytes, regardless of the work unit size. The DMA\_YMOD register is read/write prior to enabling the channel, but is read-only after enabling the channel.

Figure 36-21: DMA\_YMOD Register Diagram

<!-- image -->

Table 36-32: DMA\_YMOD Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                 |
|--------------------|------------|-------------------------------------------------------------------------|
| 31:0               | VALUE      | Outer Loop Address Increment in Bytes.                                  |
| (R/W)              |            | The DMA_YMOD.VALUE bit field contains a signed, two's-complement value. |