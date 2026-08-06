## 18   Link Port (LP)

Link ports allow the processor to connect to other processors or peripheral link ports using a simple communication protocol for high-speed parallel data transfer. This peripheral allows various I/O peripheral interconnection schemes to I/O peripheral devices as well as co-processing and multiprocessing schemes.

The link ports of the processor support 2- and 4-bit wide data transfers DDR mode. The link port pins are multiplexed in the GPIO ports. For information on processor multiplexing, see the data sheet for the specific processor.

Link ports can operate independently and simultaneously, allowing glueless high-speed connectivity of up to four external processors.

NOTE: Reference to SCLK in this chapter can be considered as OCLK0\_0.

## LP Features

All link ports are identical in their design and have the following common features.

- Bidirectional ports with four data signals ( LP\_D0 -LP\_D3 ), an acknowledge signal ( LP\_ACK ), and a clock signal ( LP\_CLK ).
- Provide high-speed, point-to-point data transfers to other processors, allowing different types of interconnections between multiple processors.
- Pack data into 32-bit words. The processor can directly read this data or transfer it through DMA to or from on-chip memory.
- Support for data buffering through a 2-deep FIFO for transmit and a 4-deep FIFO for receive.
- Programmable clock and acknowledge based handshake mechanism for efficient communication.
- A dedicated DMA channel.
- Support for data transfer in 2-bit mode (DDR) and 4-bit mode (DDR)

## LP Functional Description

This section provides a description of the link port, including a list of its registers and a functional block diagram.

## ADSP-2184x LP Register List

The Link Port LP is an 8-bit wide parallel port that can connect to another processor's LP or another LP-compatible device. This port allows a variety of interconnection schemes to I/O peripheral devices as well as co-processing and multiprocessing schemes. A set of registers governs LP operations. For more information on LP functionality, see the LP register descriptions.

Table 18-1: ADSP-2184x LP Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| LP_CTL        | Control Register                       |
| LP_DIV        | Clock Divider Value Register           |
| LP_RX         | Receive Buffer Register                |
| LP_STAT       | Status Register                        |
| LP_TX         | Transmit Buffer Register               |
| LP_TXIN_SHDW  | Shadow Input Transmit Buffer Register  |
| LP_TXOUT_SHDW | Shadow Output Transmit Buffer Register |

## ADSP-2184x LP Interrupt List

Table 18-2: ADSP-2184x LP Interrupt List

|   Interrupt ID | Name        | Description       | Sensitivity   |   DMA Channel |
|----------------|-------------|-------------------|---------------|---------------|
|            152 | LP0_DMA     | LP0 DMAChannel    |               |            30 |
|            153 | LP0_STAT    | LP0 Status        |               |               |
|            154 | LP1_DMA     | LP1 DMAChannel    |               |            36 |
|            155 | LP1_STAT    | LP1 Status        |               |               |
|            156 | LP0_DMA_ERR | LP0 DMAData Error |               |               |
|            157 | LP1_DMA_ERR | LP1 DMAData Error |               |               |

## ADSP-2184x LP Trigger List

Table 18-3: ADSP-2184x LP Trigger List Generators

|   Trigger ID | Name    | Description    | Sensitivity   |
|--------------|---------|----------------|---------------|
|           67 | LP0_DMA | LP0 DMAChannel |               |
|           68 | LP1_DMA | LP1 DMAChannel |               |

Table 18-4: ADSP-2184x LP Trigger List Receivers

|   Trigger ID | Name    | Description    | Sensitivity   |
|--------------|---------|----------------|---------------|
|          114 | LP0_DMA | LP0 DMAChannel | Pulse         |
|          115 | LP1_DMA | LP1 DMAChannel | Pulse         |

## ADSP-2184x LP DMA Channel List

Table 18-5: ADSP-2184x LP DMA Channel List

| DMAID   | DMAChannel Name   | Description    |
|---------|-------------------|----------------|
| DMA30   | LP0_DMA           | LP0 DMAChannel |
| DMA36   | LP1_DMA           | LP1 DMAChannel |

## Block Diagram

The Link Port Block Diagram shows the block diagram of a link port.

Figure 18-1: Link Port Block Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000000_88ffb8acf96f13144d4d84be9c8422a6c67a8a0a40f503cf4815c648f9799411.png)

## External Connections

As shown in the Link Port Pin Connections figure, a link port has four data lines ( LP\_D0 -LP\_D3 ), a clock line ( LP\_CLK ), and an acknowledge line ( LP\_ACK ). A link port can act as either a transmitter or a receiver but not both at the same time.

X DENOTES THE LINK PORT NUMBER, 0-1

![Image](21_Link_Port_(LP)_artifacts/image_000001_f8940621afee939662cf6150d3a2fc64c4608902afbd135820c0f44d0b6ce8f1.png)

Figure 18-2: Link Port Pin Connections

Use external pull-downs for the LP\_CLK and LP\_ACK pins so that the link port can enable the transmitter and receiver, irrespective of the state of the other.

## Internal Blocks

As shown in the block diagram, the link ports have independent modules for transmit and receive. If enabled as transmitter, the link port uses a 2 deep 32-bit FIFO. If enabled as a receiver, the port uses a 4 deep 32-bit FIFO. The core MMR access bus interfaces with these FIFOs. The distributed DMA engines (DDE) use the system cross bar (SCB) interface to access the FIFO. The link port uses the LP\_CTL.TRAN bit to determine whether the module is enabled for transmit or receive operation.

## Architectural Concepts

The following sections describe the architectural concepts of the link port.

- FIFO Buffers
- Handshake for Link Port Enable Process
- Multi-Processor Connectivity

## FIFO Buffers

When a link port is configured as transmitter, the link port uses a 2-deep FIFO buffer. A shift register unpacks the single 32-bit word to either eight 4-bit data bytes or sixteen 2-bit data bytes, based on the selected transfer mode. As the FIFO has space for more data, the link port makes a new DMA request. If the FIFO becomes empty, the LP\_CLK signal is deasserted. The core can access FIFO through the LP\_TX register.

The core or DMA makes three writes (2-stage FIFO and 1 shift register) to the transmit buffer before it signals a full condition. The link port uses the LP\_STAT.FFST bit field to reflect the status of the FIFO but not the shift register full or empty condition. However, the program can poll the LP\_STAT.LPBS bit to discover whether the link port is driving data from the shift register to the pins. The LP\_STAT.LPBS bit is also set when receiver has held off transmission by driving LP\_ACK low.

- NOTE: When the 2-deep FIFO and the output shift-register overflow, any further write to the link port buffer overwrites the input stage of the FIFO.
- NOTE: The core can also read the transmit FIFO through the data LP\_TX register.
- NOTE: If the transmitter is disabled while performing writes to the transmit FIFO, a FIFO full condition is signaled after two writes.

The transmit buffer registers have shadow registers. Using these shadow registers, both stages of the 2-deep FIFO can be read without updating the status registers. The LP\_TXIN\_SHDW register corresponds to the input stage of the FIFO. The LP\_TXOUT\_SHDW register corresponds to the output stage of the FIFO as shown in the Transmit FIFO path figure.

Figure 18-3: Transmit FIFO path

![Image](21_Link_Port_(LP)_artifacts/image_000002_daed2ea5e1543f1aa526a61b7338f3a4889c1c60f1b7d5e3f4ca5d46625bbf5c.png)

When a link port is configured as receiver, data transfers to the core or DMA from the full 4-deep receive FIFO. An internal packing register packs data to 32 bits. Four reads can occur from the receive buffer by the core or DMA before it signals an empty condition. The link port uses the LP\_STAT.FFST bits to reflect the status of the 4-deep read buffer FIFO. The core can access this FIFO through the LP\_RX register.

NOTE: When receive FIFO overflows ( LP\_STAT.ROVF bit=1), any further data from the transmitter is lost. Only the data retained in the receive FIFO can be retrieved further.

The receiver drives the LP\_ACK output signal low, after the first byte of data for the next-to-last empty slot (in the 4-deep FIFO) is received. This functionality prevents data loss due to the transmitter starting transmission of the next word before the LP\_ACK signal reaches the transmitter. (The timing is due to the larger delay in synchronization.) This functionality guarantees that even after allowing for the extra synchronization cycle in the transmitter and receiver, there is no overflow in the receive FIFO. The LACK Generation Based on Receive FIFO Status figure shows how FIFO slots influence the acknowledge signal generation. The grayed sections show received data. The white sections show empty locations where the decision to pull LP\_ACK high is taken.

Figure 18-4: LACK Generation Based on Receive FIFO Status

![Image](21_Link_Port_(LP)_artifacts/image_000003_10a396088c01012656b127c6e7437c93128efd1ba6721cca1815f162e8c32203.png)

| 32-BIT   | 32-BIT   |
|----------|----------|
| 32-BIT   | 32-BIT   |
| 8-BIT    | 24-BIT   |
| 32-BIT   | 32-BIT   |

NOTE: The link port uses a 4-deep receive FIFO only under a worst case situation, as mentioned. In all other cases, respond as if the FIFO has only a 3-deep stage. The LP\_ACK signal is pulled high before the last stage of the FIFO.

The link port has memory-mapped buffers for both receive and transmit operations. A JTAG-based emulator can read the FIFO which can cause unexpected problems in data transfers. This activity can only happen during an emulation event (typically hitting a breakpoint or single-stepping). The emulator issues core reads through JTAG. To work around this issue, see the tools documentation for more information.

## Handshake for Link Port Enable Process

In a link port-based system, the transmitter and the receiver can be enabled at different times. Use external pull-downs for the LP\_CLK and LP\_ACK signals.

If the receiver is enabled before the transmitter, the external pull-down holds the LP\_CLK signal of the transmitter low. The receiver is held off. The receiver can wait for a rising edge on the LP\_CLK signal to assert its receive service request interrupt. This rising edge occurs only when transmitter starts driving the first data on to the bus, after the application enables it.

If the transmitter is enabled before the receiver, the external pull-down holds the LP\_ACK signal of the receiver low. Transmission is held off. Refer to the Enable the Transmitter Before the Receiver figure. The transmitter can wait for a rising edge on the LP\_ACK signal to assert its transmit service request interrupt. This rising edge is asserted as soon as the receiver is enabled after the hardware drives the LP\_ACK high.

Figure 18-5: Enable the Transmitter Before the Receiver

![Image](21_Link_Port_(LP)_artifacts/image_000004_dbbb3bb2bf6b7cf38d91630a259d0af1190137bdf450503af546f10a3d9574a3.png)

NOTE: Service request interrupts or status are asserted only when the link port (receiver or transmitter) is disabled.

## Multi-Processor Connectivity

Link ports can operate independently, allowing glueless connection with external processors. Link ports have dedicated DMA channels, allowing independent data transfers. The following group of figures shows some examples of different bus connection topology that can be used in multi-processor system design. The inter-connection methods are not limited to these examples.

Figure 18-7: Link Port Full-Duplex Transfer Model

![Image](21_Link_Port_(LP)_artifacts/image_000005_df322314ecc4af5d4c37d28a2e8e67fbfc016a2853e1274044ff2a012a754978.png)

![Image](21_Link_Port_(LP)_artifacts/image_000006_17d503a02e591cf0c88cf96b699c321bf9c666a625776de0f4e4eaa8eac870d7.png)

Figure 18-6: Central Processor-Based Model

![Image](21_Link_Port_(LP)_artifacts/image_000007_e2db9c1d3d20d1d08e43ae0313a8afa1929fbe82d4c7386aa1b86b003ee6639c.png)

Figure 18-8: Link Port Ring Model

The link port protocol does not include built-in support for multiple requesters. However, there can be situations where multiple devices try to become the bus requester at the same time. Multi-requester conflicts can be resolved using token passing. In token passing, the token is a software flag that passes between processors.

At reset, the token is set to reside in the link port of one device, making it the requester and the transmitter. When a receiver (completer) wants to become the requester, it can assert its LP\_ACK signal to get the attention of the requester. The requester knows, through the software protocol, whether to respond with actual data or whether the token is requested. If the requester wishes to give up the token, it can send back a user-defined token release word and thereafter clear its token flag.

Simultaneously, the completer sets its token and can thereafter transmit. The token release word can be any user-defined value. Because the transmitter and receiver expect a code word, this word does not need to be exclusive of normal data transmission. If the requester wishes to give up the token, it can send back a user-defined token release word and thereafter clear its token flag. Simultaneously, the completer examines the data sent back and if it is the token release word, the completer sets its token, and can thereafter transmit.

The link port protocol includes handshake mechanism to inform the other end of transfer (transmit or receive) of an enable instance. However, it does not support handshakes to inform a disabled instance, while a chunk of data transfers. The application must assume the disabled state of the other end and take appropriate action.

For example, in a multi-processing environment, a receiver did not read its full FIFO for an extended time due to internal bus arbitrations. The transmitter can require software or a peripheral timer-based timeout to inform the application that the LP\_ACK signal is low for an extended period.

## LP Operating Modes

The link port does not have particular modes of operation, as the peripheral is based on a simple protocol. The following sections explain the data transfer modes, using the core and using DMA.

- Core Data Transfers
- DMA Data Transfers

## LP Data Transfer Modes

This section describes link port DMA and core data transfers.

## Core Data Transfers

If DMA is disabled for a link port buffer, the processor core can write or read internal FIFO buffers as a memory-mapped register through the MMR access bus. To avoid FIFO overflow or underflow, the core can access the FIFO registers in one of the following ways.

- Access link port registers using an interrupt service routine (ISR) mapped to the data request interrupt of the link port. The interrupt request remains high only when the FIFO is accessible (when the FIFO is not full in transmit mode and not empty in receive mode).
- Poll the FIFO status bits of the LP\_STAT register. Write to the transmit FIFO if not full or read from the receive FIFO if not empty.

## DMA Data Transfers

Dedicated DMA channels are available for each link port. DMA-related activity is explained in the following steps.

1. Data Receive - Once the DMA channel and link port module are configured and enabled, the external device begins writing data to the FIFO through the data pins of the link port. The FIFO detects this activity and in turn sends a DMA request. After the request is granted, the DMA transfer progresses until the FIFO is empty.
2. Data Transmit - Once the DMA channel and link port module are configured and enabled, setting the LP\_CTL.EN bit automatically asserts a DMA request when the transmit FIFO is empty. After the request is granted, DMA fills the FIFO. The external device begins reading data from the FIFO through the data pins of the link port. The FIFO detects that there is room in the buffer and asserts another DMA request, continuing the process.

## LP Event Control

This section describes how the link port uses interrupts and status signals.

## Interrupt Signals

Each link port has two dedicated interrupt lines-a data request interrupt and a status interrupt. Data request interrupts are asserted based on FIFO conditions for data transfer. Status interrupts are asserted when a service request status or an overflow status is set. The following list explains each of these interrupts.

- Data Request Interrupt. Asserted if the FIFO is not full in transmission mode and the FIFO is not empty in reception mode. This functionality serves as a core triggered interrupt in non-DMA mode and as the DMA interrupt request in DMA mode. Generation of this interrupt is based on the LP\_STAT.FFST (status bit of the link port buffer).
- Link Port Transmit Service Request Interrupt (LTRQ) . Allow a disabled link port to generate an interrupt when an external access is attempted. When a link port is configured as transmitter, the transmit service request interrupt is enabled by setting the LP\_CTL.TRQMSK bit. When set, an external receiver can indicate to the disabled transmitter that it must receive data through the connected link port. The receiver does so by driving a high level on the LP\_ACK line. When the LP\_ACK of the disabled transmitter link port is detected high, a LP\_STAT.LTRQ interrupt is generated. The transmitter can enable itself for data transfer with the receiver. The link port needs a pull-down on LP\_ACK for this feature to function properly.
- Link Port Receive Service Request Interrupt (LRRQ) . When a link port is configured as receiver, this interrupt is enabled by setting the LP\_CTL.RRQMSK bit. When set, an external transmitter can indicate to the disabled receiver that it must receive data through the connected link port. The transmitter does so by driving out the first data. When the LP\_CLK of the disabled receiver link port is detected high, a LP\_STAT.LRRQ interrupt is generated. The receiver can further enable itself for data transfer with the transmitter. The link port needs a pull-down on the LP\_CLK signal for this feature to function properly.
- Link Port Receive Overflow Interrupt (LPOVF) . Generated when the receiver FIFO overflows and is enabled by setting the LP\_CTL.ROVFMSK bit. This interrupt can happen if the transmitter continues to transmit data even though the receiver has deasserted LP\_ACK signal causing the receive FIFO to overflow.

## Enabling Link Port Interrupts

A data request interrupt is a direct interrupt and can be controlled separately from the application.

To mask the interrupt, set the mask bits in LP\_CTL register corresponding to service interrupts and the overflow interrupt. These interrupts are ORed and fed to the SIC as a single LP\_STAT interrupt. These interrupts are latched and stored in the associated bits of LP\_STAT register. If an LP\_STAT interrupt occurs, in the ISR, programs can read the LP\_STAT register bits to determine the type of interrupt. These bits are write-one-to-clear (W1C); writing one to the bit resets the bit and disables the corresponding interrupt.

## Status and Error Signals

This section explains the various status signals in the LP\_STAT register.

- Transfer Status signals . The link port uses the bus status bit ( LP\_STAT.LPBS ) to give the status of the bus condition (busy or idle) when the link port is configured as transmitter. The LP\_STAT.LPBS is high if the link port drives data into the link port pins. Programs can poll this bit after polling the LP\_STAT.FFST bit to disable the link port safely.

The link buffer status ( LP\_STAT.FFST ) field directly indicates the status of the FIFO (including empty or full conditions) during data transfer. Software can poll this field in the LP\_STAT register before writing to the FIFO (in case of transmission) or reading from the FIFO (in case of reception). The LP\_STAT.FFST bit is automatically cleared when the link port is disabled.

- Transfer Request Status signals . The link port uses the receive request status ( LP\_STAT.LRRQ ) bit to indicate that an external receiver wants to receive data (in case the link port is a disabled transmitter). The link port uses the transmit request status ( LP\_STAT.LTRQ ) bit to indicate that an external transmitter wants to send data (in case the link port is a disabled receiver). Software can poll these bits to enable the transmitter or receiver accordingly.
- Error Status signals . In receive mode 32-bit data is received in eight chunks of 4-bit data. This data is then packed to a single 32-bit data before loading the FIFO. The link buffer error status ( LP\_STAT.LPACK ) bit is high during this packing process and goes low after packing.

The link port overflow status ( LP\_STAT.ROVF ) bit is set when the receive FIFO overflows. This event can occur if the transmitter continues to transmit data even though the receiver has deasserted LP\_ACK causing the receiver FIFO to overflow.

- Link Port Clock Division signals . In the SC84x family of processors, the Linkport peripheral is configured to use CLK\_09 as its input clock source from the Clock Division Unit (CDU). The CDU0 input clocks can be one of the following: DCLK\_0, CCLK2\_0, DCLK\_1, or CCLK2\_1.

The clock signal provided through CLK\_09 undergoes an internal division by 4 before reaching the link port peripheral. This division occurs in two stages, with the clock being divided by 2 at each stage.

After this division, the resulting clock serves as the input to the link port block. To further reduce the clock frequency, program the LP\_DIV register using the following equation:

```
LP_DIV = 0 → ; LP_CLK = SCLK
```

LP\_DIV not equal to 0; → LP\_CLK = SCLK / (2 × DIV)

Here, SCLK refers to the clock entering the link port block after the initial division by 4 from CLK\_09.

## LP Programming Model

The following sections provide information on configuring the operating mode and enabling the link ports.

- Setting Up a DMA Transmit Operation
- Setting Up a DMA Receive Operation
- Setting Up a Core Transmit Operation
- Setting Up a Core Receive Operation

## Setting Up a DMA Transmit Operation

This following procedure describes the typical steps for configuring the link ports in DMA transmit mode.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for DMA and for transfer status (service request interrupt).
3. Configure the link port to transmit by setting the LP\_CTL bit and enable the transmit request interrupt mask by setting the LP\_CTL.TRQMSK bit.
4. Program the link port clock divider by writing a value to the LP\_DIV register.
5. 5.
6. If using DMA stop mode or auto buffer mode, program the appropriate DMA registers. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Stop/Auto, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation and memory read).
6. Wait for the link port receiver (connected externally) to be enabled. The application can wait for the transmit service request interrupt to assert.
7. Clear the transmit service request interrupt status by writing 1 to the LP\_STAT.LTRQ bit.
8. Enable DMA by setting the DMA\_CFG.EN bit.
9. Enable the link port by setting the LP\_CTL.EN bit.
10. Wait for DMA to assert a transfer completion interrupt.
11. Clear the DMA interrupt source by writing 1 to the DMA\_STAT.IRQDONE bit.

## Setting Up a DMA Receive Operation

This section describes the typical steps for using the link ports in DMA receive mode.

1. Enable the link port pins in GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for DMA and for transfer status (service request interrupt).
3. Configure the link port for reception (clear the LP\_CTL.TRAN bit) and enable the receive request interrupt mask by setting the LP\_CTL.RRQMSK bit.
4. If using DMA stop mode or auto buffer mode, program the DMA registers. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Stop/Auto, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation and memory write).
5. If using DMA array mode or list mode, create DMA configuration data structures filled with components. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Array/List, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation, memory write and fetch =4/5) and DMA\_DSCPTR\_NXT register (if list mode). Further, program DMA configuration register (Array/List, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, Memory Write and Fetch =4/5) and program the DMA\_DSCPTR\_NXT register (if list mode).
6. Wait for the link port transmitter (connected externally) to be enabled with subsequent data transmission. The application can wait for the receive service request interrupt to assert.
7. Clear the receive service request interrupt status by writing 1 to the LP\_STAT.LRRQ bit.
8. Enable DMA by setting the DMA\_CFG.EN bit.
9. Enable the link port by setting the LP\_CTL.EN bit.
10. Wait for DMA to assert the transfer complete interrupt.
11. Clear the DMA interrupt source by writing 1 to the DMA\_STAT.IRQDONE bit of the DMA status register.

## Setting Up a Core Transmit Operation

This section describes the typical steps for using the link ports in processor core based transmission.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for data transfer and for transfer status (service request interrupt). The interrupt handlers for data transfer are the same source or ID as the DMA interrupt line in the SEC.
3. Configure the link port for transmission by setting the LP\_CTL.TRAN bit) and enable the transmit request interrupt mask by setting the LP\_CTL.TRQMSK bit).
4. Program the link port clock divider by writing a value in to the LP\_DIV register.
5. Wait for the link port receiver (connected externally) to be enabled. The application can wait for a transmit service request interrupt to assert.
6. Clear the transmit service request interrupt status by writing 1 to the LP\_STAT.LTRQ bit.

7. Enable the link port by setting the LP\_CTL.EN bit.
8. The data request interrupt is asserted whenever there is free space in the FIFO. The application can write to the LP\_TX register based on the FIFO conditions (half or empty) reflected in the LP\_STAT.FFST bit field.

## Setting Up a Core Receive Operation

This section describes the typical steps for using the link ports in processor core-based reception.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for data transfer and for transfer status (service request interrupt). The interrupt handlers for data transfer are the same source or ID as the DMA interrupt line in the SEC).
3. Configure link port for reception (clear LP\_CTL.TRAN bit). Enable the receive request interrupt mask bit (set LP\_CTL.RRQMSK ).
4. Wait for the link port transmit (connected externally) to be enabled with subsequent transmission of data. The application can wait for receive service request interrupt to be asserted.
5. Clear the receive service request interrupt status by writing 1 to the LP\_STAT.LRRQ bit.
6. Enable the link port by setting the LP\_CTL.EN bit.
7. The data request interrupt is asserted whenever there is free space in the FIFO. The application can read from the LP\_RX register based on the FIFO conditions (1 or 2 or 3 data available) which is reflected in the LP\_STAT.FFST bit field.

## ADSP-2184x LP Register Descriptions

Link Port (LP) contains the following registers.

Table 18-6: ADSP-2184x LP Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| LP_CTL        | Control Register                       |
| LP_DIV        | Clock Divider Value Register           |
| LP_RX         | Receive Buffer Register                |
| LP_STAT       | Status Register                        |
| LP_TX         | Transmit Buffer Register               |
| LP_TXIN_SHDW  | Shadow Input Transmit Buffer Register  |
| LP_TXOUT_SHDW | Shadow Output Transmit Buffer Register |

## Control Register

The LP\_CTL register provides LP interrupt masking, selection of transfer direction, and link port enable.

Figure 18-9: LP\_CTL Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000008_bbab37dced294168c6d8b166766ad72bcb9c682c8fa1b59116c7efd3ecf5d99c.png)

Table 18-7: LP\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/W)        | MODE       | Mode Select. The LP_CTL.MODE field indicates the mode of operation for double data rates (DDR). Note that the LRRQ interrupt is not supported in DDR mode. The LP receiver must be enabled by the user before starting the LP transmitter. For LP DDR mode, the value of LP DDR clock should be twice that of LP clock. CDU CLKO9 is used for the LP DDR clock. | Mode Select. The LP_CTL.MODE field indicates the mode of operation for double data rates (DDR). Note that the LRRQ interrupt is not supported in DDR mode. The LP receiver must be enabled by the user before starting the LP transmitter. For LP DDR mode, the value of LP DDR clock should be twice that of LP clock. CDU CLKO9 is used for the LP DDR clock. |
| 14:12 (R/W)        | MODE       | 5                                                                                                                                                                                                                                                                                                                                                               | 4 bit DDR. (D0, D1, D2 and D3 data pins are used)                                                                                                                                                                                                                                                                                                               |
| 11                 | ROVFMSK    | Receive FIFO Overflow Interrupt Mask.                                                                                                                                                                                                                                                                                                                           | Receive FIFO Overflow Interrupt Mask.                                                                                                                                                                                                                                                                                                                           |
| (R/W)              |            | 0                                                                                                                                                                                                                                                                                                                                                               | Mask Disable Receive FIFO Overflow Interrupt                                                                                                                                                                                                                                                                                                                    |
| 9 (R/W)            | RRQMSK     | Receive Request Interrupt Mask. Link Port Receive Request Mask                                                                                                                                                                                                                                                                                                  | Receive Request Interrupt Mask. Link Port Receive Request Mask                                                                                                                                                                                                                                                                                                  |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                               | Mask Disable Receive Request interrupt.                                                                                                                                                                                                                                                                                                                         |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                               | Unmask Enable Receive Request interrupt.                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | TRQMSK     | Transmit Request Interrupt Mask. Link Port Transmit Request Mask                                                                                                                                                                                                                                                                                                | Transmit Request Interrupt Mask. Link Port Transmit Request Mask                                                                                                                                                                                                                                                                                                |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                               | Mask Disable Transmit Request interrupt.                                                                                                                                                                                                                                                                                                                        |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                               | Unmask Enable Transmit Request interrupt.                                                                                                                                                                                                                                                                                                                       |

Table 18-7: LP\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                      |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | TRAN       | Transfer Direction. The LP_CTL.TRAN bit selects the transfer direction as transmit (if set) or receive (if cleared) for link buffer. 0 Receive Direction transfer is receive 1 Transmit Direction transfer is transmit                                       |
| 0 (R/W)            | EN         | Enable. The LP_CTL.EN enables or disables the link port. When the processor disables the port ( LP_CTL.EN transitions from high to low), the processor clears the correspond- ing LP_STAT bits. 0 Disable Disable linkport 1 Enable linkport Enable linkport |

## Clock Divider Value Register

The LP\_DIV register selects the divisor for ratio between the internal LP clock (LCLK) and system clock (SCLK). This programming is applicable only for the transmitter. The receiver can operate at any asynchronous frequency up to the maximum frequency independent of the ratio programmed.

Figure 18-10: LP\_DIV Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000009_f66922c4217637698ac1f29616b1a6d453c63fa89ac812213dca0ef31e4229ad.png)

Table 18-8: LP\_DIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Divisor Value. The LP_DIV.VALUE bits select the clock divider (relating the LP' internally gener- ated clock (LCLK) to the system clock (SCLK). The LP_DIV.VALUE should be programmed prior to LP enable. For LP_DIV.VALUE = 0, LCLK = SCLK For LP_DIV.VALUE = xxxxxxxx, LCLK = SCLK / (2 x DIV) |

## Receive Buffer Register

The LP\_RX register buffers the receive data flow through the LP. The receive buffer is a four-location deep FIFO. In the receive buffer, data is transferred to the core or DMA from the receive FIFO where an internal register does the packing. This packing register is not software accessible. For more information on LP buffer features and operations, see the LP functional description.

Figure 18-11: LP\_RX Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000010_620f1dad3cb356b4121671e11326910d80a721e38ab3ad1993dba9cf77a297ef.png)

Table 18-9: LP\_RX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | DATA       | Receive Buffer.           |
| (R/W)              |            |                           |

## Status Register

The LP\_STAT register provides status information on link port interrupts, FIFO, buses, and receive/transmit requests.

Figure 18-12: LP\_STAT Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000011_876f0a3277e4a5221167831362a3a5462965e2a48bbeb08d0eacf035924e7850.png)

Table 18-10: LP\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                     | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | LPBS       | Bus Status. The LP_STAT.LPBS bit indicates the LPDAT bus status. LP_STAT.LPBS is kept high if data is being driven by the link port into the LP_D[n] pins.                                                                                                                                                                                                                                                                                                                                  | Bus Status. The LP_STAT.LPBS bit indicates the LPDAT bus status. LP_STAT.LPBS is kept high if data is being driven by the link port into the LP_D[n] pins.                                                                                                                                                                                                                                                                                                                                  |
| 8 (R/NW)           | LPBS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Link port bus is idle                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 8 (R/NW)           | LPBS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Link port bus is busy                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 7 (R/NW)           | LPACK      | Buffer Pack Status. The LP_STAT.LPACK bit indicates packing status. In receive mode, 32-bit data is received in the block size configured in the LP_CTL.MODE bit field. Then, the data is packed to get a single 32-bit data before loading the FIFO. The LP_STAT.LPACK bit is high during this packing process and goes low after packing. In transmit mode, 32-bit data in the FIFO is unpacked to the appropriate data width before sending. The LP_STAT.LPACK is high during unpacking. | Buffer Pack Status. The LP_STAT.LPACK bit indicates packing status. In receive mode, 32-bit data is received in the block size configured in the LP_CTL.MODE bit field. Then, the data is packed to get a single 32-bit data before loading the FIFO. The LP_STAT.LPACK bit is high during this packing process and goes low after packing. In transmit mode, 32-bit data in the FIFO is unpacked to the appropriate data width before sending. The LP_STAT.LPACK is high during unpacking. |
| 7 (R/NW)           | LPACK      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Packing Complete Packing done                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 7 (R/NW)           | LPACK      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Packing Incomplete Packing is in progress                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 6:4 (R/NW)         | FFST       | FIFO Status. The LP_STAT.FFST bits indicate the FIFO status. These bits are cleared when the LP is disabled.                                                                                                                                                                                                                                                                                                                                                                                | FIFO Status. The LP_STAT.FFST bits indicate the FIFO status. These bits are cleared when the LP is disabled.                                                                                                                                                                                                                                                                                                                                                                                |
| 6:4 (R/NW)         | FFST       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | TX - Empty; RX - Empty Link buffer (TX OR RX) empty                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 18-10: LP\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                       |                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 1                                                                                                                                                                                                                             | TX - Reserved ; RX - Has 1 data word RX has 1 word of data. TX reserved                                                                                                                                                       |
|                    |            | 2                                                                                                                                                                                                                             | TX - Reserved; RX - Has 2 data words RX has 2 word of data. TX reserved.                                                                                                                                                      |
|                    |            | 3                                                                                                                                                                                                                             | TX - Reserved; RX - Has 3 data words RX has 3 word of data. TX reserved.                                                                                                                                                      |
|                    |            | 4                                                                                                                                                                                                                             | TX - One Word; RX -Has 4 data words RX has 4 word of data. TX 1 word of data.                                                                                                                                                 |
|                    |            | 5                                                                                                                                                                                                                             | Reserved                                                                                                                                                                                                                      |
|                    |            | 6                                                                                                                                                                                                                             | TX - FIFO Full; RX - Reserved RX reserved. TX re- served.                                                                                                                                                                     |
|                    |            | 7                                                                                                                                                                                                                             | Reserved                                                                                                                                                                                                                      |
| 3 (R/W1C)          | ROVF       | Receive FIFO Overflow Interrupt. This interrupt is generated when the receiver FIFO overflows. This overflow may happen if the transmitter continues to transmit data even though the receiver has                            | Receive FIFO Overflow Interrupt. This interrupt is generated when the receiver FIFO overflows. This overflow may happen if the transmitter continues to transmit data even though the receiver has                            |
| 1 (R/W1C)          | LRRQ       | Receive Request. The LP generates this interrupt when the LP_CLK pin of a disabled link port (the receiver) is forced high by another link port (the transmitter). Note that the LRRQ interrupt is not supported in DDR mode. | Receive Request. The LP generates this interrupt when the LP_CLK pin of a disabled link port (the receiver) is forced high by another link port (the transmitter). Note that the LRRQ interrupt is not supported in DDR mode. |
| 0 (R/W1C)          | LTRQ       | Transmit Request. The LP generates this interrupt when the LP_ACK pin of a disabled link port (the transmitter) is forced high by another link port (the receiver).                                                           | Transmit Request. The LP generates this interrupt when the LP_ACK pin of a disabled link port (the transmitter) is forced high by another link port (the receiver).                                                           |

## Transmit Buffer Register

The LP\_TX register buffers the transmit data flow through the LP. The transmit buffer is two words deep. In the transmit buffer, the input stage of the FIFO is used to accept core data or DMA data from internal memory, and the data is transferred to the link port interface from the output stage of the FIFO. The output stage performs the unpacking in the transmit buffer. The least significant byte is transmitted first. As each word is unpacked and transmitted, the next location in FIFO becomes available and a new DMA request is made if DMA is enabled. If the register becomes empty, the LP asserts the LP\_CLK signal. For more information on LP buffer features and operations, see the LP functional description.

Figure 18-13: LP\_TX Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000012_d8cf287b13998e70df59e30f5bdb89acfa5f576f23c1916afdb547ec0117dec5.png)

Table 18-11: LP\_TX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | DATA       | Transmit Data Buffer.     |
| (R/W)              |            |                           |

## Shadow Input Transmit Buffer Register

The LP\_TXIN\_SHDW register contains the same data as the input stage of the transmit buffer. Read of this shadow transmit buffer does not update the LP\_STAT register.

Figure 18-14: LP\_TXIN\_SHDW Register Diagram

![Image](21_Link_Port_(LP)_artifacts/image_000013_e256b81bc3cc3ab125012b7d511a6cd08c0c2aabea984717165efd630ca4aa3b.png)

Table 18-12: LP\_TXIN\_SHDW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | DATA       | Transmit Data Buffer Shadow Input Stage. |
| (R/NW)             |            |                                          |

## Shadow Output Transmit Buffer Register

The LP\_TXOUT\_SHDW register contains the same data as the output stage of the transmit buffer. Read of this shadow transmit buffer does not update the LP\_STAT register.

![Image](21_Link_Port_(LP)_artifacts/image_000014_35f36a5874269065d69ab1a5e35f7275bbf1bb8c4d8f5cd9a4f6b0ce915a589a.png)

Transmit Data Output Shadow Register

Figure 18-15: LP\_TXOUT\_SHDW Register Diagram

Table 18-13: LP\_TXOUT\_SHDW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 31:0               | DATA       | Transmit Data Output Shadow Register. |
| (R/NW)             |            |                                       |