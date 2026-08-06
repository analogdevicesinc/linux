# Link Port (LP)

<!-- source: 021_Link_Port_LP.pdf | original pages 1141–1166 -->

## 19   Link Port (LP)

Link ports allow the processor to connect to other processors or peripheral link ports using a simple communication protocol for high-speed parallel data transfer. This peripheral allows various I/O peripheral interconnection schemes to I/O peripheral devices as well as co-processing and multiprocessing schemes.

The link ports of the processor support 8-bit wide data transfers. The link port pins are multiplexed in the GPIO ports. For information on processor multiplexing, see the data sheet for the specific processor.

Link ports can operate independently and simultaneously, allowing glueless high-speed connectivity of up to four external processors.

NOTE: Reference to CLKO8 in this chapter can be considered as OCLK\_0.

## LP Features

All link ports are identical in their design and have the following common features.

- Bidirectional ports with eight data signals ( LP\_D0 -LP\_D7 ), an acknowledge signal ( LP\_ACK ), and a clock signal ( LP\_CLK ).
- Provide high-speed, point-to-point data transfers to other processors, allowing different types of interconnections between multiple processors.
- Pack data into 32-bit words. The processor can directly read this data or transfer it through DMA to or from on-chip memory.
- Support for data buffering through a 2-deep FIFO for transmit and a 4-deep FIFO for receive.
- Programmable clock and acknowledge based handshake mechanism for efficient communication.
- A dedicated DMA channel.
- Support for data transfer in 2-bit mode (DDR/non-DDR only) and 4-bit mode (DDR/non-DDR)

## LP Functional Description

This section provides a description of the link port, including a list of its registers and a functional block diagram.

## ADSP-SC59x LP Register List

The Link Port LP is an 8-bit wide parallel port that can connect to another processor's LP or another LP-compatible device. This port allows a variety of interconnection schemes to I/O peripheral devices as well as co-processing and multiprocessing schemes. A set of registers governs LP operations. For more information on LP functionality, see the LP register descriptions.

Table 19-1: ADSP-SC59x LP Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| LP_CTL        | Control Register                       |
| LP_DIV        | Clock Divider Value Register           |
| LP_RX         | Receive Buffer Register                |
| LP_STAT       | Status Register                        |
| LP_TX         | Transmit Buffer Register               |
| LP_TXIN_SHDW  | Shadow Input Transmit Buffer Register  |
| LP_TXOUT_SHDW | Shadow Output Transmit Buffer Register |

## ADSP-SC59x LP Interrupt List

Table 19-2: ADSP-SC59x LP Interrupt List

|   Interrupt ID | Name        | Description       | Sensitivity   |   DMA Channel |
|----------------|-------------|-------------------|---------------|---------------|
|            117 | LP0_DMA     | LP0 DMAChannel    |               |            30 |
|            118 | LP0_STAT    | LP0 Status        |               |               |
|            119 | LP1_DMA     | LP1 DMAChannel    |               |            36 |
|            120 | LP1_STAT    | LP1 Status        |               |               |
|            277 | LP0_DMA_ERR | LP0 DMAData Error |               |               |
|            278 | LP1_DMA_ERR | LP1 DMAData Error |               |               |

## ADSP-SC59x LP Trigger List

Table 19-3: ADSP-SC59x LP Trigger List Generators

|   Trigger ID | Name    | Description    | Sensitivity   |
|--------------|---------|----------------|---------------|
|           43 | LP0_DMA | LP0 DMAChannel |               |
|           44 | LP1_DMA | LP1 DMAChannel |               |

Table 19-4: ADSP-SC59x LP Trigger List Receivers

|   Trigger ID | Name    | Description    | Sensitivity   |
|--------------|---------|----------------|---------------|
|           31 | LP0_DMA | LP0 DMAChannel | Pulse         |
|           32 | LP1_DMA | LP1 DMAChannel | Pulse         |

## ADSP-SC59x LP DMA Channel List

Table 19-5: ADSP-SC59x LP DMA Channel List

| DMAID   | DMAChannel Name   | Description    |
|---------|-------------------|----------------|
| DMA30   | LP0_DMA           | LP0 DMAChannel |
| DMA36   | LP1_DMA           | LP1 DMAChannel |

## Block Diagram

The Link Port Block Diagram shows the block diagram of a link port.

Figure 19-1: Link Port Block Diagram

<!-- image -->

## External Connections

As shown in the Link Port Pin Connections figure, a link port has eight data lines ( LP\_D0 -LP\_D7 ), a clock line ( LP\_CLK ), and an acknowledge line ( LP\_ACK ). A link port can act as either a transmitter or a receiver but not both at the same time.

X DENOTES THE LINK PORT NUMBER, 0-1

<!-- image -->

Figure 19-2: Link Port Pin Connections

Use external pull-downs for the LP\_CLK and LP\_ACK pins so that the link port can enable the transmitter and receiver, irrespective of the state of the other.

## Internal Blocks

As shown in the block diagram, the link ports have independent modules for transmit and receive. If enabled as transmitter, the link port uses a 2 deep 32-bit FIFO. If enabled as a receiver, the port uses a 4 deep 32-bit FIFO. The core MMR access bus interfaces with these FIFOs. The distributed DMA engines (DDE) use the system cross bar (SCB) interface to access the FIFO. The link port uses the LP\_CTL.TRAN bit to determine whether the module is enabled for transmit or receive operation.

## Architectural Concepts

The following sections describe the architectural concepts of the link port.

- Link Port Protocol
- FIFO Buffers
- Handshake for Link Port Enable Process
- Clocking
- Multi-Processor Connectivity

## Link Port Protocol (SC59x)

A link port transmitted word consists of 4 bytes and the communication proceeds as follows.

1. The transmitter asserts the link port clock ( LP\_CLK ) with each byte of data. The receiver uses the falling edge of LP\_CLK driven by the transmitter to latch the byte.
2. When the receiver is ready to accept another word in the receive buffer it asserts the acknowledge signal, LP\_ACK .

3. The transmitter samples LP\_ACK driven by the receiver at the beginning of each word transmission. If LP\_ACK is deasserted, then the transmitter does not transmit the next word.
4. The transmitter leaves LP\_CLK high and continues to drive the first byte of the next word until LP\_ACK is asserted.
5. When this assertion occurs, the transmitter drives LP\_CLK low. The transmission of the next word starts. If the transmit buffer is empty, LP\_CLK remains low until the buffer refills, regardless of the state of LP\_ACK .

The LP\_ACK signal can deassert when it anticipates that the buffer could fill. The receiver reasserts the LP\_ACK signal as soon as the internal DMA grant signal has occurred or the core reads the receive buffer. Either of these actions frees a buffer location.

NOTE: The LP\_ACK signal inhibits transmission of the next word and not of the current byte.

The LP\_ACK signal provides a handshake between the receiver and transmitter in the following configurations.

- When configured as a transmitter, the port drives both the data and the clock while LP\_ACK is three-stated. In this mode, LP\_CLK is always synchronous with OCLK\_0.
- When configured as a receiver, the link port drives the acknowledge signal and the data and clock lines are three-stated. In this case, the external LP\_CLK signal can either be synchronous or asynchronous with OCLK\_0.
- When the link port is disabled, the data, clock, and acknowledge signals are three-stated.

Figure 19-3: Link Port Communication and Handshake Waveform

<!-- image -->

The following list describes the stages shown in the Link Port Communication and Handshake Waveform figure.

1. LP\_CLK stays high at byte 0 when LP\_ACK is sampled low on the previous LP\_CLK rising edge. LP\_CLK high indicates a stall.
2. The LP\_ACK signal can deassert after byte 0.

3. The LP\_ACK signal reasserts as soon as the link buffer is not full (depending on Rx FIFO conditions).
4. The transmitter samples LP\_ACK to determine whether to transmit the next word.
5. The receiver accepts the remaining word even if LP\_ACK is deasserted. The transmitter does not send the following word.
6. Transmission of data for next word is held until LP\_ACK is asserted.

The transmitter samples the LP\_ACK signal. If the signal is high, the transmitter gives out the falling edges of LP\_CLK for data sampling. The LP\_ACK signal is first sampled at the rising edge of OCLK\_0. One more OCLK\_0 stage synchronizes the signal further. This synchronized signal is given to the subsequent logic. The LP\_CLK falling edge is aligned with OCLK\_0 falling edge in a 1:1 clock ratio mode and with the OCLK\_0 rising edge for the rest of the clock ratios. The following figures explain how the synchronization is maintained between the LP\_ACK and LP\_CLK signals.

In the following figure, synchronizing time is guaranteed to be 1.5 OCLK\_0 cycles.

Figure 19-4: LP\_ACK Synchronization for OCLK\_0: LP\_CLK =1:1

<!-- image -->

In the following figure, synchronizing time is guaranteed to be 2 OCLK\_0 cycles.

Figure 19-5: LP\_ACK Synchronization for OCLK\_0 LP\_CLK =1:2, 1:4 and Up

<!-- image -->

The link port uses the value programmed in the LP\_DIV register at the transmitter to determine the frequency of the link port clock ( LP\_CLK ). However, the signal appearing on the LP\_CLK pin is also dependent on the status of the LP\_ACK pin driven by the receiver. The Relationship Between Internal Link Port Clock and Link Port Clock at the Pins figure shows this relationship.

Figure 19-6: Relationship Between Internal Link Port Clock and Link Port Clock at the Pins

<!-- image -->

## FIFO Buffers

When a link port is configured as transmitter, the link port uses a 2-deep FIFO buffer. A shift register unpacks the single 32-bit word to four 8-bit data bytes. As the FIFO has space for more data, the link port makes a new DMA request. If the FIFO becomes empty, the LP\_CLK signal is deasserted. The core can access FIFO through the LP\_TX register.

The core or DMA makes three writes (2-stage FIFO and 1 shift register) to the transmit buffer before it signals a full condition. The link port uses the LP\_STAT.FFST bit field to reflect the status of the FIFO but not the shift register full or empty condition. However, the program can poll the LP\_STAT.LPBS bit to discover whether the link

port is driving data from the shift register to the pins. The LP\_STAT.LPBS bit is also set when receiver has held off transmission by driving LP\_ACK low.

- NOTE: When the 2-deep FIFO and the output shift-register overflow, any further write to the link port buffer overwrites the input stage of the FIFO.
- NOTE: The core can also read the transmit FIFO through the data LP\_TX register.
- NOTE: If the transmitter is disabled while performing writes to the transmit FIFO, a FIFO full condition is signaled after two writes.

The transmit buffer registers have shadow registers. Using these shadow registers, both stages of the 2-deep FIFO can be read without updating the status registers. The LP\_TXIN\_SHDW register corresponds to the input stage of the FIFO. The LP\_TXOUT\_SHDW register corresponds to the output stage of the FIFO as shown in the Transmit FIFO path figure.

Figure 19-7: Transmit FIFO path

<!-- image -->

When a link port is configured as receiver, data transfers to the core or DMA from the full 4-deep receive FIFO. An internal packing register packs data to 32 bits. Four reads can occur from the receive buffer by the core or DMA before it signals an empty condition. The link port uses the LP\_STAT.FFST bits to reflect the status of the 4-deep read buffer FIFO. The core can access this FIFO through the LP\_RX register.

- NOTE: When receive FIFO overflows ( LP\_STAT.ROVF bit=1), any further data from the transmitter is lost. Only the data retained in the receive FIFO can be retrieved further.

The receiver drives the LP\_ACK output signal low, after the first byte of data for the next-to-last empty slot (in the 4-deep FIFO) is received. This functionality prevents data loss due to the transmitter starting transmission of the next word before the LP\_ACK signal reaches the transmitter. (The timing is due to the larger delay in synchronization.) This functionality guarantees that even after allowing for the extra synchronization cycle in the transmitter and receiver, there is no overflow in the receive FIFO. The LACK Generation Based on Receive FIFO Status figure

shows how FIFO slots influence the acknowledge signal generation. The grayed sections show received data. The white sections show empty locations where the decision to pull LP\_ACK high is taken.

Figure 19-8: LACK Generation Based on Receive FIFO Status

<!-- image -->

| 32-BIT   | 32-BIT   |
|----------|----------|
| 32-BIT   | 32-BIT   |
| 8-BIT    | 24-BIT   |
| 32-BIT   | 32-BIT   |

NOTE: The link port uses a 4-deep receive FIFO only under a worst case situation, as mentioned. In all other cases, respond as if the FIFO has only a 3-deep stage. The LP\_ACK signal is pulled high before the last stage of the FIFO.

The link port has memory-mapped buffers for both receive and transmit operations. A JTAG-based emulator can read the FIFO which can cause unexpected problems in data transfers. This activity can only happen during an emulation event (typically hitting a breakpoint or single-stepping). The emulator issues core reads through JTAG. To work around this issue, see the tools documentation for more information.

## Handshake for Link Port Enable Process

In a link port-based system, the transmitter and the receiver can be enabled at different times. Use external pulldowns for the LP\_CLK and LP\_ACK signals.

If the receiver is enabled before the transmitter, the external pull-down holds the LP\_CLK signal of the transmitter low. The receiver is held off. The receiver can wait for a rising edge on the LP\_CLK signal to assert its receive service request interrupt. This rising edge occurs only when transmitter starts driving the first data on to the bus, after the application enables it.

If the transmitter is enabled before the receiver, the external pull-down holds the LP\_ACK signal of the receiver low. Transmission is held off. Refer to the Enable the Transmitter Before the Receiver figure. The transmitter can wait for a rising edge on the LP\_ACK signal to assert its transmit service request interrupt. This rising edge is asserted as soon as the receiver is enabled after the hardware drives the LP\_ACK high.

Figure 19-9: Enable the Transmitter Before the Receiver

<!-- image -->

NOTE: Service request interrupts or status are asserted only when the link port (receiver or transmitter) is disabled.

## Clocking

The link port clock ( LP\_CLK ) is derived from the internal system clock (OCLK\_0). The link port clock to system clock ratio can be configured in the LP\_DIV register. This value applies to the transmitter only. The receiver can operate at any asynchronous frequency up to the maximum frequency, independent of the ratio programmed. The following formula describes the relationship between the frequency of the link port clock, the OCLK\_0 frequency, and the LP\_DIV value.

- f LP\_CLK = f OCLK\_0 &lt; or = f LP\_CLK-MAX  if DIV = 0
- f LP\_CLK = f OCLK\_0 /(2 × DIV) if DIV &gt; 0

Where: f LP\_CLK  = link clock frequency, f LP\_CLK-MAX  = link clock maximum frequency, and f OCLK0\_0  = system clock frequency.

While programming the LP\_DIV register to select the clock ratio, ensure that the LP\_CLK frequency does not exceed the maximum frequency supported for the device. For supported frequencies, see the product-specific data sheet.

NOTE: The Drive Strength (DS) value must be changed to b(010) for the LPCLK pin to operate the link port at the maximum frequency of 125 MHz. Refer the PADS\_PORTB0\_DS and PADS\_PORTC0\_DS register bit descriptions for details.

## Multi-Processor Connectivity

Link ports can operate independently, allowing glueless connection with external processors. Link ports have dedicated DMA channels, allowing independent data transfers. The following group of figures shows some examples of different bus connection topology that can be used in multi-processor system design. The inter-connection methods are not limited to these examples.

Figure 19-10: Central Processor-Based Model

Figure 19-12: Link Port Ring Model

The link port protocol does not include built-in support for multiple requesters. However, there can be situations where multiple devices try to become the bus requester at the same time. Multi-requester conflicts can be resolved using token passing. In token passing, the token is a software flag that passes between processors.

At reset, the token is set to reside in the link port of one device, making it the requester and the transmitter. When a receiver (completer) wants to become the requester, it can assert its LP\_ACK signal to get the attention of the requester. The requester knows, through the software protocol, whether to respond with actual data or whether the token is requested. If the requester wishes to give up the token, it can send back a user-defined token release word and thereafter clear its token flag.

Simultaneously, the completer sets its token and can thereafter transmit. The token release word can be any userdefined value. Because the transmitter and receiver expect a code word, this word does not need to be exclusive of

CENTRAL

PROCESSOR

PROCESSOR 1

Figure 19-11: Link Port Full-Duplex Transfer Model

<!-- image -->

LP0

LP1

LP2

LP3

LPx

LPy

LPx

LPy

PROCESSOR 1

PROCESSOR 2

PROCESSOR 3

PROCESSOR 4

PROCESSOR 2

normal data transmission. If the requester wishes to give up the token, it can send back a user-defined token release word and thereafter clear its token flag. Simultaneously, the completer examines the data sent back and if it is the token release word, the completer sets its token, and can thereafter transmit.

The link port protocol includes handshake mechanism to inform the other end of transfer (transmit or receive) of an enable instance. However, it does not support handshakes to inform a disabled instance, while a chunk of data transfers. The application must assume the disabled state of the other end and take appropriate action.

For example, in a multi-processing environment, a receiver did not read its full FIFO for an extended time due to internal bus arbitrations. The transmitter can require software or a peripheral timer-based timeout to inform the application that the LP\_ACK signal is low for an extended period.

## LP Operating Modes

The link port does not have particular modes of operation, as the peripheral is based on a simple protocol. The following sections explain the data transfer modes, using the core and using DMA.

- Core Data Transfers
- DMA Data Transfers

## LP Data Transfer Modes

This section describes link port DMA and core data transfers.

## Core Data Transfers

If DMA is disabled for a link port buffer, the processor core can write or read internal FIFO buffers as a memorymapped register through the MMR access bus. To avoid FIFO overflow or underflow, the core can access the FIFO registers in one of the following ways.

- Access link port registers using an interrupt service routine (ISR) mapped to the data request interrupt of the link port. The interrupt request remains high only when the FIFO is accessible (when the FIFO is not full in transmit mode and not empty in receive mode).
- Poll the FIFO status bits of the LP\_STAT register. Write to the transmit FIFO if not full or read from the receive FIFO if not empty.

## DMA Data Transfers

Dedicated DMA channels are available for each link port. DMA-related activity is explained in the following steps.

1. Data Receive - Once the DMA channel and link port module are configured and enabled, the external device begins writing data to the FIFO through the data pins of the link port. The FIFO detects this activity and in turn sends a DMA request. After the request is granted, the DMA transfer progresses until the FIFO is empty.
2. Data Transmit - Once the DMA channel and link port module are configured and enabled, setting the LP\_CTL.EN bit automatically asserts a DMA request when the transmit FIFO is empty. After the request is granted, DMA fills the FIFO. The external device begins reading data from the FIFO through the data pins of

the link port. The FIFO detects that there is room in the buffer and asserts another DMA request, continuing the process.

## LP Event Control

This section describes how the link port uses interrupts and status signals.

## Interrupt Signals

Each link port has two dedicated interrupt lines registered with the system event controller-a data request interrupt and a status interrupt. Data request interrupts are asserted based on FIFO conditions for data transfer. Status interrupts are asserted when a service request status or an overflow status is set. The following list explains each of these interrupts.

- Data Request Interrupt. Asserted if the FIFO is not full in transmission mode and the FIFO is not empty in reception mode. This functionality serves as a core triggered interrupt in non-DMA mode and as the DMA interrupt request in DMA mode. Generation of this interrupt is based on the LP\_STAT.FFST (status bit of the link port buffer).
- Link Port Transmit Service Request Interrupt (LTRQ) . Allow a disabled link port to generate an interrupt when an external access is attempted. When a link port is configured as transmitter, the transmit service request interrupt is enabled by setting the LP\_CTL.TRQMSK bit. When set, an external receiver can indicate to the disabled transmitter that it must receive data through the connected link port. The receiver does so by driving a high level on the LP\_ACK line. When the LP\_ACK of the disabled transmitter link port is detected high, a LP\_STAT.LTRQ interrupt is generated. The transmitter can enable itself for data transfer with the receiver. The link port needs a pull-down on LP\_ACK for this feature to function properly.
- Link Port Receive Service Request Interrupt (LRRQ) . When a link port is configured as receiver, this interrupt is enabled by setting the LP\_CTL.RRQMSK bit. When set, an external transmitter can indicate to the disabled receiver that it must receive data through the connected link port. The transmitter does so by driving out the first data. When the LP\_CLK of the disabled receiver link port is detected high, a LP\_STAT.LRRQ interrupt is generated. The receiver can further enable itself for data transfer with the transmitter. The link port needs a pull-down on the LP\_CLK signal for this feature to function properly.
- Link Port Receive Overflow Interrupt (LPOVF) . Generated when the receiver FIFO overflows and is enabled by setting the LP\_CTL.ROVFMSK bit. This interrupt can happen if the transmitter continues to transmit data even though the receiver has deasserted LP\_ACK signal causing the receive FIFO to overflow.

## Enabling Link Port Interrupts

A data request interrupt is fed to the system event controller directly and can be controlled separately from the application.

To mask the interrupt, set the mask bits in LP\_CTL register corresponding to service interrupts and the overflow interrupt. These interrupts are OR'ed and fed to the SIC as a single LP\_STAT interrupt. These interrupts are latched and stored in the associated bits of LP\_STAT register. If an LP\_STAT interrupt occurs, in the ISR, programs

can read the LP\_STAT register bits to determine the type of interrupt. These bits are write-one-to-clear (W1C); writing one to the bit resets the bit and disables the corresponding interrupt.

## Status and Error Signals

This section explains the various status signals in the LP\_STAT register.

- Transfer Status signals . The link port uses the bus status bit ( LP\_STAT.LPBS ) to give the status of the bus condition (busy or idle) when the link port is configured as transmitter. The LP\_STAT.LPBS is high if the link port drives data into the link port pins. Programs can poll this bit after polling the LP\_STAT.FFST bit to disable the link port safely.

The link buffer status ( LP\_STAT.FFST ) field directly indicates the status of the FIFO (including empty or full conditions) during data transfer. Software can poll this field in the LP\_STAT register before writing to the FIFO (in case of transmission) or reading from the FIFO (in case of reception). The LP\_STAT.FFST bit is automatically cleared when the link port is disabled.

- Transfer Request Status signals . The link port uses the receive request status ( LP\_STAT.LRRQ ) bit to indicate that an external receiver wants to receive data (in case the link port is a disabled transmitter). The link port uses the transmit request status ( LP\_STAT.LTRQ ) bit to indicate that an external transmitter wants to send data (in case the link port is a disabled receiver). Software can poll these bits to enable the transmitter or receiver accordingly.
- Error Status signals . In receive mode 32-bit data is received in four chunks of 8-bit data. This data is then packed to a single 32-bit data before loading the FIFO. The link buffer error status ( LP\_STAT.LPACK ) bit is high during this packing process and goes low after packing.

The link port overflow status ( LP\_STAT.ROVF ) bit is set when the receive FIFO overflows. This event can occur if the transmitter continues to transmit data even though the receiver has deasserted LP\_ACK causing the receiver FIFO to overflow.

## LP Programming Model

The following sections provide information on configuring the operating mode and enabling the link ports.

- Setting Up a DMA Transmit Operation
- Setting Up a DMA Receive Operation
- Setting Up a Core Transmit Operation
- Setting Up a Core Receive Operation

## Setting Up a DMA Transmit Operation

This following procedure describes the typical steps for configuring the link ports in DMA transmit mode.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.

2. Configure the drive strength value to b(010) for the link port clock pins to operate the link port at maximum frequency ( PADS\_PORTB0\_DS and PADS\_PORTC0\_DS registers).
3. Install interrupt handlers for DMA and for transfer status (service request interrupt).
4. Configure the link port to transmit by setting the LP\_CTL bit and enable the transmit request interrupt mask by setting the LP\_CTL.TRQMSK bit.
5. Program the link port clock divider by writing a value to the LP\_DIV register.
6. If using DMA stop mode or auto buffer mode, program the appropriate DMA registers. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Stop/Auto, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation and memory read).
7. Wait for the link port receiver (connected externally) to be enabled. The application can wait for the transmit service request interrupt to assert.
8. Clear the transmit service request interrupt status by writing 1 to the LP\_STAT.LTRQ bit.
9. Enable DMA by setting the DMA\_CFG.EN bit.
10. Enable the link port by setting the LP\_CTL.EN bit.
11. Wait for DMA to assert a transfer completion interrupt.
12. Clear the DMA interrupt source by writing 1 to the DMA\_STAT.IRQDONE bit.

## Setting Up a DMA Receive Operation

This section describes the typical steps for using the link ports in DMA receive mode.

1. Enable the link port pins in GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for DMA and for transfer status (service request interrupt).
3. Configure the link port for reception (clear the LP\_CTL.TRAN bit) and enable the receive request interrupt mask by setting the LP\_CTL.RRQMSK bit.
4. If using DMA stop mode or auto buffer mode, program the DMA registers. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Stop/Auto, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation and memory write).
5. If using DMA array mode or list mode, create DMA configuration data structures filled with components. ADDITIONAL INFORMATION: An example configuration is: DMA\_ADDRSTART , DMA\_XCNT , DMA\_XMOD , and DMA\_CFG registers (Array/List, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, interrupt generation, memory write and fetch =4/5) and DMA\_DSCPTR\_NXT register (if list mode). Further, program

DMA configuration register (Array/List, DMA\_CFG.PSIZE =1, DMA\_CFG.MSIZE =4, Memory Write and Fetch =4/5) and program the DMA\_DSCPTR\_NXT register (if list mode).

6. Wait for the link port transmitter (connected externally) to be enabled with subsequent data transmission. The application can wait for the receive service request interrupt to assert.
7. Clear the receive service request interrupt status by writing 1 to the LP\_STAT.LRRQ bit.
8. Enable DMA by setting the DMA\_CFG.EN bit.
9. Enable the link port by setting the LP\_CTL.EN bit.
10. Wait for DMA to assert the transfer complete interrupt.
11. Clear the DMA interrupt source by writing 1 to the DMA\_STAT.IRQDONE bit of the DMA status register.

## Setting Up a Core Transmit Operation

This section describes the typical steps for using the link ports in processor core based transmission.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Configure the drive strength value to b(010) for the link port clock pins to operate the link port at maximum frequency ( PADS\_PORTB0\_DS and PADS\_PORTC0\_DS registers).
3. Install interrupt handlers for data transfer and for transfer status (service request interrupt). The interrupt handlers for data transfer are the same source or ID as the DMA interrupt line in the SEC.
4. Configure the link port for transmission by setting the LP\_CTL.TRAN bit) and enable the transmit request interrupt mask by setting the LP\_CTL.TRQMSK bit).
5. Program the link port clock divider by writing a value in to the LP\_DIV register.
6. Wait for the link port receiver (connected externally) to be enabled. The application can wait for a transmit service request interrupt to assert.
7. Clear the transmit service request interrupt status by writing 1 to the LP\_STAT.LTRQ bit.
8. Enable the link port by setting the LP\_CTL.EN bit.
9. The data request interrupt is asserted whenever there is free space in the FIFO. The application can write to the LP\_TX register based on the FIFO conditions (half or empty) reflected in the LP\_STAT.FFST bit field.

## Setting Up a Core Receive Operation

This section describes the typical steps for using the link ports in processor core-based reception.

1. Enable the link port pins in the GPIO port mux using the appropriate PORT\_FER and PORT\_MUX registers.
2. Install interrupt handlers for data transfer and for transfer status (service request interrupt). The interrupt handlers for data transfer are the same source or ID as the DMA interrupt line in the SEC).

3. Configure link port for reception (clear LP\_CTL.TRAN bit). Enable the receive request interrupt mask bit (set LP\_CTL.RRQMSK ).
4. Wait for the link port transmit (connected externally) to be enabled with subsequent transmission of data. The application can wait for receive service request interrupt to be asserted.
5. Clear the receive service request interrupt status by writing 1 to the LP\_STAT.LRRQ bit.
6. Enable the link port by setting the LP\_CTL.EN bit.
7. The data request interrupt is asserted whenever there is free space in the FIFO. The application can read from the LP\_RX register based on the FIFO conditions (1 or 2 or 3 data available) which is reflected in the LP\_STAT.FFST bit field.

## ADSP-SC59x LP Register Descriptions

Link Port (LP) contains the following registers.

Table 19-6: ADSP-SC59x LP Register List

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

Figure 19-13: LP\_CTL Register Diagram

<!-- image -->

Table 19-7: LP\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/W)        | MODE       | Mode Select. The LP_CTL.MODE field indicates the mode of operation: Single Data Rates (SDR) of 2 bit and 4 bit, Double Data Rates (DDR) of 2 bit, 4 bit, and 8 bit. Note that the LRRQ interrupt is not supported in DDR mode. The LP receiver must be enabled by the user before starting the LP transmitter. For LP DDR mode, the val- ue of LP DDR clock should be twice that of LP clock. CDU CLKO9 is used for the LP DDR clock. | Mode Select. The LP_CTL.MODE field indicates the mode of operation: Single Data Rates (SDR) of 2 bit and 4 bit, Double Data Rates (DDR) of 2 bit, 4 bit, and 8 bit. Note that the LRRQ interrupt is not supported in DDR mode. The LP receiver must be enabled by the user before starting the LP transmitter. For LP DDR mode, the val- ue of LP DDR clock should be twice that of LP clock. CDU CLKO9 is used for the LP DDR clock. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                     | 8 bit SDR. The default mode. (D0, D1, D2, D3, D4, D5, D6 and D7 data pins used)                                                                                                                                                                                                                                                                                                                                                       |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                     | 4 bit SDR. (D0, D1, D2 and D3 data pins are used)                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                     | 2 bit SDR. (D0 and D1 data pins are used)                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                     | 8 bit DDR. (D0, D1, D2, D3, D4, D5, D6 and D7 da- ta pins are used)                                                                                                                                                                                                                                                                                                                                                                   |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                                     | 4 bit DDR. (D0, D1, D2 and D3 data pins are used)                                                                                                                                                                                                                                                                                                                                                                                     |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                                     | 2 bit DDR. (D0 and D1 data pins are used)                                                                                                                                                                                                                                                                                                                                                                                             |
| 11                 | ROVFMSK    | Receive FIFO Overflow Interrupt Mask.                                                                                                                                                                                                                                                                                                                                                                                                 | Receive FIFO Overflow Interrupt Mask.                                                                                                                                                                                                                                                                                                                                                                                                 |
| (R/W)              |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                     | Mask Disable Receive FIFO Overflow Interrupt                                                                                                                                                                                                                                                                                                                                                                                          |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                     | Unmask Enable Receive FIFO Overflow Interrupt                                                                                                                                                                                                                                                                                                                                                                                         |

Table 19-7: LP\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | RRQMSK     | Receive Request Interrupt Mask. Link Port Receive Request Mask                                                                                                                                                             |
| 8 (R/W)            | TRQMSK     | Transmit Request Interrupt Mask. Link Port Transmit Request Mask                                                                                                                                                           |
| 3 (R/W)            | TRAN       | Transfer Direction. The LP_CTL.TRAN bit selects the transfer direction as transmit (if set) or receive (if cleared) for link buffer.                                                                                       |
| 0 (R/W)            | EN         | Enable. The LP_CTL.EN enables or disables the link port. When the processor disables the port ( LP_CTL.EN transitions from high to low), the processor clears the correspond- ing LP_STAT bits. 0 Disable Disable linkport |

## Clock Divider Value Register

The LP\_DIV register selects the divisor for ratio between the internal LP clock (LCLK) and system clock (CLKO8). This programming is applicable only for the transmitter. The receiver can operate at any asynchronous frequency up to the maximum frequency independent of the ratio programmed.

Figure 19-14: LP\_DIV Register Diagram

<!-- image -->

Table 19-8: LP\_DIV Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                               |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7:0 (R/W)          | VALUE      | Divisor Value. The LP_DIV.VALUE bits select the clock divider (relating the LP' internally generat- ed clock (LCLK) to the system clock (CLKO8). The LP_DIV.VALUE should be pro- grammed prior to LP enable. For LP_DIV.VALUE = 0, LCLK = CLKO8 For LP_DIV.VALUE = xxxxxxxx, LCLK = CLKO8 / (2 x DIV) |

## Receive Buffer Register

The LP\_RX register buffers the receive data flow through the LP. The receive buffer is a four-location deep FIFO. In the receive buffer, data is transferred to the core or DMA from the receive FIFO where an internal register does the packing. This packing register is not software accessible. For more information on LP buffer features and operations, see the LP functional description.

Figure 19-15: LP\_RX Register Diagram

<!-- image -->

Table 19-9: LP\_RX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | DATA       | Receive Buffer.           |
| (R/W)              |            |                           |

## Status Register

The LP\_STAT register provides status information on link port interrupts, FIFO, buses, and receive/transmit requests.

Figure 19-16: LP\_STAT Register Diagram

<!-- image -->

Table 19-10: LP\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | LPBS       | Bus Status. The LP_STAT.LPBS bit indicates the LPDAT bus status. LP_STAT.LPBS is kept high if data is being driven by the link port into the LP_D[n] pins.                                                                                                                                                                                                                                                                                              | Bus Status. The LP_STAT.LPBS bit indicates the LPDAT bus status. LP_STAT.LPBS is kept high if data is being driven by the link port into the LP_D[n] pins.                                                                                                                                                                                                                                                                                              |
| 8 (R/NW)           | LPBS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Link port bus is idle                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 8 (R/NW)           | LPBS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Link port bus is busy                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 7 (R/NW)           | LPACK      | Buffer Pack Status. The LP_STAT.LPACK bit indicates packing status. In receive mode, 32-bit data is received in 4 blocks of 8-bit data. Then, the data is packed to get a single 32-bit data before loading the FIFO. The LP_STAT.LPACK bit is high during this packing process and goes low after packing. In transmit mode, 32-bit data in the FIFO is unpacked to 4 blocks of 8-bit data before sending. The LP_STAT.LPACK is high during unpacking. | Buffer Pack Status. The LP_STAT.LPACK bit indicates packing status. In receive mode, 32-bit data is received in 4 blocks of 8-bit data. Then, the data is packed to get a single 32-bit data before loading the FIFO. The LP_STAT.LPACK bit is high during this packing process and goes low after packing. In transmit mode, 32-bit data in the FIFO is unpacked to 4 blocks of 8-bit data before sending. The LP_STAT.LPACK is high during unpacking. |
| 7 (R/NW)           | LPACK      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Packing Complete Packing done                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/NW)           | LPACK      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Packing Incomplete Packing is in progress                                                                                                                                                                                                                                                                                                                                                                                                               |
| 6:4 (R/NW)         | FFST       | FIFO Status. The LP_STAT.FFST bits indicate the FIFO status. These bits are cleared when the LP is disabled.                                                                                                                                                                                                                                                                                                                                            | FIFO Status. The LP_STAT.FFST bits indicate the FIFO status. These bits are cleared when the LP is disabled.                                                                                                                                                                                                                                                                                                                                            |
| 6:4 (R/NW)         | FFST       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                       | TX - Empty; RX - Empty Link buffer (TX OR RX) empty                                                                                                                                                                                                                                                                                                                                                                                                     |

Table 19-10: LP\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                         |                                                                                                                                                                                                                                 |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
|                    |            | 1                                                                                                                                                                                                                               | TX - Reserved ; RX - Has 1 data word RX has 1 word of data. TX reserved                                                                                                                                                         |
|                    |            | 2                                                                                                                                                                                                                               | TX - Reserved; RX - Has 2 data words RX has 2 word of data. TX reserved.                                                                                                                                                        |
|                    |            | 3                                                                                                                                                                                                                               | TX - Reserved; RX - Has 3 data words RX has 3 word of data. TX reserved.                                                                                                                                                        |
|                    |            | 4                                                                                                                                                                                                                               | TX - One Word; RX -Has 4 data words RX has 4 word of data. TX 1 word of data.                                                                                                                                                   |
|                    |            | 5                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                        |
|                    |            | 6                                                                                                                                                                                                                               | TX - FIFO Full; RX - Reserved RX reserved. TX re- served.                                                                                                                                                                       |
|                    |            | 7                                                                                                                                                                                                                               | Reserved                                                                                                                                                                                                                        |
| 3 (R/W1C)          | ROVF       | Receive FIFO Overflow Interrupt. This interrupt is generated when the receiver FIFO overflows. This overflow may hap-                                                                                                           | Receive FIFO Overflow Interrupt. This interrupt is generated when the receiver FIFO overflows. This overflow may hap-                                                                                                           |
| 1 (R/W1C)          | LRRQ       | Receive Request. The LP generates this interrupt when the LP_CLK pin of a disabled link port (the re- ceiver) is forced high by another link port (the transmitter). Note that the LRRQ interrupt is not supported in DDR mode. | Receive Request. The LP generates this interrupt when the LP_CLK pin of a disabled link port (the re- ceiver) is forced high by another link port (the transmitter). Note that the LRRQ interrupt is not supported in DDR mode. |
| 0 (R/W1C)          | LTRQ       | Transmit Request. The LP generates this interrupt when the LP_ACK pin of a disabled link port (the transmitter) is forced high by another link port (the receiver).                                                             | Transmit Request. The LP generates this interrupt when the LP_ACK pin of a disabled link port (the transmitter) is forced high by another link port (the receiver).                                                             |

## Transmit Buffer Register

The LP\_TX register buffers the transmit data flow through the LP. The transmit buffer is two words deep. In the transmit buffer, the input stage of the FIFO is used to accept core data or DMA data from internal memory, and the data is transferred to the link port interface from the output stage of the FIFO. The output stage performs the unpacking in the transmit buffer. The least significant byte is transmitted first. As each word is unpacked and transmitted, the next location in FIFO becomes available and a new DMA request is made if DMA is enabled. If the register becomes empty, the LP asserts the LP\_CLK signal. For more information on LP buffer features and operations, see the LP functional description.

Figure 19-17: LP\_TX Register Diagram

<!-- image -->

Table 19-11: LP\_TX Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 31:0               | DATA       | Transmit Data Buffer.     |
| (R/W)              |            |                           |

## Shadow Input Transmit Buffer Register

The LP\_TXIN\_SHDW register contains the same data as the input stage of the transmit buffer. Read of this shadow transmit buffer does not update the LP\_STAT register.

Figure 19-18: LP\_TXIN\_SHDW Register Diagram

<!-- image -->

Table 19-12: LP\_TXIN\_SHDW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                  |
|--------------------|------------|------------------------------------------|
| 31:0               | DATA       | Transmit Data Buffer Shadow Input Stage. |
| (R/NW)             |            |                                          |

## Shadow Output Transmit Buffer Register

The LP\_TXOUT\_SHDW register contains the same data as the output stage of the transmit buffer. Read of this shadow transmit buffer does not update the LP\_STAT register.

Transmit Data Output Shadow Register

Figure 19-19: LP\_TXOUT\_SHDW Register Diagram

<!-- image -->

Table 19-13: LP\_TXOUT\_SHDW Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration               |
|--------------------|------------|---------------------------------------|
| 31:0               | DATA       | Transmit Data Output Shadow Register. |
| (R/NW)             |            |                                       |