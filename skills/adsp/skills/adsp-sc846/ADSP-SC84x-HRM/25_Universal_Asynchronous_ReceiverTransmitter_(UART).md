## 22   Universal Asynchronous Receiver/Transmitter (UART)

The UART module is a full-duplex peripheral compatible with PC-style industry-standard UARTs. The UART converts data between serial and parallel formats. The serial communication follows an asynchronous protocol that supports various word lengths, stop bits, bit rates, and parity-generation options. Multiple events can generate interrupts.

The UART is logically compliant to EIA-232E, EIA-422, EIA-485 and LIN standards, but usually requires external transceiver devices to meet electrical requirements. In IrDA (Infrared Data Association) mode, the UART meets the half-duplex IrDA SIR (9.6/115.2 Kbps rate) protocol. In multi-drop bus mode, the UART meets the full-duplex MDB/ICP v2.0 protocol.

The UART module supports partial modem status and control functionality to allow for hardware flow control.

The UART is a DMA-capable peripheral with separate transmit and receive DMA controller channels. The use of DMA requires minimal software intervention as the DMA engine moves the data. The UART can also use a programmed core mode of operation. The core mode requires software management of the data flow using either interrupts or polling.

The UART can use one of the peripheral timers for a hardware-assisted auto-baud detection mechanism. The timers are external to the UART.

NOTE: The UARTs (UART0, UART1, and UART2) operate in the SCLK0 domain.

## UART Features

Each UART includes the following features.

- 5-8 data bits
- Programmable extra stop bit and programmable extra half-stop bit
- Even, odd, and sticky parity bit options
- Extra 8-stage receive FIFO with programmable threshold interrupt request
- Flexible transmit and receive interrupt request timing

- Three interrupt outputs for receive, transmit, and status
- Independent DMA operation for receive and transmit
- Programmable automatic request to send (RTS)/clear to send (CTS) hardware flow control
- False start bit detection
- SIR IrDA operation mode
- MDB/ICP v2.0 operation mode
- Internal loopback
- Improved bit rate granularity
- LIN break command/Inter-frame gap transmission support

Table 22-1: UART Specifications

| Feature                  | Availability      |
|--------------------------|-------------------|
| Protocol                 | Protocol          |
| Controller-Capable       | Yes               |
| Target-Capable           | Yes               |
| Transmission Simplex     | Yes               |
| Transmission Half-Duplex | Yes               |
| Transmission Full-Duplex | Yes               |
| Access Type              | Access Type       |
| Data Buffer              | Yes               |
| Core Data Access         | Yes               |
| DMAData Access           | Yes               |
| DMAChannels              | 2 (per UART Port) |
| DMADescriptor            | Yes               |
| Boot Capable             | Yes (Target Mode) |
| Local Memory             | No                |
| Clock Operation          | SCLK/16           |

## UART Functional Description

The following sections provide details on the UARTs functionality.

## ADSP-2184x UART Register List

The Universal Asynchronous Receiver/Transmitter module (UART) is a full-duplex peripheral compatible with PC-style industry-standard UARTs. The UART converts data between serial and parallel formats. The serial communication follows an asynchronous protocol that supports various word length, stop bit, parity, and interrupt generation options. A set of registers governs UART operations. For more information on UART functionality, see the UART register descriptions.

Table 22-2: ADSP-2184x UART Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| UART_CLK      | Clock Rate Register                    |
| UART_CTL      | Control Register                       |
| UART_IMSK     | Interrupt Mask Register                |
| UART_IMSK_CLR | Interrupt Mask Clear Register          |
| UART_IMSK_SET | Interrupt Mask Set Register            |
| UART_RBR      | Receive Buffer Register                |
| UART_RSR      | Receive Shift Register                 |
| UART_RXCNT    | Receive Counter Register               |
| UART_SCR      | Scratch Register                       |
| UART_STAT     | Status Register                        |
| UART_TAIP     | Transmit Address/Insert Pulse Register |
| UART_THR      | Transmit Hold Register                 |
| UART_TSR      | Transmit Shift Register                |
| UART_TXCNT    | Transmit Counter Register              |

## ADSP-2184x UART Interrupt List

Table 22-3: ADSP-2184x UART Interrupt List

|   Interrupt ID | Name        | Description       | Sensitivity   |   DMA Channel |
|----------------|-------------|-------------------|---------------|---------------|
|            367 | UART0_TXDMA | UART0 TransmitDMA | Level         |            20 |
|            368 | UART0_RXDMA | UART0 ReceiveDMA  | Level         |            21 |
|            369 | UART0_STAT  | UART0 Status      | Level         |               |
|            370 | UART1_TXDMA | UART1 TransmitDMA | Level         |            34 |
|            371 | UART1_RXDMA | UART1 ReceiveDMA  | Level         |            35 |
|            372 | UART1_STAT  | UART1 Status      | Level         |               |
|            373 | UART2_TXDMA | UART2 TransmitDMA | Level         |            37 |

Table 22-3: ADSP-2184x UART Interrupt List (Continued)

|   Interrupt ID | Name            | Description             | Sensitivity   |   DMA Channel |
|----------------|-----------------|-------------------------|---------------|---------------|
|            374 | UART2_RXDMA     | UART2 ReceiveDMA        | Level         |            38 |
|            375 | UART2_STAT      | UART2 Status            | Level         |               |
|            376 | UART0_TXDMA_ERR | UART0 Transmit DMAError | Level         |               |
|            377 | UART0_RXDMA_ERR | UART0 Receive DMAError  | Level         |               |
|            378 | UART1_TXDMA_ERR | UART1 Transmit DMAError | Level         |               |
|            379 | UART1_RXDMA_ERR | UART1 Receive DMAError  | Level         |               |
|            380 | UART2_TXDMA_ERR | UART2 Transmit DMAError | Level         |               |
|            381 | UART2_RXDMA_ERR | UART2 Receive DMAError  | Level         |               |

## ADSP-2184x UART Trigger List

Table 22-4: ADSP-2184x UART Trigger List Generators

|   Trigger ID | Name        | Description       | Sensitivity   |
|--------------|-------------|-------------------|---------------|
|          191 | UART0_TXDMA | UART0 TransmitDMA | Edge          |
|          192 | UART0_RXDMA | UART0 ReceiveDMA  | Edge          |
|          193 | UART1_TXDMA | UART1 TransmitDMA | Edge          |
|          194 | UART1_RXDMA | UART1 ReceiveDMA  | Edge          |
|          195 | UART2_TXDMA | UART2 TransmitDMA | Edge          |
|          196 | UART2_RXDMA | UART2 ReceiveDMA  | Edge          |

Table 22-5: ADSP-2184x UART Trigger List Receivers

|   Trigger ID | Name        | Description       | Sensitivity   |
|--------------|-------------|-------------------|---------------|
|          232 | UART0_TXDMA | UART0 TransmitDMA | Pulse         |
|          233 | UART0_RXDMA | UART0 ReceiveDMA  | Pulse         |
|          234 | UART1_TXDMA | UART1 TransmitDMA | Pulse         |
|          235 | UART1_RXDMA | UART1 ReceiveDMA  | Pulse         |
|          236 | UART2_TXDMA | UART2 TransmitDMA | Pulse         |
|          237 | UART2_RXDMA | UART2 ReceiveDMA  | Pulse         |

## ADSP-2184x UART DMA Channel List

Table 22-6: ADSP-2184x UART DMA Channel List

| DMAID   | DMAChannel Name   | Description       |
|---------|-------------------|-------------------|
| DMA20   | UART0_TXDMA       | UART0 TransmitDMA |
| DMA21   | UART0_RXDMA       | UART0 ReceiveDMA  |
| DMA34   | UART1_TXDMA       | UART1 TransmitDMA |
| DMA35   | UART1_RXDMA       | UART1 ReceiveDMA  |
| DMA37   | UART2_TXDMA       | UART2 TransmitDMA |
| DMA38   | UART2_RXDMA       | UART2 ReceiveDMA  |

## UART Block Diagram

The UART Block Diagram figure shows a simplified block diagram of one UART module and how it interconnects to the processor system.

Figure 22-1: UART Block Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000000_a600cb169ac27e277271eb4ce495ba0713e1edf99d11b69c4079a88316562cab.png)

## UART Architectural Concepts

The following sections provide information about the UART architecture.

## Internal Interface

The UART is a DMA-capable peripheral with support for separate transmit and receive DMA controller channels. It operates in either DMA or programmed core modes. The core mode requires software management of the data flow

using either interrupts or polling. The DMA method requires minimal software intervention, as the DMA engine itself moves the data. The UART\_RBR and UART\_THR registers also connect to one of the peripheral DMA buses.

All UART registers are 32 bits, and the registers connect to the peripheral MMR bus. Not all MMRs can be used, and unused bits are zero-filled. The UART has three interrupt outputs described as follows.

- The transmit and receive request outputs can function as DMA requests and connect to the DMA controller. Therefore, if the DMA is not enabled, the DMA controller simply forwards the request to the system event controller (SEC).
- The status interrupt output connects directly to the SEC . On many processors, the alternative capture input () of one of the GP timers also senses the UART\_RX pin. When configured in capture mode, the processor can then use the GP timer to detect the bit rate of the received signal.

## External Interface

Each UART features a UART\_RX (receive) pin and a UART\_TX (transmit) pin available through the general-purpose ports. These two pins usually connect to an external transceiver device that meets the electrical requirements of full-duplex or half-duplex standards. For example, EIA-232, EIA-422, 4-wire EIA-485 for full-duplex or 2-wire EIA-485, LIN for half-duplex. Additionally, the UART features a pair of clear-to-send, input pins ( UART\_CTS ), and request-to-send, output pins ( UART\_RTS ) for hardware flow control. UART signals are multiplexed with other functions at the pin level.

## Hardware Flow Control

To prevent the UART transmitter from sending data while the receiving counterpart is not ready, the UART features a UART\_RTS / UART\_CTS hardware flow control mechanism. The UART\_RTS signal is an output that connects to the UART\_CTS input of the communication partner. If data transfer is bidirectional, the figure shows the UART Hardware Flow Control handshake.

Figure 22-2: UART Hardware Flow Control

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000001_4ab03a03141e19280442886559333a49fe94674ec5bfacb229698e8287b2dd02.png)

In both DMA and core mode, the receiver can deassert the UART\_RTS signal to indicate that its receive buffer is almost full. Continued data transfers can cause an overrun error. The transmitter pauses when the UART\_CTS input is in a deasserted state. In this state, the transmitter completes transmission of the data currently held in the transmit shift register ( UART\_TSR ) but it does not continue with the data in the transmit hold register ( UART\_THR ). If the UART\_CTS pin is asserted again, the transmitter resumes and loads the content of UART\_THR register into the UART\_TSR register.

## Bit Rate Generation

The peripheral clock (SCLK) and the 16-bit divisor in the UART\_CLK register characterize the sample clock. The UART uses the UART\_CTL.EN bit to enable the clock. By default, every serial bit is oversampled 16 times. The bit clock is 1/16th of the sample clock. If not in IrDA mode, the bit clock can equal the sample clock if the UART\_CLK.EDBO bit is set, so that the following equation applies:

Bit Rate = SCLK/(16  (1-EDBO)  × Divisor)

The UART Bit Rate Examples with 100 MHz SCLK table provides example divide factors required to support standard baud rates at an SCLK of 100 MHz.

Table 22-7: UART Bit Rate Examples with 100 MHz SCLK

| Bit Rate (bits/sec)   | EDBO = 0   | EDBO = 0   | EDBO = 0   | EDBO = 1   | EDBO = 1   | EDBO = 1   |
|-----------------------|------------|------------|------------|------------|------------|------------|
| Bit Rate (bits/sec)   | DL         | Actual     | %Error     | DL         | Actual     | %Error     |
| 2400                  | 2604       | 2400.15    | 0.006      | 41667      | 2399.98    | 0.001      |
| 4800                  | 1302       | 4800.31    | 0.006      | 20833      | 4800.08    | 0.002      |
| 9600                  | 651        | 9600.61    | 0.006      | 10417      | 9599.69    | 0.003      |
| 19200                 | 326        | 19171.78   | 0.147      | 5208       | 19201.23   | 0.006      |
| 38400                 | 163        | 38343.56   | 0.147      | 2604       | 38402.46   | 0.006      |
| 57600                 | 109        | 57339.45   | 0.452      | 1736       | 57603.69   | 0.006      |
| 115200                | 54         | 115740.74  | 0.469      | 868        | 115207.37  | 0.006      |
| 921600                | 7          | 892857.14  | 3.119      | 109        | 917431.19  | 0.452      |
| 1500000               | 4          | 1562500    | 4.167      | 67         | 1492537.31 | 0.498      |
| 3000000               | 2          | 3125000    | 4.167      | 33         | 3030303.03 | 1.01       |
| 6250000               | 1          | 6250000    | 0          | 16         | 6250000    | 0          |

NOTE: Carefully select the SCLK frequencies. Even multiples of the bit rate can decrease the error percentage.

Setting the bit clock equal to the sample clock ( UART\_CLK.EDBO =1) improves the bit rate granularity and the bit clock matches with the bit rate of the communication partner. Use UART\_CLK.EDBO mode only when bit rate accuracy is not acceptable in the UART\_CLK.EDBO =0 mode.

The UART\_CLK.EDBO =1 mode is not intended to increase the speed of operation beyond the electrical limitations of the UART transfer protocol.

## Autobaud Detection

At the chip level, the UART\_RX pin is typically routed to an alternate capture input ( TIMER\_ACI[nn] ) of a general-purpose timer. When working in width capture mode, the processor uses this general-purpose timer to detect the bit rate applied to the UART\_RX pin automatically by an external device. It often uses the capture capabilities of the timer to supervise the bit rate at run time. If the UART communicates with any device supplied

by a weak clock oscillator that drifts over time, the processor can then readjust its UART bit rate dynamically, as required.

Often, the processor uses autobaud detection for initial bit rate negotiations where it is most likely a completer device waiting for the host to send a predefined autobaud character. This situation is common for UART booting. Do not enable the UART\_CTL.EN bit while autobaud detection is in-process, to prevent the UART from starting a receive operation with incorrect bit rate matching. Alternatively, set the UART\_CTL.LOOP\_EN bit to disconnect the UART from its UART\_RX pin.

A software routine can detect the pulse widths of serial stream bit cells. The sample base of the timer is synchronous with the UART operation (all derived from the same SCLK). The UART uses pulse widths to calculate the bit rate divider as follows:

Divisor = TIMER\_TMR[n]\_WID /(16 (1-EDBO)  × Number of captured UART bits)

To increase the number of timer counts and the resolution of the captured signal, do not measure just the pulse width of a single bit. Instead, enlarge the pulse of interest over more bits. T raditionally, a NULL character (ASCII 0x00) is used in autobaud detection, as shown in the Autobaud Detection figure.

Figure 22-3: Autobaud Detection

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000002_576fcf7013747a08d462973e625694ad24e3a636aef7b9970252e2686e4ed725.png)

Because the example frame encloses 8 data bits and 1 start bit, apply the following formula:

Divisor = TIMER\_TMR[n]\_WID /(16 (1-EDBO)  × 9)

NOTE: For processor-specific mapping of timer alternate capture inputs to the UARTs of the processor, see "Width Capture (WIDCAP) Mode" in the " General-Purpose Timer (TIMER) " chapter.

Real receive signals often have asymmetrical falling and rising edges, and the sampling logic level is not exactly in the middle of the signal voltage range. At higher bit rates, such pulse-width-based autobaud detection does not always return adequate results without extra conditioning of the analog signal. Measure signal periods to work around this issue.

For example, predefine the ASCII character '@' (0x40) as the autobaud detection character and measure the period between two subsequent falling edges. As shown in the Autobaud Detection Character 0x40 figure, measure the period between the falling edge of the start bit and the falling edge after bit 6. Since this period encloses 8 bits, apply the following formula:

```
Divisor = TIMER_TMR[n]_PER /(16 (1-EDBO)  × 8) or: · Divisor = TIMER_TMR[n]_PER>> 7, if UART_CLK.EDBO =0
```

- Divisor = TIMER\_TMR[n]\_PER&gt;&gt; 3, if UART\_CLK.EDBO =1

The Autobaud Detection Character 0x40 figure shows the ASCII '@' (0x40) detection character.

Figure 22-4: Autobaud Detection Character 0x40

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000003_ef028314501966ca4a9339a3a924ef425ec35de43018d4ea584314aa4c8a41ca.png)

## UART Debug Features

The UART can automatically calculate and transmit a parity bit. The UART Parity table summarizes parity behavior assuming 8-bit data words ( UART\_CTL.WLS =b#11).

Table 22-8: UART Parity

|   PEN | STP   | EPS   | Data (hex)   | Data (binary, LSB first)   | Parity   |
|-------|-------|-------|--------------|----------------------------|----------|
|     0 | x     | x     | x            | x                          | None     |
|     1 | 0     | 0     | 0x60         | 0000 0110                  | 1        |
|     1 | 0     | 0     | 0x57         | 1110 1010                  | 0        |
|     1 | 0     | 1     | 0x60         | 0000 0110                  | 0        |
|     1 | 0     | 1     | 0x57         | 1110 1010                  | 1        |
|     1 | 1     | 0     | x            | x                          | 1        |
|     1 | 1     | 1     | x            | x                          | 0        |

The two force error bits, UART\_CTL.FPE and UART\_CTL.FFE , are intended for test purposes. They are useful for debugging software, especially in loopback mode.

The UART can be set to internal loopback mode ( UART\_CTL.LOOP\_EN =1). Loopback mode disconnects the input of the receiver from the receive pin and internally redirects the transmit output to the receiver. The transmit pin remains active and continues to transmit data externally as well. Loopback mode also forces the UART\_RTS pin to deassert, disconnects the UART\_STAT.CTS bit from the UART\_CTS input pin, and connects the internal version of UART\_RTS to the UART\_STAT.CTS bit.

Additionally, the UART\_TX pin can be forced to zero asynchronously using the UART\_CTL.SB bit.

## UART Operating Modes

The following sections describe the main operating modes of the UART.

- UART Mode
- IrDA SIR Mode

- Multi-Drop Bus Mode

## UART Mode

The UART mode follows an asynchronous serial communication protocol with these options:

- 1 start bit
- 5-8 data bits
- Address bit (available in MDB mode only)
- None, even, odd or sticky parity
- One, 1½, or two stop bits (1½ stop bits valid only in 5-bit word length)

The UART\_CTL register controls the format of received and transmitted character frames. Data is always transmitted and received with the least significant bit (LSB) first.

The Bit Stream on a UART TX Pin Transmitting an 'S' Character (0x53) figure shows a typical physical bit stream measured on a UART\_TX pin.

Figure 22-5: Bit Stream on a UART TX Pin Transmitting an 'S' Character (0x53)

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000004_e3865e4a7d0b6a785feda857df48a58be8585f6ed45cbc35a6ab6e0efaed48e4.png)

## IrDA SIR Mode

The UART also supports serial data communication by way of infrared signals, according to the recommendations of the Infrared Data Association (IrDA). The physical layer known as IrDA SIR (9.6/115.2 Kbps rate) is based on return-to-zero-inverted (RZI) modulation. The UART does not support pulse position modulation.

Using the 16x data rate clock, RZI modulation is achieved by inverting and modulating the non-return-to-zero (NRZ) code normally transmitted by the UART. On the receive side, the UART uses a 16x clock to determine an IrDA pulse sample window, from which it recovers the RZI modulated NRZ code.

NOTE: The UART\_CLK.EDBO bit is not valid in IrDA mode. Clear (=0) this bit in this mode.

## Multi-Drop Bus Mode

The UART uses a protocol for point-to-point connections as well as in networks where the EIA-485 standard is representative of UART-based bus systems. The EIA-232 standard defines point-to-point connections. In such networks, node addressing is important.

In a multidrop bus (MDB) network, for example, an address bit enhances the UART frame. The address bit is inserted between the data bits and the optional parity bit. To configure the UART for MDB mode, set the mode of operation bits ( UART\_CTL.MOD [5:4]) to 01.

By convention, the address bit is transmitted low for regular data bytes. When set, it marks special address bytes that require the attention of all nodes on the network.

Figure 22-6: UART Frame with Address Bit

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000005_5894fe83671d8db678eda151d3962709b1f57c29858b77f2db85b8a2a1509b4b.png)

All transmit operations are processed through the transmit buffer register ( UART\_THR ), so all DMA data transmissions clear the address bit. If data is written to the transmit address or insert pulse register ( UART\_TAIP ) instead, the same transmit operation is initiated with the only exception that the address bit is sent high.

The UART uses the UART\_STAT.ADDR bit of the receiver to signal whether the previously received frame had the address bit set or not. Hardware updates it every time a new frame is received. When the enable address word interrupt bit ( UART\_IMSK.EAWI ) is set, the reception of an address byte triggers a special status interrupt request.

The address sticky bit ( UART\_STAT.ASTKY ) is the sticky version of the UART\_STAT.ADDR bit. Hardware sets it whenever the UART\_STAT.ADDR bit is set. Software can clear the UART\_STAT.ASTKY bit with a W1C operation.

In MDB mode, only address bytes progress to the receive FIFO by default. Data bytes are gated unless the UART\_STAT.ASTKY bit is set. The receiver ignores all traffic on the UART bus. This way, the processor can go into low-power mode and interrupt activity does not load the processor every time a frame is transmitted on the UART bus. If, however, an address frame is transmitted, the receiver immediately samples all further traffic. A software routine can analyze the received data, decide whether it was of relevance for the local network node, and W1C the UART\_STAT.ASTKY bit if it was not.

Software can overrule of the hardware address frame detection by setting the UART\_STAT.ADDR bit and (indirectly) the UART\_STAT.ASTKY bit with a W1S operation.

The MDB mode follows an asynchronous serial communication protocol with the following options.

- 1 start bit
- 5-8 data bits
- Address bit
- None, even, odd or sticky parity
- One, 1½, or two stop bits (1½ stop bits are valid only in 5-bit word length)

NOTE: If the address bit and parity bit are both enabled, the parity check and generation includes the address bit in its parity calculation.

## UART Data Transfer Modes

The UART can transfer data using both the core and DMA. Receive and transmit paths operate independently except that the bit rate and the frame format are identical for both transfer directions. T ransmit and receive channels are both buffered. The UART\_THR register buffers the transmit shift register ( UART\_TSR ) and the UART\_RBR register buffers the receive shift register ( UART\_RSR ).

## UART Mode Transmit Operation (Core)

In core mode, the processor core moves data to and from the UART. A write to the UART\_THR register initiates the transmit operation. If no former operation is pending, the UART\_THR register passes the data immediately to the UART\_TSR register. There, it is shifted out at the bit rate characterized by the UART\_CLK register, with start, stop, and parity bits appended as defined by the UART\_CTL register.

The UART\_THR register and the UART\_TSR register can be modeled as a two-stage transmit buffer. The least significant bit (LSB) always transmits first. This bit is bit 0 of the value written to the UART\_THR register.

## UART Mode LIN Break Command

Some UART-based protocols demand synchronization methods that are not native to standard UART implementations. For example, the Local Interconnect Network (LIN) protocol requires a low-pulse of well-defined transmit length as a prologue to every multi-byte message. Its length must be at least 13 bit-times.

With previous UARTs, there were two options to implement this protocol:

- A null byte is transmitted with a temporarily lowered bit rate, or
- A software counter generates the period, and the asynchronous set break (SB) mechanisms pull the transmit pin low

Since both methods have their disadvantages, the newer UART introduces a new inter-frame gap technique.

The feature is not available in MDB or IrDA operating modes. However, in standard UART mode (bits UART\_CTL.MOD [5:4] =00), a write to the UART\_TAIP register initiates the transmission of an inter-frame pulse. If the transmit buffer is not empty, the UART first transmits all bytes in the queue. It only initiates with pulse generation after the last stop bit of the last byte has been shifted out.

The value written into the UART\_TAIP register defines the nature and the duration of the transmitted pulse. Bits [6:0] control the duration in bit-times and bit [7] controls the value (duration = UART\_TAIP [6:0] / UART\_CLK [15:0]). If UART\_TAIP [7] is set, and an active high pulse is issued, the number of stop bits is extended. If UART\_TAIP [7] is cleared, a low pulse is generated. Invert the polarity using the UART\_CTL.FCPOL bit. Writing a value of 13 into the UART\_TAIP register generates the break command as required by the LIN protocol.

NOTE: If the UART\_CTL.TPOLC bit is enabled, an inverted most-significant bit can be transmitted.

- NOTE: If another transmission is pending (in the UART\_TSR register), the UART\_TAIP initiated pulse is queued until after all pending operations have finished and all stop bits are transmitted.

The transmission of break command/inter-frame gap precedes transmission of the number of stop bits as set in the UART\_CTL.STB and UART\_CTL.STBH bit fields.

The UART receiver can detect break commands through the break indicator ( UART\_STAT.BI ) flag. This flag reports that an entire UART frame has been received in low state. It does not report whether the duration of the received low pulse was exact or at least 13 bit-times as LIN controllers transmit. Typically, the break indicator meets LIN requirements. The processor can use GP timers to determine the pulse width more precisely, if necessary.

Each UART\_RX pin is also routed to a GP timer through its alternate capture input (TACI). This functionality is not only useful for bit rate detection ( autobaud ) but also helps to measure the pulse widths precisely on the UART\_RX input. Additionally, the GP timers can issue an interrupt request or a fault condition when the received pulse width is shorter than a bit time or longer than the worst-case break condition. The windowed watchdog width mode of the GP timers controls this functionality.

## UART Mode Receive Operation (Core)

The receive operation uses the same data format as the transmit configuration except that one valid stop bit is always sufficient. The UART\_CTL.STB and UART\_CTL.STBH bits have no impact on the receiver.

The UART receiver senses the falling edges of the receive input. When it detects an edge, the receiver starts sampling the input according to settings in the UART\_CLK register. The receiver samples the start bit (majority sampling) close to its midpoint. If sampled low, it assumes a valid start condition. Otherwise, it discards the detected falling edge.

After detection of the start bit, the received word is shifted into the UART\_RSR register.

After the corresponding stop bit is received, the content of the UART\_RSR register is transferred to the 8-deep receive FIFO and is accessible by reading the UART\_RBR register.

The receive FIFOs and the UART\_RBR register act as a 9-stage receive buffer. If the stop bit of the ninth word is received before software reads the UART\_RBR register, an overrun error is reported. Overruns protect data in the UART\_RBR register and the receive FIFO from being overwritten by further data until the software clears the UART\_STAT.OE bit. However, the data in the UART\_RSR register is immediately destroyed as soon as the overrun occurs.

The sampling clock is 16 times faster than the bit clock. The receiver oversamples every bit 16 times and makes a majority-decision based on the middle three samples. This functionality improves immunity against noise and hazards on the line. The receiver disregards spurious pulses of less than two times the sampling clock period.

Normally, the receiver samples every incoming bit at exactly the 7th, 8th, and 9th sample clock. If, however, the UART\_CLK.EDBO bit is set to 1, the receiver samples bits approximately at 7/16th, 8/16th, and 9/16th of their period. This configuration achieves better bit rate granularity and accuracy as required at high operation speeds. Hardware design must ensure that the incoming signal is stable between 6/16th and 10/16th of the nominal bit period.

Reception starts when the UART receiver detects a falling edge on the UART\_RX input pin. The receiver attempts to see a start bit. The data is shifted into the UART\_RSR register. After the ninth sample of the first, the receiver processes the stop bit and copies the received data to the 8-stage receive FIFO. The UART\_RSR recovers for further data reception.

The receiver samples data bits close to their midpoint. Because the receiver clock is typically asynchronous to the data rate of the transmitter, the sampling point can drift relative to the center of the data bits. The sampling point is synchronized again with each start bit, so the error accumulates only over the length of a single word. The polarity of received data is selectable, using the UART\_CTL.RPOLC bit.

NOTE: The receiver checks for only a single stop bit. After the third sample of the first stop bit has been received (at time 9/16th of the stop bit duration), the receiver immediately acts (status update). It then prepares for new falling edge detection (start detection).

## IrDA Transmit Operation

To generate the IrDA pulse transmitted by the UART, the normal NRZ output of the transmitter is first inverted if the UART\_CTL.TPOLC bit is configured for active-low operation. In this configuration, a zero is transmitted as a high pulse of 16 UART clock periods and a one is transmitted as a low pulse for 16 UART clock periods. Then, six UART clock periods delay the leading edge of the pulse. Similarly, eight UART clock periods truncate the trailing edge of the pulse. For a 16-cycle UART clock period, this operation results in the final representation of the original zero as a high pulse of only 3/16 clock periods. The IrDA Transmit Pulse figure shows how the pulse is centered around the middle of the bit time. The final IrDA pulse is fed to the off-chip infrared driver.

This modulation approach ensures a pulse width output from the UART of three cycles high out of every 16 UART clock cycles. As shown in the IrDA Transmit Pulse figure, the error terms associated with the bit rate generator are small and well within the tolerance of most infrared transceiver specifications.

NOTE: In IrDA mode, writes to the UART\_TAIP register are equivalent to writes to the UART\_THR register.

Figure 22-7: IrDA Transmit Pulse

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000006_c6bb7dd34b02ce931ed6210e8b2c912c0562d6b5a42da2a14ab6ae5503266e43.png)

## IrDA Receive Operation

The IrDA receiver function is more complex than the transmit function. The receiver must discriminate the IrDA pulse and reject noise. The receiver looks for the IrDA pulse in a narrow window centered around the middle of the expected pulse.

Glitch filtering is accomplished by counting 16 system clocks from the time the receiver detects an initial pulse. If the pulse is absent when the counter expires, the receiver interprets it as a glitch. Otherwise, the receiver interprets it as a zero. This assessment is acceptable because glitches originating from on-chip capacitive cross-coupling typically do not last for more than a fraction of the system clock (SCLK) period. Appropriate shielding avoids sources outside of the chip and not part of the transmitter. The only other source of a glitch is the transmitter itself. The processor relies on the transmitter to perform within specification. If the transmitter violates the specification, unpredictable results can occur. The 4-bit counter adds an extra level of protection at a minimal cost.

NOTE: Because SCLK can change across systems, the longest glitch tolerated is inversely proportional to the SCLK frequency.

A counter that is clocked at the 16x bit-time sample clock determines the receive sampling window. The sampling window is resynchronized with each start bit by centering the sampling window around the start bit.

The polarity of receive data is selectable, using the UART\_CTL.RPOLC bit. The IrDA Receiver Pulse Detection figure provides examples of each polarity type.

Figure 22-8: IrDA Receiver Pulse Detection

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000007_c2dc5a3f72ca5271cc6f8d87fba880a93aa20692c6b8bc5d5ac49e6ce5d6c134.png)

## MDB Transmit Operation

In MDB mode, receive and transmit paths operate independently from each other, except for sharing bit rate and frame formats for both transfer directions.

Transmit operation is initiated by writing the UART\_THR or UART\_TAIP registers. A write to the UART\_THR register transmits the written word with the appending address bit set low. A write to the UART\_TAIP register transmits the written word with the appended address bit set high. The data is moved into the UART\_TSR register, where it is shifted out at the bit rate programmed by the UART\_CLK register, with start, stop, address, and parity bits appended, as required.

If DMA is enabled, the DMA engine always writes the data into the UART\_THR register, and the written word is transmitted with the appending address bit set low.

The polarity of transmit data is selectable, using the UART\_CTL.TPOLC bit.

## MDB Receive Operation

Receive operations use the same data format as the transmit configuration, except that the number of stop bits is always assumed to be 1. After detection of the start bit, the received word is shifted into the UART\_RSR register at the programmed bit.

Normally, the receiver samples every incoming bit at exactly the 7th, 8th, and 9th sample clock. If, however, the UART\_CLK.EDBO bit is set, the receiver samples the bits approximately at 7/16th, 8/16th, and 9/16th of their period. This configuration achieves better bit rate granularity and accuracy needed at high operation speeds. Hardware design must ensure that the incoming signal is stable between 6/16th and 10/16th of the nominal bit period.

After the appropriate number of bits (including address, parity, and stop bits) is received, the UART\_RSR register is transferred to the receive FIFO and accessible through the UART\_RBR register.

The polarity of receive data is selectable, using the UART\_CTL.RPOLC bit.

## DMA Mode

In DMA mode, separate receive and transmit DMA channels move data between the UART and memory. The software does not have to move data; it just must set up the appropriate transfers either through the descriptor mechanism or through autobuffer mode.

DMA channels provide a 4-deep FIFO, resulting in total buffer capabilities of 6 words at the transmit side and 9 words at the receive side. In DMA mode, the bus activity and arbitration mechanism determine the latency. The processor loading and interrupt priorities do not determine the latency.

To enable UART DMA, first set up the system DMA control registers. Then, enable the UART\_IMSK.ERBFI or UART\_IMSK.ETBEI interrupts. This sequence is necessary because these interrupt request lines double as DMA request lines. With DMA enabled, once these requests are received, the DMA control unit generates a direct memory access. If DMA is not enabled, the UART interrupt is passed on to the system interrupt handling unit. The status interrupt for the UART goes directly to the system event controller (SEC), bypassing the DMA unit completely.

For transmit DMA, programs must set the DMA\_CFG.SYNC bit. With this bit set, interrupt generation is delayed until the entire DMA FIFO is drained to the UART module. The UART transmit DMA interrupt service routine can disable the DMA or to clear the UART\_IMSK.ETBEI control bit only when the DMA\_CFG.SYNC bit is set. Otherwise, up to four data bytes can be lost.

When the UART\_IMSK.ETBEI bit is set, an initial transmit DMA request is issued immediately. The program then clears the UART\_IMSK.ETBEI bit through the DMA service routine.

In DMA transmit mode, the UART\_IMSK.ETBEI bit enables the peripheral request to the DMA FIFO. The DMA\_CFG.EN bit enables the strobe on the memory side. If the DMA count is less than the DMA FIFO depth,

which is 4, then the DMA interrupt can be requested before the UART\_IMSK.ETBEI bit is set. If this behavior is unwanted, set the DMA\_CFG.SYNC bit.

Regardless of the DMA\_CFG.SYNC setting, the DMA stream has not left the UART transmitter completely at the time the interrupt request is generated. T ransmission can abort in the middle of the stream, causing data loss, when the UART clock was disabled without extra synchronization with the UART\_STAT.TEMT bit.

The UART provides functionality to avoid resource-consuming polling of the UART\_STAT.TEMT bit. The UART\_IMSK\_SET.EDTPTI bit enables the UART\_STAT.TEMT bit to trigger a DMA interrupt. To delay the DMA completion interrupt until the last data word of a STOP DMA has left the UART, keep the DMA\_CFG.INT bit cleared and set the UART\_IMSK\_SET.EDTPTI bit instead. Then, the normal DMA completion interrupt is suppressed. Later, the UART\_STAT.TEMT event triggers a DMA interrupt after the last word of the DMA has left the UART transmit buffers. If DMA\_CFG.INT and UART\_IMSK.EDTPTI are set, when finishing STOP mode, the DMA requests two interrupts.

The DMA of the UART module supports 8-bit and 16-bit operation, but not 32-bit operation. It does not support sign-extension.

## Mixing DMA and Core Modes

Switching from DMA mode to core operation dynamically requires some consideration, especially for transmit operations. By default, the interrupt timing of the DMA is synchronized with the memory side of the DMA FIFOs. Normally, the transmit DMA completion interrupt is generated after the last byte is copied from the memory into the DMA FIFO. The transmit DMA interrupt service routine is not yet permitted to disable the DMA\_CFG.EN bit. The interrupt is requested when the DMA\_STAT.IRQDONE bit is set. The DMA\_STAT.RUN bit, however, remains set until the data has completely left the transmit DMA FIFO.

When planning to switch from a DMA to core mode, set the DMA\_CFG.SYNC bit in the word of the last descriptor or work unit before handing over control. Then, after the interrupt request occurs, software can write new data into the UART\_THR register as soon as the UART\_STAT.THRE bit permits. If the DMA\_CFG.SYNC bit cannot be set, software can poll the DMA\_STAT.RUN bit instead. Alternatively, using the UART\_IMSK.EDTPTI bit can avoid expensive status bit polling.

When switching from core to DMA operation, ensure that the first DMA request is issued properly. If the DMA is enabled while the UART is still transmitting, no precaution is required. If, however, the DMA is enabled after the UART\_STAT.TEMT bit is high, pulse the UART\_IMSK.ETBEI bit to initiate DMA transmission.

## Setting Up Hardware Flow Control

The following steps show how to set up UART hardware flow control:

1. Configure automatic or manual hardware flow control for the receiver through the UART\_CTL.ARTS bit, or the transmitter through the UART\_CTL.ACTS bit.
2. Configure UART\_CTS and UART\_RTS polarity through the UART\_CTL.FCPOL bit.

On reset, when the UART is not yet enabled and the port multiplexing has not been programmed, the UART\_RTS pin is not driven. Some applications require a resistor to pull the UART\_RTS signal to either state during reset.

## UART Event Control

Status flags in the UART\_STAT register are available to signal data reception, parity, and error conditions, if necessary.

## DMA and Interrupt Multiplexing

See the Direct Memory Access (DMA) chapter for information on DMA multiplexing. Several interrupts and DMA channels in the UART can be multiplexed.

NOTE: To operate in interrupt mode without using DMA channels, set the UART\_IMSK.ELSI bit. This configuration redirects receive and transmit requests to the status interrupt output. The status interrupt goes directly to the SEC without going through the DMA controller.

## Interrupt Masks

Each UART features a set of interrupt mask registers: UART\_IMSK , UART\_IMSK\_SET , and UART\_IMSK\_CLR . The UART\_IMSK register supports read/write operations. Writing ones to the UART\_IMSK\_SET register enables interrupts, writing ones to the UART\_IMSK\_CLR register disables them. Reads from either register return the enabled bits. This way, different interrupt service routines can control transmit, receive, and status interrupt requests independently and easily.

The UART module uses the UART\_IMSK registers to enable requests for system handling of empty or full states of data registers. Unless polling is used as a means of action, the UART\_IMSK.ERBFI and UART\_IMSK.ETBEI bits in this register are normally set.

Each UART module has three interrupt outputs. It uses one for transmission, one for reception, and one for reporting status events. The UART module routes transmit and receive requests through the DMA controller. The status request goes directly to the system event controller (SEC).

If the associated DMA channel is enabled, the request functions as a DMA request. If the DMA channel is disabled, it simply forwards the request to the SEC. A DMA channel must be associated with the UART module to enable transmit and receive interrupts. Otherwise, transmit and receive requests cannot be forwarded.

NOTE: To operate in interrupt mode without using DMA channels, set the UART\_IMSK.ELSI bit. This configuration redirects receive and transmit requests to the status interrupt request output. The status interrupt goes directly to the SEC without going through the DMA controller.

## Interrupt Servicing

Interrupt service routines (ISRs) perform UART writes and reads. Separate interrupt lines are provided for transmit, receive, and status. The UART\_IMSK register group enables the independent interrupts individually. To enable UART transmit interrupts, set the UART\_CTL.EN bit.

The ISRs evaluate the status bits in the UART\_STAT register to determine the signaling interrupt source. The system event controller for the processor assigns and unmasks interrupts. The ISRs must clear the interrupt latches explicitly. T o reduce interrupt frequency on the receive side in core mode, use the UART\_IMSK.ERFCI status

interrupt as an alternative to the regular UART\_IMSK.ERBFI receive interrupt. Hardware must ensure that at least two (if UART\_CTL.RFIT =0) or four (if UART\_CTL.RFIT =1) words are available in the receive buffer by the time the interrupt is requested.

## Transmit Interrupts

The UART module uses the UART\_IMSK\_SET.ETBEI bit to enable transmit interrupt requests.

The UART\_THR and UART\_TAIP registers are the same physical register, and both affect the signaling of the UART\_STAT.TEMT , UART\_STAT.TFI , and UART\_STAT.THRE bits similarly.

Figure 22-9: Transmit Interrupts

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000008_19887fda6598a7cd128cfda59cfaecddc06ba2cece43586dc438bb4cf0990baa.png)

The UART module asserts the transmit request along with the UART\_STAT.THRE bit, indicating that the transmit buffer is ready for new data. The UART\_STAT.THRE bit resets to 1. When the UART\_IMSK\_SET.ETBEI bit is set, the UART module immediately issues an interrupt or DMA request. This way, no special handling of the first character is required when transmission of a string is initiated. Set the UART\_IMSK\_SET.ETBEI bit and let the interrupt service routine load the first character from memory and write it to the UART\_THR register in the normal manner. ISRs can clear the UART\_IMSK.ETBEI bit through the UART\_IMSK\_CLR register when the string transmission has completed.

Hardware clears the UART\_STAT.THRE bit when new data is written to the UART\_THR register. These write operations also clear the transmit interrupt request. However, they also initiate further transmission. If continued transmission is undesirable, the UART module can alternatively clear the transmit request through the UART\_IMSK\_CLR.ETBEI bit register. T ransfers of data from the UART\_THR register to the UART\_TSR register reset this status flag in the UART\_STAT register.

ISRs can interrogate the UART\_STAT.TEMT bit to discover any ongoing transmission. The sticky counterpart of the UART\_STAT.TEMT bit, UART\_STAT.TFI , indicates when the transmit buffer has drained and can trigger a status interrupt. When data is pending in either one of these registers, the UART\_STAT.TEMT flag is low. As soon as all data has left the UART\_TSR register, the UART\_STAT.TEMT bit goes high again and indicates that all pending transmit operations (including stop bits) have finished. Then, it is safe to disable the UART\_CTL.EN bit or to three-state off-chip line drivers. Then, the UART module can generate an interrupt either through the status interrupt channel when the UART\_IMSK.ETFI bit is set, or through the DMA controller when enabled by the UART\_IMSK.EDTPTI bit.

When enabled by the UART\_IMSK.ETBEI bit, the UART\_STAT.THRE flag requests data along the peripheral command lines to the DMA controller (referred to as TXREQ). This signal is routed through the DMA controller. If the associated DMA channel is enabled, the TXREQ signal functions as a DMA request, otherwise the DMA controller simply forwards it to the SEC. Alternatively the UART\_IMSK.ETXS bit can redirect the transmit interrupts to the UART status interrupt.

With interrupts disabled, the UART module can poll the status flags to determine when data is ready to move. Because polling is processor intensive; it is not typically used in real-time signal processing environments. Since read operations from UART\_STAT registers have no side effects, different software threads can interrogate these registers without mutual impacts.

Polling the SEC\_SSTAT[n] register without enabling the interrupts by the SEC\_CCTL[n] register is an alternate method of operation to consider. Software can write up to two words into the UART\_THR register before enabling the UART clock. As soon as the UART\_CTL.EN bit is set, the UART module sends those two words.

## Receive Interrupts

The UART module uses the UART\_IMSK\_SET.ERBFI bit to enable receive interrupt requests. If set, the UART\_STAT.DR flag requests an interrupt on the dedicated RXREQ output, indicating that new data is available in the UART\_RBR register. This signal is routed through the DMA controller. If the associated DMA channel is enabled, the RXREQ signal functions as a DMA request; otherwise the DMA controller simply forwards it to the SEC. Alternatively, if no DMA channel is assigned to the UART, the UART\_IMSK.ERXS bit can redirect the receive interrupts to the UART status interrupt. When software reads the UART\_RBR register, hardware clears the UART\_STAT.DR bit again, which, in turn, clears the receive interrupt request.

Figure 22-10: Receive Interrupts

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000009_5efdad620cb4e6cb534bb66715b826ceba5ad2ee6974f629b6c88dc3d5f31b4c.png)

## Hardware updates the following:

- UART\_STAT.DR bits
- UART\_STAT.ADDR bits
- UART\_STAT.ASTKY bits
- UART\_STAT.PE bits

- UART\_STAT.FE bits
- UART\_STAT.BI bits
- UART\_RBR register

The UART\_STAT.OE bit is updated as soon as an overflow condition occurs (for example when a stop bit for a frame is received and the receive FIFO is full). When software does not read the UART\_RBR register in time, the received data is protected from being overwritten by new data until software clears the UART\_STAT.OE bit. Only the content of the UART\_RSR register can be overwritten in the overrun case.

The UART module uses the UART\_STAT.RFCS bit to monitor the state of the 8-deep receive FIFO. It uses the UART\_CTL.RFIT bit to control the behavior of the buffer. If UART\_CTL.RFIT is zero, the UART\_STAT.RFCS bit is set when the receive buffer holds four or more words. If UART\_CTL.RFIT is set, the UART\_STAT.RFCS bit is set when the receive buffer holds seven or more words. Hardware clears the UART\_STAT.RFCS bit when a core or DMA reads the UART\_RBR register and when the buffer is flushed below the level of four ( UART\_CTL.RFIT =0) or seven ( UART\_CTL.RFIT =1). If the associated interrupt bit UART\_IMSK.ERFCI is enabled, a status interrupt request is reported when the UART\_STAT.RFCS bit is set.

If errors are detected during reception, an interrupt can be requested from the status interrupt output. This status interrupt request goes directly to the SEC. The bit enables status interrupt requests.

The controller detects the following error conditions, shown with their associated bits in the UART\_STAT register.

- Overrun error ( UART\_STAT.OE bit)
- Parity error ( UART\_STAT.PE bit)
- Framing error or invalid stop bit ( UART\_STAT.FE bit)
- Break indicator ( UART\_STAT.BI bit)

## Status Interrupts

The UART module uses status interrupt channels for the following purposes:

- Line status interrupt requests
- Flow control interrupt requests
- Receive FIFO threshold interrupt requests
- Transmission finished interrupt request

The UART module uses the UART\_IMSK.ELSI bit to enable the line status interrupts. If set, the status interrupt request is asserted with one of the UART\_STAT.BI , UART\_STAT.FE , UART\_STAT.PE , or UART\_STAT.OE receive errors bits. A W1C operation to the UART\_STAT register clears the error bits. Once all error conditions are cleared, the interrupt request deasserts.

The UART module uses the UART\_IMSK\_SET.ERFCI bit to enable the receive FIFO count interrupt. If set, a status interrupt request is generated when the UART\_STAT.RFCS is active. The UART\_STAT.RFCS bit

indicates a receive buffer threshold level. If the UART\_CTL.RFIT bit is cleared, software can safely read two words out of the UART\_RBR register by the time the UART\_STAT.RFCS interrupt occurs.

If the UART\_CTL.RFIT bit is set, software can safely read four words. The interrupt request and the UART\_STAT.RFCS bit are cleared when the UART\_RBR is read enough of times, so that the receive buffer drains below the threshold of two ( UART\_CTL.RFIT =0) or four ( UART\_CTL.RFIT =1). Because in DMA mode a status service routine may not be permitted to read UART\_RBR , this interrupt is only recommended in core mode. In DMA mode, use this functionality for error recovery only.

The UART module uses the UART\_IMSK\_SET.EDSSI bit to enable the flow control interrupts. If active, a status interrupt is generated when the sticky UART\_STAT.SCTS bit register is set, indicating that the UART\_CTS input of the transmitter been reasserted. A W1C operation to the UART\_STAT.SCTS bit clears the interrupt request.

The UART module uses the UART\_IMSK\_SET.ETFI bit to enable the transmission finished interrupt. If active, a status interrupt request is asserted when the UART\_STAT.TFI bit is set. The UART\_STAT.TFI is the sticky version of the UART\_STAT.TEMT bit, indicating that a byte that started transmission has finished. A W1C operation to the UART\_STAT.TFI bit clears the interrupt request.

## Multi-Drop Bus Events

Several status bits and interrupt features in the UART\_STAT and UART\_IMSK registers facilitate efficient data handling in multi-drop bus mode. These features include the address ( UART\_STAT.ADDR ) bit, the address sticky ( UART\_STAT.ASTKY ) bit and the enable address word interrupt ( UART\_IMSK.EAWI ). One of the key features of the multi-drop bus protocol is its address bit. The address bit signifies to the completer that the requester is transmitting an address word (all read it) or a data word (only the addressed target reads its). The UART hardware provides an efficient method of handling the situation described with the use of UART\_STAT.ASTKY bit.

- NOTE: The UART module uses the UART\_STAT.ASTKY bit in multi-drop bus mode to indicate when an address operation for a peripheral is occurring. The UART\_STAT.ASTKY bit is a sticky version of the UART\_STAT.ADDR bit. Hardware sets the bit whenever the UART\_STAT.ADDR bit is set. Only software clears it with a W1C operation. With the UART\_STAT.ASTKY bit set, words are received regardless of the mode bit or address bit setting. With the UART\_STAT.ASTKY bit cleared, only address words (mode bit =1) are received and words with mode bit =0 are ignored in MDB mode. This bit does not affect reception in non-MDB modes. (Words with mode bit =0 are not moved from the UART\_RSR register to the receive FIFO.)

## UART Programming Model

The following sections provide basic procedures for configuring various UART operations.

## Detecting Autobaud

To detect Autobaud:

1. Ensure that the timer is disabled.

2. Configure the following bits: UART\_CTL.MOD =00, UART\_CTL.LOOP\_EN =1, UART\_CTL.WLS =11 (8-bit data), and UART\_CTL.EN =1
3. Configure the following bits: TIMER\_TMR[n]\_CFG.TMODE =1101, TIMER\_TMR[n]\_CFG.OUTDIS =1, TIMER\_TMR[n]\_CFG.IRQMODE =10 and enable the timer.
4. Send test data through the host device and wait for the timer interrupt and disable the timer.

The bit rate can be derived from the timer period register value according to the formula provided in the Autobaud Detection section.

## Using Common Initialization Steps

When using the core or the DMA to execute transfers, the following steps are common to all UART modes.

1. All UART signals are multiplexed and compete with other functions at pin level. First, program the port registers according to the guidelines in the PORT chapter.
2. Program the UART\_CLK register. Refer to Bit Rate Generation topic.
3. Program the UART\_CTL register and enable the UART clock.

## Using Core Transfers

Write data into the UART\_THR register, when the UART\_STAT.THRE bit is set, to initiate a core transmit operation. If the UART\_STAT.DR bit is set, received data can be read from the UART\_RBR register.

## Using DMA Transfers

1. Make sure that the UART\_IMSK.ETBEI or the UART\_IMSK.ERBFI bits are cleared before configuring the DMA.
2. Configure the dedicated DMA channel.
3. Set the UART\_IMSK.ETBEI or UART\_IMSK.ERBFI bits to start the transfer.

## Using Interrupts

Each UART features three interrupt signal outputs.

1. Enable individual interrupts in the system event controller (SEC)
2. Register IRQ handlers.
3. Use the interrupts mask registers to enable specific IRQ events.

## Setting Up Hardware Flow Control

1. Configure automatic or manual hardware flow control for the receiver through the UART\_CTL.ARTS bit, or the transmitter through the UART\_CTL.ACTS bit.
2. Configure UART\_CTS and UART\_RTS polarity through the UART\_CTL.FCPOL bit.

## ADSP-2184x UART Register Descriptions

UART (UART) contains the following registers.

Table 22-9: ADSP-2184x UART Register List

| Name          | Description                            |
|---------------|----------------------------------------|
| UART_CLK      | Clock Rate Register                    |
| UART_CTL      | Control Register                       |
| UART_IMSK     | Interrupt Mask Register                |
| UART_IMSK_CLR | Interrupt Mask Clear Register          |
| UART_IMSK_SET | Interrupt Mask Set Register            |
| UART_RBR      | Receive Buffer Register                |
| UART_RSR      | Receive Shift Register                 |
| UART_RXCNT    | Receive Counter Register               |
| UART_SCR      | Scratch Register                       |
| UART_STAT     | Status Register                        |
| UART_TAIP     | Transmit Address/Insert Pulse Register |
| UART_THR      | Transmit Hold Register                 |
| UART_TSR      | Transmit Shift Register                |
| UART_TXCNT    | Transmit Counter Register              |

## Clock Rate Register

The UART\_CLK register divides the system clock ( SCLK) down to the bit clock.

Figure 22-11: UART\_CLK Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000010_734c9b2fd45cb1043624c2e37023190d98ca783decab1e384b480c614991afc8.png)

Table 22-10: UART\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | EDBO       | Enable Divide By One. The UART_CLK.EDBO bit enables the bypassing of the divide-by-16 prescaler in bit clock generation. This functionality improves bit rate granularity, especially at high bit rates. Do not set this bit in IrDA mode. Note: Properly select the SCLK frequencies. Even multiples of bit rate decrease the error percentage. Setting the bit clock equal to the sample clock ( UART_CLK.EDBO =1) improves the bit rate granularity and the bit clock matches with the bit rate of the communication partner. The disadvantage is that the power dissipation is higher and sample points are not al- ways accurate. Therefore, use UART_CLK.EDBO =1 mode only when bit rate accuracy is not acceptable in the UART_CLK.EDBO =0 mode. The UART_CLK.EDBO =1 mode is not intended to increase the speed of operation | Enable Divide By One. The UART_CLK.EDBO bit enables the bypassing of the divide-by-16 prescaler in bit clock generation. This functionality improves bit rate granularity, especially at high bit rates. Do not set this bit in IrDA mode. Note: Properly select the SCLK frequencies. Even multiples of bit rate decrease the error percentage. Setting the bit clock equal to the sample clock ( UART_CLK.EDBO =1) improves the bit rate granularity and the bit clock matches with the bit rate of the communication partner. The disadvantage is that the power dissipation is higher and sample points are not al- ways accurate. Therefore, use UART_CLK.EDBO =1 mode only when bit rate accuracy is not acceptable in the UART_CLK.EDBO =0 mode. The UART_CLK.EDBO =1 mode is not intended to increase the speed of operation |
| 31 (R/W)           | EDBO       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Bit clock prescaler = 16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 15:0 (R/W)         | DIV        | Divisor. The UART_CLK.DIV provides the divisor for the UART's clock bit rate calculation. The bit rate is defined by: Bit Rate = SCLK / (16 (1-EDBo) x UART_CLK.DIV )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Divisor. The UART_CLK.DIV provides the divisor for the UART's clock bit rate calculation. The bit rate is defined by: Bit Rate = SCLK / (16 (1-EDBo) x UART_CLK.DIV )                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

## Control Register

The UART\_CTL register provides enable and disable control for UART and IrDA mode of operation. It also provides UART line control, permitting selection of the format of received and transmitted character frames. Modem feature control also is available from this register, including partial modem functionality to allow for hardware flow control and loopback mode.

Figure 22-12: UART\_CTL Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000011_bf765a21535443e87902e00aa27ab5f7b9f9bcaf2ed675cef89aaec146483040.png)

Table 22-11: UART\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 30 (R/W)           | RFRT       | Receive FIFO RTS Threshold. The UART_CTL.RFRT bit controls UART_RTS pin assertion and deassertion tim- ing. This bit is ignored if UART_CTL.ARTS is cleared. If set, the UART_RTS pin is deasserted when the receive buffer already holds seven words and an eighth start bit is detected. It is reasserted when the FIFO contains seven words or less. If cleared, the UART_RTS pin is deasserted when the RX buffer already holds four words and a fifth                                                                                                                                                                                                                                                              | Receive FIFO RTS Threshold. The UART_CTL.RFRT bit controls UART_RTS pin assertion and deassertion tim- ing. This bit is ignored if UART_CTL.ARTS is cleared. If set, the UART_RTS pin is deasserted when the receive buffer already holds seven words and an eighth start bit is detected. It is reasserted when the FIFO contains seven words or less. If cleared, the UART_RTS pin is deasserted when the RX buffer already holds four words and a fifth                                                                                                                                                                                                                                                              |
| 30 (R/W)           | RFRT       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Deassert RTS if RX FIFO word count > 4; assert if <= 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 30 (R/W)           | RFRT       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Deassert RTS if RX FIFO word count > 7; assert if <= 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 29 (R/W)           | RFIT       | Receive FIFO IRQ Threshold. The UART_CTL.RFIT bit controls the timing of the UART_STAT.RFCS bit. If UART_CTL.RFIT is cleared, the receive threshold is two. If UART_CTL.RFIT is set, the threshold is four words in the receive buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Receive FIFO IRQ Threshold. The UART_CTL.RFIT bit controls the timing of the UART_STAT.RFCS bit. If UART_CTL.RFIT is cleared, the receive threshold is two. If UART_CTL.RFIT is set, the threshold is four words in the receive buffer.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 29 (R/W)           | RFIT       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Set RFCS=1 if RX FIFO count >= 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 29 (R/W)           | RFIT       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Set RFCS=1 if RX FIFO count >= 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 28 (R/W)           | ACTS       | Automatic CTS. The UART_CTL.ACTS bit must be set to enable the UART_CTS input pin for UART_TX handshaking. If enabled, the UART_STAT.CTS bit holds the value (if UART_CTL.FCPOL is set) or complement value (if UART_CTL.FCPOL is cleared) of the UART_CTS input pin. The UART_STAT.CTS bit can be used to determine whether the external device is ready to receive data (if UART_STAT.CTS is set) or whether it is busy (if UART_STAT.CTS is cleared). If UART_CTL.ACTS is cleared, the UART_TX handshaking protocol is disabled, and the UART_TX line transmits data whenever there is data to send, regardless of the value of UART_CTS . Software can pause ongoing transmission by setting the UART_CTL.XOFF bit. | Automatic CTS. The UART_CTL.ACTS bit must be set to enable the UART_CTS input pin for UART_TX handshaking. If enabled, the UART_STAT.CTS bit holds the value (if UART_CTL.FCPOL is set) or complement value (if UART_CTL.FCPOL is cleared) of the UART_CTS input pin. The UART_STAT.CTS bit can be used to determine whether the external device is ready to receive data (if UART_STAT.CTS is set) or whether it is busy (if UART_STAT.CTS is cleared). If UART_CTL.ACTS is cleared, the UART_TX handshaking protocol is disabled, and the UART_TX line transmits data whenever there is data to send, regardless of the value of UART_CTS . Software can pause ongoing transmission by setting the UART_CTL.XOFF bit. |
| 28 (R/W)           | ACTS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Disable TX handshaking protocol                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 28 (R/W)           | ACTS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Enable TX handshaking protocol                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 27 (R/W)           | ARTS       | Automatic RTS. The UART_CTL.ARTS bit must be set to enable the UART_RTS input pin for UART_TX handshaking. If set, the hardware guarantees a minimal UART_RTS pin deassertion pulse width of at least the number of data bits defined by the UART_CTL.WLS bit field. If cleared, the UART_RTS pin is not generated automat- ically by hardware. The UART_RTS pin can still be manually controlled by the UART_CTL.MRTS bit, and software is responsible for UART_RTS pulse width con- trol (if needed).                                                                                                                                                                                                                 | Automatic RTS. The UART_CTL.ARTS bit must be set to enable the UART_RTS input pin for UART_TX handshaking. If set, the hardware guarantees a minimal UART_RTS pin deassertion pulse width of at least the number of data bits defined by the UART_CTL.WLS bit field. If cleared, the UART_RTS pin is not generated automat- ically by hardware. The UART_RTS pin can still be manually controlled by the UART_CTL.MRTS bit, and software is responsible for UART_RTS pulse width con- trol (if needed).                                                                                                                                                                                                                 |
| 27 (R/W)           | ARTS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Disable RX handshaking protocol.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 27 (R/W)           | ARTS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Enable RX handshaking protocol.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 22-11: UART\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 26 (R/W)           | XOFF       | Transmitter off. The UART_CTL.XOFF bit (if set) turns off transmission (XOFF) by preventing the content of THR from being continued to TSR. When set, this bit turns on transmission (XON). The state of the UART_CTL.XOFF bit is ignored if the UART_CTL.ACTS bit is set.                                                                                                                                       | Transmitter off. The UART_CTL.XOFF bit (if set) turns off transmission (XOFF) by preventing the content of THR from being continued to TSR. When set, this bit turns on transmission (XON). The state of the UART_CTL.XOFF bit is ignored if the UART_CTL.ACTS bit is set.                                                                                                                                       |
| 26 (R/W)           | XOFF       | 0                                                                                                                                                                                                                                                                                                                                                                                                                | Transmission ON, if ACTS=0                                                                                                                                                                                                                                                                                                                                                                                       |
| 26 (R/W)           | XOFF       | 1                                                                                                                                                                                                                                                                                                                                                                                                                | Transmission OFF, if ACTS=0                                                                                                                                                                                                                                                                                                                                                                                      |
| 25 (R/W)           | MRTS       | Manual Request to Send. The UART_CTL.MRTS bit controls the state of the UART_RTS output pin when the UART_CTL.ARTS bit is cleared. When UART_CTL.MRTS is cleared, the UART deasserts the UART_RTS pin, signaling to the external device that the UART is not ready to receive. When UART_CTL.MRTS is set, the UART asserts the UART_RTS pin, signaling to the external device that the UART is ready to receive. | Manual Request to Send. The UART_CTL.MRTS bit controls the state of the UART_RTS output pin when the UART_CTL.ARTS bit is cleared. When UART_CTL.MRTS is cleared, the UART deasserts the UART_RTS pin, signaling to the external device that the UART is not ready to receive. When UART_CTL.MRTS is set, the UART asserts the UART_RTS pin, signaling to the external device that the UART is ready to receive. |
| 25 (R/W)           | MRTS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                | Deassert RTS pin when ARTS=0                                                                                                                                                                                                                                                                                                                                                                                     |
| 25 (R/W)           | MRTS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                | Assert RTS pin when ARTS=0                                                                                                                                                                                                                                                                                                                                                                                       |
| 24 (R/W)           | TPOLC      | IrDA TX Polarity Change. The UART_CTL.TPOLC bit selects the active low or high polarity for IrDA commu- nications. This bit is effective only in IrDA mode. If set, in IrDA mode, the UART_TX pin idles high. In UART or MDBmode, it is inverted-NRZ. If cleared, in IrDA mode, the UART_TX pin idles low. In UART or MDBmode, it is NRZ.                                                                        | IrDA TX Polarity Change. The UART_CTL.TPOLC bit selects the active low or high polarity for IrDA commu- nications. This bit is effective only in IrDA mode. If set, in IrDA mode, the UART_TX pin idles high. In UART or MDBmode, it is inverted-NRZ. If cleared, in IrDA mode, the UART_TX pin idles low. In UART or MDBmode, it is NRZ.                                                                        |
| 24 (R/W)           | TPOLC      | 0                                                                                                                                                                                                                                                                                                                                                                                                                | Active-low TX polarity setting                                                                                                                                                                                                                                                                                                                                                                                   |
| 24 (R/W)           | TPOLC      | 1                                                                                                                                                                                                                                                                                                                                                                                                                | Active-high TX polarity setting                                                                                                                                                                                                                                                                                                                                                                                  |
| 23 (R/W)           | RPOLC      | IrDA RX Polarity Change. The UART_CTL.RPOLC bit selects the active low or high polarity for IrDA commu- nications. This bit is effective only in IrDA mode. If set, in IrDA mode, the UART_RX pin idles high. In UART or MDBmode, it is inverted-NRZ. If cleared, in IrDA mode, the UART_RX pin idles low. In UART or MDBmode, it is NRZ.                                                                        | IrDA RX Polarity Change. The UART_CTL.RPOLC bit selects the active low or high polarity for IrDA commu- nications. This bit is effective only in IrDA mode. If set, in IrDA mode, the UART_RX pin idles high. In UART or MDBmode, it is inverted-NRZ. If cleared, in IrDA mode, the UART_RX pin idles low. In UART or MDBmode, it is NRZ.                                                                        |
| 23 (R/W)           | RPOLC      | 0                                                                                                                                                                                                                                                                                                                                                                                                                | Active-low RX polarity setting                                                                                                                                                                                                                                                                                                                                                                                   |
| 23 (R/W)           | RPOLC      | 1                                                                                                                                                                                                                                                                                                                                                                                                                | Active-high RX polarity setting                                                                                                                                                                                                                                                                                                                                                                                  |

Table 22-11: UART\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 22 (R/W)           | FCPOL      | Flow Control Pin Polarity. The UART_CTL.FCPOL bit selects the polarities of the UART_CTS and UART_RTS pins. When the UART_CTL.FCPOL bit is cleared, the UART_RTS and UART_CTS pins are active low, and the UART is halted when the UART_RTS and UART_CTS pin state is high. When UART_CTL.FCPOL bit is set, the UART_RTS and UART_CTS pins are active high, and the UART is halted when the UART_RTS and UART_CTS pin state is low. 0 Active low CTS/RTS  |
| 19 (R/W)           | SB         | Set Break. If set, the UART_CTL.SB bit forces the UART_TX pin to low asynchronously, regardless of whether or not data is currently transmitted. This bit functions even when the UART clock is disabled. Because the UART_TX pin normally drives high, it can be used as a flag output pin, if the UART is not used. (For example, if UART_CTL.TPOLC is cleared, drive UART_TX pin low; or if UART_CTL.TPOLC is set, drive UART_TX pin high.) 0 No force |
| 18 (R/W)           | FFE        | Force Framing Error on Transmit. The UART_CTL.FFE bit is intended for test purposes. This bit is useful for debug- ging software, especially in loopback mode. 0 Normal operation                                                                                                                                                                                                                                                                         |
| 17 (R/W)           | FPE        | Force Parity Error on Transmit. The UART_CTL.FPE bit is intended for test purposes. This bit is useful for debug- ging software, especially in loopback mode. 0 Normal operation                                                                                                                                                                                                                                                                          |

Table 22-11: UART\_CTL Register Fields (Continued)

| Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Sticky Parity.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Sticky Parity.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 1 Force (Stick) parity to defined value (if PEN=1) Even Parity Select. 0 Odd parity                                                                                                                                                                                                                                                                                                                                                                                                                    | 1 Force (Stick) parity to defined value (if PEN=1) Even Parity Select. 0 Odd parity                                                                                                                                                                                                                                                                                                                                                                                                                    |
| and UART_CTL.EPS value. The receiver issues a parity error if UART_CTL.STP and the 0 Disable 1 Enable Bit Time).                                                                                                                                                                                                                                                                                                                                                                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| Parity Enable. The UART_CTL.PEN bit enables parity transmission and parity check. The UART_CTL.PEN bit inserts one additional bit between the most significant data bit and the first stop bit. The polarity of this so-called parity bit depends on data and the UART_CTL.STP control bits. Both transmitter and receiver calculate the parity compares the received parity bit with the expected value and they do not match. If the UART_CTL.PEN bit is cleared, the UART_CTL.EPS bits are ignored. | Parity Enable. The UART_CTL.PEN bit enables parity transmission and parity check. The UART_CTL.PEN bit inserts one additional bit between the most significant data bit and the first stop bit. The polarity of this so-called parity bit depends on data and the UART_CTL.STP control bits. Both transmitter and receiver calculate the parity compares the received parity bit with the expected value and they do not match. If the UART_CTL.PEN bit is cleared, the UART_CTL.EPS bits are ignored. |
| parity transmit and check Stop Bits (Half                                                                                                                                                                                                                                                                                                                                                                                                                                                              | 0 half-bit-time stop bit                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| Stop Bits. The UART_CTL.STB bit controls how many stop bits are appended to transmitted data.                                                                                                                                                                                                                                                                                                                                                                                                          | Stop Bits. The UART_CTL.STB bit controls how many stop bits are appended to transmitted data.                                                                                                                                                                                                                                                                                                                                                                                                          |
| 0 1 1 half-bit-time stop bit                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 0 1 stop bit                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |

Table 22-11: UART\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9:8 (R/W)          | WLS        | Word Length Select. The UART_CTL.WLS field determines whether the transmitted and received UART word consists of 5, 6, 7, or 8 data bits.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 9:8 (R/W)          | WLS        | 0 5-bit word                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 9:8 (R/W)          | WLS        | 1 6-bit word                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 9:8 (R/W)          | WLS        | 2 7-bit word                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 9:8 (R/W)          | WLS        | 3 8-bit word                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 5:4 (R/W)          | MOD        | Mode of Operation. The UART_CTL.MOD selects the UART operation mode (UMOD).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5:4 (R/W)          | MOD        | 0 UART mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 5:4 (R/W)          | MOD        | 1 MDBmode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 5:4 (R/W)          | MOD        | 2 IrDA SIR mode                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 1 (R/W)            | LOOP_EN    | Loopback Enable. The UART_CTL.LOOP_EN bit enables UART loopback mode. When set, this bit disconnects the input of the receiver from the UART_RX pin, and internally redirects the transmit output to the receiver. The UART_TX pin remains active and continues to transmit data externally as well. Loopback mode also forces the UART_RTS pin to its deassertive state, disconnects the UART_CTS bit from the UART_CTS input pin, and directly connects the UART_CTL.MRTS bit to the UART_STAT.CTS bit. In loopback mode, setting the UART_CTL.MRTS bit sets the UART_STAT.CTS bit and enables the transmitter of the UART. Clearing the UART_CTL.MRTS bit clears the UART_STAT.CTS bit and disables the transmitter of the UART. Disable |
| 1 (R/W)            | LOOP_EN    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 1 (R/W)            | LOOP_EN    | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W)            | EN         | Enable UART. The UART_CTL.EN enables the UART clocks. This bit also resets the state machine and control registers when cleared. Using this bit to disable the UART -- when not used -- reduces power consumption. Disable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 0 (R/W)            | EN         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 0 (R/W)            | EN         | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

## Interrupt Mask Register

The UART\_IMSK register indicates the interrupt mask status (unmasked, if set, or masked, if cleared) of the UART status interrupt requests. This register is not a data register. Instead, it is controlled by the UART\_IMSK\_SET and UART\_IMSK\_CLR register pair. Writing ones to the UART\_IMSK\_SET register enables (unmasks) interrupt requests, and writing ones to the UART\_IMSK\_CLR register disables (masks) them. Reads from either register return the enabled bits.

The UART\_IMSK register is used to enable requests for system handling of empty or full states of UART data registers. Unless polling is used as a means of action, the UART\_IMSK.ERBFI and UART\_IMSK.ETBEI bits are normally set. Setting this register without enabling system DMA causes the UART to notify the processor of the data inventory state using interrupts. For proper operation in this mode, system interrupts must be enabled, and appropriate interrupt handling routines must be present.

Each UART features three separate interrupt channels to handle the data transmit, data receive, and line status events independently, regardless of whether DMA is enabled or not. If no DMA channels are assigned to the UART, set the UART\_IMSK.ELSI bit to reroute the transmit and receive interrupts to the status interrupt request output.

With system DMA enabled, the UART uses DMA to transfer data to or from the processor. Dedicated DMA channels are available for receive and transmit operations. Line error handling can be configured independently from the receive or transmit setup.

The DMA of the UART is enabled by first setting up the system DMA control registers and then enabling the UART\_IMSK.ERBFI and UART\_IMSK.ETBEI interrupts. This configuration is because the interrupt request lines double as DMA request lines. Depending on whether DMA is enabled or not, upon receiving these requests, the DMA control unit either generates a direct memory access or passes the UART interrupt on to the system interrupt handling unit(s). However, the error interrupt for the UART goes directly to the system interrupt handling unit(s), bypassing the DMA unit completely.

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000012_77712a84a74f0b62c95f8b8733cd312d62991e7cde875c916b70e1b89fc151f4.png)

31

0

30

0

29

0

28

0

27

0

Figure 22-13: UART\_IMSK Register Diagram

Table 22-12: UART\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | ETXS       | Enable TX to Status Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ETXS bit indicates re-direction of the TX interrupt requests to status interrupt output. If cleared, TX interrupt requests are routed to normal interrupt outputs.                                                   |
| 8 (R/W)            | ERXS       | Enable RX to Status Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ERXS bit indicates re-direction of RX interrupt requests to status interrupt output. If cleared, RX interrupt requests are routed to normal interrupt outputs.                                                       |
| 7 (R/W)            | EAWI       | Enable Address Word Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.EAWI bit indicates generation of a status interrupt request when an Address word in MDB-mode is present in the UART_RBR . A received word is an address word if the UART_STAT.ADDR bit is set. 0 Interrupt is masked |
| 7 (R/W)            | EAWI       | 1 Interrupt is unmasked                                                                                                                                                                                                                                                                                    |
| 7 (R/W)            | EAWI       |                                                                                                                                                                                                                                                                                                            |

26

0

25

0

24

0

23

0

22

0

21

0

20

0

19

0

18

0

17

0

16

0

Table 22-12: UART\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | ERFCI      | Enable Receive FIFO Count Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ERFCI bit indicates enabling of the receive buffer threshold interrupt request if signaled by the UART_STAT.RFCS bit. Read the UART_RBR register sufficient times to clear the interrupt request.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 6 (R/W)            | ERFCI      | 0 Interrupt is masked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 6 (R/W)            | ERFCI      | 1 Interrupt is unmasked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 5 (R/W)            | ETFI       | Enable Transmission Finished Interrupt Mask Status. If set (interrupt unmasked) the UART_IMSK.ETFI bit indicates enabling of inter- rupt generation on the status interrupt channel when the transmit buffer register, the transmit address register, and the transmit shift register are all empty as indicated by the UART_STAT.TFI . The UART_IMSK.ETFI interrupt can be used to avoid ex- pensive polling of the UART_STAT.TEMT bit, when the UART clock or line drivers should be disabled after transmission has completed. W1C the UART_STAT.TFI bit to clear the interrupt request. In DMAoperation, the UART_IMSK.ETFI bits functionality might be preferred.                                                                                                                                                                |
| 5 (R/W)            | ETFI       | 0 Interrupt is masked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | EDTPTI     | Enable DMATXPeripheral Triggered Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.EDTPTI bit indicates enabling of the DMAcompletion interrupt request to be delayed until the data has left the UART completely. This bit is required for DMAtransmit operation only. If set, the UART can generate a DMAinterrupt request by the time the UART_STAT.TEMT bit goes high after the last DMAdata word is transmitted. When UART_IMSK.EDTPTI is set, usually the DMA_CFG.INT field is cleared to 00 in a STOP mode DMA. This set up suppresses the normal completion in- terrupt request, and the UART_STAT.TEMT event is signaled through theDMA controller and triggers the DMAinterrupt. If both ( DMA_CFG.INT not 00 and UART_IMSK.EDTPTI set), two interrupts are requested at the end of a STOP mode DMA. masked |
| 4 (R/W)            | EDTPTI     | 0 Interrupt is                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 3 (R/W)            | EDSSI      | Enable Modem Status Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.EDSSI bit indicates enabling of a modem status interrupt request on the same status interrupt channel when the UART_STAT.SCTS bit is set. This indicates UART_CTS pin re-assertion. Write-1- to-clear (W1C) the UART_STAT.SCTS bit to clear the interrupt request.                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 3 (R/W)            | EDSSI      | 0 Interrupt is masked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 3 (R/W)            | EDSSI      | 1 Interrupt is unmasked                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

Table 22-12: UART\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | ELSI       | Enable Line Status Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ELSI bit indicates that redirection of TX and RX interrupt requests to the status interrupt output of the UART by OR'ing them with the UART_STAT.OE , UART_STAT.PE , UART_STAT.FE , and UART_STAT.BI interrupt requests. Set this bit when no DMAchannel is associated with the UART. Enabling UART_IMSK.ELSI disables the RX/TX interrupt chan- nels and negates the UART_IMSK.EDTPTI bit. 0 Interrupt is masked |
| 1 (R/W)            | ETBEI      | Enable Transmit Buffer Empty Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ETBEI bit indicates generation of a TX interrupt request if the UART_STAT.THRE bit is set.                                                                                                                                                                                                                                                                                                              |
| 0 (R/W)            | ERBFI      | Enable Receive Buffer Full Interrupt Mask Status. If set (interrupt unmasked), the UART_IMSK.ERBFI indicates generation of an RX interrupt request if the UART_STAT.DR bit is set. 0 Interrupt is masked                                                                                                                                                                                                                                                                                               |

## Interrupt Mask Clear Register

The UART\_IMSK indicates interrupt mask status (unmasked if set, masked if cleared) of UART status interrupts. This register is not a data register. Instead it is controlled by the UART\_IMSK\_SET and UART\_IMSK\_CLR register pair. Writing ones to UART\_IMSK\_SET enables (unmasks) interrupt requests, and writing ones to UART\_IMSK\_CLR disables (masks) them. Reads from either register return the enabled bits. For more information, see the UART\_IMSK register description.

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000013_a8438c4bb3388521d13e7226dfda79b479164e6e00a6db3a7193e1a264478ea5.png)

31

0

30

0

29

0

28

0

27

0

26

0

25

0

Figure 22-14: UART\_IMSK\_CLR Register Diagram

Table 22-13: UART\_IMSK\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                               |
|--------------------|------------|-------------------------------------------------------|
| 9                  | ETXS       | Enable TX to Status Interrupt Mask Clear.             |
| 8 (R/W1C)          | ERXS       | Enable RX to Status Interrupt Mask Clear. 0 No action |
| 7                  | EAWI       | Enable Address Word Interrupt Mask Clear. 0 No action |
| (R/W1C)            |            | 1 Mask interrupt                                      |

24

0

23

0

22

0

21

0

20

0

19

0

18

0

17

0

16

0

Table 22-13: UART\_IMSK\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 6 (R/W1C)          | ERFCI      | Enable Receive FIFO Count Interrupt Mask Clear. 0 No action                         |
| 5 (R/W1C)          | ETFI       | Enable Transmission Finished Interrupt Mask Clear. 0 No action 1 Mask interrupt     |
| 4 (R/W1C)          | EDTPTI     | Enable DMATXPeripheral Triggered Interrupt Mask Clear. 0 No action 1 Mask interrupt |
| 3 (R/W1C)          | EDSSI      | Enable Modem Status Interrupt Mask Clear. 0 No action 1 Mask interrupt              |
| 2 (R/W1C)          | ELSI       | Enable Line Status Interrupt Mask Clear. 0 No action 1 Mask interrupt               |
| 1 (R/W1C)          | ETBEI      | Enable Transmit Buffer Empty Interrupt Mask Clear. 0 No action 1 Mask interrupt     |
| 0 (R/W1C)          | ERBFI      | Enable Receive Buffer Full Interrupt Mask Clear. 0 No action                        |
|                    |            | 1 Mask interrupt                                                                    |

## Interrupt Mask Set Register

The UART\_IMSK indicates interrupt request mask status (unmasked if set, masked if cleared) of UART status interrupts. This register is not a data register. Instead it is controlled by the UART\_IMSK\_SET and UART\_IMSK\_CLR register pair. Writing ones to UART\_IMSK\_SET enables (unmasks) interrupt requests, and writing ones to UART\_IMSK\_CLR disables (masks) them. Reads from either register return the enabled bits. For more information, see the UART\_IMSK register description.

Figure 22-15: UART\_IMSK\_SET Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000014_f0d0b617e1252bf4b5667cacad571a616ef05561588916717478aa199f65e1c9.png)

Table 22-14: UART\_IMSK\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                    |
|--------------------|------------|------------------------------------------------------------|
| 9                  | ETXS       | Enable TX to Status Interrupt Mask Set.                    |
| 8 (R/W1S)          | ERXS       | Enable RX to Status Interrupt Mask Set. 0 No action        |
| 7                  | EAWI       | 1 Unmask interrupt Enable Address Word Interrupt Mask Set. |
| (R/W1S)            |            | 0 No action 1 Unmask interrupt                             |

Table 22-14: UART\_IMSK\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                             |
|--------------------|------------|-------------------------------------------------------------------------------------|
| 6 (R/W1S)          | ERFCI      | Enable Receive FIFO Count Interrupt Mask Set. 0 No action                           |
| 5 (R/W1S)          | ETFI       | Enable Transmission Finished Interrupt Mask Set. 0 No action 1 Unmask interrupt     |
| 4 (R/W1S)          | EDTPTI     | Enable DMATXPeripheral Triggered Interrupt Mask Set. 0 No action 1 Unmask interrupt |
| 3 (R/W1S)          | EDSSI      | Enable Modem Status Interrupt Mask Set. 0 No action 1 Unmask interrupt              |
| 2 (R/W1S)          | ELSI       | Enable Line Status Interrupt Mask Set. 0 No action 1 Unmask interrupt               |
| 1 (R/W1S)          | ETBEI      | Enable Transmit Buffer Empty Interrupt Mask Set. 0 No action 1 Unmask interrupt     |
| 0 (R/W1S)          | ERBFI      | Enable Receive Buffer Full Interrupt Mask Set. 0 No action                          |
|                    |            | 1 Unmask interrupt                                                                  |

## Receive Buffer Register

The read-only UART\_RBR register is the UART's receive buffer. It is updated when there is pending data in the receive FIFO. Newly available data is signaled by the UART\_STAT.DR bit.

Figure 22-16: UART\_RBR Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000015_9da49d7941f88c0a66b2e48c68cf190099b211fe41117771a8215daab52f77b9.png)

Table 22-15: UART\_RBR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | 8-bit data.               |
| (R/NW)             |            |                           |

## Receive Shift Register

The read only UART\_RSR register which returns the content of the UART's receive shift register.

The frame data is moved into this shift register after polarity inversion, if any (including the native polarity inversion in the IrDA case).

In the case of the longest frame (MDB, with parity mode, and 8 bit data word-length), the start bit may be shifted out and not available for reading at the end of the frame reception. This register is NOT reset at the start of frame. If read, in the middle of a frame reception, data corresponding the previous frame may not have entirely shifted out (for example, the read data that have been read may NOT correspond entirely to the frame being received).

Because the UART is receiving only 1 stop bit, the UART\_RSR contains only 1 stop bit even if more than one stop bit is present in the actual transfer. This register may be considered as storing the 10 most recently received bits (taking into consideration the stop bit receive limitation above).

Figure 22-17: UART\_RSR Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000016_93d5a6e1b7a703aca3fc3a5a4dc6670439ef77057a16beae17f48c57a513acee.png)

Table 22-16: UART\_RSR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 9:0                | VALUE      | Contents of RSR.          |
| (R/NW)             |            |                           |

## Receive Counter Register

The UART\_RXCNT register returns the content of 16-bit counter in the UART receiver. This count is used for baud rate clock generation (the lower [15:0] is the count data).

Figure 22-18: UART\_RXCNT Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000017_40e312b9ba7d1fb54e883f3ba30d7fbcf2ad54b9f46c1fcd8d2b5fdb0b76fefd.png)

Table 22-17: UART\_RXCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | VALUE      | 16-bit Counter Value.     |
| (R/NW)             |            |                           |

## Scratch Register

The UART\_SCR registers contain 8-bit scratch pad data. These registers are used for general purpose data storage and do not control the UART hardware in any way.

Figure 22-19: UART\_SCR Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000018_7fc407807ab8ccc76d155b0d47521a1c14cc9f452cf64efb6b66af7e316ae9b0.png)

Table 22-18: UART\_SCR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | Stored 8-bit Data.        |
| (R/W)              |            |                           |

## Status Register

The UART\_STAT register contains the UART line status and UART modem status, as indicated by the current states of the UART's UART\_CTS pin and internal receive buffers. Writes to this register can perform write-one-toclear (W1C) operations on most status bits. Reading this register has no side effects.

Figure 22-20: UART\_STAT Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000019_46162603ec5500923d7cc2a65a8bb5bc903d7992907f35cad71b4c1c6288b4ed.png)

Table 22-19: UART\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 17 (R/NW)          | RFCS       | Receive FIFO Count Status. The UART_STAT.RFCS bit is set when the receive buffer holds more or equal entries than a certain threshold. The threshold is controlled by the UART_CTL.RFIT bit. If UART_CTL.RFIT is cleared, the threshold is four entries. If UART_CTL.RFIT is set, the threshold is seven entries. The UART_STAT.RFCS bit is cleared when the UART_RBR register is read sufficient times until the buffer is drained below the threshold. The UART_STAT.RFCS bit can trigger a status interrupt request if enabled by the UART_IMSK_SET.ERFCI bit. | Receive FIFO Count Status. The UART_STAT.RFCS bit is set when the receive buffer holds more or equal entries than a certain threshold. The threshold is controlled by the UART_CTL.RFIT bit. If UART_CTL.RFIT is cleared, the threshold is four entries. If UART_CTL.RFIT is set, the threshold is seven entries. The UART_STAT.RFCS bit is cleared when the UART_RBR register is read sufficient times until the buffer is drained below the threshold. The UART_STAT.RFCS bit can trigger a status interrupt request if enabled by the UART_IMSK_SET.ERFCI bit. |
| 17 (R/NW)          | RFCS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | RX FIFO has less than 4 (7) entries when RFIT=0 (1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 17 (R/NW)          | RFCS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | RX FIFO has at least 4 (7) entries when RFIT=0 (1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |

Table 22-19: UART\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 16 (R/NW)          | CTS        | Clear to Send. The UART_STAT.CTS bit holds the value (if UART_CTL.FCPOL set) or the complement value (if UART_CTL.FCPOL cleared) of the UART_CTS input pin. The UART_CTL.ACTS bit must be set to enable this feature. The core can read the value of the UART_STAT.CTS bit to determine whether the external device is ready to receive ( UART_STAT.CTS set) or if it is busy ( UART_STAT.CTS cleared). If UART_CTL.ACTS is cleared, the UART_TX handshaking protocol is disabled, and the UART transmits data as long as there is data to transmit, regard- less of the value of UART_STAT.CTS . When UART_CTL.ACTS is cleared, the software can pause transmission temporarily by setting the XOFF bit. Note that in loopback mode ( UART_CTL.LOOP_EN set), the UART_STAT.CTS bit is discon- nected from the UART_CTS input pin. Instead, the bit is directly connected to the UART_CTL.MRTS bit. | Clear to Send. The UART_STAT.CTS bit holds the value (if UART_CTL.FCPOL set) or the complement value (if UART_CTL.FCPOL cleared) of the UART_CTS input pin. The UART_CTL.ACTS bit must be set to enable this feature. The core can read the value of the UART_STAT.CTS bit to determine whether the external device is ready to receive ( UART_STAT.CTS set) or if it is busy ( UART_STAT.CTS cleared). If UART_CTL.ACTS is cleared, the UART_TX handshaking protocol is disabled, and the UART transmits data as long as there is data to transmit, regard- less of the value of UART_STAT.CTS . When UART_CTL.ACTS is cleared, the software can pause transmission temporarily by setting the XOFF bit. Note that in loopback mode ( UART_CTL.LOOP_EN set), the UART_STAT.CTS bit is discon- nected from the UART_CTS input pin. Instead, the bit is directly connected to the UART_CTL.MRTS bit. |
| 16 (R/NW)          | CTS        | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Not clear to send (External device not ready to receive)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 12 (R/W1C)         | SCTS       | Sticky CTS. The UART_STAT.SCTS bit is a sticky bit that is set when UART_STAT.CTS transitions from 0 to 1. The UART_STAT.SCTS bit is cleared by software with a W1C operation. This bit can trigger a line status interrupt request if enabled by the UART_IMSK_SET.EDSSI bit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Sticky CTS. The UART_STAT.SCTS bit is a sticky bit that is set when UART_STAT.CTS transitions from 0 to 1. The UART_STAT.SCTS bit is cleared by software with a W1C operation. This bit can trigger a line status interrupt request if enabled by the UART_IMSK_SET.EDSSI bit.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 12 (R/W1C)         | SCTS       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | CTS has not transitioned from low to high                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 12 (R/W1C)         | SCTS       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | CTS has transitioned from low to high                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 11 (R/NW)          | RO         | Reception On-going.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Reception On-going.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 11 (R/NW)          | RO         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | No data reception in progress                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 11 (R/NW)          | RO         | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Data reception in progress                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 10 (R/W1S)         | ADDR       | Address Bit Status. The UART_STAT.ADDR bit is used to mirror the address bit of the word in UART_RBR in multi-drop bus protocol, and is enabled only in MDBmode. The                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                | Address Bit Status. The UART_STAT.ADDR bit is used to mirror the address bit of the word in UART_RBR in multi-drop bus protocol, and is enabled only in MDBmode. The                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 10 (R/W1S)         | ADDR       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Address bit is low                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 10 (R/W1S)         | ADDR       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Address bit is high                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 22-19: UART\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W1C)          | ASTKY      | Address Sticky. The UART_STAT.ASTKY bit is used in multi-drop bus mode to indicate whether a peripheral is currently being addressed. This bit is a sticky ver- sion of the UART_STAT.ADDR bit and is set by hardware when setting the UART_STAT.ADDR bit. The UART_STAT.ASTKY bit can only be cleared by software with a write-one-to-clear (W1C) operation. With the UART_STAT.ASTKY bit set, words will be received irrespective of the UART_CTL.MOD bit or UART_STAT.ADDR bit selection. With the UART_STAT.ASTKY bit cleared, only address words ( UART_CTL.MOD bit set) will be received and words with UART_CTL.MOD bit cleared are ignored (not moved from the RSR to the RX FIFO) in MDBmode. The UART_STAT.ASTKY bit does not affect reception in non-MDB modes. |
| 8 (R/W1C)          | TFI        | Transmission Finished Indicator. The UART_STAT.TFI bit is a sticky version of the UART_STAT.TEMT bit. While UART_STAT.TEMT is automatically cleared by hardware when new data is written                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 7 (R/NW)           | TEMT       | 1 TEMT transition from 0 to 1 TSR and THR Empty. The UART_STAT.TEMT bit indicates that the UART_THR and UART_TAIP regis- ters and the UART_TSR register are empty. In this case, the program is permitted to write to the UART_THR and UART_TAIP registers twice without losing data. The UART_STAT.TEMT bit can also be used as indicator that pending UART transmission is completed. At that time, it is safe to disable the UART_CTL.EN bit or to three-state the off-chip line driver. 0 Not empty TSR/THR 1 TSR/THR Empty                                                                                                                                                                                                                                            |

Table 22-19: UART\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 5 (R/NW)           | THRE       | Transmit Hold Register Empty. The UART_STAT.THRE bit indicates that the UART transmit channel is ready for new data and software can write to the UART_THR and UART_TAIP registers. Writes to the UART_THR and UART_TAIP registers clear the UART_STAT.THRE . The bit is set again when the UART_THR and UART_TAIP registers are empty and ready to accept data.                                                                                                                                                                                                 |
| 5 (R/NW)           | THRE       | 0 Not empty THR/TAIP                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 4 (R/W1C)          | BI         | 1 Empty THR/TAIP Break Indicator. The UART_STAT.BI bit indicates that the first stop bit is sampled low and the entire data word, including parity bit, consists of low bits only. (This condition indicates that UART_RX was held low for more than the maximum word length.) The UART_STAT.BI bit is updated simultaneously with the UART_STAT.DR bit, that is, by the time the first stop bit is received or when data is loaded from the receive FIFO to the UART_RBR register. The bit is sticky and can be cleared by W1C operations. 0 No break interrupt |
| 3 (R/W1C)          | FE         | Framing Error. The UART_STAT.FE bit indicates that the first stop bit is sampled. This bit is updated simultaneously with the UART_STAT.DR bit, that is, by the time the first stop bit is received or when data is loaded from the receive FIFO to the UART_RBR register. The UART_STAT.FE bit is sticky and can be cleared by W1C operations. Note that invalid stop bits can be simulated by setting the UART_CTL.FFE bit. No error                                                                                                                           |
| 3 (R/W1C)          | FE         | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 3 (R/W1C)          | FE         | 1 Invalid stop bit error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2 (R/W1C)          | PE         | Parity Error. The UART_STAT.PE bit indicates that the received parity bit does not match the expected value. This bit is updated simultaneously with the UART_STAT.DR bit, that is, by the time the first stop bit is received or when data is loaded from the receive                                                                                                                                                                                                                                                                                           |
| 2 (R/W1C)          | PE         | 0 No parity error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 2 (R/W1C)          | PE         | 1 Parity error                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

Table 22-19: UART\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 1                  | OE         | Overrun Error. The UART_STAT.OE bit indicates that further data is received while the internal receive buffer was full. This bit is set when sampling the stop bit of the sixth data word. To avoid overruns, read the UART_RBR register in time. In DMAreceive mode, overruns are very unlikely to happen ever. After an overrun occurs, the UART_RBR and receive FIFO are protected from being overwritten by new data until the UART_STAT.OE bit is cleared by software. The content of the UART_RSR register is lost as soon as the overrun occurs. The UART_STAT.OE bit is sticky and can be cleared by W1C operations. | Overrun Error. The UART_STAT.OE bit indicates that further data is received while the internal receive buffer was full. This bit is set when sampling the stop bit of the sixth data word. To avoid overruns, read the UART_RBR register in time. In DMAreceive mode, overruns are very unlikely to happen ever. After an overrun occurs, the UART_RBR and receive FIFO are protected from being overwritten by new data until the UART_STAT.OE bit is cleared by software. The content of the UART_RSR register is lost as soon as the overrun occurs. The UART_STAT.OE bit is sticky and can be cleared by W1C operations. |
| 0 (R/NW)           | DR         | Data Ready. The UART_STAT.DR bit indicates that data is available in the receiver and can be read from the UART_RBR register. The bit is set by hardware when the receiver detects the first valid stop bit. The bit is cleared by hardware when the UART_RBR register is read.                                                                                                                                                                                                                                                                                                                                              | Data Ready. The UART_STAT.DR bit indicates that data is available in the receiver and can be read from the UART_RBR register. The bit is set by hardware when the receiver detects the first valid stop bit. The bit is cleared by hardware when the UART_RBR register is read.                                                                                                                                                                                                                                                                                                                                              |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | No new data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            | New data in RBR                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |

## Transmit Address/Insert Pulse Register

The UART\_TAIP register and the UART\_THR register share the same physical register, but UART\_TAIP has different effect than the UART\_THR register when UART\_TAIP is written to in MDB and UART modes.

In MDB mode, data written to the UART\_TAIP register is transmitted as an address frame (as with the UART\_CTL.MOD bit set).

In UART mode, a write to UART\_TAIP causes a pulse of value UART\_TAIP [7] for a duration of UART\_TAIP [6:0] x bit time. (There is additional inversion if the UART\_CTL.TPOLC bit is set).

Bit time is defined by the UART\_CLK register. The transmission of the pulse is followed by stop bit transmission as specified by the UART\_CTL.STB and UART\_CTL.STBH bits. This could be used for supporting line break command and inter-frame gap.

In IrDA mode, writes to UART\_TAIP is treated the same as writes to UART\_THR .

Accesses to the UART\_TAIP register have the same affects as the UART\_THR register with respect to the UART\_STAT.THRE , UART\_STAT.TEMT , and UART\_STAT.TFI flags.

Figure 22-21: UART\_TAIP Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000020_626e50dfcbb6f9aff64ac6220a2a9d945101a818e8dfac97cac3429d3c15b6fb.png)

Table 22-20: UART\_TAIP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | 8-bit data.               |
| (R/W)              |            |                           |

## Transmit Hold Register

The write-only UART\_THR register is the UART's transmit buffer. The UART\_STAT.THRE bit indicates whether data can be written to UART\_THR . Writes to this register automatically propagate to the internal UART\_TSR register as soon as UART\_TSR is ready. Then, transmit operation is initiated immediately.

Figure 22-22: UART\_THR Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000021_33f8a980b8ee1ab4d06bc05f9ea511ca56e72a6d32a9e61ac9f0d98cb0ad7660.png)

Table 22-21: UART\_THR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 7:0                | VALUE      | 8 bit data.               |
| (R/W)              |            |                           |

## Transmit Shift Register

The read only UART\_TSR register which returns the content of the UART's transmit shift register.

Figure 22-23: UART\_TSR Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000022_39ab13eb82689dbe99c5115652ac908aba6d9375d04b0113fbbb6d11ace1de1c.png)

Table 22-22: UART\_TSR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 10:0               | VALUE      | Contents of TSR.          |
| (R/NW)             |            |                           |

## Transmit Counter Register

The UART\_TXCNT read only register returns the content of 16-bit counter in the UART transmitter. This count is used for baud rate clock generation (the lower [15:0] is the count data).

Figure 22-24: UART\_TXCNT Register Diagram

![Image](25_Universal_Asynchronous_ReceiverTransmitter_(UART)_artifacts/image_000023_40e312b9ba7d1fb54e883f3ba30d7fbcf2ad54b9f46c1fcd8d2b5fdb0b76fefd.png)

Table 22-23: UART\_TXCNT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration   |
|--------------------|------------|---------------------------|
| 15:0               | VALUE      | 16-bit Counter Value.     |
| (R/NW)             |            |                           |