## 20   Serial Peripheral Interface (SPI)

The serial peripheral interface is an industry-standard synchronous serial link that supports communication with multiple SPI-compatible devices. The baseline SPI peripheral is a synchronous, four-wire interface consisting of two data pins, one device select pin, and a gated clock pin. The two data pins allow full-duplex operation to other SPI-compatible devices. Two extra (optional) data pins are provided on specific SPIs to support quad SPI operation. Enhanced modes of operation such as flow control, fast mode, and dual-I/O mode (DIOM) are also supported. In addition, a direct memory access (DMA) mode allows for transferring several words with minimal CPU interaction.

With a range of configurable options, the SPI ports provide a glueless hardware interface with other SPI-compatible devices in controller mode, target mode, and multi-controller environments. The SPI peripheral includes programmable baud rates, clock phase, and clock polarity. The peripheral can operate in a multi-controller environment by interfacing with several other devices, acting as either a controller device or a target device. In a multi-controller environment, the SPI peripheral uses open-drain outputs to avoid data bus contention. The flow control features enable slow target devices to interface with fast controller devices by providing an SPI ready pin which flexibly controls the transfers.

NOTE: SPI peripherals on the processor operate in the CLKO6 domain. For more details on CLKO6 programming, refer the Clock Generation Unit (CGU) chapter.

## SPI Features

The SPI module supports the following features:

- Full-duplex, synchronous serial interface
- 8, 16-bit and 32-bit word sizes
- Programmable baud rate, clock phase, and polarity
- Programmable interframe latency
- Flow control
- Support for Fast and DIOM modes
- SPI1 and SPI2 support quad mode and memory-mapped mode. Memory-mapped mode is only supported by SPI1/SPI2.

- Independent receive and transmit DMA channels
- Burst transfer mode for non-DMA write accesses

## SPI Functional Description

This section provides information on the function of the SPI module.

## Shift register functionality

The SPI is a shift register that serially transmits and receives data bits to or from other SPI devices. During an SPI transfer, data is simultaneously transmitted (shifted out serially) and received (shifted in serially). A serial clock line synchronizes shifting and sampling of the information on the two serial data lines.

## Controller/target functionality

During a data transfer, one SPI system acts as the link controller which controls the data flow. The other system acts as the target, which has data shifted into and out of it by the controller. Different devices can take turn being controllers, and one controller can simultaneously shift data into multiple targets (broadcast mode). However, only one target can drive its output to write data back to the controller at any given time. This rule must be enforced in the broadcast mode. Several targets can be selected to receive data from the controller in this mode. But, only one target can be enabled to send data back to the controller.

## Enhanced operating modes

SPI supports enhanced modes of operation like fast mode, DIOM, and Quad-SPI, and optional flow control. In fast mode, received data is sampled on the transmit edge instead of the standard receive edge, thus enabling a full-cycle path for the received data. In DIOM, both COTI and CITO are configured as input or output pins, and 2 bits are shifted in or out on each receive or transmit edge. In Quad-SPI mode, SPI\_D3:0 are configured as input or output pins and 4 bits are shifted in or out on each receive or transmit edge. A slower target can use flow control to stall a faster controller device.

## Single and multi-controller use

The SPI can be used in a single controller as well as multi-controller environment. The SPI\_COTI , SPI\_CITO , and the SPI\_CLK signals are all tied together in both configurations. SPI transmission and reception can be enabled simultaneously or individually, depending on SPI\_RXCTL and SPI\_TXCTL settings. In broadcast mode, several target can be enabled to receive, but only one target must be in transmit mode and driving the SPI\_CITO line.

## ADSP-2184x SPI Register List

The Serial Peripheral Interface (SPI) provides a full-duplex, synchronous serial interface, which supports both controller/target modes and multi-controller environments. The SPI baud rate and clock phase/polarities are

programmable, and it has integrated DMA channels for both transmit and receive data streams. A set of registers governs SPI operations. For more information on SPI functionality, see the SPI register descriptions.

Table 20-1: ADSP-2184x SPI Register List

| Name         | Description                            |
|--------------|----------------------------------------|
| SPI_CLK      | Clock Rate Register                    |
| SPI_CTL      | Control Register                       |
| SPI_DLY      | Delay Register                         |
| SPI_ILAT     | Masked Interrupt Condition Register    |
| SPI_ILAT_CLR | Masked Interrupt Clear Register        |
| SPI_IMSK     | Interrupt Mask Register                |
| SPI_IMSK_CLR | Interrupt Mask Clear Register          |
| SPI_IMSK_SET | Interrupt Mask Set Register            |
| SPI_MMRDH    | Memory Mapped Read Header              |
| SPI_MMTOP    | SPI Memory Top Address                 |
| SPI_RFIFO    | Receive FIFO Data Register             |
| SPI_RWC      | Received Word Count Register           |
| SPI_RWCR     | Received Word Count Reload Register    |
| SPI_RXCTL    | Receive Control Register               |
| SPI_TSEL     | Target Select Register                 |
| SPI_STAT     | Status Register                        |
| SPI_TFIFO    | Transmit FIFO Data Register            |
| SPI_TWC      | Transmitted Word Count Register        |
| SPI_TWCR     | Transmitted Word Count Reload Register |
| SPI_TXCTL    | Transmit Control Register              |

## ADSP-2184x SPI Interrupt List

Table 20-2: ADSP-2184x SPI Interrupt List

|   Interrupt ID | Name       | Description        | Sensitivity   |   DMA Channel |
|----------------|------------|--------------------|---------------|---------------|
|            230 | SPI0_TXDMA | SPI0 TX DMAChannel | Level         |            22 |
|            231 | SPI0_RXDMA | SPI0 RX DMAChannel | Level         |            23 |
|            232 | SPI0_STAT  | SPI0 Status        | Level         |               |
|            233 | SPI0_ERR   | SPI0 Error         | Level         |               |

Table 20-2: ADSP-2184x SPI Interrupt List (Continued)

|   Interrupt ID | Name           | Description              | Sensitivity   |   DMA Channel |
|----------------|----------------|--------------------------|---------------|---------------|
|            234 | SPI1_TXDMA     | SPI1 TX DMAChannel       | Level         |            24 |
|            235 | SPI1_RXDMA     | SPI1 RX DMAChannel       | Level         |            25 |
|            236 | SPI1_STAT      | SPI1 Status              | Level         |               |
|            237 | SPI1_ERR       | SPI1 Error               | Level         |               |
|            238 | SPI2_TXDMA     | SPI2 TX DMAChannel       | Level         |            26 |
|            239 | SPI2_RXDMA     | SPI2 RX DMAChannel       | Level         |            27 |
|            240 | SPI2_STAT      | SPI2 Status              | Level         |               |
|            241 | SPI2_ERR       | SPI2 Error               | Level         |               |
|            242 | SPI5_TXDMA     | SPI5 TX DMAChannel       | Level         |            59 |
|            243 | SPI5_RXDMA     | SPI5 RX DMAChannel       | Level         |            60 |
|            244 | SPI5_STAT      | SPI5 Status              | Level         |               |
|            245 | SPI5_ERR       | SPI5 Error               | Level         |               |
|            246 | SPI0_TXDMA_ERR | SPI0 TX DMAChannel Error | Level         |               |
|            247 | SPI0_RXDMA_ERR | SPI0 RX DMAChannel Error | Level         |               |
|            248 | SPI1_TXDMA_ERR | SPI1 TX DMAChannel Error | Level         |               |
|            249 | SPI1_RXDMA_ERR | SPI1 RX DMAChannel Error | Level         |               |
|            250 | SPI2_TXDMA_ERR | SPI2 TX DMAChannel Error | Level         |               |
|            251 | SPI2_RXDMA_ERR | SPI2 RX DMAChannel Error | Level         |               |
|            252 | SPI5_TXDMA_ERR | SPI5 TX DMAError         | Level         |               |
|            253 | SPI5_RXDMA_ERR | SPI5 RX DMAError         | Level         |               |

## ADSP-2184x SPI Trigger List

Table 20-3: ADSP-2184x SPI Trigger List Generators

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|          117 | SPI0_TXDMA | SPI0 TX DMAChannel | Edge          |
|          118 | SPI0_RXDMA | SPI0 RX DMAChannel | Edge          |
|          119 | SPI1_TXDMA | SPI1 TX DMAChannel | Edge          |
|          120 | SPI1_RXDMA | SPI1 RX DMAChannel | Edge          |
|          121 | SPI2_TXDMA | SPI2 TX DMAChannel | Edge          |
|          122 | SPI2_RXDMA | SPI2 RX DMAChannel | Edge          |

Table 20-3: ADSP-2184x SPI Trigger List Generators (Continued)

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|          123 | SPI5_TXDMA | SPI5 TX DMAChannel | Edge          |
|          124 | SPI5_RXDMA | SPI5 RX DMAChannel | Edge          |

Table 20-4: ADSP-2184x SPI Trigger List Receivers

|   Trigger ID | Name       | Description        | Sensitivity   |
|--------------|------------|--------------------|---------------|
|          149 | SPI0_TXDMA | SPI0 TX DMAChannel | Pulse         |
|          150 | SPI0_RXDMA | SPI0 RX DMAChannel | Pulse         |
|          151 | SPI1_TXDMA | SPI1 TX DMAChannel | Pulse         |
|          152 | SPI1_RXDMA | SPI1 RX DMAChannel | Pulse         |
|          153 | SPI5_TXDMA | SPI5 TX DMAChannel | Pulse         |
|          154 | SPI5_RXDMA | SPI5 RX DMAChannel | Pulse         |
|          155 | SPI2_TXDMA | SPI2 TX DMAChannel | Pulse         |
|          156 | SPI2_RXDMA | SPI2 RX DMAChannel | Pulse         |

## ADSP-2184x SPI DMA Channel List

Table 20-5: ADSP-2184x SPI DMA Channel List

| DMAID   | DMAChannel Name   | Description        |
|---------|-------------------|--------------------|
| DMA22   | SPI0_TXDMA        | SPI0 TX DMAChannel |
| DMA23   | SPI0_RXDMA        | SPI0 RX DMAChannel |
| DMA24   | SPI1_TXDMA        | SPI1 TX DMAChannel |
| DMA25   | SPI1_RXDMA        | SPI1 RX DMAChannel |
| DMA26   | SPI2_TXDMA        | SPI2 TX DMAChannel |
| DMA27   | SPI2_RXDMA        | SPI2 RX DMAChannel |
| DMA59   | SPI5_TXDMA        | SPI5 TX DMAChannel |
| DMA60   | SPI5_RXDMA        | SPI5 RX DMAChannel |

## SPI Block Diagram

The SPI Controller Block Diagram illustrates the blocks of the SPI module. The module is comprised of three primary parts:

- SPI core contains the receive and transmit FIFOs and their associated shift registers
- Control blocks contain the synchronizer and logic to control the data flow through the data pipelines

## · Register block

Figure 20-1: SPI Controller Block Diagram, Quad Mode

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000000_c3d43e8d2203ad8acc0db6c958ae5292edaa68945e7b555fed88244fb220996a.png)

## Transfer Protocol

The SPI module implements two channels that are independent of each other. The SPI module uses the SPI\_RXCTL and SPI\_TXCTL dedicated control registers to control these channels. Except in dual and quad modes, SPI can enable and use both channels simultaneously.

The SPI protocol supports four different combinations of serial clock phase and polarity. These combinations are selected through the SPI\_CTL.CPOL and SPI\_CTL.CPHA bits.

The SPI Transfer Protocol figures demonstrate the two basic transfer formats as defined by the CPHA bit. Two waveforms are shown for SPI\_CLK ; one for SPI\_CTL.CPOL =0 and the other for SPI\_CTL.CPOL =1. The diagrams can be interpreted as controller or target timing diagrams since the SPI\_CLK , SPI\_CITO , and SPI\_COTI pins are directly connected between the controller and the target. The SPI\_CITO signal is the output from the target (target transmission), and the SPI\_COTI signal is the output from the controller (controller transmission). The controller generates the SPI\_CLK signal. The SPI\_TS signal is the target device select input to the target from the controller. The diagrams represent an 8-bit transfer ( SPI\_CTL.SIZE =0) with the MSB first ( SPI\_CTL.LSBF =0). Any combination of the SPI\_CTL.SIZE and SPI\_CTL.LSBF bits is permissible. For example, a 16-bit transfer with the LSB first is another possible configuration.

The clock polarity and the clock phase could be identical for the controller device and the target device involved in the communication link. The transfer format from the controller can be changed between transfers to adjust for various requirements of a target device.

The SPI module uses the SPI\_CTL.ATSEL bit to determine when the SPI hardware or software control the SPI\_SEL[n] line. When SPI\_CTL.ATSEL =1, the target select line must be set to the polarity set in the SPI\_CTL.POLSEL field between each serial transfer. The actual behavior of SPI\_SEL[n] also depends on the parameters programmed into the SPI\_DLY register. The SPI hardware logic automatically controls this functionality. When SPI\_CTL.ATSEL =0, SPI\_SEL[n] can either remain active between successive transfers or be inactive. The software must control this activity through manipulation of the SPI\_TSEL register.

The SPI Transfer Protocol pair of figures illustrates the case when SPI\_CTL.ATSEL = 1 and the SPI\_SEL[n] line is inactive between frames. If ASSEL = 0, the SPI\_SEL[n] line can remain active between frames; however, the first bit is only driven when an active transition of SPI\_CLK occurs.

Figure 20-2: SPI Transfer Protocol for CPHA=0

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000001_f65beefb92b0d8f294d8e67039a670c1f9b4ea29bd31f395befede12678e885a.png)

Figure 20-3: SPI Transfer Protocol for CPHA=1

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000002_28665cff8370ccd4f5b5b14c43803cffacef16a73e16327d680797eaa2fef70a.png)

## Clock Considerations

The SPI\_CLK signal is a gated clock that is only active during data transfers, for the time of the transferred word. In normal mode, the number of active edges is equal to the number of bits to be transmitted or received. In

dual-I/O mode, it is half of the number of bits to be transmitted or received, and in quad-SPI mode it is one-fourth of the number. The clock rate can be derived using both even and odd dividers from SCLK.

For controller devices, the SPI uses the SPI\_CLK register value to determine the clock rate, whereas this value is ignored for target devices.

When the SPI controller is a controller, SPI\_CLK is an output signal. Conversely, when the SPI controller is a target, SPI\_CLK is an input signal. Peripheral devices ignore the SPI clock when the target select input is driven inactive. The SPI uses the SPI\_CLK signal to shift out and shift in the data driven onto the SPI\_CITO and SPI\_COTI lines. The data is always shifted out on one edge of the clock (the active edge) and sampled on the opposite edge of the clock (the sampling edge). Clock polarity and clock phase relative to data are programmable through the SPI\_CTL register and define the transfer format.

## Controlling Delay Between Frames

The SPI Timing with Lead and Lag Programming (Independent of SPI\_CTL.CPHA Setting) figure illustrates SPI timing using the SPI\_DLY.LEADX and SPI\_DLY.LAGX programming. The SPI uses the SPI\_DLY.LAGX bits to control the timing between the chip select ( SPI\_TS ) signal assertion and the first SPI\_CLK edge. The SPI uses the SPI\_DLY.LEADX bits to control the timing between the last SPI\_CLK edge and deassertion of the SPI\_TS signal. The lead and lag timing can be extended by a 1 SPI\_CLK duration to ease timing restrictions on the target device.

Figure 20-4: SPI Timing with Lead and Lag Programming (Independent of SPI\_CTL.CPHA Setting)

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000003_08fd014b40999bd5a9b7e16024ebdc56ddf586d25bd6f3436c1b6ad553baf313.png)

The SPI Timing with SPI\_DLY.STOP Programming (Independent of SPI\_CTL.CPHA Setting) figure illustrates SPI timing with STOP programming. The SPI module uses this timing to insert multiples of SPI\_CLK period delays between transfers. The SPI\_TS line is deasserted for the duration specified in the SPI\_DLY.STOP bit field, assuming the SPI\_CTL.POLSEL bit is configured for deassertion between transfers.

If the SPI\_DLY.STOP bit =0, the controller operates in a continuous mode . This mode causes an immediate start of the second word after the last bit is transferred from the first word. During this mode of operation, the chip select line is continuously asserted.

Figure 20-5: SPI Timing with SPI\_DLY.STOP Programming (Independent of SPI\_CTL.CPHA Setting)

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000004_9410055072d008ca69d1f380899eb612cf923f129b96424d277bea879be9a343.png)

When the SPI\_DLY.STOP bit =0 and initial conditions for a transfer are not met, the interface pauses before the next transfer. During this pause, the SPI uses the SPI\_CTL.POLSEL bit to determine the state of the chip select pin. The SPI uses the SPI\_DLY.LEADX and SPI\_DLY.LAGX bits to determine the timing between SPI\_CLK edges and the chip select line.

## Flow Control

In controller mode, the target device must drive the SPI\_RDY pin. The pin acts as an input signal. The target can deassert the SPI\_RDY pin to stop the controller from initiating any new transfer. If SPI\_RDY is deasserted in the middle of a transfer, the current transfer continues, and the next transfer will not start unless the target asserts the SPI\_RDY signal. Whenever the target deasserts SPI\_RDY and stalls the controller, the SPI controller goes into a waiting state, and the SPI\_STAT.FCS bit is set. When the target asserts SPI\_RDY , the SPI controller resumes operation, and the SPI\_STAT.FCS bit is cleared.

In target mode, the SPI\_RDY pin acts as an output signal. Flow control can be configured on either the TX channel or the RX channel. The SPI uses the SPI\_CTL.FCCH bit to control this configuration. If flow control is configured on the TX channel, as the SPI\_TFIFO status nears the empty condition, the SPI\_RDY pin is deasserted. If flow control is configured on the RX channel, as the SPI\_RFIFO status nears the full condition, the SPI\_RDY pin is deasserted. The SPI uses the SPI\_CTL.FCWM bits to control the FIFO status at which SPI\_RDY deassertion takes place. Flow control in target mode is purely based on the FIFO status and does not depend on the word counters.

The SPI Flow Control Timing in Controller Mode figure illustrates this timing.

Figure 20-6: SPI Flow Control Timing in Controller Mode.

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000005_144690b2b96695f639d791e94f14d4abd24f6bb2b709c8ac5d05abbf60358ab9.png)

## Target Select Operation

If the SPI is in target mode, SPI\_TS acts as the target select input. When SPI is enabled as a controller, SPI\_TS can serve as an error detection input for the SPI in a multi-controller environment. The SPI\_CTL.PTSE bit enables this feature. When SPI\_CTL.PTSE =1, the SPI\_TS input is the controller mode error input. Otherwise, SPI\_TS is ignored.

The SPI\_TS signal is an active-low signal. The controller asserts the signal during the transfer. The signal can be deasserted or remain asserted between transfers. When SPI\_TS is deasserted, SPI\_CLK and inputs are ignored, and outputs are three-stated.

The target select bits ( SPI\_TSEL.TSEL1 -SPI\_TSEL.TSEL7 ) are used in a multiple-target SPI environment. For example, if there are eight SPI devices in the system including a processor controller, the controller processor can support the SPI mode transactions across the other seven devices. This configuration requires only one controller processor in this multi-target environment.

For example, assume that the SPI of the processor is the controller. The SPI\_TSEL.TSEL1 -SPI\_TSEL.TSEL7 bits on the processor can be connected to the target select pin of each target device. In this configuration, the target select bits can be used in three ways. In cases 1 and 2, the processor is the controller and the seven microcontrollers or targets with SPI interfaces are targets. The processor can do one of the following:

1. Transmit to all seven SPI devices at the same time in a broadcast mode. Here, all target select bits are set.

2. Receive and transmit from one SPI device by enabling only one target SPI device at a time.
3. If all the targets are also processors, then the requester can receive data from only one processor at a time. (The functionality is enabled by clearing the SPI\_CTL.ECITO bit in the six other target processors.) The requester can transmit broadcast data to all seven at the same time. This CITO enabling feature is available in some other microcontrollers. Therefore, it is possible to use the CITO enabling feature with any other SPI device that includes this functionality.

Figure 20-7: Single-Controller, Multiple-Target Configuration

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000006_190ba15aad8efcfca15ccec3e11004dec72acdfc66a5de643e4e0a4637c0afb5.png)

## Beginning and Ending a Non-DMA SPI Transfer

The start and finish of a non-DMA SPI transfer depend on the following settings.

1. Whether the device is configured as a controller or a target.
2. The state of the SPI\_CTL.ATSEL bit, which selects between hardware and software control over SPI\_TSEL .

When SPI\_CTL.CPHA =0, the enabled target select outputs are driven active. However, the SPI\_CLK signal remains inactive for the first half of the first cycle of SPI\_CLK . For a target with SPI\_CTL.CPHA =0, the transfer starts as soon as the SPI\_TS input goes low.

When SPI\_CTL.CPHA =1, a transfer starts with the first active edge of SPI\_CLK for both target and controller devices. For a controller device, a transfer is complete after it sends the last data and simultaneously receives the last data bit. A transfer for a target device ends after the last sampling edge of SPI\_CLK . If SPI\_CTL.ATSEL =1, the hardware maintains responsibility for toggling SPI\_TS between frames. If SPI\_CTL.ATSEL =0, software controls the SPI\_TS line and can keep it active between frames.

The SPI\_STAT.RFE bit defines when the receive buffer can be read, indicating that SPI\_RFIFO is not empty. The SPI\_STAT.TFF bit defines when the transmit buffer can be written, indicating that the SPI\_TFIFO is not full. The end of a single word transfer occurs when the SPI\_STAT.RFE bit is cleared. The status indicates that a new word has been received and written into the receive FIFO. The SPI\_STAT.RFE bit remains cleared when the receive FIFO has valid data.

To maintain software compatibility with other SPI devices, the SPI\_STAT.SPIF bit is also available for polling. This bit can have a slightly different behavior from other commercially available devices.

In controller mode with the SPI\_CTL.ATSEL bit cleared, software manually asserts the required target select signal before starting the transaction. After all data transfers, software typically releases the target select line.

When the receive or transmit word counters are enabled in the SPI\_TXCTL or SPI\_RXCTL registers, the SPI generates a finish interrupt at the end of the transfer. It signals the end of all transfers related to that transaction.

## Transmit Operation in Non-DMA Mode

The transmit operation in non-DMA mode is enabled through the SPI\_TXCTL.TEN bit. It can be enabled independently from the receive operation, and the transmit channel can become the initiating channel based on the SPI\_TXCTL.TTI bit setting.

Transmit underrun is not possible in this mode, as no new transfer initiates unless the transmit FIFO is empty (in the case that SPI\_TXCTL.TTI =1). A receive overflow is detected when data from a new frame transfer replaces older data in a full receive FIFO. This event can occur if SPI\_TXCTL.TTI =1 and the receive channel is enabled in a non-initiating capacity.

An SPI transmit interrupt request is signaled once the transmit channel has been enabled and the transmit FIFO is not full. The SPI uses the SPI\_TXCTL.TDR bit setting to control the frequency of the interrupt request.

## Receive Operation in Non-DMA Mode

The receive operation in non-DMA mode is enabled through the SPI\_RXCTL.REN bit. It can be enabled independently from the transmit operation, and the receive channel can become the initiating channel based on the SPI\_RXCTL.RTI bit setting.

Receive overflow is not possible in this mode, as no new transfer initiates when the receive FIFO is full (in the case of SPI\_RXCTL.RTI =1). A transmit underrun can occur ( SPI\_TXCTL.TDU bit) when no valid data is in the SPI\_TFIFO register when a transfer is initiated. This event can occur if SPI\_RXCTL.RTI =1 and the transmit channel is enabled in a non-initiating capacity.

An SPI receive interrupt request is signaled once the receive channel has been enabled and there is data waiting to be read. The SPI uses the SPI\_RXCTL.RDR bit setting to control the frequency of the interrupt request.

## Dual I/O Mode

In Dual I/O mode, the SPI\_CITO and SPI\_COTI pins are configured to operate in the same direction which doubles bandwidth. The SPI uses the SPI\_CTL.SOTI bit to determine the order of bits on the pins. When set, the processor sends the first bit on the SPI\_COTI pin and the second bit on the SPI\_CITO pin. If the SPI\_CTL.SOTI bit is cleared, the order is reversed. Since dual I/O mode uses both pins to transmit or receive data, only one channel can be enabled, either transmit or receive. Flow control through the SPI\_RDY pin is supported. Interrupt request generation is unaffected by dual I/O mode. However, the interrupt service interval is reduced because the individual transfer latency is halved.

Changing to quad SPI mode must be done when the SPI is in a quiescent state.

Figure 20-8: Dual I/O Mode Transfer Protocol for CPHA=0, SOSI=1, 8-Bit Transfer, LSBF=0.

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000007_ae917b0d085f182614804dd8ff182c250b0d0961868a44aca3cd41fd7b963ee3.png)

Figure 20-9: Dual I/O Mode Transfer Protocol for CPHA=1, SOSI=0, 8-Bit Transfer, LSBF=0.

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000008_e7c5ea06ed837a137d08ce744dfcc99a9d9e2801ed88d6d0c46d775d6b250aed.png)

## Quad I/O Mode (SPI1/SPI2)

In quad SPI mode, the SPI\_CITO and SPI\_COTI pins, in tandem with the SPI\_D2 and SPI\_D3 pins, are configured to operate in the same direction. The SPI uses the SPI\_CTL.SOTI bit to determine the order of bits on the pins. When set, the processor sends:

- The first bit on the SPI\_COTI pin
- The second bit on the SPI\_CITO pin
- The third bit on the SPI\_D2 pin
- The fourth bit on the SPI\_D3 pin

If the SPI\_CTL.SOTI bit is cleared, the order is reversed. Since quad SPI mode uses all four pins to transmit or receive data, only one channel can be enabled as either transmit or receive. Flow control through the SPI\_RDY pin is supported. Interrupt generation is unaffected by quad SPI mode.

Changing to quad SPI mode must be done when the SPI is in a quiescent state.

While using dual or quad I/O mode for communicating with flash devices, program the SPI\_CTL.CPHA and the SPI\_CTL.CPOL bits =1. This programming avoids bus contention during read operations, because the flash device starts driving out the bits immediately after dummy cycles in read header.

Figure 20-10: Quad Mode Timing for CPHA=0, SOSI=1, 16-Bit Transfer, LSBF=0.

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000009_9a1dae4fc94291ee6773b2b4c0468a000a5bbbbaf3281125cd511ed72e9ef15b.png)

NOTE: The SPI does support quad SPI 8-bit transfer in target continuous mode of operation with an SCLK: SPI\_CLK ratio of less than 1:2. A minimum of 2 SCLK cycles is required between transfers in 8-bit quad SPI target mode with an SCLK: SPI\_CLK ratio of less than 1:2.

## Fast Mode

Fast mode is like the normal mode of operation when transmitting. When receiving, data is sampled at the next transmit edge allowing a full cycle of timing in the receive direction. This mode is valid in controller mode operation only. When the SPI operates in fast mode, the target drives the data for one full cycle.

Figure 20-11: SPI Transfer Protocol in Fast Mode for SPI\_CTL.CPHA = 0

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000010_d5197292cea0c772760aaf6f78bc7ceffecdc51afb25c13887618cecad0e1fba.png)

Figure 20-12: SPI Transfer Protocol in Fast Mode for SPI\_CTL.CPHA = 1

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000011_68cb9626a0d2e46c6e1864509871a91b245364d0ea0da2473f966a5d98e9a568.png)

## Memory-Mapped Mode (SPI1/SPI2)

The SPI supports direct memory-mapped read accesses from a SPI memory device, enabled by setting the SPI\_CTL.MMSE bit. This mode allows for direct execution of instructions from a SPI memory device without the need for a low-level software driver, as hardware handles all overhead tasks (for example, transmission of the read header, pin turnaround timing, and receive data sizing). The SPI features configurable options in the memory-mapped read header register ( SPI\_MMRDH ) to provide compatibility with a wide range of SPI memory devices.

In non-memory-mapped mode, the software is responsible for providing the command and required dummy words for the read response, whereas this is all handled by hardware when the SPI is in memory-mapped mode. The memory of the SPI device is accessible directly through reads of the processor address space. The read accesses can be code or data accesses in core mode or when using memory DMA (MDMA). These accesses allow code to execute directly from SPI memory devices (true eXecute-In-Place operations), and the contents can be cached to improve performance. It is not necessary to access the SPI data buffer registers nor poll status bits; however, the hardware does not support peripheral DMA accesses nor write operations to the SPI memory space.

The Types of Operations table is a comparison of the permitted operations in the non-memory-mapped and memory-mapped modes supported by the SPI controller.

Table 20-6: Types of Operations

| SPI Operation                                         | Non-Memory-Mapped Mode   | Memory-Mapped Mode   |
|-------------------------------------------------------|--------------------------|----------------------|
| Core data write                                       | Yes                      | No                   |
| Core data read                                        | Yes                      | Yes                  |
| Code fetch: Execute-In-Place (XIP)                    | No                       | Yes                  |
| Read/Write accesses using SPI peripheralDMA           | Yes                      | No                   |
| Read/Write accesses by other peripheral DMAchan- nels | No                       | No                   |

Table 20-6: Types of Operations (Continued)

| SPI Operation   | Non-Memory-Mapped Mode   | Memory-Mapped Mode   |
|-----------------|--------------------------|----------------------|
| MDMAread *      | No                       | Yes                  |
| MDMAwrite       | No                       | No                   |

NOTE: MDMA read* - Only standard bandwidth MDMA (MDMA0 and MDMA1) and Enhanced Bandwidth MDMA (MDMA2) can be used for accessing flash address space in memory mapped mode. The maximum bandwidth MDMA (MDMA3) cannot access flash address space.

## Memory-Mapped Description of Operation

Memory-mapped mode is enabled by setting the SPI\_CTL.MMSE bit. When enabled, the SPI (if ready) accepts the read requests through a dedicated on-chip target interface. The memory subsystem controller drives this dedicated interface through the SCB fabric.

In a typical scenario, the memory subsystem controller issues read requests to the fabric, and the fabric routes these requests to the target port of the SPI peripheral. The controller describes the read access using a number of parameters such as starting address, transfer size, and burst type. The SPI responds to this read access request when it is ready for a new transfer. It loads the opcode, a specified number of address bytes, and an optional mode byte into the transmit FIFO. The SPI memory state machine begins when both the transmit and receive channels of the SPI are enabled with:

- the transmit transfer initiation bit is set ( SPI\_TXCTL.TTI =1), and
- the receive initiation bit is cleared ( SPI\_RXCTL.RTI =0)

The SPI memory read sequence starts with the assertion of SPI\_SEL1 . If the SPI memory state machine is in the reset state, it looks for a command. The SPI hardware then sends the specific 8-bit read command (which can be optionally skipped), followed by the SPI memory read address. After this, a dummy period is inserted, in which a mode byte is optionally sent, and the pins are held or three-stated during the dummy clocking period.

NOTE: This read header is transmitted over the SPI standard protocol pins ( SPI\_CLK , SPI\_COTI , SPI\_CITO , SPI\_SEL1 ) or over the extended SPI protocol pins ( SPI\_CLK , SPI\_COTI , SPI\_CITO , SPI\_D2 , SPI\_D3 , SPI\_SEL1 ), based on the SPI\_MMRDH.CMDPINS , SPI\_MMRDH.ADRPINS , and SPI\_CTL.MIOM bit settings. SPI memory devices usually support communication in MSB-first mode. In dual mode, the SPI typically uses SPI\_CITO as IO1 and SPI\_COTI as IO0. In quad mode, the SPI typically uses SPI\_D3 pin as IO3, SPI\_D2 as IO2, SPI\_CITO as IO1, and SPI\_COTI as IO0.

When all I/O data pins are three-stated, the SPI continues clocking the SPI memory device, which drives out the data bits at the addressed location, until all bytes are received. The SPI hardware reads the data as configured by the SPI\_CTL.MIOM bit setting. Upon reception of the last byte, the SPI typically deasserts SPI\_SEL1 to prepare for the next requested read header.

Application code must ensure that the opcode sent is consistent with multiple I/O programming and that the parameters specified in the memory-mapped read header register are consistent with flash read access timing.

The SPI Memory-Mapped Register Operations Flow diagram shows how the fields of the SPI\_MMRDH register determine the read header while initiating transfers in memory-mapped mode.

Figure 20-13: SPI Memory-Mapped Register Operations Flow

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000012_4051fee220b5c887a9ccffa64e3f2635dccfae261f174493981c066b9f068f31.png)

## Memory-Mapped Architectural Concepts

In memory-mapped mode, the SPI accepts read requests through a dedicated on-chip target interface. The SPI (when ready) accepts these requests and begins the process of assembling the read header based on access attributes described in both the SPI\_MMRDH register and the internal bus request. After the read header transmission completes, a pin turnaround period is timed, and the receiver is enabled. The SPI continues clocking the SPI memory device until all bytes are received.

The SPI memory-mapped hardware accommodates various memory devices with different read timing. The capabilities include extra mode bits, flexible dummy period timing, and three-state control, as configured in the SPI\_MMRDH register.

The Memory-Mapped Protocol figure shows the protocol for the SPI controller in memory-mapped mode.

Figure 20-14: Memory-Mapped Protocol

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000013_6b97b1c254820b0b18691260c632f27c06ce9fd3c692888dc83158fac8d5e807.png)

As shown in the figure, the COMMAND field ( SPI\_MMRDH.OPCODE ) is transmitted upon assertion of the SPI\_SEL[n] signal. The SPI memory interprets this 8-bit value as a read command. Any 8-bit read opcode whose timing is compliant with the processor SPI and features provided by memory-mapped hardware is allowed, the most common being:

- Standard Read (0x03)
- Fast Read (0x0B)
- Fast Read Dual Output (0x3B)
- Fast Read Dual I/O (0x6B)
- Fast Read Quad Output (0xBB)
- Fast Read Quad I/O (0xEB)

NOTE: The SPI hardware does not validate the content of the SPI\_MMRDH.OPCODE field prior to transmitting.

## DMYSIZE (Number of Dummy Bytes)

When operating at a high clock frequency in multi-IO modes, most flash devices require some dummy clocks after the address bits. These dummy clock cycles allow the internal circuits of the device extra time for setting up the initial address. These bits specify the number of bytes separating address transmission and read data return.

The number of dummy cycles required varies per manufacturer, the read command used, and the SPI access time. The SPI hardware allows dummy cycles to be programmed in bytes in the SPI\_MMRDH.DMYSIZE field, the value of which is a function of the number of pins used to transmit the address ( SPI\_MMRDH.ADRPINS ), as shown in the Pins Used to Transmit the Address (ADRPINS) table.

Table 20-7: Pins Used to Transmit the Address (ADRPINS)

|                    | Dummy clock cycles                                                     | Dummy clock cycles                                                       | Dummy clock cycles                                              |
|--------------------|------------------------------------------------------------------------|--------------------------------------------------------------------------|-----------------------------------------------------------------|
| SPI_MMRDH.DMYSIZ E | ( SPI_MMRDH.ADRPINS =0, SPI_CTL.MIOM =x) Dummy bytes elapse over 1-pin | ( SPI_MMRDH.ADRPINS =1, SPI_CTL.MIOM =1) Dummy bytes elapse over 2- pins | ( SPI_MMRDH.ADRPINS =1, MIOM=2) Dummy bytes elapse over 4- pins |
| 000                | 0                                                                      | 0                                                                        | 0                                                               |
| 001                | 8                                                                      | 4                                                                        | 2                                                               |
| 010                | 16                                                                     | 8                                                                        | 4                                                               |
| 011                | 24                                                                     | 12                                                                       | 6                                                               |
| 100                | 32                                                                     | 16                                                                       | 8                                                               |
| 101                | 40                                                                     | 20                                                                       | 10                                                              |
| 110                | 48                                                                     | 24                                                                       | 12                                                              |
| 111                | 56                                                                     | 28                                                                       | 14                                                              |

This dummy clocking period allows the mode bits to be sent, the pins to be three-stated, and the pins to be turned around in preparation for the receive data.

## Memory-Mapped Read Accesses

The SPI hardware supports the most used read operations.

- Two standard SPI reads (read and read fast), which use the unidirectional SPI\_COTI and SPI\_CITO pins in addition to SPI\_SEL[n] and SPI\_CLK
- Four extended SPI multiple I/O reads: dual output, quad output, dual I/O, and quad I/O reads

The SPI Read Operations table and SPI Flash Fast Read Sequence figures summarize the types of read operations. Program each read operation in a way that is compatible with the description given in the SPI flash data sheet.

Table 20-8: SPI Read Operations

| Operation        | Read Command (Opcode)   | CMDPIN   |   ADRPIN | DMYSIZE   | Three-state   | Multiple I/O Mode   |   Data Pins |
|------------------|-------------------------|----------|----------|-----------|---------------|---------------------|-------------|
| Read             | 0x03                    | 1        |        1 | Zero      | No            | No                  |           1 |
| Fast Read        | 0x0B                    | 1        |        1 | Non-Zero  | Yes           | No                  |           1 |
| Dual Output Read | 0x3B                    | 1        |        1 | Non-Zero  | Yes           | Yes(IO0-1)          |           2 |
| Quad Output Read | 0x6B                    | 1        |        1 | Non-Zero  | Yes           | Yes(IO0-3)          |           4 |
| Dual I/O Read    | 0xBB                    | 1, 2     |        2 | Non-Zero  | Yes           | Yes (IO0-1)         |           2 |
| Quad I/O Read    | 0xEB                    | 1, 4     |        4 | Non-Zero  | Yes           | Yes (IO0-3)         |           4 |

Some memory devices also support word quad I/O read (0xE7) and octal quad I/O read (0xE3) operations. These operations require fewer dummy cycles than normal quad I/O read operations.

Figure 20-16: SPI Flash Fast Read (Dual Output) Sequence

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000014_5f180c98569d75e6968bb49afcd3572a32d86aa6cc947096a87f5504266b5d73.png)

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000015_1c14cf90642991bfb7b68cfec5c22e8d7f44e8fd51e91cb07263b0a662b4a323.png)

Figure 20-17: SPI Flash Fast Read (Dual I/O) Sequence

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000016_6d59ef33057821dedb7e50ff24838b89d540195765784d100ac1c65385cb824f.png)

Figure 20-18: SPI Flash Quad Output Read Sequence

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000017_1f7a5d8dec1a2bb3a000b2a7a4389cc1dba05b61bfb9b5ba56670a7d1a855bf1.png)

Figure 20-19: SPI Flash Quad I/O Read Sequence

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000018_66c43e9ade6e4817f4f4c1f307d835152da736eed426b444dd85f6d9b750902e.png)

SPI memory-mapped reads can be made cacheable in the core's internal memory by properly configuring the region as cacheable memory without bypass (see the related core's cache configuration documentation for details). In the figures, the number of read data bytes (N) is based on the following:

- For an instruction fetch by core (when in XIP mode); the number of instruction bytes to be fetched depends on the cache line size of the cache.

- For a data fetch by the core (data read), the number of data bytes to be fetched depends on the cache line size of the cache.

Although the minimum size of a memory-mapped data read transfer is 4 bytes, applications can fetch a single byte or a 2-byte data. (For example, it can fetch an unsigned char or short access in C code). In this case, only the required bytes are provided to the core and the other bytes are cached.

The on-chip memory subsystem controller provides a starting address for the burst and the SPI hardware issues this address as part of the read header. The address provided is N-byte aligned. For example, to read the 30th byte from SPI memory, then the typical address to provide is:

- 28 (0x0000\_001C) for a 32-bit cache line
- 24 (0x0000\_0018) for a 64-bit cache line
- 16 (0x0000\_0010) for a 128-bit cache line
- 0 (0x0000\_0000) for a 256-bit cache line

The read data is returned to the memory subsystem in the order provided by the SPI memory. There can be considerable delay for the expected data provided to the controller.

To minimize this delay, the wrap feature can be used where the memory subsystem provides the address of the critical word.

- For MDMA reads, the number of read data bytes (N) is always equal to 4 bytes. The MDMA read does not depend on the cache setting. For MDMA reads, limit the DMA\_CFG.MSIZE field to one, two or four bytes. The address provided by the memory subsystem controller to the SPI hardware is always 4-bytealigned.

## Memory-Mapped High-Performance Features

In addition to automating the SPI memory read accesses, the memory-mapped hardware also provides some features to improve SPI memory fetches and increase the system performance. The following sections describe these features.

## Merged Read Accesses

It is common for the memory subsystem to fetch two or more cache lines from consecutive addresses (the address sequencing is linear without any jumps). To take advantage of this situation, the SPI memory-mapped hardware provides a feature called merging. Enable merging by setting the SPI\_MMRDH.MERGE bit.

When enabled, the hardware compares the address of an incoming read request to the address of a request the SPI memory is actively servicing. It can decide to merge two accesses when the address for the second access is incremental. For example, if the first address of a 32-byte cache line fetch is 0x0000\_0000 and the second fetch is to address 0x0000\_0020, then these two accesses can be merged. Merging increases efficiency and overall fetch bandwidth by eliminating the read header for those accesses which only require continuation of the SPI clock.

## Wrap Around Accesses

Many SPI flash memory devices support wrapping which is used to enhance critical word fetching of cache lines. In this mode, the SPI device automatically wraps the read address to the base of a cache line once the end of the cache line is reached.

Wrap around accesses are enabled by setting the SPI\_MMRDH.WRAP bit.

Some flash devices require programs to send a Set Wrap command to place the device in wrap mode. Other flash devices provide a configuration register which must be programmed to set the flash in wrap mode. Since the SPI memory-mapped hardware does not support any write operations to flash, perform this step in non-memory-mapped mode ( SPI\_CTL.MMSE =0) by accessing the SPI registers.

Data access is limited to 8-byte, 16-byte, or 32-byte sections of flash page in wrap mode. The Arm core uses the Wrap 4 access (64-bit data) for L1 cache. The Arm core uses Wrap 4 and Wrap 8 accesses (64-bit data) for L2 cache. The cores use the Wrap 8 accesses for unaligned accesses. During the read request to the SPI memory-mapped hardware, the memory subsystem controller of the processor provides the address of a critical word instead of the line base. The read-data starts at the address specified in the instruction. Once it reaches the end boundary of the 8, 16, or 32-byte section, the output automatically wraps around to the beginning boundary to the line base address. The data fetch continues. It is not necessary to deassert the SPI SPI\_SEL[n] signal or resend the read header to wrap to the cache line base when servicing misaligned cache fill requests.

The Byte Sequence in Wrap Modes table shows byte sequences in various wrap modes.

Table 20-9: Byte Sequence in Wrap Modes

|   Starting Address | 8-Byte Wrap (cache_line = 8 byte)   |
|--------------------|-------------------------------------|
|                  0 | 0-1-2- . . . -6-7                   |
|                  1 | 1-2- 3-. . . -7-0                   |
|                  7 | 7-0-1- . . . -5-6                   |
|                 15 | 15-8-9- . . . -13-14                |
|                 31 | 31-24-25-. . . .-29-30              |

The burst with wrap feature allows applications to fetch a critical address quickly. Applications then fill the cache afterwards within a fixed length (8/16/32-byte) of data without issuing multiple read commands. Certain applications can benefit from this feature to improve cache fill efficiency and overall performance of system code execution.

NOTE: Do not use the merge and wrap feature together. Using wrap bursts can unintentionally disable merging (merging cannot occur for unaligned wrapping bursts). A wrap burst can start fetching data words in the middle of the cache line and cannot be merged with the next access.

## Execute-In-Place (XIP, SPI2 only)

Execute-In-Place, known as XIP , permits software code to execute directly from an SPI flash device rather than downloading the code and executing it out of RAM. XIP , also known as Command Skip mode, is a general term and can be applied to fetching data as well.

There is a difference between XIP mode and standard mode. In XIP mode, after the SPI memory device is selected (CS# =LOW), the memory device does not decode the first input byte as command code. Instead, it expects the read header to directly start with address bytes. In standard mode, the memory decodes the first input byte it receives as a command code.

The XIP mode dramatically reduces random access time for applications that require fast code execution without shadowing the memory content on a RAM. The SPI memory-mapped hardware provides a control bit, SPI\_MMRDH.CMDSKIP to skip the command from read header.

Some SPI memory devices require configuration of their control register to enable the XIP mode of operation, using the non-memory-mapped mode of the processor SPI. Typically, during the dummy cycle period, the mode bits are used to confirm the XIP operation and the SPI\_MMRDH.MODE field must be set appropriately. A dummy memory-mapped access may be needed before setting the SPI\_MMRDH.CMDSKIP bit to set the SPI memory device in Command Skip mode.

For more details about how to configure SPI memories into XIP mode, refer to the device data sheet.

NOTE: When configuring the flash to XIP mode from the SHARC+ core, ensure that the routine that configures flash to XIP is not routed through the L2CC. This is accomplished by first configuring the flash to XIP mode, then enabling the L2CC from the core.

## Memory-Mapped Mode Error Status Bits

The SPI memory-mapped hardware provides bits in the SPI\_STAT register to report errors. It provides these bits for notification only and their state has no effect on SPI operations. The status register bits are sticky. A W1C (write-1-to-clear) operation clears the bits.

- Memory-Mapped Write Error ( SPI\_STAT.MMWE ). This bit is set (=1) if an attempt is made to write to address space that is reserved for memory-mapped SPI memory. The SPI memory-mapped hardware does not support automated write access to SPI memory space.
- Memory-Mapped Read Error ( SPI\_STAT.MMRE ). This bit is set (=1) if an attempt is made to read address space reserved for memory-mapped SPI memory while memory mapping is disabled ( SPI\_CTL.MMSE =0).
- Memory-Mapped Access Error ( SPI\_STAT.MMAE ). This bit is set (=1) if an attempt is made to access either the TX or RX FIFO while memory-mapped access of SPI memory is enabled. In this case, attempts to communicate with the SPI device using legacy methods are blocked and receive fabric reports an error. Legacy methods include any direct access made to the TX and RX FIFOs, whether by DMA or processor MMR.
- Memory-Mapped Write Error Mask ( SPI\_CTL.MMWEM ) bit specifies whether an error response is returned to the fabric on write attempts to address space that is reserved for memory-mapped SPI memory reads. Regardless of whether a write error response is masked using this bit, the memory-mapped write error ( SPI\_STAT.MMWE ) sticky notification bit is still set.

NOTE: Unlike other bits in the SPI\_STAT register, these memory-mapped mode error bits do not have associated bits in the SPI interrupt mask ( SPI\_IMSK ) and SPI interrupt condition ( SPI\_ILAT ) registers.

The memory-mapped top register ( SPI\_MMTOP ) is used to specify the upper limit of the SPI memory address. The memory-mapped accesses to SPI memory addresses equal to or above this range are considered illegal. The accesses are blocked, and a bus error response is generated.

This register is useful to block the invalid SPI memory address accesses. Some SPI memory vendors do not clearly specify (guarantee) that overrange address bits are ignored (address spaces can be wrapped).

## Memory-Mapped Programming Guidelines

Setting the SPI\_CTL.MMSE bit enables SPI memory-mapped mode. When enabled, the SPI interface is forced to be consistent with SPI memory requirements regardless the settings of certain control bits. The following tables specify typical settings for configuring the SPI in memory-mapped mode:

Table 20-10: SPI Control (SPI\_CTL) Register

| Bits                                                | Typical values to set   | Description                                | Comments                                                                                  |
|-----------------------------------------------------|-------------------------|--------------------------------------------|-------------------------------------------------------------------------------------------|
| SPI_CTL.MODE                                        | 1                       | Controller mode enable                     |                                                                                           |
| SPI_CTL.PTSE                                        | 0                       | Protected target select en- able           |                                                                                           |
| SPI_CTL.ODM                                         | 0                       | Open-drain mode enable                     |                                                                                           |
| SPI_CTL.CPHASPI_ CTL.CPOL                           | 0-0 or 1-1              | SPI mode of communica- tion                | Flash dependent, usually SPI flash supports mode-0 (CPHA=CPOL=0) and mode-3 (CPHA=CPOL=1) |
| SPI_CTL.ATSEL                                       | 1                       | Hardware target select pin control         |                                                                                           |
| SPI_CTL.POLSEL                                      | 1                       | Target select asserted be- tween transfers |                                                                                           |
| SPI_CTL.ECITO                                       | 1                       | CITO pin enable                            |                                                                                           |
| SPI_CTL.SIZE                                        | 2                       | 32-bit transfer size                       |                                                                                           |
| SPI_CTL.LSBF                                        | 0                       | MSB bit first mode                         | Flash dependent, usually SPI flash commu- nicates in MSB bit first mode                   |
| SPI_CTL.FCEN SPI_CTL.FCCH SPI_CTL.FCPL SPI_CTL.FCWM | 0                       | Hardware flow control re- lated bits       |                                                                                           |
| SPI_CTL.FMODE                                       | 1                       | Fast mode enable                           | Typically set to 1 for full cycle timing, 0 only works at low speed                       |
| SPI_CTL.SOTI                                        | 0                       | Treat SPI_COTI pin as IO0 pin.             |                                                                                           |

Table 20-11: SPI Receive Control Register

| Bits            |   Typical values to set | Description                         |
|-----------------|-------------------------|-------------------------------------|
| SPI_RXCTL.REN   |                       1 | Receive channel enable              |
| SPI_RXCTL.RTI   |                       0 | Receive transfer initiation disable |
| SPI_RXCTL.RWCEN |                       0 | Receive word counter disable        |

Table 20-11: SPI Receive Control Register (Continued)

| Bits           |   Typical values to set | Description                            |
|----------------|-------------------------|----------------------------------------|
| SPI_RXCTL.RDR  |                       0 | Receive data request disable           |
| SPI_RXCTL.RDO  |                       0 | Discard incoming data if RFIFO is full |
| SPI_RXCTL.RRWM |                       0 | Receive FIFO regular watermark         |
| SPI_RXCTL.RUWM |                       0 | Receive FIFO urgent watermark disable  |

Table 20-12: SPI Transmit Control Register

| Bits            |   Typical values to set | Description                            |
|-----------------|-------------------------|----------------------------------------|
| SPI_TXCTL.TEN   |                       1 | Transmit channel enable                |
| SPI_TXCTL.TTI   |                       1 | Transmit transfer initiation disable   |
| SPI_TXCTL.TWCEN |                       0 | Transmit word counter disable          |
| SPI_TXCTL.TDR   |                       0 | Transmit data request disable          |
| SPI_TXCTL.TDU   |                       0 | Send last word when TFIFO is empty     |
| SPI_TXCTL.TRWM  |                       0 | Transmit FIFO regular watermark        |
| SPI_TXCTL.TUWM  |                       0 | Transmit FIFO urgent watermark disable |

Table 20-13: SPI DLY Control Register

| Bits          |   Typical Values to Set | Description                    | Comments See Flash data sheet for CS (for example, SSEL) Timing Specs   |
|---------------|-------------------------|--------------------------------|-------------------------------------------------------------------------|
| SPI_DLY.LAGX  |                       1 | Extended lag timing            |                                                                         |
| SPI_DLY.LEADX |                       1 | Extended lead timing           |                                                                         |
| SPI_DLY.STOP  |                       3 | Stop bit between the transfers | Can be set to 1 at lower SPI clock frequencies.                         |

The multiple I/O mode ( SPI\_CTL.MIOM ) bits are partially ignored:

- The command (opcode) is transmitted using either just one or the number of pins specified by the SPI\_CTL.MIOM bits, depending on SPI\_MMRDH.CMDPINS bit setting.
- The address is then transmitted using either just one or the number of pins specified by the SPI\_CTL.MIOM bits, depending on SPI\_MMRDH.ADRPINS bit setting.
- The data is always read with the number of pins specified by the SPI\_CTL.MIOM bits.

NOTE: Set the SPI module enable bits SPI\_CTL.EN last after configuring all registers.

Use the following programming guidelines for memory-mapped mode:

- The SPI memory-mapped hardware does not check the flash status before initiating the access. It assumes that SPI memory is always able to respond to a read access. Before enabling memory-mapped mode (for example, setting the SPI\_CTL.MMSE bit) ensure that SPI flash is ready for a read access. When using non-memorymapped mode, a write-complete status can be examined prior to enabling the SPI in memory-mapped mode. (See the write in progress bit in the SPI flash memory status register.) Also, immediately after initial power-up, SPI memory devices can be inaccessible for a vendor-specified period.
- When SPI is enabled in memory-mapped mode, attempts to communicate with the SPI device using legacy methods are blocked. Legacy methods include any direct access made to the transmit or receive FIFOs, whether initiated by DMA or by a processor MMR access.
- To use some of the features offered by SPI memory devices, programs can first configure the SPI memory device by setting its control word or sending some commands. Because SPI memory-mapped hardware does not allow any type of SPI write operations, configure the SPI in non-memory-mapped mode prior to enabling memory-mapped mode.
- The memory-mapped hardware does not interpret the opcode. It does not check the validity of the timing that is specified in the SPI\_MMRDH register for a particular opcode. Programs must set the fields of the SPI\_MMRDH register to be consistent with the read-type selected.
- When the core requests the data or code fetch, the memory-mapped transfer depends on cache settings. The cache configuration register in the SPI memory device must be appropriately configured before enabling memory-mapped mode. Some of the high performance modes like merge, wrap, and transfer size depend on cache parameters.
- SPI memory-mapped MDMA reads do not support wrapping. For MDMA reads, limit the DMA\_CFG.MSIZE field to 1 byte, 2 bytes or 4 bytes.
- There is not always tool support to change the SPI memory-mapped hardware setting or cache settings on-the-fly. Changing these settings can optimize the performance of code that accesses SPI memory in memory-mapped mode. It is expected that the SPI memory, SPI peripheral, and cache are programmed to one specific set of control settings for the whole application. Profiling or benchmarking of the actual application can be done to find the setting that works best.

## SPI Interrupt Signals

The SPI controller supports three types of interrupt request signals that correspond to data, status, and error conditions.

## Data Interrupts

The SPI target supports two data interrupt channels - receive and transmit. These interrupt signals are multiplexed into the DMA request lines. Since the target interfaces with separate read and write interfaces with DMA, the read and write data interrupts are independent. When the DMA channels are not used, the interrupts are routed directly to the system event controller. The interrupts occupy the same vector locations as the corresponding DMA channels.

Each of the data interrupt requests can be individually controlled. Program the SPI\_RXCTL.RDR and SPI\_TXCTL.TDR bit fields for receive and transmit, respectively. When receive is enabled, the RX interrupt request is issued whenever there is data available in the receive data path for reading. (The event occurs according to the SPI\_RXCTL.RDR bit setting.) When transmit is enabled, the TX interrupt request is issued whenever the transmit data path can be written. (The event occurs according to the SPI\_TXCTL.TDR setting.) DMA data interrupts are compatible with second-generation DMA to incorporate urgent data requests and transfer finish interrupt requests apart from the usual data request interrupts. T ransmit interrupt requests operate independently from the word counter-value in the SPI\_TWC register.

## Status Interrupts

The SPI controller supports several status interrupt requests to indicate different conditions of the receiver and transmitter. All status interrupt requests can be masked. Status interrupt requests are signaled directly through a single SPI status IRQ line. The line cannot be combined with the SPI error IRQ line for some processors. The SPI Status Interrupts table describes the status interrupt requests that are available for the SPI controller.

Table 20-14: SPI Status Interrupts

| SPI_STAT Bit   | Description                                                                                                                                                                                                                                                                                                                                      |
|----------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPI_STAT.RUWM  | Receive FIFO urgent watermark interrupt request. Issued when the level of the RFIFO breaches the water- mark set in the SPI_RXCTL.RUWM field. It is cleared when the level of the RFIFO reaches the watermark set in the SPI_RXCTL.RRWM field. If the RX channel is configured in DMAmode, SPI_RXCTL.RUWM is multiplexed with the data request.  |
| SPI_STAT.TUWM  | Transmit FIFO urgent watermark interrupt request. Issued when the level of the TFIFO breaches the water- mark set using the SPI_TXCTL.TUWM bit. It is cleared when the level of the TFIFO reaches the watermark set in the SPI_TXCTL.TRWM field. If the TX channel is configured in DMAmode, SPI_STAT.TUWM is multiplexed with the data request. |
| SPI_STAT.TS    | Transmit start interrupt request. Issued when the start of a transmit burst is detected by loading of the SPI_TWC register with the contents of the SPI_TWCR register.                                                                                                                                                                           |
| SPI_STAT.RS    | Receive start interrupt request. Issued when the start of a receive burst is detected by the loading of SPI_RWC with the contents of SPI_RWCR .                                                                                                                                                                                                  |
| SPI_STAT.TF    | Transmit finish interrupt request. Issued when a transmit burst completes ( SPI_TWC decrements to zero).                                                                                                                                                                                                                                         |
| SPI_STAT.RF    | Receive finish interrupt request. Issued when a receive burst completes ( SPI_RWC decrements to zero).                                                                                                                                                                                                                                           |

## Error Conditions

The SPI controller supports interrupt requests upon several different error conditions. All interrupt requests are maskable. The individual error indications combine into a single SPI error IRQ signal, which can be multiplexed on some processors with the aggregated SPI status IRQ signal. The SPI Error Interrupts table details the possible error indications.

Error conditions arise depending on which of the channels (transmit or receive) are enabled. If a channel is disabled, all errors related to it are ignored. When both channels are enabled, errors from both channels are enabled.

Table 20-15: SPI Error Interrupts

| Bit          | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
|--------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| SPI_STAT.MF  | Mode fault. Signaled when another device also tries to be a controller in a multi-controller system and drives the SPI_TS input low. This error is signaled in controller mode operation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| SPI_STAT.TUR | Transmission error. Signaled when an underflow condition occurs on the transmit channel. This event occurs when a new transfer starts but SPI_TFIFO is empty. This error does not occur in controller transmit initiating mode since SPI_TFIFO Not Empty is one of the conditions for transfer initiation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| SPI_STAT.ROR | Reception error. Signaled when an overflow condition occurs on the receive channel. This event occurs when a new data word is received, but the SPI_RFIFO is full. This error condition does not occur in controller receive initiating mode since SPI_RFIFO Not Full is one of the conditions for transfer initiation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| SPI_STAT.TC  | Transmit collision error. Signaled when loading data to the transmit shift register happens near the first transmitting edge of SPI_CLK . In target mode of operation, the SPI controller is unaware of when the next transfer starts. Loading of data to the transmit shift register can happen just after the transmitting edge. This event results in the setup time not being met for the first bit transmitted. The transmitted data is corrupt. In SPI_CTL.CPHA 1 mode, the first SPI_CLK edge is taken as the first transmitting edge. If SPI_CTL.CPHA =0, then the last SPI_CLK edge of the last transmission ( SPI_CTL.POLSEL =1) or target select deassertion ( SPI_CTL.POLSEL =0) is taken as the first transmitting edge. This error is signaled only in the target mode of operation. In controller mode of operation, loading of data happens before the first transmitting edge of SPI_CLK . |

## SPI Programming Concepts

The following sections provide general programming guidelines and procedures.

## Programming Guidelines

It is acceptable to program SPI\_RXCTL and SPI\_TXCTL registers after programming the SPI\_CTL register. However, program the initiating mode register and its counter-register, if enabled, after the non-initiating mode register. For example, if transmit is the initiating mode and receive is the non-initiating mode, then program the SPI\_RXCTL and SPI\_RWC registers before the SPI\_TXCTL and SPI\_TWC registers. If enabling both transmit and receive in initiating mode, enable the SPI\_CTL register after programming both the SPI\_RXCTL and SPI\_TXCTL registers.

These programming guidelines prevent SPI from starting a transfer when SPI registers are not fully programmed. Other ways of programming are also allowed when the initiating conditions prevent the start of communication until after programming of SPI registers is complete.

Avoid data corruption when changing the SPI module configuration. Do not change the configuration during a data transfer. Additionally, change the clock polarity only when no target is selected. However, an exception to this rule exists. When an SPI communication link consists of a single controller and target, SPI\_CTL.ATSEL = 0. The target select input of the target is permanently tied low. In this case, the target is always selected. Avoid data corruption by enabling the target only after both the controller and target devices are configured.

The module supports 8, 16-bit and 32-bit word sizes. To ensure correct operation, configure both the controller and target with the same word size.

## Controller Operation in Non-DMA Modes

This section describes the operation of the SPI as a controller in non-DMA mode.

1. Write to the SPI\_TSEL register, setting one or more of the SPI select enable bits. This operation ensures that the desired targets are properly deselected while the controller is configured.
2. The SPI\_RXCTL.RTI and SPI\_TXCTL.TTI bits determine the SPI initiating mode. The initiating mode defines the primary transfer channel, and also the initiating condition for the transfer.
3. Write to the SPI\_CLK , SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers. This operation enables the device as a controller and configures the SPI system. It specifies the transfer modes and channels, appropriate word length, transfer format, baud rate, and other control information.
4. ADDITIONAL INFORMATION: If SPI\_RXCTL.RTI is enabled and SPI\_TXCTL.TTI is not, write to the SPI\_RXCTL register after writing into SPI\_CTL , SPI\_TXCTL , and SPI\_TFIFO registers to prevent a transmit underrun for the first transfer.
4. If SPI\_CTL.ATSEL =0, activate the desired targets by clearing one or more of the SPI\_TSEL flag bits. Otherwise, the SPI hardware performs target activation.
5. The SPI controller then generates the programmed clock pulses on SPI\_CLK and simultaneously shifts data out of SPI\_COTI while shifting data in from SPI\_CITO . Before a shift, the shift register is loaded with the contents of the SPI\_TFIFO register. At the end of the transfer, the contents of the shift register are loaded into SPI\_RFIFO .
6. Whenever the initiating conditions are satisfied, the SPI continues to send and receive words. If the transmit buffer remains empty or the receive buffer remains full, the device operates according to the states of the SPI\_TXCTL.TDU and SPI\_RXCTL.RDO bits.
7. It is possible to program a secondary channel in addition to the initiating channel. This feature allows usage of available channel resources for receives or transmits simultaneously with the initiating channel.

## Target Operation in Non-DMA Modes

When a device is enabled as a target in a non-DMA mode, a transition of the SPI\_TS select signal to the active state (low) triggers the start of a transfer. Or the first active edge of SPI\_CLK triggers the start, depending on the state of SPI\_CTL.CPHA bit. The interface operates in the following manner.

1. The core writes to the SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers. The operation defines the mode of the serial link to be the same as the mode setup in the SPI controller.
2. To prepare for the data transfer, the core writes data to be transmitted into SPI\_TFIFO .
3. Once the SPI\_TS falling edge is detected, the target starts sending data on active SPI\_CLK edges and sampling data on inactive SPI\_CLK edges.
4. Reception or transmission continues until SPI\_TS is released or until the target has received the proper number of clock cycles.

5. The target device continues to receive or transmit with each new falling edge transition on SPI\_TS or active SPI\_CLK edge. If the transmit buffer remains empty or the receive buffer remains full, the device operates according to the states of the SPI\_TXCTL.TDU and SPI\_RXCTL.RDO bits.

## Configuring DMA Controller Mode

The SPI interface supports a write DMA channel and a read DMA channel. It can use these functions individually or in a lock-step manner in duplex mode ( SPI\_TXCTL.TTI = SPI\_RXCTL.RTI =1).

1. Write to the appropriate DMA registers to enable the SPI DMA channel and to configure the necessary work units, access direction, word count, and so on.
2. Write to the SPI\_TSEL register, setting one or more of the SPI flag select bits.
3. Write to the SPI\_CLK and SPI\_CTL registers, enabling the device as a controller and configuring the SPI system by specifying the appropriate word length, transfer format, baud rate, and so forth.
4. Write to SPI\_RXCTL to configure SPI controller receive mode, or write to SPI\_TXCTL to configure SPI controller transmit mode.
5. Finally, write to the SPI\_RXCTL.REN bit to enable the receive channel, or write to SPI\_TXCTL.TEN to enable the transmit channel.
6. When the SPI\_RXCTL.RTI bit is enabled, a receive transfer is initiated upon enabling SPI\_CTL.EN bit. If the receive word counter is enabled ( SPI\_RXCTL.RWCEN ), then the SPI\_RWC register must be non-zero for a transfer to initiate.
7. ADDITIONAL INFORMATION: If enabling both receive and transmit DMA channels, but not enabling SPI\_TXCTL.TTI , write to the SPI\_RXCTL register after writing the SPI\_CTL and SPI\_TXCTL registers. In this way, a transmit underrun can be prevented for the first transfer. Subsequent transfers are initiated as the SPI reads data from the receive shift register and writes to the SPI receive FIFO. The SPI then requests a write from DMA to memory. Upon a DMA grant, the DMA engine reads a word from the SPI receive FIFO and writes to memory. New requests continue to be initiated as long as the receive FIFO does not fill up, when SPI\_RWC does not become zero while SPI\_RXCTL.RWCEN =1.
7. When SPI\_TXCTL.TTI is enabled, the SPI controller requests DMA reads from memory when there is space for more data in the transmit pipe. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO. As long as transmit data is available in the FIFO, and the SPI\_TWC register is non-zero when SPI\_TXCTL.TWCEN =1, the SPI continues to initiate transfers until disabled.
8. When both the SPI\_TXCTL.TTI and SPI\_RXCTL.RTI bits are enabled, the SPI controller requests a DMA read from memory. However, there must be space for more data in the transmit pipe and the number of words written into the SPI must be less than SPI\_TWC if SPI\_TXCTL.TWCEN =1. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO.

ADDITIONAL INFORMATION: As the SPI writes data from the transmit FIFO into the transmit shift register, it initiates a transfer on the SPI link. Data received from the transfer is moved from the SPI receive shift register to the receive FIFO. The SPI controller requests a write from DMA to memory. Upon a DMA

grant, the DMA engine reads a word from the receive FIFO and writes to memory. Transfer continues to be initiated when both receives and transmits can accommodate new data.

9. When the receive pipe fills up due to unavailability of DMA grants, the transmit pipe stalls until the pipe is drained. If the transmit pipe fills up, the SPI stops requesting for DMA writes. When the value in SPI\_RWC expires, further write-requests to DMA stop. However, data already written into the transmit FIFO is sent, and read requests to DMA continue until the receive data is read from the receive FIFO.
10. The SPI then generates the programmed clock pulses on SPI\_CLK and simultaneously shifts data out of SPI\_COTI while shifting data in from SPI\_CITO . For receive transfers, the value in the shift register is loaded into the SPI\_RFIFO register at the end of the transfer. For transmit transfers, the value in the SPI\_TFIFO register is loaded into the shift register at the start of the transfer.

## Configuring DMA Target Mode Operation

This mode occurs when the SPI is enabled as a target and the DMA engine is configured to transmit or receive data. A transition of the SPI\_TS signal to the active-low state triggers the start of a transfer. Or, the first active edge of SPI\_CLK triggers the start of a transfer, depending on the state of the SPI\_CTL.CPHA bit. The following steps illustrate the SPI receive or transmit DMA sequence in an SPI target (in response to a controller command). The SPI supports a receive DMA channel and a transmit DMA channel.

1. Write to the appropriate DMA registers to enable the SPI DMA channel and configure the necessary work units, access direction, word count, and so on.
2. Write to the SPI\_CTL , SPI\_RXCTL , and SPI\_TXCTL registers to define the mode of the serial link to be the same as the mode configured in the SPI controller.
3. When the receive channel is enabled ( SPI\_RXCTL.REN is asserted), the following actions occur:
- a. Once the target select input is active, the target starts receiving and transmitting data on active SPI\_CLK edges.
- b. The value in the shift register is loaded into the SPI\_RFIFO register at the end of the transfer.
- c. Once SPI\_RFIFO has valid data, it requests a write from DMA to memory.
- d. Upon a DMA grant, the DMA engine reads a word from the receive FIFO and writes to memory.
- e. When there is data in the receive FIFO, the SPI target continues to request a DMA write to memory. The DMA engine continues to read a word from the FIFO and writes to memory. The SPI target continues receiving words on active SPI\_CLK edges when the SPI\_TS input is active.
- f. When the data collected in the receive pipe breaches the set level, and the DMA engine cannot keep up with the receive rate, the target can deassert the SPI\_RDY signal. This signaling throttles the controller. The receive pipe level is set according to the SPI\_CTL.FCWM field. The signal is deasserted as the DMA drains the receive FIFO. Alternatively, the SPI can use the SPI\_RXCTL.RDO bit to decide when the incoming data is discarded or overwritten into the receive FIFO (when SPI\_CTL.FCEN is inactive).
4. When the transmit channel is enabled ( SPI\_TXCTL.TEN is asserted), the following actions occur:

- a. The SPI requests a DMA read from memory.
- b. Upon a DMA grant, the DMA engine reads a word from memory and writes to the transmit FIFO.
- c. The SPI then reads DMA data from the transmit FIFO and writes to the transmit shift register, awaiting the start of the next transfer.
- d. Once the target select input is active, the target starts receiving and transmitting data on active SPI\_CLK edges.
- e. When there is room in the transmit FIFO, the SPI target continues to request a DMA read from memory. The DMA engine continues to read a word from memory and write to the transmit FIFO. The SPI target continues transmitting words on active SPI\_CLK edges as long as the SPI\_TS input is active.
- f. When the number of outstanding data entries in the transmit pipe breaches the level set and the DMA cannot keep up with the transmit rate, the target deasserts the SPI\_RDY signal. This signaling throttles the controller. The transmit pipe level is set according to the SPI\_CTL.FCWM field. The signal is deasserted as the DMA fills the transmit FIFO. Alternately, the SPI\_TXCTL.TDU bit decides the state of the transmit data (when SPI\_CTL.FCEN is deasserted).
5. When both receive and transmit channels are enabled, the following actions occur after the actions for each channel. Transfers continue when both receive and transmit channels can accommodate new data.
- a. When the receive pipe fills up due to the unavailability of DMA grant, the SPI interface stalls the controller by asserting the SPI\_RDY pin. This signal is deasserted as the DMA drains the receive FIFO. Alternately, the SPI uses the SPI\_RXCTL.RDO bit to decide when the incoming data is discarded or overwritten in the receive FIFO (when SPI\_CTL.FCEN is deasserted).
- b. When the transmit pipe fills up, the SPI stops requesting DMA writes until the pipe clears.
- c. When there is an underflow problem in the transmit pipe, the target stalls the controller by deasserting SPI\_RDY while the DMA fills the transmit FIFO. Alternately, the SPI uses the SPI\_TXCTL.TDU bit to decide the state of the transmit data (when SPI\_CTL.FCEN is deasserted).

## ADSP-2184x SPI Register Descriptions

Serial Peripheral Interface (SPI) contains the following registers.

Table 20-16: ADSP-2184x SPI Register List

| Name         | Description                         |
|--------------|-------------------------------------|
| SPI_CLK      | Clock Rate Register                 |
| SPI_CTL      | Control Register                    |
| SPI_DLY      | Delay Register                      |
| SPI_ILAT     | Masked Interrupt Condition Register |
| SPI_ILAT_CLR | Masked Interrupt Clear Register     |

Table 20-16: ADSP-2184x SPI Register List (Continued)

| Name         | Description                            |
|--------------|----------------------------------------|
| SPI_IMSK     | Interrupt Mask Register                |
| SPI_IMSK_CLR | Interrupt Mask Clear Register          |
| SPI_IMSK_SET | Interrupt Mask Set Register            |
| SPI_MMRDH    | Memory Mapped Read Header              |
| SPI_MMTOP    | SPI Memory Top Address                 |
| SPI_RFIFO    | Receive FIFO Data Register             |
| SPI_RWC      | Received Word Count Register           |
| SPI_RWCR     | Received Word Count Reload Register    |
| SPI_RXCTL    | Receive Control Register               |
| SPI_TSEL     | Target Select Register                 |
| SPI_STAT     | Status Register                        |
| SPI_TFIFO    | Transmit FIFO Data Register            |
| SPI_TWC      | Transmitted Word Count Register        |
| SPI_TWCR     | Transmitted Word Count Reload Register |
| SPI_TXCTL    | Transmit Control Register              |

## Clock Rate Register

The SPI\_CLK register selects the baud rate for SPI data transfers, relating this rate to the SPI serial clock (SPI clock) and the system clock (SCLK).

Figure 20-20: SPI\_CLK Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000019_13acd9f15eb0995a6f97213d51e6548e7ad6a522ca34e268debf5122f45cd11f.png)

Table 20-17: SPI\_CLK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                        |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------|
| 15:0 (R/W)         | BAUD       | Baud Rate. The SPI_CLK.BAUD bits set the SPI baud rate according to the formula: BAUD = (SCLK / SPI Clock) - 1 |

## Control Register

The SPI\_CTL register enables the SPI and configures settings for operating modes, communication protocols, and buffer operations.

Figure 20-21: SPI\_CTL Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000020_7339db784a251347ae88aba5adf104920da8de5af633d77ced4a3dbf43976f58.png)

Table 20-18: SPI\_CTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W)           | MMSE       | Memory-Mapped SPI Enable. When the SPI_CTL.MMSE bit is asserted, communication to an SPI memory device is automated such that the memory it contains is accessible directly through the read of processor address space assigned to it. (As far as the SPI peripheral is concerned, this includes all read accesses received by the SPI peripherals system crossbar port.) Note that when memory-mapped access of SPI memory is enabled, attempts to com- municate with the SPI device using legacy methods are blocked and receive fabric error responses are generated. Legacy methods include any direct access made to the Tx and Rx FIFOs, whether initiated by DMAor processor MMRaccess.                                          | Memory-Mapped SPI Enable. When the SPI_CTL.MMSE bit is asserted, communication to an SPI memory device is automated such that the memory it contains is accessible directly through the read of processor address space assigned to it. (As far as the SPI peripheral is concerned, this includes all read accesses received by the SPI peripherals system crossbar port.) Note that when memory-mapped access of SPI memory is enabled, attempts to com- municate with the SPI device using legacy methods are blocked and receive fabric error responses are generated. Legacy methods include any direct access made to the Tx and Rx FIFOs, whether initiated by DMAor processor MMRaccess.                                          |
| 31 (R/W)           | MMSE       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Hardware automated access of memory-mapped SPI memory disabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 31 (R/W)           | MMSE       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Hardware-automated access of memory-mapped SPI memory enabled.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| 30 (R/W)           | MMWEM      | Memory Mapped Write Error Mask. The SPI_CTL.MMWEM bit specifies whether an error response is returned to the fabric upon write attempts to address space reserved for memory-mapped reads of SPI memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Memory Mapped Write Error Mask. The SPI_CTL.MMWEM bit specifies whether an error response is returned to the fabric upon write attempts to address space reserved for memory-mapped reads of SPI memory.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 30 (R/W)           | MMWEM      | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Write error response returned upon write attempts to memory-mapped SPI memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 30 (R/W)           | MMWEM      | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Write error response masked (not returned) upon write attempts to memory-mapped SPI memory                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| 22 (R/W)           | SOTI       | Start on COTI. The SPI_CTL.SOTI bit is valid only when SPI_CTL.MIOM is enabled for either DIOM or QIOM, and this bit selects the starting pin and the bit placement on pins for these modes. In DIOM, by default, ( SPI_CTL.SOTI =0) SPI sends the first bit on the SPI_CITO pin and the second bit on the SPI_COTI pin. In QIOM, by default, the SPI sends the first bit on the SPI_D3 pin, the second bit on the SPI_D2 pin, the third bit on the SPI_CITO pin and the fourth bit on the SPI_COTI pin. This order can be reversed by setting the SPI_CTL.SOTI bit. When this bit is set, the SPI sends the first bit on the SPI_COTI pin. The first bit referred to here depends on the SPI_CTL.LSBF bit setting (MSB bit or LSB bit). | Start on COTI. The SPI_CTL.SOTI bit is valid only when SPI_CTL.MIOM is enabled for either DIOM or QIOM, and this bit selects the starting pin and the bit placement on pins for these modes. In DIOM, by default, ( SPI_CTL.SOTI =0) SPI sends the first bit on the SPI_CITO pin and the second bit on the SPI_COTI pin. In QIOM, by default, the SPI sends the first bit on the SPI_D3 pin, the second bit on the SPI_D2 pin, the third bit on the SPI_CITO pin and the fourth bit on the SPI_COTI pin. This order can be reversed by setting the SPI_CTL.SOTI bit. When this bit is set, the SPI sends the first bit on the SPI_COTI pin. The first bit referred to here depends on the SPI_CTL.LSBF bit setting (MSB bit or LSB bit). |
| 22 (R/W)           | SOTI       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Start on CITO (DIOM) or Start on SPI_D3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 22 (R/W)           | SOTI       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Start on COTI                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

Table 20-18: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 21:20 (R/W)        | MIOM       | Multiple I/O Mode. The SPI_CTL.MIOM bits enable SPI operation in dual I/O mode (DIOM) or quad I/O mode (QIOM). These bits can only be changed when the SPI is disabled ( SPI_CTL.EN =0).                                                                                                                                                                                                                         |
| 21:20 (R/W)        | MIOM       | 0 No MIOM (disabled)                                                                                                                                                                                                                                                                                                                                                                                             |
| 21:20 (R/W)        | MIOM       | 1 DIOM operation                                                                                                                                                                                                                                                                                                                                                                                                 |
| 21:20 (R/W)        | MIOM       | 2 QIOM operation                                                                                                                                                                                                                                                                                                                                                                                                 |
| 21:20 (R/W)        | MIOM       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                       |
| 18 (R/W)           | FMODE      | Fast-Mode Enable. The SPI_CTL.FMODE bit enables fast mode operation for SPI receive transfers. SPI transmit operations in fast mode are the same as normal mode.                                                                                                                                                                                                                                                 |
| 18 (R/W)           | FMODE      | 0 Disable                                                                                                                                                                                                                                                                                                                                                                                                        |
| 18 (R/W)           | FMODE      | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                                         |
| 17:16 (R/W)        | FCWM       | Flow Control Watermark. The SPI_CTL.FCWM bits select the watermark level of the transmit channel ( SPI_TFIFO buffer) or receive channel ( SPI_RFIFO buffer) that triggers flow control operation. These bits are applicable only when the SPI is a target ( SPI_CTL.MODE = 0) and flow control is enabled ( SPI_CTL.FCEN =1). When the watermark condition is met, the SPI controller deasserts the SPI_RDY pin. |
| 17:16 (R/W)        | FCWM       | 0 TFIFO empty or RFIFO full                                                                                                                                                                                                                                                                                                                                                                                      |
| 17:16 (R/W)        | FCWM       | 1 TFIFO 75% or more empty, or RFIFO 75% or more full                                                                                                                                                                                                                                                                                                                                                             |
| 17:16 (R/W)        | FCWM       | 2 TFIFO 50% or more empty, or RFIFO 50% or more full                                                                                                                                                                                                                                                                                                                                                             |
| 17:16 (R/W)        | FCWM       | 3 Reserved                                                                                                                                                                                                                                                                                                                                                                                                       |
| 15 (R/W)           | FCPL       | Flow Control Polarity. The SPI_CTL.FCPL bit selects flow control polarity for the SPI_RDY pin when flow control is enabled. When the SPI_RDY pin is active, the SPI is indicating it is ready for data transfer.                                                                                                                                                                                                 |
| 15 (R/W)           | FCPL       | 0 Active-low RDY                                                                                                                                                                                                                                                                                                                                                                                                 |
| 15 (R/W)           | FCPL       | 1 Active-high RDY                                                                                                                                                                                                                                                                                                                                                                                                |

Table 20-18: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14 (R/W)           | FCCH       | Flow Control Channel Selection. The SPI_CTL.FCCH bit selects whether the SPI applies flow control to the transmit channel ( SPI_TFIFO buffer) or receive channel ( SPI_RFIFO buffer). This bit is applicable only when the SPI is a target and flow control is enabled.                                                                          |
| 14 (R/W)           | FCCH       | 0 Flow Control on RX Buffer                                                                                                                                                                                                                                                                                                                      |
| 14 (R/W)           | FCCH       | 1 Flow Control on TX Buffer                                                                                                                                                                                                                                                                                                                      |
| 13 (R/W)           | FCEN       | Flow Control Enable. The SPI_CTL.FCEN bit enables SPI flow control operation, which permits slow target devices to interface with fast controller devices. This bit controls the operation of the SPI_RDY pin. Note that options for flow control operation are available using the SPI_CTL.FCCH , SPI_CTL.FCPL , and SPI_CTL.FCWM bits. Disable |
| 13 (R/W)           | FCEN       | 0                                                                                                                                                                                                                                                                                                                                                |
| 13 (R/W)           | FCEN       | 1 Enable                                                                                                                                                                                                                                                                                                                                         |
| 12 (R/W)           | LSBF       | Least Significant Bit First. The SPI_CTL.LSBF bit selects whether the SPI transmits/receives data as LSB first (little endian) or MSB first (big endian). This bit can only be changed when the SPI is disabled.                                                                                                                                 |
| 12 (R/W)           | LSBF       | 0 MSB sent/received first (big endian)                                                                                                                                                                                                                                                                                                           |
| 12 (R/W)           | LSBF       | 1 LSB sent/received first (little endian)                                                                                                                                                                                                                                                                                                        |
| 10:9 (R/W)         | SIZE       | Word Transfer Size. The SPI_CTL.SIZE bits select the SPI transfer word size as 8, 16 or 32 bits. To ensure correct operation, both the controller and target must be configured with the same word size. This bit can only be changed when the SPI is disabled ( SPI_CTL.EN =0).                                                                 |
| 10:9 (R/W)         | SIZE       | 0 8-bit word                                                                                                                                                                                                                                                                                                                                     |
| 10:9 (R/W)         | SIZE       | 1 16-bit word                                                                                                                                                                                                                                                                                                                                    |
| 10:9 (R/W)         | SIZE       | 2 32-bit word                                                                                                                                                                                                                                                                                                                                    |
| 10:9 (R/W)         | SIZE       | 3 Reserved                                                                                                                                                                                                                                                                                                                                       |
| 8 (R/W)            | ECITO      | Enable CITO. The SPI_CTL.ECITO bit enables controller-in-target-out (CITO) mode. This SPI mode is applicable only when the SPI is a target.                                                                                                                                                                                                      |
| 8 (R/W)            | ECITO      | 0 Disable                                                                                                                                                                                                                                                                                                                                        |
| 8 (R/W)            | ECITO      | 1 Enable                                                                                                                                                                                                                                                                                                                                         |

Table 20-18: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W)            | POLSEL     | Target Select Polarity Between Transfers. The SPI_CTL.POLSEL bit selects the state (polarity) for the SPI_SEL[n] pin between SPI transfers when the SPI is a controller and hardware target select assertion is enabled ( SPI_CTL.ATSEL =1). In target mode, this bit affects the detection of both transmit collision ( SPI_STAT.TC and underrun ( SPI_STAT.TUR ) errors.                                                                                                                                                                                                                                                                                                                                                                           |
| 7 (R/W)            | POLSEL     | 0 Deassert Target Select (high)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| 6 (R/W)            | ATSEL      | Target Select Pin Control. The SPI_CTL.ATSEL bit selects whether the SPI hardware sets the SPI_SEL[n] pin output value (ignoring the target select SPI_TSEL.TSEL1 - SPI_TSEL.TSEL7 bits) or whether software control of the target select bits set the SPI_SEL[n] pin output value. This feature is applicable only when the SPI is a controller. When hardware control is enabled, the SPI_SEL[n] pin output is asserted during the transfers, and the pin polarity between transfers is selected by the SPI_CTL.POLSEL bit. When software control is enabled, the SPI_SEL[n] pin output value is set through software control of the target select bits, and as such, the pin may either remain asserted (low) or be deasserted between transfers. |
| 6 (R/W)            | ATSEL      | 0 Software Target Select Control                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| 5 (R/W)            | CPOL       | Clock Polarity. The SPI_CTL.CPOL bit selects whether the SPI uses an active-low or active-high signal for the SPI clock ( SPI_CLK ). This bit works with the SPI_CTL.CPHA bit to select combinations of clock phase and polarity for the SPI_CLK pin. This bit can only be changed when the SPI is disabled.                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 5 (R/W)            | CPOL       | 0 Active-high SPI CLK                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 5 (R/W)            | CPOL       | 1 Active-low SPI CLK                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| 4 (R/W)            | CPHA       | Clock Phase. The SPI_CTL.CPHA bit selects whether the SPI starts toggling the signal for the SPI clock ( SPI_CLK ) from the start of the first data bit or from the middle of the first data bit. The SPI_CTL.CPHA bit works with the SPI_CTL.CPOL bit to select combinations of clock phase and polarity for the SPI_CLK pin. This bit can only be changed when the SPI is disabled.                                                                                                                                                                                                                                                                                                                                                                |
| 4 (R/W)            | CPHA       | 0 SPI CLK toggles from middle                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| 4 (R/W)            | CPHA       | 1 SPI CLK toggles from start                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 20-18: SPI\_CTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 3 (R/W)            | ODM        | Open Drain Mode. The SPI_CTL.ODM bit configures the data output pins ( SPI_COTI and SPI_CITO ) to behave as open drain outputs, which prevents contention and possible damage to pin drivers in multi-controller or multi-target SPI systems. When SPI_CTL.ODM is enabled and the SPI is a controller, the SPI three-states the SPI_COTI pin when the data driven out on COTI is a logic-high. The SPI does not three-state the SPI_COTI pin when the driven data is a logic-low. When SPI_CTL.ODM is enabled and the SPI is a target, the SPI three-states the SPI_CITO pin when the data driven out on SPI_CITO is a logic-high. Note that an external pull-up resistor is required on both the SPI_COTI and SPI_CITO pins when SPI_CTL.ODM is enabled. 0 Disable |
| 2 (R/W)            | PTSE       | Protected Target Select Enable. The SPI_CTL.PTSE bit enables the SPI_TS pin to provide error detection input in a multi-controller environment when the SPI is in controller mode. If some other device in the system asserts the SPI_TS pin while SPI is enabled as a controller (and SPI_CTL.PTSE is enabled), this condition causes a mode fault error.                                                                                                                                                                                                                                                                                                                                                                                                          |
| 1 (R/W)            | MODE       | Controller/Target. The SPI_CTL.MODE bit toggles the SPI between controller mode and target mode. This bit can only be changed when the SPI is disabled. 0 Target                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 0 (R/W)            | EN         | Enable. The SPI_CTL.EN bit enables SPI operation.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

## Delay Register

The SPI\_DLY register selects a transfer delay and the lead/lag timing between target select signals and SPI clock edge assertion/deassertion.

Figure 20-22: SPI\_DLY Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000021_c80a38d84c4eb90c8c9a464e6e1f727f88bb3dda415dc4f2dd537466ea196498.png)

Table 20-19: SPI\_DLY Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                    |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 9 (R/W)            | LAGX       | Extended SPI Clock Lag Control. The SPI_DLY.LAGX bit enables insertion of a 1-SPI_CLK cycle lag (extend lag) in the timing between the target select ( SPI_SEL[n] ) assertion and first SPI clock edge.                                                                                                                                                                                                                    |
| 8 (R/W)            | LEADX      | Extended SPI Clock Lead Control. The SPI_DLY.LEADX bit enables insertion of a 1-SPI_CLK cycle lead (extend lead) in the timing between the target select ( SPI_SEL[n] ) deassertion and last SPI clock edge.                                                                                                                                                                                                               |
| 7:0 (R/W)          | STOP       | Transfer Delay Time in Multiples of SPI Clock Period. The SPI_DLY.STOP bits select a delay (number of stop bits in multiples of SPI clock duration) at the end of each SPI transfer. The default delay is the minimum val- ue required to comply with the SPI protocol (1-bit duration). The SPI_DLY.STOP bits can be programmed with smaller delay values, resulting in continuous operation (for example, stop bits =0). |

## Masked Interrupt Condition Register

The SPI\_ILAT register latches interrupts, queuing the interrupt requests for service. When a condition is indicated by a bit in the SPI\_STAT register and the corresponding interrupt request is unmasked in SPI\_IMSK , the SPI latches the interrupt request bit in SPI\_ILAT .

Figure 20-23: SPI\_ILAT Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000022_1720a3152eef523afadf2e44dd6402ab17681bf164c0679c7abfb09e94738a7f.png)

Table 20-20: SPI\_ILAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                   |
|--------------------|------------|---------------------------------------------------------------------------|
| 11                 | TF         | Transmit Finish Interrupt Latch.                                          |
| 10 (R/NW)          | RF         | Receive Finish Interrupt Latch. 0 No interrupt request                    |
| 9 (R/NW)           | TS         | Interrupt Latch. 0 No interrupt request                                   |
| 8                  | RS         | Transmit Start 1 Latched interrupt request Receive Start Interrupt Latch. |
| (R/NW)             |            | 0 No interrupt request 1 Latched interrupt request                        |

Table 20-20: SPI\_ILAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------|
| 7                  | MF         | Mode Fault Interrupt Latch.                                                                                               |
| 6 (R/NW)           | TC         | Transmit Collision Interrupt Latch. 0 No interrupt request 1 Latched interrupt request                                    |
| 5 (R/NW)           | TUR        | Transmit Underrun Interrupt Latch. 0 No interrupt request 1 Latched interrupt request                                     |
| 4 (R/NW)           | ROR        | Receive Overrun Interrupt Latch. 0 No interrupt request                                                                   |
| 2 (R/NW)           | TUWM       | 1 Latched interrupt request Transmit Urgent Watermark Interrupt Latch. 0 No interrupt request 1 Latched interrupt request |
| 1                  | RUWM       | Receive Urgent Watermark Interrupt Latch.                                                                                 |
| (R/NW)             |            | 0 No interrupt request                                                                                                    |
|                    |            | 1                                                                                                                         |
|                    |            | Latched interrupt request                                                                                                 |

## Masked Interrupt Clear Register

The SPI\_ILAT\_CLR register permits clearing individual mask bits in the SPI\_ILAT register without affecting other bits in the register. Use write-1-to-clear on a bit in the SPI\_ILAT\_CLR register to clear the corresponding bit in the SPI\_ILAT register.

Figure 20-24: SPI\_ILAT\_CLR Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000023_e32896ae36eb3e4d196a9b60a1aa6f2198a8c5b4db675576d2d8a2a278c2b7ab.png)

Table 20-21: SPI\_ILAT\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 11 (R/W1C)         | TF         | Clear Transmit Finish. The SPI_ILAT_CLR.TF bit clears the corresponding mask bit in the SPI_ILAT register. |
| 11 (R/W1C)         | TF         | 0 No effect                                                                                                |
| 11 (R/W1C)         | TF         | 1 Clear mask bit                                                                                           |
| 10 (R/W1C)         | RF         | Clear Receive Finish. The SPI_ILAT_CLR.RF bit clears the corresponding mask bit in the SPI_ILAT register.  |
| 10 (R/W1C)         | RF         | 0 No effect                                                                                                |
| 10 (R/W1C)         | RF         | 1 Clear mask bit                                                                                           |
| 9 (R/W1C)          | TS         | Clear Transmit Start. The SPI_ILAT_CLR.TS bit clears the corresponding mask bit in the SPI_ILAT register.  |
| 9 (R/W1C)          | TS         | 0 No effect                                                                                                |
| 9 (R/W1C)          | TS         | 1 Clear mask bit                                                                                           |

Table 20-21: SPI\_ILAT\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W1C)          | RS         | Clear Receive Start. The SPI_ILAT_CLR.RS bit clears the corresponding mask bit in the SPI_ILAT register.               |
| 8 (R/W1C)          | RS         | 0 No effect                                                                                                            |
| 8 (R/W1C)          | RS         | 1 Clear mask bit                                                                                                       |
| 7 (R/W1C)          | MF         | Clear Mode Fault. The SPI_ILAT_CLR.MF bit clears the corresponding mask bit in the SPI_ILAT register.                  |
| 7 (R/W1C)          | MF         | 0 No effect                                                                                                            |
| 7 (R/W1C)          | MF         | 1 Clear mask bit                                                                                                       |
| 6 (R/W1C)          | TC         | Clear Transmit Collision. The SPI_ILAT_CLR.TC bit clears the corresponding mask bit in the SPI_ILAT register.          |
| 6 (R/W1C)          | TC         | 0 No effect                                                                                                            |
| 6 (R/W1C)          | TC         | 1 Clear mask bit                                                                                                       |
| 5 (R/W1C)          | TUR        | Clear Transmit Underrun. The SPI_ILAT_CLR.TUR bit clears the corresponding mask bit in the SPI_ILAT register.          |
| 5 (R/W1C)          | TUR        | 0 No effect                                                                                                            |
| 5 (R/W1C)          | TUR        | 1 Clear mask bit                                                                                                       |
| 4 (R/W1C)          | ROR        | Clear Receive Overrun. The SPI_ILAT_CLR.ROR bit clears the corresponding mask bit in the SPI_ILAT register.            |
| 4 (R/W1C)          | ROR        | 0 No effect                                                                                                            |
| 4 (R/W1C)          | ROR        | 1 Clear mask bit                                                                                                       |
| 2 (R/NW)           | TUWM       | Clear Transmit Urgent Watermark. The SPI_ILAT_CLR.TUWM bit clears the corresponding mask bit in the SPI_ILAT register. |
| 2 (R/NW)           | TUWM       | 0 No effect                                                                                                            |
| 2 (R/NW)           | TUWM       | 1 Clear mask bit                                                                                                       |

Table 20-21: SPI\_ILAT\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 1 (R/NW)           | RUWM       | Clear Receive Urgent Watermark. The SPI_ILAT_CLR.RUWM bit clears the corresponding mask bit in the SPI_ILAT register. |

## Interrupt Mask Register

The SPI\_IMSK register unmasks (enables) or masks (disables) SPI interrupt requests. When a condition is indicated by a bit in the SPI\_STAT register and the corresponding interrupt request is unmasked in SPI\_IMSK , the SPI latches the interrupt request bit in the SPI\_ILAT register, queuing the interrupt request for service.

Figure 20-25: SPI\_IMSK Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000024_4ee43160e1a158ff570859d178c19e4cf07efdb155c2662ebfe1bff1662fbafb.png)

Table 20-22: SPI\_IMSK Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                             |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/NW)          | TF         | Transmit Finish. The SPI_IMSK.TF bit unmasks (enables) or masks (disables) the TF interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request |
| 10 (R/NW)          | RF         | Receive Finish. The SPI_IMSK.RF bit unmasks (enables) or masks (disables) the RF interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request  |
| 9 (R/NW)           | TS         | Transmit Start. The SPI_IMSK.TS bit unmasks (enables) or masks (disables) the TS interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request  |

Table 20-22: SPI\_IMSK Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                        |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| 8 (R/NW)           | RS         | Receive Start. The SPI_IMSK.RS bit unmasks (enables) or masks (disables) the RS interrupt.                                                     |
| 7 (R/NW)           | MF         | Mode Fault. The SPI_IMSK.MF bit unmasks (enables) or masks (disables) the MF interrupt. 0 Disable (mask) interrupt request                     |
| 6 (R/NW)           | TC         | Transmit Collision. The SPI_IMSK.TC bit unmasks (enables) or masks (disables) the TC interrupt. 0 Disable (mask) interrupt request             |
| 5 (R/NW)           | TUR        | 1 Enable (unmask) interrupt request Transmit Underrun. The SPI_IMSK.TUR bit unmasks (enables) or masks (disables) the TUR                      |
|                    |            | interrupt. 0 Disable (mask) interrupt request 1 Enable (unmask) interrupt request                                                              |
| 4 (R/NW)           | ROR        | Receive Overrun. The SPI_IMSK.ROR bit unmasks (enables) or masks (disables) the ROR interrupt. 0 Disable (mask) interrupt request              |
| 2 (R/NW)           | TUWM       | Transmit Urgent Watermark. The SPI_IMSK.TUWM bit unmasks (enables) or masks (disables) the TUWMinter- rupt. 0 Disable (mask) interrupt request |
| 1 (R/NW)           | RUWM       | Receive Urgent Watermark. The SPI_IMSK.RUWM bit unmasks (enables) or masks (disables) the RUWMinter- rupt.                                     |

## Interrupt Mask Clear Register

The SPI\_IMSK\_CLR register permits clearing individual mask bits in the SPI\_IMSK register without affecting other bits in the register. Use write-1-to-clear on a bit in the SPI\_IMSK\_CLR register to clear the corresponding bit in the SPI\_IMSK register.

Figure 20-26: SPI\_IMSK\_CLR Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000025_14d270f582e45fecfaaa8b26d8d0e7fa76e94b7da68ca73df5acf5a158f776a1.png)

Table 20-23: SPI\_IMSK\_CLR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                    |
|--------------------|------------|------------------------------------------------------------------------------------------------------------|
| 11 (R/W1C)         | TF         | Clear Transmit Finish. The SPI_IMSK_CLR.TF bit clears the corresponding mask bit in the SPI_IMSK register. |
| 11 (R/W1C)         | TF         | 0 No effect                                                                                                |
| 11 (R/W1C)         | TF         | 1 Clear mask bit                                                                                           |
| 10 (R/W1C)         | RF         | Clear Receive Finish. The SPI_IMSK_CLR.RF bit clears the corresponding mask bit in the SPI_IMSK register.  |
| 10 (R/W1C)         | RF         | 0 No effect                                                                                                |
| 10 (R/W1C)         | RF         | 1 Clear mask bit                                                                                           |
| 9 (R/W1C)          | TS         | Clear Transmit Start. The SPI_IMSK_CLR.TS bit clears the corresponding mask bit in the SPI_IMSK register.  |
| 9 (R/W1C)          | TS         | 0 No effect                                                                                                |
| 9 (R/W1C)          | TS         | 1 Clear mask bit                                                                                           |

Table 20-23: SPI\_IMSK\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------|
| 8 (R/W1C)          | RS         | Clear Receive Start. The SPI_IMSK_CLR.RS bit clears the corresponding mask bit in the SPI_IMSK register.               |
| 8 (R/W1C)          | RS         | 0 No effect                                                                                                            |
| 8 (R/W1C)          | RS         | 1 Clear mask bit                                                                                                       |
| 7 (R/W1C)          | MF         | Clear Mode Fault. The SPI_IMSK_CLR.MF bit clears the corresponding mask bit in the SPI_IMSK register.                  |
| 7 (R/W1C)          | MF         | 0 No effect                                                                                                            |
| 7 (R/W1C)          | MF         | 1 Clear mask bit                                                                                                       |
| 6 (R/W1C)          | TC         | Clear Transmit Collision. The SPI_IMSK_CLR.TC bit clears the corresponding mask bit in the SPI_IMSK register.          |
| 6 (R/W1C)          | TC         | 0 No effect                                                                                                            |
| 6 (R/W1C)          | TC         | 1 Clear mask bit                                                                                                       |
| 5 (R/W1C)          | TUR        | Clear Transmit Underrun. The SPI_IMSK_CLR.TUR bit clears the corresponding mask bit in the SPI_IMSK register.          |
| 5 (R/W1C)          | TUR        | 0 No effect                                                                                                            |
| 5 (R/W1C)          | TUR        | 1 Clear mask bit                                                                                                       |
| 4 (R/W1C)          | ROR        | Clear Receive Overrun. The SPI_IMSK_CLR.ROR bit clears the corresponding mask bit in the SPI_IMSK register.            |
| 4 (R/W1C)          | ROR        | 0 No effect                                                                                                            |
| 4 (R/W1C)          | ROR        | 1 Clear mask bit                                                                                                       |
| 2 (R/W1C)          | TUWM       | Clear Transmit Urgent Watermark. The SPI_IMSK_CLR.TUWM bit clears the corresponding mask bit in the SPI_IMSK register. |
| 2 (R/W1C)          | TUWM       | 0 No effect                                                                                                            |
| 2 (R/W1C)          | TUWM       | 1 Clear mask bit                                                                                                       |

Table 20-23: SPI\_IMSK\_CLR Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                               |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1C)          | RUWM       | Clear Receive Urgent Watermark. The SPI_IMSK_CLR.RUWM bit clears the corresponding mask bit in the SPI_IMSK register. |

## Interrupt Mask Set Register

The SPI\_IMSK\_SET register permits setting individual mask bits in the SPI\_IMSK register without affecting other bits in the register. Use write-1-to-set on a bit in the SPI\_IMSK\_SET register to set the corresponding bit in the SPI\_IMSK register.

Figure 20-27: SPI\_IMSK\_SET Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000026_5984bc49efae056a07dbe6c3f6a923ee394eb43fc96c5c4becbca48b40d417f5.png)

Table 20-24: SPI\_IMSK\_SET Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------|
| 11 (R/W1S)         | TF         | Set Transmit Finish. The SPI_IMSK_SET.TF bit sets the corresponding mask bit in the SPI_IMSK register. |
| 11 (R/W1S)         | TF         | 0 No effect                                                                                            |
| 11 (R/W1S)         | TF         | 1 Set mask bit                                                                                         |
| 10 (R/W1S)         | RF         | Set Receive Finish. The SPI_IMSK_SET.RF bit sets the corresponding mask bit in the SPI_IMSK register.  |
| 10 (R/W1S)         | RF         | 0 No effect                                                                                            |
| 10 (R/W1S)         | RF         | 1 Set mask bit                                                                                         |
| 9 (R/W1S)          | TS         | Set Transmit Start. The SPI_IMSK_SET.TS bit sets the corresponding mask bit in the SPI_IMSK register.  |
| 9 (R/W1S)          | TS         | 0 No effect                                                                                            |
| 9 (R/W1S)          | TS         | 1 Set mask bit                                                                                         |

Table 20-24: SPI\_IMSK\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------|
| 8 (R/W1S)          | RS         | Set Receive Start. The SPI_IMSK_SET.RS bit sets the corresponding mask bit in the SPI_IMSK register.               |
| 8 (R/W1S)          | RS         | 0 No effect                                                                                                        |
| 8 (R/W1S)          | RS         | 1 Set mask bit                                                                                                     |
| 7 (R/W1S)          | MF         | Set Mode Fault. The SPI_IMSK_SET.MF bit sets the corresponding mask bit in the SPI_IMSK register.                  |
| 7 (R/W1S)          | MF         | 0 No effect                                                                                                        |
| 7 (R/W1S)          | MF         | 1 Set mask bit                                                                                                     |
| 6 (R/W1S)          | TC         | Set Transmit Collision. The SPI_IMSK_SET.TC bit sets the corresponding mask bit in the SPI_IMSK register.          |
| 6 (R/W1S)          | TC         | 0 No effect                                                                                                        |
| 6 (R/W1S)          | TC         | 1 Set mask bit                                                                                                     |
| 5 (R/W1S)          | TUR        | Set Transmit Underrun. The SPI_IMSK_SET.TUR bit sets the corresponding mask bit in the SPI_IMSK register.          |
| 5 (R/W1S)          | TUR        | 0 No effect                                                                                                        |
| 5 (R/W1S)          | TUR        | 1 Set mask bit                                                                                                     |
| 4 (R/W1S)          | ROR        | Set Receive Overrun. The SPI_IMSK_SET.ROR bit sets the corresponding mask bit in the SPI_IMSK register.            |
| 4 (R/W1S)          | ROR        | 0 No effect                                                                                                        |
| 4 (R/W1S)          | ROR        | 1 Set mask bit                                                                                                     |
| 2 (R/W1S)          | TUWM       | Set Transmit Urgent Watermark. The SPI_IMSK_SET.TUWM bit sets the corresponding mask bit in the SPI_IMSK register. |
| 2 (R/W1S)          | TUWM       | 0 No effect                                                                                                        |
| 2 (R/W1S)          | TUWM       | 1 Set mask bit                                                                                                     |

Table 20-24: SPI\_IMSK\_SET Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                           |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------|
| 1 (R/W1S)          | RUWM       | Set Receive Urgent Watermark. The SPI_IMSK_SET.RUWM bit sets the corresponding mask bit in the SPI_IMSK register. |
| 1 (R/W1S)          | RUWM       | 0 No effect                                                                                                       |
| 1 (R/W1S)          | RUWM       | 1 Set mask bit                                                                                                    |

## Memory Mapped Read Header

The SPI\_MMRDH register enables the use of memory-mapped mode. This mode allows direct memory-mapped read accesses of an SPI memory device and is primarily used to directly execute instructions from an SPI FLASH memory without using a low-level software driver. All overhead tasks such as transmission of the read header, pin turnaround timing and receive data sizing are handled in hardware.

The memory-mapped access mode is enabled by setting the SPI\_CTL.MMSE bit. The features within the SPI\_MMRDH register include a command skip mode, variable length byte addressing, and independent multi-pin support for command transmission, address transmission and data reception. In addition, the command opcode and mode bytes are fully programmable.

Figure 20-28: SPI\_MMRDH Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000027_cc063a0164db7e153cf2db03e5c0f4298c3c4e8230f5ecb31933c3f6d0458c1b.png)

Table 20-25: SPI\_MMRDH Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                          | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                          |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W)           | CMDPINS    | Pins Used for Command. The SPI_MMRDH.CMDPINS bit specifies the number of pins to be used for com- mand transmission. This bit must be set consistent with the expectations established by the read opcode. Hardware does not interpret SPI_MMRDH.OPCODE , but rather relies on this bit to specify behavior. When cleared, it overrides the SPI_CTL.MIOM bits. When set, it uses bits specified by the SPI_CTL.MIOM bit setting. | Pins Used for Command. The SPI_MMRDH.CMDPINS bit specifies the number of pins to be used for com- mand transmission. This bit must be set consistent with the expectations established by the read opcode. Hardware does not interpret SPI_MMRDH.OPCODE , but rather relies on this bit to specify behavior. When cleared, it overrides the SPI_CTL.MIOM bits. When set, it uses bits specified by the SPI_CTL.MIOM bit setting. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                | Use Only One Pin: COTI (overrides SPI_CTL.MIOM bits)                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                | Use Pins Specified by SPI_CTL.MIOM Bits                                                                                                                                                                                                                                                                                                                                                                                          |

Table 20-25: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 28 (R/W)           | CMDSKIP    | Command Skip Enable. The SPI_MMRDH.CMDSKIP bit enables command skip mode where the address is sent first and the OPCODE field is not sent ( SPI_MMRDH.CMDSKIP bit =1). This mode is useful for supporting XIP (Execute-In-Place) operation where only the address is sent and the same read command is assumed. The SPI flash device must be primed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | Command Skip Enable. The SPI_MMRDH.CMDSKIP bit enables command skip mode where the address is sent first and the OPCODE field is not sent ( SPI_MMRDH.CMDSKIP bit =1). This mode is useful for supporting XIP (Execute-In-Place) operation where only the address is sent and the same read command is assumed. The SPI flash device must be primed                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 28 (R/W)           | CMDSKIP    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | OPCODE field is sent first followed by address                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| 27 (R/W)           | WRAP       | 1 OPCODE field is not sent; address is sent first SPI Memory Wrap Indicator. The SPI_MMRDH.WRAP bit must be set by software if software places a connected SPI memory device into a 8-byte, 16-byte or 32-byte wrap mode based on the ILINE and DLINE field setting of the cache configuration register address wrap mode. Software achieves this by transmitting a vendor specified command to the SPI memory device while the SPI_CTL.MMSE bit =0. If the SPI_MMRDH.WRAP bit =1, the SPI does not need to deassert the SPI target select signal and resend the read header in order to wrap to the cache line base when servicing misaligned cache fill requests. Although this improves cache fill efficiency, it requires that the SPI deassert the SPI target select pin and resend the read header whenever a DMAburst requests crosses 32 byte alignments. Setting this bit improves cache throughput but decreases DMAthroughput. | 1 OPCODE field is not sent; address is sent first SPI Memory Wrap Indicator. The SPI_MMRDH.WRAP bit must be set by software if software places a connected SPI memory device into a 8-byte, 16-byte or 32-byte wrap mode based on the ILINE and DLINE field setting of the cache configuration register address wrap mode. Software achieves this by transmitting a vendor specified command to the SPI memory device while the SPI_CTL.MMSE bit =0. If the SPI_MMRDH.WRAP bit =1, the SPI does not need to deassert the SPI target select signal and resend the read header in order to wrap to the cache line base when servicing misaligned cache fill requests. Although this improves cache fill efficiency, it requires that the SPI deassert the SPI target select pin and resend the read header whenever a DMAburst requests crosses 32 byte alignments. Setting this bit improves cache throughput but decreases DMAthroughput. |
| 27 (R/W)           | WRAP       | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | SPI Memory auto increments address purely sequential- ly                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| 27 (R/W)           | WRAP       | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | SPI Memory auto increments address but wraps within 32 Byte lines                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 26 (R/W)           | MERGE      | Merge Enable. When the SPI_MMRDH.MERGE bit is set, SPI hardware combines the two succes- sive transfers. This increases the throughput rate when accessing a large number of sequential memory locations. For more information refer to the Merged Read Accesses section.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 | Merge Enable. When the SPI_MMRDH.MERGE bit is set, SPI hardware combines the two succes- sive transfers. This increases the throughput rate when accessing a large number of sequential memory locations. For more information refer to the Merged Read Accesses section.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |

Table 20-25: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
|--------------------|------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 25:24 (R/W)        | TRIDMY     | Tristate Dummy Timing. The SPI_MMRDH.TRIDMY bits specify whether and when output pins are three- stated during the interval of time specified by the SPI_MMRDH.DMYSIZE bits. Output pins potentially three-stated include all pins which were used to transmit the                                                                                                                                                                                                                                                                              | Tristate Dummy Timing. The SPI_MMRDH.TRIDMY bits specify whether and when output pins are three- stated during the interval of time specified by the SPI_MMRDH.DMYSIZE bits. Output pins potentially three-stated include all pins which were used to transmit the                                                                                                                                                                                                                                                                              |
| 25:24 (R/W)        | TRIDMY     | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Tristate outputs immediately                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 25:24 (R/W)        | TRIDMY     | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Tristate outputs after 4 bits of dummy/mode are trans- mitted                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 25:24 (R/W)        | TRIDMY     | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Tristate outputs after 8 bits of dummy/mode are trans- mitted                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| 25:24 (R/W)        | TRIDMY     | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Never tristate outputs (previously specified output state is held)                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| 23:16 (R/W)        | MODE       | Mode Field. These bits specify up to a leading byte to be transmitted during the interval of time specified by the SPI_MMRDH.DMYSIZE bit field. This first byte, or a portion of it, is interpreted as mode bits when certain opcodes are used in conjunction with certain SPI memory devices. Mode bits are sent using the same number of pins which were used to transmit the address. Once sent, output pins will be held in their final resultant state until the conclusion of all dummy byte periods, unless three-stating the outputs is | Mode Field. These bits specify up to a leading byte to be transmitted during the interval of time specified by the SPI_MMRDH.DMYSIZE bit field. This first byte, or a portion of it, is interpreted as mode bits when certain opcodes are used in conjunction with certain SPI memory devices. Mode bits are sent using the same number of pins which were used to transmit the address. Once sent, output pins will be held in their final resultant state until the conclusion of all dummy byte periods, unless three-stating the outputs is |
| 14:12 (R/W)        | DMYSIZE    | Bytes of Dummy/Mode. The SPI_MMRDH.DMYSIZE bit field specifies the number of bytes separating ad- dress transmission and read data return. Dummy bytes elapse assuming dummy bits are transmitted using the same number of pins which were used to transmit address.                                                                                                                                                                                                                                                                            | Bytes of Dummy/Mode. The SPI_MMRDH.DMYSIZE bit field specifies the number of bytes separating ad- dress transmission and read data return. Dummy bytes elapse assuming dummy bits are transmitted using the same number of pins which were used to transmit address.                                                                                                                                                                                                                                                                            |
| 14:12 (R/W)        | DMYSIZE    | 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 0 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 1 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 2                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 2 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 3 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 4                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 4 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 5                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 5 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 6                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 6 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 14:12 (R/W)        | DMYSIZE    | 7                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | 7 Bytes                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |

Table 20-25: SPI\_MMRDH Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 11 (R/W)           | ADRPINS    | Pins Used for Address. The SPI_MMRDH.ADRPINS bit specifies the number of pins to be used for address transmission. This bit must be set consistent with expectations established by read opcode. Hardware does not interpret the SPI_MMRDH.OPCODE , but rather relies on this bit to specify behavior.                                                                                                                                                                                 |
| 10:8 (R/W)         | ADRSIZE    | 1 Use Pins Specified by SPI_CTL.MIOM Bits Bytes of Read Address. The SPI_MMRDH.ADRSIZE bit field defines the number of bytes used to specify the read address. The read address is sent immediately following the transmission of opcode. Unlike opcode bits, address bits may be sent using either one or multiple pins. The number of pins is selected using the SPI_MMRDH.ADRPINS bit. The address sent to a connected SPI memory device is an echo of the read address received by |
| 7:0 (R/W)          | OPCODE     | Read Opcode. The SPI_MMRDH.OPCODE bit field specifies the initial bits transmitted in response to a read request of SPI memory. Although any opcode may be sent, values 0x03, 0x0B, 0x3B, 0x6B, 0xBB, and 0xEB are likely to be the most commonly used. SPI_MMRDH.OPCODE is sent by the SPI without interpretation; the states of these bits have no effect beyond specifying what is initially shifted across the SPI interface.                                                      |

## SPI Memory Top Address

The SPI\_MMTOP register specifies the top populated address of a connected SPI memory device.

Figure 20-29: SPI\_MMTOP Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000028_e610f8835ef0b34ebe2f56364e2349a93bfca6d9ace76836782167a2af3c341a.png)

Table 20-26: SPI\_MMTOP Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31:0 (R/W)         | TOPADR     | SPI Memory Top Address. The SPI_MMTOP.TOPADR bit field specifies the top populated address of a connect- ed SPI memory device. Attempts to access SPI memory are not blocked if this address is exceeded and an error is generated as part of the read response. |

## Receive FIFO Data Register

The SPI\_RFIFO register has an interface to the receive shift register in the SPI and has an interface to the processor data buses. The top level of the buffer is visible to programs as the 32-bit SPI\_RFIFO register, but the size (number of word locations) of the receive FIFO is actually flexible with transfer word size. The size of the receive FIFO is 8 if the word size is 8-bit, or the size is 4 if the word size is 16-bit, or the size is 2 if the word size is 32-bit.

Both controllers and targets may stop or stall receive transfers based on FIFO status. When the receive FIFO is full, the SPI controller stops initiating new transfers on the SPI if SPI\_RXCTL.RTI is enabled. A target may stall the SPI interface when the content of the FIFO crosses the selected watermark. If data reception continues after SPI\_RFIFO is full, the data in the receive FIFO is invalid. The SPI indicates this condition with receive overrun ( SPI\_STAT.ROR ) error. This condition is possible when SPI\_RXCTL.RTI =0 and SPI\_RXCTL.REN =1 for a controller, or for a target that does not exercise flow control.

Note that the receive FIFO is reset (cleared) when the SPI is disabled after being enabled.

Figure 20-30: SPI\_RFIFO Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000029_ac39ff7b131f8be1fae437373b81f6cd184f6995f6e6be992f65ec4f5d342a32.png)

Table 20-27: SPI\_RFIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 31:0               | DATA       | Receive FIFO Data.                                           |
| (R/NW)             |            | The SPI_RFIFO.DATA bit field contains the FIFO receive data. |

## Received Word Count Register

The SPI\_RWC register holds a count of the number of words remaining to be received by the SPI. To start the decrement of the word count in SPI\_RWC , enable the receive word counter ( SPI\_RXCTL.RWCEN =1). The SPI uses the word count to control the duration of transfers and to signal the completion of a burst of transfers with the receive finish interrupt ( SPI\_ILAT.RF ). In DMA mode, the SPI uses the SPI\_RWC register to ensure that the number of frames received during a DMA transfer is equal to the number of words programmed in the DMA channel controller. The values programmed into the SPI\_RWC registers should match the word count in the DMA configuration. The SPI\_RWC register maintains the number of frames to be received in a transfer. The SPI\_RWC should only be changed when the counter is disabled.

Figure 20-31: SPI\_RWC Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000030_8b6ecb0e37bf0c5b1bee179739b0e705721b68c07020ce308d6a47b381878ae0.png)

Table 20-28: SPI\_RWC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                      |
|--------------------|------------|--------------------------------------------------------------|
| 15:0               | VALUE      | Received Word Count.                                         |
| (R/W)              |            | The SPI_RWC.VALUE bits hold the receive transfer word count. |

## Received Word Count Reload Register

The SPI\_RWCR register holds the receive word count value that the SPI loads into the SPI\_RWC register when the transfer count decrements to zero. To prevent the SPI from reloading the counter, use zero for the reload count value. The SPI\_RWCR register should only be changed when the counter is disabled.

Figure 20-32: SPI\_RWCR Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000031_3be6c92845a611f3a6eab80e34c5dd09eda636d597a12b22b65aba150d205580.png)

Table 20-29: SPI\_RWCR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                    |
|--------------------|------------|----------------------------------------------------------------------------|
| 15:0               | VALUE      | Received Word Count Reload.                                                |
| (R/W)              |            | The SPI_RWCR.VALUE bits hold the receive transfer word count reload value. |

## Receive Control Register

The SPI\_RXCTL register enables the SPI receive channel, initiates receive transfers, and configures SPI\_RFIFO buffer watermark settings.

Figure 20-33: SPI\_RXCTL Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000032_b49aaaa003b58b654c63d0004e920655186044cc50625b734cfc463cb8d47ab0.png)

Table 20-30: SPI\_RXCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                   |
|--------------------|------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | RUWM       | Receive FIFO Urgent Watermark. The SPI_RXCTL.RUWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.RUWM interrupt. When an urgent SPI_RFIFO watermark is enabled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deassertion condition for any SPI_ILAT.RUWM interrupts that are latched. | Receive FIFO Urgent Watermark. The SPI_RXCTL.RUWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for generation of the SPI_ILAT.RUWM interrupt. When an urgent SPI_RFIFO watermark is enabled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deassertion condition for any SPI_ILAT.RUWM interrupts that are latched. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                         | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                         | 25% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                         | 50% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                         | 75% full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                            |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                         | Full RFIFO                                                                                                                                                                                                                                                                                                                                                                                                                |
|                    |            | 5                                                                                                                                                                                                                                                                                                                                                                                                                         | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 6                                                                                                                                                                                                                                                                                                                                                                                                                         | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                  |
|                    |            | 7                                                                                                                                                                                                                                                                                                                                                                                                                         | Reserved                                                                                                                                                                                                                                                                                                                                                                                                                  |

Table 20-30: SPI\_RXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W)        | RRWM       | Receive FIFO Regular Watermark. The SPI_RXCTL.RRWM bits select the receive FIFO ( SPI_RFIFO ) watermark level for regular data bus requests. When an urgent SPI_RFIFO watermark is enabled with SPI_RXCTL.RUWM , the SPI_RXCTL.RRWM selection is used as the deasser- tion condition for any SPI_ILAT.RUWM interrupts that are latched. |
| 13:12 (R/W)        | RRWM       | 0 Empty RFIFO                                                                                                                                                                                                                                                                                                                           |
| 13:12 (R/W)        | RRWM       | 1 RFIFO less than 25% full                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | RRWM       | 2 RFIFO less than 50% full                                                                                                                                                                                                                                                                                                              |
| 13:12 (R/W)        | RRWM       | 3 RFIFO less than 75% full                                                                                                                                                                                                                                                                                                              |
| 8 (R/W)            | RDO        | Receive Data Overrun. The SPI_RXCTL.RDO bit selects handling for receive data requests when the receive                                                                                                                                                                                                                                 |
| 8 (R/W)            | RDO        | 0 Discard incoming data if SPI_RFIFO is full                                                                                                                                                                                                                                                                                            |
| 8 (R/W)            | RDO        | 1 Overwrite old data if SPI_RFIFO is full                                                                                                                                                                                                                                                                                               |
| 6:4 (R/W)          | RDR        | Receive Data Request. The SPI_RXCTL.RDR bits select receive FIFO ( SPI_RFIFO ) watermark condi- tions that direct the SPI to generate a receive data request.                                                                                                                                                                           |
| 6:4 (R/W)          | RDR        | 0 Disabled                                                                                                                                                                                                                                                                                                                              |
| 6:4 (R/W)          | RDR        | 1 Not empty RFIFO                                                                                                                                                                                                                                                                                                                       |
| 6:4 (R/W)          | RDR        | 2 25% full RFIFO                                                                                                                                                                                                                                                                                                                        |
| 6:4 (R/W)          | RDR        | 3 50% full RFIFO                                                                                                                                                                                                                                                                                                                        |
| 6:4 (R/W)          | RDR        | 4 75% full RFIFO                                                                                                                                                                                                                                                                                                                        |
| 6:4 (R/W)          | RDR        | 5 Full RFIFO                                                                                                                                                                                                                                                                                                                            |
| 6:4 (R/W)          | RDR        | 6 Reserved                                                                                                                                                                                                                                                                                                                              |
| 6:4 (R/W)          | RDR        | 7 Reserved                                                                                                                                                                                                                                                                                                                              |
| 3 (R/W)            | RWCEN      | Receive Word Counter Enable. The SPI_RXCTL.RWCEN bit enables the decrement of the SPI_RWC reg- ister when the count is not zero and SPI_RXCTL.RTI is enabled. Ena- bling SPI_RXCTL.RWCEN prevents receive overrun errors from occurring. The SPI_RXCTL.RWCEN bit is valid only when the SPI is a controller.                            |
| 3 (R/W)            | RWCEN      | 0 Disable                                                                                                                                                                                                                                                                                                                               |
| 3 (R/W)            | RWCEN      | 1 Enable                                                                                                                                                                                                                                                                                                                                |

Table 20-30: SPI\_RXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                 |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | RTI        | Receive Transfer Initiate. The SPI_RXCTL.RTI bit enables initiation of receive transfers if the receive FIFO ( SPI_RFIFO ) is not full. The bit also enables this initiation if SPI_RWC is not zero when SPI_RXCTL.RWCEN is enabled. Enabling SPI_RXCTL.RTI prevents receive overrun errors from occurring. The SPI_RXCTL.RTI bit is valid only when the SPI is a controller. 0 Disable |
| 0 (R/W)            | REN        | Receive Enable. The SPI_RXCTL.REN bit enables SPI receive channel operation. 0 Disable 1 Enable                                                                                                                                                                                                                                                                                         |

## Target Select Register

The SPI\_TSEL register enables the pins for output and indicates the state (high or low) of these pins when enabled.

Figure 20-34: SPI\_TSEL Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000033_6134b950cfaeb1019cd0c7b23aeff27398153eb3329cae312472298fb6db222b.png)

Table 20-31: SPI\_TSEL Register Fields

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000034_b3e6329a1f7053c0f142bfdaac0503b12b20d5984bc012cf887c7f54a9c6d88c.png)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                     |
|--------------------|------------|-----------------------------------------------------------------------------------------------------------------------------|
| 15 (R/W)           | TSEL7      | Target Select 7 Output. The SPI_TSEL.TSEL7 bit state indicates the value driven on the related SPI_SEL[n] pin.              |
| 14 (R/W)           | TSEL6      | Target Select 6 Output. The SPI_TSEL.TSEL6 bit state indicates the value driven on the related SPI_SEL[n] pin. 0 Low 1 High |

Table 20-31: SPI\_TSEL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                            |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13 (R/W)           | TSEL5      | Target Select 5 Output. The SPI_TSEL.TSEL5 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                     |
| 13 (R/W)           | TSEL5      | 0 Low                                                                                                                                                              |
| 13 (R/W)           | TSEL5      | 1 High                                                                                                                                                             |
| 12 (R/W)           | TSEL4      | Target Select 4 Output. The SPI_TSEL.TSEL4 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                     |
| 12 (R/W)           | TSEL4      | 0 Low                                                                                                                                                              |
| 12 (R/W)           | TSEL4      | 1 High                                                                                                                                                             |
| 11 (R/W)           | TSEL3      | Target Select 3 Output. The SPI_TSEL.TSEL3 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                     |
| 11 (R/W)           | TSEL3      | 0 Low                                                                                                                                                              |
| 10 (R/W)           | TSEL2      | Target Select 2 Output. The SPI_TSEL.TSEL2 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                     |
| 10 (R/W)           | TSEL2      | 0 Low                                                                                                                                                              |
| 10 (R/W)           | TSEL2      | 1 High                                                                                                                                                             |
| 9 (R/W)            | TSEL1      | Target Select 1 Output. The SPI_TSEL.TSEL1 bit state indicates the value driven on the related SPI_SEL[n] pin.                                                     |
| 9 (R/W)            | TSEL1      | 0 Low                                                                                                                                                              |
| 9 (R/W)            | TSEL1      | 1 High                                                                                                                                                             |
| 7 (R/W)            | TSE7       | Target Select 7 Enable. The SPI_TSEL.TSE7 bit enables the related SPI_SEL[n] pin for output. If disabled, the SPI three-states the related SPI_SEL[n] pin. Disable |
| 7 (R/W)            | TSE7       | 0                                                                                                                                                                  |
| 7 (R/W)            | TSE7       | 1 Enable                                                                                                                                                           |

Table 20-31: SPI\_TSEL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                          |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 6 (R/W)            | TSE6       | Target Select 6 Enable. The SPI_TSEL.TSE6 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information.         |
| 6 (R/W)            | TSE6       | 0 Disable                                                                                                                                                        |
| 6 (R/W)            | TSE6       | 1 Enable                                                                                                                                                         |
| 5 (R/W)            | TSE5       | Target Select 5 Enable. The SPI_TSEL.TSE5 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information.         |
| 5 (R/W)            | TSE5       | 0 Disable                                                                                                                                                        |
| 5 (R/W)            | TSE5       | 1 Enable                                                                                                                                                         |
| 4 (R/W)            | TSE4       | Target Select 4 Enable. The SPI_TSEL.TSE4 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information.         |
| 4 (R/W)            | TSE4       | 0 Disable                                                                                                                                                        |
| 3 (R/W)            | TSE3       | Target Select 3 Enable. The SPI_TSEL.TSE3 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information.         |
| 3 (R/W)            | TSE3       | 0 Disable                                                                                                                                                        |
| 3 (R/W)            | TSE3       | 1 Enable                                                                                                                                                         |
| 2 (R/W)            | TSE2       | Target Select 2 Enable. The SPI_TSEL.TSE2 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information.         |
| 2 (R/W)            | TSE2       | 0 Disable                                                                                                                                                        |
| 2 (R/W)            | TSE2       | 1 Enable                                                                                                                                                         |
| 1 (R/W)            | TSE1       | Target Select 1 Enable. The SPI_TSEL.TSE1 bit enables the related SPI_SEL[n] pin for output. See the SPI_TSEL.TSE7 bit description for more information. Disable |
| 1 (R/W)            | TSE1       | 0                                                                                                                                                                |
| 1 (R/W)            | TSE1       | 1 Enable                                                                                                                                                         |

## Status Register

The SPI\_STAT register indicates SPI status including FIFO status, error conditions, and interrupt conditions. When an interrupt condition from this register is unmasked (enabled) by the corresponding bit in the SPI\_IMSK register, the interrupt request is latched into the corresponding bit in the SPI\_ILAT register.

Figure 20-35: SPI\_STAT Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000035_851d8deafc37c5e73565fd31faf0c06d532facacf034dd13e69ecc1654519702.png)

Table 20-32: SPI\_STAT Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 31 (R/W1C)         | MMAE       | Memory Mapped Access Error. The SPI_STAT.MMAE bit =1 if an attempt is made to access either the Tx or Rx FIFO while memory-mapped access of SPI memory is enabled (see the SPI_CTL.MMSE bit). The SPI_STAT.MMAE bit =0 when a 1 is written to it. The SPI_STAT.MMAE bit is provided for software notification only. Its state has no further effect. |

Table 20-32: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                            |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 29 (R/W1C)         | MMRE       | Memory Mapped Read Error. The SPI_STAT.MMRE bit =1 if an attempt is made to read address space reserved for memory-mapped SPI memory while memory mapping is disabled (see the SPI_CTL.MMSE bit). The SPI_STAT.MMRE bit =0 when a 1 is written to it. This bit is provided for software notification only. Its state has no further effect.        |
| 28 (R/W1C)         | MMWE       | Memory Mapped Write Error. The SPI_STAT.MMWE bit =1 if an attempt is made to write address space reserved for memory-mapped SPI memory. The SPI_STAT.MMWE bit =0 when a 1 is written to it. This bit is provided for software notification only. Its state has no further effect.                                                                  |
| 23 (R/NW)          | TFF        | SPI_TFIFO Full. The SPI_STAT.TFF bit indicates whether the SPI_TFIFO is full or not full. 0 Not full Tx FIFO                                                                                                                                                                                                                                       |
| 22 (R/NW)          | RFE        | SPI_RFIFO Empty. The SPI_STAT.RFE bit indicates whether the SPI_RFIFO is empty or not empty. 0 Rx FIFO not empty                                                                                                                                                                                                                                   |
| 20 (R/NW)          | FCS        | 1 Rx FIFO empty Flow Control Stall Indication. The SPI_STAT.FCS bit indicates whether a target has deasserted the SPI_RDY pin to stall the SPI controller while the target is unable to service the SPI controller request. This bit is valid only when the SPI is a controller ( SPI_CTL.MODE =1) and flow control is enabled ( SPI_CTL.FCEN =1). |
| 18:16 (R/NW)       | TFS        | SPI_TFIFO Status. The SPI_STAT.TFS bits indicate the status of the SPI_TFIFO . The SPI uses this status when evaluating transmit watermark conditions.                                                                                                                                                                                             |

Table 20-32: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                              |
|--------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 14:12 (R/NW)       | RFS        | SPI_RFIFO Status. The SPI_STAT.RFS bits indicate the status of the SPI_RFIFO . The SPI uses this status when evaluating receive watermark conditions.                                                                                                                |
| 14:12 (R/NW)       | RFS        | 0 Empty RFIFO                                                                                                                                                                                                                                                        |
| 14:12 (R/NW)       | RFS        | 1 25% full RFIFO                                                                                                                                                                                                                                                     |
| 14:12 (R/NW)       | RFS        | 2 50% full RFIFO                                                                                                                                                                                                                                                     |
| 14:12 (R/NW)       | RFS        | 3 75% full RFIFO                                                                                                                                                                                                                                                     |
| 14:12 (R/NW)       | RFS        | 4 Full RFIFO                                                                                                                                                                                                                                                         |
| 11 (R/W1C)         | TF         | Transmit Finish Indication. The SPI_STAT.TF bit indicates that the SPI has detected the finish of a transmit burst transfer (the SPI_TWC count decrements to zero). This condition can only                                                                          |
| 11 (R/W1C)         | TF         | 0 No status                                                                                                                                                                                                                                                          |
| 11 (R/W1C)         | TF         | 1 Transmit finish detected                                                                                                                                                                                                                                           |
| 10 (R/W1C)         | RF         | Receive Finish Indication. The SPI_STAT.RF bit indicates that the SPI has detected the finish of a receive burst transfer (the SPI_RWC count decrements to zero). This condition can only occur when SPI_RXCTL.RTI and SPI_RXCTL.RWCEN are enabled.                  |
| 10 (R/W1C)         | RF         | 0 No status                                                                                                                                                                                                                                                          |
| 10 (R/W1C)         | RF         | 1 Receive finish detected                                                                                                                                                                                                                                            |
| 9 (R/W1C)          | TS         | Transmit Start.                                                                                                                                                                                                                                                      |
| 9 (R/W1C)          | TS         | 0 No status                                                                                                                                                                                                                                                          |
| 9 (R/W1C)          | TS         | 1 Transmit start detected                                                                                                                                                                                                                                            |
| 8 (R/W1C)          | RS         | Receive Start. The SPI_STAT.RS bit indicates that the SPI has detected the start of a receive burst transfer. A receive bursts starts with the load of SPI_RWC from the SPI_RWCR . This condition can only occur when SPI_RXCTL.RTI and SPI_RXCTL.RWCEN are enabled. |
| 8 (R/W1C)          | RS         | 0 No status                                                                                                                                                                                                                                                          |
| 8 (R/W1C)          | RS         | 1 Receive start detected                                                                                                                                                                                                                                             |

Table 20-32: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                          |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 7 (R/W1C)          | MF         | Mode Fault Indication. The SPI_STAT.MF bit, when SPI is a controller and SPI_CTL.PTSE is enabled, indicates that multiple controllers have asserted target select inputs.                                                                                                                        |
| 7 (R/W1C)          | MF         | 0 No status                                                                                                                                                                                                                                                                                      |
| 6 (R/W1C)          | TC         | Transmit Collision Indication. The SPI_STAT.TC bit, when SPI is a target, indicates that the load of data into the shift register has occurred too close to the first transmitting edge of the SPI clock. status                                                                                 |
| 6 (R/W1C)          | TC         | 0 No                                                                                                                                                                                                                                                                                             |
| 5 (R/W1C)          | TUR        | Transmit Underrun Indication. The SPI_STAT.TUR bit, when the transmit FIFO ( SPI_TFIFO ) is empty, indi- cates that the last word in the transmit FIFO has been re-sent as transmit data. Alternately, it indicates that zero has been sent as transmit data.                                    |
| 5 (R/W1C)          | TUR        | 0 No status                                                                                                                                                                                                                                                                                      |
| 4 (R/W1C)          | ROR        | Receive Overrun Indication. The SPI_STAT.ROR bit, when the receive FIFO ( SPI_RFIFO ) is full, indicates that a word in the receive FIFO has been overwritten with incoming receive data. Alternately, it indicates that incoming receive data has been discarded.                               |
| 4 (R/W1C)          | ROR        | 0 No status                                                                                                                                                                                                                                                                                      |
| 2 (R/NW)           | TUWM       | 1 Receive overrun occurred Transmit Urgent Watermark Breached. The SPI_STAT.TUWM bit indicates that the transmit urgent watermark ( SPI_TXCTL.TUWM ) has been reached. This condition is cleared when the transmit FIFO fills enough to reach the transmit regular watermark ( SPI_TXCTL.TRWM ). |
| 2 (R/NW)           | TUWM       | 0 Tx regular watermark reached                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | RUWM       | Receive Urgent Watermark Breached. The SPI_STAT.RUWM bit indicates that the receive urgent watermark ( SPI_RXCTL.RUWM ) has been reached. This condition is cleared when the receive FIFO empties enough to reach the receive regular watermark ( SPI_RXCTL.RRWM                                 |
| 1 (R/NW)           | RUWM       | 0 Rx regular watermark reached                                                                                                                                                                                                                                                                   |
| 1 (R/NW)           | RUWM       | ).                                                                                                                                                                                                                                                                                               |
|                    |            | 1 Rx urgent watermark breached                                                                                                                                                                                                                                                                   |

Table 20-32: SPI\_STAT Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                |
|--------------------|------------|----------------------------------------------------------------------------------------|
| 0 (R/NW)           | SPIF       | SPI Finished. The SPI_STAT.SPIF bit indicates that a single word transfer is complete. |
| 0 (R/NW)           | SPIF       | 0 No status                                                                            |
| 0 (R/NW)           | SPIF       | 1 Completed single word transfer                                                       |

## Transmit FIFO Data Register

The SPI\_TFIFO register has an interface to the transmit shift register in the SPI and has an interface to the processor data buses. The top level of the buffer is visible to programs as the 32-bit SPI\_TFIFO register, but the size (number of word locations) of the transmit FIFO is actually flexible with transfer word size. The size of the transmit FIFO is 8 if the word size is 8-bit, or the size is 4 if the word size is 16-bit, or the size is 2 if the word size is 32-bit.

Both controllers and targets may stop or stall transmit transfers based on FIFO status. When the transmit FIFO is empty, the SPI controller stops initiating new transfers on the SPI if SPI\_TXCTL.TTI is enabled. A target may stall the SPI interface when the content of the FIFO crosses the selected watermark. If data transmit requests continue after SPI\_TFIFO is empty, the data sent from the transmit FIFO is invalid, and the SPI indicates this condition with transmit underrun ( SPI\_STAT.TUR ). This condition is possible when SPI\_TXCTL.TTI =0 and SPI\_TXCTL.TEN =1 for a controller, or for a target that does not exercise flow control.

Note that the transmit FIFO is reset (cleared) when the SPI is disabled after being enabled.

Figure 20-36: SPI\_TFIFO Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000036_1ef23e5f824cca255bff716b527fe6403e0af009132fedb6b3817be2965a7931.png)

Table 20-33: SPI\_TFIFO Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 31:0               | DATA       | Transmit FIFO Data.                                           |
| (R/W)              |            | The SPI_TFIFO.DATA bit field contains the FIFO transmit data. |

## Transmitted Word Count Register

The SPI\_TWC register holds a count of the number of words remaining to be transmitted by the SPI. To start the decrement of the word count in SPI\_TWC , enable the transmit word counter ( SPI\_TXCTL.TWCEN =1). The SPI uses the word count to control the duration of transfers and to signal the completion of a burst of transfers with the transmit finish interrupt request. In DMA mode, the SPI uses the SPI\_TWC to ensure that the number of frames transmitted during a DMA transfer is equal to the number of words programmed in the DMA channel controller. The values programmed into the SPI\_TWC registers should match the word count in the DMA configuration. The SPI\_TWC maintains the number of frames to be transmitted in a transfer. The SPI\_TWC should only be changed when the counter is disabled.

Figure 20-37: SPI\_TWC Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000037_fd64203653d76b5dd4874651878c20add1ba0845f02de0ddd26410d37d963e3c.png)

Table 20-34: SPI\_TWC Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                       |
|--------------------|------------|---------------------------------------------------------------|
| 15:0               | VALUE      | Transmitted Word Count.                                       |
| (R/W)              |            | The SPI_TWC.VALUE bits hold the transmit transfer word count. |

## Transmitted Word Count Reload Register

The SPI\_TWCR register holds the transmit word count value that the SPI loads into the SPI\_TWC register when the transfer count decrements to zero. To prevent the SPI from reloading the counter, use zero for the reload count value. The SPI\_TWCR should only be changed when the counter is disabled.

Figure 20-38: SPI\_TWCR Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000038_13f953caf11a2e3f186990438562bc474cbf9584243b5c45a1eab678eec83b5d.png)

Table 20-35: SPI\_TWCR Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                     |
|--------------------|------------|-----------------------------------------------------------------------------|
| 15:0               | VALUE      | Transmitted Word Count Reload.                                              |
| (R/W)              |            | The SPI_TWCR.VALUE bits hold the transmit transfer word count reload value. |

## Transmit Control Register

The SPI\_TXCTL register enables the SPI transmit channel, initiates transmit transfers, and configures SPI\_TFIFO buffer watermark settings.

Figure 20-39: SPI\_TXCTL Register Diagram

![Image](23_Serial_Peripheral_Interface_(SPI)_artifacts/image_000039_a68a9473dfede90037214f7f49fe14db98f210a757213a81e8c53e6199369a73.png)

Table 20-36: SPI\_TXCTL Register Fields

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                              | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                                              |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 18:16 (R/W)        | TUWM       | FIFO Urgent Watermark. The SPI_TXCTL.TUWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for gener- ation of the SPI_ILAT.TUWM interrupt request. When an urgent SPI_TFIFO watermark is enabled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched. | FIFO Urgent Watermark. The SPI_TXCTL.TUWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for urgent data bus requests. The SPI also uses this watermark level for gener- ation of the SPI_ILAT.TUWM interrupt request. When an urgent SPI_TFIFO watermark is enabled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched. |
|                    |            | 0                                                                                                                                                                                                                                                                                                                                                                                                                                    | Disabled                                                                                                                                                                                                                                                                                                                                                                                                                             |
|                    |            | 1                                                                                                                                                                                                                                                                                                                                                                                                                                    | 25% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 2                                                                                                                                                                                                                                                                                                                                                                                                                                    | 50% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 3                                                                                                                                                                                                                                                                                                                                                                                                                                    | 75% empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                      |
|                    |            | 4                                                                                                                                                                                                                                                                                                                                                                                                                                    | Empty TFIFO                                                                                                                                                                                                                                                                                                                                                                                                                          |

Table 20-36: SPI\_TXCTL Register Fields (Continued)

| Bit No. (Access) Bit   | Description/Enumeration                                                                                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                           | Description/Enumeration                                                                                                                                                                                                                                                                                                                           |
|------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 13:12 (R/W) TRWM       | FIFO Regular Watermark. The SPI_TXCTL.TRWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for regular data bus requests. When an urgent SPI_TFIFO watermark is en- abled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched.          | FIFO Regular Watermark. The SPI_TXCTL.TRWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for regular data bus requests. When an urgent SPI_TFIFO watermark is en- abled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched.          | FIFO Regular Watermark. The SPI_TXCTL.TRWM bits select the transmit FIFO ( SPI_TFIFO ) watermark level for regular data bus requests. When an urgent SPI_TFIFO watermark is en- abled with SPI_TXCTL.TUWM , the SPI_TXCTL.TRWM selection is used as the deassertion condition for any SPI_ILAT.TUWM interrupt requests that are latched.          |
| 8 (R/W) TDU            | Transmit Data Underrun. The SPI_TXCTL.TDU bit selects handling for transmit data requests when the transmit buffer ( SPI_TFIFO ) is empty. If enabled and SPI_TFIFO is empty, the SPI transmits zero as data. If disabled and SPI_TFIFO is empty, the SPI transmits the last word in the buffer as data. 0 Send last word when SPI_TFIFO is empty | Transmit Data Underrun. The SPI_TXCTL.TDU bit selects handling for transmit data requests when the transmit buffer ( SPI_TFIFO ) is empty. If enabled and SPI_TFIFO is empty, the SPI transmits zero as data. If disabled and SPI_TFIFO is empty, the SPI transmits the last word in the buffer as data. 0 Send last word when SPI_TFIFO is empty | Transmit Data Underrun. The SPI_TXCTL.TDU bit selects handling for transmit data requests when the transmit buffer ( SPI_TFIFO ) is empty. If enabled and SPI_TFIFO is empty, the SPI transmits zero as data. If disabled and SPI_TFIFO is empty, the SPI transmits the last word in the buffer as data. 0 Send last word when SPI_TFIFO is empty |
| 6:4 (R/W) TDR          | 1 Send zeros when SPI_TFIFO is empty Request. SPI_TXCTL.TDR bits select transmit FIFO ( SPI_TFIFO ) watermark condi- SPI to generate a transmit status interrupt request. 0 Disabled                                                                                                                                                              | 1 Send zeros when SPI_TFIFO is empty Request. SPI_TXCTL.TDR bits select transmit FIFO ( SPI_TFIFO ) watermark condi- SPI to generate a transmit status interrupt request. 0 Disabled                                                                                                                                                              | 1 Send zeros when SPI_TFIFO is empty Request. SPI_TXCTL.TDR bits select transmit FIFO ( SPI_TFIFO ) watermark condi- SPI to generate a transmit status interrupt request. 0 Disabled                                                                                                                                                              |
|                        | tions that direct the 1 Not full TFIFO 2 25% empty TFIFO 3 50% empty TFIFO 4 75% empty TFIFO                                                                                                                                                                                                                                                      | tions that direct the 1 Not full TFIFO 2 25% empty TFIFO 3 50% empty TFIFO 4 75% empty TFIFO                                                                                                                                                                                                                                                      | tions that direct the 1 Not full TFIFO 2 25% empty TFIFO 3 50% empty TFIFO 4 75% empty TFIFO                                                                                                                                                                                                                                                      |
| 3 (R/W)                | 5 Empty TFIFO Word Counter Enable. SPI_TXCTL.TWCEN bit enables the decrement of the transmit word count ) register when the count is not zero and SPI_TXCTL.TTI is SPI_TXCTL.TWCEN prevents transmit underrun errors from                                                                                                                         | 5 Empty TFIFO Word Counter Enable. SPI_TXCTL.TWCEN bit enables the decrement of the transmit word count ) register when the count is not zero and SPI_TXCTL.TTI is SPI_TXCTL.TWCEN prevents transmit underrun errors from                                                                                                                         | 5 Empty TFIFO Word Counter Enable. SPI_TXCTL.TWCEN bit enables the decrement of the transmit word count ) register when the count is not zero and SPI_TXCTL.TTI is SPI_TXCTL.TWCEN prevents transmit underrun errors from                                                                                                                         |
| TWCEN                  | Transmit The ( SPI_TWC enabled. Enabling occurring. The SPI_TXCTL.TWCEN bit is valid only when the SPI is a controller.                                                                                                                                                                                                                           | Transmit The ( SPI_TWC enabled. Enabling occurring. The SPI_TXCTL.TWCEN bit is valid only when the SPI is a controller.                                                                                                                                                                                                                           | Transmit The ( SPI_TWC enabled. Enabling occurring. The SPI_TXCTL.TWCEN bit is valid only when the SPI is a controller.                                                                                                                                                                                                                           |
|                        | 0 Disable                                                                                                                                                                                                                                                                                                                                         | 0 Disable                                                                                                                                                                                                                                                                                                                                         | 0 Disable                                                                                                                                                                                                                                                                                                                                         |
|                        | 1 Enable                                                                                                                                                                                                                                                                                                                                          | 1 Enable                                                                                                                                                                                                                                                                                                                                          | 1 Enable                                                                                                                                                                                                                                                                                                                                          |

Table 20-36: SPI\_TXCTL Register Fields (Continued)

| Bit No. (Access)   | Bit Name   | Description/Enumeration                                                                                                                                                                                                                                                                                                                                                                                |
|--------------------|------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 2 (R/W)            | TTI        | Transmit Transfer Initiate. The SPI_TXCTL.TTI bit enables initiation of transmit transfers if the transmit FIFO ( SPI_TFIFO ) is not empty. The bit also enables this initiation if SPI_TWC is not zero when SPI_TXCTL.TWCEN is enabled. Enabling SPI_TXCTL.TTI prevents transmit underrun errors from occurring. The SPI_TXCTL.TTI bit is valid only when the SPI is a controller. 0 Disable 1 Enable |
| 0 (R/W)            | TEN        | Transmit Enable. The SPI_TXCTL.TEN bit enables SPI transmit channel operation. 0 Disable                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | TEN        | 1 Enable                                                                                                                                                                                                                                                                                                                                                                                               |
| 0 (R/W)            | TEN        |                                                                                                                                                                                                                                                                                                                                                                                                        |